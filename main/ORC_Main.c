//#define DAC_TEST

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_intr_types.h"

#include "Fusion.h"
#include "LTC2664_reg.h"
#include "ism330dhcx_reg.h"
#include "Transform.h"
#include "PID.h"

#define MUX_PIN_A0      33
#define MUX_PIN_A1      32
#define COMPARATOR_PIN  21
#define GPIO_MUX_PINS   ((1ULL<<MUX_PIN_A0) | (1ULL<<MUX_PIN_A1))
#define GPIO_COMP_PINS  (1ULL<<COMPARATOR_PIN)

#define CACHE_SIZE      8192*4    //bytes
#define MOUNT_POINT     "/sdcard"
#define SD_PIN_D3       13
#define SD_PIN_D2       12
#define SD_PIN_CLK      14
#define SD_PIN_CMD      15
#define SD_PIN_D0       2
#define SD_PIN_D1       4

#define IMU_HOST            VSPI_HOST
#define IMU_PIN_NUM_MISO    23
#define IMU_PIN_NUM_MOSI    19
#define IMU_PIN_NUM_CLK     18
#define IMU_PIN_NUM_CS      5
#define IMU_PIN_INT1        22//32
#define GPIO_INT_PIN_SEL    (1ULL<<IMU_PIN_INT1)

#define DAC_HOST            HSPI_HOST
#define DAC_PIN_NUM_MISO    -1
#define DAC_PIN_NUM_MOSI    27
#define DAC_PIN_NUM_CLK     26
#define DAC_PIN_NUM_CS      25

#define STATUS_LED1         17
#define STATUS_LED2         0
#define GPIO_STATUS_PINS    ((1ULL<<STATUS_LED2) | (1ULL<<STATUS_LED1))
#define MODE_SW_EN_LOGGING  34
#define MODE_SW_EN_ACTUATOR 35
#define MODE_SW_START_LOG   36    //start and stop log via this switch, or replace with external controller ouput
#define MODE_SW_EXTRA       39
#define GPIO_MODE_SWS       ((1ULL<<MODE_SW_EN_ACTUATOR) | (1ULL<<MODE_SW_EN_LOGGING) | (1ULL<<MODE_SW_EXTRA) | (1ULL<<MODE_SW_START_LOG))

#define IMU_BOOT_TIME       10 //ms

#define FA_COG      0.011    //distance to front axle linkages (in meters) from COG
#define RA_COG      0.018    //distance to rear axle linkages (in meters) from COG
#define DA_COG      0.008    //distance to drive side linkages (in meters) from COG
#define PA_COG      0.008    //distance to passenger side linkages (in meters) from COG

//spi functions for devices, platform delay for advanced IMU functions
static int32_t DAC_write(void *handle, ltc2664_DACS_t dac, uint8_t command, const uint16_t *data);
static int32_t IMU_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t IMU_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void platform_delay(uint32_t ms);

//initialization and configuration functions
static esp_err_t init_spi_IMU();
static esp_err_t init_spi_DAC();
static void init_pid_controllers(void);
static esp_err_t config_IMU(void);
static esp_err_t config_DAC(void);


//data structure to pass data from IMU Sampling to SD writing
typedef struct
{
    float_t acceleration_z;
    float_t pitch;
    float_t roll;

}log_data_t;
static QueueHandle_t log_data_queue = NULL;

//spi handle and device reference for IMU
static stmdev_ctx_t ISM330DHCX_dev_ctx;
static spi_device_handle_t IMU_spi;

//spi handle and device reference for DAC
static ltcdev_ctx_t LTC2664_dev_ctx;
static spi_device_handle_t DAC_spi;

//reference for IMU_sample_task
static TaskHandle_t IMU_TASK;

//time between IMU samples
static float_t float_time = 0;

//PID Contoller Handles
static PIDController_t accel_z_controller ={};
static PIDController_t pitch_controller={};
static PIDController_t roll_controller={};

//Boot Config booleans
static bool en_logging = false;
static bool en_actuators = false;


/**
 * @brief Capture interupt from IMU and tell the IMU_sample_task there's data avalible
 */
static void IRAM_ATTR imu_isr_handler(void* arg){
    
    vTaskNotifyGiveFromISR(IMU_TASK,NULL);

}

/**
 * @brief Initialize the IMU, it's interupts, and the fusion library, then perform continuous sampling.
 */
static void IMU_sample_task(void* arg)
{
    esp_err_t err;
    static FusionAhrs ahrs;
    int16_t data_raw_acceleration[3];
    int16_t data_raw_angular_rate[3];
    log_data_t df;
    float_t acceleration_z_float = 0;
    float_t pitch_float;
    float_t roll_float;
    FusionEuler euler;
    FusionVector linear;
    uint16_t actuator1, actuator2, actuator3, actuator4;

    const char TAG[] = "IMU Sampling Task";

    //set up fusion algorithm
    FusionAhrsInitialise(&ahrs);
    FusionAhrsSettings * const ahrs_settings = malloc(sizeof(FusionAhrsSettings));
    ahrs_settings->accelerationRejection =  10; //degrees
    ahrs_settings->gyroscopeRange =         500;//dps
    ahrs_settings->gain =                   .5; //influence of gyroscope
    ahrs_settings->convention =             FusionConventionEnu; //earth axes convention (consistent with ISM330DHCX)
    ahrs_settings->recoveryTriggerPeriod =  5/float_time; //approx 5 second recovery trigger period
    FusionAhrsSetSettings(&ahrs, ahrs_settings);

    ESP_LOGI(TAG, "AHRS Initialized");

    /* Initialize IMU*/
    err = init_spi_IMU();
    ESP_ERROR_CHECK(err);
    err = config_IMU();
    ESP_ERROR_CHECK(err);    

    /* Initialize PID Controllers */
    init_pid_controllers();

    /* Initialize DAC */
    err = init_spi_DAC();
    ESP_ERROR_CHECK(err);
    err = config_DAC();
    ESP_ERROR_CHECK(err);

    /* Initialize Transform Matrix Distances */
    err = set_distances(FA_COG,RA_COG,DA_COG,PA_COG);
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG,"Distaces set\nF-%.3f\nR-%.3f\nD-%.3f\nP-%.3f", FA_COG, RA_COG, DA_COG, PA_COG);

    /* Set up ISR */
    ESP_LOGI(TAG, "Installing ISR");
    err = gpio_install_isr_service(0);
    ESP_ERROR_CHECK(err);
    err = gpio_isr_handler_add(IMU_PIN_INT1, imu_isr_handler, (void*) IMU_PIN_INT1);
    ESP_ERROR_CHECK(err);   
    
    /* Calibrate the rest angle and acceleration for the vehicle */
    /* Read a register from the IMU to kick off interupt sampling */
    ism330dhcx_angular_rate_raw_get(&ISM330DHCX_dev_ctx, data_raw_angular_rate);
    ism330dhcx_acceleration_raw_get(&ISM330DHCX_dev_ctx, data_raw_acceleration);

    /* While fusion isn't stable, keep reading the IMU.*/
    FusionAhrsFlags fusionFlags = FusionAhrsGetFlags(&ahrs);
    while(fusionFlags.initialising || fusionFlags.angularRateRecovery){
        ulTaskNotifyTake(pdFALSE,portMAX_DELAY);
        //when IMU_PIN_INT1 rising edge

        //read accel/gyro
        memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
        ism330dhcx_acceleration_raw_get(&ISM330DHCX_dev_ctx, data_raw_acceleration);
        ism330dhcx_angular_rate_raw_get(&ISM330DHCX_dev_ctx, data_raw_angular_rate);
        
        //convert to dps for use in Fusion
        //uses full scale of 500dps for int to float conversion
        const FusionVector gyroscope = {{(float_t)data_raw_angular_rate[0] * 0.0175,
                                        (float_t)data_raw_angular_rate[1] * 0.0175f,
                                        (float_t)data_raw_angular_rate[2] * 0.0175f}};

        //convert to g for use in Fusion 
        //uses full scale of 2 g for int to float conversion
        const FusionVector accelerometer = {{(float_t)data_raw_acceleration[0] * 0.000061f,
                                        (float_t)data_raw_acceleration[1] * 0.000061f,
                                        (float_t)data_raw_acceleration[2] * 0.000061f}};

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, float_time);
        fusionFlags = FusionAhrsGetFlags(&ahrs);
    }
    euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
    linear = FusionAhrsGetLinearAcceleration(&ahrs);
    float_t acceleration_zero = linear.axis.z;
    float_t pitch_zero = euler.angle.pitch;
    float_t roll_zero = euler.angle.roll;

    ESP_LOGI(TAG, "Acceleration zero point: %.3f", acceleration_zero);
    ESP_LOGI(TAG, "Pitch zero point: %.3f", pitch_zero);
    ESP_LOGI(TAG, "Roll zero point: %.3f", roll_zero);


    vTaskDelay(500);
    /* Turn on Status LED */
    gpio_set_level(STATUS_LED2, 1);
    gpio_set_level(STATUS_LED1, 1);

    /* Read a register from the IMU to kick off interupt sampling again */
    ism330dhcx_angular_rate_raw_get(&ISM330DHCX_dev_ctx, data_raw_angular_rate);
    ism330dhcx_acceleration_raw_get(&ISM330DHCX_dev_ctx, data_raw_acceleration);
    ESP_LOGI(TAG,"Sampling started");

    for (;;) {
        
        ulTaskNotifyTake(pdFALSE,portMAX_DELAY);
        //when IMU_PIN_INT1 rising edge

        //read accel/gyro
        memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
        ism330dhcx_acceleration_raw_get(&ISM330DHCX_dev_ctx, data_raw_acceleration);
        ism330dhcx_angular_rate_raw_get(&ISM330DHCX_dev_ctx, data_raw_angular_rate);
        
        //convert to dps for use in Fusion
        //uses full scale of 500dps for int to float conversion
        const FusionVector gyroscope = {{(float_t)data_raw_angular_rate[0] * 0.0175,
                                        (float_t)data_raw_angular_rate[1] * 0.0175f,
                                        (float_t)data_raw_angular_rate[2] * 0.0175f}};

        //convert to g for use in Fusion 
        //uses full scale of 2 g for int to float conversion
        const FusionVector accelerometer = {{(float_t)data_raw_acceleration[0] * 0.000061f,
                                        (float_t)data_raw_acceleration[1] * 0.000061f,
                                        (float_t)data_raw_acceleration[2] * 0.000061f}};

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, float_time);
        euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        linear = FusionAhrsGetLinearAcceleration(&ahrs);

        //send data to be logged
        df.acceleration_z = linear.axis.z;
        df.pitch = euler.angle.pitch;
        df.roll = euler.angle.roll;
        xQueueSendToBack(log_data_queue,&df,0);

        //printf("\x1b[1F\33[2K\r%f", df.roll); //debug

        if(en_actuators == true){// below sections not needed if motors disabled
            //pid
            //acceleration_z_float = PIDController_Update(&accel_z_controller, &acceleration_zero, &linear.axis.z);
            pitch_float = PIDController_Update(&pitch_controller, &pitch_zero, &euler.angle.pitch);
            roll_float = PIDController_Update(&roll_controller, &roll_zero, &euler.angle.roll);

            //transform
            transform(&actuator1,&actuator2,&actuator3,&actuator4,&acceleration_z_float,&pitch_float,&roll_float);

            //printf("\x1b[1F\33[2K\r1: %u, 2: %u, 3: %u, 4: %u\n", actuator1, actuator2, actuator3, actuator4); //debug

            //write to dac refer to page 73 of Notebook for mapping, dependent on physical wiring
            ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_1, &actuator1);
            ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_2, &actuator2);
            ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_0, &actuator3);
            ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_3, &actuator4);
            #if defined TIMING_TESTING
            gpio_set_level(TIMING_OUT, 0);
            #endif
        }
        vTaskDelay(1); //yield so watchdog may be fed
        
    }
}

/**
    @brief Initialize the ISM330DHCX_device's SPI bus
*/
esp_err_t init_spi_IMU(void){
    esp_err_t ret;

    static const char init_spi_TAG[] = "IMU";
    ESP_LOGI(init_spi_TAG, "Initializing bus SPI%d...", IMU_HOST + 1);

    spi_bus_config_t *buscfg = malloc(sizeof(spi_bus_config_t)); //save this config in the heap
    memset(buscfg,0xFF,sizeof(spi_bus_config_t)); //disable extra config pins
    buscfg->miso_io_num = IMU_PIN_NUM_MISO;
    buscfg->mosi_io_num = IMU_PIN_NUM_MOSI;
    buscfg->sclk_io_num = IMU_PIN_NUM_CLK; 
    buscfg->quadwp_io_num = -1,
    buscfg->quadhd_io_num = -1,
    buscfg->max_transfer_sz = 7; //limit transfer sz to 7 bytes
    buscfg->flags = 0;
    buscfg->data_io_default_level = 0;
    buscfg->intr_flags = 0;
    buscfg->isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO;

    ret = spi_bus_initialize(IMU_HOST, buscfg, SPI_DMA_DISABLED); //initialize the bus using the controller defined in the preamble
    
    if(ret != 0){ // if we have an error
        return (ret); //cancel any further operation and return
    }
    //else continue setup

    ESP_LOGI(init_spi_TAG, "Init Success. Adding device...");
    
    spi_device_interface_config_t *devcfg = malloc(sizeof(spi_device_interface_config_t));
    memset(devcfg,0x00,sizeof(spi_device_interface_config_t));//zero out memory space
    devcfg->command_bits =      1;//one command bit (R/W)
    devcfg->address_bits =      7;//seven address bit (register address)
    devcfg->mode=               3;//Clock high in idle, read on rising edge
    devcfg->clock_source =      SPI_CLK_SRC_DEFAULT;
    devcfg->duty_cycle_pos =    128;//50%/50% duty cycle
    devcfg->cs_ena_pretrans =   0;//CS timed with clock
    devcfg->cs_ena_posttrans =  0;
    devcfg->clock_speed_hz=     9*1000*1000; //10Mhz
    devcfg->input_delay_ns =    0;
    devcfg->spics_io_num=       IMU_PIN_NUM_CS;
    devcfg->flags =             SPI_DEVICE_NO_DUMMY; //disable high freq(>100MHZ) workaround
    devcfg->queue_size=         7;  //queue 7 transactions at most
    devcfg->pre_cb =            NULL; //No callbacks
    devcfg->post_cb =           NULL;

    ret = spi_bus_add_device(IMU_HOST, devcfg, &IMU_spi);

    if(ret != 0){ // if we have an error
        return (ret); //cancel any further operation and return
    }

    ISM330DHCX_dev_ctx.write_reg = IMU_write;
    ISM330DHCX_dev_ctx.read_reg = IMU_read;
    ISM330DHCX_dev_ctx.mdelay = platform_delay;
    ISM330DHCX_dev_ctx.handle = IMU_spi;

    ESP_LOGI(init_spi_TAG, "Device driver ready for use");

    return(ret);

}

/**
    @brief Initialize the LTC2664 device's SPI bus
*/
esp_err_t init_spi_DAC(void){
    esp_err_t ret;

    static const char init_spi_TAG[] = "DAC";
    ESP_LOGI(init_spi_TAG, "Initializing bus SPI%d...", DAC_HOST + 1);

    spi_bus_config_t *buscfg = malloc(sizeof(spi_bus_config_t)); //save this config in the heap
    memset(buscfg,0xFF,sizeof(spi_bus_config_t)); //disable extra config pins
    buscfg->miso_io_num = DAC_PIN_NUM_MISO;
    buscfg->mosi_io_num = DAC_PIN_NUM_MOSI;
    buscfg->sclk_io_num = DAC_PIN_NUM_CLK;
    buscfg->quadwp_io_num = -1,
    buscfg->quadhd_io_num = -1,
    buscfg->max_transfer_sz = 3; //limit transfer sz to 7 bytes
    buscfg->flags = 0;
    buscfg->data_io_default_level = 0;
    buscfg->intr_flags = 0;
    buscfg->isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO;

    ret = spi_bus_initialize(DAC_HOST, buscfg, SPI_DMA_DISABLED); //initialize the bus using the controller defined in the preamble
    
    if(ret != 0){ // if we have an error
        return (ret); //cancel any further operation and return
    }
    //else continue setup

    ESP_LOGI(init_spi_TAG, "Init Success. Adding device...");
    
    spi_device_interface_config_t *devcfg = malloc(sizeof(spi_device_interface_config_t));
    memset(devcfg,0x00,sizeof(spi_device_interface_config_t));//zero out memory space
    devcfg->command_bits =      4;//four command bit (R/W)
    devcfg->address_bits =      4;//4 address bit (dac number)
    devcfg->mode=               0;//Clock low in idle, read on rising edge
    devcfg->clock_source =      SPI_CLK_SRC_DEFAULT;
    devcfg->duty_cycle_pos =    128; //50%/50% duty cycle
    devcfg->cs_ena_pretrans =   0;//CS timed with clock
    devcfg->cs_ena_posttrans =  0;
    devcfg->clock_speed_hz=     45*1000*1000;
    devcfg->input_delay_ns =    0;
    devcfg->spics_io_num=       DAC_PIN_NUM_CS;
    devcfg->flags =             SPI_DEVICE_NO_DUMMY; //disable high freq(>100MHZ) workaround
    devcfg->queue_size=         7;  //queue 7 transactions at most
    devcfg->pre_cb =            NULL; //No callbacks
    devcfg->post_cb =           NULL;

    ret = spi_bus_add_device(DAC_HOST, devcfg, &DAC_spi);

    if(ret != 0){ // if we have an error
        return (ret); //cancel any further operation and return
    }

    LTC2664_dev_ctx.write_reg = DAC_write;
    LTC2664_dev_ctx.handle = DAC_spi;

    ESP_LOGI(init_spi_TAG, "Device driver ready for use");

    return(ret);

}

/**
 * @brief Set PID constatnts and initialize the 3 controllers.
 * @attention WILL BLOCK until FLOAT_TIME has been initialized
 */
void init_pid_controllers(void){

    static const char TAG[] = "PID init";

    float_t derivative_cutoff_hz = 100.0f;
    float_t tau = 1.0f / (M_PI * derivative_cutoff_hz);
    /*  Limits defined for Integers as z,pitch,roll values 
        will be both positve and negative, easy to transform
        into uint16 as our DAC expects*/
    float_t angleUpperLimit = INT16_MAX/50.0;
    float_t angleLowerLimit = INT16_MIN/50.0;
    float_t accelLowerLimit = INT16_MAX/.2;
    float_t accelUpperLimit = INT16_MIN/.2;
    
    while (float_time == 0){
        vTaskDelay(10);
    }

    //controller constants, constants to be determined
    accel_z_controller.Kp = 0*160000.0f;//////////disabled
    accel_z_controller.Ki = 0*10000.0f;
    accel_z_controller.Kd = 0*30000.0f;
    accel_z_controller.tau = tau;
    accel_z_controller.limMin = accelLowerLimit;
    accel_z_controller.limMax = accelUpperLimit;
    accel_z_controller.T = float_time;

    ESP_LOGI(TAG, "Acceleration Controller Kp: %.3f", accel_z_controller.Kp );
    ESP_LOGI(TAG, "Ki: %.3f", accel_z_controller.Ki );
    ESP_LOGI(TAG, "Kd: %.3f", accel_z_controller.Kd);

    roll_controller.Kp = -5.0f;
    roll_controller.Ki = -70.0f;
    roll_controller.Kd = -10.0f;
    roll_controller.tau = tau;
    
    roll_controller.limMin = angleLowerLimit;
    roll_controller.limMax = angleUpperLimit;
    roll_controller.T = float_time;

    ESP_LOGI(TAG, "Roll Controller Kp: %.3f", roll_controller.Kp );
    ESP_LOGI(TAG, "Ki: %.3f", roll_controller.Ki );
    ESP_LOGI(TAG, "Kd: %.3f", roll_controller.Kd);

    pitch_controller.Kp = -5.0f;
    pitch_controller.Ki = -70.0f;
    pitch_controller.Kd = -10.0f;
    pitch_controller.tau = tau;
    pitch_controller.limMin = angleLowerLimit;
    pitch_controller.limMax = angleUpperLimit;
    pitch_controller.T = float_time;
    
    ESP_LOGI(TAG, "Pitch Controller Kp: %.3f", pitch_controller.Kp );
    ESP_LOGI(TAG, "Ki: %.3f", pitch_controller.Ki );
    ESP_LOGI(TAG, "Kd: %.3f", pitch_controller.Kd);

    PIDController_Init(&accel_z_controller);
    PIDController_Init(&roll_controller);
    PIDController_Init(&pitch_controller);

}

/**
    @brief configures the ISM330DHCX, checks that device is connected and working. Implements delay at start to allow device boot time. Must be after bus initialized.
*/
esp_err_t config_IMU(void){
    static const char Config_IMU_TAG[] = "IMU Config";
    vTaskDelay(IMU_BOOT_TIME / portTICK_PERIOD_MS);
    esp_err_t ret;
    uint8_t count = 0;
    uint8_t whoAmI, rst;

    //check that device is connected properly, try max 10 times
    do{
        ret = ism330dhcx_device_id_get(&ISM330DHCX_dev_ctx, &whoAmI);
        ESP_ERROR_CHECK(ret); //read error checked here
        platform_delay(10);
        count ++;
    }while(whoAmI != ISM330DHCX_ID && count < 10);
    ESP_LOGI(Config_IMU_TAG, "Device found...");
    if (whoAmI != ISM330DHCX_ID){
        return ESP_ERR_NOT_FOUND;
    }
    //end device check

    ret = ism330dhcx_reset_set(&ISM330DHCX_dev_ctx, PROPERTY_ENABLE);
    if (ret != 0){
        return(ret);
    }

    do{
        ESP_LOGI(Config_IMU_TAG, "In reset loop");
        ret = ism330dhcx_reset_get(&ISM330DHCX_dev_ctx,&rst);
        ESP_ERROR_CHECK(ret);
    } while (rst);

    /* Start device configuration. */
    ism330dhcx_device_conf_set(&ISM330DHCX_dev_ctx, PROPERTY_ENABLE);
    /* Enable Block Data Update */
    ism330dhcx_block_data_update_set(&ISM330DHCX_dev_ctx, PROPERTY_DISABLE);
    /* Set Output Data Rate */
    ism330dhcx_xl_data_rate_set(&ISM330DHCX_dev_ctx, ISM330DHCX_XL_ODR_6667Hz);
    ism330dhcx_gy_data_rate_set(&ISM330DHCX_dev_ctx, ISM330DHCX_GY_ODR_6667Hz);
    /* Set full scale */
    ism330dhcx_xl_full_scale_set(&ISM330DHCX_dev_ctx, ISM330DHCX_2g);
    ism330dhcx_gy_full_scale_set(&ISM330DHCX_dev_ctx, ISM330DHCX_500dps);
    /* Set Power Mode */
    ism330dhcx_xl_power_mode_set(&ISM330DHCX_dev_ctx,ISM330DHCX_HIGH_PERFORMANCE_MD);
    ism330dhcx_gy_power_mode_set(&ISM330DHCX_dev_ctx,ISM330DHCX_GY_HIGH_PERFORMANCE);
    /* Set Interupt 1 Output mode */
    ism330dhcx_pin_mode_set(&ISM330DHCX_dev_ctx,ISM330DHCX_PUSH_PULL);
    ism330dhcx_int_notification_set(&ISM330DHCX_dev_ctx,ISM330DHCX_BASE_LATCHED_EMB_PULSED);
    /* Set inturput type (interupt on drdy for accelerometer. in sync w/ gyroscope) */
    ism330dhcx_pin_int1_route_t pin_int1_route;
    ism330dhcx_pin_int1_route_get(&ISM330DHCX_dev_ctx, &pin_int1_route);
    pin_int1_route.int1_ctrl.int1_drdy_xl = PROPERTY_ENABLE;
    ism330dhcx_pin_int1_route_set(&ISM330DHCX_dev_ctx, &pin_int1_route);
    
    /* Gather frequency data from IMU */
    uint8_t freq;
    ism330dhcx_read_reg(&ISM330DHCX_dev_ctx,ISM330DHCX_INTERNAL_FREQ_FINE,&freq,1);
    /* Convert this raw data to seconds */
    float_time = 1.0/(6666.6666666666666666+(10.0*freq)); //1.0 coresponds to ODR_Coeff page 60 of ST app. note AN5398
    ESP_LOGI(Config_IMU_TAG, "Sample Period: %0.9f", float_time);

    /* Reset Timestamp */
    ism330dhcx_timestamp_rst(&ISM330DHCX_dev_ctx);

    ESP_LOGI(Config_IMU_TAG,"IMU Configured");
    return ret;

}

/**
 * @brief Find the offset where the Motor amplifier is outputting approximetly equal voltages on the A and B channel, stay at that offset.
 * @param dac number corresponding to dac to calibrate.
 */
esp_err_t DAC_find_offset(ltc2664_DACS_t dac){
    esp_err_t ret;
    
    static const char TAG[] = "DAC";

    //zero out the offsets used in writing to the dac
    int32_t signed_offset = 0;
    ltc2664_save_offset(dac, &signed_offset);

    uint16_t input = UINT16_MAX/2;
    uint16_t adjuster = input/64;
    //initialize the dac to midscale
    ret = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, dac, &input);
    //assign mux to match selected dac
    uint8_t mux[4] = {0,3,2,1};

    gpio_set_level(MUX_PIN_A0, mux[dac] & 0x0001);
    gpio_set_level(MUX_PIN_A1, (mux[dac] & 0x0002) >> 1);
    //narrow the adjuster while moving the input value up or down according to the comparator input
    while(adjuster > 2){
        vTaskDelay(15); //allow the DAC, Comparator, and amplifier to settle (set to have a single dac calibrated in less than 1 second)(70)
        if(gpio_get_level(COMPARATOR_PIN)){ //may need to flip this condition to match actual amplifier response.
            input -= adjuster;
            //ESP_LOGI(TAG, "high input"); //debug
        }
        else{
            input += adjuster;
            //ESP_LOGI(TAG, "low input"); //debug
        }
        ret = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, dac, &input);
        if(ret != ESP_OK) return ret;
        adjuster = adjuster/2;
    }
    signed_offset = input-(UINT16_MAX/2);
    //save that offset to be used in DAC writing function;
    ltc2664_save_offset(dac, &signed_offset);

    ESP_LOGI(TAG, "DAC %i zero code: %i", dac, input);

    return ret;
}

/**
 * @brief All setup for DAC goes in this function, includes zero-offset finding
 */
esp_err_t config_DAC(void){
    static const char TAG[] = "DAC";
    ESP_LOGI(TAG, "Calibrating Motor Amplifiers");
    esp_err_t ret;
    for(int dac = 0; dac < 4; dac++){ //brute force the possible dacs found in ltc2664_DACS_t
        ret = DAC_find_offset(dac); //find offsets for all of them.
        if(ret != ESP_OK) return ret;
    }
    return ret;
    ESP_LOGI(TAG, "Calibration complete, DAC ready");
}

void app_main(void){
    static const char TAG[] = "Main";
    esp_err_t err;
    #if defined DAC_TEST
    //test dac
    ESP_LOGI(TAG, "Testing DAC: ");
    float_t rad = 0;
    uint16_t dac_input = 0;
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = GPIO_MUX_PINS;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    err = init_spi_DAC();
    ESP_ERROR_CHECK(err);
    err = config_DAC();
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "Done Calibration");
    // while (1) //testing calibration
    // {
    //     vTaskDelay(1000);
    // }
    
    while(1){ //sine wave test
        rad += M_2_PI/200.0;
        //if (rad > M_2_PI) rad = 0.0001;
        dac_input = (uint16_t)32000 * (1 + 0.5*sinf(rad));
        err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_3, &dac_input);
        err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_2, &dac_input);
        err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_1, &dac_input);
        err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_0, &dac_input);
        platform_delay(10);
    }


    // uint16_t n_two_halfV = 16000;
    // uint16_t zeroV = 32000;
    // uint16_t p_two_halfV = 48000;
    // while(1){ //level testing
    //     //one second loop, multi step response from DACs go from -2.5V to 0V to 2.5V to 0V, repeat
    //     // err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_0, &n_two_halfV);
    //     // ESP_ERROR_CHECK(err);
    //     // err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_1, &p_two_halfV);
    //     // ESP_ERROR_CHECK(err);
    //     // err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_2, &p_two_halfV);
    //     // ESP_ERROR_CHECK(err);
    //     err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_3, &n_two_halfV);
    //     ESP_LOGI(TAG, "Negative ");
    //     ESP_ERROR_CHECK(err);
    //     platform_delay(3000);
    //     err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_3, &p_two_halfV);
    //     ESP_LOGI(TAG, "Positive ");
    //     ESP_ERROR_CHECK(err);
    //     // err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_2, &n_two_halfV);
    //     // ESP_ERROR_CHECK(err);
    //     // err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_1, &n_two_halfV);
    //     // ESP_ERROR_CHECK(err);
    //     // err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_0, &p_two_halfV);
    //     ESP_ERROR_CHECK(err);
    //     platform_delay(3000);
    //     err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_3, &zeroV);
    //     ESP_LOGI(TAG, "Zero ");
    //     ESP_ERROR_CHECK(err);
    //     platform_delay(3000);
    //     platform_delay(3000);
    // }
    
   
    #else
    //normal operation

/* GPIO CONFIGURATION SECTION */
    /* Configure Interupt Pin For IMU */
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = GPIO_INT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 1;
    gpio_config(&io_conf);
    /* Configure Mux Address Pins */
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = GPIO_MUX_PINS;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    /* Configure Comparator Signal Pin */
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = GPIO_COMP_PINS;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    /* Configure Status LED Pin */
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = GPIO_STATUS_PINS;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    /* Configure Mode Switches Pin */
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = GPIO_MODE_SWS;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    /* Read boot config switches, write global bools */
    if(gpio_get_level(MODE_SW_EN_LOGGING) == 1) en_logging = true;
    if(gpio_get_level(MODE_SW_EN_ACTUATOR) == 1) en_actuators = true;

    /* Turn off Status LED, not ready yet */
    gpio_set_level(STATUS_LED2, 0);    
    gpio_set_level(STATUS_LED1, 0);

    /* Create Queue for transfering data to SD card function */
    log_data_queue = xQueueCreate(7000, sizeof(log_data_t));

    /* Start tasks for IMU sampling*/
    ESP_LOGI(TAG, "Starting IMU Task");
    xTaskCreatePinnedToCore(IMU_sample_task, "IMU_sample_task", 65536, NULL, 9, &IMU_TASK, APP_CPU_NUM);

    if(en_logging == true){//do not continue if logging is not enabled
        /* Initialize SD Card*/
        esp_vfs_fat_sdmmc_mount_config_t mount_config = {
            .max_files = 5,
            .allocation_unit_size = 16 * 1024
        };
        sdmmc_card_t *card;
        const char mount_point[] = MOUNT_POINT;
        ESP_LOGI(TAG, "Initializing SD card");

        ESP_LOGI(TAG, "Using SDMMC peripheral");

        //Increase the speed here as much as possible
        sdmmc_host_t host = SDMMC_HOST_DEFAULT();
        host.max_freq_khz = 45000;

        sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
        slot_config.width = 4;
        slot_config.clk = SD_PIN_CLK;
        slot_config.cmd = SD_PIN_CMD;
        slot_config.d0 = SD_PIN_D0;
        slot_config.d1 = SD_PIN_D1;
        slot_config.d2 = SD_PIN_D2;
        slot_config.d3 = SD_PIN_D3;
        slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

        ESP_LOGI(TAG, "Mounting filesystem");
        err = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);

        if (err != ESP_OK) {
            if (err == ESP_FAIL) {
                ESP_LOGE(TAG, "Failed to mount filesystem.");
            } else {
                ESP_LOGE(TAG, "Failed to initialize the card (%s).", esp_err_to_name(err));
            }
            return;
        }
        ESP_LOGI(TAG, "Filesystem mounted");

        sdmmc_card_print_info(stdout, card);

        char temp_string[33];
        sprintf(temp_string, "%i", rand());

        char *file_name = MOUNT_POINT"/log.csv"; ///find a way to insert temp string here....
        char *cache = malloc(CACHE_SIZE);
        
        ESP_LOGI(TAG, "Opening file %s", file_name);
        FILE *f = fopen(file_name, "a");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return;
        }

        /* Log title block*/
        int written_length = snprintf(temp_string, 32, "Log #: %i\n", rand() % 9999);
        memcpy(cache, temp_string, written_length);
        uint32_t lines = written_length;
        if(en_actuators){
            written_length = snprintf(temp_string, 32, "Actuators enabled\n");
        }
        else{
            written_length = snprintf(temp_string, 32, "Actuators disabled\n");
        }
        memcpy(cache + (lines), temp_string, written_length);
        lines += written_length;
        written_length = snprintf(temp_string, 32, "Acceleration, Pitch, Roll");
        memcpy(cache + (lines), temp_string, written_length);
        lines += written_length;

        ESP_LOGI(TAG, "Begin Normal Operation");
        log_data_t log_data;
        while(gpio_get_level(MODE_SW_START_LOG) == 0){
            vTaskDelay(100);
        }
        //clear the queue right before we start saving data.
        xQueueReset(log_data_queue);
        while(1){
            if(xQueueReceive(log_data_queue,&log_data, 0)){
                written_length = snprintf(temp_string, 32, "%.4f, %.4f, %.4f\n", log_data.acceleration_z, log_data.pitch, log_data.roll);
                //printf("%i\n",(int)(cache + 32*lines));
                memcpy(cache + (lines), temp_string, written_length);
                lines += written_length;
                if(lines >= CACHE_SIZE-64){ //lose as much as 1 second of data by saving approx. every second.
                    cache[lines] = '\0';
                    lines = 0;
                    fprintf(f, cache);
                    fclose(f);
                    if(gpio_get_level(MODE_SW_START_LOG) == 0){
                        /* Log title block*/
                        written_length = snprintf(temp_string, 32, "Log #: %i\n", rand() % 9999);
                        memcpy(cache, temp_string, written_length);
                        lines += written_length;
                        if(en_actuators){
                            written_length = snprintf(temp_string, 32, "Actuators enabled\n");
                        }
                        else{
                            written_length = snprintf(temp_string, 32, "Actuators disabled\n");
                        }
                        memcpy(cache + (lines), temp_string, written_length);
                        lines += written_length;
                        written_length = snprintf(temp_string, 32, "Acceleration, Pitch, Roll");
                        memcpy(cache + (lines), temp_string, written_length);
                        lines += written_length;
                        //wait till next log should begin
                        while(gpio_get_level(MODE_SW_START_LOG) == 0){
                            vTaskDelay(100);
                        }
                    }
                    f = fopen(file_name, "a");
                    printf("%i\n",(int)uxQueueMessagesWaiting(log_data_queue)); //report number of data points waiting to be stored
                }
            }
            else
            {
                vTaskDelay(1);
            }
        }
    }//logging enabled if statement
    else{//logging disabled, do nothing on this core
        while(1){
            log_data_t log_data;
            if(xQueueReceive(log_data_queue,&log_data, portMAX_DELAY)){
                printf("\r%i", (int)uxQueueMessagesWaiting(log_data_queue)); //debug
            }
            else
            {
                vTaskDelay(1);
            }


        } 
        //vTaskDelay(1000);//wait forever
    }
    #endif
}


static void platform_delay(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

/**
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t IMU_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
    esp_err_t ret;    
    spi_transaction_t t;
    memset( &t, 0x00, sizeof(t) );
    t.cmd = 1;
    t.addr = reg;
    t.length = len*8;
    t.rxlength = len*8;
    t.rx_buffer = bufp;
    t.flags = 0;   

    ret = spi_device_transmit(handle,&t);

    return (int)ret;
}

/**
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t IMU_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
    esp_err_t ret;    
    spi_transaction_t t;
    memset( &t, 0x00, sizeof(t) );
    t.cmd = 0;
    t.addr = reg;
    t.length = len*8;
    t.tx_buffer = bufp;
    t.flags = 0;   

    ret = spi_device_transmit(handle,&t);

    return (int)ret;
}

/**
 * @brief Write DAC register
 * 
 * @param   handle  pointer to spi device handle
 * @param   dac     DAC number to write to
 * @param   command Command to to write
 * @param   data    pointer to data to write
 * 
 * @return  ret     0 = no error
 */
static int32_t DAC_write(void *handle, ltc2664_DACS_t dac, uint8_t command, const uint16_t *data)
{
    uint16_t temp_data = ((*data & 0xFF00) >> 8) |((*data & 0x00FF) << 8);
    esp_err_t ret;
    spi_transaction_t t;
    memset( &t, 0x00, sizeof(t) );
    t.cmd = command;
    t.addr = dac;
    t.length = 16;
    t.tx_buffer = &temp_data;
    t.flags = 0;

    ret = spi_device_transmit(handle, &t);

    return (int)ret;
}
