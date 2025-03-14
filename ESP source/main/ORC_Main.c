/**
  ******************************************************************************
  * @file    ORC_Main.c
  * @author  Corbin Roennebeck, WSU
  * @brief   Controls actuators and logs data for ORC project as defined by 
  *             "Theory of Operation", below.
  ******************************************************************************
  */

  /**
   *    Theory of Operation:
   *        The motions of 4 actuators between the vehicle frame and axle,
   *        located both front and rear on both the driver and passenger side,
   *        is controlled by 3 PID controllers. These controlers use measured
   *        vehicle dynamics - vertical acceleration (less gravity), pitch, and 
   *        roll - to define actuator motion that cancels out the aforementioned 
   *        vehicle dynamics. The measured vehicle dynamics are also capable of
   *        being logged to the onboard SD card. These vehicle dynamics are
   *        measured by the attached IMU, filtered onboard the IMU, and processed
   *        by the AHRS Fusion library to provide accurate, realtime information.
   *
   *    More detailed information can be found in the "ORC Documentation.pdf" file
   */

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
#include "esp_random.h"

#include "Fusion.h"
#include "LTC2664_reg.h"
#include "ism330dhcx_reg.h"
#include "Transform.h"
#include "PID.h"

#include "ORC_CONFIG.h"

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

//time between IMU samples, reported by IMU
static float_t float_time = 0;

//PID Contoller Handles
static PIDController_t accel_z_controller={};
static PIDController_t pitch_controller={};
static PIDController_t roll_controller={};

//Boot Config booleans
static bool en_logging = false;
static bool en_actuators = false;


/**
 * @brief Capture interupt from IMU and tell the IMU_sample_task there's data avalible
 */
static void IRAM_ATTR imu_isr_handler(void* arg){
    BaseType_t higherPriorityTask = pdTRUE;
    vTaskNotifyGiveFromISR(IMU_TASK,&higherPriorityTask);
}

/**
 * @brief Initialize the IMU, it's interupts, and the fusion library, then perform continuous sampling.
 */
static void IMU_sample_task(void* arg)
{
    esp_err_t err;
    //buffers for IMU data to be placed in
    int16_t data_raw_acceleration[3];
    int16_t data_raw_angular_rate[3];
    //data frame for log data to be queued
    log_data_t df;
    //PID outputs
    float_t acceleration_z_float = 0;
    float_t pitch_float;
    float_t roll_float;
    //Fusion handle, outputs, and inputs
    FusionAhrs ahrs;
    FusionEuler euler;
    FusionVector linear;
    FusionVector gyroscope = {{0,0,0}};
    FusionVector accelerometer = {{0,0,0}};
    //Transformed PID outputs to be sent to DAC
    uint16_t actuator1, actuator2, actuator3, actuator4;

    const char TAG[] = "IMU Sampling Task";

    /* Initialize and configure IMU*/
    err = init_spi_IMU();
    ESP_ERROR_CHECK(err);
    err = config_IMU();
    ESP_ERROR_CHECK(err);        
    
    //set up fusion algorithm, use standard values as defined in Fusion Library
    FusionAhrsInitialise(&ahrs);
    FusionAhrsSettings * const ahrs_settings = malloc(sizeof(FusionAhrsSettings));
    ahrs_settings->accelerationRejection =  10; //degrees
    ahrs_settings->gyroscopeRange =         500;//dps
    ahrs_settings->gain =                   .5; //influence of gyroscope
    ahrs_settings->convention =             FusionConventionEnu; //earth axes convention (consistent with ISM330DHCX)
    ahrs_settings->recoveryTriggerPeriod =  0;//5/float_time; //approx 5 second recovery trigger period
    FusionAhrsSetSettings(&ahrs, ahrs_settings);

    ESP_LOGI(TAG, "AHRS Initialized");

    /* Initialize PID Controllers */
    init_pid_controllers();

    /* Initialize  and configure DAC */
    err = init_spi_DAC();
    ESP_ERROR_CHECK(err);
    err = config_DAC();
    ESP_ERROR_CHECK(err);

    /* Initialize Transform Matrix Distances */
    err = set_distances(FA_COG,RA_COG,DA_COG,PA_COG);
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG,"Distaces set\nF-%.3f\nR-%.3f\nD-%.3f\nP-%.3f", FA_COG, RA_COG, DA_COG, PA_COG);
    
    /* Calibrate the rest angle and acceleration for the vehicle */
    /*-----------------------------------------------------------*/
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
        //reuse old data if peak detected
        for(int i=0; i<3; i++){
            if(data_raw_angular_rate[i] < IMU_GY_PEAK_REJ && data_raw_angular_rate[i] > -IMU_GY_PEAK_REJ){
                gyroscope.array[i] = (float_t)data_raw_angular_rate[i] * 0.0175f;
            }
        }

        //convert to g for use in Fusion 
        //uses full scale of 4 g for int to float conversion
        //reuse old data if peak detected
        for(int i=0; i<3; i++){
            if(data_raw_angular_rate[i] < IMU_XL_PEAK_REJ && data_raw_angular_rate[i] > -IMU_XL_PEAK_REJ){
                accelerometer.array[i] = (float_t)data_raw_acceleration[i] * 0.000122f;
            }
        }
        //update Fusion
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, float_time);
        //refresh flags
        fusionFlags = FusionAhrsGetFlags(&ahrs);
    }
    //once data is stable, set zeros to use for PID setpoint
    euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
    float_t acceleration_zero = 0;  //set to zero, as noise crushing performed on xl data later would remove this offset
    float_t pitch_zero = euler.angle.pitch;
    float_t roll_zero = euler.angle.roll;

    ESP_LOGI(TAG, "Acceleration zero point: %.3f", acceleration_zero);
    ESP_LOGI(TAG, "Pitch zero point: %.3f", pitch_zero);
    ESP_LOGI(TAG, "Roll zero point: %.3f", roll_zero);

    //delay to ensure SD card is rady
    vTaskDelay(100);

    /* Turn on Status LEDs */+
    gpio_set_level(STATUS_LED2, 1);
    gpio_set_level(STATUS_LED1, 1);

    /* Read a register from the IMU to kick off interupt sampling again, log collected data, control actuators as needed */
    /*-------------------------------------------------------------------------------------------------------------------*/
    ism330dhcx_angular_rate_raw_get(&ISM330DHCX_dev_ctx, data_raw_angular_rate);
    ism330dhcx_acceleration_raw_get(&ISM330DHCX_dev_ctx, data_raw_acceleration);
    ESP_LOGI(TAG,"Sampling started");

    while(1){
        ulTaskNotifyTake(pdFALSE,portMAX_DELAY);
        //when IMU_PIN_INT1 rising edge

        //read accel/gyro
        memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
        ism330dhcx_acceleration_raw_get(&ISM330DHCX_dev_ctx, data_raw_acceleration);
        ism330dhcx_angular_rate_raw_get(&ISM330DHCX_dev_ctx, data_raw_angular_rate);
        
        //convert to dps for use in Fusion
        //uses full scale of 500dps for int to float conversion
        //reuse old data if peak detected
        for(int i=0; i<3; i++){
            if(data_raw_angular_rate[i] < IMU_GY_PEAK_REJ && data_raw_angular_rate[i] > -IMU_GY_PEAK_REJ){
                gyroscope.array[i] = (float_t)data_raw_angular_rate[i] * 0.0175f;
            }
        }

        //convert to g for use in Fusion 
        //uses full scale of 4 g for int to float conversion
        //reuse old data if peak detected
        for(int i=0; i<3; i++){
            if(data_raw_angular_rate[i] < IMU_XL_PEAK_REJ && data_raw_angular_rate[i] > -IMU_XL_PEAK_REJ){
                accelerometer.array[i] = (float_t)data_raw_acceleration[i] * 0.000122f;
            }
        }
        //update Fusion, and get the resulting values
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, float_time);
        euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs)); //angles
        linear = FusionAhrsGetLinearAcceleration(&ahrs); //acceleration less gravity

        //send data to be logged
        df.acceleration_z = linear.axis.z;
        df.pitch = euler.angle.pitch;
        df.roll = euler.angle.roll;
        xQueueSendToBack(log_data_queue,&df,0);

        //crush acceleration data to 0 if below noise floor, crushing is based on absolute value of input data
        float_t conditioned_acceleration_z = 0;
        if (linear.axis.z > IMU_NOISE_FLOOR){
            conditioned_acceleration_z = linear.axis.z - IMU_NOISE_FLOOR;
        }
        else if (linear.axis.z < -IMU_NOISE_FLOOR){
            conditioned_acceleration_z = linear.axis.z + IMU_NOISE_FLOOR;
        }

        if(en_actuators == true){// below sections not needed if actuators disabled
            //pid updates
            acceleration_z_float = PIDController_Update(&accel_z_controller, &acceleration_zero, &conditioned_acceleration_z);
            pitch_float = PIDController_Update(&pitch_controller, &pitch_zero, &euler.angle.pitch);
            roll_float = PIDController_Update(&roll_controller, &roll_zero, &euler.angle.roll);

            //transform
            transform(&actuator1,&actuator2,&actuator3,&actuator4,&acceleration_z_float,&pitch_float,&roll_float);

            //write to dac refer to page 73 of Notebook or design document for mapping, dependent on physical wiring
            ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_1, &actuator1);
            ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_2, &actuator2);
            ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_0, &actuator3);
            ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_3, &actuator4);

        }        
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
    buscfg->max_transfer_sz = 8; //limit transfer sz to 8 bytes
    buscfg->flags = 0;
    buscfg->data_io_default_level = 0;
    buscfg->intr_flags = 0;
    buscfg->isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO;

    ret = spi_bus_initialize(IMU_HOST, buscfg, SPI_DMA_DISABLED); //initialize the bus using the controller defined in the preamble
    
    if(ret != ESP_OK){ // if we have an error
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
    devcfg->clock_speed_hz=     10*1000*1000; //10Mhz
    devcfg->input_delay_ns =    0;
    devcfg->spics_io_num=       IMU_PIN_NUM_CS;
    devcfg->flags =             SPI_DEVICE_NO_DUMMY; //disable high freq(>100MHZ) workaround
    devcfg->queue_size=         7;  //queue 7 transactions at most
    devcfg->pre_cb =            NULL; //No callbacks
    devcfg->post_cb =           NULL;

    ret = spi_bus_add_device(IMU_HOST, devcfg, &IMU_spi);

    if(ret != ESP_OK){ // if we have an error
        return (ret); //cancel any further operation and return
    }

    //now the IMU device can have it's associated functions and handle defined
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
    buscfg->max_transfer_sz = 3; //limit transfer sz to 3 bytes
    buscfg->flags = 0;
    buscfg->data_io_default_level = 0;
    buscfg->intr_flags = 0;
    buscfg->isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO;

    ret = spi_bus_initialize(DAC_HOST, buscfg, SPI_DMA_DISABLED); //initialize the bus using the controller defined in the preamble
    
    if(ret != ESP_OK){ // if we have an error
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
    devcfg->clock_speed_hz=     50*1000*1000;//50Mhz
    devcfg->input_delay_ns =    0;
    devcfg->spics_io_num=       DAC_PIN_NUM_CS;
    devcfg->flags =             SPI_DEVICE_NO_DUMMY; //disable high freq(>100MHZ) workaround
    devcfg->queue_size=         7;  //queue 7 transactions at most
    devcfg->pre_cb =            NULL; //No callbacks
    devcfg->post_cb =           NULL;

    ret = spi_bus_add_device(DAC_HOST, devcfg, &DAC_spi);

    if(ret != ESP_OK){ // if we have an error
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
        into uint16 as our DAC expects. Constrain PID 
        outputs to not oversaturate the DAC/PID output */
    float_t max_small_angle_coeff_accel = 1.1f * MAX(get_small_angle_a_2ab(),get_small_angle_b_2ab());
    float_t max_small_angle_coeff_angle = 1.1f * MAX(get_small_angle_1_2ab(),get_small_angle_1_2cd());
    float_t accelUpperLimit = INT16_MAX/max_small_angle_coeff_accel;
    float_t accelLowerLimit = INT16_MIN/max_small_angle_coeff_accel;
    float_t angleUpperLimit = INT16_MAX/max_small_angle_coeff_angle;
    float_t angleLowerLimit = INT16_MIN/max_small_angle_coeff_angle;
    
    /* PIDs require float_time variable to be initialized, stall till ready */
    while (float_time == 0){
        vTaskDelay(10);
    }

    //controller constants assigned
    accel_z_controller.Kp = Z_XL_KP;
    accel_z_controller.Ki = Z_XL_KI;
    accel_z_controller.Kd = Z_XL_KD;
    accel_z_controller.tau = tau;
    accel_z_controller.limMin = accelLowerLimit;
    accel_z_controller.limMax = accelUpperLimit;
    accel_z_controller.T = float_time;

    pitch_controller.Kp = ROLL_KP;
    pitch_controller.Ki = ROLL_KI;
    pitch_controller.Kd = ROLL_KD;
    pitch_controller.tau = tau;
    pitch_controller.limMin = angleLowerLimit;
    pitch_controller.limMax = angleUpperLimit;
    pitch_controller.T = float_time;

    roll_controller.Kp = PITCH_KP;
    roll_controller.Ki = PITCH_KI;
    roll_controller.Kd = PITCH_KD;
    roll_controller.tau = tau;
    roll_controller.limMin = angleLowerLimit;
    roll_controller.limMax = angleUpperLimit;
    roll_controller.T = float_time;


    ESP_LOGI(TAG, "Acceleration Controller Kp: %.3f", accel_z_controller.Kp );
    ESP_LOGI(TAG, "Ki: %.3f", accel_z_controller.Ki );
    ESP_LOGI(TAG, "Kd: %.3f", accel_z_controller.Kd);    
    ESP_LOGI(TAG, "Pitch Controller Kp: %.3f", pitch_controller.Kp );
    ESP_LOGI(TAG, "Ki: %.3f", pitch_controller.Ki );
    ESP_LOGI(TAG, "Kd: %.3f", pitch_controller.Kd);
    ESP_LOGI(TAG, "Roll Controller Kp: %.3f", roll_controller.Kp );
    ESP_LOGI(TAG, "Ki: %.3f", roll_controller.Ki );
    ESP_LOGI(TAG, "Kd: %.3f", roll_controller.Kd);

    PIDController_Init(&accel_z_controller);
    PIDController_Init(&roll_controller);
    PIDController_Init(&pitch_controller);

}

/**
    @brief configures the ISM330DHCX, checks that device is connected and working. Implements delay at start to allow device boot time. Must be after bus initialized.
*/
esp_err_t config_IMU(void){
    static const char Config_IMU_TAG[] = "IMU Config";
    //give the IMU enough time to properly boot
    vTaskDelay(IMU_BOOT_TIME / portTICK_PERIOD_MS);
    esp_err_t ret;
    //attempt to check device ID counter
    uint8_t count = 0;
    //
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

    //reset the device to factory settings
    ret = ism330dhcx_reset_set(&ISM330DHCX_dev_ctx, PROPERTY_ENABLE);
    if (ret != 0){
        return(ret);
    }
    //check that reset worked
    do{
        ESP_LOGI(Config_IMU_TAG, "In reset loop");
        ret = ism330dhcx_reset_get(&ISM330DHCX_dev_ctx,&rst);
        ESP_ERROR_CHECK(ret);
    } while (rst);
    //end device reset

    /* Start device configuration. */
    ret &= ism330dhcx_device_conf_set(&ISM330DHCX_dev_ctx, PROPERTY_ENABLE);
    /* Enable Block Data Update */
    ret &= ism330dhcx_block_data_update_set(&ISM330DHCX_dev_ctx, PROPERTY_DISABLE);
    /* Set Output Data Rate */
    ret &= ism330dhcx_xl_data_rate_set(&ISM330DHCX_dev_ctx, ISM330DHCX_XL_ODR_3332Hz);
    ret &= ism330dhcx_gy_data_rate_set(&ISM330DHCX_dev_ctx, ISM330DHCX_GY_ODR_3332Hz);
    /* Set full scale */
    ret &= ism330dhcx_xl_full_scale_set(&ISM330DHCX_dev_ctx, ISM330DHCX_4g);
    ret &= ism330dhcx_gy_full_scale_set(&ISM330DHCX_dev_ctx, ISM330DHCX_500dps);
    /* Set Power Mode to high performance */
    ret &= ism330dhcx_xl_power_mode_set(&ISM330DHCX_dev_ctx,ISM330DHCX_HIGH_PERFORMANCE_MD);
    ret &= ism330dhcx_gy_power_mode_set(&ISM330DHCX_dev_ctx,ISM330DHCX_GY_HIGH_PERFORMANCE);
    /* Internal filtering setup */
    ret &= ism330dhcx_xl_hp_path_on_out_set(&ISM330DHCX_dev_ctx,ISM330DHCX_LP_ODR_DIV_800);
    ret &= ism330dhcx_xl_filter_lp2_set(&ISM330DHCX_dev_ctx, PROPERTY_ENABLE);
    ret &= ism330dhcx_gy_hp_path_internal_set(&ISM330DHCX_dev_ctx,ISM330DHCX_HP_FILTER_65mHz);
    ret &= ism330dhcx_gy_lp1_bandwidth_set(&ISM330DHCX_dev_ctx, ISM330DHCX_ULTRA_LIGHT);
    ret &= ism330dhcx_gy_filter_lp1_set(&ISM330DHCX_dev_ctx, PROPERTY_ENABLE);
    /* Set Interupt 1 Output mode */
    ret &= ism330dhcx_pin_mode_set(&ISM330DHCX_dev_ctx, ISM330DHCX_PUSH_PULL);
    ret &= ism330dhcx_int_notification_set(&ISM330DHCX_dev_ctx, ISM330DHCX_BASE_LATCHED_EMB_PULSED);
    /* Set interupt type (interupt on drdy for accelerometer. in sync w/ gyroscope) */
    ism330dhcx_pin_int1_route_t pin_int1_route;
    ret &= ism330dhcx_pin_int1_route_get(&ISM330DHCX_dev_ctx, &pin_int1_route);
    pin_int1_route.int1_ctrl.int1_drdy_xl   = PROPERTY_ENABLE;
    pin_int1_route.int1_ctrl.int1_boot      = PROPERTY_DISABLE;
    pin_int1_route.int1_ctrl.int1_cnt_bdr   = PROPERTY_DISABLE;
    pin_int1_route.int1_ctrl.den_drdy_flag  = PROPERTY_DISABLE;
    pin_int1_route.int1_ctrl.int1_drdy_g    = PROPERTY_DISABLE;
    pin_int1_route.int1_ctrl.int1_fifo_full = PROPERTY_DISABLE;
    pin_int1_route.int1_ctrl.int1_fifo_ovr  = PROPERTY_DISABLE;
    pin_int1_route.int1_ctrl.int1_fifo_th   = PROPERTY_DISABLE;
    ret &= ism330dhcx_pin_int1_route_set(&ISM330DHCX_dev_ctx, &pin_int1_route);
    
    /* Gather frequency data from IMU */
    uint8_t freq;
    ret &= ism330dhcx_read_reg(&ISM330DHCX_dev_ctx,ISM330DHCX_INTERNAL_FREQ_FINE,&freq,1);
    /* Convert this raw data to seconds */
    float_time = 2.0/(6666.6666666666666666+(10.0*freq)); //1.0 coresponds to ODR_Coeff page 60 of ST app. note AN5398
    ESP_LOGI(Config_IMU_TAG, "Sample Period: %0.9f", float_time);

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
    if(ret != ESP_OK) return ret;
    //assign mux to match selected dac, mapping described in "ORC Documentation.pdf"
    uint8_t mux[4] = {0,3,2,1};

    gpio_set_level(MUX_PIN_A0, mux[dac] & 0x0001);
    gpio_set_level(MUX_PIN_A1, (mux[dac] & 0x0002) >> 1);
    //narrow the adjuster while moving the input value up or down according to the comparator input
    while(adjuster > 2){
        vTaskDelay(15); //allow the DAC, Comparator, and amplifier to settle
        if(gpio_get_level(COMPARATOR_PIN)){ //comparator determines if output A is greater than B for the given channel
            input -= adjuster;  //and adjust the offset value approprietly
        }
        else{
            input += adjuster;
        }
        ret = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, dac, &input); //see if the new offset is correct
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
    ESP_LOGI(TAG, "Calibration complete, DAC ready");
    return ret;
}

/**
 * @brief Main function. Initializes all GPIO, starts IMU task, and does SD logging
 */
void app_main(void){
    static const char TAG[] = "Main";
    esp_err_t err;

/* GPIO CONFIGURATION SECTION */
    /* Configure Interupt Pin For IMU */
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = GPIO_INT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
    /* Configure IMU_MISO Pin */
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = GPIO_MISO_PIN;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
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
    if(gpio_get_level(MODE_SW_EN_LOGGING) == 1){
        ESP_LOGI(TAG,"Logging Enabled");
        en_logging = true; 
    }
    if(gpio_get_level(MODE_SW_EN_ACTUATOR) == 1){ 
        ESP_LOGI(TAG,"Actuators Enabled");
        en_actuators = true;
    }
    /* Turn off Status LED, not ready yet */
    gpio_set_level(STATUS_LED2, 0);    
    gpio_set_level(STATUS_LED1, 0);

    /* Create Queue for transfering data to SD card function */
    log_data_queue = xQueueCreate(7000, sizeof(log_data_t));

    /* Start tasks for IMU sampling*/
    ESP_LOGI(TAG, "Starting IMU Task");
    xTaskCreatePinnedToCore(IMU_sample_task, "IMU_sample_task", 4096, NULL, 9, &IMU_TASK, APP_CPU_NUM);

    /* Set up ISR */
    ESP_LOGI(TAG, "Installing ISR");
    err = gpio_install_isr_service(0);
    ESP_ERROR_CHECK(err);
    err = gpio_isr_handler_add(IMU_PIN_INT1, imu_isr_handler, (void*) IMU_PIN_INT1);
    ESP_ERROR_CHECK(err);   

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

        char temp_string[33];//char array to have each line of log temporarily printed to. allows formating to be applied
        char *cache = malloc(CACHE_SIZE);   //cache allocation to store blocks of data to write to SD card, helps maintain write speed as less wear leveling takes place

        char *file_name = MOUNT_POINT"/ORClog.csv";//specify the file to be written to
        
        //attempt to open file
        ESP_LOGI(TAG, "Opening file %s", file_name);
        FILE *f = fopen(file_name, "a");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return;
        }

        /* Log title block*/
        int written_length = snprintf(temp_string, 32, "Log #: %li\n", esp_random() % 10000);//random 4 digit log number, differentiates multiple test in one file
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
        while(float_time == 0){
            vTaskDelay(100);
        }
        written_length = snprintf(temp_string, 32, "Interval:%f\n", float_time);
        memcpy(cache + (lines), temp_string, written_length);
        lines += written_length;
        written_length = snprintf(temp_string, 32, "Acceleration, Pitch, Roll\n");
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
                //when a new sample comes in, write it to the cache
                written_length = snprintf(temp_string, 32, "%.4f, %.4f, %.4f\n", log_data.acceleration_z, log_data.pitch, log_data.roll);
                memcpy(cache + (lines), temp_string, written_length);
                lines += written_length;
                //if the cahce is nearly full, write it to the sd card
                if(lines >= CACHE_SIZE-64){ //lose as much as 1 second of data by saving approx. every second.
                    cache[lines] = '\0';
                    lines = 0;
                    fprintf(f, cache);
                    //close file, saves data written
                    fclose(f);
                    if(gpio_get_level(MODE_SW_START_LOG) == 0){//if log not paused
                        //wait till next log should begin
                        while(gpio_get_level(MODE_SW_START_LOG) == 0){
                            vTaskDelay(100);
                        }

                        if(gpio_get_level(MODE_SW_APPEND_LOG)==0){ //if not appending to log, start new entry                        
                            //see if actuator status has changed
                            if(gpio_get_level(MODE_SW_EN_ACTUATOR) == 1){ 
                                ESP_LOGI(TAG,"Actuators Enabled");
                                en_actuators = true;
                            }
                            else{
                                ESP_LOGI(TAG,"Actuators Disabled");
                                en_actuators = false;
                            }
                            /* Log title block*/
                            written_length = snprintf(temp_string, 32, "Log #: %li\n", esp_random() % 10000);
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
                            written_length = snprintf(temp_string, 32, "Interval:%f\n", float_time);
                            memcpy(cache + (lines), temp_string, written_length);
                            lines += written_length;
                            written_length = snprintf(temp_string, 32, "Acceleration, Pitch, Roll\n");
                            memcpy(cache + (lines), temp_string, written_length);
                            lines += written_length;
                        }
                        else{ //append to log, mark as such
                            written_length = snprintf(temp_string, 32, "Log Paused, now resuming:\n");
                            memcpy(cache + (lines), temp_string, written_length);
                            lines += written_length;
                        }//end if append log
                        xQueueReset(log_data_queue); //clear out data that is out of date
                    }//end if log paused
                    //open the file again
                    f = fopen(file_name, "a");
                }
            }//end if check for new sample
            else
            {
                vTaskDelay(1); //allow idle state to feed watchdog
            }
        }
    }//logging enabled if statement
    // else{//logging disabled, do nothing on this core
    //     while(1){
    //             vTaskDelay(1000); //just continue yeilding, as this task can't be ended without causing upset
    //     } 
    // }
    return;
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
