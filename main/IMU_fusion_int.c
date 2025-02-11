//#define DAC_TEST
//#define SD_TEST

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

#define EXAMPLE_MAX_CHAR_SIZE    64
#define MOUNT_POINT "/sdcard"
#define SAMPLE_PERIOD 0.0048076923076923f

#define IMU_HOST            VSPI_HOST
#define DAC_HOST            HSPI_HOST
#define IMU_PIN_NUM_MISO    13//19
#define IMU_PIN_NUM_MOSI    12//23
#define IMU_PIN_NUM_CLK     14//18
#define IMU_PIN_NUM_CS      15//5
#define IMU_PIN_INT1        17//32
#define DAC_PIN_NUM_MISO    -1
#define DAC_PIN_NUM_MOSI    27
#define DAC_PIN_NUM_CLK     26
#define DAC_PIN_NUM_CS      25

#define GPIO_INT_PIN_SEL ((1ULL<<IMU_PIN_INT1))

#define BOOT_TIME         10 //ms

static int32_t DAC_write(void *handle, ltc2664_DACS_t dac, uint8_t command, const uint16_t *data);
static int32_t IMU_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t IMU_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void platform_delay(uint32_t ms);
esp_err_t init_spi_IMU();
esp_err_t init_spi_DAC();
void init_ahrs_fusion(void);
esp_err_t config_spi_IMU(void);

stmdev_ctx_t ISM330DHCX_dev_ctx;
spi_device_handle_t IMU_spi;
ltcdev_ctx_t LTC2664_dev_ctx;
FusionAhrs ahrs;
FusionEuler euler;
float_t float_time;
QueueHandle_t clicker;


//IMU interupt routine (does all the shiz but writing to SD)
static void IRAM_ATTR imu_isr_handler(void* arg){
    int8_t test = 123;
    xQueueSendFromISR(clicker, &test,0);
    /*//when IMU_PIN_INT1 rising edge
    static int16_t data_raw_acceleration[3];
    static int16_t data_raw_angular_rate[3];
    static float_t acceleration_g[3];
    static float_t angular_rate_dps[3];
    //read time stamp
    uint32_t time;
    ism330dhcx_timestamp_raw_get(&ISM330DHCX_dev_ctx,&time);
    ism330dhcx_timestamp_rst(&ISM330DHCX_dev_ctx);
    float_time = time * 0.000000025f;
    //reset time stamp
    //read accel
    memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
    ism330dhcx_acceleration_raw_get(&ISM330DHCX_dev_ctx, data_raw_acceleration);
    //convert to g for use in Fusion 
    //uses full scale of 2 g for int to float conversion
    acceleration_g[0] = (float_t)data_raw_acceleration[0] * 0.000061f;
    acceleration_g[1] = (float_t)data_raw_acceleration[1] * 0.000061f;
    acceleration_g[2] = (float_t)data_raw_acceleration[2] * 0.000061f;
    //read gyro
    memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
    ism330dhcx_angular_rate_raw_get(&ISM330DHCX_dev_ctx, data_raw_angular_rate);
    //convert to dps for us in Fusion
    //uses full scale of 500dps for int to float conversion
    angular_rate_dps[0] = (float_t)data_raw_angular_rate[0] * 0.0175f;
    angular_rate_dps[1] = (float_t)data_raw_angular_rate[1] * 0.0175f;
    angular_rate_dps[2] = (float_t)data_raw_angular_rate[2] * 0.0175f;
    //fusion
    const FusionVector gyroscope = {{angular_rate_dps[0],angular_rate_dps[1],angular_rate_dps[2]}};
    const FusionVector accelerometer = {{acceleration_g[0],acceleration_g[1],acceleration_g[2]}};
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, float_time);
    euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
    //printf("Roll %0.3f, Pitch %0.3f, Yaw %0.3f, Elapsed Time %f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw, float_time);
    //pid
    //transform
    //write to dac and internal buffer
    */
}


//need to add timestamp enable

void app_main(void){
    static const char TAG[] = "Main";
    esp_err_t err;
    #if defined DAC_TEST
    //test dac
    ESP_LOGI(TAG, "Testing DAC: ");
    err = init_spi_DAC();
    ESP_ERROR_CHECK(err);
    uint16_t n_two_halfV = 0;
    uint16_t zeroV = 32767;
    uint16_t p_two_halfV = 65535;
    while(1){
        //one second loop, multi step response from DACs go from -2.5V to 0V to 2.5V to 0V, repeat
        err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_0, &n_two_halfV);
        ESP_ERROR_CHECK(err);
        platform_delay(50);
        err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_1, &n_two_halfV);
        ESP_ERROR_CHECK(err);
        platform_delay(50);
        err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_2, &n_two_halfV);
        ESP_ERROR_CHECK(err);
        platform_delay(50);
        err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_3, &n_two_halfV);
        ESP_ERROR_CHECK(err);
        platform_delay(100);
        err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_0, &zeroV);
        ESP_ERROR_CHECK(err);
        platform_delay(50);
        err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_1, &zeroV);
        ESP_ERROR_CHECK(err);
        platform_delay(50);
        err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_2, &zeroV);
        ESP_ERROR_CHECK(err);
        platform_delay(50);
        err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_3, &zeroV);
        ESP_ERROR_CHECK(err);
        platform_delay(100);
        err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_3, &p_two_halfV);
        ESP_ERROR_CHECK(err);
        platform_delay(50);
        err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_2, &p_two_halfV);
        ESP_ERROR_CHECK(err);
        platform_delay(50);
        err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_1, &p_two_halfV);
        ESP_ERROR_CHECK(err);
        platform_delay(50);
        err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_0, &p_two_halfV);
        ESP_ERROR_CHECK(err);
        platform_delay(100);
        err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_0, &zeroV);
        ESP_ERROR_CHECK(err);
        platform_delay(50);
        err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_1, &zeroV);
        ESP_ERROR_CHECK(err);
        platform_delay(50);
        err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_2, &zeroV);
        ESP_ERROR_CHECK(err);
        platform_delay(50);
        err = ltc2664_write_and_update_1_dac(&LTC2664_dev_ctx, LTC2664_DAC_3, &zeroV);
        ESP_ERROR_CHECK(err);
        platform_delay(100);
    }
    
    #elif defined SD_TEST
    //test sd card
    #else
    //normal operation


    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 1;
    gpio_config(&io_conf);

    err = init_spi_IMU();
    ESP_ERROR_CHECK(err);
    init_ahrs_fusion();

    err = config_spi_IMU();
    ESP_ERROR_CHECK(err);


    ESP_LOGI(TAG, "Installing ISR");
    err = gpio_install_isr_service(0);//ESP_INTR_FLAG_LEVEL1|ESP_INTR_FLAG_LEVEL2|ESP_INTR_FLAG_LEVEL3|ESP_INTR_FLAG_EDGE);
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "ISR installed");
    //hook isr handler for specific gpio pin
    err = gpio_isr_handler_add(IMU_PIN_INT1, imu_isr_handler, (void*) NULL);
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "ISR Handler added");
    
    static int16_t data_raw_acceleration[3];
    static int16_t data_raw_angular_rate[3];
    ism330dhcx_acceleration_raw_get(&ISM330DHCX_dev_ctx, data_raw_acceleration);
    ism330dhcx_angular_rate_raw_get(&ISM330DHCX_dev_ctx, data_raw_angular_rate);


    clicker = xQueueCreate(10, sizeof(int8_t));
    
    ESP_LOGI(TAG, "Going into loop");
    
    int8_t theNumber;

    while(1){
        vTaskDelay(100);
        if(xQueueReceive(clicker,&theNumber,0)){
            ESP_LOGI(TAG, "ISR: %i", theNumber);
        }
        else{
            ESP_LOGI(TAG, "WOMP WOMP");
        }
    }

    #endif
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
    devcfg->duty_cycle_pos =    128; //50%/50% duty cycle
    devcfg->cs_ena_pretrans =   0;//CS timed with clock
    devcfg->cs_ena_posttrans =  0;
    devcfg->clock_speed_hz=     9*1000*1000; //9Mhz
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

    ESP_LOGI(init_spi_TAG, "Device configured, ready for use");

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
    
    spi_device_handle_t *spi = malloc(sizeof(spi_device_handle_t));
    spi_device_interface_config_t *devcfg = malloc(sizeof(spi_device_interface_config_t));
    memset(devcfg,0x00,sizeof(spi_device_interface_config_t));//zero out memory space
    devcfg->command_bits =      4;//one command bit (R/W)
    devcfg->address_bits =      4;//seven address bit (register address)
    devcfg->mode=               3;//Clock high in idle, read on rising edge
    devcfg->clock_source =      SPI_CLK_SRC_DEFAULT;
    devcfg->duty_cycle_pos =    128; //50%/50% duty cycle
    devcfg->cs_ena_pretrans =   0;//CS timed with clock
    devcfg->cs_ena_posttrans =  0;
    devcfg->clock_speed_hz=     49*1000*1000; //49Mhz
    devcfg->input_delay_ns =    0;
    devcfg->spics_io_num=       DAC_PIN_NUM_CS;
    devcfg->flags =             SPI_DEVICE_NO_DUMMY; //disable high freq(>100MHZ) workaround
    devcfg->queue_size=         7;  //queue 7 transactions at most
    devcfg->pre_cb =            NULL; //No callbacks
    devcfg->post_cb =           NULL;

    ret = spi_bus_add_device(IMU_HOST, devcfg, spi);

    if(ret != 0){ // if we have an error
        return (ret); //cancel any further operation and return
    }

    LTC2664_dev_ctx.write_reg = DAC_write;
    LTC2664_dev_ctx.handle = spi;

    ESP_LOGI(init_spi_TAG, "Device configured, ready for use");

    return(ret);

}


/**
    @brief Initialise the Fusion algorithim and apply necessary settings
*/
void init_ahrs_fusion(void){
    static const char ahrs_init[] = "AHRS";

    FusionAhrsInitialise(&ahrs);

    FusionAhrsSettings * const ahrs_settings = malloc(sizeof(FusionAhrsSettings));
    ahrs_settings->accelerationRejection =  10; //degrees
    ahrs_settings->gyroscopeRange =         500;//dps
    ahrs_settings->gain =                   .5; //influence of gyroscope
    ahrs_settings->convention =             FusionConventionEnu; //earth axes convention (consistent with ISM330DHCX)
    ahrs_settings->recoveryTriggerPeriod =  5/SAMPLE_PERIOD; //approx 5 second recovery trigger period

    FusionAhrsSetSettings(&ahrs, ahrs_settings);

    ESP_LOGI(ahrs_init, "AHRS Initialized");

    //more to come, need calibration settings, etc. 
    //minimum working product rn. 
}

/**
    @brief configures the ISM330DHCX, checks that device is connected and working. Implements delay at start to allow device boot time. Must be after bus initialized.
*/
esp_err_t config_spi_IMU(void){
    static const char Config_IMU_TAG[] = "IMU Config";
    vTaskDelay(BOOT_TIME / portTICK_PERIOD_MS);
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
        ESP_LOGI(Config_IMU_TAG, "stuck in reset loop");
        ret = ism330dhcx_reset_get(&ISM330DHCX_dev_ctx,&rst);
        ESP_ERROR_CHECK(ret);
    } while (rst);

    /* Start device configuration. */
    ism330dhcx_device_conf_set(&ISM330DHCX_dev_ctx, PROPERTY_ENABLE);
    /* Enable Block Data Update */
    ism330dhcx_block_data_update_set(&ISM330DHCX_dev_ctx, PROPERTY_DISABLE);
    /* Set Output Data Rate */
    ism330dhcx_xl_data_rate_set(&ISM330DHCX_dev_ctx, ISM330DHCX_XL_ODR_208Hz);
    ism330dhcx_gy_data_rate_set(&ISM330DHCX_dev_ctx, ISM330DHCX_GY_ODR_208Hz);
    /* Set full scale */
    ism330dhcx_xl_full_scale_set(&ISM330DHCX_dev_ctx, ISM330DHCX_2g);
    ism330dhcx_gy_full_scale_set(&ISM330DHCX_dev_ctx, ISM330DHCX_500dps);
    /* Set Power Mode */
    ism330dhcx_xl_power_mode_set(&ISM330DHCX_dev_ctx,ISM330DHCX_HIGH_PERFORMANCE_MD);
    ism330dhcx_gy_power_mode_set(&ISM330DHCX_dev_ctx,ISM330DHCX_GY_HIGH_PERFORMANCE);
    /* Set Interupt 1 Output mode */
    ism330dhcx_pin_mode_set(&ISM330DHCX_dev_ctx,ISM330DHCX_PUSH_PULL);
    
    ism330dhcx_pin_int1_route_t pin_int1_route;
    ism330dhcx_pin_int1_route_get(&ISM330DHCX_dev_ctx, &pin_int1_route);
    pin_int1_route.int1_ctrl.int1_drdy_xl = PROPERTY_ENABLE;
    ism330dhcx_pin_int1_route_set(&ISM330DHCX_dev_ctx, &pin_int1_route);
    ism330dhcx_int_notification_set(&ISM330DHCX_dev_ctx,ISM330DHCX_BASE_LATCHED_EMB_PULSED);







    /* Reset Timestamp */
    ism330dhcx_timestamp_rst(&ISM330DHCX_dev_ctx);

    ESP_LOGI(Config_IMU_TAG,"IMU Configured");
    return ret;

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
static int32_t DAC_write(void *handle, ltc2664_DACS_t dac, uint8_t command, const uint16_t *data){
    esp_err_t ret;
    spi_transaction_t t;
    memset( &t, 0x00, sizeof(t) );
    t.cmd = command;
    t.addr = dac;
    t.length = 16;
    t.tx_buffer = data;
    t.flags = 0;

    ret = spi_device_transmit(handle, &t);

    return (int)ret;
}