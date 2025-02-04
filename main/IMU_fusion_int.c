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

#include "ism330dhcx_reg.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_intr_types.h"


#define EXAMPLE_MAX_CHAR_SIZE    64
#define MOUNT_POINT "/sdcard"
#define SAMPLE_PERIOD 0.0048076923076923f

#include "Fusion.h"

#define IMU_HOST            VSPI_HOST
#define DAC_HOST            HSPI_HOST
#define IMU_PIN_NUM_MISO    19
#define IMU_PIN_NUM_MOSI    23
#define IMU_PIN_NUM_CLK     18
#define IMU_PIN_NUM_CS      5
#define DAC_PIN_NUM_MISO    -1
#define DAC_PIN_NUM_MOSI    27
#define DAC_PIN_NUM_CLK     26
#define DAC_PIN_NUM_CS      25

#define BOOT_TIME         10 //ms


static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void platform_delay(uint32_t ms);
esp_err_t init_spi_device(spi_host_device_t SPI_CHANNEL, gpio_num_t MISO, gpio_num_t MOSI, gpio_num_t CLK, gpio_num_t CS);
void init_ahrs_fusion(void);
esp_err_t config_spi_device(void);

static const char TAG[] = "main";


stmdev_ctx_t ISM330DHCX_dev_ctx;
static float acceleration_g[3];
static float angular_rate_dps[3];
FusionAhrs ahrs;

void app_main(void){
    esp_err_t ret;
    ret = init_spi_device(IMU_HOST, IMU_PIN_NUM_MISO, IMU_PIN_NUM_MOSI, IMU_PIN_NUM_CLK, IMU_PIN_NUM_CS);
    ESP_ERROR_CHECK(ret);
    init_ahrs_fusion();
    //ret = config_spi_device();
    ESP_ERROR_CHECK(ret);
    ret = init_spi_device(DAC_HOST, DAC_PIN_NUM_MISO, DAC_PIN_NUM_MOSI, DAC_PIN_NUM_CLK, DAC_PIN_NUM_CS);
    ESP_ERROR_CHECK(ret);

// Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.

    ESP_LOGI(TAG, "Using SDMMC peripheral");
        const gpio_config_t mtck_d3 = {
        .pin_bit_mask = (1ULL << GPIO_NUM_13),
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = true,
        .pull_down_en = false};
    ESP_ERROR_CHECK(gpio_config(&mtck_d3));

    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 40MHz for SDMMC)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_4BIT;
    host.slot = SDMMC_HOST_SLOT_1;
    //host.set_bus_width(0,4);
    host.max_freq_khz = 10000;
    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    ESP_ERROR_CHECK(gpio_pulldown_dis(GPIO_NUM_2));
    ESP_ERROR_CHECK(gpio_pullup_en(GPIO_NUM_2));

    ESP_ERROR_CHECK(gpio_pulldown_dis(GPIO_NUM_12));
    ESP_ERROR_CHECK(gpio_pullup_en(GPIO_NUM_12));
    // Enable internal pullups on enabled pins. The internal pullups
    // are insufficient however, please make sure 10k external pullups are
    // connected on the bus. This is for debug / example purpose only.
    //slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    vTaskDelay(100);

    ESP_LOGI(TAG, "Mounting filesystem");
    while (true)
    {
        ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

        if (ret != ESP_OK)
        {
            if (ret == ESP_FAIL)
            {
                ESP_LOGE(TAG, "Failed to mount filesystem. "
                              "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
            }
            else
            {
                ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                              "Make sure SD card lines have pull-up resistors in place.",
                         esp_err_to_name(ret));
            }
             vTaskDelay(pdMS_TO_TICKS(1000));
        }else{
            break;
        }
    }
    ESP_LOGI(TAG, "Filesystem mounted");


}


/*
    Initialize the ISM330DHCX_device's SPI bus
*/
esp_err_t init_spi_device(spi_host_device_t SPI_CHANNEL, gpio_num_t MISO, gpio_num_t MOSI, gpio_num_t CLK, gpio_num_t CS){
    esp_err_t ret;

    static const char init_spi_TAG[] = "SPI";
    ESP_LOGI(init_spi_TAG, "Initializing bus SPI%d...", SPI_CHANNEL + 1);

    spi_bus_config_t *buscfg = malloc(sizeof(spi_bus_config_t)); //save this config in the heap
    memset(buscfg,0xFF,sizeof(spi_bus_config_t)); //disable extra config pins
    buscfg->miso_io_num = MISO;
    buscfg->mosi_io_num = MOSI;
    buscfg->sclk_io_num = CLK;
    buscfg->max_transfer_sz = 7; //limit transfer sz to 7 bytes
    buscfg->flags = 0;
    buscfg->data_io_default_level = 0;
    buscfg->intr_flags = 0;
    buscfg->isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO;

    ret = spi_bus_initialize(SPI_CHANNEL, buscfg, SPI_DMA_DISABLED); //initialize the bus using the controller defined in the preamble
    
    if(ret != 0){ // if we have an error
        return (ret); //cancel any further operation and return
    }
    //else continue setup

    ESP_LOGI(init_spi_TAG, "Init Success. Adding device...");
    
    spi_device_handle_t *spi = malloc(sizeof(spi_device_handle_t));
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
    devcfg->spics_io_num=       CS;
    devcfg->flags =             SPI_DEVICE_NO_DUMMY; //disable high freq(>100MHZ) workaround
    devcfg->queue_size=         7;  //queue 7 transactions at most
    devcfg->pre_cb =            NULL; //No callbacks
    devcfg->post_cb =           NULL;

    ret = spi_bus_add_device(SPI_CHANNEL, devcfg, spi);

    if(ret != 0){ // if we have an error
        return (ret); //cancel any further operation and return
    }

    ISM330DHCX_dev_ctx.write_reg = platform_write;
    ISM330DHCX_dev_ctx.read_reg = platform_read;
    ISM330DHCX_dev_ctx.mdelay = platform_delay;
    ISM330DHCX_dev_ctx.handle = spi;

    ESP_LOGI(init_spi_TAG, "Device configured, ready for use");

    return(ret);

}

/*
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

/*
    @brief configures the ISM330DHCX, checks that device is connected and working. Implements delay at start to allow device boot time. Must be after bus initialized.
*/
esp_err_t config_spi_device(void){
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
    //ESP_LOGI(TAG, "Device found...");
    if (whoAmI != ISM330DHCX_ID){
        return ESP_ERR_NOT_FOUND;
    }
    //end device check

    ret = ism330dhcx_reset_set(&ISM330DHCX_dev_ctx, PROPERTY_ENABLE);
    if (ret != 0){
        return(ret);
    }

    do{
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
    /* Reset Timestamp */
    ism330dhcx_timestamp_rst(&ISM330DHCX_dev_ctx);

    return ret;

}


static void platform_delay(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
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

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
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