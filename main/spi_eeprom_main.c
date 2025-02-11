/* SPI Master Half Duplex EEPROM example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "ism330dhcx_reg.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"

#define SAMPLE_PERIOD 0.0048076923076923f

#include "Fusion.h"

#  define ACCEL_HOST       SPI2_HOST
#  define PIN_NUM_MISO      13
#  define PIN_NUM_MOSI      12
#  define PIN_NUM_CLK       14
#  define PIN_NUM_CS        15

#define    BOOT_TIME          100 //ms


static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void platform_delay(uint32_t ms);
int32_t ism330dhcx_read_data_polling(spi_device_handle_t *spi);

static const char TAG[] = "main";

void app_main(void)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "Initializing bus SPI%d...", ACCEL_HOST + 1);
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 7,
    };
    //Initialize the SPI bus
    ret = spi_bus_initialize(ACCEL_HOST, &buscfg, SPI_DMA_DISABLED);
    ESP_ERROR_CHECK(ret);

    spi_device_interface_config_t devcfg={
        .command_bits = 1,
        .address_bits = 7,
        .clock_speed_hz=9*10*1000,
        .mode=3, 
        .spics_io_num=PIN_NUM_CS,  
        .queue_size=7,
        .input_delay_ns = 0,
        .pre_cb = NULL,
        .post_cb = NULL,
        .flags = SPI_DEVICE_NO_DUMMY,
        .cs_ena_pretrans = 0,
    };
    
    
    spi_device_handle_t spi;

    ret=spi_bus_add_device(ACCEL_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    /* Initialize mems driver interface */
    // stmdev_ctx_t dev_ctx = {
    //     .write_reg = platform_write,
    //     .read_reg = platform_read,
    //     .mdelay = platform_delay,
    //     .handle = spi,
    // };
    
    // uint8_t whoamI;
    // platform_delay(BOOT_TIME);
    // ret=ism330dhcx_device_id_get(&dev_ctx, &whoamI);
    // ESP_LOGI(TAG, "%i whoamI: %i", ret, whoamI);
    // vTaskDelay(100);
    


    while (1) {
        ret = ism330dhcx_read_data_polling(&spi);
        ESP_ERROR_CHECK(ret);
    }
}

int32_t ism330dhcx_read_data_polling(spi_device_handle_t *spi){
    static int16_t data_raw_acceleration[3];
    static int16_t data_raw_angular_rate[3];
    static int16_t data_raw_temperature;
    static float acceleration_mg[3];
    static float angular_rate_mdps[3];
    static float temperature_degC;
    static uint8_t whoamI, rst; 
    static uint8_t tx_buffer[1000];  
  uint32_t ret; 
  stmdev_ctx_t dev_ctx;
  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = *spi;


  FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);
    const FusionAhrsSettings AhrsSettings = {
        .accelerationRejection = 10,
        .gyroscopeRange = 500,
        .gain = .5,
        .convention = FusionConventionEnu,
        .recoveryTriggerPeriod = 5/SAMPLE_PERIOD,
    };
    FusionAhrsSetSettings(&ahrs, &AhrsSettings);

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  uint8_t count = 0;
  do{
        ism330dhcx_device_id_get(&dev_ctx, &whoamI);
        platform_delay(10);
        count ++;
  }while(whoamI != ISM330DHCX_ID && count < 10);
    //ESP_LOGI(TAG, "Device found...");
    if (whoamI != ISM330DHCX_ID){
        return ESP_ERR_NOT_FOUND;
    }
  /* Restore default configuration */
  ism330dhcx_reset_set(&dev_ctx, PROPERTY_ENABLE);


  do {
    ism330dhcx_reset_get(&dev_ctx, &rst);
  } while (rst);
    //ESP_LOGI(TAG, "Device reset...");

  /* Start device configuration. */
  ism330dhcx_device_conf_set(&dev_ctx, PROPERTY_ENABLE);
  /* Enable Block Data Update */
  ism330dhcx_block_data_update_set(&dev_ctx, PROPERTY_DISABLE);
  /* Set Output Data Rate */
  ism330dhcx_xl_data_rate_set(&dev_ctx, ISM330DHCX_XL_ODR_208Hz);
  ism330dhcx_gy_data_rate_set(&dev_ctx, ISM330DHCX_GY_ODR_208Hz);
  /* Set full scale */
  ism330dhcx_xl_full_scale_set(&dev_ctx, ISM330DHCX_2g);
  ism330dhcx_gy_full_scale_set(&dev_ctx, ISM330DHCX_500dps);

    ism330dhcx_xl_power_mode_set(&dev_ctx,ISM330DHCX_HIGH_PERFORMANCE_MD);
    ism330dhcx_gy_power_mode_set(&dev_ctx,ISM330DHCX_GY_HIGH_PERFORMANCE);
  ism330dhcx_timestamp_rst(&dev_ctx);
  /* Configure filtering chain(No aux interface)
   *
   * Accelerometer - LPF1 + LPF2 path
   */
  //ism330dhcx_xl_hp_path_on_out_set(&dev_ctx, ISM330DHCX_LP_ODR_DIV_100);
  //ism330dhcx_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);


  int64_t current_time = esp_timer_get_time();
  float_t passed_time = 0;

    //ESP_LOGI(TAG, "Device configured...");
  /* Read samples in polling mode (no int) */
    while(1){    
        uint8_t reg;
    /* Read output only if new xl value is available */
    ism330dhcx_xl_flag_data_ready_get(&dev_ctx, &reg);
    //ESP_LOGI(TAG, "acceleration ready = %i...", reg);
    if (reg) {
        count++;
      /* Read acceleration field data */
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      ism330dhcx_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
      acceleration_mg[0] = (float_t)data_raw_acceleration[0] * 0.000061f;
        //ism330dhcx_from_fs2g_to_mg(data_raw_acceleration[0]);
      acceleration_mg[1] = (float_t)data_raw_acceleration[1] * 0.000061f;
        //ism330dhcx_from_fs2g_to_mg(data_raw_acceleration[1]);
      acceleration_mg[2] = (float_t)data_raw_acceleration[2] * 0.000061f;
        //ism330dhcx_from_fs2g_to_mg(data_raw_acceleration[2]);
      //printf(
      //        "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
      //        acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      //tx_com(tx_buffer, strlen((char const *)tx_buffer));
      }
    //  else{
    //     printf("\n");
    // }

    ism330dhcx_gy_flag_data_ready_get(&dev_ctx, &reg);

    //ESP_LOGI(TAG, "gyroscope ready    = %i...", reg);
    if (reg) {
      /* Read angular rate field data */
      memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
      ism330dhcx_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
      angular_rate_mdps[0] = (float_t)data_raw_angular_rate[0] * 0.0175f;
        ism330dhcx_from_fs250dps_to_mdps(data_raw_angular_rate[0]);
      angular_rate_mdps[1] = (float_t)data_raw_angular_rate[1] * 0.0175f;
        //ism330dhcx_from_fs250dps_to_mdps(data_raw_angular_rate[1]);
      angular_rate_mdps[2] = (float_t)data_raw_angular_rate[2] * 0.0175f;
        //ism330dhcx_from_fs250dps_to_mdps(data_raw_angular_rate[2]);
    //   printf(
    //           "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
    //           angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
      //tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }
    // else{
    //     printf("\n");
    // }

    // ism330dhcx_temp_flag_data_ready_get(&dev_ctx, &reg);

    // //ESP_LOGI(TAG, "temperature ready  = %i...", reg);
    // if (reg) {
    //   /* Read temperature data */
    //   memset(&data_raw_temperature, 0x00, sizeof(int16_t));
    //   ism330dhcx_temperature_raw_get(&dev_ctx, &data_raw_temperature);
    //   temperature_degC = ism330dhcx_from_lsb_to_celsius(
    //                        data_raw_temperature);
    // //   printf(
    // //           "Temperature [degC]:%6.2f\r\n", temperature_degC);
    //   //tx_com(tx_buffer, strlen((char const *)tx_buffer));
    // } else{
    //     printf("\n");
    // }
    for(count = 1; count>0; count--){
        
        printf("\x1b[1F"); // Move to beginning of previous line
        printf("\33[2K\r");
    }
    const FusionVector gyroscope = {{angular_rate_mdps[0],angular_rate_mdps[1],angular_rate_mdps[2]}}; // replace this with actual gyroscope data in degrees/s
    const FusionVector accelerometer = {{acceleration_mg[0],acceleration_mg[1],acceleration_mg[2]}}; // replace this with actual accelerometer data in g
    

    passed_time = (esp_timer_get_time() - current_time)/1000000.f;
    current_time = esp_timer_get_time();

    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, passed_time);
    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

    printf("Roll %0.3f, Pitch %0.3f, Yaw %0.3f, Elapsed Time %f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw, passed_time);
    vTaskDelay(1);
    platform_delay(SAMPLE_PERIOD/2);
    
  }
  return (0);
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