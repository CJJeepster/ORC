/**
  ******************************************************************************
  * @file    ORC_CONFIG.h
  * @author  Corbin Roennebeck, WSU
  * @brief   Constants and Configurations for ORC_Main
  ******************************************************************************
  */

#ifndef ORC_CONFIG_H
#define ORC_CONFIG_H

/* Calibration System IO / pin masks */
#define MUX_PIN_A0      33
#define MUX_PIN_A1      32
#define COMPARATOR_PIN  21
#define GPIO_MUX_PINS   ((1ULL<<MUX_PIN_A0) | (1ULL<<MUX_PIN_A1))
#define GPIO_COMP_PINS  (1ULL<<COMPARATOR_PIN)
/* SD Card IO / constants */
#define CACHE_SIZE      8192*8   //bytes
#define MOUNT_POINT     "/sdcard"
#define SD_PIN_D3       13
#define SD_PIN_D2       12
#define SD_PIN_CLK      14
#define SD_PIN_CMD      15
#define SD_PIN_D0       2
#define SD_PIN_D1       4
/* IMU IO / pin masks / spi controller */
#define IMU_HOST            VSPI_HOST
#define IMU_BOOT_TIME       10 //ms
#define IMU_PIN_NUM_MISO    23
#define IMU_PIN_NUM_MOSI    19
#define IMU_PIN_NUM_CLK     18
#define IMU_PIN_NUM_CS      5
#define IMU_PIN_INT1        22
#define GPIO_INT_PIN_SEL    (1ULL<<IMU_PIN_INT1)
#define GPIO_MISO_PIN       (1ULL<<IMU_PIN_NUM_MISO)
/* DAC IO / spi controller */
#define DAC_HOST            HSPI_HOST
#define DAC_PIN_NUM_MISO    -1
#define DAC_PIN_NUM_MOSI    27
#define DAC_PIN_NUM_CLK     26
#define DAC_PIN_NUM_CS      25
/* UI IO / pin masks */
#define STATUS_LED1         17
#define STATUS_LED2         0
#define GPIO_STATUS_PINS    ((1ULL<<STATUS_LED2) | (1ULL<<STATUS_LED1))
#define MODE_SW_EN_LOGGING  34  //enable logging
#define MODE_SW_EN_ACTUATOR 35  //enable actuators
#define MODE_SW_START_LOG   36  //start and stop log via this switch
#define MODE_SW_APPEND_LOG  39  //choose to start new log (same file) or append to previous after pause
#define GPIO_MODE_SWS_HIGH  ((1ULL<<MODE_SW_EN_ACTUATOR) | (1ULL<<MODE_SW_EN_LOGGING) | (1ULL<<MODE_SW_APPEND_LOG))
#define GPIO_MODE_SWS_LOW   (1ULL<<MODE_SW_START_LOG)
/* nonlinear filter constants */
#define IMU_XL_PEAK_REJ     30738 //eq to 3.75g in int16 output data format, reject xl values above this limit (below for negative)
#define IMU_GY_PEAK_REJ     28572 //eq to 500dps in int16 output data format, reject gy values above this limit (below for negative)
#define IMU_NOISE_FLOOR     0.003f //crush all xl data +- this value to zero
/* distances */
#define FA_COG      0.011    //distance to front axle linkages (in meters) from COG
#define RA_COG      0.018    //distance to rear axle linkages (in meters) from COG
#define DA_COG      0.008    //distance to drive side linkages (in meters) from COG
#define PA_COG      0.008    //distance to passenger side linkages (in meters) from COG
/* PID constants */
#define Z_XL_KP     -30000.0f
#define Z_XL_KI     -0.0f    //integral can be unstable, without providing much to response, possible value: -8000
#define Z_XL_KD     -1800.0f
#define PITCH_KP    -70.0f
#define PITCH_KI    -1.50f
#define PITCH_KD    -1.0f
#define ROLL_KP     -50.0f
#define ROLL_KI     -1.50f
#define ROLL_KD     -1.0f


#endif //ORC_CONFIG_H