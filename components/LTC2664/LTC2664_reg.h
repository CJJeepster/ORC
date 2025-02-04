/**
 *******************************************************************
 * @file    LTC2664_reg.h
 * @author  Corbin Roennebeck, WSU
 * @brief   This file contains all the functions prototypes for the
 *          LTC2664_reg.c driver.
 *******************************************************************
 */

/* Define to prevent recursive inclusion -------------------------*/
#ifndef LTC2664_REGS_H
#define LTC2664_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>




/** @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */
typedef int32_t (*ltcdev_write_ptr)(void *, uint8_t, const uint8_t *, uint16_t);
typedef void (*ltcdev_mdelay_ptr)(uint32_t millisec);

typedef struct
{
  /** Component mandatory fields **/
  ltcdev_write_ptr  write_reg;
  /** Component optional fields **/
  ltcdev_mdelay_ptr   mdelay;
  /** Customizable optional pointer **/
  void *handle;
} ltcdev_ctx_t;


int32_t ltc2664_write_reg(const ltcdev_ctx_t *ctx, uint8_t dac, uint8_t command, 
                                const uint8_t *bufp, uint16_t len);
/**
  * @}
  *
  */




 typedef enum
 {
    LTC2664_DAC_0 = 0,
    LTC2664_DAC_1 = 1,
    LTC2664_DAC_2 = 2,
    LTC2664_DAC_3 = 3,
 } ltc2664_DACS_t;

/* Write to DAC register functions*/
int32_t ltc2664_write_to_1_dac(const ltcdev_ctx_t *ctx, uint8_t dac, uint16_t data);
int32_t ltc2664_write_to_all(const ltcdev_ctx_t *ctx, uint16_t data);

/* Set output span functions*/
int32_t ltc2664_set_span_1_dac(const ltcdev_ctx_t *ctx, uint8_t dac, uint16_t data);
int32_t ltc2664_set_span_all(const ltcdev_ctx_t *ctx, uint16_t data);

/* Update DAC Ouput functions*/
int32_t ltc2664_update_1_dac(const ltcdev_ctx_t *ctx, uint8_t dac);
int32_t ltc2664_update_all(const ltcdev_ctx_t *ctx);

/* Write and update functions*/
int32_t ltc2664_write_and_update_1_dac(const ltcdev_ctx_t *ctx, uint8_t dac, uint16_t data);
int32_t ltc2664_write_and_update_all(const ltcdev_ctx_t *ctx, uint16_t data);
int32_t ltc2664_write_1_update_all(const ltcdev_ctx_t *ctx, uint8_t dac, uint16_t data);

/* Power down (disable) functions*/
int32_t ltc2664_power_down_dac(const ltcdev_ctx_t *ctx, uint8_t dac);
int32_t ltc2664_power_down_chip(const ltcdev_ctx_t *ctx);

/* Register toggle funcitons*/
int32_t ltc2664_toggle_select(const ltcdev_ctx_t *ctx, uint8_t dac, uint16_t data);
int32_t ltc2664_toggle_select_all(const ltcdev_ctx_t *ctx, uint16_t data);






#ifdef _cplusplus
}
#endif

#endif /* LTC2664_REGS_H*/




