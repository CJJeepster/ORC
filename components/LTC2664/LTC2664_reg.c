/**
  ******************************************************************************
  * @file    LTC2664_reg.c
  * @author  Corbin Roennebeck, WSU
  * @brief   LTC2664 driver file
  ******************************************************************************
  */


#include "LTC2664_reg.h"

/**
 * @brief Generic write to register
 * 
 * @param ctx       read / write interface definitions(ptr)
 * @param dac       DAC address
 * @param command   8 bit
 * @param data      pointer to data to write
 */
int32_t ltc2664_write_reg(const ltcdev_ctx_t *ctx, ltc2664_DACS_t dac, uint8_t command, const uint16_t *data){
    int32_t ret;

    if (ctx == NULL){
        return -1;
    }
    if (dac > 3){
        return -1;
    }

    ret = ctx->write_reg(ctx->handle, (uint8_t)dac, command, data);

    return ret;
}


/**
 * @brief write data to 1 dac and update dac ouptu
 * 
 * @param ctx   read / write interface definitions(ptr)
 * @param dac   DAC address
 * @param data  pointer to data to write
 */
int32_t ltc2664_write_and_update_1_dac(const ltcdev_ctx_t *ctx, ltc2664_DACS_t dac, uint16_t *data){
    int32_t ret;
    //write to dac, and update, command = 3
    ret = ltc2664_write_reg(ctx, dac, 3, data);

    return ret;
}



//eof