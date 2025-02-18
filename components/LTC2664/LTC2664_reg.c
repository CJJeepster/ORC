/**
  ******************************************************************************
  * @file    LTC2664_reg.c
  * @author  Corbin Roennebeck, WSU
  * @brief   LTC2664 driver file
  ******************************************************************************
  */


#include "LTC2664_reg.h"

//positive and negative offsets are saved to allow for easy addition and less math in each write call.
static uint16_t dac_offset_positive[4] = {0,0,0,0};
static uint16_t dac_offset_negative[4] = {0,0,0,0};
static uint16_t dac_upper_limit[4] = {UINT16_MAX,UINT16_MAX,UINT16_MAX,UINT16_MAX}; //any input larger than this will cause problems (wraparound)
static uint16_t dac_lower_limit[4] = {0,0,0,0}; //any input less than this will cause problems (wraparound)

/**
 * @brief Generic write to register
 * 
 * @param ctx       read / write interface definitions(ptr)
 * @param dac       DAC address
 * @param command   8 bit
 * @param data      pointer to data to write
 */
int32_t ltc2664_write_reg(const ltcdev_ctx_t *ctx, ltc2664_DACS_t dac, uint8_t command, const uint16_t *data){
    int32_t ret =0;

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
 * @brief save library offset values as appropriate, takes int_32 offset and corrects to uint_32
 * 
 * @param dac     DAC to edit offset values for
 * @param offset  Pointer to offset data
 */
void ltc2664_save_offset(ltc2664_DACS_t dac, int32_t *offset){
  if(*offset < 0){
    dac_offset_positive[dac] = 0;
    dac_offset_negative[dac] = (uint16_t)(*offset * -1);
    dac_lower_limit[dac] = dac_offset_negative[dac];
    dac_upper_limit[dac] = UINT16_MAX;
  }
  else{
    dac_offset_positive[dac] = (uint16_t)(*offset);
    dac_offset_negative[dac] = 0;
    dac_lower_limit[dac] = 0;
    dac_upper_limit[dac] = UINT16_MAX - *offset;
  }
}


/**
 * @brief write data to 1 dac and update dac output. prevents wraparound when applying offset values.
 * 
 * @param ctx   read / write interface definitions(ptr)
 * @param dac   DAC address
 * @param data  pointer to data to write
 */

   static const char DACTAG[] = "1Dac";
int32_t ltc2664_write_and_update_1_dac(const ltcdev_ctx_t *ctx, ltc2664_DACS_t dac, uint16_t *data){
    int32_t ret  =0;
    uint16_t temp_data;
    if(*data > dac_upper_limit[dac]){ //properly clamp high inputs
      temp_data = UINT16_MAX;
    }
    else if(*data < dac_lower_limit[dac]){ //properly clamp low inputs
      temp_data = 0;
    }
    else{ //apply offsets only if they won't cause wraparound
      temp_data = *data + dac_offset_positive[dac] - dac_offset_negative[dac];
    }
    //write to dac, and update, command = 3
    ESP_LOGI(DACTAG, "ctx %u, dac %i, data %i", ctx,dac,&temp_data);
    ret = ltc2664_write_reg(ctx, dac, 3, data);

    return ret;
}



//eof