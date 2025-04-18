/**
  ******************************************************************************
  * @file    Transform.c
  * @author  Corbin Roennebeck, WSU
  * @brief   Transform Algorithm file
  ******************************************************************************
  */

#include "Transform.h"

//distances between actuator linkage and IMU small angle aprox.
static float_t _a_2ab = 0;// a/(2(a+b))
static float_t _b_2ab = 0;// b/(2(a+b))
static float_t _1_2ab = 0;// 1/(2(a+b))
static float_t _1_2cd = 0;// 1/(2(c+d))

//member access
float_t get_small_angle_a_2ab(void){ return _a_2ab; }
float_t get_small_angle_b_2ab(void){ return _b_2ab; }
float_t get_small_angle_1_2ab(void){ return _1_2ab; }
float_t get_small_angle_1_2cd(void){ return _1_2cd; }

static const char TAG[] = "Transform";

/**
 * @brief set vehicle actuator to IMU dimensions and associated transform multiplier
 * @bug NEED TO DEFINE UNITS FOR DISTANCE
 * 
 * @param   a   distance to front axle linkages (in meters)
 * @param   b   distance to rear axle linkages (in meters)
 * @param   c   distance to drive side linkages (in meters)
 * @param   d   distance to passenger side linkages (in meters)
 */
esp_err_t set_distances(float_t a, float_t b, float_t c, float_t d){
    if(a < 0 || b < 0 || c < 0 || d <0 ){
        ESP_LOGE(TAG, "All distances must be positive!");
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    _a_2ab = a/(2.0*(a+b));
    _b_2ab = b/(2.0*(a+b));
    _1_2ab = 1.0/(2.0*(a+b));
    _1_2cd = 1.0/(2.0*(c+d));
    return ESP_OK;
}

/**
 * @brief Transform vertical acceleration, pitch, and roll forces into 4 corner forces using small angle approximations
 * @attention float data must be constrained to int16 min/max: -32768 to 32767
 *              this allows easy conversion to uint16 data: 0->65535, which the DAC will enjoy
 * 
 * @param   act1    pointer to actuator 1 (front driver) force output
 * @param   act2    pointer to actuator 2 (front passenger) force output
 * @param   act3    pointer to actuator 3 (rear driver) force output
 * @param   act4    pointer to actuator 4 (rear passenger) force output
 * @param   fZ      pointer to overall Z force needed
 * @param   fTheta  pointer to overall pitch force needed
 * @param   fPhi    pointer to overall roll force needed
 *  
 */
void transform(uint16_t *act1, uint16_t *act2, uint16_t *act3, uint16_t *act4, float_t *fZ, float_t *fTheta, float_t *fPhi){
    //generate all multiplicands (all library constants are garunteed between -1 and 1, typically -1/2 and 1/2)
    float_t _b_fZ = _b_2ab * (*fZ);
    float_t _a_fZ = _a_2ab * (*fZ);
    float_t _p_fT = _1_2ab * (*fTheta);
    float_t _p_fP = _1_2cd * (*fPhi);
    //perform addition to find each actuator force and zero reference the result.
    float_t inter_act[4];
    inter_act[0] = _b_fZ - _p_fT + _p_fP + 32768.0f;
    inter_act[1] = _b_fZ - _p_fT - _p_fP + 32768.0f;
    inter_act[2] = _a_fZ + _p_fT + _p_fP + 32768.0f;
    inter_act[3] = _a_fZ + _p_fT - _p_fP + 32768.0f;
    for (int i = 0; i < 4; i++){
        if(inter_act[i]>(float_t)UINT16_MAX) inter_act[i] = UINT16_MAX;
        else if (inter_act[i]<0.0f) inter_act[i] = 0;
    }

    *act1 = (uint16_t)inter_act[0];
    *act2 = (uint16_t)inter_act[1];
    *act3 = (uint16_t)inter_act[2];
    *act4 = (uint16_t)inter_act[3];
}











