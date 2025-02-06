/**
  ******************************************************************************
  * @file    LTC2664_reg.c
  * @author  Corbin Roennebeck, WSU
  * @brief   LTC2664 driver file
  ******************************************************************************
  */

#include "Transform.h"

//distances between actuator linkage and IMU
static float _a_2ab = 0;// a/(2(a+b))
static float _b_2ab = 0;// b/(2(a+b))
static float _1_2ab = 0;// 1/(2(a+b))
static float _1_2cd = 0;// 1/(2(c+d))

static const char TAG[] = "Transform";

/**
 * @brief set vehicle actuator to IMU dimensions and associated transform multiplier
 * @bug NEED TO DEFINE UNITS FOR DISTANCE
 * 
 * @param   a   distance to front axle linkages
 * @param   b   distance to rear axle linkages
 * @param   c   distance to drive side linkages
 * @param   d   distance to passenger side linkages
 */
esp_err_t set_distances(float a, float b, float c, float d){
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
void transform(float *act1, float *act2, float *act3, float *act4, float *fZ, float *fTheta, float *fPhi){
    //generate all multiplicands
    float _b_fZ = _b_2ab * (*fZ);
    float _a_fZ = _a_2ab * (*fZ);
    float _p_fT = _1_2ab * (*fTheta);
    float _n_fT = -1.0 * _p_fT;
    float _p_fP = _1_2cd * (*fPhi);
    float _n_fP = -1.0 * _p_fP;
    //perform addition to find each actuator force
    *act1 = _b_fZ + _n_fT + _p_fP;
    *act2 = _b_fZ + _n_fT + _n_fP;
    *act3 = _a_fZ + _p_fT + _p_fP;
    *act4 = _a_fZ + _p_fT + _n_fP;
}











