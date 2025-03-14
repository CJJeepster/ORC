/**
 *******************************************************************
 * @file    PID.h
 * @author  Corbin Roennebeck, WSU
 * @brief   This file contains all the functions prototypes for the
 *          PID.c file
 *******************************************************************
 */

/* Define to prevent recursive inclusion -------------------------*/
#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <esp_err.h>
#include "esp_log.h"
#include <math.h>

typedef struct PIDController_t {
    /* Controller gains */
    float_t Kp;
    float_t Ki;
    float_t Kd;

    /* Derivative low-pass filter time constant */
    float_t tau;

    /* Output limits */
    float_t limMin;
    float_t limMax;

    /* Sample time (in seconds) */
    float_t T;

    /* Controller "memory" */
    float_t integrator;
    float_t prevError;
    float_t differentiator;
    float_t prevMeasurement;

    /* Controller output */
    float_t out;

}PIDController_t;

void PIDController_Init(PIDController_t *pid);
float_t PIDController_Update(PIDController_t *pid, float_t *setpoint, float_t *measurement);
float_t PIDController_Update_zero_setpoint(PIDController_t *pid, float_t *measurement);

#ifdef __cplusplus
}
#endif

#endif