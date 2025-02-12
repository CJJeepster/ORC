/**
  ******************************************************************************
  * @file    PID.c
  * @author  Corbin Roennebeck, WSU
  * @brief   PID controller implementation file
  * @note    Largely Based on a video from Phil's Lab: 
  *          https://www.youtube.com/watch?v=zOByx3Izf5U&
  ******************************************************************************
  */

#include "PID.h"

void PIDController_Init(PIDController_t *pid){
    /* Clear controller variables */
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;

    pid->differentiator = 0.0f;
    pid->prevMeasurement = 0.0f;

    pid->out = 0.0f;
}

float_t PIDController_Update(PIDController_t *pid, float_t *setpoint, float_t *measurement){

    /*
    * Error Signal
    */
    float_t error = *setpoint - *measurement;

    /*
    * Proportional
    */
    float_t proportional = pid->Kp * error;

    /*
    * Integral
    */
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

    /*
    * Anti-wind-up via dynamic integrator clamping
    */
    float_t limMinInt = 0.0f, limMaxInt = 0.0f;

    /* Compute integrator limits */
    if(pid->limMax > proportional){
        limMaxInt = pid->limMax - proportional;
    }
    if(pid->limMin < proportional){
        limMinInt = pid->limMin - proportional;
    }

    /* Clamp integrator */
    if(pid->integrator > limMaxInt){
        pid->integrator = limMaxInt;
    }
    else if (pid->integrator < limMinInt){
        pid->integrator = limMinInt;
    }

    /*
    * Derivative (band-limited differentiator)
    */

   pid->differentiator = (-2.0f * pid->Kd * (*measurement - pid->prevMeasurement)
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);

    /*
    * Comput output and apply limits
    */

    pid->out = proportional + pid->integrator + pid->differentiator;

    if(pid->out > pid->limMax){
        pid->out = pid->limMax;
    } else if (pid->out < pid->limMin){
        pid->out = pid->limMin;
    }

    /*
    * Store error and measurement for later use
    */

    pid->prevError = error;
    pid->prevMeasurement = *measurement;

    return (int16_t)pid->out;

}

float_t PIDController_Update_zero_setpoint(PIDController_t *pid, float_t *measurement){
    float_t zero = 0.0f;
    return PIDController_Update(pid,&zero,measurement);
}











