/**
 *******************************************************************
 * @file    Transform.h
 * @author  Corbin Roennebeck, WSU
 * @brief   This file contains all the functions prototypes for the
 *          Transform.c file
 *******************************************************************
 */

/* Define to prevent recursive inclusion -------------------------*/
#ifndef Transform_H
#define Transform_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <esp_err.h>

esp_err_t set_distances(float a, float b, float c, float d);
void transform(float *act1, float *act2, float *act3, float *act4, float *fZ, float *fTheta, float *fPhi);



#ifdef __cplusplus
}
#endif

#endif