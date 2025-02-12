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
#include "esp_log.h"
#include <math.h>

esp_err_t set_distances(float_t a, float_t b, float_t c, float_t d);
void transform(uint16_t *act1, uint16_t *act2, uint16_t *act3, uint16_t *act4, float_t *fZ, float_t *fTheta, float_t *fPhi);



#ifdef __cplusplus
}
#endif

#endif