/**
 *      __            ____
 *     / /__ _  __   / __/                      __  
 *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
 *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
 *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
 *                    /_/   github.com/KitSprout    
 * 
 *  @file    stm32f4_adc.h
 *  @author  KitSprout
 *  @date    19-Aug-2017
 *  @brief   
 * 
 */

/* Define to prevent recursive inclusion ---------------------------------------------------*/
#ifndef __STM32F4_ADC_H
#define __STM32F4_ADC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes --------------------------------------------------------------------------------*/
#include "stm32f4_system.h"

/* Exported types --------------------------------------------------------------------------*/
typedef struct {
  ADC_HandleTypeDef *handle;
  uint16_t *convBuff;
} AdcHandle_st;

/* Exported constants ----------------------------------------------------------------------*/
#define ADC_VREF_VOL    3.3f
#define ADC_VBAT_CONV   0.5f  // R = R2/(R1+R2) = 0.5, 3300mV / (2^12 * R)

/* Exported functions ----------------------------------------------------------------------*/  
void      ADC_Config( void );
void      ADC_GetAdc1AveValue( uint16_t *ave );
int16_t   ADC_ConvInternalTemperature( uint16_t adcdata );
int16_t   ADC_ConvVoltageInput( uint16_t adcdata );

extern AdcHandle_st hAdc1;

#ifdef __cplusplus
}
#endif

#endif

/*************************************** END OF FILE ****************************************/
