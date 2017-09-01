/**
 *      __            ____
 *     / /__ _  __   / __/                      __  
 *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
 *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
 *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
 *                    /_/   github.com/KitSprout    
 * 
 *  @file    stm32f4_system.h
 *  @author  KitSprout
 *  @date    01-Aug-2017
 *  @brief   
 * 
 */

/* Define to prevent recursive inclusion ---------------------------------------------------*/
#ifndef __STM32F4_SYSTEM_H
#define __STM32F4_SYSTEM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes --------------------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4_delay.h"
#include "boardConfig.h"
#include "algorithms\mathUnit.h"

/* Exported types --------------------------------------------------------------------------*/
typedef void (*pFunc)( void );

typedef enum {
  KS_OK       = 0x00U,
  KS_ERROR    = 0x01U,
  KS_BUSY     = 0x02U,
  KS_TIMEOUT  = 0x03U
} KS_StatusTypeDef;

typedef enum {
  KS_RESET = 0U, 
  KS_SET = !KS_RESET
} KS_FlagStatus;

typedef enum {
  KS_DISABLE = 0U, 
  KS_ENABLE = !KS_DISABLE
} KS_FunctionalState;

/* Exported constants ----------------------------------------------------------------------*/
/* Exported macro --------------------------------------------------------------------------*/
#define __GPIO_SET(__PORT, __PIN)           ((__PORT)->BSRR = (__PIN))
#define __GPIO_RST(__PORT, __PIN)           ((__PORT)->BSRR = (((uint32_t)(__PIN)) << 16U))
#define __GPIO_TOG(__PORT, __PIN)           ((__PORT)->ODR ^= (__PIN))
#define __GPIO_READ(__PORT, __PIN)          ((__PORT)->IDR & (__PIN))

#define BYTE32(__BH2, __BH1, __BL2, __BL1)  (((uint8_t)(__BH2)<<24U) | ((uint8_t)(__BH1)<<16U) | \\
                                            ( (uint8_t)(__BL2)<<8U)  | ((uint8_t)(__BL1)))
#define BYTE32U16H(__B32U16)                ((uint16_t)(((uint32_t)(__B32U16)&0xFFFF0000)>>16U))
#define BYTE32U16L(__B32U16)                ((uint16_t)( (uint32_t)(__B32U16)&0x0000FFFF))
#define BYTE32U8_4(__B32U8)                 ((uint8_t)(((uint32_t)(__B32U8)&0xFF000000)>>24U))
#define BYTE32U8_3(__B32U8)                 ((uint8_t)(((uint32_t)(__B32U8)&0x00FF0000)>>16U))
#define BYTE32U8_2(__B32U8)                 ((uint8_t)(((uint32_t)(__B32U8)&0x0000FF00)>>8U))
#define BYTE32U8_1(__B32U8)                 ((uint8_t)( (uint32_t)(__B32U8)&0x000000FF))

#define BYTE16(_BH, __BL)                   ((((uint16_t)(__BH))<<8U) | ((uint16_t)(__BL)))
#define BYTE16U8H(__B16U8)                  ((uint8_t)(((uint16_t)(__B16U8)&0xFF00)>>8U))
#define BYTE16U8L(__B16U8)                  ((uint8_t)( (uint16_t)(__B16U8)&0x00FF))

/* Exported functions ----------------------------------------------------------------------*/  

#ifdef __cplusplus
}
#endif

#endif

/*************************************** END OF FILE ****************************************/
