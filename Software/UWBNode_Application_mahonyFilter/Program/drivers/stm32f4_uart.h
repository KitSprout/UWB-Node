/**
  *      __            ____
  *     / /__ _  __   / __/                      __  
  *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
  *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
  *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
  *                    /_/   github.com/KitSprout    
  * 
  * @file    stm32f4_uart.h
  * @author  KitSprout
  * @date    20-Nov-2016
  * @brief   
  * 
  */

/* Define to prevent recursive inclusion ---------------------------------------------------*/
#ifndef __STM32F4_UART_H
#define __STM32F4_UART_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes --------------------------------------------------------------------------------*/
#include "stm32f4_system.h"

/* Exported types --------------------------------------------------------------------------*/
/* Exported constants ----------------------------------------------------------------------*/
/* Exported functions ----------------------------------------------------------------------*/  

#if defined(KS_HW_UART_HAL_LIBRARY)

#define UART_SendData( _H, _DAT, _LEN, _TIME )    HAL_UART_Transmit(_H, _DAT, _LEN, _TIME)
#define UART_RecvData( _H, _DAT, _LEN, _TIME )    HAL_UART_Receive(_H, _DAT, _LEN, _TIME)
#define UART_SendDataIT( _H, _DAT, _LEN )         HAL_UART_Transmit_IT(_H, _DAT, _LEN)
#define UART_RecvDataIT( _H, _DAT, _LEN )         HAL_UART_Receive_IT(_H, _DAT, _LEN)
#define UART_SendDataDMA( _H, _DAT, _LEN )        HAL_UART_Transmit_DMA(_H, _DAT, _LEN)
#define UART_RecvDataDMA( _H, _DAT, _LEN )        HAL_UART_Receive_DMA(_H, _DAT, _LEN)

#else
int8_t  UART_SendData( UART_HandleTypeDef *huart, uint8_t *sendData, uint16_t lens, uint32_t timeout );
int8_t  UART_RecvData( UART_HandleTypeDef *huart, uint8_t *recvData, uint16_t lens, uint32_t timeout );
//int8_t  UART_SendDataIT( UART_HandleTypeDef *huart, uint8_t *sendData, uint16_t lens );
//int8_t  UART_RecvDataIT( UART_HandleTypeDef *huart, uint8_t *recvData, uint16_t lens );
//int8_t  UART_SendDataDMA( UART_HandleTypeDef *huart, uint8_t *sendData, uint16_t lens );
//int8_t  UART_RecvDataDMA( UART_HandleTypeDef *huart, uint8_t *recvData, uint16_t lens );

#endif

#ifdef __cplusplus
}
#endif

#endif

/*************************************** END OF FILE ****************************************/
