/**
  *      __            ____
  *     / /__ _  __   / __/                      __  
  *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
  *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
  *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
  *                    /_/   github.com/KitSprout    
  * 
  * @file    serial.h
  * @author  KitSprout
  * @date    16-Nov-2016
  * @brief   
  * 
  */

/* Define to prevent recursive inclusion ---------------------------------------------------*/
#ifndef __SERIAL_H
#define __SERIAL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes --------------------------------------------------------------------------------*/
#include "drivers\stm32f4_system.h"
#include "drivers\stm32f4_uart.h"
#include "algorithms\string.h"

/* Exported types --------------------------------------------------------------------------*/
/* Exported constants ----------------------------------------------------------------------*/
/* Exported functions ----------------------------------------------------------------------*/
void    Serial_Config( void );

int8_t  Serial_SendData( uint8_t *sendData, uint16_t lens, uint32_t timuout );
int8_t  Serial_RecvData( uint8_t *recvData, uint16_t lens, uint32_t timeout );
int8_t  Serial_SendDataIT( uint8_t *sendData, uint16_t lens);
int8_t  Serial_RecvDataIT( uint8_t *recvData, uint16_t lens );
int8_t  Serial_SendDataDMA( uint8_t *sendData, uint16_t lens );
int8_t  Serial_RecvDataDMA( uint8_t *recvData, uint16_t lens );

extern UartHandle_st hSerial;

#ifdef __cplusplus
}
#endif

#endif

/*************************************** END OF FILE ****************************************/

