/**
  *      __            ____
  *     / /__ _  __   / __/                      __  
  *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
  *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
  *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
  *                    /_/   github.com/KitSprout    
  * 
  * @file    serial.c
  * @author  KitSprout
  * @date    20-Nov-2016
  * @brief   
  * 
  */

/* Includes --------------------------------------------------------------------------------*/
#include "drivers\stm32f4_system.h"
#include "modules\serial.h"

/** @addtogroup STM32_Module
  * @{
  */

/* Private typedef -------------------------------------------------------------------------*/
/* Private define --------------------------------------------------------------------------*/
/* Private macro ---------------------------------------------------------------------------*/
/* Private variables -----------------------------------------------------------------------*/
extern UART_HandleTypeDef HUART_SERIAL;
extern pFunc SERIAL_TxCallback;
extern pFunc SERIAL_RxCallback;

static uint8_t TX_BUFFER[SERIAL_MAX_TXBUF] = {0};
static uint8_t RX_BUFFER[SERIAL_MAX_RXBUF] = {0};

SerialHandle_st hSerial = {
  .handle     = &HUART_SERIAL,
  .txBufLens  = SERIAL_MAX_TXBUF,
  .rxBufLens  = SERIAL_MAX_RXBUF,
  .pTxBuf     = TX_BUFFER,
  .pRxBuf     = RX_BUFFER
};

/* Private function prototypes -------------------------------------------------------------*/
/* Private functions -----------------------------------------------------------------------*/

void Serial_Config( void )
{
  /* UART Clk ******************************************************************/
  SERIAL_UARTx_CLK_ENABLE();

  /* UART Init *****************************************************************/
  hSerial.handle->Instance          = SERIAL_UARTx;
  hSerial.handle->Init.BaudRate     = SERIAL_BAUDRATE;
  hSerial.handle->Init.WordLength   = SERIAL_BYTESIZE;
  hSerial.handle->Init.StopBits     = SERIAL_STOPBITS;
  hSerial.handle->Init.Parity       = SERIAL_PARITY;
  hSerial.handle->Init.HwFlowCtl    = SERIAL_HARDWARECTRL;
  hSerial.handle->Init.Mode         = SERIAL_MODE;
  hSerial.handle->Init.OverSampling = SERIAL_OVERSAMPLE;
  HAL_UART_Init(hSerial.handle);

#if defined(KS_HW_UART_HAL_LIBRARY)

#else

  /* UART IT *******************************************************************/
  if (SERIAL_TxCallback != NULL) {
    __HAL_UART_ENABLE_IT(hSerial.handle, UART_IT_TXE);
  }
  else {
    __HAL_UART_DISABLE_IT(hSerial.handle, UART_IT_TXE);
  }

  if (SERIAL_RxCallback != NULL) {
    __HAL_UART_ENABLE_IT(hSerial.handle, UART_IT_RXNE);
  }
  else {
    __HAL_UART_DISABLE_IT(hSerial.handle, UART_IT_RXNE);
  }

  /* UART Enable ***************************************************************/
  __HAL_UART_ENABLE(hSerial.handle);
  __HAL_UART_CLEAR_FLAG(hSerial.handle, UART_FLAG_TC);

#endif

}

void Serial_SetTxCallbackFunc( pFunc callback )
{
  SERIAL_TxCallback = callback;
}

void Serial_SetRxCallbackFunc( pFunc callback )
{
  SERIAL_RxCallback = callback;
}

/**
  * @brief  Serial Send Data
  */
__INLINE int8_t Serial_SendData( uint8_t *sendData, uint16_t lens, uint32_t timuout )
{
  return UART_SendData(hSerial.handle, sendData, lens, timuout);
}

/**
  * @brief  Serial Recv Data
  */
__INLINE int8_t Serial_RecvData( uint8_t *recvData, uint16_t lens, uint32_t timeout )
{
  return UART_RecvData(hSerial.handle, recvData, lens, timeout);
}

#if defined(KS_HW_UART_HAL_LIBRARY)
/**
  * @brief  Serial Send Data IT
  */
__INLINE int8_t Serial_SendDataIT( uint8_t *sendData, uint16_t lens)
{
  return HAL_UART_Transmit_IT(hSerial.handle, sendData, lens);
}

/**
  * @brief  Serial Recv Data IT
  */
__INLINE int8_t Serial_RecvDataIT( uint8_t *recvData, uint16_t lens )
{
  return HAL_UART_Receive_IT(hSerial.handle, recvData, lens);
}

/**
  * @brief  Serial Send Data DMA
  */
__INLINE int8_t Serial_SendDataDMA( uint8_t *sendData, uint16_t lens )
{
  return HAL_UART_Transmit_DMA(hSerial.handle, sendData, lens);
}

/**
  * @brief  Serial Recv Data DMA
  */
__INLINE int8_t Serial_RecvDataDMA( uint8_t *recvData, uint16_t lens )
{
  return HAL_UART_Receive_DMA(hSerial.handle, recvData, lens);
}
#endif

/**
  * @brief  fputc
  */
int fputc( int ch, FILE *f )
{
  hSerial.handle->Instance->DR = ((uint8_t)ch & (uint16_t)0x01FF);
  while (!(hSerial.handle->Instance->SR & UART_FLAG_TC));
  return (ch);
}

/**
  * @brief  fgetc
  */
int fgetc( FILE *f )
{
  while (!(hSerial.handle->Instance->SR & UART_FLAG_RXNE));
  return (hSerial.handle->Instance->DR & (uint16_t)0x01FF);
}

/*************************************** END OF FILE ****************************************/
