/**
  *      __            ____
  *     / /__ _  __   / __/                      __  
  *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
  *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
  *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
  *                    /_/   github.com/KitSprout    
  * 
  * @file    lps22hb.c
  * @author  KitSprout
  * @date    12-Dec-2016
  * @brief   
  * 
  */

/* Includes --------------------------------------------------------------------------------*/
#include "drivers\stm32f4_system.h"
#include "drivers\stm32f4_spi.h"
#include "modules\lps22hb.h"

/** @addtogroup STM32_Module
  * @{
  */

/* Private typedef -------------------------------------------------------------------------*/
/* Private define --------------------------------------------------------------------------*/
/* Private macro ---------------------------------------------------------------------------*/
/* Private variables -----------------------------------------------------------------------*/
extern SPI_HandleTypeDef HSPI_IMU;
extern uint8_t IMU_TX_BUFFER[IMU_MAX_TXBUF];
extern uint8_t IMU_RX_BUFFER[IMU_MAX_RXBUF];

/* Private function prototypes -------------------------------------------------------------*/
/* Private functions -----------------------------------------------------------------------*/

/**
  * @brief  LPS22_WriteReg
  */
void LPS22_WriteReg( uint8_t writeAddr, uint8_t writeData )
{
//  IMU_TX_BUFFER[0] = writeAddr;
//  IMU_TX_BUFFER[1] = writeData;

  IMU_CSB_L();
//  SPI_SendRecv(&HSPI_IMU, IMU_TX_BUFFER, IMU_RX_BUFFER, 3, HAL_MAX_DELAY);
  SPI_RW(&HSPI_IMU, writeAddr);
  SPI_RW(&HSPI_IMU, writeData);
  IMU_CSB_H();
}

/**
  * @brief  LPS22_WriteRegs
  */
void LPS22_WriteRegs( uint8_t writeAddr, uint8_t *writeData, uint8_t lens )
{
//  uint8_t count = lens;
//  uint8_t *ptr = &IMU_TX_BUFFER[1];

//  IMU_TX_BUFFER[0] = writeAddr;
//  while(count--) {
//    *ptr++ = *writeData++;
//  }
//  IMU_CSB_L();
//  SPI_SendRecv(&HSPI_IMU, IMU_TX_BUFFER, IMU_RX_BUFFER, lens + 1, HAL_MAX_DELAY);
//  IMU_CSB_H();

  IMU_CSB_L();
  SPI_RW(&HSPI_IMU, writeAddr);
  for (uint8_t i = 0; i < lens; i++) {
    SPI_RW(&HSPI_IMU, writeData[i]);
  }
  IMU_CSB_H();
}

/**
  * @brief  LPS22_ReadReg
  */
uint8_t LPS22_ReadReg( uint8_t readAddr )
{
  IMU_TX_BUFFER[0] = 0x80 | readAddr;
  IMU_TX_BUFFER[1] = 0x00;

  IMU_CSB_L();
  SPI_SendRecv(&HSPI_IMU, IMU_TX_BUFFER, IMU_RX_BUFFER, 2, HAL_MAX_DELAY);
  IMU_CSB_H();

  return IMU_RX_BUFFER[1];
}

/**
  * @brief  LPS22_ReadRegs
  */
void LPS22_ReadRegs( uint8_t readAddr, uint8_t *readData, uint8_t lens )
{
  uint8_t count = lens;
  uint8_t *ptrBuf = &IMU_TX_BUFFER[1];

  IMU_TX_BUFFER[0] = 0x80 | readAddr;
  while (count--) {
    *ptrBuf++ = 0;
  }

  IMU_CSB_L();
  SPI_SendRecv(&HSPI_IMU, IMU_TX_BUFFER, IMU_RX_BUFFER, lens + 1, HAL_MAX_DELAY);
  IMU_CSB_H();

  ptrBuf = &IMU_RX_BUFFER[1];
  while (lens--) {
    *readData++ = *ptrBuf++;
  }
}

/**
  * @brief  LPS22_Config
  */
void LPS22_Config( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* SPI Pin ******************************************************************/
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

  GPIO_InitStruct.Pin   = IMU_CSB_PIN;
  HAL_GPIO_Init(IMU_CSB_GPIO_PORT, &GPIO_InitStruct);

  IMU_CSB_H();    // LOW ENABLE
}

/**
  * @brief  LPS22_Init
  */
int8_t LPS22_Init( LPS_ConfigTypeDef *LPSx )
{
  uint8_t status = ERROR;

  delay_ms(2);
  status = LPS22_DeviceCheck();
  if (status != SUCCESS)
    return ERROR;

  delay_ms(10);

  return SUCCESS;
}

/**
  * @brief  LPS22_DeviceCheck
  */
int8_t LPS22_DeviceCheck( void )
{
  uint8_t deviceID;

  deviceID = LPS22_ReadReg(LPS22HB_WHO_AM_I);
  if (deviceID != LPS22HB_DeviceID) {
    return ERROR;
  }

  return SUCCESS;
}

/**
  * @brief  LPS22_GetSensitivity
  * @param  sensitivity: point to float32_t
            sensitivity[0] - pressure
            sensitivity[1] - temperature
  * @retval None
  */
void LPS22_GetSensitivity( float32_t *sensitivity )
{
  /* Set pressure sensitivity (hPa/LSB) */
  sensitivity[0] = 1.0 / 4096.0;

  /* Set temperature sensitivity (degC/LSB) */
  sensitivity[1] = 1.0 / 100.0;
}

/**
  * @brief  LPS22_GetRawData
  * @param  data: point to int32_t
  * @retval state
  */
int8_t LPS22_GetRawData( int32_t *data )
{
  data[0] = 0;
  data[1] = 0;

  return SUCCESS;
}

/*************************************** END OF FILE ****************************************/
