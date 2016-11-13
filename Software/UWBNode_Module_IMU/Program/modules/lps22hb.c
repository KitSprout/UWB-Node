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
  * @date    13-Nov-2016
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
#define LPS22_SPIx                SPI2
#define LPS22_SPIx_CLK_ENABLE()   __HAL_RCC_SPI2_CLK_ENABLE()

#define LPS22_CSB_PIN             GPIO_PIN_8
#define LPS22_CSB_GPIO_PORT       GPIOA
#define LPS22_CSB_H()             __GPIO_SET(LPS22_CSB_GPIO_PORT, LPS22_CSB_PIN)
#define LPS22_CSB_L()             __GPIO_RST(LPS22_CSB_GPIO_PORT, LPS22_CSB_PIN)

#define LPS22_INTB_PIN            GPIO_Pin_10
#define LPS22_INTB_GPIO_PORT      GPIOB

#define LPS22_Delay(__TIME)       delay_ms(__TIME);

/* Private macro ---------------------------------------------------------------------------*/
/* Private variables -----------------------------------------------------------------------*/
/* Private function prototypes -------------------------------------------------------------*/
/* Private functions -----------------------------------------------------------------------*/

/**
  * @brief  LPS22_WriteReg
  * @param  writeAddr: 
  * @param  writeData: 
  * @retval None
  */
void LPS22_WriteReg( uint8_t writeAddr, uint8_t writeData )
{
  LPS22_CSB_L();
  SPI_RW(LPS22_SPIx, writeAddr);
  SPI_RW(LPS22_SPIx, writeData);
  LPS22_CSB_H();
}

/**
  * @brief  LPS22_WriteRegs
  * @param  writeAddr: 
  * @param  writeData: 
  * @param  lens: 
  * @retval None
  */
void LPS22_WriteRegs( uint8_t writeAddr, uint8_t *writeData, uint8_t lens )
{
  LPS22_CSB_L();
  SPI_RW(LPS22_SPIx, writeAddr);
  for (uint8_t i = 0; i < lens; i++) {
    SPI_RW(LPS22_SPIx, writeData[i]);
  }
  LPS22_CSB_H();
}

/**
  * @brief  LPS22_ReadReg
  * @param  readAddr: 
  * @retval read data
  */
uint8_t LPS22_ReadReg( uint8_t readAddr )
{
  uint8_t readData = 0;

  LPS22_CSB_L();
  SPI_RW(LPS22_SPIx, 0x80 | readAddr);
  readData = SPI_RW(LPS22_SPIx, 0x00);
  LPS22_CSB_H();

  return readData;
}

/**
  * @brief  LPS22_ReadRegs
  * @param  readAddr: 
  * @param  readData: 
  * @param  lens: 
  * @retval None
  */
void LPS22_ReadRegs( uint8_t readAddr, uint8_t *readData, uint8_t lens )
{
  LPS22_CSB_L();
  SPI_RW(LPS22_SPIx, 0x80 | readAddr);
  for (uint8_t i = 0; i < lens; i++) {
    readData[i] = SPI_RW(LPS22_SPIx, 0x00);
  }
  LPS22_CSB_H();
}

/**
  * @brief  LPS22_Config
  * @param  IMUx: 
  * @retval SPIx_BASE
  */
uint32_t LPS22_Config( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* SPI Clk ******************************************************************/
  LPS22_SPIx_CLK_ENABLE();

  /* SPI Pin ******************************************************************/
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

  GPIO_InitStruct.Pin   = LPS22_CSB_PIN;
  HAL_GPIO_Init(LPS22_CSB_GPIO_PORT, &GPIO_InitStruct);

  LPS22_CSB_H();    // LOW ENABLE

  return ((uint32_t)LPS22_SPIx);
}

/**
  * @brief  LPS22_Init
  * @param  IMUx: 
  * @retval state
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
  * @param  None
  * @retval state
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
