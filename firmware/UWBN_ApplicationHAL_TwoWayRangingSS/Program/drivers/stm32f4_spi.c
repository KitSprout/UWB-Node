/**
 *      __            ____
 *     / /__ _  __   / __/                      __  
 *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
 *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
 *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
 *                    /_/   github.com/KitSprout    
 * 
 *  @file    stm32f4_spi.c
 *  @author  KitSprout
 *  @date    19-Aug-2017
 *  @brief   
 * 
 */

/* Includes --------------------------------------------------------------------------------*/
#include "stm32f4_system.h"
#include "stm32f4_spi.h"

/** @addtogroup STM32_Driver
 *  @{
 */

/* Private typedef -------------------------------------------------------------------------*/
/* Private define --------------------------------------------------------------------------*/
/* Private macro ---------------------------------------------------------------------------*/
/* Private variables -----------------------------------------------------------------------*/
static SPI_HandleTypeDef hspi_dw1000;
SpiHandle_st hDw1000_spi = {
  .handle            = &hspi_dw1000,
  .txBufLens         = DW1000_MAX_TXBUF,
  .rxBufLens         = DW1000_MAX_RXBUF,
  .pTxBuf            = NULL,
  .pRxBuf            = NULL,
  .txEventCallback   = NULL,
  .rxEventCallback   = NULL,
  .txrxEventCallback = NULL
};

/* Private function prototypes -------------------------------------------------------------*/
/* MSP functions ---------------------------------------------------------------------------*/

/**
 *  @brief SPI MSP Initialization
 */
void HAL_SPI_MspInit( SPI_HandleTypeDef *hspi )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  if (hspi->Instance == DW1000_SPIx) {
    /* SPI Clk *******************************************************************/
    DW1000_SPIx_CLK_ENABLE();

    /* SPI Pin *******************************************************************/
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;

    GPIO_InitStruct.Pin       = DW1000_SCK_PIN;
    GPIO_InitStruct.Alternate = DW1000_SCK_AF;
    HAL_GPIO_Init(DW1000_SCK_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = DW1000_SDO_PIN;
    GPIO_InitStruct.Alternate = DW1000_SDO_AF;
    HAL_GPIO_Init(DW1000_SDO_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = DW1000_SDI_PIN;
    GPIO_InitStruct.Alternate = DW1000_SDI_AF;
    HAL_GPIO_Init(DW1000_SDI_GPIO_PORT, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(DW1000_SPIx_IRQn, DW1000_SPIx_IRQn_PREEMPT, DW1000_SPIx_IRQn_SUB);
//    HAL_NVIC_EnableIRQ(DW1000_SPIx_IRQn);
  }
}

/**
 *  @brief SPI MSP De-Initialization
 */
void HAL_SPI_MspDeInit( SPI_HandleTypeDef *hspi )
{
  if (hspi->Instance == DW1000_SPIx) {
    DW1000_SPIx_FORCE_RESET();
    DW1000_SPIx_RELEASE_RESET();
    HAL_GPIO_DeInit(DW1000_SCK_GPIO_PORT, DW1000_SCK_PIN);
    HAL_GPIO_DeInit(DW1000_SDO_GPIO_PORT, DW1000_SDO_PIN);
    HAL_GPIO_DeInit(DW1000_SDI_GPIO_PORT, DW1000_SDI_PIN);
    HAL_NVIC_DisableIRQ(DW1000_SPIx_IRQn);
  }
}

/**
 *  @brief Tx Transfer completed callback.
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == DW1000_SPIx) {
    hDw1000_spi.txEventCallback();
  }
}

/**
 *  @brief Rx Transfer completed callback.
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == DW1000_SPIx) {
    hDw1000_spi.rxEventCallback();
  }
}

/**
 *  @brief Tx and Rx Transfer completed callback.
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == DW1000_SPIx) {
    hDw1000_spi.txrxEventCallback();
  }
}

/**
 *  @brief Tx Half Transfer completed callback.
 */
void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == DW1000_SPIx) {

  }
}

/**
 *  @brief Rx Half Transfer completed callback.
 */
void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == DW1000_SPIx) {

  }
}

/**
 *  @brief Tx and Rx Half Transfer callback.
 */
void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == DW1000_SPIx) {

  }
}

/**
 *  @brief SPI error callback.
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == DW1000_SPIx) {

  }
}

/* Private functions -----------------------------------------------------------------------*/

/**
 *  @brief  SPI_SetSpeed
 */
void SPI_SetSpeed( SPI_HandleTypeDef *hspi, uint32_t speedSel )
{
  __HAL_SPI_DISABLE(hspi);
  hspi->Init.BaudRatePrescaler = speedSel;
  HAL_SPI_Init(hspi);
  __HAL_SPI_ENABLE(hspi);
}

/**
 *  @brief  SPI_RW
 */
uint16_t SPI_RW( SPI_HandleTypeDef *hspi, uint16_t sendByte )
{
  while ((hspi->Instance->SR & SPI_FLAG_TXE) == (uint16_t)RESET);
  hspi->Instance->DR = sendByte;
  while ((hspi->Instance->SR & SPI_FLAG_RXNE) == (uint16_t)RESET);
  return hspi->Instance->DR;
}

#if !KS_FW_SPI_HAL_LIBRARY

/**
 *  @brief  SPI Send Data
 */
uint32_t SPI_SendData( SPI_HandleTypeDef *hspi, uint8_t *sendData, uint16_t lens, uint32_t timeout )
{
  uint32_t tickStart = 0;

  if ((sendData == NULL) || (lens == 0)) {
    return HAL_ERROR;
  }

  while (lens--) {
    tickStart = HAL_GetTick();
    while ((hspi->Instance->SR & SPI_FLAG_TXE) == RESET) {
      if ((HAL_GetTick() - tickStart) > timeout) {
        return HAL_TIMEOUT;
      }
    }
    *((__IO uint8_t*)&hspi->Instance->DR) = *sendData++;
  }

  return HAL_OK;
}

/**
 *  @brief  SPI Recv Data
 */
uint32_t SPI_RecvData( SPI_HandleTypeDef *hspi, uint8_t *recvData, uint16_t lens, uint32_t timeout )
{
  uint32_t tickStart = 0;

  if ((recvData == NULL) || (lens == 0)) {
    return HAL_ERROR;
  }

  while (lens--) {
    tickStart = HAL_GetTick();
    while ((hspi->Instance->SR & SPI_FLAG_TXE) == RESET) {
      if ((HAL_GetTick() - tickStart) > timeout) {
        return HAL_TIMEOUT;
      }
    }
    *((__IO uint8_t*)&hspi->Instance->DR) = 0x00;
    tickStart = HAL_GetTick();
    while ((hspi->Instance->SR & SPI_FLAG_RXNE) == RESET) {
      if ((HAL_GetTick() - tickStart) > timeout) {
        return HAL_TIMEOUT;
      }
    }
    *recvData++ = *((__IO uint8_t*)&hspi->Instance->DR);
  }

  return HAL_OK;
}
#endif

/*************************************** END OF FILE ****************************************/
