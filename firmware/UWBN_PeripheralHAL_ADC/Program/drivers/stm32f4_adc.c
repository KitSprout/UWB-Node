/**
 *      __            ____
 *     / /__ _  __   / __/                      __  
 *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
 *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
 *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
 *                    /_/   github.com/KitSprout    
 * 
 *  @file    stm32f4_adc.c
 *  @author  KitSprout
 *  @date    19-Aug-2017
 *  @brief   
 * 
 */

/* Includes --------------------------------------------------------------------------------*/
#include "stm32f4_system.h"
#include "stm32f4_adc.h"

/** @addtogroup STM32_Driver
 *  @{
 */

/* Private typedef -------------------------------------------------------------------------*/
/* Private define --------------------------------------------------------------------------*/
/* Private macro ---------------------------------------------------------------------------*/
/* Private variables -----------------------------------------------------------------------*/
static __IO uint16_t adc1ConvBuff[AD1_MAX_SAMPLEBUF][AD1_MAX_CHANNEL] = {0};

static ADC_HandleTypeDef hadc_ad1;
AdcHandle_st hAdc1 = {
  .handle   = &hadc_ad1,
  .convBuff = (uint16_t*)adc1ConvBuff,
};

/* Private function prototypes -------------------------------------------------------------*/
/* MSP functions ---------------------------------------------------------------------------*/

void HAL_ADC_MspInit( ADC_HandleTypeDef *hadc )
{
  GPIO_InitTypeDef GPIO_InitStruct;
  static DMA_HandleTypeDef hAdc1_dma;

  if (hadc->Instance == AD1_ADCx) {

    /* ADC and DMA Clk ***********************************************************/
    AD1_ADCx_CLK_ENABLE();
    AD1_ADCx_DMA_CLK_ENABLE();

    /* ADC Pin *******************************************************************/
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    GPIO_InitStruct.Pin  = BAT_ADCx_CHANNEL_PIN;
    HAL_GPIO_Init(BAT_ADCx_CHANNEL_GPIO_PORT, &GPIO_InitStruct);

    /* DMA Init ******************************************************************/
    hAdc1_dma.Instance                 = AD1_ADCx_DMA_STREAM;
    hAdc1_dma.Init.Channel             = AD1_ADCx_DMA_CHANNEL;
    hAdc1_dma.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hAdc1_dma.Init.PeriphInc           = DMA_PINC_DISABLE;
    hAdc1_dma.Init.MemInc              = DMA_MINC_ENABLE;
    hAdc1_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hAdc1_dma.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hAdc1_dma.Init.Mode                = DMA_CIRCULAR;
    hAdc1_dma.Init.Priority            = DMA_PRIORITY_HIGH;
    hAdc1_dma.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hAdc1_dma.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_HALFFULL;
    hAdc1_dma.Init.MemBurst            = DMA_MBURST_SINGLE;
    hAdc1_dma.Init.PeriphBurst         = DMA_PBURST_SINGLE;
    HAL_DMA_Init(&hAdc1_dma);
    __HAL_LINKDMA(hadc, DMA_Handle, hAdc1_dma);

    /* DMA IT ********************************************************************/
    HAL_NVIC_SetPriority(AD1_ADCx_DMA_IRQn, AD1_ADCx_DMA_IRQn_PREEMPT, AD1_ADCx_DMA_IRQn_SUB);
    HAL_NVIC_EnableIRQ(AD1_ADCx_DMA_IRQn);
  }
}

/**
 *  @brief ADC MSP De-Initialization
 */
void HAL_ADC_MspDeInit( ADC_HandleTypeDef *hadc )
{
  if (hadc->Instance == AD1_ADCx) {
    ADCx_FORCE_RESET();
    ADCx_RELEASE_RESET();
    HAL_GPIO_DeInit(BAT_ADCx_CHANNEL_GPIO_PORT, BAT_ADCx_CHANNEL_PIN);
  }
}

/**
 *  @brief  Regular conversion complete callback in non blocking mode 
 */
void HAL_ADC_ConvCpltCallback( ADC_HandleTypeDef* hadc )
{
  if (hadc->Instance == AD1_ADCx) {

  }
}

/**
 *  @brief  Regular conversion half DMA transfer callback in non blocking mode 
 */
void HAL_ADC_ConvHalfCpltCallback( ADC_HandleTypeDef* hadc )
{
  if (hadc->Instance == AD1_ADCx) {

  }
}

/**
 *  @brief  Analog watchdog callback in non blocking mode 
 */
void HAL_ADC_LevelOutOfWindowCallback( ADC_HandleTypeDef* hadc )
{
  if (hadc->Instance == AD1_ADCx) {

  }
}

/**
 *  @brief  Error ADC callback.
 */
void HAL_ADC_ErrorCallback( ADC_HandleTypeDef *hadc )
{
  if (hadc->Instance == AD1_ADCx) {

  }
}

/**
 *  @brief  Injected conversion complete callback in non blocking mode 
 */
void HAL_ADCEx_InjectedConvCpltCallback( ADC_HandleTypeDef* hadc )
{
  if (hadc->Instance == AD1_ADCx) {

  }
}

/* Private functions -----------------------------------------------------------------------*/

/**
 *  @brief  ADC_Config
 */
void ADC_Config( void )
{
  ADC_ChannelConfTypeDef ADC_ChannelConfStruct;

  /* ADC Init ******************************************************************/
  hAdc1.handle->Instance                   = AD1_ADCx;
  hAdc1.handle->Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hAdc1.handle->Init.Resolution            = ADC_RESOLUTION_12B;
  hAdc1.handle->Init.ScanConvMode          = ENABLE;
  hAdc1.handle->Init.ContinuousConvMode    = ENABLE;
  hAdc1.handle->Init.DiscontinuousConvMode = DISABLE;
  hAdc1.handle->Init.NbrOfDiscConversion   = 0;
  hAdc1.handle->Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hAdc1.handle->Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
  hAdc1.handle->Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hAdc1.handle->Init.NbrOfConversion       = AD1_MAX_CHANNEL;
  hAdc1.handle->Init.DMAContinuousRequests = ENABLE;
  hAdc1.handle->Init.EOCSelection          = DISABLE;
  if (HAL_ADC_Init(hAdc1.handle) != HAL_OK)
    while (1) { ; }

  /* ADC Channel Init **********************************************************/
  ADC_ChannelConfStruct.Channel      = TEMP_ADCx_CHANNEL;
  ADC_ChannelConfStruct.Rank         = TEMP_ADCx_RANK;
  ADC_ChannelConfStruct.SamplingTime = TEMP_ADCx_SAMPLETIME;
  ADC_ChannelConfStruct.Offset       = 0;
  if (HAL_ADC_ConfigChannel(hAdc1.handle, &ADC_ChannelConfStruct) != HAL_OK) {
    while (1) { ; }
  }

  ADC_ChannelConfStruct.Channel      = BAT_ADCx_CHANNEL;
  ADC_ChannelConfStruct.Rank         = BAT_ADCx_RANK;
  ADC_ChannelConfStruct.SamplingTime = BAT_ADCx_SAMPLETIME;
  ADC_ChannelConfStruct.Offset       = 0;
  if (HAL_ADC_ConfigChannel(hAdc1.handle, &ADC_ChannelConfStruct) != HAL_OK) {
    while (1) { ; }
  }

  /* ADC Start *****************************************************************/
  if (HAL_ADC_Start_DMA(hAdc1.handle, (uint32_t*)hAdc1.convBuff, AD1_MAX_CHANNEL * AD1_MAX_SAMPLEBUF) != HAL_OK) {
    while (1) { ; }
  }
}

/**
 *  @brief  ADC_GetAdc1AveValue
 */
void ADC_GetAdc1AveValue( uint16_t *ave )
{
  uint32_t tmp[AD1_MAX_CHANNEL] = {0};

  for (uint32_t i = 0; i < AD1_MAX_CHANNEL; i++) {
    for (uint32_t j = 0; j < AD1_MAX_SAMPLEBUF; j++) {
      tmp[i] += adc1ConvBuff[j][i];
    }
    ave[i] = tmp[i] / AD1_MAX_SAMPLEBUF;
  }
}

/**
 *  @brief  ADC_ConvInternalTemperature (degC, -40 ~ 125, 100x)
 */
int16_t ADC_ConvInternalTemperature( uint16_t adcdata )
{
  float32_t tmp;
  float32_t temperature;

  tmp = adcdata * (ADC_VREF_VOL / 4096.0f);
  temperature = (tmp - 0.76f) / 0.0025f + 25.0f;

  return (int16_t)(temperature * 100);
}

/**
 *  @brief  ADC_ConvVoltageInput (mV)
 */
int16_t ADC_ConvVoltageInput( uint16_t adcdata )
{
  float32_t tmp;
  float32_t vin;

  tmp = adcdata * (ADC_VREF_VOL / 4096.0f);
  vin = tmp / ADC_VBAT_CONV;

  return (int16_t)(vin * 1000);
}

/*************************************** END OF FILE ****************************************/
