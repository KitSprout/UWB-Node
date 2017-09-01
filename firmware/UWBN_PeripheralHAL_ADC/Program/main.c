/**
 *      __            ____
 *     / /__ _  __   / __/                      __  
 *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
 *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
 *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
 *                    /_/   github.com/KitSprout    
 * 
 *  @file    main.c
 *  @author  KitSprout
 *  @date    19-Aug-2017
 *  @brief   
 * 
 */

/* Includes --------------------------------------------------------------------------------*/
#include "drivers\stm32f4_system.h"
#include "drivers\stm32f4_adc.h"
#include "modules\serial.h"
#include "stm32f4xx_bsp.h"

/** @addtogroup STM32_Program
 *  @{
 */

/* Private typedef -------------------------------------------------------------------------*/
/* Private define --------------------------------------------------------------------------*/
/* Private macro ---------------------------------------------------------------------------*/
/* Private variables -----------------------------------------------------------------------*/
/* Private function prototypes -------------------------------------------------------------*/
/* Private functions -----------------------------------------------------------------------*/

int main( void )
{
  uint16_t ave[AD1_MAX_CHANNEL] = {0};
  int16_t vin, temp;

  HAL_Init();
  BSP_GPIO_Config();
  BSP_ADC_Config();
  BSP_UART_SERIAL_Config(NULL);

  while (1) {
    ADC_GetAdc1AveValue(ave);

    temp = ADC_ConvInternalTemperature(ave[TEMP_ADCx_RANK - 1]);
    vin  = ADC_ConvVoltageInput(ave[BAT_ADCx_RANK - 1]);

    printf("TEMP = %4i degC, VIN = %4i mV\r\n", temp, vin);
    LED_G_Toggle();
    delay_ms(200);
  }
}

/*************************************** END OF FILE ****************************************/
