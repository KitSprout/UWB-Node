/**
  *      __            ____
  *     / /__ _  __   / __/                      __  
  *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
  *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
  *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
  *                    /_/   github.com/KitSprout    
  * 
  * @file    stm32f4_tim.c
  * @author  KitSprout
  * @date    16-Nov-2016
  * @brief   
  * 
  */

/* Includes --------------------------------------------------------------------------------*/
#include "stm32f4_system.h"
#include "stm32f4_tim.h"

/** @addtogroup STM32_Driver
  * @{
  */

/* Private typedef -------------------------------------------------------------------------*/
/* Private define --------------------------------------------------------------------------*/
/* Private macro ---------------------------------------------------------------------------*/
/* Private variables -----------------------------------------------------------------------*/
static TIM_HandleTypeDef Timer2Handle;

TimHandle_st hTimer2 = {
  .handle       = &Timer2Handle,
  .EvenCallback = NULL,
};

/* Private function prototypes -------------------------------------------------------------*/
/* Private functions -----------------------------------------------------------------------*/

void Timer2_Config( uint32_t prescaler, uint32_t period )
{
  /* TIMX Clk ******************************************************************/
  TIMER2_CLK_ENABLE();

  /* NVIC Config ***************************************************************/
  HAL_NVIC_SetPriority(TIMER2_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ(TIMER2_IRQn);

  /* TIM Base Config ************************************************************/
  Timer2Handle.Instance               = TIMER2;
  Timer2Handle.Init.Prescaler         = prescaler - 1;
  Timer2Handle.Init.Period            = period - 1;
  Timer2Handle.Init.ClockDivision     = 0;
  Timer2Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  Timer2Handle.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&Timer2Handle);

  /* TIM Enable *****************************************************************/
  HAL_TIM_Base_Start_IT(&Timer2Handle);
}

void Timer2_Cmd( uint8_t cmd )
{
  if (cmd == ENABLE) {
    HAL_TIM_Base_Start_IT(&Timer2Handle);
  }
  else {
    HAL_TIM_Base_Stop_IT(&Timer2Handle);
  }
}

/*************************************** END OF FILE ****************************************/
