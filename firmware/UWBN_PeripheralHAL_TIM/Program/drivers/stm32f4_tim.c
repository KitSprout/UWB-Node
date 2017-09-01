/**
 *      __            ____
 *     / /__ _  __   / __/                      __  
 *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
 *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
 *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
 *                    /_/   github.com/KitSprout    
 * 
 *  @file    stm32f4_tim.c
 *  @author  KitSprout
 *  @date    16-Aug-2017
 *  @brief   
 * 
 */

/* Includes --------------------------------------------------------------------------------*/
#include "stm32f4_system.h"
#include "stm32f4_tim.h"

/** @addtogroup STM32_Driver
 *  @{
 */

/* Private typedef -------------------------------------------------------------------------*/
/* Private define --------------------------------------------------------------------------*/
/* Private macro ---------------------------------------------------------------------------*/
/* Private variables -----------------------------------------------------------------------*/
static TIM_HandleTypeDef htim_timer2;
static TIM_HandleTypeDef htim_timer3;
static TIM_HandleTypeDef htim_timer4;

TimHandle_st hTimer2 = {
  .handle            = &htim_timer2,
  .tickEventCallback = NULL,
};

TimHandle_st hTimer3 = {
  .handle            = &htim_timer3,
  .tickEventCallback = NULL,
};

TimHandle_st hTimer4 = {
  .handle            = &htim_timer4,
  .tickEventCallback = NULL,
};

/* Private function prototypes -------------------------------------------------------------*/
/* Private functions -----------------------------------------------------------------------*/

void Timer2_Config( uint32_t prescaler, uint32_t period )
{
  /* TIMX Clk ******************************************************************/
  TIMER2_CLK_ENABLE();

  /* NVIC Config ***************************************************************/
  HAL_NVIC_SetPriority(TIMER2_IRQn, TIMER2_TIMx_IRQn_PREEMPT, TIMER2_TIMx_IRQn_SUB);
  HAL_NVIC_EnableIRQ(TIMER2_IRQn);

  /* TIM Base Config ************************************************************/
  hTimer2.handle->Instance               = TIMER2;
  hTimer2.handle->Init.Prescaler         = prescaler - 1;
  hTimer2.handle->Init.Period            = period - 1;
  hTimer2.handle->Init.ClockDivision     = 0;
  hTimer2.handle->Init.CounterMode       = TIM_COUNTERMODE_UP;
  hTimer2.handle->Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(hTimer2.handle);

  /* TIM Enable *****************************************************************/
  HAL_TIM_Base_Start_IT(hTimer2.handle);
}

void Timer3_Config( uint32_t prescaler, uint32_t period )
{
  /* TIMX Clk ******************************************************************/
  TIMER3_CLK_ENABLE();

  /* NVIC Config ***************************************************************/
  HAL_NVIC_SetPriority(TIMER3_IRQn, TIMER3_TIMx_IRQn_PREEMPT, TIMER3_TIMx_IRQn_SUB);
  HAL_NVIC_EnableIRQ(TIMER3_IRQn);

  /* TIM Base Config ************************************************************/
  hTimer3.handle->Instance               = TIMER3;
  hTimer3.handle->Init.Prescaler         = prescaler - 1;
  hTimer3.handle->Init.Period            = period - 1;
  hTimer3.handle->Init.ClockDivision     = 0;
  hTimer3.handle->Init.CounterMode       = TIM_COUNTERMODE_UP;
  hTimer3.handle->Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(hTimer3.handle);

  /* TIM Enable *****************************************************************/
  HAL_TIM_Base_Start_IT(hTimer3.handle);
}

void Timer4_Config( uint32_t prescaler, uint32_t period )
{
  /* TIMX Clk ******************************************************************/
  TIMER4_CLK_ENABLE();

  /* NVIC Config ***************************************************************/
  HAL_NVIC_SetPriority(TIMER4_IRQn, TIMER4_TIMx_IRQn_PREEMPT, TIMER4_TIMx_IRQn_SUB);
  HAL_NVIC_EnableIRQ(TIMER4_IRQn);

  /* TIM Base Config ************************************************************/
  hTimer4.handle->Instance               = TIMER4;
  hTimer4.handle->Init.Prescaler         = prescaler - 1;
  hTimer4.handle->Init.Period            = period - 1;
  hTimer4.handle->Init.ClockDivision     = 0;
  hTimer4.handle->Init.CounterMode       = TIM_COUNTERMODE_UP;
  hTimer4.handle->Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(hTimer4.handle);

  /* TIM Enable *****************************************************************/
  HAL_TIM_Base_Start_IT(hTimer4.handle);
}

void Timer2_Cmd( uint8_t cmd )
{
  if (cmd == ENABLE) {
    HAL_TIM_Base_Start_IT(hTimer2.handle);
  }
  else {
    HAL_TIM_Base_Stop_IT(hTimer2.handle);
  }
}

void Timer3_Cmd( uint8_t cmd )
{
  if (cmd == ENABLE) {
    HAL_TIM_Base_Start_IT(hTimer4.handle);
  }
  else {
    HAL_TIM_Base_Stop_IT(hTimer4.handle);
  }
}

void Timer4_Cmd( uint8_t cmd )
{
  if (cmd == ENABLE) {
    HAL_TIM_Base_Start_IT(hTimer4.handle);
  }
  else {
    HAL_TIM_Base_Stop_IT(hTimer4.handle);
  }
}

/*************************************** END OF FILE ****************************************/
