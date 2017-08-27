/**
 *      __            ____
 *     / /__ _  __   / __/                      __  
 *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
 *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
 *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
 *                    /_/   github.com/KitSprout    
 * 
 *  @file    stm32f4_delay.c
 *  @author  KitSprout
 *  @date    01-Aug-2017
 *  @brief   
 * 
 */
 
/* Includes --------------------------------------------------------------------------------*/
#include "stm32f4_system.h"
#include "stm32f4_delay.h"

/** @addtogroup STM32_Driver
 *  @{
 */

/* Private typedef -------------------------------------------------------------------------*/
/* Private define --------------------------------------------------------------------------*/
/* Private macro ---------------------------------------------------------------------------*/
/* Private variables -----------------------------------------------------------------------*/
/* Private function prototypes -------------------------------------------------------------*/
/* Private functions -----------------------------------------------------------------------*/

/**
 *  @brief  delay_nop
 */
void delay_nop( __IO uint32_t __n )
{
  while (__n--) {
    __nop();
  }
}

/**
 *  @brief  delay_us
 */
void delay_us( __IO uint32_t __us )
{
  while (__us--) {
    __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();
    __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();
    __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();
    __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();
    __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();
    __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();
    __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();
    __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();
    __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();
    __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();  __nop();
  }
}

/*************************************** END OF FILE ****************************************/
