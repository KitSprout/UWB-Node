/**
  *      __            ____
  *     / /__ _  __   / __/                      __  
  *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
  *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
  *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
  *                    /_/   github.com/KitSprout    
  * 
  * @file    main.c
  * @author  KitSprout
  * @date    28-Oct-2016
  * @brief   
  * 
  */

/* Includes --------------------------------------------------------------------------------*/
#include "drivers\stm32f4_system.h"
#include "modules\serial.h"
#include "uwbNode_bsp.h"

/** @addtogroup STM32_Program
  * @{
  */

/* Private typedef -------------------------------------------------------------------------*/
/* Private define --------------------------------------------------------------------------*/
/* Private macro ---------------------------------------------------------------------------*/
/* Private variables -----------------------------------------------------------------------*/
/* Private function prototypes -------------------------------------------------------------*/
void Serial_RecvIRQ( void );

/* Private functions -----------------------------------------------------------------------*/

int main( void )
{
  HAL_InitTick();

  UWBN_GPIO_Config();
  UWBN_UART_Config(Serial_RecvIRQ);

  while (1) {
    while (KEY_Read()) {
      printf("/f");
    }
  }
}

void Serial_RecvIRQ( void )
{
  uint8_t recvData = Serial_RecvByte();
  if (recvData == 0x0D)
    Serial_SendStr("\r\n");
  else
    Serial_SendByte(recvData);
}

/*************************************** END OF FILE ****************************************/
