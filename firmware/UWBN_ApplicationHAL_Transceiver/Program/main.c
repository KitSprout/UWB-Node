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
 *  @date    03-Sep-2017
 *  @brief   
 * 
 */

/* Includes --------------------------------------------------------------------------------*/
#include "drivers\stm32f4_system.h"
#include "modules\serial.h"
#include "modules\kSerial.h"
#include "stm32f4xx_bsp.h"
#include "ultraWideband.h"

/** @addtogroup STM32_Program
 *  @{
 */

/* Private typedef -------------------------------------------------------------------------*/
/* Private define --------------------------------------------------------------------------*/
#define TX_BUFFLENS   10
#define RX_BUFFLENS   TX_BUFFLENS

/* Private macro ---------------------------------------------------------------------------*/
/* Private variables -----------------------------------------------------------------------*/
static uint8_t tx_buffer[TX_BUFFLENS];
static uint8_t rx_buffer[RX_BUFFLENS];

/* Private function prototypes -------------------------------------------------------------*/
/* Private functions -----------------------------------------------------------------------*/

int main( void )
{
  uint16_t count = 0;

  HAL_Init();
  BSP_GPIO_Config();
  BSP_UART_SERIAL_Config(NULL);
  BSP_UWB_Config();

  if (KEY_Read()) {
    while (1) {
      LED_B_Toggle();
      count++;
      kSerial_Pack(tx_buffer, &count, NULL, 0, KS_NTYPE);
      UWB_SendPacket(tx_buffer, TX_BUFFLENS, UWB_IMMEDIATE);
      printf("[TX] ");
      for (uint32_t i = 0; i < TX_BUFFLENS; i++) {
        printf("%02X ", tx_buffer[i]);
      }
      printf("\r\n");
      delay_ms(200);
    }
  }
  else {
    while (1) {
      LED_G_Toggle();
      UWB_RecvPacket(rx_buffer, RX_BUFFLENS, UWB_IMMEDIATE);
      printf("[RX] ");
      for (uint32_t i = 0; i < RX_BUFFLENS; i++) {
        printf("%02X ", rx_buffer[i]);
      }
      printf("\r\n");
    }
  }
}

/*************************************** END OF FILE ****************************************/
