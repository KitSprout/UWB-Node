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
 *  @date    27-Aug-2017
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
const uint8_t tagAddr[4]    = {0x00, 0x22, 0x20, 0xFE};
const uint8_t anchorAddr[4] = {0x00, 0x10, 0xAB, 0xCE};

/* Private macro ---------------------------------------------------------------------------*/
/* Private variables -----------------------------------------------------------------------*/
/* Private function prototypes -------------------------------------------------------------*/
void DEMO_SSTWR_INITIATOR( void );
void DEMO_SSTWR_RESPONDER( void );

/* Private functions -----------------------------------------------------------------------*/

int main( void )
{
  HAL_Init();
  BSP_GPIO_Config();
  BSP_UART_SERIAL_Config(NULL);
  BSP_UWB_Config();

  if (KEY_Read()) {
    LED_G_Reset();
    delay_ms(500);
    LED_G_Set();
    printf(" ---- INITIATOR ----\r\n");
    DEMO_SSTWR_INITIATOR();
  }
  else {
    LED_B_Reset();
    delay_ms(500);
    LED_B_Set();
    printf(" ---- RESPONDER ----\r\n");
    DEMO_SSTWR_RESPONDER();
  }

  while (1) { ; }
}

/* ---------------------------------------------------------------------------------------- *
 *                                                                                          *
 *                          Single-Sided Two-Way Ranging Initiator                          *
 *                                                                                          *
 * ---------------------------------------------------------------------------------------- */

void DEMO_SSTWR_INITIATOR( void )
{
  uint32_t status;

  uint8_t tagAddress = tagAddr[1];
  uint8_t anchorAddress = 0;

  float32_t distance[4] = {0};

  UWB_SetRxAfterTxDelay(UWB_POLL_TX_TO_RESP_RX_DLY_UUS);
  UWB_SetRxTimeout(UWB_RESP_RX_TIMEOUT_UUS);

  while (1) {
    for (uint8_t i = 0; i < 4; i++) {
      anchorAddress = anchorAddr[i];
      status = UWB_Ranging(tagAddress, &anchorAddress, &distance[i]);
      if (status == KS_OK) {
        LED_G_Toggle();
        printf(" [%02X] [OK] %10.7f m\r\n", anchorAddress, distance[i]);
      }
      else {
        printf(" [%02X] [ER] %10.7f m\r\n", anchorAddress, distance[i]);
      }
    }
    printf("\r\n");
    delay_ms(100);
  }
}

/* ---------------------------------------------------------------------------------------- *
 *                                                                                          *
 *                          Single-Sided Two-Way Ranging Responder                          *
 *                                                                                          *
 * ---------------------------------------------------------------------------------------- */

void DEMO_SSTWR_RESPONDER( void )
{
  uint32_t status;

  uint8_t tagAddress = 0;
  uint8_t anchorAddress = 0;

  while (1) {
    anchorAddress = anchorAddr[2];
    status = UWB_RangingResp(&tagAddress, &anchorAddress);
    if (status == KS_OK) {
      LED_B_Toggle();
      printf(" TAG[%02X] ANCHOR[%02X] [OK]\r\n", tagAddress, anchorAddress);
    }
    else {
      printf(" TAG[%02X] ANCHOR[%02X] [ER]\r\n", tagAddress, anchorAddress);
    }
  }
}

/*************************************** END OF FILE ****************************************/
