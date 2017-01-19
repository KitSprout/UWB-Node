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
  * @date    19-Nov-2016
  * @brief   
  * 
  */

/* Includes --------------------------------------------------------------------------------*/
#include "drivers\stm32f4_system.h"
#include "drivers\stm32f4_tim.h"
#include "modules\serial.h"
#include "modules\kSerial.h"
#include "modules\imu.h"
#include "algorithms\mahonyFilter.h"
#include "stm32f4xx_bsp.h"

/** @addtogroup STM32_Program
  * @{
  */

/* Private typedef -------------------------------------------------------------------------*/
enum {
  FSM_INIT  = 0,
  FSM_RUN,
  FSM_RUNOK,
};
/* Private define --------------------------------------------------------------------------*/
#define PACKET_DATALENS   15

/* Private macro ---------------------------------------------------------------------------*/
#define TIME_CONST(_FREQ)   (1.0f / (2.0f * PI * _FREQ))

/* Private variables -----------------------------------------------------------------------*/
extern IMU_DataTypeDef IMU;

MahonyFilter_t mf;
static float32_t sendBuf[PACKET_DATALENS] = {0};
__IO int8_t fsm_state = FSM_RUN;

float32_t t_s  = 0.0f;
float32_t t_ms = 0.0f;

/* Private function prototypes -------------------------------------------------------------*/
void TIMER_CallBack( void );

/* Private functions -----------------------------------------------------------------------*/

int main( void )
{
  HAL_Init();
  BSP_GPIO_Config();
  BSP_TIMER2_Config(TIMER_CallBack, SAMPLE_RATE);
  BSP_UART_Config(NULL, NULL);
  BSP_IMU_Config();
  MahonyFilter_Init(&mf, &IMU, SAMPLE_TIME);
  Timer2_Cmd(ENABLE);

  while (1) {
    if (fsm_state == FSM_RUNOK) {
      fsm_state = FSM_RUN;
      sendBuf[0]  = t_s;
      sendBuf[1]  = t_ms;
      sendBuf[2]  = IMU.gyrData[0];
      sendBuf[3]  = IMU.gyrData[1];
      sendBuf[4]  = IMU.gyrData[2];
      sendBuf[5]  = IMU.accData[0];
      sendBuf[6]  = IMU.accData[1];
      sendBuf[7]  = IMU.accData[2];
      sendBuf[8]  = IMU.magData[0];
      sendBuf[9]  = IMU.magData[1];
      sendBuf[10] = IMU.magData[2];
      sendBuf[11] = mf.numQ.q0;
      sendBuf[12] = mf.numQ.q1;
      sendBuf[13] = mf.numQ.q2;
      sendBuf[14] = mf.numQ.q3;
      kSerial_sendData(sendBuf, PACKET_DATALENS, KS_FLOAT32);
    }
  }
}

void TIMER_CallBack( void )
{
  static uint16_t t_count = 0;

  if (++t_count == SAMPLE_RATE) {
    t_count = 0;
    t_s += 1.0f;
  }
  t_ms = t_count * SAMPLE_TIME;

  IMU_GetScaleData(&IMU);

  switch(fsm_state) {
    case FSM_RUN:
    case FSM_RUNOK:
      LED_G_Set();
      MahonyFilter_Update(&mf);
      fsm_state = FSM_RUNOK;
      LED_G_Reset();
      break;
  }
}

/*************************************** END OF FILE ****************************************/
