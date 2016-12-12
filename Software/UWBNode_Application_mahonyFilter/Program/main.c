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
/* Private define --------------------------------------------------------------------------*/
#define PACKET_DATALENS 19

/* Private macro ---------------------------------------------------------------------------*/
/* Private variables -----------------------------------------------------------------------*/
extern IMU_DataTypeDef IMU;

MahonyFilter_t mf;
static float32_t sendBuf[PACKET_DATALENS] = {0};
struct {
  float32_t msec;
  float32_t sec;
  float32_t min;
} time;

__IO int8_t flag = HAL_ERROR;

/* Private function prototypes -------------------------------------------------------------*/
void TIMER_CallBack( void );

/* Private functions -----------------------------------------------------------------------*/

int main( void )
{
  HAL_Init();
  BSP_GPIO_Config();
  BSP_TIMER2_Config(TIMER_CallBack, SAMPLE_RATE_FREQ);
  BSP_UART_Config(NULL, NULL);
  BSP_IMU_Config();
  MahonyFilter_Init(&mf, &IMU, SAMPLE_RATE);

  time.msec = 0.0f;
  time.sec  = 0.0f;
  time.min  = 0.0f;

  Timer2_Cmd(ENABLE);
  while (1) {
    if (flag == HAL_OK) {
      flag = HAL_ERROR;
      sendBuf[0]  = time.min;
      sendBuf[1]  = time.sec;
      sendBuf[2]  = time.msec;
      sendBuf[3]  = IMU.gyrData[0];
      sendBuf[4]  = IMU.gyrData[1];
      sendBuf[5]  = IMU.gyrData[2];
      sendBuf[6]  = IMU.accData[0];
      sendBuf[7]  = IMU.accData[1];
      sendBuf[8]  = IMU.accData[2];
      sendBuf[9]  = IMU.magData[0];
      sendBuf[10] = IMU.magData[1];
      sendBuf[11] = IMU.magData[2];
      sendBuf[12] = mf.angE.pitch;
      sendBuf[13] = mf.angE.roll;
      sendBuf[14] = mf.angE.yaw;
      sendBuf[15] = mf.numQ.q0;
      sendBuf[16] = mf.numQ.q1;
      sendBuf[17] = mf.numQ.q2;
      sendBuf[18] = mf.numQ.q3;
      kSerial_sendData(sendBuf, PACKET_DATALENS, KS_FLOAT32);
    }
  }
}

#define CUTOFF_FREQ     (0.02f)
#define TIME_CONST      (1.0f / (2.0f * PI * CUTOFF_FREQ))
#define THRESHOLD       (2.0f)
#define STIRLESS_TIME   (2.0f)
void GyroBiasCorrection( float32_t *gyro )
{
  static const float32_t alpha   = TIME_CONST / (TIME_CONST + SAMPLE_RATE);
  static const float32_t alpha_n = 1.0f - alpha;

  static float32_t stirlessTime = 0;
  static float32_t gyroBias[3] = {0};

  if ((gyro[0] > THRESHOLD) || (gyro[0] < -THRESHOLD) ||
      (gyro[1] > THRESHOLD) || (gyro[1] < -THRESHOLD) ||
      (gyro[2] > THRESHOLD) || (gyro[2] < -THRESHOLD)) {
    stirlessTime = 0.0f;
  }
  else {
    if (stirlessTime >= STIRLESS_TIME) {
      gyroBias[0] = alpha * gyroBias[0] + alpha_n * gyro[0];
      gyroBias[1] = alpha * gyroBias[1] + alpha_n * gyro[1];
      gyroBias[2] = alpha * gyroBias[2] + alpha_n * gyro[2];
    }
    else {
      stirlessTime += SAMPLE_RATE;
    }
  }

  gyro[0] -= gyroBias[0];
  gyro[1] -= gyroBias[1];
  gyro[2] -= gyroBias[2];
}

void TIMER_CallBack( void )
{
  LED_R_Set();

  time.msec += SAMPLE_RATE;
  if (time.msec < 1.0f) {
    
  }
  else {
    time.msec -= 1.0f;
    time.sec  += 1.0f;
    if (time.sec < 60.0f) {

    }
    else {
      time.sec -= 60.0f;
      time.min += 1.0f;
    }
  }

  IMU_GetScaleData(&IMU);
  GyroBiasCorrection(IMU.gyrData);
  MahonyFilter_Update(&mf);
  flag = HAL_OK;

  LED_R_Reset();
}

/*************************************** END OF FILE ****************************************/
