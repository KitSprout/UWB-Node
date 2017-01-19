/**
  *      __            ____
  *     / /__ _  __   / __/                      __  
  *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
  *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
  *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
  *                    /_/   github.com/KitSprout    
  * 
  * @file    mahonyFilter.c
  * @author  KitSprout
  * @date    12-Oct-2016
  * @brief   
  * 
  */

/* Includes --------------------------------------------------------------------------------*/
#include "drivers\stm32f4_system.h"
#include "algorithms\mahonyFilter.h"

/** @addtogroup STM32_Algorithms
  * @{
  */

/* Private typedef -------------------------------------------------------------------------*/
/* Private define --------------------------------------------------------------------------*/
#define MF_A_THR  0.4f
#define MF_M_THR  0.1f

#define MF_KPA 0.4f
//#define MF_KIA 0.001f

#define MF_KPM 0.01f
//#define MF_KIM 0.001f

//#define MAHONY_FILTER_9DOF

/* Private macro ---------------------------------------------------------------------------*/
/* Private variables -----------------------------------------------------------------------*/
/* Private function prototypes -------------------------------------------------------------*/
/* Private functions -----------------------------------------------------------------------*/

/**
  * @brief  MahonyFilter_Init
  * @param  mf: point to struct of mahonyFilter
  * @param  imu: 
  * @retval None
  */
void MahonyFilter_Init( MahonyFilter_t *mf, IMU_DataTypeDef *imu, float32_t sampleTime )
{
  mf->angE.pitch = 0.0f;
  mf->angE.roll  = 0.0f;
  mf->angE.yaw   = 0.0f;

  Quaternion_Clear(&mf->numQ);

  mf->imu = imu;

  mf->sampleTime = sampleTime;
}

/**
  * @brief  MahonyFilter_Update
  * @param  mf: point to struct of mahonyFilter
  * @retval None
  */
void MahonyFilter_Update( MahonyFilter_t *mf )
{
  float32_t err[3];

  float32_t gVect[3];
  float32_t gyr[3], acc[3];
#if defined(MF_KIA)
  static float32_t errIntA[3] = {0};
#endif

#if defined(MAHONY_FILTER_9DOF)
  float32_t hVect[3], wVect[3];
  float32_t mag[3];
#if defined(MF_KIM)
  static float32_t errIntM[3] = {0};
#endif
#endif

  gyr[0] = toRad(mf->imu->gyrData[0]);
  gyr[1] = toRad(mf->imu->gyrData[1]);
  gyr[2] = toRad(mf->imu->gyrData[2]);
  acc[0] = mf->imu->accData[0];
  acc[1] = mf->imu->accData[1];
  acc[2] = mf->imu->accData[2];

  /* Normalise accelerometer measurement */
  const float32_t normAcc = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
  if ((normAcc < (mf->imu->accStrength * (1.0f + MF_A_THR))) && (normAcc > (mf->imu->accStrength * (1.0f - MF_A_THR)))) {
    acc[0] /= normAcc;
    acc[1] /= normAcc;
    acc[2] /= normAcc;

    /* Estimated direction of gravity */
    gVect[0] = mf->numQ.rMat[0][2];
    gVect[1] = mf->numQ.rMat[1][2];
    gVect[2] = mf->numQ.rMat[2][2];

    err[0] = acc[1] * gVect[2] - acc[2] * gVect[1];
    err[1] = acc[2] * gVect[0] - acc[0] * gVect[2];
    err[2] = acc[0] * gVect[1] - acc[1] * gVect[0];

#if defined(MF_KIA)
    /* Compute and apply integral feedback */
    errIntA[0] += MF_KIA * err[0];
    errIntA[1] += MF_KIA * err[1];
    errIntA[2] += MF_KIA * err[2];

    /* Apply proportional feedback */
    gyr[0] += MF_KPA * err[0] + errIntA[0];
    gyr[1] += MF_KPA * err[1] + errIntA[1];
    gyr[2] += MF_KPA * err[2] + errIntA[2];

#else
    gyr[0] += MF_KPA * err[0];
    gyr[1] += MF_KPA * err[1];
    gyr[2] += MF_KPA * err[2];

#endif
  }

#if defined(MAHONY_FILTER_9DOF)
  mag[0] = mf->imu->magData[0];
  mag[1] = mf->imu->magData[1];
  mag[2] = mf->imu->magData[2];
  const float32_t normMag = sqrtf(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
  if ((normMag < (mf->imu->magStrength * (1.0f + MF_M_THR))) && (normMag > (mf->imu->magStrength * (1.0f - MF_M_THR)))) {
  }

  /* Reference direction of Earth's magnetic field */
  wVect[0] = acc[1] * mag[2] - acc[2] * mag[1];
  wVect[1] = acc[2] * mag[0] - acc[0] * mag[2];
  wVect[2] = acc[0] * mag[1] - acc[1] * mag[0];

  /* Normalise magnetometer measurement */
  const float32_t normVect = invSqrtf(wVect[0] * wVect[0] + wVect[1] * wVect[1] + wVect[2] * wVect[2]);
  wVect[0] *= normVect;
  wVect[1] *= normVect;
  wVect[2] *= normVect;

  /* Estimated direction of Earth's magnetic field  */
  hVect[0] = mf->numQ.rMat[0][1];
  hVect[1] = mf->numQ.rMat[1][1];
  hVect[2] = mf->numQ.rMat[2][1];

  /* Error is sum of cross product between estimated direction and measured direction of field vectors */
  err[0] = wVect[1] * hVect[2] - wVect[2] * hVect[1];
  err[1] = wVect[2] * hVect[0] - wVect[0] * hVect[2];
  err[2] = wVect[0] * hVect[1] - wVect[1] * hVect[0];

#if defined(MF_KIA)
    /* Compute and apply integral feedback */
    errIntA[0] += MF_KIM * err[0];
    errIntA[1] += MF_KIM * err[1];
    errIntA[2] += MF_KIM * err[2];

    /* Apply proportional feedback */
    gyr[0] += MF_KPM * err[0] + errIntM[0];
    gyr[1] += MF_KPM * err[1] + errIntM[1];
    gyr[2] += MF_KPM * err[2] + errIntM[2];

#else
    gyr[0] += MF_KPM * err[0];
    gyr[1] += MF_KPM * err[1];
    gyr[2] += MF_KPM * err[2];

#endif

#endif

  /* Integrate rate of change of quaternion */
  Quaternion_RungeKutta(&mf->numQ, gyr, mf->sampleTime * 0.5f);
  Quaternion_Normalize(&mf->numQ, &mf->numQ);
  Quaternion_UpdateRotMatrix(&mf->numQ);
//  Quaternion_ToAngE(&mf->angE, &mf->numQ);
}

/*************************************** END OF FILE ****************************************/
