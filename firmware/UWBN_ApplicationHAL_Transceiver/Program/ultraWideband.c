/**
 *      __            ____
 *     / /__ _  __   / __/                      __  
 *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
 *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
 *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
 *                    /_/   github.com/KitSprout    
 * 
 *  @file    ultraWideband.c
 *  @author  KitSprout
 *  @date    27-Aug-2017
 *  @brief   
 * 
 */
 
/* Includes --------------------------------------------------------------------------------*/
#include "drivers\stm32f4_system.h"
#include "modules\kSerial.h"
#include "ultraWideband.h"

/** @addtogroup STM32_Program
 *  @{
 */

/* Private typedef -------------------------------------------------------------------------*/
/* Private define --------------------------------------------------------------------------*/
/* Private macro ---------------------------------------------------------------------------*/
/* Private variables -----------------------------------------------------------------------*/
/* Private function prototypes -------------------------------------------------------------*/
/* Private functions -----------------------------------------------------------------------*/

/**
 *  @brief  UWB_Init
 */
void UWB_Init( DWT_ConfigTypeDef *pConfig )
{
  DWT_SetSpeedLow();
  DWT_Initialise(pConfig);
  DWT_SetSpeedHigh();

  DWT_Configure(pConfig);

  DWT_SetRxAntennaDelay(UWB_RX_ANT_DLY);
  DWT_SetTxAntennaDelay(UWB_TX_ANT_DLY);
}

/**
 *  @brief  UWB_GetReplyTime
 */
int32_t UWB_GetReplyTime( uint8_t *packet )
{
  int32_t time = 0;

  for (uint32_t i = 0; i < 4; i++) {
    time += packet[i] << (i << 3);
  }

  return time;
}

/**
 *  @brief  UWB_SetReplyTime
 */
void UWB_SetReplyTime( uint8_t *packet, const uint32_t time )
{
  for (uint32_t i = 0; i < 4; i++) {
    packet[i] = (time >> (i << 3)) & 0xFF;
  }
}

/**
 *  @brief  UWB_GetInitiatorTime
 */
int32_t UWB_GetInitiatorTime( void )
{
  int32_t ti;
  uint32_t ti_1, ti_2;

  ti_1 = DWT_ReadTxTimestampL32();
  ti_2 = DWT_ReadRxTimestampL32();
  ti = ti_2 - ti_1;

  return ti;
}

/**
 *  @brief  UWB_GetResponderTime
 */
int32_t UWB_GetResponderTime( uint32_t *tx_time )
{
  int32_t tr;
  uint64_t tr_1, tr_2;

  tr_1 = DWT_ReadRxTimestamp64();
  *tx_time = (tr_1 + (UWB_POLL_RX_TO_RESP_TX_DLY_UUS * DWT_UUS_TO_DWT_TIME)) >> 8;
  tr_2 = (((uint64_t)(*tx_time & 0xFFFFFFFE)) << 8) + UWB_TX_ANT_DLY;
  tr = tr_2 - tr_1;

  return tr;
}

/**
 *  @brief  UWB_SetDelayedTxRxTime
 */
void UWB_SetDelayedTxRxTime( uint32_t time )
{
  DWT_SetDelayedTxRxTime(time);
}

/**
 *  @brief  UWB_SetRxAfterTxDelay
 */
void UWB_SetRxAfterTxDelay( uint32_t time )
{
  DWT_SetRxAfterTxDelay(time);
}

/**
 *  @brief  UWB_SetRxTimeout
 */
void UWB_SetRxTimeout( uint32_t time )
{
  DWT_SetRxTimeout(time);
}

/**
 *  @brief  UWB_GetDistance
 */
float64_t UWB_GetDistance( int32_t init, int32_t resp )
{
  float64_t tof;
  float64_t distance;

  tof = ((init - resp) / 2.0) * DWT_TIME_UNITS;
  distance = tof * UWB_SPEED_OF_LIGHT;

  return distance;
}


/**
 *  @brief  UWB_SendPacket
 */
uint32_t UWB_SendPacket( uint8_t *packet, uint16_t lens, uint8_t mode )
{
  uint32_t status;

  DWT_SetSysStatus(SYS_STATUS_TXFRS);
  DWT_WriteTxData(lens, packet, 0);
  if (mode & UWB_RANGING)
    DWT_WriteTxFCtrl(lens, 0, ENABLE);
  else {
    DWT_WriteTxFCtrl(lens, 0, DISABLE);
  }

  status = DWT_StartTx(mode & 0x03);

  if ((status == HAL_OK) && ((mode & DWT_TX_DELAYED) == DWT_TX_DELAYED)) {
    do {
      status = DWT_GetSysStatus();
    } while (!(status & SYS_STATUS_TXFRS));
    DWT_SetSysStatus(SYS_STATUS_TXFRS);
  }

  return status;
}

/**
 *  @brief  UWB_RecvPacket
 */
uint32_t UWB_RecvPacket( uint8_t *packet, uint16_t lens, uint8_t mode )
{
  uint16_t frameLens;
  uint32_t status;

  if ((mode & UWB_RX_CONTINUE) != UWB_RX_CONTINUE) {
    DWT_RxEnable(mode);
  }

  do {
    status = DWT_GetSysStatus();
  } while (!(status & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)));

  if (status & SYS_STATUS_RXFCG) {
    DWT_SetSysStatus(SYS_STATUS_RXFCG);
    frameLens = DWT_ReadData32(DW1000_RX_FINFO, 0x00) & 0x000003FFUL;
    if (frameLens <= lens) {
      DWT_ReadRxData(packet, frameLens, 0x00);
      return HAL_OK;
    }
    else {
      return HAL_ERROR;
    }
  }
  else {
    DWT_SetSysStatus(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    DWT_RxReset();

    return HAL_TIMEOUT;
  }
}

/**
 *  @brief  UWB_Ranging
 */
#define RANGING_LENS  8
uint32_t UWB_Ranging( const uint8_t tagAddress, uint8_t *anchorAddress, float32_t *distance )
{
  uint32_t status;

  uint8_t buffer[RANGING_LENS + 6] = {0};   // PACKET + TIME (4) + CHECK SUM (2)
  uint8_t address[2] = {tagAddress, *anchorAddress};

  uint8_t type;
  uint16_t lens;

  int32_t t_init, t_resp;

//  *distance = 0;

  kSerial_Pack(buffer, address, NULL, 0, KS_NTYPE);
  UWB_SendPacket(buffer, RANGING_LENS + 2, UWB_RANGING | UWB_IMMEDIATE | UWB_TX_RESPONSE);

  status = UWB_RecvPacket(buffer, RANGING_LENS + 6, UWB_RX_CONTINUE);
  if (status == KS_OK) {
    status = kSerial_Unpack(buffer, address, NULL, &lens, &type);
    if (status == KS_OK) {
      if ((address[1] == tagAddress) || (address[1] == 0)) {
        if (*anchorAddress == 0) {
          *anchorAddress = address[0];
        }
        else if (address[0] != *anchorAddress) {
          return KS_ERROR;
        }

        t_init = UWB_GetInitiatorTime();
        t_resp = UWB_GetReplyTime(&buffer[RANGING_LENS]);

        *distance = UWB_GetDistance(t_init, t_resp);
        return KS_OK;
      }
    }
  }

  return KS_ERROR;
}

/**
 *  @brief  UWB_RangingResp
 */
uint32_t UWB_RangingResp( uint8_t *tagAddress, uint8_t *anchorAddress )
{
  uint32_t status;

  uint8_t buffer[RANGING_LENS + 6] = {0};   // PACKET + TIME (4) + CHECK SUM (2)
  uint8_t address[2] = {*tagAddress, *anchorAddress};

  uint8_t type;
  uint16_t lens;

  int32_t t_resp;
  uint32_t tx_time;

  status = UWB_RecvPacket(buffer, 1024, DWT_RX_IMMEDIATE);
  if (status == KS_OK) {
    status = kSerial_Unpack(buffer, address, NULL, &lens, &type);
    if (status == KS_OK) {
      if ((address[1] == *anchorAddress) || (address[1] == 0)) {
        *tagAddress = address[0];

        t_resp = UWB_GetResponderTime(&tx_time);
        UWB_SetDelayedTxRxTime(tx_time);

        address[0] = *anchorAddress;
        if (address[1] == 0) {
          *anchorAddress = address[1];
        }
        address[1] = *tagAddress;
        kSerial_Pack(buffer, address, NULL, 0, KS_NTYPE);
        UWB_SetReplyTime(&buffer[RANGING_LENS], t_resp);
        UWB_SendPacket(buffer, RANGING_LENS + 6, UWB_RANGING | UWB_DELAYED);
        return KS_OK;
      }
      *anchorAddress = address[1];
    }
  }

  return KS_ERROR;
}

/*************************************** END OF FILE ****************************************/
