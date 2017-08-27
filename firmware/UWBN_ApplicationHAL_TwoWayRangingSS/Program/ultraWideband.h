/**
 *      __            ____
 *     / /__ _  __   / __/                      __  
 *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
 *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
 *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
 *                    /_/   github.com/KitSprout    
 * 
 *  @file    ultraWideband.h
 *  @author  KitSprout
 *  @date    27-Aug-2017
 *  @brief   
 * 
 */

/* Define to prevent recursive inclusion ---------------------------------------------------*/
#ifndef __ULTRAWIDEBAND_H
#define __ULTRAWIDEBAND_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes --------------------------------------------------------------------------------*/
#include "modules\dw1000.h"

/* Exported types --------------------------------------------------------------------------*/
enum {
  /* tx rx mode */
  UWB_IMMEDIATE       = 0x00,
  UWB_DELAYED         = 0x01,
  UWB_RANGING         = 0x80,

  /* tx mode */
  UWB_TX_RESPONSE     = 0x02,

  /* rx mode */
  UWB_RX_ON_DLY_ERR   = 0x02,
  UWB_RX_NO_SYNC_PTRS = 0x04,
  UWB_RX_CONTINUE     = 0x40,
};

/* Exported constants ----------------------------------------------------------------------*/

/* Speed of light in air, in metres per second. */
#define UWB_SPEED_OF_LIGHT              299702547

/* Default antenna delay values for 64 MHz PRF. */
#define UWB_TX_ANT_DLY                  16436
#define UWB_RX_ANT_DLY                  16436

/* Delay between frames, in UWB microseconds. */
#define UWB_POLL_TX_TO_RESP_RX_DLY_UUS  140

/* Receive response timeout. */
#define UWB_RESP_RX_TIMEOUT_UUS         210

/* Delay between frames, in UWB microseconds. */
#define UWB_POLL_RX_TO_RESP_TX_DLY_UUS  330

/* Exported functions ----------------------------------------------------------------------*/
void      UWB_Init( DWT_ConfigTypeDef *pConfig );
int32_t   UWB_GetReplyTime( uint8_t *packet );
void      UWB_SetReplyTime( uint8_t *packet, const uint32_t time );
int32_t   UWB_GetInitiatorTime( void );
int32_t   UWB_GetResponderTime( uint32_t *tx_time );
void      UWB_SetDelayedTxRxTime( uint32_t time );
void      UWB_SetRxAfterTxDelay( uint32_t time );
void      UWB_SetRxTimeout( uint32_t time );
float64_t UWB_GetDistance( int32_t t_init, int32_t t_resp );
uint32_t  UWB_SendPacket( uint8_t *packet, uint16_t lens, uint8_t mode );
uint32_t  UWB_RecvPacket( uint8_t *packet, uint16_t lens, uint8_t mode );
uint32_t  UWB_Ranging( const uint8_t tagAddress, uint8_t *anchorAddress, float32_t *distance );
uint32_t  UWB_RangingResp( uint8_t *tagAddress, uint8_t *anchorAddress );

#ifdef __cplusplus
}
#endif

#endif

/*************************************** END OF FILE ****************************************/
