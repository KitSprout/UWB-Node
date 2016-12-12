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
  * @date    27-Nov-2016
  * @brief   
  * 
  */

/* Includes --------------------------------------------------------------------------------*/
#include "drivers\stm32f4_system.h"
#include "modules\serial.h"
#include "modules\dw1000.h"
#include "stm32f4xx_bsp.h"

/** @addtogroup STM32_Program
  * @{
  */

/* Private typedef -------------------------------------------------------------------------*/
/* Private define --------------------------------------------------------------------------*/
#define ANCHOR_ADDR               1
#define MAX_ANCHOR                4

/* Default antenna delay values for 64 MHz PRF. */
#define TX_ANT_DLY                16436
#define RX_ANT_DLY                16436

/* Length of the common part of the message. */
#define ALL_MSG_COMMON_LEN        20

/* Indexes to access some of the fields in the frames defined above. */
#define RESP_MSG_TS_LEN           4
#define RESP_MSG_POLL_RX_TS_IDX   ALL_MSG_COMMON_LEN
#define RESP_MSG_RESP_TX_TS_IDX   (ALL_MSG_COMMON_LEN + RESP_MSG_TS_LEN)

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 ms and 1 ms = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME           65536

#define INITIATOR_TX_BUF_LEN      (ALL_MSG_COMMON_LEN + 2)    // ALL_MSG_COMMON_LEN + 2
#define INITIATOR_RX_BUF_LEN      (ALL_MSG_COMMON_LEN + 10)   // ALL_MSG_COMMON_LEN + 10

/* Private macro ---------------------------------------------------------------------------*/
/* Private variables -----------------------------------------------------------------------*/

static DWT_ConfigTypeDef dwtConfig = {
  .LoadCode          = DW_LOAD_UCODE,
  .Channel           = DW_CHANNEL_2,
  .PulseRepFreq      = DW_PRF_64M,
  .TxPreambLen       = DW_TX_PLEN_128,
  .PreambleAcqChunk  = DW_PAC_8,
  .TxCode            = 9,
  .RxCode            = 9,
  .NonStandardSFD    = DISABLE,
  .DataRate          = DW_DATARATE_6M8,
  .PhrMode           = DW_PHR_MODE_STD,
  .SFDTimeout        = (128 + 1 + 8 - 8) // TxPreambLen + 1 + SFD length - PAC
};

static uint32_t status;

/* Private function prototypes -------------------------------------------------------------*/
void DEMO_SSTWR_INITIATOR( void );
void DEMO_SSTWR_RESPONDER( void );

/* Private functions -----------------------------------------------------------------------*/

int main( void )
{
  HAL_Init();

  BSP_GPIO_Config();
  BSP_UART_Config(NULL, NULL);
  BSP_UWB_Config();

  DWT_SetSpeedLow();
  DWT_Initialise(&dwtConfig);
  DWT_SetSpeedHigh();

  /* Configure DW1000.
   * >> NOTE:
   * In a real application, for optimum performance within regulatory limits, it may be
   * necessary to set TX pulse bandwidth and TX power, (using the dwt_configuretxrf API
   * call) to per device calibrated values saved in the target system or the DW1000
   * OTP memory. */
  DWT_Configure(&dwtConfig);

  /* Apply default antenna delay value.
   * >> NOTE:
   * The sum of the values is the TX to RX antenna delay, experimentally determined by a
   * calibration process. Here we use a hard coded typical value but, in a real application,
   * each device should have its own antenna delay properly calibrated to get the best
   * possible precision when performing range measurements. */
  DWT_SetRxAntennaDelay(RX_ANT_DLY);
  DWT_SetTxAntennaDelay(TX_ANT_DLY);

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

int8_t checkPacket( uint8_t *pMsg1, uint8_t *pMsg2 )
{
  for (uint8_t i = 0; i < 8; i++) {
    if (pMsg1[i] != pMsg2[i]) {
      return -1;
    }
  }
  return 0;
}

/* ---------------------------------------------------------------------------------------- *
 *                                                                                          *
 *                          Single-Sided Two-Way Ranging Initiator                          *
 *                                                                                          *
 * ---------------------------------------------------------------------------------------- */

/* Delay between frames, in UWB microseconds.
 * >> NOTE:
 * The single-sided two-way ranging scheme implemented here has to be considered carefully as
 * the accuracy of the distance measured is highly sensitive to the clock offset error between
 * the devices and the length of the response delay between frames. To achieve the best
 * possible accuracy, this response delay must be kept as low as possible. In order to do so,
 * 6.8 Mbps data rate is used in this example and the response delay between frames is defined
 * as low as possible. The user is referred to User Manual for more details about the
 * single-sided two-way ranging process. */
#define POLL_TX_TO_RESP_RX_DLY_UUS  140

/* Receive response timeout.
 * >> NOTE:
 * This timeout is for complete reception of a frame, i.e. timeout duration must take into
 * account the length of the expected frame. Here the value is arbitrary but chosen large
 * enough to make sure that there is enough time to receive the complete response frame sent
 * by the responder at the 6.8M data rate used (around 200 ms).*/
#define RESP_RX_TIMEOUT_UUS         210

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT              299702547

static void resp_msg_get_ts( uint8_t *ts_field, uint32_t *ts )
{
  *ts = 0;
  for (uint32_t i = 0; i < RESP_MSG_TS_LEN; i++) {
    *ts += ts_field[i] << (i * 8);
  }
}

void DEMO_SSTWR_INITIATOR( void )
{
  /* Frames used in the ranging process. */
  static uint8_t tx_poll_msg[INITIATOR_TX_BUF_LEN] = {0};
  static uint8_t rx_resp_msg[INITIATOR_RX_BUF_LEN] = {0};

  static uint8_t rx_buffer[INITIATOR_RX_BUF_LEN]   = {0};

  static float64_t tof[4]        = {0};
  static float64_t distance[4]   = {0};
  static float32_t distance32[4] = {0};

  uint32_t frameLen;

  tx_poll_msg[0] = 'K';
  tx_poll_msg[1] = 'S';
  tx_poll_msg[2] = 'D';
  tx_poll_msg[3] = 'W';
  tx_poll_msg[4] = 0; // source address
  tx_poll_msg[5] = 0; // source address
  tx_poll_msg[6] = ANCHOR_ADDR;
  tx_poll_msg[7] = 0;

  rx_resp_msg[0] = 'K';
  rx_resp_msg[1] = 'S';
  rx_resp_msg[2] = 'D';
  rx_resp_msg[3] = 'W';
  rx_resp_msg[4] = ANCHOR_ADDR;
  rx_resp_msg[5] = 0;
  rx_resp_msg[6] = 0;
  rx_resp_msg[7] = 0;


  /* Set expected response's delay and timeout.
   * As this example only handles one incoming frame with always the same delay and timeout,
   * those values can be set here once for all. */
  DWT_SetRxAfterTxDelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  DWT_SetRxTimeout(RESP_RX_TIMEOUT_UUS);

  while (1) {

    for (uint8_t anchor = 0; anchor < MAX_ANCHOR; anchor++) {

      /* Write frame data to DW1000 and prepare transmission.
       * >> NOTE:
       * DWT_WriteTxData() takes the full size of the message as a parameter but only
       * copies (size - 2) bytes as the check-sum at the end of the frame is automatically
       * appended by the DW1000. This means that our variable could be two bytes shorter
       * without losing any data (but the sizeof would not work anymore then as we would
       * still have to indicate the full length of the frame to DWT_WriteTxData()). */
      rx_resp_msg[4] = ANCHOR_ADDR + anchor;
      tx_poll_msg[6] = ANCHOR_ADDR + anchor;
      DWT_SetSysStatus(SYS_STATUS_TXFRS);
      DWT_WriteTxData(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
      DWT_WriteTxFCtrl(sizeof(tx_poll_msg), 0, 0);          /* Zero offset in TX buffer, ranging. */

      /* Start transmission, indicating that a response is expected so that reception
       * is enabled automatically after the frame is sent and the delay set by
       * DWT_SetRxAfterTxDelay() has elapsed. */
      DWT_StartTx(DWT_TX_IMMEDIATE | DWT_RESPONSE);

      /* We assume that the transmission is achieved correctly,
       * poll for reception of a frame or error/timeout.
       * >> NOTE:
       * We use polled mode of operation here to keep the example as simple as possible but
       * all status events can be used to generate interrupts. Please refer to DW1000 User
       * Manual for more details on "interrupts". It is also to be noted that STATUS register
       * is 5 bytes long but, as the event we use are all in the first bytes of the register,
       * we can use the simple dwt_read32bitreg() API call to access it instead of reading
       * the whole 5 bytes. */
      do {
        status = DWT_GetSysStatus();
      } while (!(status & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)));

      if (status & SYS_STATUS_RXFCG) {
        /* Clear good RX frame event in the DW1000 status register. */
        DWT_SetSysStatus(SYS_STATUS_RXFCG);

        /* A frame has been received, read it into the local buffer. */
        frameLen = DWT_ReadData32(DW1000_RX_FINFO, 0x00) & 0x0000007FUL;
        if (frameLen <= INITIATOR_RX_BUF_LEN) {
          DWT_ReadRxData(rx_buffer, frameLen, 0x00);
        }
        if (checkPacket(rx_buffer, rx_resp_msg) == 0) {
          uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
          int32_t rtd_init, rtd_resp;

          /* Retrieve poll transmission and response reception timestamps.
           * >> NOTE:
           * The high order byte of each 40-bit time-stamps is discarded here. This is acceptable
           * as, on each device, those time-stamps are not separated by more than 2^32 device
           * time units (which is around 67 ms) which means that the calculation of the round-trip
           * delays can be handled by a 32-bit subtraction. */
          poll_tx_ts = DWT_ReadTxTimestampL32();
          resp_rx_ts = DWT_ReadRxTimestampL32();

          /* Get timestamps embedded in response message. */
          resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
          resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

          /* Compute time of flight and distance. */
          rtd_init = resp_rx_ts - poll_tx_ts;
          rtd_resp = resp_tx_ts - poll_rx_ts;

          tof[anchor] = ((rtd_init - rtd_resp) / 2.0) * DWT_TIME_UNITS;
          distance[anchor] = tof[anchor] * SPEED_OF_LIGHT;

          LED_R_Set();
          LED_G_Toggle();
          LED_B_Set();
        }
        else {
          LED_R_Set();
          LED_G_Set();
          LED_B_Toggle();
        }
        delay_ms(1);
      }
      else {
        /* Clear RX error/timeout events in the DW1000 status register. */
        DWT_SetSysStatus(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

        /* Reset RX to properly reinitialise LDE operation. */
        DWT_RxReset();

        LED_R_Reset();
        LED_G_Set();
        LED_B_Set();
      }
    }

    for (uint8_t i = 0; i < MAX_ANCHOR; i++) {
      distance32[i] = distance[i];
      printf(" [%2i] %10.7f m\r\n", ANCHOR_ADDR + i, distance32[i]);
    }
    printf("\r\n");
//    float32_t sendData[SERIAL_DATA_LENS] = {0};
//    sendData[0] = distance[0];
//    sendData[1] = distance[1];
//    sendData[2] = distance[2];
//    sendData[3] = distance[3];
//    sendData[4] = 0;
//    sendData[5] = 0;
//    kSerial_sendData(sendData, 1, KS_FLOAT32);
//    LED_B_Toggle();
    delay_ms(100);
  }
}




/* ---------------------------------------------------------------------------------------- *
 *                                                                                          *
 *                          Single-Sided Two-Way Ranging Responder                          *
 *                                                                                          *
 * ---------------------------------------------------------------------------------------- */

/* Delay between frames, in UWB microseconds. */
#define POLL_RX_TO_RESP_TX_DLY_UUS  330

static uint64_t get_rx_timestamp_u64( void )
{
  uint8_t ts_tab[5];
  uint64_t ts = 0;

  DWT_ReadRxTimestamp(ts_tab);
  for (int32_t i = 4; i >= 0; i--) {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
}

static void resp_msg_set_ts( uint8_t *ts_field, const uint64_t ts )
{
  for (uint32_t i = 0; i < RESP_MSG_TS_LEN; i++) {
    ts_field[i] = (ts >> (i * 8)) & 0xFF;
  }
}

void DEMO_SSTWR_RESPONDER( void )
{
  /* Frames used in the ranging process. */
  static uint8_t rx_poll_msg[INITIATOR_TX_BUF_LEN] = {0};
  static uint8_t tx_resp_msg[INITIATOR_RX_BUF_LEN] = {0};

  static uint8_t rx_rasp_buffer[INITIATOR_TX_BUF_LEN] = {0};

  uint32_t frameLen;

  rx_poll_msg[0] = 'K';
  rx_poll_msg[1] = 'S';
  rx_poll_msg[2] = 'D';
  rx_poll_msg[3] = 'W';
  rx_poll_msg[4] = 0;   // source address
  rx_poll_msg[5] = 0;   // source address
  rx_poll_msg[6] = ANCHOR_ADDR;
  rx_poll_msg[7] = 0;

  tx_resp_msg[0] = 'K';
  tx_resp_msg[1] = 'S';
  tx_resp_msg[2] = 'D';
  tx_resp_msg[3] = 'W';
  tx_resp_msg[4] = ANCHOR_ADDR;
  tx_resp_msg[5] = 0;
  tx_resp_msg[6] = 0;
  tx_resp_msg[7] = 0;

  while (1) {

    /* Activate reception immediately. */
    DWT_RxEnable(DWT_RX_IMMEDIATE);

    /* Poll for reception of a frame or error/timeout. */
    do {
      status = DWT_GetSysStatus();
    } while (!(status & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)));

    if (status & SYS_STATUS_RXFCG) {

      /* Clear good RX frame event in the DW1000 status register. */
      DWT_SetSysStatus(SYS_STATUS_RXFCG);

      /* A frame has been received, read it into the local buffer. */
      frameLen = DWT_ReadData32(DW1000_RX_FINFO, 0x00) & 0x000003FF;
      if (frameLen <= 1024) {
        DWT_ReadRxData(rx_rasp_buffer, frameLen, 0);
      }

      if (checkPacket(rx_rasp_buffer, rx_poll_msg) == 0) {
        uint32_t resp_tx_time;
        uint64_t poll_rx_ts, resp_tx_ts;

        /* Retrieve poll reception timestamp. */
        poll_rx_ts = get_rx_timestamp_u64();

        /* Compute final message transmission time.
         * >> NOTE:
         * As we want to send final TX timestamp in the final message, we have to compute it
         * in advance instead of relying on the reading of DW1000 register. Timestamps and
         * delayed transmission time are both expressed in device time units so we just have
         * to add the desired response delay to response RX timestamp to get final
         * transmission time. The delayed transmission time resolution is 512 device time
         * units which means that the lower 9 bits of the obtained value must be zeroed.
         * This also allows to encode the 40-bit value in a 32-bit words by shifting the
         * all-zero lower 8 bits. */
        resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
        DWT_SetDelayedTxRxTime(resp_tx_time);

         /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
        resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY;

        /* Write all timestamps in the final message. */
        resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
        resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

        /* Write and send the response message. */
        DWT_WriteTxData(sizeof(tx_resp_msg), tx_resp_msg, 0);
        DWT_WriteTxFCtrl(sizeof(tx_resp_msg), 0, 0);
        status = DWT_StartTx(DWT_TX_DELAYED);

        /* If DWT_StartTx() returns an error, abandon this ranging exchange and proceed to the next one. */
        if (status == HAL_OK) {
          /* Poll DW1000 until TX frame sent event set. See NOTE 6 below. */
          while (!(DWT_GetSysStatus() & SYS_STATUS_TXFRS));

          /* Clear TXFRS event. */
          DWT_SetSysStatus(SYS_STATUS_TXFRS);
        }

        LED_R_Set();
        LED_G_Set();
        LED_B_Toggle();
      }
    }
    else {
      /* Clear RX error events in the DW1000 status register. */
      DWT_SetSysStatus(SYS_STATUS_ALL_RX_ERR);

      /* Reset RX to properly reinitialise LDE operation. */
      DWT_RxReset();

      LED_R_Reset();
      LED_G_Set();
      LED_B_Set();
    }
  }
}

/*************************************** END OF FILE ****************************************/
