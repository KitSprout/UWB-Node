/**
  *      __            ____
  *     / /__ _  __   / __/                      __  
  *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
  *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
  *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
  *                    /_/   github.com/KitSprout    
  * 
  * @file    dw1000.h
  * @author  KitSprout
  * @date    27-Nov-2016
  * @brief   
  * 
  */

/* Define to prevent recursive inclusion ---------------------------------------------------*/
#ifndef __DW1000_H
#define __DW1000_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes --------------------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported types --------------------------------------------------------------------------*/

typedef struct {
  SPI_HandleTypeDef *handle;
  uint16_t txBufLens;
  uint16_t rxBufLens;
  uint8_t *pTxBuf;
  uint8_t *pRxBuf;
} __attribute__((aligned)) DwHandle_st;

typedef enum {
  DW_LOAD_NONE  = 0x00,
  DW_LOAD_UCODE = 0x01
} DWT_LoadCode_TypeDef;

typedef enum {
  DW_CHANNEL_1 = 0x01,
  DW_CHANNEL_2 = 0x02,
  DW_CHANNEL_3 = 0x03,
  DW_CHANNEL_4 = 0x04,
  DW_CHANNEL_5 = 0x05,
  DW_CHANNEL_7 = 0x07
} DWT_Channel_TypeDef;

typedef enum {
  DW_PRF_16M = 0x01,    // UWB PRF 16 MHz
  DW_PRF_64M = 0x02,    // UWB PRF 64 MHz
} DWT_PulseRepFreq_TypeDef;

typedef enum {
  DW_TX_PLEN_64   = 0x04,   // Standard preamble length 64 symbols
  DW_TX_PLEN_128  = 0x14,   // Non-standard preamble length 128 symbols
  DW_TX_PLEN_256  = 0x24,   // Non-standard preamble length 256 symbols
  DW_TX_PLEN_512  = 0x34,   // Non-standard preamble length 512 symbols
  DW_TX_PLEN_1024 = 0x08,   // Standard preamble length 1024 symbols
  DW_TX_PLEN_1536 = 0x18,   // Non-standard preamble length 1536 symbols
  DW_TX_PLEN_2048 = 0x28,   // Non-standard preamble length 2048 symbols
  DW_TX_PLEN_4096 = 0x0C,   // Standard preamble length 4096 symbols
} DWT_TxPreambLen_TypeDef;

typedef enum {
  DW_PAC_8  = 0x00,   // PAC  8 (recommended for RX of preamble length  128 and below
  DW_PAC_16 = 0x01,   // PAC 16 (recommended for RX of preamble length  256
  DW_PAC_32 = 0x02,   // PAC 32 (recommended for RX of preamble length  512
  DW_PAC_64 = 0x03,   // PAC 64 (recommended for RX of preamble length 1024 and up
} DWT_PreambleAcqChunk_TypeDef;

typedef enum {
  DW_DATARATE_110K = 0x00,    // UWB bit rate 110 kbits/s
  DW_DATARATE_850K = 0x01,    // UWB bit rate 850 kbits/s
  DW_DATARATE_6M8  = 0x02     // UWB bit rate 6.8 Mbits/s
} DWT_DataRate_TypeDef;

typedef enum {
  DW_PHR_MODE_STD = 0x00,    // standard frame mode
  DW_PHR_MODE_EXT = 0x03,    // long frames mode
} DWT_PhrMode_TypeDef;

typedef struct {
  uint8_t                         TxCode;           // TX preamble code. Used in TX only.
  uint8_t                         RxCode;           // RX preamble code. Used in RX only.
  uint8_t                         NonStandardSFD;   // ENABLE: non-standard SFD, DISABLE: to use standard SFD
  uint16_t                        SFDTimeout;       // SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only.

  DWT_LoadCode_TypeDef            LoadCode;
  DWT_Channel_TypeDef             Channel;          // Channel number 1, 2, 3, 4, 5, 7
  DWT_PulseRepFreq_TypeDef        PulseRepFreq;     // pulse repetition frequency
  DWT_TxPreambLen_TypeDef         TxPreambLen;      // TX preamble length
  DWT_PreambleAcqChunk_TypeDef    PreambleAcqChunk; // TX preamble length
  DWT_DataRate_TypeDef            DataRate;         // data rate
  DWT_PhrMode_TypeDef             PhrMode;          // PHR mode
} DWT_ConfigTypeDef;

typedef struct {
  uint8_t PGdelay;
  // TX POWER
  // [31:24] BOOST_0.125ms_PWR
  // [23:16] BOOST_0.25ms_PWR-TX_SHR_PWR
  // [15:08] BOOST_0.5ms_PWR-TX_PHR_PWR
  // [07:00] DEFAULT_PWR-TX_DATA_PWR
  uint32_t power;
} dwt_txconfig_t;

typedef struct {
  uint16_t maxNoise;        // LDE max value of noise
  uint16_t firstPathAmp1;   // Amplitude at floor(index FP) + 1
  uint16_t stdNoise;        // Standard deviation of noise
  uint16_t firstPathAmp2;   // Amplitude at floor(index FP) + 2
  uint16_t firstPathAmp3;   // Amplitude at floor(index FP) + 3
  uint16_t maxGrowthCIR;    // Channel Impulse Response max growth CIR
  uint16_t rxPreamCount;    // Count of preamble symbols accumulated
  uint16_t firstPath;       // First path index (10.6 bits fixed point integer)
} dwt_rxdiag_t;

// TX/RX call-back data
typedef struct {
  uint32_t status;      // initial value of register as ISR is entered
  uint16_t datalength;  // length of frame
  uint8_t  fctrl[2];    // frame control bytes
  uint8_t  rx_flags;    // RX frame flags, see above
} dwt_cb_data_t;

typedef void (*dwt_cb_t)(const dwt_cb_data_t *);

typedef struct {
  // all of the below are mapped to a 12-bit register in DW1000
  uint16_t PHE;     // number of received header errors
  uint16_t RSL;     // number of received frame sync loss events
  uint16_t CRCG;    // number of good CRC received frames
  uint16_t CRCB;    // number of bad CRC (CRC error) received frames
  uint16_t ARFE;    // number of address filter errors
  uint16_t OVER;    // number of receiver overflows (used in double buffer mode)
  uint16_t SFDTO;   // SFD timeouts
  uint16_t PTO;     // Preamble timeouts
  uint16_t RTO;     // RX frame wait timeouts
  uint16_t TXF;     // number of transmitted frames
  uint16_t HPW;     // half period warn
  uint16_t TXW;     // power up warn
} dwt_deviceentcnts_t;

/* Exported constants ----------------------------------------------------------------------*/

/* device ID */
#define DW1000_DEVICE_ID        ((uint32_t)0xDECA0130)

/* dw1000 reg */
#define DW1000_DEV_ID           ((uint8_t)0x00)     // Lens = 4,    RO,  Device Identifier – includes device type and revision info
#define DW1000_EUI              ((uint8_t)0x01)     // Lens = 8,    RW,  Extended Unique Identifier
#define DW1000_PANADR           ((uint8_t)0x03)     // Lens = 4,    RW,  PAN Identifier and Short Address
#define DW1000_SYS_CFG          ((uint8_t)0x04)     // Lens = 4,    RW,  System Configuration bitmap
#define DW1000_SYS_TIME         ((uint8_t)0x06)     // Lens = 5,    RO,  System Time Counter (40-bit)
#define DW1000_TX_FCTRL         ((uint8_t)0x08)     // Lens = 5,    RW,  Transmit Frame Control
#define DW1000_TX_BUFFER        ((uint8_t)0x09)     // Lens = 1024, WO,  Transmit Data Buffer
#define DW1000_DX_TIME          ((uint8_t)0x0A)     // Lens = 5,    RW,  Delayed Send or Receive Time (40-bit)
#define DW1000_RX_FWTO          ((uint8_t)0x0C)     // Lens = 2,    RW,  Receive Frame Wait Timeout Period
#define DW1000_SYS_CTRL         ((uint8_t)0x0D)     // Lens = 4,    SRW, System Control Register
#define DW1000_SYS_MASK         ((uint8_t)0x0E)     // Lens = 4,    RW,  System Event Mask Register
#define DW1000_SYS_STATUS       ((uint8_t)0x0F)     // Lens = 5,    SRW, System Event Status Register
#define DW1000_RX_FINFO         ((uint8_t)0x10)     // Lens = 4,    ROD, RX Frame Information (in double buffer set)
#define DW1000_RX_BUFFER        ((uint8_t)0x11)     // Lens = 1024, ROD, Receive Data (in double buffer set)
#define DW1000_RX_FQUAL         ((uint8_t)0x12)     // Lens = 8,    ROD, Rx Frame Quality information (in double buffer set)
#define DW1000_RX_TTCKI         ((uint8_t)0x13)     // Lens = 4,    ROD, Receiver Time Tracking Interval (in double buffer set)
#define DW1000_RX_TTCKO         ((uint8_t)0x14)     // Lens = 5,    ROD, Receiver Time Tracking Offset (in double buffer set)
#define DW1000_RX_TIME          ((uint8_t)0x15)     // Lens = 14,   ROD, Receive Message Time of Arrival (in double buffer set)
#define DW1000_TX_TIME          ((uint8_t)0x17)     // Lens = 10,   RO,  Transmit Message Time of Sending
#define DW1000_TX_ANTD          ((uint8_t)0x18)     // Lens = 2,    RW,  16-bit Delay from Transmit to Antenna
#define DW1000_SYS_STATE        ((uint8_t)0x19)     // Lens = 5,    RO,  System State information
#define DW1000_ACK_RESP_T       ((uint8_t)0x1A)     // Lens = 4,    RW,  Acknowledgement Time and Response Time
#define DW1000_RX_SNIFF         ((uint8_t)0x1D)     // Lens = 4,    RW,  Pulsed Preamble Reception Configuration
#define DW1000_TX_POWER         ((uint8_t)0x1E)     // Lens = 4,    RW,  TX Power Control
#define DW1000_CHAN_CTRL        ((uint8_t)0x1F)     // Lens = 4,    RW,  Channel Control
#define DW1000_USR_SFD          ((uint8_t)0x21)     // Lens = 41,   RW,  User-specified short/long TX/RX SFD sequences

#define DW1000_AGC_CTRL         ((uint8_t)0x23)     // Lens = 32,   RW,  Automatic Gain Control configuration
#define DW1000_SUB_AGC_CTRL1    ((uint8_t)0x02)     // Lens = 2,    RW,  AGC Control #1
#define DW1000_SUB_AGC_TUNE1    ((uint8_t)0x04)     // Lens = 2,    RW,  AGC Tuning register 1
#define DW1000_SUB_AGC_TUNE2    ((uint8_t)0x0C)     // Lens = 4,    RW,  AGC Tuning register 2
#define DW1000_SUB_AGC_TUNE3    ((uint8_t)0x12)     // Lens = 2,    RW,  AGC Tuning register 3
#define DW1000_SUB_AGC_STAT1    ((uint8_t)0x1E)     // Lens = 3,    RO,  AGC Status

#define DW1000_EXT_SYNC         ((uint8_t)0x24)     // Lens = 12,   RW,  External synchronisation control
#define DW1000_SUB_EC_CTRL      ((uint8_t)0x00)     // Lens = 4,    RW,  External clock synchronisation counter configuration
#define DW1000_SUB_EC_RXTC      ((uint8_t)0x04)     // Lens = 4,    RO,  External clock counter captured on RMARKER
#define DW1000_SUB_EC_GOLP      ((uint8_t)0x08)     // Lens = 4,    RO,  External clock offset to first path 1 GHz counter

#define DW1000_ACC_MEM          ((uint8_t)0x25)     // Lens = 4064, RO,  Read access to accumulator data

#define DW1000_GPIO_CTRL        ((uint8_t)0x26)     // Lens = 44,   RW,  Peripheral register bus 1 access - GPIO control
#define DW1000_SUB_GPIO_MODE    ((uint8_t)0x00)     // Lens = 4,    RW,  GPIO Mode Control Register
#define DW1000_SUB_GPIO_DIR     ((uint8_t)0x08)     // Lens = 4,    RW,  GPIO Direction Control Register
#define DW1000_SUB_GPIO_DOUT    ((uint8_t)0x0C)     // Lens = 4,    RW,  GPIO Data Output register
#define DW1000_SUB_GPIO_IRQE    ((uint8_t)0x10)     // Lens = 4,    RW,  GPIO Interrupt Enable
#define DW1000_SUB_GPIO_ISEN    ((uint8_t)0x14)     // Lens = 4,    RW,  GPIO Interrupt Sense Selection
#define DW1000_SUB_GPIO_IMODE   ((uint8_t)0x18)     // Lens = 4,    RW,  GPIO Interrupt Mode (Level / Edge)
#define DW1000_SUB_GPIO_IBES    ((uint8_t)0x1C)     // Lens = 4,    RW,  GPIO Interrupt “Both Edge” Select
#define DW1000_SUB_GPIO_ICLR    ((uint8_t)0x20)     // Lens = 4,    RW,  GPIO Interrupt Latch Clear
#define DW1000_SUB_GPIO_IDBE    ((uint8_t)0x24)     // Lens = 4,    RW,  GPIO Interrupt De-bounce Enable
#define DW1000_SUB_GPIO_RAW     ((uint8_t)0x28)     // Lens = 4,    RO,  GPIO raw state

#define DW1000_DRX_CONF         ((uint8_t)0x27)     // Lens = 44,   RW,  Digital Receiver configuration
#define DW1000_SUB_DRX_TUNE0b   ((uint8_t)0x02)     // Lens = 2,    RW,  Digital Tuning Register 0b
#define DW1000_SUB_DRX_TUNE1a   ((uint8_t)0x04)     // Lens = 2,    RW,  Digital Tuning Register 1a
#define DW1000_SUB_DRX_TUNE1b   ((uint8_t)0x06)     // Lens = 2,    RW,  Digital Tuning Register 1b
#define DW1000_SUB_DRX_TUNE2    ((uint8_t)0x08)     // Lens = 4,    RW,  Digital Tuning Register 2
#define DW1000_SUB_DRX_SFDTOC   ((uint8_t)0x20)     // Lens = 2,    RW,  SFD timeout
#define DW1000_SUB_DRX_PRETOC   ((uint8_t)0x24)     // Lens = 2,    RW,  Preamble detection timeout
#define DW1000_SUB_DRX_TUNE4H   ((uint8_t)0x26)     // Lens = 2,    RW,  Digital Tuning Register 4H
#define DW1000_SUB_RXPACC_NOSAT ((uint8_t)0x2C)     // Lens = 2,    RO,  Unsaturated accumulated preamble symbols

#define DW1000_RF_CONF          ((uint8_t)0x28)     // Lens = 58,   RW,  Analog RF Configuration
#define DW1000_SUB_RF_CONF      ((uint8_t)0x00)     // Lens = 4,    RW,  RF Configuration Register
#define DW1000_SUB_RF_RXCTRLH   ((uint8_t)0x0B)     // Lens = 1,    RW,  Analog RX Control Register
#define DW1000_SUB_RF_TXCTRL    ((uint8_t)0x0C)     // Lens = 4,    RW,  Analog TX Control Register
#define DW1000_SUB_RF_STATUS    ((uint8_t)0x2C)     // Lens = 4,    RO,  RF Status Register
#define DW1000_SUB_LDOTUNE      ((uint8_t)0x30)     // Lens = 5,    RW,  LDO voltage tuning

#define DW1000_TX_CAL           ((uint8_t)0x2A)     // Lens = 52,   RW,  Transmitter calibration block
#define DW1000_SUB_TC_SARC      ((uint8_t)0x00)     // Lens = 2,    RW,  Transmitter Calibration – SAR control
#define DW1000_SUB_TC_SARL      ((uint8_t)0x03)     // Lens = 3,    RO,  Transmitter Calibration – Latest SAR readings
#define DW1000_SUB_TC_SARW      ((uint8_t)0x06)     // Lens = 2,    RO,  Transmitter Calibration – SAR readings at last Wake-Up
#define DW1000_SUB_TC_PGDELAY   ((uint8_t)0x0B)     // Lens = 1,    RW,  Transmitter Calibration – Pulse Generator Delay
#define DW1000_SUB_TC_PGTEST    ((uint8_t)0x0C)     // Lens = 1,    RW,  Transmitter Calibration – Pulse Generator Test

#define DW1000_FS_CTRL          ((uint8_t)0x2B)     // Lens = 21,   RW,  Frequency synthesiser control block
#define DW1000_SUB_FS_PLLCFG    ((uint8_t)0x07)     // Lens = 4,    RW,  Frequency synthesiser – PLL configuration
#define DW1000_SUB_FS_PLLTUNE   ((uint8_t)0x0B)     // Lens = 1,    RW,  Frequency synthesiser – PLL Tuning
#define DW1000_SUB_FS_XTALT     ((uint8_t)0x0E)     // Lens = 1,    RW,  Frequency synthesiser – Crystal trim

#define DW1000_AON              ((uint8_t)0x2C)     // Lens = 12,   RW,  Always-On register set
#define DW1000_SUB_AON_WCFG     ((uint8_t)0x00)     // Lens = 2,    RW,  AON Wakeup Configuration Register
#define DW1000_SUB_AON_CTRL     ((uint8_t)0x02)     // Lens = 1,    RW,  AON Control Register
#define DW1000_SUB_AON_RDAT     ((uint8_t)0x03)     // Lens = 1,    RW,  AON Direct Access Read Data Result
#define DW1000_SUB_AON_ADDR     ((uint8_t)0x04)     // Lens = 1,    RW,  AON Direct Access Address
#define DW1000_SUB_AON_CFG0     ((uint8_t)0x06)     // Lens = 4,    RW,  AON Configuration Register 0
#define DW1000_SUB_AON_CFG1     ((uint8_t)0x0A)     // Lens = 2,    RW,  AON Configuration Register 1

#define DW1000_OTP_IF           ((uint8_t)0x2D)     // Lens = 18,   RW,  One Time Programmable Memory Interface
#define DW1000_SUB_OTP_WDAT     ((uint8_t)0x00)     // Lens = 4,    RW,  OTP Write Data
#define DW1000_SUB_OTP_ADDR     ((uint8_t)0x04)     // Lens = 2,    RW,  OTP Address
#define DW1000_SUB_OTP_CTRL     ((uint8_t)0x06)     // Lens = 2,    RW,  OTP Control
#define DW1000_SUB_OTP_STAT     ((uint8_t)0x08)     // Lens = 2,    RW,  OTP Status
#define DW1000_SUB_OTP_RDAT     ((uint8_t)0x0A)     // Lens = 4,    R,   OTP Read Data
#define DW1000_SUB_OTP_SRDAT    ((uint8_t)0x0E)     // Lens = 4,    RW,  OTP SR Read Data
#define DW1000_SUB_OTP_SF       ((uint8_t)0x12)     // Lens = 1,    RW,  OTP Special Function

#define DW1000_LDE_CTRL         ((uint8_t)0x2E)     // Lens =  -    RW,  Leading edge detection control block
#define DW1000_SUB_LDE_THRESH   ((uint16_t)0x0000)  // Lens = 2,    RO,  LDE Threshold report
#define DW1000_SUB_LDE_CFG1     ((uint16_t)0x0806)  // Lens = 1,    RW,  LDE Configuration Register 1
#define DW1000_SUB_LDE_PPINDX   ((uint16_t)0x1000)  // Lens = 2,    RO,  LDE Peak Path Index
#define DW1000_SUB_LDE_PPAMPL   ((uint16_t)0x1002)  // Lens = 2,    RO,  LDE Peak Path Amplitude
#define DW1000_SUB_LDE_RXANTD   ((uint16_t)0x1804)  // Lens = 2,    RW,  LDE Receive Antenna Delay configuration
#define DW1000_SUB_LDE_CFG2     ((uint16_t)0x1806)  // Lens = 2,    RW,  LDE Configuration Register 2
#define DW1000_SUB_LDE_REPC     ((uint16_t)0x2804)  // Lens = 2,    RW,  LDE Replica Coefficient configuration

#define DW1000_DIG_DIAG         ((uint8_t)0x2F)     // Lens = 41,   RW,  Digital Diagnostics Interface
#define DW1000_SUB_EVC_CTRL     ((uint8_t)0x00)     // Lens = 4,    SRW, Event Counter Control
#define DW1000_SUB_EVC_PHE      ((uint8_t)0x04)     // Lens = 2,    RO,  PHR Error Counter
#define DW1000_SUB_EVC_RSE      ((uint8_t)0x06)     // Lens = 2,    RO,  RSD Error Counter
#define DW1000_SUB_EVC_FCG      ((uint8_t)0x08)     // Lens = 2,    RO,  Frame Check Sequence Good Counter
#define DW1000_SUB_EVC_FCE      ((uint8_t)0x0A)     // Lens = 2,    RO,  Frame Check Sequence Error Counter
#define DW1000_SUB_EVC_FFR      ((uint8_t)0x0C)     // Lens = 2,    RO,  Frame Filter Rejection Counter
#define DW1000_SUB_EVC_OVR      ((uint8_t)0x0E)     // Lens = 2,    RO,  RX Overrun Error Counter
#define DW1000_SUB_EVC_STO      ((uint8_t)0x10)     // Lens = 2,    RO,  SFD Timeout Counter
#define DW1000_SUB_EVC_PTO      ((uint8_t)0x12)     // Lens = 2,    RO,  Preamble Timeout Counter
#define DW1000_SUB_EVC_FWTO     ((uint8_t)0x14)     // Lens = 2,    RO,  RX Frame Wait Timeout Counter
#define DW1000_SUB_EVC_TXFS     ((uint8_t)0x16)     // Lens = 2,    RO,  TX Frame Sent Counter
#define DW1000_SUB_EVC_HPW      ((uint8_t)0x18)     // Lens = 2,    RO,  Half Period Warning Counter
#define DW1000_SUB_EVC_TPW      ((uint8_t)0x1A)     // Lens = 2,    RO,  Transmitter Power-Up Warning Counter
#define DW1000_SUB_DIAG_TMC     ((uint8_t)0x24)     // Lens = 2,    RW,  Test Mode Control Register

#define DW1000_PMSC             ((uint8_t)0x36)     // Lens = 48,   RW,  Power Management System Control Block
#define DW1000_SUB_PMSC_CTRL0   ((uint8_t)0x00)     // Lens = 4,    RW,  PMSC Control Register 0
#define DW1000_SUB_PMSC_CTRL1   ((uint8_t)0x04)     // Lens = 4,    RW,  PMSC Control Register 1
#define DW1000_SUB_PMSC_SNOZT   ((uint8_t)0x0C)     // Lens = 1,    RW,  PMSC Snooze Time Register
#define DW1000_SUB_PMSC_TXFSEQ  ((uint8_t)0x26)     // Lens = 2,    RW,  PMSC fine grain TX sequencing control
#define DW1000_SUB_PMSC_LEDC    ((uint8_t)0x28)     // Lens = 4,    RW,  PMSC LED Control Register

/* clock config */
#define ENABLE_ALL_SEQ          ((uint8_t)0x00)
#define FORCE_SYS_XTI           ((uint8_t)0x01)
#define FORCE_SYS_PLL           ((uint8_t)0x02)
#define FORCE_TX_PLL            ((uint8_t)0x03)
#define FORCE_LDE               ((uint8_t)0x04)
#define FORCE_OTP_ON            ((uint8_t)0x05)
#define FORCE_OTP_OFF           ((uint8_t)0x06)
#define READ_ACC_ON             ((uint8_t)0x07)
#define READ_ACC_OFF            ((uint8_t)0x08)


/* System event Status */
/*masks */
#define SYS_STATUS_MASK_32      0xFFF7FFFFUL    /* System event Status Register access mask (all unused fields should always be writen as zero) */

/*offset 0 */
#define SYS_STATUS_IRQS         0x00000001UL    /* Interrupt Request Status READ ONLY */
#define SYS_STATUS_CPLOCK       0x00000002UL    /* Clock PLL Lock */
#define SYS_STATUS_ESYNCR       0x00000004UL    /* External Sync Clock Reset */
#define SYS_STATUS_AAT          0x00000008UL    /* Automatic Acknowledge Trigger */
#define SYS_STATUS_TXFRB        0x00000010UL    /* Transmit Frame Begins */
#define SYS_STATUS_TXPRS        0x00000020UL    /* Transmit Preamble Sent */
#define SYS_STATUS_TXPHS        0x00000040UL    /* Transmit PHY Header Sent */
#define SYS_STATUS_TXFRS        0x00000080UL    /* Transmit Frame Sent: This is set when the transmitter has completed the sending of a frame */

/*offset 8 */
#define SYS_STATUS_RXPRD        0x00000100UL    /* Receiver Preamble Detected status */
#define SYS_STATUS_RXSFDD       0x00000200UL    /* Receiver Start Frame Delimiter Detected. */
#define SYS_STATUS_LDEDONE      0x00000400UL    /* LDE processing done */
#define SYS_STATUS_RXPHD        0x00000800UL    /* Receiver PHY Header Detect */
#define SYS_STATUS_RXPHE        0x00001000UL    /* Receiver PHY Header Error */
#define SYS_STATUS_RXDFR        0x00002000UL    /* Receiver Data Frame Ready */
#define SYS_STATUS_RXFCG        0x00004000UL    /* Receiver FCS Good */
#define SYS_STATUS_RXFCE        0x00008000UL    /* Receiver FCS Error */

/*offset 16 */
#define SYS_STATUS_RXRFSL       0x00010000UL    /* Receiver Reed Solomon Frame Sync Loss */
#define SYS_STATUS_RXRFTO       0x00020000UL    /* Receive Frame Wait Timeout */
#define SYS_STATUS_LDEERR       0x00040000UL    /* Leading edge detection processing error */
#define SYS_STATUS_reserved     0x00080000UL    /* bit19 reserved */
#define SYS_STATUS_RXOVRR       0x00100000UL    /* Receiver Overrun */
#define SYS_STATUS_RXPTO        0x00200000UL    /* Preamble detection timeout */
#define SYS_STATUS_GPIOIRQ      0x00400000UL    /* GPIO interrupt */
#define SYS_STATUS_SLP2INIT     0x00800000UL    /* SLEEP to INIT */

/*offset 24 */
#define SYS_STATUS_RFPLL_LL     0x01000000UL    /* RF PLL Losing Lock */
#define SYS_STATUS_CLKPLL_LL    0x02000000UL    /* Clock PLL Losing Lock */
#define SYS_STATUS_RXSFDTO      0x04000000UL    /* Receive SFD timeout */
#define SYS_STATUS_HPDWARN      0x08000000UL    /* Half Period Delay Warning */
#define SYS_STATUS_TXBERR       0x10000000UL    /* Transmit Buffer Error */
#define SYS_STATUS_AFFREJ       0x20000000UL    /* Automatic Frame Filtering rejection */
#define SYS_STATUS_HSRBP        0x40000000UL    /* Host Side Receive Buffer Pointer */
#define SYS_STATUS_ICRBP        0x80000000UL    /* IC side Receive Buffer Pointer READ ONLY */

/*offset 32 */
#define SYS_STATUS_RXRSCS       0x0100000000ULL /* Receiver Reed-Solomon Correction Status */
#define SYS_STATUS_RXPREJ       0x0200000000ULL /* Receiver Preamble Rejection */
#define SYS_STATUS_TXPUTE       0x0400000000ULL /* Transmit power up time error */

#define SYS_STATUS_TXERR        (0x0408)        /* These bits are the 16 high bits of status register TXPUTE and HPDWARN flags */

/* All RX events after a correct packet reception mask. */
#define SYS_STATUS_ALL_RX_GOOD  (SYS_STATUS_RXDFR  | SYS_STATUS_RXFCG | SYS_STATUS_RXPRD | \
                                 SYS_STATUS_RXSFDD | SYS_STATUS_RXPHD | SYS_STATUS_LDEDONE)

/* All double buffer events mask. */
#define SYS_STATUS_ALL_DBLBUFF  (SYS_STATUS_RXDFR | SYS_STATUS_RXFCG)

/* All RX errors mask. */
#define SYS_STATUS_ALL_RX_ERR   (SYS_STATUS_RXPHE   | SYS_STATUS_RXFCE  | SYS_STATUS_RXRFSL | \
                                 SYS_STATUS_RXSFDTO | SYS_STATUS_AFFREJ | SYS_STATUS_LDEERR)

/* User defined RX timeouts (frame wait timeout and preamble detect timeout) mask. */
#define SYS_STATUS_ALL_RX_TO    (SYS_STATUS_RXRFTO | SYS_STATUS_RXPTO)

/* All TX events mask. */
#define SYS_STATUS_ALL_TX       (SYS_STATUS_AAT | SYS_STATUS_TXFRB | SYS_STATUS_TXPRS | \
                                 SYS_STATUS_TXPHS | SYS_STATUS_TXFRS )

/* Crystal frequency, in hertz. */
#define DWT_XTAL_FREQ_HZ        38400000

#define DWT_TIME_UNITS          (1.0/499.2e6/128.0)   //!< = 15.65e-12 s

/* DW1000 SLEEP and WAKEUP configuration parameters */
#define DWT_PRESRV_SLP          0x0100    // PRES_SLP - on wakeup preserve sleep bit
#define DWT_LOADOPSET           0x0080    // ONW_L64P - on wakeup load operating parameter set for 64 PSR
#define DWT_CONFIG              0x0040    // ONW_LDC  - on wakeup restore (load) the saved configurations (from AON array into HIF)
#define DWT_RX_EN               0x0002    // ONW_RX   - on wakeup activate reception
#define DWT_TANDV               0x0001    // ONW_RADC - on wakeup run ADC to sample temperature and voltage sensor values

#define DWT_XTAL_EN             0x10    // keep XTAL running during sleep
#define DWT_WAKE_SLPCNT         0x08    // wake up after sleep count
#define DWT_WAKE_CS             0x04    // wake up on chip select
#define DWT_WAKE_WK             0x02    // wake up on WAKEUP PIN
#define DWT_SLP_EN              0x01    // enable sleep/deep sleep functionality

/* DW1000 interrupt events */
#define DWT_INT_TFRS            0x00000080    // frame sent
#define DWT_INT_LDED            0x00000400    // micro-code has finished execution
#define DWT_INT_RFCG            0x00004000    // frame received with good CRC
#define DWT_INT_RPHE            0x00001000    // receiver PHY header error
#define DWT_INT_RFCE            0x00008000    // receiver CRC error
#define DWT_INT_RFSL            0x00010000    // receiver sync loss error
#define DWT_INT_RFTO            0x00020000    // frame wait timeout
#define DWT_INT_RXOVRR          0x00100000    // receiver overrun
#define DWT_INT_RXPTO           0x00200000    // preamble detect timeout
#define DWT_INT_SFDT            0x04000000    // SFD timeout
#define DWT_INT_ARFE            0x20000000    // frame rejected (due to frame filtering configuration)

/* Analog RF Configuration */
#define RF_CONF_TXEN            0x00400000UL   /* TX enable */
#define RF_CONF_RXEN            0x00200000UL   /* RX enable */
#define RF_CONF_TXPOW           0x001F0000UL   /* turn on power all LDOs */
#define RF_CONF_PLLEN           0x0000E000UL   /* enable PLLs */
#define RF_CONF_TXBLOCKSEN      0x00001F00UL   /* enable TX blocks */
#define RF_CONF_TXPLLPOWEN      (RF_CONF_PLLEN | RF_CONF_TXPOW)
#define RF_CONF_TXALLEN         (RF_CONF_TXEN | RF_CONF_TXPOW | RF_CONF_PLLEN | RF_CONF_TXBLOCKSEN)

#define DWT_TX_IMMEDIATE        0
#define DWT_TX_DELAYED          1
#define DWT_RESPONSE            2

#define DWT_RX_IMMEDIATE        0
#define DWT_RX_DELAYED          1   // Set up delayed RX, if "late" error triggers, then the RX will be enabled immediately
#define DWT_IDLE_ON_DLY_ERR     2   // If delayed RX failed due to "late" error then if this
                                    // flag is set the RX will not be re-enabled immediately, and device will be in IDLE when function exits
#define DWT_NO_SYNC_PTRS        4   // Do not try to sync IC side and Host side buffer pointers when enabling RX. This is used to perform manual RX
                                    // re-enabling when receiving a frame in double buffer mode.

/* Exported functions ----------------------------------------------------------------------*/  

void      DWT_ReadData( uint16_t regAddr, uint16_t subIndex, void *readBuf, uint32_t lens );
void      DWT_WriteData( uint16_t regAddr, uint16_t subIndex, const void *writeBuf, uint32_t lens );
uint8_t   DWT_ReadData8( uint16_t regAddr, uint16_t subIndex );
void      DWT_WriteData8( uint16_t regAddr, uint16_t subIndex, uint8_t writeData );
uint16_t  DWT_ReadData16( uint16_t regAddr, uint16_t subIndex );
void      DWT_WriteData16( uint16_t regAddr, uint16_t subIndex, uint16_t writeData );
uint32_t  DWT_ReadData32( uint16_t regAddr, uint16_t subIndex );
void      DWT_WriteData32( uint16_t regAddr, uint16_t subIndex, uint32_t writeData );

void      DWT_SetSpeedLow( void );
void      DWT_SetSpeedHigh( void );
void      DWT_Config( void );
void      DWT_EnableIRQ( void );
void      DWT_DisableIRQ( void );

int8_t    DWT_Initialise( DWT_ConfigTypeDef *config );
void      DWT_SetFineGrainTxSeq( uint8_t enable );
void      DWT_SetLnaPaMode( uint8_t lna, uint8_t pa );
uint32_t  DWT_ReadDeviceID( void );
void      DWT_ConfigureTxRF( dwt_txconfig_t *config );
void      DWT_Configure( DWT_ConfigTypeDef *config );
void      DWT_SetRxAntennaDelay( uint16_t rxDelay );
void      DWT_SetTxAntennaDelay( uint16_t txDelay );
int8_t    DWT_WriteTxData( uint16_t txFrameLength, uint8_t *txFrameBytes, uint16_t txBufOffset );
void      DWT_WriteTxFCtrl( uint16_t txFrameLength, uint16_t txBufOffset, uint8_t ranging );
void      DWT_ReadRxData( uint8_t *buffer, uint16_t length, uint16_t rxBufOffset );
void      DWT_ReadAccData( uint8_t *buffer, uint16_t length, uint16_t accOffset );
void      DWT_ReadDiagnostics( dwt_rxdiag_t *diagnostics );
void      DWT_ReadTxTimestamp( uint8_t *timestamp );
uint32_t  DWT_ReadTxTimestampH32( void );
uint32_t  DWT_ReadTxTimestampL32( void );
void      DWT_ReadRxTimestamp( uint8_t *timestamp );
uint32_t  DWT_ReadRxTimestampH32( void );
uint32_t  DWT_ReadRxTimestampL32( void );
uint32_t  DWT_ReadSysTimestampH32( void );
void      DWT_ReadSysTime( uint8_t *timestamp );
void      DWT_EnableFrameFilter( uint16_t enable );
void      DWT_SetPanID( uint16_t panID );
void      DWT_SetAddress16( uint16_t shortAddress );
void      DWT_SetEUI( uint8_t *eui );
void      DWT_GetEUI( uint8_t *eui );
void      DWT_OtpRead( uint32_t address, uint32_t *array, uint8_t length );
//int8_t    DWT_OtpWriteAndVerify( uint32_t value, uint16_t address );
void      DWT_EnterSleep( void );
void      DWT_ConfigureSleepCnt( uint16_t sleepCnt );
uint16_t  DWT_CalibRateSleepCnt( void );
void      DWT_ConfigureSleep( uint16_t mode, uint8_t wake );
void      DWT_EnterSleepAfterTx( uint8_t enable );
int8_t    DWT_SpiCsWakeup( uint8_t *buffer, uint16_t length );
//void      OPT_LoadOpsetTabFromOtp( uint8_t opsSel );
void      DWT_SetSmartTxPower( uint8_t enable );
void      DWT_EnableAutoAck( uint8_t responseDelayTime );
void      DWT_SetDblRxBufMode( uint8_t enable );
void      DWT_SetRxAfterTxDelay( uint32_t rxDelayTime );
void      DWT_SetCallbacks( dwt_cb_t cbTxDone, dwt_cb_t cbRxOk, dwt_cb_t cbRxTo, dwt_cb_t cbRxErr );
void      DWT_SetSysStatus( uint32_t status );
uint32_t  DWT_GetSysStatus( void );
uint8_t   DWT_CheckIRQ( void );
void      DWT_ISR( void );
//void      DWT_LowPowerListenISR( void );
//void      DWT_SetLeds( uint8_t mode );
void      DWT_SetDelayedTxRxTime( uint32_t startTime );
int8_t    DWT_StartTx( uint8_t mode );
void      DWT_ForceTxRxOff( void );
void      DWT_SyncRxBufPtrs( void );
void      DWT_SetSniffMode( uint8_t enable, uint8_t timeOn, uint8_t timeOff );
void      DWT_SetLowPowerListening( uint8_t enable );
void      DWT_SetSnoozeTime( uint8_t snoozeTime );
int8_t    DWT_RxEnable( uint8_t mode );
void      DWT_SetRxTimeout( uint16_t time );
void      DWT_SetPreambleDetectTimeout( uint16_t timeout );
void      DWT_SetInterrupt( uint32_t bitmask, uint8_t enable );
void      DWT_ConfigEventCounters( uint8_t enable );
void      DWT_ReadEventCounters( dwt_deviceentcnts_t *counters );
void      DWT_RxReset( void );
void      DWT_SoftwareReset( void );
void      DWT_SetXtalTrim( uint8_t xtal );
void      DWT_ConfigCWMode( uint8_t channel );
void      DWT_ConfigContinuousFrameMode( uint32_t frameRepetitionRate );
//uint16_t  DWT_ReadTempVbat( uint8_t fastSPI );
//uint8_t   DWT_ReadWakeupTemp( void );
//uint8_t   DWT_ReadWakeupVbat( void );

extern DwHandle_st dw1000_spi;

#ifdef __cplusplus
}
#endif

#endif

/*************************************** END OF FILE ****************************************/
