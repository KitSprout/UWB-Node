/**
  *      __            ____
  *     / /__ _  __   / __/                      __  
  *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
  *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
  *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
  *                    /_/   github.com/KitSprout    
  * 
  * @file    dw1000.c
  * @author  KitSprout
  * @date    27-Nov-2016
  * @brief   
  * 
  */

/* Includes --------------------------------------------------------------------------------*/
#include "drivers\stm32f4_system.h"
#include "drivers\stm32f4_spi.h"
#include "modules\dw1000.h"

/** @addtogroup STM32_Module
  * @{
  */

/* Private typedef -------------------------------------------------------------------------*/
/* Private define --------------------------------------------------------------------------*/

/* OPT memory address */
#define LDOTUNE_ADDR            ((uint16_t)0x0004)
#define PARTID_ADDR             ((uint16_t)0x0006)
#define LOTID_ADDR              ((uint16_t)0x0007)
#define VBAT_ADDR               ((uint16_t)0x0008)
#define VTEMP_ADDR              ((uint16_t)0x0009)
#define XTRIM_ADDR              ((uint16_t)0x001E)

/* Private macro ---------------------------------------------------------------------------*/
#define DWT_Delay(__TIME)   delay_ms(__TIME);

/* Private variables -----------------------------------------------------------------------*/
extern SPI_HandleTypeDef HSPI_DW1000;
extern pFunc DW1000_ExtiCallback;

static uint8_t TX_BUFFER[DW1000_MAX_TXBUF] = {0};
static uint8_t RX_BUFFER[DW1000_MAX_RXBUF] = {0};

DwHandle_st dw1000_spi = {
  .handle    = &HSPI_DW1000,
  .txBufLens = DW1000_MAX_TXBUF,
  .rxBufLens = DW1000_MAX_RXBUF,
  .pTxBuf    = TX_BUFFER,
  .pRxBuf    = RX_BUFFER,
};

/* Private function prototypes -------------------------------------------------------------*/
/* Private functions -----------------------------------------------------------------------*/

/**
  * @brief  DWT_ReadData
  */
void DWT_ReadData( uint16_t regAddr, uint16_t subIndex, void *readBuf, uint32_t lens )
{
  uint8_t header[3];
  uint32_t count = 0;

  if (subIndex == 0) {
    header[count++] = (uint8_t)regAddr;
  }
  else {
    header[count++] = 0x40 | (uint8_t)regAddr;
    if (subIndex < 0xF0) {   /* 7-bit, subIndex <= 0x7F */
      header[count++] = (uint8_t)subIndex;
    }
    else {                  /* 15-bit, subIndex <= 0x7FFF, extended address */
      header[count++] = 0x80 | (uint8_t)subIndex;
      header[count++] = (uint8_t)(subIndex >> 7);
    }
  }

  DW1000_CSD_L();
  SPI_SendData(dw1000_spi.handle, header, count, HAL_MAX_DELAY);
  SPI_RecvData(dw1000_spi.handle, readBuf, lens, HAL_MAX_DELAY);
  DW1000_CSD_H();
}

/**
  * @brief  DWT_WriteData
  */
void DWT_WriteData( uint16_t regAddr, uint16_t subIndex, const void *writeBuf, uint32_t lens )
{
  uint8_t header[3] = {0};
  uint32_t count = 0;

  if (subIndex == 0) {
    header[count++] = 0x80 | (uint8_t)regAddr;
  }
  else {
    header[count++] = 0xC0 | (uint8_t)regAddr;
    if (subIndex < 0xF0) {   /* 7-bit, subIndex <= 0x7F */
      header[count++] = (uint8_t)subIndex;
    }
    else {                  /* 15-bit, subIndex <= 0x7FFF, extended address */
      header[count++] = 0x80 | (uint8_t)subIndex;
      header[count++] = (uint8_t)(subIndex >> 7);
    }
  }

  DW1000_CSD_L();
  SPI_SendData(dw1000_spi.handle, header, count, HAL_MAX_DELAY);
  SPI_SendData(dw1000_spi.handle, (uint8_t*)writeBuf, lens, HAL_MAX_DELAY);
  DW1000_CSD_H();
}

/**
  * @brief  DWT_ReadData8
  */
uint8_t DWT_ReadData8( uint16_t regAddr, uint16_t subIndex )
{
  uint8_t readData;
  DWT_ReadData(regAddr, subIndex, &readData, sizeof(readData));
  return readData;
}

/**
  * @brief  DWT_WriteData8
  */
void DWT_WriteData8( uint16_t regAddr, uint16_t subIndex, uint8_t writeData )
{
  DWT_WriteData(regAddr, subIndex, &writeData, sizeof(writeData));
}

/**
  * @brief  DWT_ReadData16
  */
uint16_t DWT_ReadData16( uint16_t regAddr, uint16_t subIndex )
{
  uint16_t readData;
  DWT_ReadData(regAddr, subIndex, &readData, sizeof(readData));
  return readData;
}

/**
  * @brief  DWT_WriteData16
  */
void DWT_WriteData16( uint16_t regAddr, uint16_t subIndex, uint16_t writeData )
{
  DWT_WriteData(regAddr, subIndex, &writeData, sizeof(writeData));
}

/**
  * @brief  DWT_ReadData32
  */
uint32_t DWT_ReadData32( uint16_t regAddr, uint16_t subIndex )
{
  uint32_t readData;
  DWT_ReadData(regAddr, subIndex, &readData, sizeof(readData));
  return readData;
}

/**
  * @brief  DWT_WriteData32
  */
void DWT_WriteData32( uint16_t regAddr, uint16_t subIndex, uint32_t writeData )
{
  DWT_WriteData(regAddr, subIndex, &writeData, sizeof(writeData));
}

/**
  * @brief  DWT_SetSpeedLow
  */
void DWT_SetSpeedLow( void )
{
  SPI_SetSpeed(dw1000_spi.handle, DW1000_SPIx_SPEED_LOW);
}

/**
  * @brief  DWT_SetSpeedHigh
  */
void DWT_SetSpeedHigh( void )
{
  SPI_SetSpeed(dw1000_spi.handle, DW1000_SPIx_SPEED_HIGH);
}

/**
  * @brief  DWT_HardwareReset
  */
static void DWT_HardwareReset( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin   = DW1000_RST_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(DW1000_RST_GPIO_PORT, &GPIO_InitStruct);

  DW1000_RST_L();

  GPIO_InitStruct.Pin   = DW1000_RST_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(DW1000_RST_GPIO_PORT, &GPIO_InitStruct);

  DWT_Delay(5);
}

/**
  * @brief  DWT_Config
  */
void DWT_Config( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* SPI Pin ******************************************************************/
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

  GPIO_InitStruct.Pin   = DW1000_CSD_PIN;
  HAL_GPIO_Init(DW1000_CSD_GPIO_PORT, &GPIO_InitStruct);

  DW1000_CSD_H();  // LOW ENABLE

  GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;

  GPIO_InitStruct.Pin   = DW1000_IRQ_PIN;
  HAL_GPIO_Init(DW1000_IRQ_GPIO_PORT, &GPIO_InitStruct);

  /* EXTI IT *******************************************************************/
  HAL_NVIC_SetPriority(DW1000_IRQn, 0x0F, 0);
//  HAL_NVIC_EnableIRQ(DW1000_IRQn);

  /* SPI Init ****************************************************************/
  dw1000_spi.handle->Instance               = DW1000_SPIx;
  dw1000_spi.handle->Init.Mode              = SPI_MODE_MASTER;
  dw1000_spi.handle->Init.Direction         = SPI_DIRECTION_2LINES;
  dw1000_spi.handle->Init.DataSize          = SPI_DATASIZE_8BIT;
  dw1000_spi.handle->Init.CLKPolarity       = SPI_POLARITY_LOW;
  dw1000_spi.handle->Init.CLKPhase          = SPI_PHASE_1EDGE;
  dw1000_spi.handle->Init.NSS               = SPI_NSS_SOFT;
  dw1000_spi.handle->Init.BaudRatePrescaler = DW1000_SPIx_SPEED_LOW;
  dw1000_spi.handle->Init.FirstBit          = SPI_FIRSTBIT_MSB;
  dw1000_spi.handle->Init.TIMode            = SPI_TIMODE_DISABLE;
  dw1000_spi.handle->Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  dw1000_spi.handle->Init.CRCPolynomial     = 7;
  HAL_SPI_Init(dw1000_spi.handle);
}

/**
  * @brief  DWT_EnableIRQ
  */
void DWT_EnableIRQ( void )
{
  HAL_NVIC_EnableIRQ(DW1000_IRQn);
}

/**
  * @brief  DWT_DisableIRQ
  */
void DWT_DisableIRQ( void )
{
  HAL_NVIC_DisableIRQ(DW1000_IRQn);
}






/** ---- Dwcawave drivers --------------------------------------------------------------------
  *
  * @brief Decawave device configuration and control functions
  *
  */

/* Private typedef -------------------------------------------------------------------------*/

typedef struct {
  uint32_t low32;
  uint16_t target[2];
} agc_cfg_struct;

typedef struct {
  uint32_t      deviceID;     // Device ID - 0xDECA0130
  uint32_t      partID;       // IC Part ID - read during initialisation
  uint32_t      lotID;        // IC Lot ID - read during initialisation
  uint8_t       longFrames;   // Flag in non-standard long frame mode
  uint8_t       otpRev;       // OTP revision number (read during initialisation)
  uint32_t      txFCTRL;      // Keep TX_FCTRL register config
  uint8_t       xtrim;        // initial XTAL trim value read from OTP (or defaulted to mid-range if OTP not programmed)
  uint8_t       dblbuffon;    // Double RX buffer mode flag
  uint32_t      sysCFGreg;    // Local copy of system config register
  uint16_t      sleepMode;    // Used for automatic reloading of LDO tune and microcode at wake-up
  uint8_t       wait4resp;    // wait4response was set with last TX start command
  dwt_cb_data_t cbData;       // Callback data structure
  dwt_cb_t      cbTxDone;     // Callback for TX confirmation event
  dwt_cb_t      cbRxOk;       // Callback for RX good frame event
  dwt_cb_t      cbRxTo;       // Callback for RX timeout events
  dwt_cb_t      cbRxErr;      // Callback for RX error events
} dwt_local_data_t;

/* Private define --------------------------------------------------------------------------*/

/* OPT memory address */
#define LDOTUNE_ADDR      ((uint16_t)0x0004)
#define PARTID_ADDR       ((uint16_t)0x0006)
#define LOTID_ADDR        ((uint16_t)0x0007)
#define VBAT_ADDR         ((uint16_t)0x0008)
#define VTEMP_ADDR        ((uint16_t)0x0009)
#define XTRIM_ADDR        ((uint16_t)0x001E)

/* Private macro ---------------------------------------------------------------------------*/
/* Private variables -----------------------------------------------------------------------*/

const uint8_t chan_idx[8] = { 0, 0, 1, 2, 3, 4, 0, 5 };

// 32-bit value to program to Sub-Register 0x28:0C RF_TXCTRL
const uint32_t tx_config[6] = { 0x00005C40UL, 0x00045CA0UL, 0x00086CC0UL, 0x00045C80UL, 0x001E3FE0UL, 0x001E7DE0UL };

// Frequency Synthesiser - PLL configuration
const uint32_t fs_pll_cfg[6] = { 0x09000407UL, 0x08400508UL, 0x08401009UL, 0x08400508UL, 0x0800041DUL, 0x0800041DUL };

// Frequency Synthesiser - PLL tuning
const uint8_t fs_pll_tune[6] = { 0x1E, 0x26, 0x5E, 0x26, 0xA6, 0xA6 };

// bandwidth configuration
const uint8_t rx_config[2] = { 0xD8, 0xBC };

const agc_cfg_struct agc_config = { 0X2502A907UL, { 0x8870 , 0x889B } };

// DW non-standard SFD length for 110k, 850k and 6.81M
const uint8_t dwnsSFDlen[3] = { 64, 16, 8 };

// SFD Threshold
const uint16_t sftsh[3][2] = { {0x000A, 0x0016}, {0x0001, 0x0006}, {0x0001,  0x0002} };

const uint16_t dtune1[2] = { 0x0087, 0x008D };

const uint32_t digital_bb_config[2][4] = { { 0x311A002DUL, 0x331A0052UL, 0x351A009AUL, 0x371A011DUL },
                                           { 0x313B006BUL, 0x333B00BEUL, 0x353B015EUL, 0x373B0296UL } };

const uint16_t lde_replicaCoeff[25] = { 0, // No preamble code 0
  0x5998, 0x5998, 0x51EA, 0x428E, 0x451E, 0x2E14, 0x8000, 0x51EA, 0x28F4, 0x3332, 0x3AE0, 0x3D70,
  0x3AE0, 0x35C2, 0x2B84, 0x35C2, 0x3332, 0x35C2, 0x35C2, 0x47AE, 0x3AE0, 0x3850, 0x30A2, 0x3850
};

static dwt_local_data_t DW1000Local = {
  .deviceID   = 0,
  .partID     = 0,
  .lotID      = 0,
  .longFrames = 0,
  .otpRev     = 0,
  .txFCTRL    = 0,
  .xtrim      = 0,
  .dblbuffon  = 0,
  .sysCFGreg  = 0,
  .sleepMode  = 0,
  .wait4resp  = 0,
  .cbData     = 0,
  .cbTxDone   = NULL,
  .cbRxOk     = NULL,
  .cbRxTo     = NULL,
  .cbRxErr    = NULL,
};

/* Private function prototypes -------------------------------------------------------------*/
static uint32_t _DWT_OtpRead( uint32_t address );
static void _DWT_ConfigLDE( uint8_t prfIndex );
static void _DWT_LoadUCodeFromROM( void );
static void _DWT_EnableClocks( uint8_t clocks );

/* Private functions -----------------------------------------------------------------------*/

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_Initialise()
 * @brief  This function initiates communications with the DW1000 transceiver
 *         and reads its DEV_ID register (address 0x00) to verify the IC is one supported
 *         by this software (e.g. DW1000 32-bit device ID value is 0xDECA0130). Then it
 *         does any initial once only device configurations needed for use and initialises
 *         as necessary any static data items belonging to this low-level driver.
 *         >> NOTE:
 *         1. this function needs to be run before DWT_ConfigureSleep,
 *            also the SPI frequency has to be < 3MHz
 *         2. it also reads and applies LDO tune and crystal trim values from OTP memory
 * @param  config: specifies what configuration to load
 *          - DWT_LOADUCODE  0x1 - load the LDE microcode from ROM - enabled accurate RX timestamp
 *          - DWT_LOADNONE   0x0 - do not load any values from OTP memory
 * @retval None
 */
int8_t DWT_Initialise( DWT_ConfigTypeDef *config )
{
  uint16_t tmp16 = 0;
  uint32_t tmp32 = 0;

  // Hardware reset
  DWT_HardwareReset();
  DWT_Delay(2);

  // Check device
  if (DW1000_DEVICE_ID != DWT_ReadDeviceID()) {
    return HAL_ERROR;
  }

  // Link IRQ EXTI function
  DW1000_ExtiCallback = DWT_ISR;

  // Make sure the device is completely reset before starting initialisation
  DWT_SoftwareReset();
  DWT_Delay(2);

  // NOTE: set system clock to XTI - this is necessary to make sure the values read by _DWT_OtpRead are reliable
  _DWT_EnableClocks(FORCE_SYS_XTI);

  // Configure the CPLL lock detect
  DWT_WriteData8(DW1000_EXT_SYNC, DW1000_SUB_EC_CTRL, 0x04);

  // Read OTP revision number
  tmp16 = _DWT_OtpRead(XTRIM_ADDR) & 0xFFFF;  // Read 32 bit value, XTAL trim val is in low octet-0 (5 bits)
  DW1000Local.otpRev = (tmp16 >> 8) & 0xFF;    // OTP revision is next byte

  // Load LDO tune from OTP and kick it if there is a value actually programmed.
  tmp32 = _DWT_OtpRead(LDOTUNE_ADDR);
  if (tmp32 & 0xFF) {
    // Kick LDO tune
    DWT_WriteData8(DW1000_OTP_IF, DW1000_SUB_OTP_SF, 0x02);   // Set load LDE kick bit
    DW1000Local.sleepMode |= 0x1000;   // LDO tune must be kicked at wake-up
  }

  // Load Part and Lot ID from OTP
  DW1000Local.partID = _DWT_OtpRead(PARTID_ADDR);
  DW1000Local.lotID  = _DWT_OtpRead(LOTID_ADDR);

  // XTAL trim value is set in OTP for DW1000 module and EVK/TREK boards but that might not be the case in a custom design
  DW1000Local.xtrim = tmp16 & 0x1F;
  if (!DW1000Local.xtrim) {    // A value of 0 means that the crystal has not been trimmed
    DW1000Local.xtrim = 0x10;  // Set to mid-range if no calibration value inside
  }
  // Configure XTAL trim
  DWT_SetXtalTrim(DW1000Local.xtrim);

  // Load leading edge detect code
  if (config->LoadCode & DW_LOAD_UCODE) {
    _DWT_LoadUCodeFromROM();
    DW1000Local.sleepMode |= 0x0800;   // microcode must be loaded at wake-up
  }
  else {  // Should disable the LDERUN enable bit in 0x36, 0x4
    tmp16 = DWT_ReadData16(DW1000_PMSC, DW1000_SUB_PMSC_CTRL1 + 1) & 0xFDFF;
    DWT_WriteData16(DW1000_PMSC, DW1000_SUB_PMSC_CTRL1 + 1, tmp16);
  }

  _DWT_EnableClocks(ENABLE_ALL_SEQ); // Enable clocks for sequencing

  // The 3 bits in AON CFG1 register must be cleared to ensure proper operation of the DW1000 in DEEPSLEEP mode.
  DWT_WriteData8(DW1000_AON, DW1000_SUB_AON_CFG1, 0x00);

  // Read system register / store local copy
  DW1000Local.sysCFGreg = DWT_ReadData32(DW1000_SYS_CFG, 0x00);  // Read sysconfig register

  return HAL_OK;
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_SetFineGrainTxSeq()
 * @brief  This function enables/disables the fine grain TX sequencing (enabled by default).
 * @param  enable: 1 to enable fine grain TX sequencing, 0 to disable it.
 * @retval None
 */
void DWT_SetFineGrainTxSeq( uint8_t enable )
{
  if (enable) {
    DWT_WriteData16(DW1000_PMSC, DW1000_SUB_PMSC_TXFSEQ, 0x0B74);
  }
  else {
    DWT_WriteData16(DW1000_PMSC, DW1000_SUB_PMSC_TXFSEQ, 0x0000);
  }
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_SetLnaPaMode()
 * @brief  This function enables/disables the fine grain TX sequencing (enabled by default).
 *         >> NOTE:
 *         Enabling PA functionality requires that fine grain TX sequencing is deactivated.
 *         This can be done using DWT_SetFineGrainTxSeq().
 * @param  lna: 1 to enable LNA functionality, 0 to disable it
 * @param  pa: 1 to enable PA functionality, 0 to disable it
 * @retval None
 */
void DWT_SetLnaPaMode( uint8_t lna, uint8_t pa )
{
  uint32_t gpioMode = DWT_ReadData32(DW1000_GPIO_CTRL, DW1000_SUB_GPIO_MODE);

  gpioMode &= ~0x000FC000UL;    // GPIO4/EXTPA, GPIO5/EXTTXE, GPIO6/EXTRXE

  if (lna) {
    gpioMode |= 0x00040000UL;   // EXTRXE
  }
  if (pa) {
    gpioMode |= 0x00014000UL;   // EXTPA, EXTTXE
  }

  DWT_WriteData32(DW1000_GPIO_CTRL, DW1000_SUB_GPIO_MODE, gpioMode);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_ReadDeviceID()
 * @brief  This is used to return the read device type and revision information of the DW1000 device (MP part is 0xDECA0130)
 * @param  None
 * @retval the read value which for DW1000 is 0xDECA0130
 */
uint32_t DWT_ReadDeviceID( void )
{
  return DWT_ReadData32(DW1000_DEV_ID, 0x00);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_ConfigureTxRF()
 * @brief  This function provides the API for the configuration of the TX spectrum
 *         including the power and pulse generator delay. The input is a pointer to the
 *         data structure of type dwt_txconfig_t that holds all the configurable items.
 * @param  config: pointer to the txrf configuration structure, which contains the tx rf config data
 * @retval None
 */
void DWT_ConfigureTxRF( dwt_txconfig_t *config )
{
  /* Configure RF TX PG_DELAY */
  DWT_WriteData8(DW1000_TX_CAL, DW1000_SUB_TC_PGDELAY, config->PGdelay);
  /* Configure TX power */
  DWT_WriteData32(DW1000_TX_POWER, 0x00, config->power);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_Configure()
 * @brief  This function provides the main API for the configuration of the
 *         DW1000 and this low-level driver. The input is a pointer to the data structure
 *         of type dwt_config_t that holds all the configurable items.
 * @param  config: pointer to the configuration structure, which contains the device configuration data.
 * @retval None
 */
void DWT_Configure( DWT_ConfigTypeDef *config )
{
  uint8_t chan = config->Channel;
  uint8_t bw = ((chan == 4) || (chan == 7)) ? 1 : 0;
  uint8_t prfIndex = config->PulseRepFreq - DW_PRF_16M;
  uint16_t tmp16 = lde_replicaCoeff[config->RxCode];
  uint32_t tmp32;

  // For 110 kbps we need a special setup
  if (DW_DATARATE_110K == config->DataRate) {
    DW1000Local.sysCFGreg |= 0x00400000UL;
    tmp16 >>= 3;  // lde_replicaCoeff must be divided by 8
  }
  else {
    DW1000Local.sysCFGreg &= (~0x00400000UL);
  }

  DW1000Local.longFrames = config->PhrMode ;

  DW1000Local.sysCFGreg &= ~0x00030000UL;
  DW1000Local.sysCFGreg |= (0x00030000UL & (config->PhrMode << 16));
  DWT_WriteData32(DW1000_SYS_CFG, 0x00, DW1000Local.sysCFGreg);

  // Set the lde_replicaCoeff
  DWT_WriteData16(DW1000_LDE_CTRL, DW1000_SUB_LDE_REPC, tmp16) ;

  _DWT_ConfigLDE(prfIndex);

  // Configure PLL2/RF PLL block CFG/TUNE (for a given channel)
  DWT_WriteData32(DW1000_FS_CTRL, DW1000_SUB_FS_PLLCFG, fs_pll_cfg[chan_idx[chan]]);
  DWT_WriteData8(DW1000_FS_CTRL, DW1000_SUB_FS_PLLTUNE, fs_pll_tune[chan_idx[chan]]);

  // Configure RF RX blocks (for specified channel/bandwidth)
  DWT_WriteData8(DW1000_RF_CONF, DW1000_SUB_RF_RXCTRLH, rx_config[bw]);

  // Configure RF TX blocks (for specified channel and PRF)
  DWT_WriteData32(DW1000_RF_CONF, DW1000_SUB_RF_TXCTRL, tx_config[chan_idx[chan]]);

  // Configure the baseband parameters (for specified PRF, bit rate, PAC, and SFD settings)
  DWT_WriteData16(DW1000_DRX_CONF, DW1000_SUB_DRX_TUNE0b, sftsh[config->DataRate][config->NonStandardSFD]);
  DWT_WriteData16(DW1000_DRX_CONF, DW1000_SUB_DRX_TUNE1a, dtune1[prfIndex]);

  if(config->DataRate == DW_DATARATE_110K) {
    DWT_WriteData16(DW1000_DRX_CONF, DW1000_SUB_DRX_TUNE1b, 0x0064);
  }
  else {
    if(config->TxPreambLen == DW_TX_PLEN_64) {
      DWT_WriteData16(DW1000_DRX_CONF, DW1000_SUB_DRX_TUNE1b, 0x0010);
      DWT_WriteData8(DW1000_DRX_CONF, DW1000_SUB_DRX_TUNE4H, 0x10);
    }
    else {
      DWT_WriteData16(DW1000_DRX_CONF, DW1000_SUB_DRX_TUNE1b, 0x0020);
      DWT_WriteData8(DW1000_DRX_CONF, DW1000_SUB_DRX_TUNE4H, 0x28);
    }
  }

  DWT_WriteData32(DW1000_DRX_CONF, DW1000_SUB_DRX_TUNE2, digital_bb_config[prfIndex][config->PreambleAcqChunk]);

  // Don't allow 0 - SFD timeout will always be enabled
  if (config->SFDTimeout == 0) {
    config->SFDTimeout = 0x1041;   // default SFD timeout value
  }
  DWT_WriteData16(DW1000_DRX_CONF, DW1000_SUB_DRX_SFDTOC, config->SFDTimeout);

  // Configure AGC parameters
  DWT_WriteData32(DW1000_AGC_CTRL, DW1000_SUB_AGC_TUNE2, agc_config.low32);
  DWT_WriteData16(DW1000_AGC_CTRL, DW1000_SUB_AGC_TUNE1, agc_config.target[prfIndex]);

  // Set (non-standard) user SFD for improved performance,
  if (config->NonStandardSFD) {
    // Write non standard (DW) SFD length
    DWT_WriteData8(DW1000_USR_SFD, 0x00, dwnsSFDlen[config->DataRate]);
  }

  tmp32 = (0x0000000FUL & (chan))                         |   // Transmit Channel
          (0x000000F0UL & (chan                   << 4))  |   // Receive Channel
          (0x00020000UL & (config->NonStandardSFD << 17)) |   // Use DW nsSFD
          (0x000C0000UL & (config->PulseRepFreq   << 18)) |   // RX PRF
          (0x07C00000UL & (config->TxCode         << 22)) |   // TX Preamble Code
          (0xF8000000UL & (config->RxCode         << 27));    // RX Preamble Code
  DWT_WriteData32(DW1000_CHAN_CTRL, 0x00, tmp32) ;

  // Set up TX Preamble Size, PRF and Data Rate
  DW1000Local.txFCTRL = ((config->TxPreambLen | config->PulseRepFreq) << 16) | (config->DataRate << 13);
  DWT_WriteData32(DW1000_TX_FCTRL, 0x00, DW1000Local.txFCTRL);

  // Request TX start and TRX off at the same time
  DWT_WriteData8(DW1000_SYS_CTRL, 0x00, 0x42);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_SetRxAntennaDelay()
 * @brief  This API function writes the antenna delay (in time units) to RX registers
 * @param  rxDelay: this is the total (RX) antenna delay value, which will be programmed into the RX register
 * @retval None
 */
void DWT_SetRxAntennaDelay( uint16_t rxDelay )
{
  // Set the RX antenna delay for auto TX timestamp adjustment
  DWT_WriteData16(DW1000_LDE_CTRL, DW1000_SUB_LDE_RXANTD, rxDelay);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_SetTxAntennaDelay()
 * @brief  This API function writes the antenna delay (in time units) to TX registers
 * @param  txDelay: this is the total (TX) antenna delay value, which will be programmed into the TX delay register
 * @retval None
 */
void DWT_SetTxAntennaDelay( uint16_t txDelay )
{
  // Set the TX antenna delay for auto TX timestamp adjustment
  DWT_WriteData16(DW1000_TX_ANTD, 0x00, txDelay);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_WriteTxData()
 * @brief  This API function writes the supplied TX data into the DW1000's TX buffer.
 *         The input parameters are the data length in bytes and a pointer to those data bytes.
 * @param  rxDelay: this is the total (RX) antenna delay value, which will be programmed into the RX register
 * @param  txFrameLength: This is the total frame length, including the two byte CRC.
 *                        Note: this is the length of TX message (including the 2 byte CRC) - max is 1023
 *                        standard PHR mode allows up to 127 bytes
 *                        if > 127 is programmed, DWT_PHRMODE_EXT needs to be set in the phrMode configuration
 *                        see dwt_configure function
 * @param  txFrameBytes: Pointer to the userӳ buffer containing the data to send.
 * @param  txBufOffset: This specifies an offset in the DW1000ӳ TX Buffer at which to start writing data.
 * @retval HAL_OK for success, or HAL_ERROR for error
 */
int8_t DWT_WriteTxData( uint16_t txFrameLength, uint8_t *txFrameBytes, uint16_t txBufOffset )
{
  if ((txBufOffset + txFrameLength) <= 1024) {
    // Write the data to the IC TX buffer, (-2 bytes for auto generated CRC)
    DWT_WriteData(DW1000_TX_BUFFER, txBufOffset, txFrameBytes, txFrameLength - 2);
    return HAL_OK;
  }
  else {
    return HAL_ERROR;
  }
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_WriteTxFCtrl()
 * @brief  This API function configures the TX frame control register before the transmission of a frame
 * @param  txFrameLength: this is the length of TX message (including the 2 byte CRC) - max is 1023
 *                        NOTE: standard PHR mode allows up to 127 bytes
 *                        if > 127 is programmed, DWT_PHRMODE_EXT needs to be set in the phrMode configuration
 *                        see dwt_configure function
 * @param  txBufOffset: the offset in the tx buffer to start writing the data
 * @param  ranging: 1 if this is a ranging frame, else 0
 * @retval None
 */
void DWT_WriteTxFCtrl( uint16_t txFrameLength, uint16_t txBufOffset, uint8_t ranging )
{
  // Write the frame length to the TX frame control register
  // DW1000Local.txFCTRL has kept configured bit rate information
  uint32_t tmp32 = DW1000Local.txFCTRL | txFrameLength | (txBufOffset << 22) | (ranging << 15);
  DWT_WriteData32(DW1000_TX_FCTRL, 0x00, tmp32);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_ReadRxData()
 * @brief  This is used to read the data from the RX buffer, from an offset location give by offset parameter
 * @param  buffer: the buffer into which the data will be read
 * @param  length: the length of data to read (in bytes)
 * @param  rxBufOffset: the offset in the rx buffer from which to read the data
 * @retval None
 */
void DWT_ReadRxData( uint8_t *buffer, uint16_t length, uint16_t rxBufOffset )
{
  DWT_ReadData(DW1000_RX_BUFFER, rxBufOffset, buffer, length);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_ReadAccData()
 * @brief  This is used to read the data from the Accumulator buffer, from an offset location give by offset parameter
 *         >> NOTE:
 *         Because of an internal memory access delay when reading the accumulator the first octet output is a dummy octet
 *         that should be discarded. This is true no matter what sub-index the read begins at.
 * @param  buffer: the buffer into which the data will be read
 * @param  length: the length of data to read (in bytes)
 * @param  accOffset: the offset in the acc buffer from which to read the data
 * @retval None
 */
void DWT_ReadAccData( uint8_t *buffer, uint16_t length, uint16_t accOffset )
{
  // Force on the ACC clocks if we are sequenced
  _DWT_EnableClocks(READ_ACC_ON);
  DWT_ReadData(DW1000_ACC_MEM, accOffset, buffer, length) ;
  _DWT_EnableClocks(READ_ACC_OFF); // Revert clocks back
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_ReadDiagnostics()
 * @brief  this function reads the RX signal quality diagnostic data
 * @param  diagnostics: diagnostic structure pointer, this will contain the diagnostic data read from the DW1000
 * @retval None
 */
void DWT_ReadDiagnostics( dwt_rxdiag_t *diagnostics )
{
  // Read the HW FP index
  diagnostics->firstPath = DWT_ReadData16(DW1000_RX_TIME, 0x05);
  // LDE diagnostic data
  diagnostics->maxNoise = DWT_ReadData16(DW1000_LDE_CTRL, 0x00);

  // Read all 8 bytes in one SPI transaction
  DWT_ReadData(DW1000_RX_FQUAL, 0x00, (uint8_t*)&diagnostics->stdNoise, 8);

  diagnostics->firstPathAmp1 = DWT_ReadData16(DW1000_RX_TIME, 0x07);
  diagnostics->rxPreamCount = (DWT_ReadData32(DW1000_RX_FINFO, 0x00) & 0xFFF00000UL) >> 20;
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_ReadTxTimestamp()
 * @brief  This is used to read the TX timestamp (adjusted with the programmed antenna delay)
 * @param  timestamp: a pointer to a 5-byte buffer which will store the read TX timestamp time
 * @retval None
 */
void DWT_ReadTxTimestamp( uint8_t *timestamp )
{
  // Read bytes directly into buffer
  DWT_ReadData(DW1000_TX_TIME, 0x00, timestamp, 5);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_ReadTxTimestampH32()
 * @brief  This is used to read the high 32-bits of the TX timestamp (adjusted with the programmed antenna delay)
 * @param  None
 * @retval high 32-bits of TX timestamp
 */
uint32_t DWT_ReadTxTimestampH32( void )
{
  // Offset is 1 to get the 4 upper bytes out of 5
  return DWT_ReadData32(DW1000_TX_TIME, 0x01);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_ReadTxTimestampL32()
 * @brief  This is used to read the low 32-bits of the TX timestamp (adjusted with the programmed antenna delay)
 * @param  None
 * @retval low 32-bits of TX timestamp
 */
uint32_t DWT_ReadTxTimestampL32( void )
{
  // Read TX TIME as a 32-bit register to get the 4 lower bytes out of 5
  return DWT_ReadData32(DW1000_TX_TIME, 0x00);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_ReadRxTimestamp()
 * @brief  This is used to read the RX timestamp (adjusted time of arrival)
 * @param  timestamp: a pointer to a 5-byte buffer which will store the read RX timestamp time
 * @retval None
 */
void DWT_ReadRxTimestamp( uint8_t *timestamp )
{
  // Get the adjusted time of arrival
  DWT_ReadData(DW1000_RX_TIME, 0x00, timestamp, 5);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_ReadRxTimestampH32()
 * @brief  This is used to read the high 32-bits of the RX timestamp (adjusted with the programmed antenna delay)
 * @param  None
 * @retval high 32-bits of RX timestamp
 */
uint32_t DWT_ReadRxTimestampH32( void )
{
  // Offset is 1 to get the 4 upper bytes out of 5
  return DWT_ReadData32(DW1000_RX_TIME, 0x01);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_ReadRxTimestampL32()
 * @brief  This is used to read the low 32-bits of the RX timestamp (adjusted with the programmed antenna delay)
 * @param  None
 * @retval low 32-bits of RX timestamp
 */
uint32_t DWT_ReadRxTimestampL32( void )
{
  // Read RX TIME as a 32-bit register to get the 4 lower bytes out of 5
  return DWT_ReadData32(DW1000_RX_TIME, 0x00);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_ReadSysTimestampH32()
 * @brief  This is used to read the high 32-bits of the system time
 * @param  None
 * @retval high 32-bits of system time timestamp
 */
uint32_t DWT_ReadSysTimestampH32( void )
{
  // Offset is 1 to get the 4 upper bytes out of 5
  return DWT_ReadData32(DW1000_SYS_TIME, 0x01);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_ReadSysTime()
 * @brief  This is used to read the system time
 * @param  timestamp: a pointer to a 5-byte buffer which will store the read system time
 *                    the timestamp buffer will contain the value after the function call
 * @retval None
 */
void DWT_ReadSysTime( uint8_t *timestamp )
{
  DWT_ReadData(DW1000_SYS_TIME, 0x00, timestamp, 5);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_EnableFrameFilter()
 * @brief  This is used to enable the frame filtering - (the default option is to
 *         accept any data and ACK frames with correct destination address
 * @param  enable: enables/disables the frame filtering options according to
 *          - DWT_FF_NOTYPE_EN   0x000   no frame types allowed
 *          - DWT_FF_COORD_EN    0x002   behave as coordinator (can receive frames with no destination address (PAN ID has to match))
 *          - DWT_FF_BEACON_EN   0x004   beacon frames allowed
 *          - DWT_FF_DATA_EN     0x008   data frames allowed
 *          - DWT_FF_ACK_EN      0x010   ack frames allowed
 *          - DWT_FF_MAC_EN      0x020   mac control frames allowed
 *          - DWT_FF_RSVD_EN     0x040   reserved frame types allowed
 * @retval None
 */
void DWT_EnableFrameFilter( uint16_t enable )
{
  uint32_t sysconfig = 0xF047FFFFUL & DWT_ReadData32(DW1000_SYS_CFG, 0x00); // Read sysconfig register

  if (enable) {
    // Enable frame filtering and configure frame types
    sysconfig &= ~(0x000001FEUL); // Clear all
    sysconfig |= (enable & 0x000001FEUL) | 0x00000001UL;
  }
  else {
    sysconfig &= ~(0x00000001UL);
  }

  DW1000Local.sysCFGreg = sysconfig;
  DWT_WriteData32(DW1000_SYS_CFG, 0x00, sysconfig);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_SetPanID()
 * @brief  This is used to set the PAN ID
 * @param  panID: this is the PAN ID
 * @retval None
 */
void DWT_SetPanID( uint16_t panID )
{
  // PAN ID is high 16 bits of register
  DWT_WriteData16(DW1000_PANADR, 0x02, panID);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_SetAddress16()
 * @brief  This is used to set 16-bit (short) address
 * @param  shortAddress: this sets the 16 bit short address
 * @retval None
 */
void DWT_SetAddress16( uint16_t shortAddress )
{
  // Short address into low 16 bits
  DWT_WriteData16(DW1000_PANADR, 0x00, shortAddress);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_SetEUI()
 * @brief  This is used to set the EUI 64-bit (long) address
 * @param  eui: this is the pointer to a buffer that contains the 64bit address
 * @retval None
 */
void DWT_SetEUI( uint8_t *eui )
{
  DWT_WriteData(DW1000_EUI, 0x00, eui, 8);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_GetEUI()
 * @brief  This is used to get the EUI 64-bit from the DW1000
 * @param  eui: this is the pointer to a buffer that will contain the read 64-bit EUI value
 * @retval None
 */
void DWT_GetEUI( uint8_t *eui )
{
  DWT_ReadData(DW1000_EUI, 0x00, eui, 8);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_OtpRead()
 * @brief  This is used to read the OTP data from given address into provided array
 * @param  address: this is the OTP address to read from
 * @param  array: this is the pointer to the array into which to read the data
 * @param  length: this is the number of 32 bit words to read (array needs to be at least this length)
 * @retval None
 */
void DWT_OtpRead( uint32_t address, uint32_t *array, uint8_t length )
{
  // NOTE: Set system clock to XTAL - this is necessary to make sure the values read by _DWT_OtpRead are reliable
  _DWT_EnableClocks(FORCE_SYS_XTI);

  for (uint8_t i = 0; i < length; i++) {
    array[i] = _DWT_OtpRead(address + i);
  }

  // Restore system clock to PLL
  _DWT_EnableClocks(ENABLE_ALL_SEQ);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     _DWT_OtpRead()
 * @brief  function to read the OTP memory. Ensure that MR,MRa,MRb are reset to 0.
 * @param  address - address to read at
 * @retval the 32bit of read data
 */
static uint32_t _DWT_OtpRead( uint32_t address )
{
  uint32_t tmp32;

  // Write the address
  DWT_WriteData16(DW1000_OTP_IF, DW1000_SUB_OTP_ADDR, address);

  // Perform OTP Read - Manual read mode has to be set
  DWT_WriteData8(DW1000_OTP_IF, DW1000_SUB_OTP_CTRL, 0x03);
  // OTPREAD is self clearing but OTPRDEN is not
  DWT_WriteData8(DW1000_OTP_IF, DW1000_SUB_OTP_CTRL, 0x00);

  // Read read data, available 40ns after rising edge of OTP_READ
  tmp32 = DWT_ReadData32(DW1000_OTP_IF, DW1000_SUB_OTP_RDAT);

  return tmp32;
}

///*! ------------------------------------------------------------------------------------------
// * @fn     _DWT_OtpSetMRRegs()
// * @brief  Configure the MR registers for initial programming (enable charge pump). Read margin is 
// *         used to stress the read back from the programmed bit. In normal operation this is relaxed.
// * @param  mode: "0" : Reset all to 0x0:           MRA = 0x0000, MRB = 0x0000, MR = 0x0000
// *               "1" : Set for inital programming: MRA = 0x9220, MRB = 0x000E, MR = 0x1024
// *               "2" : Set for soak programming:   MRA = 0x9220, MRB = 0x0003, MR = 0x1824
// *               "3" : High Vpp:                   MRA = 0x9220, MRB = 0x004E, MR = 0x1824
// *               "4" : Low Read Margin:            MRA = 0x0000, MRB = 0x0003, MR = 0x0000
// *               "5" : Array Clean:                MRA = 0x0049, MRB = 0x0003, MR = 0x0024
// *               "4" : Very Low Read Margin:       MRA = 0x0000, MRB = 0x0003, MR = 0x0000
// * @retval HAL_OK for success, or HAL_ERROR for error
// */
//static uint32_t _DWT_OtpSetMRRegs( int mode )
//{
//  uint8_t readbuf[4];
//  uint8_t writebuf[4];
//  uint32_t mra = 0, mrb = 0, mr = 0;

//  // PROGRAMME MRA
//  // Set MRA, MODE_SEL
//  writebuf[0] = 0x03;
//  DWT_WriteData(DW1000_OTP_IF, DW1000_SUB_OTP_CTRL + 0x01, writebuf, 1);

//  // Load data
//  switch (mode & 0x0F) {
//    case 0x00: { mra = 0x0000; mrb = 0x0000; mr = 0x0000; break; }
//    case 0x01: { mra = 0x9220; mrb = 0x000E; mr = 0x1024; break; }
//    case 0x02: { mra = 0x9220; mrb = 0x0003; mr = 0x1824; break; }
//    case 0x03: { mra = 0x9220; mrb = 0x004E; mr = 0x1824; break; }
//    case 0x04: { mra = 0x0000; mrb = 0x0003; mr = 0x0000; break; }
//    case 0x05: { mra = 0x0000; mrb = 0x0003; mr = 0x0024; break; }
//    default  : { return HAL_ERROR; }
//  }

//  writebuf[0] =  mra & 0x00FF;
//  writebuf[1] = (mra & 0xFF00) >> 8;
//  DWT_WriteData(DW1000_OTP_IF, DW1000_SUB_OTP_WDAT, writebuf, 2);

//  // Set WRITE_MR
//  writebuf[0] = 0x08;
//  DWT_WriteData(DW1000_OTP_IF, DW1000_SUB_OTP_CTRL, writebuf, 1);

//  // Wait?
//  // DWT_Delay(10);

//  // Set Clear Mode sel
//  writebuf[0] = 0x02;
//  DWT_WriteData(DW1000_OTP_IF, DW1000_SUB_OTP_CTRL + 0x01, writebuf, 1);

//  // Set AUX update, write MR
//  writebuf[0] = 0x88;
//  DWT_WriteData(DW1000_OTP_IF, DW1000_SUB_OTP_CTRL, writebuf, 1);
//  // Clear write MR
//  writebuf[0] = 0x80;
//  DWT_WriteData(DW1000_OTP_IF, DW1000_SUB_OTP_CTRL, writebuf, 1);
//  // Clear AUX update
//  writebuf[0] = 0x00;
//  DWT_WriteData(DW1000_OTP_IF, DW1000_SUB_OTP_CTRL, writebuf, 1);

//  ///////////////////////////////////////////
//  // PROGRAM MRB
//  // Set SLOW, MRB, MODE_SEL
//  writebuf[0] = 0x05;
//  DWT_WriteData(DW1000_OTP_IF, DW1000_SUB_OTP_CTRL + 0x01, writebuf, 1);

//  writebuf[0] =  mrb & 0x00FF;
//  writebuf[1] = (mrb & 0xFF00) >> 8;
//  DWT_WriteData(DW1000_OTP_IF, DW1000_SUB_OTP_WDAT, writebuf, 2);

//  // Set WRITE_MR
//  writebuf[0] = 0x08;
//  DWT_WriteData(DW1000_OTP_IF, DW1000_SUB_OTP_CTRL, writebuf, 1);

//  // Wait?
//  // DWT_Delay(10);

//  // Set Clear Mode sel
//  writebuf[0] = 0x04;
//  DWT_WriteData(DW1000_OTP_IF, DW1000_SUB_OTP_CTRL + 0x01, writebuf, 1);

//  // Set AUX update, write MR
//  writebuf[0] = 0x88;
//  DWT_WriteData(DW1000_OTP_IF, DW1000_SUB_OTP_CTRL, writebuf,1 );
//  // Clear write MR
//  writebuf[0] = 0x80;
//  DWT_WriteData(DW1000_OTP_IF, DW1000_SUB_OTP_CTRL, writebuf, 1);
//  // Clear AUX update
//  writebuf[0] = 0x00;
//  DWT_WriteData(DW1000_OTP_IF, DW1000_SUB_OTP_CTRL, writebuf, 1);

//  ///////////////////////////////////////////
//  // PROGRAM MR
//  // Set SLOW, MODE_SEL
//  writebuf[0] = 0x01;
//  DWT_WriteData(DW1000_OTP_IF, DW1000_SUB_OTP_CTRL + 0x01, writebuf, 1);
//  // Load data

//  writebuf[0] =  mr & 0x00FF;
//  writebuf[1] = (mr & 0xFF00) >> 8;
//  DWT_WriteData(DW1000_OTP_IF, DW1000_SUB_OTP_WDAT, writebuf, 2);

//  // Set WRITE_MR
//  writebuf[0] = 0x08;
//  DWT_WriteData(DW1000_OTP_IF, DW1000_SUB_OTP_CTRL, writebuf, 1);

//  // Wait?
//  DWT_Delay(10);

//  // Set Clear Mode sel
//  writebuf[0] = 0x00;
//  DWT_WriteData(DW1000_OTP_IF, DW1000_SUB_OTP_CTRL + 0x01, writebuf, 1);

//  // Read confirm mode writes.
//  // Set man override, MRA_SEL
//  writebuf[0] = 0x01;
//  DWT_WriteData(DW1000_OTP_IF, DW1000_SUB_OTP_CTRL, writebuf, 1);
//  writebuf[0] = 0x02;
//  DWT_WriteData(DW1000_OTP_IF, DW1000_SUB_OTP_CTRL + 0x01, writebuf, 1);
//  // MRB_SEL
//  writebuf[0] = 0x04;
//  DWT_WriteData(DW1000_OTP_IF, DW1000_SUB_OTP_CTRL + 0x01, writebuf, 1);
//  DWT_Delay(100);

//  // Clear mode sel
//  writebuf[0] = 0x00;
//  DWT_WriteData(DW1000_OTP_IF, DW1000_SUB_OTP_CTRL + 0x01, writebuf, 1);
//  // Clear MAN_OVERRIDE
//  writebuf[0] = 0x00;
//  DWT_WriteData(DW1000_OTP_IF, DW1000_SUB_OTP_CTRL, writebuf, 1);

//  DWT_Delay(10);

//  if (((mode & 0x0F) == 0x01)||((mode & 0x0F) == 0x02)) {
//    // Read status register
//    DWT_ReadData(DW1000_OTP_IF, DW1000_SUB_OTP_STAT, readbuf, 1);
//  }

//  return HAL_OK;
//}

///*! ------------------------------------------------------------------------------------------
// * @fn     _DWT_OtpProgWord32()
// * @brief  function to program the OTP memory. Ensure that MR,MRa,MRb are reset to 0.
// *         VNM Charge pump needs to be enabled (see _DWT_OtpSetMRRegs)
// *         >> Note:
// *         the address is only 11 bits long.
// * @param  data: program data
// * @param  address: address to read at
// * @retval HAL_OK for success, or HAL_ERROR for error
// */
//static uint32_t _DWT_OtpProgWord32( uint32_t data, uint16_t address )
//{
//  uint8_t readbuf[1];
//  uint8_t writebuf[4];
//  uint8_t otpDone = HAL_ERROR;

//  // Read status register
//  DWT_ReadData(DW1000_OTP_IF, DW1000_SUB_OTP_STAT, readbuf, 1);

//  if ((readbuf[0] & 0x02) != 0x02) {
//    return HAL_ERROR;
//  }

//  // Write the data
//  writebuf[3] = (data >> 24) & 0xFF;
//  writebuf[2] = (data >> 16) & 0xFF;
//  writebuf[1] = (data >> 8)  & 0xFF;
//  writebuf[0] =  data & 0xFF;
//  DWT_WriteData(DW1000_OTP_IF, DW1000_SUB_OTP_WDAT, writebuf, 4);

//  // Write the address [10:0]
//  writebuf[1] = (address >> 8) & 0x07;
//  writebuf[0] =  address & 0xFF;
//  DWT_WriteData(DW1000_OTP_IF, DW1000_SUB_OTP_ADDR, writebuf, 2);

//  // Enable Sequenced programming
//  writebuf[0] = 0x40;
//  DWT_WriteData(DW1000_OTP_IF, DW1000_SUB_OTP_CTRL, writebuf, 1);
//  writebuf[0] = 0x00; // And clear
//  DWT_WriteData(DW1000_OTP_IF, DW1000_SUB_OTP_CTRL, writebuf, 1);

//  // WAIT for status to flag PRGM OK..
//  while (otpDone == HAL_ERROR) {
//    DWT_Delay(1);
//    DWT_ReadData(DW1000_OTP_IF, DW1000_SUB_OTP_STAT, readbuf, 1);
//    if ((readbuf[0] & 0x01) == 0x01) {
//      otpDone = HAL_OK;
//    }
//  }

//  return HAL_OK;
//}

///*! ------------------------------------------------------------------------------------------
// * @fn     DWT_OtpWriteAndVerify()
// * @brief  This is used to program 32-bit value into the DW1000 OTP memory.
// * @param  value: this is the 32-bit value to be programmed into OTP
// * @param  address: this is the 16-bit OTP address into which the 32-bit value is programmed
// * @retval HAL_OK for success, or HAL_ERROR for error
// */
//int8_t DWT_OtpWriteAndVerify( uint32_t value, uint16_t address )
//{
//  int8_t status = HAL_OK;
//  int8_t count = 0;

//  // Firstly set the system clock to crystal
//  _DWT_EnableClocks(FORCE_SYS_XTI); //set system clock to XTI

//  // !!!!!!!!!!!!!! NOTE !!!!!!!!!!!!!!!!!!!!!
//  // Set the supply to 3.7V
//  //

//  _DWT_OtpSetMRRegs(1); // Set mode for programming

//  // For each value to program - the readback/check is done couple of times to verify it has programmed successfully
//  while (1) {
//    _DWT_OtpProgWord32(value, address);

//    if (_DWT_OtpRead(address) == value) {
//      break;
//    }
//    count++;
//    if (count == 5) {
//      break;
//    }
//  }

//  // Even if the above does not exit before retry reaches 5, the programming has probably been successful

//  _DWT_OtpSetMRRegs(4);   // Set mode for reading

//  if(_DWT_OtpRead(address) != value) {    // If this does not pass please check voltage supply on VDDIO
//    status = HAL_ERROR;
//  }

//  _DWT_OtpSetMRRegs(0);   // Setting OTP mode register for low RM read - resetting the device would be alternative

//  return status;
//}

/*! ------------------------------------------------------------------------------------------
 * @fn     _DWT_ANOConfigUpload()
 * @brief  This function uploads always on (AON) configuration, as set in the AON_CFG0_OFFSET register.
 * @param  None
 * @retval None
 */
static void _DWT_AONConfigUpload( void )
{
  DWT_WriteData8(DW1000_AON, DW1000_SUB_AON_CTRL, 0x04);
  DWT_WriteData8(DW1000_AON, DW1000_SUB_AON_CTRL, 0x00);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     _DWT_AONArrayUpload()
 * @brief  This function uploads always on (AON) data array and configuration.
 *         Thus if this function is used, then _DWT_AONConfigUpload is not necessary.
 *         The DW1000 will go so SLEEP straight after this if the DWT_SLP_EN has been set.
 * @param  None
 * @retval None
 */
static void _DWT_AONArrayUpload( void )
{
  DWT_WriteData8(DW1000_AON, DW1000_SUB_AON_CTRL, 0x00);
  DWT_WriteData8(DW1000_AON, DW1000_SUB_AON_CTRL, 0x02);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_EnterSleep()
 * @brief  This function puts the device into deep sleep or sleep. DWT_ConfigureSleep()
 *         should be called first to configure the sleep and on-wake/wake-up parameters
 * @param  None
 * @retval None
 */
void DWT_EnterSleep( void )
{
  // Copy config to AON - upload the new configuration
  _DWT_AONArrayUpload();
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_ConfigureSleepCnt()
 * @brief  sets the sleep counter to new value, this function programs the high 16-bits of the 28-bit counter
 *         >> NOTE:
 *         this function needs to be run before DWT_ConfigureSleep, also the SPI frequency has to be < 3MHz
 * @param  sleepCnt: this it value of the sleep counter to program
 * @retval None
 */
void DWT_ConfigureSleepCnt( uint16_t sleepCnt )
{
  // Force system clock to crystal
  _DWT_EnableClocks(FORCE_SYS_XTI);

  // Reset sleep configuration to make sure we don't accidentally go to sleep
  // NB: this write change the default LPCLKDIVA value which is not used anyway.
  DWT_WriteData8(DW1000_AON, DW1000_SUB_AON_CFG0, 0x00);
  DWT_WriteData8(DW1000_AON, DW1000_SUB_AON_CFG1, 0x00);

  // Disable the sleep counter
  _DWT_AONConfigUpload();

  // Set new value
  DWT_WriteData16(DW1000_AON, DW1000_SUB_AON_CFG0 + 0x02, sleepCnt);
  _DWT_AONConfigUpload();

  // Enable the sleep counter
  DWT_WriteData8(DW1000_AON, DW1000_SUB_AON_CFG1, 0x01);
  _DWT_AONConfigUpload();

  // Put system PLL back on
  _DWT_EnableClocks(ENABLE_ALL_SEQ);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_CalibRateSleepCnt()
 * @brief  calibrates the local oscillator as its frequency can vary between 7 and 13kHz depending on temp and voltage
 *         >> NOTE:
 *         this function needs to be run before DWT_ConfigureSleepCnt, so that we know what the counter units are
 * @param  None
 * @retval the number of XTAL/2 cycles per low-power oscillator cycle. LP OSC frequency = 19.2 MHz/return value
 */
uint16_t DWT_CalibRateSleepCnt( void )
{
  uint16_t result;

  // Enable calibration of the sleep counter
  DWT_WriteData8(DW1000_AON, DW1000_SUB_AON_CFG1, 0x04);
  _DWT_AONConfigUpload();

  // Disable calibration of the sleep counter
  DWT_WriteData8(DW1000_AON, DW1000_SUB_AON_CFG1, 0x00);
  _DWT_AONConfigUpload();

  // Force system clock to crystal
  _DWT_EnableClocks(FORCE_SYS_XTI);

  DWT_Delay(1);

  // Read the number of XTAL/2 cycles one LP oscillator cycle took.
  // Set up address - Read upper byte first
  DWT_WriteData8(DW1000_AON, DW1000_SUB_AON_ADDR, 118);

  // Enable manual override
  DWT_WriteData8(DW1000_AON, DW1000_SUB_AON_CTRL, 0x80);

  // Read confirm data that was written
  DWT_WriteData8(DW1000_AON, DW1000_SUB_AON_CTRL, 0x88);

  // Read back byte from AON
  result = DWT_ReadData8(DW1000_AON, DW1000_SUB_AON_RDAT);
  result <<= 8;

  // Set up address - Read lower byte
  DWT_WriteData8(DW1000_AON, DW1000_SUB_AON_ADDR, 117);

  // Enable manual override
  DWT_WriteData8(DW1000_AON, DW1000_SUB_AON_CTRL, 0x80);

  // Read confirm data that was written
  DWT_WriteData8(DW1000_AON, DW1000_SUB_AON_CTRL, 0x88);

  // Read back byte from AON
  result |= DWT_ReadData8(DW1000_AON, DW1000_SUB_AON_RDAT);

  // Disable manual override
  DWT_WriteData8(DW1000_AON, DW1000_SUB_AON_CTRL, 0x00);

  // Put system PLL back on
  _DWT_EnableClocks(ENABLE_ALL_SEQ);

  // Returns the number of XTAL/2 cycles per one LP OSC cycle
  // This can be converted into LP OSC frequency by 19.2 MHz/result
  return result;
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_ConfigureSleep()
 * @brief  configures the device for both DEEP_SLEEP and SLEEP modes, and on-wake mode
 *         i.e. before entering the sleep, the device should be programmed for TX or RX,
 *         then upon "waking up" the TX/RX settings will be preserved and the device can
 *         immediately perform the desired action TX/RX
 *         >> NOTE:
 *         e.g. Tag operation - after deep sleep, the device needs to just load the TX buffer and send the frame
 * @param  mode: the array and LDE code (OTP/ROM) and LDO tune, and set sleep persist
 *          - DWT_PRESRV_SLEEP 0x0100   preserve sleep
 *          - DWT_LOADOPSET    0x0080   load operating parameter set on wakeup
 *          - DWT_CONFIG       0x0040   download the AON array into the HIF (configuration download)
 *          - DWT_LOADEUI      0x0008
 *          - DWT_GOTORX       0x0002
 *          - DWT_TANDV        0x0001
 * @param  wake: wake up parameters
 *          - DWT_XTAL_EN      0x10   keep XTAL running during sleep
 *          - DWT_WAKE_SLPCNT  0x08   wake up after sleep count
 *          - DWT_WAKE_CS      0x04   wake up on chip select
 *          - DWT_WAKE_WK      0x02   wake up on WAKEUP PIN
 *          - DWT_SLP_EN       0x01   enable sleep/deep sleep functionality
 * @retval None
 */
void DWT_ConfigureSleep( uint16_t mode, uint8_t wake )
{
  // Add predefined sleep settings before writing the mode
  mode |= DW1000Local.sleepMode;
  DWT_WriteData16(DW1000_AON, DW1000_SUB_AON_WCFG, mode);

  DWT_WriteData8(DW1000_AON, DW1000_SUB_AON_CFG0, wake);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_EnterSleepAfterTx()
 * @brief  sets the auto TX to sleep bit. This means that after a frame
 *         transmission the device will enter deep sleep mode. The DWT_ConfigureSleep() function
 *         needs to be called before this to configure the on-wake settings
 *         >> NOTE:
 *         the IRQ line has to be low/inactive (i.e. no pending events)
 * @param  enable: 1 to configure the device to enter deep sleep after TX, 0 - disables the configuration
 * @retval None
 */
void DWT_EnterSleepAfterTx( uint8_t enable )
{
  uint32_t reg32 = DWT_ReadData32(DW1000_PMSC, DW1000_SUB_PMSC_CTRL1);

  // Set the auto TX -> sleep bit
  if (enable) {
    reg32 |= 0x00000800UL;
  }
  else {
    reg32 &= ~(0x00000800UL);
  }
  DWT_WriteData32(DW1000_PMSC, DW1000_SUB_PMSC_CTRL1, reg32);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_SpiCsWakeup()
 * @brief  wake up the device from sleep mode using the SPI read,
 *         the device will wake up on chip select line going low if the line is held low for at least 500us.
 *         To define the length depending on the time one wants to hold
 *         the chip select line low, use the following formula:
 *
 *         length (bytes) = time (s) * byte_rate (Hz)
 *
 *         where fastest byte_rate is spi_rate (Hz) / 8 if the SPI is sending the bytes back-to-back.
 *         To save time and power, a system designer could determine byte_rate value more precisely.
 *         >> NOTE:
 *         Alternatively the device can be waken up with WAKE_UP pin if configured for that operation
 * @param buffer: this is a pointer to the dummy buffer which will be used in the SPI read transaction used for the WAKE UP of the device
 * @param length: this is the length of the dummy buffer
 * @retval HAL_OK for success, or HAL_ERROR for error
 */
int8_t DWT_SpiCsWakeup( uint8_t *buffer, uint16_t length )
{
  // Device was in deep sleep (the first read fails)
  if (DWT_ReadDeviceID() != DW1000_DEVICE_ID) {

    // Need to keep chip select line low for at least 500us
    DWT_ReadData(DW1000_DEV_ID, 0x00, buffer, length); // Do a long read to wake up the chip (hold the chip select low)

    // Need 5ms for XTAL to start and stabilise (could wait for PLL lock IRQ status bit !!!)
    // NOTE: Polling of the STATUS register is not possible unless frequency is < 3MHz
    DWT_Delay(5);
  }
  else {
    return HAL_OK;
  }
  // DEBUG - check if still in sleep mode
  if (DWT_ReadDeviceID() != DW1000_DEVICE_ID) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/*! ------------------------------------------------------------------------------------------
 * @fn     _DWT_ConfigLDE()
 * @brief  configure LDE algorithm parameters
 * @param  prfIndex: this is the PRF index (0 or 1) 0 corresponds to 16 and 1 to 64 PRF
 * @retval None
 */
static void _DWT_ConfigLDE( uint8_t prfIndex )
{
  DWT_WriteData8(DW1000_LDE_CTRL, DW1000_SUB_LDE_CFG1, 0x6D);  // 8-bit configuration register

  if (prfIndex) {
    DWT_WriteData16(DW1000_LDE_CTRL, DW1000_SUB_LDE_CFG2, 0x0607);  // 16-bit LDE configuration tuning register
  }
  else {
    DWT_WriteData16(DW1000_LDE_CTRL, DW1000_SUB_LDE_CFG2, 0x1607);
  }
}

/*! ------------------------------------------------------------------------------------------
 * @fn     _DWT_LoadUCodeFromROM()
 * @brief  load ucode from OTP MEMORY or ROM
 * @param  None
 * @retval None
 */
static void _DWT_LoadUCodeFromROM( void )
{
  // Set up clocks
  _DWT_EnableClocks(FORCE_LDE);

  // Kick off the LDE load
  DWT_WriteData16(DW1000_OTP_IF, DW1000_SUB_OTP_CTRL, 0x8000);  // Set load LDE kick bit
  DWT_Delay(1); // Allow time for code to upload (should take up to 120 us)

  // Default clocks (ENABLE_ALL_SEQ)
  _DWT_EnableClocks(ENABLE_ALL_SEQ); // Enable clocks for sequencing
}

///*! ------------------------------------------------------------------------------------------
// * @fn     OPT_LoadOpsetTabFromOtp()
// * @brief  This is used to select which Operational Parameter Set table to load from OTP memory
// * @param  opsSel: Operational Parameter Set table to load:
// *          - DWT_OPSET_64LEN = 0x0 - load the operational parameter set table for 64 length preamble configuration
// *          - DWT_OPSET_TIGHT = 0x1 - load the operational parameter set table for tight xtal offsets (<1ppm)
// *          - DWT_OPSET_DEFLT = 0x2 - load the default operational parameter set table (this is loaded from reset)
// * @retval None
// */
//void OPT_LoadOpsetTabFromOtp( uint8_t opsSel )
//{
//  uint16_t reg16 = ((opsSel << 5) & 0x60) | 0x01; // Select defined OPS table and trigger its loading

//  // Set up clocks
//  _DWT_EnableClocks(FORCE_LDE);

//  DWT_WriteData16(DW1000_OTP_IF, DW1000_SUB_OTP_SF, reg16);

//  // Default clocks (ENABLE_ALL_SEQ)
//  _DWT_EnableClocks(ENABLE_ALL_SEQ); // Enable clocks for sequencing
//}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_SetSmartTxPower()
 * @brief  This call enables or disables the smart TX power feature.
 * @param  enable: this enables or disables the TX smart power (1 = enable, 0 = disable)
 * @retval None
 */
void DWT_SetSmartTxPower( uint8_t enable )
{
  // Config system register
  DW1000Local.sysCFGreg = DWT_ReadData32(DW1000_SYS_CFG, 0x00);  // Read sysconfig register

  // Disable smart power configuration
  if (enable) {
    DW1000Local.sysCFGreg &= ~(0x00040000UL);
  }
  else {
    DW1000Local.sysCFGreg |= 0x00040000UL;
  }

  DWT_WriteData32(DW1000_SYS_CFG, 0x00, DW1000Local.sysCFGreg);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_EnableAutoAck()
 * @brief  This call enables the auto-ACK feature. If the responseDelayTime (parameter) is 0, the ACK will be sent asap.
 *         otherwise it will be sent with a programmed delay (in symbols), max is 255.
 *         >> NOTE:
 *         needs to have frame filtering enabled as well
 * @param  responseDelayTime: if non-zero the ACK is sent after this delay, max is 255.
 * @retval None
 */
void DWT_EnableAutoAck( uint8_t responseDelayTime )
{
  // Set auto ACK reply delay
  DWT_WriteData8(DW1000_ACK_RESP_T, 0x03, responseDelayTime);   // In symbols

  // Enable auto ACK
  DW1000Local.sysCFGreg |= 0x40000000UL;
  DWT_WriteData32(DW1000_SYS_CFG, 0x00, DW1000Local.sysCFGreg) ;
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_SetDblRxBufMode()
 * @brief  This call enables the double receive buffer mode
 * @param  enable: 1 to enable, 0 to disable the double buffer mode
 * @retval None
 */
void DWT_SetDblRxBufMode( uint8_t enable )
{
  if (enable) {
    // Enable double RX buffer mode
    DW1000Local.sysCFGreg &= ~0x00001000UL;
    DW1000Local.dblbuffon = 1;
  }
  else {
    // Disable double RX buffer mode
    DW1000Local.sysCFGreg |= 0x00001000UL;
    DW1000Local.dblbuffon = 0;
  }

  DWT_WriteData32(DW1000_SYS_CFG, 0x00, DW1000Local.sysCFGreg);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_SetRxAfterTxDelay()
 * @brief  This sets the receiver turn on delay time after a transmission of a frame
 * @param  rxDelayTime: the delay is in UWB microseconds (20 bits)
 * @retval None
 */
void DWT_SetRxAfterTxDelay( uint32_t rxDelayTime )
{
  uint32_t tmp32 = DWT_ReadData32(DW1000_ACK_RESP_T, 0x00) ; // Read ACK_RESP_T_ID register

  // In UWB microseconds (e.g. turn the receiver on 20uus after TX)
  tmp32 = (tmp32 & 0xFFF00000UL) | (rxDelayTime & 0x000FFFFFUL);

  DWT_WriteData32(DW1000_ACK_RESP_T, 0x00, tmp32);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_SetCallbacks()
 * @brief  This function is used to register the different callbacks called when one of the corresponding event occurs.
 *         >> NOTE:
 *         Callbacks can be undefined (set to NULL). In this case, DWT_ISR() will process the event as usual
 *         but the 'null' callback will not be called.
 * @param cbTxDone: the pointer to the TX confirmation event callback function
 * @param cbRxOk: the pointer to the RX good frame event callback function
 * @param cbRxTo: the pointer to the RX timeout events callback function
 * @param cbRxErr: the pointer to the RX error events callback function
 * @retval None
 */
void DWT_SetCallbacks( dwt_cb_t cbTxDone, dwt_cb_t cbRxOk, dwt_cb_t cbRxTo, dwt_cb_t cbRxErr )
{
  DW1000Local.cbTxDone = cbTxDone;
  DW1000Local.cbRxOk   = cbRxOk;
  DW1000Local.cbRxTo   = cbRxTo;
  DW1000Local.cbRxErr  = cbRxErr;
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_SetSysStatus()
 * @brief  This function clear system status
 * @param  status: clear system status
 * @retval None
 */
void DWT_SetSysStatus( uint32_t status )
{
  DWT_WriteData32(DW1000_SYS_STATUS, 0x00, status);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_GetSysStatus()
 * @brief  This function get system status
 * @param  None
 * @retval system status
 */
uint32_t DWT_GetSysStatus( void )
{
  return DWT_ReadData32(DW1000_SYS_STATUS, 0x00);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_CheckIRQ()
 * @brief  This function checks if the IRQ line is active - this is used instead of interrupt handler
 * @param  None
 * @retval value is 1 if the IRQS bit is set and 0 otherwise
 */
uint8_t DWT_CheckIRQ( void )
{
  // Reading the lower byte only is enough for this operation
  return (DWT_ReadData8(DW1000_SYS_STATUS, 0x00) & SYS_STATUS_IRQS);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_ISR()
 * @brief  This is the DW1000's general Interrupt Service Routine. It will process/report the following events:
 *          - RXFCG (through cbRxOk callback)
 *          - TXFRS (through cbTxDone callback)
 *          - RXRFTO/RXPTO (through cbRxTo callback)
 *          - RXPHE/RXFCE/RXRFSL/RXSFDTO/AFFREJ/LDEERR (through cbRxTo cbRxErr)
 *         For all events, corresponding interrupts are cleared and necessary resets are performed. In addition, in the RXFCG case,
 *         received frame information and frame control are read before calling the callback. If double buffering is activated, it
 *         will also toggle between reception buffers once the reception callback processing has ended.
 *
 *         !!! This version of the ISR supports double buffering but does not support automatic RX re-enabling!
 *
 *         >> NOTE:
 *         In PC based system using (Cheetah or ARM) USB to SPI converter there can be no interrupts, however we still need something
 *         to take the place of it and operate in a polled way. In an embedded system this function should be configured to be triggered
 *         on any of the interrupts described above.
 * @param  None
 * @retval None
 */
void DWT_ISR( void )
{
  // Read status register low 32bits
  uint32_t status = DW1000Local.cbData.status = DWT_ReadData32(DW1000_SYS_STATUS, 0x00);

  // Handle RX good frame event
  if (status & SYS_STATUS_RXFCG) {
    uint16_t len;
    uint16_t tmp16;

    DWT_WriteData32(DW1000_SYS_STATUS, 0x00, SYS_STATUS_ALL_RX_GOOD); // Clear all receive status bits

    DW1000Local.cbData.rx_flags = 0;

    // Read frame info - Only the first two bytes of the register are used here.
    tmp16 = DWT_ReadData16(DW1000_RX_FINFO, 0x00);

    // Report frame length - Standard frame length up to 127, extended frame length up to 1023 bytes
    len = tmp16 & 0x000003FFUL;
    if (DW1000Local.longFrames == 0) {
      len &= 0x0000007FUL;
    }
    DW1000Local.cbData.datalength = len;

    // Report ranging bit
    if (tmp16 & 0x00008000UL) {
      DW1000Local.cbData.rx_flags |= 0x01;
    }

    // Report frame control - First bytes of the received frame.
    DWT_ReadData(DW1000_RX_BUFFER, 0x00, DW1000Local.cbData.fctrl, 2);

    // Because of a previous frame not being received properly, AAT bit can be set upon the proper reception of a frame not requesting for
    // acknowledgement (ACK frame is not actually sent though). If the AAT bit is set, check ACK request bit in frame control to confirm (this
    // implementation works only for IEEE802.15.4-2011 compliant frames).
    if ((status & SYS_STATUS_AAT) && ((DW1000Local.cbData.fctrl[0] & 0x20) == 0)) {
      DWT_WriteData32(DW1000_SYS_STATUS, 0x00, SYS_STATUS_AAT); // Clear AAT status bit in register
      DW1000Local.cbData.status &= ~SYS_STATUS_AAT; // Clear AAT status bit in callback data register copy
      DW1000Local.wait4resp = 0;
    }

    // Call the corresponding callback if present
    if (DW1000Local.cbRxOk != NULL) {
      DW1000Local.cbRxOk(&DW1000Local.cbData);
    }
    if (DW1000Local.dblbuffon) {
      // Toggle the Host side Receive Buffer Pointer
      DWT_WriteData8(DW1000_SYS_CTRL, 0x03, 1);
    }
  }

  // Handle TX confirmation event
  if (status & SYS_STATUS_TXFRS) {
    DWT_WriteData32(DW1000_SYS_STATUS, 0x00, SYS_STATUS_ALL_TX); // Clear TX event bits

    // In the case where this TXFRS interrupt is due to the automatic transmission of an ACK solicited by a response (with ACK request bit set)
    // that we receive through using wait4resp to a previous TX (and assuming that the IRQ processing of that TX has already been handled), then
    // we need to handle the IC issue which turns on the RX again in this situation (i.e. because it is wrongly applying the wait4resp after the
    // ACK TX).
    // See section "Transmit and automatically wait for response" in DW1000 User Manual
    if ((status & SYS_STATUS_AAT) && DW1000Local.wait4resp) {
      DWT_ForceTxRxOff(); // Turn the RX off
      DWT_RxReset(); // Reset in case we were late and a frame was already being received
    }

    // Call the corresponding callback if present
    if (DW1000Local.cbTxDone != NULL) {
      DW1000Local.cbTxDone(&DW1000Local.cbData);
    }
  }

  // Handle frame reception/preamble detect timeout events
  if (status & SYS_STATUS_ALL_RX_TO) {
    DWT_WriteData32(DW1000_SYS_STATUS, 0x00, SYS_STATUS_RXRFTO); // Clear RX timeout event bits

    DW1000Local.wait4resp = 0;

    // Because of an issue with receiver restart after error conditions, an RX reset must be applied after any error or timeout event to ensure
    // the next good frame's timestamp is computed correctly.
    // See section "RX Message timestamp" in DW1000 User Manual.
    DWT_ForceTxRxOff();
    DWT_RxReset();

    // Call the corresponding callback if present
    if (DW1000Local.cbRxTo != NULL) {
      DW1000Local.cbRxTo(&DW1000Local.cbData);
    }
  }

  // Handle RX errors events
  if (status & SYS_STATUS_ALL_RX_ERR) {
    DWT_WriteData32(DW1000_SYS_STATUS, 0x00, SYS_STATUS_ALL_RX_ERR); // Clear RX error event bits

    DW1000Local.wait4resp = 0;

    // Because of an issue with receiver restart after error conditions, an RX reset must be applied after any error or timeout event to ensure
    // the next good frame's timestamp is computed correctly.
    // See section "RX Message timestamp" in DW1000 User Manual.
    DWT_ForceTxRxOff();
    DWT_RxReset();

    // Call the corresponding callback if present
    if(DW1000Local.cbRxErr != NULL) {
      DW1000Local.cbRxErr(&DW1000Local.cbData);
    }
  }
}

///*! ------------------------------------------------------------------------------------------
// * @fn     DWT_LowPowerListenISR()
// * @brief  This is the DW1000's Interrupt Service Routine to use when low-power listening scheme is implemented. It will
// *         only process/report the RXFCG event (through cbRxOk callback).
// *         It clears RXFCG interrupt and reads received frame information and frame control before calling the callback.
// *
// *         !!! This version of the ISR is designed for single buffering case only!
// * @param  None
// * @retval None
// */
//void DWT_LowPowerListenISR( void )
//{
//  uint16_t len;
//  uint16_t tmp16;
//  uint32_t status = DWT_ReadData32(DW1000_SYS_STATUS, 0x00); // Read status register low 32bits

//  DW1000Local.cbData.status = status;

//  // The only interrupt handled when in low-power listening mode is RX good frame so proceed directly to the handling of the received frame.

//  // Deactivate low-power listening before clearing the interrupt. If not, the DW1000 will go back to sleep as soon as the interrupt is cleared.
//  DWT_SetLowPowerListening(0);

//  DWT_WriteData32(DW1000_SYS_STATUS, 0x00, SYS_STATUS_ALL_RX_GOOD); // Clear all receive status bits

//  DW1000Local.cbData.rx_flags = 0;

//  // Read frame info - Only the first two bytes of the register are used here.
//  tmp16 = DWT_ReadData16(DW1000_RX_FINFO, 0);

//  // Report frame length - Standard frame length up to 127, extended frame length up to 1023 bytes
//  len = tmp16 & 0x000003FFUL;
//  if (DW1000Local.longFrames == 0) {
//    len &= 0x0000007FUL;
//  }
//  DW1000Local.cbData.datalength = len;

//  // Report ranging bit
//  if (tmp16 & 0x00008000UL) {
//    DW1000Local.cbData.rx_flags |= 0x01;
//  }

//  // Report frame control - First bytes of the received frame.
//  DWT_ReadData(DW1000_RX_BUFFER, 0x00, DW1000Local.cbData.fctrl, 2);

//  // Because of a previous frame not being received properly, AAT bit can be set upon the proper reception of a frame not requesting for
//  // acknowledgement (ACK frame is not actually sent though). If the AAT bit is set, check ACK request bit in frame control to confirm (this
//  // implementation works only for IEEE802.15.4-2011 compliant frames).
//  // This issue is not documented at the time of writing this code. It should be in next release of DW1000 User Manual (v2.09, from July 2016).
//  if((status & SYS_STATUS_AAT) && ((DW1000Local.cbData.fctrl[0] & 0x20) == 0)) {
//    DWT_WriteData32(DW1000_SYS_STATUS, 0x00, SYS_STATUS_AAT);  // Clear AAT status bit in register
//    DW1000Local.cbData.status &= ~SYS_STATUS_AAT;  // Clear AAT status bit in callback data register copy
//    DW1000Local.wait4resp = 0;
//  }

//  // Call the corresponding callback if present
//  if (DW1000Local.cbRxOk != NULL) {
//    DW1000Local.cbRxOk(&DW1000Local.cbData);
//  }
//}

///*! ------------------------------------------------------------------------------------------
// * @fn     DWT_SetLeds()
// * @brief  This is used to set up Tx/Rx GPIOs which could be used to control LEDs
// *         >> Note:
// *         not completely IC dependent, also needs board with LEDS fitted on right I/O lines
// *         this function enables GPIOs 2 and 3 which are connected to LED3 and LED4 on EVB1000
// * @param  mode: this is a bit field interpreted as follows:
// *          - bit 0: 1 to enable LEDs, 0 to disable them
// *          - bit 1: 1 to make LEDs blink once on init. Only valid if bit 0 is set (enable LEDs)
// *          - bit 2 to 7: reserved
// * @retval None
// */
//void DWT_SetLeds( uint8_t mode )
//{
//  uint32_t tmp32;

//  if (mode & 0x01) {

//    // Set up MFIO for LED output.
//    tmp32 = DWT_ReadData32(DW1000_GPIO_CTRL, DW1000_SUB_GPIO_MODE);
//    tmp32 &= ~(GPIO_MSGP2_MASK | GPIO_MSGP3_MASK);
//    tmp32 |= (GPIO_PIN2_RXLED | GPIO_PIN3_TXLED);
//    DWT_WriteData32(DW1000_GPIO_CTRL, DW1000_SUB_GPIO_MODE, tmp32);

//    // Enable LP Oscillator to run from counter and turn on de-bounce clock.
//    tmp32 = DWT_ReadData32(DW1000_PMSC, PMSC_CTRL0_OFFSET);
//    tmp32 |= (PMSC_CTRL0_GPDCE | PMSC_CTRL0_KHZCLEN);
//    DWT_WriteData32(DW1000_PMSC, DW1000_SUB_PMSC_CTRL0, tmp32);

//    // Enable LEDs to blink and set default blink time.
//    tmp32 = PMSC_LEDC_BLNKEN | PMSC_LEDC_BLINK_TIME_DEF;
//    // Make LEDs blink once if requested.
//    if (mode & DWT_LEDS_INIT_BLINK) {
//      tmp32 |= PMSC_LEDC_BLINK_NOW_ALL;
//    }
//    DWT_WriteData32(DW1000_PMSC, PMSC_LEDC_OFFSET, tmp32);
//    // Clear force blink bits if needed.
//    if(mode & DWT_LEDS_INIT_BLINK) {
//      tmp32 &= ~PMSC_LEDC_BLINK_NOW_ALL;
//      DWT_WriteData32(DW1000_PMSC, PMSC_LEDC_OFFSET, tmp32);
//    }
//  }
//  else {
//    // Clear the GPIO bits that are used for LED control.
//    tmp32 = DWT_ReadData32(DW1000_GPIO_CTRL, GPIO_MODE_OFFSET);
//    tmp32 &= ~(GPIO_MSGP2_MASK | GPIO_MSGP3_MASK);
//    DWT_WriteData32(DW1000_GPIO_CTRL, GPIO_MODE_OFFSET, tmp32);
//  }
//}

/*! ------------------------------------------------------------------------------------------
 * @fn     _DWT_EnableClocks()
 * @brief  function to enable/disable clocks to particular digital blocks/system
 * @param  clocks: set of clocks to enable/disable
 * @retval None
 */
static void _DWT_EnableClocks( uint8_t clocks )
{
  uint8_t reg[2];

  DWT_ReadData(DW1000_PMSC, DW1000_SUB_PMSC_CTRL0, reg, 2);
  switch (clocks) {
    case ENABLE_ALL_SEQ: {
      reg[0] = 0x00 ;
      reg[1] = reg[1] & 0xFE;
      break;
    }
    case FORCE_SYS_XTI: {
      // System and RX
      reg[0] = 0x01 | (reg[0] & 0xFC);
      break;
    }
    case FORCE_SYS_PLL: {
      // System
      reg[0] = 0x02 | (reg[0] & 0xFC);
      break;
    }
    case READ_ACC_ON: {
      reg[0] = 0x48 | (reg[0] & 0xB3);
      reg[1] = 0x80 | reg[1];
      break;
    }
    case READ_ACC_OFF: {
      reg[0] = reg[0] & 0xB3;
      reg[1] = 0x7F & reg[1];
      break;
    }
    case FORCE_OTP_ON: {
      reg[1] = 0x02 | reg[1];
      break;
    }
    case FORCE_OTP_OFF: {
      reg[1] = reg[1] & 0xFD;
      break;
    }
    case FORCE_TX_PLL: {
      reg[0] = 0x20 | (reg[0] & 0xCF);
      break;
    }
    case FORCE_LDE: {
      reg[0] = 0x01;
      reg[1] = 0x03;
      break;
    }
    default:
      break;
  }

  // Need to write lower byte separately before setting the higher byte(s)
  DWT_WriteData(DW1000_PMSC, DW1000_SUB_PMSC_CTRL0, &reg[0], 1);
  DWT_WriteData(DW1000_PMSC, DW1000_SUB_PMSC_CTRL0 + 0x01, &reg[1], 1);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     _DWT_DisableSequencing()
 * @brief  This function disables the TX blocks sequencing, it disables PMSC control of RF blocks, system clock is also set to XTAL
 * @param  None
 * @retval None
 */
static void _DWT_DisableSequencing( void )
{
  _DWT_EnableClocks(FORCE_SYS_XTI);  // Set system clock to XTI
  DWT_WriteData16(DW1000_PMSC, DW1000_SUB_PMSC_CTRL1, 0x00);  // Disable PMSC ctrl of RF and RX clk blocks
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_SetDelayedTxRxTime()
 * @brief  This API function configures the delayed transmit time or the delayed RX on time
 * @param  starttime: the TX/RX start time (the 32 bits should be the high 32 bits of the system
 *                    time at which to send the message, or at which to turn on the receiver)
 * @retval None
 */
void DWT_SetDelayedTxRxTime( uint32_t startTime )
{
  // Write at offset 1 as the lower 9 bits of this register are ignored
  DWT_WriteData32(DW1000_DX_TIME, 0x01, startTime);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_StartTx()
 * @brief  This call initiates the transmission, input parameter indicates which TX mode is used see below
 * @param  mode:
 *          - DWT_TX_IMMEDIATE                  0 immediate TX (no response expected)
 *          - DWT_TX_DELAYED                    1 delayed TX   (no response expected)
 *          - DWT_TX_IMMEDIATE | DWT_RESPONSE   2 immediate TX (response expected - so the receiver will be automatically turned on after TX is done)
 *          - DWT_TX_DELAYED   | DWT_RESPONSE   3 delayed TX   (response expected - so the receiver will be automatically turned on after TX is done)
 * @retval HAL_OK for success, or HAL_ERROR for error
 *         (e.g. a delayed transmission will fail if the delayed time has passed)
 */
int8_t DWT_StartTx( uint8_t mode )
{
  int8_t status = HAL_OK;
  uint8_t tmp8 = 0x00;
  uint16_t checkTxOK = 0;

  if (mode & DWT_RESPONSE) {
    tmp8 = 0x80 ; // Set wait4response bit
    DWT_WriteData8(DW1000_SYS_CTRL, 0x00, tmp8);
    DW1000Local.wait4resp = 1;
  }

  if (mode & DWT_TX_DELAYED) {
    // Both SYS_CTRL_TXSTRT and SYS_CTRL_TXDLYS to correctly enable TX
    tmp8 |= (0x06);
    DWT_WriteData8(DW1000_SYS_CTRL, 0x00, tmp8);
    checkTxOK = DWT_ReadData16(DW1000_SYS_STATUS, 3); // Read at offset 3 to get the upper 2 bytes out of 5

    // Transmit Delayed Send set over Half a Period away or Power Up error (there is enough time to send but not to power up individual blocks).
    if ((checkTxOK & SYS_STATUS_TXERR) == 0) {
      status = HAL_OK ; // All okay
    }
    else {
      // I am taking DSHP set to Indicate that the TXDLYS was set too late for the specified DX_TIME.
      // Remedial Action - (a) cancel delayed send
      // This assumes the bit is in the lowest byte
      DWT_WriteData8(DW1000_SYS_CTRL, 0x00, 0x40);
      // Note event Delayed TX Time too Late
      // Could fall through to start a normal send (below) just sending late.....
      // ... instead return and assume return value of 1 will be used to detect and recover from the issue.
      DW1000Local.wait4resp = 0;
      status = HAL_ERROR ; // Failed !
    }
  }
  else {
    tmp8 |= 0x02;
    DWT_WriteData8(DW1000_SYS_CTRL, 0x00, tmp8);
  }

  return status;
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_ForceTxRxOff()
 * @brief  This is used to turn off the transceiver
 * @param  None
 * @retval None
 */
void DWT_ForceTxRxOff( void )
{
  uint32_t mask;

  mask = DWT_ReadData32(DW1000_SYS_MASK, 0x00); // Read set interrupt mask

  // Need to beware of interrupts occurring in the middle of following read modify write cycle
  // We can disable the radio, but before the status is cleared an interrupt can be set (e.g. the
  // event has just happened before the radio was disabled)
  // thus we need to disable interrupt during this operation
  DWT_DisableIRQ();

  DWT_WriteData32(DW1000_SYS_MASK, 0x00, 0); // Clear interrupt mask - so we don't get any unwanted events

  DWT_WriteData8(DW1000_SYS_CTRL, 0x00, 0x40); // Disable the radio

  // Forcing Transceiver off - so we do not want to see any new events that may have happened
  DWT_WriteData32(DW1000_SYS_STATUS, 0x00, (SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_GOOD));

  DWT_SyncRxBufPtrs();

  DWT_WriteData32(DW1000_SYS_MASK, 0x00, mask); // Set interrupt mask to what it was

  // Enable/restore interrupts again...
  DWT_EnableIRQ();

  DW1000Local.wait4resp = 0;
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_SyncRxBufPtrs()
 * @brief  this function synchronizes rx buffer pointers
 *         need to make sure that the host/IC buffer pointers are aligned before starting RX
 * @param  None
 * @retval None
 */
void DWT_SyncRxBufPtrs( void )
{
  uint8_t tmp8;

  // Need to make sure that the host/IC buffer pointers are aligned before starting RX
  tmp8 = DWT_ReadData8(DW1000_SYS_STATUS, 0x03);  // Read 1 byte at offset 3 to get the 4th byte out of 5

  if ((tmp8 & (SYS_STATUS_ICRBP >> 24)) !=        // IC side Receive Buffer Pointer
     ((tmp8 & (SYS_STATUS_HSRBP>>24)) << 1) ) {   // Host Side Receive Buffer Pointer
    // We need to swap RX buffer status reg (write one to toggle internally)
    DWT_WriteData8(DW1000_SYS_CTRL, 0x03, 0x01);
  }
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_SetSniffMode()
 * @brief  enable/disable and configure SNIFF mode.
 *         SNIFF mode is a low-power reception mode where the receiver is sequenced on and off instead of being on all the time.
 *         The time spent in each state (on/off) is specified through the parameters below.
 *         See DW1000 User Manual section 4.5 "Low-Power SNIFF mode" for more details.
 * @param  enable: 1 to enable SNIFF mode, 0 to disable. When 0, all other parameters are not taken into account.
 * @param  timeOn: duration of receiver ON phase, expressed in multiples of PAC size. The counter automatically adds 1 PAC
 *                 size to the value set. Min value that can be set is 1 (i.e. an ON time of 2 PAC size), max value is 15.
 * @param  timeOff: duration of receiver OFF phase, expressed in multiples of 128/125 ֳ (~1 ֳ). Max value is 255.
 * @retval None
 */
void DWT_SetSniffMode( uint8_t enable, uint8_t timeOn, uint8_t timeOff )
{
  uint16_t tmp16;
  uint32_t tmp32;

  if (enable) {
    /* Configure ON/OFF times and enable PLL2 on/off sequencing by SNIFF mode. */
    tmp16 = ((timeOff << 8) | timeOn) & 0xFF0FUL;
    DWT_WriteData16(DW1000_RX_SNIFF, 0x00, tmp16);
    tmp32 = DWT_ReadData32(DW1000_PMSC, DW1000_SUB_PMSC_CTRL0);
    tmp32 |= 0x01000000UL;
    DWT_WriteData32(DW1000_PMSC, DW1000_SUB_PMSC_CTRL0, tmp32);
  }
  else {
    /* Clear ON/OFF times and disable PLL2 on/off sequencing by SNIFF mode. */
    DWT_WriteData16(DW1000_RX_SNIFF, 0x00, 0x0000);
    tmp32 = DWT_ReadData32(DW1000_PMSC, DW1000_SUB_PMSC_CTRL0);
    tmp32 &= ~0x01000000UL;
    DWT_WriteData32(DW1000_PMSC, DW1000_SUB_PMSC_CTRL0, tmp32);
  }
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_SetLowPowerListening()
 * @brief  enable/disable low-power listening mode.
 *
 *         Low-power listening is a feature whereby the DW1000 is predominantly in the SLEEP state but wakes periodically, (after
 *         this "long sleep"), for a very short time to sample the air for a preamble sequence. This preamble sampling "listening"
 *         phase is actually two reception phases separated by a "short sleep" time. See DW1000 User Manual section "Low-Power
 *         Listening" for more details.
 *
 *         >> NOTE:
 *         Before enabling low-power listening, the following functions have to be called to fully configure it:
 *          - DWT_ConfigureSleep() to configure long sleep phase. "mode" parameter should at least have DWT_PRESRV_SLEEP,
 *           DWT_CONFIG and DWT_RX_EN set and "wake" parameter should at least have both DWT_WAKE_SLPCNT and DWT_SLP_EN set.
 *          - DWT_CalibRateSleepCnt() and DWT_ConfigureSleepCnt() to define the "long sleep" phase duration.
 *          - DWT_SetSnoozeTime() to define the "short sleep" phase duration.
 *          - DWT_SetPreambleDetectTimeout() to define the reception phases duration.
 *          - DWT_SetInterrupt() to activate RX good frame interrupt (DWT_INT_RFCG) only.
 *         When configured, low-power listening mode can be triggered either by putting the DW1000 to sleep (using
 *         DWT_EnterSleep()) or by activating reception (using DWT_RxEnable()).
 *
 *         Please refer to the low-power listening examples (examples 8a/8b accompanying the API distribution on Decawave's
 *         website). They form a working example code that shows how to use low-power listening correctly.
 * @param  enable: 1 to enable low-power listening, 0 to disable.
 * @retval None
 */
void DWT_SetLowPowerListening( uint8_t enable )
{
  uint32_t tmp32 = DWT_ReadData32(DW1000_PMSC, DW1000_SUB_PMSC_CTRL1);

  if (enable) {
    /* Configure RX to sleep and snooze features. */
    tmp32 |= (0x00003000UL);
  }
  else {
    /* Reset RX to sleep and snooze features. */
    tmp32 &= ~(0x00003000UL);
  }
  DWT_WriteData32(DW1000_PMSC, DW1000_SUB_PMSC_CTRL1, tmp32);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_SetSnoozeTime()
 * @brief  Set duration of "short sleep" phase when in low-power listening mode.
 * @param  snoozeTime: "short sleep" phase duration, expressed in multiples of 512/19.2 ֳ (~26.7 ֳ). The counter
 *                     automatically adds 1 to the value set. The smallest working value that should be set is 1,
 *                     i.e. giving a snooze time of 2 units (or ~53 ֳ).
 * @retval None
 */
void DWT_SetSnoozeTime( uint8_t snoozeTime )
{
  DWT_WriteData8(DW1000_PMSC, DW1000_SUB_PMSC_SNOZT, snoozeTime);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_RxEnable()
 * @brief  This call turns on the receiver, can be immediate or delayed (depending on the mode parameter). In the case of a
 *         "late" error the receiver will only be turned on if the DWT_IDLE_ON_DLY_ERR is not set.
 *         The receiver will stay turned on, listening to any messages until
 *         it either receives a good frame, an error (CRC, PHY header, Reed Solomon) or  it times out (SFD, Preamble or Frame).
 * @param  mode: this can be one of the following allowed values:
 *          - DWT_RX_IMMEDIATE                      0 used to enbale receiver immediately
 *          - DWT_RX_DELAYED                        1 used to set up delayed RX, if "late" error triggers, then the RX will be enabled immediately
 *          - DWT_RX_DELAYED | DWT_IDLE_ON_DLY_ERR  3 used to disable re-enabling of receiver if delayed RX failed due to "late" error
 *          - DWT_RX_IMMEDIATE | DWT_NO_SYNC_PTRS   4 used to re-enable RX without trying to sync IC and host side buffer pointers, typically when
 *                                                          performing manual RX re-enabling in double buffering mode
 * @retval HAL_OK for success, or HAL_ERROR for error (e.g. a delayed receive enable will be too far in the future if delayed time has passed)
 */
int8_t DWT_RxEnable( uint8_t mode )
{
  uint8_t tmp8;
  uint16_t tmp16;

  if ((mode & DWT_NO_SYNC_PTRS) == 0) {
    DWT_SyncRxBufPtrs();
  }

  tmp16 = 0x0100UL ;

  if (mode & DWT_RX_DELAYED) {
    tmp16 |= 0x0200UL;
  }

  DWT_WriteData16(DW1000_SYS_CTRL, 0x00, tmp16);

  if (mode & DWT_RX_DELAYED) {  // check for errors
    tmp8 = DWT_ReadData8(DW1000_SYS_STATUS, 0x03); // Read 1 byte at offset 3 to get the 4th byte out of 5
    if ((tmp8 & (SYS_STATUS_HPDWARN >> 24)) != 0) { // if delay has passed do immediate RX on unless DWT_IDLE_ON_DLY_ERR is true
      DWT_ForceTxRxOff(); // turn the delayed receive off
      if ((mode & DWT_IDLE_ON_DLY_ERR) == 0) { // if DWT_IDLE_ON_DLY_ERR not set then re-enable receiver
        DWT_WriteData16(DW1000_SYS_CTRL, 0x00, 0x0100UL);
      }
      return HAL_ERROR; // return warning indication
    }
  }

  return HAL_OK;
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_SetRxTimeout()
 * @brief  This call enables RX timeout (SY_STAT_RFTO event)
 * @param  time: how long the receiver remains on from the RX enable command
 *               The time parameter used here is in 1.0256 us (512/499.2MHz) units
 *               If set to 0 the timeout is disabled.
 * @retval None
 */
void DWT_SetRxTimeout( uint16_t time )
{
  uint8_t tmp8;

  tmp8 = DWT_ReadData8(DW1000_SYS_CFG, 0x03); // Read at offset 3 to get the upper byte only

  if (time > 0) {
    DWT_WriteData16(DW1000_RX_FWTO, 0x00, time);

    tmp8 |= (uint8_t)(0x10000000UL >> 24); // Shift RXWTOE mask as we read the upper byte only
    // OR in 32bit value (1 bit set), I know this is in high byte.
    DW1000Local.sysCFGreg |= 0x10000000UL;
  }
  else {
    tmp8 &= ~((uint8_t)(0x10000000UL >> 24)); // Shift RXWTOE mask as we read the upper byte only
    // AND in inverted 32bit value (1 bit clear), I know this is in high byte.
    DW1000Local.sysCFGreg &= ~(0x10000000UL);
  }

  DWT_WriteData8(DW1000_SYS_CFG, 0x03, tmp8); // Write at offset 3 to write the upper byte only
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_SetPreambleDetectTimeout()
 * @brief  This call enables preamble timeout (SY_STAT_RXPTO event)
 * @param  timeout: Preamble detection timeout, expressed in multiples of PAC size. The counter automatically adds 1 PAC
 *                  size to the value set. Min value that can be set is 1 (i.e. a timeout of 2 PAC size).
 * @retval None
 */
void DWT_SetPreambleDetectTimeout( uint16_t timeout )
{
  DWT_WriteData16(DW1000_DRX_CONF, DW1000_SUB_DRX_PRETOC, timeout);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_SetInterrupt()
 * @brief  This function enables the specified events to trigger an interrupt.
 *         The following events can be enabled:
 *          - DWT_INT_TFRS    0x00000080    // frame sent
 *          - DWT_INT_RFCG    0x00004000    // frame received with good CRC
 *          - DWT_INT_RPHE    0x00001000    // receiver PHY header error
 *          - DWT_INT_RFCE    0x00008000    // receiver CRC error
 *          - DWT_INT_RFSL    0x00010000    // receiver sync loss error
 *          - DWT_INT_RFTO    0x00020000    // frame wait timeout
 *          - DWT_INT_RXPTO   0x00200000    // preamble detect timeout
 *          - DWT_INT_SFDT    0x04000000    // SFD timeout
 *          - DWT_INT_ARFE    0x20000000    // frame rejected (due to frame filtering configuration)
 * @param  bitmask: sets the events which will generate interrupt
 * @param  enable: if set the interrupts are enabled else they are cleared
 * @retval None
 */
void DWT_SetInterrupt( uint32_t bitmask, uint8_t enable )
{
  uint32_t mask;

  // Need to beware of interrupts occurring in the middle of following read modify write cycle
  DWT_DisableIRQ();

  mask = DWT_ReadData32(DW1000_SYS_MASK, 0x00) ; // Read register

  if(enable) {
    mask |= bitmask;
  }
  else {
    mask &= ~bitmask; // Clear the bit
  }
  DWT_WriteData32(DW1000_SYS_MASK, 0x00, mask) ; // New value

  DWT_EnableIRQ();
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_ConfigEventCounters()
 * @brief  This is used to enable/disable the event counter in the IC
 * @param  enable: 1 enables (and reset), 0 disables the event counters
 * @retval None
 */
void DWT_ConfigEventCounters( uint8_t enable )
{
  // Need to clear and disable, can't just clear
  DWT_WriteData8(DW1000_DIG_DIAG, DW1000_SUB_EVC_CTRL, 0x02);

  if (enable) {
    DWT_WriteData8(DW1000_DIG_DIAG, DW1000_SUB_EVC_CTRL, 0x01); // Enable
  }
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_ReadEventCounters()
 * @brief  This is used to read the event counters in the IC
 * @param  counters: pointer to the dwt_deviceentcnts_t structure which will hold the read data
 * @retval None
 */
void DWT_ReadEventCounters( dwt_deviceentcnts_t *counters )
{
  uint32_t tmp32;

  tmp32 = DWT_ReadData32(DW1000_DIG_DIAG, DW1000_SUB_EVC_PHE);  // Read sync loss (31-16), PHE (15-0)
  counters->PHE =  tmp32 & 0xFFF;
  counters->RSL = (tmp32 >> 16) & 0xFFF;

  tmp32 = DWT_ReadData32(DW1000_DIG_DIAG, DW1000_SUB_EVC_FCG);  // Read CRC bad (31-16), CRC good (15-0)
  counters->CRCG =  tmp32 & 0xFFF;
  counters->CRCB = (tmp32 >> 16) & 0xFFF;

  tmp32 = DWT_ReadData32(DW1000_DIG_DIAG, DW1000_SUB_EVC_FFR);  // Overruns (31-16), address errors (15-0)
  counters->ARFE =  tmp32 & 0xFFF;
  counters->OVER = (tmp32 >> 16) & 0xFFF;

  tmp32 = DWT_ReadData32(DW1000_DIG_DIAG, DW1000_SUB_EVC_STO);  // Read PTO (31-16), SFDTO (15-0)
  counters->PTO   = (tmp32 >> 16) & 0xFFF;
  counters->SFDTO =  tmp32 & 0xFFF;

  tmp32 = DWT_ReadData32(DW1000_DIG_DIAG, DW1000_SUB_EVC_FWTO); // Read RX TO (31-16), TXFRAME (15-0)
  counters->TXF = (tmp32 >> 16) & 0xFFF;
  counters->RTO =  tmp32 & 0xFFF;

  tmp32 = DWT_ReadData32(DW1000_DIG_DIAG, DW1000_SUB_EVC_HPW);  // Read half period warning events
  counters->HPW =  tmp32 & 0xFFF;
  counters->TXW = (tmp32 >> 16) & 0xFFF;                        // Power-up warning events
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_RxReset()
 * @brief  this function resets the receiver of the DW1000
 * @param  None
 * @retval None
 */
void DWT_RxReset( void )
{
  // Set RX reset
  DWT_WriteData8(DW1000_PMSC, 0x03, 0xE0);

  // Clear RX reset
  DWT_WriteData8(DW1000_PMSC, 0x03, 0xF0);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_SoftwareReset()
 * @brief  this function resets the DW1000
 * @param  None
 * @retval None
 */
void DWT_SoftwareReset( void )
{
  _DWT_DisableSequencing();

  // Clear any AON auto download bits (as reset will trigger AON download)
  DWT_WriteData16(DW1000_AON, DW1000_SUB_AON_WCFG, 0x00);
  // Clear the wake-up configuration
  DWT_WriteData8(DW1000_AON, DW1000_SUB_AON_CFG0, 0x00);
  // Upload the new configuration
  _DWT_AONArrayUpload();

  // Reset HIF, TX, RX and PMSC
  DWT_WriteData8(DW1000_PMSC, DW1000_SUB_PMSC_CTRL0 + 0x03, 0x00);

  // DW1000 needs a 10us sleep to let clk PLL lock after reset - the PLL will automatically lock after the reset
  // Could also have polled the PLL lock flag, but then the SPI needs to be < 3MHz !! So a simple delay is easier
  DWT_Delay(1);

  // Clear reset
  DWT_WriteData8(DW1000_PMSC, DW1000_SUB_PMSC_CTRL0 + 0x03, 0xF0);

  DW1000Local.wait4resp = 0;
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_SetXtalTrim()
 * @brief  This is used to adjust the crystal frequency
 * @param  xtal: crystal trim value (in range 0x0 to 0x1F) 31 steps (~1.5ppm per step)
 * @retval None
 */
void DWT_SetXtalTrim( uint8_t xtal )
{
  // The 3 MSb in this 8-bit register must be kept to 0b011 to avoid any malfunction.
  uint8_t tmp8 = (3 << 5) | (xtal & 0x1F);
  DWT_WriteData8(DW1000_FS_CTRL, DW1000_SUB_FS_XTALT, tmp8);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_ConfigCWMode()
 * @brief  this function sets the DW1000 to transmit cw signal at specific channel frequency
 * @param  channel: specifies the operating channel (e.g. 1, 2, 3, 4, 5, 6 or 7)
 * @retval None
 */
void DWT_ConfigCWMode( uint8_t channel )
{
  // Disable TX/RX RF block sequencing (needed for cw frame mode)
  _DWT_DisableSequencing();

  // Config RF pll (for a given channel)
  // Configure PLL2/RF PLL block CFG/TUNE
  DWT_WriteData32(DW1000_FS_CTRL, DW1000_SUB_FS_PLLCFG, fs_pll_cfg[chan_idx[channel]]);
  DWT_WriteData8(DW1000_FS_CTRL, DW1000_SUB_FS_PLLTUNE, fs_pll_tune[chan_idx[channel]]);

  // PLL wont be enabled until a TX/RX enable is issued later on
  // Configure RF TX blocks (for specified channel and prf)
  // Config RF TX control
  DWT_WriteData32(DW1000_RF_CONF, DW1000_SUB_RF_TXCTRL, tx_config[chan_idx[channel]]);

  // Enable RF PLL
  DWT_WriteData32(DW1000_RF_CONF, 0x00, RF_CONF_TXPLLPOWEN); // Enable LDO and RF PLL blocks
  DWT_WriteData32(DW1000_RF_CONF, 0x00, RF_CONF_TXALLEN); // Enable the rest of TX blocks

  // Configure TX clocks
  DWT_WriteData8(DW1000_PMSC, DW1000_SUB_PMSC_CTRL0, 0x22);
  DWT_WriteData8(DW1000_PMSC, 0x1, 0x07);

  // Disable fine grain TX sequencing
  DWT_SetFineGrainTxSeq(0);

  // Configure CW mode
  DWT_WriteData8(DW1000_TX_CAL, DW1000_SUB_TC_PGTEST, 0x13);
}

/*! ------------------------------------------------------------------------------------------
 * @fn     DWT_ConfigContinuousFrameMode()
 * @brief  this function sets the DW1000 to continuous tx frame mode for regulatory approvals testing.
 * @param  frameRepetitionRate: This is a 32-bit value that is used to set the interval between transmissions.
 *                              The minimum value is 4. The units are approximately 8 ns.
 *                              (or more precisely 512/(499.2e6*128) seconds)).
 * @retval None
 */
void DWT_ConfigContinuousFrameMode( uint32_t frameRepetitionRate )
{
  // Disable TX/RX RF block sequencing (needed for continuous frame mode)
  _DWT_DisableSequencing();

  // Enable RF PLL and TX blocks
  DWT_WriteData32(DW1000_RF_CONF, 0x00, RF_CONF_TXPLLPOWEN); // Enable LDO and RF PLL blocks
  DWT_WriteData32(DW1000_RF_CONF, 0x00, RF_CONF_TXALLEN);    // Enable the rest of TX blocks

  // Configure TX clocks
  _DWT_EnableClocks(FORCE_SYS_PLL);
  _DWT_EnableClocks(FORCE_TX_PLL);

  // Set the frame repetition rate
  if (frameRepetitionRate < 4) {
    frameRepetitionRate = 4;
  }
  DWT_WriteData32(DW1000_DX_TIME, 0x00, frameRepetitionRate);

  // Configure continuous frame TX
  DWT_WriteData8(DW1000_DIG_DIAG, DW1000_SUB_DIAG_TMC, 0x10); // Turn the tx power spectrum test mode - continuous sending of frames
}

///*! ------------------------------------------------------------------------------------------
// * @fn     DWT_ReadTempVbat()
// * @brief  this function reads the battery voltage and temperature of the MP
// *         The values read here will be the current values sampled by DW1000 AtoD converters.
// *         Note on Temperature: the temperature value needs to be converted to give the real temperature
// *         the formula is: 1.13 * reading - 113.0
// *         Note on Voltage: the voltage value needs to be converted to give the real voltage
// *         the formula is: 0.0057 * reading + 2.3
// *
// *         NB: To correctly read the temperature this read should be done with xtal clock
// *         however that means that the receiver will be switched off, if receiver needs to be on then
// *         the timer is used to make sure the value is stable before reading
// * @param  fastSPI: set to 1 if SPI rate > than 3MHz is used
// * @retval (temp_raw << 8)|(vbat_raw)
// */
//uint16_t DWT_ReadTempVbat( uint8_t fastSPI )
//{
//  uint8_t writebuf[2];
//  uint8_t vbatRaw;
//  uint8_t tempRaw;

//  // These writes should be single writes and in sequence
//  writebuf[0] = 0x80; // Enable TLD Bias
//  DWT_WriteData(DW1000_RF_CONF, 0x11, writebuf, 1);

//  writebuf[0] = 0x0A; // Enable TLD Bias and ADC Bias
//  DWT_WriteData(DW1000_RF_CONF, 0x12, writebuf, 1);

//  writebuf[0] = 0x0f; // Enable Outputs (only after Biases are up and running)
//  DWT_WriteData(DW1000_RF_CONF, 0x12, writebuf, 1);    //

//  // Reading All SAR inputs
//  writebuf[0] = 0x00;
//  DWT_WriteData(DW1000_TX_CAL, DW1000_SUB_TC_SARC, writebuf, 1);
//  writebuf[0] = 0x01; // Set SAR enable
//  DWT_WriteData(DW1000_TX_CAL, DW1000_SUB_TC_SARC, writebuf, 1);

//  if (fastSPI == 1) {
//    DWT_Delay(1); // If using PLL clocks(and fast SPI rate) then this sleep is needed
//    // Read voltage and temperature.
//    DWT_ReadData(DW1000_TX_CAL, DW1000_SUB_TC_SARL, writebuf, 2);
//  }
//  else { //change to a slow clock
//    _DWT_EnableClocks(FORCE_SYS_XTI); // NOTE: set system clock to XTI - this is necessary to make sure the values read are reliable
//    // Read voltage and temperature.
//    DWT_ReadData(DW1000_TX_CAL, DW1000_SUB_TC_SARL, writebuf, 2);
//    // Default clocks (ENABLE_ALL_SEQ)
//    _DWT_EnableClocks(ENABLE_ALL_SEQ); // Enable clocks for sequencing
//  }

//  vbatRaw = writebuf[0];
//  tempRaw = writebuf[1];

//  writebuf[0] = 0x00; // Clear SAR enable
//  DWT_WriteData(DW1000_TX_CAL, DW1000_SUB_TC_SARC, writebuf, 1);

//  return ((tempRaw << 8) | (vbatRaw));
//}

///*! ------------------------------------------------------------------------------------------
// * @fn     DWT_ReadWakeupTemp()
// * @brief  this function reads the temperature of the DW1000 that was sampled
// *         on waking from Sleep/Deepsleep. They are not current values, but read on last
// *         wakeup if DWT_TANDV bit is set in mode parameter of DWT_ConfigureSleep
// * @param  None
// * @retval 8-bit raw temperature sensor value
// */
//uint8_t DWT_ReadWakeupTemp( void )
//{
//  return DWT_ReadData8(DW1000_TX_CAL, DW1000_SUB_TC_SARL + 0x01);
//}

///*! ------------------------------------------------------------------------------------------
// * @fn     DWT_ReadWakeupVbat()
// * @brief  this function reads the battery voltage of the DW1000 that was sampled
// *         on waking from Sleep/Deepsleep. They are not current values, but read on last
// *         wakeup if DWT_TANDV bit is set in mode parameter of DWT_ConfigureSleep
// * @param  None
// * @retval 8-bit raw battery voltage sensor value
// */
//uint8_t DWT_ReadWakeupVbat( void )
//{
//  return DWT_ReadData8(DW1000_TX_CAL, DW1000_SUB_TC_SARL);
//}

/*************************************** END OF FILE ****************************************/
