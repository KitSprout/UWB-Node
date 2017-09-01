/**
 *      __            ____
 *     / /__ _  __   / __/                      __  
 *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
 *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
 *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
 *                    /_/   github.com/KitSprout    
 * 
 *  @file    uwb_ranger.c
 *  @author  KitSprout
 *  @date    19-Aug-2017
 *  @brief   
 * 
 */
 
/* Includes --------------------------------------------------------------------------------*/
#include "drivers\stm32f4_system.h"
#include "uwb_ranger.h"

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
  /* Init DW1000 */
  DWT_SetSpeedLow();
  DWT_Initialise(pConfig);
  DWT_SetSpeedHigh();

  /* Configure DW1000.
   * >> NOTE:
   * In a real application, for optimum performance within regulatory limits, it may be
   * necessary to set TX pulse bandwidth and TX power, (using the dwt_configuretxrf API
   * call) to per device calibrated values saved in the target system or the DW1000
   * OTP memory. */
  DWT_Configure(pConfig);

  /* Apply default antenna delay value.
   * >> NOTE:
   * The sum of the values is the TX to RX antenna delay, experimentally determined by a
   * calibration process. Here we use a hard coded typical value but, in a real application,
   * each device should have its own antenna delay properly calibrated to get the best
   * possible precision when performing range measurements. */
  DWT_SetRxAntennaDelay(RX_ANT_DLY);
  DWT_SetTxAntennaDelay(TX_ANT_DLY);
}

/*************************************** END OF FILE ****************************************/
