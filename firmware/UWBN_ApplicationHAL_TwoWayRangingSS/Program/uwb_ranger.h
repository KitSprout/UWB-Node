/**
 *      __            ____
 *     / /__ _  __   / __/                      __  
 *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
 *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
 *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
 *                    /_/   github.com/KitSprout    
 * 
 *  @file    uwb_ranger.h
 *  @author  KitSprout
 *  @date    19-Aug-2017
 *  @brief   
 * 
 */

/* Define to prevent recursive inclusion ---------------------------------------------------*/
#ifndef __UWB_RANGER_H
#define __UWB_RANGER_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes --------------------------------------------------------------------------------*/
#include "modules\dw1000.h"

/* Exported types --------------------------------------------------------------------------*/
/* Exported constants ----------------------------------------------------------------------*/

/* Default antenna delay values for 64 MHz PRF. */
#define TX_ANT_DLY    16436
#define RX_ANT_DLY    16436

/* Exported functions ----------------------------------------------------------------------*/
void UWB_Init( DWT_ConfigTypeDef *pConfig );

#ifdef __cplusplus
}
#endif

#endif

/*************************************** END OF FILE ****************************************/
