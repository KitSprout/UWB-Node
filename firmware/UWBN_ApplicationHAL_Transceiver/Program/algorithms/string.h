/**
 *      __            ____
 *     / /__ _  __   / __/                      __  
 *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
 *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
 *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
 *                    /_/   github.com/KitSprout    
 * 
 *  @file    string.h
 *  @author  KitSprout
 *  @date    01-Aug-2017
 *  @brief   
 * 
 */

/* Define to prevent recursive inclusion ---------------------------------------------------*/
#ifndef __STRING_H
#define __STRING_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes --------------------------------------------------------------------------------*/
#include <string.h>

#include "core_cm4.h"

/* Exported types --------------------------------------------------------------------------*/
typedef enum {
  KS_BIN   = 2,   /* unsigned binary      */
  KS_OCT   = 8,   /* unsigned octal       */
  KS_DEC   = 10,  /* unsigned decimal     */
  KS_HEX   = 16,  /* unsigned hexadecimal */
  KS_INT   = 0,   /* signed decimal       */
  KS_FLOAT = 1,   /* float point          */
} KSStringType_t;

/* Exported constants ----------------------------------------------------------------------*/
/* Exported functions ----------------------------------------------------------------------*/  
void      num2Str( KSStringType_t type, uint8_t lens, char *pStr, int32_t number );
uint32_t  lenOfStr( char *pStr );

#ifdef __cplusplus
}
#endif

#endif

/*************************************** END OF FILE ****************************************/
