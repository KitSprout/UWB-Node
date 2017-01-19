/**
  *      __            ____
  *     / /__ _  __   / __/                      __  
  *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
  *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
  *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
  *                    /_/   github.com/KitSprout    
  * 
  * @file    boardConfig.h
  * @author  KitSprout
  * @date    12-Dec-2016
  * @brief   
  * 
  */

/* Define to prevent recursive inclusion ---------------------------------------------------*/
#ifndef __BOARDCONFIG_H
#define __BOARDCONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes --------------------------------------------------------------------------------*/
/* Exported types --------------------------------------------------------------------------*/
/* Exported constants ----------------------------------------------------------------------*/

#define KS_HW_BOARD_NAME              "UWBNode"
#define KS_HW_MCU_NAME                "STM32F411xE"

#define KS_HW_CLOCK_SOUCE_HSE
//#define KS_HW_USE_CLOCK_SOUCE_HSI

//#define KS_HW_UART_HAL_LIBRARY
#define KS_HW_SPI_HAL_LIBRARY
//#define KS_HW_I2C_HAL_LIBRARY

#define __SAMPLE_RATE_100Hz_

#ifdef __SAMPLE_RATE_100Hz_
  #define SAMPLE_RATE                 ((uint16_t)100)       // 100Hz
  #define SAMPLE_TIME                 ((float32_t)0.01f)    // 10ms
#endif

#ifdef __SAMPLE_RATE_200Hz_
  #define SAMPLE_RATE                 ((uint16_t)200)       // 200Hz
  #define SAMPLE_TIME                 ((float32_t)0.005f)   // 5ms
#endif

#ifdef __SAMPLE_RATE_400Hz_
  #define SAMPLE_RATE                 ((uint16_t)400)       // 400Hz
  #define SAMPLE_TIME                 ((float32_t)0.0025f)  // 2.5ms
#endif

#ifdef __SAMPLE_RATE_500Hz_
  #define SAMPLE_RATE                 ((uint16_t)500)       // 500Hz
  #define SAMPLE_TIME                 ((float32_t)0.002f)   // 2.0ms
#endif

#ifdef __SAMPLE_RATE_1KHz_
  #define SAMPLE_RATE                 ((uint16_t)1000)      // 1KHz
  #define SAMPLE_TIME                 ((float32_t)0.001f)   // 1.0ms
#endif

/* -------- LED and KEY */
#define LED_R_PIN                     GPIO_PIN_15
#define LED_R_GPIO_PORT               GPIOC
#define LED_R_Set()                   __GPIO_SET(LED_R_GPIO_PORT, LED_R_PIN)
#define LED_R_Reset()                 __GPIO_RST(LED_R_GPIO_PORT, LED_R_PIN)
#define LED_R_Toggle()                __GPIO_TOG(LED_R_GPIO_PORT, LED_R_PIN)

#define LED_G_PIN                     GPIO_PIN_14
#define LED_G_GPIO_PORT               GPIOC
#define LED_G_Set()                   __GPIO_SET(LED_G_GPIO_PORT, LED_G_PIN)
#define LED_G_Reset()                 __GPIO_RST(LED_G_GPIO_PORT, LED_G_PIN)
#define LED_G_Toggle()                __GPIO_TOG(LED_G_GPIO_PORT, LED_G_PIN)

#define LED_B_PIN                     GPIO_PIN_13
#define LED_B_GPIO_PORT               GPIOC
#define LED_B_Set()                   __GPIO_SET(LED_B_GPIO_PORT, LED_B_PIN)
#define LED_B_Reset()                 __GPIO_RST(LED_B_GPIO_PORT, LED_B_PIN)
#define LED_B_Toggle()                __GPIO_TOG(LED_B_GPIO_PORT, LED_B_PIN)

#define KEY_PIN                       GPIO_PIN_2
#define KEY_GPIO_PORT                 GPIOB
#define KEY_Read()                    (__GPIO_READ(KEY_GPIO_PORT, KEY_PIN) == KEY_PIN)

/* -------- TIM Timer */
#define TIMER2                        TIM2
#define TIMER2_CLK_ENABLE()           __HAL_RCC_TIM2_CLK_ENABLE()
#define TIMER2_IRQn                   TIM2_IRQn

/* -------- UART Serial */
#define SERIAL_MAX_TXBUF              16
#define SERIAL_MAX_RXBUF              16

#define SERIAL_UARTx                  USART1
#define SERIAL_UARTx_CLK_ENABLE()     __HAL_RCC_USART1_CLK_ENABLE()
#define SERIAL_UARTx_IRQn             USART1_IRQn
#define SERIAL_UARTx_IRQn_PREEMPT     0x0F
#define SERIAL_UARTx_IRQn_SUB         1

#define SERIAL_UARTx_FORCE_RESET()    __HAL_RCC_USART1_FORCE_RESET()
#define SERIAL_UARTx_RELEASE_RESET()  __HAL_RCC_USART1_RELEASE_RESET()

#define SERIAL_TX_PIN                 GPIO_PIN_6
#define SERIAL_TX_GPIO_PORT           GPIOB
#define SERIAL_TX_AF                  GPIO_AF7_USART1

#define SERIAL_RX_PIN                 GPIO_PIN_7
#define SERIAL_RX_GPIO_PORT           GPIOB
#define SERIAL_RX_AF                  GPIO_AF7_USART1

#define SERIAL_BAUDRATE               115200
#define SERIAL_BYTESIZE               UART_WORDLENGTH_8B
#define SERIAL_STOPBITS               UART_STOPBITS_1
#define SERIAL_PARITY                 UART_PARITY_NONE
#define SERIAL_HARDWARECTRL           UART_HWCONTROL_NONE
#define SERIAL_MODE                   UART_MODE_TX_RX
#define SERIAL_OVERSAMPLE             UART_OVERSAMPLING_16

/* -------- Inertial Sensor */

#define IMU_MAX_TXBUF                 32
#define IMU_MAX_RXBUF                 32

#define IMU_SPIx                      SPI2
#define IMU_SPIx_CLK_ENABLE()         __HAL_RCC_SPI2_CLK_ENABLE()
#define IMU_SPIx_IRQn                 SPI2_IRQn
#define IMU_SPIx_IRQn_PREEMPT         0x0F
#define IMU_SPIx_IRQn_SUB             1
#define IMU_SPIx_FORCE_RESET()        __HAL_RCC_SPI2_FORCE_RESET()
#define IMU_SPIx_RELEASE_RESET()      __HAL_RCC_SPI2_RELEASE_RESET()

#define IMU_SPIx_SPEED_HIGH           SPI_BAUDRATEPRESCALER_2
#define IMU_SPIx_SPEED_LOW            SPI_BAUDRATEPRESCALER_256

#define IMU_SCK_PIN                   GPIO_PIN_13
#define IMU_SCK_GPIO_PORT             GPIOB
#define IMU_SCK_AF                    GPIO_AF5_SPI2

#define IMU_SDO_PIN                   GPIO_PIN_14
#define IMU_SDO_GPIO_PORT             GPIOB
#define IMU_SDO_AF                    GPIO_AF5_SPI2

#define IMU_SDI_PIN                   GPIO_PIN_15
#define IMU_SDI_GPIO_PORT             GPIOB
#define IMU_SDI_AF                    GPIO_AF5_SPI2

#define IMU_CSM_PIN                   GPIO_PIN_12
#define IMU_CSM_GPIO_PORT             GPIOB
#define IMU_CSM_H()                   __GPIO_SET(IMU_CSM_GPIO_PORT, IMU_CSM_PIN)
#define IMU_CSM_L()                   __GPIO_RST(IMU_CSM_GPIO_PORT, IMU_CSM_PIN)

#define IMU_CSB_PIN                   GPIO_PIN_8
#define IMU_CSB_GPIO_PORT             GPIOA
#define IMU_CSB_H()                   __GPIO_SET(IMU_CSB_GPIO_PORT, IMU_CSB_PIN)
#define IMU_CSB_L()                   __GPIO_RST(IMU_CSB_GPIO_PORT, IMU_CSB_PIN)

#define IMU_INTM_PIN                  GPIO_Pin_0
#define IMU_INTM_GPIO_PORT            GPIOB
#define IMU_INTM                      EXTI0_IRQn

#define IMU_INTB_PIN                  GPIO_Pin_10
#define IMU_INTB_GPIO_PORT            GPIOB
#define IMU_INTB                      EXTI15_10_IRQn

/* Exported functions ----------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif

/*************************************** END OF FILE ****************************************/
