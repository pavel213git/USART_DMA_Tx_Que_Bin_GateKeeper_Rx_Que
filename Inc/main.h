/**
  ******************************************************************************
  * @file    Examples_LL/USART/USART_Communication_TxRx_DMA/Inc/main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_adc.h"      
#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
      
#include "adc_test.h"      

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Virtual Com Port use is enabled 
       USART2 instance is used. (TX on PA.02, RX on PA.03)
       (please ensure that USART communication between the target MCU and ST-LINK MCU is properly enabled 
       on HW board in order to support Virtual Com Port)
*/



/**
  * @brief LED2 
  */
#define LED2_PIN                           LL_GPIO_PIN_5
#define LED2_GPIO_PORT                     GPIOA
#define LED2_GPIO_CLK_ENABLE()             LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA)

/**
  * @brief Toggle periods for various blinking modes
  */
#define LED_BLINK_FAST  200
#define LED_BLINK_SLOW  500
#define LED_BLINK_ERROR 1000


/**
  * @brief Key push-button
  */
#define USER_BUTTON_PIN                         LL_GPIO_PIN_13
#define USER_BUTTON_GPIO_PORT                   GPIOC
#define USER_BUTTON_GPIO_CLK_ENABLE()           LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC)   
#define USER_BUTTON_EXTI_LINE                   LL_EXTI_LINE_13
#define USER_BUTTON_EXTI_IRQn                   EXTI15_10_IRQn
#define USER_BUTTON_EXTI_LINE_ENABLE()          LL_EXTI_EnableIT_0_31(USER_BUTTON_EXTI_LINE)   
#define USER_BUTTON_EXTI_FALLING_TRIG_ENABLE()  LL_EXTI_EnableFallingTrig_0_31(USER_BUTTON_EXTI_LINE)   
#define USER_BUTTON_SYSCFG_SET_EXTI()           do {                                                                     \
                                                  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);                  \
                                                  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTC, LL_GPIO_AF_EXTI_LINE13);  \
                                                } while(0)
#define USER_BUTTON_IRQHANDLER                  EXTI15_10_IRQHandler
                                                    
#define DMA_HALF_TRANSFER_UART (uint8_t)(1 << 0)   
#define DMA_FULL_TRANSFER_UART (uint8_t)(1 << 1)   
#define DMA_HALF_RECEIVE_UART (uint8_t)(1 << 2)   
#define DMA_FULL_RECEIVE_UART (uint8_t)(1 << 3)   
#define DMA_HALF_TRANSFER_ADC (uint8_t)(1 << 4)   
#define DMA_FULL_TRANSFER_ADC (uint8_t)(1 << 5)   


/* Functions prototypes------------------------------------------------------- */                                                    
void vTaskFunction1( void *pvParameters );
void vTaskFunction2( void *pvParameters );
void vPrintString( const char *pcString );
void initTask( void *pvParameters );
void prvStdioGatekeeperTask( void *pvParameters );                                                    
void AdcProcessTask(void *pvParameters );

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
extern void     LED_On(void);
extern void     LED_Blinking(uint32_t Period);
extern void     LED_Off(void);                                                    
/* IRQ Handler treatment functions */
void DMA1_TransmitHalf_Callback(void);                                                    
void DMA1_TransmitComplete_Callback(void);
void DMA1_ReceiveComplete_Callback(void);
void USART_TransferError_Callback(void);
void UserButton_Callback(void);

/* Exported variables ------------------------------------------------------------*/
extern SemaphoreHandle_t xBinSemaADC;
extern __IO uint8_t flag_DMA;
extern uint16_t adc_raw_data[ADC_CONVERTED_DATA_BUFFER_SIZE];
#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
