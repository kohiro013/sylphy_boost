/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SWITCH_Pin LL_GPIO_PIN_13
#define SWITCH_GPIO_Port GPIOC
#define LED_BLUE_Pin LL_GPIO_PIN_0
#define LED_BLUE_GPIO_Port GPIOH
#define LED_GREEN_Pin LL_GPIO_PIN_1
#define LED_GREEN_GPIO_Port GPIOH
#define SEN_FL_Pin LL_GPIO_PIN_0
#define SEN_FL_GPIO_Port GPIOA
#define SEN_SL_Pin LL_GPIO_PIN_1
#define SEN_SL_GPIO_Port GPIOA
#define SEN_SR_Pin LL_GPIO_PIN_2
#define SEN_SR_GPIO_Port GPIOA
#define SEN_FR_Pin LL_GPIO_PIN_3
#define SEN_FR_GPIO_Port GPIOA
#define LED_FL_Pin LL_GPIO_PIN_4
#define LED_FL_GPIO_Port GPIOA
#define LED_SL_Pin LL_GPIO_PIN_5
#define LED_SL_GPIO_Port GPIOA
#define LED_SR_Pin LL_GPIO_PIN_6
#define LED_SR_GPIO_Port GPIOA
#define LED_FR_Pin LL_GPIO_PIN_7
#define LED_FR_GPIO_Port GPIOA
#define V_BAT_Pin LL_GPIO_PIN_0
#define V_BAT_GPIO_Port GPIOB
#define V_DRIVE_Pin LL_GPIO_PIN_1
#define V_DRIVE_GPIO_Port GPIOB
#define SPI2_CS0_Pin LL_GPIO_PIN_12
#define SPI2_CS0_GPIO_Port GPIOB
#define SPI2_CS1_Pin LL_GPIO_PIN_8
#define SPI2_CS1_GPIO_Port GPIOA
#define LED_RED_Pin LL_GPIO_PIN_11
#define LED_RED_GPIO_Port GPIOA
#define LED_YELLOW_Pin LL_GPIO_PIN_12
#define LED_YELLOW_GPIO_Port GPIOA
#define SPI1_CS0_Pin LL_GPIO_PIN_15
#define SPI1_CS0_GPIO_Port GPIOA
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
