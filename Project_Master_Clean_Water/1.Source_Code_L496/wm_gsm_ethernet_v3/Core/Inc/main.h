/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32l4xx_hal.h"

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
#define SIM_RESET_Pin GPIO_PIN_2
#define SIM_RESET_GPIO_Port GPIOE
#define BUTTON_1_Pin GPIO_PIN_3
#define BUTTON_1_GPIO_Port GPIOE
#define VBAT_DETECT_Pin GPIO_PIN_6
#define VBAT_DETECT_GPIO_Port GPIOE
#define VBAT_DETECT_EXTI_IRQn EXTI9_5_IRQn
#define V_PIN_Pin GPIO_PIN_1
#define V_PIN_GPIO_Port GPIOA
#define V_IN_Pin GPIO_PIN_4
#define V_IN_GPIO_Port GPIOA
#define PRESURE_IN1_Pin GPIO_PIN_5
#define PRESURE_IN1_GPIO_Port GPIOA
#define PRESURE_IN2_Pin GPIO_PIN_6
#define PRESURE_IN2_GPIO_Port GPIOA
#define PRESURE_IN3_Pin GPIO_PIN_7
#define PRESURE_IN3_GPIO_Port GPIOA
#define PRESURE_IN4_Pin GPIO_PIN_4
#define PRESURE_IN4_GPIO_Port GPIOC
#define PRESURE_IN5_Pin GPIO_PIN_5
#define PRESURE_IN5_GPIO_Port GPIOC
#define PRESURE_IN6_Pin GPIO_PIN_0
#define PRESURE_IN6_GPIO_Port GPIOB
#define ON_OFF_BOOT_Pin GPIO_PIN_8
#define ON_OFF_BOOT_GPIO_Port GPIOE
#define HARD_DETECT_Pin GPIO_PIN_9
#define HARD_DETECT_GPIO_Port GPIOE
#define PULSE1_DTC_Pin GPIO_PIN_10
#define PULSE1_DTC_GPIO_Port GPIOE
#define PULSE1_DIR_Pin GPIO_PIN_11
#define PULSE1_DIR_GPIO_Port GPIOE
#define PULSE1_DIR_EXTI_IRQn EXTI15_10_IRQn
#define PULSE2_DTC_Pin GPIO_PIN_12
#define PULSE2_DTC_GPIO_Port GPIOE
#define PULSE2_DIR_Pin GPIO_PIN_13
#define PULSE2_DIR_GPIO_Port GPIOE
#define PULSE2_DIR_EXTI_IRQn EXTI15_10_IRQn
#define RS485_1_TXDE_Pin GPIO_PIN_14
#define RS485_1_TXDE_GPIO_Port GPIOE
#define RS485_ON_OFF_Pin GPIO_PIN_15
#define RS485_ON_OFF_GPIO_Port GPIOE
#define RS485_1_RX_Pin GPIO_PIN_10
#define RS485_1_RX_GPIO_Port GPIOB
#define RS485_1_TX_Pin GPIO_PIN_11
#define RS485_1_TX_GPIO_Port GPIOB
#define LCD_DETECT_Pin GPIO_PIN_12
#define LCD_DETECT_GPIO_Port GPIOB
#define RS485_2_TXDE_Pin GPIO_PIN_14
#define RS485_2_TXDE_GPIO_Port GPIOB
#define RS485_2_TX_Pin GPIO_PIN_8
#define RS485_2_TX_GPIO_Port GPIOD
#define RS485_2_RX_Pin GPIO_PIN_9
#define RS485_2_RX_GPIO_Port GPIOD
#define LCD_ON_OFF_Pin GPIO_PIN_10
#define LCD_ON_OFF_GPIO_Port GPIOD
#define LCD_CS_Pin GPIO_PIN_11
#define LCD_CS_GPIO_Port GPIOD
#define LCD_RST_Pin GPIO_PIN_12
#define LCD_RST_GPIO_Port GPIOD
#define LCD_RW_Pin GPIO_PIN_13
#define LCD_RW_GPIO_Port GPIOD
#define LCD_C86_Pin GPIO_PIN_14
#define LCD_C86_GPIO_Port GPIOD
#define LCD_A0_Pin GPIO_PIN_15
#define LCD_A0_GPIO_Port GPIOD
#define BUTTON_PRESS_1_Pin GPIO_PIN_6
#define BUTTON_PRESS_1_GPIO_Port GPIOC
#define BUTTON_PRESS_2_Pin GPIO_PIN_7
#define BUTTON_PRESS_2_GPIO_Port GPIOC
#define BUTTON_PRESS_3_Pin GPIO_PIN_8
#define BUTTON_PRESS_3_GPIO_Port GPIOC
#define BUTTON_PRESS_4_Pin GPIO_PIN_9
#define BUTTON_PRESS_4_GPIO_Port GPIOC
#define Magnetic_switch_Pin GPIO_PIN_8
#define Magnetic_switch_GPIO_Port GPIOA
#define Magnetic_switch_EXTI_IRQn EXTI9_5_IRQn
#define DEBUG_TX_Pin GPIO_PIN_9
#define DEBUG_TX_GPIO_Port GPIOA
#define DEBUG_RX_Pin GPIO_PIN_10
#define DEBUG_RX_GPIO_Port GPIOA
#define ETH_ON_OFF_Pin GPIO_PIN_15
#define ETH_ON_OFF_GPIO_Port GPIOA
#define ETH_RSTn_Pin GPIO_PIN_10
#define ETH_RSTn_GPIO_Port GPIOC
#define ETH_SCSn_Pin GPIO_PIN_11
#define ETH_SCSn_GPIO_Port GPIOC
#define SIM_TX_Pin GPIO_PIN_12
#define SIM_TX_GPIO_Port GPIOC
#define ETH_INTn_Pin GPIO_PIN_0
#define ETH_INTn_GPIO_Port GPIOD
#define ETH_INTn_EXTI_IRQn EXTI0_IRQn
#define SIM_RX_Pin GPIO_PIN_2
#define SIM_RX_GPIO_Port GPIOD
#define LED_Pin GPIO_PIN_4
#define LED_GPIO_Port GPIOD
#define SIM_ON_OFF_Pin GPIO_PIN_6
#define SIM_ON_OFF_GPIO_Port GPIOB
#define SIM_DTR_Pin GPIO_PIN_7
#define SIM_DTR_GPIO_Port GPIOB
#define SIM_PWR_KEY_Pin GPIO_PIN_0
#define SIM_PWR_KEY_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
