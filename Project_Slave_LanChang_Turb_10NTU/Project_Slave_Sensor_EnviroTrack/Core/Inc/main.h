/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define SPEAKERS_Pin GPIO_PIN_13
#define SPEAKERS_GPIO_Port GPIOC
#define TOGGLE_RESET_Pin GPIO_PIN_0
#define TOGGLE_RESET_GPIO_Port GPIOC
#define BUTTON_PRESS_4_Pin GPIO_PIN_5
#define BUTTON_PRESS_4_GPIO_Port GPIOC
#define BUTTON_PRESS_3_Pin GPIO_PIN_0
#define BUTTON_PRESS_3_GPIO_Port GPIOB
#define BUTTON_PRESS_2_Pin GPIO_PIN_1
#define BUTTON_PRESS_2_GPIO_Port GPIOB
#define BUTTON_PRESS_1_Pin GPIO_PIN_2
#define BUTTON_PRESS_1_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_6
#define LCD_CS_GPIO_Port GPIOC
#define LCD_RST_Pin GPIO_PIN_7
#define LCD_RST_GPIO_Port GPIOC
#define LCD_A0_Pin GPIO_PIN_8
#define LCD_A0_GPIO_Port GPIOC
#define LCD_RW_Pin GPIO_PIN_9
#define LCD_RW_GPIO_Port GPIOC
#define LCD_RD_E_Pin GPIO_PIN_8
#define LCD_RD_E_GPIO_Port GPIOA
#define LCD_C86_Pin GPIO_PIN_9
#define LCD_C86_GPIO_Port GPIOA
#define LCD_PS_Pin GPIO_PIN_10
#define LCD_PS_GPIO_Port GPIOA
#define MCU_RL1_Pin GPIO_PIN_11
#define MCU_RL1_GPIO_Port GPIOA
#define MCU_RL2_Pin GPIO_PIN_12
#define MCU_RL2_GPIO_Port GPIOA
#define RS485_2_TXDE_Pin GPIO_PIN_12
#define RS485_2_TXDE_GPIO_Port GPIOC
#define ON_PW_SEN2_Pin GPIO_PIN_2
#define ON_PW_SEN2_GPIO_Port GPIOD
#define ON_PW_SEN1_Pin GPIO_PIN_3
#define ON_PW_SEN1_GPIO_Port GPIOB
#define ON_PW_RS485_Pin GPIO_PIN_4
#define ON_PW_RS485_GPIO_Port GPIOB
#define RS485_1_TXDE_Pin GPIO_PIN_5
#define RS485_1_TXDE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
