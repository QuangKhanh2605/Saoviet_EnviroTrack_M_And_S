/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file    usart_if.c
* @author  MCD Application Team
* @brief   Configuration of UART driver interface for hyperterminal communication
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under Ultimate Liberty license
* SLA0044, the "License"; You may not use this file except in compliance with
* the License. You may obtain a copy of the License at:
*                             www.st.com/SLA0044
*
******************************************************************************
*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stm32_usart_if.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/**
* @brief DMA handle
*/
extern DMA_HandleTypeDef hdma_usart2_tx;

/**
* @brief UART handle
*/
extern UART_HandleTypeDef huart3;

/**
* @brief buffer to receive 1 character
*/
uint8_t charRx;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/**
* @brief Trace driver callbacks handler
*/
const UTIL_ADV_TRACE_Driver_s UTIL_TraceDriver =
{
    vcom_Init,
    vcom_DeInit,
    //  vcom_ReceiveInit,
    vcom_Trace_DMA,
};

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
* @brief  TX complete callback
* @return none
*/
static void (*TxCpltCallback)(void *);
/**
* @brief  RX complete callback
* @param  rxChar ptr of chars buffer sent by user
* @param  size buffer size
* @param  error errorcode
* @return none
*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Exported functions --------------------------------------------------------*/

UTIL_ADV_TRACE_Status_t vcom_Init(void (*cb)(void *))
{
    TxCpltCallback = cb;
//    MX_USART3_UART_Init();
    return UTIL_ADV_TRACE_OK;
}

UTIL_ADV_TRACE_Status_t vcom_DeInit(void)
{
    __HAL_RCC_USART3_FORCE_RESET();
    __HAL_RCC_USART3_RELEASE_RESET();

    HAL_UART_MspDeInit(&huart3);
    
    return UTIL_ADV_TRACE_OK;
}

void vcom_Trace(uint8_t *p_data, uint16_t size)
{
    if (pFunc_UTIL_Debug_Send != NULL)
        pFunc_UTIL_Debug_Send(p_data, size);
}

UTIL_ADV_TRACE_Status_t vcom_Trace_DMA(uint8_t *p_data, uint16_t size)
{    
    if (pFunc_UTIL_Debug_Send != NULL)
        pFunc_UTIL_Debug_Send(p_data, size);

    TxCpltCallback(NULL);
    
    return UTIL_ADV_TRACE_OK;
}

void vcom_Resume(void)
{
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
