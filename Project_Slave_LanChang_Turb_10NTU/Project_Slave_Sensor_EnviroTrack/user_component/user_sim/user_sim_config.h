


#ifndef  _USER_SIM_CONFIG_H
#define  _USER_SIM_CONFIG_H


#include "user_util.h"
#include "main.h"
#include "usart.h"


/*
    * Uart danh cho Sim
        + Baurate
        + Port uart
        ...
*/
extern DMA_HandleTypeDef            hdma_uart5_rx;

#define uart_sim	                huart5
#define uart_dma_sim                hdma_uart5_rx

#define MX_UART_SIM_Init            MX_UART5_Init            //Func init Uart sim

/* Definition for USARTx's NVIC */
#define USART_SIM_IRQn              UART5_IRQn
#define USART_SIM_Priority          1

/*-- Rx uart: DMA with IDLE line | Interupt line  ----*/
#define UART_SIM_DMA      0
#define UART_SIM_IT       1

#define UART_SIM_MODE     UART_SIM_DMA


/*
    * transparent mode or not
*/
//#define USING_TRANSPARENT_MODE     


/*
    * Protocol
*/
#define MQTT_PROTOCOL           1
//#define HTTP_PROTOCOL           1
//#define HTTPS_PROTOCOL          1












#endif