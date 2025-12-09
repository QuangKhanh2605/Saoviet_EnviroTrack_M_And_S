#ifndef USER_CONNECT_SENSOR_H__
#define USER_CONNECT_SENSOR_H__

#define USING_CONNECT_SENSOR

#include "user_util.h"
#include "event_driven.h"

/*==================Define UART sensor===================*/
extern DMA_HandleTypeDef              hdma_usart3_rx;

#define uart_rs485SS	              huart3
#define uart_dma_rs485SS              hdma_usart3_rx

#define MX_UART_RS485SS_Init          MX_USART3_UART_Init            //Func init Uart

/* Definition for USARTx's NVIC */
#define USART_RS485SS_IRQn            USART3_IRQn
#define USART_RS485SS_Priority        1

/*-- Rx uart: DMA with IDLE line | Interupt line  ----*/
#define UART_RS485SS_DMA              0
#define UART_RS485SS_IT               1

#define UART_RS485SS_MODE             UART_RS485SS_IT

#define SENSOR_DE_GPIO_PORT                RS485_2_TXDE_GPIO_Port  
#define SENSOR_DE_GPIO_PIN                 RS485_2_TXDE_Pin    

/*======================Function======================*/
void RS485SS_Init_Data (void);
void RS485SS_Init_Uart (void);
void RS485SS_Init_RX_Mode(void);
void RS485SS_Stop_RX_Mode (void);
void RS485SS_Rx_Callback (uint16_t Size);


#endif
