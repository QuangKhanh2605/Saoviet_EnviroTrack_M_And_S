
#ifndef _USER_RS485_
#define _USER_RS485_    _USER_RS485_H

#include "user_util.h"
#include "main.h"
#include "usart.h"
#include "user_app_wm.h"

/*======================== Define ======================*/
/*
    * Uart danh cho 485 so 1
        + Baurate
        + Port uart
        ...
*/
extern DMA_HandleTypeDef            hdma_lpuart_rx;

#define uart_rs485	                hlpuart1
#define uart_dma_rs485              hdma_lpuart_rx

#define MX_UART_RS485_Init          MX_LPUART1_UART_Init            //Func init Uart

/* Definition for USARTx's NVIC */
#define USART_RS485_IRQn            LPUART1_IRQn
#define USART_RS485_Priority        1

/*-- Rx uart: DMA with IDLE line | Interupt line  ----*/
#define UART_RS485_DMA              0
#define UART_RS485_IT               1

#define UART_RS485_MODE             UART_RS485_DMA

#define DE_GPIO_PORT                RS485_1_TXDE_GPIO_Port  
#define DE_GPIO_PIN                 RS485_1_TXDE_Pin     

/*
    * Uart danh cho 485 so 1
        + Baurate
        + Port uart
        ...
*/
extern DMA_HandleTypeDef            hdma_usart3_rx;

#define uart_rs485_2	            huart3
#define uart_dma_rs485_2            hdma_usart3_rx

#define MX_UART_RS485_2_Init        MX_USART3_UART_Init            //Func init Uart

/* Definition for USARTx's NVIC */
#define USART_RS485_2_IRQn          USART3_IRQn
#define USART_RS485_2_Priority      1

/*-- Rx uart: DMA with IDLE line | Interupt line  ----*/
#define UART_RS485_2_DMA            0
#define UART_RS485_2_IT             1

#define UART_RS485_2_MODE           UART_RS485_2_DMA

#define DE_2_GPIO_PORT              RS485_2_TXDE_GPIO_Port  
#define DE_2_GPIO_PIN               RS485_2_TXDE_Pin     


/*-------------------------------*/
#define RS485_POWER_PORT    RS485_ON_OFF_GPIO_Port     //thay doi chan nay thanh chan on/off 485
#define RS485_POWER_PIN     RS485_ON_OFF_Pin    

#define RS485_ON        HAL_GPIO_WritePin(RS485_POWER_PORT, RS485_POWER_PIN, GPIO_PIN_RESET);
#define RS485_OFF       HAL_GPIO_WritePin(RS485_POWER_PORT, RS485_POWER_PIN, GPIO_PIN_SET);


extern uint8_t Rs485Status_u8;
extern uint8_t Rs485_2Status_u8;

/*======================== Funcion ======================*/
void RS485_Init_Data (void);
void RS485_Init_Uart (void);
void RS485_Init_RX_Mode(void);
void RS485_Stop_RX_Mode (void);
void RS485_Rx_Callback (uint16_t Size);
void RS485_Modbus_Read_Value (uint8_t SlaveID, uint16_t Addr, uint16_t nRegis);


uint8_t RS485_Modbus_Check_Format (uint8_t SlaveID, uint16_t nRegis, 
                                   sData *pSource, sData *Content);


void RS485_2_Init_Data (void);
void RS485_2_Init_Uart (void);
void RS485_2_Init_RX_Mode(void);
void RS485_2_Stop_RX_Mode (void);

void RS485_2_Modbus_Read_Value (uint8_t SlaveID, uint16_t Addr, uint16_t nRegis);
void RS485_2_Rx_Callback (uint16_t Size);

#endif






