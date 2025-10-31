/*
 * myUart.c
 *
 *  Created on: Dec 7, 2021
 *      Author: lenovo
 */

#include "user_uart.h"
#include "user_sim.h"
/*==================var struct======================*/


volatile static uint8_t IndexUartError  = 0;
static uint16_t UartDebugCountFail = 0;

/*==================Function======================*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == uart_sim.Instance)
	{
        Sim_Rx_Callback(1);     
	} 

	if (huart->Instance == uart_debug.Instance)
	{
        AppComm_DBG_Rx_Callback(1);
	}
    
    if (huart->Instance == uart_rs485.Instance)
	{
        RS485_Rx_Callback(1);
    }
    
    if (huart->Instance == uart_rs485_2.Instance)
    {
        RS485_2_Rx_Callback(1);
    }
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == uart_debug.Instance)
    {
        AppComm_DBG_Rx_Callback(Size);
    }
    
    if (huart->Instance == uart_sim.Instance)
    {
        Sim_Rx_Callback(Size);
    }
    
    if (huart->Instance == uart_rs485.Instance)
    {
        RS485_Rx_Callback(Size);
    }
    
    if (huart->Instance == uart_rs485_2.Instance)
    {
        RS485_2_Rx_Callback(Size);
    }
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{    
    if (huart->Instance == uart_debug.Instance)
    {
        IndexUartError = _UART_DEBUG;
    }
    
    if (huart->Instance == uart_sim.Instance)
    {
        IndexUartError = _UART_SIM;
    }
    
    if (huart->Instance == uart_rs485.Instance)
    {
        IndexUartError = _UART_485;
    }
    
    if (huart->Instance == uart_rs485_2.Instance)
    {
        IndexUartError = _UART_485_2;
    }
    
    fevent_active(sEventAppComm, _EVENT_UART_ERROR);
}


void ReInit_Rx_Mode_Uart_Error (void)
{
    APP_LOG(TS_OFF, DBLEVEL_M, "u_app_com: uart %d error!\r\n", IndexUartError );
    
    if (IndexUartError == _UART_DEBUG)
    {
        AppComm_DBG_Stop_RX_Mode();
        
        UartDebugCountFail++;
        if (UartDebugCountFail >= 20)
        {
            UartDebugCountFail = 0;
            
            AppComm_DBG_Init_Uart();   
        }
        
        AppComm_DBG_Init_RX_Mode();
    }  
    
    if (IndexUartError == _UART_SIM)
    {
        Sim_Stop_RX_Mode();
        Sim_Init_RX_Mode();
    } 
    
    if (IndexUartError == _UART_485)
    {
        RS485_Stop_RX_Mode();
        RS485_Init_RX_Mode();
    }
    
    if (IndexUartError == _UART_485_2)
    {
        RS485_2_Stop_RX_Mode();
        RS485_2_Init_RX_Mode();
    }
}




