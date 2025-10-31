/*
 * myUart.h
 *
 *  Created on: Dec 7, 2021
 *      Author: lenovo
 */

#ifndef USER_UART_H_
#define USER_UART_H_

#include "user_util.h"
#include "user_define.h"


/*===========Var struct=============*/

typedef enum
{
    _UART_DEBUG = 0,
    _UART_SIM,
    _UART_485,
    _UART_485_2,
}eTypeUart;


/*==================Function==================*/

void ReInit_Rx_Mode_Uart_Error (void);

    
#endif /* USER_UART_H_ */
