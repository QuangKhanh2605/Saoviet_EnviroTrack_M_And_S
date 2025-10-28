/*
 * user_define.h
 *
 *  Created on: Dec 31, 2021
 *      Author: Chien
 */

#ifndef INC_USER_DEFINE_H_
#define INC_USER_DEFINE_H_

#include "user_app_comm.h"

/* 
    - Using lib app sim
    #define INC_APP_SIM_FEATURE
*/
//#define INC_APP_SIM_FEATURE
#ifdef INC_APP_SIM_FEATURE
    #include "user_app_sim.h"
#endif 

/* 
    - Using lib app water meter
    #define INC_APP_WM_FEATURE
*/
//#define INC_APP_WM_FEATURE
#ifdef INC_APP_WM_FEATURE
    #include "user_app_wm.h"
#endif 

/* 
    - Using lib app water meter
    #define INC_APP_WM_FEATURE
*/
#define INC_APP_RS485
#ifdef INC_APP_RS485
    #include "user_app_rs485.h"
#endif 

#define INC_APP_SENSOR
#ifdef INC_APP_SENSOR
    #include "user_app_sensor.h"
#endif 

/* 
    - Using lib app water meter
    #define INC_APP_WM_FEATURE
*/
//#define INC_APP_SENSOR
#ifdef INC_APP_SENSOR
    #include "user_app_sensor.h"
#endif 

/* 
    - Using lib app Emeter
    #define INC_APP_EMET_FEATURE 
*/

#ifdef INC_APP_EMET_FEATURE
    #include "user_app_emet.h"
#endif 

/* 
    - Using lib app Lora
    #define INC_APP_LORA_FEATURE 
*/
//#define INC_APP_LORA_FEATURE 

#ifdef INC_APP_LORA_FEATURE
    #include "user_app_lora.h"
#endif 


/* 
    - Using lib app Lora
    #define INC_APP_TEMH_FEATURE 
*/
//#define INC_APP_TEMH_FEATURE 

#ifdef INC_APP_TEMH_FEATURE
    #include "user_app_temh.h"
#endif 


/* 
    - Using lib app Lora
    #define INC_APP_ETH_FEATURE 
*/
//#define INC_APP_ETH_FEATURE 

#ifdef INC_APP_ETH_FEATURE
    #include "user_app_ethernet.h"
#endif 


/* 
    - Using lib app Lora
    #define INC_APP_ETH_FEATURE 
*/
#define INC_APP_LCD_FEATURE 

#ifdef INC_APP_LCD_FEATURE
    #include "lcd_display.h"
#endif 


//#include "user_app_mem.h"
#include "user_internal_mem.h" 

#include "user_modem.h"
#include "user_log.h"

#include "user_uart.h"
#include "event_driven.h"
#include "queue_p.h"
#include "user_time.h"
#include "user_obis_sv.h"
#include "user_string.h"
#include "user_at_serial.h"  
//#include "user_mqtt.h"  
#include "user_rs485.h"  
#include "user_connect_sensor.h"  
#include "user_app_modbus_rtu.h"
/*========================== User define =================================*/
/*     Chu ki wake up
      - Chu ki nay se duoc set timer danh thuc MCU day.
      - Don vi (s)
*/
#define DEFAULT_FREQ_WAKEUP                             5     //don vi min

/* So lan thiet bi day roi moi gui ban tin */    //tu thong so nay va chu ki wakeup se tinh ra duoc chu ki cua ban tin
#define DEFAULT_NUMBER_WAKE_UP                          1

#define MAX_NUMBER_WAKE_UP                              50
#define MAX_NUNBER_SEND_INIT                            1
#define MAX_MESS_IN_PACKET                              1       //1 ban tin 31 byte. Max 512 byte - 45 byte header

/*--- ID cua thiet bi -------*/
#define DEVICE_ID                                       "SVWMDL25009999"   //"SVWMGE24002222"  // "SVLEV230000001"   // "SVWMGC23000001"  

#define DEFAULT_FREQ_SEND_ONLINE                        5


/*
    * Uart danh cho debug
        + Baurate
        + Port uart
        ...
*/
extern DMA_HandleTypeDef            hdma_usart3_rx;

#define uart_debug	                huart3
#define uart_dma_debug              hdma_usart3_rx

#define MX_UART_DBG_Init            MX_USART3_UART_Init            //Func init Uart sim

/* Definition for USARTx's NVIC */
#define USART_DBG_IRQn              USART3_IRQn
#define USART_DBG_Priority          1

/*-- Rx uart: DMA with IDLE line | Interupt line  ----*/
#define UART_DBG_DMA                0
#define UART_DBG_IT                 1

#define UART_DBG_MODE               UART_DBG_DMA



#endif /* INC_USER_DEFINE_H_ */
