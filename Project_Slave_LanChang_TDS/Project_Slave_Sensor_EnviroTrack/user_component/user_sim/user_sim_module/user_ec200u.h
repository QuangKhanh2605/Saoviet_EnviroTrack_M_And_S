/*
 * siml506.h
 *
 *  Created on: Feb 16, 2022
 *      Author: chien
 */

#ifndef USER_EC200U_H_
#define USER_EC200U_H_

#include "user_util.h"
#include "main.h"
#include "user_sim_common.h"


/*================== Var struct =====================*/
typedef enum
{
    //Cmd Pre Init
	_EC200_AT_AT,
	_EC200_AT_ECHO,
    //Cmd Init
    _EC200_AT_BAUD_RATE,
    _EC200_AT_GET_ID,
	_EC200_AT_CHECK_RSSI,
	_EC200_AT_GET_IMEI,
	_EC200_AT_CHECK_SIM,
    _EC200_AT_SET_APN_1,
    _EC200_AT_SET_APN_2,
    _EC200_AT_APN_AUTH_1,
    _EC200_AT_APN_AUTH_2,
    
    _EC200_AT_DTR_MODE,
    _EC200_AT_TEST_0,
    _EC200_AT_GET_IP_APN,
    _EC200_AT_ACTIVE_APN,
    _EC200_AT_URC_PORT,
    //SMS
    _EC200_SMS_FORMAT_TEXT,
    _EC200_SMS_NEW_MESS_IND,
    _EC200_SMS_READ_UNREAD,
    _EC200_SMS_SEND_1,
    _EC200_SMS_SEND_2,
    _EC200_SMS_SEND_3,
    _EC200_SMS_READ_ALL,
    _EC200_SMS_DEL_ALL,
    //Calling
    _EC200_CALLING_LINE,
    _EC200_CALLING_CONN,
    _EC200_CALLING_ESTAB_1,
    _EC200_CALLING_ESTAB_2,
    _EC200_CALLING_DISCONN,
    
    //Cmd Check Network
    _EC200_NET_TIME_ZONE,
	_EC200_NET_CEREG,
	_EC200_NET_CHECK_ATTACH,
    //Cmd clock
	_EC200_NET_GET_RTC,
    //Cmd Data mode and command mode
    _EC200_TCP_OUT_DATAMODE,
    _EC200_TCP_IN_DATAMODE,
    //Cmd TCP
    _EC200_TCP_IP_SIM,
    
    _EC200_TCP_CLOSE_1,
    _EC200_TCP_CLOSE_2,
    
	_EC200_TCP_CONNECT_1,
    _EC200_TCP_CONNECT_2,
    
    _EC200_TCP_SEND_CMD_1,
    _EC200_TCP_SEND_CMD_2,
    _EC200_TCP_SEND_CMD_3,
        
    _EC200_MQTT_CONN_1,
    _EC200_MQTT_CONN_2,
    _EC200_MQTT_SUB_1,
    _EC200_MQTT_SUB_2,
    _EC200_MQTT_PUB_1,
    _EC200_MQTT_PUB_2,
    _EC200_MQTT_PUB_FB_1,
    _EC200_MQTT_PUB_FB_2,
    _EC200_MQTT_PUB_PING,
    
    _EC200_TCP_PING_TEST1,
    _EC200_TCP_PING_TEST2,
    //File
    _EC200_SYS_FILE_DEL1,  
    _EC200_SYS_FILE_DEL2,
    _EC200_SYS_FILE_DEL_ALL,
    _EC200_SYS_FILE_LIST,
    _EC200_SYS_FILE_OPEN1,
    _EC200_SYS_FILE_OPEN2,
    _EC200_SYS_FILE_CLOSE1,
    _EC200_SYS_FILE_CLOSE2,
    _EC200_SYS_FILE_SEEK1,
    _EC200_SYS_FILE_SEEK2,
    _EC200_SYS_FILE_POS1,
    _EC200_SYS_FILE_POS2,
    _EC200_SYS_FILE_READ1,
    _EC200_SYS_FILE_READ2,
    //FTP
    _EC200_FTP_CONTEXT,
    _EC200_FTP_ACCOUNT,
    _EC200_FTP_FILE_TYPE,
    _EC200_FTP_TRANS_MODE,
    _EC200_FTP_RESP_TIME,
    
    _EC200_FTP_LOGIN_1,
    _EC200_FTP_LOGIN_2,
    
    _EC200_FTP_PATH_1,
    _EC200_FTP_PATH_2,
    
    _EC200_FTP_FILE_DEL_1,
    _EC200_FTP_FILE_DEL_2,
    
    _EC200_FTP_FILE_SIZE_1,
    _EC200_FTP_FILE_SIZE_2,
    
    _EC200_FTP_GET_TO_RAM_1,
    _EC200_FTP_GET_TO_RAM_2,
    
    _EC200_FTP_READ_1,
    _EC200_FTP_READ_2,
    //SSL Config
    _EC200_HTTP_CFG_SSL_VER,
    _EC200_HTTP_CFG_SSL_CIP,
    _EC200_HTTP_CFG_SSL_LEV,
    
    _EC200_HTTP_CFG_SSL_CA,
    _EC200_HTTP_CFG_SSL_CLC,
    _EC200_HTTP_CFG_SSL_CLK,
    //Http
    _EC200_HTTP_CFG_CONTEXT,
    _EC200_HTTP_CFG_HEADER1,
    _EC200_HTTP_CFG_HEADER2,
    _EC200_HTTP_CFG_SSL_CTX,
    
    _EC200_HTTP_SET_URL_1,
    _EC200_HTTP_SET_URL_2,
    _EC200_HTTP_SET_URL_3,
    
    _EC200_HTTP_POST_UART_1,
    _EC200_HTTP_POST_UART_2,
    _EC200_HTTP_POST_UART_3,
    
    _EC200_HTTP_REQUEST_GET,
    _EC200_HTTP_STORE_FILE1,
    _EC200_HTTP_STORE_FILE2,

    _EC200_HTTP_READ_1,
    _EC200_HTTP_READ_2,
    
    _EC200_GPS_CFG_TOKEN,
    _EC200_GPS_CFG_FORMAT,
    _EC200_GPS_QUERY_TOKEN,
    _EC200_GPS_GET_LOCATION,
    
    _EC200_GPS_CFG_OUT_PORT,
    _EC200_GPS_CFG_AUTO_RUN,
    _EC200_GPS_DEL_CUR_DATA,
    _EC200_GPS_TURN_ON,
    _EC200_GPS_POSTION_INF,
    _EC200_GPS_TURN_OFF,
    _EC200_AUDIO_FAR_END,
        
	_EC200_END,                // don't use
}AT_EC200_TypeDef;

typedef enum
{
	_EC200_URC_RESET_SIM900 = 0,
	_EC200_URC_SIM_LOST,
	_EC200_URC_SIM_REMOVE,
	_EC200_URC_CLOSED,
	_EC200_URC_PDP_DEACT,
	_EC200_URC_CALL_READY,
    _EC200_URC_CALLING,
    _EC200_URC_CALL_FAIL,
    _EC200_URC_SMS_CONTENT,
    _EC200_URC_SMS_INDICATION1,
    _EC200_URC_SMS_INDICATION2,
	_EC200_URC_ERROR,
    _EC200_URC_RECEIV_SERVER,
	_EC200_URC_END,
}URC_EC200_TypeDef;




/*===================== extern ====================*/
extern const sCommand_Sim_Struct aEC200Step[];
extern const sCommand_Sim_Struct aEC200Urc[];

extern uint8_t aEC200_BLOCK_INIT[17];
extern uint8_t aEC200_BLOCK_NETWORK[9];

#ifdef USING_TRANSPARENT_MODE
    extern uint8_t aEC200_BLOCK_CONNECT[7];
    extern uint8_t aEC200_BLOCK_PUB_FB[2];
    extern uint8_t aEC200_BLOCK_PUB[2];
    extern uint8_t aEC200_BLOCK_PING[2];
#else
    extern uint8_t aEC200_BLOCK_CONNECT[11];
    extern uint8_t aEC200_BLOCK_PUB_FB[4];
    extern uint8_t aEC200_BLOCK_PUB[4];
    extern uint8_t aEC200_BLOCK_PING[4];
#endif
    
extern uint8_t aEC200_BLOCK_CSQ[3];
extern uint8_t aEC200_BLOCK_DIS_CONN[4];
extern uint8_t aEC200_BLOCK_CLOCK[3];

extern uint8_t aEC200_BLOCK_HTTP_INIT[19];
extern uint8_t aEC200_BLOCK_HTTP_READ[2];

extern uint8_t aEC200_BLOCK_READ_SMS[4]; 
extern uint8_t aEC200_BLOCK_SEND_SMS[5];
extern uint8_t aEC200_BLOCK_CALLING[6];

extern uint8_t aEC200_BLOCK_FILE_CLOSE[2];

extern uint8_t aEC200_BLOCK_GNSS[2]; 
extern uint8_t aEC200_BLOCK_GNSS_DELL[1];
extern uint8_t aEC200_BLOCK_GNSS_ON[1];

extern uint8_t aEC200_BLOCK_FTP_INIT[16];
extern uint8_t aEC200_BLOCK_FTP_READ[2] ;

/*================== Function ==========================*/
uint32_t    EC200_Get_Timeout_Follow_Cmd (uint8_t sim_step);
sCommand_Sim_Struct * EC200_Get_AT_Cmd (uint8_t step);
uint32_t    EC200_Get_Delay_AT (uint8_t Step);
uint8_t     EC200_Is_Step_Check_URC (uint8_t sim_step);
uint8_t     EC200_Check_Step_Skip_Error (uint8_t step);
uint8_t     EC200_Is_Ignore_Failure (uint8_t Step, uint8_t (*pJum_Next_At)(uint8_t Type));

uint8_t     EC200_Is_Send_Hex (uint8_t step);

#endif /* SIM_MODULE_SIML506_SIML506_H_ */
