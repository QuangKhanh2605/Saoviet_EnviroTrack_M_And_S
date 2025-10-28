/*
 * siml511.h
 *
 *  Created on: Dec 13, 2023
 *      Author: chien
 */

#ifndef USER_L511_H_
#define USER_L511_H_

#include "user_util.h"
#include "main.h"
#include "user_sim_common.h"


/*================== Var struct =====================*/
typedef enum
{
    //Cmd Pre Init
	_L511_AT_AT,
	_L511_AT_ECHO,
	_L511_AT_MAN_LOG,
    //Cmd Init
    _L511_AT_BAUD_RATE,
    _L511_AT_GET_ID,
	_L511_AT_CHECK_RSSI,
	_L511_AT_GET_IMEI,
	_L511_AT_CHECK_SIM,
    _L511_AT_SET_APN_1,
    _L511_AT_SET_APN_2,
    
    _L511_AT_TEST_0,
    _L511_AT_GET_IP_APN,
    _L511_AT_ACTIVE_APN,
//    //SMS
//    _L511_SMS_FORMAT_TEXT,
//    _L511_SMS_NEW_MESS_IND,
//    _L511_SMS_READ_UNREAD,
//    _L511_SMS_READ_ALL,
//    _L511_SMS_DEL_ALL,
//    //Calling
//    _L511_CALLING_LINE,
//    //Cmd Set 2G 3G 4G
//	_L511_NET_RA_AUTO,
//	_L511_NET_RA_2G,
//    _L511_NET_RA_3G,
//    _L511_NET_RA_4G,
//    
//    _L511_NET_CHECK_RA,   
//    _L511_NET_CHECK_BAND,
//    _L511_NET_CHECK_CSPI,
    //Cmd Check Network
	_L511_NET_CEREG,
	_L511_NET_CHECK_ATTACH,
    //Cmd Config
	_L511_NET_SYN_TIME_ZONE,
	_L511_NET_GET_RTC,
    //Cmd Data mode and command mode
    _L511_TCP_OUT_DATAMODE,
    _L511_TCP_TRANS,
    _L511_TCP_IN_DATAMODE,
    
    _L511_TCP_TRANS_SETUP,
    //Cmd TCP
    _L511_TCP_GET_IP_SIM,
    _L511_TCP_NETOPEN,
    _L511_TCP_NETCLOSE,
    
    _L511_TCP_CLOSE_1,
    _L511_TCP_CLOSE_2,
	_L511_TCP_CONNECT_1,
    _L511_TCP_CONNECT_2,
    
    _L511_TCP_SEND_CMD_1,
    _L511_TCP_SEND_CMD_2,
    _L511_TCP_SEND_CMD_3,
        
    _L511_MQTT_CONN_1,
    _L511_MQTT_CONN_2,
    _L511_MQTT_SUB_1,
    _L511_MQTT_SUB_2,
    _L511_MQTT_PUB_1,
    _L511_MQTT_PUB_2,
    _L511_MQTT_PUB_FB_1,
    _L511_MQTT_PUB_FB_2,
    _L511_MQTT_PUB_PING,
    
    _L511_TCP_PING_TEST1,
    _L511_TCP_PING_TEST2,
    //File
    _L511_SYS_DEL_FILE_1,
    _L511_SYS_DEL_FILE_2,
    _L511_SYS_LIST_FILE,
    //FTP
    _L511_FTP_SERVER_1,
    _L511_FTP_SERVER_2,
    _L511_FTP_PORT,
    
    _L511_FTP_USERNAME,
    _L511_FTP_PASSWORD,
    _L511_FTP_TYPE,
    _L511_FTP_LIST_FILE,
    _L511_FTP_GET_FILE_1,
    _L511_FTP_GET_FILE1,
    _L511_FTP_GET_FILE2,
    _L511_FTP_GET_FILE3,
    _L511_FTP_READ_1,
    _L511_FTP_READ_2,

    //Http
    _L511_HTTP_OPEN,
    _L511_HTTP_CLOSE,
    _L511_HTTP_CLOSE_2,
    _L511_HTTP_RECV_LOCAL,
    _L511_HTTP_RECV_TE,
    _L511_HTTP_SET_HEADER_1,
    _L511_HTTP_SET_HEADER_2,
    _L511_HTTP_SET_URL_1,
    _L511_HTTP_SET_URL_2,
    _L511_HTTP_REQ_POST_1,
    _L511_HTTP_REQ_POST_3,
    _L511_HTTP_REQUEST_GET,
    
    _L511_HTTP_POST_DATA_1, 
    _L511_HTTP_POST_DATA_2,
    _L511_HTTP_POST_DATA_3,
    
    _L511_HTTP_POST_DATA_EX1,
    _L511_HTTP_POST_DATA_EX2,
    
    _L511_HTTP_POST_DAT_END,
    _L511_HTTP_POST_SEND,
    _L511_HTTP_POST_SEND_2,
    _L511_HTTP_LENGTH,
    _L511_HTTP_READ_RES,
    
    _L511_HTTP_READ_1,
    _L511_HTTP_READ_2,
        
	_L511_END,                // don't use
}AT_L511_TypeDef;

typedef enum
{
	_L511_URC_RESET_SIM900 = 0,
	_L511_URC_SIM_LOST,
	_L511_URC_SIM_REMOVE,
	_L511_URC_CLOSED,
	_L511_URC_PDP_DEACT,
	_L511_URC_CALL_READY,
    _L511_URC_CALLING,
    _L511_URC_SMS_CONTENT,
    _L511_URC_SMS_INDICATION1,
    _L511_URC_SMS_INDICATION2,
	_L511_URC_ERROR,
    _L511_URC_RECEIV_SERVER,
	_L511_URC_END,
}URC_L511_TypeDef;




/*===================== extern ====================*/
extern const sCommand_Sim_Struct aSimL511Step[];
extern const sCommand_Sim_Struct aL511Urc[];

extern uint8_t aL511_BLOCK_INIT[9];
#ifdef MQTT_PROTOCOL
    extern uint8_t aL511_BLOCK_NETWORK[6];
#else
    extern uint8_t aL511_BLOCK_NETWORK[12];
#endif

#ifdef USING_TRANSPARENT_MODE
    extern uint8_t aL511_BLOCK_CONNECT[13];
    extern uint8_t aL511_BLOCK_PUB_FB[2];
    extern uint8_t aL511_BLOCK_PUB[2];
    extern uint8_t aL511_BLOCK_PING[2];
#else
    #ifdef MQTT_PROTOCOL
        extern uint8_t aL511_BLOCK_CONNECT[17];
    #endif
        
    #ifdef HTTP_PROTOCOL
        extern uint8_t aL511_BLOCK_CONNECT[6];
        extern uint8_t aL511_BLOCK_HTTP_POST[9]; 
    #endif
        
    #ifdef HTTPS_PROTOCOL
        extern uint8_t aL511_BLOCK_CONNECT[2];
        extern uint8_t aL511_BLOCK_HTTP_POST[5]; 
    #endif
        
    extern uint8_t aL511_BLOCK_PUB_FB[4];
    extern uint8_t aL511_BLOCK_PUB[4];
    extern uint8_t aL511_BLOCK_PING[4];
#endif
    
extern uint8_t aL511_BLOCK_HTTP_HEAD[2];

extern uint8_t aL511_BLOCK_CSQ[3];

#ifdef MQTT_PROTOCOL
    extern uint8_t aL511_BLOCK_DIS_CONN[4];
#else
    extern uint8_t aL511_BLOCK_DIS_CONN[1];
#endif
    
extern uint8_t aL511_BLOCK_CLOCK[3];

extern uint8_t aL511_BLOCK_HTTP_INIT[17];
extern uint8_t aL511_BLOCK_HTTP_READ[2];
extern uint8_t aL511_BLOCK_SEND_SMS[2];
extern uint8_t aL511_BLOCK_CALLING[2];
extern uint8_t aL511_BLOCK_READ_SMS[2];

/*================== Function ==========================*/
sCommand_Sim_Struct * L511_Get_AT_Cmd (uint8_t step);
uint8_t     L511_Is_Step_Check_URC (uint8_t sim_step);

uint8_t     L511_Check_Step_Skip_Error (uint8_t step);
uint8_t     L511_Is_Ignore_Failure (uint8_t Step, uint8_t (*pJum_Next_At)(uint8_t Type));
uint32_t    L511_Get_Timeout_Follow_Cmd (uint8_t sim_step);
uint32_t    L511_Get_Delay_AT (uint8_t Step);
uint8_t     L511_Is_Send_Hex (uint8_t step);

#endif /* SIM_MODULE_SIML511_SIML511_H_ */
