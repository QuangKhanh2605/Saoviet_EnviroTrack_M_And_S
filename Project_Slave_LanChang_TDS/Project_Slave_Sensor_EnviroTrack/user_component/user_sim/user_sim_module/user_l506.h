/*
 * siml506.h
 *
 *  Created on: Feb 16, 2022
 *      Author: chien
 */

#ifndef USER_L506_H_
#define USER_L506_H_

#include "user_util.h"
#include "main.h"
#include "user_sim_common.h"

  
     
     
/*================== Var struct =====================*/
typedef enum
{
    //Cmd Pre Init
	_L506_AT_AT,
	_L506_AT_ECHO,
	_L506_AT_MAN_LOG,
    //Cmd Init
    _L506_AT_BAUD_RATE,
    _L506_AT_GET_ID,
	_L506_AT_CHECK_RSSI,
	_L506_AT_GET_IMEI,
	_L506_AT_CHECK_SIM,
    _L506_AT_SET_APN_1,
    _L506_AT_SET_APN_2,
    _L506_AT_APN_AUTH_1,
    _L506_AT_APN_AUTH_2,
    
    _L506_AT_DTR_MODE,
    _L506_AT_TEST_0,
    _L506_AT_GET_IP_APN,
    _L506_AT_ACTIVE_APN,
    //SMS
    _L506_SMS_FORMAT_TEXT,
    _L506_SMS_NEW_MESS_IND,
    _L506_SMS_READ_UNREAD,
    _L506_SMS_READ_ALL,
    _L506_SMS_DEL_ALL,
    //Calling
    _L506_CALLING_LINE,
    //Cmd Set 2G 3G 4G
	_L506_NET_RA_AUTO,
	_L506_NET_RA_2G,
    _L506_NET_RA_3G,
    _L506_NET_RA_4G,
    
    _L506_NET_CHECK_RA,   
    _L506_NET_CHECK_BAND,
    _L506_NET_CHECK_CSPI,
    //Cmd Check Network
	_L506_NET_CEREG,
	_L506_NET_CHECK_ATTACH,
    //Cmd Config
	_L506_NET_SYN_TIME_ZONE,
	_L506_NET_GET_RTC,
    //Cmd Data mode and command mode
    _L506_TCP_OUT_DATAMODE,
    _L506_TCP_TRANS,
    _L506_TCP_IN_DATAMODE,
    
    _L506_TCP_TRANS_SETUP,
    //Cmd TCP
    _L506_TCP_GET_IP_SIM,
    _L506_TCP_NETOPEN,
    _L506_TCP_NETCLOSE,
    
    _L506_TCP_CLOSE_1,
    _L506_TCP_CLOSE_2,
	_L506_TCP_CONNECT_1,
    _L506_TCP_CONNECT_2,
    
    _L506_TCP_SEND_CMD_1,
    _L506_TCP_SEND_CMD_2,
    _L506_TCP_SEND_CMD_3,
        
    _L506_MQTT_CONN_1,
    _L506_MQTT_CONN_2,
    _L506_MQTT_SUB_1,
    _L506_MQTT_SUB_2,
    _L506_MQTT_PUB_1,
    _L506_MQTT_PUB_2,
    _L506_MQTT_PUB_FB_1,
    _L506_MQTT_PUB_FB_2,
    _L506_MQTT_PUB_PING,
    
    _L506_TCP_PING_TEST1,
    _L506_TCP_PING_TEST2,
    //File
    _L506_SYS_DEL_FILE_1,
    _L506_SYS_DEL_FILE_2,
    _L506_SYS_LIST_FILE,
    //FTP
    _L506_FTP_SERVER_1,
    _L506_FTP_SERVER_2,
    _L506_FTP_PORT,
    
    _L506_FTP_USERNAME,
    _L506_FTP_PASSWORD,
    _L506_FTP_TYPE,
    _L506_FTP_LIST_FILE,
    _L506_FTP_GET_FILE_1,
    _L506_FTP_GET_FILE1,
    _L506_FTP_GET_FILE2,
    _L506_FTP_GET_FILE3,
    _L506_FTP_READ_1,
    _L506_FTP_READ_2,
    //https
    _L506_HTTPS_ACQUIRE,
    _L506_HTTPS_STOP,
    
    _L506_HTTPS_OPEN_SES_1,
    _L506_HTTPS_OPEN_SES_2,
    
    _L506_HTTPS_CLOSE_SES,
    
    _L506_HTTPS_SEND_1,
    _L506_HTTPS_SEND_2,
    _L506_HTTPS_SEND_3,
    //Http
    _L506_HTTP_OPEN,
    _L506_HTTP_CLOSE,
    _L506_HTTP_CLOSE_2,
    _L506_HTTP_RECV_LOCAL,
    _L506_HTTP_RECV_TE,
    _L506_HTTP_SET_HEADER_1,
    _L506_HTTP_SET_HEADER_2,
    _L506_HTTP_SET_URL_1,
    _L506_HTTP_SET_URL_2,
    _L506_HTTP_REQ_POST_1,
    _L506_HTTP_REQ_POST_3,
    _L506_HTTP_REQUEST_GET,
    
    _L506_HTTP_POST_DATA_1, 
    _L506_HTTP_POST_DATA_2,
    _L506_HTTP_POST_DATA_3,
    
    _L506_HTTP_POST_DATA_EX1,
    _L506_HTTP_POST_DATA_EX2,
    
    _L506_HTTP_POST_DAT_END,
    _L506_HTTP_POST_SEND,
    _L506_HTTP_POST_SEND_2,
    _L506_HTTP_LENGTH,
    _L506_HTTP_READ_RES,
    
    _L506_HTTP_READ_1,
    _L506_HTTP_READ_2,
        
	_L506_END,                // don't use
}AT_L506_TypeDef;

typedef enum
{
	_L506_URC_RESET_SIM900 = 0,
	_L506_URC_SIM_LOST,
	_L506_URC_SIM_REMOVE,
	_L506_URC_CLOSED,
	_L506_URC_PDP_DEACT,
	_L506_URC_CALL_READY,
    _L506_URC_CALLING,
    _L506_URC_SMS_CONTENT,
    _L506_URC_SMS_INDICATION1,
    _L506_URC_SMS_INDICATION2,
	_L506_URC_ERROR,
    _L506_URC_RECEIV_SERVER,
	_L506_URC_END,
}URC_L506_TypeDef;




/*===================== extern ====================*/
extern const sCommand_Sim_Struct aSimL506Step[];
extern const sCommand_Sim_Struct aL506Urc[];

extern uint8_t aL506_BLOCK_INIT[15];
#ifdef MQTT_PROTOCOL
    extern uint8_t aL506_BLOCK_NETWORK[10];
#else
    extern uint8_t aL506_BLOCK_NETWORK[12];
#endif

#ifdef USING_TRANSPARENT_MODE
    extern uint8_t aL506_BLOCK_CONNECT[13];
    extern uint8_t aL506_BLOCK_PUB_FB[2];
    extern uint8_t aL506_BLOCK_PUB[2];
    extern uint8_t aL506_BLOCK_PING[2]; 
#else
    #ifdef MQTT_PROTOCOL
        extern uint8_t aL506_BLOCK_CONNECT[17];
    #endif
        
    #ifdef HTTP_PROTOCOL
        extern uint8_t aL506_BLOCK_CONNECT[6];
        extern uint8_t aL506_BLOCK_HTTP_POST[9]; 
    #endif
        
    #ifdef HTTPS_PROTOCOL
        extern uint8_t aL506_BLOCK_CONNECT[2];
        extern uint8_t aL506_BLOCK_HTTP_POST[5]; 
    #endif
        
    extern uint8_t aL506_BLOCK_PUB_FB[4];
    extern uint8_t aL506_BLOCK_PUB[4];
    extern uint8_t aL506_BLOCK_PING[4]; 
#endif
    
extern uint8_t aL506_BLOCK_HTTP_HEAD[2];
extern uint8_t aL506_BLOCK_CSQ[3];

#ifdef MQTT_PROTOCOL
    extern uint8_t aL506_BLOCK_DIS_CONN[4];
#else
    extern uint8_t aL506_BLOCK_DIS_CONN[1];
#endif
    
extern uint8_t aL506_BLOCK_CLOCK[3];

extern uint8_t aL506_BLOCK_HTTP_INIT[19];
extern uint8_t aL506_BLOCK_HTTP_READ[2];
 
extern uint8_t aL506_BLOCK_READ_SMS[4];
extern uint8_t aL506_BLOCK_SEND_SMS[2];
extern uint8_t aL506_BLOCK_CALLING[2];


/*================== Function ==========================*/
sCommand_Sim_Struct * L506_Get_AT_Cmd (uint8_t step);
uint8_t     L506_Is_Step_Check_URC (uint8_t sim_step);

uint8_t     L506_Check_Step_Skip_Error (uint8_t step);
uint32_t    L506_Get_Delay_AT (uint8_t Step);
uint32_t    L506_Get_Timeout_Follow_Cmd (uint8_t sim_step);
uint8_t     L506_Is_Ignore_Failure (uint8_t Step, uint8_t (*pJum_Next_At)(uint8_t Type));
uint8_t     L506_Is_Send_Hex (uint8_t step);

#endif /* SIM_MODULE_SIML506_SIML506_H_ */
