
/*
 * user_bc66.h
 *
 *  Created on: Dec 31, 2021
 *      Author: Chien
 */

#ifndef USER_BC660_H_
#define USER_BC660_H_


#include "user_util.h"
#include "user_sim_common.h"

/*=============Define====================*/


/*===================var struct enum===========*/

typedef enum
{
	_BC660_AT_AT,
    _BC660_AT_ECHO,
    
    _BC660_AT_REVISION,
	_BC660_AT_GET_ID,
	_BC660_AT_BAUD_RATE,
    
    _BC660_AT_CHECK_SIM,
    _BC660_AT_CHECK_RSSI,
	_BC660_AT_SET_APN,
    
    //config
    _BC660_AT_SOFT_RESET,
    
    _BC660_AT_SAVE_SETT,
	_BC660_AT_POWER_OFF,
    
    _BC660_AT_EN_WAKE_URC,
    
    _BC660_AT_EN_URC,
	_BC660_AT_DIS_URC,
    
    _BC660_DEEP_SLEEP_DIS,
    _BC660_DEEP_SLEEP_EN,
    _BC660_LIGHT_SLEEP,
    
    _BC660_CONF_EVENT_SLEEP,
    _BC660_CONF_WAKE_PIN,
    
    //net
	_BC660_NET_CHECK_ATTACH,
	_BC660_NET_SET_BAND_3,
	_BC660_NET_SET_BAND_ALL,
   
    _BC660_NET_GET_ENG,
	_BC660_NET_GET_RTC,
  
    _BC660_PSM_DIS,
    _BC660_PSM_EN,
    
    _BC660_eDRX_EN,
    _BC660_eDRX_DIS,
        
    //tcp
	_BC660_TCP_CONF_DATA,

    _BC660_AT_SET_CREG,
    _BC660_AT_CHECK_CREG,
      
    _BC660_TCP_CONF_MODE_ACC,
    _BC660_TCP_CONF_MODE_PUSH,
    
    _BC660_TCP_OPEN_1,
    _BC660_TCP_OPEN_2,
    
    _BC660_TCP_CLOSE_1,
    _BC660_TCP_CLOSE_2,
    
    _BC660_TCP_SEND_CMD_1,
    _BC660_TCP_SEND_CMD_2,
    _BC660_TCP_SEND_CMD_3,
    
    //mqtt
	_BC660_MQTT_CONN_1,
	_BC660_MQTT_CONN_2,

	_BC660_MQTT_SUB_1,
	_BC660_MQTT_SUB_2,

	_BC660_MQTT_PUB_1,
	_BC660_MQTT_PUB_2,

    _BC660_MQTT_PUB_FB_1,
    _BC660_MQTT_PUB_FB_2,
    
    _BC660_MQTT_PUB_PING,
    
	_BC660_END, // don't use
    
}AT_SIM_TypeDef;


typedef enum
{   
    _BC660_URC_RESET_SIM900,
    _BC660_URC_CLOSED,
    _BC660_URC_PDP_DEACT,
    _BC660_URC_ERROR,
    _BC660_URC_WAKEUP,
    _BC660_URC_DEEP_SLEEP,
    
    _BC660_URC_RECEIV_SERVER,
    
	_BC660_URC_END,
}Kind_URC_BC66;


extern const sCommand_Sim_Struct aBC660Step[];
extern const sCommand_Sim_Struct aBC660Urc[];

extern uint8_t aBC660_BLOCK_INIT[13];
extern uint8_t aBC660_BLOCK_NETWORK[5];

extern uint8_t aBC660_BLOCK_CONNECT[15];
extern uint8_t aBC660_BLOCK_PUB_FB[4];
extern uint8_t aBC660_BLOCK_PUB[4];
extern uint8_t aBC660_BLOCK_PING[4];
    
extern uint8_t aBC660_BLOCK_CSQ[1];
extern uint8_t aBC660_BLOCK_DIS_CONN[2];
extern uint8_t aBC660_BLOCK_CLOCK[1];

extern uint8_t aBC660_BLOCK_HTTP_INIT[1];
extern uint8_t aBC660_BLOCK_HTTP_READ[1];

extern uint8_t aBC660_BLOCK_READ_SMS[1]; 
extern uint8_t aBC660_BLOCK_SEND_SMS[1];
extern uint8_t aBC660_BLOCK_CALLING[1];

extern uint8_t aBC660_BLOCK_FILE_CLOSE[1];

extern uint8_t aBC660_BLOCK_FTP_INIT[1];
extern uint8_t aBC660_BLOCK_FTP_READ[1] ;

extern uint8_t aBC660_BLOCK_LOW_POWER[2];

/*================Function===============*/


sCommand_Sim_Struct *BC660_Get_AT_Cmd (uint8_t step);

uint32_t BC660_Get_Delay_AT (uint8_t Step);
uint32_t BC660_Get_Timeout_Follow_Cmd (uint8_t sim_step);
uint8_t  BC660_Is_Ignore_Failure (uint8_t Step, uint8_t (*pJum_Next_At)(uint8_t Type));
uint8_t  BC660_Is_Step_Check_URC (uint8_t sim_step);
uint8_t  BC660_Check_Step_Skip_Error (uint8_t step);
uint8_t  BC660_Is_Send_Hex (uint8_t step);



#endif /* USER_BC660_H_ */
