


#ifndef USER_APP_ETHERNET_H
#define USER_APP_ETHERNET_H

#include "user_util.h"
#include "event_driven.h"
#include "user_define.h"

#include "dns.h"
#include "dhcp.h"
#include "user_sim_common.h"
#include "socket.h"
#include "ftpc.h"

/*===================== Define =========================*/
#define USING_APP_ETHERNET
/* define using domain or up */

#define VER_H		    	1
#define VER_L		    	00

#define SOCK_TCPS       	0
#define PORT_TCPS			5000

#ifndef DATA_BUF_SIZE
    #define DATA_BUF_SIZE   2048   
#endif

#define SOCKET_DEMO			0
#define SOCK_DNS       		6

#define S_RECEIVE		    0x01		
#define S_TRANSMITOK	    0x02	

#define W5500_RST_LOW  		HAL_GPIO_WritePin(ETH_RSTn_GPIO_Port, ETH_RSTn_Pin, GPIO_PIN_RESET)
#define W5500_RST_HIGH 		HAL_GPIO_WritePin(ETH_RSTn_GPIO_Port, ETH_RSTn_Pin, GPIO_PIN_SET)

#define W5500_SCS_LOW  		HAL_GPIO_WritePin(ETH_SCSn_GPIO_Port, ETH_SCSn_Pin, GPIO_PIN_RESET)
#define W5500_SCS_HIGH 		HAL_GPIO_WritePin(ETH_SCSn_GPIO_Port, ETH_SCSn_Pin, GPIO_PIN_SET)

#define W5500_ON  		    HAL_GPIO_WritePin(ETH_ON_OFF_GPIO_Port, ETH_ON_OFF_Pin, GPIO_PIN_RESET)
#define W5500_OFF 		    HAL_GPIO_WritePin(ETH_ON_OFF_GPIO_Port, ETH_ON_OFF_Pin, GPIO_PIN_SET)


#define ETH_MAX_RETRY_BUS   10  
#define ETH_MAX_FAIL_PHY    5
#define ETH_MAX_RETRY_DHCP  10  

#define ETH_MAX_CMD_QUEUE       20
#define ETH_MAX_DHCP_RETRY	    2
#define ETH_MAX_SOCK_RETRY	    5

#define ETH_MAX_TIME_CTRL_SOCK  10000

#define MAX_ETH_CHANGE_SERVER   5
#define MAX_ETH_HARD_RESET		20
#define ETH_MAX_RETRY_UPDATE    10

/*===================== Struct var =====================*/
typedef enum
{
    _EVENT_ETH_CONTROL_SIM,
	_EVENT_ETH_HARD_RESET,
    _EVENT_ETH_CHECK_BUS,
	_EVENT_ETH_PHY_LINK,
	_EVENT_ETH_DNS_PROCESS,
    _EVENT_ETH_RUN_DHCP,
    _EVENT_ETH_SOCK_CTRL,

    _EVENT_ETH_SEND_PING,   
    _EVENT_ETH_SEND_DATA,
    
    _EVENT_ETH_SEND,   
    _EVENT_ETH_SEND_OK,
    _EVENT_ETH_SEND_TIMEOUT,
    _EVENT_ETH_RECV_DATA,

    _EVENT_ETH_INTn_PROCESS,
    
    _EVENT_ETH_POWER_OFF, 
    
	_EVENT_APP_ETH_END,
}sEventAppEthernet;

typedef enum
{
    _ETH_POWER_OFF,         //0
	_ETH_START,             //1
	_ETH_CHECK_SPI,         //2
	_ETH_CHECK_PHY,         //3
    _ETH_DET_GATEWAY,       //4
    _ETH_DNS_RUNNING,       //5
    _ETH_DHCP_RUNNING,      //6
    _ETH_TCP_CONNECT,       //7    
    _ETH_MQTT_CONNECT,      //8
    _ETH_MQTT_SUBCRIBE,     //9
    _ETH_MQTT_CONNECTED,    //10
    _ETH_FTP_CONNECTED,     //11
    _ETH_HTTP_CONNECTED,    //12
}eEthernetStatus;

typedef enum  
{
    _ETH_MQTT_CONN,
    _ETH_MQTT_SUB,
    _ETH_MQTT_PUB_NACK,
    _ETH_MQTT_PUB_ACK,
    _ETH_MQTT_PING,
    
    _ETH_FTP_SET_USER,
    _ETH_FTP_SET_PASS,
    
	_ETH_FTP_SET_TYPE_ASCII,
	_ETH_FTP_SET_TYPE_BIN,
    
	_ETH_FTP_QUER_SER_DIR,
    _ETH_FTP_QUER_SIZE,
    
    _ETH_FTP_SET_DIR,
    _ETH_FTP_SET_PASV_PORT,
    _ETH_FTP_SET_GET_FILE,  
    
    _ETH_CMD_END,
}eListCmdEthernet;



typedef enum  
{
    _ETH_MQTT_URC_SERVER,
    
    _ETH_FTP_URC_READY,
    _ETH_FTP_URC_CLOSING,
    
    _ETH_URC_END,
}eListUrcEthernet;


typedef enum
{
    _ETH_MODE_CONN_MAIN = 0,		
    _ETH_MODE_CONN_BACKUP,		
    _ETH_MODE_CONN_FTP,		       
    _ETH_MODE_CONN_HTTP , 
}sKindEthConnect;

typedef struct
{
	sData  	*sIP;
	sData  	*sPort;
	sData  	*sMQTTUserName;
	sData  	*sMQTTPassword;
    
    uint8_t     *aIP;
    uint16_t    *Port_u16;
    
    uint32_t	*MQTTKeepAlive_u32;
    uint8_t     *DomainOrIp_u8;
}sEthServerInfor;

typedef struct 
{
    sData	*pFirmIP;
    sData   *pFirmPort;
    
    sData	*pFirmUser;
    sData   *pFirmPass;
    
    sData	*pFirmPath;
    sData	*pFirmName;
    sData   *pUrlHTTP;
    
    uint8_t     *aIP;
    uint16_t    *Port_u16;
    uint8_t     *DomainOrIp_u8;
}sEthUpdateinfor;

typedef enum 
{
    __SUCCESSFUL,
    __FAIL_CONNECT,
    __FAIL_LENGTH,
    __FAIL_HEADER,
    __FAIL_FLASH,
    __FAIL_CRC,
}eUpdateStatus;

typedef struct
{
    uint32_t	FwSizeGet_u32;
	uint32_t	Addr_u32;                    //Addr Save Fw
    
	uint32_t	Total_u8;
    uint16_t    cPacket_u8;
    
    uint8_t     CrcFile_u8;
    uint8_t     CrcCalcu_u8;
    
    uint8_t     HeaderValid_u8;
    
    uint8_t     Status_u8;
}sEthUpdateVariable;

typedef struct
{ 
    uint8_t         *pModePower_u8;
    sEthServerInfor sServer;
    sEthUpdateinfor sUpdate;
    sEthUpdateVariable  sUpdateVar;
    uint8_t         Status_u8;
	uint8_t 		PHYstatus_u8;
    uint8_t         ReConnect_u8;
    uint32_t        LandMarkRun_u32;
    uint8_t         Retry_u8;
    uint16_t        cHardReset_u16;
    uint16_t        cSoftReset_u16;
    uint32_t        LandMarkConnMQTT_u32;
        
    uint8_t         *pModeConnNow_u8;
    uint8_t         *pModeConnLast_u8;
    uint8_t         *pModeConnFuture_u8;
    
    uint8_t     ( *pReset_MCU ) (void);
    uint8_t     ( *pReset_MCU_Imediate ) (void);
    void        ( *pReq_Save_Box ) (void);    
    sData*      ( *pGet_Fw_Version ) (void);
}sEthernetVariable;

typedef uint8_t (*Function_Send) (void);
typedef uint8_t (*Callback_Success) (sData *pData);

typedef struct
{
	uint8_t             CmdName_u8;
    Function_Send       pFunc_Send;
	Callback_Success	pCb_Success;    
    char* 	            sResp;
} sCommandEth; 
 

extern sEvent_struct 			sEventAppEth[];
extern sEthernetVariable		sAppEthVar;

extern sCommandEth      sEthCmd[];
extern sCommandEth      sEthResCode[];

extern uint8_t aETH_MQTT_CONN[2];
extern sEthFTPvariable sFTPvar;
extern uint8_t aETH_FTP_CONN[7];
/*===================== Extern struct var ==============*/


/*===================== Funcion ========================*/

void    AppEth_Init(void);
uint8_t AppEth_Task(void);

uint8_t AppEth_W5500_Hard_Reset(void);
void    AppEth_W5500_Init(void);

void 	AppEth_Net_Config (void);
void 	AppEth_Display_Net_Conf(void);
uint8_t AppEth_Get_PHYStatus(void);


int32_t AppEth_Socket_Control (uint8_t sn, uint8_t* destip, uint16_t destport, uint16_t *any_port);

uint8_t AppEth_IRQ_Handler(void);  

void 	AppEth_Cb_IP_Assign (void);
void 	AppEth_Cb_IP_Conflict (void);

int8_t 	AppEth_DHCP_Start (void);

uint8_t AppEth_Send_Mess (void);
void    AppEth_Restart_If_PSM(void);

uint8_t AppEth_Get_Queue(uint8_t Type);
uint8_t AppEth_Push_Cmd_To_Queue(uint8_t cmd);
uint8_t AppEth_Push_Block_To_Queue(uint8_t *blockCmd, uint8_t size);

void    AppEth_Init_Default (void);
void    AppEth_FTP_Init_Default (void);

int     pportc(char * arg);
uint8_t AppEth_FTP_DataSock_Run (void);


void    AppEth_Default_Struct_UpdateVar (void);
uint8_t AppEth_Save_Firmware (uint8_t *pData, uint16_t length);
void    AppEth_Finish_Update (uint8_t status);

#endif  


