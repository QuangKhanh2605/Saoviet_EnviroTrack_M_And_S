
/*
    8/2021
    Thu vien quan ly modem
*/

#ifndef USER_MODEM_H
#define USER_MODEM_H

#include "user_util.h"

#include "user_define.h"
#include "user_lpm.h"

/*===================Define=========================*/
#define MAX_DCU_ID_LENGTH	    20

#define LEGNTH_AT_SERIAL        256
#define MAX_LENGTH_ALAMR_EMER   512

#define MAX_LENGTH_IP		    40
#define MAX_LENGTH_PORT		    10
#define MAX_LENGTH_USER_PASS	40

#define MAX_LENGTH_PATH			30
#define MAX_LENGTH_NAME         30

#define MAX_LENGTH_URL          200

/*===================Struct, Var=========================*/
/*
 * 		KIND DCU
 */
typedef enum
{
	_MODEM_WM_NBIOT,
	_MODEM_WM_GSM,
	_MODEM_COS_PHI,
	_MODEM_EM_NBIOT,
    _MODEM_LEVEL,
    _MODEM_WM_GSM_LORA,
    _MODEM_TEMH_GSM,
    _MODEM_CONV_GSM,
    _CONTROL_OXY,
    _MODEM_WM_GSM_ETH,
}sTypeModem;


typedef enum
{
    MODE_CONNECT_DATA_MAIN = 0,		
    MODE_CONNECT_DATA_BACKUP,		
    MODE_CONNECT_FTP,		       
    MODE_CONNECT_HTTP , 
}sKindSimConnect;


typedef enum
{
    __SERVER_IP,
    __SERVER_DOMAIN,
}eKindServer;


typedef enum
{
    __HARD_ONLINE_ONLY,
    __HARD_ONLINE_PSM,
}eKindHardWare;

typedef struct
{
    uint8_t			NumWakeup_u8;           //So lan lay mau roi gui ban tin: Che do SAVE_MODE
    uint32_t        Duration_u32;           //Chu ki thuc day
    
    uint32_t		DurOnline_u32;          //Tan suat gui ban tin: che do ONLINE
	uint16_t		UnitMin_u16;            //Gia tri dong goi vao ban tin: min
}sFrequenceSend;


typedef struct
{
	sData  	sIP;
	sData  	sPort;
	sData  	sMQTTUserName;
	sData  	sMQTTPassword;
    
    uint8_t     aIP[4];
    uint16_t    Port_u16;
    
    uint32_t	MQTTKeepAlive_u32;
    uint8_t     DomainOrIp_u8;
}sModemServerInfor;

typedef struct 
{
    sData	FirmIP;
    sData   FirmPort;
    sData	FTPUser;
    sData	FTPPass;
    
    sData	FirmPath;
    sData	FirmName;
    sData   UrlHTTP;
    
    uint8_t     aIP[4];
    uint16_t    Port_u16;
    uint8_t     DomainOrIp_u8;
}sModemUpdateinfor;

typedef struct
{
    uint8_t         Type_u8;            //Loai Modem
    sData  			sId;                //ID
    uint8_t         ModePower_u8;       //Che do hoat dong: online hay psm
    
    sFrequenceSend  sFrequence;       
    
    sModemServerInfor  sServerMain;
    sModemServerInfor  sServerBack;
    sModemUpdateinfor  sUpdate;
    
    uint8_t        DetHardware_u8;
}sModemInformation;

typedef struct
{
	//Information
    uint8_t         ModeConnNow_u8;
    uint8_t         ModeConnFuture_u8;
    uint8_t         ModeConnLast_u8;  
    
	uint8_t			rSaveBox_u8;		    //Flag danh dau vao che do Save box
    uint8_t	        CountSleep_u8;          //So lan sleep de thuc day gui ban tin
    
    sData           strATResp;
    sData           strAlarm;

    uint8_t         CountConfig_u8;
    
    ST_TIME_FORMAT  sRTCSet;
}sModemVariable;



typedef struct 
{
	uint8_t     Name_u8;
	sData	    sTopic;					
} sKindModem;



/*===================External Var, Struct=========================*/
extern sModemVariable	    sModemVar;
extern sModemInformation    sModemInfor;
extern StructpFuncLPM       sModemLPMHandle;
extern const sKindModem     sSV_Modem[];
/*================Extern var struct=====================*/


/*===================Function=========================*/
/*------ Cac function xu ly cac task----------------*/
void    Modem_Init(void);
void 	Modem_Init_ID(void);
void 	Modem_Save_ID(void);

void    Modem_Init_Freq (void);
void    Modem_Save_Freq (void);

void    Modem_Init_Server_Infor(void);
void    Modem_Save_Server_Infor (void);

uint16_t Modem_Init_u16 (uint32_t Addr, uint16_t MAX);
void    Modem_Save_u16 (uint32_t Addr, uint16_t Val);

uint8_t Modem_Reset_MCU (void);
uint8_t Modem_Reset_MCU_Immediately (void);

void    Modem_Packet_Alarm (uint8_t *pData, uint16_t length);
void    Modem_Packet_Alarm_String (const char *str);
//Func Modem App
void    Modem_Respond(uint8_t portNo, uint8_t *data, uint16_t length, uint8_t ack);
void    Modem_Respond_Str(uint8_t portNo, const char *str, uint8_t ack);

void    Modem_Deinit_Peripheral (void);
void    Modem_Init_Peripheral (void);
void    Modem_Init_Before_IRQ_Handle (void);
void    Modem_Deinit_Before_IRQ_Handle (void);

void    MX_GPIO_DeInit(void);
void    Modem_Uart_DeInit(void);
void    Modem_Init_Gpio_Again(void);

/*Func Cb Setting serial*/
#ifdef USING_AT_CONFIG
void    Modem_SER_Set_ID (sData *str, uint16_t Pos);
void    Modem_SER_Get_ID (sData *str, uint16_t Pos);

void    Modem_SER_Set_Duty_Cycle (sData *strRecei, uint16_t Pos);
void    Modem_SER_Get_Duty_Cycle (sData *strRecei, uint16_t Pos);
void    Modem_SER_Set_Dura_Online (sData *strRecei, uint16_t Pos);
void    Modem_SER_Get_Dura_Online (sData *strRecei, uint16_t Pos);

void    Modem_SER_Set_Numwakeup (sData *strRecei, uint16_t Pos);
void    Modem_SER_Set_sTime (sData *strRecei, uint16_t Pos);
void    Modem_SER_Get_sTime (sData *strRecei, uint16_t Pos);
void    Modem_SER_Get_Firm_Version (sData *strRecei, uint16_t Pos);
void    Modem_SER_Req_Reset (sData *strRecei, uint16_t Pos);


void    Modem_Extract_Sever_Infor (sData *strRecei, uint16_t Pos,
                                sData *sIPGet, sData *sPortGet, sData *sUserGet, sData *sPassGet);
void    Modem_SER_Set_Server_Main (sData *strRecei, uint16_t Pos);
uint8_t Modem_Set_Server_Main (sData *pIP, sData *pPort, sData *pUser, sData *pPass, uint32_t Alive);
void    Modem_SER_Get_Server_Main (sData *strRecei, uint16_t Pos);

void    Modem_SER_Set_Server_Back (sData *strRecei, uint16_t Pos);
uint8_t Modem_Set_Server_Back (sData *pIP, sData *pPort, sData *pUser, sData *pPass, uint32_t Alive);
void    Modem_SER_Get_Server_Back (sData *strRecei, uint16_t Pos);

void    Modem_SER_Change_Server (sData *strRecei, uint16_t Pos);

void    Modem_SER_HTTP_Update (sData *strRecei, uint16_t Pos);
uint8_t Modem_Set_Http_Update (sData *pIP, sData *pPort, sData *pUser, sData *pPass,
                                        sData *pPath, sData *pName, sData *pURL);

void    Modem_SER_FTP_Update (sData *strRecei, uint16_t Pos);
uint8_t Modem_Set_FTP_Update (sData *pIP, sData *pPort, sData *pUser, sData *pPass,
                                        sData *pPath, sData *pName, sData *pURL);

void    Modem_Set_Update_Infor (sData *pIP, sData *pPort, sData *pUser, sData *pPass,
                                        sData *pPath, sData *pName, sData *pURL);

void    Modem_SER_Get_Level_Debug (sData *str_Receiv, uint16_t Pos);
void    Modem_SER_Set_Level_Debug (sData *str_Receiv, uint16_t Pos);

#endif

uint8_t Modem_Check_AT(sData *StrUartRecei, uint8_t Type);
uint8_t	Modem_Split_Update_Infor (sData *str_Receiv, uint16_t Pos);

void    Modem_Packet_MePDV (sData *pData);
void    Modem_Packet_Mess_Opera (sData *pData);
void    Modem_Packet_Mess_Status (sData *pData);
void    Modem_Packet_Mess_Intan (sData *pData);
void    Modem_Packet_Mess_Alarm (sData *pData);
void    Modem_Packet_Mess_RespAT (sData *pData);
void    Modem_Packet_Mess_SimID (sData *pData);

void    Modem_SER_Config (sData *strRecv, uint16_t pos);
void    Modem_SER_Setting (sData *strRecv, uint16_t pos);
void    Modem_SER_Set_Time (sData *strRecv, uint16_t Pos);
void    Modem_SER_Req_AT (sData *strRecv, uint16_t pos);
void    Modem_SER_Req_RF (sData *strRecv, uint16_t pos);
void    Modem_SER_Req_Update (sData *strRecv, uint16_t pos);

void    Modem_Set_Server_Infor_To_App(void);
void    Modem_Monitor_Connect_Server (void);


uint8_t Modem_Set_sTime (ST_TIME_FORMAT *sClock, uint8_t GMT);
sData*  Modem_Get_Firm_Version (void);
sData*  Modem_Get_ID (void);
uint8_t Modem_Change_Server (void);



#endif /*  */






