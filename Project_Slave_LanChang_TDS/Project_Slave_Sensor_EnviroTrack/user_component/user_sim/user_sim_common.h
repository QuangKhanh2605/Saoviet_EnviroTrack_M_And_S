


#ifndef USER_SIM_COMMON_H_
#define USER_SIM_COMMON_H_

#include "user_util.h"
#include "main.h"

#include "onchipflash.h"
#include "queue_p.h"
#include "user_sim_config.h"


/*=========== Define =====================*/
#define SIM_ID_LENGTH			21

/*===== Char MQTT =======*/
#define MY_CONNACK              0x20
#define MY_PUBACK               0x40
#define MY_SUBACK               0x90
#define MY_PINGRESP             0xD0

#define MAX_LEVEL_CSQ     	    100
#define MAX_CSQ     	        0x20
#define MIN_CSQ      	        0

#define SIM_CMD_TIMEOUT		    10000
#define SIM_CMD_TIMEOUT2 	    5000
#define SIM_TCP_TIMEOUT		    60000
#define SIM_CALL_TIMEOUT	    40000
#define SIM_AUDIO_TIMEOUT	    40000
/* -----------Setting----------*/

#define CID_SERVER              0x32
#define HTTP_HEADER_MAX         3

/*=========== Struct =====================*/

typedef uint8_t (*CB_AT_Success) (sData *str_Receive);
typedef uint8_t (*CB_AT_Failure) (uint8_t Type);
typedef struct
{
	uint8_t at_name;
	CB_AT_Success	callback_success;
    char 	*at_response;
    char	*at_string;
} sCommand_Sim_Struct; // it is not a declaration, it is a new type of data


typedef struct
{
	sData  	*sIP;
	sData  	*sPort;
	sData  	*sMQTTUserName;
	sData  	*sMQTTPassword;
    
    uint32_t *MQTTKeepAlive_u32;
}sServerInformation;


typedef struct
{
    sData sName;
    sData sDialString;
	sData sUsername;
	sData sPassword;
}sAPNInformation;


typedef struct
{
	sData		sSIMid;      //String Sim ID
	uint8_t		RSSI_u8;     //RSSI module SIM
	uint8_t		Ber_u8;
    uint8_t		RSSILevel_u8;  
    sData       sImei;
    
    sAPNInformation *sAPNInfor;
    
    //Server Infor
    sServerInformation sServer;
    ST_TIME_FORMAT  sTime; 
    
    uint16_t    EarFcn_u16;
    uint16_t    EarFcnOffs_u16;
    uint16_t    nCellID_u16;
    char        aCELL_ID[20];
    int16_t     scRSRP_i16;
    int16_t     scRSRQ_i16;
    int16_t     scRSSI_i16;
    uint8_t     Band_u8;
    
} sSimInformation;

typedef struct
{
    uint8_t Status_u8;
    uint8_t Code_u8;
}StructErrorCode;


typedef struct
{       
	uint8_t 	CallReady_u8;		        // Bao card sim
    uint8_t     DataMode_u8;  
    
//    uint8_t     Status_u8;
    uint8_t     State_u8;
    uint8_t     Wakeup_u8;
    //status sim
    uint8_t     RxPinReady_u8;          // Chan Rx OK: Khi nhan dc ki tu tu uart rx
    uint8_t     TxPinReady_u8;          // Truyen 1 lenh vao co phan hoi lai -> Tx Pin OK
    uint8_t     NetReady_u8;            // CGATT = 1
    uint8_t     ServerReady_u8;         // Connect Server
    
    uint8_t     HttpHeaderIndex_u8;
    
    sData       pSender;
    StructErrorCode  sErrorCode;
}sSimVariable;


typedef struct 
{
    sData	*pFirmIP;
    sData   *pFirmPort;
    sData	*pFirmPath;
    sData	*pFirmName;
    sData   *pUrlHTTP;
}sUpdateinformation;

typedef struct
{
    sUpdateinformation  sUpdateInfor;
    
    uint32_t	FirmLength_u32;
	sData		strFirmLength;			        // string Length receive from SIM module

	uint8_t		DataTemp_u8;				    // Bo dem Uart FTP
	uint32_t	AddSave_u32;                    //Add Save Fw
    
	uint8_t		Pending_u8;				        // Doi tinh toan uart FTP
	uint32_t	CountByteTotal_u32;
    //
    uint32_t    AddOffset_u32;
    uint16_t    CountPacket_u8;
    uint8_t     LastCrcFile_u8;
    uint8_t     LasCrcCal_u8;
    uint8_t     IsFirmSaoViet_u8;
    
    uint8_t     FailStatus_u8;
}sSimUpdateVariable;


typedef enum
{
	_SIM_COMM_URC_RESET_SIM = 0,
	_SIM_COMM_URC_SIM_LOST,
	_SIM_COMM_URC_SIM_REMOVE,
	_SIM_COMM_URC_CLOSED,
	_SIM_COMM_URC_PDP_DEACT,
	_SIM_COMM_URC_CALL_READY,
    _SIM_COMM_URC_CALLING,
    _SIM_COMM_URC_CALL_FAIL,
    _SIM_COMM_URC_SMS_CONTENT,
    _SIM_COMM_URC_SMS_IND,
	_SIM_COMM_URC_ERROR,
    _SIM_COMM_URC_RECEIV_SERVER,
	_SIM_COMM_URC_END,
}KindSimCommonURC;


typedef enum 
{
    // Don't need feedback
	SEND_UPDATE_FIRMWARE,               //0
	SEND_UPDATE_FIRMWARE_PENDING,       //1
	SEND_UPDATE_FIRMWARE_OK,            //2
	SEND_UPDATE_FIRMWARE_FAIL_SETUP,    //3    
	SEND_UPDATE_FIRMWARE_FAIL_TIME,     //4
	SEND_UPDATE_FIRMWARE_FAIL_FLASH,    //5    
	SEND_UPDATE_FIRMWARE_FAIL_LENGTH,   //6
	SEND_ERASE_FLASH_SUCCESS,           //7
	SEND_ERASE_FLASH_FAIL,              //8
}Struct_Step_UpdateFirm;
    


/*====== Ma loi module sim======*/
typedef enum
{
    _SIM_COMM_ERROR_PIN_RX,          // k nhan dc rx uart: do chan rx hoac power on fail
    _SIM_COMM_ERROR_PIN_TX,          // Truyen Tx vao va k nha dc gi
    _SIM_COMM_ERROR_CARD,            //Loi CPIN not ready: k co card sim
    _SIM_COMM_ERROR_ATTACH,          //Loi CGATT chua OK
    _SIM_COMM_ERROR_TCP,             //Chua ket noi TCP
    _SIM_COMM_ERROR_NONE,
}eSimErrorCode;



typedef struct
{
    uint8_t     aNAME[50];
    uint8_t     LengthName_u8;
    
    uint8_t     aTEMP_STR_HANDLE[5];
    sData       strHandle;
    
    uint32_t    Handle_u32;        
}sSimFileVariable;


typedef enum
{
    _SIM_START,	        //0
    _SIM_INIT,          //1
    _SIM_NETWORK,       //2
    _SIM_SETTING,       //3
    _SIM_INIT_TCP,      //4
    _SIM_OPENED_TCP,    //5    
    _SIM_CONN_MQTT,	    //6    
    _SIM_CONN_HTTP,	    //7
    _SIM_POWER_OFF,     //8
    _SIM_PSM,           //9
}sKindStateSim;


typedef enum
{
	_SIM_COMM_SIM_CARD,
    _SIM_COMM_GET_STIME, 
    
    _SIM_COMM_CALLING_1,
    _SIM_COMM_CALLING_2,
    
    _SIM_COMM_SMS_SEND_1,
    _SIM_COMM_SMS_SEND_2,
    _SIM_COMM_SMS_SEND_3,
    
    _SIM_COMM_TCP_SEND_1,
    _SIM_COMM_TCP_SEND_2,
    
    _SIM_COMM_MQTT_CONN_1,
    _SIM_COMM_MQTT_CONN_2,
    
    _SIM_COMM_MQTT_SUB_1,
    _SIM_COMM_MQTT_SUB_2,
    
    _SIM_COMM_MQTT_PUB_1,
    _SIM_COMM_MQTT_PUB_2,
    
    _SIM_COMM_HTTP_HEAD_1,
    _SIM_COMM_HTTP_HEAD_2,
    
    _SIM_COMM_HTTP_SEND_1,
    _SIM_COMM_HTTP_SEND_2,
    
    _SIM_COMM_HTTP_SEND_EX,
    
    _SIM_COMM_HTTP_UPDATE_OK,
    _SIM_COMM_HTTP_UPDATE_FAIL,
    
    _SIM_COMM_HTTP_READ_OK,
    
    _SIM_COMM_AUDIO_OK,
    
}sKindCallBackAtSIM;

typedef struct
{
    void        ( *pFailure ) (uint8_t step);  
    void        ( *pHandle_AT ) (uint8_t step); 
    void        ( *phandle_URC ) (uint8_t Type);

    sData*      ( *pGet_Fw_Version ) (void);
    void        ( *pGet_GPS_OK ) (uint8_t *pData, uint16_t Length);
    void        ( *pGet_GPS_ERROR ) (void);
    void        ( *pConfig_Update ) (void);
                                     
    void        ( *pSend ) (uint8_t *pData, uint16_t length); 
}sFuncCallbackSim;

/*===================== extern ====================*/
extern sFuncCallbackSim         sCbSimComm;
extern sSimInformation	        sSimCommInfor;
extern sSimVariable		        sSimCommVar;
extern sSimUpdateVariable       sSimFwUpdate; 
extern sSimFileVariable         sFileSys;


/*================= Struct =================*/

uint8_t Sim_Common_Get_Sim_ID (sData *uart_string);
uint8_t Sim_Common_Get_CSQ (sData *strRecv);
uint8_t Sim_Common_Get_Imei(sData *strRecv);

uint8_t Sim_Common_CPIN_Ready (sData *strRecv);
uint8_t Sim_Common_Get_CCLK (sData *strRecv);
uint8_t Sim_Common_Tranparent(sData *strRecv);
uint8_t Sim_Common_Non_Tranparent(sData *strRecv);
uint8_t Sim_Common_Check_Attached (sData *strRecv);

uint8_t Sim_Common_SMS_Send_1 (sData *uart_string);
uint8_t Sim_Common_SMS_Send_2 (sData *uart_string);
uint8_t Sim_Common_SMS_Send_3 (sData *uart_string);

uint8_t Sim_Common_Call_1 (sData *uart_string);
uint8_t Sim_Common_Call_2 (sData *uart_string);

uint8_t Sim_Common_TCP_Close(sData *strRecv);
uint8_t Sim_Common_TCP_Open_1 (sData *strRecv);
uint8_t Sim_Common_TCP_Open_2 (sData *strRecv);

uint8_t Sim_Common_Ping (sData *strRecv);

uint8_t Sim_Common_TCP_Send_1 (sData *strRecv);
uint8_t Sim_Common_TCP_Send_2 (sData *strRecv);
uint8_t Sim_Common_MQTT_Conn_1 (sData *strRecv);
uint8_t Sim_Common_MQTT_Conn_2 (sData *strRecv);
uint8_t Sim_Common_MQTT_Sub_1 (sData *strRecv);
uint8_t Sim_Common_MQTT_Sub_2 (sData *strRecv);
uint8_t Sim_Common_MQTT_Pub_1 (sData *strRecv);
uint8_t Sim_Common_MQTT_Pub_2 (sData *strRecv);

//Http
uint8_t Sim_Common_HTTP_Head_1 (sData *strRecv);
uint8_t Sim_Common_HTTP_Head_2 (sData *strRecv);
uint8_t Sim_Common_HTTP_Send_1 (sData *strRecv);
uint8_t Sim_Common_HTTP_Send_2 (sData *strRecv);

uint8_t Sim_Common_HTTP_Send_Ex (sData *strRecv);
uint8_t Sim_Common_Http_Read_Data (sData *uart_string);

//Call func URC
uint8_t Sim_Common_URC_Reset (sData *strRecv);
uint8_t Sim_Common_URC_Lost_Card (sData *strRecv);
uint8_t Sim_Common_URC_Closed (sData *strRecv);
uint8_t Sim_Common_URC_Error (sData *strRecv);
uint8_t Sim_Common_URC_Recv_Server (sData *strRecv);
uint8_t Sim_Common_URC_Calling (sData *strRecv);
uint8_t Sim_Common_URC_Call_Fail (sData *strRecv);
uint8_t Sim_Common_URC_Sms_Content (sData *strRecv);
uint8_t Sim_Common_URC_Sms_Indi (sData *strRecv);
uint8_t Sim_Common_URC_Call_Ready (sData *strRecv);

uint8_t Sim_Common_Init_UpdateFw (void);
void    Sim_Common_Init_Struct_UpdateFw (void);
//
void    Sim_Common_Send_AT_Cmd (uint8_t *string, uint16_t length); 
void    Sim_Common_Init_Struct_FileSys (void);

sSimInformation *pSim_Common_Get_Infor (void);

void    Sim_Common_Init (sSimVariable *sSimCommVar);
void    Sim_Common_Queue_Clear (Struct_Queue_Type *pQueue);


#endif
