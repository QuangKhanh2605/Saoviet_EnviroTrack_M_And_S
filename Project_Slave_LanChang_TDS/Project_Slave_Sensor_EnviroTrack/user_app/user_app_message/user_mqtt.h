/*
    8/2021
    Thu vien xu ly Uart
*/
#ifndef USER_MQTT_H
#define USER_MQTT_H

#include "user_define.h"
#include "user_util.h"
#include "user_obis_sv.h"
/*======================== Define ======================*/
#define PACKET_MQTT_PROTOCOL

#define MY_CONNECT             0x10
#define MY_CONNACK             0x20
#define MY_PUBLISH             0x32
#define MY_PUBACK              0x40
#define MY_SUBSCRIBE           0x82
#define MY_SUBACK              0x90
#define MY_PINGREQ             0xC0
#define MY_PINGRESP            0xD0
#define MY_DISCONNECT          0xE0
#define SUB_QOS                0x00
#define MY_QOS                 0x01
#define MY_CONNECTIONFLASG     0XC2
#define MY_MQTTVERSION         0x03

#define MY_ID1 					0x0A
#define MY_ID2 					0x00


#define HEADER_LENGTH 			50
#define MAX_LENGTH_MQTT  		1200

#define TOTAL_SLOT				30

#define NONE					0
#define SUCCESS					1

#define TIME_DELAY_MQTT			60000
#define	TIME_RETRY				3

#define	TSVH_OPERA              0
#define	TSVH_FLASH              1
#define	TSVH_MULTI              2


#ifdef PAYLOAD_HEX_FORMAT
#define	TSVH_OPERA_OR_FLASH		TSVH_OPERA  //TSVH_MULTI //  TSVH_FLASH
#else
#define	TSVH_OPERA_OR_FLASH		TSVH_OPERA
#endif
/*======================== Structs var======================*/
typedef enum {

    //Mess wait ACK
	TOPIC_NOTIF,            //0

    DATA_HANDSHAKE,         //1
        
    DATA_TSVH_MULTI,
	DATA_TSVH_FLASH,        //2
    DATA_TSVH_OPERA,        //3
    DATA_INTAN,             //4
    
    DATA_REC_HISTORY,       //5

    DATA_EVENT,             //6
    DATA_GPS,               //7

	SEND_SIM_ID,            //8
    SEND_RESPOND_SERVER_ACK, // 9
    SEND_SAVE_BOX_FAIL,     //10
    SEND_SAVE_BOX_OK,       //11
    
    SEND_UPDATE_FIRM_OK,    //12
    SEND_UPDATE_FIRM_FAIL,  //13
    //Mess no wait ACK      
    SEND_RESPOND_SERVER,    //14
    SEND_RESPOND_FROM_RF,   //15    
	SEND_SHUTTING_DOWN,     //16
    SEND_RECONNECT_SERVER,  //17    
    SEND_HARD_RS_MCU,       //18
    SEND_ALARM,             //19
    DATA_PING,              //20
    DATA_PING_2,            //21
    DATA_STATUS,            //22

	SEND_SERVER_TIME_PENDING,   //23
	SEND_SERVER_TIME_OK,        //24
	SEND_SERVER_TIME_FAIL,      //25
    SEND_EMPTY_MESS,            //26

	END_MQTT_SEND,
} Type_Message_MQTT_Send;

typedef enum {
	REQUEST_RESET,
    INTAN_DATA,
	REQUEST_SETTING,
    SER_SETTING_PARAM,
	UPDATE_TIME_SERVER,
    RESET_DATA,
    REQUEST_AT,
    REQUEST_RF,
    
    REQUEST_UPDATE_FIRM,
    REQUEST_STATUS,
    
	END_MQTT_RECEI,
} Type_Message_MQTT_Receiv;
//

typedef uint8_t (*task_callback_Send)(int Kind_Send);
typedef struct {
	int 				idMark;
	task_callback_Send 	CallBack;
	sData		sKind;
} struct_MARK_TYPE_Message_SEND;


//
typedef void (*task_callback_r)(sData *str_Receiv, int16_t Pos);
typedef struct {
	int 			idMark;
	task_callback_r	CallBack;
	sData	        sKind;
} struct_MARK_TYPE_Message_Receiv;


typedef struct {
	int				aMARK_MESS_PENDING[TOTAL_SLOT];	// Danh dau ban tin can SEND di (0-2 : ban tin quan trong; 3-12 : ban tin data; 13 - log -> clear khi reset)
// Cal Data
	uint32_t		LandmarkSendPing_u32;
	sData 	        str;
	sData 	        strHexData;
// 	publish
    sData 	        sPublishHeader;
	int 			PulishQos;
	int				SubcribeQos;
	unsigned char	PulishRetained;
	unsigned short	PulishPacketId;
//  Subcribe
    sData           *sTopic;
    sData           *sModemID;
	sData 	        sSubcribe;
	int 			MsgId;
	int				ReqQos;
	uint8_t 		KeepAliveTime_u8;
    //
	uint32_t		LandMarkLastSend_u32;   //Thoi gian cuoi cung gui len server. De check trong truong hop Danh thuc bang PIN
    uint8_t         MessType_u8;            //Danh dau ban tin dang chuan bi gui di
    uint8_t         Status_u8;              //Flag dang xu ly Send:   2: Pending, 1: Success 0: False
    uint8_t         TypeConfigServer_u8;    //Type Config Server: Wait ACK:  1: Config FreqSend;  3: ConfigSaveBox.    
    sData           sDataFromMem;
    
    void    (*pPacket_Handshake) (sData *pData);
    void    (*pPacket_Status) (sData *pData);
    void    (*pPacket_Opera) (sData *pData);
    void    (*pPacket_Intan) (sData *pData);
    void    (*pPacket_Alarm) (sData *pData);
    void    (*pPacket_RespAT) (sData *pData);
    void    (*pPacket_SimID) (sData *pData);
    
    
    void    (*pSER_Config) (sData *strRecv, uint16_t pos);
    void    (*pSER_Setting) (sData *strRecv, uint16_t pos);
    void    (*pSER_Set_Time) (sData *strRecv, uint16_t pos);
    void    (*pSER_Clear_Rec) (sData *strRecv, uint16_t pos);
    void    (*pSER_Req_AT) (sData *strRecv, uint16_t pos);
    void    (*pSER_Req_RF) (sData *strRecv, uint16_t pos);
    void    (*pSER_Req_Update) (sData *strRecv, uint16_t pos);
    
} struct_MQTT;


/*======================== External Var struct ======================*/

extern const struct_MARK_TYPE_Message_SEND sMark_MessageSend_Type[];
extern const struct_MARK_TYPE_Message_Receiv sMark_MessageRecei_Type[];

extern struct_MQTT					sMQTT;

/*======================== Function ======================*/

/*---------------Function Callback send-----------------------*/
uint8_t     _mDATA_HANDSHAKE(int Kind_Send);

uint8_t     _mDATA_TSVH_MULTI(int Kind_Send);
uint8_t 	_mDATA_TSVH_FLASH(int Kind_Send);
uint8_t     _mDATA_TSVH_OPERA (int Kind_Send);
uint8_t 	_mDATA_REC_HISTORY(int Kind_Send);
uint8_t     _mDATA_INTAN(int Kind_Send);
uint8_t 	_mSEND_SHUTTING_DOWN(int Kind_Send);
uint8_t 	_mDATA_PING(int Kind_Send);
uint8_t 	_mSEND_SIM_ID(int Kind_Send);

uint8_t     _mDATA_EVENT(int Kind_Send);
uint8_t     _mDATA_GPS(int Kind_Send);

uint8_t		_mSEND_RESPOND_SERVER(int Kind_Send);
uint8_t     _mSEND_RESPOND_FROM_RF(int Kind_Send);
uint8_t		_mSEND_RESPOND_SERVER_ACK(int Kind_Send);

uint8_t		_mSEND_SERVER_TIME_PENDING(int Kind_Send);
uint8_t		_mSEND_SERVER_TIME_OK(int Kind_Send);
uint8_t		_mSEND_SERVER_TIME_FAIL(int Kind_Send);
uint8_t		_mSEND_SAVE_BOX_FAIL(int Kind_Send);
uint8_t     _mSEND_SAVE_BOK_OK(int Kind_Send);
uint8_t		_mSEND_EMPTY_MESS(int Kind_Send);

uint8_t     _mSEND_UPDATE_FIRM_OK(int Kind_Send);
uint8_t     _mSEND_UPDATE_FIRM_FAIL(int Kind_Send);
uint8_t     _mSEND_RECONNECT_SERVER(int Kind_Send);
uint8_t     _mSEND_HARD_RS_MCU(int Kind_Send);
uint8_t     _mSEND_ALARM(int Kind_Send);
uint8_t     _mDATA_STATUS(int Kind_Send);

/*---------------Function Calback receive-----------------------*/
void 		_rREQUEST_RESET(sData *str_Receiv,int16_t Pos);
void 		_rINTAN_DATA (sData *str_Receiv,int16_t Pos);
void        _rREQUEST_SETTING(sData *str_Receiv,int16_t Pos);
void 		_rUPDATE_TIME_SERVER(sData *str_Receiv,int16_t Pos);
void        _sSETTING_PARAM(sData *str_Receiv, int16_t Pos);
void 		_rRESET_DATA(sData *str_Receiv,int16_t Pos);
void        _rREQ_AT_CMD(sData *str_Receiv,int16_t Pos);
void        _rREQ_AT_TO_RF(sData *str_Receiv,int16_t Pos);
void        _rUPDATE_FIRMWARE(sData *str_Receiv,int16_t Pos);
void        _rREQUEST_STATUS(sData *str_Receiv,int16_t Pos);

/*---------------Function handle-----------------------*/
//handle Send
void        mInit_Header_Subcribe (void);
void 		mSet_default_MQTT(void);
uint8_t 	mWrite_publish_header(sData *sTarget,int Step_Data, uint8_t Kind);
void        mPayload_Load_MesErr(sData *pData, sData *MesErr);

void 		mInit_Header_MQTT(void);
void        mConnect_MQTT (char *ClientID, char *sUser, char *sPass, uint8_t KeepAlive);
void     	mSubcribe_MQTT (void);

void        mData_MQTT (sData *sPayload, uint8_t MessType, uint8_t Qos);
uint8_t     mCheck_Empty_Mess (void);
void        mRespond_Config_Server (uint8_t portNo, uint8_t *data, uint16_t length);

void        mUnmark_Mess_Share_Buff (void);
void        mUnmark_Mess (uint8_t TypeMess);
void        mGet_Data_From_Mem (uint8_t MessType, uint8_t *pData, uint16_t Length);


#endif
