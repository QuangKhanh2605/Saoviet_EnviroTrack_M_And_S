



#include "user_mqtt.h"

#include "user_string.h"

#ifdef PACKET_MQTT_PROTOCOL
#include "MQTTConnect.h"
#include "MQTTFormat.h"
#include "MQTTPacket.h"
#include "MQTTPublish.h"
#include "MQTTSubscribe.h"
#include "MQTTUnsubscribe.h"
#include "StackTrace.h"
#endif


/*======================== Structs var======================*/
const struct_MARK_TYPE_Message_SEND     sMark_MessageSend_Type[] =
{
    //Mess wait ACK
    {TOPIC_NOTIF,						NULL,								{(uint8_t*)"Notif",5}},

    {DATA_HANDSHAKE,					_mDATA_HANDSHAKE,				    {(uint8_t*)"MePDV",5}}, //

    {DATA_TSVH_MULTI,				    _mDATA_TSVH_MULTI,			        {(uint8_t*)"Multi",5}}, //
    {DATA_TSVH_FLASH,				    _mDATA_TSVH_FLASH,			        {(uint8_t*)"Flash",5}}, //
    {DATA_TSVH_OPERA,				    _mDATA_TSVH_OPERA,			        {(uint8_t*)"Opera",5}}, //
    {DATA_INTAN,				        _mDATA_INTAN,				        {(uint8_t*)"Intan",5}}, //
    
    {DATA_REC_HISTORY,				    _mDATA_REC_HISTORY,				    {(uint8_t*)"Debug",5}}, //

    {DATA_EVENT,					    _mDATA_EVENT,				        {(uint8_t*)"Event",5}}, //
    {DATA_GPS,					        _mDATA_GPS,				            {(uint8_t*)"Gpsda",5}}, //
    
    {SEND_SIM_ID,					    _mSEND_SIM_ID,					    {(uint8_t*)"SimID",5}},
    {SEND_RESPOND_SERVER_ACK,	    	_mSEND_RESPOND_SERVER_ACK,			{(uint8_t*)"sCoFi",5}},
    {SEND_SAVE_BOX_FAIL,				_mSEND_SAVE_BOX_FAIL,				{(uint8_t*)"sCoFi",5}},
    {SEND_SAVE_BOX_OK,				    _mSEND_SAVE_BOK_OK,				    {(uint8_t*)"sCoFi",5}},
    
    {SEND_UPDATE_FIRM_OK,				_mSEND_UPDATE_FIRM_OK,				{(uint8_t*)"Notif",5}},    
    {SEND_UPDATE_FIRM_FAIL,				_mSEND_UPDATE_FIRM_FAIL,			{(uint8_t*)"Notif",5}},    
    
    //Mess No wait ACK
    {SEND_RESPOND_SERVER,				_mSEND_RESPOND_SERVER,				{(uint8_t*)"sCoFi",5}},
    {SEND_RESPOND_FROM_RF,				_mSEND_RESPOND_FROM_RF,				{(uint8_t*)"ReqRF",5}},
    {SEND_SHUTTING_DOWN,				_mSEND_SHUTTING_DOWN,				{(uint8_t*)"Reset",5}},
    {SEND_RECONNECT_SERVER,				_mSEND_RECONNECT_SERVER,			{(uint8_t*)"ReCon",5}},
    {SEND_HARD_RS_MCU,				    _mSEND_HARD_RS_MCU,			        {(uint8_t*)"Alarm",5}},
    {SEND_ALARM,				        _mSEND_ALARM,				        {(uint8_t*)"Alarm",5}},
    {DATA_PING,							_mDATA_PING,						{(uint8_t*)"mPing",5}}, //
    {DATA_PING_2,					    _mDATA_PING,						{(uint8_t*)"mPing",5}}, //
    {DATA_STATUS,				        _mDATA_STATUS,				        {(uint8_t*)"Statu",5}}, //
    //
    {SEND_SERVER_TIME_PENDING,			_mSEND_SERVER_TIME_PENDING,			{(uint8_t*)"sTime",5}},    // {(uint8_t*)"Realtime pending",16}},
    {SEND_SERVER_TIME_OK,				_mSEND_SERVER_TIME_OK,				{(uint8_t*)"Notif",5}},    // {(uint8_t*)"Realtime OK",11}},
    {SEND_SERVER_TIME_FAIL,				_mSEND_SERVER_TIME_FAIL,			{(uint8_t*)"Notif",5}},    // {(uint8_t*)"Realtime fail",13}},
    {SEND_EMPTY_MESS,				    _mSEND_EMPTY_MESS,				    {(uint8_t*)"Notif",5}},    // {(uint8_t*)"Empty Mess FLash!",17}},

    {END_MQTT_SEND,						NULL,								{NULL}},
};

/* */
const struct_MARK_TYPE_Message_Receiv sMark_MessageRecei_Type[] =
{
    // ban tin danh rieng cho SIM
    {REQUEST_RESET,				_rREQUEST_RESET,				{(uint8_t*)"Reset",5}},
    {INTAN_DATA,                _rINTAN_DATA,                   {(uint8_t*)"Intan",5}},
    {REQUEST_SETTING,			_rREQUEST_SETTING,				{(uint8_t*)"sCoFi",5}},
    {SER_SETTING_PARAM,			_sSETTING_PARAM,				{(uint8_t*)"sSett",5}},
    {UPDATE_TIME_SERVER,		_rUPDATE_TIME_SERVER,			{(uint8_t*)"sTime",5}},
    {RESET_DATA,				_rRESET_DATA,	    			{(uint8_t*)"Clear",5}},
    {REQUEST_AT,				_rREQ_AT_CMD,	    			{(uint8_t*)"ReqAT",5}},
    {REQUEST_RF,				_rREQ_AT_TO_RF,	    			{(uint8_t*)"ReqRF",5}},
    
    {REQUEST_UPDATE_FIRM,	    _rUPDATE_FIRMWARE,		        {(uint8_t*)"rFirm",5}},
    {REQUEST_STATUS,		    _rREQUEST_STATUS,			    {(uint8_t*)"Statu",5}},
    
    {END_MQTT_RECEI,			NULL,							{NULL}},
};

static uint8_t aDATA_FROM_MEM[1024];

struct_MQTT				sMQTT = 
{
    .sDataFromMem = {aDATA_FROM_MEM, 0},
};

#ifdef PACKET_MQTT_PROTOCOL
MQTTPacket_connectData 	Connect_Packet = MQTTPacket_connectData_initializer;
MQTTString 				topicString = MQTTString_initializer;
#endif

uint8_t	aSubcribe_Header[HEADER_LENGTH];
uint8_t	aPublish_Header[HEADER_LENGTH];		//Header

uint8_t	aDATA_MQTT[MAX_LENGTH_MQTT];        //Buffer chưa data da duoc dong goi MQTT: Bao gom Payload + Header MQTT

/*======================== Function ======================*/

void mSet_default_MQTT(void)
{
	sMQTT.str.Data_a8 		= aDATA_MQTT;
	sMQTT.str.Length_u16 	= 0;

    sMQTT.Status_u8         = FALSE;
}



/*
 * 		FUNC : ghi ra chuoi Header Publish message day du
 * Param :
 * 		- sData *sTarget : Chuoi su dung *
 * 		- Step_Data				: Dang Header
 * 		- Kind 					: 0 Type Message in table  sMark_MessageSend_Type
 * 								: 1 Type Message in table  sMark_MessageRecei_Type
 * Return :
 * 		0 - FALSE 	: 	ERROR :tran chuoi copy vao,
 * 		1 - True 	:	Tao Thanh Cong
 */

uint8_t Write_publish_header(sData *sTarget, int Step_Data, uint8_t Kind)
{
    //Set up Header Pub again
    UTIL_MEM_set(aPublish_Header, 0, sizeof (aPublish_Header));

    sTarget->Data_a8 = &aPublish_Header[0];
    sTarget->Length_u16 = 0;

    Copy_String(sTarget, sMQTT.sTopic, 255, HEADER_LENGTH);

    if (Kind == 1)
		Copy_String(sTarget, (sData *)&sMark_MessageRecei_Type[Step_Data].sKind, 255, HEADER_LENGTH);
	else
        Copy_String(sTarget, (sData *)&sMark_MessageSend_Type[Step_Data].sKind, 255, HEADER_LENGTH);

	*(sTarget->Data_a8 + sTarget->Length_u16++) = '/';

	Copy_String(sTarget, sMQTT.sModemID, 255, HEADER_LENGTH);

	return 1;
}



void mData_MQTT (sData *sPayload, uint8_t MessType, uint8_t Qos)
{
    //Dua con tro MQTT.str.data_u8 tro dau mang aDATA_MQTT
    mSet_default_MQTT();        

#ifdef PACKET_MQTT_PROTOCOL
    sMQTT.PulishQos = Qos;  
	sMQTT.PulishRetained = 0;
	sMQTT.PulishPacketId = 1;

    Write_publish_header(&sMQTT.sPublishHeader, MessType, 0);
	topicString.cstring = (char*) sMQTT.sPublishHeader.Data_a8;

    sMQTT.str.Length_u16 = MQTTSerialize_publish(sMQTT.str.Data_a8, sizeof(aDATA_MQTT), 0, sMQTT.PulishQos, \
            sMQTT.PulishRetained, sMQTT.PulishPacketId, topicString, sPayload->Data_a8, sPayload->Length_u16);
#endif
    
#ifdef PACKET_HTTP_PROTOCOL
    for (uint16_t i = 0; i < sPayload->Length_u16; i++)  
    {
        *(sMQTT.str.Data_a8 + sMQTT.str.Length_u16++) = *(sPayload->Data_a8 + i);
    }
#endif
}

/*============= Func Callback ============================*/

uint8_t _mDATA_HANDSHAKE(int Kind_Send)
{
    uint8_t     aTEMP_PAYLOAD[512] = {0};
    sData       strPayload = {aTEMP_PAYLOAD, 0};

    if (sMQTT.pPacket_Handshake != NULL)
        sMQTT.pPacket_Handshake(&strPayload);
    //Send MQTT
    mData_MQTT(&strPayload, Kind_Send, MY_QOS);

    return 1;
}


uint8_t _mDATA_TSVH_FLASH(int Kind_Send)
{
    //Get Data Payload
    uint8_t     aTEMP_PAYLOAD[1024] = {0};
    sData       strPayload = {aTEMP_PAYLOAD, 0};
    
    if (sMQTT.pPacket_Opera != NULL)
        sMQTT.pPacket_Opera(&strPayload);
        
    mData_MQTT(&strPayload, Kind_Send, MY_QOS);
    
    return 1;
}



uint8_t _mDATA_TSVH_MULTI(int Kind_Send)
{
    //Get Data Payload
    uint8_t     aTEMP_PAYLOAD[1024] = {0};
    sData       strPayload = {aTEMP_PAYLOAD, 0};
    
    if (sMQTT.pPacket_Opera != NULL)
        sMQTT.pPacket_Opera(&strPayload);
        
    mData_MQTT(&strPayload, Kind_Send, MY_QOS);
    
    return 1;
}


uint8_t _mDATA_TSVH_OPERA (int Kind_Send)
{   
    //Get Data Payload
    uint8_t     aTEMP_PAYLOAD[1024] = {0};
    sData       strPayload = {aTEMP_PAYLOAD, 0};

    if (sMQTT.pPacket_Opera != NULL)
        sMQTT.pPacket_Opera(&strPayload);
    
    mData_MQTT(&strPayload, Kind_Send, MY_QOS);
    
    return 1;
}


uint8_t _mDATA_INTAN(int Kind_Send)
{
    uint8_t     aTEMP_PAYLOAD[1024] = {0};
    sData       strPayload = {aTEMP_PAYLOAD, 0};
    
    if (sMQTT.pPacket_Intan != NULL)
        sMQTT.pPacket_Intan(&strPayload);

    mData_MQTT(&strPayload, Kind_Send, MY_QOS);

    return 1;
}


uint8_t _mDATA_REC_HISTORY(int Kind_Send)
{
    
    
    return 1;
}



uint8_t _mSEND_SHUTTING_DOWN(int Kind_Send)
{
    sData sPayLoadShutdown = {(uint8_t*)"MCU shutting down", 17};

    mData_MQTT(&sPayLoadShutdown, Kind_Send, 0);

    return 1;
}

uint8_t _mSEND_RECONNECT_SERVER(int Kind_Send)
{
    sData sPayLoadReConn = {(uint8_t*)"MQTT ReConnect!", 15};

    mData_MQTT(&sPayLoadReConn, Kind_Send, 0);

    return 1;
}

uint8_t _mSEND_HARD_RS_MCU(int Kind_Send)
{
    sData sPayLoadReConn = {(uint8_t*)"Alarm: Hard Reset MCU!", 23};

    mData_MQTT(&sPayLoadReConn, Kind_Send, 0);

    return 1;
}


uint8_t _mSEND_ALARM(int Kind_Send)
{
    uint8_t     aTEMP_PAYLOAD[512] = {0};
    sData       strPayload = {aTEMP_PAYLOAD, 0};

    if (sMQTT.pPacket_Alarm != NULL)
        sMQTT.pPacket_Alarm(&strPayload);
       
    mData_MQTT(&strPayload, Kind_Send, 0);
        
    return 1;
}



/*
 * 		PUBLISH MESSAGE
 */
uint8_t _mDATA_PING(int Kind_Send)
{   
//    mSet_default_MQTT(); 
//    
//    sMQTT.PulishQos = 0;  
//	sMQTT.PulishRetained = 0;
//	sMQTT.PulishPacketId = 1;
//
//    *(sMQTT.str.Data_a8 + sMQTT.str.Length_u16++) = MY_PINGREQ;
//    *(sMQTT.str.Data_a8 + sMQTT.str.Length_u16++) = 0x00;
  
    mSet_default_MQTT(); 
    
    sMQTT.PulishQos = 0;  
	sMQTT.PulishRetained = 0;
	sMQTT.PulishPacketId = 1;
    
    uint8_t var = 0;
    uint8_t	    i = 0;
	uint8_t     TempCrc = 0;
    uint8_t     aTEMP_PAYLOAD[128] = {0};
    sData       strPayload = {aTEMP_PAYLOAD, 0};
    
    Reset_Buff(&sMQTT.str);
    *(strPayload.Data_a8 + strPayload.Length_u16++) = '#';
	// SIM_ID
	for (var = 0; var < sSimCommInfor.sSIMid.Length_u16; ++var)
		*(strPayload.Data_a8 + strPayload.Length_u16++)  = sSimCommInfor.sSIMid.Data_a8[var];
    //Insert Version
    *(strPayload.Data_a8 + strPayload.Length_u16++)  = '#';
    for (var = 0; var < sFirmVersion.Length_u16; ++var)
		*(strPayload.Data_a8 + strPayload.Length_u16++)  = sFirmVersion.Data_a8[var];

    *(strPayload.Data_a8 + strPayload.Length_u16++)  = '#';
    
#ifdef USING_APP_SIM
    *(strPayload.Data_a8 + strPayload.Length_u16++)  = 'C';
    *(strPayload.Data_a8 + strPayload.Length_u16++)  = 'S';
    *(strPayload.Data_a8 + strPayload.Length_u16++)  = 'Q';
    *(strPayload.Data_a8 + strPayload.Length_u16++)  = '-';
    
    Convert_Uint64_To_StringDec(&strPayload, sSimCommInfor.RSSI_u8, 0);
    
    *(strPayload.Data_a8 + strPayload.Length_u16++)  = '#';
#endif

#ifdef USING_APP_SIM
//    if (sSimVar.ModeConnectNow_u8 == MODE_CONNECT_DATA_MAIN)
//    {
        //IP
//        *(sMQTT.sPayload.Data_a8 + sMQTT.sPayload.Length_u16++) = sSimInfor.sServer.sIPMain.Length_u16 + sSimInfor.sServer.sPortMain.Length_u16 + 1; // length
        for (var = 0; var < sSimCommInfor.sServer.sIP->Length_u16 ; ++var)
            *(strPayload.Data_a8 + strPayload.Length_u16++)  = sSimCommInfor.sServer.sIP->Data_a8[var];
        //Port
        *(strPayload.Data_a8 + strPayload.Length_u16++) = ':'; // length
        for (var = 0; var < sSimCommInfor.sServer.sPort->Length_u16; ++var)
            *(strPayload.Data_a8 + strPayload.Length_u16++)  = sSimCommInfor.sServer.sPort->Data_a8[var];
//    } else
//    {
//        //IP
////         *(sMQTT.sPayload.Data_a8 + sMQTT.sPayload.Length_u16++) = sSimInfor.sServer.sIPBackUp.Length_u16 + sSimInfor.sServer.sPortBackUp.Length_u16 + 1;
//        for (var = 0; var < sSimCommInfor.sServer.sIP.Length_u16; ++var)
//            *(sMQTT.str.Data_a8 + sMQTT.str.Length_u16++)  = sSimCommInfor.sServer.sIPBackUp.Data_a8[var];
//        //Port
//        *(sMQTT.str.Data_a8 + sMQTT.str.Length_u16++) = ':';
//        for (var = 0; var < sSimCommInfor.sServer.sPortBackUp.Length_u16; ++var)
//            *(sMQTT.str.Data_a8 + sMQTT.str.Length_u16++)  = sSimCommInfor.sServer.sPortBackUp.Data_a8[var];
//    }
    
    *(strPayload.Data_a8 + strPayload.Length_u16++)  = '#';
#endif
    
    	// caculator crc
	strPayload.Length_u16++;
	for (i = 0; i < (strPayload.Length_u16 - 1); i++)
		TempCrc ^= *(strPayload.Data_a8 + i);

       
    mData_MQTT(&strPayload, Kind_Send, MY_QOS);
    return 1;
}



uint8_t _mDATA_STATUS(int Kind_Send)
{   
    uint8_t     aTEMP_PAYLOAD[128] = {0};
    sData       strPayload = {aTEMP_PAYLOAD, 0};
    
    if (sMQTT.pPacket_Status != NULL)
        sMQTT.pPacket_Status(&strPayload);
    
    mData_MQTT(&strPayload, Kind_Send, 0);
    
    return 1;
}



uint8_t _mDATA_EVENT(int Kind_Send)
{        
    //Get Data Payload
    uint8_t     aTEMP_PAYLOAD[128] = {0};
    sData       strPayload = {aTEMP_PAYLOAD, 0};

    if (sMQTT.pPacket_Opera != NULL)
        sMQTT.pPacket_Opera(&strPayload);
    
    mData_MQTT(&strPayload, Kind_Send, MY_QOS);
    
    return 1;
}


uint8_t _mDATA_GPS(int Kind_Send)
{        
    //Get Data Payload
    uint8_t     aTEMP_PAYLOAD[1024] = {0};
    sData       strPayload = {aTEMP_PAYLOAD, 0};
    
    if (sMQTT.pPacket_Opera != NULL)
        sMQTT.pPacket_Opera(&strPayload);
    
    mData_MQTT(&strPayload, Kind_Send, MY_QOS);

    return 1;
}



uint8_t _mSEND_SIM_ID(int Kind_Send)
{
    uint8_t     aTEMP_PAYLOAD[32] = {0};
    sData       strPayload = {aTEMP_PAYLOAD, 0};
    
    if (sMQTT.pPacket_SimID != NULL)
        sMQTT.pPacket_SimID(&strPayload);

    mData_MQTT(&strPayload, Kind_Send, MY_QOS);

    return 1;
}



uint8_t _mSEND_RESPOND_SERVER(int Kind_Send)
{
	uint8_t     aTEMP_PAYLOAD[256] = {0};
    sData       strPayload = {aTEMP_PAYLOAD, 0};

    if (sMQTT.pPacket_RespAT != NULL)
        sMQTT.pPacket_RespAT(&strPayload);

    mData_MQTT(&strPayload, Kind_Send, 0);

    return 1;
}


uint8_t _mSEND_RESPOND_FROM_RF(int Kind_Send)
{
	uint8_t     aTEMP_PAYLOAD[256] = {0};
    sData       strPayload = {aTEMP_PAYLOAD, 0};

    if (sMQTT.pPacket_RespAT != NULL)
        sMQTT.pPacket_RespAT(&strPayload);

    mData_MQTT(&strPayload, Kind_Send, 0);

    return 1;
}




uint8_t _mSEND_RESPOND_SERVER_ACK(int Kind_Send)
{
    uint8_t     aTEMP_PAYLOAD[256] = {0};
    sData       strPayload = {aTEMP_PAYLOAD, 0};

    if (sMQTT.pPacket_RespAT != NULL)
        sMQTT.pPacket_RespAT(&strPayload);
    
    mData_MQTT(&strPayload, Kind_Send, MY_QOS);

    return 1;
}


//
void mPayload_Load_MesErr(sData *pData, sData *MesErr)
{
 	uint8_t var;

	for (var = 0; var < MesErr->Length_u16; ++var)
		*(pData->Data_a8 + pData->Length_u16++) = *(MesErr->Data_a8 + var);
}

uint8_t _mSEND_SERVER_TIME_PENDING(int Kind_Send)
{
    sData sPayTimePending = {(uint8_t *) "Realtime pending",16};

    mData_MQTT(&sPayTimePending, Kind_Send, 0);

    return 1;
}

uint8_t _mSEND_SERVER_TIME_OK(int Kind_Send)
{
    sData sPayTimeOK = {(uint8_t *) "Realtime OK", 11};

    mData_MQTT(&sPayTimeOK, Kind_Send, 0);

    return 1;
}


uint8_t _mSEND_SERVER_TIME_FAIL(int Kind_Send)
{
    sData sPayTimeFail = {(uint8_t *) "Realtime fail", 13};

    mData_MQTT(&sPayTimeFail, Kind_Send, 0);

    return 1;
}

uint8_t _mSEND_SAVE_BOX_FAIL(int Kind_Send)
{
    sData sPaySaveBoxFail = {(uint8_t*)"Luu Kho Fail", 12};

    mData_MQTT(&sPaySaveBoxFail, Kind_Send, MY_QOS);

    return 1;
}

uint8_t _mSEND_SAVE_BOK_OK(int Kind_Send)
{
    sData sPaySaveBoxOK = {(uint8_t*)"\r\nOK", 4};

    mData_MQTT(&sPaySaveBoxOK, Kind_Send, MY_QOS);

    return 1;
}




uint8_t _mSEND_EMPTY_MESS(int Kind_Send)
{
    sData sPayEmpFlashFail = {(uint8_t*)"Empty Mess FLash!", 17};
  
    mData_MQTT(&sPayEmpFlashFail, Kind_Send, 0);

    return 1;
}


uint8_t _mSEND_UPDATE_FIRM_OK(int Kind_Send)
{
    sData       strFirmSuccess = {(uint8_t*)"Update Firmware Successfully!", 29};
    uint8_t     aTEMP_PAYLOAD[64] = {0};
    sData       strPayload = {aTEMP_PAYLOAD, 0};
    
	mPayload_Load_MesErr(&strPayload, &strFirmSuccess);

    UTIL_Log(DBLEVEL_M, strFirmSuccess.Data_a8, strFirmSuccess.Length_u16);

#ifdef USING_APP_WM
    sData strWaitPulse = {(uint8_t*)" - Wait increase 5 pulse", 24};

    if ( (sPulse[0].IsOver5Pulse_u8 == false) || (sPulse[1].IsOver5Pulse_u8 == false) )
    {
        mPayload_Load_MesErr(&strPayload, &strWaitPulse);
        UTIL_Log(DBLEVEL_M, strWaitPulse.Data_a8, strWaitPulse.Length_u16);
    }
#endif

    mData_MQTT(&strPayload, Kind_Send, MY_QOS);

    return 1;
}


uint8_t _mSEND_UPDATE_FIRM_FAIL(int Kind_Send)
{
    sData   strFirmFail = {"Update Firmware Fail!", 21};
    uint8_t aTEMP_PAYLOAD[64] = {0};
    sData   strPayload = {aTEMP_PAYLOAD, 0};

	mPayload_Load_MesErr(&strPayload, &strFirmFail);

    UTIL_Log(DBLEVEL_M, strFirmFail.Data_a8, strFirmFail.Length_u16);

#ifdef USING_APP_WM
    sData strWaitPulse = {(uint8_t*)" - Wait increase 5 pulse", 24};

    if ( (sPulse[0].IsOver5Pulse_u8 == false) || (sPulse[1].IsOver5Pulse_u8 == false) )
    {
        mPayload_Load_MesErr(&strPayload, &strWaitPulse);
        UTIL_Log(DBLEVEL_M, strWaitPulse.Data_a8, strWaitPulse.Length_u16);
    }
#endif

    mData_MQTT(&strPayload, Kind_Send, MY_QOS);

    return 1;
}




/*---------------Function Callbacck recei-----------------------*/

void _rREQUEST_RESET(sData *str_Receiv, int16_t Pos)
{
    sMQTT.aMARK_MESS_PENDING[SEND_SHUTTING_DOWN] = TRUE;
}

void _rINTAN_DATA (sData *str_Receiv,int16_t Pos)
{
#ifdef USING_APP_WM
//	//31/8/2022: intan -> measure ap suat -> send Intan
//	sWmVar.rIntan_u8 = true;
	//Active event Measure Press
	fevent_active(sEventAppWM, _EVENT_ENTRY_WM);
#endif
}


void _rREQUEST_SETTING(sData *str_Receiv, int16_t Pos)
{
    if (sMQTT.pSER_Config != NULL)
        sMQTT.pSER_Config(str_Receiv, Pos + sMark_MessageRecei_Type[REQUEST_SETTING].sKind.Length_u16);
}


void _sSETTING_PARAM(sData *str_Receiv, int16_t Pos)
{
    if (sMQTT.pSER_Setting != NULL)
        sMQTT.pSER_Setting(str_Receiv, Pos + sMark_MessageRecei_Type[REQUEST_SETTING].sKind.Length_u16);
}

void _rUPDATE_TIME_SERVER(sData *str_Receiv, int16_t Pos) 
{
    if (sMQTT.pSER_Set_Time != NULL)
        sMQTT.pSER_Set_Time(str_Receiv, Pos + sMark_MessageRecei_Type[UPDATE_TIME_SERVER].sKind.Length_u16);
}

void _rRESET_DATA(sData *str_Receiv,int16_t Pos)
{
    if (sMQTT.pSER_Clear_Rec != NULL)
        sMQTT.pSER_Clear_Rec(str_Receiv, Pos);
}

void _rREQ_AT_CMD(sData *str_Receiv,int16_t Pos)
{    
    if (sMQTT.pSER_Req_AT != NULL)
        sMQTT.pSER_Req_AT(str_Receiv, Pos);
}

void _rREQ_AT_TO_RF(sData *str_Receiv,int16_t Pos)
{
    if (sMQTT.pSER_Req_RF != NULL)
        sMQTT.pSER_Req_RF(str_Receiv, Pos);
}




void _rUPDATE_FIRMWARE(sData *str_Receiv,int16_t Pos)
{
    if (sMQTT.pSER_Req_Update != NULL)
        sMQTT.pSER_Req_Update(str_Receiv, Pos);
}


void _rREQUEST_STATUS(sData *str_Receiv,int16_t Pos)
{
    sMQTT.aMARK_MESS_PENDING[DATA_STATUS] = true;
}


/*---------------Function handle-----------------------*/
void mInit_Header_Subcribe (void)
{
#ifdef PACKET_MQTT_PROTOCOL
	sData           Temp 			= {(uint8_t*)"/#",2};
	MQTTString      topicSubcri[] 	= {MQTTString_initializer,MQTTString_initializer};
	int			    MsgId, ReqQos[] = {SUB_QOS,SUB_QOS};
	sData 	        cHeaderSUB_8u   = {(uint8_t*)"AMI/",4} ;
    uint8_t		    aHeader1[HEADER_LENGTH] = {0};
    uint8_t		    aHeader2[HEADER_LENGTH] = {0};
    sData sHeader[] = { {aHeader1, 0}, {aHeader2, 0} };

    // init
    UTIL_MEM_set(aSubcribe_Header, 0, sizeof(aSubcribe_Header));
	sMQTT.sSubcribe.Data_a8				= aSubcribe_Header;
	sMQTT.sSubcribe.Length_u16 			= 0;
    // Subcriber
    // Topic 1
	Copy_String(&sHeader[0], (sData *) &cHeaderSUB_8u, 255, HEADER_LENGTH);
	Copy_String(&sHeader[0], sMQTT.sModemID, 255, HEADER_LENGTH);
	Copy_String(&sHeader[0], &Temp, 255, HEADER_LENGTH);

	topicSubcri[0].cstring = (char*)sHeader[0].Data_a8;
	MsgId = MY_ID1 | (MY_ID2 << 8);
    // Topic 2
	Copy_String(&sHeader[1],(sData *)&cHeaderSUB_8u,255,HEADER_LENGTH);
	Copy_String(&sHeader[1],(sData *)&sMark_MessageRecei_Type[UPDATE_TIME_SERVER].sKind,255,HEADER_LENGTH);

	topicSubcri[1].cstring = (char*)sHeader[1].Data_a8;

	sMQTT.sSubcribe.Length_u16 = MQTTSerialize_subscribe(sMQTT.sSubcribe.Data_a8, HEADER_LENGTH, 0, MsgId, 2, topicSubcri, ReqQos);
#endif
}



void mConnect_MQTT (char *ClientID, char *sUser, char *sPass, uint8_t KeepAlive)
{
#ifdef PACKET_MQTT_PROTOCOL
	mSet_default_MQTT();
        
    sMQTT.KeepAliveTime_u8 = KeepAlive;
        
	Connect_Packet.clientID.cstring = ClientID;
	Connect_Packet.keepAliveInterval = sMQTT.KeepAliveTime_u8;
	Connect_Packet.cleansession = 1;
    
    Connect_Packet.username.cstring = (char*)sUser;
    Connect_Packet.password.cstring = (char*)sPass;

	//dua ban tin chua id username pass vao buff
	sMQTT.str.Length_u16 = MQTTSerialize_connect(sMQTT.str.Data_a8, sizeof(aDATA_MQTT), &Connect_Packet);
#endif
}



void mSubcribe_MQTT (void)
{
    mInit_Header_Subcribe();

	mSet_default_MQTT();
	sMQTT.str.Data_a8 		= sMQTT.sSubcribe.Data_a8;
	sMQTT.str.Length_u16 	= sMQTT.sSubcribe.Length_u16;
}

/*
    Func: Check empty mess
*/
uint8_t mCheck_Empty_Mess (void)
{
    uint16_t i = 0;

    for (i = TOPIC_NOTIF; i < END_MQTT_SEND; i++)
        if (sMQTT.aMARK_MESS_PENDING[i] == 1)
            return 0;

    return 1;
}




/*
    Func: Get buffer data from Flash
        + Check crc and format again: 
            + False: -> Luu vao flash part B and skip 
            + true: Copy data to buff sim mqtt
                    mark new mess to send now
*/

void mGet_Data_From_Mem (uint8_t MessType, uint8_t *pData, uint16_t Length)
{ 
    //Memset buff 
    Reset_Buff (&sMQTT.sDataFromMem);
          
    if (Length > sizeof (aDATA_FROM_MEM) )
        Length = sizeof (aDATA_FROM_MEM);
    
    for (uint16_t i = 0; i < Length; i++)
        *(sMQTT.sDataFromMem.Data_a8 + sMQTT.sDataFromMem.Length_u16++) = *(pData + i);
    
    //active event send mess mqtt
    sMQTT.aMARK_MESS_PENDING[MessType] = TRUE;  
}



void mUnmark_Mess (uint8_t TypeMess)
{
    sMQTT.aMARK_MESS_PENDING[TypeMess] = FALSE; 
    Reset_Buff(&sMQTT.sDataFromMem);
}

void mUnmark_Mess_Share_Buff (void)
{
    mUnmark_Mess(DATA_TSVH_MULTI);
    mUnmark_Mess(DATA_TSVH_FLASH);
    mUnmark_Mess(DATA_TSVH_OPERA); 
    mUnmark_Mess(DATA_EVENT); 
    mUnmark_Mess(DATA_GPS); 
}


