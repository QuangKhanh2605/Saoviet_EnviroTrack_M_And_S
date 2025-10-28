/*
    8/2021
    Thu vien Quan ly Modem
*/

#include "user_modem.h"

#include "user_rtc.h"
#include "onchipflash.h"
//#include "user_adc.h"
#include "dma.h"
        
        

/*===================Struct, Var=========================*/
ST_TIME_FORMAT          sRTCSet = {0};

/*================ struct Feature=====================*/
static uint8_t aMODEM_ID[MAX_DCU_ID_LENGTH] = DEVICE_ID;
static uint8_t aAT_RESPONSE [LEGNTH_AT_SERIAL];
static uint8_t aALARM [MAX_LENGTH_ALAMR_EMER];
  
/*=== Server Infor Buffer*/  // 
static uint8_t aIP_SERVER[MAX_LENGTH_IP]  = "124.158.5.154";  //124.158.5.154 port 1883   117.6.163.177 
static uint8_t aPORT_SERVER[MAX_LENGTH_PORT] = "1883";  // 1883
static uint8_t aUSER_MQTT[MAX_LENGTH_USER_PASS] = "syswmsv";  // "syswmsv";
static uint8_t aPASS_MQTT[MAX_LENGTH_USER_PASS] = "sv@wmsv568";  // "sv@wmsv568";

static uint8_t aIP_SERVER_BACKUP[MAX_LENGTH_IP] = "117.6.163.177";  
static uint8_t aPORT_SERVER_BACKUP[MAX_LENGTH_PORT] = "1883";
static uint8_t aUSER_MQTT_BACKUP[MAX_LENGTH_USER_PASS] = "syswmsv";
static uint8_t aPASS_MQTT_BACKUP[MAX_LENGTH_USER_PASS] = "sv@wmsv568";

//Buff FTP file
static uint8_t aUPDATE_IP[MAX_LENGTH_IP]={"124.158.5.154"};
static uint8_t aUPDATE_PORT[MAX_LENGTH_PORT]={"21"};
static uint8_t aUPDATE_USER[MAX_LENGTH_USER_PASS]={"saoviet"};
static uint8_t aUPDATE_PASS[MAX_LENGTH_USER_PASS]={"sv123@"};

static uint8_t aUPDATE_PATH[MAX_LENGTH_PATH] = {"FTP/chien"};
static uint8_t aUPDATE_NAME[MAX_LENGTH_NAME] = {"SVWM_GSM_ETH_V1_1_6.bin"};
static uint8_t aUPDATE_URL[MAX_LENGTH_URL]= {"http://172.16.11.141:8909/FIRM_HCMCHES1_4.bin"};

sModemVariable	sModemVar = 
{
    .ModeConnNow_u8 = MODE_CONNECT_DATA_MAIN,
    .strATResp  = {&aAT_RESPONSE[0], 0},   
    .strAlarm   = {&aALARM[0], 0},
};

sModemInformation sModemInfor = 
{
    .Type_u8        = _CONTROL_OXY,
    .sId            = {&aMODEM_ID[0], 14},
    .ModePower_u8   = _POWER_MODE_SAVE,  // _POWER_MODE_ONLINE,    //  _POWER_MODE_SAVE,  // 
    
    .sFrequence.NumWakeup_u8    = DEFAULT_NUMBER_WAKE_UP,  
    .sFrequence.Duration_u32    = DEFAULT_FREQ_WAKEUP,
        
    .sFrequence.DurOnline_u32   = DEFAULT_FREQ_SEND_ONLINE,
    .sFrequence.UnitMin_u16     = 2,
    
    //Server Infor
    .sServerMain.sIP                = {&aIP_SERVER[0], 13},
    .sServerMain.sPort              = {&aPORT_SERVER[0], 4},
    .sServerMain.sMQTTUserName      = {&aUSER_MQTT[0], 7},
    .sServerMain.sMQTTPassword      = {&aPASS_MQTT[0], 10},
    
    .sServerMain.aIP = {124, 158, 5, 154},  // {117, 6, 163, 177},
    .sServerMain.Port_u16 = 1883,
    
    .sServerMain.MQTTKeepAlive_u32 = 60,
    .sServerMain.DomainOrIp_u8 = __SERVER_IP,
    
    //Server Infor
    .sServerBack.sIP                = {&aIP_SERVER_BACKUP[0], 13},  // 24
    .sServerBack.sPort              = {&aPORT_SERVER_BACKUP[0], 4},
    .sServerBack.sMQTTUserName      = {&aUSER_MQTT_BACKUP[0], 7},
    .sServerBack.sMQTTPassword      = {&aPASS_MQTT_BACKUP[0], 10},
    
    .sServerBack.aIP = {124, 158, 5, 154},
    .sServerBack.Port_u16 = 7525,
    
    .sServerBack.MQTTKeepAlive_u32 = 60,
    .sServerBack.DomainOrIp_u8 = __SERVER_DOMAIN, 
    
    //Update
    .sUpdate.FirmIP      = {&aUPDATE_IP[0], 9},
    .sUpdate.FirmPort    = {&aUPDATE_PORT[0], 2},
    .sUpdate.FTPUser    = {&aUPDATE_USER[0], 6},  
    .sUpdate.FTPPass    = {&aUPDATE_PASS[0], 6},
    
    .sUpdate.FirmPath    = {&aUPDATE_PATH[0], 13},
    .sUpdate.FirmName    = {&aUPDATE_NAME[0], 23},
    .sUpdate.UrlHTTP        = {&aUPDATE_URL[0], 50},
    
    .sUpdate.aIP = {124, 158, 5, 154},  
    .sUpdate.Port_u16 = 21,
    
    .sUpdate.DomainOrIp_u8 = __SERVER_IP,
};


StructpFuncLPM sModemLPMHandle = 
{
    .LPM_Deinit_Peripheral          = Modem_Deinit_Peripheral,  
    .LPM_Init_Peripheral            = Modem_Init_Peripheral,
    .LPM_Deinit_Before_IRQ_Handle   = Modem_Deinit_Before_IRQ_Handle,
    .LPM_Init_Before_IRQ_Handle     = Modem_Init_Before_IRQ_Handle,
};
    


const sKindModem sSV_Modem[] =
{
	{ _MODEM_WM_NBIOT,              {(uint8_t*)"DCU/WMSV/WME1/",14}}, // 15 - fix 26_03_18  DCU/EMSV/SVE1/
	{ _MODEM_WM_GSM,	            {(uint8_t*)"DCU/WMSV/WMP2/",14}},
	{ _MODEM_COS_PHI,		        {(uint8_t*)"DCU/EMET/COSP/",14}},
	{ _MODEM_EM_NBIOT,		        {(uint8_t*)"DCU/EMSV/SVE1/",14}},
    { _MODEM_LEVEL,		            {(uint8_t*)"DCU/WMSV/WML1/",14}},
    { _MODEM_WM_GSM_LORA,		    {(uint8_t*)"DCU/WMSV/WMLG/",14}},
    { _MODEM_TEMH_GSM,		        {(uint8_t*)"DCU/THSV/THGS/",14}},
    { _MODEM_CONV_GSM,		        {(uint8_t*)"DCU/WMSV/WMPI/",14}},
    { _CONTROL_OXY,                 {(uint8_t*)"DCU/WMSV/WMCO/",14}},
#ifdef PAYLOAD_HEX_FORMAT
    { _MODEM_WM_GSM_ETH,		    {(uint8_t*)"DCU/WMSV/WMGE/",14}},
#else
    { _MODEM_WM_GSM_ETH,		    {(uint8_t*)"DCU/WMDC/WMGE/",14}},
#endif
};

static uint8_t aDATA_CONFIG[128];

static uint8_t aAT_CMD[LENGTH_AT_SERIAL];
sData strATcmd = {aAT_CMD, 0};

/*===================Function=========================*/    


void Modem_Init(void)
{
    Modem_Init_Freq();
	Modem_Init_ID();
    Modem_Init_Server_Infor();

#ifdef USING_AT_CONFIG
    /* regis cb serial */
    CheckList_AT_CONFIG[_SET_DEV_SERIAL].CallBack = Modem_SER_Set_ID;
    CheckList_AT_CONFIG[_QUERY_DEV_SERIAL].CallBack = Modem_SER_Get_ID;
    
    CheckList_AT_CONFIG[_SET_DUTY_READ_DATA].CallBack = Modem_SER_Set_Duty_Cycle;
    CheckList_AT_CONFIG[_QUERY_DUTY_READ_DATA].CallBack = Modem_SER_Get_Duty_Cycle;
    
    CheckList_AT_CONFIG[_SET_FREQ_ONLINE].CallBack = Modem_SER_Set_Dura_Online;
    CheckList_AT_CONFIG[_QUERY_FREQ_ONLINE].CallBack = Modem_SER_Get_Dura_Online;
    
    CheckList_AT_CONFIG[_SET_NUM_WAKEUP].CallBack = Modem_SER_Set_Numwakeup;
    
    CheckList_AT_CONFIG[_SET_RTC].CallBack = Modem_SER_Set_sTime;
    CheckList_AT_CONFIG[_QUERY_RTC].CallBack = Modem_SER_Get_sTime;  
    
    CheckList_AT_CONFIG[_SET_LEVEL_DEBUG].CallBack = Modem_SER_Set_Level_Debug;
    CheckList_AT_CONFIG[_QUERY_LEVEL_DEBUG].CallBack = Modem_SER_Get_Level_Debug; 
    
    CheckList_AT_CONFIG[_RESET_MODEM].CallBack = Modem_SER_Req_Reset;
    CheckList_AT_CONFIG[_QUERY_FIRM_VER].CallBack = Modem_SER_Get_Firm_Version;
    
    CheckList_AT_CONFIG[_SET_SERVER_MAIN].CallBack = Modem_SER_Set_Server_Main;
    CheckList_AT_CONFIG[_QUERY_SERVER_MAIN].CallBack = Modem_SER_Get_Server_Main;
    
    CheckList_AT_CONFIG[_SET_SERVER_BACKUP].CallBack = Modem_SER_Set_Server_Back;
    CheckList_AT_CONFIG[_QUERY_SERVER_BACKUP].CallBack = Modem_SER_Get_Server_Back;
 
    CheckList_AT_CONFIG[_REQ_SWITCH_SERVER].CallBack = Modem_SER_Change_Server;
    
    CheckList_AT_CONFIG[_QUERY_UP_FIRM_HTTP].CallBack = Modem_SER_HTTP_Update;
    CheckList_AT_CONFIG[_QUERY_UP_FIRM_FTP].CallBack = Modem_SER_FTP_Update;
#endif
    
#ifdef USING_APP_SIM
    /*regis cb for app sim*/
    sAppSimVar.pRespond = Modem_Respond ;
    sAppSimVar.pReset_MCU = Modem_Reset_MCU;
    sAppSimVar.pReset_MCU_Imediate = Modem_Reset_MCU_Immediately;
    sAppSimVar.pReq_Set_sTime = Modem_Set_sTime;
    sAppSimVar.pReq_Save_Box = NULL;
    sAppSimVar.pCheck_AT_User = Modem_Check_AT;
    
    sAppSimVar.pModePower_u8 = &sModemInfor.ModePower_u8;
    sAppSimVar.pModeConnNow_u8 = &sModemVar.ModeConnNow_u8;
    sAppSimVar.pModeConnFuture_u8 = &sModemVar.ModeConnFuture_u8;
    sAppSimVar.pModeConnLast_u8 = &sModemVar.ModeConnLast_u8;

    sCbSimComm.pGet_Fw_Version = Modem_Get_Firm_Version;
        
    sSimFwUpdate.sUpdateInfor.pFirmIP       = &sModemInfor.sUpdate.FirmIP;
    sSimFwUpdate.sUpdateInfor.pFirmPort     = &sModemInfor.sUpdate.FirmPort;
    sSimFwUpdate.sUpdateInfor.pFirmPath     = &sModemInfor.sUpdate.FirmPath;
    sSimFwUpdate.sUpdateInfor.pFirmName     = &sModemInfor.sUpdate.FirmName;
    sSimFwUpdate.sUpdateInfor.pUrlHTTP      = &sModemInfor.sUpdate.UrlHTTP;
    
#endif
  
#ifdef USING_APP_WM
    /*regis cb for app wm*/
    sWmVar.pRespond         = Modem_Respond;
    sWmVar.pRespond_Str     = Modem_Respond_Str;
    sWmVar.pPack_Alarm      = Modem_Packet_Alarm;
    sWmVar.pPack_Alarm_Str  = Modem_Packet_Alarm_String;
    sWmVar.pGet_ID          = Modem_Get_ID;
        
    sWmVar.pModePower_u8    = &sModemInfor.ModePower_u8;
    sWmVar.pHardware_u8     = &sModemInfor.DetHardware_u8;
#endif
    
    
#ifdef USING_APP_MEM  
    /*regis cb for app wm*/
    sAppMem.pReset_MCU              = Modem_Reset_MCU;
    sAppMem.pRespond                = Modem_Respond,
    sAppMem.pRespond_Str            = Modem_Respond_Str,
    sAppMem.pModePower_u8           = &sModemInfor.ModePower_u8;
#endif
    
#ifdef USING_APP_ETHERNET 
    sAppEthVar.pReset_MCU = Modem_Reset_MCU;
    sAppEthVar.pReset_MCU_Imediate = Modem_Reset_MCU_Immediately;
    sAppEthVar.pReq_Save_Box = NULL;
    sAppEthVar.pGet_Fw_Version = Modem_Get_Firm_Version;
    
    sAppEthVar.pModePower_u8            = &sModemInfor.ModePower_u8;
    sAppEthVar.pModeConnNow_u8          = &sModemVar.ModeConnNow_u8;
    sAppEthVar.pModeConnFuture_u8       = &sModemVar.ModeConnFuture_u8;
    sAppEthVar.pModeConnLast_u8         = &sModemVar.ModeConnLast_u8;
    
    sAppEthVar.sUpdate.pFirmIP     = &sModemInfor.sUpdate.FirmIP;
    sAppEthVar.sUpdate.pFirmPort   = &sModemInfor.sUpdate.FirmPort;
    sAppEthVar.sUpdate.pFirmUser   = &sModemInfor.sUpdate.FTPUser;   
    sAppEthVar.sUpdate.pFirmPass   = &sModemInfor.sUpdate.FTPPass;
        
    sAppEthVar.sUpdate.pFirmPath   = &sModemInfor.sUpdate.FirmPath;
    sAppEthVar.sUpdate.pFirmName   = &sModemInfor.sUpdate.FirmName;
    sAppEthVar.sUpdate.pUrlHTTP    = &sModemInfor.sUpdate.UrlHTTP;
    
    sAppEthVar.sUpdate.aIP              = &sModemInfor.sUpdate.aIP[0];
    sAppEthVar.sUpdate.Port_u16         = &sModemInfor.sUpdate.Port_u16;
        
    sAppEthVar.sUpdate.DomainOrIp_u8    = &sModemInfor.sUpdate.DomainOrIp_u8;
                
#endif
    /*regis cb for message*/
    /*Regis cb for user message*/
    sMQTT.sModemID = &sModemInfor.sId;
    sMQTT.sTopic = (sData *) &sSV_Modem[sModemInfor.Type_u8].sTopic;
    
    sMQTT.pPacket_Handshake = Modem_Packet_MePDV;
    sMQTT.pPacket_Status    = Modem_Packet_Mess_Status;
    sMQTT.pPacket_Opera     = Modem_Packet_Mess_Opera;
    sMQTT.pPacket_Intan     = Modem_Packet_Mess_Intan;
    sMQTT.pPacket_Alarm     = Modem_Packet_Mess_Alarm;
    sMQTT.pPacket_RespAT    = Modem_Packet_Mess_RespAT;
    sMQTT.pPacket_SimID     = Modem_Packet_Mess_SimID;
    
    sMQTT.pSER_Config       = Modem_SER_Config;
    sMQTT.pSER_Setting      = Modem_SER_Setting;
    sMQTT.pSER_Set_Time     = Modem_SER_Set_Time;
    sMQTT.pSER_Clear_Rec    = NULL;
    sMQTT.pSER_Req_AT       = Modem_SER_Req_AT;
    sMQTT.pSER_Req_RF       = Modem_SER_Req_RF;
    sMQTT.pSER_Req_Update   = Modem_SER_Req_Update;
}


void Modem_Init_ID(void)
{
#ifdef USING_INTERNAL_MEM
    uint8_t temp = 0;
	//DCU ID
	temp = *(__IO uint8_t*) ADDR_DCUID;
	if (temp != FLASH_BYTE_EMPTY)
	{
        OnchipFlash_Get_Infor ((ADDR_DCUID + 1), sModemInfor.sId.Data_a8, &sModemInfor.sId.Length_u16, 14);
	} else
	{
		Modem_Save_ID();
	}
#endif
}

void Modem_Save_ID(void)
{
#ifdef USING_INTERNAL_MEM
    uint8_t i=0;
	uint8_t temp_arr[40] = {0};

	temp_arr[0] = BYTE_TEMP_FIRST;
	temp_arr[1] = sModemInfor.sId.Length_u16;

	for (i = 0;i < sModemInfor.sId.Length_u16; i++)
		temp_arr[i+2] = sModemInfor.sId.Data_a8[i];

    OnchipFlashPageErase(ADDR_DCUID);
    OnchipFlashWriteData(ADDR_DCUID, temp_arr, 40);
#endif
}



void Modem_Init_Freq (void)
{
#ifdef USING_INTERNAL_MEM
    uint8_t     temp = 0;
    uint8_t 	Buff_temp[24] = {0};
    uint8_t 	Length = 0;

    temp = *(__IO uint8_t*) (ADDR_FREQ_ACTIVE);
    //Check Byte EMPTY
    if (temp != FLASH_BYTE_EMPTY)
    {
        OnchipFlashReadData(ADDR_FREQ_ACTIVE, &Buff_temp[0], 24);
        Length = Buff_temp[1];
        if (Length < 24)
        {
            //Get Freq Save Power
            sModemInfor.sFrequence.NumWakeup_u8 = Buff_temp[2];
            
            sModemInfor.sFrequence.Duration_u32 = Buff_temp[3];
            sModemInfor.sFrequence.Duration_u32 = (sModemInfor.sFrequence.Duration_u32 << 8) | Buff_temp[4];
            
            //Get Freq Online
            sModemInfor.sFrequence.DurOnline_u32 = Buff_temp[5];
            sModemInfor.sFrequence.DurOnline_u32 = sModemInfor.sFrequence.DurOnline_u32 << 8 | Buff_temp[6];
            sModemInfor.sFrequence.DurOnline_u32 = sModemInfor.sFrequence.DurOnline_u32 << 8 | Buff_temp[7];
            sModemInfor.sFrequence.DurOnline_u32 = sModemInfor.sFrequence.DurOnline_u32 << 8 | Buff_temp[8];
        }    
    } else
    {
        Modem_Save_Freq();
    }
#endif
}

 


void Modem_Save_Freq (void)
{
#ifdef USING_INTERNAL_MEM
    uint8_t Buff_temp[24] = {0};

    Buff_temp[0] = BYTE_TEMP_FIRST;
    Buff_temp[1] = 7;
    Buff_temp[2] = sModemInfor.sFrequence.NumWakeup_u8;

    Buff_temp[3] = (sModemInfor.sFrequence.Duration_u32 >> 8) & 0xFF;
    Buff_temp[4] = sModemInfor.sFrequence.Duration_u32 & 0xFF;
    
    Buff_temp[5] = (sModemInfor.sFrequence.DurOnline_u32 >> 24) & 0xFF;
    Buff_temp[6] = (sModemInfor.sFrequence.DurOnline_u32 >> 16) & 0xFF;
    Buff_temp[7] = (sModemInfor.sFrequence.DurOnline_u32 >> 8) & 0xFF;
    Buff_temp[8] =  sModemInfor.sFrequence.DurOnline_u32 & 0xFF;

    Erase_Firmware(ADDR_FREQ_ACTIVE, 1);
    OnchipFlashWriteData(ADDR_FREQ_ACTIVE, &Buff_temp[0], 16);
#endif
}




/*
 * Func: Init IP va Port server
 * */

void Modem_Init_Server_Infor(void)
{
#ifdef USING_INTERNAL_MEM
	uint8_t     temp = 0;
	uint8_t 	PosGet = 0;   //Vi tri cat data ra

	//MQTT server info
	temp = *(__IO uint8_t*)ADDR_SERVER_INFOR;
	if (temp == BYTE_TEMP_FIRST)
	{
		//Get IP
		PosGet = 1;
        OnchipFlash_Get_Infor (ADDR_SERVER_INFOR + PosGet, sModemInfor.sServerMain.sIP.Data_a8, &sModemInfor.sServerMain.sIP.Length_u16, MAX_LENGTH_IP);
		PosGet = PosGet + 1 + sModemInfor.sServerMain.sIP.Length_u16;
        //Get Port
        OnchipFlash_Get_Infor (ADDR_SERVER_INFOR + PosGet, sModemInfor.sServerMain.sPort.Data_a8, &sModemInfor.sServerMain.sPort.Length_u16, MAX_LENGTH_PORT);
        PosGet = PosGet + 1 + sModemInfor.sServerMain.sPort.Length_u16;
        //Get User
        OnchipFlash_Get_Infor (ADDR_SERVER_INFOR + PosGet, sModemInfor.sServerMain.sMQTTUserName.Data_a8, &sModemInfor.sServerMain.sMQTTUserName.Length_u16, MAX_LENGTH_USER_PASS);
        PosGet = PosGet + 1 + sModemInfor.sServerMain.sMQTTUserName.Length_u16;
		//Get Pass MQTT
        OnchipFlash_Get_Infor (ADDR_SERVER_INFOR + PosGet, sModemInfor.sServerMain.sMQTTPassword.Data_a8, &sModemInfor.sServerMain.sMQTTPassword.Length_u16, MAX_LENGTH_USER_PASS);
        
        //MQTT backup luu o vi tri 64
        PosGet = 64;
        OnchipFlash_Get_Infor (ADDR_SERVER_INFOR + PosGet, sModemInfor.sServerBack.sIP.Data_a8, &sModemInfor.sServerBack.sIP.Length_u16, MAX_LENGTH_IP);
		PosGet = PosGet + 1 + sModemInfor.sServerBack.sIP.Length_u16;
        //Get Port
        OnchipFlash_Get_Infor (ADDR_SERVER_INFOR + PosGet, sModemInfor.sServerBack.sPort.Data_a8, &sModemInfor.sServerBack.sPort.Length_u16, MAX_LENGTH_PORT);
        PosGet = PosGet + 1 + sModemInfor.sServerBack.sPort.Length_u16;
        //Get User
        OnchipFlash_Get_Infor (ADDR_SERVER_INFOR + PosGet, sModemInfor.sServerBack.sMQTTUserName.Data_a8, &sModemInfor.sServerBack.sMQTTUserName.Length_u16, MAX_LENGTH_USER_PASS);
        PosGet = PosGet + 1 + sModemInfor.sServerBack.sMQTTUserName.Length_u16;
		//Get Pass MQTT
        OnchipFlash_Get_Infor (ADDR_SERVER_INFOR + PosGet, sModemInfor.sServerBack.sMQTTPassword.Data_a8, &sModemInfor.sServerBack.sMQTTPassword.Length_u16, MAX_LENGTH_USER_PASS);
        
        //Convert lai IP Main       
        if (UTIL_Convert_IP_To_Buff(&sModemInfor.sServerMain.sIP, sModemInfor.sServerMain.aIP) == true)
        {
            sModemInfor.sServerMain.DomainOrIp_u8 = __SERVER_IP;
        } else
        {
            sModemInfor.sServerMain.DomainOrIp_u8 = __SERVER_DOMAIN;
        }
        
        sModemInfor.sServerMain.Port_u16 = (uint16_t) UtilStringToInt2(sModemInfor.sServerMain.sPort.Data_a8,
                                                                        sModemInfor.sServerMain.sPort.Length_u16);
        
        //Convert lai IP back 
        if (UTIL_Convert_IP_To_Buff(&sModemInfor.sServerBack.sIP, sModemInfor.sServerBack.aIP) == true)
        {
            sModemInfor.sServerBack.DomainOrIp_u8 = __SERVER_IP;
        } else
        {
            sModemInfor.sServerBack.DomainOrIp_u8 = __SERVER_DOMAIN;
        }
        
        sModemInfor.sServerBack.Port_u16 = (uint16_t) UtilStringToInt2(sModemInfor.sServerBack.sPort.Data_a8,
                                                                        sModemInfor.sServerBack.sPort.Length_u16);
            
	} else
    {
		Modem_Save_Server_Infor();
    }
#endif
}


void Modem_Save_Server_Infor (void)
{
#ifdef USING_INTERNAL_MEM
	uint8_t     aTEMP_SEVER_INFOR[256] = {0};
	uint8_t 	PosGet = 0;   //Vi tri cat data ra
	uint16_t    i = 0;

	aTEMP_SEVER_INFOR[PosGet++] = BYTE_TEMP_FIRST;
	//Packet IP
	aTEMP_SEVER_INFOR[PosGet++] = sModemInfor.sServerMain.sIP.Length_u16;
	for (i = 0; i < sModemInfor.sServerMain.sIP.Length_u16; i++)
		aTEMP_SEVER_INFOR[PosGet++] = *(sModemInfor.sServerMain.sIP.Data_a8 + i);
	//Packet Port
	aTEMP_SEVER_INFOR[PosGet++] = sModemInfor.sServerMain.sPort.Length_u16;
	for (i = 0; i < sModemInfor.sServerMain.sPort.Length_u16; i++)
		aTEMP_SEVER_INFOR[PosGet++] = *(sModemInfor.sServerMain.sPort.Data_a8 + i);
	//Packet User
	aTEMP_SEVER_INFOR[PosGet++] = sModemInfor.sServerMain.sMQTTUserName.Length_u16;
	for (i = 0; i < sModemInfor.sServerMain.sMQTTUserName.Length_u16; i++)
		aTEMP_SEVER_INFOR[PosGet++] = *(sModemInfor.sServerMain.sMQTTUserName.Data_a8 + i);
	//Packet Pas
	aTEMP_SEVER_INFOR[PosGet++] = sModemInfor.sServerMain.sMQTTPassword.Length_u16;
	for (i = 0; i < sModemInfor.sServerMain.sMQTTPassword.Length_u16; i++)
		aTEMP_SEVER_INFOR[PosGet++] = *(sModemInfor.sServerMain.sMQTTPassword.Data_a8 + i);
	//Luu
    
    PosGet = 64;
    //Packet IP
	aTEMP_SEVER_INFOR[PosGet++] = sModemInfor.sServerBack.sIP.Length_u16;
	for (i = 0; i < sModemInfor.sServerBack.sIP.Length_u16; i++)
		aTEMP_SEVER_INFOR[PosGet++] = *(sModemInfor.sServerBack.sIP.Data_a8 + i);
	//Packet Port
	aTEMP_SEVER_INFOR[PosGet++] = sModemInfor.sServerBack.sPort.Length_u16;
	for (i = 0; i < sModemInfor.sServerBack.sPort.Length_u16; i++)
		aTEMP_SEVER_INFOR[PosGet++] = *(sModemInfor.sServerBack.sPort.Data_a8 + i);
	//Packet User
	aTEMP_SEVER_INFOR[PosGet++] = sModemInfor.sServerBack.sMQTTUserName.Length_u16;
	for (i = 0; i < sModemInfor.sServerBack.sMQTTUserName.Length_u16; i++)
		aTEMP_SEVER_INFOR[PosGet++] = *(sModemInfor.sServerBack.sMQTTUserName.Data_a8 + i);
	//Packet Pas
	aTEMP_SEVER_INFOR[PosGet++] = sModemInfor.sServerBack.sMQTTPassword.Length_u16;
	for (i = 0; i < sModemInfor.sServerBack.sMQTTPassword.Length_u16; i++)
		aTEMP_SEVER_INFOR[PosGet++] = *(sModemInfor.sServerBack.sMQTTPassword.Data_a8 + i);
    
    OnchipFlashPageErase(ADDR_SERVER_INFOR);
	OnchipFlashWriteData(ADDR_SERVER_INFOR, aTEMP_SEVER_INFOR, 256);
#endif
}

uint16_t Modem_Init_u16 (uint32_t Addr, uint16_t MAX)
{
    uint8_t temp = 0;
    uint8_t Buff_temp[20] = {0};
    uint16_t IndexFind = 0;
    
    //Doc vi tri send va luu ra
    temp = *(__IO uint8_t*) Addr;
    
	if (temp != FLASH_BYTE_EMPTY)
	{
        OnchipFlashReadData ((Addr + 2), Buff_temp, 2);   
        IndexFind = (Buff_temp[0] << 8) | Buff_temp[1];
      
        //kiem tra dieu kien gioi han InDex
        if (IndexFind >= MAX)
        	IndexFind = 0;
        
    } else
    {
    	Modem_Save_u16(Addr, 0);
    }
    
    return IndexFind;
}


void Modem_Save_u16 (uint32_t Addr, uint16_t Val)
{
    uint8_t aTemp[4] = {0};

    aTemp[0] = (uint8_t) (Val >> 8);
    aTemp[1] = (uint8_t) Val;

    Save_Array(Addr, &aTemp[0], 2);
}





uint8_t Modem_Reset_MCU (void)
{
#ifdef USING_APP_WM
    if ((sPulse[0].IsOver5Pulse_u8 == TRUE) && (sPulse[1].IsOver5Pulse_u8 == TRUE) )
    {
#endif
    	Modem_Packet_Alarm_String("u_modem: soft reset mcu\r\n");

    #ifdef BOARD_READ_PULSE
    #ifdef USING_APP_WM
        if (AppWm_Save_Pulse() == true)
    #endif
        {
        #ifdef USING_APP_SIM
            //Power off module sim
            SIM_PW_OFF1;
        #endif 
            __disable_irq();
            //Reset
            NVIC_SystemReset(); // Reset MCU
        }
    #else
        #ifdef USING_APP_SIM
            //Power off module sim
            SIM_PW_OFF1;
        #endif 
        __disable_irq();
        //Reset
        NVIC_SystemReset(); // Reset MCU
    #endif
        
#ifdef USING_APP_WM
    } else
    {
    	UTIL_Printf_Str(DBLEVEL_L, "u_modem: xung chua vuot qua 5\r\n");
    }
#endif

#ifdef USING_APP_WM
    return 0;
#endif
}


uint8_t Modem_Reset_MCU_Immediately (void)
{
	Modem_Packet_Alarm_String("u_modem: soft reset mcu imediately\r\n");
    
#ifdef BOARD_READ_PULSE
#ifdef USING_APP_WM
    if (AppWm_Save_Pulse() == true)
#endif
    {
    #ifdef USING_APP_SIM
        //Power off module sim
        SIM_PW_OFF1;
    #endif 
        __disable_irq();
        //Reset
        NVIC_SystemReset(); // Reset MCU
    }
#else
#ifdef USING_APP_SIM
    //Power off module sim
    SIM_PW_OFF1;
#endif 
    __disable_irq();
    //Reset
    NVIC_SystemReset(); // Reset MCU
#endif
    
    
#ifdef USING_APP_WM
    return false;
#endif 
}



/*
    Func: vao che do luu kho
        + Power off module SIM
        + Go to standby mode
        + Wait wake up later
*/
void Modem_Save_Box (void)
{
//    //Power off module SIM
//    BC660_POW_PIN_OFF;
//    
//    /* Disable the Alarm A interrupt */
//    __HAL_RTC_ALARMA_DISABLE(&hrtc);
//    /* Disable the Alarm A interrupt */
//    HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A );
//    /* Clear RTC Alarm Flag */
//    __HAL_RTC_ALARM_CLEAR_FLAG( &hrtc, RTC_FLAG_ALRAF);
//    /* Clear the EXTI's line Flag for RTC Alarm */  
//    __HAL_RTC_ALARM_EXTI_CLEAR_FLAG();
//            
//    HAL_LPTIM_Encoder_Stop_IT(&hlptim1);
//        
//    UTIL_LPM_EnterStandbyMode();
//
//    //Reset
//    __disable_irq();
//       
//    NVIC_SystemReset(); // Reset MCU
}



void Modem_Packet_Alarm (uint8_t *pData, uint16_t length)
{
    UTIL_Log(DBLEVEL_M, pData, length);
    //Copy vao buff send len server
    *(sModemVar.strAlarm.Data_a8 + sModemVar.strAlarm.Length_u16++) = '#';
    for (uint16_t i = 0; i < length; i++)
    {
        if (sModemVar.strAlarm.Length_u16 >= MAX_LENGTH_ALAMR_EMER)
            break;
        
        *(sModemVar.strAlarm.Data_a8 + sModemVar.strAlarm.Length_u16++) = *(pData + i);
    }
#ifdef USING_APP_SIM
//    sMQTT.aMARK_MESS_PENDING[SEND_ALARM] = TRUE;
#endif
}


void Modem_Packet_Alarm_String (const char *str)
{
    uint16_t len = strlen(str);
    
    if ( (sModemVar.strAlarm.Length_u16 + len + 1) >= MAX_LENGTH_ALAMR_EMER)
        return;
    
    UTIL_Log_Str (DBLEVEL_M, str);
    //Copy vao buff send len server
    *(sModemVar.strAlarm.Data_a8 + sModemVar.strAlarm.Length_u16++) = '#';
    for (uint16_t i = 0; i < len; i++)
        *(sModemVar.strAlarm.Data_a8 + sModemVar.strAlarm.Length_u16++) = *(str + i);

#ifdef USING_APP_SIM
//    sMQTT.aMARK_MESS_PENDING[SEND_ALARM] = TRUE;
#endif
}

                        
/*============ Func Modem App ======================*/    

/*
    Func: Resp to user
        + PortNo: serial or server
        + Type: Server ACK (1) or Server Not ACK  (0)
*/

void Modem_Respond (uint8_t portNo, uint8_t *data, uint16_t length, uint8_t ack)
{
    uint16_t i = 0;
    
    switch(portNo)   
    {
        case _AT_REQUEST_SERIAL:
            HAL_UART_Transmit(&uart_debug, data,length, 1000);
            break;
        case _AT_REQUEST_SERVER:
        case _AT_REQUEST_LORA: 
        
            UTIL_MEM_set (aAT_RESPONSE, 0, sizeof ( aAT_RESPONSE ));
            sModemVar.strATResp.Length_u16 = 0;
            //Dong goi Cmd
            aAT_RESPONSE[sModemVar.strATResp.Length_u16++] = '(';
            
            for (i = 0; i < strATcmd.Length_u16; i++)
                aAT_RESPONSE[sModemVar.strATResp.Length_u16++] = *(strATcmd.Data_a8 + i);
            
            aAT_RESPONSE[sModemVar.strATResp.Length_u16++] = ')';
            //Dong goi phan hoi
            aAT_RESPONSE[sModemVar.strATResp.Length_u16++] = '(';
            for (i = 0; i < length; i++)
                aAT_RESPONSE[sModemVar.strATResp.Length_u16++] = *(data + i);
            
            aAT_RESPONSE[sModemVar.strATResp.Length_u16++] = ')';
            
            strATcmd.Length_u16 = 0;  //Reset buff AT cmd
            
        #ifdef USING_APP_SIM  
            if (ack == 0)
                sMQTT.aMARK_MESS_PENDING[SEND_RESPOND_SERVER] = TRUE;
            else
                sMQTT.aMARK_MESS_PENDING[SEND_RESPOND_SERVER_ACK] = TRUE;
        #endif
            break;
        default:
            break;
    }
}




void Modem_Respond_Str (uint8_t portNo, const char *str, uint8_t ack)
{
    uint16_t i = 0;
    
    switch(portNo)   
    {
        case _AT_REQUEST_SERIAL:
            HAL_UART_Transmit(&uart_debug, (uint8_t *) str, strlen(str), 1000);
            break;
        case _AT_REQUEST_SERVER:
        case _AT_REQUEST_LORA: 
        
            UTIL_MEM_set (aAT_RESPONSE, 0, sizeof ( aAT_RESPONSE ));
            sModemVar.strATResp.Length_u16 = 0;
            //Dong goi Cmd
            aAT_RESPONSE[sModemVar.strATResp.Length_u16++] = '(';
            
            for (i = 0; i < strATcmd.Length_u16; i++)
                aAT_RESPONSE[sModemVar.strATResp.Length_u16++] = *(strATcmd.Data_a8 + i);
            
            aAT_RESPONSE[sModemVar.strATResp.Length_u16++] = ')';
            //Dong goi phan hoi
            aAT_RESPONSE[sModemVar.strATResp.Length_u16++] = '(';
            for (i = 0; i < strlen(str); i++)
                aAT_RESPONSE[sModemVar.strATResp.Length_u16++] = *(str + i);
            
            aAT_RESPONSE[sModemVar.strATResp.Length_u16++] = ')';
            
            strATcmd.Length_u16 = 0;  //Reset buff AT cmd
            
        #ifdef USING_APP_SIM  
            if (ack == 0)
                sMQTT.aMARK_MESS_PENDING[SEND_RESPOND_SERVER] = TRUE;
            else
                sMQTT.aMARK_MESS_PENDING[SEND_RESPOND_SERVER_ACK] = TRUE;
        #endif
            break;
        default:
            break;
    }
}





/*============ Func Pointer LPM lib ================*/

void Modem_Deinit_Peripheral (void)
{
#ifdef USING_APP_TEMH
    AppTemH_Off_Energy_Consumption();
#endif
    
#ifdef USING_APP_WM
    AppWm_Off_Energy_Consumption();
#endif
    MX_GPIO_DeInit();

#ifdef BOARD_QN_V3_2
	ADC_Desequence_Powerhungry_Channels();
    
    HAL_ADC_DeInit(&hadc1);

    //Deinit Uart
    Sim_Stop_RX_Mode();
    HAL_UART_DeInit(&uart_sim);
    
    AppComm_DBG_Stop_RX_Mode();
    HAL_UART_DeInit(&uart_debug);
    
    RS485_Stop_RX_Mode();
    HAL_UART_DeInit(&uart_485);
    
    HAL_UART_DeInit(&hlpuart1);
#endif
    
#ifdef BOARD_QN_V4_0 
    ADC_Desequence_Powerhungry_Channels();
    
    HAL_ADC_DeInit(&hadc1);

    //Deinit Uart
    Sim_Stop_RX_Mode();
    HAL_UART_DeInit(&uart_sim);
    
    AppComm_DBG_Stop_RX_Mode();
    HAL_UART_DeInit(&uart_debug);
    
    RS485_Stop_RX_Mode();
    HAL_UART_DeInit(&uart_rs485);
    
    RS485_2_Stop_RX_Mode();
    HAL_UART_DeInit(&uart_rs485_2);
#endif
}


void Modem_Init_Peripheral (void)
{
    Modem_Init_Gpio_Again();  //Only init GPIO pin deinit

#ifdef BOARD_QN_V3_2
    Sim_Stop_RX_Mode();
    Sim_Init_Uart();
    Sim_Init_RX_Mode();
    
    AppComm_DBG_Stop_RX_Mode();
    AppComm_DBG_Init_Uart(); 
    AppComm_DBG_Init_RX_Mode();
    
//    RS485_Stop_RX_Mode();
//    RS485_Init_Uart(); 
//    RS485_Init_RX_Mode();
    
    MX_LPUART1_UART_Init(); 
#endif
    
#ifdef BOARD_QN_V4_0
    Sim_Stop_RX_Mode();
    Sim_Init_Uart();
    Sim_Init_RX_Mode();
    
    AppComm_DBG_Stop_RX_Mode();
    AppComm_DBG_Init_Uart(); 
    AppComm_DBG_Init_RX_Mode();
    
//    RS485_Stop_RX_Mode();
//    RS485_Init_Uart(); 
//    RS485_Init_RX_Mode();
//    
//    RS485_2_Stop_RX_Mode();
//    RS485_2_Init_Uart(); 
//    RS485_2_Init_RX_Mode();
#endif
}

void Modem_Init_Before_IRQ_Handle (void)
{
#ifdef USING_APP_LORA
    AppLora_Init_IO_Radio();
#endif
}


void Modem_Deinit_Before_IRQ_Handle (void)
{
#ifdef USING_APP_LORA
    AppLora_Deinit_IO_Radio();
#endif
}




void Modem_Uart_DeInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure ={0};
    
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStructure.Pull = GPIO_NOPULL;

    GPIO_InitStructure.Pin =  GPIO_PIN_2|GPIO_PIN_3;
    HAL_GPIO_Init( GPIOA, &GPIO_InitStructure );
    
    GPIO_InitStructure.Pin =  GPIO_PIN_9|GPIO_PIN_10;  
    HAL_GPIO_Init( GPIOA, &GPIO_InitStructure );
    
    GPIO_InitStructure.Pin =  GPIO_PIN_10|GPIO_PIN_11;  
    HAL_GPIO_Init( GPIOC, &GPIO_InitStructure );
    
    __HAL_RCC_USART1_CLK_DISABLE();
    __HAL_RCC_USART2_CLK_DISABLE();
    __HAL_RCC_USART3_CLK_DISABLE();
}





void MX_GPIO_DeInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure ={0};
    
#ifdef BOARD_QN_V3_2
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
	
	GPIO_InitStructure.Mode   = GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull   = GPIO_NOPULL;
    
    //Port A
	GPIO_InitStructure.Pin = (SIM_PWR_KEY_Pin | RS485_TXDE_Pin | SIM_DTR_Pin| BUTTON_1_Pin);  // RS485_ON_OFF_Pin
    
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
                
    //Port B
    GPIO_InitStructure.Pin = ( BLE_WAKE_Pin | ETH_INTn_Pin ) ;    //ON_OFF_12V_Pin|ON_OFF_BOOT_Pin
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);  
    
    //Port C
    GPIO_InitStructure.Pin = (LED_Pin | ETH_RSTn_Pin | PULSE1_DTC_Pin | PULSE2_DTC_Pin | SIM_RESET_Pin );    //SIM_ON_OFF_Pin | ETH_ON_OFF_Pin 
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);  
    
    //Port D
    GPIO_InitStructure.Pin = (ETH_SCSn_Pin) ;  
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);  
    
    //Port H
    GPIO_InitStructure.Pin = (BQ_CHG_Pin | Vac_Status_Pin )  ;  
	HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);      
#endif
   
#ifdef BOARD_QN_V4_0
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
	
	GPIO_InitStructure.Mode   = GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull   = GPIO_NOPULL;
    
    //Port A
	GPIO_InitStructure.Pin = ( BUTTON_1_Pin); 
    
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
                 
    //Port B 
    GPIO_InitStructure.Pin = ( RS485_2_TXDE_Pin | SIM_DTR_Pin| ETH_INTn_Pin | LCD_DETECT_Pin ) ;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);  
    
    //Port C 
    GPIO_InitStructure.Pin = (LED_Pin | ETH_RSTn_Pin | ETH_SCSn_Pin 
                              | BUTTON_PRESS_1_Pin|BUTTON_PRESS_2_Pin|BUTTON_PRESS_3_Pin|BUTTON_PRESS_4_Pin);   
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);  
    
    //Port D
    GPIO_InitStructure.Pin = ( LCD_CS_Pin | LCD_CS_Pin | LCD_RST_Pin
                              | LCD_RW_Pin | LCD_C86_Pin | LCD_A0_Pin | LED_Pin) ;  
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);   
    
    //Port E
    GPIO_InitStructure.Pin = (SIM_RESET_Pin | RS485_1_TXDE_Pin | SIM_PWR_KEY_Pin  
                              | BUTTON_1_Pin | PULSE1_DTC_Pin | PULSE2_DTC_Pin | HARD_DETECT_Pin) ;   
	HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);  
#endif
}


/*
    Func: Cau hinh lai nhung chan deinit truoc khi ngu
    */
void Modem_Init_Gpio_Again(void)
{
#ifdef BOARD_QN_V3_2
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, LED_Pin|ETH_RSTn_Pin|SIM_RESET_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, SIM_PWR_KEY_Pin|SIM_DTR_Pin|RS485_TXDE_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(ETH_SCSn_GPIO_Port, ETH_SCSn_Pin, GPIO_PIN_RESET);
  
    /*Configure GPIO pins : PCPin PCPin PCPin PCPin */
    GPIO_InitStruct.Pin = LED_Pin|ETH_RSTn_Pin|SIM_RESET_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    /*Configure GPIO pins : PHPin PHPin */
    GPIO_InitStruct.Pin = BQ_CHG_Pin|Vac_Status_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
    
    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = SIM_PWR_KEY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SIM_PWR_KEY_GPIO_Port, &GPIO_InitStruct);
    
    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = SIM_DTR_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SIM_DTR_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = BUTTON_1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BUTTON_1_GPIO_Port, &GPIO_InitStruct);
  
    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = BLE_WAKE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BLE_WAKE_GPIO_Port, &GPIO_InitStruct);
    
    /*Configure GPIO pins : PCPin PCPin */
    GPIO_InitStruct.Pin = PULSE1_DTC_Pin|PULSE2_DTC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = RS485_TXDE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(RS485_TXDE_GPIO_Port, &GPIO_InitStruct);
  
    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = ETH_SCSn_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ETH_SCSn_GPIO_Port, &GPIO_InitStruct);
    
    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = ETH_INTn_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ETH_INTn_GPIO_Port, &GPIO_InitStruct);
    
    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
#endif
    

#ifdef BOARD_QN_V4_0
     GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SIM_RESET_Pin|RS485_1_TXDE_Pin|SIM_PWR_KEY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RS485_2_TXDE_Pin|SIM_DTR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LCD_CS_Pin|LCD_RST_Pin|LCD_RW_Pin
                          |LCD_C86_Pin|LCD_A0_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ETH_RSTn_Pin|ETH_SCSn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PEPin PEPin */
  GPIO_InitStruct.Pin = SIM_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PEPin PEPin PEPin */
  GPIO_InitStruct.Pin = BUTTON_1_Pin|PULSE1_DTC_Pin|PULSE2_DTC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = VBAT_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBAT_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = HARD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(HARD_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = RS485_1_TXDE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(RS485_1_TXDE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LCD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LCD_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = RS485_2_TXDE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(RS485_2_TXDE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PDPin PDPin PDPin PDPin
                           PDPin PDPin PDPin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_RST_Pin|LCD_RW_Pin
                          |LCD_C86_Pin|LCD_A0_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = BUTTON_PRESS_1_Pin|BUTTON_PRESS_2_Pin|BUTTON_PRESS_3_Pin|BUTTON_PRESS_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin */
  GPIO_InitStruct.Pin = ETH_RSTn_Pin|ETH_SCSn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = ETH_INTn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ETH_INTn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = SIM_DTR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SIM_DTR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = SIM_PWR_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SIM_PWR_KEY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  
#endif
}



/*Func Cb Setting serial*/
#ifdef USING_AT_CONFIG

void Modem_SER_Set_ID (sData *str, uint16_t Pos)
{
    uint8_t Fix = 0;
    uint16_t Length = 0;
    
    if (PortConfig == 2)
    	Fix = 1;

    if (str->Length_u16 <= (Pos + 20 + Fix))   	//1 byte crc phia sau neu dung server
    {
        Length = str->Length_u16 - Pos - Fix;
        
        if (Length >= sizeof(aMODEM_ID))
            Length = sizeof(aMODEM_ID);
    
        for (uint16_t i = 0; i < Length; i++)
        {
            aMODEM_ID[i] = *(str->Data_a8 + Pos + i);
        }
        
        aMODEM_ID[Length] = 0;
            
        sModemInfor.sId.Data_a8 = aMODEM_ID;
        sModemInfor.sId.Length_u16 = Length;
        
        Modem_Save_ID();
        Modem_Reset_MCU_Immediately();
        
        Modem_Respond_Str (PortConfig, "\r\nOK", 1);
    } else
    {
    	Modem_Respond_Str(PortConfig, "\r\nFAIL LENGTH!", 0);  
    }
}


void Modem_SER_Get_ID (sData *str, uint16_t Pos)
{
    Modem_Respond (PortConfig, sModemInfor.sId.Data_a8, sModemInfor.sId.Length_u16, 0) ;
}

void Modem_SER_Set_Duty_Cycle (sData *strRecei, uint16_t Pos)
{
    uint16_t TempFre = 0;
    uint8_t TempNum = 0;

    if (strRecei->Length_u16 < (Pos + 5))
    {
        Modem_Respond_Str(PortConfig, "\r\nFAIL LENGTH", 0);
        return;
    }
    
    TempNum = *(strRecei->Data_a8 + Pos++) - 0x30;
    TempNum = TempNum * 10 + *(strRecei->Data_a8 + Pos++) - 0x30;
    Pos++;
    TempFre = *(strRecei->Data_a8 + Pos++) - 0x30;
    TempFre = TempFre * 10 + *(strRecei->Data_a8 + Pos++) - 0x30;

    if (((TempNum > 0) && (TempNum <= MAX_NUMBER_WAKE_UP)) && (TempFre <= 60))
    {
        sModemInfor.sFrequence.NumWakeup_u8 = TempNum;
        sModemInfor.sFrequence.Duration_u32 = TempFre;
    
        Modem_Save_Freq();
        AppComm_Set_Next_TxTimer();
        
        Modem_Respond_Str (PortConfig, "\r\nOK", 0);
        return;
    }
   
    Modem_Respond_Str(PortConfig, "\r\nERROR", 0);
}


void Modem_SER_Get_Duty_Cycle (sData *strRecei, uint16_t Pos)
{
    char aData[32] = {0};
    
    sprintf(aData, "duty = %02dx%02d\r\n", sModemInfor.sFrequence.NumWakeup_u8,
                                           sModemInfor.sFrequence.Duration_u32);

    Modem_Respond_Str(PortConfig, aData, 0);
}


void Modem_SER_Set_Dura_Online (sData *strRecei, uint16_t Pos)
{
    sModemInfor.sFrequence.DurOnline_u32 = UTIL_Get_Num_From_Str (strRecei, &Pos);
    
    Modem_Save_Freq ();
    //Set lai timer
    AppComm_Set_Next_TxTimer();
}


void Modem_SER_Get_Dura_Online (sData *strRecei, uint16_t Pos)
{
    char aData[32] = {0};

    sprintf(aData, "duration online = %d\r\n", sModemInfor.sFrequence.DurOnline_u32);
    
    Modem_Respond_Str(PortConfig, aData, 0);
}


void Modem_SER_Set_Numwakeup (sData *strRecei, uint16_t Pos)
{
    uint8_t TempU8 = 0;

    TempU8 = *(strRecei->Data_a8 + Pos++) - 0x30;
    TempU8 = TempU8 * 10 + *(strRecei->Data_a8 + Pos++) - 0x30;
    
    if (TempU8 <= MAX_NUMBER_WAKE_UP)
    {
        sModemInfor.sFrequence.NumWakeup_u8 = TempU8;
        Modem_Save_Freq ();
        
        Modem_Respond_Str(PortConfig, "\r\nOK",  1);
        return;
    }
    
    Modem_Respond_Str(PortConfig, "\r\nERROR", 0);
}


void Modem_SER_Set_sTime (sData *strRecei, uint16_t Pos)
{
    ST_TIME_FORMAT   sRTC_temp = {0};
	
    if ( (*(strRecei->Data_a8+1) == '-') && (*(strRecei->Data_a8+4) == '-') &&  (*(strRecei->Data_a8+7) == '-')  
        && (*(strRecei->Data_a8+10) == ',') && (*(strRecei->Data_a8+13) == ':') && (*(strRecei->Data_a8+16) == ':') )
	{
		sRTC_temp.day   = (*(strRecei->Data_a8))-0x30 - 1;
		sRTC_temp.date  = ((*(strRecei->Data_a8 + 2))-0x30)*10 + (*(strRecei->Data_a8 + 3) - 0x30);
		sRTC_temp.month = ((*(strRecei->Data_a8 + 5))-0x30)*10 + (*(strRecei->Data_a8 + 6) - 0x30);
		sRTC_temp.year  = ((*(strRecei->Data_a8 + 8))-0x30)*10 + (*(strRecei->Data_a8 + 9) - 0x30);
  
        sRTC_temp.hour  = ((*(strRecei->Data_a8 + 11))-0x30)*10 + (*(strRecei->Data_a8 + 12) - 0x30);
		sRTC_temp.min   = ((*(strRecei->Data_a8 + 14))-0x30)*10 + (*(strRecei->Data_a8 + 15) - 0x30);
		sRTC_temp.sec   = ((*(strRecei->Data_a8 + 17))-0x30)*10 + (*(strRecei->Data_a8 + 18) - 0x30);
        
        if (Check_update_Time(&sRTC_temp) == true)
        {
            sModemVar.sRTCSet.year   = sRTC_temp.year;
            sModemVar.sRTCSet.month  = sRTC_temp.month;
            sModemVar.sRTCSet.date   = sRTC_temp.date;
            sModemVar.sRTCSet.hour   = sRTC_temp.hour;
            sModemVar.sRTCSet.min    = sRTC_temp.min;
            sModemVar.sRTCSet.sec    = sRTC_temp.sec;
            sModemVar.sRTCSet.day    = sRTC_temp.day;

            Convert_sTime_ToGMT(&sModemVar.sRTCSet, 7); 
            //Convert lai day. 1/1/2012 la chu nhat. Thu 2 - cn: 2-8
            sModemVar.sRTCSet.day = ((HW_RTC_GetCalendarValue_Second (sModemVar.sRTCSet, 1) / SECONDS_IN_1DAY) + 6) % 7 + 1;
            
            fevent_active(sEventAppComm, _EVENT_SET_RTC);
            
            Modem_Respond_Str(PortConfig, "OK", 0);
            return;
        }
	}
    
    Modem_Respond_Str(PortConfig, "ERROR", 0);
}


void Modem_SER_Get_sTime (sData *strRecei, uint16_t Pos)
{
    char aData[48] = {0};
    
    sprintf(aData, "\r\n%d-%d-%d-%d \r\n%d:%d:%d\r\n", sRTC.day, sRTC.date, sRTC.month, sRTC.year,
                                                        sRTC.hour, sRTC.min, sRTC.sec);
    
    Modem_Respond_Str(PortConfig, aData, 0);
}


void Modem_SER_Set_Level_Debug (sData *str_Receiv, uint16_t Pos)
{
    //set level debug
    uint8_t temp = *(str_Receiv->Data_a8 + Pos) - 0x30;
    
    if ( (temp >= DBLEVEL_MESS) && (temp <= DBLEVEL_H) )
    {
        VLevelDebug = temp;
        UTIL_ADV_TRACE_SetVerboseLevel(DBLEVEL_M);
        
        Modem_Respond_Str(PortConfig, "OK", 0);
    } else
    {
        Modem_Respond_Str(PortConfig, "ERROR", 0);
    }
}

void Modem_SER_Get_Level_Debug (sData *str_Receiv, uint16_t Pos)
{
    uint8_t temp = VLevelDebug + 0x30;
    
    Modem_Respond(PortConfig, &temp, 1, 0);
}





void Modem_SER_Get_Firm_Version (sData *strRecei, uint16_t Pos)
{
    Modem_Respond(PortConfig, sFirmVersion.Data_a8, sFirmVersion.Length_u16, 0);
}


void Modem_SER_Req_Reset (sData *strRecei, uint16_t Pos)
{
    Modem_Reset_MCU_Immediately();
}


void Modem_SER_Set_Server_Main (sData *strRecei, uint16_t Pos)
{
    sData	sIPGet = {NULL, 0};
	sData	sPortGet = {NULL, 0};
	sData	sUserGet = {NULL, 0};
	sData	sPassGet = {NULL, 0};
    
    Modem_Extract_Sever_Infor(strRecei, Pos,
                              &sIPGet, &sPortGet, &sUserGet, &sPassGet);
    
    if (Modem_Set_Server_Main(&sIPGet, &sPortGet, &sUserGet, &sPassGet, 60) == true)
    {
        Modem_Respond_Str(PortConfig, "OK", 0);
        return;
    }

    Modem_Respond_Str(PortConfig, "ERROR", 0);
}

void Modem_Extract_Sever_Infor (sData *strRecei, uint16_t Pos,
                                sData *sIPGet, sData *sPortGet, sData *sUserGet, sData *sPassGet)
{
    if (strRecei->Length_u16 > 64)
    {
        return;
    }
    
    //Chay qua khoang trang (neu co)
    for (Pos = 0; Pos < strRecei->Length_u16; Pos++)
        if (*(strRecei->Data_a8 + Pos) != ' ')   //khac khoang trang thi dung lai
            break;

    // Mark finish
    *(strRecei->Data_a8 + 63)  = ','; 						
    //Get IP
    sIPGet->Data_a8 = strRecei->Data_a8 + Pos;
    //Tinh length
	while (*(strRecei->Data_a8 + Pos++) != ',')
	{
		sIPGet->Length_u16++;
		if (Pos > strRecei->Length_u16)
			break;
	}
	//Get Port
	sPortGet->Data_a8 = strRecei->Data_a8 + Pos;
	if (Pos < 64)
	{
		while (*(strRecei->Data_a8 + Pos++) != ',')
		{
			sPortGet->Length_u16++;
			if (Pos > strRecei->Length_u16)
				break;
		}
	}
	//Get user
	sUserGet->Data_a8 = strRecei->Data_a8 + Pos;
	if (Pos < 64)
	{
		while (*(strRecei->Data_a8 + Pos++) != ',')
		{
			sUserGet->Length_u16++;
			if (Pos > strRecei->Length_u16)
				break;
		}
	}
	//Get Pass
	sPassGet->Data_a8 = strRecei->Data_a8 + Pos;
    if (Pos < 64)
    {
        while (Pos < strRecei->Length_u16)
        {
        	sPassGet->Length_u16++;
            Pos++;
        }
    }
}



uint8_t Modem_Set_Server_Main (sData *pIP, sData *pPort, sData *pUser, sData *pPass, uint32_t Alive)
{
    uint16_t i = 0;
    
    if ((pIP->Length_u16 > 0) && (pIP->Length_u16 < MAX_LENGTH_IP)
    		&& (pPort->Length_u16 > 0) && (pPort->Length_u16 < MAX_LENGTH_PORT))
    {
    	if (pUser->Length_u16 > MAX_LENGTH_USER_PASS)
    		pUser->Length_u16 = MAX_LENGTH_USER_PASS;
        
    	if (pPass->Length_u16 > MAX_LENGTH_USER_PASS)
    		pPass->Length_u16 = MAX_LENGTH_USER_PASS;
        
        UTIL_MEM_set(&aIP_SERVER, 0, sizeof(aIP_SERVER));
        UTIL_MEM_set(&aPORT_SERVER, 0, sizeof(aPORT_SERVER));
        UTIL_MEM_set(&aUSER_MQTT, 0, sizeof(aUSER_MQTT));
        UTIL_MEM_set(&aPASS_MQTT, 0, sizeof(aPASS_MQTT)); 
        
        //Set IP
        sModemInfor.sServerMain.sIP.Length_u16 = pIP->Length_u16;
        for (i = 0; i < sModemInfor.sServerMain.sIP.Length_u16; i++)
            *(sModemInfor.sServerMain.sIP.Data_a8 + i) = *(pIP->Data_a8 + i);
        //Set Port
        sModemInfor.sServerMain.sPort.Length_u16 = pPort->Length_u16;
        for (i = 0; i < sModemInfor.sServerMain.sPort.Length_u16; i++)
            *(sModemInfor.sServerMain.sPort.Data_a8 + i) = *(pPort->Data_a8 + i);
        //Set User
        sModemInfor.sServerMain.sMQTTUserName.Length_u16 = pUser->Length_u16;
        for (i = 0; i < sModemInfor.sServerMain.sMQTTUserName.Length_u16; i++)
            *(sModemInfor.sServerMain.sMQTTUserName.Data_a8 + i) = *(pUser->Data_a8 + i);
        //Set User
        sModemInfor.sServerMain.sMQTTPassword.Length_u16 = pPass->Length_u16;
        for (i = 0; i < sModemInfor.sServerMain.sMQTTPassword.Length_u16; i++)
            *(sModemInfor.sServerMain.sMQTTPassword.Data_a8 + i) = *(pPass->Data_a8 + i);
            
        sModemInfor.sServerMain.MQTTKeepAlive_u32 = Alive;
        
        //Convert lai IP       
        if (UTIL_Convert_IP_To_Buff(&sModemInfor.sServerMain.sIP, sModemInfor.sServerMain.aIP) == true)
        {
            sModemInfor.sServerMain.DomainOrIp_u8 = __SERVER_IP;
        } else
        {
            sModemInfor.sServerMain.DomainOrIp_u8 = __SERVER_DOMAIN;
        }
        
        sModemInfor.sServerMain.Port_u16 = (uint16_t) UtilStringToInt2(sModemInfor.sServerMain.sPort.Data_a8,
                                                                        sModemInfor.sServerMain.sPort.Length_u16);
        
        Modem_Save_Server_Infor();
        
        return true;
    }
    
    return false;
}


void Modem_SER_Set_Server_Back (sData *strRecei, uint16_t Pos)
{
    sData	sIPGet = {NULL, 0};
	sData	sPortGet = {NULL, 0};
	sData	sUserGet = {NULL, 0};
	sData	sPassGet = {NULL, 0};
    
    Modem_Extract_Sever_Infor(strRecei, Pos,
                              &sIPGet, &sPortGet, &sUserGet, &sPassGet);
    
    if (Modem_Set_Server_Back(&sIPGet, &sPortGet, &sUserGet, &sPassGet, 60) == true)
    {
        Modem_Respond_Str(PortConfig, "OK", 0);
        return;
    }

    Modem_Respond_Str(PortConfig, "ERROR", 0);
}


uint8_t Modem_Set_Server_Back (sData *pIP, sData *pPort, sData *pUser, sData *pPass, uint32_t Alive)
{
    uint16_t i = 0;
    
    if ((pIP->Length_u16 > 0) && (pIP->Length_u16 < MAX_LENGTH_IP)
    		&& (pPort->Length_u16 > 0) && (pPort->Length_u16 < MAX_LENGTH_PORT))
    {
    	if (pUser->Length_u16 > MAX_LENGTH_USER_PASS)
    		pUser->Length_u16 = MAX_LENGTH_USER_PASS;
        
    	if (pPass->Length_u16 > MAX_LENGTH_USER_PASS)
    		pPass->Length_u16 = MAX_LENGTH_USER_PASS;
        
        UTIL_MEM_set(&aIP_SERVER_BACKUP, 0, sizeof(aIP_SERVER_BACKUP));
        UTIL_MEM_set(&aPORT_SERVER_BACKUP, 0, sizeof(aPORT_SERVER_BACKUP));
        UTIL_MEM_set(&aUSER_MQTT_BACKUP, 0, sizeof(aUSER_MQTT_BACKUP));
        UTIL_MEM_set(&aPASS_MQTT_BACKUP, 0, sizeof(aPASS_MQTT_BACKUP)); 
        
        //Set IP
        sModemInfor.sServerBack.sIP.Length_u16 = pIP->Length_u16;
        for (i = 0; i < sModemInfor.sServerBack.sIP.Length_u16; i++)
            *(sModemInfor.sServerBack.sIP.Data_a8 + i) = *(pIP->Data_a8 + i);
        //Set Port
        sModemInfor.sServerBack.sPort.Length_u16 = pPort->Length_u16;
        for (i = 0; i < sModemInfor.sServerBack.sPort.Length_u16; i++)
            *(sModemInfor.sServerBack.sPort.Data_a8 + i) = *(pPort->Data_a8 + i);
        //Set User
        sModemInfor.sServerBack.sMQTTUserName.Length_u16 = pUser->Length_u16;
        for (i = 0; i < sModemInfor.sServerBack.sMQTTUserName.Length_u16; i++)
            *(sModemInfor.sServerBack.sMQTTUserName.Data_a8 + i) = *(pUser->Data_a8 + i);
        //Set User
        sModemInfor.sServerBack.sMQTTPassword.Length_u16 = pPass->Length_u16;
        for (i = 0; i < sModemInfor.sServerBack.sMQTTPassword.Length_u16; i++)
            *(sModemInfor.sServerBack.sMQTTPassword.Data_a8 + i) = *(pPass->Data_a8 + i);
            
        sModemInfor.sServerBack.MQTTKeepAlive_u32 = Alive;
        
        //Convert lai IP       
        if (UTIL_Convert_IP_To_Buff(&sModemInfor.sServerBack.sIP, sModemInfor.sServerBack.aIP) == true)
        {
            sModemInfor.sServerBack.DomainOrIp_u8 = __SERVER_IP;
        } else
        {
            sModemInfor.sServerBack.DomainOrIp_u8 = __SERVER_DOMAIN;
        }
        
        sModemInfor.sServerBack.Port_u16 = (uint16_t) UtilStringToInt2(sModemInfor.sServerBack.sPort.Data_a8,
                                                                        sModemInfor.sServerBack.sPort.Length_u16);
        
        Modem_Save_Server_Infor();
            
        return true;
    }
    
    return false;
}


void Modem_SER_Get_Server_Main (sData *strRecei, uint16_t Pos)
{
    char aData[128] = {0};
    
    sprintf((char*) aData, "%s:%s\r\n", sModemInfor.sServerMain.sIP.Data_a8,
                                        sModemInfor.sServerMain.sPort.Data_a8);  
    
    Modem_Respond_Str(PortConfig, aData, 0);
}


void Modem_SER_Get_Server_Back (sData *strRecei, uint16_t Pos)
{
    char aData[128] = {0};
    
    sprintf((char*) aData, "%s:%s\r\n", sModemInfor.sServerBack.sIP.Data_a8,
                                        sModemInfor.sServerBack.sPort.Data_a8);  
    
    Modem_Respond_Str(PortConfig, aData, 0);
}


void Modem_SER_Change_Server (sData *strRecei, uint16_t Pos)
{
    Modem_Change_Server();
}




void Modem_SER_HTTP_Update (sData *strRecei, uint16_t Pos)
{
    int PosFind;
    uint8_t Flag_Get_name = 0;
    uint8_t aTEMP_IP[40] = {0};
    uint8_t aTEMP_PORT[8] = {0};
    uint8_t aTEMP_USER[40] = {0};
    uint8_t aTEMP_PASS[40] = {0};
    
    uint8_t aTEMP_PATH[40] = {0};
    uint8_t aTEMP_NAME[40] = {0};
    uint8_t aTEMP_URL[200] = {"http://"};
    
    sData sIP = {aTEMP_IP, 0};
    sData sPort = {aTEMP_PORT, 0}; 
    sData sUser = {aTEMP_USER, 0}; 
    sData sPass = {aTEMP_PASS, 0}; 
    
    sData sPath = {aTEMP_PATH, 0}; 
    sData sName = {aTEMP_NAME, 0}; 
    sData sURL = {aTEMP_URL, 7}; 
    
    PosFind = Find_String_V2(&sURL, strRecei);
    if (PosFind >=0)
    {
        PosFind += sURL.Length_u16;
        
        if (Cut_String_2(strRecei, (uint16_t *) &PosFind, ':', &sIP, sizeof(aTEMP_IP)) == false)
        {
            Modem_Respond_Str(PortConfig, "ERROR", 0);
            return;
        }

        PosFind++;
        
        if (Cut_String_2(strRecei, (uint16_t *) &PosFind, ',', &sPort, sizeof(aTEMP_PORT)) == false)
        {
            Modem_Respond_Str(PortConfig, "ERROR", 0);
            return;
        }
        
        PosFind++;
        
        if (Cut_String_2(strRecei, (uint16_t *) &PosFind, ',', &sUser, sizeof(aTEMP_USER)) == false)
        {
            Modem_Respond_Str(PortConfig, "ERROR", 0);
            return;
        }

        PosFind++;
        
        if (Cut_String_2(strRecei, (uint16_t *) &PosFind, '/', &sPass, sizeof(aTEMP_PASS)) == false)
        {
            Modem_Respond_Str(PortConfig, "ERROR", 0);
            return;
        }
        
        PosFind++;
        
        while (Flag_Get_name == 0)
        {
            if (Cut_String_2(strRecei, (uint16_t *) &PosFind, '/', &sName, sizeof(aTEMP_NAME)) == false)
            {
                //tim duoc chuoi name cuoi cung
                Flag_Get_name = 1;
            } else
            {
                //copy sang path
                if ( (sPath.Length_u16 + sName.Length_u16) >= sizeof(aTEMP_PATH) )
                {
                    Modem_Respond_Str(PortConfig, "ERROR", 0);
                    return;
                }
                
                if (sPath.Length_u16 > 0)
                    *(sPath.Data_a8 + sPath.Length_u16++) = '/';  //them '/'
                
                Copy_String_2(&sPath , &sName);
                Reset_Buff(&sName);
            }

            PosFind++;
        }
                
        Copy_String_2(&sURL , &sIP);
        *(sURL.Data_a8 + sURL.Length_u16++) = ':';
        Copy_String_2(&sURL , &sPort);
        *(sURL.Data_a8 + sURL.Length_u16++) = '/';
        Copy_String_2(&sURL , &sPath);
        *(sURL.Data_a8 + sURL.Length_u16++) = '/';
        Copy_String_2(&sURL , &sName);
        
        
        Modem_Set_Http_Update(&sIP, &sPort, &sUser, &sPass, &sPath, &sName, &sURL);  
        Modem_Respond_Str(PortConfig, "OK", 0);
    }
}



uint8_t Modem_Set_Http_Update (sData *pIP, sData *pPort, sData *pUser, sData *pPass,
                                        sData *pPath, sData *pName, sData *pURL)
{  
    Modem_Set_Update_Infor(pIP, pPort, pUser, pPass, pPath, pName, pURL);
    
    sModemVar.ModeConnFuture_u8 = MODE_CONNECT_HTTP; 
    
#ifdef USING_APP_ETHERNET
    sAppEthVar.Status_u8  = _ETH_START;
    fevent_active(sEventAppEth, _EVENT_ETH_HARD_RESET);
#endif
    
#ifdef USING_APP_SIM
    if (sAppEthVar.Status_u8 < _ETH_TCP_CONNECT)
        AppSim_Restart_Imediate();
#endif
    
    return 1;
}




void Modem_SER_FTP_Update (sData *strRecei, uint16_t Pos)
{
    int PosFind;
    uint8_t Flag_Get_name = 0;
    uint8_t aTEMP_IP[40] = {0};
    uint8_t aTEMP_PORT[8] = {0};
    uint8_t aTEMP_USER[40] = {0};
    uint8_t aTEMP_PASS[40] = {0};
    
    uint8_t aTEMP_PATH[40] = {0};
    uint8_t aTEMP_NAME[40] = {0};
    uint8_t aTEMP_URL[200] = {"http://"};
    
    sData sIP = {aTEMP_IP, 0};
    sData sPort = {aTEMP_PORT, 0}; 
    sData sUser = {aTEMP_USER, 0}; 
    sData sPass = {aTEMP_PASS, 0}; 
    
    sData sPath = {aTEMP_PATH, 0}; 
    sData sName = {aTEMP_NAME, 0}; 
    sData sURL = {aTEMP_URL, 7}; 
    
    PosFind = Find_String_V2(&sURL, strRecei);
    if (PosFind >=0)
    {
        PosFind += sURL.Length_u16;
        
        if (Cut_String_2(strRecei, (uint16_t *) &PosFind, ':', &sIP, sizeof(aTEMP_IP)) == false)
        {
            Modem_Respond_Str(PortConfig, "ERROR", 0);
            return;
        }

        PosFind++;
        
        if (Cut_String_2(strRecei, (uint16_t *) &PosFind, ',', &sPort, sizeof(aTEMP_PORT)) == false)
        {
            Modem_Respond_Str(PortConfig, "ERROR", 0);
            return;
        }
        
        PosFind++;
        
        if (Cut_String_2(strRecei, (uint16_t *) &PosFind, ',', &sUser, sizeof(aTEMP_USER)) == false)
        {
            Modem_Respond_Str(PortConfig, "ERROR", 0);
            return;
        }

        PosFind++;
        
        if (Cut_String_2(strRecei, (uint16_t *) &PosFind, '/', &sPass, sizeof(aTEMP_PASS)) == false)
        {
            Modem_Respond_Str(PortConfig, "ERROR", 0);
            return;
        }
        
        PosFind++;
        
        while (Flag_Get_name == 0)
        {
            if (Cut_String_2(strRecei, (uint16_t *) &PosFind, '/', &sName, sizeof(aTEMP_NAME)) == false)
            {
                //tim duoc chuoi name cuoi cung
                Flag_Get_name = 1;
            } else
            {
                //copy sang path
                if ( (sPath.Length_u16 + sName.Length_u16) >= sizeof(aTEMP_PATH) )
                {
                    Modem_Respond_Str(PortConfig, "ERROR", 0);
                    return;
                }
                
                if (sPath.Length_u16 > 0)
                    *(sPath.Data_a8 + sPath.Length_u16++) = '/';  //them '/'
                
                Copy_String_2(&sPath , &sName);
                Reset_Buff(&sName);
            }

            PosFind++;
        }
                
        Copy_String_2(&sURL , &sIP);
        *(sURL.Data_a8 + sURL.Length_u16++) = ':';
        Copy_String_2(&sURL , &sPort);
        *(sURL.Data_a8 + sURL.Length_u16++) = '/';
        Copy_String_2(&sURL , &sPath);
        *(sURL.Data_a8 + sURL.Length_u16++) = '/';
        Copy_String_2(&sURL , &sName);
        
        
        Modem_Set_FTP_Update(&sIP, &sPort, &sUser, &sPass, &sPath, &sName, &sURL);  
        Modem_Respond_Str(PortConfig, "OK", 0);
    }
}


uint8_t Modem_Set_FTP_Update (sData *pIP, sData *pPort, sData *pUser, sData *pPass,
                                        sData *pPath, sData *pName, sData *pURL)
{  
    Modem_Set_Update_Infor(pIP, pPort, pUser, pPass, pPath, pName, pURL);
    
    sModemVar.ModeConnFuture_u8 = MODE_CONNECT_FTP; 
    
#ifdef USING_APP_ETHERNET
    sAppEthVar.Status_u8  = _ETH_START;
    fevent_active(sEventAppEth, _EVENT_ETH_HARD_RESET);
#endif
    
#ifdef USING_APP_SIM
    if (sAppEthVar.Status_u8 < _ETH_TCP_CONNECT)
        AppSim_Restart_Imediate();
#endif
    
    return 1;
}


void Modem_Set_Update_Infor (sData *pIP, sData *pPort, sData *pUser, sData *pPass,
                                        sData *pPath, sData *pName, sData *pURL)
{  
    Reset_Buff(&sModemInfor.sUpdate.FirmIP);
    Reset_Buff(&sModemInfor.sUpdate.FirmPort);
    Reset_Buff(&sModemInfor.sUpdate.FTPUser);
    Reset_Buff(&sModemInfor.sUpdate.FTPPass);
    
    Reset_Buff(&sModemInfor.sUpdate.FirmPath);
    Reset_Buff(&sModemInfor.sUpdate.FirmName);
    Reset_Buff(&sModemInfor.sUpdate.UrlHTTP);
    
    Copy_String(&sModemInfor.sUpdate.FirmIP, pIP, pIP->Length_u16, MAX_LENGTH_IP);
    Copy_String(&sModemInfor.sUpdate.FirmPort, pPort, pPort->Length_u16, MAX_LENGTH_PORT);
    Copy_String(&sModemInfor.sUpdate.FTPUser, pUser, pUser->Length_u16, MAX_LENGTH_USER_PASS);
    Copy_String(&sModemInfor.sUpdate.FTPPass, pPass, pPass->Length_u16, MAX_LENGTH_USER_PASS);
    
    Copy_String(&sModemInfor.sUpdate.FirmPath, pPath, pPath->Length_u16, MAX_LENGTH_PATH);
    Copy_String(&sModemInfor.sUpdate.FirmName, pName, pName->Length_u16, MAX_LENGTH_NAME);
    Copy_String(&sModemInfor.sUpdate.UrlHTTP, pURL, pURL->Length_u16, MAX_LENGTH_URL);
    
    //Convert lai IP back 
    if (UTIL_Convert_IP_To_Buff(&sModemInfor.sUpdate.FirmIP, sModemInfor.sUpdate.aIP) == true)
    {
        sModemInfor.sUpdate.DomainOrIp_u8 = __SERVER_IP;
    } else
    {
        sModemInfor.sUpdate.DomainOrIp_u8 = __SERVER_DOMAIN;
    }
    
    sModemInfor.sUpdate.Port_u16 = (uint16_t) UtilStringToInt2(sModemInfor.sUpdate.FirmPort.Data_a8,
                                                                    sModemInfor.sUpdate.FirmPort.Length_u16);
}


#endif


/*
    Func: Set server Infor
        type: 0: Main
              1: Back
*/
void Modem_Set_Server_Infor_To_App(void)
{
    if (sModemVar.ModeConnNow_u8 == _ETH_MODE_CONN_MAIN)
    {
    #ifdef USING_APP_ETHERNET
        sAppEthVar.sServer.sIP = &sModemInfor.sServerMain.sIP;
        sAppEthVar.sServer.sPort = &sModemInfor.sServerMain.sPort;
        sAppEthVar.sServer.sMQTTUserName = &sModemInfor.sServerMain.sMQTTUserName;
        sAppEthVar.sServer.sMQTTPassword = &sModemInfor.sServerMain.sMQTTPassword;
        
        sAppEthVar.sServer.aIP = sModemInfor.sServerMain.aIP;
        sAppEthVar.sServer.Port_u16 = &sModemInfor.sServerMain.Port_u16;
            
        sAppEthVar.sServer.MQTTKeepAlive_u32 = &sModemInfor.sServerMain.MQTTKeepAlive_u32;
        sAppEthVar.sServer.DomainOrIp_u8 = &sModemInfor.sServerMain.DomainOrIp_u8 ;
    #endif
            
    #ifdef USING_APP_SIM
        sSimCommInfor.sServer.sIP = &sModemInfor.sServerMain.sIP;
        sSimCommInfor.sServer.sPort = &sModemInfor.sServerMain.sPort;
        sSimCommInfor.sServer.sMQTTUserName = &sModemInfor.sServerMain.sMQTTUserName;
        sSimCommInfor.sServer.sMQTTPassword = &sModemInfor.sServerMain.sMQTTPassword;
        
        sSimCommInfor.sServer.MQTTKeepAlive_u32 = &sModemInfor.sServerMain.MQTTKeepAlive_u32;    
    #endif   
    } else 
    {
    #ifdef USING_APP_ETHERNET
        sAppEthVar.sServer.sIP = &sModemInfor.sServerBack.sIP;
        sAppEthVar.sServer.sPort = &sModemInfor.sServerBack.sPort;
        sAppEthVar.sServer.sMQTTUserName = &sModemInfor.sServerBack.sMQTTUserName;
        sAppEthVar.sServer.sMQTTPassword = &sModemInfor.sServerBack.sMQTTPassword;
        
        sAppEthVar.sServer.aIP = sModemInfor.sServerBack.aIP;
        sAppEthVar.sServer.Port_u16 = &sModemInfor.sServerBack.Port_u16;
            
        sAppEthVar.sServer.MQTTKeepAlive_u32 = &sModemInfor.sServerBack.MQTTKeepAlive_u32;
        sAppEthVar.sServer.DomainOrIp_u8 = &sModemInfor.sServerBack.DomainOrIp_u8 ;
    #endif
            
    #ifdef USING_APP_SIM
        sSimCommInfor.sServer.sIP = &sModemInfor.sServerBack.sIP;
        sSimCommInfor.sServer.sPort = &sModemInfor.sServerBack.sPort;
        sSimCommInfor.sServer.sMQTTUserName = &sModemInfor.sServerBack.sMQTTUserName;
        sSimCommInfor.sServer.sMQTTPassword = &sModemInfor.sServerBack.sMQTTPassword;
        
        sSimCommInfor.sServer.MQTTKeepAlive_u32 = &sModemInfor.sServerBack.MQTTKeepAlive_u32;    
    #endif  
    }
}


//Func cb for message

void Modem_Packet_MePDV (sData *pData)
{
    uint16_t i = 0;
    uint8_t TempCrc = 0;
#ifdef USING_APP_SIM
    uint8_t aTEMP[128] = {0};
    uint8_t Length = 0;
    sData *pstrTemp;
#endif
    uint8_t DcuType = 1;
    
    //Thoi gian thu thap
    SV_Protocol_Packet_Data(pData->Data_a8, &pData->Length_u16, OBIS_TIME_DEVICE, &sRTC, 6, 0xAA);
    //Cuong do song
#ifdef USING_APP_SIM
    SV_Protocol_Packet_Data(pData->Data_a8, &pData->Length_u16, OBIS_RSSI_1, &sSimCommInfor.RSSI_u8, 1, 0x00);
#endif
    //Thoi gian thu thap
    SV_Protocol_Packet_Data(pData->Data_a8, &pData->Length_u16, OBIS_FREQ_SEND, &sModemInfor.sFrequence.UnitMin_u16, 2, 0x00);
#ifdef USING_APP_SIM
    //Seri SIM
    pstrTemp = AppSim_Get_Sim_ID();
    if (pstrTemp->Length_u16 != 0)
    {
        SV_Protocol_Packet_Data(pData->Data_a8, &pData->Length_u16, OBIS_SERI_SIM, pstrTemp->Data_a8, pstrTemp->Length_u16, 0xAA);
    }
    
    //Dia chi server dang connect
    if (sModemVar.ModeConnNow_u8 == MODE_CONNECT_DATA_MAIN)
    {
        sprintf((char*) aTEMP, "%s:%s\r\n", sModemInfor.sServerMain.sIP.Data_a8,
                                        sModemInfor.sServerMain.sPort.Data_a8);  
    } else
    {
        sprintf((char*) aTEMP, "%s:%s\r\n", sModemInfor.sServerBack.sIP.Data_a8,
                                        sModemInfor.sServerBack.sPort.Data_a8); 
    }

    SV_Protocol_Packet_Data(pData->Data_a8, &pData->Length_u16, OBIS_SER_ADD, aTEMP, Length, 0xAA);
    
#endif
    
    *(pData->Data_a8 + pData->Length_u16++) = OBIS_DCU_TYPE;  
    *(pData->Data_a8 + pData->Length_u16++) = 0x01;   
    *(pData->Data_a8 + pData->Length_u16++) = 0x00;   
    
    *(pData->Data_a8 + pData->Length_u16++) = OBIS_INDEX_OBIS_TSVH;   
    *(pData->Data_a8 + pData->Length_u16++) = 0x08;
    *(pData->Data_a8 + pData->Length_u16++) = OBIS_TIME_DEVICE; 
    *(pData->Data_a8 + pData->Length_u16++) = OBIS_ENVI_CLO_DU; 
    *(pData->Data_a8 + pData->Length_u16++) = OBIS_ENVI_PH_WATER; 
    *(pData->Data_a8 + pData->Length_u16++) = OBIS_ENVI_NTU; 
    *(pData->Data_a8 + pData->Length_u16++) = OBIS_ENVI_SALINITY_UNIT; 
    *(pData->Data_a8 + pData->Length_u16++) = OBIS_ENVI_OXY_TEMPERATURE; 
    *(pData->Data_a8 + pData->Length_u16++) = OBIS_ENVI_EC; 
    *(pData->Data_a8 + pData->Length_u16++) = OBIS_RSSI_1; 
    *(pData->Data_a8 + pData->Length_u16++) = OBIS_FREQ_SEND; 
    
//    //Firmware version
//    SV_Protocol_Packet_Data(pData->Data_a8, &pData->Length_u16, OBIS_FW_VERSION, sFirmVersion.Data_a8, sFirmVersion.Length_u16, 0xAA);
// 
//#ifdef USING_APP_ETHERNET
//    //Network Sim or Ethernet
//    if (sAppEthVar.Status_u8 == _ETH_MQTT_CONNECTED)
//    {
//        SV_Protocol_Packet_Data(pData->Data_a8, &pData->Length_u16, OBIS_NET_STATUS, (uint8_t *) "Ethernet", 8, 0xAA);  
//        
//    } else
//    {
//        SV_Protocol_Packet_Data(pData->Data_a8, &pData->Length_u16, OBIS_NET_STATUS, (uint8_t *)"Cellular", 8, 0xAA);
//    }
//#endif
//    
//    SV_Protocol_Packet_Data(pData->Data_a8, &pData->Length_u16, OBIS_TYPE, &DcuType, 1, 0xAA);
//
//#ifdef USING_APP_WM 
//    //Number channel water meter
//    SV_Protocol_Packet_Data(pData->Data_a8, &pData->Length_u16, OBIS_NUM_CHANNEL_WM, &sWmVar.nChannel_u8, 1, 0xAA);
//
//    //setting press and pulse
//    for (i = 0; i < MAX_CHANNEL; i++)
//    {
//        Length = AppWm_Packet_Setting(aTEMP, i);
//        
//        *(pData->Data_a8 + pData->Length_u16++) = OBIS_CHANNEL_WM_CONFIG;
//        *(pData->Data_a8 + pData->Length_u16++) = Length;
//        
//        for (uint8_t count = 0; count < Length; count++)
//            *(pData->Data_a8 + pData->Length_u16++) = aTEMP[count];
//    }
//#endif
    
    // caculator crc
    pData->Length_u16++;
	for (i = 0; i < (pData->Length_u16 - 1); i++)
		TempCrc ^= *(pData->Data_a8 + i);

    *(pData->Data_a8 + pData->Length_u16 - 1) = TempCrc;    
}


void Modem_Packet_Mess_Status (sData *pData)
{
    uint8_t	i = 0;
	uint8_t TempCrc = 0;
    char aTEMP[20] = {0};
#ifdef USING_APP_SIM
    sData *pstrTemp;
#endif
    
#ifdef USING_APP_ETHERNET
    uint8_t aNET_SIM[8] = {"Cellular"};
    uint8_t aNET_ETH[8] = {"Ethernet"};
#endif
    
*(pData->Data_a8 + pData->Length_u16++)  = '#';
#ifdef USING_APP_SIM
	// SIM_ID
    pstrTemp = AppSim_Get_Sim_ID();
    
	for (i = 0; i < pstrTemp->Length_u16; ++i)
		*(pData->Data_a8 + pData->Length_u16++)  = *(pstrTemp->Data_a8 + i);
#endif
    //Insert Version
    *(pData->Data_a8 + pData->Length_u16++)  = '#';
    for (i = 0; i < sFirmVersion.Length_u16; ++i)
		*(pData->Data_a8 + pData->Length_u16++)  = sFirmVersion.Data_a8[i];

    *(pData->Data_a8 + pData->Length_u16++)  = '#';
#ifdef USING_APP_SIM
//    //Test them GPS
//    if (sAppSimVar.sGPS.Status_u8 == true)
//    {
//        for (i = 0; i < sAppSimVar.sGPS.Length_u8; ++i)
//            *(pData->Data_a8 + pData->Length_u16++)  = sAppSimVar.sGPS.aPOS_INFOR[i];
//    }
#endif
    
    *(pData->Data_a8 + pData->Length_u16++)  = '#';
#ifdef USING_APP_SIM
    sprintf(aTEMP, "CSQ-%d", sSimCommInfor.RSSI_u8);
    
    for (i = 0; i < strlen(aTEMP); i++)
        *(pData->Data_a8 + pData->Length_u16++) = aTEMP[i];
#endif
    
    *(pData->Data_a8 + pData->Length_u16++)  = '#';
#ifdef USING_APP_ETHERNET
    if (sAppEthVar.Status_u8 == _ETH_MQTT_CONNECTED)
    {
        for (i = 0; i < 8; i++)
            *(pData->Data_a8 + pData->Length_u16++) = aNET_ETH[i];   
        
    } else
    {
        for (i = 0; i < 8; i++)
            *(pData->Data_a8 + pData->Length_u16++) = aNET_SIM[i];
    }
#endif
    *(pData->Data_a8 + pData->Length_u16++)  = '#';
    
	// caculator crc
	pData->Length_u16++;
	for (i = 0; i < (pData->Length_u16 - 1); i++)
		TempCrc ^= *(pData->Data_a8 + i);

	*(pData->Data_a8 + pData->Length_u16 - 1) = TempCrc;
}


void Modem_Packet_Mess_Opera (sData *pData)
{
    for (uint16_t i = 0; i < sMQTT.sDataFromMem.Length_u16; i++)
    {
        *(pData->Data_a8 + pData->Length_u16++) = *(sMQTT.sDataFromMem.Data_a8 + i);
    }
}

void Modem_Packet_Mess_Intan (sData *pData)
{
  
#if defined(USING_APP_SENSOR_RS485) 
        pData->Length_u16 = SensorRS485_Packet_TSVH (pData->Data_a8);
#endif
  
//#if defined(USING_APP_WM) || defined (USING_APP_EMET)
//#ifdef PAYLOAD_HEX_FORMAT
//    AppWm_Packet_TSVH (pData);
//#else
//    AppWm_Log_Data_TSVH();
//#endif
//#endif
//   
//#if defined(USING_APP_TEMH) 
//    pData->Length_u16 = AppTemH_Packet_TSVH (pData);
//#endif
//    
//
//    
//#if defined(USING_APP_LORA)  
//    for (uint16_t i = 0; i < sLoraVar.sIntanData.Length_u16; i++)
//        *(pData->Data_a8 + pData->Length_u16++) = *(sLoraVar.sIntanData.Data_a8 + i); 
//#endif
}


void Modem_Packet_Mess_Alarm (sData *pData)
{
    //bang gia tri buff alarm
    if (sModemVar.strAlarm.Length_u16 > 512)
        sModemVar.strAlarm.Length_u16 = 512;  
            
    for (uint16_t i = 0; i < sModemVar.strAlarm.Length_u16; i++)
    {
        *(pData->Data_a8 + pData->Length_u16++) = *(sModemVar.strAlarm.Data_a8 + i);
    }

    Reset_Buff(&sModemVar.strAlarm);
}

void Modem_Packet_Mess_RespAT (sData *pData)
{
    if (sModemVar.strATResp.Length_u16 > 256)
        sModemVar.strATResp.Length_u16 = 256;
            
    for (uint16_t i = 0; i < sModemVar.strATResp.Length_u16; i++)
      *(pData->Data_a8 +  pData->Length_u16++) = *(sModemVar.strATResp.Data_a8 + i);
}


void Modem_Packet_Mess_SimID (sData *pData)
{
    uint8_t	    i = 0;
    uint8_t     TempCrc = 0;
    
#ifdef USING_APP_SIM
    //Seri SIM
    sData *pstrTemp = AppSim_Get_Sim_ID();
    SV_Protocol_Packet_Data(pData->Data_a8, &pData->Length_u16, OBIS_SERI_SIM, pstrTemp->Data_a8, pstrTemp->Length_u16, 0xAA);
#endif 

    // caculator crc
    pData->Length_u16++;
	for (i = 0; i < (pData->Length_u16 - 1); i++)
		TempCrc ^= *(pData->Data_a8 + i);

    *(pData->Data_a8 + pData->Length_u16 - 1) = TempCrc;
}


void Modem_SER_Config (sData *strRecv, uint16_t pos)
{
    uint16_t  PosFix = pos;
    uint8_t   TempLeng;
    uint8_t   ObisConfig = 0;
    uint8_t   TempU8 = 0, i = 0;
    uint16_t  TempU16 = 0;
    //chay tu tren xuong duoi de check tat ca cac obis cau hinh
    while ((PosFix + 4) <= strRecv->Length_u16)   //vi byte cuoi la crc
    {
        TempLeng = 0;
        ObisConfig = *(strRecv->Data_a8 + PosFix++);
        switch(ObisConfig)   //luon luon co 1 byte crc phía sau
        {
            case 0x01:  //Lenh set duty cycle: Ex: 0102000A09.
                if (sModemInfor.ModePower_u8 == _POWER_MODE_ONLINE)
                {
                    TempU16 = 0;
                    TempLeng = *(strRecv->Data_a8 + PosFix++);
                    for (i = 0; i < TempLeng; i++)
                        TempU16 = (TempU16 << 8 ) | *(strRecv->Data_a8 + PosFix++);

                    sModemInfor.sFrequence.DurOnline_u32 = TempU16;
                    
                } else
                {
                    TempU16 = 0;
                    TempLeng = *(strRecv->Data_a8 + PosFix++);
                    for (i = 0; i < TempLeng; i++)
                        TempU16 = (TempU16 << 8 ) | *(strRecv->Data_a8 + PosFix++);

                    sModemInfor.sFrequence.Duration_u32 = TempU16;
                }
                
                //Luu lai
                Modem_Save_Freq();
                Modem_Respond_Str(_AT_REQUEST_SERVER,"\r\nOK", 1);
                AppComm_Set_Next_TxTimer();
                break;
            case 0x02:   //Lenh Reset so xung ve 0
                PosFix++;
            #ifdef USING_APP_WM
                sPulse[0].Number_u32 = 0;
            #endif
                Modem_Respond_Str(_AT_REQUEST_SERVER, "\r\nRESET SO NUOC OK!", 1);
                break;
            case 0x03:  //set ve che do luu kho
                PosFix += 2;

                Modem_Respond_Str (_AT_REQUEST_SERVER, "\r\nOK", 1);
                sModemVar.rSaveBox_u8 = true;
                break;
            case 0x06:  //Set so lan thuc day. Cho dong bo voi Nuoc hoa binh cua tiep de thong nhat
                TempLeng = *(strRecv->Data_a8 + PosFix++);   //length
                TempU8 = *(strRecv->Data_a8 + PosFix++);

                if ((TempLeng == 1) && (TempU8 <= MAX_NUMBER_WAKE_UP))
                {
                    sModemInfor.sFrequence.NumWakeup_u8 = TempU8;

                    Modem_Save_Freq();
                    Modem_Respond_Str (_AT_REQUEST_SERVER, "OK", 1);
                } else
                    Modem_Respond_Str(_AT_REQUEST_SERVER, "FAIL", 0);
                break;
            default:
                return;
        }
    }
}

void Modem_SER_Setting (sData *strRecv, uint16_t pos)
{
    uint16_t  PosFix = pos;
    uint8_t   Obis, Len, scale, chann = 0xFF;
    uint8_t   Result = true, mPulseSett = false, mPressSett = false;
    uint16_t  i = 0;
    uint32_t  TempU32 = 0;
    
    //chay tu tren xuong duoi de check tat ca cac obis cau hinh //vi byte cuoi la crc
    while ( (PosFix + 4) <= strRecv->Length_u16 )   
    {
        Len = 0; TempU32 = 0;
        
        Obis = *(strRecv->Data_a8 + PosFix++);
        Len = *(strRecv->Data_a8 + PosFix++);
        
        if (Len <= 8)
        {
            for (i = 0; i < Len; i++)
                TempU32 = TempU32 * 256 + *(strRecv->Data_a8 + PosFix++);    
        } else
        {
            Result = false;
            break;
        }
        
        switch (Obis)   
        {
            case OBIS_SERI_SENSOR: //lay byte cuoi cung
                chann = (uint8_t) TempU32 - 0x30;
                if ((chann == 0) || (chann > MAX_CHANNEL))
                    Result = false;
                break;
            case OBIS_SETT_PULSE_FACTOR:                 
                //lay scale vao he so xung
                sPulse[chann - 1].Factor_u8 = UTIL_Get_Scale(TempU32);
                sPulse[chann - 1].Factor_u8 += *(strRecv->Data_a8 + PosFix++);
                mPulseSett = true;
                break;
            case OBIS_SETT_PULSE_START:   
                scale = *(strRecv->Data_a8 + PosFix++);
                sPulse[chann - 1].Start_u32 = (uint32_t) ( TempU32 * Convert_Scale((uint8_t ) (scale - sPulse[chann - 1].Factor_u8)) );
                
                mPulseSett = true;
                break;
            case OBIS_SETT_LI_IN_MIN:                
                PosFix++; // scale
                sWmVar.aPRESSURE[chann - 1].sLinearInter.InMin_u16 = ( uint16_t ) TempU32;    
                
                mPressSett = true;
                break;
            case OBIS_SETT_LI_IN_MAX:
                PosFix++; // scale
                sWmVar.aPRESSURE[chann - 1].sLinearInter.InMax_u16 = ( uint16_t ) TempU32;
                
                mPressSett = true;
                break;
            case OBIS_SETT_LI_IN_UNIT:
                if (TempU32 <= _UNIT_BAR)
                    sWmVar.aPRESSURE[chann - 1].sLinearInter.InUnit_u8 = (uint8_t) TempU32;
                
                mPressSett = true;
                break;
            case OBIS_SETT_LI_OUT_MIN:
                PosFix++; // scale
                sWmVar.aPRESSURE[chann - 1].sLinearInter.OutMin_u16 = ( uint16_t ) TempU32; 
                
                mPressSett = true;
                break;
            case OBIS_SETT_LI_OUT_MAX:
                PosFix++; // scale
                sWmVar.aPRESSURE[chann - 1].sLinearInter.OutMax_u16 = ( uint16_t ) TempU32;  
                
                mPressSett = true;
                break;
            case OBIS_SETT_LI_OUT_UNIT:
                if (TempU32 <= _UNIT_BAR)
                    sWmVar.aPRESSURE[chann - 1].sLinearInter.OutUnit_u8 = (uint8_t) TempU32;
                
                mPressSett = true;
                break;
            case OBIS_SETT_PRESS_FACTOR:                
                PosFix++; // scale
                sWmVar.aPRESSURE[chann - 1].sLinearInter.Factor_u16 = (uint16_t) TempU32;
                
                mPressSett = true;
                break;
            case OBIS_SETT_PRESS_TYPE:
                if (TempU32 <= 1)
                    sWmVar.aPRESSURE[chann - 1].sLinearInter.Type_u8 = (uint8_t) TempU32;
                break;
            default:
                Result = false;
                break;
        }
        
        if (Result == false)
            break;
    }
    
    if (Result == false)
    {
        Modem_Respond_Str(_AT_REQUEST_SERVER, "FAIL", 0);
    } else
    {
        //luu lai gia tri config
        if (mPressSett == true)
            AppWm_Save_Press_Infor();
        
        if (mPulseSett == true)
            AppWm_Save_Pulse();
            
        Modem_Respond_Str (_AT_REQUEST_SERVER, "OK", 1);        
        //danh dau ban tin cap nhat config
        sMQTT.aMARK_MESS_PENDING[DATA_HANDSHAKE] = TRUE;
    }
}


void Modem_SER_Set_Time (sData *strRecv, uint16_t Pos)
{
    ST_TIME_FORMAT 	sRTC_temp = {0};

	if (Pos > 0)
    { // nhan duoc ban tin Realtime
		sRTC_temp.year 		= *(strRecv->Data_a8 + Pos);
		sRTC_temp.month 	= *(strRecv->Data_a8 + Pos + 1);
		sRTC_temp.date 		= *(strRecv->Data_a8 + Pos + 2);
		sRTC_temp.day 		= *(strRecv->Data_a8 + Pos + 3);
		sRTC_temp.hour 		= *(strRecv->Data_a8 + Pos + 4);
		sRTC_temp.min 		= *(strRecv->Data_a8 + Pos + 5);
		sRTC_temp.sec 		= *(strRecv->Data_a8 + Pos + 6);
	}
    
    if (Modem_Set_sTime(&sRTC_temp, 0) == true)
    {
        sMQTT.aMARK_MESS_PENDING[SEND_SERVER_TIME_OK] = 1;
	} else
    {
		if ((*(strRecv->Data_a8 + Pos + 2) == 'U') && (*(strRecv->Data_a8 + Pos + 3) == 'd'))
		{ // Cap nhat thoi gian
			sMQTT.aMARK_MESS_PENDING[SEND_SERVER_TIME_PENDING] = 1;
		} else
			sMQTT.aMARK_MESS_PENDING[SEND_SERVER_TIME_FAIL] = 1;
	}
}


void Modem_SER_Req_AT (sData *strRecv, uint16_t pos)
{
    Modem_Check_AT(strRecv, _AT_REQUEST_SERVER);  
}

void Modem_SER_Req_RF (sData *strRecv, uint16_t pos)
{
    Modem_Check_AT(strRecv, _AT_REQUEST_LORA);  
}


void Modem_SER_Req_Update (sData *strRecv, uint16_t pos)
{
    if (sModemInfor.ModePower_u8 == _POWER_MODE_ONLINE)
    {
        // Split FTP
        if (Modem_Split_Update_Infor(strRecv, pos) == TRUE)
        {

        } else
            Modem_Respond_Str(_AT_REQUEST_SERVER, "\r\nERROR", 0);
    } else
    {
        Modem_Respond_Str(_AT_REQUEST_SERVER, "\r\nERROR", 0);
    }
}




/*
 *  Return 	1 : Neu chia cac chuoi ten ra thanh cong
 *  		0 : ko thanh cong
 */

uint8_t	 Modem_Split_Update_Infor (sData *str_Receiv, uint16_t Pos)
{
#ifdef USING_APP_SIM
//    //Start path
//    Reset_Buff(&sSimFwUpdate.StrURL_FirmPath);
//	if (Cut_String(str_Receiv,&Pos, &sSimFwUpdate.StrURL_FirmPath) == FALSE)
//        return 0;
//
//    Reset_Buff(&sSimFwUpdate.StrURL_FirmName);
//	if (Cut_String(str_Receiv, &Pos, &sSimFwUpdate.StrURL_FirmName) == FALSE)
//        return 0;
//
//    //IP port user pass de mac dinh
#endif
	return 1;
}





uint8_t Modem_Check_AT(sData *StrUartRecei, uint8_t Type)
{
#ifdef USING_AT_CONFIG
	uint8_t var;
	int Pos_Str = -1;
	uint16_t i = 0;
	sData sDataConfig = {&aDATA_CONFIG[0], 0};

	//convert lai chu in hoa thanh chu thuong
	for (i = 0; i < StrUartRecei->Length_u16; i++)
	{
		if ((*(StrUartRecei->Data_a8 + i) == '=') || (*(StrUartRecei->Data_a8 + i) == '?'))
			break;

		if ((*(StrUartRecei->Data_a8 + i) >= 0x41) && (*(StrUartRecei->Data_a8 + i) <= 0x5A))
			*(StrUartRecei->Data_a8 + i) = *(StrUartRecei->Data_a8 + i) + 0x20;
	}

	for (var = _SET_DEV_SERIAL; var < _END_AT_CMD; var++)
	{
		Pos_Str = Find_String_V2((sData*) &CheckList_AT_CONFIG[var].sTempReceiver, StrUartRecei);
        
		if ((Pos_Str >= 0) && (CheckList_AT_CONFIG[var].CallBack != NULL))
		{
            if (CheckList_AT_CONFIG[var].CallBack == NULL)
            {
                Modem_Respond_Str(Type, "\r\nNOT SUPPORT!\r\n", 0);
                
                return 1;
            }
            
            //Copy lenh vao buff. de repond kem theo lenh
            Reset_Buff(&strATcmd);

            for (i = 0; i <(StrUartRecei->Length_u16 - Pos_Str); i++)
            {
                if ((*(StrUartRecei->Data_a8 + Pos_Str + i) == '?') || (*(StrUartRecei->Data_a8+Pos_Str+i) == '=') \
                    || (strATcmd.Length_u16 >= LENGTH_AT_SERIAL))
                    break;
                else 
                    *(strATcmd.Data_a8 + strATcmd.Length_u16++) = *(StrUartRecei->Data_a8+Pos_Str+i); 
            }
            //Copy data after at cmd
            Pos_Str += CheckList_AT_CONFIG[var].sTempReceiver.Length_u16;

            for (i = Pos_Str; i < StrUartRecei->Length_u16; i++)
            {
                if (*(StrUartRecei->Data_a8 + i) == 0x0D)
                {
                    if ( (i < (StrUartRecei->Length_u16 - 1)) && (*(StrUartRecei->Data_a8 + i + 1) == 0x0A) )
                    {
                        break;
                    }
                }
                //
                if ( sDataConfig.Length_u16 < sizeof(aDATA_CONFIG) )
                    *(sDataConfig.Data_a8 + sDataConfig.Length_u16++) = *(StrUartRecei->Data_a8 + i);
            }
            
            PortConfig = Type;
            
            //Test
            switch (Type)
            {
                case _AT_REQUEST_SERIAL:
                case _AT_REQUEST_SERVER: 
                    //check at request
                    CheckList_AT_CONFIG[var].CallBack(&sDataConfig, 0);
                    break;
                case _AT_REQUEST_LORA:       
                #ifdef USING_APP_LORA
                    //Copy data to lora at cmd handle
                    shLora.LengthConfig_u8 = 0;
                
                    for (i = 0; i < CheckList_AT_CONFIG[var].sTempReceiver.Length_u16; i++)
                    {
                        shLora.aDATA_CONFIG[shLora.LengthConfig_u8++] = *(CheckList_AT_CONFIG[var].sTempReceiver.Data_a8 + i); 
                    } 
                        
                    for (i = 0; i <sDataConfig.Length_u16; i++)
                    {
                        shLora.aDATA_CONFIG[shLora.LengthConfig_u8++] = *(sDataConfig.Data_a8 + i); 
                    }
                    
                    shLora.rAtcmd_u8 = TRUE;  
                
                    #ifndef DEVICE_TYPE_GATEWAY 
                        CheckList_AT_CONFIG[var].CallBack(&sDataConfig, 0);
                    #endif
                #endif
                    break;
                default:
                    break;
            }

			return 1;
		}
	}
#endif
    
	return 0;
}




void Modem_Monitor_Connect_Server (void)
{
//    //Kiem tra doi server
//    uint8_t Sub1 = 0, Sub2 = 0;
//    static uint16_t LastcSimHardReset = 0;
//    static uint16_t LastcEthHardReset = 0;
//        
//#ifdef USING_APP_SIM
//    if (sAppSimVar.cHardReset_u16 == 0)
//        LastcSimHardReset = 0;
//    else
//    {
//        //Cacul Diff Hard RS
//        if (sAppSimVar.cHardReset_u16 > LastcSimHardReset)
//            Sub1 = sAppSimVar.cHardReset_u16 - LastcSimHardReset;
//        else
//            Sub1 = LastcSimHardReset - sAppSimVar.cHardReset_u16;
//    }
//#endif
//    
//#ifdef USING_APP_ETHERNET
//    if (sAppEthVar.cHardReset_u16 == 0)
//        LastcEthHardReset = 0;
//    else
//    {
//        //Cacul Diff Hard RS
//        if (sAppEthVar.cHardReset_u16 > LastcEthHardReset)
//            Sub2 = sAppEthVar.cHardReset_u16 - LastcEthHardReset;
//        else
//            Sub2 = LastcEthHardReset - sAppEthVar.cHardReset_u16;
//    }
//#endif
//    
//    //Check Change Server
//#ifdef USING_APP_SIM
//    if (Sub1 >= MAX_SIM_CHANGE_SERVER)
//#endif
//    {
//    #ifdef USING_APP_ETHERNET
//        if (Sub2 >= MAX_ETH_CHANGE_SERVER)
//    #endif
//        {
//            if ((sModemVar.ModeConnNow_u8 != MODE_CONNECT_DATA_MAIN) 
//                && (sModemVar.ModeConnNow_u8 != MODE_CONNECT_DATA_BACKUP))
//            {
//                //doi mode neu dang o update
//                sModemVar.ModeConnNow_u8 = sModemVar.ModeConnLast_u8;
//                sMQTT.aMARK_MESS_PENDING[SEND_UPDATE_FIRM_FAIL] = TRUE;
//            } else
//            {
//                UTIL_Log_Str(DBLEVEL_M, "u_modem: change server >>> \r\n" );
//                Modem_Change_Server();
//            }
//            //Mark Count HRS
//            LastcSimHardReset = sAppSimVar.cHardReset_u16;
//            LastcEthHardReset = sAppEthVar.cHardReset_u16;
//        }
//    }
//
//    //Kiem tra ket noi: de reset thiet bi
//#ifdef USING_APP_SIM
//    if ( (sAppSimVar.cHardReset_u16 >= MAX_HARD_RESET) 
//        && (Check_Time_Out(sAppSimVar.LandMarkConnMQTT_u32, 600000) == true) )
//#endif
//    {  
//    #ifdef USING_APP_ETHERNET
//        if ( (sAppEthVar.cHardReset_u16 >= MAX_ETH_HARD_RESET) 
//            && (Check_Time_Out(sAppEthVar.LandMarkConnMQTT_u32, 600000) == true) )
//    #endif
//        {
//            if ((sModemVar.ModeConnNow_u8 == MODE_CONNECT_DATA_MAIN) 
//                || (sModemVar.ModeConnNow_u8 == MODE_CONNECT_DATA_BACKUP))
//            {
//                if (sModemInfor.ModePower_u8 == _POWER_MODE_ONLINE)
//                    Modem_Reset_MCU();
//            }
//        }
//    }
}



uint8_t Modem_Set_sTime (ST_TIME_FORMAT *sClock, uint8_t GMT)
{
    if (Check_update_Time(sClock) == true)
    {
        sModemVar.sRTCSet.year   = sClock->year;
        sModemVar.sRTCSet.month  = sClock->month;
        sModemVar.sRTCSet.date   = sClock->date;
        sModemVar.sRTCSet.hour   = sClock->hour;
        sModemVar.sRTCSet.min    = sClock->min;
        sModemVar.sRTCSet.sec    = sClock->sec;
        sModemVar.sRTCSet.day    = sClock->day;

        Convert_sTime_ToGMT(&sModemVar.sRTCSet, GMT); 
        //Convert lai day. 1/1/2012 la chu nhat. Thu 2 - cn: 2-8
        sModemVar.sRTCSet.day = ((HW_RTC_GetCalendarValue_Second (sModemVar.sRTCSet, 1) / SECONDS_IN_1DAY) + 6) % 7 + 1;
        
        fevent_active(sEventAppComm, _EVENT_SET_RTC);
        
        return true;
    }
    
    return false;
}



sData* Modem_Get_Firm_Version (void)
{
    return &sFirmVersion;
}

sData* Modem_Get_ID (void)
{
    return &sModemInfor.sId;
}



uint8_t Modem_Change_Server (void)
{
    //Doi server
    if (sModemVar.ModeConnNow_u8 == MODE_CONNECT_DATA_BACKUP)
    {
        sModemVar.ModeConnNow_u8 = MODE_CONNECT_DATA_MAIN;
        Modem_Set_Server_Infor_To_App();
    } else if (sModemVar.ModeConnNow_u8 == MODE_CONNECT_DATA_MAIN)
    {
        sModemVar.ModeConnNow_u8 = MODE_CONNECT_DATA_BACKUP;
        Modem_Set_Server_Infor_To_App();
    }
    
#ifdef USING_APP_ETHERNET
    sAppEthVar.Status_u8  = _ETH_START;
    fevent_active(sEventAppEth, _EVENT_ETH_HARD_RESET);
#endif
    
#ifdef USING_APP_SIM
    if (sAppEthVar.Status_u8 < _ETH_TCP_CONNECT)
        AppSim_Restart_Imediate();
#endif
    
    return 1;
}



