

#include "user_app_sim.h"
#include "user_string.h"
#include "math.h"
#include "user_timer.h"
#include "user_time.h"

#include "user_uart.h"


/*================ Var struct =================*/  
static uint8_t _Cb_Sim_Hard_Reset(uint8_t event);
static uint8_t _Cb_Sim_Turn_On (uint8_t event);
static uint8_t _Cb_Sim_WakeUp(uint8_t event);
static uint8_t _Cb_Sim_Ctrl_DTR(uint8_t event);
static uint8_t _Cb_Sim_Power_Off(uint8_t event);

static uint8_t _Cb_Event_Sim_Send_Ping (uint8_t event);
static uint8_t _Cb_Event_Sim_Send_Mess (uint8_t event);
static uint8_t _Cb_Event_Sim_Req_GPS (uint8_t event);
static uint8_t _Cb_Event_Sim_Get_GPS (uint8_t event);
static uint8_t _Cb_Event_Sim_Sms_Ctrl (uint8_t event);
static uint8_t _Cb_Event_Sim_Call_Ctrl (uint8_t event);

sEvent_struct sEventAppSim[] = 
{
    { _EVENT_SIM_HARD_RESET,		0, 0, 0, 		_Cb_Sim_Hard_Reset },
	{ _EVENT_SIM_TURN_ON, 			0, 0, 0, 		_Cb_Sim_Turn_On },
	{ _EVENT_SIM_WAKEUP, 			0, 0, 0, 		_Cb_Sim_WakeUp },
    { _EVENT_SIM_DTR_PIN, 	        0, 0, 1000, 	_Cb_Sim_Ctrl_DTR },
    { _EVENT_SIM_POWER_OFF, 		0, 0, 0, 		_Cb_Sim_Power_Off },
    
    { _EVENT_SIM_SEND_MESS,  		0, 0, 1000,     _Cb_Event_Sim_Send_Mess },  
    { _EVENT_SIM_SEND_PING,  		1, 0, 30000,    _Cb_Event_Sim_Send_Ping },
    { _EVENT_SIM_REQ_GPS,  		    1, 0, 5000,     _Cb_Event_Sim_Req_GPS },  
    { _EVENT_SIM_GET_GPS,  		    1, 0, 100,      _Cb_Event_Sim_Get_GPS },   
    
    { _EVENT_SIM_SMS_CTRL,  		0, 0, 1000,     _Cb_Event_Sim_Sms_Ctrl }, 
    { _EVENT_SIM_CALL_CTRL,  		0, 0, 1000,     _Cb_Event_Sim_Call_Ctrl },  
};


static void AppSim_AT_Faillure (uint8_t Step);
static void AppSim_Handle_AT (uint8_t step);
static void AppSim_Process_URC (uint8_t Type);
static void AppSim_Uart_Send (uint8_t *pData, uint16_t length);

static uint8_t aDATA_GPS[256];

sAppSimVariable sAppSimVar = 
{
    .sDataGPS      = {&aDATA_GPS[0], 0},
};



static UTIL_TIMER_Object_t TimerPowerOn;
static void OnTimerPowerOn(void *context);

sFuncCallbackSim sAppSimCallback = 
{
    .pFailure       = AppSim_AT_Faillure,
    .pHandle_AT     = AppSim_Handle_AT,
    .phandle_URC    = AppSim_Process_URC,
    .pGet_GPS_OK    = AppSim_GPS_OK,
    .pGet_GPS_ERROR = AppSim_GPS_Error,
    .pConfig_Update = AppSim_Config_Update,
    .pSend          = AppSim_Uart_Send,
};


static uint8_t aAPN_NAME[BUFF_LENGTH_SETUP] = "SV123";  
static uint8_t aAPN_DIAL_STRING[BUFF_LENGTH_SETUP] = "12345";
static uint8_t aAPN_USERNAME[BUFF_LENGTH_SETUP] = "xyz";  
static uint8_t aAPN_PASSWORD[BUFF_LENGTH_SETUP] = "1111";

sAppSimInformation   sAppSimInfor = 
{    
    .sAPN.sName    = {&aAPN_NAME[0], 5},
    .sAPN.sDialString = {&aAPN_DIAL_STRING[0], 5},
    .sAPN.sUsername   = {&aAPN_USERNAME[0], 3},
    .sAPN.sPassword   = {&aAPN_PASSWORD[0], 4},
};

/*================ Func =================*/


static uint8_t _Cb_Sim_Hard_Reset(uint8_t event)
{
    fevent_active(sEventAppSim, _EVENT_SIM_TURN_ON);

	return 1;
}



static uint8_t _Cb_Sim_Turn_On (uint8_t event)
{
    AppSim_Default_Sms_Call();
    Sim_Common_Init(&sSimCommVar);
    //Disable Event
	Sim_Disable_All_Event();  
    //Power on 
	if (AppSim_Power_On() == 1)
	{
        sSimCommVar.State_u8  = _SIM_INIT;
        //Push AT cmd 
        if (*sAppSimVar.pModeConnFuture_u8 != 0)
        {
            if ((*sAppSimVar.pModeConnNow_u8 == MODE_CONNECT_DATA_MAIN) 
                || (*sAppSimVar.pModeConnNow_u8 == MODE_CONNECT_DATA_BACKUP))
            {
                *sAppSimVar.pModeConnLast_u8 = *sAppSimVar.pModeConnNow_u8;    //Luu lai mode cuoi cung
            }
            
            *sAppSimVar.pModeConnNow_u8 = *sAppSimVar.pModeConnFuture_u8;
            *sAppSimVar.pModeConnFuture_u8 = 0; 
        }
        //Check Mode 
        Modem_Set_Server_Infor_To_App();
        
        switch (*sAppSimVar.pModeConnNow_u8)
        {
            case MODE_CONNECT_DATA_MAIN:    
            case MODE_CONNECT_DATA_BACKUP:
                Sim_Clear ();
                Sim_Push_Block_To_Queue(aSIM_BLOCK_STEP_INIT, sizeof(aSIM_BLOCK_STEP_INIT));  
                Sim_Push_Block_To_Queue(aSIM_BLOCK_STEP_NETWORK, sizeof(aSIM_BLOCK_STEP_NETWORK));
                
                if (AppSim_Need_Sms() == true)
                    AppSim_Push_Send_Sms();
                
                if (sAppSimVar.sCall.Mark_u8 == true)
                    AppSim_Push_Call();
                
                Sim_Push_Block_To_Queue(aSIM_BLOCK_STEP_CONNECT, sizeof(aSIM_BLOCK_STEP_CONNECT));
                break;
            case MODE_CONNECT_FTP:
                sAppSimVar.cHardReset_u16 += MAX_SIM_CHANGE_SERVER*2;  //de chuyen mode conn
                break;
            case MODE_CONNECT_HTTP:
                //Dung chung http va ftp = http
                Sim_Clear ();
                Sim_Push_Block_To_Queue(aSIM_BLOCK_STEP_INIT, sizeof(aSIM_BLOCK_STEP_INIT)); 
                Sim_Push_Block_To_Queue(aSIM_BLOCK_STEP_HTTP_INIT, sizeof(aSIM_BLOCK_STEP_HTTP_INIT));
                Sim_Push_Block_To_Queue(aSIM_BLOCK_STEP_HTTP_READ, sizeof(aSIM_BLOCK_STEP_HTTP_READ));
                break;
            default:
                break;      
        } 
	}

	return 1;
}


static uint8_t _Cb_Sim_WakeUp(uint8_t event)
{
	return 1;
}

/*
    Func: Control DTR PIN to Out Data mode
    Lan dau tien vao keo chan DTR low -> 1s sau Keo DTR lï¿½n high: High to low?
*/

static uint8_t _Cb_Sim_Ctrl_DTR(uint8_t event)
{
    static uint8_t DTRStep_u8 = 0;
    
    if (DTRStep_u8 == 0)
    {
        SIM_DTR_ON1;
        DTRStep_u8++;
        fevent_enable(sEventAppSim, _EVENT_SIM_DTR_PIN);
    } else
    {
        SIM_DTR_OFF1;
        DTRStep_u8 = 0;
    }
    
	return 1;
}

static uint8_t _Cb_Sim_Power_Off(uint8_t event)
{
    Sim_Disable_All_Event();
    
    UTIL_TIMER_Stop (&TimerPowerOn);
    sAppSimVar.StepPowerOn_u8 = 0;

    UTIL_Log_Str(DBLEVEL_M, "u_app_sim: power off SIM\r\n" );
    //clear queue
    Sim_Clear ();
    //Set status PSM
    sSimCommVar.State_u8 = _SIM_POWER_OFF;
    //Tat ngat nhan Uart
    Sim_Stop_RX_Mode();
    
    //Power off bang cach ngat nguon dien
    SIM_PW_OFF1; 
    
    return 1;
}


static uint8_t _Cb_Event_Sim_Send_Ping (uint8_t event)
{
    static uint8_t Count_Ping = 0;
    
    if (*sAppSimVar.pModePower_u8 == _POWER_MODE_ONLINE)
    {
        sMQTT.aMARK_MESS_PENDING[DATA_PING] = TRUE;
        fevent_enable(sEventAppSim, _EVENT_SIM_SEND_PING);   
    } else
    {
        Count_Ping++;
        
        if (Count_Ping < MAX_PING_TO_CONFIG)
        {
            sMQTT.aMARK_MESS_PENDING[DATA_PING] = TRUE;
            fevent_enable(sEventAppSim, _EVENT_SIM_SEND_PING);            //Tiep tuc cho check Event PING sau 30s.
        } else
        {
            //Ket thuc ping ->Di ngu sau
            Count_Ping = 0;
            sAppSimVar.rExternIrq_u8 = false;
        }
    }
    
    return 1;
}


/*
    Cu 5s vao day de push at get GPS
*/
static uint8_t _Cb_Event_Sim_Req_GPS (uint8_t event)
{
 
    return 1;
}


static uint8_t _Cb_Event_Sim_Get_GPS (uint8_t event)
{

    return 1;
}


static uint8_t _Cb_Event_Sim_Sms_Ctrl (uint8_t event)
{
    static uint8_t Retry = 0;
    
    if (sSimCommVar.State_u8 >= _SIM_SETTING)
    {
        if (sAppSimVar.sSms.Pending_u8 == false)
        {
            if (AppSim_Need_Sms() == true)
            {
                AppSim_Push_Send_Sms();
            } else
            {
                Retry = 0;
            }
        }
    }

    //qua so lan retry
    if (Retry >= 3)
    {
        Retry = 0;
        sAppSimVar.sCall.Pending_u8 = false;
        sAppSimVar.sCall.Mark_u8 = false;
    }
    
     //check queue sim ma k co lenh at thi push lại call
    if (qGet_Number_Items(&qSimStep) == 0)
    {
        sAppSimVar.sSms.Pending_u8 = false;
    }
    
    fevent_enable(sEventAppSim, event); 
    
    return 1;
}


static uint8_t _Cb_Event_Sim_Call_Ctrl (uint8_t event)
{
    static uint8_t Retry = 0;
    
    if (sSimCommVar.State_u8 >= _SIM_SETTING)
    {
        if (sAppSimVar.sCall.Pending_u8 == false)
        {
            if (sAppSimVar.sCall.Mark_u8 == true)
            {
                Retry++;
                AppSim_Push_Call();
            } else
            {
                Retry = 0;
            }
        }
    }
    
    //qua so lan retry
    if (Retry >= 3)
    {
        Retry = 0;
        sAppSimVar.sCall.Pending_u8 = false;
        sAppSimVar.sCall.Mark_u8 = false;
    }
    
    //check queue sim ma k co lenh at thi push lại call
    if (qGet_Number_Items(&qSimStep) == 0)
    {
        sAppSimVar.sCall.Pending_u8 = false;
    }

    fevent_enable(sEventAppSim, event); 
    
    return 1;
} 


/*
    
*/
static uint8_t _Cb_Event_Sim_Send_Mess (uint8_t event)
{
    if (sSimCommVar.State_u8 != _SIM_CONN_MQTT)
    {
        return 1;
    }
       
    //
    sSimCommVar.ServerReady_u8 = true;
    
    if (AppSim_Send_Mess () == true)  //Co ban tin
    {
        fevent_disable(sEventAppSim, _EVENT_SIM_SEND_PING);
        
        if (*sAppSimVar.pModePower_u8 == _POWER_MODE_ONLINE)
            sEventAppSim[event].e_period = 5000;
        else
            sEventAppSim[event].e_period = 2000;
    } else
    {
        sEventAppSim[event].e_period = 10;
        //Neu Online: enable Ping  | Enable event Power SIM
        if ( (*sAppSimVar.pModePower_u8 == _POWER_MODE_ONLINE)
            || (sAppSimVar.rExternIrq_u8 == true) )
        {
            //Mark Send PING
            if (sEventAppSim[_EVENT_SIM_SEND_PING].e_status == 0)
                fevent_enable(sEventAppSim, _EVENT_SIM_SEND_PING);
        } else
        {
            #ifdef USING_APP_MEM
                //Neu con ban tin moi: Enable lai check new rec
                if (AppMem_Is_New_Record() != true)
                {
                    sSimCommVar.ServerReady_u8 = false;
                    fevent_active(sEventAppSim, _EVENT_SIM_POWER_OFF);
                    return 1;
                }
            #else
                sSimCommVar.ServerReady_u8 = false;
                fevent_active(sEventAppSim, _EVENT_SIM_POWER_OFF);
                return 1;
            #endif
        }
    }
    
    fevent_enable(sEventAppSim, event);
    
    return 1;
}


/*============== Function Handler ====================*/   

/*
    Func Init App SIM
*/

void AppSim_Init (void)
{
    //Init callback 
    sCbSimComm.pFailure         = sAppSimCallback.pFailure;
    sCbSimComm.pHandle_AT       = sAppSimCallback.pHandle_AT;
    sCbSimComm.phandle_URC      = sAppSimCallback.phandle_URC;
    sCbSimComm.pGet_GPS_OK      = sAppSimCallback.pGet_GPS_OK;
    sCbSimComm.pGet_GPS_ERROR   = sAppSimCallback.pGet_GPS_ERROR;
    sCbSimComm.pConfig_Update   = sAppSimCallback.pConfig_Update; 
    sCbSimComm.pSend            = sAppSimCallback.pSend;
    
    //Init infor
    sSimCommInfor.sAPNInfor = &sAppSimInfor.sAPN;

#ifdef USING_AT_CONFIG
    CheckList_AT_CONFIG[_QUERY_SIM_ID].CallBack = AppSim_SER_Get_Sim_ID,
#endif
    
#ifdef USING_APP_WM
    sWmVar.pRq_Instant = AppSim_Set_Instant;
    sWmVar.pCtrl_Sim = AppSim_Allow_Send;
#endif
    
    //Init Module Sim
    Sim_Init();
         
#ifdef MQTT_PROTOCOL
    sMQTT.aMARK_MESS_PENDING[DATA_HANDSHAKE] = TRUE;
#endif
    
    sMQTT.aMARK_MESS_PENDING[DATA_INTAN] = TRUE;
    sMQTT.aMARK_MESS_PENDING[SEND_SERVER_TIME_PENDING] = TRUE;
    //Firt: mark error GPS
    AppSim_GPS_Error();
    
    //active power on first
    fevent_active(sEventAppSim, _EVENT_SIM_TURN_ON);
    UTIL_TIMER_Create(&TimerPowerOn, 0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnTimerPowerOn, NULL);
    
}


 
uint8_t AppSim_Task(void)
{
	uint8_t i = 0;
	uint8_t Result = 0;

	for (i = 0; i < _EVENT_END_SIM; i++)
	{
		if (sEventAppSim[i].e_status == 1)
		{
            if ( (i != _EVENT_SIM_REQ_GPS) && (i != _EVENT_SIM_GET_GPS) ) 
                Result = 1;
            
			if ((sEventAppSim[i].e_systick == 0) ||
					((HAL_GetTick() - sEventAppSim[i].e_systick)  >=  sEventAppSim[i].e_period))
			{
                sEventAppSim[i].e_status = 0; 
				sEventAppSim[i].e_systick = HAL_GetTick();
				sEventAppSim[i].e_function_handler(i);
			}
		}
	}
    
    Result |= Sim_Task();

	return Result;
}



/*-------Init From memory ----*/
/*
    Func: Check New Mess
*/

uint8_t AppSim_Send_Mess (void)
{
    uint16_t i = 0;
    uint8_t Result = FALSE;
    
    //Kiem tra xem co ban tin nao can gui di khong
    for (i = TOPIC_NOTIF; i < END_MQTT_SEND; i++)
    {
        if (sMQTT.aMARK_MESS_PENDING[i] == TRUE)
        {
            // Dang ket noi ethernet chi xu ly SIM Ping.
        #ifdef USING_APP_ETHERNET
            if ( ((sAppEthVar.Status_u8 >= _ETH_TCP_CONNECT) && (i != DATA_PING) ) 
                  || (i == DATA_PING_2)
                )
            {
                continue;
            }
        #endif
            
            Result = TRUE;
            //
            if (sMQTT.Status_u8 != PENDING)
            {
                //Danh dau Mess type hien tai ->neu OK clear di
                sMQTT.MessType_u8 = i;
                //Dong goi ban tin vao buff aPAYLOAD bang cach thuc hien callback
                if (sMark_MessageSend_Type[i].CallBack(i) == TRUE)
                {
                    //Day 2 step Publish vao Queue send AT: 2 option wait ACK and No wait ACK
                #ifdef MQTT_PROTOCOL      
//                    if (sMQTT.MessType_u8 == DATA_PING)
//                    {
//                        Sim_Push_Block_To_Queue(aSIM_BLOCK_STEP_PING, sizeof(aSIM_BLOCK_STEP_PING));
//                    } else
//                    {
                        if (sMQTT.PulishQos != 0)
                            Sim_Push_Block_To_Queue(aSIM_BLOCK_STEP_PUB_FB, sizeof(aSIM_BLOCK_STEP_PUB_FB));
                        else
                            Sim_Push_Block_To_Queue(aSIM_BLOCK_STEP_PUB, sizeof(aSIM_BLOCK_STEP_PUB)); 
//                    }
                #else
                    Sim_Push_Block_To_Queue(aSIM_BLOCK_STEP_HTTP_POST, sizeof(aSIM_BLOCK_STEP_HTTP_POST));
                #endif
                     //Set flag status= pending
                    sMQTT.Status_u8 = PENDING;
                } else
                    sMQTT.aMARK_MESS_PENDING[i] = FALSE;
            } 
        }
    }
    
    return Result;
}



/*
    Func: Func pointer Handler AT cmd
        + Input: Type: Get ID SIM,....
        + Ouput: Handler
*/


static void AppSim_Handle_AT (uint8_t step)
{    
    switch (step)
    { 
        case _SIM_COMM_GET_STIME:  
            _CbAppSim_Recv_sTime (&sSimCommInfor.sTime);
            break;
        case _SIM_COMM_SMS_SEND_1:  
            _CbAppSim_SMS_Send_1();
            break;
        case _SIM_COMM_SMS_SEND_2:  
            _CbAppSim_SMS_Send_2();
            break;
        case _SIM_COMM_SMS_SEND_3:  
            _CbAppSim_SMS_Send_3();
            break;
        case _SIM_COMM_CALLING_1:  
            _CbAppSim_Calling_1();
            break;
        case _SIM_COMM_CALLING_2:  
            _CbAppSim_Calling_2();
            break;
        case _SIM_COMM_TCP_SEND_1:  
            _CbAppSim_TCP_Send_1(&sSimCommVar.pSender);
            break;
        case _SIM_COMM_TCP_SEND_2:  
            sSimCommVar.pSender.Data_a8 = sMQTT.str.Data_a8;
            sSimCommVar.pSender.Length_u16 = sMQTT.str.Length_u16;
            break;
        case _SIM_COMM_MQTT_CONN_1:
            mConnect_MQTT( (char *) sMQTT.sModemID->Data_a8,
                           (char *) sSimCommInfor.sServer.sMQTTUserName->Data_a8, 
                           (char *) sSimCommInfor.sServer.sMQTTPassword->Data_a8,
                            *sSimCommInfor.sServer.MQTTKeepAlive_u32 );
  
            sSimCommVar.pSender.Data_a8 = sMQTT.str.Data_a8;
            sSimCommVar.pSender.Length_u16 = sMQTT.str.Length_u16;
            
//        #ifdef MQTT_PROTOCOL 
//            sMQTT.aMARK_MESS_PENDING[DATA_HANDSHAKE] = TRUE;
//        #endif
            break;
        case _SIM_COMM_MQTT_SUB_1:
            //Truyen chuoi Subcribe
            mSubcribe_MQTT ();
        
            sSimCommVar.pSender.Data_a8 = sMQTT.str.Data_a8;
            sSimCommVar.pSender.Length_u16 = sMQTT.str.Length_u16;
            break;
        case _SIM_COMM_MQTT_SUB_2:
            mSet_default_MQTT(); 
            fevent_active(sEventAppSim, _EVENT_SIM_SEND_MESS);
            UTIL_Printf_Str( DBLEVEL_M, "u_app_sim: mqtt connect ok!\r\n" );
            break;
        case _SIM_COMM_MQTT_PUB_1:
            sSimCommVar.pSender.Data_a8 = sMQTT.str.Data_a8;
            sSimCommVar.pSender.Length_u16 = sMQTT.str.Length_u16;           
            break;
        case _SIM_COMM_MQTT_PUB_2:
            _CbAppSim_Recv_PUBACK ();
            break;
        case _SIM_COMM_HTTP_HEAD_1:  
            _CbAppSim_Http_Head_1(&sMQTT.str);
            break;
         case _SIM_COMM_HTTP_HEAD_2:  
            _CbAppSim_Http_Head_2();
            break;
        case _SIM_COMM_HTTP_SEND_1:  
            _CbAppSim_Http_Send_1(&sMQTT.str);
            break;
        case _SIM_COMM_HTTP_SEND_2:  
            sSimCommVar.pSender.Data_a8 = sMQTT.str.Data_a8;
            sSimCommVar.pSender.Length_u16 = sMQTT.str.Length_u16;
            break;
        case _SIM_COMM_HTTP_SEND_EX:  
            _CbAppSim_Http_Send_Ex(&sMQTT.str);
            break;
        case _SIM_COMM_HTTP_READ_OK:
            Sim_Push_Block_To_Queue(aSIM_BLOCK_STEP_HTTP_READ, sizeof(aSIM_BLOCK_STEP_HTTP_READ)); 
            break; 
        case _SIM_COMM_HTTP_UPDATE_OK: 
        #ifdef USING_INTERNAL_MEM
            Erase_Firmware(ADDR_FLAG_HAVE_NEW_FW, 1);
            //ghi Flag update va Size firm vao Inflash
            HAL_FLASH_Unlock();
            HAL_Delay(10);
            
        #if (FLASH_BYTE_WRTIE == 8)
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 
                              ADDR_FLAG_HAVE_NEW_FW, 0xAA);
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 
                              ADDR_FLAG_HAVE_NEW_FW + FLASH_BYTE_WRTIE, sSimFwUpdate.CountByteTotal_u32);
        #else
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 
                              ADDR_FLAG_HAVE_NEW_FW, 0xAA);
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 
                              ADDR_FLAG_HAVE_NEW_FW + FLASH_BYTE_WRTIE, sSimFwUpdate.CountByteTotal_u32);
        #endif
            
            HAL_Delay(10);
            HAL_FLASH_Lock();
        #endif
            sMQTT.aMARK_MESS_PENDING[SEND_UPDATE_FIRM_OK] = TRUE;
            //Init Sim again
            *sAppSimVar.pModeConnNow_u8 = *sAppSimVar.pModeConnLast_u8;
            AppSim_Restart_Imediate();
            break;
        case _SIM_COMM_HTTP_UPDATE_FAIL:
            sMQTT.aMARK_MESS_PENDING[SEND_UPDATE_FIRM_FAIL] = TRUE;
            AppSim_Restart_Imediate();
            break;
        case _SIM_COMM_AUDIO_OK:
            //Hoan thanh cuoc goi canh bao
            sAppSimVar.sCall.Mark_u8 = false;
            sAppSimVar.sCall.Pending_u8 = false;
            break;
        default: 
            break;
    }
}

   


/*
    Func: _Cb Get sTime From AT cmd
        + Input: sTime
        + Active Log TSVH for the first Get sTime
*/

void _CbAppSim_Recv_sTime (ST_TIME_FORMAT *sClock)
{    
    if (sAppSimVar.pReq_Set_sTime != NULL)
    {
        sAppSimVar.pReq_Set_sTime(sClock, 7);
    }
}

/*
    Func: _cb Publish AT cmd
        + Set Status Send OK
        + Increase Index send
        + Active Event Send Mess
*/

void _CbAppSim_Recv_PUBACK (void)
{
    sMQTT.Status_u8 = TRUE;   //Set status ve true
        
    switch (sMQTT.MessType_u8)
    {
        case DATA_TSVH_MULTI:
        case DATA_TSVH_FLASH:
        case DATA_TSVH_OPERA:
        #ifdef USING_APP_MEM
            if (AppMem_Inc_Index_Send_2(&sRecTSVH, 1) == false)
            {
                //set lai timeout cho qua trinh luu phan B truoc khi doc tiep ban tin
                sAppMem.cPending_u8 = 0;  
                AppMem_Write_Data(sAppMem.RecMemType_u8, _MEM_DATA_TSVH_B, 0,
                                  sMQTT.sDataFromMem.Data_a8, sMQTT.sDataFromMem.Length_u16, sRecTSVH.SizeRecord_u16);
            }
        #endif
            //Unmark mess 
            mUnmark_Mess_Share_Buff(); 
            break;
        case DATA_EVENT:
        #ifdef USING_APP_MEM
            if (AppMem_Inc_Index_Send_2(&sRecEvent, 1) == false)
            {
                //set lai timeout cho qua trinh luu phan B truoc khi doc tiep ban tin
                sAppMem.cPending_u8 = 0;  
                AppMem_Write_Data(sAppMem.RecMemType_u8, _MEM_DATA_EVENT_B, 0, 
                                  sMQTT.sDataFromMem.Data_a8, sMQTT.sDataFromMem.Length_u16, sRecEvent.SizeRecord_u16);
            }
        #endif
            //Unmark mess 
            mUnmark_Mess_Share_Buff();
            break;
        case DATA_GPS:
        #ifdef USING_APP_MEM
            if (AppMem_Inc_Index_Send_2(&sRecGPS, 1) == false)
            {
                //set lai timeout cho qua trinh luu phan B truoc khi doc tiep ban tin
                sAppMem.cPending_u8 = 0;  
                AppMem_Write_Data(sAppMem.RecMemType_u8, _MEM_DATA_GPS_B, 0 , 
                                  sMQTT.sDataFromMem.Data_a8, sMQTT.sDataFromMem.Length_u16, sRecGPS.SizeRecord_u16);
            }
        #endif
            //Unmark mess 
            mUnmark_Mess_Share_Buff();
            break;
        case SEND_SAVE_BOX_OK:
            sMQTT.aMARK_MESS_PENDING[sMQTT.MessType_u8] = FALSE; 
            
            if (sAppSimVar.pReq_Save_Box != NULL)
                sAppSimVar.pReq_Save_Box();
            break;
        case SEND_SHUTTING_DOWN:
            if (sAppSimVar.pReset_MCU_Imediate != NULL)
                sAppSimVar.pReset_MCU_Imediate();
            break;
        case SEND_UPDATE_FIRM_OK:
        case SEND_UPDATE_FIRM_FAIL:
            sAppSimVar.IsFwUpdateSuccess_u8 = TRUE;          
            sMQTT.aMARK_MESS_PENDING[sMQTT.MessType_u8] = FALSE; 
            
            if (sAppSimVar.pReset_MCU_Imediate != NULL)
                sAppSimVar.pReset_MCU_Imediate();
            break;
        default:
            sMQTT.aMARK_MESS_PENDING[sMQTT.MessType_u8] = FALSE;  
            break;
    }
            
    sAppSimVar.cHardReset_u16 = 0;
    sAppSimVar.cSoftReset_u8 = 0;
    sAppSimVar.LandMarkConnMQTT_u32 = RtCountSystick_u32;
}
    


void AppSim_Process_Server_DL (sData *sUartSim)
{
    uint8_t var = 0;
    int PosFind = 0;
    
    for (var = REQUEST_RESET; var < END_MQTT_RECEI; ++var)
    {
        PosFind = Find_String_V2 ((sData*) &sMark_MessageRecei_Type[var].sKind, sUartSim);
        
        if ((PosFind >= 0) && (sMark_MessageRecei_Type[var].CallBack != NULL))
            sMark_MessageRecei_Type[var].CallBack(sUartSim, PosFind);
    }
}

static void AppSim_Process_Sms (sData *sUartSim)
{
    UTIL_Printf_Str( DBLEVEL_M, "u_app_sim: request at by sms: \r\n" );
    
    if (sAppSimVar.pCheck_AT_User != NULL)
        sAppSimVar.pCheck_AT_User(sUartSim, _AT_REQUEST_SERVER);
}


void _CbAppSim_SMS_Send_1 (void)
{
    //Sdt
    Sim_Common_Send_AT_Cmd( (uint8_t *) "979156872", strlen("979156872") );
}

void _CbAppSim_SMS_Send_2 (void)
{
    //content
    Sim_Common_Send_AT_Cmd( (uint8_t *) "test sms alarm!", strlen("test sms alarm!") );
}

void _CbAppSim_SMS_Send_3 (void)
{
    //Send SMS OK: unmark, pending = false
    sAppSimVar.sSms.Mark_u8 = false;
    sAppSimVar.sSms.Pending_u8 = false;
}


void _CbAppSim_Calling_1 (void)
{
    //send sdt goi
    Sim_Common_Send_AT_Cmd( (uint8_t *) "0979156872", strlen("0979156872") );
//    Sim_Common_Send_AT_Cmd( (uint8_t *) "0966722554", strlen("0966722554") );
}

void _CbAppSim_Calling_2 (void)
{
    //Da thiet lap dc cuoc goi: push lenh
}


/*
    Func: Process at: AT+CIPSEND=cid,length...
        + Send: Cid + length Data
*/

void _CbAppSim_TCP_Send_1 (sData *pData)
{
    char aTEMP[40] = {0};
    
    aTEMP[0] = CID_SERVER; 
    aTEMP[1] = ',';
    
    UtilIntToString(pData->Length_u16, aTEMP + strlen(aTEMP));
    
    Sim_Common_Send_AT_Cmd((uint8_t *) aTEMP, strlen(aTEMP));
}



void _CbAppSim_Http_Head_1 (sData *pData)
{
#ifndef MQTT_PROTOCOL
    char aDATA_TEMP[40] = {0};

    if (sSimCommVar.HttpHeaderIndex_u8 >= HTTP_HEADER_MAX)
    {
        sSimCommVar.HttpHeaderIndex_u8 = 0;
    }
    
    switch (sSimCommVar.HttpHeaderIndex_u8)
    {
        case 0:
            UtilIntToString(pData->Length_u16, aDATA_TEMP);
            Sim_Common_Send_AT_Cmd(&uart_sim, (uint8_t *) "content-length,", 15, 1000); 
            Sim_Common_Send_AT_Cmd(&uart_sim, aDATA_TEMP, strlen(aDATA_TEMP), 1000);
            break;
        case 1:
            Sim_Common_Send_AT_Cmd(&uart_sim, (uint8_t *) "content-type,application/json", strlen("content-type,application/json"), 1000);
//            Sim_Common_Send_AT_Cmd(&uart_sim, (uint8_t *) "connection,keep-alive", strlen("connection,keep-alive"), 1000); 
            break;
        case 2:
            Sim_Common_Send_AT_Cmd(&uart_sim, (uint8_t *) "host,n8n.ilotusland.com", strlen("host,n8n.ilotusland.com"), 1000); 
            break;
        default:
            Sim_Common_Send_AT_Cmd(&uart_sim, (uint8_t *) "content-type,application/json", strlen("content-type,application/json"), 1000); //  "api-key": "123456"
            break;
    }
  
    sSimCommVar.State_u8 = _POWER_CONN_MQTT;    
#endif
}


void _CbAppSim_Http_Head_2 (void)
{
#ifndef MQTT_PROTOCOL
    uint8_t StepSet = 0;;
    uint8_t Stepcurr = 0;
    
    sSimCommVar.HttpHeaderIndex_u8++;
    
    if (sSimCommVar.HttpHeaderIndex_u8 < HTTP_HEADER_MAX)
    {
        Stepcurr = Sim_Get_Step_From_Queue(0);
        
        //Tien them 1 step
        Sim_Get_Step_From_Queue(1);
        
        StepSet = aSIM_STEP_HTTP_HEAD[1];
        qQueue_Send(&qSimStep, (uint8_t *) &StepSet, _TYPE_SEND_TO_HEAD);
        StepSet = aSIM_STEP_HTTP_HEAD[0];
        qQueue_Send(&qSimStep, (uint8_t *) &StepSet, _TYPE_SEND_TO_HEAD);
        //tra lai step cu
        StepSet = Stepcurr;
        qQueue_Send(&qSimStep, (uint8_t *) &StepSet, _TYPE_SEND_TO_HEAD);
    }
#endif
}




void _CbAppSim_Http_Send_1 (sData *pData)
{
#ifndef MQTT_PROTOCOL
    char aDATA_TEMP[40] = {0};
    
    UtilIntToString(pData->Length_u16, aDATA_TEMP);
    
    Sim_Common_Send_AT_Cmd(&uart_sim, aDATA_TEMP, strlen(aDATA_TEMP), 1000);
#endif
}


void _CbAppSim_Http_Send_Ex (sData *pData)
{
#ifndef MQTT_PROTOCOL
    char aDATA_TEMP[40] = {0};
    
    UtilIntToString(pData->Length_u16, aDATA_TEMP);
    
    Sim_Common_Send_AT_Cmd(&uart_sim, aDATA_TEMP, strlen(aDATA_TEMP), 1000);
    Sim_Common_Send_AT_Cmd(&uart_sim, (uint8_t *) ",\"", 2, 1000);
    Sim_Common_Send_AT_Cmd(&uart_sim, pData->Data_a8, pData->Length_u16, 1000);
    Sim_Common_Send_AT_Cmd(&uart_sim, (uint8_t *) "\"", 1, 1000);
#endif
}



static void AppSim_Process_URC (uint8_t Type)
{
    uint8_t StepHandle = 0;
    
    switch (Type)
    {
        case _SIM_COMM_URC_RESET_SIM: 
            //Print power down
            UTIL_Log_Str (DBLEVEL_M, "u_app_sim: URC POWER DOWN\r\n" );
            fevent_active(sEventSim, _EVENT_SIM_AT_SEND_TIMEOUT);
            
            sSimVar.Retry_u8 = SIM_CMD_RETRY;
            break;
        case _SIM_COMM_URC_SIM_LOST: 
            //Sim Remove
            UTIL_Log_Str (DBLEVEL_M, "u_app_sim: URC SIM LOSST\r\n" );
            fevent_active(sEventSim, _EVENT_SIM_AT_SEND_TIMEOUT);
            sSimVar.Retry_u8 = SIM_CMD_RETRY;
            break;
        case _SIM_COMM_URC_CLOSED: 
            //Push lenh dong ket noi va Khoi dong lai
            UTIL_Log_Str (DBLEVEL_M, "u_app_sim: URC TCP CLOSE\r\n" );

            //Trong truong h?p ngat nguon: power off dot ngot: -> k cho no la tcp close
            if (sSimCommVar.State_u8 != _SIM_POWER_OFF) 
            {
                fevent_active(sEventSim, _EVENT_SIM_AT_SEND_TIMEOUT);  
                sSimVar.Retry_u8 = SIM_CMD_RETRY;
            }
            break;
        case _SIM_COMM_URC_CALLING: 
            
            break;
        case _SIM_COMM_URC_CALL_FAIL: 
            StepHandle = Sim_Get_Step_From_Queue(0);      
            if (StepHandle == SIM_STEP_CALLING)
                fevent_active(sEventSim, _EVENT_SIM_AT_SEND_TIMEOUT);
            
            break;
        case _SIM_COMM_URC_SMS_CONTENT: 
            //Check at cmd trong noi dung sms
            AppSim_Process_Sms (&sUartSim);
            break;    
        case _SIM_COMM_URC_SMS_IND: 
            //push at cmd read sms
            Sim_Push_Block_To_Queue(aSIM_BLOCK_STEP_READ_SMS, sizeof(aSIM_BLOCK_STEP_READ_SMS)); 
            break;                     
        case _SIM_COMM_URC_ERROR: 
            if (sSimCommVar.State_u8 != _SIM_POWER_OFF)
            {
                UTIL_Printf_Str ( DBLEVEL_M, "u_app_sim: URC ERROR\r\n" );
                
                StepHandle = Sim_Get_Step_From_Queue(0);
                
                if (SIM_IS_STEP_SKIP_ERROR(StepHandle) == true)
                    fevent_active(sEventSim, _EVENT_SIM_AT_SEND_OK );
                else
                    fevent_active(sEventSim, _EVENT_SIM_AT_SEND_TIMEOUT);
            }
            break;
        case _SIM_COMM_URC_RECEIV_SERVER: 
            AppSim_Process_Server_DL(&sUartSim);
            break;
        default:
            break;
    }
}

/*
    Func: get location value
*/
void AppSim_GPS_OK (uint8_t *pData, uint16_t Length)
{
//#ifdef SIM_EC200U_LIB
//    uint16_t i = 0;
//    double LatTemp = 0;
//    double LongTemp = 0;
//    
//    sAppSimVar.sGPS.MarkFirstError_u8 = false;
//    
//    //Get GPS data
//    
//    //extract lat long to check format
//    if (AppSim_GPS_Extract_Lat_Long (sSimCommVar.sGPS.aPOS_INFOR, sSimCommVar.sGPS.Length_u8, &LatTemp, &LongTemp) == true)
//    {       
//        if ( sAppSimVar.sGPS.IsGetOrigin_u8 == false)
//        {
//            //Lay toa do goc
//            sAppSimVar.sGPS.IsGetOrigin_u8 = true;
//            
//            sAppSimVar.sGPS.sLocaOrigin.Length_u8 = sSimCommVar.sGPS.Length_u8;
//            for (i = 0; i < sAppSimVar.sGPS.sLocaOrigin.Length_u8; i++)
//            {
//                sAppSimVar.sGPS.sLocaOrigin.aData[i] = sSimCommVar.sGPS.aPOS_INFOR[i];
//            }
//            //
//            sAppSimVar.sGPS.sLocaOrigin.Lat = LatTemp;
//            sAppSimVar.sGPS.sLocaOrigin.Long = LongTemp;
//        } else
//        {
//            if (sAppSimVar.sGPS.Status_u8 != false)
//                return;
//            
//            //Lay toa do goc            
//            sAppSimVar.sGPS.sLocation[sAppSimVar.sGPS.Index_u8].Length_u8 = sSimCommVar.sGPS.Length_u8;
//            for (i = 0; i < sAppSimVar.sGPS.sLocation[sAppSimVar.sGPS.Index_u8].Length_u8; i++)
//            {
//                sAppSimVar.sGPS.sLocation[sAppSimVar.sGPS.Index_u8].aData[i] = sSimCommVar.sGPS.aPOS_INFOR[i];
//            }
//            //
//            sAppSimVar.sGPS.sLocation[sAppSimVar.sGPS.Index_u8].Lat = LatTemp;
//            sAppSimVar.sGPS.sLocation[sAppSimVar.sGPS.Index_u8].Long = LongTemp;
//            
//            sAppSimVar.sGPS.Index_u8++;
//            sAppSimVar.sGPS.LandMarkGPSOk_u32 = RtCountSystick_u32;
//                        
//            //Kiem tra xem da lay du so diem (14 diem)
//            if (sAppSimVar.sGPS.Index_u8 >= MAX_SAMPLE_GPS)
//            {
//                sAppSimVar.sGPS.Status_u8 = true;
//            }
//        }
//    } else
//    {
//        //error gps data
//        AppSim_GPS_Error();
//    }
//#endif
}

/*
    Func: tach gia tri lat va long tu chuoi gps nhan dc tu module sim
        + Format: format: (-)dd.ddddd,(-)ddd.ddddd
        + Ex: 
*/
uint8_t AppSim_GPS_Extract_Lat_Long (uint8_t *pData, uint16_t Length, double *Lat, double *Long)
{
#ifdef SIM_EC200U_LIB
    uint16_t    i = 0;
    uint32_t	TempU32 = 0;
    uint8_t 	Decimal = 0;
    uint8_t		MarkDecimal = false;
         
    double 		TempFloat[3] = {0};
    uint8_t 	Index = 0;
    uint8_t 	Sign[3] = {false, false};
    uint8_t     Status[3] = {false, false};

    //Cho them 1 vi tri: de tao so ket thuc
    for (i = 0; i < (Length + 1) ; i++)
    {
        if ( (pData[i] == ',') || (i == Length) )
        {
            //Get Value
            Status[Index] = true; 
            
            if (Sign[Index] == true)
            {
                TempFloat[Index] = 0 - TempU32 / ( pow (10, Decimal) ) ;
            } else
            {
                TempFloat[Index] = TempU32 / ( pow (10, Decimal) ) ;
            }
            //Tang index
            Index++;	
            
            //mem set variable
            Decimal = 0;
            MarkDecimal = false;
            TempU32 = 0;
            
            continue;
        }
        
        //Danh dau so am
        if (pData[i] == '-')
        {
            Sign[Index] = true;	
            continue;
        } 

        //Danh dau vi tri decimal
        if (pData[i] == '.')
        {
            MarkDecimal = true;
            continue;
        }

        //Increase count decimal
        if (MarkDecimal == true)
        {
            Decimal++;
        }
        
        //Value
        if ( (pData[i] < 0x30) || ( pData[i] > 0x39 ) )     
        {
            return false;
        }
        
        TempU32 = TempU32 * 10 + pData[i] - 0x30;
    }
    
    //Kiem tra gia tri lay ra
    if ((Status[0] == true) && (Status[1] == true) )
    {
        *Lat = TempFloat[0];
        *Long = TempFloat[1];
        
        return true;
    }
    
#endif
    
    return false;
}



/*
    Func: Kiem tra khoang cach giua cac diem gps voi diem goc
        + Neu qua 1 nua so diem: co khoang cach >50m
            -> xe dang chay: Copy het gps vao buff -> Push mess
        + Neu < 1 nua so diem: co khoang cach > 50m
            -> xe k di chuyen: tinh trung binh

        return: 0xFF: xe dang di chuyen -> Dong goi toan bo data gps nhuu thu thap
                Vi tri gan nhat voi trung binh: -> dong goi toan bo data la diem gan nhat do
*/


uint8_t AppSim_GPS_Check_Moving (void)
{
#ifdef SIM_EC200U_LIB
    double Distance[20] = {0};
    uint16_t i = 0;
    uint8_t CountMore50m = 0;
    uint8_t CountPosError = 0;
    double DistAverage = 0, TempDist = 0, MinDist = 0;
    uint8_t PosFind = 0;
    
    //Lan luot tinh toan khoang cach
    for (i = 0; i < sAppSimVar.sGPS.Index_u8 ; i++)
    {
        if ( (sAppSimVar.sGPS.sLocation[i].Lat == 0) && (sAppSimVar.sGPS.sLocation[i].Long == 0) )
        {
            Distance[i] = -1;   //Nhung diem error danh dau gia tri dis = -1
            CountPosError++;
            continue;
        }
        
        Distance[i] = 1000 * UTIL_Cacul_Distance(sAppSimVar.sGPS.sLocation[i].Lat, sAppSimVar.sGPS.sLocation[i].Long, 
                                                 sAppSimVar.sGPS.sLocaOrigin.Lat, sAppSimVar.sGPS.sLocaOrigin.Long, 'K'); 
    }
    
    //so sanh
    for (i = 0; i < sAppSimVar.sGPS.Index_u8; i++)
    {
        if (Distance[i] > DISTANCE_GPS_MOVE)
            CountMore50m++;
    }
    
    
    if ( CountMore50m > ( (sAppSimVar.sGPS.Index_u8 - CountPosError) / 2) )   
    {
        //Neu qua nua vi tri > 50m: Xem dang di chuyen
        return 0xFF;   
    } else
    {
        //Neu chua dc 1 nua vi tri > 50m. Xe dang dung yen: 
        // Tinh trung binh va tim vi tri gan nhat de dong goi ban tin
        for (i = 0; i < sAppSimVar.sGPS.Index_u8 ; i++)
        {
            if (Distance[i] == -1)
                continue;
            
            DistAverage += Distance[i];
        }
        DistAverage = DistAverage / (sAppSimVar.sGPS.Index_u8 - CountPosError);
        
        //Tim gia tri gan voi gia tri trung binh    
        MinDist = DistAverage;
        for (i = 0; i < sAppSimVar.sGPS.Index_u8 ; i++)
        {
            if (Distance[i] == -1)
            {
                continue;
            }
            //Tinh khoang c�ch voi gia tri trung binh
            if (DistAverage > Distance[i])
            {
                TempDist = DistAverage - Distance[i];
            } else
            {
                TempDist = Distance[i] - DistAverage;
            }
            
            //Check xem co phai gia tri nho nhat k?
            if (TempDist < MinDist)
            {
                MinDist = TempDist;
                PosFind = i;
            }
        }
        
        return PosFind;
    }
#else
    return 1;
#endif
}

/*
    Func: Dong goi GPS Record to save
*/

void AppSim_GPS_Packet_Record (uint8_t CheckResult)
{
//#ifdef SIM_EC200U_LIB
//    uint16_t  i = 0, Count = 0;
//    
//    Reset_Buff(&sAppSimVar.sDataGPS);
//    //Them Stime vao truoc
//    *(sAppSimVar.sDataGPS.Data_a8 + sAppSimVar.sDataGPS.Length_u16++) = OBIS_TIME_DEVICE;   // sTime
//    *(sAppSimVar.sDataGPS.Data_a8 + sAppSimVar.sDataGPS.Length_u16++) = 0x06;
//    *(sAppSimVar.sDataGPS.Data_a8 + sAppSimVar.sDataGPS.Length_u16++) = sRTC.year;
//    *(sAppSimVar.sDataGPS.Data_a8 + sAppSimVar.sDataGPS.Length_u16++) = sRTC.month;
//    *(sAppSimVar.sDataGPS.Data_a8 + sAppSimVar.sDataGPS.Length_u16++) = sRTC.date;
//    *(sAppSimVar.sDataGPS.Data_a8 + sAppSimVar.sDataGPS.Length_u16++) = sRTC.hour;
//    *(sAppSimVar.sDataGPS.Data_a8 + sAppSimVar.sDataGPS.Length_u16++) = sRTC.min;
//    *(sAppSimVar.sDataGPS.Data_a8 + sAppSimVar.sDataGPS.Length_u16++) = sRTC.sec;
//    
//    if (CheckResult == 0xFF)
//    {          
//        for (i = 0; i < sAppSimVar.sGPS.Index_u8; i++)
//        {
//            //Ngan cach giua cac toa do la dau ;
//            if (i != 0)
//                *(sAppSimVar.sDataGPS.Data_a8 + sAppSimVar.sDataGPS.Length_u16++) = ';';
//            //Data cua tung vi tri
//            for (Count = 0; Count < sAppSimVar.sGPS.sLocation[i].Length_u8; Count++)
//            {
//                *(sAppSimVar.sDataGPS.Data_a8 + sAppSimVar.sDataGPS.Length_u16++) = sAppSimVar.sGPS.sLocation[i].aData[Count];
//            }
//            
//            if (sAppSimVar.sDataGPS.Length_u16 >= 236)
//            {
//                break;
//            }
//        }
//    #ifdef USING_APP_MEM
//        //Send to queue write 
//        AppMem_Write_Data(AppMem_Cacul_Addr_Mess(_MEM_DATA_GPS_A, __MEM_WRITE), sAppSimVar.sDataGPS.Data_a8, sAppSimVar.sDataGPS.Length_u16, sRecGPS.SizeRecord_u16);
//    #endif
//        //Lay lai toa do goc: Toa do cuoi cung (neu di chuyen): tim vi tri toa do khac 0 tu cuoi
//        for (i = sAppSimVar.sGPS.Index_u8; i > 0; i--)
//        {
//            if ( (sAppSimVar.sGPS.sLocation[i - 1].Lat != 0) || (sAppSimVar.sGPS.sLocation[i - 1].Long != 0 ) )
//                break;
//        }
//        
//        sAppSimVar.sGPS.sLocaOrigin.Lat = sAppSimVar.sGPS.sLocation[i - 1].Lat;
//        sAppSimVar.sGPS.sLocaOrigin.Long = sAppSimVar.sGPS.sLocation[i - 1].Long;
//        
//        sAppSimVar.sGPS.sLocaOrigin.Length_u8 = sAppSimVar.sGPS.sLocation[i - 1].Length_u8;
//        
//        for (Count = 0; Count < sAppSimVar.sGPS.sLocaOrigin.Length_u8; Count++)
//            sAppSimVar.sGPS.sLocaOrigin.aData[Count] = sAppSimVar.sGPS.sLocation[i - 1].aData[Count];
//    } else
//    {
//        //Dong goi tat ca gps l� toa do gan nhat
//        for (i = 0; i < sAppSimVar.sGPS.Index_u8; i++)
//        {
//            //Ngan cach giua cac toa do la dau ;
//            if (i != 0)
//                *(sAppSimVar.sDataGPS.Data_a8 + sAppSimVar.sDataGPS.Length_u16++) = ';';
//            
//            if ( (sAppSimVar.sGPS.sLocation[i].Lat == 0) && (sAppSimVar.sGPS.sLocation[i].Long == 0) )
//            {
//                //Data tai vi tri mat ket noi van giu nguyen
//                for (Count = 0; Count < sAppSimVar.sGPS.sLocation[i].Length_u8; Count++)
//                    *(sAppSimVar.sDataGPS.Data_a8 + sAppSimVar.sDataGPS.Length_u16++) = sAppSimVar.sGPS.sLocation[i].aData[Count];
//            } else
//            {
//                //Data cua tung vi tri
////                for (Count = 0; Count < sAppSimVar.sGPS.sLocation[CheckResult].Length_u8; Count++)
////                    *(sAppSimVar.sDataGPS.Data_a8 + sAppSimVar.sDataGPS.Length_u16++) = sAppSimVar.sGPS.sLocation[CheckResult].aData[Count];
//                
//                for (Count = 0; Count < sAppSimVar.sGPS.sLocaOrigin.Length_u8; Count++)
//                    *(sAppSimVar.sDataGPS.Data_a8 + sAppSimVar.sDataGPS.Length_u16++) = sAppSimVar.sGPS.sLocaOrigin.aData[Count];
//            }
//            
//            if (sAppSimVar.sDataGPS.Length_u16 >= 236)
//            {
//                break;
//            }
//        }
//    #ifdef USING_APP_MEM
//        //Send to queue write 
//        AppMem_Write_Data(AppMem_Cacul_Addr_Mess(_MEM_DATA_GPS_A, __MEM_WRITE), sAppSimVar.sDataGPS.Data_a8, sAppSimVar.sDataGPS.Length_u16, sRecGPS.SizeRecord_u16);
//    #endif
////        //Lay lai toa do goc: toa do diem gan nhat (neu dung yen)
////        sAppSimVar.sGPS.sLocaOrigin.Lat = sAppSimVar.sGPS.sLocation[CheckResult].Lat;
////        sAppSimVar.sGPS.sLocaOrigin.Long = sAppSimVar.sGPS.sLocation[CheckResult].Long;
//    }    
//#endif
}

/*
    Func: callback gps error  : Packet lat and long 0,0
        - Neu lan tiep theo van loi: k lay du lieu 0,0
        - util next gps ok:
*/
void AppSim_GPS_Error (void)
{
#ifdef SIM_EC200U_LIB
    if (sAppSimVar.sGPS.MarkFirstError_u8 == false)
    {
        sAppSimVar.sGPS.CountError_u16 = 0;
        //
        sAppSimVar.sGPS.MarkFirstError_u8 = true;
        
        sAppSimVar.sGPS.sLocation[sAppSimVar.sGPS.Index_u8].aData[0] = '0';
        sAppSimVar.sGPS.sLocation[sAppSimVar.sGPS.Index_u8].aData[1] = ',';
        sAppSimVar.sGPS.sLocation[sAppSimVar.sGPS.Index_u8].aData[2] = '0';
        
        sAppSimVar.sGPS.sLocation[sAppSimVar.sGPS.Index_u8].Length_u8 = 3;
        
        sAppSimVar.sGPS.sLocation[sAppSimVar.sGPS.Index_u8].Lat = 0;
        sAppSimVar.sGPS.sLocation[sAppSimVar.sGPS.Index_u8].Long = 0;
        
        sAppSimVar.sGPS.Index_u8++;
            
        //Kiem tra xem da lay du so diem (14 diem)
        if (sAppSimVar.sGPS.Index_u8 >= MAX_SAMPLE_GPS)
        {
            sAppSimVar.sGPS.Status_u8 = true;
        }
    } else
    {
        sAppSimVar.sGPS.CountError_u16++;
        
        //Cu moi 2p: 2*60/5 : push at power on again
        if ( (sAppSimVar.sGPS.CountError_u16 % 24) == 0 )
        {
            Sim_Push_Block_To_Queue(aEC200_BLOCK_GNSS_ON, sizeof(aEC200_BLOCK_GNSS_ON));   
        }
        
        //10phut: 600/5 = 120 Power on moudle again (power on gps?)
        if (sAppSimVar.sGPS.CountError_u16 >= 120)
        {
            UTIL_Printf_Str( DBLEVEL_M, "u_app_sim: gps error -> restart sim\r\n");
            
            sAppSimVar.sGPS.CountError_u16 = 0;
            
            //Khoi dong lai module sim
            AppSim_Restart_Imediate();
        }
    }
#endif
}

void AppSim_Config_Update (void)  
{
    Erase_Firmware(ADDR_UPDATE_PROGRAM, FIRMWARE_SIZE_PAGE); 
    sSimFwUpdate.AddSave_u32 = ADDR_UPDATE_PROGRAM;
}


/*
    Func: Restart module sim neu nhu sim dang o che do PSM
*/
void AppSim_Restart_If_PSM(void)
{
    //Check them thoi gian hoạt dong
    if ( ( sSimCommVar.State_u8 == _SIM_POWER_OFF )
        || (Check_Time_Out(sSimVar.LandMark_u32, 120000) == true) )
    {
        sSimVar.LandMark_u32 = RtCountSystick_u32;
        sSimCommVar.State_u8  = _SIM_START;
        fevent_active(sEventAppSim, _EVENT_SIM_TURN_ON);
    }
}

/*
    Func: Restart Module sim send data
*/
void AppSim_Restart_Imediate (void)
{
    sSimCommVar.State_u8 = _SIM_START;
    fevent_active(sEventAppSim, _EVENT_SIM_TURN_ON);
}




/*
    Func:Power ON module SIM
        + Giu chan Boot va Reset o muc Thap
        + Cap nguon cho module bang chan POW
*/

uint8_t AppSim_Power_On(void)
{   
	switch (sAppSimVar.StepPowerOn_u8)
	{
		case 0:      
            UTIL_Log_Str(DBLEVEL_M, "u_app_sim: power on...\r\n" );  
            //Reinit uart
            Sim_Reinit_Uart();
            
            SIM_PW_OFF1;
            
            UTIL_TIMER_SetPeriod(&TimerPowerOn, ((sAppSimVar.cHardReset_u16 % 10) * 1000) + 3000);
            UTIL_TIMER_Start(&TimerPowerOn);
			break;
        case 1:   
            SIM_PWKEY_OFF1;
            SIM_RESET_OFF1;
            SIM_PW_ON1;

			UTIL_TIMER_SetPeriod(&TimerPowerOn, 1000);
            UTIL_TIMER_Start(&TimerPowerOn);
			break;
        case 2:      
            SIM_PWKEY_ON1;
        #ifdef SIM_L506_LIB
			UTIL_TIMER_SetPeriod(&TimerPowerOn, 500);
            UTIL_TIMER_Start(&TimerPowerOn);
        #endif
            
        #ifdef SIM_EC200U_LIB
            UTIL_TIMER_SetPeriod(&TimerPowerOn, 3000);
            UTIL_TIMER_Start(&TimerPowerOn);
        #endif
            
        #ifdef SIM_L511_LIB
            UTIL_TIMER_SetPeriod(&TimerPowerOn, 1500);
            UTIL_TIMER_Start(&TimerPowerOn);
        #endif
			break;
        case 3:      
            SIM_PWKEY_OFF1;
        #ifdef SIM_L506_LIB
			UTIL_TIMER_SetPeriod(&TimerPowerOn, 15000);
            UTIL_TIMER_Start(&TimerPowerOn);
        #endif
            
        #ifdef SIM_EC200U_LIB
            UTIL_TIMER_SetPeriod(&TimerPowerOn, 5000);
            UTIL_TIMER_Start(&TimerPowerOn);
        #endif
            
        #ifdef SIM_L511_LIB
            UTIL_TIMER_SetPeriod(&TimerPowerOn, 10000);
            UTIL_TIMER_Start(&TimerPowerOn);
        #endif
			break;
		default:
			sAppSimVar.StepPowerOn_u8 = 0;
            UTIL_Printf_Str( DBLEVEL_M, "\r\nu_appsim: power on success!\r\n" );
			return 1;
	}

	sAppSimVar.StepPowerOn_u8++;

	return 0;
}

static void OnTimerPowerOn(void *context)
{
	fevent_active(sEventAppSim, _EVENT_SIM_TURN_ON);
}






static void AppSim_AT_Faillure (uint8_t Step)
{
    //Neu dang poweron module thi bo qua
    if (sAppSimVar.StepPowerOn_u8 != 0)
        return;

    Sim_Reinit_Uart();
    //Tang soft Reset
	sAppSimVar.cSoftReset_u8++;
    
	Sim_Clear();
    
    if ((*sAppSimVar.pModeConnNow_u8 != MODE_CONNECT_DATA_MAIN) 
        && (*sAppSimVar.pModeConnNow_u8 != MODE_CONNECT_DATA_BACKUP))
    {
        sAppSimVar.cSoftReset_u8 = MAX_SOFT_RESET;
    }
	//Neu Soft Reset > 2
	if (sAppSimVar.cSoftReset_u8 < MAX_SOFT_RESET)
	{        
		switch (sSimCommVar.State_u8)
		{
			case _SIM_INIT:
			case _SIM_NETWORK:
			case _SIM_SETTING:
				Sim_Push_Block_To_Queue(aSIM_BLOCK_STEP_INIT, sizeof(aSIM_BLOCK_STEP_INIT));
                Sim_Push_Block_To_Queue(aSIM_BLOCK_STEP_NETWORK, sizeof(aSIM_BLOCK_STEP_NETWORK));
                Sim_Push_Block_To_Queue(aSIM_BLOCK_STEP_CONNECT, sizeof(aSIM_BLOCK_STEP_CONNECT));
                sSimCommVar.State_u8 = _SIM_INIT; 
				break;
			case _SIM_INIT_TCP:
                Sim_Push_Block_To_Queue(aSIM_BLOCK_STEP_INIT, sizeof(aSIM_BLOCK_STEP_INIT));
                Sim_Push_Block_To_Queue(aSIM_BLOCK_STEP_NETWORK, sizeof(aSIM_BLOCK_STEP_NETWORK));
                Sim_Push_Block_To_Queue(aSIM_BLOCK_STEP_CONNECT, sizeof(aSIM_BLOCK_STEP_CONNECT));
                sSimCommVar.State_u8 = _SIM_INIT; 
                break;
			case _SIM_OPENED_TCP:
            case _SIM_CONN_MQTT:
				Sim_Push_Block_To_Queue(aSIM_BLOCK_STEP_DIS_CONN, sizeof(aSIM_BLOCK_STEP_DIS_CONN));
                Sim_Push_Block_To_Queue(aSIM_BLOCK_STEP_CONNECT, sizeof(aSIM_BLOCK_STEP_CONNECT));
                sSimCommVar.State_u8 = _SIM_INIT_TCP; 
				break;
            case _SIM_CONN_HTTP:
                Sim_Clear();
                fevent_active(sEventAppSim, _EVENT_SIM_TURN_ON); 
                sSimCommVar.State_u8 = _SIM_INIT; 
                break;
			default:
				break;
		}
	} else
	{
        
        sAppSimVar.cSoftReset_u8 = 0;
        sAppSimVar.cHardReset_u16++;
        //Neu dang Case Online ->Pow on lai | Neu case Save Mode -> Pow off module sim
        sSimCommVar.State_u8 = _SIM_INIT; 
        
        if (*sAppSimVar.pModePower_u8 == _POWER_MODE_SAVE)
        {
            fevent_active(sEventAppSim, _EVENT_SIM_POWER_OFF);
        } else
        {
            fevent_active(sEventAppSim, _EVENT_SIM_TURN_ON);
        }
	}
}



static void AppSim_Uart_Send (uint8_t *pData, uint16_t length)
{
    HAL_UART_Transmit(&uart_sim, pData, length, 2000);
}



#ifdef USING_AT_CONFIG

void AppSim_SER_Get_Sim_ID (sData *str, uint16_t Pos)
{
    if ((sAppSimVar.pRespond != NULL) && (sSimCommInfor.sSIMid.Length_u16 != 0))
    {
        sAppSimVar.pRespond(PortConfig, sSimCommInfor.sSIMid.Data_a8, sSimCommInfor.sSIMid.Length_u16, 0);
    } else
    {
        sAppSimVar.pRespond(PortConfig, (uint8_t *) "\r\nERROR\r\n", strlen("\r\nERROR\r\n"), 0);
    }
}

#endif


void AppSim_Set_Instant (void)
{
    sAppSimVar.rExternIrq_u8 = true;
}


void AppSim_Allow_Send(uint8_t allow)
{
    sSimVar.IsPause_u8 = allow;
}

void AppSim_Default_Sms_Call(void)
{
    sAppSimVar.sSms.Pending_u8 = false;
    sAppSimVar.sCall.Pending_u8 = false;
}


uint8_t AppSim_Need_Sms (void)
{
    if (sAppSimVar.sSms.Mark_u8 == true)
        return true;
    
    return false;
}


void AppSim_Push_Send_Sms (void)
{
    //check xem co ban tin nao can gui
    sAppSimVar.sSms.Pending_u8 = true;
    sAppSimVar.sSms.Type_u8 = 1;
    //Push block send sms
    Sim_Push_Block_To_Queue(aSIM_BLOCK_STEP_SEND_SMS, sizeof(aSIM_BLOCK_STEP_SEND_SMS));
}


void AppSim_Push_Call (void)
{
    //check xem co ban tin nao can gui
    sAppSimVar.sCall.Pending_u8 = true;
    //Push block send sms
    Sim_Push_Block_To_Queue(aSIM_BLOCK_STEP_CALL, sizeof(aSIM_BLOCK_STEP_CALL));
}

sData* AppSim_Get_Sim_ID (void)
{
    return &sSimCommInfor.sSIMid;
}

