/*
 * user_app.c
 *
 *  Created on: Dec 14, 2021
 *      Author: Chien
 */

#include "user_app_comm.h"
#include "user_define.h"

#include "user_uart.h"

#include "user_timer.h"
#include "user_lpm.h"

#include "gpio.h"
//#include "i2c.h"
//#include "adc.h"
     

/*==========Static func====================*/
static uint8_t _Cb_Timer_IRQ(uint8_t event);
static uint8_t _Cb_Uart_Rx_Debug(uint8_t event);
static uint8_t _Cb_Uart_Error(uint8_t event);

static uint8_t _Cb_HandLer_IDLE(uint8_t event);
static uint8_t _Cb_Tx_Timer (uint8_t event);
static uint8_t _Cb_Set_RTC (uint8_t event);
static void _Cb_TX_Timer_Event(void *context);
static uint8_t _Cb_Save_Box (uint8_t event);

//static uint32_t  AppCom_Calcu_Period_To_RoudTime (uint32_t FreqWakeup);

static void AppComm_Uart_Debug_Send (uint8_t *pData, uint16_t Length);


/*=================Var struct======================*/
sEvent_struct sEventAppComm[] =
{
	{ _EVENT_TIMMER_IRQ, 		    0, 0, 0, 	    _Cb_Timer_IRQ },
	{ _EVENT_UART_RX_DEBUG, 	    0, 0, 20, 	    _Cb_Uart_Rx_Debug },
    { _EVENT_UART_ERROR, 	        0, 0, 20, 	    _Cb_Uart_Error },
    { _EVENT_SET_RTC, 				0, 0, 0, 	    _Cb_Set_RTC },
	{ _EVENT_IDLE, 					1, 0, 1000,     _Cb_HandLer_IDLE },
    { _EVENT_TX_TIMER,		        0, 0, 0, 	    _Cb_Tx_Timer },
    //Event Log
    { _EVENT_SAVE_BOX,		        0, 0, 0, 	    _Cb_Save_Box },  
};

/*===============================================*/
char aSaoVietCom[15][71] =
{
    {"=====================================================================\r\n"},
    {"*                                                                    \r\n"},
    {"*          * * *         *                 *         * * *           \r\n"},
    {"*        *       *        *               *        *       *         \r\n"},
    {"*        *                 *             *        *                  \r\n"},
    {"*         *                 *           *        *                   \r\n"},
    {"*           *                *         *         *                   \r\n"},
    {"*             *               *       *          *                   \r\n"},
    {"*               *              *     *           *                   \r\n"},
    {"*       *        *              *   *             *                  \r\n"},
    {"*        *       *               * *               *       *         \r\n"},
    {"*          * * *                  *           *      * * *           \r\n"},
    {"*                                                                    \r\n"},
    {"*                                                                    \r\n"},
    {"=====================================================================\r\n"},
};


sData   sFirmVersion = {(uint8_t *) "SVTH_SVM_DLS_V1_2_2", 19}; 

static UTIL_TIMER_Object_t TimerTx;

uint32_t TimeTaskProcess = 0;
volatile uint8_t SourceWakeup_u8 = 0;


static uint8_t aUART_DBG_DMA_RX [100];

uint8_t UartDebugBuff[1200] = {0};
sData 	sUartDebug = {&UartDebugBuff[0], 0};

uint8_t UartDebugRxByte = 0;
static uint16_t OldPosDMADebug = 0;

//Trace
static void (*TxCpltCallback)(void *);

const UTIL_ADV_TRACE_Driver_s UTIL_TraceDriver =
{
    vcom_Init,
    vcom_DeInit,
    //  vcom_ReceiveInit,
    vcom_Trace_DMA,
};


/*======================Function========================*/
/*====== Func Call Event ==========*/
static uint8_t _Cb_Timer_IRQ(uint8_t event)
{
	UTIL_TIMER_IRQ_Handler();

	return 1;
}

/*
    Khi co su kien:
        + Start: lay moc thoi gian systick
                 Lay moc length hien tai
        + cu 5ms vao check:
                ++ Neu length bang voi length truoc: Nhan xong
                ++ Neu length khac length truoc: thuc hien nhu buoc start
                                                 tiep tuc enable cho check tiep
*/
static uint8_t _Cb_Uart_Rx_Debug(uint8_t event)
{
    static uint8_t  MarkFirstRecvUart = 0;
    static uint16_t LastLengthRecv = 0; 
        
    if (MarkFirstRecvUart == 0)
    {
        MarkFirstRecvUart = 1;
        LastLengthRecv = sUartDebug.Length_u16;
        fevent_enable(sEventAppComm, event);
    } else
    {
        if (sUartDebug.Length_u16 == LastLengthRecv)
        {
            MarkFirstRecvUart = 0;
            UTIL_Printf( DBLEVEL_L, sUartDebug.Data_a8, sUartDebug.Length_u16 );
            UTIL_Printf_Str( DBLEVEL_L,"\r\n");
            Modem_Check_AT(&sUartDebug, _AT_REQUEST_SERIAL);
            Reset_Buff(&sUartDebug);
        } else
        {
            LastLengthRecv = sUartDebug.Length_u16;
            fevent_enable(sEventAppComm, event);
        }
    }
    
//    sModemVar.CountConfig_u8 = 0;  //Cho phep giu 20s config

	return 1;
}

/*
    Func: Event Uart Error: 
        + DMA khi error mcu se disable IRQ, nen can init lai
        + Int: Khong bi disable, nen k can init la
*/
static uint8_t _Cb_Uart_Error(uint8_t event)
{
    ReInit_Rx_Mode_Uart_Error();
    
    return 1;
}



static uint8_t _Cb_Set_RTC(uint8_t event)
{
    UTIL_Log_Str(DBLEVEL_M, "u_app_comm: set rtc!\r\n");
    
//    UTIL_Set_RTC(sModemVar.sRTCSet);

    fevent_enable(sEventAppComm, _EVENT_IDLE);

    return 1;
}

static uint8_t _Cb_HandLer_IDLE(uint8_t event)
{
//    static uint8_t MarkGetStime_u8 = 0;
//    static uint8_t MarkResetDaily = 0;
//    static uint32_t LandMarkWaitGPSEmpty = 0;
//    uint8_t IsAllowResetMcu = true;
//    
//	Get_RTC();
//   
//    //Check True Update firmware -> wait Pulse increase to reset.
//#ifdef USING_APP_WM
//    if ( (sPulse[0].IsOver5Pulse_u8 == TRUE) && (sPulse[1].IsOver5Pulse_u8 == TRUE) )
//#endif
//    {
//    #ifdef USING_APP_SIM
//        if (sAppSimVar.IsFwUpdateSuccess_u8 == true)
//        {
//            Modem_Reset_MCU();
//        }
//    #endif
//    }
//    
//    //Check Request Stime To Server
//    if ((sRTC.hour == 0) && (MarkGetStime_u8 == 0))
//    {
//        MarkGetStime_u8 = 1;
//        sMQTT.aMARK_MESS_PENDING[SEND_SERVER_TIME_PENDING] = TRUE;
//    } else if (sRTC.hour != 0)
//    {
//        MarkGetStime_u8 = 0;
//    }
//
//    
//#ifdef USING_APP_SIM
//    //Neu connect mqtt ma van k lay dc thoi gian thuc -> request stime to server
//    if ( (sRTC.year < 20) && (sSimCommVar.State_u8 == _SIM_CONN_MQTT) )
//    {
//        sMQTT.aMARK_MESS_PENDING[SEND_SERVER_TIME_PENDING] = TRUE;
//    }
//#endif
//    
//#ifdef USING_APP_ETHERNET
//    if ( (sRTC.year < 20) && (sAppEthVar.Status_u8 == _ETH_MQTT_CONNECTED) )
//    {
//        sMQTT.aMARK_MESS_PENDING[SEND_SERVER_TIME_PENDING] = TRUE;
//    }
//#endif
//    
//#ifdef USING_APP_MEM
//    if (AppMem_Queue_Write_Empty() == true) 
//    {
//    #ifdef USING_APP_SIM
//        if (sAppSimVar.sGPS.Index_u8 > 1)
//        {
//            if (Check_Time_Out(LandMarkWaitGPSEmpty, 600000) == false)
//            {
//                IsAllowResetMcu = false;
//            }
//        } else
//        {
//            LandMarkWaitGPSEmpty = RtCountSystick_u32;
//        }
//    #endif
//    }   
//#endif
//    
//#ifndef BOARD_READ_PULSE
//    //Check reset daily: neu dang lay do gps-> doi xong ms reset
//    if (sRTC.hour != 14)
//    {
//        MarkResetDaily = 1;
//    }
//    
//    if ( (sRTC.hour == 14) && (sRTC.min == 5) 
//        && (IsAllowResetMcu == true) && (MarkResetDaily == 1) )
//    {
//        //Reset thiet bi
//        Modem_Reset_MCU();
//    }  
//#endif
//    
//#ifdef ENABLE_TEST_HARDWARE
//    static uint32_t LandMarkTestHardware = 0;
//    
//    if (Check_Time_Out(LandMarkTestHardware, 5000) == true)
//    {
//        LandMarkTestHardware = RtCountSystick_u32;
//        
//        APP_LOG(TS_OFF, DBLEVEL_M, "\r\n\r\ntest_hardware: realtime:%d/%d/%d %d:%d:%d\r\n", 
//                                    sRTC.year, sRTC.month, sRTC.date, sRTC.hour, sRTC.min, sRTC.sec );
//        
//    #ifdef LPTIM_ENCODER_MODE 
//        LPTIM_Encoder_Get_Pulse(&hlptim1, &sPulse[0]);
//    #else   
//        LPTIM_Counter_Get_Pulse(&hlptim1, &sPulse[0]);
//        AppWm_Plus_5Pulse(&sPulse[0]);
//    #endif
//            
//    #ifdef LPTIM2_ENCODER_MODE 
//        LPTIM_Encoder_Get_Pulse(&hlptim2, &sPulse[1]);
//    #else   
//        LPTIM_Counter_Get_Pulse(&hlptim2, &sPulse[1]);
//        AppWm_Plus_5Pulse(&sPulse[1]);
//    #endif
//          
//        TIM_Counter_Get_Pulse(&htim1, &sPulse[2]);  
//        TIM_Counter_Get_Pulse(&htim2, &sPulse[3]);
//            
//        APP_LOG(TS_OFF, DBLEVEL_M, "test_hardware: pulse: %d, %d, %d, %d \r\n", 
//                                    sPulse[0].Number_u32, sPulse[1].Number_u32, sPulse[2].Number_u32, sPulse[3].Number_u32 );
//       
//        APP_LOG(TS_OFF, DBLEVEL_M, "test_hardware: pressure: \r\n");
//                    
//        for (uint8_t i = 0; i < MAX_PRESSURE_INPUT; i++)
//        {
//        #ifdef PRESSURE_DECODE
//            APP_LOG(TS_OFF, DBLEVEL_M, "Pressủe Chan.%d: %d mbar\r\n", i, sWmVar.aPRESSURE[i].Val_i32 );
//        #else
//            APP_LOG(TS_OFF, DBLEVEL_M, "Pressủe Chan.%d: %d mV\r\n", i, sWmVar.aPRESSURE[i].Val_i32);
//        #endif
//        }
//        
//        APP_LOG(TS_OFF, DBLEVEL_M, "test_hardware: input: ");
//
//        for (uint8_t i = 0; i < MAX_PORT_INPUT; i++)
//        {
//            APP_LOG(TS_OFF, DBLEVEL_M, " %d, ", sWmVar.aINPUT_VAL[i]); 
//        }
//        
//        //Test memory
//        if (AppMem_Rec_Status() == true)
//        {
//            UTIL_Printf_Str(DBLEVEL_M, "\r\ntest_hardware: EEPROM OK!\r\n" );
//        } else
//        {
//            UTIL_Printf_Str(DBLEVEL_M, "\r\ntest_hardware: EEPROM FAIL!\r\n" );
//        }
//        
//        //Test module sim
//        if (sSimCommVar.State_u8 == _SIM_CONN_MQTT)
//        {
//            UTIL_Printf_Str(DBLEVEL_M, "test_hardware: sim connected mqtt OK!\r\n" );
//        } else
//        {
//            UTIL_Printf_Str(DBLEVEL_M, "test_hardware: sim connect mqtt FAIL!\r\n" );
//        }
//        
//        //Test module ethernet
//        if (sAppEthVar.Status_u8 >= _ETH_MQTT_CONNECTED)
//        {
//            UTIL_Printf_Str(DBLEVEL_M, "test_hardware: ethernet connected mqtt OK!\r\n" );
//        } else if (sAppEthVar.Status_u8 >  _ETH_CHECK_SPI)
//        {
//            UTIL_Printf_Str(DBLEVEL_M, "test_hardware: ethernet SPI OK!\r\n" );
//        } else
//        {
//            UTIL_Printf_Str(DBLEVEL_M, "test_hardware: ethernet SPI FAIL!\r\n\r\n" );
//        }
//        
//        //Gui ra logic cua cac chan INPUT: BUTTON, DIR1,2, DTC 1 2
//        APP_LOG(TS_OFF, DBLEVEL_M, "Logic Button: %d\r\n", (uint8_t ) HAL_GPIO_ReadPin(BUTTON_1_GPIO_Port, BUTTON_1_Pin)); 
//        APP_LOG(TS_OFF, DBLEVEL_M, "Logic DIR1: %d\r\n", (uint8_t ) HAL_GPIO_ReadPin(PULSE1_DIR_GPIO_Port, PULSE1_DIR_Pin)); 
//        APP_LOG(TS_OFF, DBLEVEL_M, "Logic DTC1: %d\r\n", (uint8_t ) HAL_GPIO_ReadPin(PULSE1_DTC_GPIO_Port, PULSE1_DTC_Pin)); 
//        
//        APP_LOG(TS_OFF, DBLEVEL_M, "Logic DIR2: %d\r\n", (uint8_t ) HAL_GPIO_ReadPin(PULSE2_DIR_GPIO_Port, PULSE2_DIR_Pin)); 
//        APP_LOG(TS_OFF, DBLEVEL_M, "Logic DTC2: %d\r\n", (uint8_t ) HAL_GPIO_ReadPin(PULSE2_DTC_GPIO_Port, PULSE2_DTC_Pin)); 
//        
//        //Task Process
//        APP_LOG(TS_OFF, DBLEVEL_M, "test_hardware: max time task process = %d \r\n", TimeTaskProcess); 
//    }
//    
//#endif
//    
//    Modem_Monitor_Connect_Server();
    
	return 1;
}



static uint8_t _Cb_Tx_Timer(uint8_t event)
{
//#ifdef USING_APP_WM
//    //Active Event Check Alarm
//    sWmVar.IrqPowUp_u8 = pending;
//    fevent_active(sEventAppWM, _EVENT_ENTRY_WM);
//#endif
//    
//#ifdef USING_APP_EMET
//    //Active Event Check Alarm
//    fevent_active(sEventAppEmet, _EVENT_TEMH_LOG_TSVH);
//    fevent_active(sEventAppEmet, _EVENT_SCAN_ALARM);
//#endif
//
//#ifdef USING_APP_TEMH
//    //Active Check Mode
//    sTempHumi.IRQMainPowerDetect_u8 = pending;             //avoid sleep
//    fevent_active(sEventAppTempH, _EVENT_CHECK_AC_POWER);
//    fevent_active(sEventAppTempH, _EVENT_TEMH_ENTRY);
//#endif 
//        
//    if (sModemInfor.ModePower_u8 == _POWER_MODE_SAVE)
//    {
//        //Increase Count To Send Mess
//        sModemVar.CountSleep_u8++;
//        if (sModemVar.CountSleep_u8 >= sModemInfor.sFrequence.NumWakeup_u8)
//        {
//            sModemVar.CountSleep_u8 = 0;
//        #ifdef USING_APP_ETHERNET   
//            AppEth_Restart_If_PSM();
//        #endif
//            
//        #ifdef USING_APP_SIM
//            fevent_active(sEventAppSim, _EVENT_SIM_REQ_GPS);
//            AppSim_Restart_If_PSM();
//        #endif
//        }
//    } else
//    {
//        sModemVar.CountSleep_u8 = 0;
//
//    #ifdef USING_APP_ETHERNET    
//        AppEth_Restart_If_PSM();
//    #endif
//        
//    #ifdef USING_APP_SIM
//        fevent_active(sEventAppSim, _EVENT_SIM_REQ_GPS);
//        AppSim_Restart_If_PSM();
//    #endif
//    }
         
    return 1;
}


static uint8_t _Cb_Save_Box (uint8_t event)
{
    
   return 1;
}




/*=========================== Func App Main ========================*/
void SysApp_Init (void)
{       
#ifdef USING_APP_EMET
    __HAL_IWDG_START(&hiwdg);
#endif

#ifdef USING_APP_WM
    LPTIM_Init_Feature();
#endif
       
#ifdef USING_APP_LORA
    AppLora_Init ();
#else
    //Init timer: Init RTC...cau h�nh alarm profile
	UTIL_TIMER_Init();
#endif
    
	//Get RTC
	Get_RTC();
    
    /*Init low power manager*/
//    UTIL_LPM_Init();
}
 

void SysApp_Setting (void)
{
    AppComm_Init();
    
#ifdef USING_APP_MEM
    AppMem_Init();
#endif
    
#ifdef USING_APP_SIM
    AppSim_Init(); 
#endif
   
#ifdef USING_APP_WM
    AppWm_Init();
#endif
    
#ifdef USING_APP_RS485
    Init_AppRs485();
#endif
    
#ifdef USING_APP_SENSOR
    Init_AppSensor();
#endif
    
#ifdef USING_APP_TEMH
    AppTemH_Init();
#endif
    
#ifdef USING_APP_ETHERNET
    AppEth_Init();
#endif
    
#ifdef USING_LCD_DISPLAY
    Display_Init();
#endif
    
#ifdef USING_APP_SENSOR
    Init_AppSensor();
#endif
    
    //Dang ki cac event
	UTIL_TIMER_Create(&TimerTx,  0xFFFFFFFFU, UTIL_TIMER_ONESHOT, _Cb_TX_Timer_Event, NULL);
	UTIL_TIMER_SetPeriod (&TimerTx, 60000*4);   //Set sFreqInfor.UnitMin_u16 * 60000
	UTIL_TIMER_Start (&TimerTx);
}

/*
    Note:
    - khi gen lai
        + Phai sua lai chan uart 2 thanh Pull up o trong code
        + Comment fun init in rtc

*/

/*
    Func: Init Queue trong main
*/
//uint8_t TaskStatus_u8 = 0;
void Main_Task (void)
{
    uint8_t TaskStatus_u8 = 0;
#ifdef ENABLE_TEST_HARDWARE
    uint32_t LandMarkProcessTask = 0;
#endif
    
    SysApp_Init();
    SysApp_Setting();
    
    UTIL_Printf_Str (DBLEVEL_M, "\r\n=========================\r\n" );
    UTIL_Printf_Str (DBLEVEL_M,     "==== MAIN PROGRAMING ====\r\n" );
    UTIL_Printf_Str (DBLEVEL_M,     "=========================\r\n" );
    
//    APP_LOG(TS_OFF, DBLEVEL_M, "SVWM VERSION: \"%s\"\r\n", (char *) sFirmVersion.Data_a8 );

    if (AppComm_Get_Reset_Source () == TRUE)
    {
        HAL_Delay(2000); 
    }
    
	for (;;)
	{
		TaskStatus_u8 = 0;
    #ifdef ENABLE_TEST_HARDWARE
        LandMarkProcessTask = RtCountSystick_u32;
    #endif
		TaskStatus_u8 |= AppComm_Task();
        
    #ifdef USING_APP_RS485
        TaskStatus_u8 |= AppRs485_Task(); 
    #endif
        
    #ifdef USING_APP_SENSOR
        TaskStatus_u8 |= AppSensor_Task(); 
    #endif
        
    #ifdef USING_LCD_DISPLAY
        Display_Task();
    #endif
        
//        
//    #ifdef USING_APP_MEM
//    #ifdef USING_APP_SIM
//        if ( sSimCommVar.State_u8 == _SIM_CONN_MQTT) 
//        {
//            if (Check_Time_Out(sSimVar.LandMark_u32, 800) == true)
//                TaskStatus_u8 |= AppMem_Task();
//        } else
//            TaskStatus_u8 |= AppMem_Task();
//    #else
//        TaskStatus_u8 |= AppMem_Task();
//    #endif
//    #endif
//        
//    #ifdef USING_APP_SIM
//        TaskStatus_u8 |= AppSim_Task();
//        TaskStatus_u8 |= Sim_Task();
//    #endif
//      
//    #ifdef USING_APP_TEMH
//        TaskStatus_u8 |= AppTemH_Task();
//    #endif
//            
//    #ifdef USING_APP_WM
//        TaskStatus_u8 |= AppWm_Task(); 
//    #endif
//        
//    #ifdef USING_APP_RS485
//        TaskStatus_u8 |= AppRs485_Task(); 
//    #endif
//        
//    #ifdef USING_APP_SENSOR
//        TaskStatus_u8 |= AppSensor_Task(); 
//    #endif
//        
//    #ifdef USING_APP_EMET
//        Button_Task();
//        Screen_Task();
//        EMeasure_Task();
//        Relay_Task();
//        LED_Task();
//        Temp_Humid_Task();
//    #endif
//      
//    #ifdef USING_APP_LORA
//        TaskStatus_u8 |= AppLora_Process();
//    #endif
//
//    #ifdef USING_APP_ETHERNET
//        TaskStatus_u8 |= AppEth_Task();
//    #endif
//        
//    #ifdef USING_LCD_DISPLAY
//        Display_Task();
//    #endif
//    
//        if ( (TaskStatus_u8 == 0)  \
//            && (sModemInfor.ModePower_u8 == _POWER_MODE_SAVE)
//        #ifdef USING_APP_TEMH
//            && (sTempHumi.IRQMainPowerDetect_u8 == FALSE) 
//        #endif
//        #ifdef USING_APP_WM
//            && (sWmVar.IrqPowUp_u8 == FALSE)
//        #endif
//            )
//        {
//        #ifdef USING_APP_SIM
//            if ( (sSimCommVar.State_u8 == _SIM_POWER_OFF)
//        #ifdef USING_APP_ETHERNET
//                && (sAppEthVar.Status_u8 == _ETH_POWER_OFF) 
//        #endif
//                )
//            {
////                UTIL_LPM_SetStopMode((UTIL_LPM_State_t) LPM_FALSE);
//                APP_LOG(TS_OFF, DBLEVEL_M, "u_app_com: go to stopmode: %d\r\n" , SourceWakeup_u8);
//            } else
////                UTIL_LPM_SetStopMode((UTIL_LPM_State_t) LPM_TRUE);
//        #else
////            UTIL_LPM_SetStopMode((UTIL_LPM_State_t) LPM_FALSE);
////            APP_LOG(TS_OFF, DBLEVEL_M, "u_app_com: go to stopmode: %d\r\n" , SourceWakeup_u8);
//        #endif
//            
//            //Func Lowpower
////            UTIL_LPM_EnterLowPower();
//        }
//        
//    #ifdef ENABLE_TEST_HARDWARE
//        if (RtCountSystick_u32 - LandMarkProcessTask > TimeTaskProcess)
//            TimeTaskProcess = RtCountSystick_u32 - LandMarkProcessTask;
//    #endif
	}
}


/*=================Func handler=====================*/

void AppComm_Init (void)
{
//    uint8_t Count = 0;
    
    //Init Modem
	AppComm_DBG_Init_Uart();
    AppComm_DBG_Init_RX_Mode();
    //Init information in Memory
//	Modem_Init();
    //Func Pointer Lib Timer
    pModemProcessIRQTimer = AppComm_IRQ_Timer_CallBack;
    //Func Pointer Lib LPM
//    sLPMFeatureHandler = &sModemLPMHandle; 
    //Init Func Pointer Log to mem
//    pFunc_UTIL_Log_To_Mem = &LOG_Save_Record; 
    pFunc_UTIL_Debug_Send  = &AppComm_Uart_Debug_Send;
    
//    UTIL_ADV_TRACE_Init();
//    UTIL_ADV_TRACE_SetVerboseLevel(DBLEVEL_M);
    
    //Nhan dang phan cung: 0: ca online va PSM. 1: ONLINE
//    for (uint8_t i = 0; i < 10; i++)
//    {
//         if (HAL_GPIO_ReadPin(HARD_DETECT_GPIO_Port, HARD_DETECT_Pin) == GPIO_PIN_RESET)
//            Count++;
//    }
         
//    if (Count >= 6)
//    {
//        sModemInfor.DetHardware_u8 = __HARD_ONLINE_ONLY;
//        sModemInfor.ModePower_u8   = _POWER_MODE_ONLINE;
//        
//        UTIL_Printf_Str(DBLEVEL_M, "u_app_com: hardware online only\r\n" );
//    } else
//    {
//        sModemInfor.DetHardware_u8 = __HARD_ONLINE_PSM;
//        UTIL_Printf_Str(DBLEVEL_M, "u_app_com: hardware both online and psm\r\n" );
//    }
}

/*
    Func: tao gia tri delay theo ID
*/
uint32_t AppComm_Get_Random_Follow_ID (void)
{
    uint32_t Result = 0;
    
#ifdef USING_APP_LORA
//    uint16_t Index = sModemVar.sDCU_id.Length_u16 - 1;
    
//    Result = (*(sModemVar.sDCU_id.Data_a8 + Index) - 0x30) + (*(sModemVar.sDCU_id.Data_a8 + Index - 1) - 0x30) * 10 ;

//    if (sFreqInfor.Duration_u32 != 0)
//    {
//        Result = (Result * 2) % (sFreqInfor.Duration_u32 * 60);
//        Result *= 1000;
//    } else
//    {
//        Result = 10000;
//    }
#endif
    
    return Result;
}

uint8_t AppComm_Task(void)
{
	uint8_t i = 0;
	uint8_t Result = 0;

	for (i = 0; i < _EVENT_END_COMM; i++)
	{
		if (sEventAppComm[i].e_status == 1)
		{
            if (i != _EVENT_IDLE)
                Result = 1;

			if ((sEventAppComm[i].e_systick == 0) ||
					((HAL_GetTick() - sEventAppComm[i].e_systick)  >=  sEventAppComm[i].e_period))
			{
                sEventAppComm[i].e_status = 0;  //Disable event
				sEventAppComm[i].e_systick = HAL_GetTick();
				sEventAppComm[i].e_function_handler(i);
			}
		}
	}
    
    //Enable Event Period again
    if (sEventAppComm[_EVENT_IDLE].e_status == 0)
        fevent_enable(sEventAppComm, _EVENT_IDLE);

	return Result;
}


 
void AppComm_IRQ_Timer_CallBack (void)
{
    fevent_active(sEventAppComm, _EVENT_TIMMER_IRQ);
}




/*
    - Func: Tinh lai thoi gian sai lệch de hẹn gio tròn
*/
//static uint32_t AppCom_Calcu_Period_To_RoudTime (uint32_t FreqWakeup)
//{    
//    uint32_t FreqCacul = 0;
//
//    Get_RTC();
//    
//    FreqCacul = FreqWakeup - (sRTC.min % FreqWakeup);   //Phep tinh nay tinh ra gia tri 1->Stimer.min. Khong co gia tri 0
//    //Sub second current, and add to 2 (dư phong)
//    if (sRTC.sec < (FreqCacul * 60))
//    {
//        FreqCacul  = FreqCacul * 60 - sRTC.sec + 2;
//        return (FreqCacul * 1000);
//    }
//
//    return (2 * 60000); //default
//}


static void _Cb_TX_Timer_Event(void *context)
{
    UTIL_Printf_Str (DBLEVEL_M, "u_app_comm: tx timer cb\r\n" );

#ifdef USING_APP_MEM
    if (AppMem_Rec_Status() == ERROR)
    {
//        Modem_Packet_Alarm_String("u_app_comm: memory error\r\n");
    }
#endif
    fevent_active(sEventAppComm, _EVENT_TX_TIMER);

    AppComm_Set_Next_TxTimer();
}


void AppComm_Set_Next_TxTimer (void)
{
//    static uint8_t CountMessInit = 0;
//    uint32_t FreqCacul = 10000;
//    char aData[48] = {0};
//    UTIL_TIMER_Status_t ret = UTIL_TIMER_OK;
//    
//    UTIL_Printf_Str (DBLEVEL_M, "u_app_comm: set tx timer\r\n" );
//    //Get chu ki gui tiep theo
//	if (CountMessInit < MAX_NUNBER_SEND_INIT)
//    {
//		CountMessInit ++;
////        sModemVar.CountSleep_u8 = sModemInfor.sFrequence.NumWakeup_u8;  //cho send luon ban tin
//	} else
//    {
//        if (sModemInfor.ModePower_u8 == _POWER_MODE_ONLINE)
//        {
//            sModemInfor.sFrequence.UnitMin_u16 = sModemInfor.sFrequence.DurOnline_u32;  //Doi sang minute
//            //cacul period again to send at rounding stime:
//            FreqCacul = AppCom_Calcu_Period_To_RoudTime (sModemInfor.sFrequence.DurOnline_u32);
//        } else
//        {
//            sModemInfor.sFrequence.UnitMin_u16 = sModemInfor.sFrequence.NumWakeup_u8 * sModemInfor.sFrequence.Duration_u32;
//            //cacul period again to send at rounding stime:
//            FreqCacul = AppCom_Calcu_Period_To_RoudTime (sModemInfor.sFrequence.Duration_u32);
//        }
//           
//        sprintf(aData, "%d-%d-%d %d:%d:%d: %d\r\n", sRTC.date, sRTC.month, sRTC.year,
//                                                sRTC.hour, sRTC.min, sRTC.sec, FreqCacul/1000);
////        Modem_Packet_Alarm_String(aData); 
//        //set Period again.
//        ret = UTIL_TIMER_SetPeriod(&TimerTx, FreqCacul);
//        
//        if (ret == UTIL_TIMER_INVALID_PARAM)
//            UTIL_Log_Str(DBLEVEL_M, "u_app_comm: invalid period tx\r\n");
//    }
//    //Set tiep timer Tx vơi Duty = Thơi gian tròn cua chu ki
//	UTIL_TIMER_Start(&TimerTx);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//    switch (GPIO_Pin)
//    {
//        case PULSE1_DIR_Pin: //Pulse1 Dir
//        #ifdef USING_APP_WM 
//            LPTIM_Counter_Get_Pulse(&hlptim1, &sPulse[0]);
//            fevent_active(sEventAppWM, _EVENT_DIR_CHANGE);
//        #endif
//            break;
//        case Magnetic_switch_Pin: // Intan
//        #ifdef USING_APP_WM 
//            fevent_active(sEventAppWM, _EVENT_EXT_IRQ);
//        #endif
//            break;
//        case VBAT_DETECT_Pin:  //Co nguon 12v tro lai
//        #ifdef USING_APP_WM 
//            fevent_enable(sEventAppWM, _EVENT_POWER_UP_12V);
//        #endif
//            break;
//        case ETH_INTn_Pin:  // Ethernet
//        #ifdef USING_APP_ETHERNET
//            fevent_active(sEventAppEth, _EVENT_ETH_INTn_PROCESS);
//        #endif
//            break;
//        case PULSE2_DIR_Pin:  //Pulse2 Dir
//        #ifdef USING_APP_WM 
//            LPTIM_Counter_Get_Pulse(&hlptim2, &sPulse[1]);
//            fevent_active(sEventAppWM, _EVENT_DIR_CHANGE);
//        #endif
//            break;
//        default:
//            break;
//    }
}

/*
    OBLRSTF     Option Byte Loader reset flag           
    PINRSTF     PIN reset flag                          
    BORRSTF     BOR or POR/PDR reset flag 
    SFTRSTF     Software Reset flag 
    IWDGRSTF    Independent Watchdog reset flag 
    WWDGRSTF    Window watchdog reset flag 
    LPWRRSTF    Low power reset flag                   
*/

uint8_t App_Get_Reset_Source (void)
{
//    uint32_t ResetSource = 0;
    uint8_t Result = FALSE;
    
//    ResetSource = HAL_RCC_GetResetSource();
//    
//    if (ResetSource & RCC_RESET_FLAG_OBL)
//    {
//        Modem_Packet_Alarm_String("u_app_comm: reset source: OBL\r\n");
//    } else if (ResetSource & RCC_RESET_FLAG_SW)
//    {
//        Modem_Packet_Alarm_String("u_app_comm: reset source: SW\r\n");
//    } else if (ResetSource & RCC_RESET_FLAG_IWDG)
//    {
//        Modem_Packet_Alarm_String("u_app_comm: reset source: IWDG\r\n");
//    } else if (ResetSource & RCC_RESET_FLAG_WWDG)
//    {
//        Modem_Packet_Alarm_String("u_app_comm: reset source: WWDG\r\n");
//    } else if (ResetSource & RCC_RESET_FLAG_LPWR)
//    {
//        Modem_Packet_Alarm_String("u_app_comm: reset source: LPWR\r\n");
//    } else if (ResetSource & RCC_RESET_FLAG_PIN) 
//    {
//        Modem_Packet_Alarm_String("u_app_comm: reset source: PIN\r\n");
//        Result = TRUE;
//    } else if (ResetSource & RCC_RESET_FLAG_PWR)
//    {
//        Modem_Packet_Alarm_String("u_app_comm: reset source: PWR\r\n");
//        Result = TRUE;
//    }
                                
    return Result;
}
     

static void AppComm_Uart_Debug_Send (uint8_t *pData, uint16_t Length)
{
    #ifdef USB_CDC_DEBUG
        CDC_Transmit_FS(pData, Length);
    #else
        HAL_UART_Transmit(&uart_debug, pData, Length, 1000);
    #endif
}





#if defined (STM32L433xx) || defined (STM32L452xx) || defined (STM32L496xx)
/*
    OBLRSTF     Option Byte Loader reset flag           
    PINRSTF     PIN reset flag                          
    BORRSTF     BOR or POR/PDR reset flag 
    SFTRSTF     Software Reset flag 
    IWDGRSTF    Independent Watchdog reset flag 
    WWDGRSTF    Window watchdog reset flag 
    LPWRRSTF    Low power reset flag          
*/

uint8_t AppComm_Get_Reset_Source (void)
{
    uint32_t ResetSource = 0;
    uint8_t Result = FALSE;
    
    ResetSource = HAL_RCC_GetResetSource();
    
    if (ResetSource & RCC_RESET_FLAG_OBL)
    {
//        Modem_Packet_Alarm_String("u_app_comm: reset source: OBL\r\n");
    } else if (ResetSource & RCC_RESET_FLAG_SW)
    {
//        Modem_Packet_Alarm_String("u_app_comm: reset source: SW\r\n");
    } else if (ResetSource & RCC_RESET_FLAG_IWDG)
    {
//        Modem_Packet_Alarm_String("u_app_comm: reset source: IWDG\r\n");
    } else if (ResetSource & RCC_RESET_FLAG_WWDG)
    {
//        Modem_Packet_Alarm_String("u_app_comm: reset source: WWDG\r\n");
    } else if (ResetSource & RCC_RESET_FLAG_LPWR)
    {
//        Modem_Packet_Alarm_String("u_app_comm: reset source: LPWR\r\n");
    } else if (ResetSource & RCC_RESET_FLAG_PIN) 
    {
//        Modem_Packet_Alarm_String("u_app_comm: reset source: PIN\r\n");
        Result = TRUE;
    } else if (ResetSource & RCC_RESET_FLAG_PWR)
    {
//        Modem_Packet_Alarm_String("u_app_comm: reset source: PWR\r\n");
        Result = TRUE;
    }
                    
    
    return Result;
}
#endif  

#if defined (STM32L072xx) || defined (STM32L082xx)

/** @brief  Check RCC flag is set or not.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *     @arg @ref RCC_FLAG_HSIRDY HSI oscillator clock ready
  *     @arg @ref RCC_FLAG_HSI48RDY HSI48 oscillator clock ready (not available on all devices)
  *     @arg @ref RCC_FLAG_HSIDIV HSI16 divider flag
  *     @arg @ref RCC_FLAG_MSIRDY MSI oscillator clock ready
  *     @arg @ref RCC_FLAG_HSERDY HSE oscillator clock ready
  *     @arg @ref RCC_FLAG_PLLRDY PLL clock ready
  *     @arg @ref RCC_FLAG_LSECSS LSE oscillator clock CSS detected
  *     @arg @ref RCC_FLAG_LSERDY LSE oscillator clock ready
  *     @arg @ref RCC_FLAG_FWRST Firewall reset
  *     @arg @ref RCC_FLAG_LSIRDY LSI oscillator clock ready
  *     @arg @ref RCC_FLAG_OBLRST Option Byte Loader (OBL) reset
  *     @arg @ref RCC_FLAG_PINRST Pin reset
  *     @arg @ref RCC_FLAG_PORRST POR/PDR reset
  *     @arg @ref RCC_FLAG_SFTRST Software reset
  *     @arg @ref RCC_FLAG_IWDGRST Independent Watchdog reset
  *     @arg @ref RCC_FLAG_WWDGRST Window Watchdog reset
  *     @arg @ref RCC_FLAG_LPWRRST Low Power reset 


  *         The reset flags are RCC_FLAG_PINRST, RCC_FLAG_PORRST, RCC_FLAG_SFTRST,
  *         RCC_FLAG_OBLRST, RCC_FLAG_IWDGRST, RCC_FLAG_WWDGRST, RCC_FLAG_LPWRRST
*/

uint8_t AppComm_Get_Reset_Source (void)
{
    uint8_t Result = FALSE;
        
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_OBLRST) == true)
    {
        UTIL_Log_Str(DBLEVEL_M, "u_app_comm: reset source: OBL\r\n");
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) == true)
    {
        UTIL_Log_Str(DBLEVEL_M, "u_app_comm: reset source: SW\r\n");
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) == true)
    {
        UTIL_Log_Str(DBLEVEL_M, "u_app_comm: reset source: IWDG\r\n");
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) == true)
    {
        UTIL_Log_Str(DBLEVEL_M, "u_app_comm: reset source: WWDG\r\n");
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) == true)
    {
        UTIL_Log_Str(DBLEVEL_M, "u_app_comm: reset source: LPWR\r\n");
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) == true)
    {
        UTIL_Log_Str(DBLEVEL_M, "u_app_comm: reset source: PIN\r\n");
        Result = TRUE;
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) == true)
    {
        UTIL_Log_Str(DBLEVEL_M, "u_app_comm: reset source: PWR\r\n");
        Result = TRUE;
    }
                    
    
    return Result;
}

#endif

    
/*
    Func: Init Uart debug   
*/
void AppComm_DBG_Init_Uart (void)
{
    MX_UART_DBG_Init();
}

/*
    Func: Init Rx Uart debug: DMA or IT
*/
void AppComm_DBG_Init_RX_Mode(void)
{
#if (UART_DBG_MODE == UART_DBG_IT)
//    __HAL_UART_ENABLE_IT(&uart_debug, UART_IT_RXNE);
    HAL_UART_Receive_IT(&uart_debug, &UartDebugRxByte, 1);
#else
    HAL_UARTEx_ReceiveToIdle_DMA(&uart_debug, aUART_DBG_DMA_RX, sizeof(aUART_DBG_DMA_RX));
    __HAL_DMA_DISABLE_IT(&uart_dma_debug, DMA_IT_HT);  
    OldPosDMADebug = 0;
#endif
    
    HAL_NVIC_SetPriority(USART_DBG_IRQn, USART_DBG_Priority, 0);
    HAL_NVIC_EnableIRQ(USART_DBG_IRQn);
}


/*
    Func: stop rx Uart
*/
void AppComm_DBG_Stop_RX_Mode (void)
{
#if (UART_DBG_MODE == UART_DBG_IT)
//    __HAL_UART_DISABLE_IT(&uart_debug, UART_IT_RXNE);   
#else
    HAL_UART_DMAStop(&uart_debug);
#endif
}


/*
    Func: Callback Rx uart: nhan byte va xu ly
*/
void AppComm_DBG_Rx_Callback (uint16_t Size)
{
#if (UART_SIM_MODE == UART_DBG_IT)
    *(sUartDebug.Data_a8 + sUartDebug.Length_u16++) = UartDebugRxByte;

    if (sUartDebug.Length_u16 >= (sizeof(UartDebugBuff) - 1))
        sUartDebug.Length_u16 = 0;

    HAL_UART_Receive_IT(&uart_debug, &UartDebugRxByte, 1);

    fevent_active(sEventAppComm, _EVENT_UART_RX_DEBUG);
#else
    uint16_t NbByteRec = 0, i = 0;
    uint16_t OldPosData = OldPosDMADebug;
       
    if (uart_dma_debug.Init.Mode == DMA_NORMAL)   //sua o day
    {
        if ( (sUartDebug.Length_u16 + Size) >= sizeof(UartDebugBuff) )
                sUartDebug.Length_u16 = 0;
        
        for (i = 0; i < Size; i++)
            *(sUartDebug.Data_a8 + sUartDebug.Length_u16 + i) = aUART_DBG_DMA_RX[i]; 

        sUartDebug.Length_u16 += Size ;
    } else
    {
        //Chia 2 truong hop
        if (Size > OldPosData)
        {
            NbByteRec = Size - OldPosData;
            
            //Kiem tra length cua buff nhan
            if ( (sUartDebug.Length_u16 + NbByteRec) >= sizeof(UartDebugBuff) )
                sUartDebug.Length_u16 = 0;
        
            for (i = 0; i < NbByteRec; i++)
                *(sUartDebug.Data_a8 + sUartDebug.Length_u16 + i) = aUART_DBG_DMA_RX[OldPosData + i]; 
            
            sUartDebug.Length_u16 += NbByteRec;
        } else
        {
            NbByteRec = sizeof(aUART_DBG_DMA_RX) - OldPosData;

            //Kiem tra length cua buff nhan
            if ( (sUartDebug.Length_u16 + Size + NbByteRec) >= sizeof(UartDebugBuff) )
                sUartDebug.Length_u16 = 0;
            
            for (i = 0; i < NbByteRec; i++)
                *(sUartDebug.Data_a8 + sUartDebug.Length_u16 + i) = aUART_DBG_DMA_RX[OldPosData + i]; 
            
            for (i = 0; i < Size; i++)
                *(sUartDebug.Data_a8 + sUartDebug.Length_u16 + i) = aUART_DBG_DMA_RX[i]; 
            
            sUartDebug.Length_u16 += (Size + NbByteRec) ;
        }
        
        //Cap nhat lai Old Position
        OldPosDMADebug = Size;
    }
          
    if (uart_dma_debug.Init.Mode == DMA_NORMAL)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&uart_debug, aUART_DBG_DMA_RX, sizeof(aUART_DBG_DMA_RX));
        __HAL_DMA_DISABLE_IT(&uart_dma_debug, DMA_IT_HT);
    }
        
    fevent_active(sEventAppComm, _EVENT_UART_RX_DEBUG); 
#endif
}


//Init trace
UTIL_ADV_TRACE_Status_t vcom_Init(void (*cb)(void *))
{
    TxCpltCallback = cb;
    return UTIL_ADV_TRACE_OK;
}

UTIL_ADV_TRACE_Status_t vcom_DeInit(void)
{
    __HAL_RCC_USART1_FORCE_RESET();
    __HAL_RCC_USART1_RELEASE_RESET();

    HAL_UART_MspDeInit(&uart_debug);
    
    return UTIL_ADV_TRACE_OK;
}

void vcom_Trace(uint8_t *p_data, uint16_t size)
{
    if (pFunc_UTIL_Debug_Send != NULL)
        pFunc_UTIL_Debug_Send(p_data, size);
}

UTIL_ADV_TRACE_Status_t vcom_Trace_DMA(uint8_t *p_data, uint16_t size)
{    
    if (pFunc_UTIL_Debug_Send != NULL)
        pFunc_UTIL_Debug_Send(p_data, size);

    TxCpltCallback(NULL);
    
    return UTIL_ADV_TRACE_OK;
}


