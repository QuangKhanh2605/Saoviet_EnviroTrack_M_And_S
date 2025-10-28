/*
 * user_sim.c
 *
 *  Created on: Dec 31, 2021
 *      Author: Chien
 */
#include "string.h"
     
#include "user_sim.h"

#include "user_timer.h"
#include "user_string.h"

     
     
/*=========================var struct====================*/
//Define timer handler sim Module
static UTIL_TIMER_Object_t TimerSendTimeout;
static UTIL_TIMER_Object_t TimerNextSendAT;

/*====================Static function=================*/
/* event handler function */
static uint8_t fevent_sim_at_send_handler(uint8_t event);
static uint8_t fevent_sim_at_send_ok_handler(uint8_t event);
static uint8_t fevent_sim_at_send_timeout_handler(uint8_t event);
static uint8_t fevent_sim_uart_receive_handler(uint8_t event);

//Timer event handler
static void OnTimerSendTimeOutEvent(void *context);
static void OnTimerNextSendATEvent(void *context);

extern sData sUartSim;

/*==================Struct var========================*/

sEvent_struct sEventSim[] =
{
	{ _EVENT_SIM_AT_SEND, 			0, 0, SIM_CMD_FREQ,    	fevent_sim_at_send_handler },
    { _EVENT_SIM_UART_RECEIVE, 		0, 0, 50, 				fevent_sim_uart_receive_handler },  //20
	{ _EVENT_SIM_AT_SEND_OK, 		0, 0, 0, 			    fevent_sim_at_send_ok_handler },
	{ _EVENT_SIM_AT_SEND_TIMEOUT, 	0, 0, SIM_CMD_TIMEOUT, 	fevent_sim_at_send_timeout_handler },
};


/*======= Buffer Uart SIM =============*/
//
StructSimVariable    sSimVar;
/*========= Func Pointer ===========*/
static uint8_t aQSimStepComtrol[SIM_MAX_ITEM_QUEUE];
Struct_Queue_Type qSimStep;


#define SIM_DMA_LENGTH  100

uint8_t UartSimRxByte = 0;
uint8_t uartSimBuffReceive[1200] = {0};
sData 	sUartSim = {&uartSimBuffReceive[0], 0};

static uint8_t aUART_RX_DMA_2 [SIM_DMA_LENGTH];
static uint16_t OldPosDMASim = 0;
/*=================Function callback event sim==============*/
void Sim_Init(void)
{
    Sim_Init_Uart();
    Sim_Init_RX_Mode();
    //Init Queue Sim Step
    qQueue_Create (&qSimStep, SIM_MAX_ITEM_QUEUE, sizeof (uint8_t), (uint8_t *) &aQSimStepComtrol); 

	//Khoi tao timer moudle sim: TỉmerATsend, TimerATSendOK...
	UTIL_TIMER_Create(&TimerSendTimeout, 0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnTimerSendTimeOutEvent, NULL);
	UTIL_TIMER_SetPeriod(&TimerSendTimeout, sEventSim[_EVENT_SIM_AT_SEND_TIMEOUT].e_period); 

	UTIL_TIMER_Create(&TimerNextSendAT, 0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnTimerNextSendATEvent, NULL);
	UTIL_TIMER_SetPeriod(&TimerNextSendAT, sEventSim[_EVENT_SIM_AT_SEND].e_period);

    Sim_Common_Init(&sSimCommVar);
}


uint8_t Sim_Task(void)
{
	uint8_t Result = 0;

	uint8_t i = 0;

	for (i = 0; i < _EVENT_SIM_END; i++)
	{
		if (sEventSim[i].e_status == 1)
		{
			Result = 1;
			if ((sEventSim[i].e_systick == 0) || ((HAL_GetTick() - sEventSim[i].e_systick) >= sEventSim[i].e_period))
			{
                sEventSim[i].e_status = 0;   //Disable event
				sEventSim[i].e_systick = HAL_GetTick();
				sEventSim[i].e_function_handler(i);
			}
		}
	}

	return Result;
}


/*
    Func: Init Uart sim   
*/
void Sim_Init_Uart (void)
{
    MX_UART_SIM_Init();
}

/*
    Func: Init Rx Uart sim: DMA or IT
*/
void Sim_Init_RX_Mode(void)
{
#if (UART_SIM_MODE == UART_SIM_IT)
//    __HAL_UART_ENABLE_IT(&uart_sim, UART_IT_RXNE);
    HAL_UART_Receive_IT(&uart_sim, &UartSimRxByte, 1);
#else
    HAL_UARTEx_ReceiveToIdle_DMA(&uart_sim, aUART_RX_DMA_2, sizeof(aUART_RX_DMA_2));
    __HAL_DMA_DISABLE_IT(&uart_dma_sim, DMA_IT_HT);  
    OldPosDMASim = 0;
#endif
    
    HAL_NVIC_SetPriority(USART_SIM_IRQn, USART_SIM_Priority, 0);
    HAL_NVIC_EnableIRQ(USART_SIM_IRQn);
}


/*
    Func: stop rx Uart
*/
void Sim_Stop_RX_Mode (void)
{
#if (UART_SIM_MODE == UART_SIM_IT)
//    __HAL_UART_DISABLE_IT(&uart_sim, UART_IT_RXNE);   
#else
    HAL_UART_DMAStop(&uart_sim);
#endif
}


/*
    Func: Callback Rx uart: nhan byte va xu ly
*/
void Sim_Rx_Callback (uint16_t Size)
{
#if (UART_SIM_MODE == UART_SIM_IT)
    *(sUartSim.Data_a8 + sUartSim.Length_u16++) = UartSimRxByte;

    if (sUartSim.Length_u16 >= (sizeof(uartSimBuffReceive) - 1))
        sUartSim.Length_u16 = 0;

    HAL_UART_Receive_IT(&uart_sim, &UartSimRxByte, 1);

    fevent_active(sEventSim, _EVENT_SIM_UART_RECEIVE);  
#else
    uint16_t NbByteRec = 0, i = 0;
    uint16_t OldPosData = OldPosDMASim;
       
    if (uart_dma_sim.Init.Mode == DMA_NORMAL)   //sua o day
    {
        if ( (sUartSim.Length_u16 + Size) >= sizeof(uartSimBuffReceive) )
                sUartSim.Length_u16 = 0;
        
        for (i = 0; i < Size; i++)
            *(sUartSim.Data_a8 + sUartSim.Length_u16 + i) = aUART_RX_DMA_2[i]; 

        sUartSim.Length_u16 += Size ;
    } else
    {
        //Chia 2 truong hop
        if (Size > OldPosData)
        {
            NbByteRec = Size - OldPosData;
            
            //Kiem tra length cua buff nhan
            if ( (sUartSim.Length_u16 + NbByteRec) >= sizeof(uartSimBuffReceive) )
                sUartSim.Length_u16 = 0;
        
            for (i = 0; i < NbByteRec; i++)
                *(sUartSim.Data_a8 + sUartSim.Length_u16 + i) = aUART_RX_DMA_2[OldPosData + i]; 
            
            sUartSim.Length_u16 += NbByteRec;
        } else
        {
            NbByteRec = sizeof(aUART_RX_DMA_2) - OldPosData;

            //Kiem tra length cua buff nhan
            if ( (sUartSim.Length_u16 + Size + NbByteRec) >= sizeof(uartSimBuffReceive) )
                sUartSim.Length_u16 = 0;
            
            for (i = 0; i < NbByteRec; i++)
                *(sUartSim.Data_a8 + sUartSim.Length_u16 + i) = aUART_RX_DMA_2[OldPosData + i]; 
            
            for (i = 0; i < Size; i++)
                *(sUartSim.Data_a8 + sUartSim.Length_u16 + i) = aUART_RX_DMA_2[i]; 
            
            sUartSim.Length_u16 += (Size + NbByteRec) ;
        }
        
        //Cap nhat lai Old Position
        OldPosDMASim = Size;
    }
          
    if (uart_dma_sim.Init.Mode == DMA_NORMAL)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&uart_sim, aUART_RX_DMA_2, sizeof(aUART_RX_DMA_2));
        __HAL_DMA_DISABLE_IT(&uart_dma_sim, DMA_IT_HT);
    }
        
    fevent_active(sEventSim, _EVENT_SIM_UART_RECEIVE);  
#endif
}


void Sim_Reinit_Uart (void)
{
    Sim_Stop_RX_Mode();
    Sim_Init_Uart();
    Sim_Init_RX_Mode();
}


/* 
    Func: Send Chuoi AT cmd sang Module sim
        + Neu loai AT bt: Timeout 10s
        + Neu loai AT TCP..  Timeout 60s -> Test 30s
        + Retry 3 time
*/

static uint8_t fevent_sim_at_send_handler(uint8_t event)
{
	uint8_t sim_step = 0;
    sCommand_Sim_Struct *sATCmd = NULL;
    
    if (sSimVar.IsPause_u8 == FALSE)
    {
        //Get step current and don't clear
        sim_step = Sim_Get_Step_From_Queue(0); 
            
        if (sim_step < SIM_STEP_END)
        {
            sSimVar.Pending_u8 = true;
            sSimVar.LandMark_u32 = RtCountSystick_u32;
            //Neu lenh Out Data mode -> Dieu khien chan DTR
            if ( (sim_step == SIM_STEP_OUT_DATA_MODE) && (Sim_Active_DTR_Pin() == true) )
            {      
                //Delay 1s before and affer send +++                  
                UTIL_TIMER_SetPeriod(&TimerNextSendAT, 1000); 
                UTIL_TIMER_Start(&TimerNextSendAT);
                
                return 1;
            }
            
            //Increase Num At Precess
            sSimVar.ATProcessed_u8++;
            //Get Struct AT Cmd
            sATCmd = SIM_GET_AT_CMD(sim_step);        
            //Send To uart Sim string AT
            if (SIM_IS_SEND_HEX(sim_step) == true)
            {
                Sim_Common_Send_AT_Cmd( sSimCommVar.pSender.Data_a8 , sSimCommVar.pSender.Length_u16 );
            } else
            {
                if (sATCmd->at_string  != NULL)
                {
                    Sim_Common_Send_AT_Cmd( (uint8_t *) sATCmd->at_string , strlen( sATCmd->at_string ) );
                }
            }

            //Neu k co str respond -> Excute Callback -> active send OK
            if (sATCmd->at_response == NULL)
            {
                if (sATCmd->callback_success != NULL)
                    sATCmd->callback_success (&sUartSim );

                fevent_active(sEventSim, _EVENT_SIM_AT_SEND_OK); 
                sSimVar.IsMatch_u8 = TRUE;
            } else
            {
                UTIL_TIMER_SetPeriod(&TimerSendTimeout, SIM_GET_TIMEOUT(sim_step)); 
                UTIL_TIMER_Start(&TimerSendTimeout);
                //Increase Retry in case need check AT cmd
                sSimVar.Retry_u8++;
            }
        } else
        {
            //Neu dang k xu lý at nao: -> Cho phep active khi push lenh
            sSimVar.Pending_u8 = false;
        }
    } else
    {
        fevent_enable(sEventSim, _EVENT_SIM_AT_SEND);
    }

	return 1;
}


/*
 *Func: Handler: Excute AT OK
 *	- Active event send
 *	- Jum step to 1
 *
 *note:
 *	- Trong trường hợp attach (k cần gửi quá nhanh)
 *	- Set timeout check attach:
 * */

static uint8_t fevent_sim_at_send_ok_handler(uint8_t event)
{
	uint8_t StepNow = 0;
    sCommand_Sim_Struct *sATCmd = NULL;
    sCommand_Sim_Struct *sATCmdNext = NULL;
    static uint8_t cAttach_u8 = 0;
    
	UTIL_TIMER_Stop(&TimerSendTimeout);

    StepNow = Sim_Get_Step_From_Queue(0);
    //ResPond True: Reset retry,...  | else CGATT retry
	if (sSimVar.IsMatch_u8 == TRUE)
	{
        //Get Struct AT Cmd
        sATCmd = SIM_GET_AT_CMD(StepNow); 
        //Neu AT have Response need Check must reset Retry and NumATProcess
        if (sATCmd->at_response != NULL)
        {
            sSimVar.Retry_u8 = 0;
            sSimVar.ATProcessed_u8 = 0;
        }
        //Reset count retry
		cAttach_u8 = 0;
        //at next 
        sATCmdNext = SIM_GET_AT_CMD(Sim_Get_Step_From_Queue(1));  
            
        if (SIM_GET_DELAY_AT(StepNow) != 0)
        {
            sEventSim[_EVENT_SIM_AT_SEND].e_status = 0; 
            
            UTIL_TIMER_SetPeriod(&TimerNextSendAT, SIM_GET_DELAY_AT(StepNow));
            UTIL_TIMER_Start(&TimerNextSendAT);
        } else
        {
            //Neu nhung lenh tiep theo nao k co gui di: active ngay send at | Lenh truoc k co resp
            if ( ( sATCmdNext->at_string  == NULL) || (sATCmd->at_response == NULL) )
                fevent_active(sEventSim, _EVENT_SIM_AT_SEND); 
            else
                fevent_enable(sEventSim, _EVENT_SIM_AT_SEND);
        }
	} else 
	{
		/*=== Neu Attach can retry lai 20 lan=  | Neu Open TCP hay lenh khac ========Cho sang timeout====*/
		if ((StepNow == SIM_STEP_ATTACH) && (cAttach_u8 < SIM_MAX_CGATT))
		{
            cAttach_u8++;
            sSimVar.Retry_u8 = 0;
            fevent_enable(sEventSim, _EVENT_SIM_AT_SEND_TIMEOUT); 
		} else  
		{
			sSimVar.Retry_u8 = SIM_CMD_RETRY;
            fevent_active(sEventSim, _EVENT_SIM_AT_SEND_TIMEOUT); 
		}
	}

	return 1;
}

/*
 * Func: Handler timeout AT
 * 		- Disable event
 * 		- Retry Send again
            + fail: thuc hien cb fail
                    bo qua cac lenh lien quan: chay tiep
 * */

static uint8_t fevent_sim_at_send_timeout_handler(uint8_t event)
{
	uint8_t StepNow = 0;
    uint8_t nAtProcess = sSimVar.ATProcessed_u8;
    
    sSimVar.ATProcessed_u8 = 0;
    UTIL_TIMER_Stop(&TimerSendTimeout);
	//Get step Timeout and print to debug step timeout
	StepNow = Sim_Get_Step_From_Queue(0);
    
    //Neu nhu k co step nao dang chay -> khoi dong lai sim
    if (StepNow == SIM_STEP_END)
    {
        UTIL_Printf_Str( DBLEVEL_M, "u_sim: timeout without at!\r\n" );
        
        sSimCommVar.State_u8 = _SIM_INIT;
        if (sCbSimComm.pFailure != NULL)
            sCbSimComm.pFailure(StepNow);
        
        return 1;
    }
    
    if ( (StepNow + 1 >= nAtProcess)
        && (StepNow + 1 - nAtProcess < SIM_STEP_END) )
    {
        APP_LOG(TS_OFF, DBLEVEL_M, "u_sim: step timeout: %d: %s \r\n",
                                        StepNow + 1 - nAtProcess, 
                                        SIM_GET_AT_CMD(StepNow + 1 - nAtProcess)->at_string ); 
        
        sSimCommVar.sErrorCode.Status_u8 = true;
        sSimCommVar.sErrorCode.Code_u8 = StepNow + 1 - nAtProcess;
    }
    
    if (sSimVar.Retry_u8 >= SIM_CMD_RETRY) 
    {
        sSimVar.Retry_u8 = 0;
        if (SIM_IS_IGNORE_FAILURE (StepNow, Sim_Get_Step_From_Queue) == true)
        {
            fevent_enable(sEventSim, _EVENT_SIM_AT_SEND);
            
            return 1;
        } 
    } else
    {
        //Back AT in queue and Send Again
        if (Sim_Back_Current_AT(nAtProcess - 1) == 1)
        {    
            fevent_active(sEventSim, _EVENT_SIM_AT_SEND);
            
            return 1;
        }
    }
    
    if (sCbSimComm.pFailure != NULL)
        sCbSimComm.pFailure(StepNow);
    
	return 1;
}

static uint8_t fevent_sim_uart_receive_handler(uint8_t event)
{
    static uint8_t  MarkFirst = 0;
    static uint16_t LastLengthRecv = 0; 
	uint8_t sim_step = 0;

    if (MarkFirst == 0)
    {
        MarkFirst = 1;
        LastLengthRecv = sUartSim.Length_u16;
        fevent_enable(sEventSim, event);
    } else
    {
        if (sUartSim.Length_u16 == LastLengthRecv)
        {
            MarkFirst = 0;

            sim_step = Sim_Get_Step_From_Queue(0);
            Sim_Check_Response(sim_step);
        } else
        {
            LastLengthRecv = sUartSim.Length_u16;
            fevent_enable(sEventSim, event);
        }
    }
    
	return 1;
}




uint8_t Sim_Push_Step_To_Queue(uint8_t sim_step)
{
	if (sim_step >= SIM_STEP_END)
		return 0;

	if (qQueue_Send(&qSimStep, (uint8_t *) &sim_step, _TYPE_SEND_TO_END) == 0)
		return 0;

    if ((sSimVar.Pending_u8 == false ) 
        || (Check_Time_Out(sSimVar.LandMark_u32, 80000) == true) )
    {
		fevent_active(sEventSim, _EVENT_SIM_AT_SEND);
    }

	return 1;
}

uint8_t Sim_Push_Block_To_Queue(uint8_t *block_sim_step, uint8_t size)
{
	uint8_t i = 0;

	for (i = 0; i < size; i++)
	{
		if (Sim_Push_Step_To_Queue (block_sim_step[i]) == 0)
			return 0;
	}

	return 1;
}

uint8_t Sim_Check_Response(uint8_t sim_step) 
{
    int     PosFind = 0;
    uint16_t  i = 0;
    sCommand_Sim_Struct *sATCmd = SIM_GET_AT_CMD(sim_step);
    sData   strCheck;
      
    //Printf debug
    UTIL_Printf( DBLEVEL_M, sUartSim.Data_a8, sUartSim.Length_u16);
    
	//Check Respond
    if (sim_step < SIM_STEP_END)
	{
        strCheck.Data_a8 = (uint8_t*) sATCmd->at_response;
        strCheck.Length_u16 = strlen( sATCmd->at_response );      
    
        if (sATCmd->at_response != NULL)
        {
            PosFind = Find_String_V2(&strCheck, &sUartSim);
            if (PosFind >= 0)
            {                   
                sSimVar.IsMatch_u8 = sATCmd->callback_success (&sUartSim);
                fevent_active(sEventSim, _EVENT_SIM_AT_SEND_OK);  
                
                sSimCommVar.TxPinReady_u8 = TRUE;
            }
        }
    }
    //Check Urc
    if (SIM_IS_STEP_CHECK_URC (sim_step) == TRUE)
    {
        for (i = SIM_URC_FIRST; i < SIM_URC_END; i++) 
        {
            strCheck.Data_a8 = (uint8_t *) sSIM_URC[i].at_response;
            strCheck.Length_u16 = strlen(sSIM_URC[i].at_response);
            //
            PosFind = Find_String_V2(&strCheck, &sUartSim);
            if (PosFind >= 0)
            {
                sSIM_URC[i].callback_success(&sUartSim);
            }
        }
    }
    
    Reset_Buff(&sUartSim);
    sUartSim.Length_u16 = 0;
    //Chan rx OK
    sSimCommVar.RxPinReady_u8 = TRUE;   

	return 1;
}


/*===================Function hanlder event timer==================*/

static void OnTimerSendTimeOutEvent(void *context)
{
	UTIL_Printf_Str( DBLEVEL_M, "u_sim: send at timeout!\r\n" );
	fevent_active(sEventSim, _EVENT_SIM_AT_SEND_TIMEOUT);
}

static void OnTimerNextSendATEvent(void *context)
{
	fevent_active(sEventSim, _EVENT_SIM_AT_SEND);
}

/*
 * Type = 0 - don't clear queue
 * */
uint8_t Sim_Get_Step_From_Queue(uint8_t Type)
{
	uint8_t sim_step = 0;

	if (qGet_Number_Items(&qSimStep) == 0)
		return SIM_STEP_END;
    
	if (qQueue_Receive(&qSimStep, (uint8_t *) &sim_step, Type) == 0)
        return SIM_STEP_END;

	return sim_step;
}

/*
    Fun: back numitem in queue to send again AT
*/
uint8_t Sim_Back_Current_AT (uint8_t NumAT)
{
    return qQueue_Back_Item (&qSimStep, NumAT);
}


void Sim_Disable_All_Event (void)
{
	uint8_t i = 0;

    //Disable Event
	for (i = 0; i < _EVENT_SIM_END; i++)
        sEventSim[i].e_status = 0;  
    
    Sim_Clear();
    //Disable Timer TX
    UTIL_TIMER_Stop (&TimerSendTimeout);
    UTIL_TIMER_Stop (&TimerNextSendAT);
}



/*
    Func: Clear Queue step
        + Clear
        + AtStatus = false
*/

void Sim_Clear (void)
{
    Sim_Common_Queue_Clear(&qSimStep);
    sSimVar.Pending_u8 = false;
}

/*
    Func: active chan DTR pin
        + return true: active dtr va cho 1s
        + return false: k active chan nua
*/
uint8_t Sim_Active_DTR_Pin (void)
{
    if (sSimVar.Retry_u8 == 0)   
    {
        sSimVar.Retry_u8++;
        
        if (sSimVar.pSim_Active_DTR_PIN != NULL)
        {
            sSimVar.pSim_Active_DTR_PIN ();
            return true;
        }
    }
    
    return false;
}


















