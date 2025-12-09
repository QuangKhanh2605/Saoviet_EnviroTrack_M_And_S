


#include "user_lptim.h"
#include "lptim.h"

#include "main.h"


#define PERIOD_COUNTER     0xFFFF
/*================Function=====================*/

void LPTIM_Init_Feature (void)
{
#ifdef LPTIM_ENCODER_MODE
    HAL_LPTIM_Encoder_Start_IT(&hlptim1, PERIOD_COUNTER);
#else  
    HAL_LPTIM_Counter_Start(&hlptim1, PERIOD_COUNTER);
#endif
    
#ifdef LPTIM_ENCODER_MODE
    HAL_LPTIM_Encoder_Start_IT(&hlptim2, PERIOD_COUNTER);
#else  
    HAL_LPTIM_Counter_Start(&hlptim2, PERIOD_COUNTER);
#endif
    
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_Base_Start(&htim2);
}



/*
    Feature LpTimer.
*/

void LPTIM_Encoder_Get_Pulse (LPTIM_HandleTypeDef *hlptim, Struct_Pulse *sPulse)
{
    uint16_t LPTRegisValue_u32 = 0;
    static uint16_t LasValue_u16[2] = {0};
    uint8_t Index = 0;
    
    if (hlptim->Instance == LPTIM1)
    {
        Index = 0;
    } else
    {
        Index = 1;
    }
    
	LPTRegisValue_u32 = (uint16_t) HAL_LPTIM_ReadCounter(hlptim);

	if (LPTRegisValue_u32 >=  LasValue_u16[Index])
	{
		if ((LPTRegisValue_u32 - LasValue_u16[Index]) < 0xFFAF)   //khong the quay nguoc dc 0x50 xung trong 10p va khong the quay tien 0xFFAF
		{
			sPulse->Number_u32 	    += (LPTRegisValue_u32 - LasValue_u16[Index]);  //binh thuong
			LasValue_u16[Index]   = LPTRegisValue_u32;
		} else
		{
			sPulse->Number_u32 		-= (PERIOD_COUNTER - LPTRegisValue_u32 + LasValue_u16[Index] + 1); //quay nguoc
			LasValue_u16[Index]   = LPTRegisValue_u32;
		}
	} else
	{
		if ((LasValue_u16[Index] - LPTRegisValue_u32) > 0x50)   //khong the quay nguoc dc 0x50 xung trong 10p
		{
			sPulse->Number_u32  += (PERIOD_COUNTER - LasValue_u16[Index] + LPTRegisValue_u32 + 1);  //binh thuong
			LasValue_u16[Index]		= LPTRegisValue_u32;
		} else
		{
			sPulse->Number_u32  -= (LasValue_u16[Index] - LPTRegisValue_u32);  //binh thuong
			LasValue_u16[Index]		= LPTRegisValue_u32;
		}
	}
}


/*
    Func: Callback Lptim Interupt HAL
*/
void HAL_LPTIM_DirectionUpCallback(LPTIM_HandleTypeDef *hlptim)
{
    UTIL_Printf_Str( DBLEVEL_M, "u_lptim: dir change to forward\r\n" );
    pLPTIM_Func_Direct_Change_Handler (_DIRECT_FORWARD);
}


void HAL_LPTIM_DirectionDownCallback(LPTIM_HandleTypeDef *hlptim)
{
    UTIL_Printf_Str( DBLEVEL_M, "u_lptim: dir change to reverse\r\n" ); 
    pLPTIM_Func_Direct_Change_Handler (_DIRECT_REVERSE);
}

                     

void LPTIM_Counter_Get_Pulse (LPTIM_HandleTypeDef *hlptim, Struct_Pulse *sPulse)
{
    uint16_t LPTRegisValue_u32 = 0;
    static uint16_t LasValue_u16[2] = {0};
    uint8_t Index = 0;
    
    if (hlptim->Instance == LPTIM1)
    {
        Index = 0;
    } else
    {
        Index = 1;
    }
    
    LPTRegisValue_u32 = (uint16_t) HAL_LPTIM_ReadCounter(hlptim);

    if (sPulse->Direct_u8 == 1)
    {
        sPulse->Number_u32 = sPulse->Number_u32 - LPTIM_Check_Pulse_Period( LPTRegisValue_u32, LasValue_u16[Index]);
    }else
    {
        sPulse->Number_u32 = sPulse->Number_u32 + LPTIM_Check_Pulse_Period(LPTRegisValue_u32, LasValue_u16[Index]);
    }
    
#ifdef LPTIM_ENCODER_MODE 
    sPulse->Total_u32 = sPulse->Number_u32 / 4 + sPulse->Start_u32;
#else   
    sPulse->Total_u32 = sPulse->Number_u32 + sPulse->Start_u32;
#endif

    LasValue_u16[Index]      = LPTRegisValue_u32;
    
    LPTIM_Check_DIR_PIN (hlptim, sPulse);
}


void LPTIM_Check_DIR_PIN (LPTIM_HandleTypeDef *hlptim, Struct_Pulse *sPulse)
{
    if (hlptim->Instance == LPTIM1)
    {
        sPulse->Direct_u8 = !HAL_GPIO_ReadPin (PULSE1_DIR_GPIO_Port, PULSE1_DIR_Pin);
    } else
    {
        sPulse->Direct_u8 = !HAL_GPIO_ReadPin (PULSE2_DIR_GPIO_Port, PULSE2_DIR_Pin);
    }
}

uint16_t LPTIM_Check_Pulse_Period (uint16_t Last_Pulse , uint16_t First_Pulse)
{
    if (Last_Pulse < First_Pulse)
		return (PERIOD_COUNTER - First_Pulse + Last_Pulse + 1);
	return (Last_Pulse - First_Pulse);
}



void TIM_Counter_Get_Pulse (TIM_HandleTypeDef *htim, Struct_Pulse *sPulse)
{
    uint16_t TimCurrCnt_u32 = 0;
    static uint16_t LasValue_u16[2] = {0};
    uint8_t Index = 0;
    
    if (htim->Instance == TIM1)
    {
        Index = 0;
    } else
    {
        Index = 1;
    }
    
    TimCurrCnt_u32 = (uint16_t) __HAL_TIM_GetCounter (htim);

    sPulse->Number_u32 = sPulse->Number_u32 + LPTIM_Check_Pulse_Period(TimCurrCnt_u32, LasValue_u16[Index]);
#ifdef LPTIM_ENCODER_MODE 
    sPulse->Total_u32 = sPulse->Number_u32 / 4 + sPulse->Start_u32;
#else   
    sPulse->Total_u32 = sPulse->Number_u32 + sPulse->Start_u32;
#endif
    LasValue_u16[Index]      = TimCurrCnt_u32;
}













