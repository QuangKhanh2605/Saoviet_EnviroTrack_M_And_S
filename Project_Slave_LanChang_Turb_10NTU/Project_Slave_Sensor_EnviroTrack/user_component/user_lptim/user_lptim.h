


#ifndef _USER_LPTIM_H
#define _USER_LPTIM_H 	

#include "user_util.h"
#include "lptim.h"
#include "tim.h"

/*================Define=====================*/

//#ifndef LPTIM_ENCODER_MODE
//	#define LPTIM_ENCODER_MODE
//#endif

//#ifndef LPTIM2_ENCODER_MODE
//	#define LPTIM2_ENCODER_MODE
//#endif

/*================Var struct=====================*/
typedef enum
{
    _DIRECT_FORWARD = 1,
    _DIRECT_REVERSE,
    _DIRECT_END,
}Kind_Direct_WM;


typedef struct
{
	uint32_t	Number_u32;             //counter pulse
    uint32_t	NumberInit_u32;         //init counter pulse
    uint8_t     Direct_u8;              //0: quay thuan, 1: quay nguoc
    int32_t	    Flow_i32;               //luu luong    
    int32_t	    Quantity_i32;           //San luong 
 
    uint8_t     IsOver5Pulse_u8;        //Da chay qua 5 xung chua
    
    uint32_t    Total_u32;              //Chi so thuc te
    uint32_t    nPositive_u32;          //chieu thuan
    uint32_t    nNegative_u32;          //chieu nguoc
    
    uint32_t    Start_u32;              //Chi so ban dau
    uint8_t     Factor_u8;              //He so xung:  
    uint8_t     Status_u8;              //trang thai hoạt dong
}Struct_Pulse;


/*================Extern var struct=====================*/
extern void (*pLPTIM_Func_Direct_Change_Handler) (uint8_t Type);


/*================Function=====================*/
void        LPTIM_Encoder_Get_Pulse (LPTIM_HandleTypeDef *hlptim, Struct_Pulse *sPulse);

uint16_t    LPTIM_Check_Pulse_Period (uint16_t Last_Pulse , uint16_t First_Pulse);
void        LPTIM_Check_DIR_PIN (LPTIM_HandleTypeDef *hlptim, Struct_Pulse *sPulse);

void        LPTIM_Init_Feature (void);
void        LPTIM_Counter_Get_Pulse (LPTIM_HandleTypeDef *hlptim, Struct_Pulse *sPulse);

void        TIM_Counter_Get_Pulse (TIM_HandleTypeDef *htim, Struct_Pulse *sPulse);

#endif
