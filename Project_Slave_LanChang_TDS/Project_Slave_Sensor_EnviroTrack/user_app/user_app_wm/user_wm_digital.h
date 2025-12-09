


#ifndef USER_WM_DIGITAL_H
#define USER_WM_DIGITAL_H


#include "user_util.h"

#include "woteck_wm.h"
#include "level_S_YW_01B.h"
#include "level_SUP_PX261_B.h"
#include "wm_mt100.h"
#include "level_ultrasonic.h"

#include "user_rs485.h"


#define MAX_SLAVE_MODBUS        10

/*=========== Var struct =========*/
typedef enum
{
    __MET_WOTECK,
    __MET_LEVEL_LIQ,
    __MET_LEVEL_LIQ_SUP,
    __MET_MT100,
    __MET_LEVEL_ULTRA,
    __MET_UNKNOWN,
}eRS485MeterType;


typedef struct 
{
    uint32_t BaudRate;
    uint32_t WordLength; 
	uint32_t StopBits;
    uint32_t Parity;
} sUartInterfaceInfor;

typedef struct
{
    uint8_t     Status_u8;
    uint8_t     Type_u8;
    
    int32_t     nTotal_i32;
    uint32_t    nForw_u32;
    uint32_t    nRev_u32;
    int32_t	    Flow_i32;               //luu luong   
    
    char        *sFlowUnit;
    char        *sTotalUnit;
    
    int32_t	    Pressure_i32;           //Ap suat
    
    uint8_t     Factor;
    
    uint16_t    LUnit_u16;
    uint16_t    LDecimal_u16;
    int16_t     LVal_i16;
    int16_t     LZeroPoint_i16;
}sModbusDeviceData;



typedef void (*pFunc_Init ) (uint8_t Type);
typedef void (*pFunc_Decode )(sData *pdata);
typedef struct
{
    uint8_t             Type_u8;
    uint32_t            Addr_u32;
    uint8_t             MaxRegis_u8;
    sUartInterfaceInfor sUart;
	pFunc_Decode        fDecode;
}sWMDigitalInfor;

typedef struct 
{
    uint8_t             MType_u8;
    uint8_t             SlaveId_u8;
}sModbusInfor;

typedef struct 
{
    sModbusInfor        sModbInfor[MAX_SLAVE_MODBUS]; 
    sModbusDeviceData   sModbDevData[MAX_SLAVE_MODBUS];
    uint8_t             nModbus_u8;
}sWMDigitalVariable;


extern sWMDigitalVariable       sWmDigVar;
extern sWMDigitalInfor          sListWmDigital[6];
/*============= Function =============*/
void WM_DIG_Init_Uart (UART_HandleTypeDef *huart, uint8_t type);

void WM_DIG_Get_Data (uint8_t chann, uint8_t type);
void WM_DIG_Packet_Mess (sData *pData, uint8_t chann);


#endif