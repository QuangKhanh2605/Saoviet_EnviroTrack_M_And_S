
#ifndef USER_APP_RS485_H__
#define USER_APP_RS485_H__

#define USING_APP_RS485

#include "user_util.h"
#include "event_driven.h"

#define ID_DEFAULT_OXY          5
#define ID_DEFAULT_PH           3

#define ID_DEFAULT_SS_PH        6
#define ID_DEFAULT_SS_EC        7
#define ID_DEFAULT_SS_CLO       8
#define ID_DEFAULT_SS_TURBIDITY 9

#define DEFAULT_SCALE_CLO           0xFE
#define DEFAULT_SCALE_PH            0xFE
#define DEFAULT_SCALE_NTU           0xFE
#define DEFAULT_SCALE_SALINITY      0xFE
#define DEFAULT_SCALE_TEMPERATURE   0xFE
#define DEFAULT_SCALE_EC            0x00

typedef enum
{
    _EVENT_RS485_ENTRY,
    
    _EVENT_RS485_TRANSMIT,
    _EVENT_RS485_RECEIVE_HANDLE,
    _EVENT_RS485_RECEIVE_COMPLETE,
    
    _EVENT_RS485_WAIT_CALIB,
    _EVENT_RS485_REFRESH,
    
    _EVENT_RS485_END,
}eKindEventRs485;

typedef enum
{
    _RS485_SS_PH_OPERA = 0,
    _RS485_SS_CLO_SEND_PH,
    _RS485_SS_EC_OPERA,
    _RS485_SS_CLO_READ_CURRENT,
    _RS485_SS_TURBIDITY_OPERA,
    _RS485_SS_CLO_OPERA,
    
    _RS485_SS_PH_CALIB_READ_AD,
    _RS485_SS_PH_CALIB_READ_POINT_CALIB,
    _RS485_SS_PH_CALIB_ZERO_STD_SOLUTION,
    _RS485_SS_PH_CALIB_SLOPE_STD_SOLUTION,
    _RS485_SS_PH_CALIB_ZERO,
    _RS485_SS_PH_CALIB_SLOPE,
    
    _RS485_SS_CLO_CALIB_READ_AD,
    _RS485_SS_CLO_CALIB_READ_SOLUTION,
    _RS485_SS_CLO_CALIB_STD_SOLUTION,
    _RS485_SS_CLO_CALIB_ZERO,
    _RS485_SS_CLO_CALIB_SLOPE,
    
    _RS485_SS_EC_CALIB_READ_DATA,
    _RS485_SS_EC_CALIB_CONST_STD,
    _RS485_SS_EC_CALIB_CONST_REAL,
    
    _RS485_SS_TUR_CALIB_READ_MEASURE_AD,
    _RS485_SS_TUR_CALIB_READ_POINT_CALIB,
    _RS485_SS_TUR_CALIB_1ST_VALUE,
    _RS485_SS_TUR_CALIB_1ST_AD,
    _RS485_SS_TUR_CALIB_2ND_VALUE,
    _RS485_SS_TUR_CALIB_2ND_AD,
    _RS485_SS_TUR_CALIB_3RD_VALUE,
    _RS485_SS_TUR_CALIB_3RD_AD,
    _RS485_SS_TUR_CALIB_4TH_VALUE,
    _RS485_SS_TUR_CALIB_4TH_AD,
    
    _RS485_5_END,
}eKindMode485;

typedef enum
{
    _SENSOR_DISCONNECT = 0,
    _SENSOR_CONNECT,
}eKindStateSensor;

typedef enum
{
    _RS485_UNRESPOND = 0,
    _RS485_RESPOND,
}eKindStateRs485Respond;

typedef enum
{
    _STATE_CALIB_FREE = 0,
    _STATE_CALIB_ENTER,
    _STATE_CALIB_WAIT,
    _STATE_CALIB_DONE,
    _STATE_CALIB_ERROR,
}eKindStateSendCalib;

typedef struct
{   
    int16_t Value;
    uint8_t Scale;
}Struct_SS_Value;

typedef struct
{
    Struct_SS_Value sClo_Du;
    Struct_SS_Value spH_Water;
    Struct_SS_Value sNTU;
    Struct_SS_Value sSalinity;
    Struct_SS_Value sTemperature;
    Struct_SS_Value sEC;
    
    Struct_SS_Value sClo_Du_Filter;
    Struct_SS_Value spH_Water_Filter;
    Struct_SS_Value sNTU_Filter;
    Struct_SS_Value sSalinity_Filter;
    Struct_SS_Value sTemperature_Filter;
    Struct_SS_Value sEC_Filter;
}Struct_Data_Sensor_Measure;

typedef struct 
{
    uint8_t CountDisconnectRS485_1;
    uint8_t CountDisconnectRS485_2;
  
    uint8_t State_Wait_Calib;
  
    uint8_t State_Recv_Clo;
    uint8_t State_Recv_pH;
    uint8_t State_Recv_NTU;
    uint8_t State_Recv_Salinity;
    uint8_t State_Recv_Temperature;
    uint8_t State_Recv_EC;
}Struct_Hanlde_RS485;

typedef struct
{
    uint8_t Trans;
    uint8_t Recv;
}Struct_KindMode485;

typedef struct
{
    uint8_t State_Connect;
    uint8_t Count_Disconnect;

    Struct_SS_Value sPH_Value;
    Struct_SS_Value sTemperature_Value;
    
    Struct_SS_Value sZero_Calib;
    Struct_SS_Value sSlope_Calib;

    uint32_t        Measure_AD;
}Struct_Sensor_pH;

typedef struct
{
    uint8_t State_Connect;
    uint8_t Count_Disconnect;
    
    Struct_SS_Value sClo_Value;
    Struct_SS_Value sTemperature_Value;
    Struct_SS_Value sPH;
    
    Struct_SS_Value sSolution_Calibration;
    
    uint32_t        Measure_AD;
}Struct_Sensor_Clo;

typedef struct
{
    uint8_t State_Connect;
    uint8_t Count_Disconnect;
    
    Struct_SS_Value sConductivity_Value;
    Struct_SS_Value sResistivity_Value;
    Struct_SS_Value sTemperature_Value;
    Struct_SS_Value sTDS_Value;
    Struct_SS_Value sSalinity_Value;
    
    Struct_SS_Value sConductivity_Const;
    Struct_SS_Value sConductivity_Calib;
}Struct_Sensor_EC;

typedef struct
{
    uint8_t State_Connect;
    uint8_t Count_Disconnect;
    
    Struct_SS_Value sNTU_Value;
    Struct_SS_Value sTemperature_Value;
    
    Struct_SS_Value sFirst_Calibration;
    uint32_t        First_AD_u32;
    Struct_SS_Value sSecond_Calibration;
    uint32_t        Second_AD_u32;
    Struct_SS_Value sThird_Calibration;
    uint32_t        Third_AD_u32;
    Struct_SS_Value sFourth_Calibration;
    uint32_t        Fourth_AD_u32;
    
    uint32_t        Measure_AD;
}Struct_Sensor_Turbidity;

extern sEvent_struct        sEventAppRs485[];
extern Struct_KindMode485   sKindMode485;
extern Struct_Sensor_pH            sSensor_pH;
extern Struct_Sensor_EC            sSensor_EC;
extern Struct_Sensor_Clo           sSensor_Clo;
extern Struct_Sensor_Turbidity     sSensor_Turbidity;
extern Struct_Data_Sensor_Measure  sDataSensorMeasure;
extern Struct_Hanlde_RS485         sHandleRs485;
/*====================Function Handle====================*/

uint8_t    AppRs485_Task(void);
void       Init_AppRs485(void);
void       Save_IdSlave(uint8_t ID_Oxy, uint8_t ID_pH);
void       Init_IdSlave(void);

void       AT_CMD_Get_State_Sensor(sData *str, uint16_t Pos);
void       AT_CMD_Get_Measure_Value (sData *str_Receiv, uint16_t Pos);
void       AT_CMD_Get_Measure_Filter (sData *str_Receiv, uint16_t Pos);

void       Init_Parameter_Sensor(void);

//void       Send_RS458_Normal(uint8_t *aData, uint16_t Length_u16);
void       Init_UartRs485(void);
void       Send_RS458_Sensor(uint8_t *aData, uint16_t Length_u16);

int32_t    Hanlde_Float_To_Int32_Scale_Round(float varFloat, uint8_t scale);
int32_t    Handle_HexFloat_To_Int32_Round(uint32_t hexFloat, uint8_t scale);
float      Handle_int32_To_Float_Scale(int32_t var, uint8_t scale);
uint32_t   Handle_Float_To_hexUint32(float num);

void       RS485_Done_Calib(void);
void       RS485_Enter_Calib(void);
void       RS485_LogData_Calib(uint8_t Kind_Send, const void *data, uint16_t size);
uint32_t   Read_Register_Rs485(uint8_t aData[], uint16_t *pos, uint8_t LengthData);

int16_t    _fSet_pH_Zero_Calib_UpDown(int16_t Value, int8_t Kind);
int16_t    _fSet_pH_Slope_Calib_UpDown(int16_t Value, int8_t Kind);
int16_t    _fRead_pH_Zero_Point(int16_t Value);
int16_t    _fRead_pH_Slope_Point(int16_t Value);
int16_t    _fRead_pH_Zero_Calib(int16_t point);
int16_t    _fRead_pH_Slope_Calib(int16_t point);

void       Handle_State_SS_pH(uint8_t KindRecv, uint8_t KindDetect);
void       Handle_State_SS_Clo(uint8_t KindRecv, uint8_t KindDetect);
void       Handle_State_SS_EC(uint8_t KindRecv, uint8_t KindDetect);
void       Handle_State_SS_Turbidity(uint8_t KindRecv, uint8_t KindDetect);
void       Handle_State_Sensor(uint8_t KindRecv, uint8_t KindDetect);

void       Handle_Data_Recv_SS_pH(sData sDataRS485, uint8_t KindRecv);
void       Handle_Data_Recv_SS_Clo(sData sDataRS485, uint8_t KindRecv);
void       Handle_Data_Recv_SS_EC(sData sDataRS485, uint8_t KindRecv);
void       Handle_Data_Recv_SS_Turbidity(sData sDataRS485, uint8_t KindRecv);
void       Handle_Data_Recv_Sensor(sData sDataRS485, uint8_t KindRecv);

void       Handle_Data_Trans_SS_pH(sData *sFrame, uint8_t KindRecv);
void       Handle_Data_Trans_SS_Clo(sData *sFrame, uint8_t KindRecv);
void       Handle_Data_Trans_SS_EC(sData *sFrame, uint8_t KindRecv);
void       Handle_Data_Trans_SS_Turbidity(sData *sFrame, uint8_t KindRecv);
void       Handle_Data_Trans_Sensor(sData *sFrame, uint8_t KindRecv);
void       Handle_Data_Measure(uint8_t KindRecv);
#endif

