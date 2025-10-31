
#ifndef USER_APP_RS485_H__
#define USER_APP_RS485_H__

#define USING_APP_RS485

#include "user_util.h"
#include "event_driven.h"

#define ID_DEFAULT_OXY          5
#define ID_DEFAULT_PH           3

#define ID_DEFAULT_SS_PH        1
#define ID_DEFAULT_SS_CLO       2
#define ID_DEFAULT_SS_EC        3
#define ID_DEFAULT_SS_TURB      4

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
    _RS485_SS_CLO_OPERA,
    
    _RS485_SS_EC_OPERA,
    
    _RS485_SS_TURB_OPERA,
    
    _RS485_SS_END,
}eKindModeModbusRTU;

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
    uint8_t State_u8;
    uint8_t Scale_u8;
    int32_t Value_i32;
}Struct_SS_Value;

typedef struct
{
    Struct_SS_Value sClo;
    Struct_SS_Value spH;
    Struct_SS_Value sTurb;
    Struct_SS_Value sEC;
    Struct_SS_Value sSal;
    Struct_SS_Value sTemp;
}Struct_Data_Sensor_Measure;

typedef struct 
{
    uint8_t CountDisconnectRS485_1;
    uint8_t CountDisconnectRS485_2;
  
    uint8_t State_Wait_Calib;
}Struct_Hanlde_RS485;

typedef struct
{
    uint8_t Trans;
    uint8_t Recv;
}Struct_KindMode485;

typedef struct
{
    uint8_t State_Connect_u8;
    uint8_t Count_Disconnect;
    uint8_t State_Recv_Data;
    
    float   pH_Value_f;
    float   Temp_Value_f;
}sStruct_RS485_pH;

typedef struct
{
    uint8_t State_Connect_u8;
    uint8_t Count_Disconnect;
    uint8_t State_Recv_Data;
    
    float   Clo_Value_f;
    float   Temp_Value_f;
}sStruct_RS485_Clo;

typedef struct
{
    uint8_t State_Connect_u8;
    uint8_t Count_Disconnect;
    uint8_t State_Recv_Data;
    
    float   EC_Value_f;
    float   TDS_Value_f;
    float   Sal_Value_f;
    float   Temp_Value_f;
}sStruct_RS485_EC;

typedef struct
{
    uint8_t State_Connect_u8;
    uint8_t Count_Disconnect;
    uint8_t State_Recv_Data;
    
    float   Turb_Value_f;
    float   Temp_Value_f;
}sStruct_RS485_Turb;

extern sEvent_struct        sEventAppRs485[];
extern Struct_KindMode485   sKindMode485;

extern Struct_Data_Sensor_Measure  sDataSensorMeasure;
extern Struct_Hanlde_RS485         sHandleRs485;

extern sStruct_RS485_pH            sRs485_pH;
extern sStruct_RS485_Clo           sRs485_Clo;
extern sStruct_RS485_EC            sRs485_EC;
extern sStruct_RS485_Turb          sRs485_Turb;
/*====================Function Handle====================*/

uint8_t    AppRs485_Task(void);
void       Init_AppRs485(void);

void       Save_IdSlave(uint8_t ID_Oxy, uint8_t ID_pH);
void       Init_IdSlave(void);

void       Init_Parameter_Sensor(void);

void       Init_UartRs485(void);
void       Send_RS458_Sensor(uint8_t *aData, uint16_t Length_u16);

uint32_t   Read_Register_Rs485(uint8_t aData[], uint16_t *pos, uint8_t LengthData);

void       Handle_Data_Trans_SS_pH(sData *sFrame, uint8_t KindRecv);
void       Handle_Data_Trans_SS_Clo(sData *sFrame, uint8_t KindRecv);
void       Handle_Data_Trans_SS_EC(sData *sFrame, uint8_t KindRecv);
void       Handle_Data_Trans_SS_Turb(sData *sFrame, uint8_t KindRecv);
void       Handle_Data_Trans_Sensor(sData *sFrame, uint8_t KindRecv);
void       Handle_Data_Measure(uint8_t KindRecv);

void       Handle_Data_Recv_SS_pH(sData sDataRS485, uint8_t KindRecv);
void       Handle_Data_Recv_SS_Clo(sData sDataRS485, uint8_t KindRecv);
void       Handle_Data_Recv_SS_EC(sData sDataRS485, uint8_t KindRecv);
void       Handle_Data_Recv_SS_Turb(sData sDataRS485, uint8_t KindRecv);
void       Handle_Data_Recv_Sensor(sData sDataRS485, uint8_t KindRecv);

void       Handle_State_SS_pH(uint8_t KindRecv, uint8_t KindDetect);
void       Handle_State_SS_Clo(uint8_t KindRecv, uint8_t KindDetect);
void       Handle_State_SS_EC(uint8_t KindRecv, uint8_t KindDetect);
void       Handle_State_SS_Turb(uint8_t KindRecv, uint8_t KindDetect);
void       Handle_State_Sensor(uint8_t KindRecv, uint8_t KindDetect);

#endif

