#ifndef USER_APP_SENSOR_H__
#define USER_APP_SENSOR_H__

#define USING_APP_SENSOR

#include "user_util.h"
#include "event_driven.h"

#define ID_DEFAULT_SS_TURB      6

#define NUMBER_SAMPLING_SS      10

#define LEVEL_MIN               50
#define LEVEL_MAX               600

#define CURR_OUT_MIN            4
#define CURR_OUT_MAX            20

#define DAC_MIN                 0
#define DAC_MAX                 4095

#define TURB_RANGE_MAX          10

typedef enum
{
    _EVENT_SENSOR_ENTRY,
    _EVENT_SENSOR_TRANSMIT,
    _EVENT_SENSOR_RECEIVE_HANDLE,
    _EVENT_SENSOR_RECEIVE_COMPLETE,
    
    _EVENT_SENSOR_WAIT_CALIB,
    
    _EVENT_DETECT_CONNECT,
    _EVENT_TEMP_ALARM,
    _EVENT_REFRESH_IWDG,
    
    _EVENT_SENSOR_END,
}eKindEventSENSOR;

typedef enum
{
    _KIND_CALIB_OFFSET,
    _KIND_CALIB_POINT_1,
    _KIND_CALIB_POINT_2,
}eKindCalibLevel;

typedef enum
{
    _RS485_SS_TURBIDITY_OPERA = 0,
    
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

typedef struct 
{
    uint8_t CountDisconnectRS485_1;
    uint8_t CountDisconnectRS485_2;
  
    uint8_t State_Wait_Calib;

    uint8_t State_Recv_Turb;
}Struct_Hanlde_RS485;

typedef struct
{
    uint8_t Trans;
    uint8_t Recv;
}Struct_KindMode485;

typedef struct
{
    uint8_t State;
    float Alarm_Lower;
    float Alarm_Upper;
}struct_TempAlarm;

typedef struct
{   
    int16_t Value;
    uint8_t Scale;
}Struct_SS_Value;

typedef struct
{
    uint8_t State_Connect;
    uint8_t Count_Disconnect;
    
    float   Turb_Value_f;
    float   temp_Value_f;
    
    float   Turb_Filter_f;
    float   temp_Filter_f;
    
    float   Turb_Offset_f;
    float   temp_Offset_f;
    
    float           First_Value_f;
    uint32_t        First_AD_u32;
    float           Second_Value_f;
    uint32_t        Second_AD_u32;
    float           Third_Value_f;
    uint32_t        Third_AD_u32;
    float           Fourth_Value_f;
    uint32_t        Fourth_AD_u32;

    uint32_t        Measure_AD;
}Struct_Sensor_Turb;

extern sEvent_struct        sEventAppSensor[];
extern Struct_KindMode485   sKindMode485;
extern struct_TempAlarm     sTempAlarm;
extern Struct_Sensor_Turb   sSensor_Turb;
extern Struct_Hanlde_RS485  sHandleRs485;
/*====================Function Handle====================*/

uint8_t    AppSensor_Task(void);
void       Init_AppSensor(void);

void       Save_ParamCalib(float Turb_Offset_f, float temp_Offset_f);
void       Init_ParamCalib(void);

void       Save_TempAlarm(uint8_t State, float AlarmLower, float AlarmUpper);
void       Init_TempAlarm(void);

float      Filter_Turb(float var);
float      Filter_Temp(float var);
float      ConvertTemperature_Calib(float var);

void       quickSort_Sampling(int32_t array_stt[],int32_t array_sampling[], uint8_t left, uint8_t right);
float      quickSort_Sampling_Value(int32_t Value);

void       Send_RS458_Sensor(uint8_t *aData, uint16_t Length_u16);
uint32_t   Read_Register_Rs485(uint8_t aData[], uint16_t *pos, uint8_t LengthData);

void       RS485_Done_Calib(void);
void       RS485_Enter_Calib(void);
void       RS485_LogData_Calib(uint8_t Kind_Send, const void *data, uint16_t size);

void       Handle_Data_Trans_Sensor(sData *sFrame, uint8_t KindRecv);
void       Handle_Data_Trans_SS_Turb(sData *sFrame, uint8_t KindTrans);

void       Handle_Data_Recv_Sensor(sData sDataRS485, uint8_t KindRecv);
void       Handle_Data_Recv_SS_Turb(sData sDataRS485, uint8_t KindRecv);

void       Handle_State_Sensor(uint8_t KindRecv, uint8_t KindDetect);
void       Handle_State_SS_Turb(uint8_t KindRecv, uint8_t KindDetect);

void       Handle_Data_Measure(uint8_t KindRecv);
#endif
