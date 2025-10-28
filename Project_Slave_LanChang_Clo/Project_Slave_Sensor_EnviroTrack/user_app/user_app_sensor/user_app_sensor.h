#ifndef USER_APP_SENSOR_H__
#define USER_APP_SENSOR_H__

#define USING_APP_SENSOR

#include "user_util.h"
#include "event_driven.h"

#define ID_DEFAULT_SS_CLO       7

#define NUMBER_SAMPLING_SS      10

#define LEVEL_MIN               50
#define LEVEL_MAX               600

#define CURR_OUT_MIN            4
#define CURR_OUT_MAX            20

#define DAC_MIN                 0
#define DAC_MAX                 4095

#define CLO_RANGE_MAX           2

typedef enum
{
    _EVENT_SENSOR_ENTRY,
    _EVENT_SENSOR_TRANSMIT,
    _EVENT_SENSOR_RECEIVE_HANDLE,
    _EVENT_SENSOR_RECEIVE_COMPLETE,
    
    _EVENT_SENSOR_WAIT_CALIB,
    _EVENT_DETECT_PH_RECV,
    
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
    _RS485_SS_CLO_SEND_PH = 0,
    _RS485_SS_CLO_READ_CURRENT,
    _RS485_SS_CLO_OPERA,
    
    _RS485_SS_CLO_CALIB_READ_AD,
    _RS485_SS_CLO_CALIB_READ_SOLUTION,
    _RS485_SS_CLO_CALIB_STD_SOLUTION,
    _RS485_SS_CLO_CALIB_ZERO,
    _RS485_SS_CLO_CALIB_SLOPE,
    
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
    _DCU_CALIB_CLO_ZERO,
    _DCU_CALIB_CLO_SLOPE,
    _DCU_CALIB_CLO_POINT1,
    _DCU_CALIB_CLO_POINT2,
    _DCU_CALIB_CLO_CONST_TEMP,
}eKindDCU_Calib_Sensor;

typedef struct 
{
    uint8_t CountDisconnectRS485_1;
    uint8_t CountDisconnectRS485_2;
  
    uint8_t State_Wait_Calib;
    
    uint8_t State_Recv_Clo;
    uint8_t State_Recv_Temperature;
}Struct_Hanlde_RS485;

typedef struct
{
    uint8_t Trans;
    uint8_t Recv;
}Struct_KindMode485;

typedef struct
{   
    int16_t Value;
    uint8_t Scale;
}Struct_SS_Value;

typedef struct 
{
    Struct_SS_Value sClo_Du;
    Struct_SS_Value sConst_Compensation_Temp;
    
    uint8_t     Scale;
    uint16_t    Measure_AD;
    
    uint16_t    ADC_Zero;
    
    uint16_t    ADC_SLope;
    int16_t     Clo_Calib_Slope;
    int16_t     Temp_Calib_Slope;
    int16_t     Ph_Calib_Slope;
    
    uint16_t    ADC_CalibPoint_1;
    int16_t     Clo_CalibPoint_1;
    int16_t     Temp_CalibPoint_1;
    int16_t     Ph_CalibPoint_1;
    
    uint16_t    ADC_CalibPoint_2;
    int16_t     Clo_CalibPoint_2;
    int16_t     Temp_CalibPoint_2;
    int16_t     Ph_CalibPoint_2;
    
}Struct_ConvertChlorine;

typedef struct
{
    uint8_t State;
    float Alarm_Lower;
    float Alarm_Upper;
}struct_TempAlarm;

typedef struct
{
    uint8_t State_Connect;
    uint8_t Count_Disconnect;
    
    float   Clo_Value_f;
    float   temp_Value_f;
    
    float   Clo_Filter_f;
    float   temp_Filter_f;
    
    float   Clo_Offset_f;
    float   temp_Offset_f;
    
    Struct_SS_Value sSolution_Calibration;
    
    uint32_t        Measure_AD;
}Struct_Sensor_Clo;

typedef struct
{
    uint8_t StateConnect;
    float   pH_f;
}Struct_pH_Recv_Master;

extern sEvent_struct        sEventAppSensor[];
extern Struct_KindMode485   sKindMode485;
extern struct_TempAlarm     sTempAlarm;
extern Struct_Sensor_Clo    sSensor_Clo;
extern Struct_Hanlde_RS485  sHandleRs485;
extern Struct_ConvertChlorine      sConvertChlorine;
extern Struct_pH_Recv_Master   spHRecvMaster;
/*====================Function Handle====================*/

uint8_t    AppSensor_Task(void);
void       Init_AppSensor(void);
void       Init_Parameter_Sensor(void);

void       Save_ParamCalib(float Clo_Offset_f, float temp_Offset_f);
void       Init_ParamCalib(void);

void       Save_TempAlarm(uint8_t State, float AlarmLower, float AlarmUpper);
void       Init_TempAlarm(void);

float      Filter_Clo(float var);
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
void       Handle_Data_Trans_SS_Clo(sData *sFrame, uint8_t KindRecv);

void       Handle_Data_Recv_Sensor(sData sDataRS485, uint8_t KindRecv);
void       Handle_Data_Recv_SS_Clo(sData sDataRS485, uint8_t KindRecv);

void       Handle_State_Sensor(uint8_t KindRecv, uint8_t KindDetect);
void       Handle_State_SS_Clo(uint8_t KindRecv, uint8_t KindDetect);

void       Handle_Data_Measure(uint8_t KindRecv);

void Init_Chlorine_Calib(void);
void Save_Chlorine_Calib(uint16_t    ADC_Zero,
                         uint16_t    ADC_SLope,
                         int16_t     Clo_Calib_Slope,
                         int16_t     Temp_Calib_Slope,
                         int16_t     Ph_Calib_Slope);
//float compute_clo_du(uint16_t adc, int16_t pH, int16_t temp_C);

void Save_Chlorine_PointCalib(uint16_t    ADC_Point_1,
                              int16_t     Clo_Point_1,
                              int16_t     Temp_Point_1,
                              int16_t     Ph_Point_1,
                              uint16_t    ADC_Point_2,
                              int16_t     Clo_Point_2,
                              int16_t     Temp_Point_2,
                              int16_t     Ph_Point_2);
void Init_Chlorine_PointCalib_1(void);
float Chlorine_Compensation_pH(uint16_t adc, int16_t pH, int16_t temp_C);
void DCU_Logdata_Calib(uint8_t KindCalib, int32_t Value);
void DCU_Enter_Calib(void);

void Save_Const_Temp_Compensation_Chlorine(uint16_t value);
void Init_Const_Temp_Compensation_Chlorine(void);
#endif
