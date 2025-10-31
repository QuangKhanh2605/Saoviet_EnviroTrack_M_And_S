

#ifndef USER_APP_SENSOR_H_
#define USER_APP_SENSOR_H_

#define USING_APP_SENSOR

#define USING_APP_SENSOR_DEBUG

#define TIME_RESEND_WARNING         5

#define SCALE_SENSOR_DEFAULT        0xFE


#include "event_driven.h"
#include "user_util.h"
#include "user_app_rs485.h"

typedef enum
{
    _EVENT_SENSOR_ENTRY,
    _EVENT_SENSOR_STATE_PH,
    _EVENT_SENSOR_STATE_CLO,
    _EVENT_SENSOR_STATE_EC,
    _EVENT_SENSOR_STATE_TURB,
    
    _EVENT_SENSOR_END,
}eKindEventSensor;

typedef enum 
{
    _OFFSET_CLO,            //Offset Clo
    _OFFSET_PH,             //Offset pH
    _OFFSET_TURB,           //Offset Turbidity
    _OFFSET_EC,             //Offset EC
    _OFFSET_SAL,            //Offset Salinity (do man)
    _OFFSET_TEMP,           //Offset Temperature
}eKindOffsetMeasure;

typedef enum
{
    _INACTIVE_SENSOR,       //Non active sensor
    _ACTIVE_SENSOR,         //Active sensor
}eKindDCU_UserSensor;

typedef enum
{
    _ACTIVE_PH,             //Active sensor pH
    _ACTIVE_CLO,            //Active sensor Clo
    _ACTIVE_EC,             //Active sensor EC
    _ACTIVE_TURB,           //Active sensor Turbidity
}eKindDCU_StateSensor;

typedef struct
{
    float Clo_f;            //Value float offset Clo
    float pH_f;             //Value float offset pH
    float Turb_f;           //Value float offset Turbidity
    float EC_f;             //Value float offset EC
    float Sal_f;            //Value float offset Salinity
    float Temp_f;           //Value float offset Temperature
}Struct_Offset_Measure;

typedef struct
{
    uint8_t User_pH;            //Chon su dung pH hay khong
    uint8_t User_Clo;           //Chon su dung Clo hay khong
    uint8_t User_EC;            //Chon su dung EC hay khong
    uint8_t User_Turb;     //Chon su dung Turb 10 NTU hay khong
}Struct_UserSensor;

extern sEvent_struct                sEventAppSensor[];
extern Struct_Offset_Measure        sOffsetMeasure;
extern Struct_UserSensor            sUserSensor;
/*=============== Function handle ================*/

uint8_t     AppSensor_Task(void);
void        Init_AppSensor(void);

void Log_EventWarnig(uint8_t Obis, uint8_t LengthData, uint8_t *aDataWaring);
void Save_TimeWarningSensor(uint8_t Duration);
void Init_TimeWarningSensor(void);

void Save_OffsetMeasure(uint8_t KindOffset, float Var_Offset_f);
void Init_OffsetMeasure(void);

void Save_UserSensor(uint8_t KindSensor, uint8_t State);
void Init_UserSensor(void);
#endif
