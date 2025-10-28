


#ifndef USER_APP_SIM2_H_
#define USER_APP_SIM2_H_


#include "user_util.h"

#include "user_define.h"
#include "user_sim2.h"


#define USING_APP_SIM2  

#define TOTAL_GPS_ANALYS            10 
#define MAX_SAMPLE                  10 
#define MAX_GPS_LENGTH_PAYLOAD      1024 

/*================ Var struct =================*/
typedef enum
{
    _EVENT_SIM2_CONTROL,
    _EVENT_SIM2_GPS_GET,  
    _EVENT_SIM2_GPS_PACK,
    
    _EVENT_SIM2_CHECK_ACTIVE,
        
	_EVENT_END_SIM2,
}eKindsEventSim2;

typedef struct
{
    double      Lat;
    double      Long;
}sGpsData2;


typedef struct
{
    sGpsData2   aLocation[TOTAL_GPS_ANALYS];
    uint8_t     Index_u8; 
    
    sGpsData2   aLocation5s[MAX_SAMPLE];
    uint8_t     Index5s_u8;
    
    sGpsData2   sLocaOrigin;
    
    sData       sPayload;
    
    uint8_t     Status_u8;       
    uint8_t     DataOK_u8;
    uint32_t    LandMarkDisConnect_u32;
}sGPSInformation;



typedef struct
{
    sGPSInformation        sGPS;
}sAppSim2Variable;



extern sEvent_struct        sEventAppSim2[]; 

extern sAppSim2Variable     sAppSim2Var;

/*================ Func =================*/
void        AppSim2_Init (void);
uint8_t     AppSim2_Task(void);


uint8_t     AppSim2_Process_GPS_AtResp(sData *strSim);
void        AppSim2_Default_Struct_GPS (void);

void        AppSim2_GPS_Analys_Data (void);

uint8_t     AppSim2_GPS_Moving (void);
void        AppSim2_GPS_Packet_Payload (uint8_t Type);


#endif

