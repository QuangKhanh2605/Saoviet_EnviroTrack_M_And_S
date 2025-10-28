


#ifndef USER_APP_SIM_H_
#define USER_APP_SIM_H_


#include "user_util.h"
#include "user_sim.h"
#include "user_mqtt.h"

#include "user_at_serial.h"  
#include "user_define.h"



#define USING_APP_SIM   


/*GPS: So vi tri lay mau kiem tra di chuyen*/
#define MAX_SAMPLE_GPS      12

/*GPS: Khoang cach so sanh di chuyen*/
#define DISTANCE_GPS_MOVE   50

#define MAX_HARD_RESET		    20
#define MAX_SIM_CHANGE_SERVER   5
#define MAX_SOFT_RESET		    2
//
#define BUFF_LENGTH_SETUP	30

 
// ********** ON - OFF POWER
#ifdef SIM_BC660_LIB
    #define SIM_PW_OFF1         HAL_GPIO_WritePin(SIM_ON_OFF_GPIO_Port, SIM_ON_OFF_Pin, GPIO_PIN_RESET  )  
    #define SIM_PW_ON1          HAL_GPIO_WritePin(SIM_ON_OFF_GPIO_Port, SIM_ON_OFF_Pin, GPIO_PIN_SET)

    // ********** PWKEY
    #define SIM_PWKEY_ON1       //HAL_GPIO_WritePin(SIM_PWR_KEY_GPIO_Port, SIM_PWR_KEY_Pin, GPIO_PIN_SET)
    #define SIM_PWKEY_OFF1      //HAL_GPIO_WritePin(SIM_PWR_KEY_GPIO_Port, SIM_PWR_KEY_Pin, GPIO_PIN_RESET)
    //============Sim Reset
    #define SIM_RESET_ON1       HAL_GPIO_WritePin(SIM_RESET_GPIO_Port, SIM_RESET_Pin, GPIO_PIN_SET)  
    #define SIM_RESET_OFF1      HAL_GPIO_WritePin(SIM_RESET_GPIO_Port, SIM_RESET_Pin, GPIO_PIN_RESET)

    // ********** ON - OFF DTR  
    #define SIM_DTR_OFF1        //HAL_GPIO_WritePin(SIM_DTR_GPIO_Port, SIM_DTR_Pin, GPIO_PIN_RESET)  
    #define SIM_DTR_ON1         //HAL_GPIO_WritePin(SIM_DTR_GPIO_Port, SIM_DTR_Pin, GPIO_PIN_SET)

    // ********** ON - OFF DTR  
    #define SIM_WAKE_LOW        HAL_GPIO_WritePin(SIM_PWR_KEY_GPIO_Port, SIM_PWR_KEY_Pin, GPIO_PIN_RESET)
    #define SIM_WAKE_HI         HAL_GPIO_WritePin(SIM_PWR_KEY_GPIO_Port, SIM_PWR_KEY_Pin, GPIO_PIN_SET)
#else
    // ********** ON - OFF POWER
    #define SIM_PW_OFF1         HAL_GPIO_WritePin(SIM_ON_OFF_GPIO_Port, SIM_ON_OFF_Pin, GPIO_PIN_RESET)
    #define SIM_PW_ON1          HAL_GPIO_WritePin(SIM_ON_OFF_GPIO_Port, SIM_ON_OFF_Pin, GPIO_PIN_SET)

    // ********** PWKEY
    #define SIM_PWKEY_ON1       HAL_GPIO_WritePin(SIM_PWR_KEY_GPIO_Port, SIM_PWR_KEY_Pin, GPIO_PIN_SET)
    #define SIM_PWKEY_OFF1      HAL_GPIO_WritePin(SIM_PWR_KEY_GPIO_Port, SIM_PWR_KEY_Pin, GPIO_PIN_RESET)
    //============Sim Reset
    #define SIM_RESET_ON1       HAL_GPIO_WritePin(SIM_RESET_GPIO_Port, SIM_RESET_Pin, GPIO_PIN_SET)  
    #define SIM_RESET_OFF1      HAL_GPIO_WritePin(SIM_RESET_GPIO_Port, SIM_RESET_Pin, GPIO_PIN_RESET)

    // ********** ON - OFF DTR  
    #define SIM_DTR_OFF1        //HAL_GPIO_WritePin(SIM_DTR_GPIO_Port, SIM_DTR_Pin, GPIO_PIN_RESET)  
    #define SIM_DTR_ON1         //HAL_GPIO_WritePin(SIM_DTR_GPIO_Port, SIM_DTR_Pin, GPIO_PIN_SET)

    // ********** ON - OFF DTR  
    #define SIM_WAKE_LOW        //HAL_GPIO_WritePin(SIM_PWR_KEY_GPIO_Port, SIM_PWR_KEY_Pin, GPIO_PIN_RESET)
    #define SIM_WAKE_HI         //HAL_GPIO_WritePin(SIM_PWR_KEY_GPIO_Port, SIM_PWR_KEY_Pin, GPIO_PIN_SET)
#endif //sim_bc660



/*================ Var struct =================*/
typedef enum
{
    _EVENT_SIM_HARD_RESET,
	_EVENT_SIM_TURN_ON,
	_EVENT_SIM_WAKEUP,
    _EVENT_SIM_DTR_PIN,
    _EVENT_SIM_POWER_OFF,
    
    _EVENT_SIM_SEND_MESS,      
    _EVENT_SIM_SEND_PING,  
    _EVENT_SIM_REQ_GPS,
    _EVENT_SIM_GET_GPS,
    
    _EVENT_SIM_SMS_CTRL,
    _EVENT_SIM_CALL_CTRL,
    
	_EVENT_END_SIM,
}eKindsEventSim;


typedef struct
{
    uint8_t     aData[30];
    uint8_t     Length_u8;
    double      Lat;
    double      Long;
}sGpsData;

typedef struct
{
    sGpsData    sLocation[MAX_SAMPLE_GPS];
    sGpsData    sLocaOrigin;
    
    uint8_t     Status_u8;
    uint8_t     Index_u8;
    uint8_t     IsGetOrigin_u8;
    uint8_t     MarkFirstError_u8;
    uint16_t    CountError_u16;
    uint32_t    LandMarkGPSOk_u32;
}sGpsInformation;



typedef struct
{
    sAPNInformation     sAPN;
//    sServerInformation  sServerMain;
//    sServerInformation  sServerBack;
//    sUpdateinformation  sUpdate;
    
}sAppSimInformation;

typedef struct
{
    uint8_t Pending_u8;
    uint8_t Mark_u8;
    uint8_t Type_u8;
}sSmsVariable;

typedef struct
{
    uint8_t Pending_u8;
    uint8_t Mark_u8;
}sCallVariable;


typedef struct
{
    sData       sDataGPS;
    
    sGpsInformation sGPS;
    uint8_t     IsFinishHandleSim_u8;
    
    uint16_t	cHardReset_u16;         
    uint8_t     cSoftReset_u8;
    uint32_t    LandMarkConnMQTT_u32;
    
    uint8_t     StepPowerOn_u8;        
    
    uint8_t     *pModeConnNow_u8;
    uint8_t     *pModeConnFuture_u8;
    uint8_t     *pModeConnLast_u8;  
    
    uint8_t     *pModePower_u8;   
    
    void        ( *pRespond ) (uint8_t portNo, uint8_t *data, uint16_t length, uint8_t ack);
    uint8_t     ( *pReset_MCU ) (void);
    uint8_t     ( *pReset_MCU_Imediate ) (void);
    uint8_t     ( *pReq_Set_sTime ) (ST_TIME_FORMAT *sClock, uint8_t GMT);
    void        ( *pReq_Save_Box ) (void);
    uint8_t     ( *pCheck_AT_User) (sData *str, uint8_t type);

    uint8_t     IsFwUpdateSuccess_u8;
    uint8_t     rExternIrq_u8;
    
    //test sms
    sSmsVariable    sSms;
    sCallVariable   sCall;
}sAppSimVariable;


extern sEvent_struct        sEventAppSim []; 
extern sAppSimVariable      sAppSimVar;
extern sAppSimInformation   sAppSimInfor;

/*================ Func =================*/
void    AppSim_Init (void);
uint8_t AppSim_Task(void);

uint8_t AppSim_Send_Mess (void);

void    _CbAppSim_Recv_sTime (ST_TIME_FORMAT *sClock);
void    _CbAppSim_SMS_Send_1 (void);
void    _CbAppSim_SMS_Send_2 (void);
void    _CbAppSim_SMS_Send_3 (void);
void    _CbAppSim_Calling_1 (void);
void    _CbAppSim_Calling_2 (void);

void    _CbAppSim_TCP_Send_1 (sData *pData);

void    _CbAppSim_Recv_PUBACK (void);
void    _CbAppSim_Http_Head_1 (sData *pData);
void    _CbAppSim_Http_Send_1 (sData *pData);

void    _CbAppSim_Http_Send_Ex (sData *pData);
void    _CbAppSim_Http_Head_2 (void) ;

void    AppSim_GPS_OK (uint8_t *pData, uint16_t Length);
uint8_t AppSim_GPS_Extract_Lat_Long (uint8_t *pData, uint16_t Length, double *Lat, double *Long);

uint8_t AppSim_GPS_Check_Moving (void);
void    AppSim_GPS_Packet_Record (uint8_t CheckResult);
void    AppSim_GPS_Error (void);
void    AppSim_Config_Update (void); 
void    AppSim_Process_Server_DL (sData *sUartSim);

uint8_t AppSim_Power_On(void);

void    AppSim_Restart_If_PSM(void);
void    AppSim_Restart_Imediate (void);

void    AppSim_Set_Instant (void);
void    AppSim_Allow_Send(uint8_t allow);

void    AppSim_Default_Sms_Call(void);
uint8_t AppSim_Need_Sms (void);
void    AppSim_Push_Send_Sms (void);

void    AppSim_Push_Call (void);
sData*  AppSim_Get_Sim_ID (void);

#ifdef USING_AT_CONFIG
void    AppSim_SER_Get_Sim_ID (sData *str, uint16_t Pos);
#endif

#endif

