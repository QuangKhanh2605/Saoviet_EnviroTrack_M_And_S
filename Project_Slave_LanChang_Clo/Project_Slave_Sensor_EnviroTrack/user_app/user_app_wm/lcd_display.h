


#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include "user_util.h"
#include "event_driven.h"

#include "user_define.h"

#include "glcd.h"
#include "font5x7.h"
#include "button.h"
#include "Liberation_Sans17x17_Alpha.h"


/*===================Define=========================*/

#define USING_LCD_DISPLAY

#define CONNECT_DISPLAY             0x80U
#define DISCONNECT_DISPLAY          0x81U
#define COS_DISPLAY                 0x83U
#define PIN_ZERO                    0x84U
#define FONT_ETHERNET               0x88U
#define FONT_D0_C                   0x89U

#define DURATION_DISPALY            10000
#define TIME_TOGGLE                 300

/*===================extern struct, Var==============*/

typedef enum
{
    _EVENT_DISP_INIT,
    _EVENT_DISP_LOGO,
    _EVENT_DISP_SHOW,
    _EVENT_DISP_AUTO_SW,
    
    _EVENT_BUTTON_SCAN,
    _EVENT_BUTTON_DETECTTED,
    
    _EVENT_END_DISPLAY,
}sEVENT_DISPLAY;




typedef enum
{
    __PARAM_COMM,
    __SERIAL_1,
    __BATTERY_VOL,
    __POWWER_12V,
    __FREQ,
    __RSSI,
    
    __PARAM_CM44,
//    __SC1_ID_DCU,
    __SC1_CLO_DU,
    __SC1_PH_WATER,
    __SC1_NTU,
    __SC1_SALINITY,
    __SC1_TEMP,
    __SC1_EC,
    
    __CHANEL_1,
    __PULSE_1,
    __PRESS_1,
    __FLOW_1,
    
    __CHANEL_2,
    __PULSE_2,
    __PRESS_2,
    __FLOW_2,
    
    __CHANEL_3,
    __PULSE_3,
    __PRESS_3,
    __FLOW_3,
    
    __CHANEL_4,
    __PULSE_4,
    __PRESS_4,
    __FLOW_4,
    
    __CHANEL_5,
    __PULSE_5,
    __PRESS_5,
    __FLOW_5,
    
    __CHANEL_6,
    __PULSE_6,
    __PRESS_6,
    __FLOW_6,
    
    __CHANEL_7,
    __PULSE_7,
    __PRESS_7,
    __FLOW_7,
    
    __INFORMATION,
    __FW_VERSION_1,
    
    __PASS_WORD_1,
    __PASS_WORD_2,
    
    __PASS_CALIB_1,
    __PASS_CALIB_2,
    
    __SET_REQ_1,
    __SET_CALIB_SENSOR,
    __DEVICE_INFOR,
    
    __SET_PULSE_SETT,
    __SET_RESTORE_PULSE,
    __SET_PRESSURE,
    __SET_PRESS_CALIB,
    
    __SET_MANUFACTOR,
        
    __SET_REQ_2_1,
    __SET_REQ_2_2,
    
    __SET_CALIB_TITLE,
    __SET_CALIB_SS_PH,
    __SET_CALIB_SS_CLO,
    __SET_CALIB_SS_EC,
    __SET_CALIB_SS_TUR,
    __SET_CALIB_VALUE,
    
    __SET_PH_TITLE,
    __SET_PH_Z_SOLUTION,
    __SET_PH_ZERO,
    __SET_PH_S_SOLUTION,
    __SET_PH_SLOPE,
    
    __SET_CLO_TITLE,
    __SET_CLO_ZERO,
    __SET_CLO_SLOPE,
    __SET_CLO_CLB_PH_1,
    __SET_CLO_CLB_PH_2,
    __SET_CLO_CLB_TEMP,
    
    __SET_EC_TITLE,
    __SET_EC_CONST,
    __SET_EC_CALIB,
    
    __SET_TUR_TITLE_1,
    __SET_TUR_VALUE_P1,
    __SET_TUR_AD_P1,
    __SET_TUR_VALUE_P2,
    __SET_TUR_AD_P2,
    
    __SET_TUR_TITLE_2,
    __SET_TUR_VALUE_P3,
    __SET_TUR_AD_P3,
    __SET_TUR_VALUE_P4,
    __SET_TUR_AD_P4,
    
    __SET_VALUE_CLO_DU,
    __SET_VALUE_PH_WATER,
    __SET_VALUE_NTU,
    __SET_VALUE_SALINITY,
    __SET_VALUE_TEMP,
    __SET_VALUE_EC,
    
    __SET_CALIB_STATE,
    __INFOR_FW_VERSION_1,
    __INFOR_FW_VERSION_2,
    
    __SET_PRESSURE_1_1,
    __SET_PRESSURE_1_2,
    __SET_PRESSURE_1_3,
    __SET_PRESSURE_1_4,
    __SET_PRESSURE_1_5,
    __SET_PRESSURE_1_6,
    
    __SET_PRESSURE_2,
    __SET_PRESSURE_2_0,
    __SET_PRESSURE_2_1,
    __SET_PRESSURE_2_2,
    __SET_PRESSURE_2_3,
    __SET_PRESSURE_2_4,
    
    __SET_PRESSURE_2_5,
    __SET_PRESSURE_2_6,
    __SET_PRESSURE_2_7,
    
    __SET_PULSE_SET1_1,
    __SET_PULSE_SET1_2,
    __SET_PULSE_SET1_3,
    __SET_PULSE_SET1_4,

    __SET_PULSE_SET2_1,
    __SET_PULSE_SET2_2,
    
    __SET_P_RS_CONF_1,
    __SET_P_RS_CONF_2,
    
    __SET_RESTORE_1,
    
    __CAL_PRESS_CH_1,
    __CAL_PRESS_CH_2,
    __CAL_PRESS_CH_3,
    __CAL_PRESS_CH_4,
    __CAL_PRESS_CH_5,
    __CAL_PRESS_CH_6,
    
    __CAL_PRESSURE_1,
    __CAL_PRESSURE_2,
    __CAL_PRESSURE_3,
    __CAL_PRESSURE_4,
    
    __OJECT_END,
}sOJECT_WM;


typedef enum
{
    _LCD_SCREEN_1 = 1,
    _LCD_SCREEN_CM44,
    _LCD_SCREEN_2,
    _LCD_SCREEN_3,
    _LCD_SCREEN_4,
    _LCD_SCREEN_5,
    _LCD_SCREEN_6,
    _LCD_SCREEN_7,
    _LCD_SCREEN_7b,
    _LCD_SCREEN_8,
    _LCD_SCR_PASS,
    _LCD_SCR_PASSCALIB,
    _LCD_SCR_SETTING,
    _LCD_SCR_SETTING_2,
    _LCD_SCR_SET_FREQ,
    
    _LCD_SCR_SET_CALIB,
    _LCD_SCR_SET_CALIB_SS_PH,
    _LCD_SCR_SET_CALIB_SS_CLO,
    _LCD_SCR_SET_CALIB_SS_EC,
    _LCD_SCR_SET_CALIB_SS_TUR_TAB_1,
    _LCD_SCR_SET_CALIB_SS_TUR_TAB_2,
    _LCD_SCR_SET_CALIB_VALUE,
    
    _LCD_SCR_SET_CALIB_STATE,
    _LCD_SCR_INFORMATION,
    
    _LCD_SCR_SET_PRESS_1,
    _LCD_SCR_SET_PRESS_1_2, 
    _LCD_SCR_SET_PRESS_2,
    _LCD_SCR_SET_PULSE_1,
    _LCD_SCR_SET_PULSE_2,
    _LCD_SCR_PULSE_RS,
    _LCD_SCR_SET_RESTORE,
    _LCD_SCR_CAL_CHANN_1,
    _LCD_SCR_CAL_CHANN_2,
    _LCD_SCR_CAL_PRESS,
    
}sScreenWm;


typedef enum
{
    _DTYPE_U8,
    _DTYPE_I8,
    _DTYPE_U16,
    _DTYPE_I16,
    _DTYPE_U32,
    _DTYPE_I32,
    _DTYPE_CHAR,
    _DTYPE_STRING,
}sDataTypeValue;



typedef struct
{
    uint8_t     Para_u8;
    char        *sName;
    
    void        *pData;
    uint8_t     dType_u8;       //0: u8  1: char
    uint8_t     Scale_u8;
    
    char        *Unit;
    
    uint8_t     Row_u8;
    uint8_t     Col_u8;
    
    uint8_t     Mode_u8;        //Toggle Name|value
    uint8_t     Screen_u8;      //Vi tri screen hien thi
}sOjectInformation;

typedef struct
{
    uint8_t     Index_u8;       //screen index
    uint8_t     SubIndex_u8;    //screen sub
    uint8_t     Para_u8;        //tham so cua screen
    uint8_t     ParaMin_u8;     //gia tri min cua Para
    uint8_t     ParaMax_u8;     //gia tri max cua Para
    uint8_t     Flag_u8;        //Toggle flag: 0x01: name   0x2:value 
}sScreenInformation;


typedef struct
{
    uint8_t     Mode_u8;
    uint8_t     Ready_u8;
    
    sScreenInformation  sScreenNow;
    sScreenInformation  sScreenBack;
}sLCDinformation;


extern sEvent_struct        sEventDisplay[];
extern sLCDinformation      sLCD;

extern sOjectInformation   sLCDObject[];
extern uint8_t   aDISPLAY_OPERA[20];

extern char aUnitWm[5][10]; 
extern uint8_t aPASSWORD[4];
extern char AnalogType[2][10];

/*===================Function=========================*/

uint8_t Display_Task(void);
void    Display_Init (void);

void    Display_Update (void);
void    Display_Show_Oject (uint8_t object);
void    Display_Show_Screen (uint8_t screen);

uint8_t Display_Check_Toggle (uint8_t object, uint8_t Flag);

void    Display_Setup_SCREEN_Modb (void);
void    Display_Set_Screen (sScreenInformation *screen, uint8_t index, uint8_t subindex,
                             uint8_t para, uint8_t paramin, uint8_t paramax,
                             void *pData, uint8_t flag);

void    Display_Set_Screen_Flag (sScreenInformation *screen, void *pData, uint8_t flag);

uint8_t Display_Check_Password (uint8_t *pPass);
uint8_t Display_Check_PassCalib (uint8_t pPass[]);

void    Display_Process_Down_Pressure_2 (sPressureLinearInter *pPress);
void    Display_Process_Up_Pressure_2 (sPressureLinearInter *pPress);

void    Display_Show_Static_Param (void);
uint8_t Display_Show_Charge_Bat (uint8_t *level);

void    Display_Show_State_Sensor_Network (uint8_t screen);
void    Display_Show_State_Calib_Sensor (uint8_t screen);
void    Display_Show_Confirm_Calib_Sensor (uint8_t screen);

#endif