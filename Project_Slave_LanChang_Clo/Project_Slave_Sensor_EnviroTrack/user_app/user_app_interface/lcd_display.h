#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include "user_util.h"
#include "event_driven.h"

#include "user_define.h"
#include "glcd.h"
#include "font5x7.h"
#include "button.h"
#include "Liberation_Sans17x17_Alpha.h"
#include "Liberation_Sans11x14_Numbers.h"
#include "Liberation_Sans15x21_Numbers.h"

/*=========================Define=========================*/
#define USING_LCD_DISPLAY
#define CONNECT_DISPLAY             0x80U
#define DISCONNECT_DISPLAY          0x81U
#define COS_DISPLAY                 0x83U
#define PIN_ZERO                    0x84U
#define FONT_ETHERNET               0x88U
#define FONT_D0_C                   0x89U

#define DURATION_DISPALY            10000
#define TIME_TOGGLE                 300

/*========================extern struct, Var====================*/

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
    __SC1_CLO,
    __SC1_TEMP,
    
    __PASS_WORD_1,
    __PASS_WORD_2,
    
    __SCR_SET_TITLE,
    __SCR_SET_MODBUS,
    __SCR_SET_CALIB,
    __SCR_SET_OFFSET,
    __SCR_SET_INFOR,
    
    __SET_MODBUS_TITLE,
    __SET_MODBUS_ID,
    __SET_MODBUS_BR,

    __SET_CLO_TITLE,
    __SET_CLO_ZERO,
    __SET_CLO_SLOPE,
    __SET_CLO_CLB_PH_1,
    __SET_CLO_CLB_PH_2,
    __SET_CLO_CLB_TEMP,
    
    __SET_OFFSET_TITLE,
    __SET_OFFSET_CLO,
    __SET_OFFSET_TEMP,

    __SCR_INFOR_FW_VERSION_1,
    __SCR_INFOR_FW_VERSION_2,
    
    __SCR_INFOR_MODEL_1,
    __SCR_INFOR_MODEL_2,
    
    __CHECK_STATE_SETTING,
    
    __SET_RESTORE_1,
    
    __OJECT_END,
}sOJECT_WM;

typedef enum
{
    _LCD_SCREEN_1,
    _LCD_SCR_PASS,
    
    _LCD_SCR_SETTING,
    _LCD_SCR_SET_MODBUS,
    _LCD_SCR_SET_CALIB_SS_CLO,
    _LCD_SCR_SET_OFFSET,
    
    _LCD_SCR_SET_INFORMATION,
    
    _LCD_SCR_CHECK_SETTING,
    
    _LCD_SCR_SET_RESTORE,
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

typedef enum
{
    _STATE_SETTING_FREE = 0,
    _STATE_SETTING_ENTER,
    _STATE_SETTING_WAIT,
    _STATE_SETTING_DONE,
    _STATE_SETTING_ERROR,
}eKindStateSendCalib;

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

typedef struct
{
    uint8_t     State_Setting;
    uint8_t     ID_u8;
    uint32_t    Baudrate_u32;

    int32_t     Clo_Value_i32;
    int32_t     Temp_Value_i32;
    
    int32_t     Clo_Filter_i32;
    int32_t     Temp_Filter_i32;
    
    int32_t     Clo_Offset_i32;
    int32_t     temp_Offset_i32;
}sParameter_Display;

extern sEvent_struct        sEventDisplay[];
extern sLCDinformation      sLCD;

extern sOjectInformation   sLCDObject[];
extern sParameter_Display   sParaDisplay;
/*=========================Function=======================*/

uint8_t Display_Task(void);
void    Display_Init(void);

void    Display_Show_Screen (uint8_t screen);
void    Display_Show_Oject (uint8_t object);
uint8_t Display_Check_Toggle (uint8_t object, uint8_t Flag);

void    Update_ParaDisplay(void);
void    Display_Set_Screen (sScreenInformation *screen, uint8_t index, uint8_t subindex,
                             uint8_t para, uint8_t paramin, uint8_t paramax,
                             void *pData, uint8_t flag);

void    Display_Set_Screen_Flag (sScreenInformation *screen, void *pData, uint8_t flag);

uint8_t Display_Check_Password (uint8_t *pPass);

void    Display_Show_State_Sensor_Network (uint8_t screen);
void    Display_Show_State_Setting (uint8_t screen);
void    Deinit_LCD12864(void);
#endif


