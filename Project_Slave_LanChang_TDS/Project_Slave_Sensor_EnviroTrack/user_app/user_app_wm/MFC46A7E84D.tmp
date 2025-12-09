

#include "lcd_display.h"
#include "string.h"



static uint8_t _Cb_Display_Init (uint8_t event);
static uint8_t _Cb_Display_Show (uint8_t event);
static uint8_t _Cb_button_scan (uint8_t event);
static uint8_t _Cb_button_detect (uint8_t event);
static uint8_t _Cb_Display_Auto_SW (uint8_t event);
static uint8_t _Cb_Display_Logo (uint8_t event);
extern sData   sFirmVersion;


sEvent_struct sEventDisplay [] =
{
    { _EVENT_DISP_INIT, 		    0, 0, 500, 	    _Cb_Display_Init }, 
    { _EVENT_DISP_LOGO, 		    1, 0, 2000, 	_Cb_Display_Logo}, 
    { _EVENT_DISP_SHOW, 		    0, 0, 100,      _Cb_Display_Show }, 
    { _EVENT_DISP_AUTO_SW, 		    0, 0, 5000,     _Cb_Display_Auto_SW }, 
    
    { _EVENT_BUTTON_SCAN, 		    0, 0, 2,    	_Cb_button_scan  },
    { _EVENT_BUTTON_DETECTTED, 	    0, 0, 10, 		_Cb_button_detect },
};
         

sLCDinformation      sLCD;

char aUnitWm[5][10] =  
{
    {"mV"},
    {"mA"},
    {"V"},
    {"met"},
    {"bar"},
};

char AnalogType[2][10] = { "pressure", "level"};

uint8_t aPASSWORD[4] = {"1111"};
uint8_t aPASSCALIB[4] = {"2605"};

uint8_t aSTT_CALIB_FREE[14]   = {"              "};
uint8_t aSTT_CALIB_ENTER[14]  = {"Enter to Calib"};
uint8_t aSTT_CALIB_WAIT[14]   = {"    Waiting   "};
uint8_t aSTT_CALIB_DONE[14]   = {"     Done    "};
uint8_t aSTT_CALIB_ERROR[14]  = {"     Error   "};

sOjectInformation   sLCDObject[] = 
{
//          para          name                  value      dtype         scale   unit      row  col      screen
    {   __PARAM_COMM,       "Pcomm.",           NULL,   _DTYPE_STRING,   0,      NULL,      0,  0,  0x00,    _LCD_SCREEN_1    },
    {   __SERIAL_1,         NULL,               NULL,   _DTYPE_STRING,   0,      NULL,      2,  20, 0x00,    _LCD_SCREEN_1    },
    {   __BATTERY_VOL,      "V_bat: ",          NULL,   _DTYPE_U32,      0xFD,   " (V)",    3,  0,  0x00,    _LCD_SCREEN_1    },
    {   __POWWER_12V,       "V_pow: ",          NULL,   _DTYPE_U32,      0xFD,   " (V)",    4,  0,  0x00,    _LCD_SCREEN_1    },
    {   __FREQ,             "Tsend: ",          NULL,   _DTYPE_U16,      0,      " (min)",  5,  0,  0x00,    _LCD_SCREEN_1    },
    {   __RSSI,             "Rssi:  -",         NULL,   _DTYPE_U8,       0,      " (dbm)",  6,  0,  0x00,    _LCD_SCREEN_1    },
    
    {   __PARAM_CM44,       "Sensor.",            NULL,   _DTYPE_STRING,   0,      NULL,      0,  0,  0x00,    _LCD_SCREEN_CM44    },
//    {   __SC1_ID_DCU,       NULL,               NULL,   _DTYPE_STRING,   0,      NULL,      2,  20, 0x00,    _LCD_SCREEN_CM44    },
    {   __SC1_CLO_DU,       "CLO DU: ",         NULL,   _DTYPE_I16,      0xFE,   "  mg/L",  2,  0,  0x00,    _LCD_SCREEN_CM44    },
    {   __SC1_PH_WATER,     "PH ATC: ",         NULL,   _DTYPE_I16,      0xFE,   "  pH",    3,  0,  0x00,    _LCD_SCREEN_CM44    },
    {   __SC1_NTU,          "NTU   : ",         NULL,   _DTYPE_I16,      0xFE,   "  NTU",   4,  0,  0x00,    _LCD_SCREEN_CM44    },
    {   __SC1_SALINITY,     "Do man: ",         NULL,   _DTYPE_I16,      0xFE,   "  %",     5,  0,  0x00,    _LCD_SCREEN_CM44    },
    {   __SC1_TEMP,         "Temp  : ",         NULL,   _DTYPE_I16,      0xFE,   "  �C",    6,  0,  0x00,    _LCD_SCREEN_CM44    },
    {   __SC1_EC,           "EC    : ",         NULL,   _DTYPE_I16,      0xFE,   "  uS/cm", 7,  0,  0x00,    _LCD_SCREEN_CM44    },
    //screen channel 1

    {   __CHANEL_1,         "CH.1  ",           NULL,   _DTYPE_STRING,   0,      NULL,      0,  0,  0x00,    _LCD_SCREEN_2    },
    {   __PULSE_1,          "To: ",             NULL,   _DTYPE_U32,      0,      " (m3)",   2,  0,  0x00,    _LCD_SCREEN_2    },
    {   __PRESS_1,          "P : ",             NULL,   _DTYPE_I32,      0xFD,   " (bar)",  3,  0,  0x00,    _LCD_SCREEN_2    },
    {   __FLOW_1,           "F : ",             NULL,   _DTYPE_I32,      0xFE,   " (m3/h)", 4,  0,  0x00,    _LCD_SCREEN_2    },
    
    {   __CHANEL_2,         "CH.2  ",           NULL,   _DTYPE_STRING,   0,      NULL,      0,  0,  0xF0,    _LCD_SCREEN_3    },
    {   __PULSE_2,          "To: ",             NULL,   _DTYPE_U32,      0,      " (m3)",   2,  0,  0x00,    _LCD_SCREEN_3    },
    {   __PRESS_2,          "P : ",             NULL,   _DTYPE_I32,      0xFD,   " (bar)",  3,  0,  0x00,    _LCD_SCREEN_3    },
    {   __FLOW_2,           "F : ",             NULL,   _DTYPE_I32,      0xFE,   " (m3/h)", 4,  0,  0x00,    _LCD_SCREEN_3    },
    
    {   __CHANEL_3,         "CH.3  ",           NULL,   _DTYPE_STRING,   0,      NULL,      0,  0,  0x00,    _LCD_SCREEN_4    },
    {   __PULSE_3,          "To: ",             NULL,   _DTYPE_U32,      0,      " (m3)",   2,  0,  0x00,    _LCD_SCREEN_4    },
    {   __PRESS_3,          "P : ",             NULL,   _DTYPE_I32,      0xFD,   " (bar)",  3,  0,  0x00,    _LCD_SCREEN_4    },
    {   __FLOW_3,           "F : ",             NULL,   _DTYPE_I32,      0xFE,   " (m3/h)", 4,  0,  0x00,    _LCD_SCREEN_4    },
    
    {   __CHANEL_4,         "CH.4  ",           NULL,   _DTYPE_STRING,   0,      NULL,      0,  0,  0x00,    _LCD_SCREEN_5    },
    {   __PULSE_4,          "To: ",             NULL,   _DTYPE_U32,      0,      " (m3)",   2,  0,  0x00,    _LCD_SCREEN_5    },
    {   __PRESS_4,          "P : ",             NULL,   _DTYPE_I32,      0xFD,   " (bar)",  3,  0,  0x00,    _LCD_SCREEN_5    },
    {   __FLOW_4,           "F : ",             NULL,   _DTYPE_I32,      0xFE,   " (m3/h)", 4,  0,  0x00,    _LCD_SCREEN_5    },
    
    {   __CHANEL_5,         "CH.5  ",           NULL,   _DTYPE_STRING,   0,      NULL,      0,  0,  0x00,    _LCD_SCREEN_6    },
    {   __PULSE_5,          "To: ",             NULL,   _DTYPE_U32,      0,      " (m3)",   2,  0,  0x00,    _LCD_SCREEN_6    },
    {   __PRESS_5,          "P : ",             NULL,   _DTYPE_I32,      0xFD,   " (bar)",  3,  0,  0x00,    _LCD_SCREEN_6    },
    {   __FLOW_5,           "F : ",             NULL,   _DTYPE_I32,      0xFE,   " (m3/h)", 4,  0,  0x00,    _LCD_SCREEN_6    },
    
    {   __CHANEL_6,         "CH.6  ",           NULL,   _DTYPE_STRING,   0,      NULL,      0,  0,  0x00,    _LCD_SCREEN_7    },
    {   __PULSE_6,          "To: ",             NULL,   _DTYPE_U32,      0,      " (m3)",   2,  0,  0x00,    _LCD_SCREEN_7    },
    {   __PRESS_6,          "P : ",             NULL,   _DTYPE_I32,      0xFD,   " (bar)",  3,  0,  0x00,    _LCD_SCREEN_7    },
    {   __FLOW_6,           "F : ",             NULL,   _DTYPE_I32,      0xFE,   " (m3/h)", 4,  0,  0x00,    _LCD_SCREEN_7    },
    
    {   __CHANEL_7,         "CH.7  ",           NULL,   _DTYPE_STRING,   0,      NULL,      0,  0,  0x00,    _LCD_SCREEN_7b    },
    {   __PULSE_7,          "To: ",             NULL,   _DTYPE_U32,      0,      " (m3)",   2,  0,  0x00,    _LCD_SCREEN_7b    },
    {   __PRESS_7,          "P : ",             NULL,   _DTYPE_I32,      0xFD,   " (bar)",  3,  0,  0x00,    _LCD_SCREEN_7b    },
    {   __FLOW_7,           "F : ",             NULL,   _DTYPE_I32,      0xFE,   " (m3/h)", 4,  0,  0x00,    _LCD_SCREEN_7b    },
    
    {   __INFORMATION,      "Infor.",           NULL,   _DTYPE_STRING,   0,      NULL,      0,  0,  0x00,    _LCD_SCREEN_8    },
    {   __FW_VERSION_1,     "Ver:  ",           NULL,   _DTYPE_STRING,   0,      NULL,      2,  0,  0x00,    _LCD_SCREEN_8    },
    
    {   __PASS_WORD_1,      "Enter Password",   NULL,   _DTYPE_STRING,   0,      NULL,      2,  24, 0x00,    _LCD_SCR_PASS    },
    {   __PASS_WORD_2,      NULL,               NULL,   _DTYPE_STRING,   0,      NULL,      3,  48, 0x00,    _LCD_SCR_PASS    },
    
    {   __PASS_CALIB_1,     "Enter Passcalib",  NULL,   _DTYPE_STRING,   0,      NULL,      2,  24, 0x00,    _LCD_SCR_PASSCALIB},
    {   __PASS_CALIB_2,     NULL,               NULL,   _DTYPE_STRING,   0,      NULL,      3,  54, 0x00,    _LCD_SCR_PASSCALIB},
    
    {   __SET_REQ_1,        "1.Tsend Data",     NULL,   _DTYPE_STRING,   0,      NULL,      2,  14, 0x00,    _LCD_SCR_SETTING },
    {   __SET_CALIB_SENSOR, "2.Calib Sensor",   NULL,   _DTYPE_STRING,   0,      NULL,      3,  14, 0x00,    _LCD_SCR_SETTING },
    {   __DEVICE_INFOR,     "3.Information",    NULL,   _DTYPE_STRING,   0,      NULL,      4,  14, 0x00,    _LCD_SCR_SETTING },
    
    {   __SET_PULSE_SETT,   NULL,  NULL,   _DTYPE_STRING,   0,      NULL,      2,  24, 0x00,    _LCD_SCR_SETTING },
    {   __SET_RESTORE_PULSE,NULL,  NULL,   _DTYPE_STRING,   0,      NULL,      3,  24, 0x00,    _LCD_SCR_SETTING },
    {   __SET_PRESSURE,     NULL,   NULL,   _DTYPE_STRING,   0,      NULL,      4,  24, 0x00,    _LCD_SCR_SETTING },
    {   __SET_PRESS_CALIB,  NULL, NULL,   _DTYPE_STRING,   0,      NULL,      5,  24, 0x00,    _LCD_SCR_SETTING },
    
    {   __SET_MANUFACTOR,   NULL,    NULL,   _DTYPE_STRING,   0,      NULL,      1,  24, 0x00,    _LCD_SCR_SETTING_2 },
    
    {   __SET_REQ_2_1,      "*Tsend Data:",     NULL,   _DTYPE_STRING,   0,      NULL,      2,  24, 0x00,    _LCD_SCR_SET_FREQ },
    {   __SET_REQ_2_2,      NULL,               NULL,   _DTYPE_U32,      0,      " (min)",  3,  36, 0x02,    _LCD_SCR_SET_FREQ },
    
    {   __SET_CALIB_TITLE,  "Calib Sensor",     NULL,   _DTYPE_STRING,   0,     NULL,       2,  30, 0x00,    _LCD_SCR_SET_CALIB},
    {   __SET_CALIB_SS_PH,  "1.pH",             NULL,   _DTYPE_STRING,   0,     NULL,       3,  14, 0x00,    _LCD_SCR_SET_CALIB},
    {   __SET_CALIB_SS_CLO, "2.Chlorine",       NULL,   _DTYPE_STRING,   0,     NULL,       4,  14, 0x00,    _LCD_SCR_SET_CALIB},
    {   __SET_CALIB_SS_EC,  "3.EC",             NULL,   _DTYPE_STRING,   0,     NULL,       5,  14, 0x00,    _LCD_SCR_SET_CALIB},
    {   __SET_CALIB_SS_TUR, "4.Turbididy",      NULL,   _DTYPE_STRING,   0,     NULL,       6,  14, 0x00,    _LCD_SCR_SET_CALIB},
    
    {   __SET_PH_TITLE,     "pH         AD: ",  NULL,   _DTYPE_U32,      0x00,  NULL,       2,  5,  0x00,    _LCD_SCR_SET_CALIB_SS_PH},
    {   __SET_PH_Z_SOLUTION,"1.Z_Solution: ",   NULL,   _DTYPE_I16,      0xFE,  NULL,       3,  14, 0x00,    _LCD_SCR_SET_CALIB_SS_PH},
    {   __SET_PH_ZERO,      "2.Zero",           NULL,   _DTYPE_STRING,   0,     NULL,       4,  14, 0x00,    _LCD_SCR_SET_CALIB_SS_PH},
    {   __SET_PH_S_SOLUTION,"3.S_Solution: ",   NULL,   _DTYPE_I16,      0xFE,  NULL,       5,  14, 0x00,    _LCD_SCR_SET_CALIB_SS_PH},
    {   __SET_PH_SLOPE,     "4.Slope",          NULL,   _DTYPE_STRING,   0,     NULL,       6,  14, 0x00,    _LCD_SCR_SET_CALIB_SS_PH},
    
    {   __SET_CLO_TITLE,    "Chlorine   AD: ",  NULL,   _DTYPE_U32,      0x00,  NULL,       2,  5,  0x00,    _LCD_SCR_SET_CALIB_SS_CLO},
    {   __SET_CLO_SOLUTION, "1.Solution: ",     NULL,   _DTYPE_I16,      0xFE,  NULL,       3,  14, 0x00,    _LCD_SCR_SET_CALIB_SS_CLO},
    {   __SET_CLO_ZERO,     "2.Zero",           NULL,   _DTYPE_STRING,   0,     NULL,       4,  14, 0x00,    _LCD_SCR_SET_CALIB_SS_CLO},
    {   __SET_CLO_SLOPE,    "3.Slope",          NULL,   _DTYPE_STRING,   0,     NULL,       5,  14, 0x00,    _LCD_SCR_SET_CALIB_SS_CLO},
    
    {   __SET_EC_TITLE,     "EC         AD: ",  NULL,   _DTYPE_I16,      0x00,  NULL,       2,  5,  0x00,    _LCD_SCR_SET_CALIB_SS_EC},
    {   __SET_EC_CONST,     "1.Const: ",        NULL,   _DTYPE_I16,      0x00,  NULL,       3,  14, 0x00,    _LCD_SCR_SET_CALIB_SS_EC},
    {   __SET_EC_CALIB,     "2.Calib: ",        NULL,   _DTYPE_I16,      0x00,  NULL,       4,  14, 0x00,    _LCD_SCR_SET_CALIB_SS_EC},
    
    {   __SET_TUR_TITLE_1,  "Tur    AD:",       NULL,   _DTYPE_U32,      0x00,  NULL,       2,  5,  0x00,    _LCD_SCR_SET_CALIB_SS_TUR_TAB_1},
    {   __SET_TUR_VALUE_P1, "1.1st NTU: ",      NULL,   _DTYPE_I16,      0xFE,  NULL,       3,  14, 0x00,    _LCD_SCR_SET_CALIB_SS_TUR_TAB_1},
    {   __SET_TUR_AD_P1,    "2.1st AD : ",      NULL,   _DTYPE_U32,      0x00,  NULL,       4,  14, 0x00,    _LCD_SCR_SET_CALIB_SS_TUR_TAB_1},
    {   __SET_TUR_VALUE_P2, "3.2nd NTU: ",      NULL,   _DTYPE_I16,      0xFE,  NULL,       5,  14, 0x00,    _LCD_SCR_SET_CALIB_SS_TUR_TAB_1},
    {   __SET_TUR_AD_P2,    "4.2nd AD : ",      NULL,   _DTYPE_U32,      0x00,  NULL,       6,  14, 0x00,    _LCD_SCR_SET_CALIB_SS_TUR_TAB_1},
    
    {   __SET_TUR_TITLE_2,  "Tur    AD:",       NULL,   _DTYPE_U32,      0x00,  NULL,       2,  5, 0x00,    _LCD_SCR_SET_CALIB_SS_TUR_TAB_2},
    {   __SET_TUR_VALUE_P3, "5.3rd NTU: ",      NULL,   _DTYPE_I16,      0xFE,  NULL,       3,  14, 0x00,    _LCD_SCR_SET_CALIB_SS_TUR_TAB_2},
    {   __SET_TUR_AD_P3,    "6.3rd AD : ",      NULL,   _DTYPE_U32,      0x00,  NULL,       4,  14, 0x00,    _LCD_SCR_SET_CALIB_SS_TUR_TAB_2},
    {   __SET_TUR_VALUE_P4, "7.4th NTU: ",      NULL,   _DTYPE_I16,      0xFE,  NULL,       5,  14, 0x00,    _LCD_SCR_SET_CALIB_SS_TUR_TAB_2},
    {   __SET_TUR_AD_P4,    "8.4th AD : ",      NULL,   _DTYPE_U32,      0x00,  NULL,       6,  14, 0x00,    _LCD_SCR_SET_CALIB_SS_TUR_TAB_2},
    
    {   __SET_CALIB_STATE,  NULL,               NULL,   _DTYPE_STRING,   0,     NULL,       3,  24, 0x00,    _LCD_SCR_SET_CALIB_STATE},
    
    {   __INFOR_FW_VERSION_1,"*Version",        NULL,   _DTYPE_STRING,   0,     NULL,      2,  24, 0x00,     _LCD_SCR_INFORMATION },
    {   __INFOR_FW_VERSION_2, NULL,             NULL,   _DTYPE_STRING,   0,     NULL,      3,  5,  0x00,     _LCD_SCR_INFORMATION },
    
    {   __SET_PRESSURE_1_1, "1.Pressure CH.1",  NULL,   _DTYPE_STRING,   0,      NULL,      1,  24, 0x00,    _LCD_SCR_SET_PRESS_1 },
    {   __SET_PRESSURE_1_2, "2.Pressure CH.2",  NULL,   _DTYPE_STRING,   0,      NULL,      2,  24, 0x00,    _LCD_SCR_SET_PRESS_1 },
    {   __SET_PRESSURE_1_3, "3.Pressure CH.3",  NULL,   _DTYPE_STRING,   0,      NULL,      3,  24, 0x00,    _LCD_SCR_SET_PRESS_1 },
    {   __SET_PRESSURE_1_4, "4.Pressure CH.4",  NULL,   _DTYPE_STRING,   0,      NULL,      4,  24, 0x00,    _LCD_SCR_SET_PRESS_1 },
    {   __SET_PRESSURE_1_5, "5.Pressure CH.5",  NULL,   _DTYPE_STRING,   0,      NULL,      5,  24, 0x00,    _LCD_SCR_SET_PRESS_1 },
    
    {   __SET_PRESSURE_1_6, "6.Pressure CH.6",  NULL,   _DTYPE_STRING,   0,      NULL,      1,  24, 0x00,    _LCD_SCR_SET_PRESS_1_2 },
     
    {   __SET_PRESSURE_2,   "*Linear Inter:",   NULL,   _DTYPE_STRING,   0,      NULL,      1,  24, 0x00,    _LCD_SCR_SET_PRESS_2 },
    {   __SET_PRESSURE_2_0, "Type: ",           NULL,   _DTYPE_STRING,   0,      NULL,      2,  0,  0x02,    _LCD_SCR_SET_PRESS_2 },
    {   __SET_PRESSURE_2_1, "Factor: ",         NULL,   _DTYPE_U16,      0,      NULL,      3,  0,  0x02,    _LCD_SCR_SET_PRESS_2 },
    {   __SET_PRESSURE_2_2, "In : ",            NULL,   _DTYPE_U16,      0,      NULL,      4,  0,  0x02,    _LCD_SCR_SET_PRESS_2 },
    {   __SET_PRESSURE_2_3, " - ",              NULL,   _DTYPE_U16,      0,      NULL,      4,  54, 0x02,    _LCD_SCR_SET_PRESS_2 },
    {   __SET_PRESSURE_2_4, NULL,               NULL,   _DTYPE_STRING,   0,      NULL,      4,  108,0x02,    _LCD_SCR_SET_PRESS_2 },
    
    {   __SET_PRESSURE_2_5, "Out: ",            NULL,   _DTYPE_U16,      0,      NULL,      5,  0,  0x02,    _LCD_SCR_SET_PRESS_2 },
    {   __SET_PRESSURE_2_6, " - ",              NULL,   _DTYPE_U16,      0,      NULL,      5,  54, 0x02,    _LCD_SCR_SET_PRESS_2 },
    {   __SET_PRESSURE_2_7, NULL,               NULL,   _DTYPE_STRING,   0,      NULL,      5,  108,0x02,    _LCD_SCR_SET_PRESS_2 },
    
    //cai dat he so xung
    {   __SET_PULSE_SET1_1, "1.Pulse CH.1",     NULL,   _DTYPE_STRING,   0,      NULL,      1,  24, 0x00,    _LCD_SCR_SET_PULSE_1 },
    {   __SET_PULSE_SET1_2, "2.Pulse CH.2",     NULL,   _DTYPE_STRING,   0,      NULL,      2,  24, 0x00,    _LCD_SCR_SET_PULSE_1 },
    {   __SET_PULSE_SET1_3, "3.Pulse CH.3",     NULL,   _DTYPE_STRING,   0,      NULL,      3,  24, 0x00,    _LCD_SCR_SET_PULSE_1 },
    {   __SET_PULSE_SET1_4, "4.Pulse CH.4",     NULL,   _DTYPE_STRING,   0,      NULL,      4,  24, 0x00,    _LCD_SCR_SET_PULSE_1 },
    
    //
    {   __SET_PULSE_SET2_1, "Factor: ",         NULL,   _DTYPE_U8,       0,      NULL,      2,  0,  0x00,    _LCD_SCR_SET_PULSE_2 },
    {   __SET_PULSE_SET2_2, "Start: ",          NULL,   _DTYPE_STRING,   0,      NULL,      3,  0,  0x02,    _LCD_SCR_SET_PULSE_2 },
    
    {   __SET_P_RS_CONF_1,  "Pulse Restore?",   NULL,   _DTYPE_STRING,   0,      NULL,      2,  24, 0x00,    _LCD_SCR_PULSE_RS },
    {   __SET_P_RS_CONF_2,  NULL,               NULL,   _DTYPE_STRING,   0,      NULL,      3,  54, 0x00,    _LCD_SCR_PULSE_RS },
    
    {   __SET_RESTORE_1,    "Successfully!",    NULL,   _DTYPE_STRING,   0,      NULL,      3,  24, 0x02,    _LCD_SCR_SET_RESTORE},
    
    {   __CAL_PRESS_CH_1,   "1.Calib P_CH.1",   NULL,   _DTYPE_STRING,   0,      NULL,      1,  24, 0x00,    _LCD_SCR_CAL_CHANN_1 },
    {   __CAL_PRESS_CH_2,   "2.Calib P_CH.2",   NULL,   _DTYPE_STRING,   0,      NULL,      2,  24, 0x00,    _LCD_SCR_CAL_CHANN_1 },
    {   __CAL_PRESS_CH_3,   "3.Calib P_CH.3",   NULL,   _DTYPE_STRING,   0,      NULL,      3,  24, 0x00,    _LCD_SCR_CAL_CHANN_1 },
    {   __CAL_PRESS_CH_4,   "4.Calib P_CH.4",   NULL,   _DTYPE_STRING,   0,      NULL,      4,  24, 0x00,    _LCD_SCR_CAL_CHANN_1 },
    {   __CAL_PRESS_CH_5,   "5.Calib P_CH.5",   NULL,   _DTYPE_STRING,   0,      NULL,      5,  24, 0x00,    _LCD_SCR_CAL_CHANN_1 },
    
    {   __CAL_PRESS_CH_6,   "6.Calib P_CH.6",   NULL,   _DTYPE_STRING,   0,      NULL,      1,  24, 0x00,    _LCD_SCR_CAL_CHANN_2 },
    
    {   __CAL_PRESSURE_1,   "Cur Calib: ",      NULL,   _DTYPE_I16,      0,      " (mV)",   1,  0, 0x00,    _LCD_SCR_CAL_PRESS }, 
    {   __CAL_PRESSURE_2,   "Vadc: ",           NULL,   _DTYPE_I16,      0,      " (mV)",   2,  0, 0x02,    _LCD_SCR_CAL_PRESS },
    
    {   __CAL_PRESSURE_3,   "*Take Input 0 (V)",NULL,   _DTYPE_STRING,   0,      NULL,      4,  0, 0x00,    _LCD_SCR_CAL_PRESS },
    {   __CAL_PRESSURE_4, "Press Enter To Save",NULL,   _DTYPE_STRING,   0,      NULL,      5,  6, 0x00,    _LCD_SCR_CAL_PRESS }, 
};

static char charNotDetectPress = '-';

/*===================Function=========================*/
void Display_Init (void)
{
    static uint32_t TempPulse_u32 = 0;
    //Init
    HAL_GPIO_WritePin (LCD_ON_OFF_GPIO_Port, LCD_ON_OFF_Pin, GPIO_PIN_SET);   
    HAL_GPIO_WritePin (LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin (LCD_C86_GPIO_Port, LCD_C86_Pin, GPIO_PIN_RESET); 
    
    glcd_init();
    
    //Gan cac bien data cho truong thong tin
    sLCDObject[__BATTERY_VOL].pData    = &sBattery.mVol_u32; 
    sLCDObject[__POWWER_12V].pData    = &sVout.mVol_u32;   
    sLCDObject[__RSSI].pData    = &sSimCommInfor.RSSI_u8;   
    sLCDObject[__FREQ].pData    = &sModemInfor.sFrequence.DurOnline_u32;    
    
//    sLCDObject[__SC1_ID_DCU].pData      = sModemInfor.sId.Data_a8;
    sLCDObject[__SC1_CLO_DU].pData      = &sDataSensorMeasure.sClo_Du.Value; 
    sLCDObject[__SC1_CLO_DU].Scale_u8   = sDataSensorMeasure.sClo_Du.Scale; 
    sLCDObject[__SC1_PH_WATER].pData    = &sDataSensorMeasure.spH_Water.Value;    
    sLCDObject[__SC1_PH_WATER].Scale_u8 = sDataSensorMeasure.spH_Water.Scale;    
    sLCDObject[__SC1_NTU].pData         = &sDataSensorMeasure.sNTU.Value;   
    sLCDObject[__SC1_NTU].Scale_u8      = sDataSensorMeasure.sNTU.Scale;   
    sLCDObject[__SC1_SALINITY].pData    = &sDataSensorMeasure.sSalinity.Value;    
    sLCDObject[__SC1_SALINITY].Scale_u8 = sDataSensorMeasure.sSalinity.Scale; 
    sLCDObject[__SC1_TEMP].pData        = &sDataSensorMeasure.sTemperature.Value;    
    sLCDObject[__SC1_TEMP].Scale_u8     = sDataSensorMeasure.sTemperature.Scale;   
    sLCDObject[__SC1_EC].pData          = &sDataSensorMeasure.sEC.Value;   
    sLCDObject[__SC1_EC].Scale_u8       = sDataSensorMeasure.sEC.Scale;   
    
    sLCDObject[__SET_PH_TITLE].pData        = &sSensor_pH.Measure_AD; 
    sLCDObject[__SET_PH_Z_SOLUTION].pData   = &sSensor_pH.sZero_Calib.Value ; 
    sLCDObject[__SET_PH_ZERO].Scale_u8      = sSensor_pH.sZero_Calib.Scale; 
    sLCDObject[__SET_PH_S_SOLUTION].pData   = &sSensor_pH.sSlope_Calib.Value; 
    sLCDObject[__SET_PH_SLOPE].Scale_u8     = sSensor_pH.sSlope_Calib.Scale; 
    
    sLCDObject[__SET_CLO_TITLE].pData    = &sSensor_Clo.Measure_AD; 
    sLCDObject[__SET_CLO_SOLUTION].pData = &sSensor_Clo.sSolution_Calibration.Value; 
    sLCDObject[__SET_CLO_SOLUTION].Scale_u8 = sSensor_Clo.sSolution_Calibration.Scale; 
    
    sLCDObject[__SET_EC_TITLE].pData     = &sSensor_EC.sConductivity_Value.Value;
    sLCDObject[__SET_EC_TITLE].Scale_u8  = sSensor_EC.sConductivity_Value.Scale;
    sLCDObject[__SET_EC_CONST].pData     = &sSensor_EC.sConductivity_Const.Value;
    sLCDObject[__SET_EC_CONST].Scale_u8  = sSensor_EC.sConductivity_Const.Scale;
    sLCDObject[__SET_EC_CALIB].pData     = &sSensor_EC.sConductivity_Calib.Value;
    sLCDObject[__SET_EC_CALIB].Scale_u8  = sSensor_EC.sConductivity_Calib.Scale;
    
    sLCDObject[__SET_TUR_TITLE_1].pData      = &sSensor_Turbidity.Measure_AD;
    sLCDObject[__SET_TUR_VALUE_P1].pData     = &sSensor_Turbidity.sFirst_Calibration.Value;
    sLCDObject[__SET_TUR_VALUE_P1].Scale_u8  = sSensor_Turbidity.sFirst_Calibration.Scale;
    sLCDObject[__SET_TUR_AD_P1].pData        = &sSensor_Turbidity.First_AD_u32;
    sLCDObject[__SET_TUR_VALUE_P2].pData     = &sSensor_Turbidity.sSecond_Calibration.Value;
    sLCDObject[__SET_TUR_VALUE_P2].Scale_u8  = sSensor_Turbidity.sSecond_Calibration.Scale;
    sLCDObject[__SET_TUR_AD_P2].pData        = &sSensor_Turbidity.Second_AD_u32;
    
    sLCDObject[__SET_TUR_TITLE_2].pData      = &sSensor_Turbidity.Measure_AD;
    sLCDObject[__SET_TUR_VALUE_P3].pData     = &sSensor_Turbidity.sThird_Calibration.Value;
    sLCDObject[__SET_TUR_VALUE_P3].Scale_u8  = sSensor_Turbidity.sThird_Calibration.Scale;
    sLCDObject[__SET_TUR_AD_P3].pData        = &sSensor_Turbidity.Third_AD_u32;
    sLCDObject[__SET_TUR_VALUE_P4].pData     = &sSensor_Turbidity.sFourth_Calibration.Value;
    sLCDObject[__SET_TUR_VALUE_P4].Scale_u8  = sSensor_Turbidity.sFourth_Calibration.Scale;
    sLCDObject[__SET_TUR_AD_P4].pData        = &sSensor_Turbidity.Fourth_AD_u32;
    
    sLCDObject[__INFOR_FW_VERSION_2].pData   = sFirmVersion.Data_a8;
    
    sLCDObject[__PULSE_1].pData =  &sPulse[0].Total_u32;
    sLCDObject[__PRESS_1].pData =  &sWmVar.aPRESSURE[0].Val_i32;
    sLCDObject[__FLOW_1].pData =  &sPulse[0].Flow_i32;
    
    sLCDObject[__PULSE_2].pData =  &sPulse[1].Total_u32;
    sLCDObject[__PRESS_2].pData =  &sWmVar.aPRESSURE[1].Val_i32;
    sLCDObject[__FLOW_2].pData =  &sPulse[1].Flow_i32;
    
    sLCDObject[__PULSE_3].pData =  &sPulse[2].Total_u32;
    sLCDObject[__PRESS_3].pData =  &sWmVar.aPRESSURE[2].Val_i32;
    sLCDObject[__FLOW_3].pData =  &sPulse[2].Flow_i32;
    
    sLCDObject[__PULSE_4].pData =  &sPulse[3].Total_u32;
    sLCDObject[__PRESS_4].pData =  &sWmVar.aPRESSURE[3].Val_i32;
    sLCDObject[__FLOW_4].pData =  &sPulse[3].Flow_i32;
    
    sLCDObject[__PULSE_5].pData =  &TempPulse_u32;
    sLCDObject[__PRESS_5].pData =  &sWmVar.aPRESSURE[4].Val_i32;
    sLCDObject[__FLOW_5].pData =  &TempPulse_u32;
    
    sLCDObject[__PULSE_6].pData =  &TempPulse_u32;
    sLCDObject[__PRESS_6].pData =  &sWmVar.aPRESSURE[5].Val_i32;
    sLCDObject[__FLOW_6].pData =  &TempPulse_u32;
    
    sLCDObject[__SERIAL_1].pData        = sModemInfor.sId.Data_a8;
    sLCDObject[__FW_VERSION_1].pData    = sFirmVersion.Data_a8 + 5;
    //
    sLCD.sScreenNow.Index_u8 = _LCD_SCREEN_1; 
    sLCD.sScreenNow.Para_u8 = 0xFF;
    sLCD.sScreenNow.ParaMin_u8 = 0;
    sLCD.sScreenNow.ParaMax_u8 = 0;
}



uint8_t Display_Task(void)
{
	uint8_t i = 0;

	for (i = 0; i < _EVENT_END_DISPLAY; i++)
	{
		if (sEventDisplay[i].e_status == 1)
		{
			if ((sEventDisplay[i].e_systick == 0) ||
					((HAL_GetTick() - sEventDisplay[i].e_systick)  >=  sEventDisplay[i].e_period))
			{
                sEventDisplay[i].e_status = 0;
				sEventDisplay[i].e_systick = HAL_GetTick();
				sEventDisplay[i].e_function_handler(i);
			}
		}
	}
    
	return 0;
}



/*----------- Func callback ------------*/
static uint8_t _Cb_Display_Init (uint8_t event)
{
    if (sLCD.Ready_u8 == false)
    {
        UTIL_Printf_Str(DBLEVEL_M, "u_lcd: init...\r\n");
        
        HAL_GPIO_WritePin (LCD_ON_OFF_GPIO_Port, LCD_ON_OFF_Pin, GPIO_PIN_SET);   
        HAL_GPIO_WritePin (LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin (LCD_C86_GPIO_Port, LCD_C86_Pin, GPIO_PIN_RESET); 
        
        glcd_init();
        glcd_tiny_set_font(Font5x7, 5, 7, 32, 127 + 10);
        
        sLCD.Ready_u8 = true;
            
        fevent_active(sEventDisplay, _EVENT_DISP_SHOW); 
    }
    
	return 1;
}


static uint8_t _Cb_Display_Logo (uint8_t event)
{
    static uint8_t step_u8 = 0;
         
    switch (step_u8)
    {
        case 0:
            glcd_test_bitmap_128x64();
            break;
        case 1:
            glcd_clear_buffer();
            glcd_set_font(Liberation_Sans17x17_Alpha, 17, 17, 65, 90);
            
            glcd_draw_string_xy(30, 13, "SV");
            
            glcd_set_pixel(60, 25, BLACK);
            glcd_set_pixel(60, 26, BLACK);
            glcd_set_pixel(61, 26, BLACK);
            glcd_set_pixel(61, 25, BLACK);
            
            glcd_draw_string_xy(65, 13, "JSC");
            
            glcd_tiny_set_font(Font5x7, 5, 7, 32, 127 + 10);
            glcd_tiny_draw_string(9, 4, "CHAT LUONG LA NIEM");
            glcd_tiny_draw_string(54, 5, "TIN");
            
            glcd_draw_rect(0, 0, 128, 64,BLACK);
            
            glcd_write();	
            break;
        default:
            sLCD.Ready_u8 = true;
            glcd_tiny_set_font(Font5x7, 5, 7, 32, 127 + 10);
            fevent_active(sEventDisplay, _EVENT_DISP_SHOW); 
            fevent_active(sEventDisplay, _EVENT_DISP_AUTO_SW); 
            fevent_active(sEventDisplay, _EVENT_BUTTON_SCAN); 
            return 1;
    }
    step_u8++;
    fevent_enable(sEventDisplay, event); 
    
    return 1;
}



static uint8_t _Cb_Display_Show (uint8_t event)
{
    static uint8_t ScreenLast = 0;
    static uint32_t LandMarkChange_u32 = 0;
    
    //update cac bien moi cai dat
    Display_Update();
    //hien thi man hinh: index
    if (sLCD.Ready_u8 == true) {
        //neu index 9: modbus disp: gan lai cac bien va subindex
        if (sLCD.sScreenNow.Index_u8 == _LCD_SCREEN_7b) {
            Display_Setup_SCREEN_Modb();
        }
        
        //hien thị
        Display_Show_Screen(sLCD.sScreenNow.Index_u8);
    } else {
        fevent_active(sEventDisplay, _EVENT_DISP_INIT); 
    }
    
    //ghi moc thoi gian man hinh dc chuyen: cho su kien auto next
    if (sLCD.sScreenNow.Index_u8 != ScreenLast) {
        ScreenLast = sLCD.sScreenNow.Index_u8;
        LandMarkChange_u32 = RtCountSystick_u32;
    }
    
    //hien thi man hinh cho "OK" sau do quay lai man hinh truoc do
    if ( (sLCD.sScreenNow.Index_u8 == _LCD_SCR_SET_RESTORE)
        && (Check_Time_Out(LandMarkChange_u32, 2000) == true ) ) {
        LandMarkChange_u32 = RtCountSystick_u32;
        //doi ve man hinh truoc do
        UTIL_MEM_cpy(&sLCD.sScreenNow, &sLCD.sScreenBack, sizeof(sScreenInformation));
    }
    
    fevent_enable(sEventDisplay, event); 
    
	return 1;
}



static uint8_t _Cb_Display_Auto_SW (uint8_t event)
{
    static uint16_t cNext = 0;
    static uint8_t MarkButtPressed = false;
    
    if (Check_Time_Out(sButton.LandMarkPressButton_u32, 60000*5) == true) {
        if (MarkButtPressed == true) {
            MarkButtPressed = false;
            sLCD.sScreenNow.Index_u8 = _LCD_SCREEN_1;
        }
        
        cNext++;
        if (cNext >= (DURATION_DISPALY/sEventDisplay[event].e_period)) {
            cNext = 0;
            
            if (sLCD.sScreenNow.Index_u8 == _LCD_SCREEN_7b) {                
                sLCD.sScreenNow.SubIndex_u8++;
                if (sLCD.sScreenNow.SubIndex_u8 >= sWmDigVar.nModbus_u8) {
                    sLCD.sScreenNow.Index_u8++;
                }
            } else {
                if (sLCD.sScreenNow.Index_u8 == _LCD_SCREEN_7) {
                    sLCD.sScreenNow.SubIndex_u8 = 0;
                }
                
                sLCD.sScreenNow.Index_u8++;
            }
            
            if (sLCD.sScreenNow.Index_u8 > _LCD_SCREEN_CM44) {
                sLCD.sScreenNow.Index_u8 = _LCD_SCREEN_1;
            }
        }
    } else {
        MarkButtPressed = true;
    }
    
    fevent_enable(sEventDisplay, event); 
    
	return 1;
}




/*------------ Func Handle -------------*/
/*
    Func: show all oject of screen

*/


void Display_Update (void)
{
    //relate struct global aUnitWm
    static char aUnitWm[5][10] =  
    {
        {" (mV)"},
        {" (mA)"},
        {" (V)"},
        {" (met)"},
        {" (bar)"},
    };
    static char sAnalogName[2][10] = {"P : ", "L : "};
    
    uint8_t StepChann = __CHANEL_2 - __CHANEL_1;
    
    
    for (uint8_t i = 0; i < MAX_CHANNEL; i++) {
        //update unit pressure and scale pulse
        sLCDObject[__PRESS_1 + i * StepChann].Unit = aUnitWm[sWmVar.aPRESSURE[i].sLinearInter.OutUnit_u8];
        sLCDObject[__PULSE_1 + i * StepChann].Scale_u8 =  sPulse[i].Factor_u8;
        //update val press = -1 to convert string '-'
        sLCDObject[__PRESS_1 + i * StepChann].sName =  sAnalogName[sWmVar.aPRESSURE[i].sLinearInter.Type_u8];
        if (sWmVar.aPRESSURE[i].Val_i32 == -1000) {
            sLCDObject[__PRESS_1 + i * StepChann].pData = &charNotDetectPress;
            sLCDObject[__PRESS_1 + i * StepChann].dType_u8 = _DTYPE_CHAR;
        } else {
            sLCDObject[__PRESS_1 + i * StepChann].pData = &sWmVar.aPRESSURE[i].Val_i32;
            sLCDObject[__PRESS_1 + i * StepChann].dType_u8 = _DTYPE_I32;
        }
    }
}


void Display_Show_Screen (uint8_t screen)
{
    uint16_t  i = 0; 
    
    //Clear buff lcd data
    glcd_clear_buffer();
    //Show static param: stime, icon internet,...
    Display_Show_Static_Param();
    
    //Show state connect sensor
    Display_Show_State_Sensor_Network(screen);
    Display_Show_State_Calib_Sensor(screen);
    
    //Show name of screen
    if (screen >= _LCD_SCR_CAL_CHANN_1) {
        glcd_tiny_draw_string(0, 0, "Cal.");
    } else if (screen >= _LCD_SCR_SETTING) {
        glcd_tiny_draw_string(0, 0, "Set.");
    }
    
    for (i = 0; i < __OJECT_END; i++) {
        if (sLCDObject[i].Screen_u8 == screen) {
            Display_Show_Oject(i);
        }
    }
    
    glcd_write();
}


void Display_Show_Oject (uint8_t object)
{
    char aTEMP[32] = {0}; 
    int64_t TempVal = 0;
    uint8_t type = 0, temp = 0;
    uint16_t PosX = sLCDObject[object].Col_u8;
    
    //show name
    if (sLCDObject[object].sName != NULL) {
        if (Display_Check_Toggle(object, 0x01) == false) {
            glcd_tiny_draw_string(PosX, sLCDObject[object].Row_u8, sLCDObject[object].sName);
        } else {
            for (uint8_t i = 0; i < strlen(sLCDObject[object].sName); i++) {
                aTEMP[i] = ' ';
            }
            
            glcd_tiny_draw_string(PosX, sLCDObject[object].Row_u8, aTEMP);
        }
   
        PosX += strlen(sLCDObject[object].sName) * (font_current.width + 1);
    }
        
    //show value
    if (sLCDObject[object].pData != NULL) {
        switch (sLCDObject[object].dType_u8) 
        {
            case _DTYPE_U8:
                TempVal = *( (uint8_t *) sLCDObject[object].pData );
                break;
            case _DTYPE_I8:
                TempVal = *( (int8_t *) sLCDObject[object].pData );
                break;
            case _DTYPE_U16:
                TempVal = *( (uint16_t *) sLCDObject[object].pData );
                break;
            case _DTYPE_I16:
                TempVal = *( (int16_t *) sLCDObject[object].pData );
                break;
            case _DTYPE_U32:
                TempVal = *( (uint32_t *) sLCDObject[object].pData );
                break;
            case _DTYPE_I32:
                TempVal = *( (int32_t *) sLCDObject[object].pData );
                break;
            case _DTYPE_STRING:
                type = 1;
                break;
            case _DTYPE_CHAR:
                type = 2;
                break;
        }
        
        if (type == 0) {            
            UtilIntToStringWithScale (TempVal, aTEMP, 0xFF - sLCDObject[object].Scale_u8 + 1);
        } else if (type == 1) {
            if (strlen ((char *) sLCDObject[object].pData) < sizeof (aTEMP))    
                UTIL_MEM_cpy( aTEMP, (char *) sLCDObject[object].pData, strlen ((char *) sLCDObject[object].pData) );
        } else {
            aTEMP[0] = * ( (char *)sLCDObject[object].pData );
        }
        
        if (Display_Check_Toggle(object, 0x02) == true) {
            //check xem nhay all hay nhay 1 vi tri
            temp = (sLCDObject[object].Mode_u8 >> 4) & 0x0F; 
            if ( temp != 0x0F) {
                aTEMP[temp] = ' ';
            } else {
                for (uint8_t i = 0; i < strlen(aTEMP); i++)
                    aTEMP[i] = ' ';
            }
        }
        
        glcd_tiny_draw_string(PosX, sLCDObject[object].Row_u8, aTEMP);
        PosX += strlen(aTEMP)* (font_current.width + 1);
    }
    
    //Show Unit

    if ( (sLCDObject[object].Unit != NULL)
        && ( (sLCDObject[object].pData != NULL) && (sLCDObject[object].pData != &charNotDetectPress) ) ) {
        glcd_tiny_draw_string(PosX, sLCDObject[object].Row_u8, (char *) sLCDObject[object].Unit );
    }
}


static uint8_t _Cb_button_scan (uint8_t event)
{
    BUTTON_scan();
    
    if (sButton.Status == 1) {
        fevent_active(sEventDisplay, _EVENT_BUTTON_DETECTTED);
        sButton.LandMarkPressButton_u32 = RtCountSystick_u32;
    }

    fevent_enable(sEventDisplay, event);
    
	return 1;
}


static uint8_t _Cb_button_detect (uint8_t event)
{
    BUTTON_Process();
    
    sButton.Value = 0;
    sButton.Status = 0;
    
	return 1;
}


/*
    Func: check toggle object
        + creat toggle effect
*/

uint8_t Display_Check_Toggle (uint8_t object, uint8_t Flag)
{
    static uint32_t LandMarkToggle_u32[2][__OJECT_END] = {0};
    static uint8_t Hide[3][__OJECT_END] = {0};

    if ((sLCDObject[object].Mode_u8 & Flag) == Flag) {
        if (Check_Time_Out(LandMarkToggle_u32[Flag][object], TIME_TOGGLE) == true) {
            LandMarkToggle_u32[Flag][object] = RtCountSystick_u32;
            Hide[Flag][object] =  1- Hide[Flag][object];
        }
    } else {
        Hide[Flag][object] = false;
    }
    
    return Hide[Flag][object];
}

/*
    Func: setup screen modbus: many slave and many type data
        + change data modbus: name, pdata, scale....
*/

void Display_Setup_SCREEN_Modb (void)
{
    static char aCHANNEL[] = "Modb.1  ";

    if (sLCD.sScreenNow.SubIndex_u8 >= sWmDigVar.nModbus_u8) {
        return;
    }
    
    //check status data cua meter
    if (sWmDigVar.sModbDevData[sLCD.sScreenNow.SubIndex_u8].Status_u8 == true) {
        aCHANNEL[5] = sLCD.sScreenNow.SubIndex_u8 + 0x31;
        
        sLCDObject[__CHANEL_7].sName = aCHANNEL;
        
        switch (sWmDigVar.sModbDevData[sLCD.sScreenNow.SubIndex_u8].Type_u8)
        {
            case __MET_WOTECK:  //wm
            case __MET_MT100:  
                sLCDObject[__PULSE_7].sName = "To: ";
                sLCDObject[__PULSE_7].pData = &sWmDigVar.sModbDevData[sLCD.sScreenNow.SubIndex_u8].nTotal_i32;
                sLCDObject[__PULSE_7].dType_u8 = _DTYPE_I32;
                sLCDObject[__PULSE_7].Scale_u8 = sWmDigVar.sModbDevData[sLCD.sScreenNow.SubIndex_u8].Factor;
                sLCDObject[__PULSE_7].Unit = sWmDigVar.sModbDevData[sLCD.sScreenNow.SubIndex_u8].sTotalUnit;

                sLCDObject[__PRESS_7].sName = "P : ";
                sLCDObject[__PRESS_7].pData = &charNotDetectPress;
                sLCDObject[__PRESS_7].dType_u8 = _DTYPE_CHAR; 
                sLCDObject[__PRESS_7].Unit = NULL; 
                sLCDObject[__PRESS_7].Scale_u8 = 0x00;
                    
                sLCDObject[__FLOW_7].sName = "F : ";
                sLCDObject[__FLOW_7].pData = &sWmDigVar.sModbDevData[sLCD.sScreenNow.SubIndex_u8].Flow_i32;
                sLCDObject[__FLOW_7].dType_u8 = _DTYPE_I32;
                sLCDObject[__FLOW_7].Scale_u8 = sWmDigVar.sModbDevData[sLCD.sScreenNow.SubIndex_u8].Factor;
                sLCDObject[__FLOW_7].Unit = sWmDigVar.sModbDevData[sLCD.sScreenNow.SubIndex_u8].sFlowUnit; 
                break;
            case __MET_LEVEL_LIQ: //level
            case __MET_LEVEL_ULTRA: 
            case __MET_LEVEL_LIQ_SUP: 
                sLCDObject[__PULSE_7].sName = "L : ";
                sLCDObject[__PULSE_7].pData = &sWmDigVar.sModbDevData[sLCD.sScreenNow.SubIndex_u8].LVal_i16;
                sLCDObject[__PULSE_7].dType_u8 = _DTYPE_I16;
                sLCDObject[__PULSE_7].Scale_u8 = 0xFF - sWmDigVar.sModbDevData[sLCD.sScreenNow.SubIndex_u8].LDecimal_u16 + 1;
                sLCDObject[__PULSE_7].Unit = " (cm)";
            
                sLCDObject[__PRESS_7].sName = NULL;
                sLCDObject[__PRESS_7].pData = NULL;
                sLCDObject[__PRESS_7].Scale_u8 = 0x00;
                    
                sLCDObject[__FLOW_7].sName = NULL;
                sLCDObject[__FLOW_7].pData = NULL;
                break;
        }              
    } else {
        if (sButton.Old_value == _LCD_SCREEN_1) {
            //dang giam
            if (sLCD.sScreenNow.SubIndex_u8 > 0) {
                sLCD.sScreenNow.SubIndex_u8--;
            } else {
                sLCD.sScreenNow.Index_u8--;
            }
        } else {
            if ( (sLCD.sScreenNow.SubIndex_u8 + 1) >= sWmDigVar.nModbus_u8 ) {
                sLCD.sScreenNow.SubIndex_u8 = 0;
                sLCD.sScreenNow.Index_u8 = _LCD_SCREEN_1;
            } else {
                sLCD.sScreenNow.SubIndex_u8++;
            }
        }
    }
}



void Display_Set_Screen_Flag (sScreenInformation *screen, void *pData, uint8_t flag)
{
    screen->Flag_u8 = flag;
    //set ting mode    
    for (uint8_t i = screen->ParaMin_u8; i <= screen->ParaMax_u8; i++) {
        sLCDObject[i].Mode_u8 = 0xF0;
    }
    
    sLCDObject[screen->Para_u8].Mode_u8 = screen->Flag_u8;
    if (pData != NULL) {
        sLCDObject[screen->Para_u8].pData = pData;
    }
}

/*
    Func: set screen next
        + index:
        + param: curr, min, max
        + pdata: option
        + flag: toggle: name | pdata | index of pdata
*/

void Display_Set_Screen (sScreenInformation *screen, uint8_t index, uint8_t subindex,
                         uint8_t para, uint8_t paramin, uint8_t paramax,
                         void *pData, uint8_t flag)
{
    screen->Index_u8 = index;
    screen->SubIndex_u8 = subindex;
    screen->Para_u8 = para;
    screen->ParaMin_u8 = paramin;
    screen->ParaMax_u8 = paramax;

    Display_Set_Screen_Flag(screen, pData, flag);
}

/*
    Func: check pass to setting
*/

uint8_t Display_Check_Password (uint8_t pPass[])
{
    for (uint8_t i = 0; i < sizeof(aPASSWORD); i++) {
        if (pPass[i] != aPASSWORD[i])
            return false;
    }
    
    return true;
}

uint8_t Display_Check_PassCalib (uint8_t pPass[])
{
    for (uint8_t i = 0; i < sizeof(aPASSCALIB); i++) {
        if (pPass[i] != aPASSCALIB[i])
            return false;
    }
    
    return true;
}


/*
    Func: sub process up button: control config pressure
*/

void Display_Process_Up_Pressure_2 (sPressureLinearInter *pPress)
{
    switch (sLCD.sScreenNow.Para_u8)
    {
        case __SET_PRESSURE_2_0:
            pPress->Type_u8++;
            if (pPress->Type_u8 > 1)
                pPress->Type_u8 = 0;
            sLCDObject[sLCD.sScreenNow.Para_u8].pData = AnalogType[pPress->Type_u8];  
            break;
        case __SET_PRESSURE_2_1:
            pPress->Factor_u16++;
            break;
        case __SET_PRESSURE_2_2:
            pPress->InMin_u16++;
            break;
        case __SET_PRESSURE_2_3:
            pPress->InMax_u16++;
            break;
        case __SET_PRESSURE_2_4:
            if (pPress->InUnit_u8 < _UNIT_BAR) {
                pPress->InUnit_u8++;
                sLCDObject[sLCD.sScreenNow.Para_u8].pData = aUnitWm[pPress->InUnit_u8];
            } else {
                pPress->InUnit_u8 = _UNIT_MILIVOL;
                sLCDObject[sLCD.sScreenNow.Para_u8].pData = aUnitWm[pPress->InUnit_u8];
            }
            break;
        case __SET_PRESSURE_2_5:
            pPress->OutMin_u16++;
            break;
        case __SET_PRESSURE_2_6:
            pPress->OutMax_u16 ++;
            break;
        case __SET_PRESSURE_2_7:
            if (pPress->OutUnit_u8 < _UNIT_BAR) {
                pPress->OutUnit_u8++;
                sLCDObject[sLCD.sScreenNow.Para_u8].pData = aUnitWm[pPress->OutUnit_u8];
            } else {
                pPress->OutUnit_u8 = _UNIT_MILIVOL;
                sLCDObject[sLCD.sScreenNow.Para_u8].pData = aUnitWm[pPress->OutUnit_u8];
            }
            break;
    }  
}

/*
    Func: sub process down button: control config pressure
*/

void Display_Process_Down_Pressure_2 (sPressureLinearInter *pPress)
{
    switch (sLCD.sScreenNow.Para_u8)
    {
        case __SET_PRESSURE_2_0:
            if (pPress->Type_u8 > 1) {
                pPress->Type_u8--;
            } else {
                pPress->Type_u8 = 1;
            }
            sLCDObject[sLCD.sScreenNow.Para_u8].pData = AnalogType[pPress->Type_u8];  
            break;
        case __SET_PRESSURE_2_1:
            pPress->Factor_u16--;
            break;
        case __SET_PRESSURE_2_2:
            pPress->InMin_u16--;
            break;
        case __SET_PRESSURE_2_3:
            pPress->InMax_u16--;
            break;
        case __SET_PRESSURE_2_4:
            if (pPress->InUnit_u8 > _UNIT_MILIVOL) {
                pPress->InUnit_u8--;
            } else {
                pPress->InUnit_u8 = _UNIT_BAR;
            }
            sLCDObject[sLCD.sScreenNow.Para_u8].pData = aUnitWm[pPress->InUnit_u8];
            break;
        case __SET_PRESSURE_2_5:
            pPress->OutMin_u16--;
            break;
        case __SET_PRESSURE_2_6:
            pPress->OutMax_u16--;
            break;
        case __SET_PRESSURE_2_7:
            if (pPress->OutUnit_u8 > _UNIT_MILIVOL) {
                pPress->OutUnit_u8--;
            } else {
                pPress->OutUnit_u8 = _UNIT_BAR; 
            }
            sLCDObject[sLCD.sScreenNow.Para_u8].pData = aUnitWm[pPress->OutUnit_u8];
            break;
    }  
}

/*
    Func: show static param
        + stime
        + icon: internet, baterry, "____"
*/
void Display_Show_Static_Param (void)
{
    static uint8_t cCharge = 0, batlevel = 0;
    uint8_t TempPos = 0;
    char aData[32] = {0};
    static uint32_t LandMarkNextScreen = 0;
    
    //Hien thi sac pin
    if (Display_Show_Charge_Bat(&batlevel) == 1) {
        if (Check_Time_Out(LandMarkNextScreen, 500) == true) {
            LandMarkNextScreen = RtCountSystick_u32;
            cCharge++;
        } 
        
        if(cCharge > 3) {
            cCharge = batlevel;
        }
    } else {
        LandMarkNextScreen = RtCountSystick_u32;
        cCharge = batlevel;
    }
        
    glcd_tiny_draw_char(120, 0, PIN_ZERO + cCharge);
        
    //Hien thi cot song sim
    if (sSimCommVar.State_u8 == _SIM_CONN_MQTT) {
        glcd_tiny_draw_char(114, 0, CONNECT_DISPLAY);
        TempPos = 6;
    } else {
        glcd_tiny_draw_char(114, 0, 0x20U);    
    }
    
    //hien thi cot ethernet
    if (sAppEthVar.Status_u8 == _ETH_MQTT_CONNECTED) {
        glcd_tiny_draw_char(114 - TempPos, 0, FONT_ETHERNET);    
    } else {
        glcd_tiny_draw_char(114 - TempPos, 0, 0x20U);    
    }
    
    //Hien thi stime
    sprintf(aData, "%02d:%02d:%02d", sRTC.hour, sRTC.min, sRTC.sec);
    glcd_tiny_draw_string(48, 0, (char *) aData);
    
    //Hien thi gạch duoi
    glcd_draw_line(0, 8, 127, 8, BLACK); 
}

/*
    Func: show static param
        + stime
        + icon: internet, baterry, "____"
*/
void Display_Show_State_Sensor_Network (uint8_t screen)
{
    if(screen == _LCD_SCREEN_CM44)
    {
        if(sSensor_pH.State_Connect == _SENSOR_DISCONNECT)
        {
            glcd_tiny_draw_string(120, sLCDObject[__SC1_PH_WATER].Row_u8, " ");
        }
        else
        {
            glcd_tiny_draw_string(120, sLCDObject[__SC1_PH_WATER].Row_u8, "N");
        }
        
        if(sSensor_Clo.State_Connect == _SENSOR_DISCONNECT)
        {
            glcd_tiny_draw_string(120, sLCDObject[__SC1_CLO_DU].Row_u8, " ");
            glcd_tiny_draw_string(120, sLCDObject[__SC1_TEMP].Row_u8, " ");
        }
        else
        {
            glcd_tiny_draw_string(120, sLCDObject[__SC1_CLO_DU].Row_u8, "N");
            glcd_tiny_draw_string(120, sLCDObject[__SC1_TEMP].Row_u8, "N");
        }
        
        if(sSensor_EC.State_Connect == _SENSOR_DISCONNECT)
        {
            glcd_tiny_draw_string(120, sLCDObject[__SC1_EC].Row_u8, " ");
            glcd_tiny_draw_string(120, sLCDObject[__SC1_SALINITY].Row_u8, " ");
        }
        else
        {
            glcd_tiny_draw_string(120, sLCDObject[__SC1_EC].Row_u8, "N");
            glcd_tiny_draw_string(120, sLCDObject[__SC1_SALINITY].Row_u8, "N");
        }
        
        if(sSensor_Turbidity.State_Connect == _SENSOR_DISCONNECT)
        {
            glcd_tiny_draw_string(120, sLCDObject[__SC1_NTU].Row_u8, " ");
        }
        else
        {
            glcd_tiny_draw_string(120, sLCDObject[__SC1_NTU].Row_u8, "N");
        }
    }
}

/*
    Func: show static param
        + stime
        + icon: internet, baterry, "____"
*/
void Display_Show_State_Calib_Sensor (uint8_t screen)
{
    if(screen == _LCD_SCR_SET_CALIB_STATE)
    {
        switch(sDataSensorMeasure.State_Wait_Calib)
        {
            case _STATE_CALIB_FREE:
              sLCDObject[__SET_CALIB_STATE].pData = aSTT_CALIB_FREE;
              break;
              
            case _STATE_CALIB_ENTER:
              sLCDObject[__SET_CALIB_STATE].pData = aSTT_CALIB_ENTER;
              break;

            case _STATE_CALIB_WAIT:
              sLCDObject[__SET_CALIB_STATE].pData = aSTT_CALIB_WAIT;
              break;
              
            case _STATE_CALIB_DONE:
              sLCDObject[__SET_CALIB_STATE].pData = aSTT_CALIB_DONE;
              break;
              
            case _STATE_CALIB_ERROR:
              sLCDObject[__SET_CALIB_STATE].pData = aSTT_CALIB_ERROR;
              break;
              
            default:
              break;
        }
    }
    else
    {
        sLCDObject[__SET_CALIB_STATE].pData = aSTT_CALIB_FREE;
    }
}


/*
    Func: caculator level Vbat : 
        + co 4 muc: < 2.9v : 0
                    < 3.5v : 1
                    < 4.1  : 2
                    > 4.1  : 3
        + return: 1: co nguon ngoai va dang sac: chay hieu ung sac
                  0: mat nguon ngoai: hien thi cot pin
*/

uint8_t Display_Show_Charge_Bat (uint8_t *level)
{
    if (sBattery.mVol_u32 < 2900) {
        *level = 0;
    } else if (sBattery.mVol_u32 < 3500) {
        *level = 1;
    } else if (sBattery.mVol_u32 < 4100) {
        *level = 2;
    } else {
        *level = 3;
    }
   
    //neu co nguon ngoai return 1
    if (sVout.mVol_u32 > 5000) {
        if (*level < 3) {
            return 1;
        }
    }
    
    return 0;
}





