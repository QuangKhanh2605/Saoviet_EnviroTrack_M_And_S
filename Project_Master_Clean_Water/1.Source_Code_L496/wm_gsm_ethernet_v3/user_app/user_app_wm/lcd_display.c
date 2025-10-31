

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

Struct_ParaDisplay      sParaDisplay = {0};

char AnalogType[2][10] = { "pressure", "level"};

uint8_t aPASSWORD[4] = {"0000"};

uint8_t aSTT_SETTING_FREE[14]   = {"              "};
uint8_t aSTT_SETTING_ENTER[14]  = {"Enter to Setup"};
uint8_t aSTT_SETTING_WAIT[14]   = {"    Waiting   "};
uint8_t aSTT_SETTING_DONE[14]   = {"     Done    "};
uint8_t aSTT_SETTING_ERROR[14]  = {"     Error   "};

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
    {   __SC1_CLO_DU,       "CLO DU: ",         NULL,   _DTYPE_I16,        0,   "  mg/L     ",  2,  0,  0x00,    _LCD_SCREEN_CM44    },
    {   __SC1_PH_WATER,     "PH ATC: ",         NULL,   _DTYPE_I16,        0,   "  pH       ",  2,  0,  0x00,    _LCD_SCREEN_CM44    },
    {   __SC1_TURB,         "NTU   : ",         NULL,   _DTYPE_I16,        0,   "  NTU      ",  2,  0,  0x00,    _LCD_SCREEN_CM44    },
    {   __SC1_EC,           "EC    : ",         NULL,   _DTYPE_I16,        0,   "  uS/cm    ",  2,  0,  0x00,    _LCD_SCREEN_CM44    },
    {   __SC1_SALINITY,     "Do man: ",         NULL,   _DTYPE_I16,        0,   "  %        ",  2,  0,  0x00,    _LCD_SCREEN_CM44    },
    {   __SC1_TEMP,         "Temp  : ",         NULL,   _DTYPE_I16,        0,   "  ‰C       ",  2,  0,  0x00,    _LCD_SCREEN_CM44    },
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
    
    {   __SET_REQ_1,        "1.Tsend Data",     NULL,   _DTYPE_STRING,   0,      NULL,      2,  14, 0x00,    _LCD_SCR_SETTING },
    {   __SET_OFFSET_SENSOR,"2.Offset Sensor",   NULL,   _DTYPE_STRING,  0,      NULL,      3,  14, 0x00,    _LCD_SCR_SETTING },
    {   __SET_OPTION_SENSOR,"3.Option Sensor",  NULL,   _DTYPE_STRING,   0,      NULL,      4,  14, 0x00,    _LCD_SCR_SETTING },
    {   __DEVICE_INFOR,     "4.Information",    NULL,   _DTYPE_STRING,   0,      NULL,      5,  14, 0x00,    _LCD_SCR_SETTING },
    
    {   __SET_PULSE_SETT,   NULL,  NULL,   _DTYPE_STRING,   0,      NULL,      2,  24, 0x00,    _LCD_SCR_SETTING },
    {   __SET_RESTORE_PULSE,NULL,  NULL,   _DTYPE_STRING,   0,      NULL,      3,  24, 0x00,    _LCD_SCR_SETTING },
    {   __SET_PRESSURE,     NULL,  NULL,   _DTYPE_STRING,   0,      NULL,      4,  24, 0x00,    _LCD_SCR_SETTING },
    {   __SET_PRESS_CALIB,  NULL,  NULL,   _DTYPE_STRING,   0,      NULL,      5,  24, 0x00,    _LCD_SCR_SETTING },
    
    {   __SET_MANUFACTOR,   NULL,    NULL,   _DTYPE_STRING,   0,      NULL,      1,  24, 0x00,    _LCD_SCR_SETTING_2 },
    
    {   __SET_REQ_2_1,      "*Tsend Data:",     NULL,   _DTYPE_STRING,   0,      NULL,      2,  24, 0x00,    _LCD_SCR_SET_FREQ },
    {   __SET_REQ_2_2,      NULL,               NULL,   _DTYPE_U32,      0,      " (min)",  3,  36, 0x02,    _LCD_SCR_SET_FREQ },

    {   __SET_OFFSET_SS_PH,         "1.pH  : ",     NULL,   _DTYPE_I32,    0xFE,      "  pH       ",      2,  0, 0x00,    _LCD_SCR_SET_OFFSET_SENSOR },
    {   __SET_OFFSET_SS_CLO,        "2.Clo : ",     NULL,   _DTYPE_I32,    0xFE,      "  mg/L     ",      3,  0, 0x00,    _LCD_SCR_SET_OFFSET_SENSOR },
    {   __SET_OFFSET_SS_TURB,       "3.Turb: ",     NULL,   _DTYPE_I32,    0xFE,      "  NTU      ",      4,  0, 0x00,    _LCD_SCR_SET_OFFSET_SENSOR },
    {   __SET_OFFSET_SS_EC,         "4.EC  : ",     NULL,   _DTYPE_I32,    0xFE,      "  uS/cm    ",      5,  0, 0x00,    _LCD_SCR_SET_OFFSET_SENSOR },
    {   __SET_OFFSET_SS_SALINITY,   "5.Sal : ",     NULL,   _DTYPE_I32,    0xFE,      "  %        ",      6,  0, 0x00,    _LCD_SCR_SET_OFFSET_SENSOR },
    {   __SET_OFFSET_SS_TEMP,       "6.Temp: ",     NULL,   _DTYPE_I32,    0xFE,      "  ‰C       ",      7,  0, 0x00,    _LCD_SCR_SET_OFFSET_SENSOR },
    
    {   __SET_OPTION_SS_TITLE,  "OPTION SENSOR",  NULL,   _DTYPE_STRING,   0x00, NULL,        2,  26,  0x00,    _LCD_SCR_SET_OPTION_SENSOR  },
    {   __SET_OPTION_SS_PH,     "1.pH      : ",   NULL,   _DTYPE_U8,   0x00, NULL,            3,  14,  0x00,    _LCD_SCR_SET_OPTION_SENSOR  },
    {   __SET_OPTION_SS_CLO,    "2.Clo     : ",   NULL,   _DTYPE_U8,   0x00, NULL,            4,  14,  0x00,    _LCD_SCR_SET_OPTION_SENSOR  },
    {   __SET_OPTION_SS_EC,     "3.EC      : ",   NULL,   _DTYPE_U8,   0x00, NULL,            5,  14,  0x00,    _LCD_SCR_SET_OPTION_SENSOR  },
    {   __SET_OPTION_SS_TURB,   "4.TUR     : ",   NULL,   _DTYPE_U8,   0x00, NULL,            6,  14,  0x00,    _LCD_SCR_SET_OPTION_SENSOR  },
    
    {   __CHECK_STATE_SETTING,        NULL,             NULL,   _DTYPE_STRING,   0,      NULL,      1,  24, 0x00,     _LCD_SCR_CHECK_SETTING},
    
    {   __INFOR_FW_VERSION_1,"*Version",        NULL,   _DTYPE_STRING,   0,     NULL,       2,  24, 0x00,     _LCD_SCR_INFORMATION },
    {   __INFOR_FW_VERSION_2, NULL,             NULL,   _DTYPE_STRING,   0,     NULL,       3,  5,  0x00,     _LCD_SCR_INFORMATION },
    
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
    sLCDObject[__SC1_CLO_DU].pData      = &sDataSensorMeasure.sClo.Value_i32 ; 
    sLCDObject[__SC1_CLO_DU].Scale_u8   = sDataSensorMeasure.sClo.Scale_u8; 
    sLCDObject[__SC1_PH_WATER].pData    = &sDataSensorMeasure.spH.Value_i32;    
    sLCDObject[__SC1_PH_WATER].Scale_u8 = sDataSensorMeasure.spH.Scale_u8;    
    sLCDObject[__SC1_TURB].pData         = &sDataSensorMeasure.sTurb.Value_i32;   
    sLCDObject[__SC1_TURB].Scale_u8      = sDataSensorMeasure.sTurb.Scale_u8;   
    sLCDObject[__SC1_SALINITY].pData    = &sDataSensorMeasure.sSal.Value_i32;    
    sLCDObject[__SC1_SALINITY].Scale_u8 = sDataSensorMeasure.sSal.Scale_u8; 
    sLCDObject[__SC1_TEMP].pData        = &sDataSensorMeasure.sTemp.Value_i32;    
    sLCDObject[__SC1_TEMP].Scale_u8     = sDataSensorMeasure.sTemp.Scale_u8;   
    sLCDObject[__SC1_EC].pData          = &sDataSensorMeasure.sEC.Value_i32;   
    sLCDObject[__SC1_EC].Scale_u8       = sDataSensorMeasure.sEC.Scale_u8;    
    
    sLCDObject[__SET_OFFSET_SS_PH].pData        = &sParaDisplay.pH_Offset_i32;
    sLCDObject[__SET_OFFSET_SS_CLO].pData       = &sParaDisplay.Clo_Offset_i32;
    sLCDObject[__SET_OFFSET_SS_TURB].pData      = &sParaDisplay.Turb_Offset_i32;
    sLCDObject[__SET_OFFSET_SS_EC].pData        = &sParaDisplay.EC_Offset_i32;
    sLCDObject[__SET_OFFSET_SS_SALINITY].pData  = &sParaDisplay.Salinity_Offset_i32;
    sLCDObject[__SET_OFFSET_SS_TEMP].pData      = &sParaDisplay.Temp_Offset_i32;
    
    sLCDObject[__SET_OPTION_SS_PH].pData      = &sUserSensor.User_pH;
    sLCDObject[__SET_OPTION_SS_CLO].pData     = &sUserSensor.User_Clo;
    sLCDObject[__SET_OPTION_SS_EC].pData      = &sUserSensor.User_EC;
    sLCDObject[__SET_OPTION_SS_TURB].pData   = &sUserSensor.User_Turb;
    
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
    Update_ParaDisplay();
    Display_Update();
    //hien thi man hinh: index
    if (sLCD.Ready_u8 == true) {
        //neu index 9: modbus disp: gan lai cac bien va subindex
        if (sLCD.sScreenNow.Index_u8 == _LCD_SCREEN_7b) {
            Display_Setup_SCREEN_Modb();
        }
        
        //hien thá»‹
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
    
    if (Check_Time_Out(sButton.LandMarkPressButton_u32, 60000*10) == true) {
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
    Display_Update_ScrSensor(screen);
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
    static uint32_t LandMarkToggle_u32[3][__OJECT_END] = {0};
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
    
    //Hien thi gáº¡ch duoi
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
        if(sDataSensorMeasure.spH.State_u8 == 0)
        {
            glcd_tiny_draw_string(120, sLCDObject[__SC1_PH_WATER].Row_u8, " ");
        }
        else
        {
            glcd_tiny_draw_string(120, sLCDObject[__SC1_PH_WATER].Row_u8, "N");
        }
        
        if(sDataSensorMeasure.sClo.State_u8 == 0)
        {
            glcd_tiny_draw_string(120, sLCDObject[__SC1_CLO_DU].Row_u8, " ");
        }
        else
        {
            glcd_tiny_draw_string(120, sLCDObject[__SC1_CLO_DU].Row_u8, "N");
        }

        if(sDataSensorMeasure.sTurb.State_u8 == 0)
        {
            glcd_tiny_draw_string(120, sLCDObject[__SC1_TURB].Row_u8, " ");
        }
        else
        {
            glcd_tiny_draw_string(120, sLCDObject[__SC1_TURB].Row_u8, "N");
        }
        
        if(sDataSensorMeasure.sEC.State_u8 == 0)
        {
            glcd_tiny_draw_string(120, sLCDObject[__SC1_EC].Row_u8, " ");
        }
        else
        {
            glcd_tiny_draw_string(120, sLCDObject[__SC1_EC].Row_u8, "N");
        }
        
        if(sDataSensorMeasure.sSal.State_u8 == 0)
        {
            glcd_tiny_draw_string(120, sLCDObject[__SC1_SALINITY].Row_u8, " ");
        }
        else
        {
            glcd_tiny_draw_string(120, sLCDObject[__SC1_SALINITY].Row_u8, "N");
        }
        
        if(sDataSensorMeasure.sTemp.State_u8 == 0)
        {
            glcd_tiny_draw_string(120, sLCDObject[__SC1_TEMP].Row_u8, " ");
        }
        else
        {
            glcd_tiny_draw_string(120, sLCDObject[__SC1_TEMP].Row_u8, "N");
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
    if(screen == _LCD_SCR_CHECK_SETTING)
    {
        switch(sHandleRs485.State_Wait_Calib)
        {
            case _STATE_CALIB_FREE:
              sLCDObject[__CHECK_STATE_SETTING].pData = aSTT_SETTING_FREE;
              break;
              
            case _STATE_CALIB_ENTER:
              sLCDObject[__CHECK_STATE_SETTING].pData = aSTT_SETTING_ENTER;
              break;

            case _STATE_CALIB_WAIT:
              sLCDObject[__CHECK_STATE_SETTING].pData = aSTT_SETTING_WAIT;
              break;
              
            case _STATE_CALIB_DONE:
              sLCDObject[__CHECK_STATE_SETTING].pData = aSTT_SETTING_DONE;
              break;
              
            case _STATE_CALIB_ERROR:
              sLCDObject[__CHECK_STATE_SETTING].pData = aSTT_SETTING_ERROR;
              break;
              
            default:
              break;
        }
    }
    else
    {
        sLCDObject[__CHECK_STATE_SETTING].pData = aSTT_SETTING_FREE;
    }
}

void Display_Update_ScrSensor (uint8_t screen)
{
    if(screen == _LCD_SCREEN_CM44)
    {
        uint8_t Row = 2;

        if(sUserSensor.User_Clo == _ACTIVE_SENSOR)
            sLCDObject[__SC1_CLO_DU].Row_u8 = Row++;
        else
            sLCDObject[__SC1_CLO_DU].Row_u8 = Row;
        
        if(sUserSensor.User_pH == _ACTIVE_SENSOR)
            sLCDObject[__SC1_PH_WATER].Row_u8 = Row++;
        else
            sLCDObject[__SC1_PH_WATER].Row_u8 = Row;

        if(sUserSensor.User_Turb == _ACTIVE_SENSOR)
            sLCDObject[__SC1_TURB].Row_u8 = Row++;
        else
            sLCDObject[__SC1_TURB].Row_u8 = Row;

        if(sUserSensor.User_EC == _ACTIVE_SENSOR)
        {
            sLCDObject[__SC1_EC].Row_u8 = Row++;
            sLCDObject[__SC1_SALINITY].Row_u8 = Row++;
        }
        else
        {
            sLCDObject[__SC1_EC].Row_u8 = Row;
            sLCDObject[__SC1_SALINITY].Row_u8 = Row;
        }
        
        sLCDObject[__SC1_TEMP].Row_u8 = Row;
    }
}

void Update_ParaDisplay(void)
{
    sParaDisplay.pH_Offset_i32 = (int32_t)(sOffsetMeasure.pH_f * 100);
    sParaDisplay.Clo_Offset_i32 = (int32_t)(sOffsetMeasure.Clo_f * 100);
    sParaDisplay.Turb_Offset_i32 = (int32_t)(sOffsetMeasure.Turb_f * 100);
    sParaDisplay.EC_Offset_i32 = (int32_t)(sOffsetMeasure.EC_f * 100);
    sParaDisplay.Salinity_Offset_i32 = (int32_t)(sOffsetMeasure.Sal_f * 100);
    sParaDisplay.Temp_Offset_i32 = (int32_t)(sOffsetMeasure.Temp_f * 100);
    
    sLCDObject[__SC1_CLO_DU].Scale_u8   = sDataSensorMeasure.sClo.Scale_u8;   
    sLCDObject[__SC1_PH_WATER].Scale_u8 = sDataSensorMeasure.spH.Scale_u8;    
    sLCDObject[__SC1_TURB].Scale_u8      = sDataSensorMeasure.sTurb.Scale_u8;    
    sLCDObject[__SC1_SALINITY].Scale_u8 = sDataSensorMeasure.sSal.Scale_u8;  
    sLCDObject[__SC1_TEMP].Scale_u8     = sDataSensorMeasure.sTemp.Scale_u8;    
    sLCDObject[__SC1_EC].Scale_u8       = sDataSensorMeasure.sEC.Scale_u8; 
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





