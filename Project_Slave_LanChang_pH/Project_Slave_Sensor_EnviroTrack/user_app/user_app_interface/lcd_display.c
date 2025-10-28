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

sParameter_Display   sParaDisplay = {0};

sData   sModelVersion = {(uint8_t *) "LEVEL_ULTRA_SAOVIET", 19}; 

uint8_t aPASSWORD[4] = {"0000"};

uint8_t aSTT_SETTING_FREE[14]   = {"              "};
uint8_t aSTT_SETTING_ENTER[14]  = {"Enter to Setup"};
uint8_t aSTT_SETTING_WAIT[14]   = {"    Waiting   "};
uint8_t aSTT_SETTING_DONE[14]   = {"     Done    "};
uint8_t aSTT_SETTING_ERROR[14]  = {"     Error   "};

sOjectInformation  sLCDObject[] = 
{
//          para          name                  value      dtype         scale   unit      row  col      screen
    {   __SC1_PH_ATC,     "PH_ATC: ",       NULL,   _DTYPE_STRING,   0xFE,      NULL,      0,  4,  0x00,      _LCD_SCREEN_1  },
    {   __SC1_TEMP,       "Temp  : ",       NULL,   _DTYPE_STRING,   0xFE,      NULL,      3,  4,  0x00,      _LCD_SCREEN_1  },
  
    {   __PASS_WORD_1,    "Enter Password",   NULL,   _DTYPE_STRING,   0,      NULL,      1,  28, 0x00,      _LCD_SCR_PASS    },
    {   __PASS_WORD_2,    NULL,               NULL,   _DTYPE_STRING,   0,      NULL,      2,  58, 0x00,      _LCD_SCR_PASS    },
    
    {   __SCR_SET_TITLE,  "SETTING",          NULL,   _DTYPE_STRING,   0,      NULL,      0,   50, 0x00,      _LCD_SCR_SETTING },
    {   __SCR_SET_MODBUS, "1.Modbus RTU",     NULL,   _DTYPE_STRING,   0,      NULL,      1,   18, 0x00,      _LCD_SCR_SETTING },
    {   __SCR_SET_CALIB,  "2.Calibration",    NULL,   _DTYPE_STRING,   0,      NULL,      2,   18, 0x00,      _LCD_SCR_SETTING },
    {   __SCR_SET_OFFSET, "3.Offset",         NULL,   _DTYPE_STRING,   0,      NULL,      3,   18, 0x00,      _LCD_SCR_SETTING },
    {   __SCR_SET_INFOR,  "4.Information",    NULL,   _DTYPE_STRING,   0,      NULL,      4,   18, 0x00,      _LCD_SCR_SETTING },
    
    {   __SET_MODBUS_TITLE,     "SET MODBUS RTU", NULL,   _DTYPE_STRING,   0,      NULL,      0,   28, 0x00,     _LCD_SCR_SET_MODBUS },
    {   __SET_MODBUS_ID,        "1.ID      : ",   NULL,   _DTYPE_U8,       0x00,   NULL,      1,   4, 0x00,      _LCD_SCR_SET_MODBUS },
    {   __SET_MODBUS_BR,        "2.Baudrate: ",   NULL,   _DTYPE_U32,      0x00,   NULL,      2,   4, 0x00,      _LCD_SCR_SET_MODBUS },
    
    {   __SET_PH_TITLE,     "CALIB SENSOR",     NULL,   _DTYPE_STRING,   0x00,  NULL,       1,  5,  0x00,    _LCD_SCR_SET_CALIB_SS_PH},
    {   __SET_PH_VALUE,     "pH: ",             NULL,   _DTYPE_I32,      0x00,  NULL,       2,  5,  0x00,    _LCD_SCR_SET_CALIB_SS_PH},
    {   __SET_AD_VALUE,     "AD: ",             NULL,   _DTYPE_I32,      0x00,  NULL,       2,  90, 0x00,    _LCD_SCR_SET_CALIB_SS_PH},
    {   __SET_PH_Z_SOLUTION,"1.Z_Solution: ",   NULL,   _DTYPE_I16,      0xFE,  NULL,       3,  14, 0x00,    _LCD_SCR_SET_CALIB_SS_PH},
    {   __SET_PH_ZERO,      "2.Zero",           NULL,   _DTYPE_STRING,   0,     NULL,       4,  14, 0x00,    _LCD_SCR_SET_CALIB_SS_PH},
    {   __SET_PH_S_SOLUTION,"3.S_Solution: ",   NULL,   _DTYPE_I16,      0xFE,  NULL,       5,  14, 0x00,    _LCD_SCR_SET_CALIB_SS_PH},
    {   __SET_PH_SLOPE,     "4.Slope",          NULL,   _DTYPE_STRING,   0,     NULL,       6,  14, 0x00,    _LCD_SCR_SET_CALIB_SS_PH},
    
    {   __SET_OFFSET_TITLE, "OFFSET",           NULL,   _DTYPE_STRING,  0,      NULL,       0,  28, 0x00,    _LCD_SCR_SET_OFFSET},
    {   __SET_OFFSET_PH,    "1.pH_ATC: ",       NULL,   _DTYPE_I32,     0xFE,   NULL,       1,  14, 0x00,    _LCD_SCR_SET_OFFSET},
    {   __SET_OFFSET_TEMP,  "2.Temp  : ",       NULL,   _DTYPE_I32,     0xFE,   NULL,       2,  14, 0x00,    _LCD_SCR_SET_OFFSET},
    
    {   __SCR_INFOR_FW_VERSION_1,     "*Version",       NULL,   _DTYPE_STRING,   0,      NULL,      0,   28, 0x00,      _LCD_SCR_SET_INFORMATION },
    {   __SCR_INFOR_FW_VERSION_2,           NULL,       NULL,   _DTYPE_STRING,   0,      NULL,      1,    4, 0x00,       _LCD_SCR_SET_INFORMATION },
    
    {   __SCR_INFOR_MODEL_1,          "*Model",         NULL,   _DTYPE_STRING,   0,      NULL,      2,   28, 0x00,      _LCD_SCR_SET_INFORMATION },
    {   __SCR_INFOR_MODEL_2,                NULL,       NULL,   _DTYPE_STRING,   0,      NULL,      3,    4, 0x00,       _LCD_SCR_SET_INFORMATION },
    
    {   __CHECK_STATE_SETTING,        NULL,             NULL,   _DTYPE_STRING,   0,      NULL,      1,  24, 0x00,     _LCD_SCR_CHECK_SETTING},
    
    {   __SET_RESTORE_1,    "Successfully!",    NULL,   _DTYPE_STRING,   0,      NULL,      3,  24, 0x02,    _LCD_SCR_SET_RESTORE},
};

static char charNotDetectPress = '-';

/*=========================Function=========================*/
void Display_Init (void)
{
    glcd_init();
    glcd_clear_buffer();
    glcd_write();	
    Deinit_LCD12864();
    
    sLCDObject[__SET_MODBUS_ID].pData  = &sParaDisplay.ID_u8; 
    sLCDObject[__SET_MODBUS_BR].pData  = &sParaDisplay.Baudrate_u32; 
    
    sLCDObject[__SC1_PH_ATC].pData     = &sParaDisplay.pH_Filter_i32;    
    sLCDObject[__SC1_TEMP].pData       = &sParaDisplay.Temp_Filter_i32;    
    
    sLCDObject[__SET_PH_VALUE].pData        = &sParaDisplay.pH_Value_i32; 
    sLCDObject[__SET_AD_VALUE].pData        = &sSensor_pH.Measure_AD;
    
    sLCDObject[__SET_PH_Z_SOLUTION].pData   = &sSensor_pH.sZero_Calib.Value ; 
    sLCDObject[__SET_PH_ZERO].Scale_u8      = sSensor_pH.sZero_Calib.Scale; 
    sLCDObject[__SET_PH_S_SOLUTION].pData   = &sSensor_pH.sSlope_Calib.Value; 
    sLCDObject[__SET_PH_SLOPE].Scale_u8     = sSensor_pH.sSlope_Calib.Scale; 

    sLCDObject[__SCR_INFOR_FW_VERSION_2].pData   = sFirmVersion.Data_a8;
    sLCDObject[__SCR_INFOR_MODEL_2].pData   = sModelVersion.Data_a8;
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

/*=======================Func Callback=====================*/
static uint8_t _Cb_Display_Init (uint8_t event)
{
    if(sLCD.Ready_u8 == false)
    {
//        UTIL_Printf_Str(DBLEVEL_M, "u_lcd: init...\r\n");
        
//        HAL_GPIO_WritePin (LCD_ON_OFF_GPIO_Port, LCD_ON_OFF_Pin, GPIO_PIN_SET);   
//        HAL_GPIO_WritePin (LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET);
//        HAL_GPIO_WritePin (LCD_C86_GPIO_Port, LCD_C86_Pin, GPIO_PIN_RESET); 
        
        glcd_init();
        glcd_clear_buffer();
        glcd_write();	
        Deinit_LCD12864();
    
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
//            glcd_set_font(Liberation_Sans17x17_Alpha, 17, 17, 65, 90);
//            
//            glcd_draw_string_xy(30, 13, "SV");
//            
//            glcd_set_pixel(60, 25, BLACK);
//            glcd_set_pixel(60, 26, BLACK);
//            glcd_set_pixel(61, 26, BLACK);
//            glcd_set_pixel(61, 25, BLACK);
//            
//            glcd_draw_string_xy(65, 13, "JSC");
            
              glcd_tiny_set_font(Font5x7, 5, 7, 32, 127 + 10);
//            glcd_tiny_draw_string(34+4, 1, "CHAT LUONG");
//            glcd_tiny_draw_string(31+4, 2, "LA NIEM TIN");
//            https://saovietgroup.com.vn/
              glcd_tiny_draw_string(44, 1, "Website:");
              glcd_tiny_draw_string(8, 2, "saovietgroup.com.vn");
            
//            glcd_draw_rect(0, 0, 128, 64,BLACK);
            
//            glcd_set_font(Liberation_Sans11x14_Numbers,11,14,46,57);
//            glcd_draw_string_xy(4,0, ".963");
//            
//            glcd_set_font(Liberation_Sans11x14_Numbers,11,14,46,57);
//            glcd_draw_string_xy(4,16, ".852");
            
            glcd_write();	
            sEventDisplay[_EVENT_DISP_LOGO].e_period = 5000;
          break;
          
        case 2:
            sLCD.Ready_u8 = true;
            glcd_tiny_set_font(Font5x7, 5, 7, 32, 127 + 10);
            fevent_active(sEventDisplay, _EVENT_DISP_SHOW); 
            fevent_active(sEventDisplay, _EVENT_DISP_AUTO_SW); 
            fevent_active(sEventDisplay, _EVENT_BUTTON_SCAN); 
//            HAL_TIM_Base_Start_IT(&htim2);
          break;
          
        default:
          break;
    }
    step_u8++;
    fevent_enable(sEventDisplay, event);
    
    return 1;
}

static uint8_t _Cb_Display_Show (uint8_t event)
{
    static uint8_t ScreenLast = 0;
    static uint32_t LandMarkChange_u32 = 0;
    
    Update_ParaDisplay();
//    //update cac bien moi cai dat
//    Display_Update();
    //hien thi man hinh: index
    if (sLCD.Ready_u8 == true) {
        //neu index 9: modbus disp: gan lai cac bien va subindex
//        if (sLCD.sScreenNow.Index_u8 == _LCD_SCREEN_7b) {
//            Display_Setup_SCREEN_Modb();
//        }
        
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
    
    if (Check_Time_Out(sButton.LandMarkPressButton_u32, 60000*10) == true) {
        if (MarkButtPressed == true) {
            MarkButtPressed = false;
            sLCD.sScreenNow.Index_u8 = _LCD_SCREEN_1;
        }
        
        cNext++;
        if (cNext >= (DURATION_DISPALY/sEventDisplay[event].e_period)) {
            cNext = 0;
            
//            if (sLCD.sScreenNow.Index_u8 == _LCD_SCREEN_7b) {                
//                sLCD.sScreenNow.SubIndex_u8++;
//                if (sLCD.sScreenNow.SubIndex_u8 >= sWmDigVar.nModbus_u8) {
//                    sLCD.sScreenNow.Index_u8++;
//                }
//            } else {
//                if (sLCD.sScreenNow.Index_u8 == _LCD_SCREEN_7) {
//                    sLCD.sScreenNow.SubIndex_u8 = 0;
//                }
//                
//                sLCD.sScreenNow.Index_u8++;
//            }
            
            if (sLCD.sScreenNow.Index_u8 > _LCD_SCREEN_1) {
                sLCD.sScreenNow.Index_u8 = _LCD_SCREEN_1;
            }
        }
    } else {
        MarkButtPressed = true;
    }
    
    fevent_enable(sEventDisplay, event); 
    
	return 1;
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

/*==========================Function LCD=======================*/
void Update_ParaDisplay(void)
{
    sParaDisplay.ID_u8 = sSlave_ModbusRTU.ID;
    sParaDisplay.Baudrate_u32 = aBaudrate_value[sSlave_ModbusRTU.Baudrate];
    
    sParaDisplay.pH_Value_i32 = (int32_t)(sSensor_pH.pH_Value_f*100);
    sParaDisplay.Temp_Value_i32 = (int32_t)(sSensor_pH.temp_Value_f*100);
    
    sParaDisplay.pH_Filter_i32 = (int32_t)(sSensor_pH.pH_Filter_f*100);
    sParaDisplay.Temp_Filter_i32 = (int32_t)(sSensor_pH.temp_Filter_f*100);
    
    sParaDisplay.pH_Offset_i32   = (int32_t)(sSensor_pH.pH_Offset_f*100);
    sParaDisplay.temp_Offset_i32 = (int32_t)(sSensor_pH.temp_Offset_f*100);
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

        if(object == _LCD_SCREEN_1)
        {
          glcd_set_font(Liberation_Sans15x21_Numbers,15,21,46,57);
          glcd_draw_string_xy(PosX, sLCDObject[object].Row_u8, aTEMP);
          glcd_set_font(Liberation_Sans17x17_Alpha, 17, 17, 65, 90);
          glcd_draw_string_xy(100, 7, "M");
          glcd_tiny_set_font(Font5x7, 5, 7, 32, 127 + 10);
        }
        else
        {
          glcd_tiny_set_font(Font5x7, 5, 7, 32, 127 + 10);
          glcd_tiny_draw_string(PosX, sLCDObject[object].Row_u8, aTEMP);
        }
        PosX += strlen(aTEMP)* (font_current.width + 1);
    }
    
    //Show Unit

    if ( (sLCDObject[object].Unit != NULL)
        && ( (sLCDObject[object].pData != NULL) && (sLCDObject[object].pData != &charNotDetectPress) ) ) {
        glcd_tiny_draw_string(PosX, sLCDObject[object].Row_u8, (char *) sLCDObject[object].Unit );
    }
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

void Display_Show_Screen (uint8_t screen)
{
    uint16_t  i = 0; 
    
    //Clear buff lcd data
    glcd_clear_buffer();
//    LCD_Clear();
//    //Show static param: stime, icon internet,...
//    Display_Show_Static_Param();
//    
//    //Show state connect sensor
    Display_Show_State_Sensor_Network(screen);
    Display_Show_State_Setting(screen);
    
//    //Show name of screen
//    if (screen >= _LCD_SCR_CAL_CHANN_1) {
//        glcd_tiny_draw_string(0, 0, "Cal.");
//    } else if (screen >= _LCD_SCR_SETTING) {
//        glcd_tiny_draw_string(0, 0, "Set.");
//    }
    
    for (i = 0; i < __OJECT_END; i++) {
        if (sLCDObject[i].Screen_u8 == screen) {
            Display_Show_Oject(i);
        }
    }
    
    glcd_write();	
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
    Func: show static param
        + stime
        + icon: internet, baterry, "____"
*/
void Display_Show_State_Sensor_Network (uint8_t screen)
{
    if(screen == _LCD_SCREEN_1)
    {
        if(sSensor_pH.State_Connect == _SENSOR_DISCONNECT)
        {
            glcd_tiny_draw_string(120, sLCDObject[__SC1_PH_ATC].Row_u8, " ");
            glcd_tiny_draw_string(120, sLCDObject[__SC1_TEMP].Row_u8, " ");
        }
        else
        {
            glcd_tiny_draw_string(120, sLCDObject[__SC1_PH_ATC].Row_u8, "N");
            glcd_tiny_draw_string(120, sLCDObject[__SC1_TEMP].Row_u8, "N");
        }
    }
}

/*
    Func: show static param
        + stime
        + icon: internet, baterry, "____"
*/
void Display_Show_State_Setting (uint8_t screen)
{
    if(screen == _LCD_SCR_CHECK_SETTING)
    {
        switch(sParaDisplay.State_Setting)
        {
            case _STATE_SETTING_FREE:
              sLCDObject[__CHECK_STATE_SETTING].pData = aSTT_SETTING_FREE;
              break;
              
            case _STATE_SETTING_ENTER:
              sLCDObject[__CHECK_STATE_SETTING].pData = aSTT_SETTING_ENTER;
              break;

            case _STATE_SETTING_WAIT:
              sLCDObject[__CHECK_STATE_SETTING].pData = aSTT_SETTING_WAIT;
              break;
              
            case _STATE_SETTING_DONE:
              sLCDObject[__CHECK_STATE_SETTING].pData = aSTT_SETTING_DONE;
              break;
              
            case _STATE_SETTING_ERROR:
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

void Deinit_LCD12864(void)
{
    glcd_command(0xa0); /* ADC select in normal mode */
	glcd_command(0xae); /* Display OFF */
	glcd_command(0xc0); /* Common output mode select: reverse direction (last 3 bits are ignored) */
	glcd_command(0xa1); 
    glcd_command(0xa2); /* LCD bias set at 1/9 */
	glcd_command(0x2f); /* Power control set to operating mode: 7 */
	glcd_command(0x21); /* Internal resistor ratio, set to: 1 */
	glcd_set_contrast(40); /* Set contrast, value experimentally determined, can set to 6-bit value, 0 to 63 */
	glcd_command(0xaf); /* Display on */
}

uint32_t test_tim2 = 0;
