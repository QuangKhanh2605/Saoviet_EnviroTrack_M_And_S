

#include "button.h"
#include "lcd_display.h"

sButtonStruct sButton;


/*============ Function =============*/

void BUTTON_Process(void)
{
    switch(sButton.Value)
    {
        case _ENTER:  
            BUTTON_Enter_Process();
            break;
        case _UP:  
            BUTTON_Up_Process();
            break;
        case _DOWN:               
            BUTTON_Down_Process();
            break;
        case _ESC:
            BUTTON_ESC_Process();
            break;
        default:
            break;
    }
}



uint8_t BUTTON_scan(void)
{
    uint8_t r_value = 0;
    static uint8_t last_value = 0;
    static uint32_t hold = 0;
    
	if (sButton.Status == 0)
	{
		r_value = (!BUTTON_ENTER<<0)|(!BUTTON_UP<<1)|(!BUTTON_DOWN<<2)|(!BUTTON_ESC<<3);
        
		switch(r_value)
		{
			case _ENTER:
            case _UP:
            case _DOWN:
            case _ESC:
                sButton.Count++;
                if (sButton.Count >= BUT_TIME_DETECT) {
                    sButton.Value = r_value;
                    sButton.Status = 1;
                    
                    if ( r_value == last_value ) {
                        hold++;
                        if ( hold >= BUT_COUNT_FAST ) {
                            sButton.Count = BUT_TIME_DETECT;
                            hold = BUT_COUNT_FAST;
                        } else if (hold >= BUT_COUNT_HOLD) {
                            sButton.Count = BUT_TIME_DETECT - 2;
                        } else {
                            sButton.Count = 0;
                        }
                    } else {
                        sButton.Count = 0;  
                        hold = 0;
                    } 
                                
                    last_value = r_value;
                }
  				break;
			default:
                last_value = 0;
				sButton.Count = BUT_TIME_DETECT/2;
                hold = 0;
				break;
		}
	}

    return sButton.Value;
}

static char aTEST[10] = {"1234"};
static sPressureLinearInter sPressConfig = {0};
static Struct_Pulse sPulseConfig = {0};
static int16_t PressOldCalib_i16 = 0;
static char sConfirm[2][10] = {"No", "Yes"};

float       Old_Value_f = 0;
uint32_t    Old_Value_u32 = 0;
int16_t     Old_Value_i16 = 0;
float       Button_Stamp1_f = 0;
float       Button_Stamp2_f = 0;
      
void BUTTON_Enter_Process (void)
{
    static uint8_t tempu8 = 0;
    static uint8_t TempScale = 1;
    


    switch (sLCD.sScreenNow.Index_u8)
    {
        case _LCD_SCREEN_1:
        case _LCD_SCREEN_2:
        case _LCD_SCREEN_3:
        case _LCD_SCREEN_4:
        case _LCD_SCREEN_5:
        case _LCD_SCREEN_6:
        case _LCD_SCREEN_7:
        case _LCD_SCREEN_7b:
            sButton.Old_value = 0;
            UTIL_MEM_set(aTEST, 0, sizeof(aTEST));
            aTEST[0] = '0'; aTEST[1] = '0'; aTEST[2] = '0'; aTEST[3] = '0';

            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_PASS, 0,
                               __PASS_WORD_2, __PASS_WORD_1, __PASS_WORD_2,
                               aTEST, (sButton.Old_value << 4) + 0x02);
            break;
        case _LCD_SCR_PASS:
            sButton.Old_value++;
            if (sButton.Old_value > 3)
            { 
                if (Display_Check_Password((uint8_t *) aTEST) == true) {
                    //copy screen now to screenback
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                                       __SET_REQ_1, __SET_REQ_1, __DEVICE_INFOR,
                                       NULL, 0xF1);
                } else {
                    sLCD.sScreenNow.Index_u8 = _LCD_SCREEN_1;  //thoat
                }
            } else {
                Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, (sButton.Old_value << 4) + 0x02);
            }
            break;
            
        case _LCD_SCR_SETTING:
            switch (sLCD.sScreenNow.Para_u8)
            {
                case __SET_REQ_1:
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_FREQ, 0,
                                       __SET_REQ_2_2, __SET_REQ_2_1, __SET_REQ_2_2,
                                       &sButton.Old_value, 0xF2);

                    sButton.Old_value = sModemInfor.sFrequence.DurOnline_u32;
                    break;
                    
                case __SET_CALIB_SENSOR:
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    
                    sButton.Old_value = 0;
                    UTIL_MEM_set(aTEST, 0, sizeof(aTEST));
                    aTEST[0] = '0'; aTEST[1] = '0'; aTEST[2] = '0'; aTEST[3] = '0';

                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_PASSCALIB, 0,
                                       __PASS_CALIB_2, __PASS_CALIB_1, __PASS_CALIB_2,
                                       aTEST, (sButton.Old_value << 4) + 0x02);
                    break;
                    
                case __DEVICE_INFOR:
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_INFORMATION, 0,
                                       __INFOR_FW_VERSION_1, __INFOR_FW_VERSION_1, __INFOR_FW_VERSION_2,
                                       NULL, 0x00);
                    break;
                    
                case __SET_PULSE_SETT:
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_PULSE_1, 0,
                                       __SET_PULSE_SET1_1, __SET_PULSE_SET1_1, __SET_PULSE_SET1_4,
                                       NULL, 0xF1);
                    break;
                case __SET_RESTORE_PULSE:
                    sButton.Old_value = 0;
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_PULSE_RS, 0,
                                       __SET_P_RS_CONF_2, __SET_P_RS_CONF_1, __SET_P_RS_CONF_2,
                                       sConfirm[sButton.Old_value], 0xF2);  
                    break;
                case __SET_PRESSURE:
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_PRESS_1, 0,
                                       __SET_PRESSURE_1_1, __SET_PRESSURE_1_1, __SET_PRESSURE_1_5,
                                       NULL, 0xF1);
                    break;
                case __SET_PRESS_CALIB:
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_CAL_CHANN_1, 0,
                                       __CAL_PRESS_CH_1, __CAL_PRESS_CH_1, __CAL_PRESS_CH_5,
                                       NULL, 0xF1);
                    break;
            }
            
            break;
            
        case _LCD_SCR_PASSCALIB:
            sButton.Old_value++;
            if (sButton.Old_value > 3)
            { 
                if (Display_Check_PassCalib((uint8_t *) aTEST) == true) {
                    //copy screen now to screenback
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB, 0,
                                       __SET_CALIB_SS_PH, __SET_CALIB_SS_PH, __SET_CALIB_VALUE,
                                       NULL, 0xF1);
                } else {
                    UTIL_MEM_cpy(&sLCD.sScreenNow, &sLCD.sScreenBack, sizeof(sScreenInformation));
                }
            } else {
                Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, (sButton.Old_value << 4) + 0x02);
            }
            break;
            
        case _LCD_SCR_SET_CALIB:
            switch (sLCD.sScreenNow.Para_u8)
            {
                case __SET_CALIB_SS_PH:
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_PH, 0,
                                       __SET_PH_Z_SOLUTION, __SET_PH_Z_SOLUTION, __SET_PH_SLOPE,
                                       NULL, 0xF1);
                    break;
                    
                case __SET_CALIB_SS_CLO:
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_CLO, 0,
                                       __SET_CLO_ZERO, __SET_CLO_ZERO, __SET_CLO_CLB_TEMP,
                                       NULL, 0xF1);
                    break;
                    
                case __SET_CALIB_SS_EC:
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_EC, 0,
                                       __SET_EC_CONST, __SET_EC_CONST, __SET_EC_CALIB,
                                       NULL, 0xF1);
                    break;
                    
                case __SET_CALIB_SS_TUR:
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_TUR_TAB_1, 0,
                                       __SET_TUR_VALUE_P1, __SET_TUR_VALUE_P1, __SET_TUR_AD_P2,
                                       NULL, 0xF1);
                    break;
                    
                case __SET_CALIB_VALUE:
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_VALUE, 0,
                                       __SET_VALUE_CLO_DU, __SET_VALUE_EC, __SET_VALUE_CLO_DU,
                                       NULL, 0x00);
                    break;
            }
            break;
            
        case _LCD_SCR_SET_CALIB_SS_PH:
            switch (sLCD.sScreenNow.Para_u8)
            {
                case __SET_PH_Z_SOLUTION:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_PH, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_PH_Z_SOLUTION, __SET_PH_Z_SOLUTION, __SET_PH_SLOPE,
                                               &sButton.Old_value, 0xF2);
                             sButton.Old_value = sSensor_pH.sZero_Calib.Value;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_STATE, 0,
                                               __SET_CALIB_STATE, __SET_CALIB_STATE, __SET_CALIB_STATE,
                                               NULL, 0xF0);
                            
                            Old_Value_i16 = _fRead_pH_Zero_Point(sButton.Old_value);
                            RS485_LogData_Calib(_RS485_SS_PH_CALIB_ZERO_STD_SOLUTION, &Old_Value_i16, sizeof(int16_t));
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                case __SET_PH_ZERO:
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_STATE, 0,
                                       __SET_CALIB_STATE, __SET_CALIB_STATE, __SET_CALIB_STATE,
                                       NULL, 0xF0);
                    
                    Old_Value_i16 = 0x00FF;
                    RS485_LogData_Calib(_RS485_SS_PH_CALIB_ZERO, &Old_Value_i16, sizeof(int16_t));
                    break;
                    
                case __SET_PH_S_SOLUTION:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_PH, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_PH_S_SOLUTION, __SET_PH_Z_SOLUTION, __SET_PH_SLOPE,
                                               &sButton.Old_value, 0xF2);
                             sButton.Old_value = sSensor_pH.sSlope_Calib.Value;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_STATE, 0,
                                               __SET_CALIB_STATE, __SET_CALIB_STATE, __SET_CALIB_STATE,
                                               NULL, 0xF0);
                            Old_Value_i16 = _fRead_pH_Slope_Point(sButton.Old_value);
                            RS485_LogData_Calib(_RS485_SS_PH_CALIB_SLOPE_STD_SOLUTION, &Old_Value_i16, sizeof(int16_t));
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                case __SET_PH_SLOPE:
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_STATE, 0,
                                       __SET_CALIB_STATE, __SET_CALIB_STATE, __SET_CALIB_STATE,
                                       NULL, 0xF0);

                    Old_Value_i16 = 0x00FF;
                    RS485_LogData_Calib(_RS485_SS_PH_CALIB_SLOPE, &Old_Value_i16, sizeof(int16_t));
                    break;
            }
            break;
            
        case _LCD_SCR_SET_CALIB_SS_CLO:
            switch (sLCD.sScreenNow.Para_u8)
            {  
                case __SET_CLO_ZERO:
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_STATE, 0,
                                       __SET_CALIB_STATE, __SET_CALIB_STATE, __SET_CALIB_STATE,
                                       NULL, 0xF0);
                    Old_Value_i16 = 0x00FF;
                    DCU_Logdata_Calib(_DCU_CALIB_CLO_ZERO, sButton.Old_value);
                    break;
                    
                case __SET_CLO_SLOPE:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_CLO, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_CLO_SLOPE, __SET_CLO_SLOPE, __SET_CLO_CLB_TEMP,
                                               &sButton.Old_value, 0xF2);
                             sButton.Old_value = sConvertChlorine.Clo_Calib_Slope;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_STATE, 0,
                                               __SET_CALIB_STATE, __SET_CALIB_STATE, __SET_CALIB_STATE,
                                               NULL, 0xF0);
                            Old_Value_f = Handle_int32_To_Float_Scale(sButton.Old_value, sSensor_Clo.sSolution_Calibration.Scale);
                            DCU_Logdata_Calib(_DCU_CALIB_CLO_SLOPE, sButton.Old_value);
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                case __SET_CLO_CLB_PH_1:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_CLO, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_CLO_CLB_PH_1, __SET_CLO_CLB_PH_1, __SET_CLO_CLB_TEMP,
                                               &sButton.Old_value, 0xF2);
                            sButton.Old_value = sConvertChlorine.Clo_CalibPoint_1;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_STATE, 0,
                                               __SET_CALIB_STATE, __SET_CALIB_STATE, __SET_CALIB_STATE,
                                               NULL, 0xF0);
                            Old_Value_f = Handle_int32_To_Float_Scale(sButton.Old_value, sSensor_Clo.sSolution_Calibration.Scale);
                            DCU_Logdata_Calib(_DCU_CALIB_CLO_POINT1, sButton.Old_value);
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                case __SET_CLO_CLB_PH_2:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_CLO, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_CLO_CLB_PH_2, __SET_CLO_CLB_PH_2, __SET_CLO_CLB_TEMP,
                                               &sButton.Old_value, 0xF2);
                             sButton.Old_value = sConvertChlorine.Clo_CalibPoint_2;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_STATE, 0,
                                               __SET_CALIB_STATE, __SET_CALIB_STATE, __SET_CALIB_STATE,
                                               NULL, 0xF0);
                            Old_Value_f = Handle_int32_To_Float_Scale(sButton.Old_value, sSensor_Clo.sSolution_Calibration.Scale);
                            DCU_Logdata_Calib(_DCU_CALIB_CLO_POINT2, sButton.Old_value);
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                case __SET_CLO_CLB_TEMP:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_CLO, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_CLO_CLB_TEMP, __SET_CLO_CLB_TEMP, __SET_CLO_CLB_TEMP,
                                               &sButton.Old_value, 0xF2);
                             sButton.Old_value = sConvertChlorine.sConst_Compensation_Temp.Value;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_STATE, 0,
                                               __SET_CALIB_STATE, __SET_CALIB_STATE, __SET_CALIB_STATE,
                                               NULL, 0xF0);
                            Old_Value_f = Handle_int32_To_Float_Scale(sButton.Old_value, sSensor_Clo.sSolution_Calibration.Scale);
                            DCU_Logdata_Calib(_DCU_CALIB_CLO_CONST_TEMP, sButton.Old_value);
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                    default:
                      break;
            }
            break;
            
        case _LCD_SCR_SET_CALIB_SS_EC:
            switch (sLCD.sScreenNow.Para_u8)
            {
                case __SET_EC_CONST:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_EC, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_EC_CONST, __SET_EC_CONST, __SET_EC_CALIB,
                                               &sButton.Old_value, 0xF2);
                            sButton.Old_value = sSensor_EC.sConductivity_Const.Value;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_STATE, 0,
                                               __SET_CALIB_STATE, __SET_CALIB_STATE, __SET_CALIB_STATE,
                                               NULL, 0xF0);
                            Old_Value_f = 1;

                            RS485_LogData_Calib(_RS485_SS_EC_CALIB_CONST_STD, &Old_Value_f, sizeof(float));
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                case __SET_EC_CALIB:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_EC, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_EC_CALIB, __SET_EC_CONST, __SET_EC_CALIB,
                                               &sButton.Old_value, 0xF2);
                            sButton.Old_value = sSensor_EC.sConductivity_Calib.Value;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_STATE, 0,
                                               __SET_CALIB_STATE, __SET_CALIB_STATE, __SET_CALIB_STATE,
                                               NULL, 0xF0);
                            Old_Value_f = 1;
                            Button_Stamp1_f = Handle_int32_To_Float_Scale(sSensor_EC.sConductivity_Value.Value, sSensor_EC.sConductivity_Value.Scale);
                            Button_Stamp2_f = Handle_int32_To_Float_Scale(sButton.Old_value, sSensor_EC.sConductivity_Calib.Scale);
                            
                            if(Button_Stamp1_f != 0)
                                Old_Value_f = Button_Stamp2_f/Button_Stamp1_f;
                            
                            RS485_LogData_Calib(_RS485_SS_EC_CALIB_CONST_REAL, &Old_Value_f, sizeof(int32_t));
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                default:
                  break;
            }
            break;
            
        case _LCD_SCR_SET_CALIB_SS_TUR_TAB_1:
            switch (sLCD.sScreenNow.Para_u8)
            {
                case __SET_TUR_VALUE_P1:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_TUR_TAB_1, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_TUR_VALUE_P1, __SET_TUR_VALUE_P1, __SET_TUR_AD_P2,
                                               &sButton.Old_value, 0xF2);
                            sButton.Old_value = sSensor_Turbidity.sFirst_Calibration.Value;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_STATE, 0,
                                               __SET_CALIB_STATE, __SET_CALIB_STATE, __SET_CALIB_STATE,
                                               NULL, 0xF0);
                            
                            Old_Value_f = Handle_int32_To_Float_Scale(sButton.Old_value, sSensor_Turbidity.sFirst_Calibration.Scale);
                            RS485_LogData_Calib(_RS485_SS_TUR_CALIB_1ST_VALUE, &Old_Value_f, sizeof(float));
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                case __SET_TUR_AD_P1:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_TUR_TAB_1, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_TUR_AD_P1, __SET_TUR_VALUE_P1, __SET_TUR_AD_P2,
                                               &sButton.Old_value, 0xF2);
                            sButton.Old_value = sSensor_Turbidity.First_AD_u32;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_STATE, 0,
                                               __SET_CALIB_STATE, __SET_CALIB_STATE, __SET_CALIB_STATE,
                                               NULL, 0xF0);
                            
                            Old_Value_u32 = sButton.Old_value;
                            RS485_LogData_Calib(_RS485_SS_TUR_CALIB_1ST_AD, &Old_Value_u32, sizeof(uint32_t));
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                case __SET_TUR_VALUE_P2:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_TUR_TAB_1, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_TUR_VALUE_P2, __SET_TUR_VALUE_P1, __SET_TUR_AD_P2,
                                               &sButton.Old_value, 0xF2);
                            sButton.Old_value = sSensor_Turbidity.sSecond_Calibration.Value;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_STATE, 0,
                                               __SET_CALIB_STATE, __SET_CALIB_STATE, __SET_CALIB_STATE,
                                               NULL, 0xF0);
                            
                            Old_Value_f = Handle_int32_To_Float_Scale(sButton.Old_value, sSensor_Turbidity.sSecond_Calibration.Scale);
                            RS485_LogData_Calib(_RS485_SS_TUR_CALIB_2ND_VALUE, &Old_Value_f, sizeof(float));
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                case __SET_TUR_AD_P2:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_TUR_TAB_1, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_TUR_AD_P2, __SET_TUR_VALUE_P1, __SET_TUR_AD_P2,
                                               &sButton.Old_value, 0xF2);
                            sButton.Old_value = sSensor_Turbidity.Second_AD_u32;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_STATE, 0,
                                               __SET_CALIB_STATE, __SET_CALIB_STATE, __SET_CALIB_STATE,
                                               NULL, 0xF0);
                            
                            Old_Value_u32 = sButton.Old_value;
                            RS485_LogData_Calib(_RS485_SS_TUR_CALIB_2ND_AD, &Old_Value_u32, sizeof(uint32_t));
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                default:
                  break;
            }
            break;
            
        case _LCD_SCR_SET_CALIB_SS_TUR_TAB_2:
            switch (sLCD.sScreenNow.Para_u8)
            {
                case __SET_TUR_VALUE_P3:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_TUR_TAB_2, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_TUR_VALUE_P3, __SET_TUR_VALUE_P3, __SET_TUR_AD_P4,
                                               &sButton.Old_value, 0xF2);
                            sButton.Old_value = sSensor_Turbidity.sThird_Calibration.Value;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_STATE, 0,
                                               __SET_CALIB_STATE, __SET_CALIB_STATE, __SET_CALIB_STATE,
                                               NULL, 0xF0);

                            Old_Value_f = Handle_int32_To_Float_Scale(sButton.Old_value, sSensor_Turbidity.sThird_Calibration.Scale);
                            RS485_LogData_Calib(_RS485_SS_TUR_CALIB_3RD_VALUE, &Old_Value_f, sizeof(float));
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                case __SET_TUR_AD_P3:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_TUR_TAB_2, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_TUR_AD_P3, __SET_TUR_VALUE_P3, __SET_TUR_AD_P4,
                                               &sButton.Old_value, 0xF2);
                            sButton.Old_value = sSensor_Turbidity.Third_AD_u32;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_STATE, 0,
                                               __SET_CALIB_STATE, __SET_CALIB_STATE, __SET_CALIB_STATE,
                                               NULL, 0xF0);

                            Old_Value_u32 = sButton.Old_value;
                            RS485_LogData_Calib(_RS485_SS_TUR_CALIB_3RD_AD, &Old_Value_u32, sizeof(uint32_t));
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                case __SET_TUR_VALUE_P4:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_TUR_TAB_2, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_TUR_VALUE_P4, __SET_TUR_VALUE_P3, __SET_TUR_AD_P4,
                                               &sButton.Old_value, 0xF2);
                            sButton.Old_value = sSensor_Turbidity.sFourth_Calibration.Value;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_STATE, 0,
                                               __SET_CALIB_STATE, __SET_CALIB_STATE, __SET_CALIB_STATE,
                                               NULL, 0xF0);

                            Old_Value_f = Handle_int32_To_Float_Scale(sButton.Old_value, sSensor_Turbidity.sFourth_Calibration.Scale);
                            RS485_LogData_Calib(_RS485_SS_TUR_CALIB_4TH_VALUE, &Old_Value_f, sizeof(float));
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                case __SET_TUR_AD_P4:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_TUR_TAB_2, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_TUR_AD_P4, __SET_TUR_VALUE_P3, __SET_TUR_AD_P4,
                                               &sButton.Old_value, 0xF2);
                            sButton.Old_value = sSensor_Turbidity.Fourth_AD_u32;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_STATE, 0,
                                               __SET_CALIB_STATE, __SET_CALIB_STATE, __SET_CALIB_STATE,
                                               NULL, 0xF0);
                            
                            Old_Value_u32 = sButton.Old_value;
                            RS485_LogData_Calib(_RS485_SS_TUR_CALIB_4TH_AD, &Old_Value_u32, sizeof(uint32_t));
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                default:
                  break;
            }
            break;
        
        case _LCD_SCR_SET_CALIB_STATE:
          switch(sLCD.sScreenBack.Index_u8)
          {
              case _LCD_SCR_SET_CALIB_SS_CLO:
                DCU_Enter_Calib();
                break;
                
              default:
                RS485_Enter_Calib();
                break;
          }
          break;
          
        case _LCD_SCR_SETTING_2:
            switch (sLCD.sScreenNow.Para_u8)
            {
                case __SET_MANUFACTOR:
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCREEN_8, 0,
                                       __INFORMATION, __INFORMATION, __FW_VERSION_1,
                                       NULL, 0xF0);
                    break;
            }
            break;
        case _LCD_SCR_SET_FREQ:
            sModemInfor.sFrequence.DurOnline_u32 = sButton.Old_value; 
            //Luu lai gia tri
            Modem_Save_Freq();
            AppComm_Set_Next_TxTimer();

            //quay lai man cu
            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_RESTORE, 0,
                               __SET_RESTORE_1, __SET_RESTORE_1, __SET_RESTORE_1,
                               NULL, 0xF0); 
            break;
        case _LCD_SCR_SET_PRESS_1:
        case _LCD_SCR_SET_PRESS_1_2:
            //Set lai con tro data press
            tempu8 = sLCD.sScreenNow.Para_u8 - __SET_PRESSURE_1_1;
            UTIL_MEM_cpy(&sPressConfig, &sWmVar.aPRESSURE[tempu8].sLinearInter, sizeof(sPressureLinearInter));
            sLCDObject[__SET_PRESSURE_2_0].pData = AnalogType[sPressConfig.Type_u8];  
            sLCDObject[__SET_PRESSURE_2_1].pData = &sPressConfig.Factor_u16; 
            sLCDObject[__SET_PRESSURE_2_2].pData = &sPressConfig.InMin_u16; 
            sLCDObject[__SET_PRESSURE_2_3].pData = &sPressConfig.InMax_u16; 
            sLCDObject[__SET_PRESSURE_2_4].pData = aUnitWm[sPressConfig.InUnit_u8]; 
            sLCDObject[__SET_PRESSURE_2_5].pData = &sPressConfig.OutMin_u16; 
            sLCDObject[__SET_PRESSURE_2_6].pData = &sPressConfig.OutMax_u16; 
            sLCDObject[__SET_PRESSURE_2_7].pData = aUnitWm[sPressConfig.OutUnit_u8];
            //
            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_PRESS_2, 0,
                                       __SET_PRESSURE_2_0, __SET_PRESSURE_2_0, __SET_PRESSURE_2_7,
                                       NULL, 0xF2);
            break;
        case _LCD_SCR_SET_PRESS_2:               
            sLCD.sScreenNow.Para_u8++;
            Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF2);
            
            if (sLCD.sScreenNow.Para_u8 > sLCD.sScreenNow.ParaMax_u8) {
                //Copy data ra bien 
                UTIL_MEM_cpy(&sWmVar.aPRESSURE[tempu8].sLinearInter, &sPressConfig, sizeof(sPressureLinearInter));
                //luu gia tri
                AppWm_Save_Press_Infor();
                //cap nhat len server
                sMQTT.aMARK_MESS_PENDING[DATA_HANDSHAKE] = TRUE;
                
                //ket thuc chuyen man
                Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_RESTORE, 0,
                                   __SET_RESTORE_1, __SET_RESTORE_1, __SET_RESTORE_1,
                                   NULL, 0xF0); 
            }
            
            break;
        case _LCD_SCR_SET_PULSE_1:
            sButton.Old_value = 0;
            tempu8 = sLCD.sScreenNow.Para_u8 - __SET_PULSE_SET1_1;
            UTIL_MEM_cpy(&sPulseConfig, &sPulse[tempu8], sizeof(Struct_Pulse));
            
            UTIL_MEM_set(aTEST, 0, sizeof(aTEST));
            UtilIntToStringWithScale (sPulseConfig.Start_u32, aTEST, 0xFF - sPulseConfig.Factor_u8 + 1);
      
            sLCDObject[__SET_PULSE_SET2_1].pData = &TempScale;  //so 1
            sLCDObject[__SET_PULSE_SET2_1].Scale_u8 = sPulseConfig.Factor_u8; 
            sLCDObject[__SET_PULSE_SET2_2].pData = aTEST; 
                
            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_PULSE_2, 0,
                                       __SET_PULSE_SET2_1, __SET_PULSE_SET2_1, __SET_PULSE_SET2_2,
                                       NULL, 0xF2);
            break;
        case _LCD_SCR_SET_PULSE_2:
            if (sLCD.sScreenNow.Para_u8 == __SET_PULSE_SET2_1) {
                sPulseConfig.Factor_u8 = sLCDObject[sLCD.sScreenNow.Para_u8].Scale_u8;
                sLCD.sScreenNow.Para_u8++;
                Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, (sButton.Old_value << 4) + 0x02);
                
                UTIL_MEM_set(aTEST, 0, sizeof(aTEST));
                aTEST[0] = '0';
            } else {
                if (sLCD.sScreenNow.Flag_u8 == 0xF0) {
                    //convert lai so start
                    sPulse[tempu8].Start_u32 = (uint32_t) ( UtilStringToFloat(aTEST) * Convert_Scale(0xFF - sPulseConfig.Factor_u8 + 1)) ;  
                    sPulse[tempu8].Factor_u8 = sPulseConfig.Factor_u8;
                    //luu gia tri
                    AppWm_Save_Pulse();
                    //cap nhat len server
                    sMQTT.aMARK_MESS_PENDING[DATA_HANDSHAKE] = TRUE;
                    //ket thuc chuyen man
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_RESTORE, 0,
                                       __SET_RESTORE_1, __SET_RESTORE_1, __SET_RESTORE_1,
                                       NULL, 0xF0); 
                } else { 
                    if ( (sButton.Old_value >= 9) || (aTEST[sButton.Old_value] == '_') ) {
                        if (aTEST[sButton.Old_value] == '_') {
                            aTEST[sButton.Old_value] = 0;
                        }
                             
                        Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF0);
                        break;
                    } else {  
                        sButton.Old_value++; 
                        aTEST[sButton.Old_value] = '_';
                        Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, (sButton.Old_value << 4) + 0x02);
                    }
                }
            }
            break;
        case _LCD_SCR_PULSE_RS:
            if (sButton.Old_value == 1) {
                AppWm_Default_Pulse();
                
                Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_RESTORE, 0,
                                   __SET_RESTORE_1, __SET_RESTORE_1, __SET_RESTORE_1,
                                   NULL, 0xF0); 
            } else {
                UTIL_MEM_cpy(&sLCD.sScreenNow, &sLCD.sScreenBack, sizeof(sScreenInformation));
            }
            break; 
        case _LCD_SCR_SET_RESTORE:
            UTIL_MEM_cpy(&sLCD.sScreenNow, &sLCD.sScreenBack, sizeof(sScreenInformation));
            break;
        case _LCD_SCR_CAL_CHANN_1:
        case _LCD_SCR_CAL_CHANN_2:
            tempu8 = sLCD.sScreenNow.Para_u8 - __CAL_PRESS_CH_1;
            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_CAL_PRESS, 0,
                               __CAL_PRESSURE_2, __CAL_PRESSURE_1, __CAL_PRESSURE_4,
                               &sWmVar.aPRESSURE[tempu8].mVol_i32, 0xF2);
            
            sLCDObject[__CAL_PRESSURE_1].pData  = &PressOldCalib_i16;
            
            PressOldCalib_i16 = sWmVar.aPRESSURE[tempu8].Calib_i16;
            sButton.Old_value = tempu8;
            sWmVar.aPRESSURE[tempu8].Calib_i16 = 0;
            break;
        case _LCD_SCR_CAL_PRESS:
            //ghi nhan gia tri va luu lai
            sWmVar.aPRESSURE[tempu8].Calib_i16 = sWmVar.aPRESSURE[tempu8].mVol_i32;
            AppWm_Save_Press_Infor();
            
            //ket thuc chuyen man
            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_RESTORE, 0,
                                   __SET_RESTORE_1, __SET_RESTORE_1, __SET_RESTORE_1,
                                   NULL, 0xF0); 
            break;
        case _LCD_SCREEN_8:
            UTIL_MEM_cpy(&sLCD.sScreenNow, &sLCD.sScreenBack, sizeof(sScreenInformation));
            break;
        default:
            break;
    }
}

void BUTTON_Up_Process (void)
{
    switch (sLCD.sScreenNow.Index_u8)
    {
        case _LCD_SCREEN_1:
//            sButton.Old_value = _LCD_SCREEN_1;
//            sLCD.sScreenNow.Index_u8 = _LCD_SCREEN_7b;
//            if (sWmDigVar.nModbus_u8 >= 1) {
//                sLCD.sScreenNow.SubIndex_u8 = sWmDigVar.nModbus_u8 - 1;
//            } else {
//                sLCD.sScreenNow.SubIndex_u8 = 0;
//            }
            break;
        case _LCD_SCREEN_2:
        case _LCD_SCREEN_3:
        case _LCD_SCREEN_4:
        case _LCD_SCREEN_5:
        case _LCD_SCREEN_6:
        case _LCD_SCREEN_7:
            sLCD.sScreenNow.Index_u8--;
            break;
        case _LCD_SCREEN_7b:
            sButton.Old_value = _LCD_SCREEN_1;
            if (sLCD.sScreenNow.SubIndex_u8 > 0) {
                sLCD.sScreenNow.SubIndex_u8--;
            } else {
                sLCD.sScreenNow.Index_u8--;
            }
            break;
        case _LCD_SCR_PASS:
        case _LCD_SCR_PASSCALIB:
            aTEST[sButton.Old_value] ++;
            if (aTEST[sButton.Old_value] > '9')
                aTEST[sButton.Old_value] = '0';
            break;
        case _LCD_SCR_SETTING:
        case _LCD_SCR_SET_PRESS_1:
        case _LCD_SCR_CAL_CHANN_1:
            if (sLCD.sScreenNow.Para_u8 > sLCD.sScreenNow.ParaMin_u8 ) {
                sLCD.sScreenNow.Para_u8--;
            }
            Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
            break;
        case _LCD_SCR_SETTING_2:
            if (sLCD.sScreenNow.Para_u8 > sLCD.sScreenNow.ParaMin_u8 ) {
                sLCD.sScreenNow.Para_u8--;
                Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
            } else {
                Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                                       __SET_PRESS_CALIB, __SET_REQ_1, __SET_PRESS_CALIB,
                                       NULL, 0xF1);
            }
            break;
        case _LCD_SCR_SET_PULSE_1:
            if (sLCD.sScreenNow.Para_u8 > sLCD.sScreenNow.ParaMin_u8) {
                sLCD.sScreenNow.Para_u8--;
            } else {
                sLCD.sScreenNow.Para_u8 = sLCD.sScreenNow.ParaMax_u8;
            }
            Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
            break;
        case _LCD_SCR_CAL_CHANN_2:
            if (sLCD.sScreenNow.Para_u8 > sLCD.sScreenNow.ParaMin_u8 ) {
                sLCD.sScreenNow.Para_u8--;
                Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
            } else {
                Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_CAL_CHANN_1, 0,
                                       __CAL_PRESS_CH_5, __CAL_PRESS_CH_1, __CAL_PRESS_CH_5,
                                       NULL, 0xF1);
            }
            break;
        case _LCD_SCR_SET_FREQ:
            sButton.Old_value ++; 
            break;
            
        case _LCD_SCR_SET_CALIB_SS_PH:
              if(sLCD.sScreenNow.SubIndex_u8 == 0)
              {
                    if (sLCD.sScreenNow.Para_u8 > sLCD.sScreenNow.ParaMin_u8 ) {
                        sLCD.sScreenNow.Para_u8--;
                    }
                    Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
              }
              else
              {
                switch (sLCD.sScreenNow.Para_u8)
                {
                    case __SET_PH_Z_SOLUTION:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                sButton.Old_value = _fSet_pH_Zero_Calib_UpDown(sButton.Old_value, 1);
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    case __SET_PH_ZERO:
                        break;
                        
                    case __SET_PH_S_SOLUTION:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                sButton.Old_value = _fSet_pH_Slope_Calib_UpDown(sButton.Old_value, 1);
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    case __SET_PH_SLOPE:
                        break;
                }
            }
            break;
            
        case _LCD_SCR_SET_CALIB_SS_CLO:
              if(sLCD.sScreenNow.SubIndex_u8 == 0)
              {
                    if (sLCD.sScreenNow.Para_u8 > sLCD.sScreenNow.ParaMin_u8 ) {
                        sLCD.sScreenNow.Para_u8--;
                    }
                    Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
              }
              else
              {
                switch (sLCD.sScreenNow.Para_u8)
                { 
                    case __SET_CLO_ZERO:
                        break;
                        
                    case __SET_CLO_SLOPE:
                    case __SET_CLO_CLB_PH_1:
                    case __SET_CLO_CLB_PH_2:
                    case __SET_CLO_CLB_TEMP:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                sButton.Old_value++;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
//                    case __SET_CLO_CLB_PH_2:
//                        break;
                }
            }
            break;
            
        case _LCD_SCR_SET_CALIB_SS_EC:
              if(sLCD.sScreenNow.SubIndex_u8 == 0)
              {
                    if (sLCD.sScreenNow.Para_u8 > sLCD.sScreenNow.ParaMin_u8 ) {
                        sLCD.sScreenNow.Para_u8--;
                    }
                    Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
              }
              else
              {
                switch (sLCD.sScreenNow.Para_u8)
                {
                    case __SET_EC_CONST:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                sButton.Old_value = 1000;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    case __SET_EC_CALIB:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                sButton.Old_value++;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    default:
                      break;
                }
            }
            break;
            
        case _LCD_SCR_SET_CALIB_SS_TUR_TAB_1:
              if(sLCD.sScreenNow.SubIndex_u8 == 0)
              {
                    if (sLCD.sScreenNow.Para_u8 > sLCD.sScreenNow.ParaMin_u8 ) {
                        sLCD.sScreenNow.Para_u8--;
                    }
                    Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
              }
              else
              {
                switch (sLCD.sScreenNow.Para_u8)
                {
                    case __SET_TUR_VALUE_P1:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                sButton.Old_value++;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    case __SET_TUR_AD_P1:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                sButton.Old_value = sSensor_Turbidity.Measure_AD;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    case __SET_TUR_VALUE_P2:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                sButton.Old_value++;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    case __SET_TUR_AD_P2:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                sButton.Old_value = sSensor_Turbidity.Measure_AD;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    default:
                      break;
                }
            }
            break;
            
        case _LCD_SCR_SET_CALIB_SS_TUR_TAB_2:
              if(sLCD.sScreenNow.SubIndex_u8 == 0)
              {
                    if (sLCD.sScreenNow.Para_u8 > sLCD.sScreenNow.ParaMin_u8 ) {
                        sLCD.sScreenNow.Para_u8--;
                        Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
                    } else {
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_TUR_TAB_1, 0,
                                               __SET_TUR_AD_P2, __SET_TUR_VALUE_P1, __SET_TUR_AD_P2,
                                               NULL, 0xF1);
                    }
              }
              else
              {
                switch (sLCD.sScreenNow.Para_u8)
                {
                    case __SET_TUR_VALUE_P3:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                sButton.Old_value++;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    case __SET_TUR_AD_P3:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                sButton.Old_value = sSensor_Turbidity.Measure_AD;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    case __SET_TUR_VALUE_P4:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                sButton.Old_value++;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    case __SET_TUR_AD_P4:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                sButton.Old_value = sSensor_Turbidity.Measure_AD;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    default:
                      break;
                }
            }
            break;
            
        case _LCD_SCR_SET_CALIB:
            if (sLCD.sScreenNow.Para_u8 > sLCD.sScreenNow.ParaMin_u8 ) {
                sLCD.sScreenNow.Para_u8--;
            }
            Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
            break;
            
        case _LCD_SCR_SET_PRESS_1_2:
            if (sLCD.sScreenNow.Para_u8 > sLCD.sScreenNow.ParaMin_u8 ) {
                sLCD.sScreenNow.Para_u8--;
                Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
            } else {
                Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_PRESS_1, 0,
                                    __SET_PRESSURE_1_5, __SET_PRESSURE_1_1, __SET_PRESSURE_1_5,
                                    NULL, 0xF1);
            }
            break;
        case _LCD_SCR_SET_PRESS_2:
             Display_Process_Up_Pressure_2(&sPressConfig);
            break;
        case _LCD_SCR_SET_PULSE_2:
            //tang scale
            if (sLCD.sScreenNow.Para_u8 == __SET_PULSE_SET2_1) {
                if ( (sLCDObject[__SET_PULSE_SET2_1].Scale_u8 < 4) 
                     || (sLCDObject[__SET_PULSE_SET2_1].Scale_u8 >= 0xFC) ) {
                    sLCDObject[__SET_PULSE_SET2_1].Scale_u8++;
                }
            } else {
                if (sLCD.sScreenNow.Flag_u8 != 0xF0) {
                    if ( (aTEST[sButton.Old_value] == '.')
                        || (aTEST[sButton.Old_value] == '_') ) {
                        aTEST[sButton.Old_value] = '0';
                    } else if (aTEST[sButton.Old_value] < '9') {
                        aTEST[sButton.Old_value] ++;
                    } else {
                        aTEST[sButton.Old_value] = '.';
                    }
                }
            }
            break;
        case _LCD_SCR_PULSE_RS: 
            sButton.Old_value = 1 - sButton.Old_value;
            sLCDObject[sLCD.sScreenNow.Para_u8].pData = sConfirm[sButton.Old_value];  
            break;
        default:
            break;
    }
}


void BUTTON_Down_Process (void)
{
    switch (sLCD.sScreenNow.Index_u8)
    {
        case _LCD_SCREEN_1:
        case _LCD_SCREEN_2:
        case _LCD_SCREEN_3:
        case _LCD_SCREEN_4:
        case _LCD_SCREEN_5:
        case _LCD_SCREEN_6:
        case _LCD_SCREEN_7:
//            sLCD.sScreenNow.Index_u8 ++;
//            sLCD.sScreenNow.SubIndex_u8 = 0;
//            sButton.Old_value = _LCD_SCREEN_7;
            break;
        case _LCD_SCREEN_7b:
            sButton.Old_value = _LCD_SCREEN_7;
            if ( (sLCD.sScreenNow.SubIndex_u8 + 1) >= sWmDigVar.nModbus_u8 ) {
                sLCD.sScreenNow.SubIndex_u8 = 0;
                sLCD.sScreenNow.Index_u8 = _LCD_SCREEN_1;
            } else {
                sLCD.sScreenNow.SubIndex_u8++;
            }
            break;
        case _LCD_SCR_PASS:
        case _LCD_SCR_PASSCALIB:
            aTEST[sButton.Old_value] --;
            if (aTEST[sButton.Old_value] < '0') {
                aTEST[sButton.Old_value] = '9';
            }
            break;
        case _LCD_SCR_SETTING:
//            if (sLCD.sScreenNow.Para_u8 == sLCD.sScreenNow.ParaMax_u8) {
//                Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING_2, 0,
//                                    __SET_MANUFACTOR, __SET_MANUFACTOR, __SET_MANUFACTOR,
//                                    NULL, 0xF1);
//            } else {
//                sLCD.sScreenNow.Para_u8++;
//                Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
//            }
          
            if (sLCD.sScreenNow.Para_u8 < sLCD.sScreenNow.ParaMax_u8) {
                sLCD.sScreenNow.Para_u8++;
            }
            Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
            break;
        case _LCD_SCR_SETTING_2:
        case _LCD_SCR_SET_PRESS_1_2:
        case _LCD_SCR_CAL_CHANN_2:
            if (sLCD.sScreenNow.Para_u8 < sLCD.sScreenNow.ParaMax_u8) {
                sLCD.sScreenNow.Para_u8++;
            }
            Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
            break;
        case _LCD_SCR_SET_PULSE_1:             
            if (sLCD.sScreenNow.Para_u8 < sLCD.sScreenNow.ParaMax_u8) {
                sLCD.sScreenNow.Para_u8++;
            } else {
                sLCD.sScreenNow.Para_u8 = sLCD.sScreenNow.ParaMin_u8;
            }
            Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
            break;
        case _LCD_SCR_CAL_CHANN_1:
            if (sLCD.sScreenNow.Para_u8 == sLCD.sScreenNow.ParaMax_u8) {
                Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_CAL_CHANN_2, 0,
                                    __CAL_PRESS_CH_6, __CAL_PRESS_CH_6, __CAL_PRESS_CH_6,
                                    NULL, 0xF1);
            } else {
                sLCD.sScreenNow.Para_u8++;
                Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
            }
            break;
        case _LCD_SCR_SET_FREQ:
            if (sButton.Old_value > 0) {
                sButton.Old_value --;
            }
            break; 
            
        case _LCD_SCR_SET_CALIB:
            if (sLCD.sScreenNow.Para_u8 < sLCD.sScreenNow.ParaMax_u8) {
                sLCD.sScreenNow.Para_u8++;
            }
            Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
            break;
            
        case _LCD_SCR_SET_CALIB_SS_PH:
              if(sLCD.sScreenNow.SubIndex_u8 == 0)
              {
                    if (sLCD.sScreenNow.Para_u8 < sLCD.sScreenNow.ParaMax_u8) {
                        sLCD.sScreenNow.Para_u8++;
                    }
                    Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
              }
              else
              {
                switch (sLCD.sScreenNow.Para_u8)
                {
                    case __SET_PH_Z_SOLUTION:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                sButton.Old_value = _fSet_pH_Zero_Calib_UpDown(sButton.Old_value, -1);
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    case __SET_PH_ZERO:
                        break;
                        
                    case __SET_PH_S_SOLUTION:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                sButton.Old_value = _fSet_pH_Slope_Calib_UpDown(sButton.Old_value, -1);
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    case __SET_PH_SLOPE:
                        break;
                }
            }
            break;
            
        case _LCD_SCR_SET_CALIB_SS_CLO:
              if(sLCD.sScreenNow.SubIndex_u8 == 0)
              {
                    if (sLCD.sScreenNow.Para_u8 < sLCD.sScreenNow.ParaMax_u8) {
                        sLCD.sScreenNow.Para_u8++;
                    }
                    Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
              }
              else
              {
                switch (sLCD.sScreenNow.Para_u8)
                {   
                    case __SET_CLO_ZERO:
                        break;
                        
                    case __SET_CLO_SLOPE:
                    case __SET_CLO_CLB_PH_1:
                    case __SET_CLO_CLB_PH_2:
                    case __SET_CLO_CLB_TEMP:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                if(sButton.Old_value > 0)
                                    sButton.Old_value--;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
//                    case __SET_CLO_CLB_PH_2:
//                        break;
                }
            }
            break;
            
        case _LCD_SCR_SET_CALIB_SS_EC:
              if(sLCD.sScreenNow.SubIndex_u8 == 0)
              {
                    if (sLCD.sScreenNow.Para_u8 < sLCD.sScreenNow.ParaMax_u8) {
                        sLCD.sScreenNow.Para_u8++;
                    }
                    Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
              }
              else
              {
                switch (sLCD.sScreenNow.Para_u8)
                {
                    case __SET_EC_CONST:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                sButton.Old_value = 1000;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    case __SET_EC_CALIB:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                if(sButton.Old_value > 0)
                                    sButton.Old_value--;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                }
            }
            break;
            
        case _LCD_SCR_SET_CALIB_SS_TUR_TAB_1:
              if(sLCD.sScreenNow.SubIndex_u8 == 0)
              {
                    if (sLCD.sScreenNow.Para_u8 == sLCD.sScreenNow.ParaMax_u8) {
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_TUR_TAB_2, 0,
                                            __SET_TUR_VALUE_P3, __SET_TUR_VALUE_P3, __SET_TUR_AD_P4,
                                            NULL, 0xF1);
                    } else {
                        sLCD.sScreenNow.Para_u8++;
                        Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
                    }
              }
              else
              {
                switch (sLCD.sScreenNow.Para_u8)
                {
                    case __SET_TUR_VALUE_P1:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                if(sButton.Old_value >0)
                                    sButton.Old_value--;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    case __SET_TUR_AD_P1:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                sButton.Old_value = sSensor_Turbidity.Measure_AD;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    case __SET_TUR_VALUE_P2:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                if(sButton.Old_value >0)
                                    sButton.Old_value--;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    case __SET_TUR_AD_P2:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                sButton.Old_value = sSensor_Turbidity.Measure_AD;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    default:
                      break;
                }
            }
            break;
            
        case _LCD_SCR_SET_CALIB_SS_TUR_TAB_2:
              if(sLCD.sScreenNow.SubIndex_u8 == 0)
              {
                    if (sLCD.sScreenNow.Para_u8 < sLCD.sScreenNow.ParaMax_u8) {
                        sLCD.sScreenNow.Para_u8++;
                    }
                    Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
              }
              else
              {
                switch (sLCD.sScreenNow.Para_u8)
                {
                    case __SET_TUR_VALUE_P3:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                if(sButton.Old_value >0)
                                    sButton.Old_value--;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    case __SET_TUR_AD_P3:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                sButton.Old_value = sSensor_Turbidity.Measure_AD;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    case __SET_TUR_VALUE_P4:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                if(sButton.Old_value >0)
                                    sButton.Old_value--;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    case __SET_TUR_AD_P4:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                sButton.Old_value = sSensor_Turbidity.Measure_AD;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    default:
                      break;
                }
            }
            break;
            
        case _LCD_SCR_SET_PRESS_1:
            if (sLCD.sScreenNow.Para_u8 == sLCD.sScreenNow.ParaMax_u8) {
                Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_PRESS_1_2, 0,
                                    __SET_PRESSURE_1_6, __SET_PRESSURE_1_6, __SET_PRESSURE_1_6,
                                    NULL, 0xF1);
            } else {
                sLCD.sScreenNow.Para_u8++;
                Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
            }
            break;
        case _LCD_SCR_SET_PRESS_2:
            Display_Process_Down_Pressure_2(&sPressConfig);
            break;
        case _LCD_SCR_SET_PULSE_2:
            if (sLCD.sScreenNow.Para_u8 == __SET_PULSE_SET2_1) {
                if ( (sLCDObject[__SET_PULSE_SET2_1].Scale_u8 > 0xFC)
                    || (sLCDObject[__SET_PULSE_SET2_1].Scale_u8 < 5) ) {
                    sLCDObject[__SET_PULSE_SET2_1].Scale_u8--;
                }
            } else {     
                if (sLCD.sScreenNow.Flag_u8 != 0xF0) {
                    if ( (aTEST[sButton.Old_value] == '.')
                        || (aTEST[sButton.Old_value] == '_') ) {
                        aTEST[sButton.Old_value] = '9';
                    } else if (aTEST[sButton.Old_value] > '0') {
                        aTEST[sButton.Old_value] --;
                    } else {
                        aTEST[sButton.Old_value] = '.';
                    }
                }
            }
            break;
        case _LCD_SCR_PULSE_RS: 
            sButton.Old_value = 1 - sButton.Old_value;
            sLCDObject[sLCD.sScreenNow.Para_u8].pData = sConfirm[sButton.Old_value];  
            break;
        default:
            break;
    }
}

void BUTTON_ESC_Process (void)
{
    switch (sLCD.sScreenNow.Index_u8)
    {
        case _LCD_SCREEN_CM44:
            sLCD.sScreenNow.Index_u8 = _LCD_SCREEN_1;
            break;
            
        case _LCD_SCREEN_1:
            sLCD.sScreenNow.Index_u8 = _LCD_SCREEN_CM44;
            break;
      
      
        case _LCD_SCR_PASS:
            sLCD.sScreenNow.Index_u8 = _LCD_SCREEN_1;
            break;
            
        case _LCD_SCR_PASSCALIB:
            UTIL_MEM_cpy(&sLCD.sScreenNow, &sLCD.sScreenBack, sizeof(sScreenInformation));
            break;
          
            
        case _LCD_SCR_SETTING:
        case _LCD_SCR_SETTING_2:
            sLCD.sScreenNow.Index_u8 = _LCD_SCREEN_1;
            break;
        case _LCD_SCR_SET_PRESS_1:
        case _LCD_SCR_SET_PRESS_1_2:
            sLCD.sScreenNow.Index_u8 = _LCD_SCR_SETTING; 
            sLCD.sScreenNow.Para_u8 = __SET_PRESSURE;
            sLCD.sScreenNow.ParaMin_u8 = __SET_REQ_1;
            sLCD.sScreenNow.ParaMax_u8 = __SET_PRESS_CALIB;
            break;
        case _LCD_SCR_SET_PULSE_1:
            sLCD.sScreenNow.Index_u8 = _LCD_SCR_SETTING; 
            sLCD.sScreenNow.Para_u8 = __SET_PULSE_SETT;
            sLCD.sScreenNow.ParaMin_u8 = __SET_REQ_1;
            sLCD.sScreenNow.ParaMax_u8 = __SET_PRESS_CALIB;
            break;
        case _LCD_SCR_SET_FREQ:
        case _LCD_SCR_INFORMATION:
        case _LCD_SCR_SET_PRESS_2:
        case _LCD_SCR_SET_RESTORE:
        case _LCD_SCREEN_8:
        case _LCD_SCR_PULSE_RS:
            UTIL_MEM_cpy(&sLCD.sScreenNow, &sLCD.sScreenBack, sizeof(sScreenInformation));
            break;
            
        case _LCD_SCR_SET_CALIB_SS_PH:   
          switch(sLCD.sScreenNow.Para_u8)
          {
            case __SET_PH_Z_SOLUTION:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB, 0,
                                            __SET_CALIB_SS_PH, __SET_CALIB_SS_PH, __SET_CALIB_VALUE,
                                            NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;             
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_PH, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_PH_Z_SOLUTION, __SET_PH_Z_SOLUTION, __SET_PH_SLOPE,
                                           &sSensor_pH.sZero_Calib.Value, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
            case __SET_PH_S_SOLUTION:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB, 0,
                                            __SET_CALIB_SS_PH, __SET_CALIB_SS_PH, __SET_CALIB_VALUE,
                                            NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;             
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_PH, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_PH_S_SOLUTION, __SET_PH_Z_SOLUTION, __SET_PH_SLOPE,
                                           &sSensor_pH.sSlope_Calib.Value, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
              case __SET_PH_ZERO:
              case __SET_PH_SLOPE:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB, 0,
                                            __SET_CALIB_SS_PH, __SET_CALIB_SS_PH, __SET_CALIB_VALUE,
                                            NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                break;
                
              default:
                break;
          }
          break;
            
        case _LCD_SCR_SET_CALIB_SS_CLO:   
          switch(sLCD.sScreenNow.Para_u8)
          {
           case __SET_CLO_ZERO:
                Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB, 0,
                                    __SET_CALIB_SS_CLO, __SET_CALIB_SS_PH, __SET_CALIB_VALUE,
                                    NULL, 0xF1);
                UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                break;
            case __SET_CLO_SLOPE:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB, 0,
                                            __SET_CALIB_SS_CLO, __SET_CALIB_SS_PH, __SET_CALIB_VALUE,
                                            NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;             
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_CLO, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_CLO_SLOPE, __SET_CLO_ZERO, __SET_CLO_CLB_TEMP,
                                           &sConvertChlorine.Clo_Calib_Slope, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
            case __SET_CLO_CLB_PH_1:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB, 0,
                                            __SET_CALIB_SS_CLO, __SET_CALIB_SS_PH, __SET_CALIB_VALUE,
                                            NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;             
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_CLO, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_CLO_CLB_PH_1, __SET_CLO_ZERO, __SET_CLO_CLB_TEMP,
                                           &sConvertChlorine.Clo_CalibPoint_1 , 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
            case __SET_CLO_CLB_PH_2:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB, 0,
                                            __SET_CALIB_SS_CLO, __SET_CALIB_SS_PH, __SET_CALIB_VALUE,
                                            NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;             
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_CLO, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_CLO_CLB_PH_2, __SET_CLO_ZERO, __SET_CLO_CLB_TEMP,
                                           &sConvertChlorine.Clo_CalibPoint_2 , 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
            case __SET_CLO_CLB_TEMP:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB, 0,
                                            __SET_CALIB_SS_CLO, __SET_CALIB_SS_PH, __SET_CALIB_VALUE,
                                            NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;             
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_CLO, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_CLO_CLB_TEMP, __SET_CLO_ZERO, __SET_CLO_CLB_TEMP,
                                           &sConvertChlorine.sConst_Compensation_Temp.Value , 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
              default:
                break;
          }
          break;
            
        case _LCD_SCR_SET_CALIB_SS_EC:   
          switch(sLCD.sScreenNow.Para_u8)
          {
            case __SET_EC_CONST:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB, 0,
                                            __SET_CALIB_SS_EC, __SET_CALIB_SS_PH, __SET_CALIB_VALUE,
                                            NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_EC, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_EC_CONST, __SET_EC_CONST, __SET_EC_CALIB,
                                           &sSensor_EC.sConductivity_Const.Value, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
            case __SET_EC_CALIB:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB, 0,
                                            __SET_CALIB_SS_EC, __SET_CALIB_SS_PH, __SET_CALIB_VALUE,
                                            NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_EC, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_EC_CALIB, __SET_EC_CONST, __SET_EC_CALIB,
                                           &sSensor_EC.sConductivity_Calib.Value, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
              default:
                break;
          }
          break;
          
        case _LCD_SCR_SET_CALIB_SS_TUR_TAB_1:
          switch(sLCD.sScreenNow.Para_u8)
          {
            case __SET_TUR_VALUE_P1:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB, 0,
                                            __SET_CALIB_SS_TUR, __SET_CALIB_SS_PH, __SET_CALIB_VALUE,
                                            NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_TUR_TAB_1, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_TUR_VALUE_P1, __SET_TUR_VALUE_P1, __SET_TUR_AD_P2,
                                           &sSensor_Turbidity.sFirst_Calibration.Value, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
            case __SET_TUR_AD_P1:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB, 0,
                                            __SET_CALIB_SS_TUR, __SET_CALIB_SS_PH, __SET_CALIB_VALUE,
                                            NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_TUR_TAB_1, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_TUR_AD_P1, __SET_TUR_VALUE_P1, __SET_TUR_AD_P2,
                                           &sSensor_Turbidity.First_AD_u32, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
            case __SET_TUR_VALUE_P2:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB, 0,
                                            __SET_CALIB_SS_TUR, __SET_CALIB_SS_PH, __SET_CALIB_VALUE,
                                            NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_TUR_TAB_1, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_TUR_VALUE_P2, __SET_TUR_VALUE_P1, __SET_TUR_AD_P2,
                                           &sSensor_Turbidity.sSecond_Calibration.Value, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
            case __SET_TUR_AD_P2:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB, 0,
                                            __SET_CALIB_SS_TUR, __SET_CALIB_SS_PH, __SET_CALIB_VALUE,
                                            NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_TUR_TAB_1, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_TUR_AD_P2, __SET_TUR_VALUE_P1, __SET_TUR_AD_P2,
                                           &sSensor_Turbidity.Second_AD_u32, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
              default:
                break;
          }
          break;
            
        case _LCD_SCR_SET_CALIB_SS_TUR_TAB_2:
          switch(sLCD.sScreenNow.Para_u8)
          {
            case __SET_TUR_VALUE_P3:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB, 0,
                                            __SET_CALIB_SS_TUR, __SET_CALIB_SS_PH, __SET_CALIB_VALUE,
                                            NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_TUR_TAB_2, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_TUR_VALUE_P3, __SET_TUR_VALUE_P3, __SET_TUR_AD_P4,
                                           &sSensor_Turbidity.sThird_Calibration.Value, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
            case __SET_TUR_AD_P3:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB, 0,
                                            __SET_CALIB_SS_TUR, __SET_CALIB_SS_PH, __SET_CALIB_VALUE,
                                            NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_TUR_TAB_2, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_TUR_AD_P3, __SET_TUR_VALUE_P3, __SET_TUR_AD_P4,
                                           &sSensor_Turbidity.Third_AD_u32, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
            case __SET_TUR_VALUE_P4:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB, 0,
                                            __SET_CALIB_SS_TUR, __SET_CALIB_SS_PH, __SET_CALIB_VALUE,
                                            NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_TUR_TAB_2, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_TUR_VALUE_P4, __SET_TUR_VALUE_P3, __SET_TUR_AD_P4,
                                           &sSensor_Turbidity.sFourth_Calibration.Value, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
            case __SET_TUR_AD_P4:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB, 0,
                                            __SET_CALIB_SS_TUR, __SET_CALIB_SS_PH, __SET_CALIB_VALUE,
                                            NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_TUR_TAB_2, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_TUR_AD_P4, __SET_TUR_VALUE_P3, __SET_TUR_AD_P4,
                                           &sSensor_Turbidity.Fourth_AD_u32, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
              default:
                break;
          }
          break;
            
        case _LCD_SCR_SET_CALIB_VALUE:
            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB, 0,
                                __SET_CALIB_VALUE, __SET_CALIB_SS_PH, __SET_CALIB_VALUE,
                                NULL, 0xF1);
            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
            break;
          
        case _LCD_SCR_SET_CALIB:
            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                               __SET_CALIB_SENSOR, __SET_REQ_1, __DEVICE_INFOR,
                               NULL, 0xF1);
            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
            break;
            
        case _LCD_SCR_SET_CALIB_STATE:
          if(sHandleRs485.State_Wait_Calib != _STATE_CALIB_WAIT)
          {
            UTIL_MEM_cpy(&sLCD.sScreenNow, &sLCD.sScreenBack, sizeof(sScreenInformation));
            sHandleRs485.State_Wait_Calib = _STATE_CALIB_FREE;
          }
          break; 
            
        case _LCD_SCR_SET_PULSE_2:
            if ( (sLCD.sScreenNow.Para_u8 == __SET_PULSE_SET2_2) &&
                  (sLCD.sScreenNow.Flag_u8 != 0xF0) ) {   
                if (aTEST[sButton.Old_value] == '_') {
                    aTEST[sButton.Old_value] = 0;
                }
                
                Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF0);
            } else {
                UTIL_MEM_cpy(&sLCD.sScreenNow, &sLCD.sScreenBack, sizeof(sScreenInformation));
            }  
            break;
        case _LCD_SCR_CAL_CHANN_1:
        case _LCD_SCR_CAL_CHANN_2:
            sLCD.sScreenNow.Index_u8 = _LCD_SCR_SETTING_2; 
            sLCD.sScreenNow.Para_u8 = __SET_MANUFACTOR;
            sLCD.sScreenNow.ParaMin_u8 = __SET_MANUFACTOR;
            sLCD.sScreenNow.ParaMax_u8 = __SET_MANUFACTOR;
            break;
        case _LCD_SCR_CAL_PRESS:
            //lay lai gia tri calib trươc
            sWmVar.aPRESSURE[sButton.Old_value].Calib_i16 = PressOldCalib_i16;
            UTIL_MEM_cpy(&sLCD.sScreenNow, &sLCD.sScreenBack, sizeof(sScreenInformation));
            break;
        default:
            break;
    }
}



