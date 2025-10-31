

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
                    
                case __SET_OFFSET_SENSOR:
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET_SENSOR, 0,
                                       __SET_OFFSET_SS_PH, __SET_OFFSET_SS_PH, __SET_OFFSET_SS_TEMP,
                                       NULL, 0xF1);
                    break;
                    
                case __SET_OPTION_SENSOR:
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OPTION_SENSOR, 0,
                                       __SET_OPTION_SS_PH, __SET_OPTION_SS_PH, __SET_OPTION_SS_TURB,
                                       NULL, 0xF1);
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
            
        case _LCD_SCR_SET_OFFSET_SENSOR:
            switch (sLCD.sScreenNow.Para_u8)
            {  
                case __SET_OFFSET_SS_PH:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET_SENSOR, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_OFFSET_SS_PH, __SET_OFFSET_SS_PH, __SET_OFFSET_SS_TEMP,
                                               &sButton.Old_value, 0xF2);
                            sButton.Old_value = sParaDisplay.pH_Offset_i32;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET_SENSOR, 0,
                                               __SET_OFFSET_SS_PH, __SET_OFFSET_SS_PH, __SET_OFFSET_SS_TEMP,
                                               &sParaDisplay.pH_Offset_i32, 0xF1);
                            Save_OffsetMeasure(_OFFSET_PH, ((float)(sButton.Old_value)/100));
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                case __SET_OFFSET_SS_CLO:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET_SENSOR, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_OFFSET_SS_CLO, __SET_OFFSET_SS_PH, __SET_OFFSET_SS_TEMP,
                                               &sButton.Old_value, 0xF2);
                            sButton.Old_value = sParaDisplay.Clo_Offset_i32;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET_SENSOR, 0,
                                               __SET_OFFSET_SS_CLO, __SET_OFFSET_SS_PH, __SET_OFFSET_SS_TEMP,
                                               &sParaDisplay.Clo_Offset_i32, 0xF1);
                            Save_OffsetMeasure(_OFFSET_CLO, ((float)(sButton.Old_value)/100));
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                case __SET_OFFSET_SS_TURB:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET_SENSOR, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_OFFSET_SS_TURB, __SET_OFFSET_SS_PH, __SET_OFFSET_SS_TEMP,
                                               &sButton.Old_value, 0xF2);
                            sButton.Old_value = sParaDisplay.Turb_Offset_i32;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET_SENSOR, 0,
                                               __SET_OFFSET_SS_TURB, __SET_OFFSET_SS_PH, __SET_OFFSET_SS_TEMP,
                                               &sParaDisplay.Turb_Offset_i32, 0xF1);
                            Save_OffsetMeasure(_OFFSET_TURB, ((float)(sButton.Old_value)/100));
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                case __SET_OFFSET_SS_EC:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET_SENSOR, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_OFFSET_SS_EC, __SET_OFFSET_SS_PH, __SET_OFFSET_SS_TEMP,
                                               &sButton.Old_value, 0xF2);
                            sButton.Old_value = sParaDisplay.EC_Offset_i32;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET_SENSOR, 0,
                                               __SET_OFFSET_SS_EC, __SET_OFFSET_SS_PH, __SET_OFFSET_SS_TEMP,
                                               &sParaDisplay.EC_Offset_i32, 0xF1);
                            Save_OffsetMeasure(_OFFSET_EC, ((float)(sButton.Old_value)/100));
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                case __SET_OFFSET_SS_SALINITY:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET_SENSOR, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_OFFSET_SS_SALINITY, __SET_OFFSET_SS_PH, __SET_OFFSET_SS_TEMP,
                                               &sButton.Old_value, 0xF2);
                            sButton.Old_value = sParaDisplay.Salinity_Offset_i32;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET_SENSOR, 0,
                                               __SET_OFFSET_SS_SALINITY, __SET_OFFSET_SS_PH, __SET_OFFSET_SS_TEMP,
                                               &sParaDisplay.Salinity_Offset_i32, 0xF1);
                            Save_OffsetMeasure(_OFFSET_SAL, ((float)(sButton.Old_value)/100));
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                case __SET_OFFSET_SS_TEMP:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET_SENSOR, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_OFFSET_SS_TEMP, __SET_OFFSET_SS_PH, __SET_OFFSET_SS_TEMP,
                                               &sButton.Old_value, 0xF2);
                            sButton.Old_value = sParaDisplay.Temp_Offset_i32;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET_SENSOR, 0,
                                               __SET_OFFSET_SS_TEMP, __SET_OFFSET_SS_PH, __SET_OFFSET_SS_TEMP,
                                               &sParaDisplay.Temp_Offset_i32, 0xF1);
                            Save_OffsetMeasure(_OFFSET_TEMP, ((float)(sButton.Old_value)/100));
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                    default:
                      break;
            }
            break;

        case _LCD_SCR_SET_OPTION_SENSOR:
            switch (sLCD.sScreenNow.Para_u8)
            {  
                case __SET_OPTION_SS_PH:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OPTION_SENSOR, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_OPTION_SS_PH, __SET_OPTION_SS_PH, __SET_OPTION_SS_TURB,
                                               &sButton.Old_value, 0xF2);
                            sButton.Old_value = sUserSensor.User_pH;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OPTION_SENSOR, 0,
                                               __SET_OPTION_SS_PH, __SET_OPTION_SS_PH, __SET_OPTION_SS_TURB,
                                               &sUserSensor.User_pH, 0xF1);
                            Save_UserSensor(_ACTIVE_PH, sButton.Old_value);
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                case __SET_OPTION_SS_CLO:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OPTION_SENSOR, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_OPTION_SS_CLO, __SET_OPTION_SS_PH, __SET_OPTION_SS_TURB,
                                               &sButton.Old_value, 0xF2);
                            sButton.Old_value = sUserSensor.User_Clo;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OPTION_SENSOR, 0,
                                               __SET_OPTION_SS_CLO, __SET_OPTION_SS_PH, __SET_OPTION_SS_TURB,
                                               &sUserSensor.User_Clo, 0xF1);
                            Save_UserSensor(_ACTIVE_CLO, sButton.Old_value);
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                case __SET_OPTION_SS_EC:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OPTION_SENSOR, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_OPTION_SS_EC, __SET_OPTION_SS_PH, __SET_OPTION_SS_TURB,
                                               &sButton.Old_value, 0xF2);
                            sButton.Old_value = sUserSensor.User_EC;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OPTION_SENSOR, 0,
                                               __SET_OPTION_SS_EC, __SET_OPTION_SS_PH, __SET_OPTION_SS_TURB,
                                               &sUserSensor.User_EC, 0xF1);
                            Save_UserSensor(_ACTIVE_EC, sButton.Old_value);
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                case __SET_OPTION_SS_TURB:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OPTION_SENSOR, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_OPTION_SS_TURB, __SET_OPTION_SS_PH, __SET_OPTION_SS_TURB,
                                               &sButton.Old_value, 0xF2);
                            sButton.Old_value = sUserSensor.User_Turb;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OPTION_SENSOR, 0,
                                               __SET_OPTION_SS_TURB, __SET_OPTION_SS_PH, __SET_OPTION_SS_TURB,
                                               &sUserSensor.User_Turb, 0xF1);
                            Save_UserSensor(_ACTIVE_TURB, sButton.Old_value);
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                    default:
                      break;
            }
            break;
        
        case _LCD_SCR_CHECK_SETTING:
//          switch(sLCD.sScreenBack.Index_u8)
//          {
//              case _LCD_SCR_SET_CALIB_SS_CLO:
//                DCU_Enter_Calib();
//                break;
//                
//              default:
//                RS485_Enter_Calib();
//                break;
//          }
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
            
        case _LCD_SCR_SET_OFFSET_SENSOR:
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
                    case __SET_OFFSET_SS_PH:
                    case __SET_OFFSET_SS_CLO:
                    case __SET_OFFSET_SS_TURB:
                    case __SET_OFFSET_SS_EC:
                    case __SET_OFFSET_SS_SALINITY:
                    case __SET_OFFSET_SS_TEMP:
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
            
         case _LCD_SCR_SET_OPTION_SENSOR:
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
                    case __SET_OPTION_SS_PH:
                    case __SET_OPTION_SS_CLO:
                    case __SET_OPTION_SS_EC:
                    case __SET_OPTION_SS_TURB:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                if(sButton.Old_value == 0)
                                  sButton.Old_value = 1;
                                else
                                  sButton.Old_value = 0;
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

        case _LCD_SCR_SET_OFFSET_SENSOR:
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
                    case __SET_OFFSET_SS_PH:
                    case __SET_OFFSET_SS_CLO:
                    case __SET_OFFSET_SS_TURB:
                    case __SET_OFFSET_SS_EC:
                    case __SET_OFFSET_SS_SALINITY:
                    case __SET_OFFSET_SS_TEMP:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                sButton.Old_value--;
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
            
        case _LCD_SCR_SET_OPTION_SENSOR:
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
                    case __SET_OPTION_SS_PH:
                    case __SET_OPTION_SS_CLO:
                    case __SET_OPTION_SS_EC:
                    case __SET_OPTION_SS_TURB:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                if(sButton.Old_value == 0)
                                  sButton.Old_value = 1;
                                else
                                  sButton.Old_value = 0;
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

        case _LCD_SCR_SET_OFFSET_SENSOR:   
          switch(sLCD.sScreenNow.Para_u8)
          {
            case __SET_OFFSET_SS_PH:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                                           __SET_OFFSET_SENSOR, __SET_REQ_1, __DEVICE_INFOR,
                                           NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;        
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET_SENSOR, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_OFFSET_SS_PH, __SET_OFFSET_SS_PH, __SET_OFFSET_SS_TEMP,
                                           &sParaDisplay.pH_Offset_i32, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
            case __SET_OFFSET_SS_CLO:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                                           __SET_OFFSET_SENSOR, __SET_REQ_1, __DEVICE_INFOR,
                                           NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;        
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET_SENSOR, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_OFFSET_SS_CLO, __SET_OFFSET_SS_PH, __SET_OFFSET_SS_TEMP,
                                           &sParaDisplay.Clo_Offset_i32, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
            case __SET_OFFSET_SS_TURB:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                                           __SET_OFFSET_SENSOR, __SET_REQ_1, __DEVICE_INFOR,
                                           NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;        
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET_SENSOR, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_OFFSET_SS_TURB, __SET_OFFSET_SS_PH, __SET_OFFSET_SS_TEMP,
                                           &sParaDisplay.Turb_Offset_i32, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
            case __SET_OFFSET_SS_EC:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                                           __SET_OFFSET_SENSOR, __SET_REQ_1, __DEVICE_INFOR,
                                           NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;        
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET_SENSOR, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_OFFSET_SS_EC, __SET_OFFSET_SS_PH, __SET_OFFSET_SS_TEMP,
                                           &sParaDisplay.EC_Offset_i32, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
            case __SET_OFFSET_SS_SALINITY:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                                           __SET_OFFSET_SENSOR, __SET_REQ_1, __DEVICE_INFOR,
                                           NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;        
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET_SENSOR, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_OFFSET_SS_SALINITY, __SET_OFFSET_SS_PH, __SET_OFFSET_SS_TEMP,
                                           &sParaDisplay.Salinity_Offset_i32, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
            case __SET_OFFSET_SS_TEMP:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                                           __SET_OFFSET_SENSOR, __SET_REQ_1, __DEVICE_INFOR,
                                           NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;        
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET_SENSOR, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_OFFSET_SS_TEMP, __SET_OFFSET_SS_PH, __SET_OFFSET_SS_TEMP,
                                           &sParaDisplay.Temp_Offset_i32, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
              default:
                break;
          }
          break;
          
        case _LCD_SCR_SET_OPTION_SENSOR:   
          switch(sLCD.sScreenNow.Para_u8)
          {
            case __SET_OPTION_SS_PH:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                                           __SET_OPTION_SENSOR, __SET_REQ_1, __DEVICE_INFOR,
                                           NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;        
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OPTION_SENSOR, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_OPTION_SS_PH, __SET_OPTION_SS_PH, __SET_OPTION_SS_TURB,
                                           &sUserSensor.User_pH, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
            case __SET_OPTION_SS_CLO:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                                           __SET_OPTION_SENSOR, __SET_REQ_1, __DEVICE_INFOR,
                                           NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;        
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OPTION_SENSOR, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_OPTION_SS_CLO, __SET_OPTION_SS_PH, __SET_OPTION_SS_TURB,
                                           &sUserSensor.User_Clo, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
            case __SET_OPTION_SS_EC:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                                           __SET_OPTION_SENSOR, __SET_REQ_1, __DEVICE_INFOR,
                                           NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;        
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OPTION_SENSOR, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_OPTION_SS_EC, __SET_OPTION_SS_PH, __SET_OPTION_SS_TURB,
                                           &sUserSensor.User_EC, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
            case __SET_OPTION_SS_TURB:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                                           __SET_OPTION_SENSOR, __SET_REQ_1, __DEVICE_INFOR,
                                           NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;        
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OPTION_SENSOR, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_OPTION_SS_TURB, __SET_OPTION_SS_PH, __SET_OPTION_SS_TURB,
                                           &sUserSensor.User_Turb, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
              default:
                break;
          }
          break;
            
        case _LCD_SCR_CHECK_SETTING:
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



