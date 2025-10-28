

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
//static sPressureLinearInter sPressConfig = {0};
//static Struct_Pulse sPulseConfig = {0};
//static int16_t PressOldCalib_i16 = 0;
//static char sConfirm[2][10] = {"No", "Yes"};

float       Old_Value_f = 0;
uint32_t    Old_Value_u32 = 0;
int16_t     Old_Value_i16 = 0;
float       Button_Stamp1_f = 0;
float       Button_Stamp2_f = 0;
      
void BUTTON_Enter_Process (void)
{
//    static uint8_t tempu8 = 0;
//    static uint8_t TempScale = 1;
    switch (sLCD.sScreenNow.Index_u8)
    {
        case _LCD_SCREEN_1:
            sButton.Old_value = 0;
            UTIL_MEM_set(aTEST, 0, sizeof(aTEST));
            aTEST[0] = '0'; aTEST[1] = '0', aTEST[2] = '0', aTEST[3] = '0';
          
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
                                       __SCR_SET_MODBUS, __SCR_SET_MODBUS, __SCR_SET_INFOR,
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
                case __SCR_SET_MODBUS:
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_MODBUS, 0,
                                       __SET_MODBUS_ID, __SET_MODBUS_ID, __SET_MODBUS_BR,
                                       NULL, 0xF1);
                    break;
                    
                case __SCR_SET_CALIB:
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_EC, 0,
                                       __SET_EC_CONST, __SET_EC_CONST, __SET_EC_CALIB,
                                       NULL, 0xF1);
                    break;
                    
                case __SCR_SET_OFFSET:
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET, 0,
                                       __SET_OFFSET_EC, __SET_OFFSET_EC, __SET_OFFSET_TEMP,
                                       NULL, 0xF1);
                    
                case __SCR_SET_INFOR:
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_INFORMATION, 0,
                                       __SCR_INFOR_FW_VERSION_2, __SCR_INFOR_FW_VERSION_2, __SCR_INFOR_FW_VERSION_2,
                                       NULL, 0x00);
                    break;
            }
            break;
            
        case _LCD_SCR_SET_MODBUS:
            switch (sLCD.sScreenNow.Para_u8)
            {
                case __SET_MODBUS_ID:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_MODBUS, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_MODBUS_ID, __SET_MODBUS_ID, __SET_MODBUS_BR,
                                               &sButton.Old_value, 0xF2);
                             sButton.Old_value = sParaDisplay.ID_u8;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_CHECK_SETTING, 0,
                                               __CHECK_STATE_SETTING, __CHECK_STATE_SETTING, __CHECK_STATE_SETTING,
                                               NULL, 0xF0);
                            sParaDisplay.State_Setting = _STATE_SETTING_ENTER;
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                case __SET_MODBUS_BR:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_MODBUS, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_MODBUS_BR, __SET_MODBUS_BR, __SET_MODBUS_BR,
                                               &sButton.Old_value, 0xF2);
                             sButton.Old_value = sParaDisplay.Baudrate_u32;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_CHECK_SETTING, 0,
                                               __CHECK_STATE_SETTING, __CHECK_STATE_SETTING, __CHECK_STATE_SETTING,
                                               NULL, 0xF0);
                            sParaDisplay.State_Setting = _STATE_SETTING_ENTER;
                            break;
                        
                        default:
                            break;
                    }
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
                            sButton.Old_value = sParaDisplay.EC_Calib_Const_i32;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_CHECK_SETTING, 0,
                                               __CHECK_STATE_SETTING, __CHECK_STATE_SETTING, __CHECK_STATE_SETTING,
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
                            sButton.Old_value = sParaDisplay.EC_Calib_Value_i32;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_CHECK_SETTING, 0,
                                               __CHECK_STATE_SETTING, __CHECK_STATE_SETTING, __CHECK_STATE_SETTING,
                                               NULL, 0xF0);
                            Old_Value_f = 1;
                            Button_Stamp1_f = Handle_int32_To_Float_Scale(sParaDisplay.EC_Calib_Value_i32, 0x00);
                            Button_Stamp2_f = Handle_int32_To_Float_Scale(sButton.Old_value, 0x00);
                            
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
            
        case _LCD_SCR_SET_OFFSET:
            switch(sLCD.sScreenNow.Para_u8)
            {
                case __SET_OFFSET_EC:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_OFFSET_EC, __SET_OFFSET_EC, __SET_OFFSET_TEMP,
                                               &sButton.Old_value, 0xF2);
                            sButton.Old_value = sParaDisplay.EC_Offset_i32;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_CHECK_SETTING, 0,
                                               __CHECK_STATE_SETTING, __CHECK_STATE_SETTING, __CHECK_STATE_SETTING,
                                               NULL, 0xF0);
                            sParaDisplay.State_Setting = _STATE_SETTING_ENTER;
                            break;
                            
                        default:
                            break;
                    }
                    break;
                    
                case __SET_OFFSET_TDS:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_OFFSET_TDS, __SET_OFFSET_EC, __SET_OFFSET_TEMP,
                                               &sButton.Old_value, 0xF2);
                            sButton.Old_value = sParaDisplay.TDS_Offset_i32;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_CHECK_SETTING, 0,
                                               __CHECK_STATE_SETTING, __CHECK_STATE_SETTING, __CHECK_STATE_SETTING,
                                               NULL, 0xF0);
                            sParaDisplay.State_Setting = _STATE_SETTING_ENTER;
                            break;
                            
                        default:
                            break;
                    }
                    break;
                    
                case __SET_OFFSET_SALINITY:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_OFFSET_SALINITY, __SET_OFFSET_EC, __SET_OFFSET_TEMP,
                                               &sButton.Old_value, 0xF2);
                            sButton.Old_value = sParaDisplay.Salinity_Offset_i32;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_CHECK_SETTING, 0,
                                               __CHECK_STATE_SETTING, __CHECK_STATE_SETTING, __CHECK_STATE_SETTING,
                                               NULL, 0xF0);
                            sParaDisplay.State_Setting = _STATE_SETTING_ENTER;
                            break;
                            
                        default:
                            break;
                    }
                    break;
                    
                case __SET_OFFSET_TEMP:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_OFFSET_TEMP, __SET_OFFSET_EC, __SET_OFFSET_TEMP,
                                               &sButton.Old_value,  0xF2);
                            sButton.Old_value = sParaDisplay.temp_Offset_i32;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_CHECK_SETTING, 0,
                                               __CHECK_STATE_SETTING, __CHECK_STATE_SETTING, __CHECK_STATE_SETTING,
                                               NULL, 0xF0);
                            sParaDisplay.State_Setting = _STATE_SETTING_ENTER;
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
          switch(sLCD.sScreenBack.Index_u8)
          {
              case _LCD_SCR_SET_MODBUS:
                sParaDisplay.State_Setting = _STATE_SETTING_DONE;
                switch (sLCD.sScreenBack.Para_u8)
                {
                    case __SET_MODBUS_ID:
                      Save_InforSlaveModbusRTU((uint8_t)sButton.Old_value, sSlave_ModbusRTU.Baudrate);
                      break;
                      
                    case __SET_MODBUS_BR:
                      for(uint8_t i = 0; i < 11; i++)
                      {
                        if(aBaudrate_value[i] == sButton.Old_value)
                        {
                            sSlave_ModbusRTU.Baudrate = i;
                            break;
                        }
                      }
                      Save_InforSlaveModbusRTU(sSlave_ModbusRTU.ID, sSlave_ModbusRTU.Baudrate);
                      break;
                      
                    default:
                      break;
                }
                break;
                
              case _LCD_SCR_SET_OFFSET:
                sParaDisplay.State_Setting = _STATE_SETTING_DONE;
                switch (sLCD.sScreenBack.Para_u8)
                {
                    case __SET_OFFSET_EC:
                      Save_ParamCalib((float)(sButton.Old_value/100), sSensor_EC.TDS_Offset_f, sSensor_EC.Salinity_Offset_f, sSensor_EC.temp_Offset_f);
                      break;
                      
                    case __SET_OFFSET_TDS:
                      Save_ParamCalib(sSensor_EC.EC_Offset_f, (float)(sButton.Old_value/100), sSensor_EC.Salinity_Offset_f, sSensor_EC.temp_Offset_f);
                      break;
                      
                    case __SET_OFFSET_SALINITY:
                      Save_ParamCalib(sSensor_EC.EC_Offset_f, sSensor_EC.TDS_Offset_f, (float)(sButton.Old_value/100), sSensor_EC.temp_Offset_f);
                      break;
                      
                    case __SET_OFFSET_TEMP:
                      Save_ParamCalib(sSensor_EC.EC_Offset_f, sSensor_EC.TDS_Offset_f, sSensor_EC.Salinity_Offset_f, (float)(sButton.Old_value/100));
                      break;
                      
                    default:
                      break;
                }
                break;
                
              case _LCD_SCR_SET_CALIB_SS_EC:
                RS485_Enter_Calib();
                break;

              default:
                break;
          }
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
          break;
          
        case _LCD_SCR_PASS:
            aTEST[sButton.Old_value] ++;
            if (aTEST[sButton.Old_value] > '9')
                aTEST[sButton.Old_value] = '0';
            break;
            
        case _LCD_SCR_SETTING:
            if (sLCD.sScreenNow.Para_u8 > sLCD.sScreenNow.ParaMin_u8 ) {
                sLCD.sScreenNow.Para_u8--;
            }
            Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
            break;
            
        case _LCD_SCR_SET_MODBUS:
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
                    case __SET_MODBUS_ID:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                if(sButton.Old_value < 256)
                                    sButton.Old_value++;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    case __SET_MODBUS_BR:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                for(uint8_t i = 0; i < 11; i++)
                                {
                                    if(aBaudrate_value[i] == sButton.Old_value)
                                    {
                                        if(i<10)
                                        {
                                            sButton.Old_value = aBaudrate_value[i+1];
                                            break;
                                        }
                                    }
                                }
                                break;
                            
                            default:
                                break;
                        }
                        break;
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
            
        case _LCD_SCR_SET_OFFSET:
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
                case __SET_OFFSET_EC:
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
                    
                case __SET_OFFSET_TDS:
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
                    
                case __SET_OFFSET_SALINITY:
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
                    
                case __SET_OFFSET_TEMP:
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
            
        default:
          break;
    }
}


void BUTTON_Down_Process (void)
{
    switch (sLCD.sScreenNow.Index_u8)
    {
        case _LCD_SCREEN_1:
          break;
          
        case _LCD_SCR_PASS:
            aTEST[sButton.Old_value] --;
            if (aTEST[sButton.Old_value] < '0') {
                aTEST[sButton.Old_value] = '9';
            }
            break;
            
        case _LCD_SCR_SETTING:
            if (sLCD.sScreenNow.Para_u8 < sLCD.sScreenNow.ParaMax_u8) {
                sLCD.sScreenNow.Para_u8++;
            }
            Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
            break;
            
        case _LCD_SCR_SET_MODBUS:
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
                    case __SET_MODBUS_ID:
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
                        
                    case __SET_MODBUS_BR:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                for(uint8_t i = 10; i>0; i--)
                                {
                                    if(aBaudrate_value[i] == sButton.Old_value)
                                    {
                                        if(i>0)
                                        {
                                            sButton.Old_value = aBaudrate_value[i-1];
                                            break;
                                        }
                                    }
                                }
                                break;
                            
                            default:
                                break;
                        }
                        break;
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
            
        case _LCD_SCR_SET_OFFSET:
          if(sLCD.sScreenNow.SubIndex_u8 == 0)
          {
                if (sLCD.sScreenNow.Para_u8 < sLCD.sScreenNow.ParaMax_u8 ) {
                    sLCD.sScreenNow.Para_u8++;
                }
                Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
          }
          else
          {
            switch (sLCD.sScreenNow.Para_u8)
            {
                case __SET_OFFSET_EC:
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
                    
                case __SET_OFFSET_TDS:
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
                    
                case __SET_OFFSET_SALINITY:
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
                    
                case __SET_OFFSET_TEMP:
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
            
        default:
          break;
    }
}

void BUTTON_ESC_Process (void)
{
    switch (sLCD.sScreenNow.Index_u8)
    {
        case _LCD_SCREEN_1:
          break;
          
        case _LCD_SCR_PASS:
            sLCD.sScreenNow.Index_u8 = _LCD_SCREEN_1;
            break;
            
        case _LCD_SCR_SETTING:
            sLCD.sScreenNow.Index_u8 = _LCD_SCREEN_1;
            break;
          
        case _LCD_SCR_SET_MODBUS:   
          switch(sLCD.sScreenNow.Para_u8)
          {
                case __SET_MODBUS_ID:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                                                __SCR_SET_MODBUS, __SCR_SET_MODBUS, __SCR_SET_INFOR,
                                                NULL, 0xF1);
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            break;             
                          
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_MODBUS, (sLCD.sScreenNow.SubIndex_u8-1),
                                               __SET_MODBUS_ID, __SET_MODBUS_ID, __SET_MODBUS_BR,
                                               &sParaDisplay.ID_u8, 0xF1);
                            break;
                            
                        default:
                            break;
                    }
                    break;
                
                case __SET_MODBUS_BR:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                                                __SCR_SET_MODBUS, __SCR_SET_MODBUS, __SCR_SET_INFOR,
                                                NULL, 0xF1);
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            break;         
                          
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_MODBUS, (sLCD.sScreenNow.SubIndex_u8-1),
                                               __SET_MODBUS_BR, __SET_MODBUS_ID, __SET_MODBUS_BR,
                                               &sParaDisplay.Baudrate_u32, 0xF1);
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
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                                            __SET_EC_CONST, __SET_EC_CONST, __SET_EC_CALIB,
                                            NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_EC, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_EC_CONST, __SET_EC_CONST, __SET_EC_CALIB,
                                           &sParaDisplay.EC_Calib_Const_i32, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
            case __SET_EC_CALIB:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                                            __SET_EC_CALIB, __SET_EC_CONST, __SET_EC_CALIB,
                                            NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_SS_EC, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SET_EC_CALIB, __SET_EC_CONST, __SET_EC_CALIB,
                                           &sParaDisplay.EC_Calib_Value_i32, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
              default:
                break;
          }
          break;
          
        case _LCD_SCR_SET_OFFSET:
          switch(sLCD.sScreenNow.Para_u8)
          {
                case __SET_OFFSET_EC:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                                                __SCR_SET_MODBUS, __SCR_SET_MODBUS, __SCR_SET_INFOR,
                                                NULL, 0xF1);
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            break;             
                          
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET, (sLCD.sScreenNow.SubIndex_u8-1),
                                               __SET_OFFSET_EC, __SET_OFFSET_EC, __SET_OFFSET_TEMP,
                                               &sParaDisplay.EC_Offset_i32, 0xF1);
                            break;
                            
                        default:
                            break;
                    }
                    break;
                    
                case __SET_OFFSET_TDS:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                                                __SCR_SET_MODBUS, __SCR_SET_MODBUS, __SCR_SET_INFOR,
                                                NULL, 0xF1);
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            break;             
                          
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET, (sLCD.sScreenNow.SubIndex_u8-1),
                                               __SET_OFFSET_TDS, __SET_OFFSET_EC, __SET_OFFSET_TEMP,
                                               &sParaDisplay.TDS_Offset_i32, 0xF1);
                            break;
                            
                        default:
                            break;
                    }
                    break;
                    
                case __SET_OFFSET_SALINITY:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                                                __SCR_SET_MODBUS, __SCR_SET_MODBUS, __SCR_SET_INFOR,
                                                NULL, 0xF1);
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            break;             
                          
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET, (sLCD.sScreenNow.SubIndex_u8-1),
                                               __SET_OFFSET_SALINITY, __SET_OFFSET_EC, __SET_OFFSET_TEMP,
                                               &sParaDisplay.Salinity_Offset_i32, 0xF1);
                            break;
                            
                        default:
                            break;
                    }
                    break;
                
                case __SET_OFFSET_TEMP:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                                                __SCR_SET_MODBUS, __SCR_SET_MODBUS, __SCR_SET_INFOR,
                                                NULL, 0xF1);
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            break;         
                          
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_OFFSET, (sLCD.sScreenNow.SubIndex_u8-1),
                                               __SET_OFFSET_TEMP, __SET_OFFSET_EC, __SET_OFFSET_TEMP,
                                               &sParaDisplay.temp_Offset_i32, 0xF1);
                            break;
                            
                        default:
                            break;
                    }
                    break;
                
              default:
                break;
          }
          break;
          
        case _LCD_SCR_SET_INFORMATION:
            UTIL_MEM_cpy(&sLCD.sScreenNow, &sLCD.sScreenBack, sizeof(sScreenInformation));
            break;
          
        case _LCD_SCR_CHECK_SETTING:
          if(sParaDisplay.State_Setting != _STATE_SETTING_WAIT)
          {
            UTIL_MEM_cpy(&sLCD.sScreenNow, &sLCD.sScreenBack, sizeof(sScreenInformation));
            sParaDisplay.State_Setting = _STATE_SETTING_FREE;
          }
          break;
          
        default:
          break;
    }
}



