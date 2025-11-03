
#include "user_app_sensor.h"
#include "user_define.h"
#include "user_convert_variable.h"
#include "math.h"

/*============== Function static ===============*/
static uint8_t fevent_sensor_entry(uint8_t event);

static uint8_t fevent_sensor_handle_state(uint8_t event);
/*=================== struct ==================*/
sEvent_struct               sEventAppSensor[] = 
{
  {_EVENT_SENSOR_ENTRY,              1, 5, 60000,                fevent_sensor_entry},
  
  {_EVENT_SENSOR_HANDLE_STATE,       0, 5, 500,                  fevent_sensor_handle_state},
};
uint8_t DurationTimeWarningSensor = 0;
Struct_Offset_Measure       sOffsetMeasure = {0};
Struct_UserSensor           sUserSensor = {0};

uint8_t _Cb_Handle_Detect_Power(uint8_t State);
uint8_t _Cb_Handle_SS_pH(uint8_t State);
uint8_t _Cb_Handle_SS_Clo(uint8_t State);
uint8_t _Cb_Handle_SS_EC(uint8_t State);
uint8_t _Cb_Handle_SS_Turb(uint8_t State);
Struct_SensorWarning        sSensorWarning[] = 
{
    //e_Name        State_Active    State_Connect_Now       State_Connect_Befor     Gettick_Handle      _Cb_Handler_SS_Connect
  {_DETECT_POWER,   _ACTIVE_SENSOR, _SENSOR_CONNECT,      _SENSOR_CONNECT,                  0,          _Cb_Handle_Detect_Power},
  {_SENSOR_PH,      NULL,           _SENSOR_DISCONNECT,   _SENSOR_DISCONNECT,               0,          _Cb_Handle_SS_pH},
  {_SENSOR_CLO,     NULL,           _SENSOR_DISCONNECT,   _SENSOR_DISCONNECT,               0,          _Cb_Handle_SS_Clo},
  {_SENSOR_EC,      NULL,           _SENSOR_DISCONNECT,   _SENSOR_DISCONNECT,               0,          _Cb_Handle_SS_EC},
  {_SENSOR_TURB,    NULL,           _SENSOR_DISCONNECT,   _SENSOR_DISCONNECT,               0,          _Cb_Handle_SS_Turb},
};
/*================= Function Handle ==============*/

static uint8_t fevent_sensor_entry(uint8_t event)
{
    fevent_enable(sEventAppSensor, _EVENT_SENSOR_HANDLE_STATE);
    return 1;
}

static uint8_t fevent_sensor_handle_state(uint8_t event)
{   
    static uint8_t i = _DETECT_POWER;
    
    if(sVout.mVol_u32 < 5000)
        sSensorWarning[_DETECT_POWER].State_Connect_Now = _SENSOR_DISCONNECT;
    else
        sSensorWarning[_DETECT_POWER].State_Connect_Now = _SENSOR_CONNECT;
    
    sSensorWarning[_SENSOR_PH].State_Active = sUserSensor.User_pH;
    sSensorWarning[_SENSOR_CLO].State_Active = sUserSensor.User_Clo;
    sSensorWarning[_SENSOR_EC].State_Active = sUserSensor.User_EC;
    sSensorWarning[_SENSOR_TURB].State_Active = sUserSensor.User_Turb;
    
    sSensorWarning[_SENSOR_PH].State_Connect_Now = sRs485_pH.State_Connect_u8;
    sSensorWarning[_SENSOR_CLO].State_Connect_Now = sRs485_Clo.State_Connect_u8;
    sSensorWarning[_SENSOR_EC].State_Connect_Now = sRs485_EC.State_Connect_u8;
    sSensorWarning[_SENSOR_TURB].State_Connect_Now = sRs485_Turb.State_Connect_u8;
    
    if(sSensorWarning[i].State_Active == _ACTIVE_SENSOR)
    {
        if(sSensorWarning[i].State_Connect_Now != sSensorWarning[i].State_Connect_Befor)
        {
            sSensorWarning[i].Gettick_Handle = HAL_GetTick();
            if(sSensorWarning[i].State_Connect_Now == _SENSOR_DISCONNECT)
                sSensorWarning[i]._Cb_Handler_SS_Connect(sSensorWarning[i].State_Connect_Now);
            
            sSensorWarning[i].State_Connect_Befor = sSensorWarning[i].State_Connect_Now;
        }
        
        if(sSensorWarning[i].State_Connect_Now == _SENSOR_DISCONNECT)
        {
            if(HAL_GetTick() - sSensorWarning[i].Gettick_Handle >= DurationTimeWarningSensor*60000)
            {
                sSensorWarning[i].Gettick_Handle = HAL_GetTick();
                sSensorWarning[i]._Cb_Handler_SS_Connect(sSensorWarning[i].State_Connect_Now);
            }
        }
    }
    else
    {
        sSensorWarning[i].Gettick_Handle = HAL_GetTick();
    }
    
    if(i+1 < _END_SENSOR)
        i++;
    else
        i = _DETECT_POWER;
    
    if(sSensorWarning[_DETECT_POWER].State_Connect_Now == _SENSOR_DISCONNECT)
    {
        i = _DETECT_POWER;
        for(uint8_t j = _DETECT_POWER + 1; j < _END_SENSOR; j++)
        {
            sSensorWarning[j].Gettick_Handle = HAL_GetTick();
            sSensorWarning[j].State_Connect_Befor = _SENSOR_DISCONNECT;
        }
    }
    else
    {
        sSensorWarning[_DETECT_POWER].Gettick_Handle = 0;
    }
    
    fevent_enable(sEventAppSensor, event);
    return 0;
}
/*====================Function Handle====================*/
uint8_t _Cb_Handle_Detect_Power(uint8_t State)
{
    uint8_t aData[2] = {0};
    aData[0] = 0x00;
    aData[1] = 0x00;
    Log_EventWarnig(OBIS_WARNING_DETECT_POWER, 0x01, aData);
    return 1;
}

uint8_t _Cb_Handle_SS_pH(uint8_t State)
{
    uint8_t aData[2] = {0};
    aData[0] = 0x00;
    aData[1] = 0x00;
    Log_EventWarnig(OBIS_WARNING_SENSOR_CONNECT, 0x01, aData);
    return 1;
}

uint8_t _Cb_Handle_SS_Clo(uint8_t State)
{
    uint8_t aData[2] = {0};
    aData[0] = 0x02;
    aData[1] = 0x00;
    Log_EventWarnig(OBIS_WARNING_SENSOR_CONNECT, 0x01, aData);
    return 1;
}

uint8_t _Cb_Handle_SS_EC(uint8_t State)
{
    uint8_t aData[2] = {0};
    aData[0] = 0x04;
    aData[1] = 0x00;
    Log_EventWarnig(OBIS_WARNING_SENSOR_CONNECT, 0x01, aData);
    return 1;
}

uint8_t _Cb_Handle_SS_Turb(uint8_t State)
{
    uint8_t aData[2] = {0};
    aData[0] = 0x06;
    aData[1] = 0x00;
    Log_EventWarnig(OBIS_WARNING_SENSOR_CONNECT, 0x01, aData);
    return 1;
}

/*=======================Handle Sensor======================*/
void Log_EventWarnig(uint8_t Obis, uint8_t LengthData, uint8_t *aDataWaring)
{
  Get_RTC();
  
  if(sRTC.year > 20)
  {
    uint8_t     aData[10]={0};
    uint16_t    length = 0;
    uint16_t	i = 0;
    uint8_t     TempCrc = 0;
    
    SV_Protocol_Packet_Data(aData, &length, OBIS_TIME_DEVICE, &sRTC, 6, 0xAA);
    aData[length++] = Obis;
    aData[length++] = LengthData;
    
    for(uint8_t i = 0; i < LengthData; i++)
        aData[length++] = *(aDataWaring+i);
    
    // caculator crc
    length++;
	for (i = 0; i < (length - 1); i++)
		TempCrc ^= aData[i];

    aData[length-1] = TempCrc;
  
#ifdef USING_APP_MEM
    AppMem_Write_Data(sAppMem.RecMemType_u8, _MEM_DATA_EVENT_A, 0, &aData[0], length, sRecEvent.SizeRecord_u16);
#endif
  }
}
/*==================Handle Define AT command=================*/
#ifdef USING_AT_CONFIG
void AT_CMD_Get_Time_Warning_Sensor(sData *str, uint16_t Pos)
{
    uint8_t aTemp[50] = "TimeWarningSensor: ";   //13 ki tu dau tien
    sData StrResp = {&aTemp[0], 19}; 

    Convert_Uint64_To_StringDec (&StrResp, (uint64_t) (DurationTimeWarningSensor), 0);
    Insert_String_To_String(StrResp.Data_a8, &StrResp.Length_u16, (uint8_t*)" min",0 , 4);

	Modem_Respond(PortConfig, StrResp.Data_a8, StrResp.Length_u16, 0);
}

void AT_CMD_Set_Time_Warning_Sensor (sData *str_Receiv, uint16_t Pos)
{
    uint32_t TempU32 = 0;
    if( str_Receiv->Data_a8[0] >= '0' && str_Receiv->Data_a8[0] <= '9')
    {
        uint8_t length = 0;
        for(uint8_t i = 0; i < str_Receiv->Length_u16; i++)
        {
            if( str_Receiv->Data_a8[i] < '0' || str_Receiv->Data_a8[i]>'9') break;
            else length++;
        }
        TempU32 = Convert_String_To_Dec(str_Receiv->Data_a8 , length);
        if(TempU32 <=255 && TempU32 >=1)
        {
            Save_TimeWarningSensor(TempU32);
            Modem_Respond(PortConfig, (uint8_t*)"OK", 2, 0);
        }
        else
        {
            Modem_Respond(PortConfig, (uint8_t*)"ERROR", 5, 0);
        }
    }
    else
    {
        Modem_Respond(PortConfig, (uint8_t*)"ERROR", 5, 0);
    }
}

void AT_CMD_Get_Offset_Clo (sData *str, uint16_t Pos)
{
    uint8_t aTemp[50] = "Offset Clo: ";   //13 ki tu dau tien
    sData StrResp = {&aTemp[0], 12}; 

    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sOffsetMeasure.Clo_f*100), 0xFE);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" mg/L",0 , 5);

	Modem_Respond(PortConfig, StrResp.Data_a8, StrResp.Length_u16, 0);
}

void AT_CMD_Set_Offset_Clo (sData *str_Receiv, uint16_t Pos)
{
    int32_t  Temp_I32 = 0;
    float    Temp_f = 0;
    
    uint8_t checkTemp = 0;
    
    if(str_Receiv->Data_a8[0] == '-')
      checkTemp = 1;
    
    if( str_Receiv->Data_a8[checkTemp] >= '0' && str_Receiv->Data_a8[checkTemp] <= '9')
    {
        uint8_t length = 0;
        for(uint8_t i = checkTemp; i < str_Receiv->Length_u16; i++)
        {
            if( str_Receiv->Data_a8[i] < '0' || str_Receiv->Data_a8[i]>'9') break;
            else length++;
        }
        Temp_I32 = Convert_String_To_Dec(str_Receiv->Data_a8 + checkTemp, length);
        
        if(checkTemp == 1)
          Temp_I32 = 0 - Temp_I32;
        
        Temp_f = Handle_int32_To_Float_Scale(Temp_I32, 0xFE);
        Save_OffsetMeasure(_OFFSET_CLO, Temp_f);
        Modem_Respond(PortConfig, (uint8_t*)"OK", 2, 0);
    }
    else
    {
        Modem_Respond(PortConfig, (uint8_t*)"ERROR", 5, 0);
    }
}

void AT_CMD_Get_Offset_pH (sData *str, uint16_t Pos)
{
    uint8_t aTemp[50] = "Offset pH: ";   //13 ki tu dau tien
    sData StrResp = {&aTemp[0], 11}; 

    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sOffsetMeasure.pH_f*100), 0xFE);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" pH",0 , 3);

	Modem_Respond(PortConfig, StrResp.Data_a8, StrResp.Length_u16, 0);
}

void AT_CMD_Set_Offset_pH (sData *str_Receiv, uint16_t Pos)
{
    int32_t  Temp_I32 = 0;
    float    Temp_f = 0;
    
    uint8_t checkTemp = 0;
    
    if(str_Receiv->Data_a8[0] == '-')
      checkTemp = 1;
    
    if( str_Receiv->Data_a8[checkTemp] >= '0' && str_Receiv->Data_a8[checkTemp] <= '9')
    {
        uint8_t length = 0;
        for(uint8_t i = checkTemp; i < str_Receiv->Length_u16; i++)
        {
            if( str_Receiv->Data_a8[i] < '0' || str_Receiv->Data_a8[i]>'9') break;
            else length++;
        }
        Temp_I32 = Convert_String_To_Dec(str_Receiv->Data_a8 + checkTemp, length);
        
        if(checkTemp == 1)
          Temp_I32 = 0 - Temp_I32;
        
        Temp_f = Handle_int32_To_Float_Scale(Temp_I32, 0xFE);
        Save_OffsetMeasure(_OFFSET_PH, Temp_f);
        Modem_Respond(PortConfig, (uint8_t*)"OK", 2, 0);
    }
    else
    {
        Modem_Respond(PortConfig, (uint8_t*)"ERROR", 5, 0);
    }
}
void AT_CMD_Get_Offset_NTU (sData *str, uint16_t Pos)
{
    uint8_t aTemp[50] = "Offset NTU: ";   //13 ki tu dau tien
    sData StrResp = {&aTemp[0], 12}; 

    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sOffsetMeasure.Turb_f*100), 0xFE);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" NTU",0 , 4);

	Modem_Respond(PortConfig, StrResp.Data_a8, StrResp.Length_u16, 0);
}

void AT_CMD_Set_Offset_NTU (sData *str_Receiv, uint16_t Pos)
{
    int32_t  Temp_I32 = 0;
    float    Temp_f = 0;
    
    uint8_t checkTemp = 0;
    
    if(str_Receiv->Data_a8[0] == '-')
      checkTemp = 1;
    
    if( str_Receiv->Data_a8[checkTemp] >= '0' && str_Receiv->Data_a8[checkTemp] <= '9')
    {
        uint8_t length = 0;
        for(uint8_t i = checkTemp; i < str_Receiv->Length_u16; i++)
        {
            if( str_Receiv->Data_a8[i] < '0' || str_Receiv->Data_a8[i]>'9') break;
            else length++;
        }
        Temp_I32 = Convert_String_To_Dec(str_Receiv->Data_a8 + checkTemp, length);
        
        if(checkTemp == 1)
          Temp_I32 = 0 - Temp_I32;
        
        Temp_f = Handle_int32_To_Float_Scale(Temp_I32, 0xFE);
        Save_OffsetMeasure(_OFFSET_TURB, Temp_f);
        Modem_Respond(PortConfig, (uint8_t*)"OK", 2, 0);
    }
    else
    {
        Modem_Respond(PortConfig, (uint8_t*)"ERROR", 5, 0);
    }
}

void AT_CMD_Get_Offset_EC (sData *str, uint16_t Pos)
{
    uint8_t aTemp[50] = "Offset EC: ";   //13 ki tu dau tien
    sData StrResp = {&aTemp[0], 11}; 

    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sOffsetMeasure.EC_f*100), 0xFE);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" uS/Cm",0 , 6);

	Modem_Respond(PortConfig, StrResp.Data_a8, StrResp.Length_u16, 0);
}

void AT_CMD_Set_Offset_EC (sData *str_Receiv, uint16_t Pos)
{
    int32_t  Temp_I32 = 0;
    float    Temp_f = 0;
    
    uint8_t checkTemp = 0;
    
    if(str_Receiv->Data_a8[0] == '-')
      checkTemp = 1;
    
    if( str_Receiv->Data_a8[checkTemp] >= '0' && str_Receiv->Data_a8[checkTemp] <= '9')
    {
        uint8_t length = 0;
        for(uint8_t i = checkTemp; i < str_Receiv->Length_u16; i++)
        {
            if( str_Receiv->Data_a8[i] < '0' || str_Receiv->Data_a8[i]>'9') break;
            else length++;
        }
        Temp_I32 = Convert_String_To_Dec(str_Receiv->Data_a8 + checkTemp, length);
        
        if(checkTemp == 1)
          Temp_I32 = 0 - Temp_I32;
        
        Temp_f = Handle_int32_To_Float_Scale(Temp_I32, 0xFE);
        Save_OffsetMeasure(_OFFSET_EC, Temp_f);
        Modem_Respond(PortConfig, (uint8_t*)"OK", 2, 0);
    }
    else
    {
        Modem_Respond(PortConfig, (uint8_t*)"ERROR", 5, 0);
    }
}

void AT_CMD_Get_Offset_Sal (sData *str, uint16_t Pos)
{
    uint8_t aTemp[50] = "Offset Sal: ";   //13 ki tu dau tien
    sData StrResp = {&aTemp[0], 12}; 

    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sOffsetMeasure.Sal_f*100), 0xFE);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" %",0 , 2);

	Modem_Respond(PortConfig, StrResp.Data_a8, StrResp.Length_u16, 0);
}

void AT_CMD_Set_Offset_Sal (sData *str_Receiv, uint16_t Pos)
{
    int32_t  Temp_I32 = 0;
    float    Temp_f = 0;
    
    uint8_t checkTemp = 0;
    
    if(str_Receiv->Data_a8[0] == '-')
      checkTemp = 1;
    
    if( str_Receiv->Data_a8[checkTemp] >= '0' && str_Receiv->Data_a8[checkTemp] <= '9')
    {
        uint8_t length = 0;
        for(uint8_t i = checkTemp; i < str_Receiv->Length_u16; i++)
        {
            if( str_Receiv->Data_a8[i] < '0' || str_Receiv->Data_a8[i]>'9') break;
            else length++;
        }
        Temp_I32 = Convert_String_To_Dec(str_Receiv->Data_a8 + checkTemp, length);
        
        if(checkTemp == 1)
          Temp_I32 = 0 - Temp_I32;
        
        Temp_f = Handle_int32_To_Float_Scale(Temp_I32, 0xFE);
        Save_OffsetMeasure(_OFFSET_SAL, Temp_f);
        Modem_Respond(PortConfig, (uint8_t*)"OK", 2, 0);
    }
    else
    {
        Modem_Respond(PortConfig, (uint8_t*)"ERROR", 5, 0);
    }
}

void AT_CMD_Get_Offset_Temp (sData *str, uint16_t Pos)
{
    uint8_t aTemp[50] = "Offset Temp: ";   //13 ki tu dau tien
    sData StrResp = {&aTemp[0], 13}; 

    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sOffsetMeasure.Temp_f*100), 0xFE);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" °C",0 , 3);

	Modem_Respond(PortConfig, StrResp.Data_a8, StrResp.Length_u16, 0);
}

void AT_CMD_Set_Offset_Temp (sData *str_Receiv, uint16_t Pos)
{
    int32_t  Temp_I32 = 0;
    float    Temp_f = 0;
    
    uint8_t checkTemp = 0;
    
    if(str_Receiv->Data_a8[0] == '-')
      checkTemp = 1;
    
    if( str_Receiv->Data_a8[checkTemp] >= '0' && str_Receiv->Data_a8[checkTemp] <= '9')
    {
        uint8_t length = 0;
        for(uint8_t i = checkTemp; i < str_Receiv->Length_u16; i++)
        {
            if( str_Receiv->Data_a8[i] < '0' || str_Receiv->Data_a8[i]>'9') break;
            else length++;
        }
        Temp_I32 = Convert_String_To_Dec(str_Receiv->Data_a8 + checkTemp, length);
        
        if(checkTemp == 1)
          Temp_I32 = 0 - Temp_I32;
        
        Temp_f = Handle_int32_To_Float_Scale(Temp_I32, 0xFE);
        Save_OffsetMeasure(_OFFSET_TEMP, Temp_f);
        Modem_Respond(PortConfig, (uint8_t*)"OK", 2, 0);
    }
    else
    {
        Modem_Respond(PortConfig, (uint8_t*)"ERROR", 5, 0);
    }
}

void AT_CMD_Get_User_Sensor (sData *str_Receiv, uint16_t Pos)
{
    uint8_t aTemp[80] = "";   //11 ki tu dau tien
    uint16_t length = 0;

    Insert_String_To_String(aTemp, &length, (uint8_t*)"User_pH:",0 , 8);
    Convert_Point_Int_To_String_Scale (aTemp, &length, (int)(sUserSensor.User_pH), 0x00);

    Insert_String_To_String(aTemp, &length, (uint8_t*)" User_Clo: ",0 , 10);
    Convert_Point_Int_To_String_Scale (aTemp, &length, (int)(sUserSensor.User_Clo), 0x00);
    
    Insert_String_To_String(aTemp, &length, (uint8_t*)" User_EC:",0 , 9);
    Convert_Point_Int_To_String_Scale (aTemp, &length, (int)(sUserSensor.User_EC), 0x00);
    
    Insert_String_To_String(aTemp, &length, (uint8_t*)" User_Turb:",0 , 11);
    Convert_Point_Int_To_String_Scale (aTemp, &length, (int)(sUserSensor.User_Turb), 0x00);

	Modem_Respond(PortConfig, aTemp, length, 0);
}

void AT_CMD_Set_User_Sensor (sData *str_Receiv, uint16_t Pos)
{
    uint32_t TempU32 = 0;
    if( str_Receiv->Data_a8[0] >= '0' && str_Receiv->Data_a8[0] <= '9')
    {
        uint8_t length = 0;
        for(uint8_t i = 0; i < str_Receiv->Length_u16; i++)
        {
            if( str_Receiv->Data_a8[i] < '0' || str_Receiv->Data_a8[i]>'9') break;
            else length++;
        }
        TempU32 = Convert_String_To_Dec(str_Receiv->Data_a8 , length);
        
        switch(TempU32)
        {
        case 10:
            Save_UserSensor(_ACTIVE_PH, 0);
            Modem_Respond(PortConfig, (uint8_t*)"OK", 2, 0);
            break;
            
        case 11:
            Save_UserSensor(_ACTIVE_PH, 1);
            Modem_Respond(PortConfig, (uint8_t*)"OK", 2, 0);
            break;
            
        case 20:
            Save_UserSensor(_ACTIVE_CLO, 0);
            Modem_Respond(PortConfig, (uint8_t*)"OK", 2, 0);
            break;
            
        case 21:
            Save_UserSensor(_ACTIVE_CLO, 1);
            Modem_Respond(PortConfig, (uint8_t*)"OK", 2, 0);
            break;
            
        case 30:
            Save_UserSensor(_ACTIVE_EC, 0);
            Modem_Respond(PortConfig, (uint8_t*)"OK", 2, 0);
            break;
            
        case 31:
            Save_UserSensor(_ACTIVE_EC, 1);
            Modem_Respond(PortConfig, (uint8_t*)"OK", 2, 0);
            break;
            
        case 40:
            Save_UserSensor(_ACTIVE_TURB, 0);
            Modem_Respond(PortConfig, (uint8_t*)"OK", 2, 0);
            break;
            
        case 41:
            Save_UserSensor(_ACTIVE_TURB, 1);
            Modem_Respond(PortConfig, (uint8_t*)"OK", 2, 0);
            break;
          
        default:
            Modem_Respond(PortConfig, (uint8_t*)"ERROR", 5, 0);
            break;
        }
    }
    else
    {
        Modem_Respond(PortConfig, (uint8_t*)"ERROR", 5, 0);
    }
}

#endif

/*---------------------Save and Init User Sensor----------------------*/
void Save_UserSensor(uint8_t KindSensor, uint8_t State)
{
    uint8_t aData[10] = {0};
    uint8_t length = 0;
  
    uint8_t Sensor_pH = 0;
    uint8_t Sensor_Clo = 0;
    uint8_t Sensor_EC = 0;
    uint8_t Sensor_Turb = 0;
    
    switch(KindSensor)
    {
        case _ACTIVE_PH:
            sUserSensor.User_pH = State;
            break;
            
        case _ACTIVE_CLO:
            sUserSensor.User_Clo = State;
            break;
            
        case _ACTIVE_EC:
            sUserSensor.User_EC = State;
            break;
            
        case _ACTIVE_TURB:
            sUserSensor.User_Turb = State;
            break;
    }
    
    Sensor_pH = sUserSensor.User_pH;
    Sensor_Clo = sUserSensor.User_Clo;
    Sensor_EC = sUserSensor.User_EC;
    Sensor_Turb = sUserSensor.User_Turb;
    
    aData[length++] = Sensor_pH;
    aData[length++] = Sensor_Clo;
    aData[length++] = Sensor_EC;
    aData[length++] = Sensor_Turb;
    
    Save_Array(ADDR_USER_SENSOR, aData, length);
}

void Init_UserSensor(void)
{
#ifdef USING_INTERNAL_MEM
    if(*(__IO uint8_t*)(ADDR_USER_SENSOR) != FLASH_BYTE_EMPTY)
    {
        sUserSensor.User_pH = *(__IO uint8_t*)(ADDR_USER_SENSOR+2);
        sUserSensor.User_Clo = *(__IO uint8_t*)(ADDR_USER_SENSOR+3);
        sUserSensor.User_EC = *(__IO uint8_t*)(ADDR_USER_SENSOR+4);
        sUserSensor.User_Turb = *(__IO uint8_t*)(ADDR_USER_SENSOR+5);
    }
    else
    {
        sUserSensor.User_pH = 0;
        sUserSensor.User_Clo = 0;
        sUserSensor.User_EC = 0;
        sUserSensor.User_Turb = 0;
    }
#endif    
}
/*===================Save and Init Offset Measure=================*/
void Save_OffsetMeasure(uint8_t KindOffset, float Var_Offset_f)
{
#ifdef USING_INTERNAL_MEM
    uint8_t aData[100] = {0};
    uint8_t length = 0;
    
    uint32_t hexUint_Compensation_Clo = 0;
    uint32_t hexUint_Compensation_pH = 0;
    uint32_t hexUint_Compensation_Turb = 0;
    uint32_t hexUint_Compensation_EC = 0;
    uint32_t hexUint_Compensation_Sal = 0;
    uint32_t hexUint_Compensation_Temp = 0;
 
    switch(KindOffset)
    {
        case _OFFSET_CLO:
          sOffsetMeasure.Clo_f = Var_Offset_f;
          break;
        case _OFFSET_PH:
          sOffsetMeasure.pH_f = Var_Offset_f;
          break;
        case _OFFSET_TURB:
          sOffsetMeasure.Turb_f = Var_Offset_f;
          break;
        case _OFFSET_EC:
          sOffsetMeasure.EC_f = Var_Offset_f;
          break;
        case _OFFSET_SAL:
          sOffsetMeasure.Sal_f = Var_Offset_f;
          break;
        case _OFFSET_TEMP:
          sOffsetMeasure.Temp_f = Var_Offset_f;
          break;
          
        default:
          break;
    }
    
    hexUint_Compensation_Clo = Handle_Float_To_hexUint32(sOffsetMeasure.Clo_f);
    hexUint_Compensation_pH = Handle_Float_To_hexUint32(sOffsetMeasure.pH_f);
    hexUint_Compensation_Turb = Handle_Float_To_hexUint32(sOffsetMeasure.Turb_f);
    hexUint_Compensation_EC = Handle_Float_To_hexUint32(sOffsetMeasure.EC_f);
    hexUint_Compensation_Sal = Handle_Float_To_hexUint32(sOffsetMeasure.Sal_f);
    hexUint_Compensation_Temp = Handle_Float_To_hexUint32(sOffsetMeasure.Temp_f);
    
    aData[length++] = hexUint_Compensation_Clo >> 24;
    aData[length++] = hexUint_Compensation_Clo >> 16;
    aData[length++] = hexUint_Compensation_Clo >> 8;
    aData[length++] = hexUint_Compensation_Clo ;
    
    aData[length++] = hexUint_Compensation_pH >> 24;
    aData[length++] = hexUint_Compensation_pH >> 16;
    aData[length++] = hexUint_Compensation_pH >> 8;
    aData[length++] = hexUint_Compensation_pH ;
    
    aData[length++] = hexUint_Compensation_Turb >> 24;
    aData[length++] = hexUint_Compensation_Turb >> 16;
    aData[length++] = hexUint_Compensation_Turb >> 8;
    aData[length++] = hexUint_Compensation_Turb ;
    
    aData[length++] = hexUint_Compensation_EC >> 24;
    aData[length++] = hexUint_Compensation_EC >> 16;
    aData[length++] = hexUint_Compensation_EC >> 8;
    aData[length++] = hexUint_Compensation_EC ;
    
    aData[length++] = hexUint_Compensation_Sal >> 24;
    aData[length++] = hexUint_Compensation_Sal >> 16;
    aData[length++] = hexUint_Compensation_Sal >> 8;
    aData[length++] = hexUint_Compensation_Sal ;
    
    aData[length++] = hexUint_Compensation_Temp >> 24;
    aData[length++] = hexUint_Compensation_Temp >> 16;
    aData[length++] = hexUint_Compensation_Temp >> 8;
    aData[length++] = hexUint_Compensation_Temp ;
    
    Save_Array(ADDR_OFFSET_MEASURE, aData, length);
#endif
}

void Init_OffsetMeasure(void)
{
#ifdef USING_INTERNAL_MEM
  
    uint32_t hexUint_Compensation_Clo = 0;
    uint32_t hexUint_Compensation_pH = 0;
    uint32_t hexUint_Compensation_Turb = 0;
    uint32_t hexUint_Compensation_EC = 0;
    uint32_t hexUint_Compensation_Sal = 0;
    uint32_t hexUint_Compensation_Temp = 0;
    
    if(*(__IO uint8_t*)(ADDR_OFFSET_MEASURE) != FLASH_BYTE_EMPTY)
    {
        hexUint_Compensation_Clo   = *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+2) << 24;
        hexUint_Compensation_Clo  |= *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+3)<< 16;
        hexUint_Compensation_Clo  |= *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+4)<< 8;
        hexUint_Compensation_Clo  |= *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+5);
        
        hexUint_Compensation_pH   = *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+6) << 24;
        hexUint_Compensation_pH  |= *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+7)<< 16;
        hexUint_Compensation_pH  |= *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+8)<< 8;
        hexUint_Compensation_pH  |= *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+9);
        
        hexUint_Compensation_Turb   = *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+10) << 24;
        hexUint_Compensation_Turb  |= *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+11)<< 16;
        hexUint_Compensation_Turb  |= *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+12)<< 8;
        hexUint_Compensation_Turb  |= *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+13);
        
        hexUint_Compensation_EC   = *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+14) << 24;
        hexUint_Compensation_EC  |= *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+15)<< 16;
        hexUint_Compensation_EC  |= *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+16)<< 8;
        hexUint_Compensation_EC  |= *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+17);
        
        hexUint_Compensation_Sal   = *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+18) << 24;
        hexUint_Compensation_Sal  |= *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+19)<< 16;
        hexUint_Compensation_Sal  |= *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+20)<< 8;
        hexUint_Compensation_Sal  |= *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+21);
        
        hexUint_Compensation_Temp   = *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+22) << 24;
        hexUint_Compensation_Temp  |= *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+23)<< 16;
        hexUint_Compensation_Temp  |= *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+24)<< 8;
        hexUint_Compensation_Temp  |= *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+25);
        
        Convert_uint32Hex_To_Float(hexUint_Compensation_Clo,  &sOffsetMeasure.Clo_f);
        Convert_uint32Hex_To_Float(hexUint_Compensation_pH,   &sOffsetMeasure.pH_f);
        Convert_uint32Hex_To_Float(hexUint_Compensation_Turb, &sOffsetMeasure.Turb_f);
        Convert_uint32Hex_To_Float(hexUint_Compensation_EC,   &sOffsetMeasure.EC_f);
        Convert_uint32Hex_To_Float(hexUint_Compensation_Sal,  &sOffsetMeasure.Sal_f);
        Convert_uint32Hex_To_Float(hexUint_Compensation_Temp, &sOffsetMeasure.Temp_f);
    }

#endif    
}

/*====================Save and Init Time Warning==================*/
void Save_TimeWarningSensor(uint8_t Duration)
{
#ifdef USING_INTERNAL_MEM
    uint8_t aData[8] = {0};
    uint8_t length = 0;
    
    DurationTimeWarningSensor = Duration;
    
    aData[length++] = DurationTimeWarningSensor;

    Save_Array(ADDR_TIME_WARNING_SENSOR, aData, length);
#endif
}

void Init_TimeWarningSensor(void)
{
#ifdef USING_INTERNAL_MEM
    if(*(__IO uint8_t*)(ADDR_TIME_WARNING_SENSOR) != FLASH_BYTE_EMPTY)
    {
        DurationTimeWarningSensor = *(__IO uint8_t*)(ADDR_TIME_WARNING_SENSOR+2);
    }
    else
    {
        DurationTimeWarningSensor = TIME_RESEND_WARNING;
    }
#endif    
}

/*=====================Handle Task and Init app===================*/
void Init_AppSensor(void)
{
    Init_TimeWarningSensor();
    Init_OffsetMeasure();
    Init_UserSensor();
#ifdef USING_AT_CONFIG
    /* regis cb serial */
    CheckList_AT_CONFIG[_GET_FREQ_WARNING_SENSOR].CallBack = AT_CMD_Get_Time_Warning_Sensor;
    CheckList_AT_CONFIG[_SET_FREQ_WARNING_SENSOR].CallBack = AT_CMD_Set_Time_Warning_Sensor;
    
    CheckList_AT_CONFIG[_GET_OFFSET_CLO].CallBack = AT_CMD_Get_Offset_Clo;
    CheckList_AT_CONFIG[_SET_OFFSET_CLO].CallBack = AT_CMD_Set_Offset_Clo;
    
    CheckList_AT_CONFIG[_GET_OFFSET_PH].CallBack = AT_CMD_Get_Offset_pH;
    CheckList_AT_CONFIG[_SET_OFFSET_PH].CallBack = AT_CMD_Set_Offset_pH;
    
    CheckList_AT_CONFIG[_GET_OFFSET_NTU].CallBack = AT_CMD_Get_Offset_NTU;
    CheckList_AT_CONFIG[_SET_OFFSET_NTU].CallBack = AT_CMD_Set_Offset_NTU;
    
    CheckList_AT_CONFIG[_GET_OFFSET_EC].CallBack = AT_CMD_Get_Offset_EC;
    CheckList_AT_CONFIG[_SET_OFFSET_EC].CallBack = AT_CMD_Set_Offset_EC;
    
    CheckList_AT_CONFIG[_GET_OFFSET_SAL].CallBack = AT_CMD_Get_Offset_Sal;
    CheckList_AT_CONFIG[_SET_OFFSET_SAL].CallBack = AT_CMD_Set_Offset_Sal;
    
    CheckList_AT_CONFIG[_GET_OFFSET_TEMP].CallBack = AT_CMD_Get_Offset_Temp;
    CheckList_AT_CONFIG[_SET_OFFSET_TEMP].CallBack = AT_CMD_Set_Offset_Temp;
    
    CheckList_AT_CONFIG[_GET_USER_SENSOR].CallBack = AT_CMD_Get_User_Sensor;
    CheckList_AT_CONFIG[_SET_USER_SENSOR].CallBack = AT_CMD_Set_User_Sensor;
#endif
}

uint8_t AppSensor_Task(void)
{
    uint8_t i = 0;
    uint8_t Result = false;
    for( i = 0; i < _EVENT_SENSOR_END; i++)
    {
        if(sEventAppSensor[i].e_status == 1)
        {
            Result = true;
            if((sEventAppSensor[i].e_systick == 0) ||
                ((HAL_GetTick() - sEventAppSensor[i].e_systick) >= sEventAppSensor[i].e_period))
            {
                sEventAppSensor[i].e_status = 0; //Disable event
                sEventAppSensor[i].e_systick= HAL_GetTick();
                sEventAppSensor[i].e_function_handler(i);
            }
        }
    }
    
    return Result;
}
