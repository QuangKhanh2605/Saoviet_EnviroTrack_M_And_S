
#include "user_app_sensor.h"
#include "user_define.h"
#include "user_convert_variable.h"
#include "math.h"

/*============== Function static ===============*/
static uint8_t fevent_sensor_entry(uint8_t event);

static uint8_t fevent_sensor_state_ph(uint8_t event);
static uint8_t fevent_sensor_state_clo(uint8_t event);
static uint8_t fevent_sensor_state_ec(uint8_t event);
static uint8_t fevent_sensor_state_turbidity(uint8_t event);
/*=================== struct ==================*/
sEvent_struct               sEventAppSensor[] = 
{
  {_EVENT_SENSOR_ENTRY,              1, 5, 60000,                fevent_sensor_entry},
  
  {_EVENT_SENSOR_STATE_PH,           0, 5, 500,                  fevent_sensor_state_ph},
  {_EVENT_SENSOR_STATE_CLO,          0, 5, 500,                  fevent_sensor_state_clo},
  {_EVENT_SENSOR_STATE_EC,           0, 5, 500,                  fevent_sensor_state_ec},
  {_EVENT_SENSOR_STATE_TURBIDITY,    0, 5, 500,                  fevent_sensor_state_turbidity},
};

Struct_Offset_Measure   sOffsetMeasure = {0};

uint8_t DurationTimeWarningSensor = 0;

int16_t aSampling_STT_pH[NUMBER_SAMPLING_SS] = {0};
int16_t aSampling_VALUE_pH[NUMBER_SAMPLING_SS] = {0};

int16_t aSampling_STT_Clo[NUMBER_SAMPLING_SS] = {0};
int16_t aSampling_VALUE_Clo[NUMBER_SAMPLING_SS] = {0};

int16_t aSampling_STT_EC[NUMBER_SAMPLING_SS] = {0};
int16_t aSampling_VALUE_EC[NUMBER_SAMPLING_SS] = {0};

int16_t aSampling_STT_Turbidity[NUMBER_SAMPLING_SS] = {0};
int16_t aSampling_VALUE_Turbidity[NUMBER_SAMPLING_SS] = {0};

Struct_ConvertChlorine      sConvertChlorine ={0};
int32_t DCU_Log_DataCalib = 0;
uint8_t DCU_Log_KindCalib = 0;
/*================= Function Handle ==============*/

static uint8_t fevent_sensor_entry(uint8_t event)
{
    fevent_enable(sEventAppSensor, _EVENT_SENSOR_STATE_PH);
    return 1;
}

static uint8_t fevent_sensor_state_ph(uint8_t event)
{
    static uint32_t gettick_state_slave = 0;
    static uint8_t StateSensor_Before = _SENSOR_CONNECT;
    uint8_t aData[2] = {0};

    if(sSensor_pH.State_Connect != StateSensor_Before)
    {
        gettick_state_slave = HAL_GetTick();
        
        if(sSensor_pH.State_Connect == _SENSOR_CONNECT)      
        {

        }
        else
        {
            aData[0] = 0x00;
            aData[1] = 0x00;
            Log_EventWarnig(OBIS_WARNING_SENSOR_CONNECT, 0x01, aData);
        }

        StateSensor_Before = sSensor_pH.State_Connect;
    }

    if(sSensor_pH.State_Connect == _SENSOR_DISCONNECT)
    {
        if(HAL_GetTick() - gettick_state_slave >= DurationTimeWarningSensor*60000)
        {
            gettick_state_slave = HAL_GetTick();
            aData[0] = 0x00;
            aData[1] = 0x00;
            Log_EventWarnig(OBIS_WARNING_SENSOR_CONNECT, 0x01, aData);
        }
    }

    fevent_enable(sEventAppSensor, _EVENT_SENSOR_STATE_CLO);
    return 1;
}

static uint8_t fevent_sensor_state_clo(uint8_t event)
{
    static uint32_t gettick_state_slave = 0;
    static uint8_t StateSensor_Before = _SENSOR_CONNECT;
    uint8_t aData[2] = {0};

    if(sSensor_Clo.State_Connect != StateSensor_Before)
    {
        gettick_state_slave = HAL_GetTick();
        
        if(sSensor_Clo.State_Connect == _SENSOR_CONNECT)      
        {

        }
        else
        {
            aData[0] = 0x02;
            aData[1] = 0x00;
            Log_EventWarnig(OBIS_WARNING_SENSOR_CONNECT, 0x01, aData);
        }

        StateSensor_Before = sSensor_Clo.State_Connect;
    }

    if(sSensor_Clo.State_Connect == _SENSOR_DISCONNECT)
    {
        if(HAL_GetTick() - gettick_state_slave >= DurationTimeWarningSensor*60000)
        {
            gettick_state_slave = HAL_GetTick();
            aData[0] = 0x02;
            aData[1] = 0x00;
            Log_EventWarnig(OBIS_WARNING_SENSOR_CONNECT, 0x01, aData);
        }
    }

    fevent_enable(sEventAppSensor, _EVENT_SENSOR_STATE_EC);
    return 1;
}

static uint8_t fevent_sensor_state_ec(uint8_t event)
{
    static uint32_t gettick_state_slave = 0;
    static uint8_t StateSensor_Before = _SENSOR_CONNECT;
    uint8_t aData[2] = {0};

    if(sSensor_EC.State_Connect != StateSensor_Before)
    {
        gettick_state_slave = HAL_GetTick();
        
        if(sSensor_EC.State_Connect == _SENSOR_CONNECT)      
        {

        }
        else
        {
            aData[0] = 0x04;
            aData[1] = 0x00;
            Log_EventWarnig(OBIS_WARNING_SENSOR_CONNECT, 0x01, aData);
        }

        StateSensor_Before = sSensor_EC.State_Connect;
    }

    if(sSensor_EC.State_Connect == _SENSOR_DISCONNECT)
    {
        if(HAL_GetTick() - gettick_state_slave >= DurationTimeWarningSensor*60000)
        {
            gettick_state_slave = HAL_GetTick();
            aData[0] = 0x04;
            aData[1] = 0x00;
            Log_EventWarnig(OBIS_WARNING_SENSOR_CONNECT, 0x01, aData);
        }
    }

    fevent_enable(sEventAppSensor, _EVENT_SENSOR_STATE_TURBIDITY);
    return 1;
}

static uint8_t fevent_sensor_state_turbidity(uint8_t event)
{
    static uint32_t gettick_state_slave = 0;
    static uint8_t StateSensor_Before = _SENSOR_CONNECT;
    uint8_t aData[2] = {0};

    if(sSensor_Turbidity.State_Connect != StateSensor_Before)
    {
        gettick_state_slave = HAL_GetTick();
        
        if(sSensor_Turbidity.State_Connect == _SENSOR_CONNECT)      
        {

        }
        else
        {
            aData[0] = 0x06;
            aData[1] = 0x00;
            Log_EventWarnig(OBIS_WARNING_SENSOR_CONNECT, 0x01, aData);
        }

        StateSensor_Before = sSensor_Turbidity.State_Connect;
    }

    if(sSensor_Turbidity.State_Connect == _SENSOR_DISCONNECT)
    {
        if(HAL_GetTick() - gettick_state_slave >= DurationTimeWarningSensor*60000)
        {
            gettick_state_slave = HAL_GetTick();
            aData[0] = 0x06;
            aData[1] = 0x00;
            Log_EventWarnig(OBIS_WARNING_SENSOR_CONNECT, 0x01, aData);
        }
    }

    fevent_enable(sEventAppSensor, _EVENT_SENSOR_STATE_PH);
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

    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sOffsetMeasure.sClo_Du.Value), sOffsetMeasure.sClo_Du.Scale);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" mg/L",0 , 5);

	Modem_Respond(PortConfig, StrResp.Data_a8, StrResp.Length_u16, 0);
}

void AT_CMD_Set_Offset_Clo (sData *str_Receiv, uint16_t Pos)
{
    uint32_t TempU32 = 0;
    Struct_SS_Value sMeasure;
    if( str_Receiv->Data_a8[0] >= '0' && str_Receiv->Data_a8[0] <= '9')
    {
        uint8_t length = 0;
        for(uint8_t i = 0; i < str_Receiv->Length_u16; i++)
        {
            if( str_Receiv->Data_a8[i] < '0' || str_Receiv->Data_a8[i]>'9') break;
            else length++;
        }
        TempU32 = Convert_String_To_Dec(str_Receiv->Data_a8 , length);
        if(TempU32 <= 32765)
        {
            sMeasure.Value = TempU32;
            Save_OffsetMeasure_cmdAT(_OFFSET_CLO_DU, sMeasure);
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

void AT_CMD_Get_Offset_pH (sData *str, uint16_t Pos)
{
    uint8_t aTemp[50] = "Offset pH: ";   //13 ki tu dau tien
    sData StrResp = {&aTemp[0], 11}; 

    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sOffsetMeasure.spH_Water.Value), sOffsetMeasure.spH_Water.Scale);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" pH",0 , 3);

	Modem_Respond(PortConfig, StrResp.Data_a8, StrResp.Length_u16, 0);
}

void AT_CMD_Set_Offset_pH (sData *str_Receiv, uint16_t Pos)
{
    uint32_t TempU32 = 0;
    Struct_SS_Value sMeasure;
    if( str_Receiv->Data_a8[0] >= '0' && str_Receiv->Data_a8[0] <= '9')
    {
        uint8_t length = 0;
        for(uint8_t i = 0; i < str_Receiv->Length_u16; i++)
        {
            if( str_Receiv->Data_a8[i] < '0' || str_Receiv->Data_a8[i]>'9') break;
            else length++;
        }
        TempU32 = Convert_String_To_Dec(str_Receiv->Data_a8 , length);
        if(TempU32 <= 32765)
        {
            sMeasure.Value = TempU32;
            Save_OffsetMeasure_cmdAT(_OFFSET_PH, sMeasure);
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
void AT_CMD_Get_Offset_NTU (sData *str, uint16_t Pos)
{
    uint8_t aTemp[50] = "Offset NTU: ";   //13 ki tu dau tien
    sData StrResp = {&aTemp[0], 12}; 

    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sOffsetMeasure.sNTU.Value), sOffsetMeasure.sNTU.Scale);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" NTU",0 , 4);

	Modem_Respond(PortConfig, StrResp.Data_a8, StrResp.Length_u16, 0);
}

void AT_CMD_Set_Offset_NTU (sData *str_Receiv, uint16_t Pos)
{
    uint32_t TempU32 = 0;
    Struct_SS_Value sMeasure;
    if( str_Receiv->Data_a8[0] >= '0' && str_Receiv->Data_a8[0] <= '9')
    {
        uint8_t length = 0;
        for(uint8_t i = 0; i < str_Receiv->Length_u16; i++)
        {
            if( str_Receiv->Data_a8[i] < '0' || str_Receiv->Data_a8[i]>'9') break;
            else length++;
        }
        TempU32 = Convert_String_To_Dec(str_Receiv->Data_a8 , length);
        if(TempU32 <= 32765)
        {
            sMeasure.Value = TempU32;
            Save_OffsetMeasure_cmdAT(_OFFSET_NTU, sMeasure);
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

void       AT_CMD_Get_Clo_Const_Temp(sData *str_Receiv, uint16_t Pos)
{
    uint8_t aTemp[100] = "Clo Const Temp: ";   //13 ki tu dau tien
    sData StrResp = {&aTemp[0], 16}; 

    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sConvertChlorine.sConst_Compensation_Temp.Value), sConvertChlorine.sConst_Compensation_Temp.Scale);
    
	Modem_Respond(PortConfig, StrResp.Data_a8, StrResp.Length_u16, 0);
}
void       AT_CMD_Set_Clo_Const_Temp(sData *str_Receiv, uint16_t Pos)
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
        if(TempU32 <= 10000 && TempU32 >= 1)
        {
            Save_Const_Temp_Compensation_Chlorine(TempU32);
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


void       AT_CMD_Get_Clo_Zero_Slope(sData *str_Receiv, uint16_t Pos)
{
  uint8_t aTemp[100] = "Clo Zero Slope:ADC: ";   //13 ki tu dau tien
    sData StrResp = {&aTemp[0], 20}; 

    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sConvertChlorine.Measure_AD), 0xFE);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" Zero:",0 , 6);
    
    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sConvertChlorine.ADC_Zero), 0xFE);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" Slope:",0 , 7);
    
    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sConvertChlorine.ADC_SLope), 0xFE);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" Clo:",0 , 5);
    
    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sConvertChlorine.Clo_Calib_Slope), 0xFE);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" Temp:",0 , 6);
    
    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sConvertChlorine.Temp_Calib_Slope), 0xFE);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" pH:",0 , 4);
    
    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sConvertChlorine.Ph_Calib_Slope), 0xFE);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" Measure:",0 , 9);
    
    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sConvertChlorine.sClo_Du.Value), 0xFE);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)"END",0 , 3);

	Modem_Respond(PortConfig, StrResp.Data_a8, StrResp.Length_u16, 0);
}
void       AT_CMD_Set_Clo_Zero_Slope(sData *str_Receiv, uint16_t Pos)
{
    uint32_t TempU32 = 0;
    uint8_t Text_1 = 0;
    uint8_t Text_2 = 0;
    Text_1 = str_Receiv->Data_a8[0];
    Text_2 = str_Receiv->Data_a8[1];
    if( str_Receiv->Data_a8[3] >= '0' && str_Receiv->Data_a8[3] <= '9')
    {
        uint8_t length = 0;
        for(uint8_t i = 3; i < str_Receiv->Length_u16; i++)
        {
            if( str_Receiv->Data_a8[i] < '0' || str_Receiv->Data_a8[i]>'9') break;
            else length++;
        }
        TempU32 = Convert_String_To_Dec(&(str_Receiv->Data_a8[3]) , length);
        if(TempU32 <= 5000 && TempU32 >= 1)
        {
            uint16_t ADC_Zero = sConvertChlorine.ADC_Zero;
            uint16_t ADC_Slope = sConvertChlorine.ADC_SLope;
            int16_t Clo_Calib = sConvertChlorine.Clo_Calib_Slope;
            int16_t Temp_Calib = sConvertChlorine.Temp_Calib_Slope;
            int16_t pH_Calib = sConvertChlorine.Ph_Calib_Slope;
            uint8_t Result = 1;
            if(Text_1 == 'Z' && Text_2 == '0')
                ADC_Zero = TempU32;
            else if(Text_1 == 'S' && Text_2 == '0')
                ADC_Slope = TempU32;
            else if(Text_1 == 'C' && Text_2 == '0')
                Clo_Calib = TempU32;
            else if(Text_1 == 'T' && Text_2 == '0')
                Temp_Calib = TempU32;
            else if(Text_1 == 'P' && Text_2 == '0')
                pH_Calib = TempU32;
            else
                Result = 0;
            
            if(Result == 1)
            {
                Save_Chlorine_Calib(ADC_Zero, ADC_Slope, Clo_Calib, Temp_Calib, pH_Calib);
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
    else
    {
        Modem_Respond(PortConfig, (uint8_t*)"ERROR", 5, 0);
    }
}


void       AT_CMD_Get_Clo_Calib_Point(sData *str_Receiv, uint16_t Pos)
{
  uint8_t aTemp[100] = "Clo Calib Point:ADC_1: ";   //13 ki tu dau tien
    sData StrResp = {&aTemp[0], 23}; 
    
    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sConvertChlorine.ADC_CalibPoint_1), 0xFE);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" Clo1:",0 , 6);
    
    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sConvertChlorine.Clo_CalibPoint_1), 0xFE);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" Temp1:",0 , 7);
    
    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sConvertChlorine.Temp_CalibPoint_1), 0xFE);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" pH1:",0 , 5);
    
    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sConvertChlorine.Ph_CalibPoint_1), 0xFE);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" ADC2:",0 , 6);
    
    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sConvertChlorine.ADC_CalibPoint_2), 0xFE);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" Clo2:",0 , 6);
    
    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sConvertChlorine.Clo_CalibPoint_2), 0xFE);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" Temp2:",0 , 7);
    
    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sConvertChlorine.Temp_CalibPoint_2), 0xFE);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" pH2:",0 , 5);
    
    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sConvertChlorine.Ph_CalibPoint_2), 0xFE);

	Modem_Respond(PortConfig, StrResp.Data_a8, StrResp.Length_u16, 0);
}
void       AT_CMD_Set_Clo_Calib_Point(sData *str_Receiv, uint16_t Pos)
{
    uint32_t TempU32 = 0;
    uint8_t Text_1 = 0;
    uint8_t Text_2 = 0;
    Text_1 = str_Receiv->Data_a8[0];
    Text_2 = str_Receiv->Data_a8[1];
    if( str_Receiv->Data_a8[3] >= '0' && str_Receiv->Data_a8[3] <= '9')
    {
        uint8_t length = 0;
        for(uint8_t i = 3; i < str_Receiv->Length_u16; i++)
        {
            if( str_Receiv->Data_a8[i] < '0' || str_Receiv->Data_a8[i]>'9') break;
            else length++;
        }
        TempU32 = Convert_String_To_Dec(&(str_Receiv->Data_a8[3]) , length);
        if(TempU32 <= 5000 && TempU32 >= 1)
        {
            uint16_t ADC_Point1 = sConvertChlorine.ADC_CalibPoint_1;
            int16_t Clo_Point1 = sConvertChlorine.Clo_CalibPoint_1;
            int16_t Temp_Point1 = sConvertChlorine.Temp_CalibPoint_1;
            int16_t pH_Point1 = sConvertChlorine.Ph_CalibPoint_1;
            uint16_t ADC_Point2 = sConvertChlorine.ADC_CalibPoint_2;
            int16_t Clo_Point2 = sConvertChlorine.Clo_CalibPoint_2;
            int16_t Temp_Point2 = sConvertChlorine.Temp_CalibPoint_2;
            int16_t pH_Point2 = sConvertChlorine.Ph_CalibPoint_2;
            
            uint8_t Result = 1;
            if(Text_1 == 'A' && Text_2 == '1')
                ADC_Point1 = TempU32;
            else if(Text_1 == 'C' && Text_2 == '1')
                Clo_Point1 = TempU32;
            else if(Text_1 == 'T' && Text_2 == '1')
                Temp_Point1 = TempU32;
            else if(Text_1 == 'P' && Text_2 == '1')
                pH_Point1 = TempU32;
            else if(Text_1 == 'A' && Text_2 == '2')
                ADC_Point2 = TempU32;
            else if(Text_1 == 'C' && Text_2 == '2')
                Clo_Point2 = TempU32;
            else if(Text_1 == 'T' && Text_2 == '2')
                Temp_Point2 = TempU32;
            else if(Text_1 == 'P' && Text_2 == '2')
                pH_Point2 = TempU32;
            else
                Result = 0;
            
            if(Result == 1)
            {
                Save_Chlorine_PointCalib(ADC_Point1, Clo_Point1, Temp_Point1, pH_Point1,
                                         ADC_Point2, Clo_Point2, Temp_Point2, pH_Point2);
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
    else
    {
        Modem_Respond(PortConfig, (uint8_t*)"ERROR", 5, 0);
    }
}
#endif
/*===================Save and Init Offset Measure=================*/
void Save_OffsetMeasure_cmdAT(uint8_t KindOffset, Struct_SS_Value sMeasure)
{
#ifdef USING_INTERNAL_MEM
    switch(KindOffset)
    {
        case _OFFSET_CLO_DU:
          if(sMeasure.Value != 0)
              sOffsetMeasure.sClo_Du.Value = sMeasure.Value - sOffsetMeasure.sClo_Du_Sampling.Value;
          else
              sOffsetMeasure.sClo_Du.Value = 0;
          
          Save_OffsetMeasure(KindOffset, sOffsetMeasure.sClo_Du);
          break;
          
        case _OFFSET_PH:
          if(sMeasure.Value != 0)
              sOffsetMeasure.spH_Water.Value = sMeasure.Value - sOffsetMeasure.spH_Water_Sampling.Value;
          else
              sOffsetMeasure.spH_Water.Value = 0;
          
          Save_OffsetMeasure(KindOffset, sOffsetMeasure.spH_Water);
          break;
          
        case _OFFSET_NTU:
          if(sMeasure.Value != 0)
              sOffsetMeasure.sNTU.Value = sMeasure.Value - sOffsetMeasure.sNTU_Sampling.Value;
          else
              sOffsetMeasure.sNTU.Value = 0;
          
          Save_OffsetMeasure(KindOffset, sOffsetMeasure.sNTU);
          break;
          
        case _OFFSET_SALINITY:
          if(sMeasure.Value != 0)
              sOffsetMeasure.sSalinity.Value = sMeasure.Value - sOffsetMeasure.sSalinity_Sampling.Value;
          else
              sOffsetMeasure.sSalinity.Value = 0;
          
          Save_OffsetMeasure(KindOffset, sOffsetMeasure.sSalinity);
          break;
          
        case _OFFSET_TEMPERATURE:
          if(sMeasure.Value != 0)
              sOffsetMeasure.sTemperature.Value = sMeasure.Value - sOffsetMeasure.sTemperature_Sampling.Value;
          else
              sOffsetMeasure.sTemperature.Value = 0;
          
          Save_OffsetMeasure(KindOffset, sOffsetMeasure.sTemperature);
          break;
          
        case _OFFSET_EC:
          if(sMeasure.Value != 0)
              sOffsetMeasure.sEC.Value = sMeasure.Value - sOffsetMeasure.sEC_Sampling.Value;
          else
              sOffsetMeasure.sEC.Value = 0;
          
          Save_OffsetMeasure(KindOffset, sOffsetMeasure.sEC);
          break;
          
        default:
          break;
    }
#endif
}

void Save_OffsetMeasure(uint8_t KindOffset, Struct_SS_Value sValueMeasure)
{
#ifdef USING_INTERNAL_MEM
    uint8_t aData[20] = {0};
    uint8_t length = 0;

    aData[length++] = sOffsetMeasure.sClo_Du.Value >> 8;
    aData[length++] = sOffsetMeasure.sClo_Du.Value;
    
    aData[length++] = sOffsetMeasure.spH_Water.Value >> 8;
    aData[length++] = sOffsetMeasure.spH_Water.Value;
    
    aData[length++] = sOffsetMeasure.sNTU.Value >> 8;
    aData[length++] = sOffsetMeasure.sNTU.Value;
    
    aData[length++] = sOffsetMeasure.sSalinity.Value >> 8;
    aData[length++] = sOffsetMeasure.sSalinity.Value;
    
    aData[length++] = sOffsetMeasure.sTemperature.Value >> 8;
    aData[length++] = sOffsetMeasure.sTemperature.Value;
    
    aData[length++] = sOffsetMeasure.sEC.Value >> 8;
    aData[length++] = sOffsetMeasure.sEC.Value;
    
    switch(KindOffset)
    {
        case _OFFSET_CLO_DU:
          sOffsetMeasure.sClo_Du = sValueMeasure;
          aData[0] = sOffsetMeasure.sClo_Du.Value >> 8;
          aData[1] = sOffsetMeasure.sClo_Du.Value;
          break;
          
        case _OFFSET_PH:
          sOffsetMeasure.spH_Water = sValueMeasure;
          aData[2] = sOffsetMeasure.spH_Water.Value >> 8;
          aData[3] = sOffsetMeasure.spH_Water.Value;
          break;
          
        case _OFFSET_NTU:
          sOffsetMeasure.sNTU = sValueMeasure;
          aData[4] = sOffsetMeasure.sNTU.Value >> 8;
          aData[5] = sOffsetMeasure.sNTU.Value;
          break;
          
        case _OFFSET_SALINITY:
          sOffsetMeasure.sSalinity = sValueMeasure;
          aData[6] = sOffsetMeasure.sSalinity.Value >> 8;
          aData[7] = sOffsetMeasure.sSalinity.Value;
          break;
          
        case _OFFSET_TEMPERATURE:
          sOffsetMeasure.sTemperature = sValueMeasure;
          aData[8] = sOffsetMeasure.sTemperature.Value >> 8;
          aData[9] = sOffsetMeasure.sTemperature.Value;
          break;
          
        case _OFFSET_EC:
          sOffsetMeasure.sEC = sValueMeasure;
          aData[10] = sOffsetMeasure.sEC.Value >> 8;
          aData[11] = sOffsetMeasure.sEC.Value;
          break;
          
        default:
          break;
    }
    
    Save_Array(ADDR_OFFSET_MEASURE, aData, length);
#endif
}

void Init_OffsetMeasure(void)
{
#ifdef USING_INTERNAL_MEM
    if(*(__IO uint8_t*)(ADDR_OFFSET_MEASURE) != FLASH_BYTE_EMPTY)
    {
        sOffsetMeasure.sClo_Du.Value = *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+2) <<8;
        sOffsetMeasure.sClo_Du.Value |= *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+3);
        
        sOffsetMeasure.spH_Water.Value  = *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+4) <<8;
        sOffsetMeasure.spH_Water.Value |= *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+5);
        
        sOffsetMeasure.sNTU.Value = *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+6) <<8;
        sOffsetMeasure.sNTU.Value |= *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+7);
        
        sOffsetMeasure.sSalinity.Value = *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+8) <<8;
        sOffsetMeasure.sSalinity.Value |= *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+9);
        
        sOffsetMeasure.sTemperature.Value  = *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+10) <<8;
        sOffsetMeasure.sTemperature.Value |= *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+11);
        
        sOffsetMeasure.sEC.Value = *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+12) <<8;
        sOffsetMeasure.sEC.Value |= *(__IO uint8_t*)(ADDR_OFFSET_MEASURE+13);
    }
    
    sOffsetMeasure.sClo_Du.Scale = DEFAULT_SCALE_CLO;
    sOffsetMeasure.sClo_Du_Sampling.Scale = DEFAULT_SCALE_CLO;
    
    sOffsetMeasure.spH_Water.Scale = DEFAULT_SCALE_PH;
    sOffsetMeasure.spH_Water_Sampling.Scale = DEFAULT_SCALE_PH;
    
    sOffsetMeasure.sNTU.Scale = DEFAULT_SCALE_NTU;
    sOffsetMeasure.sNTU_Sampling.Scale = DEFAULT_SCALE_NTU;
    
    sOffsetMeasure.sSalinity.Scale = DEFAULT_SCALE_SALINITY;
    sOffsetMeasure.sSalinity_Sampling.Scale = DEFAULT_SCALE_SALINITY;
    
    sOffsetMeasure.sTemperature.Scale = DEFAULT_SCALE_TEMPERATURE;
    sOffsetMeasure.sTemperature_Sampling.Scale = DEFAULT_SCALE_TEMPERATURE;
    
    sOffsetMeasure.sEC.Scale = DEFAULT_SCALE_EC;
    sOffsetMeasure.sEC_Sampling.Scale = DEFAULT_SCALE_EC;
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
/*======================Hanlde Sampling Sensor====================*/
void quickSort_Sampling(int16_t array_stt[],int16_t array_sampling[], uint8_t left, uint8_t right)
{
//	uint16_t pivot = array_sampling[(left+right)/2];
//	uint16_t low = left;
//    uint16_t high = right;
//	while (low < high)
//    {
//		while (array_sampling[low] < pivot) low++;
//        
//		while (array_sampling[high] > pivot) high--;
//        
//		if (low <= high)
//        {
//			int temp = 0;
//            temp = array_sampling[low];
//			array_sampling[low] = array_sampling[high];
//			array_sampling[high] = temp;
//            
//            temp = array_stt[low];
//			array_stt[low] = array_stt[high];
//			array_stt[high] = temp;
//            
//			low++;
//			high--;
//		}
//	}
//	if (low < right)
//		quickSort_Sampling_pH(array_stt, array_sampling, low, right);
//
//	if (left < high)
//		quickSort_Sampling_pH(array_stt, array_sampling, left, high);
  
/*---------------------- Sap xep noi bot --------------------*/
  
  for(uint8_t i = 0; i < NUMBER_SAMPLING_SS; i++)
  {
    for(uint8_t j = 0; j < NUMBER_SAMPLING_SS - 1; j++)
    {
        if(array_sampling[j] > array_sampling[j + 1])
        {
			int temp = 0;
            temp = array_sampling[j];
			array_sampling[j] = array_sampling[j+1];
			array_sampling[j+1] = temp;
            
            temp = array_stt[j];
			array_stt[j] = array_stt[j+1];
			array_stt[j+1] = temp;
        }
    }
  }
}

void quickSort_Sampling_pH(void)
{
    static uint8_t Handle_Once = 0;

    if(sSensor_pH.State_Connect == _SENSOR_DISCONNECT)
    {
        sDataSensorMeasure.spH_Water.Value = 0;
        Handle_Once = 0;
    }
    else
    {
        if(Handle_Once == 0)
        {
            Handle_Once = 1;
            for(uint8_t i = 0; i< NUMBER_SAMPLING_SS; i++)
            {
              aSampling_STT_pH[i] = i;
              
              if(aSampling_VALUE_pH[i] == 0)
                aSampling_VALUE_pH[i] = sSensor_pH.sPH_Value.Value;
            }
            
            quickSort_Sampling(aSampling_STT_pH, aSampling_VALUE_pH, 0, NUMBER_SAMPLING_SS - 1);
            sDataSensorMeasure.spH_Water.Value = aSampling_VALUE_pH[NUMBER_SAMPLING_SS/2];
        }
        else
        {
            for(uint8_t i = 0; i < NUMBER_SAMPLING_SS; i++)
            {
                if(aSampling_STT_pH[i] == NUMBER_SAMPLING_SS - 1)
                {
                    aSampling_STT_pH[i] = 0;
                    aSampling_VALUE_pH[i] = sSensor_pH.sPH_Value.Value;
                }
                else
                    aSampling_STT_pH[i] = aSampling_STT_pH[i] + 1;
            }

            quickSort_Sampling(aSampling_STT_pH, aSampling_VALUE_pH, 0, NUMBER_SAMPLING_SS - 1);
            sDataSensorMeasure.spH_Water.Value = aSampling_VALUE_pH[NUMBER_SAMPLING_SS/2];
        }
    }
}

void quickSort_Sampling_Clo(void)
{
    static uint8_t Handle_Once = 0;

    if(sSensor_Clo.State_Connect == _SENSOR_DISCONNECT)
    {
        sDataSensorMeasure.sClo_Du.Value = 0;
        Handle_Once = 0;
    }
    else
    {
        if(Handle_Once == 0)
        {
            Handle_Once = 1;
            for(uint8_t i = 0; i< NUMBER_SAMPLING_SS; i++)
            {
              aSampling_STT_Clo[i] = i;
              
              if(aSampling_VALUE_Clo[i] == 0)
                aSampling_VALUE_Clo[i] = sSensor_Clo.sClo_Value.Value;
            }
            
            quickSort_Sampling(aSampling_STT_Clo, aSampling_VALUE_Clo, 0, NUMBER_SAMPLING_SS - 1);
            sDataSensorMeasure.sClo_Du.Value = aSampling_VALUE_Clo[NUMBER_SAMPLING_SS/2];
        }
        else
        {
            for(uint8_t i = 0; i < NUMBER_SAMPLING_SS; i++)
            {
                if(aSampling_STT_Clo[i] == NUMBER_SAMPLING_SS - 1)
                {
                    aSampling_STT_Clo[i] = 0;
                    aSampling_VALUE_Clo[i] = sSensor_Clo.sClo_Value.Value;
                }
                else
                    aSampling_STT_Clo[i] = aSampling_STT_Clo[i] + 1;
            }

            quickSort_Sampling(aSampling_STT_Clo, aSampling_VALUE_Clo, 0, NUMBER_SAMPLING_SS - 1);
            sDataSensorMeasure.sClo_Du.Value = aSampling_VALUE_Clo[NUMBER_SAMPLING_SS/2];
        }
    }
}

void quickSort_Sampling_EC(void)
{
    static uint8_t Handle_Once = 0;

    if(sSensor_EC.State_Connect == _SENSOR_DISCONNECT)
    {
        sDataSensorMeasure.sEC.Value = 0;
        Handle_Once = 0;
    }
    else
    {
        if(Handle_Once == 0)
        {
            Handle_Once = 1;
            for(uint8_t i = 0; i< NUMBER_SAMPLING_SS; i++)
            {
              aSampling_STT_EC[i] = i;
              
              if(aSampling_VALUE_EC[i] == 0)
                aSampling_VALUE_EC[i] = sSensor_EC.sConductivity_Value.Value;
            }
            
            quickSort_Sampling(aSampling_STT_EC, aSampling_VALUE_EC, 0, NUMBER_SAMPLING_SS - 1);
            sDataSensorMeasure.sEC.Value = aSampling_VALUE_EC[NUMBER_SAMPLING_SS/2];
        }
        else
        {
            for(uint8_t i = 0; i < NUMBER_SAMPLING_SS; i++)
            {
                if(aSampling_STT_EC[i] == NUMBER_SAMPLING_SS - 1)
                {
                    aSampling_STT_EC[i] = 0;
                    aSampling_VALUE_EC[i] = sSensor_EC.sConductivity_Value.Value;
                }
                else
                    aSampling_STT_EC[i] = aSampling_STT_EC[i] + 1;
            }

            quickSort_Sampling(aSampling_STT_EC, aSampling_VALUE_EC, 0, NUMBER_SAMPLING_SS - 1);
            sDataSensorMeasure.sEC.Value = aSampling_VALUE_EC[NUMBER_SAMPLING_SS/2];
        }
    }
}

void quickSort_Sampling_Turbidity(void)
{
    static uint8_t Handle_Once = 0;

    if(sSensor_Turbidity.State_Connect == _SENSOR_DISCONNECT)
    {
        sDataSensorMeasure.sNTU.Value = 0;
        Handle_Once = 0;
    }
    else
    {
        if(Handle_Once == 0)
        {
            Handle_Once = 1;
            for(uint8_t i = 0; i< NUMBER_SAMPLING_SS; i++)
            {
              aSampling_STT_Turbidity[i] = i;
              
              if(aSampling_VALUE_Turbidity[i] == 0)
                aSampling_VALUE_Turbidity[i] = sSensor_Turbidity.sNTU_Value.Value;
            }
            
            quickSort_Sampling(aSampling_STT_Turbidity, aSampling_VALUE_Turbidity, 0, NUMBER_SAMPLING_SS - 1);
            sDataSensorMeasure.sNTU.Value = aSampling_VALUE_Turbidity[NUMBER_SAMPLING_SS/2];
        }
        else
        {
            for(uint8_t i = 0; i < NUMBER_SAMPLING_SS; i++)
            {
                if(aSampling_STT_Turbidity[i] == NUMBER_SAMPLING_SS - 1)
                {
                    aSampling_STT_Turbidity[i] = 0;
                    aSampling_VALUE_Turbidity[i] = sSensor_Turbidity.sNTU_Value.Value;
                }
                else
                    aSampling_STT_Turbidity[i] = aSampling_STT_Turbidity[i] + 1;
            }

            quickSort_Sampling(aSampling_STT_Turbidity, aSampling_VALUE_Turbidity, 0, NUMBER_SAMPLING_SS - 1);
            sDataSensorMeasure.sNTU.Value = aSampling_VALUE_Turbidity[NUMBER_SAMPLING_SS/2];
        }
    }
}
/*==========================Filter Measure=========================*/
int32_t Filter_Turbidity(int16_t var, uint8_t scale)
{
    //Kalman Filter
    static float x_est = 0.0;   // Uoc luong ban dau
    static float P = 1.0;       // Hiep phuong sai ban dau
    
    static float x_est_last = 0;
  
    float Q = 0.0000001;  // Nhieu mo hinh
    float R = 0.0001;   // Nhieu cam bien

    float x_pred, P_pred, K;
    
    float varFloat = 0;
    int32_t varInt32 = 0;
    
    if(var != 0)
    {
        varFloat = Handle_int32_To_Float_Scale(var, scale);
        
        //Thay doi nhanh du lieu
        if(x_est_last - varFloat > 0.2 || varFloat - x_est_last > 0.2)
        {
           Q *=1000; 
        }
        
        // Buoc du doan
        x_pred = x_est;
        P_pred = P + Q;

        // Tinh he so kalman
        K = P_pred / (P_pred + R);

        // Cap nhat gia tri
        x_est = x_pred + K * (varFloat - x_pred);
        P = (1 - K) * P_pred;
        
        varInt32 = Hanlde_Float_To_Int32_Scale_Round(x_est, scale);
    }
    else
    {
        P = 1;
        x_est = 0;
    }
    x_est_last = x_est;
    return varInt32;
}

int32_t Filter_CloDu(int16_t var, uint8_t scale)
{
    //Kalman_Filter
    static float x_est = 0.0;   // Uoc luong ban dau
    static float P = 1.0;       // Hiep phuong sai ban dau
    
    static float x_est_last = 0;
  
    float Q = 0.0000001;  // Nhieu mo hinh
    float R = 0.0001;   // Nhieu cam bien

    float x_pred, P_pred, K;
    
    float varFloat = 0;
    int32_t varInt32 = 0;
    
    if(var != 0)
    {
        varFloat = Handle_int32_To_Float_Scale(var, scale);
        
        //Thay doi nhanh du lieu
        if(x_est_last - varFloat > 0.2 || varFloat - x_est_last > 0.2)
        {
           Q *=1000; 
        }
        
        // Buoc du doan
        x_pred = x_est;
        P_pred = P + Q;

        // Tinh he so kalman
        K = P_pred / (P_pred + R);

        // Cap nhat gia tri
        x_est = x_pred + K * (varFloat - x_pred);
        P = (1 - K) * P_pred;
        
        varInt32 = Hanlde_Float_To_Int32_Scale_Round(x_est, scale);
    }
    else
    {
        P = 1;
        x_est = 0;
    }
    x_est_last = x_est;
    return varInt32;
}

int32_t Filter_pH(int16_t var, uint8_t scale)
{
    //Kalman Filter
    static float x_est = 0.0;   // Uoc luong ban dau
    static float P = 1.0;       // Hiep phuong sai ban dau
    
    static float x_est_last = 0;
  
    float Q = 0.0000001;  // Nhieu mo hinh
    float R = 0.0001;   // Nhieu cam bien

    float x_pred, P_pred, K;
    
    float varFloat = 0;
    int32_t varInt32 = 0;
    
    if(var != 0)
    {
        varFloat = Handle_int32_To_Float_Scale(var, scale);
        
        //Thay doi nhanh du lieu
        if(x_est_last - varFloat > 0.2 || varFloat - x_est_last > 0.2)
        {
           Q *=1000; 
        }
        
        // Buoc du doan
        x_pred = x_est;
        P_pred = P + Q;

        // Tinh he so kalman
        K = P_pred / (P_pred + R);

        // Cap nhat gia tri
        x_est = x_pred + K * (varFloat - x_pred);
        P = (1 - K) * P_pred;
        
        varInt32 = Hanlde_Float_To_Int32_Scale_Round(x_est, scale);
    }
    else
    {
        P = 1;
        x_est = 0;
    }
    x_est_last = x_est;
    return varInt32;
}

int32_t Filter_Salinity(int16_t var, uint8_t scale)
{
    return var;
}

int32_t Filter_Temperature(int16_t var, uint8_t scale)
{
    return var;
}

int32_t Filter_EC(int16_t var, uint8_t scale)
{
    return var;
}
/*=====================Handle Task and Init app===================*/
void Init_AppSensor(void)
{
    Init_TimeWarningSensor();
    Init_OffsetMeasure();
    Init_Chlorine_Calib();
    Init_Chlorine_PointCalib_1();
    Init_Const_Temp_Compensation_Chlorine();
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
    
    CheckList_AT_CONFIG[_GET_CLO_CONST_TEMP].CallBack = AT_CMD_Get_Clo_Const_Temp;
    CheckList_AT_CONFIG[_SET_CLO_CONST_TEMP].CallBack = AT_CMD_Set_Clo_Const_Temp;
    
    CheckList_AT_CONFIG[_GET_CLO_ZERO_SLOPE].CallBack = AT_CMD_Get_Clo_Zero_Slope;
    CheckList_AT_CONFIG[_SET_CLO_ZERO_SLOPE].CallBack = AT_CMD_Set_Clo_Zero_Slope;
    
    CheckList_AT_CONFIG[_GET_CLO_CALIB_POINT].CallBack = AT_CMD_Get_Clo_Calib_Point;
    CheckList_AT_CONFIG[_SET_CLO_CALIB_POINT].CallBack = AT_CMD_Set_Clo_Calib_Point;
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
/*=======================Convert ADC Chlorine====================*/
void Save_Const_Temp_Compensation_Chlorine(uint16_t value)
{
#ifdef USING_INTERNAL_MEM
    uint8_t aData[8] = {0};
    uint8_t length = 0;
    
    sConvertChlorine.sConst_Compensation_Temp.Value = value;
    
    aData[length++] = sConvertChlorine.sConst_Compensation_Temp.Value >> 8;
    aData[length++] = sConvertChlorine.sConst_Compensation_Temp.Value;

    Save_Array(ADDR_CONST_PH_COMPENSATION, aData, length);
#endif
}

void Init_Const_Temp_Compensation_Chlorine(void)
{
#ifdef USING_INTERNAL_MEM
    if(*(__IO uint8_t*)(ADDR_CONST_PH_COMPENSATION) != FLASH_BYTE_EMPTY)
    {
        sConvertChlorine.sConst_Compensation_Temp.Value  = *(__IO uint8_t*)(ADDR_CONST_PH_COMPENSATION+2) <<8;
        sConvertChlorine.sConst_Compensation_Temp.Value |= *(__IO uint8_t*)(ADDR_CONST_PH_COMPENSATION+3);
    }
    else
    {
        sConvertChlorine.sConst_Compensation_Temp.Value = 220;
    }
#endif    
    sConvertChlorine.sConst_Compensation_Temp.Scale = 0xFC;
}

void Save_Chlorine_Calib(uint16_t    ADC_Zero,
                         uint16_t    ADC_SLope,
                         int16_t     Clo_Calib_Slope,
                         int16_t     Temp_Calib_Slope,
                         int16_t     Ph_Calib_Slope)
{
#ifdef USING_INTERNAL_MEM
    uint8_t aData[20] = {0};
    uint8_t length = 0;

    aData[length++] = ADC_Zero >> 8;
    aData[length++] = ADC_Zero;
    
    aData[length++] = ADC_SLope >> 8;
    aData[length++] = ADC_SLope;
    
    aData[length++] = Clo_Calib_Slope >> 8;
    aData[length++] = Clo_Calib_Slope;
    
    aData[length++] = Temp_Calib_Slope >> 8;
    aData[length++] = Temp_Calib_Slope;
    
    aData[length++] = Ph_Calib_Slope >> 8;
    aData[length++] = Ph_Calib_Slope;
    
    sConvertChlorine.ADC_Zero = ADC_Zero;
    sConvertChlorine.ADC_SLope = ADC_SLope;
    sConvertChlorine.Clo_Calib_Slope = Clo_Calib_Slope;
    sConvertChlorine.Temp_Calib_Slope = Temp_Calib_Slope;
    sConvertChlorine.Ph_Calib_Slope = Ph_Calib_Slope;

    Save_Array(ADDR_CALIB_CHLORINE, aData, length);
#endif
}

void Init_Chlorine_Calib(void)
{
#ifdef USING_INTERNAL_MEM
    if(*(__IO uint8_t*)(ADDR_CALIB_CHLORINE) != FLASH_BYTE_EMPTY)
    {
        sConvertChlorine.ADC_Zero = *(__IO uint8_t*)(ADDR_CALIB_CHLORINE+2) <<8;
        sConvertChlorine.ADC_Zero |= *(__IO uint8_t*)(ADDR_CALIB_CHLORINE+3);
        
        sConvertChlorine.ADC_SLope  = *(__IO uint8_t*)(ADDR_CALIB_CHLORINE+4) <<8;
        sConvertChlorine.ADC_SLope |= *(__IO uint8_t*)(ADDR_CALIB_CHLORINE+5);
        
        sConvertChlorine.Clo_Calib_Slope = *(__IO uint8_t*)(ADDR_CALIB_CHLORINE+6) <<8;
        sConvertChlorine.Clo_Calib_Slope |= *(__IO uint8_t*)(ADDR_CALIB_CHLORINE+7);
        
        sConvertChlorine.Temp_Calib_Slope = *(__IO uint8_t*)(ADDR_CALIB_CHLORINE+8) <<8;
        sConvertChlorine.Temp_Calib_Slope |= *(__IO uint8_t*)(ADDR_CALIB_CHLORINE+9);
        
        sConvertChlorine.Ph_Calib_Slope  = *(__IO uint8_t*)(ADDR_CALIB_CHLORINE+10) <<8;
        sConvertChlorine.Ph_Calib_Slope |= *(__IO uint8_t*)(ADDR_CALIB_CHLORINE+11);
    }
    else
    {
        sConvertChlorine.ADC_Zero = 400;
        sConvertChlorine.ADC_SLope = 1185;
        sConvertChlorine.Clo_Calib_Slope = 114;
        sConvertChlorine.Temp_Calib_Slope = 2794;
        sConvertChlorine.Ph_Calib_Slope = 745;
    }
#endif    
}

void Save_Chlorine_PointCalib(uint16_t    ADC_Point_1,
                              int16_t     Clo_Point_1,
                              int16_t     Temp_Point_1,
                              int16_t     Ph_Point_1,
                              uint16_t    ADC_Point_2,
                              int16_t     Clo_Point_2,
                              int16_t     Temp_Point_2,
                              int16_t     Ph_Point_2)
{
#ifdef USING_INTERNAL_MEM
    uint8_t aData[30] = {0};
    uint8_t length = 0;

    aData[length++] = ADC_Point_1 >> 8;
    aData[length++] = ADC_Point_1;
    
    aData[length++] = Clo_Point_1 >> 8;
    aData[length++] = Clo_Point_1;
    
    aData[length++] = Temp_Point_1 >> 8;
    aData[length++] = Temp_Point_1;
    
    aData[length++] = Ph_Point_1 >> 8;
    aData[length++] = Ph_Point_1;
    
    aData[length++] = ADC_Point_2 >> 8;
    aData[length++] = ADC_Point_2;
    
    aData[length++] = Clo_Point_2 >> 8;
    aData[length++] = Clo_Point_2;
    
    aData[length++] = Temp_Point_2 >> 8;
    aData[length++] = Temp_Point_2;
    
    aData[length++] = Ph_Point_2 >> 8;
    aData[length++] = Ph_Point_2;
    
    sConvertChlorine.ADC_CalibPoint_1 = ADC_Point_1;
    sConvertChlorine.Clo_CalibPoint_1 = Clo_Point_1;
    sConvertChlorine.Temp_CalibPoint_1 = Temp_Point_1;
    sConvertChlorine.Ph_CalibPoint_1 = Ph_Point_1;
    
    sConvertChlorine.ADC_CalibPoint_2 = ADC_Point_2;
    sConvertChlorine.Clo_CalibPoint_2 = Clo_Point_2;
    sConvertChlorine.Temp_CalibPoint_2 = Temp_Point_2;
    sConvertChlorine.Ph_CalibPoint_2 = Ph_Point_2;

    Save_Array(ADDR_CALIBPINT_CHLORINE, aData, length);
#endif
}

void Init_Chlorine_PointCalib_1(void)
{
    sConvertChlorine.Scale = 0xFE;
#ifdef USING_INTERNAL_MEM
    if(*(__IO uint8_t*)(ADDR_CALIBPINT_CHLORINE) != FLASH_BYTE_EMPTY)
    {
        sConvertChlorine.ADC_CalibPoint_1 = *(__IO uint8_t*)(ADDR_CALIBPINT_CHLORINE+2) <<8;
        sConvertChlorine.ADC_CalibPoint_1 |= *(__IO uint8_t*)(ADDR_CALIBPINT_CHLORINE+3);
        
        sConvertChlorine.Clo_CalibPoint_1  = *(__IO uint8_t*)(ADDR_CALIBPINT_CHLORINE+4) <<8;
        sConvertChlorine.Clo_CalibPoint_1 |= *(__IO uint8_t*)(ADDR_CALIBPINT_CHLORINE+5);
        
        sConvertChlorine.Temp_CalibPoint_1 = *(__IO uint8_t*)(ADDR_CALIBPINT_CHLORINE+6) <<8;
        sConvertChlorine.Temp_CalibPoint_1 |= *(__IO uint8_t*)(ADDR_CALIBPINT_CHLORINE+7);
        
        sConvertChlorine.Ph_CalibPoint_1 = *(__IO uint8_t*)(ADDR_CALIBPINT_CHLORINE+8) <<8;
        sConvertChlorine.Ph_CalibPoint_1 |= *(__IO uint8_t*)(ADDR_CALIBPINT_CHLORINE+9);
        
        sConvertChlorine.ADC_CalibPoint_2 = *(__IO uint8_t*)(ADDR_CALIBPINT_CHLORINE+10) <<8;
        sConvertChlorine.ADC_CalibPoint_2 |= *(__IO uint8_t*)(ADDR_CALIBPINT_CHLORINE+11);
        
        sConvertChlorine.Clo_CalibPoint_2  = *(__IO uint8_t*)(ADDR_CALIBPINT_CHLORINE+12) <<8;
        sConvertChlorine.Clo_CalibPoint_2 |= *(__IO uint8_t*)(ADDR_CALIBPINT_CHLORINE+13);
        
        sConvertChlorine.Temp_CalibPoint_2 = *(__IO uint8_t*)(ADDR_CALIBPINT_CHLORINE+14) <<8;
        sConvertChlorine.Temp_CalibPoint_2 |= *(__IO uint8_t*)(ADDR_CALIBPINT_CHLORINE+15);
        
        sConvertChlorine.Ph_CalibPoint_2 = *(__IO uint8_t*)(ADDR_CALIBPINT_CHLORINE+16) <<8;
        sConvertChlorine.Ph_CalibPoint_2 |= *(__IO uint8_t*)(ADDR_CALIBPINT_CHLORINE+17);
    }
    else
    {
        sConvertChlorine.ADC_CalibPoint_1 = 1323;
        sConvertChlorine.Clo_CalibPoint_1 = 114;
        sConvertChlorine.Temp_CalibPoint_1 = 2749;
        sConvertChlorine.Ph_CalibPoint_1 = 698;
        
        sConvertChlorine.ADC_CalibPoint_2 = 1128;
        sConvertChlorine.Clo_CalibPoint_2 = 112;
        sConvertChlorine.Temp_CalibPoint_2 = 2856;
        sConvertChlorine.Ph_CalibPoint_2 = 790;
    }
#endif    
}

//float beta_const = 0;
//
//float compute_clo_du(uint16_t adc, int16_t pH, int16_t temp_C) 
//{
////    float pKa = 0;
////    float pKa_ref = 0;
//    float clo_bu_pH = 0;
//    float clo_final = 0;
//    float clo_tho = 0;
//  
//    // H?ng s? hi?u chu?n
//    uint16_t ADC_Zero = 0;
//    uint16_t ADC_Slope = 0;
//    float alpha = 0.025f; // h? s? nhi?t d?
//    float T_ref = 0.0f;  // nhi?t d? hi?u chu?n
//    float pH_ref = 0.0f;  // pH hi?u chu?n
//    float clo_ref = 0.0f;   //Gia tri clo 
//    float pH_var = 0;
//    float temp_var = 0;
//
////    float ADC_f = 0;
//
//    
//    T_ref = Handle_int32_To_Float_Scale(sConvertChlorine.Temp_Calib_Slope, 0xFE);
//    pH_ref = Handle_int32_To_Float_Scale(sConvertChlorine.Ph_Calib_Slope, 0xFE);
//    clo_ref = Handle_int32_To_Float_Scale(sConvertChlorine.Clo_Calib_Slope, 0xFE);
//    
//    ADC_Zero = sConvertChlorine.ADC_Zero;
//    ADC_Slope = sConvertChlorine.ADC_SLope;
//
//    pH_var = Handle_int32_To_Float_Scale(pH, 0xFE);
//    temp_var = Handle_int32_To_Float_Scale(temp_C, 0xFE); 
//    
////    ADC_f = Handle_int32_To_Float_Scale(adc, 0xFE); 
//    
//    beta_const = Handle_int32_To_Float_Scale(sConvertChlorine.sConst_Compensation_Temp.Value, sConvertChlorine.sConst_Compensation_Temp.Scale);
//
//    // Tinh gia tri mg/L tho tu ADC
//    clo_tho = ((float)(adc - ADC_Zero) / (float)(ADC_Slope - ADC_Zero))*clo_ref;
//
//    // Tính ti le HOCl tai diem do
////     pKa = 7.54f - 0.01f * (temp_var - 25.0f);
////     f_do = 1.0f / (1.0f + pow(10.0f, pH_var - pKa));
//
//    // Tính ti le HOCl tai diem hieu chuan (pH = 7.0, T = 28°C)
////     pKa_ref = 7.54f - 0.01f * (T_ref - 25.0f);
////     f_ref = 1.0f / (1.0f + powf(10.0f, pH_ref - pKa_ref));
//
//    // Bu pH
//     clo_bu_pH = clo_tho * (powf(pH_var, beta_const*(pH_ref - pH_var)));
//
//    // Bu nhiet do
//     clo_final = clo_bu_pH / (1.0f + alpha * (temp_var - T_ref));
//
//    return clo_final;
//}

    
float Chlorine_Compensation_pH(uint16_t adc, int16_t pH, int16_t temp_C) 
{
    float Result = 0;

    float alpha = 0.001f; // he so nhiet do
    float Const_Compensation_Point = 0;
    float Const_Compensation_Clo = 0;
    float temp_var = 0;
    float pH_var = 0;
    float Clo_Point = 0;
    float Clo_Var = 0;
    
    uint16_t ADC_Zero = 0;
    
    uint16_t ADC_Slope_u = 0;
    float Clo_Slope_f = 0;
    float Temp_Slope_f = 0;
    float pH_Slope_f = 0;
    
    uint16_t ADC_Point_u = 0;
    float Clo_Point_f = 0;
    float Temp_Point_f = 0;
    float pH_Point_f = 0;
    
    uint16_t stamp_ADC_Slope_u = 0;
    int16_t stamp_Clo_Slope_f = 0;
    int16_t stamp_Temp_Slope_f = 0;
    int16_t stamp_pH_Slope_f = 0;
    
    uint16_t stamp_ADC_Point_u = 0;
    int16_t stamp_Clo_Point_f = 0;
    int16_t stamp_Temp_Point_f = 0;
    int16_t stamp_pH_Point_f = 0;
    
    if(pH==0)
      pH = 700;
    
    //Sap xep diem calib
    uint8_t  array_stt[3]={0};
    int16_t  array_pH[3]={0};
    
    array_stt[0] = 1;
    array_pH[0] = sConvertChlorine.Ph_Calib_Slope;
    
    array_stt[1] = 2;
    array_pH[1] = sConvertChlorine.Ph_CalibPoint_1;
    
    array_stt[2] = 3;
    array_pH[2] = sConvertChlorine.Ph_CalibPoint_2;
    
    for(uint8_t i = 0; i < 3; i++)
    {
        for(uint8_t j = 0; j < 3 - 1; j++)
        {
            if(array_pH[j] > array_pH[j + 1])
            {
                int16_t temp = 0;
                temp = array_pH[j];
                array_pH[j] = array_pH[j+1];
                array_pH[j+1] = temp;
                
                temp = array_stt[j];
                array_stt[j] = array_stt[j+1];
                array_stt[j+1] = temp;
            }
        }
    }
    
    //Chon Slope vs Point pH
    uint8_t arr_Slope = 2;
    uint8_t arr_Point = 1;
    if(pH < array_pH[1])
    {
        if(array_pH[1] - pH < pH -  array_pH[0])
        {
            arr_Slope = array_stt[1];
            arr_Point = array_stt[0];
        }
        else
        {
            arr_Slope = array_stt[0];
            arr_Point = array_stt[1];
        }
    }
    else
    {
        if(pH - array_pH[1] < array_pH[2] - pH)
        {
            arr_Slope = array_stt[1];
            arr_Point = array_stt[2];
        }
        else
        {
            arr_Slope = array_stt[2];
            arr_Point = array_stt[1];
        }
    }
    
    switch(arr_Slope)
    {
        case 1:
            stamp_ADC_Slope_u = sConvertChlorine.ADC_SLope;
            stamp_Clo_Slope_f = sConvertChlorine.Clo_Calib_Slope;
            stamp_Temp_Slope_f = sConvertChlorine.Temp_Calib_Slope;
            stamp_pH_Slope_f = sConvertChlorine.Ph_Calib_Slope;
          break;
          
        case 2:
            stamp_ADC_Slope_u = sConvertChlorine.ADC_CalibPoint_1;
            stamp_Clo_Slope_f = sConvertChlorine.Clo_CalibPoint_1;
            stamp_Temp_Slope_f = sConvertChlorine.Temp_CalibPoint_1;
            stamp_pH_Slope_f = sConvertChlorine.Ph_CalibPoint_1;
          break;
          
        case 3:
            stamp_ADC_Slope_u = sConvertChlorine.ADC_CalibPoint_2;
            stamp_Clo_Slope_f = sConvertChlorine.Clo_CalibPoint_2;
            stamp_Temp_Slope_f = sConvertChlorine.Temp_CalibPoint_2;
            stamp_pH_Slope_f = sConvertChlorine.Ph_CalibPoint_2;
          break;
          
        default:
          break;
    }
    
    switch(arr_Point)
    {
        case 1:
            stamp_ADC_Point_u = sConvertChlorine.ADC_SLope;
            stamp_Clo_Point_f = sConvertChlorine.Clo_Calib_Slope;
            stamp_Temp_Point_f = sConvertChlorine.Temp_Calib_Slope;
            stamp_pH_Point_f = sConvertChlorine.Ph_Calib_Slope;
          break;
          
        case 2:
            stamp_ADC_Point_u = sConvertChlorine.ADC_CalibPoint_1;
            stamp_Clo_Point_f = sConvertChlorine.Clo_CalibPoint_1;
            stamp_Temp_Point_f = sConvertChlorine.Temp_CalibPoint_1;
            stamp_pH_Point_f = sConvertChlorine.Ph_CalibPoint_1;
          break;
          
        case 3:
            stamp_ADC_Point_u = sConvertChlorine.ADC_CalibPoint_2;
            stamp_Clo_Point_f = sConvertChlorine.Clo_CalibPoint_2;
            stamp_Temp_Point_f = sConvertChlorine.Temp_CalibPoint_2;
            stamp_pH_Point_f = sConvertChlorine.Ph_CalibPoint_2;
          break;
          
        default:
          break;
    }

    // Tinh he so a và b cua phuong trinh y = a*pH + b
    float a = 0;
    float b = 0;
    
    alpha = Handle_int32_To_Float_Scale(sConvertChlorine.sConst_Compensation_Temp.Value, sConvertChlorine.sConst_Compensation_Temp.Scale); 
    temp_var = Handle_int32_To_Float_Scale(temp_C, 0xFE); 
    pH_var = Handle_int32_To_Float_Scale(pH, 0xFE);
    
    ADC_Zero = sConvertChlorine.ADC_Zero;
    
    ADC_Slope_u = stamp_ADC_Slope_u;
    Clo_Slope_f = Handle_int32_To_Float_Scale(stamp_Clo_Slope_f, 0xFE);
    Temp_Slope_f = Handle_int32_To_Float_Scale(stamp_Temp_Slope_f, 0xFE);
    pH_Slope_f = Handle_int32_To_Float_Scale(stamp_pH_Slope_f, 0xFE);
    
    ADC_Point_u = stamp_ADC_Point_u;
    Clo_Point_f = Handle_int32_To_Float_Scale(stamp_Clo_Point_f, 0xFE);
    Temp_Point_f = Handle_int32_To_Float_Scale(stamp_Temp_Point_f, 0xFE);
    pH_Point_f = Handle_int32_To_Float_Scale(stamp_pH_Point_f, 0xFE);
    
    //Tinh gia tri Clo Point
    Clo_Point = ((float)(ADC_Point_u - ADC_Zero) / (float)(ADC_Slope_u - ADC_Zero))*Clo_Slope_f;
    // Bu nhiet do
    if(sConvertChlorine.sConst_Compensation_Temp.Value % 10 == 0)
        Clo_Point = Clo_Point * (1.0f + alpha * (Temp_Point_f - Temp_Slope_f));
    else if(sConvertChlorine.sConst_Compensation_Temp.Value % 10 == 1)
        Clo_Point = Clo_Point * (1.0f - alpha * (Temp_Point_f - Temp_Slope_f));
    else if(sConvertChlorine.sConst_Compensation_Temp.Value % 10 == 2)
        Clo_Point = Clo_Point / (1.0f - alpha * (Temp_Point_f - Temp_Slope_f));
    else 
        Clo_Point = Clo_Point / (1.0f + alpha * (Temp_Point_f - Temp_Slope_f));
      
    //He so bu pH tai point
    Const_Compensation_Point = Clo_Point_f/Clo_Point;
    
    //Tinh he so bu pH cho  clo tai diem pH measure
    a = (1.0 - Const_Compensation_Point) / (pH_Slope_f - pH_Point_f);
    b = Const_Compensation_Point - a * pH_Point_f;
    Const_Compensation_Clo = a * pH_var + b;
    
    //Tinh Clo du
    Clo_Var = ((float)(adc - ADC_Zero) / (float)(ADC_Slope_u - ADC_Zero))*Clo_Slope_f;
    // Bu nhiet do
    if(sConvertChlorine.sConst_Compensation_Temp.Value % 10 == 0)
        Clo_Var = Clo_Var * (1.0f + alpha * (temp_var - Temp_Slope_f));
    else if(sConvertChlorine.sConst_Compensation_Temp.Value % 10 == 1)
        Clo_Var = Clo_Var * (1.0f - alpha * (temp_var - Temp_Slope_f));
    else if(sConvertChlorine.sConst_Compensation_Temp.Value % 10 == 2)
        Clo_Var = Clo_Var / (1.0f - alpha * (temp_var - Temp_Slope_f));
    else 
        Clo_Var = Clo_Var / (1.0f + alpha * (temp_var - Temp_Slope_f));
    
    //Ket qua
    Result = Const_Compensation_Clo*Clo_Var;
    
    if(adc > 0 && pH > 0 && temp_C >0)
        return Result;
    else 
        return 0;
}


void DCU_Logdata_Calib(uint8_t KindCalib, int32_t Value)
{
    DCU_Log_KindCalib = KindCalib,
    DCU_Log_DataCalib = Value;
    sHandleRs485.State_Wait_Calib = _STATE_CALIB_ENTER;
}

void DCU_Enter_Calib(void)
{
    int16_t Value = 0;
    Value = DCU_Log_DataCalib;
    
    if(sSensor_pH.sPH_Value.Value > 0 && sConvertChlorine.Temp_Calib_Slope>0)
    {
        sHandleRs485.State_Wait_Calib = _STATE_CALIB_DONE;

        switch(DCU_Log_KindCalib)
        {
            case _DCU_CALIB_CLO_ZERO:
              Save_Chlorine_Calib(sConvertChlorine.Measure_AD, 
                                  sConvertChlorine.ADC_SLope, 
                                  sConvertChlorine.Clo_Calib_Slope,
                                  sConvertChlorine.Temp_Calib_Slope,
                                  sConvertChlorine.Ph_Calib_Slope);
              break;
              
            case _DCU_CALIB_CLO_SLOPE:
              Save_Chlorine_Calib(sConvertChlorine.ADC_Zero, 
                                  sConvertChlorine.Measure_AD, 
                                  Value,
                                  sSensor_Clo.sTemperature_Value.Value,
                                  sSensor_pH.sPH_Value.Value);
              break;
              
            case _DCU_CALIB_CLO_POINT1:
              Save_Chlorine_PointCalib(sConvertChlorine.Measure_AD, 
                                       Value, 
                                       sSensor_Clo.sTemperature_Value.Value,
                                       sSensor_pH.sPH_Value.Value,
                                       sConvertChlorine.ADC_CalibPoint_2, 
                                       sConvertChlorine.Clo_CalibPoint_2, 
                                       sConvertChlorine.Temp_CalibPoint_2, 
                                       sConvertChlorine.Ph_CalibPoint_2);
              break;
              
            case _DCU_CALIB_CLO_POINT2:
              Save_Chlorine_PointCalib(sConvertChlorine.ADC_CalibPoint_1, 
                                       sConvertChlorine.Clo_CalibPoint_1, 
                                       sConvertChlorine.Temp_CalibPoint_1,
                                       sConvertChlorine.Ph_CalibPoint_1,
                                       sConvertChlorine.Measure_AD, 
                                       Value, 
                                       sSensor_Clo.sTemperature_Value.Value,
                                       sSensor_pH.sPH_Value.Value);
              break;
              
            case _DCU_CALIB_CLO_CONST_TEMP:
              Save_Const_Temp_Compensation_Chlorine(Value);
              break;
              
            default:
              break;
        }
    }
    else
      sHandleRs485.State_Wait_Calib = _STATE_CALIB_ERROR;
}
