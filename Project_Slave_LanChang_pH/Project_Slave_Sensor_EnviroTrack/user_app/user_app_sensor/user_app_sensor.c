#include "user_app_sensor.h"
#include "user_define.h"
#include "user_convert_variable.h"
#include "user_app_modbus_rtu.h"
#include "iwdg.h"
#include "user_connect_sensor.h"
#include "user_modbus_rtu.h"

/*=========================Fucntion Static=========================*/
static uint8_t fevent_sensor_entry(uint8_t event);
static uint8_t fevent_sensor_transmit(uint8_t event);
static uint8_t fevent_sensor_receive_handle(uint8_t event);
static uint8_t fevent_sensor_receive_complete(uint8_t event);
static uint8_t fevent_sensor_wait_calib(uint8_t event);

static uint8_t fevent_detect_connect(uint8_t event);
static uint8_t fevent_temp_alarm(uint8_t event);
static uint8_t fevent_refresh_iwdg(uint8_t event);
/*==============================Struct=============================*/
sEvent_struct               sEventAppSensor[]=
{
  {_EVENT_SENSOR_ENTRY,              1, 5, 2,                fevent_sensor_entry},            //Doi slave khoi dong moi truyen opera
  
  {_EVENT_SENSOR_TRANSMIT,           0, 0, 1500,             fevent_sensor_transmit},
  {_EVENT_SENSOR_RECEIVE_HANDLE,     0, 5, 5,                fevent_sensor_receive_handle},
  {_EVENT_SENSOR_RECEIVE_COMPLETE,   0, 5, 500,              fevent_sensor_receive_complete},
  
  {_EVENT_SENSOR_WAIT_CALIB,         0, 5, 5000,             fevent_sensor_wait_calib},
  
  {_EVENT_DETECT_CONNECT,            1, 5, 15000,            fevent_detect_connect},
  {_EVENT_TEMP_ALARM,                1, 5, 2000,             fevent_temp_alarm},
  {_EVENT_REFRESH_IWDG,              1, 5, 250,              fevent_refresh_iwdg},
};
int32_t aSampling_STT[NUMBER_SAMPLING_SS] = {0};
int32_t aSampling_VALUE[NUMBER_SAMPLING_SS] = {0};
uint16_t CountBufferHandleRecvSS = 0;

Struct_KindMode485  sKindMode485=
{
    .Trans = _RS485_SS_PH_OPERA,
};

uint8_t aDATA_CALIB[4] = {0};
sData   sData_Calib ={aDATA_CALIB , 0};

uint8_t Kind_Trans_Calib = 0;

struct_TempAlarm    sTempAlarm = {0};
Struct_Sensor_pH    sSensor_pH={0};
Struct_Hanlde_RS485 sHandleRs485 = {0};



int16_t aPH_ZERO_CALIB[2] = {700, 686};
int16_t aPH_SLOPE_CALIB[5] = {168, 401, 918, 1010, 1245};

extern sData sUart485SS;
/*========================Function Handle========================*/
static uint8_t fevent_sensor_entry(uint8_t event)
{
    return 1;
}

static uint8_t fevent_sensor_transmit(uint8_t event)
{
/*--------------------Hoi du lieu tu Slave--------------------*/
    uint8_t Frame[20]={0};
    sData sFrame = {&Frame[0], 0};

    Handle_Data_Trans_Sensor(&sFrame, sKindMode485.Trans);
    
    sKindMode485.Recv = sKindMode485.Trans;
    
    //Transmit RS485
    Send_RS458_Sensor(sFrame.Data_a8, sFrame.Length_u16);
   
    switch (sLCD.sScreenNow.Index_u8)
    {
        case _LCD_SCR_SET_CALIB_SS_PH:
          if(sKindMode485.Trans < _RS485_SS_PH_CALIB_READ_POINT_CALIB)
          {
             sKindMode485.Trans++;
          }
          else
          {
            sKindMode485.Trans = _RS485_SS_PH_OPERA;
          }
          break;
          
        default:
              sKindMode485.Trans = _RS485_SS_PH_OPERA;
          break;
    }
    
    
    fevent_active(sEventAppRs485, _EVENT_SENSOR_RECEIVE_HANDLE);
    fevent_enable(sEventAppRs485, _EVENT_SENSOR_RECEIVE_COMPLETE);
    fevent_enable(sEventAppRs485, event);
    return 1;
}

static uint8_t fevent_sensor_receive_handle(uint8_t event)
{
/*-----------------Kiem tra da nhan xong tu 485------------*/
    if(sUart485SS.Length_u16 != 0)
    {
        if(CountBufferHandleRecvSS == sUart485SS.Length_u16)
        {
            CountBufferHandleRecvSS = 0;
            fevent_active(sEventAppRs485, _EVENT_RS485_RECEIVE_COMPLETE);
            return 1;
        }
        else
        {
            CountBufferHandleRecvSS = sUart485SS.Length_u16;
        }
    }
    
    fevent_enable(sEventAppRs485, event);
    return 1;
}

static uint8_t fevent_sensor_receive_complete(uint8_t event)
{
/*------------------Xu ly chuoi nhan duoc----------------*/
    uint16_t Crc_Check = 0;
    uint16_t Crc_Recv  = 0;

    if(sUart485SS.Length_u16 > 2)
    {
        Crc_Recv = (sUart485SS.Data_a8[sUart485SS.Length_u16-1] << 8) |
                   (sUart485SS.Data_a8[sUart485SS.Length_u16-2]);
        Crc_Check = ModRTU_CRC(sUart485SS.Data_a8, sUart485SS.Length_u16 - 2);
        if(Crc_Check == Crc_Recv)
        {
            fevent_enable(sEventAppRs485, _EVENT_RS485_REFRESH);
            
            if(sUart485SS.Data_a8[0] == ID_DEFAULT_SS_PH)
            {
                Handle_State_Sensor(sKindMode485.Recv, _RS485_RESPOND);
                Handle_Data_Recv_Sensor(sUart485SS, sKindMode485.Recv);
            }
            else
            {
                Handle_State_Sensor(sKindMode485.Recv, _RS485_UNRESPOND);
            }
        }
        else
        {
            Handle_State_Sensor(sKindMode485.Recv, _RS485_UNRESPOND);
        } 
    }
    else
    {
        Handle_State_Sensor(sKindMode485.Recv, _RS485_UNRESPOND);
    } 
    
    Handle_Data_Measure(sKindMode485.Recv);

    fevent_disable(sEventAppRs485, _EVENT_SENSOR_RECEIVE_HANDLE);
    return 1;
}

static uint8_t fevent_sensor_wait_calib(uint8_t event)
{
    if(sParaDisplay.State_Setting != _STATE_SETTING_DONE)
    {
        sParaDisplay.State_Setting = _STATE_SETTING_ERROR;
    }
    return 1;
}

static uint8_t fevent_detect_connect(uint8_t event)
{
//    sSensorLevel.LevelValueFilter_f = 0;
    fevent_enable(sEventAppSensor, event);
    return 1;
}

static uint8_t fevent_temp_alarm(uint8_t event)
{
//    if(sTempAlarm.State == 1)
//    {
//        if((sSensorTemp.TempObject_f > sTempAlarm.Alarm_Upper) || 
//           (sSensorTemp.TempObject_f < sTempAlarm.Alarm_Lower))
//        {
//            ALARM_ON;
//        }
//        else
//            ALARM_OFF;
//    }
//    else 
//        ALARM_OFF;
    
    fevent_enable(sEventAppSensor, event);
    return 1;
}

static uint8_t fevent_refresh_iwdg(uint8_t event)
{
    HAL_IWDG_Refresh(&hiwdg);
    fevent_enable(sEventAppSensor, event);
    return 1;
}
/*==================Function Handle Data=================*/
void Handle_Data_Measure(uint8_t KindRecv)
{
    switch(KindRecv)
    {
        case _RS485_SS_PH_OPERA:
            
            sSensor_pH.pH_Value_f = quickSort_Sampling_Value((int32_t)(sSensor_pH.pH_Value_f));
            
            sSensor_pH.pH_Filter_f = Filter_pH(sSensor_pH.pH_Value_f);
            sSensor_pH.temp_Filter_f = Filter_Temp(sSensor_pH.temp_Value_f);
            
            sSensor_pH.temp_Filter_f += sSensor_pH.temp_Offset_f;
            
            if(sSensor_pH.pH_Filter_f + sSensor_pH.pH_Offset_f < 0)
            {
                sSensor_pH.pH_Filter_f = 0;
            }
            else if(sSensor_pH.pH_Filter_f + sSensor_pH.pH_Offset_f > PH_RANGE_MAX)
            {
                sSensor_pH.pH_Filter_f = PH_RANGE_MAX;
            }
            else
            {
                sSensor_pH.pH_Filter_f += sSensor_pH.pH_Offset_f;
            }
          break;
          
        default:
          break;
    }
    
    if(sSensor_pH.State_Connect == _SENSOR_DISCONNECT)
    {
        sSensor_pH.pH_Value_f = 0;
        sSensor_pH.pH_Filter_f = Filter_pH(sSensor_pH.pH_Value_f);
    }
    
    if(sSensor_pH.State_Connect == _SENSOR_DISCONNECT)
    {
        sSensor_pH.temp_Value_f = 0;
        sSensor_pH.pH_Filter_f = Filter_Temp(sSensor_pH.temp_Value_f);;
    }
}

/*====================Filter Data===================*/
void quickSort_Sampling(int32_t array_stt[],int32_t array_sampling[], uint8_t left, uint8_t right)
{
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

float quickSort_Sampling_Value(int32_t Value)
{
    static uint8_t Handle_Once = 0;
    float Result = 0;
    
//    if(Value == 0)
//    {
//        Handle_Once = 0;
//    }
//    else
//    {
        if(Handle_Once == 0)
        {
            Handle_Once = 1;
            for(uint8_t i = 0; i< NUMBER_SAMPLING_SS; i++)
            {
              aSampling_STT[i] = i;
              
              if(aSampling_VALUE[i] == 0)
                aSampling_VALUE[i] = Value;
            }
            
            quickSort_Sampling(aSampling_STT, aSampling_VALUE, 0, NUMBER_SAMPLING_SS - 1);
            Result = aSampling_VALUE[NUMBER_SAMPLING_SS/2];
        }
        else
        {
            for(uint8_t i = 0; i < NUMBER_SAMPLING_SS; i++)
            {
                if(aSampling_STT[i] == NUMBER_SAMPLING_SS - 1)
                {
                    aSampling_STT[i] = 0;
                    aSampling_VALUE[i] = Value;
                }
                else
                    aSampling_STT[i] = aSampling_STT[i] + 1;
            }

            quickSort_Sampling(aSampling_STT, aSampling_VALUE, 0, NUMBER_SAMPLING_SS - 1);
            Result = aSampling_VALUE[NUMBER_SAMPLING_SS/2];
        }
//    }
    return Result;
}

float Filter_pH(float var)
{
    //Kalman Filter
    static float x_est = 0.0;   // Uoc luong ban dau
    static float P = 1.0;       // Hiep phuong sai ban dau
    
    static float x_est_last = 0;
  
    float Q = 0.0000001;  // Nhieu mo hinh
    float R = 0.0001;   // Nhieu cam bien

    float x_pred, P_pred, K;
    
    float varFloat = 0;
//    int32_t varInt32 = 0;
    
    if(var != 0)
    {
//        varFloat = Handle_int32_To_Float_Scale(var, scale);
        varFloat = var;
        
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
        
//        varInt32 = Hanlde_Float_To_Int32_Scale_Round(x_est, scale);
    }
    else
    {
        P = 1;
        x_est = 0;
    }
    x_est_last = x_est;
//    return varInt32;
    return x_est;
}

float Filter_Temp(float var)
{
    return var;
}

/*=====================Handle Sensor=====================*/
void Handle_Data_Trans_Sensor(sData *sFrame, uint8_t KindRecv)
{
    Handle_Data_Trans_SS_pH(sFrame, KindRecv);
}
void Handle_Data_Recv_Sensor(sData sDataRS485, uint8_t KindRecv)
{
    if(sDataRS485.Data_a8[0] == ID_DEFAULT_SS_PH)
        Handle_Data_Recv_SS_pH(sDataRS485, KindRecv);
}

void Handle_State_Sensor(uint8_t KindRecv, uint8_t KindDetect)
{
    Handle_State_SS_pH(KindRecv, KindDetect);
}
/*==================Handle Sensor pH===================*/
void Handle_Data_Trans_SS_pH(sData *sFrame, uint8_t KindTrans)
{
    switch(KindTrans)
    {
        //Trans Opera
        case _RS485_SS_PH_OPERA:
            ModRTU_Master_Read_Frame(sFrame, ID_DEFAULT_SS_PH, 0x03, 0x0001, 0x04);
            break;
            
         //Trans Calib
        case _RS485_SS_PH_CALIB_READ_AD:
            ModRTU_Master_Read_Frame(sFrame, ID_DEFAULT_SS_PH, 0x03, 0x0066, 0x01);
            break;
            
        case _RS485_SS_PH_CALIB_READ_POINT_CALIB:
            ModRTU_Master_Read_Frame(sFrame, ID_DEFAULT_SS_PH, 0x03, 0x0036, 0x04);
            break;
            
        case _RS485_SS_PH_CALIB_ZERO_STD_SOLUTION:
            if(sData_Calib.Length_u16 == 2)
                ModRTU_Master_Write_Frame(sFrame, ID_DEFAULT_SS_PH, 0x06, 0x0036, (sData_Calib.Length_u16/2), sData_Calib.Data_a8);
            break;
            
        case _RS485_SS_PH_CALIB_SLOPE_STD_SOLUTION:
            if(sData_Calib.Length_u16 == 2)
                ModRTU_Master_Write_Frame(sFrame, ID_DEFAULT_SS_PH, 0x06, 0x0038, (sData_Calib.Length_u16/2), sData_Calib.Data_a8);
            break;
            
        case _RS485_SS_PH_CALIB_ZERO:
            if(sData_Calib.Length_u16 == 2)
                ModRTU_Master_Write_Frame(sFrame, ID_DEFAULT_SS_PH, 0x06, 0x003E, (sData_Calib.Length_u16/2), sData_Calib.Data_a8);
            break;
            
        case _RS485_SS_PH_CALIB_SLOPE:
            if(sData_Calib.Length_u16 == 2)
                ModRTU_Master_Write_Frame(sFrame, ID_DEFAULT_SS_PH, 0x06, 0x003F, (sData_Calib.Length_u16/2), sData_Calib.Data_a8);
            break;
      
        default:
          break;
    }
}

void Handle_Data_Recv_SS_pH(sData sDataRS485, uint8_t KindRecv)
{
    uint16_t Pos = 0;
    uint32_t Stamp_Hex = 0;
    switch(KindRecv)
    {
        //Recv Opera
        case _RS485_SS_PH_OPERA:
          Pos = 3;
            
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
//          sSensor_pH.sPH_Value.Value =  Handle_HexFloat_To_Int32_Round(Stamp_Hex, sSensor_pH.sTemperature_Value.Scale);
          Convert_uint32Hex_To_Float(Stamp_Hex, &sSensor_pH.pH_Value_f);
          
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
//          sSensor_pH.sTemperature_Value.Value =  Handle_HexFloat_To_Int32_Round(Stamp_Hex, sSensor_pH.sTemperature_Value.Scale);
          Convert_uint32Hex_To_Float(Stamp_Hex, &sSensor_pH.temp_Value_f);
          break;
          
        //Recv Calib
        case _RS485_SS_PH_CALIB_READ_AD:
          Pos = 3;
          
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 2);
          sSensor_pH.Measure_AD =  Stamp_Hex;
          break;
          
        case _RS485_SS_PH_CALIB_READ_POINT_CALIB:
          Pos = 3;
            
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
          sSensor_pH.sZero_Calib.Value = _fRead_pH_Zero_Calib(Stamp_Hex);
          
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
          sSensor_pH.sSlope_Calib.Value = _fRead_pH_Slope_Calib(Stamp_Hex);
          break;
          
        case _RS485_SS_PH_CALIB_ZERO_STD_SOLUTION:
        case _RS485_SS_PH_CALIB_SLOPE_STD_SOLUTION:
        case _RS485_SS_PH_CALIB_ZERO:
        case _RS485_SS_PH_CALIB_SLOPE:
          RS485_Done_Calib();
          break;
          
        default:
          break;
    }
}

void Handle_State_SS_pH(uint8_t KindRecv, uint8_t KindDetect)
{
    switch(KindRecv)
    {
        case _RS485_SS_PH_OPERA:
        case _RS485_SS_PH_CALIB_READ_AD:
        case _RS485_SS_PH_CALIB_READ_POINT_CALIB:
        case _RS485_SS_PH_CALIB_ZERO_STD_SOLUTION:
        case _RS485_SS_PH_CALIB_SLOPE_STD_SOLUTION:
        case _RS485_SS_PH_CALIB_ZERO:
        case _RS485_SS_PH_CALIB_SLOPE:
          if(KindDetect == _RS485_RESPOND)
          {
            sSensor_pH.Count_Disconnect = 0;
            sSensor_pH.State_Connect = _SENSOR_CONNECT;
            
            if(KindRecv == _RS485_SS_PH_OPERA)
              sHandleRs485.State_Recv_pH = 1;
          }
          else
          {
            if(sSensor_pH.Count_Disconnect < 3)
                sSensor_pH.Count_Disconnect++;
          }
          break;
          
        default:
          break;
    }
    
    if(sSensor_pH.Count_Disconnect == 0)
    {
        
    }
    else if(sSensor_pH.Count_Disconnect >=3)
    {
        sSensor_pH.State_Connect = _SENSOR_DISCONNECT;
//        sSensor_pH.sPH_Value.Value = 0;
        sSensor_pH.pH_Value_f   = 0;
//        sSensor_pH.sTemperature_Value.Value = 0;
        sSensor_pH.temp_Value_f = 0;
        sSensor_pH.sZero_Calib.Value = 0;
        sSensor_pH.sSlope_Calib.Value = 0;
        
        sSensor_pH.Measure_AD = 0;
        sHandleRs485.State_Recv_pH = 0;
    }
}

/*======================Function Calib Sensor=====================*/
void RS485_Done_Calib(void)
{
    sParaDisplay.State_Setting = _STATE_SETTING_DONE;
    fevent_disable(sEventAppSensor, _EVENT_SENSOR_WAIT_CALIB);
}

void RS485_Enter_Calib(void)
{
    sKindMode485.Trans = Kind_Trans_Calib;
    sParaDisplay.State_Setting = _STATE_SETTING_WAIT;
    fevent_enable(sEventAppSensor, _EVENT_SENSOR_WAIT_CALIB);
}

void RS485_LogData_Calib(uint8_t Kind_Send, const void *data, uint16_t size)
{
    uint8_t* dst8= (uint8_t *) data;
    
    Reset_Buff(&sData_Calib);
    switch(size)
    {
        case 2:
          sData_Calib.Data_a8[0] = *(dst8+1);
          sData_Calib.Data_a8[1] = *(dst8);
          sData_Calib.Length_u16 = size;
          break;
          
        case 4:
          sData_Calib.Data_a8[0] = *(dst8+1);
          sData_Calib.Data_a8[1] = *(dst8);
          sData_Calib.Data_a8[2] = *(dst8+3);
          sData_Calib.Data_a8[3] = *(dst8+2);
          sData_Calib.Length_u16 = size;
          break;
          
        default:
          sData_Calib.Length_u16 = 0;
          break;
    }
    Kind_Trans_Calib = Kind_Send;
    sParaDisplay.State_Setting = _STATE_SETTING_ENTER;
}

/*======================Function Handle Data====================*/
int16_t _fSet_pH_Zero_Calib_UpDown(int16_t Value, int8_t Kind)
{
    uint8_t size_data = 2;
    if(Kind == 1)
    {
        for(uint8_t i = 0; i< size_data; i++)
        {
            if(Value == aPH_ZERO_CALIB[i])
            {
              if(i == (size_data - 1))
                return aPH_ZERO_CALIB[i];
              else
                return aPH_ZERO_CALIB[i+1];
            }
        }
        return aPH_ZERO_CALIB[0];
    }
    else if(Kind == -1)
    {
        for(uint8_t i = 0; i< size_data; i++)
        {
            if(Value == aPH_ZERO_CALIB[i])
            {
              if(i == 0)
                return aPH_ZERO_CALIB[0];
              else
                return aPH_ZERO_CALIB[i-1];
            }
        }
        return aPH_ZERO_CALIB[0];
    }
    return aPH_ZERO_CALIB[0];
}

int16_t _fSet_pH_Slope_Calib_UpDown(int16_t Value, int8_t Kind)
{
    uint8_t size_data = 5;
    if(Kind == 1)
    {
        for(uint8_t i = 0; i< size_data; i++)
        {
            if(Value == aPH_SLOPE_CALIB[i])
            {
              if(i == size_data - 1)
                return aPH_SLOPE_CALIB[i];
              else
                return aPH_SLOPE_CALIB[i+1];
            }
        }
        return aPH_SLOPE_CALIB[0];
    }
    else if(Kind == -1)
    {
        for(uint8_t i = 0; i< size_data; i++)
        {
            if(Value == aPH_SLOPE_CALIB[i])
            {
              if(i == 0)
                return aPH_SLOPE_CALIB[0];
              else
                return aPH_SLOPE_CALIB[i-1];
            }
        }
        return aPH_SLOPE_CALIB[0];
    }
    return aPH_SLOPE_CALIB[0];
}

int16_t _fRead_pH_Zero_Point(int16_t Value)
{
    uint8_t size_data = 2;
    for(uint8_t i = 0; i< size_data; i++)
    {
        if(Value == aPH_ZERO_CALIB[i])
          return i;
    }
    return 0;
}

int16_t _fRead_pH_Slope_Point(int16_t Value)
{
    uint8_t size_data = 5;
    for(uint8_t i = 0; i< size_data; i++)
    {
        if(Value == aPH_SLOPE_CALIB[i])
          return i;
    }
    return 0;
}

int16_t _fRead_pH_Zero_Calib(int16_t point)
{
    uint8_t size_data = 2;
    if(point >= 0 && point < size_data)
        return aPH_ZERO_CALIB[point];
    else
        return 0;
}

int16_t _fRead_pH_Slope_Calib(int16_t point)
{
    uint8_t size_data = 5;
    if(point >= 0 && point < size_data)
        return aPH_SLOPE_CALIB[point];
    else
        return 0;
}

uint32_t Read_Register_Rs485(uint8_t aData[], uint16_t *pos, uint8_t LengthData)
{
    uint32_t stamp = 0;
    uint16_t length = *pos;
    if(LengthData == 4)
    {
        stamp = aData[length+2]<<8 | aData[length+3];
        stamp = (stamp << 16) | (aData[length]<<8 | aData[length+1]);
    }
    else if(LengthData == 2)
    {
        stamp = aData[length]<<8 | aData[length+1];
    }
    *pos = *pos + LengthData;
    return stamp;
}
/*===================== Send Data RS485 ======================*/
/*
    @brief Send 485 sensor
*/
void        Send_RS458_Sensor(uint8_t *aData, uint16_t Length_u16) 
{
//    HAL_GPIO_WritePin(RS485_TXDE_S_GPIO_Port, RS485_TXDE_S_Pin, GPIO_PIN_SET);
//    HAL_Delay(5);
//    HAL_UART_Transmit(&uart_485, aData, Length_u16, 1000);
//    
//    UTIL_MEM_set(sUart485.Data_a8 , 0x00, sUart485.Length_u16);
//    sUart485.Length_u16 = 0;
//    CountBufferHandleRecv = 0;
//    
//    HAL_GPIO_WritePin(RS485_TXDE_S_GPIO_Port, RS485_TXDE_S_Pin, GPIO_PIN_RESET);
  

    HAL_GPIO_WritePin(SENSOR_DE_GPIO_PORT, SENSOR_DE_GPIO_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
    // Send
//    RS485_Init_Data();
    HAL_UART_Transmit(&uart_rs485SS, aData , Length_u16, 1000); 
    
    //Dua DE ve Receive
    HAL_GPIO_WritePin(SENSOR_DE_GPIO_PORT, SENSOR_DE_GPIO_PIN, GPIO_PIN_RESET);
}
/*===================Save and Init Calib====================*/
void Save_ParamCalib(float pH_Offset_f, float temp_Offset_f)
{
#ifdef USING_APP_SENSOR
    uint8_t aData[50] = {0};
    uint8_t length = 0;
  
    uint32_t hexUint_Compensation_pH = 0;
    uint32_t hexUint_Compensation_temp = 0;
    
    sSensor_pH.pH_Offset_f = pH_Offset_f;
    sSensor_pH.temp_Offset_f = temp_Offset_f;
    
    hexUint_Compensation_pH = Handle_Float_To_hexUint32(sSensor_pH.pH_Offset_f);
    hexUint_Compensation_temp = Handle_Float_To_hexUint32(sSensor_pH.temp_Offset_f);
    
    aData[length++] = hexUint_Compensation_pH >> 24;
    aData[length++] = hexUint_Compensation_pH >> 16;
    aData[length++] = hexUint_Compensation_pH >> 8;
    aData[length++] = hexUint_Compensation_pH ;
    
    aData[length++] = hexUint_Compensation_temp >> 24;
    aData[length++] = hexUint_Compensation_temp >> 16;
    aData[length++] = hexUint_Compensation_temp >> 8;
    aData[length++] = hexUint_Compensation_temp ;

    Save_Array(ADDR_CALIB_TEMPERATURE, aData, length);
#endif   
}

void Init_ParamCalib(void)
{
#ifdef USING_APP_SENSOR
    uint32_t hexUint_Compensation_pH = 0;
    uint32_t hexUint_Compensation_temp = 0;
  
    if(*(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE) != FLASH_BYTE_EMPTY)
    {
        hexUint_Compensation_pH  = *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+2) << 24;
        hexUint_Compensation_pH  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+3)<< 16;
        hexUint_Compensation_pH  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+4)<< 8;
        hexUint_Compensation_pH  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+5);
        
        hexUint_Compensation_temp  = *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+6) << 24;
        hexUint_Compensation_temp  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+7)<< 16;
        hexUint_Compensation_temp  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+8)<< 8;
        hexUint_Compensation_temp  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+9);
        
        Convert_uint32Hex_To_Float(hexUint_Compensation_pH,  &sSensor_pH.pH_Offset_f);
        Convert_uint32Hex_To_Float(hexUint_Compensation_temp, &sSensor_pH.temp_Offset_f);
    }
    else
    {
        sSensor_pH.pH_Offset_f = 0;
        sSensor_pH.temp_Offset_f = 0;
    }
#endif   
}

void Save_TempAlarm(uint8_t State, float AlarmLower, float AlarmUpper)
{
#ifdef USING_APP_SENSOR
    uint8_t aData[50] = {0};
    uint8_t length = 0;
  
    uint32_t hexUint_AlarmUpper  = 0;
    uint32_t hexUint_AlarmLower  = 0;
    
    sTempAlarm.State = State;
    sTempAlarm.Alarm_Upper = AlarmUpper;
    sTempAlarm.Alarm_Lower = AlarmLower;
    
    hexUint_AlarmUpper  = Handle_Float_To_hexUint32(sTempAlarm.Alarm_Upper);
    hexUint_AlarmLower  = Handle_Float_To_hexUint32(sTempAlarm.Alarm_Lower);
    
    aData[length++] = sTempAlarm.State;
    
    aData[length++] = hexUint_AlarmUpper >> 24;
    aData[length++] = hexUint_AlarmUpper >> 16;
    aData[length++] = hexUint_AlarmUpper >> 8;
    aData[length++] = hexUint_AlarmUpper ;
    
    aData[length++] = hexUint_AlarmLower >> 24;
    aData[length++] = hexUint_AlarmLower >> 16;
    aData[length++] = hexUint_AlarmLower >> 8;
    aData[length++] = hexUint_AlarmLower ;

    Save_Array(ADDR_TEMPERATURE_ALARM, aData, length);
#endif   
}

void Init_TempAlarm(void)
{
#ifdef USING_APP_SENSOR
  
    uint32_t hexUint_AlarmUpper  = 0;
    uint32_t hexUint_AlarmLower  = 0;
  
    if(*(__IO uint8_t*)(ADDR_TEMPERATURE_ALARM) != FLASH_BYTE_EMPTY)
    {
        sTempAlarm.State  = *(__IO uint8_t*)(ADDR_TEMPERATURE_ALARM+2);
      
        hexUint_AlarmUpper  = *(__IO uint8_t*)(ADDR_TEMPERATURE_ALARM+3) << 24;
        hexUint_AlarmUpper  |= *(__IO uint8_t*)(ADDR_TEMPERATURE_ALARM+4)<< 16;
        hexUint_AlarmUpper  |= *(__IO uint8_t*)(ADDR_TEMPERATURE_ALARM+5)<< 8;
        hexUint_AlarmUpper  |= *(__IO uint8_t*)(ADDR_TEMPERATURE_ALARM+6);
        
        hexUint_AlarmLower  = *(__IO uint8_t*)(ADDR_TEMPERATURE_ALARM+7) << 24;
        hexUint_AlarmLower  |= *(__IO uint8_t*)(ADDR_TEMPERATURE_ALARM+8)<< 16;
        hexUint_AlarmLower  |= *(__IO uint8_t*)(ADDR_TEMPERATURE_ALARM+9)<< 8;
        hexUint_AlarmLower  |= *(__IO uint8_t*)(ADDR_TEMPERATURE_ALARM+10);
        
        Convert_uint32Hex_To_Float(hexUint_AlarmUpper, &sTempAlarm.Alarm_Upper);
        Convert_uint32Hex_To_Float(hexUint_AlarmLower, &sTempAlarm.Alarm_Lower);
    }
    else
    {
        sTempAlarm.State = 0;
        sTempAlarm.Alarm_Upper = LEVEL_MIN;
        sTempAlarm.Alarm_Lower = LEVEL_MAX;
    }
#endif   
}
/*==================Handle Define AT command=================*/
#ifdef USING_AT_CONFIG
void AT_CMD_Reset_Slave(sData *str, uint16_t Pos)
{
    uint8_t aTemp[60] = {0};   
    uint16_t length = 0;
    Save_InforSlaveModbusRTU(ID_DEFAULT, BAUDRATE_DEFAULT);
      
    Insert_String_To_String(aTemp, &length, (uint8_t*)"Reset OK!\r\n",0 , 11);
//	Modem_Respond(PortConfig, aTemp, length, 0);
    HAL_UART_Transmit(&uart_debug, aTemp,length, 1000);
}

void AT_CMD_Restore_Slave(sData *str, uint16_t Pos)
{
    uint8_t aTemp[60] = {0};   
    uint16_t length = 0;
    OnchipFlashPageErase(ADDR_CALIB_TEMPERATURE);
//    sSensorLevel.Calib_Offset = 0;
//    sSensorLevel.CalibPoint1_x_f = LEVEL_MIN;
//    sSensorLevel.CalibPoint1_y_f = LEVEL_MIN;
//    sSensorLevel.CalibPoint2_x_f = LEVEL_MAX;
//    sSensorLevel.CalibPoint2_y_f = LEVEL_MAX;
    
    OnchipFlashPageErase(ADDR_TEMPERATURE_ALARM);
    sTempAlarm.State = 0;
    sTempAlarm.Alarm_Upper = LEVEL_MAX;
    sTempAlarm.Alarm_Lower = LEVEL_MIN;
    
    
    Insert_String_To_String(aTemp, &length, (uint8_t*)"Restore OK!\r\n",0 , 13);
//	Modem_Respond(PortConfig, aTemp, length, 0);
    HAL_UART_Transmit(&uart_debug, aTemp,length, 1000);
}

void AT_CMD_Get_ID_Slave (sData *str, uint16_t Pos)
{
    uint8_t aTemp[50] = "ID Slave: ";   //13 ki tu dau tien
    sData StrResp = {&aTemp[0], 12}; 

    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sSlave_ModbusRTU.ID), 0x00);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)"\r\n",0 , 2);

    HAL_UART_Transmit(&uart_debug, StrResp.Data_a8, StrResp.Length_u16, 1000);
}

void AT_CMD_Set_ID_Slave (sData *str_Receiv, uint16_t Pos)
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
        if(TempU32 <= 255 )
        {
            Save_InforSlaveModbusRTU(TempU32, sSlave_ModbusRTU.Baudrate);
            HAL_UART_Transmit(&uart_debug, (uint8_t*)"OK", 2, 1000);
        }
        else
        {
//            Modem_Respond(PortConfig, (uint8_t*)"ERROR", 5, 0);
            HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 1000);
        }
    }
    else
    {
        HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 1000);
    }
}

void AT_CMD_Get_BR_Slave (sData *str, uint16_t Pos)
{
    uint8_t aTemp[50] = "BR Slave: ";   //13 ki tu dau tien
    sData StrResp = {&aTemp[0], 12}; 

    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sSlave_ModbusRTU.Baudrate), 0x00);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)"\r\n",0 , 2);

    HAL_UART_Transmit(&uart_debug, StrResp.Data_a8, StrResp.Length_u16, 1000);
}

void AT_CMD_Set_BR_Slave (sData *str_Receiv, uint16_t Pos)
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
        if(TempU32 <= 11 )
        {
            Save_InforSlaveModbusRTU(sSlave_ModbusRTU.ID, TempU32);
            HAL_UART_Transmit(&uart_debug, (uint8_t*)"OK", 2, 1000);
        }
        else
        {
//            Modem_Respond(PortConfig, (uint8_t*)"ERROR", 5, 0);
            HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 1000);
        }
    }
    else
    {
        HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 1000);
    }
}
#endif

/*==================Handle Task and Init app=================*/
void       Init_AppSensor(void)
{
    Init_ParamCalib();
    Init_TempAlarm();
#ifdef USING_AT_CONFIG
    /* regis cb serial */
    CheckList_AT_CONFIG[_RESET_SLAVE].CallBack = AT_CMD_Reset_Slave;
    CheckList_AT_CONFIG[_RESTORE_SLAVE].CallBack = AT_CMD_Restore_Slave;
    
    CheckList_AT_CONFIG[_GET_ID_SLAVE].CallBack = AT_CMD_Get_ID_Slave;
    CheckList_AT_CONFIG[_SET_ID_SLAVE].CallBack = AT_CMD_Set_ID_Slave;
    CheckList_AT_CONFIG[_GET_BR_SLAVE].CallBack = AT_CMD_Get_BR_Slave;
    CheckList_AT_CONFIG[_SET_BR_SLAVE].CallBack = AT_CMD_Set_BR_Slave;
#endif
    RS485SS_Stop_RX_Mode();
    RS485SS_Init_RX_Mode();
}

uint8_t        AppSensor_Task(void)
{
    uint8_t i = 0;
    uint8_t Result =  false;
    
    for(i = 0; i < _EVENT_SENSOR_END; i++)
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
