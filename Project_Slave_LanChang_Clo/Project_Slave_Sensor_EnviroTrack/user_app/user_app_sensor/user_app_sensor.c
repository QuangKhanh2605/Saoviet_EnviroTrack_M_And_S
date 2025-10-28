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

static uint8_t fevent_detect_ph_recv(uint8_t event);
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
  
  {_EVENT_DETECT_PH_RECV,            1, 5, 60000,            fevent_detect_ph_recv},
  {_EVENT_TEMP_ALARM,                1, 5, 2000,             fevent_temp_alarm},
  {_EVENT_REFRESH_IWDG,              1, 5, 250,              fevent_refresh_iwdg},
};
int32_t aSampling_STT[NUMBER_SAMPLING_SS] = {0};
int32_t aSampling_VALUE[NUMBER_SAMPLING_SS] = {0};
uint16_t CountBufferHandleRecvSS = 0;
uint8_t aDATA_CALIB[4] = {0};
sData   sData_Calib ={aDATA_CALIB , 0};
extern sData sUart485SS;

Struct_KindMode485  sKindMode485=
{
    .Trans = _RS485_SS_CLO_SEND_PH,
};

Struct_ConvertChlorine      sConvertChlorine ={0};

uint8_t Kind_Trans_Calib = 0;

struct_TempAlarm    sTempAlarm = {0};
Struct_Sensor_Clo   sSensor_Clo= {0};
Struct_Hanlde_RS485 sHandleRs485 = {0};
Struct_pH_Recv_Master   spHRecvMaster={0,7};

int32_t DCU_Log_DataCalib = 0;
uint8_t DCU_Log_KindCalib = 0;
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
        case _LCD_SCR_SET_CALIB_SS_CLO:
          if(sKindMode485.Trans < _RS485_SS_CLO_CALIB_READ_SOLUTION)
          {
             sKindMode485.Trans++;
          }
          else
          {
            sKindMode485.Trans = _RS485_SS_CLO_SEND_PH;
          }
          break;
          
        default:
              sKindMode485.Trans = _RS485_SS_CLO_SEND_PH;
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
            
            if(sUart485SS.Data_a8[0] == ID_DEFAULT_SS_CLO)
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

static uint8_t fevent_detect_ph_recv(uint8_t event)
{
    spHRecvMaster.StateConnect = _SENSOR_DISCONNECT;
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
        case _RS485_SS_CLO_OPERA:
            
            sSensor_Clo.Clo_Value_f = quickSort_Sampling_Value((int32_t)(sSensor_Clo.Clo_Value_f));
            
            sSensor_Clo.Clo_Filter_f = Filter_Clo(sSensor_Clo.Clo_Value_f);
            sSensor_Clo.temp_Filter_f = Filter_Temp(sSensor_Clo.temp_Value_f);
            
            sSensor_Clo.temp_Filter_f += sSensor_Clo.temp_Offset_f;
            
            if(sSensor_Clo.Clo_Filter_f + sSensor_Clo.Clo_Offset_f < 0)
            {
                sSensor_Clo.Clo_Filter_f = 0;
            }
            else if(sSensor_Clo.Clo_Filter_f + sSensor_Clo.Clo_Offset_f > CLO_RANGE_MAX)
            {
                sSensor_Clo.Clo_Filter_f = CLO_RANGE_MAX;
            }
            else
            {
                sSensor_Clo.Clo_Filter_f += sSensor_Clo.Clo_Offset_f;
            }
            
          break;
          
        default:
          break;
    }
    
    if(sSensor_Clo.State_Connect == _SENSOR_DISCONNECT)
    {
        sSensor_Clo.Clo_Value_f = 0;
        sSensor_Clo.Clo_Filter_f = Filter_Clo(sSensor_Clo.Clo_Value_f);
    }
    
    if(sSensor_Clo.State_Connect == _SENSOR_DISCONNECT)
    {
        sSensor_Clo.temp_Value_f = 0;
        sSensor_Clo.Clo_Filter_f = Filter_Temp(sSensor_Clo.temp_Value_f);;
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

float Filter_Clo(float var)
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
    Handle_Data_Trans_SS_Clo(sFrame, KindRecv);
}
void Handle_Data_Recv_Sensor(sData sDataRS485, uint8_t KindRecv)
{
    if(sDataRS485.Data_a8[0] == ID_DEFAULT_SS_CLO)
        Handle_Data_Recv_SS_Clo(sDataRS485, KindRecv);
}

void Handle_State_Sensor(uint8_t KindRecv, uint8_t KindDetect)
{
    Handle_State_SS_Clo(KindRecv, KindDetect);
}

/*==================Handle Sensor Clo===================*/

float Const_PH_Compensation_Chlorine_f = 0;

void Handle_Data_Trans_SS_Clo(sData *sFrame, uint8_t KindTrans)
{
    uint8_t aData[4] = {0};
    float ph_Send_f = 0;
//    float ph_Send_f_stamp = 0;
    uint32_t ph_Send_u32 = 0;
    switch(KindTrans)
    {
        //Trans Opera
        case _RS485_SS_CLO_SEND_PH:
//            if(sDataSensorMeasure.spH_Water.Value > 500 && sDataSensorMeasure.spH_Water.Value < 900)
//                ph_Send_f = Handle_int32_To_Float_Scale(sDataSensorMeasure.spH_Water.Value, 0xFE);
//            else if(sDataSensorMeasure.spH_Water.Value == 0)
//                ph_Send_f = Handle_int32_To_Float_Scale(700, 0xFE);
//            else if(sDataSensorMeasure.spH_Water.Value <= 500)
//                ph_Send_f = Handle_int32_To_Float_Scale(500, 0xFE);
//            else
//                ph_Send_f = Handle_int32_To_Float_Scale(900, 0xFE);
            
//            ph_Send_f = Handle_int32_To_Float_Scale(796, 0xFE); // Khong bu pH cho gia tri Clo du
                   
//            Const_PH_Compensation_Chlorine_f = Handle_int32_To_Float_Scale(sConvertChlorine.sConst_Compensation_Temp.Value, sConvertChlorine.sConst_Compensation_Temp.Scale);
            
//            if(ph_Send_f  > 7)
//            {
//                ph_Send_f_stamp = (ph_Send_f - 7)/2;
//                ph_Send_f = (ph_Send_f - 7)/(2 - ph_Send_f_stamp) + 7;
//            }
            
//            if(ph_Send_f  > 7)
//            {
//                ph_Send_f_stamp = (ph_Send_f - 7)/2;
//                ph_Send_f = (ph_Send_f - 7)/(Const_PH_Compensation_Chlorine_f - ph_Send_f_stamp) + 7;
//            }
            
            ph_Send_f = Handle_int32_To_Float_Scale(700, 0xFE);
              
            ph_Send_u32 = Handle_Float_To_hexUint32(ph_Send_f);
            aData[0] = ph_Send_u32 >> 8;
            aData[1] = ph_Send_u32;
            aData[2] = ph_Send_u32 >> 24;
            aData[3] = ph_Send_u32 >> 16;
            ModRTU_Master_Write_Frame(sFrame, ID_DEFAULT_SS_CLO, 0x10, 0x0026, 2, aData);
            break;
            
        case _RS485_SS_CLO_READ_CURRENT:
            ModRTU_Master_Read_Frame(sFrame, ID_DEFAULT_SS_CLO, 0x03, 0x0005, 0x02);
            break;
            
        case _RS485_SS_CLO_OPERA:
            ModRTU_Master_Read_Frame(sFrame, ID_DEFAULT_SS_CLO, 0x03, 0x0001, 0x04);
            break;
     
        //Trans Calib
        case _RS485_SS_CLO_CALIB_READ_AD:
//            ModRTU_Master_Read_Frame(sFrame, ID_DEFAULT_SS_CLO, 0x03, 0x0066, 0x01);
            ModRTU_Master_Read_Frame(sFrame, ID_DEFAULT_SS_CLO, 0x03, 0x0005, 0x02);
            break;
            
        case _RS485_SS_CLO_CALIB_READ_SOLUTION:
            ModRTU_Master_Read_Frame(sFrame, ID_DEFAULT_SS_CLO, 0x03, 0x0024, 0x02);
            break;
            
        case _RS485_SS_CLO_CALIB_STD_SOLUTION:
            if(sData_Calib.Length_u16 == 4)
                ModRTU_Master_Write_Frame(sFrame, ID_DEFAULT_SS_CLO, 0x10, 0x0024, (sData_Calib.Length_u16/2), sData_Calib.Data_a8);
            break;
          
        case _RS485_SS_CLO_CALIB_ZERO:
            if(sData_Calib.Length_u16 == 2)
                ModRTU_Master_Write_Frame(sFrame, ID_DEFAULT_SS_CLO, 0x06, 0x003E, (sData_Calib.Length_u16/2), sData_Calib.Data_a8);
            break;
          
        case _RS485_SS_CLO_CALIB_SLOPE:
            if(sData_Calib.Length_u16 == 2)
                ModRTU_Master_Write_Frame(sFrame, ID_DEFAULT_SS_CLO, 0x06, 0x003F, (sData_Calib.Length_u16/2), sData_Calib.Data_a8);
            break;
            
        default:
          break;
    }
}

void Handle_Data_Recv_SS_Clo(sData sDataRS485, uint8_t KindRecv)
{
    uint16_t Pos = 0;
    uint32_t Stamp_Hex = 0;
    
    float test_1 = 0;
    
    switch(KindRecv)
    {
        //Recv Opera
        case  _RS485_SS_CLO_SEND_PH:
          break;
      
        case _RS485_SS_CLO_READ_CURRENT:
          Pos = 3;
            
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
          sConvertChlorine.Measure_AD =  Handle_HexFloat_To_Int32_Round(Stamp_Hex, 0xFE);
          
          break;
//          Pos = 3;
//          
//          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
//          sSensor_Clo.sPH.Value =  Handle_HexFloat_To_Int32_Round(Stamp_Hex, sSensor_Clo.sPH.Scale);
//          break;
          
        case _RS485_SS_CLO_OPERA:
          Pos = 3;
            
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
          Convert_uint32Hex_To_Float(Stamp_Hex, &sSensor_Clo.Clo_Value_f);

          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
          Convert_uint32Hex_To_Float(Stamp_Hex, &sSensor_Clo.temp_Value_f);
          
          //chuyen doi du lieu
          test_1 = Chlorine_Compensation_pH(sConvertChlorine.Measure_AD, (int16_t)(spHRecvMaster.pH_f *100), (int16_t)(sSensor_Clo.temp_Value_f*100));
          
          sConvertChlorine.sClo_Du.Value = (int16_t)(test_1*100);
          
          if(sConvertChlorine.sClo_Du.Value < 0)
            sConvertChlorine.sClo_Du.Value = 0;
          
          sSensor_Clo.Clo_Value_f = (float)(sConvertChlorine.sClo_Du.Value/100);
          break;
          
        //Recv Calib
        case _RS485_SS_CLO_CALIB_READ_AD:
//          Pos = 3;
//          
//          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 2);
//          sSensor_Clo.Measure_AD =  Stamp_Hex;
          Pos = 3;
            
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
          sConvertChlorine.Measure_AD =  Handle_HexFloat_To_Int32_Round(Stamp_Hex, 0xFE);
          
          test_1 = Chlorine_Compensation_pH(sConvertChlorine.Measure_AD, (int16_t)(spHRecvMaster.pH_f*100), (int16_t)(sSensor_Clo.temp_Value_f));
          
          sConvertChlorine.sClo_Du.Value = (int16_t)(test_1*100);
          break;
          
        case _RS485_SS_CLO_CALIB_READ_SOLUTION:
          Pos = 3;
          
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
          sSensor_Clo.sSolution_Calibration.Value =  Handle_HexFloat_To_Int32_Round(Stamp_Hex, sSensor_Clo.sSolution_Calibration.Scale);
          break;
          
        case _RS485_SS_CLO_CALIB_STD_SOLUTION:
        case _RS485_SS_CLO_CALIB_ZERO:
        case _RS485_SS_CLO_CALIB_SLOPE:
          RS485_Done_Calib();
          break;
      
        default:
          break;
    }
}

void Handle_State_SS_Clo(uint8_t KindRecv, uint8_t KindDetect)
{
    switch(KindRecv)
    {
        case _RS485_SS_CLO_SEND_PH:
        case _RS485_SS_CLO_READ_CURRENT:
        case _RS485_SS_CLO_OPERA:  
        case _RS485_SS_CLO_CALIB_READ_AD:
        case _RS485_SS_CLO_CALIB_READ_SOLUTION:
        case _RS485_SS_CLO_CALIB_STD_SOLUTION:
        case _RS485_SS_CLO_CALIB_ZERO:
        case _RS485_SS_CLO_CALIB_SLOPE:
          if(KindDetect == _RS485_RESPOND)
          {
             sSensor_Clo.Count_Disconnect = 0;
             sSensor_Clo.State_Connect = _SENSOR_CONNECT;
             
             if(KindRecv == _RS485_SS_CLO_OPERA)
             {
                 sHandleRs485.State_Recv_Clo = 1;
                 sHandleRs485.State_Recv_Temperature = 1;
             }
          }
          else
          {
             if(sSensor_Clo.Count_Disconnect <3)
                sSensor_Clo.Count_Disconnect++;
          }
          break;
          
        default:
          break;
    }
    
    if(sSensor_Clo.Count_Disconnect == 0)
    {
        
    }
    else if(sSensor_Clo.Count_Disconnect >=3)
    {
        sSensor_Clo.State_Connect = _SENSOR_DISCONNECT;
        
        sSensor_Clo.Clo_Value_f = 0;
        sSensor_Clo.temp_Value_f = 0;
    
        sSensor_Clo.sSolution_Calibration.Value = 0;
    
        sSensor_Clo.Measure_AD = 0;
        
        sHandleRs485.State_Recv_Clo = 0;
        sHandleRs485.State_Recv_Temperature = 0;
    }
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
        sConvertChlorine.sConst_Compensation_Temp.Value = 1;
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
    sParaDisplay.State_Setting = _STATE_SETTING_ENTER;
}

void DCU_Enter_Calib(void)
{
    int16_t Value = 0;
    Value = DCU_Log_DataCalib;
    
//    if(sSensor_pH.sPH_Value.Value > 0 && sConvertChlorine.Temp_Calib_Slope>0)
//    {
        sParaDisplay.State_Setting = _STATE_SETTING_DONE;

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
                                  (int16_t)(sSensor_Clo.temp_Value_f*100),
                                  (int16_t)(spHRecvMaster.pH_f*100));
              break;
              
            case _DCU_CALIB_CLO_POINT1:
              Save_Chlorine_PointCalib(sConvertChlorine.Measure_AD, 
                                       Value, 
                                       (int16_t)(sSensor_Clo.temp_Value_f*100),
                                       (int16_t)(spHRecvMaster.pH_f*100),
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
                                       (int16_t)(sSensor_Clo.temp_Value_f*100),
                                       (int16_t)(spHRecvMaster.pH_f*100));
              break;
              
            case _DCU_CALIB_CLO_CONST_TEMP:
              Save_Const_Temp_Compensation_Chlorine(Value);
              break;
              
            default:
              break;
        }
//    }
//    else
//      sHandleRs485.State_Wait_Calib = _STATE_CALIB_ERROR;
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
void Save_ParamCalib(float Clo_Offset_f, float temp_Offset_f)
{
#ifdef USING_APP_SENSOR
    uint8_t aData[50] = {0};
    uint8_t length = 0;
  
    uint32_t hexUint_Compensation_Clo = 0;
    uint32_t hexUint_Compensation_temp = 0;
    
    sSensor_Clo.Clo_Offset_f = Clo_Offset_f;
    sSensor_Clo.temp_Offset_f = temp_Offset_f;
    
    hexUint_Compensation_Clo = Handle_Float_To_hexUint32(sSensor_Clo.Clo_Offset_f);
    hexUint_Compensation_temp = Handle_Float_To_hexUint32(sSensor_Clo.temp_Offset_f);
    
    aData[length++] = hexUint_Compensation_Clo >> 24;
    aData[length++] = hexUint_Compensation_Clo >> 16;
    aData[length++] = hexUint_Compensation_Clo >> 8;
    aData[length++] = hexUint_Compensation_Clo ;
    
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
    uint32_t hexUint_Compensation_Clo = 0;
    uint32_t hexUint_Compensation_temp = 0;
  
    if(*(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE) != FLASH_BYTE_EMPTY)
    {
        hexUint_Compensation_Clo  = *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+2) << 24;
        hexUint_Compensation_Clo  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+3)<< 16;
        hexUint_Compensation_Clo  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+4)<< 8;
        hexUint_Compensation_Clo  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+5);
        
        hexUint_Compensation_temp  = *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+6) << 24;
        hexUint_Compensation_temp  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+7)<< 16;
        hexUint_Compensation_temp  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+8)<< 8;
        hexUint_Compensation_temp  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+9);
        
        Convert_uint32Hex_To_Float(hexUint_Compensation_Clo,  &sSensor_Clo.Clo_Offset_f);
        Convert_uint32Hex_To_Float(hexUint_Compensation_temp, &sSensor_Clo.temp_Offset_f);
    }
    else
    {
        sSensor_Clo.Clo_Offset_f = 0;
        sSensor_Clo.temp_Offset_f = 0;
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

void        Init_Parameter_Sensor(void)
{
    sSensor_Clo.sSolution_Calibration.Value     = 100;
    sSensor_Clo.sSolution_Calibration.Scale     = 0xFE;
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

void       AT_CMD_Get_Clo_Const_Temp(sData *str_Receiv, uint16_t Pos)
{
    uint8_t aTemp[100] = "Clo Const Temp: ";   //13 ki tu dau tien
    sData StrResp = {&aTemp[0], 16}; 

    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sConvertChlorine.sConst_Compensation_Temp.Value), sConvertChlorine.sConst_Compensation_Temp.Scale);
    
	HAL_UART_Transmit(&uart_debug, StrResp.Data_a8, StrResp.Length_u16, 0);
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
            HAL_UART_Transmit(&uart_debug, (uint8_t*)"OK", 2, 0);
        }
        else
        {
            HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 0);
        }
    }
    else
    {
        HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 0);
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

	HAL_UART_Transmit(&uart_debug, StrResp.Data_a8, StrResp.Length_u16, 0);
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
                HAL_UART_Transmit(&uart_debug, (uint8_t*)"OK", 2, 0);
            }
            else
            {
                HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 0);
            }
        }
        else
        {
            HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 0);
        }
    }
    else
    {
        HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 0);
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

	HAL_UART_Transmit(&uart_debug, StrResp.Data_a8, StrResp.Length_u16, 0);
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
                HAL_UART_Transmit(&uart_debug, (uint8_t*)"OK", 2, 0);
            }
            else
            {
                HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 0);
            }
        }
        else
        {
            HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 0);
        }
    }
    else
    {
        HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 0);
    }
}
#endif
/*==================Handle Task and Init app=================*/
void       Init_AppSensor(void)
{
    Init_ParamCalib();
//    Init_TempAlarm();
    Init_Parameter_Sensor();
    
    Init_Chlorine_Calib();
    Init_Chlorine_PointCalib_1();
    Init_Const_Temp_Compensation_Chlorine();
#ifdef USING_AT_CONFIG
    /* regis cb serial */
    CheckList_AT_CONFIG[_RESET_SLAVE].CallBack = AT_CMD_Reset_Slave;
    CheckList_AT_CONFIG[_RESTORE_SLAVE].CallBack = AT_CMD_Restore_Slave;
    
    CheckList_AT_CONFIG[_GET_ID_SLAVE].CallBack = AT_CMD_Get_ID_Slave;
    CheckList_AT_CONFIG[_SET_ID_SLAVE].CallBack = AT_CMD_Set_ID_Slave;
    CheckList_AT_CONFIG[_GET_BR_SLAVE].CallBack = AT_CMD_Get_BR_Slave;
    CheckList_AT_CONFIG[_SET_BR_SLAVE].CallBack = AT_CMD_Set_BR_Slave;
    
    CheckList_AT_CONFIG[_GET_CLO_CONST_TEMP].CallBack = AT_CMD_Get_Clo_Const_Temp;
    CheckList_AT_CONFIG[_SET_CLO_CONST_TEMP].CallBack = AT_CMD_Set_Clo_Const_Temp;
    
    CheckList_AT_CONFIG[_GET_CLO_ZERO_SLOPE].CallBack = AT_CMD_Get_Clo_Zero_Slope;
    CheckList_AT_CONFIG[_SET_CLO_ZERO_SLOPE].CallBack = AT_CMD_Set_Clo_Zero_Slope;
    
    CheckList_AT_CONFIG[_GET_CLO_CALIB_POINT].CallBack = AT_CMD_Get_Clo_Calib_Point;
    CheckList_AT_CONFIG[_SET_CLO_CALIB_POINT].CallBack = AT_CMD_Set_Clo_Calib_Point;
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
