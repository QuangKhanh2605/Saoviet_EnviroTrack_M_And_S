#include "user_app_rs485.h"
#include "user_modbus_rtu.h"
#include "user_internal_mem.h"
#include "user_rs485.h"
#include "user_app_wm.h"
#include "user_define.h"
#include "user_convert_variable.h"
/*=========================Fucntion Static=========================*/
static uint8_t fevent_rs485_entry(uint8_t event);
static uint8_t fevent_rs485_transmit(uint8_t event);
static uint8_t fevent_rs485_receive_handle(uint8_t event);
static uint8_t fevent_rs485_receive_complete(uint8_t event);

static uint8_t fevent_rs485_wait_calib(uint8_t event);
static uint8_t fevent_rs485_refresh(uint8_t event);
/*==============================Struct=============================*/
sEvent_struct               sEventAppRs485[]=
{
  {_EVENT_RS485_ENTRY,              1, 5, 30000,            fevent_rs485_entry},            //Doi slave khoi dong moi truyen opera
  {_EVENT_RS485_TRANSMIT,           0, 0, 1500,             fevent_rs485_transmit},
  {_EVENT_RS485_RECEIVE_HANDLE,     0, 5, 5,                fevent_rs485_receive_handle},
  {_EVENT_RS485_RECEIVE_COMPLETE,   0, 5, 500,              fevent_rs485_receive_complete},
  
  {_EVENT_RS485_WAIT_CALIB,         0, 5, 5000,             fevent_rs485_wait_calib},
  {_EVENT_RS485_REFRESH,            0, 5, 60000,            fevent_rs485_refresh},
};
uint16_t CountBufferHandleRecv = 0;

Struct_KindMode485  sKindMode485=
{
    .Trans = _RS485_SS_PH_OPERA,
};
extern sData sUart485;

uint8_t Kind_Trans_Calib = 0;
uint8_t aDATA_CALIB[4] = {0};
sData   sData_Calib ={aDATA_CALIB , 0};


Struct_Sensor_pH            sSensor_pH={0};
Struct_Sensor_EC            sSensor_EC={0};
Struct_Sensor_Clo           sSensor_Clo={0};
Struct_Sensor_Turbidity     sSensor_Turbidity={0};
Struct_Data_Sensor_Measure  sDataSensorMeasure = {0};
Struct_Hanlde_RS485         sHandleRs485 = {0};

int16_t aPH_ZERO_CALIB[2] = {700, 686};
int16_t aPH_SLOPE_CALIB[5] = {168, 401, 918, 1010, 1245};

/*========================Function Handle========================*/
static uint8_t fevent_rs485_entry(uint8_t event)
{
    fevent_enable(sEventAppRs485, _EVENT_RS485_TRANSMIT);
    fevent_enable(sEventAppRs485, _EVENT_RS485_REFRESH);
    return 1;
}

static uint8_t fevent_rs485_transmit(uint8_t event)
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
          if(sKindMode485.Trans < _RS485_SS_PH_CALIB_READ_POINT_CALIB && 
             sKindMode485.Trans >= _RS485_SS_PH_CALIB_READ_AD)
          {
             sKindMode485.Trans++;
          }
          else
          {
            sKindMode485.Trans = _RS485_SS_PH_CALIB_READ_AD;
          }
          break;
      
        case _LCD_SCR_SET_CALIB_SS_CLO:
          if(sKindMode485.Trans < _RS485_SS_CLO_CALIB_READ_SOLUTION && 
             sKindMode485.Trans >= _RS485_SS_CLO_CALIB_READ_AD)
          {
             sKindMode485.Trans++;
          }
          else
          {
            sKindMode485.Trans = _RS485_SS_CLO_CALIB_READ_AD;
          }
          break;
          
        case _LCD_SCR_SET_CALIB_SS_EC:
          if(sKindMode485.Trans != _RS485_SS_EC_CALIB_READ_DATA)
            sKindMode485.Trans = _RS485_SS_EC_CALIB_READ_DATA;
          break;
          
        case _LCD_SCR_SET_CALIB_SS_TUR_TAB_1:
        case _LCD_SCR_SET_CALIB_SS_TUR_TAB_2:
          if(sKindMode485.Trans < _RS485_SS_TUR_CALIB_READ_POINT_CALIB && 
             sKindMode485.Trans >= _RS485_SS_TUR_CALIB_READ_MEASURE_AD)
          {
             sKindMode485.Trans++;
          }
          else
          {
            sKindMode485.Trans = _RS485_SS_TUR_CALIB_READ_MEASURE_AD;
          }
          break;
          
        default:
          if(sKindMode485.Trans < _RS485_SS_CLO_OPERA)
          {
              sKindMode485.Trans++;
          }
          else
          {
              sKindMode485.Trans = _RS485_SS_PH_OPERA;
          }
          break;
    }
    
    
    fevent_active(sEventAppRs485, _EVENT_RS485_RECEIVE_HANDLE);
    fevent_enable(sEventAppRs485, _EVENT_RS485_RECEIVE_COMPLETE);
    fevent_enable(sEventAppRs485, event);
    return 1;
}

static uint8_t fevent_rs485_receive_handle(uint8_t event)
{
/*-----------------Kiem tra da nhan xong tu 485------------*/
    if(sUart485.Length_u16 != 0)
    {
        if(CountBufferHandleRecv == sUart485.Length_u16)
        {
            CountBufferHandleRecv = 0;
            fevent_active(sEventAppRs485, _EVENT_RS485_RECEIVE_COMPLETE);
            return 1;
        }
        else
        {
            CountBufferHandleRecv = sUart485.Length_u16;
        }
    }
    
    fevent_enable(sEventAppRs485, event);
    return 1;
}

static uint8_t fevent_rs485_receive_complete(uint8_t event)
{
/*------------------Xu ly chuoi nhan duoc----------------*/
    uint16_t Crc_Check = 0;
    uint16_t Crc_Recv  = 0;

    if(sUart485.Length_u16 > 2)
    {
        Crc_Recv = (sUart485.Data_a8[sUart485.Length_u16-1] << 8) |
                   (sUart485.Data_a8[sUart485.Length_u16-2]);
        Crc_Check = ModRTU_CRC(sUart485.Data_a8, sUart485.Length_u16 - 2);
        if(Crc_Check == Crc_Recv)
        {
            fevent_enable(sEventAppRs485, _EVENT_RS485_REFRESH);
            
            if(sUart485.Data_a8[0] == ID_DEFAULT_SS_PH ||
               sUart485.Data_a8[0] == ID_DEFAULT_SS_CLO ||
               sUart485.Data_a8[0] == ID_DEFAULT_SS_EC ||
               sUart485.Data_a8[0] == ID_DEFAULT_SS_TURBIDITY)
            {
                Handle_State_Sensor(sKindMode485.Recv, _RS485_RESPOND);
                Handle_Data_Recv_Sensor(sUart485, sKindMode485.Recv);
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

    fevent_disable(sEventAppRs485, _EVENT_RS485_RECEIVE_HANDLE);
    return 1;
}

static uint8_t fevent_rs485_wait_calib(uint8_t event)
{
    if(sHandleRs485.State_Wait_Calib != _STATE_CALIB_DONE)
    {
        sHandleRs485.State_Wait_Calib = _STATE_CALIB_ERROR;
    }
    return 1;
}

static uint8_t fevent_rs485_refresh(uint8_t event)
{
    Init_UartRs485();
    fevent_enable(sEventAppRs485, event);
    return 1;
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
          sSensor_pH.sPH_Value.Value =  Handle_HexFloat_To_Int32_Round(Stamp_Hex, sSensor_pH.sTemperature_Value.Scale);
          
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
          sSensor_pH.sTemperature_Value.Value =  Handle_HexFloat_To_Int32_Round(Stamp_Hex, sSensor_pH.sTemperature_Value.Scale);
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
        sSensor_pH.sPH_Value.Value = 0;
        sSensor_pH.sTemperature_Value.Value = 0;
        sSensor_pH.sZero_Calib.Value = 0;
        sSensor_pH.sSlope_Calib.Value = 0;
        
        sSensor_pH.Measure_AD = 0;
        sHandleRs485.State_Recv_pH = 0;
    }
}

/*==================Handle Sensor Clo===================*/

float Const_PH_Compensation_Chlorine_f = 0;

void Handle_Data_Trans_SS_Clo(sData *sFrame, uint8_t KindTrans)
{
    uint8_t aData[4] = {0};
    float ph_Send_f = 0;
    float ph_Send_f_stamp = 0;
    uint32_t ph_Send_u32 = 0;
    switch(KindTrans)
    {
        //Trans Opera
        case _RS485_SS_CLO_SEND_PH:
            if(sDataSensorMeasure.spH_Water.Value > 500 && sDataSensorMeasure.spH_Water.Value < 900)
                ph_Send_f = Handle_int32_To_Float_Scale(sDataSensorMeasure.spH_Water.Value, 0xFE);
            else if(sDataSensorMeasure.spH_Water.Value == 0)
                ph_Send_f = Handle_int32_To_Float_Scale(700, 0xFE);
            else if(sDataSensorMeasure.spH_Water.Value <= 500)
                ph_Send_f = Handle_int32_To_Float_Scale(500, 0xFE);
            else
                ph_Send_f = Handle_int32_To_Float_Scale(900, 0xFE);
            
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
          sSensor_Clo.sClo_Value.Value =  Handle_HexFloat_To_Int32_Round(Stamp_Hex, sSensor_Clo.sClo_Value.Scale);

          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
          sSensor_Clo.sTemperature_Value.Value =  Handle_HexFloat_To_Int32_Round(Stamp_Hex, sSensor_Clo.sTemperature_Value.Scale);
          
          //chuyen doi du lieu
          test_1 = Chlorine_Compensation_pH(sConvertChlorine.Measure_AD, sSensor_pH.sPH_Value.Value, sSensor_Clo.sTemperature_Value.Value);
          
          sConvertChlorine.sClo_Du.Value = (int16_t)(test_1*100);
          
          if(sConvertChlorine.sClo_Du.Value < 0)
            sConvertChlorine.sClo_Du.Value = 0;
          
          sSensor_Clo.sClo_Value.Value = sConvertChlorine.sClo_Du.Value;
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
          
          test_1 = Chlorine_Compensation_pH(sConvertChlorine.Measure_AD, sSensor_pH.sPH_Value.Value, sSensor_Clo.sTemperature_Value.Value);
          
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
        
        sSensor_Clo.sClo_Value.Value = 0;
        sSensor_Clo.sTemperature_Value.Value = 0;
        sSensor_Clo.sPH.Value = 0;
    
        sSensor_Clo.sSolution_Calibration.Value = 0;
    
        sSensor_Clo.Measure_AD = 0;
        
        sHandleRs485.State_Recv_Clo = 0;
        sHandleRs485.State_Recv_Temperature = 0;
    }
}

/*==================Handle Sensor EC===================*/
void Handle_Data_Trans_SS_EC(sData *sFrame, uint8_t KindTrans)
{
    switch(KindTrans)
    {
        //Send Opera
        case _RS485_SS_EC_OPERA:
            ModRTU_Master_Read_Frame(sFrame, ID_DEFAULT_SS_EC, 0x03, 0x0000, 0x0A);
            break;
            
        //Send Calib
        case _RS485_SS_EC_CALIB_READ_DATA:
            ModRTU_Master_Read_Frame(sFrame, ID_DEFAULT_SS_EC, 0x03, 0x0000, 0x0C);
            break;
            
        case _RS485_SS_EC_CALIB_CONST_STD:
            if(sData_Calib.Length_u16 == 4)
                ModRTU_Master_Write_Frame(sFrame, ID_DEFAULT_SS_EC, 0x10, 0x000A, (sData_Calib.Length_u16/2), sData_Calib.Data_a8);
            break;
            
        case _RS485_SS_EC_CALIB_CONST_REAL:
            if(sData_Calib.Length_u16 == 4)
                ModRTU_Master_Write_Frame(sFrame, ID_DEFAULT_SS_EC, 0x10, 0x000A, (sData_Calib.Length_u16/2), sData_Calib.Data_a8);
            break;
      
        default:
          break;
    }
}

void Handle_Data_Recv_SS_EC(sData sDataRS485, uint8_t KindRecv)
{
    uint16_t Pos = 0;
    uint32_t Stamp_Hex = 0;
    uint32_t Stamp_i32;
    switch(KindRecv)
    {
        //Recv Opera
        case _RS485_SS_EC_OPERA:
          Pos = 3;
          
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
          Stamp_i32 = Handle_HexFloat_To_Int32_Round(Stamp_Hex, 0xFD);
          if(Stamp_i32 < 32760)
            sSensor_EC.sConductivity_Value.Value = Stamp_i32;  //mS/cm -> uS/cm
          else 
            sSensor_EC.sConductivity_Value.Value = 32760;
          
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
          sSensor_EC.sResistivity_Value.Value =  Handle_HexFloat_To_Int32_Round(Stamp_Hex, sSensor_EC.sResistivity_Value.Scale);
          
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
          sSensor_EC.sTemperature_Value.Value =  Handle_HexFloat_To_Int32_Round(Stamp_Hex, sSensor_EC.sTemperature_Value.Scale);
          
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
          sSensor_EC.sTDS_Value.Value =  Handle_HexFloat_To_Int32_Round(Stamp_Hex, sSensor_EC.sTDS_Value.Scale);
          
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
          Stamp_i32 =  Handle_HexFloat_To_Int32_Round(Stamp_Hex, sSensor_EC.sSalinity_Value.Scale);
          sSensor_EC.sSalinity_Value.Value = Stamp_i32/10000;
          break;
          
        //Recv Calib
        case _RS485_SS_EC_CALIB_READ_DATA:
          Pos = 3;
          
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
          sSensor_EC.sConductivity_Value.Value =  Handle_HexFloat_To_Int32_Round(Stamp_Hex, 0xFD);  //mS/cm -> uS/cm
          sSensor_EC.sConductivity_Calib.Value = sSensor_EC.sConductivity_Value.Value;
          
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 16);
          
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
          sSensor_EC.sConductivity_Const.Value =  Handle_HexFloat_To_Int32_Round(Stamp_Hex, sSensor_EC.sConductivity_Const.Scale);
          break;
          
        case _RS485_SS_EC_CALIB_CONST_STD:
        case _RS485_SS_EC_CALIB_CONST_REAL:
          RS485_Done_Calib();
          break;
      
        default:
          break;
    }
}

void Handle_State_SS_EC(uint8_t KindRecv, uint8_t KindDetect)
{
    switch(KindRecv)
    {
        case _RS485_SS_EC_OPERA:
        case _RS485_SS_EC_CALIB_READ_DATA:
        case _RS485_SS_EC_CALIB_CONST_STD:
        case _RS485_SS_EC_CALIB_CONST_REAL:
          if(KindDetect == _RS485_RESPOND)
          {
             sSensor_EC.Count_Disconnect = 0;
             sSensor_EC.State_Connect = _SENSOR_CONNECT;
             
             if(KindRecv == _RS485_SS_EC_OPERA)
             {
                sHandleRs485.State_Recv_Salinity = 1;
                sHandleRs485.State_Recv_EC = 1;
             }
          }
          else
          {
             if(sSensor_EC.Count_Disconnect <3)
               sSensor_EC.Count_Disconnect++;
          }
          break;
          
        default:
          break;
    }
    
    if(sSensor_EC.Count_Disconnect == 0)
    {
        
    }
    else if(sSensor_EC.Count_Disconnect >=3)
    {
        sSensor_EC.State_Connect = _SENSOR_DISCONNECT;
        
        sSensor_EC.sConductivity_Value.Value = 0;
        sSensor_EC.sResistivity_Value.Value = 0;
        sSensor_EC.sTemperature_Value.Value = 0;
        sSensor_EC.sTDS_Value.Value = 0;
        sSensor_EC.sSalinity_Value.Value = 0;
    
        sSensor_EC.sConductivity_Const.Value = 0;
        sSensor_EC.sConductivity_Calib.Value = 0;
        
        sHandleRs485.State_Recv_Salinity = 0;
        sHandleRs485.State_Recv_EC = 0;
    }
}

/*==================Handle Sensor Turbidity===================*/
void Handle_Data_Trans_SS_Turbidity(sData *sFrame, uint8_t KindTrans)
{
    switch(KindTrans)
    {
        //Send Opera
        case _RS485_SS_TURBIDITY_OPERA:
            ModRTU_Master_Read_Frame(sFrame, ID_DEFAULT_SS_TURBIDITY, 0x03, 0x0001, 0x04);
            break;
            
        //Send Calib
        case _RS485_SS_TUR_CALIB_READ_MEASURE_AD:
            ModRTU_Master_Read_Frame(sFrame, ID_DEFAULT_SS_TURBIDITY, 0x03, 0x0018, 0x02);
            break; 
            
        case _RS485_SS_TUR_CALIB_READ_POINT_CALIB:
            ModRTU_Master_Read_Frame(sFrame, ID_DEFAULT_SS_TURBIDITY, 0x03, 0x0029, 0x10);
            break;
            
        case _RS485_SS_TUR_CALIB_1ST_VALUE:
            if(sData_Calib.Length_u16 == 4)
                ModRTU_Master_Write_Frame(sFrame, ID_DEFAULT_SS_TURBIDITY, 0x10, 0x0029, (sData_Calib.Length_u16/2), sData_Calib.Data_a8);
            break;
          
        case _RS485_SS_TUR_CALIB_1ST_AD:
            if(sData_Calib.Length_u16 == 4)
                ModRTU_Master_Write_Frame(sFrame, ID_DEFAULT_SS_TURBIDITY, 0x10, 0x002B, (sData_Calib.Length_u16/2), sData_Calib.Data_a8);
            break;
          
        case _RS485_SS_TUR_CALIB_2ND_VALUE:
            if(sData_Calib.Length_u16 == 4)
                ModRTU_Master_Write_Frame(sFrame, ID_DEFAULT_SS_TURBIDITY, 0x10, 0x002D, (sData_Calib.Length_u16/2), sData_Calib.Data_a8);
            break;
          
        case _RS485_SS_TUR_CALIB_2ND_AD:
            if(sData_Calib.Length_u16 == 4)
                ModRTU_Master_Write_Frame(sFrame, ID_DEFAULT_SS_TURBIDITY, 0x10, 0x002F, (sData_Calib.Length_u16/2), sData_Calib.Data_a8);
            break;
          
        case _RS485_SS_TUR_CALIB_3RD_VALUE:
            if(sData_Calib.Length_u16 == 4)
                ModRTU_Master_Write_Frame(sFrame, ID_DEFAULT_SS_TURBIDITY, 0x10, 0x0031, (sData_Calib.Length_u16/2), sData_Calib.Data_a8);
            break;
          
        case _RS485_SS_TUR_CALIB_3RD_AD:
            if(sData_Calib.Length_u16 == 4)
                ModRTU_Master_Write_Frame(sFrame, ID_DEFAULT_SS_TURBIDITY, 0x10, 0x0033, (sData_Calib.Length_u16/2), sData_Calib.Data_a8);
            break;
          
        case _RS485_SS_TUR_CALIB_4TH_VALUE:
            if(sData_Calib.Length_u16 == 4)
                ModRTU_Master_Write_Frame(sFrame, ID_DEFAULT_SS_TURBIDITY, 0x10, 0x0035, (sData_Calib.Length_u16/2), sData_Calib.Data_a8);
            break;
          
        case _RS485_SS_TUR_CALIB_4TH_AD:
            if(sData_Calib.Length_u16 == 4)
                ModRTU_Master_Write_Frame(sFrame, ID_DEFAULT_SS_TURBIDITY, 0x10, 0x0037, (sData_Calib.Length_u16/2), sData_Calib.Data_a8);
            break;
      
        default:
          break;
    }
}

void Handle_Data_Recv_SS_Turbidity(sData sDataRS485, uint8_t KindRecv)
{
    uint16_t Pos = 0;
    uint32_t Stamp_Hex = 0;
    switch(KindRecv)
    {
        //Recv Opera
        case _RS485_SS_TURBIDITY_OPERA:
          Pos = 3;
          
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
          sSensor_Turbidity.sNTU_Value.Value =  Handle_HexFloat_To_Int32_Round(Stamp_Hex, sSensor_Turbidity.sNTU_Value.Scale);
          
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
          sSensor_Turbidity.sTemperature_Value.Value =  Handle_HexFloat_To_Int32_Round(Stamp_Hex, sSensor_Turbidity.sTemperature_Value.Scale);
          break; 
  
        //Recv Calib
        case _RS485_SS_TUR_CALIB_READ_MEASURE_AD:
          Pos = 3;
          
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
          sSensor_Turbidity.Measure_AD = Handle_HexFloat_To_Int32_Round(Stamp_Hex, 0x00);
          break;
          
        case _RS485_SS_TUR_CALIB_READ_POINT_CALIB:
          Pos = 3;
          
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
          sSensor_Turbidity.sFirst_Calibration.Value =  Handle_HexFloat_To_Int32_Round(Stamp_Hex, sSensor_Turbidity.sFirst_Calibration.Scale);  //mS/cm -> uS/cm
          
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
          sSensor_Turbidity.First_AD_u32 =  Stamp_Hex;
          
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
          sSensor_Turbidity.sSecond_Calibration.Value =  Handle_HexFloat_To_Int32_Round(Stamp_Hex, sSensor_Turbidity.sSecond_Calibration.Scale);
          
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
          sSensor_Turbidity.Second_AD_u32 =  Stamp_Hex;
          
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
          sSensor_Turbidity.sThird_Calibration.Value =  Handle_HexFloat_To_Int32_Round(Stamp_Hex, sSensor_Turbidity.sThird_Calibration.Scale);  //mS/cm -> uS/cm
          
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
          sSensor_Turbidity.Third_AD_u32 =  Stamp_Hex;
          
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
          sSensor_Turbidity.sFourth_Calibration.Value =  Handle_HexFloat_To_Int32_Round(Stamp_Hex, sSensor_Turbidity.sFourth_Calibration.Scale);
          
          Stamp_Hex = Read_Register_Rs485(sDataRS485.Data_a8, &Pos, 4);
          sSensor_Turbidity.Fourth_AD_u32 =  Stamp_Hex;
          break;
      
        case _RS485_SS_TUR_CALIB_1ST_VALUE:
        case _RS485_SS_TUR_CALIB_1ST_AD:
        case _RS485_SS_TUR_CALIB_2ND_VALUE:
        case _RS485_SS_TUR_CALIB_2ND_AD:
        case _RS485_SS_TUR_CALIB_3RD_VALUE:
        case _RS485_SS_TUR_CALIB_3RD_AD:
        case _RS485_SS_TUR_CALIB_4TH_VALUE:
        case _RS485_SS_TUR_CALIB_4TH_AD:
          RS485_Done_Calib();
            break;
          
        default:
          break;
    }
}
void Handle_State_SS_Turbidity(uint8_t KindRecv, uint8_t KindDetect)
{
    switch(KindRecv)
    {
        case _RS485_SS_TURBIDITY_OPERA:
        case _RS485_SS_TUR_CALIB_READ_MEASURE_AD:
        case _RS485_SS_TUR_CALIB_READ_POINT_CALIB:
        case _RS485_SS_TUR_CALIB_1ST_VALUE:
        case _RS485_SS_TUR_CALIB_1ST_AD:
        case _RS485_SS_TUR_CALIB_2ND_VALUE:
        case _RS485_SS_TUR_CALIB_2ND_AD:
        case _RS485_SS_TUR_CALIB_3RD_VALUE:
        case _RS485_SS_TUR_CALIB_3RD_AD:
        case _RS485_SS_TUR_CALIB_4TH_VALUE:
        case _RS485_SS_TUR_CALIB_4TH_AD:
          if(KindDetect == _RS485_RESPOND)
          {
            sSensor_Turbidity.Count_Disconnect = 0;
            sSensor_Turbidity.State_Connect = _SENSOR_CONNECT;
            
            if(KindRecv  == _RS485_SS_TURBIDITY_OPERA)
            {
                sHandleRs485.State_Recv_NTU = 1;
            }
          }
          else
          {
            if(sSensor_Turbidity.Count_Disconnect <3)
            sSensor_Turbidity.Count_Disconnect++;
          }
          break;
          
        default:
          break;
    }
    
    if(sSensor_Turbidity.Count_Disconnect == 0)
    {
        
    }
    else if(sSensor_Turbidity.Count_Disconnect >=3)
    {
        sSensor_Turbidity.State_Connect = _SENSOR_DISCONNECT;
        
        sSensor_Turbidity.sNTU_Value.Value = 0;
        sSensor_Turbidity.sTemperature_Value.Value = 0;
        
        sSensor_Turbidity.sFirst_Calibration.Value = 0;
        sSensor_Turbidity.First_AD_u32= 0;
        sSensor_Turbidity.sSecond_Calibration.Value = 0;
        sSensor_Turbidity.Second_AD_u32= 0;
        sSensor_Turbidity.sThird_Calibration.Value = 0;
        sSensor_Turbidity.Third_AD_u32= 0;
        sSensor_Turbidity.sFourth_Calibration.Value = 0;
        sSensor_Turbidity.Fourth_AD_u32= 0;
        
        sSensor_Turbidity.Measure_AD= 0;
        
        sHandleRs485.State_Recv_NTU = 0;
    }
}

/*=====================Handle Sensor=====================*/
void Handle_Data_Trans_Sensor(sData *sFrame, uint8_t KindRecv)
{
    Handle_Data_Trans_SS_pH(sFrame, KindRecv);
    Handle_Data_Trans_SS_Clo(sFrame, KindRecv);
    Handle_Data_Trans_SS_EC(sFrame, KindRecv);
    Handle_Data_Trans_SS_Turbidity(sFrame, KindRecv);
}

void Handle_Data_Recv_Sensor(sData sDataRS485, uint8_t KindRecv)
{
    if(sDataRS485.Data_a8[0] == ID_DEFAULT_SS_PH)
        Handle_Data_Recv_SS_pH(sDataRS485, KindRecv);
    
    if(sDataRS485.Data_a8[0] == ID_DEFAULT_SS_CLO)
        Handle_Data_Recv_SS_Clo(sDataRS485, KindRecv);
    
    if(sDataRS485.Data_a8[0] == ID_DEFAULT_SS_EC)
        Handle_Data_Recv_SS_EC(sDataRS485, KindRecv);
    
    if(sDataRS485.Data_a8[0] == ID_DEFAULT_SS_TURBIDITY)
        Handle_Data_Recv_SS_Turbidity(sDataRS485, KindRecv);
}

void Handle_State_Sensor(uint8_t KindRecv, uint8_t KindDetect)
{
    Handle_State_SS_pH(KindRecv, KindDetect);
    Handle_State_SS_Clo(KindRecv, KindDetect);
    Handle_State_SS_EC(KindRecv, KindDetect);
    Handle_State_SS_Turbidity(KindRecv, KindDetect);
}

void Handle_Data_Measure(uint8_t KindRecv)
{
    switch(KindRecv)
    {
        case _RS485_SS_PH_OPERA:
            sDataSensorMeasure.spH_Water = sSensor_pH.sPH_Value;
            
            quickSort_Sampling_pH();
            sOffsetMeasure.spH_Water_Sampling.Value = sDataSensorMeasure.spH_Water.Value;
            
            if(sDataSensorMeasure.spH_Water.Value + sOffsetMeasure.spH_Water.Value >= 0)
                sDataSensorMeasure.spH_Water.Value += sOffsetMeasure.spH_Water.Value;
            
            sDataSensorMeasure.spH_Water_Filter.Value = Filter_pH(sDataSensorMeasure.spH_Water.Value, sDataSensorMeasure.spH_Water.Scale);
          break;
          
        case _RS485_SS_CLO_OPERA:
            sDataSensorMeasure.sClo_Du = sSensor_Clo.sClo_Value;
            sDataSensorMeasure.sTemperature = sSensor_Clo.sTemperature_Value;
            
            quickSort_Sampling_Clo();
            sOffsetMeasure.sClo_Du_Sampling.Value = sDataSensorMeasure.sClo_Du.Value;
            sOffsetMeasure.sTemperature_Sampling.Value = sDataSensorMeasure.sTemperature.Value;
            
            if(sDataSensorMeasure.sClo_Du.Value + sOffsetMeasure.sClo_Du.Value >= 0)
                sDataSensorMeasure.sClo_Du.Value += sOffsetMeasure.sClo_Du.Value;

            if(sDataSensorMeasure.sTemperature.Value + sOffsetMeasure.sTemperature.Value >= 0)
                sDataSensorMeasure.sTemperature.Value += sOffsetMeasure.sTemperature.Value;
            
            sDataSensorMeasure.sClo_Du_Filter.Value = Filter_CloDu(sDataSensorMeasure.sClo_Du.Value, sDataSensorMeasure.sClo_Du.Scale);
            sDataSensorMeasure.sTemperature_Filter.Value = Filter_Temperature(sDataSensorMeasure.sTemperature.Value, sDataSensorMeasure.sTemperature.Scale);
          break;
          
        case _RS485_SS_EC_OPERA:
            sDataSensorMeasure.sEC = sSensor_EC.sConductivity_Value;
            sDataSensorMeasure.sSalinity = sSensor_EC.sSalinity_Value;
            quickSort_Sampling_EC();
            sOffsetMeasure.sEC_Sampling.Value = sDataSensorMeasure.sEC.Value;
            sOffsetMeasure.sSalinity_Sampling.Value = sDataSensorMeasure.sSalinity.Value;
            
            if(sDataSensorMeasure.sEC.Value + sOffsetMeasure.sEC.Value >= 0)
                sDataSensorMeasure.sEC.Value += sOffsetMeasure.sEC.Value;
            
            if(sDataSensorMeasure.sSalinity.Value + sOffsetMeasure.sSalinity.Value >= 0)
                sDataSensorMeasure.sSalinity.Value += sOffsetMeasure.sSalinity.Value;
            
            sDataSensorMeasure.sSalinity_Filter.Value = Filter_Salinity(sDataSensorMeasure.sSalinity.Value, sDataSensorMeasure.sSalinity.Scale);
            sDataSensorMeasure.sEC_Filter.Value = Filter_EC(sDataSensorMeasure.sEC.Value, sDataSensorMeasure.sEC.Scale);
          break;
          
        case _RS485_SS_TURBIDITY_OPERA:
            sDataSensorMeasure.sNTU = sSensor_Turbidity.sNTU_Value;
            quickSort_Sampling_Turbidity();
            sOffsetMeasure.sNTU_Sampling.Value = sDataSensorMeasure.sNTU.Value;
            
            if(sDataSensorMeasure.sNTU.Value + sOffsetMeasure.sNTU.Value >= 0)
                sDataSensorMeasure.sNTU.Value += sOffsetMeasure.sNTU.Value;
            
            sDataSensorMeasure.sNTU_Filter.Value = Filter_Turbidity(sDataSensorMeasure.sNTU.Value, sDataSensorMeasure.sNTU.Scale);
          break;
          
        default:
          break;
    }
    
    if(sSensor_pH.State_Connect == _SENSOR_DISCONNECT)
    {
        sDataSensorMeasure.spH_Water.Value = 0;
        sDataSensorMeasure.spH_Water_Filter.Value = Filter_pH(sDataSensorMeasure.spH_Water.Value, sDataSensorMeasure.spH_Water.Scale);
    }
    
    if(sSensor_Clo.State_Connect == _SENSOR_DISCONNECT)
    {
        sDataSensorMeasure.sClo_Du.Value = 0;
        sDataSensorMeasure.sClo_Du_Filter.Value = Filter_CloDu(sDataSensorMeasure.sClo_Du.Value, sDataSensorMeasure.sClo_Du.Scale);
    }
    
    if(sSensor_Clo.State_Connect == _SENSOR_DISCONNECT)
    {
        sDataSensorMeasure.sTemperature.Value = 0;
        sDataSensorMeasure.sTemperature_Filter.Value = Filter_Temperature(sDataSensorMeasure.sTemperature.Value, sDataSensorMeasure.sTemperature.Scale);
    }
    
    if(sSensor_EC.State_Connect == _SENSOR_DISCONNECT)
    {
        sDataSensorMeasure.sEC.Value = 0;
        sDataSensorMeasure.sEC_Filter.Value = Filter_EC(sDataSensorMeasure.sEC.Value, sDataSensorMeasure.sEC.Scale);
    }
    
    if(sSensor_EC.State_Connect == _SENSOR_DISCONNECT)
    {
        sDataSensorMeasure.sSalinity.Value = 0;
        sDataSensorMeasure.sSalinity_Filter.Value = Filter_Salinity(sDataSensorMeasure.sSalinity.Value, sDataSensorMeasure.sSalinity.Scale);
    }
    
    if(sSensor_Turbidity.State_Connect == _SENSOR_DISCONNECT)
    {
        sDataSensorMeasure.sNTU.Value = 0;
        sDataSensorMeasure.sNTU_Filter.Value = Filter_Turbidity(sDataSensorMeasure.sNTU.Value, sDataSensorMeasure.sNTU.Scale);
    }
}
/*==========================Handle==========================*/

///*-----------------------Handle Send RS485-----------------------*/
///*
//    @brief Send 485 Debug
//*/
//void        Send_RS458_Normal(uint8_t *aData, uint16_t Length_u16) 
//{
//    HAL_GPIO_WritePin(RS485_TXDE_GPIO_Port, RS485_TXDE_Pin, GPIO_PIN_SET);
//    HAL_Delay(1);
//    HAL_UART_Transmit(&uart_debug, aData, Length_u16, 1000);
//    HAL_Delay(1);
//    HAL_GPIO_WritePin(RS485_TXDE_GPIO_Port, RS485_TXDE_Pin, GPIO_PIN_RESET);
//}

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
  

    HAL_GPIO_WritePin(DE_GPIO_PORT, DE_GPIO_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
    // Send
    RS485_Init_Data();
    HAL_UART_Transmit(&uart_rs485, aData , Length_u16, 1000); 
    
    //Dua DE ve Receive
    HAL_GPIO_WritePin(DE_GPIO_PORT, DE_GPIO_PIN, GPIO_PIN_RESET);
}

/*----------------------Save and Init Id Slave---------------------*/
void        Save_IdSlave(uint8_t ID_Oxy, uint8_t ID_pH)
{
//    uint8_t aData[2] = {0};
//    uint8_t length = 0;
//    
//    sIdSlave485.Oxy = ID_Oxy;
//    sIdSlave485.pH  = ID_pH;
//    aData[length++] = sIdSlave485.Oxy ;
//    aData[length++] = sIdSlave485.pH;
//
//    Save_Array(ADDR_INFOR_SLAVE_RS485, aData, length);
}

void        Init_IdSlave(void)
{
//    if(*(__IO uint8_t*)(ADDR_INFOR_SLAVE_RS485) == BYTE_TEMP_FIRST)
//    {
//        sIdSlave485.Oxy = *(__IO uint8_t*)(ADDR_INFOR_SLAVE_RS485+2);
//        sIdSlave485.pH  = *(__IO uint8_t*)(ADDR_INFOR_SLAVE_RS485+3);
//    }
}

void        Init_Parameter_Sensor(void)
{
    sSensor_pH.sPH_Value.Scale                  = DEFAULT_SCALE_PH;
    sSensor_pH.sTemperature_Value.Scale         = DEFAULT_SCALE_TEMPERATURE;
    sSensor_pH.sZero_Calib.Value                = 0;
    sSensor_pH.sZero_Calib.Scale                = 0xFE;
    sSensor_pH.sSlope_Calib.Value               = 0;
    sSensor_pH.sSlope_Calib.Scale               = 0xFE;
    
    sSensor_Clo.sClo_Value.Scale                = DEFAULT_SCALE_CLO;
    sSensor_Clo.sTemperature_Value.Scale        = DEFAULT_SCALE_TEMPERATURE;
    sSensor_Clo.sPH.Scale                       = 0xFE;
    sSensor_Clo.sSolution_Calibration.Value     = 100;
    sSensor_Clo.sSolution_Calibration.Scale     = 0xFE;
    
    sSensor_EC.sConductivity_Value.Scale        = DEFAULT_SCALE_EC;
    sSensor_EC.sResistivity_Value.Scale         = 0xFE;
    sSensor_EC.sSalinity_Value.Scale            = DEFAULT_SCALE_SALINITY;
    sSensor_EC.sTDS_Value.Scale                 = 0xFE;
    sSensor_EC.sTemperature_Value.Scale         = DEFAULT_SCALE_TEMPERATURE;
    
    sSensor_EC.sConductivity_Calib.Scale        = 0x00;
    sSensor_EC.sConductivity_Const.Value        = 1000;
    sSensor_EC.sConductivity_Const.Scale        = 0xFD;
    
    sSensor_Turbidity.sNTU_Value.Scale          = DEFAULT_SCALE_NTU;
    sSensor_Turbidity.sTemperature_Value.Scale  = DEFAULT_SCALE_TEMPERATURE;
    sSensor_Turbidity.sFirst_Calibration.Scale  = 0xFE;
    sSensor_Turbidity.sSecond_Calibration.Scale = 0xFE;
    sSensor_Turbidity.sThird_Calibration.Scale  = 0xFE;
    sSensor_Turbidity.sFourth_Calibration.Scale = 0xFE;
    
    sDataSensorMeasure.spH_Water = sSensor_pH.sPH_Value;
    sDataSensorMeasure.sClo_Du = sSensor_Clo.sClo_Value;
    sDataSensorMeasure.sTemperature = sSensor_Clo.sTemperature_Value;
    sDataSensorMeasure.sEC = sSensor_EC.sConductivity_Value;
    
    sDataSensorMeasure.sSalinity = sSensor_EC.sSalinity_Value;
    sDataSensorMeasure.sNTU = sSensor_Turbidity.sNTU_Value;
    
    sDataSensorMeasure.spH_Water_Filter = sDataSensorMeasure.spH_Water;
    sDataSensorMeasure.sClo_Du_Filter = sDataSensorMeasure.sClo_Du;
    sDataSensorMeasure.sTemperature_Filter = sDataSensorMeasure.sTemperature;
    sDataSensorMeasure.sEC_Filter = sDataSensorMeasure.sEC;
    
    sDataSensorMeasure.sSalinity_Filter = sDataSensorMeasure.sSalinity;
    sDataSensorMeasure.sNTU_Filter = sDataSensorMeasure.sNTU;
}
/*=========================Handle Data=======================*/
int32_t Hanlde_Float_To_Int32_Scale_Round(float varFloat, uint8_t scale)
{
    float f_hexFloat = 0;
    int32_t u_hexFloat = 0;
    
    uint32_t division = 1;
    uint32_t stamp_div = 0;
    float f_compensation = 0;

    f_hexFloat = varFloat;
    
    switch(scale)
    {
        case 0x00:
          division = 1;
          f_compensation = 0;
          break;
          
        case 0xFF:
          division = 10;
          f_compensation = 0.1;
          break;
          
        case 0xFE:
          division = 100;
          f_compensation = 0.01;
          break;
          
        case 0xFD:
          division = 1000;
          f_compensation = 0.001;
          break;
         
        case 0xFC:
          division = 10000;
          f_compensation = 0.0001;
          break;
          
        case 0xFB:
          division = 100000;
          f_compensation = 0.00001;
          break;
          
        default:
          break;
    }
   
    if(f_hexFloat >= 0)    
    {
      stamp_div = (uint32_t)(f_hexFloat*division*10)%10;
      if(stamp_div >= 5) f_hexFloat += f_compensation;
    }
    else                
    {
      stamp_div = (uint32_t)((0-f_hexFloat)*division*10)%10;
      if(stamp_div >= 5) f_hexFloat -= f_compensation;
    }
    
    u_hexFloat = (int32_t)(f_hexFloat*division);
    return u_hexFloat;
}

int32_t Handle_HexFloat_To_Int32_Round(uint32_t hexFloat, uint8_t scale)
{
    float f_hexFloat = 0;
    int32_t u_hexFloat = 0;
    
    uint32_t division = 1;
    uint32_t stamp_div = 0;
    float f_compensation = 0;
  
    Convert_uint32Hex_To_Float(hexFloat, &f_hexFloat);
    
    switch(scale)
    {
        case 0x00:
          division = 1;
          f_compensation = 0;
          break;
          
        case 0xFF:
          division = 10;
          f_compensation = 0.1;
          break;
          
        case 0xFE:
          division = 100;
          f_compensation = 0.01;
          break;
          
        case 0xFD:
          division = 1000;
          f_compensation = 0.001;
          break;
         
        case 0xFC:
          division = 10000;
          f_compensation = 0.0001;
          break;
          
        case 0xFB:
          division = 100000;
          f_compensation = 0.00001;
          break;
          
        default:
          break;
    }
   
    if(f_hexFloat >= 0)    
    {
      stamp_div = (uint32_t)(f_hexFloat*division*10)%10;
      if(stamp_div >= 5) f_hexFloat += f_compensation;
    }
    else                
    {
      stamp_div = (uint32_t)((0-f_hexFloat)*division*10)%10;
      if(stamp_div >= 5) f_hexFloat -= f_compensation;
    }
    
    u_hexFloat = (int32_t)(f_hexFloat*division);
    return u_hexFloat;
}

float Handle_int32_To_Float_Scale(int32_t var, uint8_t scale)
{
    float result = 0;
    switch(scale)
    {
        case 0x00:
          result = var;
          break;
          
        case 0xFF:
          result = var/10;
          result = result + (var%10)*0.1;
          break;
          
        case 0xFE:
          result = var/100;
          result = result + (var%100)*0.01;
          break;
          
        case 0xFD:
          result = var/1000;
          result = result + (var%1000)*0.001;
          break;
         
        case 0xFC:
          result = var/10000;
          result = result + (var%10000)*0.0001;
          break;
          
        case 0xFB:
          result = var/100000;
          result = result + (var%100000)*0.00001;
          break;
          
        default:
          break;
    }
    return result;
}

uint32_t Handle_Float_To_hexUint32(float num)
{
    uint32_t u_hexFloat = 0;
    unsigned char *bytePtr = (unsigned char*)&num;

    for (int i = 0; i < sizeof(float); i++) 
    {
        u_hexFloat = u_hexFloat | (bytePtr[i] << 8*i);
    }
    return u_hexFloat;
}

void RS485_Done_Calib(void)
{
    sHandleRs485.State_Wait_Calib = _STATE_CALIB_DONE;
    fevent_disable(sEventAppRs485, _EVENT_RS485_WAIT_CALIB);
}

void RS485_Enter_Calib(void)
{
    sKindMode485.Trans = Kind_Trans_Calib;
    sHandleRs485.State_Wait_Calib = _STATE_CALIB_WAIT;
    fevent_enable(sEventAppRs485, _EVENT_RS485_WAIT_CALIB);
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
    sHandleRs485.State_Wait_Calib = _STATE_CALIB_ENTER;
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

/*=================Handle array pH calib================*/
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
/*==================Handle Define AT command=================*/
#ifdef USING_AT_CONFIG
void AT_CMD_Get_State_Sensor(sData *str, uint16_t Pos)
{
  uint8_t aTemp[60] = "State_Sensor Clo:";   //11 ki tu dau tien
    uint16_t length = 17;

    Convert_Point_Int_To_String_Scale (aTemp, &length, (int)(sSensor_Clo.State_Connect), 0x00);
    Insert_String_To_String(aTemp, &length, (uint8_t*)" pH:",0 , 4);
    
    Convert_Point_Int_To_String_Scale (aTemp, &length, (int)(sSensor_pH.State_Connect), 0x00);
    Insert_String_To_String(aTemp, &length, (uint8_t*)" EC:",0 , 4);
    
    Convert_Point_Int_To_String_Scale (aTemp, &length, (int)(sSensor_EC.State_Connect), 0x00);
    Insert_String_To_String(aTemp, &length, (uint8_t*)" Turbidity:",0 , 11);
    
    Convert_Point_Int_To_String_Scale (aTemp, &length, (int)(sSensor_Turbidity.State_Connect), 0x00);
//    Insert_String_To_String(aTemp, &length, (uint8_t*)" \r\n",0 , 3);

	Modem_Respond(PortConfig, aTemp, length, 0);
}

void AT_CMD_Get_Measure_Value (sData *str_Receiv, uint16_t Pos)
{
    uint8_t aTemp[80] = "Measure_Value: Clo=";   //11 ki tu dau tien
    uint16_t length = 19;

    Convert_Point_Int_To_String_Scale (aTemp, &length, (int)(sDataSensorMeasure.sClo_Du.Value), sDataSensorMeasure.sClo_Du.Scale);
    Insert_String_To_String(aTemp, &length, (uint8_t*)"mg/L,pH=",0 , 8);
    
    Convert_Point_Int_To_String_Scale (aTemp, &length, (int)(sDataSensorMeasure.spH_Water.Value), sDataSensorMeasure.spH_Water.Scale);
    Insert_String_To_String(aTemp, &length, (uint8_t*)",Temp=",0 , 6);
    
    Convert_Point_Int_To_String_Scale (aTemp, &length, (int)(sDataSensorMeasure.sTemperature.Value), sDataSensorMeasure.sTemperature.Scale);
    Insert_String_To_String(aTemp, &length, (uint8_t*)"'C,EC=",0 , 6);
    
    Convert_Point_Int_To_String_Scale (aTemp, &length, (int)(sDataSensorMeasure.sEC.Value), sDataSensorMeasure.sEC.Scale);
    Insert_String_To_String(aTemp, &length, (uint8_t*)"uS/cm,Sali=",0 , 11);
    
    Convert_Point_Int_To_String_Scale (aTemp, &length, (int)(sDataSensorMeasure.sSalinity.Value), sDataSensorMeasure.sSalinity.Scale);
    Insert_String_To_String(aTemp, &length, (uint8_t*)"%,NTU=",0 , 6);
    
    Convert_Point_Int_To_String_Scale (aTemp, &length, (int)(sDataSensorMeasure.sNTU.Value), sDataSensorMeasure.sNTU.Scale);
    Insert_String_To_String(aTemp, &length, (uint8_t*)"NTU",0 , 3);

	Modem_Respond(PortConfig, aTemp, length, 0);
}

void AT_CMD_Get_Measure_Filter (sData *str_Receiv, uint16_t Pos)
{
    uint8_t aTemp[80] = "Measure_Filter: Clo=";   //11 ki tu dau tien
    uint16_t length = 20;

    Convert_Point_Int_To_String_Scale (aTemp, &length, (int)(sDataSensorMeasure.sClo_Du_Filter.Value), sDataSensorMeasure.sClo_Du_Filter.Scale);
    Insert_String_To_String(aTemp, &length, (uint8_t*)"mg/L,pH=",0 , 8);
    
    Convert_Point_Int_To_String_Scale (aTemp, &length, (int)(sDataSensorMeasure.spH_Water_Filter.Value), sDataSensorMeasure.spH_Water_Filter.Scale);
    Insert_String_To_String(aTemp, &length, (uint8_t*)",Temp=",0 , 6);
    
    Convert_Point_Int_To_String_Scale (aTemp, &length, (int)(sDataSensorMeasure.sTemperature_Filter.Value), sDataSensorMeasure.sTemperature_Filter.Scale);
    Insert_String_To_String(aTemp, &length, (uint8_t*)"'C,EC=",0 , 6);
    
    Convert_Point_Int_To_String_Scale (aTemp, &length, (int)(sDataSensorMeasure.sEC.Value), sDataSensorMeasure.sEC.Scale);
    Insert_String_To_String(aTemp, &length, (uint8_t*)"uS/cm,Sali=",0 , 11);
    
    Convert_Point_Int_To_String_Scale (aTemp, &length, (int)(sDataSensorMeasure.sSalinity_Filter.Value), sDataSensorMeasure.sSalinity_Filter.Scale);
    Insert_String_To_String(aTemp, &length, (uint8_t*)"%,NTU=",0 , 6);
    
    Convert_Point_Int_To_String_Scale (aTemp, &length, (int)(sDataSensorMeasure.sNTU_Filter.Value), sDataSensorMeasure.sNTU_Filter.Scale);
    Insert_String_To_String(aTemp, &length, (uint8_t*)"NTU",0 , 3);

	Modem_Respond(PortConfig, aTemp, length, 0);
}
#endif
/*==================Handle Task and Init app=================*/
void Init_UartRs485(void)
{
    RS485_Stop_RX_Mode();
    WM_DIG_Init_Uart(&uart_rs485, sWmDigVar.sModbInfor[0].MType_u8);
    RS485_Init_RX_Mode();
}

void       Init_AppRs485(void)
{
    Init_UartRs485();
    Init_IdSlave();
    Init_Parameter_Sensor();
#ifdef USING_AT_CONFIG
    /* regis cb serial */
    CheckList_AT_CONFIG[_GET_STATE_SENSOR].CallBack = AT_CMD_Get_State_Sensor;
    CheckList_AT_CONFIG[_GET_MEASURE_VALUE].CallBack = AT_CMD_Get_Measure_Value;
    CheckList_AT_CONFIG[_GET_MEASURE_FILTER].CallBack = AT_CMD_Get_Measure_Filter;
#endif
}

uint8_t        AppRs485_Task(void)
{
    uint8_t i = 0;
    uint8_t Result =  false;
    
    for(i = 0; i < _EVENT_RS485_END; i++)
    {
        if(sEventAppRs485[i].e_status == 1)
        {
            Result = true; 
            
            if((sEventAppRs485[i].e_systick == 0) ||
               ((HAL_GetTick() - sEventAppRs485[i].e_systick) >= sEventAppRs485[i].e_period))
            {
                sEventAppRs485[i].e_status = 0; //Disable event
                sEventAppRs485[i].e_systick= HAL_GetTick();
                sEventAppRs485[i].e_function_handler(i);
            }
        }
    }
    
    return Result;
}



