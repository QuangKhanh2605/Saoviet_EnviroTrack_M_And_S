#include "user_app_modbus_rtu.h"
#include "user_modbus_rtu.h"
#include "user_convert_variable.h"

uint8_t NONE_Register_CallBack(sData *str, uint16_t Pos);

uint8_t _Cb_R_ModbusRTU_REG_ID(sData *str, uint16_t Pos);
uint8_t _Cb_W_ModbusRTU_REG_ID(sData *str, uint16_t Pos);

uint8_t _Cb_R_ModbusRTU_REG_Baudrate(sData *str, uint16_t Pos);
uint8_t _Cb_W_ModbusRTU_REG_Baudrate(sData *str, uint16_t Pos);

uint8_t _Cb_R_ModbusRTU_REG_pH_Measure(sData *str, uint16_t Pos);
uint8_t _Cb_R_ModbusRTU_REG_Temp_Measure(sData *str, uint16_t Pos);

/*============================ Struct var ============================*/
struct_CheckList_Reg_Modbus_RTU sCheckList_Reg_Modbus_RTU[] =
{
//      id ENUM                     id Reg      Length          Cb_Read         Cb_Write
      {_E_REGISTER_BEGIN,           NULL,       NULL,     NONE_Register_CallBack,           NONE_Register_CallBack},
      {_E_REGISTER_ID,              0x0000,     1,        _Cb_R_ModbusRTU_REG_ID,           _Cb_W_ModbusRTU_REG_ID},
      {_E_REGISTER_BAUDRATE,        0x0001,     1,        _Cb_R_ModbusRTU_REG_Baudrate,     _Cb_W_ModbusRTU_REG_Baudrate},
      {_E_REGISTER_PH_MEASURE,      0x0002,     2,        _Cb_R_ModbusRTU_REG_pH_Measure,   NONE_Register_CallBack},
      {_E_REGISTER_TEMP_MEASURE,    0x0004,     2,        _Cb_R_ModbusRTU_REG_Temp_Measure, NONE_Register_CallBack},
};
static uint8_t aDATA_CONFIG[128];

uint32_t aBaudrate_value[11]={1200,2400,4800,9600,14400,19200,28800,38400,56000,57600,115200};
struct_Slave_ModbusRTU sSlave_ModbusRTU = {1,3};

static uint8_t aDATA_LOG_REGISTER[128];
static uint8_t aDATA_LOG_FRAME[128];

sData   sLogData_ModbusRTU = {aDATA_LOG_REGISTER,0};
sData   sFrameData_ModbusRTU = {aDATA_LOG_FRAME,0};

/*===================== Handle Callback Reg Modbus RTU ===================*/
uint8_t NONE_Register_CallBack(sData *str, uint16_t Pos)
{
    return 0;
}

/*----------- _E_REGISTER_ID -----------*/
uint8_t _Cb_R_ModbusRTU_REG_ID(sData *str, uint16_t Pos)
{
    sLogData_ModbusRTU.Data_a8[sLogData_ModbusRTU.Length_u16++] = 0;
    sLogData_ModbusRTU.Data_a8[sLogData_ModbusRTU.Length_u16++] = sSlave_ModbusRTU.ID;
    return 1;
}

uint8_t _Cb_W_ModbusRTU_REG_ID(sData *str, uint16_t Pos)
{
    uint16_t ConvertData = 0;
    uint8_t pos = 0;
    pos = Pos;
    ConvertData = str->Data_a8[pos] << 8 | str->Data_a8[pos+1]; 
    if(ConvertData < 256 && ConvertData > 0)
    {
        sLogData_ModbusRTU.Data_a8[sLogData_ModbusRTU.Length_u16++] = str->Data_a8[pos];
        sLogData_ModbusRTU.Data_a8[sLogData_ModbusRTU.Length_u16++] = str->Data_a8[pos+1];
        
        Save_InforSlaveModbusRTU( (uint8_t)ConvertData,sSlave_ModbusRTU.Baudrate);
        return 1;
    }
    return 0;
}

/*----------- _E_REGISTER_BAUDRATE -----------*/
uint8_t _Cb_R_ModbusRTU_REG_Baudrate(sData *str, uint16_t Pos)
{
    sLogData_ModbusRTU.Data_a8[sLogData_ModbusRTU.Length_u16++] = 0;
    sLogData_ModbusRTU.Data_a8[sLogData_ModbusRTU.Length_u16++] = sSlave_ModbusRTU.Baudrate;
    return 1;
}

uint8_t _Cb_W_ModbusRTU_REG_Baudrate(sData *str, uint16_t Pos)
{
    uint16_t ConvertData = 0;
    uint8_t pos = 0;
    pos = Pos;
    ConvertData = str->Data_a8[pos] << 8 | str->Data_a8[pos+1]; 
    if(ConvertData < 11)
    {
        sLogData_ModbusRTU.Data_a8[sLogData_ModbusRTU.Length_u16++] = str->Data_a8[pos];
        sLogData_ModbusRTU.Data_a8[sLogData_ModbusRTU.Length_u16++] = str->Data_a8[pos+1];
        Save_InforSlaveModbusRTU( sSlave_ModbusRTU.ID, (uint8_t)ConvertData);
        return 1;
    }
    return 0;
}

/*----------- _E_REGISTER_TEMP_OBJECT -----------*/
uint8_t _Cb_R_ModbusRTU_REG_pH_Measure(sData *str, uint16_t Pos)
{
    uint32_t hexValue = 0;
    hexValue = Handle_Float_To_hexUint32(sSensor_Turb.Turb_Value_f);
    sLogData_ModbusRTU.Data_a8[sLogData_ModbusRTU.Length_u16++] = hexValue >> 8;
    sLogData_ModbusRTU.Data_a8[sLogData_ModbusRTU.Length_u16++] = hexValue;
    sLogData_ModbusRTU.Data_a8[sLogData_ModbusRTU.Length_u16++] = hexValue >> 24;
    sLogData_ModbusRTU.Data_a8[sLogData_ModbusRTU.Length_u16++] = hexValue >> 16;
    return 1;
}

/*----------- _E_REGISTER_TEMP_AMBIENT -----------*/
uint8_t _Cb_R_ModbusRTU_REG_Temp_Measure(sData *str, uint16_t Pos)
{
    uint32_t hexValue = 0;
    hexValue = Handle_Float_To_hexUint32(sSensor_Turb.temp_Value_f);
    sLogData_ModbusRTU.Data_a8[sLogData_ModbusRTU.Length_u16++] = hexValue >> 8;
    sLogData_ModbusRTU.Data_a8[sLogData_ModbusRTU.Length_u16++] = hexValue;
    sLogData_ModbusRTU.Data_a8[sLogData_ModbusRTU.Length_u16++] = hexValue >> 24;
    sLogData_ModbusRTU.Data_a8[sLogData_ModbusRTU.Length_u16++] = hexValue >> 16;
    return 1;
}

/*===================== Send Data RS485 ======================*/
/*
    @brief Send 485 sensor
*/
void        Send_RS458_Master(uint8_t *aData, uint16_t Length_u16) 
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
//    RS485_Init_Data();
    HAL_UART_Transmit(&uart_rs485, aData , Length_u16, 1000); 
    
    //Dua DE ve Receive
    HAL_GPIO_WritePin(DE_GPIO_PORT, DE_GPIO_PIN, GPIO_PIN_RESET);
}

/*====================== Handle Function =======================*/
void Reset_sData(sData *str)
{
	uint16_t i;
    
	for (i = 0; i < str->Length_u16; i++)
		*(str->Data_a8 +i ) = 0;
    
	str->Length_u16 = 0;
}
/*======================= Function ID Slave =======================*/
void Save_InforSlaveModbusRTU(uint8_t ID, uint8_t Baudrate)
{
#ifdef USING_CHECK_MODBUS_RTU
    uint8_t aData[8] = {0};
    uint8_t length = 0;
    
    sSlave_ModbusRTU.ID = ID;
    sSlave_ModbusRTU.Baudrate = Baudrate;
    
    aData[length++] = sSlave_ModbusRTU.ID;
    aData[length++] = sSlave_ModbusRTU.Baudrate;

    Save_Array(ADDR_INFOR_SLAVE_MODBUS_RTU, aData, length);
#endif
}

void Init_InforSlaveModbusRTU(void)
{
#ifdef USING_CHECK_MODBUS_RTU
    if(*(__IO uint8_t*)(ADDR_INFOR_SLAVE_MODBUS_RTU) != FLASH_BYTE_EMPTY)
    {
        sSlave_ModbusRTU.ID  = *(__IO uint8_t*)(ADDR_INFOR_SLAVE_MODBUS_RTU+2);
        sSlave_ModbusRTU.Baudrate = *(__IO uint8_t*)(ADDR_INFOR_SLAVE_MODBUS_RTU+3);
    }
    else
    {
        sSlave_ModbusRTU.ID = ID_DEFAULT;
        sSlave_ModbusRTU.Baudrate = BAUDRATE_DEFAULT;
    }
#endif    
}
    uint8_t aframe[128] = {0};
/*====================== Handle Receive Modbus RTU =======================*/
uint8_t Modem_Check_RTU(sData *StrUartRecei)
{
#ifdef USING_CHECK_MODBUS_RTU
    uint16_t Crc_Check = 0;
    uint16_t Crc_Recv = 0;
    uint16_t Register = 0;
    uint16_t Length = 0;
    
	sData sDataConfig = {aDATA_CONFIG, 0};
    uint16_t var = 0;
    

    sData sFrame={aframe,0};
    
    Reset_Buff(&sLogData_ModbusRTU);
    
    if(StrUartRecei->Length_u16 >= 8)
    {
        Crc_Recv = (StrUartRecei->Data_a8[StrUartRecei->Length_u16 - 1] << 8) |
                   (StrUartRecei->Data_a8[StrUartRecei->Length_u16 - 2]);
        Crc_Check = ModRTU_CRC(StrUartRecei->Data_a8, StrUartRecei->Length_u16 - 2);
        
        if(Crc_Check == Crc_Recv && StrUartRecei->Data_a8[0] == sSlave_ModbusRTU.ID)
        {
            Register = (StrUartRecei->Data_a8[2] << 8) | StrUartRecei->Data_a8[3];
            switch(StrUartRecei->Data_a8[1])
            {
                case 0x03:
                  Length = (StrUartRecei->Data_a8[4] << 8) | StrUartRecei->Data_a8[5];
                  for(uint8_t i = _E_REGISTER_BEGIN+1; i < _E_REGISTER_END; i++)
                  {
                    if(sCheckList_Reg_Modbus_RTU[i].idRegister == Register)
                    {
                       if(sCheckList_Reg_Modbus_RTU[i].Length <= Length)
                       {
                           if(sCheckList_Reg_Modbus_RTU[i].CallBack_Read(NULL, NULL) == 1)
                           {
                               Length -= sCheckList_Reg_Modbus_RTU[i].Length;
                               Register += sCheckList_Reg_Modbus_RTU[i].Length;
                           }
                           else
                             break;
                       }
                    }
                  }
                  if(sLogData_ModbusRTU.Length_u16 > 0 && sLogData_ModbusRTU.Length_u16%2 == 0)
                  {
                    ModRTU_Slave_ACK_Read_Frame(&sFrame, StrUartRecei->Data_a8[0], 0x03, Register, sLogData_ModbusRTU.Length_u16/2, sLogData_ModbusRTU.Data_a8);
                    Send_RS458_Master(sFrame.Data_a8, sFrame.Length_u16);
                  }
                  break;
                  
                case 0x06:
                  for(uint8_t i = 4; i < StrUartRecei->Length_u16-2; i++)
                  {
                    sDataConfig.Data_a8[sDataConfig.Length_u16++] = StrUartRecei->Data_a8[i];
                  }
                  for(uint8_t i = _E_REGISTER_BEGIN+1; i < _E_REGISTER_END; i++)
                  {
                    if(sCheckList_Reg_Modbus_RTU[i].idRegister == Register)
                    {
                      sCheckList_Reg_Modbus_RTU[i].CallBack_Write(&sDataConfig, 0);
                    }
                  }         
                  if(sLogData_ModbusRTU.Length_u16 > 0 && sLogData_ModbusRTU.Length_u16%2 == 0)
                  {
                    ModRTU_Slave_ACK_Write_Frame(&sFrame, StrUartRecei->Data_a8[0], 0x06, Register, sLogData_ModbusRTU.Length_u16/2, sLogData_ModbusRTU.Data_a8);
                    Send_RS458_Master(sFrame.Data_a8, sFrame.Length_u16);
                  }
                  break;
                  
                case 0x10:
                  var = 0;
                  Length = (StrUartRecei->Data_a8[4] << 8) | StrUartRecei->Data_a8[5];
                  for(uint8_t i = 7; i < StrUartRecei->Length_u16-2; i++)
                  {
                    sDataConfig.Data_a8[sDataConfig.Length_u16++] = StrUartRecei->Data_a8[i];
                  }
                  if((Length*2 == StrUartRecei->Data_a8[6]) && (sDataConfig.Length_u16 == StrUartRecei->Data_a8[6]))
                  {
                      for(uint8_t i = _E_REGISTER_BEGIN+1; i < _E_REGISTER_END; i++)
                      {
                        if(sCheckList_Reg_Modbus_RTU[i].idRegister == Register)
                        {
                           if(sCheckList_Reg_Modbus_RTU[i].Length <= Length)
                           {
                               if(sCheckList_Reg_Modbus_RTU[i].CallBack_Write(&sDataConfig, var) == 1)
                               {
                                   var += (sCheckList_Reg_Modbus_RTU[i].Length)*2;
                                   Length -= sCheckList_Reg_Modbus_RTU[i].Length;
                                   Register += sCheckList_Reg_Modbus_RTU[i].Length;
                               }
                               else 
                                 break;
                           }
                        }
                      }   
                  }
                  if(sLogData_ModbusRTU.Length_u16 > 0 && sLogData_ModbusRTU.Length_u16%2 == 0)
                  {
                    Register = (StrUartRecei->Data_a8[2] << 8) | StrUartRecei->Data_a8[3];
                    ModRTU_Slave_ACK_Write_Frame(&sFrame, StrUartRecei->Data_a8[0], 0x10, Register, sLogData_ModbusRTU.Length_u16/2, sLogData_ModbusRTU.Data_a8);
                    Send_RS458_Master(sFrame.Data_a8, sFrame.Length_u16);
                  }
                  break;
              
                default:
                  return 0;
                  break;
            }
        }
        else
            return 0;
    }
    else
        return 0;
  
    return 1;
#endif
    return 0;
}


