#include "user_app_rs485.h"
#include "user_app_modbus_rtu.h"
#include "user_define.h"

/*=========================Fucntion Static=========================*/
static uint8_t fevent_rs485_entry(uint8_t event);
static uint8_t fevent_rs485_init_uart(uint8_t event);
static uint8_t fevent_rs485_receive_complete(uint8_t event);
static uint8_t fevent_rs485_refresh(uint8_t event);
/*==============================Struct=============================*/
sEvent_struct               sEventAppRs485[]=
{
  {_EVENT_RS485_ENTRY,              1, 5, 5,                fevent_rs485_entry},            //Doi slave khoi dong moi truyen opera
  {_EVENT_RS485_INIT_UART,          1, 5, 5,                fevent_rs485_init_uart},
  {_EVENT_RS485_RECEIVE_COMPLETE,   0, 5, 20,               fevent_rs485_receive_complete},

  {_EVENT_RS485_REFRESH,            0, 5, 60000,            fevent_rs485_refresh},
};
extern sData sUart485;
uint16_t CountBufferHandleRecv = 0;
/*========================Function Handle========================*/
static uint8_t fevent_rs485_entry(uint8_t event)
{
    return 1;
}

static uint8_t fevent_rs485_init_uart(uint8_t event)
{
    static uint8_t stamp = 0;
    if(stamp != sSlave_ModbusRTU.Baudrate)
    {
        Init_UartRs485(aBaudrate_value[sSlave_ModbusRTU.Baudrate]);
        stamp = sSlave_ModbusRTU.Baudrate;
    }
    fevent_enable(sEventAppRs485, event);
    return 1;
}


static uint8_t fevent_rs485_receive_complete(uint8_t event)
{
/*------------------Xu ly chuoi nhan duoc----------------*/
    Modem_Check_RTU(&sUart485);
    Reset_Buff(&sUart485);
    return 1;
}

static uint8_t fevent_rs485_refresh(uint8_t event)
{
    return 1;
}

/*==================Handle Task and Init app=================*/
void Init_UartRs485(uint32_t Baudrate)
{
//    Init_Uart485(115200);
  huart1.Instance = USART1;
  huart1.Init.BaudRate = Baudrate;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  
  RS485_Init_RX_Mode();
}

void       Init_AppRs485(void)
{
    Init_InforSlaveModbusRTU();
    Init_UartRs485(aBaudrate_value[sSlave_ModbusRTU.Baudrate]);
//    Init_IdSlave();
//    Init_Parameter_Sensor();
#ifdef USING_AT_CONFIG
    /* regis cb serial */
//    CheckList_AT_CONFIG[_GET_STATE_SENSOR].CallBack = AT_CMD_Get_State_Sensor;
//    CheckList_AT_CONFIG[_GET_MEASURE_VALUE].CallBack = AT_CMD_Get_Measure_Value;
//    CheckList_AT_CONFIG[_GET_MEASURE_FILTER].CallBack = AT_CMD_Get_Measure_Filter;
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
