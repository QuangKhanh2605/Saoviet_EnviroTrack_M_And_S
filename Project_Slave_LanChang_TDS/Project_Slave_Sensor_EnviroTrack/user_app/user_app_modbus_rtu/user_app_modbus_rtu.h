#ifndef __USER_APP_MODBUS_RTU_
#define __USER_APP_MODBUS_RTU_

#include "user_define.h"
#define USING_CHECK_MODBUS_RTU

#define ID_DEFAULT          3
#define BAUDRATE_DEFAULT    3       //9600

typedef enum
{
    _E_REGISTER_BEGIN,
    _E_REGISTER_ID,
    _E_REGISTER_BAUDRATE,
    _E_REGISTER_VALUE_MEASURE_1,
    _E_REGISTER_VALUE_MEASURE_2,
    _E_REGISTER_VALUE_MEASURE_3,
    _E_REGISTER_VALUE_MEASURE_4,
    
    _E_REGISTER_STATE_SENSOR,
    _E_REGISTER_STATE_VALUE_1,
    _E_REGISTER_STATE_VALUE_2,
    _E_REGISTER_STATE_VALUE_3,
    _E_REGISTER_STATE_VALUE_4,
    
    _E_REGISTER_ALARM_STATE,
    _E_REGISTER_ALARM_UPPER,
    _E_REGISTER_ALARM_LOWER,

    _E_REGISTER_END,
}eRegister_ModbusRTU;

typedef uint8_t (*_func_callback_handle)(sData *str_Receiv, uint16_t Pos);

typedef struct {
	int 			    idStep;
    int                 idRegister;
    int                 Length;
	_func_callback_handle	CallBack_Read;
    _func_callback_handle	CallBack_Write;
}struct_CheckList_Reg_Modbus_RTU;

typedef struct {
    uint8_t ID;
    uint8_t Baudrate;
}struct_Slave_ModbusRTU;

extern struct_CheckList_Reg_Modbus_RTU sCheckList_Reg_Modbus_RTU[];
extern sData   sLogData_ModbusRTU;
extern sData   sFrameData_ModbusRTU;
extern struct_Slave_ModbusRTU sSlave_ModbusRTU;
extern uint32_t aBaudrate_value[11];
/*====================Function Handle====================*/
void Save_InforSlaveModbusRTU(uint8_t ID, uint8_t Baudrate);
void Init_InforSlaveModbusRTU(void);

void Reset_sData(sData *str);
uint8_t Modem_Check_RTU(sData *StrUartRecei);
void    Send_RS458_Master(uint8_t *aData, uint16_t Length_u16);

#endif
