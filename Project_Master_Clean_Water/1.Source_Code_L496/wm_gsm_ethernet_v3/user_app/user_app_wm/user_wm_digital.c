


#include "user_wm_digital.h"
#include "user_obis_sv.h"
#include "user_json.h"


sWMDigitalInfor sListWmDigital[6] = 
{
    {   __MET_WOTECK,       0,      20,    { 9600, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE },      Woteck_Decode},  
    {   __MET_LEVEL_LIQ,    0,      6,      { 9600, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE },      Level_S_YW_Decode},
    {   __MET_LEVEL_LIQ_SUP,0,      6,      { 9600, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE },      Level_SUP_Decode},
    {   __MET_MT100,        4112,   22,     { 9600, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE },      MT100_Decode},
    {   __MET_LEVEL_ULTRA,  0x0100, 4,      { 9600, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE },      Level_Ultra_Decode},
    {   __MET_UNKNOWN,      0,      1,      { 9600, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE },      NULL},
};


sWMDigitalVariable       sWmDigVar = 
{
    .nModbus_u8 = 2,
    .sModbInfor[0].MType_u8 = __MET_WOTECK,
    .sModbInfor[0].SlaveId_u8 = 0x03,
        
    .sModbInfor[1].MType_u8 = __MET_WOTECK,
    .sModbInfor[1].SlaveId_u8 = 0x03,
    
    .sModbDevData[0] =
    {
        .Status_u8 = false,
        .Type_u8 = 0,
        .nTotal_i32 = 234,
        .nForw_u32 = 5667,
        .nRev_u32 = 9789,
        
        .Pressure_i32 = -1,
        
        .Flow_i32 = 96,
        .Factor = 0xFD,
        .sFlowUnit = " (l/s)",
        .sTotalUnit = " (m3)",
    },

    .sModbDevData[1] =
    {
        .Status_u8 = false,
        .Type_u8 = 0,
        
        .nTotal_i32 = 256,
        .nForw_u32 = 125,
        .nRev_u32 = 258,
        
        .Pressure_i32 = -1,
        
        .Flow_i32 = 126,
        .Factor = 0xFD,
        .sFlowUnit = " (l/s)",
        .sTotalUnit = " (m3)",
    },
};


/*============= Function =============*/
void WM_DIG_Init_Uart (UART_HandleTypeDef *huart, uint8_t type)
{
    if (huart->Instance == uart_rs485.Instance)
    {
        uart_rs485.Init.BaudRate = sListWmDigital[type].sUart.BaudRate;
        uart_rs485.Init.WordLength = sListWmDigital[type].sUart.WordLength;
        uart_rs485.Init.StopBits = sListWmDigital[type].sUart.StopBits;
        uart_rs485.Init.Parity = sListWmDigital[type].sUart.Parity;
        uart_rs485.Init.Mode = UART_MODE_TX_RX;
        uart_rs485.Init.HwFlowCtl = UART_HWCONTROL_NONE;
        uart_rs485.Init.OverSampling = UART_OVERSAMPLING_16;
        uart_rs485.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
        uart_rs485.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
        if (HAL_UART_Init(&uart_rs485) != HAL_OK)
        {
            Error_Handler();
        }
    } else if (huart->Instance == uart_rs485_2.Instance)
    {
        uart_rs485_2.Init.BaudRate = sListWmDigital[type].sUart.BaudRate;
        uart_rs485_2.Init.WordLength = sListWmDigital[type].sUart.WordLength;
        uart_rs485_2.Init.StopBits = sListWmDigital[type].sUart.StopBits;
        uart_rs485_2.Init.Parity = sListWmDigital[type].sUart.Parity;
        uart_rs485_2.Init.Mode = UART_MODE_TX_RX;
        uart_rs485_2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
        uart_rs485_2.Init.OverSampling = UART_OVERSAMPLING_16;
        uart_rs485_2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
        uart_rs485_2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
        if (HAL_UART_Init(&uart_rs485_2) != HAL_OK)
        {
            Error_Handler();
        }
    }
}




void WM_DIG_Get_Data (uint8_t chann, uint8_t type)
{
    float TempFloat = 0;
    
    switch (type)
    {
        case __MET_WOTECK:
            sWmDigVar.sModbDevData[chann].Status_u8 = true;
            sWmDigVar.sModbDevData[chann].Type_u8 = type;
            
            sWmDigVar.sModbDevData[chann].nTotal_i32  = (uint32_t ) (sWoteck.TotalVal_f * 1000);
            sWmDigVar.sModbDevData[chann].nRev_u32    = (uint32_t ) (sWoteck.TotalRev_f * 1000);
            sWmDigVar.sModbDevData[chann].nForw_u32   = (uint32_t ) (sWoteck.TotalForw_f * 1000);
            sWmDigVar.sModbDevData[chann].Flow_i32    = (uint32_t ) (sWoteck.Flow_f * 1000);
            
            sWmDigVar.sModbDevData[chann].sFlowUnit   = " (m3/h)";
            sWmDigVar.sModbDevData[chann].sTotalUnit  = " (m3)";
            
            sWmDigVar.sModbDevData[chann].Factor      = 0xFD;
            
            break;
        case __MET_LEVEL_LIQ:
            sWmDigVar.sModbDevData[chann].Status_u8 = true;
            sWmDigVar.sModbDevData[chann].Type_u8 = type;

            sWmDigVar.sModbDevData[chann].LVal_i16  = sLevelInfor.Value_i16;
            sWmDigVar.sModbDevData[chann].LDecimal_u16 = sLevelInfor.Decimal_u16;
            sWmDigVar.sModbDevData[chann].LUnit_u16 = sLevelInfor.Unit_u16;
            break;
        case __MET_LEVEL_LIQ_SUP:
            sWmDigVar.sModbDevData[chann].Status_u8 = true;
            sWmDigVar.sModbDevData[chann].Type_u8 = type;

            sWmDigVar.sModbDevData[chann].LVal_i16  = sLevelSupInfor.Value_i16;
            sWmDigVar.sModbDevData[chann].LDecimal_u16 = sLevelSupInfor.Decimal_u16;
            sWmDigVar.sModbDevData[chann].LUnit_u16 = sLevelSupInfor.Unit_u16;
            break;
        case __MET_MT100:
            sWmDigVar.sModbDevData[chann].Status_u8 = true;
            sWmDigVar.sModbDevData[chann].Type_u8 = type;

            TempFloat = (sMT100.PosInt_i32 + sMT100.PosDec_f) - (sMT100.NegInt_i32 + sMT100.NegDec_f);
            sWmDigVar.sModbDevData[chann].nTotal_i32   = (int32_t ) (TempFloat * 1000);
            
            TempFloat = sMT100.PosInt_i32 + sMT100.PosDec_f;
            sWmDigVar.sModbDevData[chann].nForw_u32   = (uint32_t ) (TempFloat * 1000);
            
            TempFloat = sMT100.NegInt_i32 + sMT100.NegDec_f;
            sWmDigVar.sModbDevData[chann].nRev_u32    = (uint32_t ) (TempFloat * 1000);
            
            sWmDigVar.sModbDevData[chann].Flow_i32    = (uint32_t ) (sMT100.Flow_f * 1000);
            
            sWmDigVar.sModbDevData[chann].sFlowUnit   = MT100_Decode_Unit(0);
            sWmDigVar.sModbDevData[chann].sTotalUnit  = MT100_Decode_Unit(1);
            
            sWmDigVar.sModbDevData[chann].Factor      = 0xFD;
            break;
        case __MET_LEVEL_ULTRA:
            sWmDigVar.sModbDevData[chann].Status_u8 = true;
            sWmDigVar.sModbDevData[chann].Type_u8 = type;

            sWmDigVar.sModbDevData[chann].LVal_i16  = sLevelUltra.Value_i16;
            sWmDigVar.sModbDevData[chann].LDecimal_u16 = sLevelUltra.Decimal_u16;
            sWmDigVar.sModbDevData[chann].LUnit_u16 = sLevelUltra.Unit_u16;
            break;
        default:
            break;
    }
}



void WM_DIG_Packet_Mess (sData *pData, uint8_t chann)
{
    int32_t Flow_i32 = 0;
    uint16_t TempU16 = 0;  
    
    switch (sWmDigVar.sModbDevData[chann].Type_u8)
    {
        case __MET_WOTECK:
        case __MET_MT100:
            //total forward
        #ifdef PAYLOAD_HEX_FORMAT
            SV_Protocol_Packet_Data(pData->Data_a8, &pData->Length_u16, 
                                    OBIS_WM_PULSE_FORWARD, &sWmDigVar.sModbDevData[chann].nForw_u32, 4, sWmDigVar.sModbDevData[chann].Factor);
        #else
            Json_Add_Object_Number(pData, (uint8_t *) "pulse forward", strlen("pulse forward"), 
                                   &sWmDigVar.sModbDevData[chann].nForw_u32, sWmDigVar.sModbDevData[chann].Factor, 1);
        #endif
            //total revert
        #ifdef PAYLOAD_HEX_FORMAT
            SV_Protocol_Packet_Data(pData->Data_a8, &pData->Length_u16, 
                                    OBIS_WM_PULSE_REVERSE, &sWmDigVar.sModbDevData[chann].nRev_u32, 4, sWmDigVar.sModbDevData[chann].Factor);
        #else
            Json_Add_Object_Number(pData, (uint8_t *) "pulse reverse", strlen("pulse reverse"), 
                                   &sWmDigVar.sModbDevData[chann].nRev_u32, sWmDigVar.sModbDevData[chann].Factor, 1);
        #endif
            //total consump
        #ifdef PAYLOAD_HEX_FORMAT
            SV_Protocol_Packet_Data(pData->Data_a8, &pData->Length_u16, 
                                    OBIS_WM_PULSE, &sWmDigVar.sModbDevData[chann].nTotal_i32, 4, sWmDigVar.sModbDevData[chann].Factor);
        #else
            Json_Add_Object_Number(pData, (uint8_t *) "pulse total", strlen("pulse total"), 
                                   &sWmDigVar.sModbDevData[chann].nTotal_i32, sWmDigVar.sModbDevData[chann].Factor, 1);
        #endif
            //flow
            //tinh lai ra m3/h
            if (sWmDigVar.sModbDevData[chann].Type_u8 == __MET_MT100)
            {
                Flow_i32 = (int32_t) MT100_Convert_Flow_m3h ((float)sWmDigVar.sModbDevData[chann].Flow_i32);
            } else
            {
                Flow_i32 = sWmDigVar.sModbDevData[chann].Flow_i32;
            }
            //
        #ifdef PAYLOAD_HEX_FORMAT
            SV_Protocol_Packet_Data(pData->Data_a8, &pData->Length_u16, 
                                    OBIS_WM_DIG_FLOW, &Flow_i32, 4, sWmDigVar.sModbDevData[chann].Factor);
        #else
            Json_Add_Object_Number(pData, (uint8_t *) "flow", strlen("flow"), 
                                   &Flow_i32, sWmDigVar.sModbDevData[chann].Factor, 1);
        #endif 
            break;
        case __MET_LEVEL_LIQ:
        case __MET_LEVEL_LIQ_SUP:
        case __MET_LEVEL_ULTRA:
            #ifdef PAYLOAD_HEX_FORMAT
                SV_Protocol_Packet_Data(pData->Data_a8, &pData->Length_u16, OBIS_WM_PULSE, &TempU16, 2, 0x00);

                //----------Level Unit --------------------
                SV_Protocol_Packet_Data(pData->Data_a8, &pData->Length_u16, OBIS_WM_LEVEL_UNIT, &sWmDigVar.sModbDevData[chann].LUnit_u16, 1, 0xAA);
   
                //----------Level Value Sensor --------------------
                SV_Protocol_Packet_Data(pData->Data_a8, &pData->Length_u16, OBIS_WM_LEVEL_VAL_REAL, &sWmDigVar.sModbDevData[chann].LVal_i16, 2, 
                                                0 - (uint8_t)sWmDigVar.sModbDevData[chann].LDecimal_u16);
             #else
                Json_Add_Object_Number(pData, (uint8_t *) "level", strlen("level"), 
                                                       sWmDigVar.sModbDevData[chann].LVal_i16, sWmDigVar.sModbDevData[chann].LDecimal_u16, 1);
            #endif
            break;
        default:
            break;
    }
}

