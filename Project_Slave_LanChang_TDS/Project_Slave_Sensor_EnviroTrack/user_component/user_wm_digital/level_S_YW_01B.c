


#include "level_S_YW_01B.h"


sLevelParameter sLevelInfor;


void Level_S_YW_Decode (sData *pData)
{
    uint16_t TempU16 = 0;   
    
    sLevelInfor.Unit_u16 = *(pData->Data_a8 + REGIS_UNIT * 2) * 256 + *(pData->Data_a8 + REGIS_UNIT * 2 + 1);
    sLevelInfor.Decimal_u16 = *(pData->Data_a8 + REGIS_DECIMAL * 2) * 256 + *(pData->Data_a8 + REGIS_DECIMAL * 2 + 1);
    
    TempU16 = *(pData->Data_a8 + REGIS_VALUE * 2) * 256 + *(pData->Data_a8 + REGIS_VALUE * 2 + 1);
    sLevelInfor.Value_i16 = (int16_t) TempU16;

    TempU16 = *(pData->Data_a8 + REGIS_ZERO_POINT * 2) * 256 + *(pData->Data_a8 + REGIS_ZERO_POINT * 2 + 1);
    sLevelInfor.ZeroPoint_i16 = (int16_t ) TempU16;
}




uint32_t Level_S_YW_Reverse_Decimal (uint16_t Decimal)
{
    uint32_t Result = 1;
    
    for (uint16_t i = 0; i < Decimal; i++)
        Result *= 10;
    
    return Result;
}

/*
    Func: Convert gia tri calib tu mm -> Value with decimal and Unit curent
*/
int16_t Level_S_YW_Convert_Value_Calib (int16_t Calib_mm, uint16_t Decimal, uint16_t Unit)
{
    int16_t CalibVal = 0;
    
    CalibVal = Level_S_YW_Reverse_Decimal (Decimal) * Calib_mm;
        
    if (Unit == 1)
    {
        CalibVal = CalibVal / 10;
    } 
    
    return CalibVal;
}





