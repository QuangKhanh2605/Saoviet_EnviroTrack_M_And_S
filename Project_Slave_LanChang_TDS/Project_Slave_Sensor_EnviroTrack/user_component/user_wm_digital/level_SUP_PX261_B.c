


#include "level_SUP_PX261_B.h"



sLevelSupParameter sLevelSupInfor;

int16_t LevelSupRealVal_i16 = 0;
int16_t LevelSupCalibVal_i16 = 0;


void Level_SUP_Decode (sData *pData)
{
    uint16_t TempU16 = 0;   
    
    sLevelSupInfor.Unit_u16 = *(pData->Data_a8 + SUP_REG_UNIT * 2) * 256 + *(pData->Data_a8 + SUP_REG_UNIT * 2 + 1);
    sLevelSupInfor.Decimal_u16 = *(pData->Data_a8 + SUP_REG_DECIMAL * 2) * 256 + *(pData->Data_a8 + SUP_REG_DECIMAL * 2 + 1);
    
    TempU16 = *(pData->Data_a8 + SUP_REG_VALUE * 2) * 256 + *(pData->Data_a8 + SUP_REG_VALUE * 2 + 1);
    sLevelSupInfor.Value_i16 = (int16_t) TempU16;
    
    switch (sLevelSupInfor.Unit_u16)
    {
        case 0:
        default:
            sLevelSupInfor.Unit_u16 = 1;
            break;
    }
}




uint32_t Level_SUP_Reverse_Decimal (uint16_t Decimal)
{
    uint32_t Result = 1;
    
    for (uint16_t i = 0; i < Decimal; i++)
        Result *= 10;
    
    return Result;
}

/*
    Func: Convert gia tri calib tu mm -> Value with decimal and Unit curent
*/
int16_t Level_SUP_Convert_Value_Calib (int16_t Calib_mm, uint16_t Decimal, uint16_t Unit)
{
    int16_t CalibVal = 0;
    
    CalibVal = Level_SUP_Reverse_Decimal (Decimal) * Calib_mm;
        
    if (Unit == 1)
    {
        CalibVal = CalibVal / 10;
    } 
    
    return CalibVal;
}





