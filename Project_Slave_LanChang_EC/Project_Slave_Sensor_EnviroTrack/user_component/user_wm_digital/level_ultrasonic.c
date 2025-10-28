


#include "level_ultrasonic.h"


sLevelUltrasonic sLevelUltra;


void Level_Ultra_Decode (sData *pData)
{
    sLevelUltra.Unit_u16 = 1;
    sLevelUltra.Decimal_u16 = 1;

    sLevelUltra.Value_i16 = *(pData->Data_a8 + REGIS_REALTIME_VAL * 2) * 256 + *(pData->Data_a8 + REGIS_REALTIME_VAL * 2 + 1);;

    sLevelUltra.ZeroPoint_i16 = 0;
}




