


#ifndef LEVEL_ULTRASONIC_H
#define LEVEL_ULTRASONIC_H


#include "user_util.h"


/*=== Register liquid level S_YW_01B ===*/

#define REGIS_CAL_VAL	    0
#define REGIS_REALTIME_VAL  1
#define REGIS_TEMP  	    2

typedef struct
{
    uint16_t        Unit_u16;
    uint16_t        Decimal_u16;
    int16_t         Value_i16;
    int16_t         ZeroPoint_i16;
}sLevelUltrasonic;

extern sLevelUltrasonic sLevelUltra;
/*======================== Funcion ======================*/

void        Level_Ultra_Decode (sData *pData);

#endif
