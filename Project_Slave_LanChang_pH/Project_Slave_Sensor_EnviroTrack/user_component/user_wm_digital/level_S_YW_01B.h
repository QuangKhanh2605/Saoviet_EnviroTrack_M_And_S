


#ifndef LEVEL_S_YW_01B_H
#define LEVEL_S_YW_01B_H


#include "user_util.h"


/*=== Register liquid level S_YW_01B ===*/

#define REGIS_SLAVE_ID	    0x0000
#define REGIS_BAURATE	    0x0001
#define REGIS_UNIT  	    0x0002
#define REGIS_DECIMAL	    0x0003
#define REGIS_VALUE 	    0x0004
#define REGIS_ZERO_POINT    0x0005

typedef struct
{
    uint16_t        Unit_u16;
    uint16_t        Decimal_u16;
    int16_t         Value_i16;
    int16_t         ZeroPoint_i16;
}sLevelParameter;

extern sLevelParameter sLevelInfor;
/*======================== Funcion ======================*/

void        Level_S_YW_Decode (sData *pData);
uint32_t    Level_S_YW_Reverse_Decimal (uint16_t Decimal);
int16_t     Level_S_YW_Convert_Value_Calib (int16_t Calib_mm, uint16_t Decimal, uint16_t Unit);


#endif
