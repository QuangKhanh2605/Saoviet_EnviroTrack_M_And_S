


#ifndef LEVEL_SUP_PX261_B_H
#define LEVEL_SUP_PX261_B_H


#include "user_util.h"


/*=== Register liquid level S_YW_01B ===*/

#define SUP_REG_SLAVE_ID	    0x0000
#define SUP_REG_BAURATE	        0x0001
#define SUP_REG_DECIMAL   	    0x0002
#define SUP_REG_UNIT	        0x0003
#define SUP_REG_VALUE 	        0x0005

typedef struct
{
    uint16_t        Unit_u16;
    uint16_t        Decimal_u16;
    int16_t         Value_i16;
    int16_t         ZeroPoint_i16;
}sLevelSupParameter;

extern sLevelSupParameter sLevelSupInfor;

/*======================== Funcion ======================*/

void        Level_SUP_Decode (sData *pData);
uint32_t    Level_SUP_Reverse_Decimal (uint16_t Decimal);
int16_t     Level_SUP_Convert_Value_Calib (int16_t Calib_mm, uint16_t Decimal, uint16_t Unit);


#endif
