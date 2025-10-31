

#ifndef WM_MT100_H
#define WM_MT100_H


#include "user_util.h"



typedef enum
{
    __FM_USHORT,
    __FM_LONG,
    __FM_FLOAT,
}eMT100Format;


typedef enum
{
    __MT100_FLOW,
    
    __MT100_POS_INT,
    __MT100_POS_DEC,  
    __MT100_NEG_INT,
    __MT100_NEG_DEC,
    
    __MT100_FLOW_UNIT,
    __MT100_TOTAL_UNIT,

}sMT100Parameter;


typedef struct
{
    int32_t Val_i32;
    uint8_t Scale_u8;
}sMTValueInfor;



typedef struct
{
    uint8_t     Param_u8;
    uint16_t    Addr;
    eMT100Format   Format_u8;        
}sMT100RegList;


typedef struct
{
    float   Flow_f;
    int32_t PosInt_i32;
    float   PosDec_f;
    int32_t NegInt_i32;
    float   NegDec_f;
    
    uint16_t FlowUnit_u8;
    uint16_t TotalUnit_u8;
}sMT100Data;


extern sMT100Data   sMT100;

/*=========== Function =========*/
void MT100_Decode (sData *pData);
void MT100_Extract_Data (sData *pSource, uint8_t Param, void *data);

char *MT100_Decode_Unit (uint8_t type);
float MT100_Convert_Flow_m3h (float val);

#endif




