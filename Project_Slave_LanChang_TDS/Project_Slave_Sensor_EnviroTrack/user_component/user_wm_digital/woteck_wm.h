

#ifndef WOTECK_WM_H
#define WOTECK_WM_H


#include "user_util.h"



typedef enum
{
    __DW_1,
    __DW_100,
    __DW_1000,
    __WORD,
    __BYTE,
    __SF,
}eWoFormat;

typedef enum
{
    __TOTAL_FORW,
    __TOTAL_REV,
    __TOTAL_VAL,

    __FLOW,
}sWoteckParameter;


typedef struct
{
    uint8_t     Param_u8;
    uint16_t    Addr;
    eWoFormat   Format_u8;        
}sWoteckRegList;


typedef struct
{
    float Flow_f;
    float TotalForw_f;
    float TotalRev_f;
    float TotalVal_f;
}sWoteckData;

extern sWoteckData      sWoteck;

/*=========== Function =========*/
void Woteck_Decode (sData *pData);
void Woteck_Extract_Data (sData *pSource, uint8_t Param, void *data);




#endif




