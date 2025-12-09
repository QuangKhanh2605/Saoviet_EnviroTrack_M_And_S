


#include "wm_mt100.h"
#include "user_app_wm.h"
#include "user_obis_sv.h"
#include "user_json.h"

/*
    Float format：
     
    The IEEE754 format is used for the 4-bytes float data as following:
            Register 1                       Register 2
        BYTE1           BYTE2           BYTE3       BYTE4
        S EEEEEEE      E MMMMMMM      MMMMMMMM      MMMMMMMM
 */

/*Bang than ghi modbus*/
sMT100RegList sMT100_REGISTER[] = 
{
           //param           Register(2byte)     format
    {   __MT100_FLOW,               0,      __FM_FLOAT},
    
    {   __MT100_POS_INT,            8,      __FM_LONG},
    {   __MT100_POS_DEC,            10,     __FM_FLOAT},
    {   __MT100_NEG_INT,            12,     __FM_LONG},
    {   __MT100_NEG_DEC,            14,     __FM_FLOAT},
    
    {   __MT100_FLOW_UNIT,          16,     __FM_USHORT},
    {   __MT100_TOTAL_UNIT,         17,     __FM_USHORT}, 
};


sMT100Data   sMT100;

const char MT100_UNIT_FLOW_1[] = {" (l/s)"};
const char MT100_UNIT_FLOW_2[] = {" (l/m)"};
const char MT100_UNIT_FLOW_3[] = {" (l/h)"};
const char MT100_UNIT_FLOW_4[] = {" (m3/s)"};
const char MT100_UNIT_FLOW_5[] = {" (m3/m)"};
const char MT100_UNIT_FLOW_6[] = {" (m3/h)"};

const char MT100_UNIT_CUM_1[] = {" (l)"};
const char MT100_UNIT_CUM_2[] = {" (m3)"};


/*============= Function ==================*/
void MT100_Decode (sData *pData)
{
    MT100_Extract_Data (pData, __MT100_FLOW, &sMT100.Flow_f);  
    
    MT100_Extract_Data (pData, __MT100_POS_INT, &sMT100.PosInt_i32);  
    MT100_Extract_Data (pData, __MT100_POS_DEC, &sMT100.PosDec_f);  
    
    MT100_Extract_Data (pData, __MT100_NEG_INT, &sMT100.NegInt_i32);  
    MT100_Extract_Data (pData, __MT100_NEG_DEC, &sMT100.NegDec_f);  
    
    MT100_Extract_Data (pData, __MT100_FLOW_UNIT, &sMT100.FlowUnit_u8);  
    MT100_Extract_Data (pData, __MT100_TOTAL_UNIT, &sMT100.TotalUnit_u8); 
}



void MT100_Extract_Data (sData *pSource, uint8_t Param, void *data)
{
    uint8_t aTEMP[4] = {0};
    uint32_t TempU32 = 0;
    uint16_t TempU16 = 0;
    
    if (pSource->Length_u16 < sMT100_REGISTER[Param].Addr * 2 + 4)
    {        
        UTIL_Printf_Str (DBLEVEL_M, "wm_mt100: not value\r\n" );
        
        return ;
    }
    
    switch (sMT100_REGISTER[Param].Format_u8)
    {
        case __FM_USHORT:
            for (uint8_t i = 0; i < 2; i++)
                aTEMP[i] = *(pSource->Data_a8 + sMT100_REGISTER[Param].Addr * 2 + i);
            
            TempU16 = aTEMP[0] << 8 | aTEMP[1];
                
            *(uint16_t *) data = ( TempU16 ); 
            break;
        case __FM_LONG:
            for (uint8_t i = 0; i < 4; i++)
                aTEMP[i] = *(pSource->Data_a8 + sMT100_REGISTER[Param].Addr * 2 + i);
            
            TempU32 = aTEMP[0] << 24 | aTEMP[1] << 16 | aTEMP[2] << 8 | aTEMP[3];

            *(int32_t *) data = (int32_t) TempU32; 
            break;
        case __FM_FLOAT:
            for (uint8_t i = 0; i < 4; i++)
                aTEMP[i] = *(pSource->Data_a8 + sMT100_REGISTER[Param].Addr * 2 + i);
            
            TempU32 = aTEMP[0] << 24 | aTEMP[1] << 16 | aTEMP[2] << 8 | aTEMP[3];

            *(float *)data = Convert_FloatPoint_2Float(TempU32);
            break;
        default:
            break;
    }   
}

/*
    Func: decode unit meter MT100
        type: 0: flow;  1: cumulative
*/
char *MT100_Decode_Unit (uint8_t type)
{
    char *pUnit = NULL;
    
    if (type == 0)
    {
        switch (sMT100.FlowUnit_u8)
        {
            case 0:
                pUnit = (char *) MT100_UNIT_FLOW_1;
                break;
            case 1:
                pUnit = (char *) MT100_UNIT_FLOW_2;
                break;
            case 2:
                pUnit = (char *) MT100_UNIT_FLOW_3;
                break;
            case 3:
                pUnit = (char *) MT100_UNIT_FLOW_4;
                break;
            case 4:
                pUnit = (char *) MT100_UNIT_FLOW_5;
                break;
            case 5:
                pUnit = (char *) MT100_UNIT_FLOW_6;
                break;
            default:
                break;
        }
    } else
    {
        switch (sMT100.TotalUnit_u8)
        {
            case 0:
                pUnit = (char *) MT100_UNIT_CUM_1;
                break;
            case 1:
                pUnit = (char *) MT100_UNIT_CUM_2;
                break;
            default:
                break;
        }
    }
    
    return pUnit;
}


float MT100_Convert_Flow_m3h (float val)
{
    float result = 0;
    
    switch (sMT100.FlowUnit_u8)
    {
        case 0:  //l/s -> m3/h
            result = val * 3600 / 1000;
            break;
        case 1:  //l/m
            result = val * 60 / 1000;
            break;
        case 2:  //l/h
            result = val / 1000;
            break;
        case 3:  //m3/s
            result = val * 3600;
            break;
        case 4: //m3/m
            result = val * 60;
            break;
        case 5: // m3/h
            result = val;
            break;
        default:
            break;
    }
    
    return result;
}






