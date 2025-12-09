


#include "woteck_wm.h"

/*
    DW 4-bytes integer data;
    W 2-bytes integer data;
    B 1-byte integer, this parameter will be added to 2-bytes with the ‘0x00’ high byte;
    SF 4-bytes single-precision IEEE 754 ABCD floating-point format data; Reference here

    Fixed Point Data: For Example: DW*1000 means the parameter is amplified 1000-fold. 
    If the real valueis 0.123, in the Modbus, the slave will response the value as 123.

    Float format：
     
    The IEEE754 format is used for the 4-bytes float data as following:
            Register 1                       Register 2
        BYTE1           BYTE2           BYTE3       BYTE4
        S EEEEEEE      E MMMMMMM      MMMMMMMM      MMMMMMMM
 */

/*Bang than ghi modbus*/
sWoteckRegList sWOTECK_REGISTER[] = 
{
            //param     Register(2byte)     format
    {   __TOTAL_FORW,           90,       __SF},
    {   __TOTAL_REV,            92,       __SF},
    {   __TOTAL_VAL,            94,       __SF},
    
    {   __FLOW,                 98,       __SF},
};


/*
    Modebus Decode
*/
uint8_t aDATA_TEST [200] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x63, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 
    0x00, 0x00, 0x00, 0x14, 0x00, 0x09, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x19, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x03, 
    0x00, 0x05, 0x00, 0x03, 0x27, 0x10, 0x00, 0x02, 0x00, 0x00, 
    0x00, 0x01, 0x00, 0xC8, 0x00, 0x1E, 0x00, 0x01, 0x00, 0x01, 
    0x00, 0x01, 0x00, 0x14, 0x00, 0x00, 0x2E, 0xE0, 0x00, 0x00, 
    0x01, 0xF4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 
    0x00, 0x00, 0x07, 0xD0, 0x00, 0x01, 0x00, 0x0A, 0x00, 0x02, 
    0x00, 0x0A, 0x02, 0x58, 0x00, 0x12, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x27, 0x10, 0x03, 0x20, 0x00, 0x28, 0x00, 0x06, 0x00, 0x0A, 
    0x30, 0x12, 0x00, 0x00, 0x26, 0xAC, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x27, 0x10, 0x00, 0x00, 0x4E, 0x20, 0x00, 0x00, 
    0x75, 0x30, 0x00, 0x00, 0x9C, 0x40, 0x00, 0x00, 0xC3, 0x50, 
    0x00, 0x00, 0x27, 0x10, 0x00, 0x00, 0x4E, 0x20, 0x00, 0x00, 
    0x75, 0x30, 0x00, 0x00, 0x9C, 0x40, 0x00, 0x00, 0xC3, 0x50, 
    0x3C, 0x9A, 0x69, 0x38, 0x41, 0xA2, 0x2E, 0x93, 0xC1, 0xA2, 
    0x07, 0xF9, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00
};


sWoteckData      sWoteck;


void Woteck_Decode (sData *pData)
{
    Woteck_Extract_Data (pData, __TOTAL_FORW, &sWoteck.TotalForw_f);      
    Woteck_Extract_Data (pData, __TOTAL_REV, &sWoteck.TotalRev_f);  
    Woteck_Extract_Data (pData, __TOTAL_VAL, &sWoteck.TotalVal_f);  
    
    Woteck_Extract_Data (pData, __FLOW, &sWoteck.Flow_f);  
}



void Woteck_Extract_Data (sData *pSource, uint8_t Param, void *data)
{
    uint8_t aTEMP_u32[4] = {0};
    uint32_t TempU32 = 0;
    uint16_t TempU16 = 0;
        
    if (pSource->Length_u16 < sWOTECK_REGISTER[Param].Addr * 2 + 4)
    {
        UTIL_Printf_Str (DBLEVEL_M, "woteck_wm: not value\r\n" );
        
        return;
    }
    
    switch (sWOTECK_REGISTER[Param].Format_u8)
    {
        case __DW_1:
        case __DW_100:
        case __DW_1000:
            for (uint8_t i = 0; i < 4; i++)
                aTEMP_u32[i] = *(pSource->Data_a8 + sWOTECK_REGISTER[Param].Addr * 2 + i);
            
            TempU32 = aTEMP_u32[0] << 24 | aTEMP_u32[1] << 16 | aTEMP_u32[2] << 8 | aTEMP_u32[3];
                
            *(int32_t *)data = (int32_t) TempU32; 
            break;
        case __WORD:
            for (uint8_t i = 0; i < 2; i++)
                aTEMP_u32[i] = *(pSource->Data_a8 + sWOTECK_REGISTER[Param].Addr * 2 + i);
            
            TempU16 = aTEMP_u32[0] << 8 | aTEMP_u32[1];
                
            *(int8_t *)data = (int16_t) TempU16; 
            break;
        case __BYTE:
            for (uint8_t i = 0; i < 2; i++)
                aTEMP_u32[i] = *(pSource->Data_a8 + sWOTECK_REGISTER[Param].Addr * 2 + i);
            
            *(int8_t *)data = (int8_t) aTEMP_u32[1];
            break;
        case __SF:
            for (uint8_t i = 0; i < 4; i++)
                aTEMP_u32[i] = *(pSource->Data_a8 + sWOTECK_REGISTER[Param].Addr * 2 + i);
            
            TempU32 = aTEMP_u32[0] << 24 | aTEMP_u32[1] << 16 | aTEMP_u32[2] << 8 | aTEMP_u32[3];

            *(float *)data = Convert_FloatPoint_2Float(TempU32);
            break;
        default:
            break;
    }   
}









