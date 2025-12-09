
#ifndef USER_CONVERT_VARIABLE_H__
#define USER_CONVERT_VARIABLE_H__    

#include "user_util.h"

uint32_t    Calculator_Scale(uint8_t Scale);
uint32_t    Calculator_Value_Scale(uint32_t Value, uint8_t Scale, uint8_t Scale_Default);

uint8_t     Convert_Int_To_String_Scale(uint8_t cData[], int var, uint8_t Scale);
uint8_t     Convert_Int_To_String(uint8_t cData[], int var);
void        Convert_Var_Packet_Integer (uint8_t *pTarget, uint16_t *LenTarget, uint32_t Data);


uint8_t     Convert_Point_Int_To_String(uint8_t cData[], uint16_t *Pos, int var);
uint8_t     Convert_Point_Int_To_String_Scale(uint8_t cData[], uint16_t *Pos, int var, uint8_t Scale);
void        Insert_String_To_String(uint8_t destination[], uint16_t *PosDes, uint8_t source[], uint16_t PosSour, uint32_t length);

uint32_t    Convert_String_To_Dec(uint8_t *pData, uint8_t lenData);

void        Convert_uint32Hex_To_Float(uint32_t uint32Hex, float *Var);

int32_t    Hanlde_Float_To_Int32_Scale_Round(float varFloat, uint8_t scale);
int32_t    Handle_HexFloat_To_Int32_Round(uint32_t hexFloat, uint8_t scale);
float      Handle_int32_To_Float_Scale(int32_t var, uint8_t scale);
uint32_t   Handle_Float_To_hexUint32(float num);
#endif

