

#ifndef USER_JSON_H_
#define USER_JSON_H_



#include "user_util.h"




void Json_Add_Root (sData *pData, uint8_t *pKey, uint8_t keylen, uint8_t rootlevel);
void Json_Add_Object_String (sData *pData, uint8_t *pKey, uint8_t keylen,
                                           uint8_t *pVal, uint8_t vallen, uint8_t rootlevel);

void Json_Add_Object_Number (sData *pData, uint8_t *pKey, uint8_t keylen,
                                           int64_t num, uint8_t scale, uint8_t rootlevel);

uint8_t Json_Is_Exist_Child (sData *pData);

void Json_Add_Root_L506 (sData *pData, uint8_t *pKey, uint8_t keylen, uint8_t rootlevel);
void Json_Add_Object_String_L506 (sData *pData, uint8_t *pKey, uint8_t keylen,
                                           uint8_t *pVal, uint8_t vallen, uint8_t rootlevel);
                             
void Json_Add_Object_Number_L506 (sData *pData, uint8_t *pKey, uint8_t keylen,
                                           int64_t num, uint8_t scale, uint8_t rootlevel);
                             


#endif


