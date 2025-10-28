


#include "user_json.h"

#include "string.h"




/*
    Add Object root
        + pKey: name
        + keylen: 
                NULL: Root fist: write {}
                else: root: write "pKey": :{}

        + rootlevel: to sub number of '}' at end data
*/

void Json_Add_Root (sData *pData, uint8_t *pKey, uint8_t keylen, uint8_t rootlevel)
{
    uint16_t i = 0;
    
    //Tru di dau "}" o truoc root
    if (pData->Length_u16 >= rootlevel)
    {
        pData->Length_u16 -= rootlevel;
    } else
    {
        UTIL_Printf_Str(DBLEVEL_M, "u_json: invalid root level!");
    }
        
    if (keylen == 0)
    {
        *(pData->Data_a8 + pData->Length_u16++) = '{';
        *(pData->Data_a8 + pData->Length_u16++) = '}';
    } else
    {
        //neu da co child o tren: them dau ','
        if (Json_Is_Exist_Child(pData) == true)
        {
            *(pData->Data_a8 + pData->Length_u16++) = ',';
        }
        //Ghi object root vao
        *(pData->Data_a8 + pData->Length_u16++) = '"';
        
        for (i = 0; i < keylen; i++)
            *(pData->Data_a8 + pData->Length_u16++) = *(pKey + i);
        
        *(pData->Data_a8 + pData->Length_u16++) = '"';
        
        *(pData->Data_a8 + pData->Length_u16++) = ':';
        *(pData->Data_a8 + pData->Length_u16++) = '{';
        *(pData->Data_a8 + pData->Length_u16++) = '}';
    }
    
    //Ghi lai dau "}" da clear o tren
    for (i = 0; i < rootlevel; i++)
    {
        *(pData->Data_a8 + pData->Length_u16++) = '}';
    }
}

/*
    Func: add object string
        ex: "modemid":"123456"
        + rootlevel: to sub number of '}' at end data
*/

void Json_Add_Object_String (sData *pData, uint8_t *pKey, uint8_t keylen,
                                           uint8_t *pVal, uint8_t vallen, uint8_t rootlevel)
{    
    uint16_t i = 0;
    
    //Tru di dau "}" o truoc root
    if (pData->Length_u16 >= rootlevel)
    {
        pData->Length_u16 -= rootlevel;
    } else
    {
        UTIL_Printf_Str(DBLEVEL_M, "u_json: invalid root level!");
    }
    
    //neu da co child o tren: them dau ','
    if (Json_Is_Exist_Child(pData) == true)
    {
        *(pData->Data_a8 + pData->Length_u16++) = ',';
    }
    
    //Ghi object string vao
    //String key
    *(pData->Data_a8 + pData->Length_u16++) = '"';
        
    for (i = 0; i < keylen; i++)
        *(pData->Data_a8 + pData->Length_u16++) = *(pKey + i);
    
    *(pData->Data_a8 + pData->Length_u16++) = '"';
    
    *(pData->Data_a8 + pData->Length_u16++) = ':';
    //string val
    *(pData->Data_a8 + pData->Length_u16++) = '"';
        
    for (i = 0; i < vallen; i++)
        *(pData->Data_a8 + pData->Length_u16++) = *(pVal + i);
    
    *(pData->Data_a8 + pData->Length_u16++) = '"';
    
    //Ghi lai dau "}" da clear o tren
    for (i = 0; i < rootlevel; i++)
    {
        *(pData->Data_a8 + pData->Length_u16++) = '}';
    }
}

 
void Json_Add_Object_Number (sData *pData, uint8_t *pKey, uint8_t keylen,
                                           int64_t num, uint8_t scale, uint8_t rootlevel)
{
    uint16_t i = 0;
    char aTEMP[20] = {0}; 
    
    //Convert gia tri number sang string
    UtilIntToStringWithScale (num, aTEMP, scale);
        
    //Tru di dau "}" o truoc root
    if (pData->Length_u16 >= rootlevel)
    {
        pData->Length_u16 -= rootlevel;
    } else
    {
        UTIL_Printf_Str(DBLEVEL_M, "u_json: invalid root level!");
    }
    
    //neu da co child o tren: them dau ','
    if (Json_Is_Exist_Child(pData) == true)
    {
        *(pData->Data_a8 + pData->Length_u16++) = ',';
    }
    
    //Ghi object string vao
    //String key
    *(pData->Data_a8 + pData->Length_u16++) = '"';
        
    for (i = 0; i < keylen; i++)
        *(pData->Data_a8 + pData->Length_u16++) = *(pKey + i);
    
    *(pData->Data_a8 + pData->Length_u16++) = '"';
    
    *(pData->Data_a8 + pData->Length_u16++) = ':';
    
    //string val        
    for (i = 0; i < strlen(aTEMP); i++)
        *(pData->Data_a8 + pData->Length_u16++) = aTEMP[i];
    
    //Ghi lai dau "}" da clear o tren
    for (i = 0; i < rootlevel; i++)
    {
        *(pData->Data_a8 + pData->Length_u16++) = '}';
    }
}

/*
    Func: Check exist Chill to add ","
        - return : false: + find {} or length = 0-> Not child :
                   true:  + find {...} -> have child
*/

uint8_t Json_Is_Exist_Child (sData *pData)
{
    if (pData->Length_u16 == 0)
        return false;
    
    if ( *(pData->Data_a8 + pData->Length_u16 - 1) == '{')
    {
        return false;
    }
    
    return true;
}
                     



/*Json theo dinh dang tap lenh module sim L506*/


/*
    Add Object root
        + pKey: name
        + keylen: 
                NULL: Root fist: write {}
                else: root: write "pKey": :{}

        + rootlevel: to sub number of '}' at end data
*/

void Json_Add_Root_L506 (sData *pData, uint8_t *pKey, uint8_t keylen, uint8_t rootlevel)
{
    uint16_t i = 0;
    
    //Tru di dau "}" o truoc root
    if (pData->Length_u16 >= rootlevel)
    {
        pData->Length_u16 -= rootlevel;
    } else
    {
        UTIL_Printf_Str(DBLEVEL_M, "u_json: invalid root level!");
    }
        
    if (keylen == 0)
    {
        *(pData->Data_a8 + pData->Length_u16++) = '{';
        *(pData->Data_a8 + pData->Length_u16++) = '}';
    } else
    {
        //neu da co child o tren: them dau ','
        if (Json_Is_Exist_Child(pData) == true)
        {
            *(pData->Data_a8 + pData->Length_u16++) = '&';
        }
        //Ghi object root vao       
        for (i = 0; i < keylen; i++)
            *(pData->Data_a8 + pData->Length_u16++) = *(pKey + i);
        
        *(pData->Data_a8 + pData->Length_u16++) = '=';
        *(pData->Data_a8 + pData->Length_u16++) = '{';
        *(pData->Data_a8 + pData->Length_u16++) = '}';
    }
    
    //Ghi lai dau "}" da clear o tren
    for (i = 0; i < rootlevel; i++)
    {
        *(pData->Data_a8 + pData->Length_u16++) = '}';
    }
}

/*
    Func: add object string
        ex: "modemid":"123456"
        + rootlevel: to sub number of '}' at end data
*/

void Json_Add_Object_String_L506 (sData *pData, uint8_t *pKey, uint8_t keylen,
                                           uint8_t *pVal, uint8_t vallen, uint8_t rootlevel)
{    
    uint16_t i = 0;
    
    //Tru di dau "}" o truoc root
    if (pData->Length_u16 >= rootlevel)
    {
        pData->Length_u16 -= rootlevel;
    } else
    {
        UTIL_Printf_Str(DBLEVEL_M, "u_json: invalid root level!");
    }
    
    //neu da co child o tren: them dau ','
    if (Json_Is_Exist_Child(pData) == true)
    {
        *(pData->Data_a8 + pData->Length_u16++) = '&';
    }
    
    //Ghi object string vao
    //String key        
    for (i = 0; i < keylen; i++)
        *(pData->Data_a8 + pData->Length_u16++) = *(pKey + i);
    
    *(pData->Data_a8 + pData->Length_u16++) = '=';
    //string val
    for (i = 0; i < vallen; i++)
        *(pData->Data_a8 + pData->Length_u16++) = *(pVal + i);
    
    //Ghi lai dau "}" da clear o tren
    for (i = 0; i < rootlevel; i++)
    {
        *(pData->Data_a8 + pData->Length_u16++) = '}';
    }
}

 
void Json_Add_Object_Number_L506 (sData *pData, uint8_t *pKey, uint8_t keylen,
                                           int64_t num, uint8_t scale, uint8_t rootlevel)
{
    uint16_t i = 0;
    char aTEMP[20] = {0}; 
    
    //Convert gia tri number sang string
    UtilIntToStringWithScale (num, aTEMP, scale);
    //Tru di dau "}" o truoc root
    if (pData->Length_u16 >= rootlevel)
    {
        pData->Length_u16 -= rootlevel;
    } else
    {
        UTIL_Printf_Str(DBLEVEL_M, "u_json: invalid root level!");
    }
    
    //neu da co child o tren: them dau ','
    if (Json_Is_Exist_Child(pData) == true)
    {
        *(pData->Data_a8 + pData->Length_u16++) = '&';
    }
    
    //Ghi object string vao
    //String key
    for (i = 0; i < keylen; i++)
        *(pData->Data_a8 + pData->Length_u16++) = *(pKey + i);
    
    *(pData->Data_a8 + pData->Length_u16++) = '=';
    
    //string val        
    for (i = 0; i < strlen(aTEMP); i++)
        *(pData->Data_a8 + pData->Length_u16++) = aTEMP[i];
    
    //Ghi lai dau "}" da clear o tren
    for (i = 0; i < rootlevel; i++)
    {
        *(pData->Data_a8 + pData->Length_u16++) = '}';
    }
}


                    
                    
                    
                    
                    