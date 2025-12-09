/*
    8/2021
    Thu vien Quan ly cac function convert
*/
#include "user_util.h"
#include "string.h"
#include "math.h"


/*===================Struct, Var=========================*/
uint32_t        RtCountSystick_u32;
ST_TIME_FORMAT  sRTC;
uint8_t         VLevelDebug = DBLEVEL_M;

/*Func Pointer to Save Log to mem*/
uint8_t (*pFunc_UTIL_Log_To_Mem) (uint8_t *pData, uint16_t Length);
void (*pFunc_UTIL_Debug_Send) (uint8_t *pData, uint16_t Length);



/*===================function=========================*/
uint8_t Check_Time_Out (uint32_t Millstone_Time, uint32_t Time_Period_ms)
{	
	if (Cal_Time(Millstone_Time, RtCountSystick_u32) >= Time_Period_ms) 
        return 1;
	
	return 0;
}

uint8_t Check_Time_Out_2(uint32_t Millstone_Time, uint32_t Time_Period_ms)
{	
	if (Millstone_Time == 0) 
        return 0;
	
	if (Cal_Time(Millstone_Time, RtCountSystick_u32) >= Time_Period_ms) 
        return 1;
	
	return 0;
}

uint32_t Cal_Time(uint32_t Millstone_Time, uint32_t Systick_now) 
{
	if (Systick_now < Millstone_Time)
		return (0xFFFFFFFF - Millstone_Time + Systick_now);
	return (Systick_now - Millstone_Time);
}	

uint32_t Cal_Time_s(uint32_t Millstone_Time, uint32_t Systick_now) 
{
	if (Systick_now < Millstone_Time)
		return ((0xFFFFFFFF - Millstone_Time + Systick_now) / 1000);
	return ((Systick_now - Millstone_Time) / 1000);
}	


int32_t Convert_float_2int (uint32_t Float_Point_IEEE, uint8_t scale)
{
	int 		scaleVal = 1;
	float 		floatTmp = 0;
	uint8_t		i = 0;

	//Get scale value
	for (i = 0; i < scale; i++) 
	{
		scaleVal *= 10;
	}
	
	floatTmp = (float) Convert_FloatPoint_2Float(Float_Point_IEEE);

	return (int32_t) (floatTmp * scaleVal);
}


float Convert_FloatPoint_2Float (uint32_t Float_Point)
{
	return *((float*)(&Float_Point));
}

int32_t Convert_uint_2int (uint32_t Num)
{
	return *((int32_t*)(&Num));
}

int16_t Convert_uint16_2int16 (uint16_t Num)
{
	return *((int16_t*)(&Num));
}


int64_t Convert_uint64_2int64 (uint64_t Num)
{
	return *((int64_t*)(&Num));
}

//t�nh BBC bang cach xor c�c byte.

uint8_t BBC_Cacul (uint8_t* Buff, uint16_t length)
{
	uint16_t i = 0;
	uint8_t Result = 0;
	
	for (i = 0; i < length; i++)
		Result ^= *(Buff + i);
    
	return Result;
}
 

uint8_t Check_BBC (sData* Str)
{
	uint8_t BBC_Value;
	
	BBC_Value = BBC_Cacul((Str->Data_a8 + 1), (Str->Length_u16 - 2));
	
	if (BBC_Value == *(Str->Data_a8 + Str->Length_u16 - 1))
        return 1;
	else 
        return 0;
}


void UtilIntToStringWithScale (int number, char *pdata, uint8_t scale)
{
    float val = 0;
    
    if (scale >= 0xFB) {
        val = number * Convert_Scale(0xFF - scale + 1);
        UtilFloatToString(val, pdata, 0);
    } else {
        val = number / Convert_Scale(scale);
        UtilFloatToString(val, pdata, scale);
    }
}

void Reset_Buff(sData *str)
{
	uint16_t i;
    
	for (i = 0; i < str->Length_u16; i++)
		*(str->Data_a8 +i ) = 0;
    
	str->Length_u16 = 0;
}




void UTIL_MEM_set( void *dst, uint8_t value, uint16_t size )
{
    uint8_t* dst8= (uint8_t *) dst;

    while( size-- )
    {
        *dst8++ = value;
    }
}



void UTIL_Printf (uint8_t Level, uint8_t *pData, uint16_t Length)
{
#if (DEBUG_ENABLED == 1)
    if (Level <= VLevelDebug)
    {
        if (pFunc_UTIL_Debug_Send != NULL)
            pFunc_UTIL_Debug_Send(pData, Length);
    }
#endif
}

void UTIL_Printf_Str (uint8_t Level, const char *str)
{
    UTIL_Printf(Level, (uint8_t *) str, strlen(str));
}


void UTIL_Printf_Hex (uint8_t Level, uint8_t *pData, uint16_t Length)
{
    char buffer[10] = {0};
    
    for (uint16_t i = 0; i < Length; i++)
    {
        sprintf(buffer, "%02X", pData[i]);
    
        UTIL_Printf_Str(Level, buffer);
    }
}



void UTIL_Log (uint8_t Level, uint8_t *pData, uint16_t Length)
{
    //Print to 
	UTIL_Printf (Level, pData, Length );
    //Save to Flash
    if (pFunc_UTIL_Log_To_Mem != NULL)
        pFunc_UTIL_Log_To_Mem (pData, Length);
}


void UTIL_Log_Str (uint8_t Level, const char *str)
{
    //Print to 
	UTIL_Printf (Level, (uint8_t *) str, strlen(str));
    //Save to Flash
    if (pFunc_UTIL_Log_To_Mem != NULL)
        pFunc_UTIL_Log_To_Mem ((uint8_t *) str, strlen(str));
}



void UTIL_Printf_Dec (uint8_t Level, int32_t number)
{
    char buff[20] = {0};
    
    UtilIntToString(number, buff);
    
    UTIL_Printf_Str (Level, buff );
}



/*
    Passed to function:                                                    
        lat1, lon1 = Latitude and Longitude of point 1 (in decimal degrees)  
        lat2, lon2 = Latitude and Longitude of point 2 (in decimal degrees)  
        unit = the unit you desire for results                               
                where: 
                    'M' is statute miles (default)                         
                    'K' is kilometers                                  
                    'N' is nautical miles   
*/

double UTIL_Cacul_Distance(double lat1, double lon1, double lat2, double lon2, char unit) 
{
    double theta, dist;
    
    if ((lat1 == lat2) && (lon1 == lon2))
    {
        return 0;
    }
    else 
    {
        theta = lon1 - lon2;
        dist = sin(UTIL_Deg_2Rad(lat1)) * sin(UTIL_Deg_2Rad(lat2)) + cos(UTIL_Deg_2Rad(lat1)) * cos(UTIL_Deg_2Rad(lat2)) * cos(UTIL_Deg_2Rad(theta));
        dist = acos(dist);
        dist = UTIL_Rad_2Deg(dist);
        dist = dist * 60 * 1.1515;
        
        switch(unit) 
        {
          case 'M':
            break;
          case 'K':
            dist = dist * 1.609344;
            break;
          case 'N':
            dist = dist * 0.8684;
            break;
        }
        return (dist);
    }
}

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  This function converts decimal degrees to radians             :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
double UTIL_Deg_2Rad (double deg) 
{
    return (deg * pi / 180);
}

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  This function converts radians to decimal degrees             :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
double UTIL_Rad_2Deg (double rad) 
{
    return (rad * 180 / pi);
}


/*
    Func: Convert GNRMC data type to Location
        + $GNRMC,025454.000,A,2101.77694,N,10547.10438,E,0.00,0.00,241023
            Data type: DDmm.mmmmm -> DD.DDDD
        + Lam nhu sau: DDmm.mmmmm = DD + mm.mmmmm/60
*/
double UTIL_Convert_GNRMC_To_Location (double GNRMC_Val)
{
    uint16_t NguyenDD = 0;
    
    NguyenDD = (uint16_t) (GNRMC_Val / 100);
    
    GNRMC_Val -= NguyenDD * 100;
    
    return (double) (GNRMC_Val / 60 + NguyenDD);
}




/*
    Func: Chuyen Day IP ve dang buffer
        return: 1: Success; 0: Fail format
*/

uint8_t UTIL_Convert_IP_To_Buff (sData *sIP, uint8_t *aIP_BUF)
{
    uint8_t aIP[6] = {0};
    uint8_t i = 0, TempU8 = 0;
    uint8_t Index = 0, Num = 0;
    
    for (i = 0; i < sIP->Length_u16; i++)
    {
        TempU8 = *(sIP->Data_a8 + i);
        
        if (TempU8 == '.')
        {
            aIP[Index++] = Num;
            Num = 0;
            
            if (Index > 6)
                return false;
            
            continue;
        }
        
        if ( (TempU8 > 0x39) || (TempU8 < 0x30) )
            return false;
        
        Num = Num * 10 + TempU8 - 0x30;
        
        //Day so cuoi cung
        if (i == (sIP->Length_u16 - 1))
            aIP[Index++] = Num;
    }
    
    for (i = 0; i < Index; i++)
    {
        aIP_BUF[i] = aIP[i];
    }
    
    return true;
}



void UTIL_MEM_cpy( void *dst, const void *src, uint16_t size )
{
  uint8_t* dst8= (uint8_t *) dst;
  uint8_t* src8= (uint8_t *) src;

  while( size-- )
    {
        *dst8++ = *src8++;
    }
}

void UTIL_MEM_cpyr( void *dst, const void *src, uint16_t size )
{
    uint8_t* dst8= (uint8_t *) dst;
    uint8_t* src8= (uint8_t *) src;
    
    dst8 = dst8 + ( size - 1 );
    while( size-- )
    {
        *dst8-- = *src8++;
    }
}

/*
    Func: Get Num string from Pos in sData
        + Find Begin number
        + Convert num 

    * return - 0xFFFF if not find number
                value convert
             - *Pos end
*/
uint32_t UTIL_Get_Num_From_Str(sData *strRecei, uint16_t *Pos)
{
    uint32_t Value = 0xFFFFFFFF, i = 0;
    uint8_t temp = 0, count = 0;
    
    //Tim vi tri bat dau so
    for (i = *Pos; i < strRecei->Length_u16; i++)
    {
        temp = *(strRecei->Data_a8 + i);
        if ((temp >= 0x30) && (temp <= 0x39))
        {
            break;
        }
    }
    //lay so
    if (i < strRecei->Length_u16)
    {
        Value = 0;
        
        while (i < strRecei->Length_u16)
        {
            temp = *(strRecei->Data_a8 + i++);
            if ((count >= 8) || (temp < '0') || (temp > '9'))
                break;
            
            Value = Value * 10 + (temp - 0x30);
            count++;
        }
    }

    *Pos = i;
    
    return Value;
}



float Convert_Scale (uint8_t scale)
{
    if (scale > 0xFA)
        return (float) ( 1 / pow (10, 0xFF - scale + 1) );
    else
        return (double) pow (10, scale);
}

void UtilHexStringToHex(const char* hexStr, char* buffer)
{
    int number = strtol(hexStr, NULL, 16); // Convert hex string to integer
    sprintf(buffer, "%X", number); // Convert integer back to hex string
}

void UtilHexToString(unsigned int hex, char* buffer) 
{
    sprintf(buffer, "%X", hex);
}
    
float UtilStringToFloat(const char* str) 
{
    float result = atof(str);
    return result;
}

    
int UtilStringToInt(const char* str)
{
    int result = atoi(str);
    return result;
}


int UtilStringToInt2(uint8_t *pData, uint8_t length)
{
    uint8_t *array = NULL;
    int val = 0;
    
    array = (uint8_t*)calloc(length + 1, sizeof(uint8_t));
    
    UTIL_MEM_cpy(array, pData, length);
    
    val = UtilStringToInt( (const char *) array);
    
    free(array);

    return val;
}

void UtilIntToString(int number, char* buffer) 
{
    sprintf(buffer, "%d", number);
}

void UtilFloatToString(float number, char* buffer, int decimals) 
{
    sprintf(buffer, "%.*f", decimals, number);
}


uint16_t UTIL_Cut_String (char *src, char *dst, uint16_t max)
{
    uint16_t i = 0;
    
    while (*(src + i) != ',')
    {
        *(dst++) = *(src + i);
        i++;
        if (i >= max)
            break;
    }
    
    return i;
}
    
uint8_t UTIL_Get_Scale (uint32_t val)
{
    uint8_t scale = 0;
    
    while (val != 0)
    {
        val = val / 10;
        if (val != 0)
            scale++;
    }
    
    return scale;
}


void Convert_Uint64_To_StringDec (sData *Payload, uint64_t Data, uint8_t Scale)
{
    uint16_t    i = 0;
    uint8_t     PDu = 0;
    uint8_t     Temp = 0;    
    uint8_t     BufStrDec[30] = {0};
    sData    StrDecData = {&BufStrDec[0], 0};
    
    if (Data == 0)
    {
        *(Payload->Data_a8 + Payload->Length_u16) = '0';
        Payload->Length_u16++;
        return;
    }
    
    while (Data != 0)
    {
        PDu = (uint8_t) (Data % 10);
        Data = Data / 10;
        *(StrDecData.Data_a8 + StrDecData.Length_u16++) = PDu + 0x30;
    }
    //�ao nguoc lai
    for (i = 0; i < (StrDecData.Length_u16 / 2); i++)
    {
        Temp = *(StrDecData.Data_a8 + StrDecData.Length_u16 - i - 1);
        *(StrDecData.Data_a8 + StrDecData.Length_u16 - i - 1) = *(StrDecData.Data_a8 + i);
        *(StrDecData.Data_a8 + i) = Temp;
    }
    Scale_String_Dec (&StrDecData, Scale);
        
    for (i = 0; i < StrDecData.Length_u16; ++i) 
		*(Payload->Data_a8 +Payload->Length_u16 + i) = *(StrDecData.Data_a8 + i);

	Payload->Length_u16 +=  StrDecData.Length_u16;
}


void Scale_String_Dec (sData *StrDec, uint8_t Scale)
{
    uint16_t    i = 0;
    uint8_t     aRESULT[20] = {0};
    uint8_t     Length = 0;
    uint8_t     NumZero = 0, NumNguyen = 0;
    uint8_t     TempLength = 0;

    if (((StrDec->Length_u16 == 1) && (*(StrDec->Data_a8) == 0x30)) || (Scale == 0))
        return;

    if (StrDec->Length_u16 <= Scale)
    {
        aRESULT[Length++] = '0';
        aRESULT[Length++] = '.';
        NumZero = Scale - StrDec->Length_u16;   //So luong so '0' them vao phia sau dau ','
        //Them cac so 0 sau dau ','
        for (i = 0; i < NumZero; i++)
            aRESULT[Length++] = '0';
        //Them tiep cac byte strdec.
        for (i = 0; i < StrDec->Length_u16; i++)
             aRESULT[Length++] = *(StrDec->Data_a8 + i);
    } else
    {
        NumNguyen = StrDec->Length_u16 - Scale;
        //Copy phan nguyen vao
        for (i = 0; i < NumNguyen; i++)
            aRESULT[Length++] = *(StrDec->Data_a8 + i);
        //
        aRESULT[Length++] = '.';
        //Copy phan thap phan vao
        for (i = NumNguyen; i < StrDec->Length_u16; i++)
            aRESULT[Length++] = *(StrDec->Data_a8 + i);
        //
    }

    TempLength = Length;
    //Neu co so '0' phia sau cung thi bo qua het
    for (i = (TempLength - 1); i > 0; i--)
    {
        //Cho den khi co so khac '0' se break;
        if (aRESULT[i] != '0')
        {
            if (aRESULT[i] == '.')   //'.'
                Length--;

			break;
        }
        if (aRESULT[i] == '0')
            Length--;
    }
    //Ket qua thu duoc trong buff aRESULT chieu dai length-> Copy sang strdec dâu vao làm dau ra
    Reset_Buff (StrDec);
    for (i = 0; i < Length; i++)
        *(StrDec->Data_a8 + StrDec->Length_u16++) = aRESULT[i];
}

//------------------------Convert uint32Hex To Float------------------------
    
    