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


void Convert_Int64_To_StringDec (sData *Payload, int64_t Data, uint8_t Scale)
{
    uint8_t     i = 0;
    uint8_t     BufStrDec[30] = {0};
    uint8_t     Sign = 0;
    sData    StrDecData = {&BufStrDec[0], 0};
    
    if (Data < 0)
    {
        Sign = 1;
        Data = 0 - Data;
    }
    
    Convert_Uint64_To_StringDec(&StrDecData, (uint64_t)Data, Scale);
    
    if (*(StrDecData.Data_a8 + StrDecData.Length_u16) != '0')
    {
        if (Sign == 1)
        {
            for (i = 0; i < StrDecData.Length_u16; i++)
              *(StrDecData.Data_a8 + StrDecData.Length_u16 - i) =  *(StrDecData.Data_a8 + StrDecData.Length_u16 - i - 1);
            
            *(StrDecData.Data_a8) = '-';
            StrDecData.Length_u16 ++;
        }
    }
    //
    for (i = 0; i < StrDecData.Length_u16; ++i) 
		*(Payload->Data_a8 +Payload->Length_u16 + i) = *(StrDecData.Data_a8 + i);

	Payload->Length_u16 +=  StrDecData.Length_u16;
}



void Reset_Buff(sData *str)
{
	uint16_t i;
    
	for (i = 0; i < str->Length_u16; i++)
		*(str->Data_a8 +i ) = 0;
    
	str->Length_u16 = 0;
}



uint32_t Convert_String_To_Dec(uint8_t *pData, uint8_t lenData)
{
    uint8_t     index = 0;
    uint8_t     tempData = 0;
    uint32_t    reVal = 0;
    uint8_t     TempU8 = 0;

    for (index = 0; index < lenData; index++)
    {
        TempU8 = *(pData + index);
        if (('0' <= TempU8) && (TempU8 <= '9'))
        {
            tempData = (TempU8 - 0x30);
        } else
            tempData = 0;
        
        if (index == 0)
            reVal = tempData;
        else
            reVal = (reVal * 10) + tempData;
    }

    return reVal;
}




void Convert_Hex_To_String_Hex(sData *sTaget, sData *sSource)
{
    uint8_t 	Temp_1 = 0;
    uint8_t 	Temp_2 = 0;
    uint16_t 	i = 0;

    for (i = 0; i < sSource->Length_u16; i++)
    {
        //Convert 4 bit dau
        Temp_1 = ((*(sSource->Data_a8 + i)) >> 4) & 0x0F;
        if (Temp_1 <= 9)
        	*(sTaget->Data_a8 + 2*i) = Temp_1 + 0x30;
        else
        	*(sTaget->Data_a8 + 2*i) = Temp_1 + 0x37;
        //Convert 4 bit sau
        Temp_2 = ((*(sSource->Data_a8 + i)) & 0x0F);
        if (Temp_2 <= 9)
        	*(sTaget->Data_a8 + 2*i + 1) = Temp_2 + 0x30;
        else
        	*(sTaget->Data_a8 + 2*i + 1) = Temp_2 + 0x37;
    }
    //Length gap doi source
    sTaget->Length_u16 = sSource->Length_u16 * 2;
}

void UTIL_MEM_set( void *dst, uint8_t value, uint16_t size )
{
    uint8_t* dst8= (uint8_t *) dst;

    while( size-- )
    {
        *dst8++ = value;
    }
}


void Convert_String_Hex_To_Hex (sData *sSource, int Pos_Begin, sData *sTarget, uint16_t MaxLength)
{
	uint8_t Temp_Data = 0;
	uint8_t Temp_Data2 = 0;
	int Pos_Convert = Pos_Begin;
	
	while (Pos_Convert < sSource->Length_u16)
	{
		 Temp_Data = *(sSource->Data_a8 + Pos_Convert++);
         Temp_Data = Convert_String_To_Hex (Temp_Data);
		 Temp_Data2 = *(sSource->Data_a8 + Pos_Convert++);
         Temp_Data2 = Convert_String_To_Hex (Temp_Data2);
         //Add byte to target
		 *(sTarget->Data_a8 + sTarget->Length_u16++) = Temp_Data * 16 + Temp_Data2;;
         //Max target return
         if (sTarget->Length_u16 == MaxLength)
             return;
	}
}


uint8_t Convert_String_To_Hex (uint8_t NumString)
{
    if (NumString < 0x3A)
        return (NumString - 0x30);
    else if (NumString < 0x61)
        return (NumString - 0x37);	
    else
        return (NumString - 0x57);	 
}

uint8_t Convert_Hex_To_StringHex (uint8_t Hex)
{
    return ((Hex /10) *16 + (Hex %10));
}


void UTIL_Printf (uint8_t Level, uint8_t *pData, uint16_t Length)
{
#if (DEBUG_ENABLED == 1)
    if (Level <= _VLEVEL_DEBUG)
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
    sData   sDataIn;
    uint8_t aTEMP_OUT[2] = {0};
    sData   sDataOut = {aTEMP_OUT, 0};
    
    for (uint16_t i = 0; i < Length; i++)
    {
        sDataIn.Data_a8 = pData + i;
        sDataIn.Length_u16 = 1;
        sDataOut.Length_u16 = 0;
        
        Convert_Hex_To_String_Hex(&sDataOut, &sDataIn); 
    
        UTIL_Printf(Level, sDataOut.Data_a8, sDataOut.Length_u16);
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



void UTIL_Printf_Dec (uint8_t Level, int32_t Number)
{
    uint8_t Buff[10] = {0};
    sData   strNum = {&Buff[0], 0};
    
    Convert_Int64_To_StringDec(&strNum, Number, 0);

    UTIL_Printf (Level, strNum.Data_a8 , strNum.Length_u16 );
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


double UTIL_Convert_String_To_Double (uint8_t *pData, uint8_t length)
{
    uint8_t    i = 0;
    
    uint32_t	TempU32 = 0;
    uint8_t 	Decimal = 0;
    uint8_t		MarkDecimal = false;
    uint8_t 	Sign =false;    
    double 		TempFloat = 0;
        
    for (i = 0; i < length ; i++)
    {
        //Danh dau so am
        if (pData[i] == '-')
        {
            Sign = true;	
            continue;
        } 

        //Danh dau vi tri decimal
        if (pData[i] == '.')
        {
            MarkDecimal = true;
            continue;
        }

        //Increase count decimal
        if (MarkDecimal == true)
        {
            Decimal++;
        }
        
        //Value
        if ( (pData[i] < 0x30) || ( pData[i] > 0x39 ) )     
        {
            return 0xFFFFFFFF;
        }
        
        TempU32 = TempU32 * 10 + pData[i] - 0x30;
    }
    
    if (Sign  == true)
    {
        TempFloat = 0 - TempU32 / ( pow (10, Decimal) ) ;
    } else
    {
        TempFloat = TempU32 / ( pow (10, Decimal) ) ;
    }
           
    return TempFloat;
}


/*
    Func: convert double to string
        + pData: target
        + Num:  double value
        + Decimal: set decimal expect
*/
void UTIL_Convert_Double_To_string (sData *pData, double Num, uint8_t Decimal)
{
    uint8_t i = 0;

    for (i = 0; i < Decimal; i++)
    {
        Num *= 10;
    }
    
    Convert_Int64_To_StringDec(pData, (int64_t) Num, Decimal);
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


