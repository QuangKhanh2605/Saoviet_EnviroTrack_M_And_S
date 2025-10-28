#include "user_convert_variable.h"
#include "math.h"
/*
    Chuyen Scale sang Uint
*/
uint32_t Calculator_Scale(uint8_t Scale)
{
    /*
    Scale
    00->1 
    FF->0.1 
    FE->0.01 
    FD->0.001  
    FC->0.0001  
    FB->0.00001 
    */
  
    if(Scale == 0x00) return 1;
    else if(Scale == 0xFF) return 10;
    else if(Scale == 0xFE) return 100;
    else if(Scale == 0xFD) return 1000;
    else if(Scale == 0xFC) return 10000;
    else if(Scale == 0xFB) return 100000;
    
    return 1;
}

/*
    @brief  Tinh tan so theo Scale
    @retval Tan so nap vao bien tan
*/
uint32_t Calculator_Value_Scale(uint32_t Value, uint8_t Scale, uint8_t Scale_Default)
{
      /*
    Scale
    00->1 
    FF->0.1 
    FE->0.01 
    FD->0.001  
    FC->0.0001  
    FB->0.00001 
    */
  uint64_t Stamp_Scale = Value * Calculator_Scale(Scale_Default);
  return Stamp_Scale/Calculator_Scale(Scale);
}

/*=============== Convert Int To String ==============*/

/*
    @brief  Convert Int to String
    @return Length var
*/
uint8_t Convert_Int_To_String(uint8_t cData[], int var)
{
    uint8_t length = 0;
    uint32_t stamp = 0;
    uint64_t division = 10;
    if(var < 0)
    {
        stamp = 0 - var;
        cData[length++] = '-';
    }
    else
    {
        stamp = var;
    }
    
    while( stamp/division > 0 )
    {
        division *= 10;
    }
    division = division/10;
    while(division > 0)
    {
        cData[length++] = stamp/division + 0x30;
        stamp = stamp - (stamp/division) * division;
        division = division/10;
    }
    
    return length;
}

/*
    @brief  Convert Int to String(Scale)
    @return Length Data
*/
uint8_t Convert_Int_To_String_Scale(uint8_t cData[], int var, uint8_t Scale)
{
    uint8_t length = 0;
    uint32_t division = 0;
    uint32_t stamp = 0;
    uint8_t size_cData = 0;
    if(var < 0)
    {
        stamp = 0 - var;
    }
    else
    {
        stamp = var;
    }
    
    length = Convert_Int_To_String(cData, var);
    if(stamp != 0)
    {
        division = Calculator_Scale(Scale);
        while(stamp < division)
        {
            stamp *=10;
            size_cData++;
            if(stamp == 0) break;
        }
        while(size_cData > 0)
        {
            for(uint8_t i = length; ; i--)
            {
                if(cData[i-1] == '-' ||i == 0)
                {
                    cData[i] = '0';
                    break;
                }
                else
                {
                    cData[i] = cData[i-1];
                }
            }
            length++;
            size_cData--;
        }
        division = division/10;
        size_cData = length;
        while(division > 0)
        {
            division = division/10;
            cData[size_cData] = cData[size_cData-1];
            size_cData--;
            if(division == 0) 
            {
              cData[size_cData]='.';
              length++;
            }
        }
    }
    return length;
}

//---------------------- Convert Int to String Point ---------------------
/*
    @brief  Convert Int to String at Point
    @param  *Pos location in array cData
    @return Length Data
*/
uint8_t Convert_Point_Int_To_String(uint8_t cData[], uint16_t *Pos, int var)
{
    uint16_t length = 0;
    uint32_t stamp = 0;
    uint64_t division = 10;
    length = *Pos;
    if(var < 0)
    {
        stamp = 0 - var;
        cData[length++] = '-';
    }
    else
    {
        stamp = var;
    }
    
    while( stamp/division > 0 )
    {
        division *= 10;
    }
    division = division/10;
    while(division > 0)
    {
        cData[length++] = stamp/division + 0x30;
        stamp = stamp - (stamp/division) * division;
        division = division/10;
    }
    
    *Pos = length;
    return length;
}
/*
    @brief  Convert Int to String at Point (Scale)
    @param  *Pos location in array cData
    @return Length Data
*/
uint8_t Convert_Point_Int_To_String_Scale(uint8_t cData[], uint16_t *Pos, int var, uint8_t Scale)
{
    uint16_t length = 0;            //Vi tri du lieu 
    uint16_t location_pos = 0;      //Vi tri dau tien cua Pos
    uint32_t division = 0;          //Kiem tra do dai du lieu cua variable tu scale
    uint32_t stamp = 0;             //Stamp cua variable
    uint8_t size_cData = 0;
    
    location_pos = *Pos;
    if(var < 0)
    {
        stamp = 0 - var;
    }
    else
    {
        stamp = var;
    }
    
    Convert_Point_Int_To_String(cData, Pos, var);
    length = *Pos;
    if(stamp != 0)
    {
        division = Calculator_Scale(Scale);
        while(stamp < division)
        {
            stamp *=10;
            size_cData++;
            if(stamp == 0) break;
        }
        while(size_cData > 0)
        {
            for(uint8_t i = length; ; i--)
            {
                if(cData[i-1] == '-' ||i == location_pos)
                {
                    cData[i] = '0';
                    break;
                }
                else
                {
                    cData[i] = cData[i-1];
                }
            }
            length++;
            size_cData--;
        }
        division = division/10;
        size_cData = length;
        while(division > 0)
        {
            division = division/10;
            cData[size_cData] = cData[size_cData-1];
            size_cData--;
            if(division == 0) 
            {
              cData[size_cData]='.';
              length++;
            }
        }
    }
    *Pos = length;
    return length;
}

//---------------------Convert Variable Packet Integer ----------------------

/*
    @brief  Dong goi bien Integer vao mang
*/
void Convert_Var_Packet_Integer (uint8_t *pTarget, uint16_t *LenTarget, uint32_t Data)
{
    uint16_t Pos = *LenTarget;
    
    uint64_t division = 10;
    
    while( Data/division > 0 )
    {
        division *= 10;
    }
    division = division/10;
    while(division > 0)
    {
        pTarget[Pos++] = Data/division + 0x30;
        Data = Data - (Data/division) * division;
        division = division/10;
    }
    *LenTarget = Pos ;
}

//------------------------Insert String------------------------
/*
    @brief  Insert String to String
*/
void Insert_String_To_String(uint8_t destination[], uint16_t *PosDes, uint8_t source[], uint16_t PosSour, uint32_t length)
{
    for(uint16_t i = 0; i< length; i++)
    {
        destination[(*PosDes)++] = source[PosSour + i];
    }
}

//------------------------Convert uint32Hex To Float------------------------
/*
    @brief  Convert uint32Hex To Float
*/
void Convert_uint32Hex_To_Float(uint32_t uint32Hex, float *Var)
{
    float stamp = 0;
    uint8_t check_bit = 0;
    int8_t stamp_Sign = 1;
    uint8_t stamp_Exponent = 0;
    float   stamp_Mantissa = 1;
    for(uint8_t i = 0; i<32; i++)
    {
        check_bit = 0x01 & (uint32Hex>>i);
        if(i<23)
        {
            stamp_Mantissa += check_bit * pow(2, (-1)*(23-i));
        }
        else if(i<31)
        {
            stamp_Exponent +=  check_bit * pow(2, i-23);
        }
        else
        {
            if(check_bit == 1)  stamp_Sign = -1;
        }
    }
    stamp = stamp_Sign * pow(2, stamp_Exponent -127)*stamp_Mantissa;
    *Var = stamp;
}

uint32_t Convert_String_To_Dec(uint8_t *pData, uint8_t lenData)
{
    uint8_t     index = 0;
    uint8_t     tempData = 0;
    uint32_t    reVal = 0;

    for (index = 0; index < lenData; index++)
    {
        if (('0' <= *(pData + index)) && (*(pData + index) <= '9'))
        {
            tempData = (*(pData + index) - 0x30);
        } else
            tempData = 0;
        
        if (index == 0)
            reVal = tempData;
        else
            reVal = (reVal * 10) + tempData;
    }

    return reVal;
}

int32_t Hanlde_Float_To_Int32_Scale_Round(float varFloat, uint8_t scale)
{
    float f_hexFloat = 0;
    int32_t u_hexFloat = 0;
    
    uint32_t division = 1;
    uint32_t stamp_div = 0;
    float f_compensation = 0;

    f_hexFloat = varFloat;
    
    switch(scale)
    {
        case 0x00:
          division = 1;
          f_compensation = 0;
          break;
          
        case 0xFF:
          division = 10;
          f_compensation = 0.1;
          break;
          
        case 0xFE:
          division = 100;
          f_compensation = 0.01;
          break;
          
        case 0xFD:
          division = 1000;
          f_compensation = 0.001;
          break;
         
        case 0xFC:
          division = 10000;
          f_compensation = 0.0001;
          break;
          
        case 0xFB:
          division = 100000;
          f_compensation = 0.00001;
          break;
          
        default:
          break;
    }
   
    if(f_hexFloat >= 0)    
    {
      stamp_div = (uint32_t)(f_hexFloat*division*10)%10;
      if(stamp_div >= 5) f_hexFloat += f_compensation;
    }
    else                
    {
      stamp_div = (uint32_t)((0-f_hexFloat)*division*10)%10;
      if(stamp_div >= 5) f_hexFloat -= f_compensation;
    }
    
    u_hexFloat = (int32_t)(f_hexFloat*division);
    return u_hexFloat;
}

int32_t Handle_HexFloat_To_Int32_Round(uint32_t hexFloat, uint8_t scale)
{
    float f_hexFloat = 0;
    int32_t u_hexFloat = 0;
    
    uint32_t division = 1;
    uint32_t stamp_div = 0;
    float f_compensation = 0;
  
    Convert_uint32Hex_To_Float(hexFloat, &f_hexFloat);
    
    switch(scale)
    {
        case 0x00:
          division = 1;
          f_compensation = 0;
          break;
          
        case 0xFF:
          division = 10;
          f_compensation = 0.1;
          break;
          
        case 0xFE:
          division = 100;
          f_compensation = 0.01;
          break;
          
        case 0xFD:
          division = 1000;
          f_compensation = 0.001;
          break;
         
        case 0xFC:
          division = 10000;
          f_compensation = 0.0001;
          break;
          
        case 0xFB:
          division = 100000;
          f_compensation = 0.00001;
          break;
          
        default:
          break;
    }
   
    if(f_hexFloat >= 0)    
    {
      stamp_div = (uint32_t)(f_hexFloat*division*10)%10;
      if(stamp_div >= 5) f_hexFloat += f_compensation;
    }
    else                
    {
      stamp_div = (uint32_t)((0-f_hexFloat)*division*10)%10;
      if(stamp_div >= 5) f_hexFloat -= f_compensation;
    }
    
    u_hexFloat = (int32_t)(f_hexFloat*division);
    return u_hexFloat;
}

float Handle_int32_To_Float_Scale(int32_t var, uint8_t scale)
{
    float result = 0;
    switch(scale)
    {
        case 0x00:
          result = var;
          break;
          
        case 0xFF:
          result = var/10;
          result = result + (var%10)*0.1;
          break;
          
        case 0xFE:
          result = var/100;
          result = result + (var%100)*0.01;
          break;
          
        case 0xFD:
          result = var/1000;
          result = result + (var%1000)*0.001;
          break;
         
        case 0xFC:
          result = var/10000;
          result = result + (var%10000)*0.0001;
          break;
          
        case 0xFB:
          result = var/100000;
          result = result + (var%100000)*0.00001;
          break;
          
        default:
          break;
    }
    return result;
}

uint32_t Handle_Float_To_hexUint32(float num)
{
    uint32_t u_hexFloat = 0;
    unsigned char *bytePtr = (unsigned char*)&num;

    for (int i = 0; i < sizeof(float); i++) 
    {
        u_hexFloat = u_hexFloat | (bytePtr[i] << 8*i);
    }
    return u_hexFloat;
}
