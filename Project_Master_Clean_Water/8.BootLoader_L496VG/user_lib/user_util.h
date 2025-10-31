/*
    8/2021
    Thu vien Quan ly cac function convert
*/

#if defined (STM32L151xC)  
    #include "stm32l1xx_hal.h"
    #include "stm32l1xx.h"

    #define FLASH_BYTE_WRTIE    4
    #define FLASH_BYTE_EMPTY    0x00   
#endif

#if defined (STM32L433xx) || defined (STM32L452xx) || defined (STM32L496xx)  
    #include "stm32l4xx_hal.h"
    #include "stm32l4xx.h"
    
    #define FLASH_BYTE_EMPTY    0xFF   
    #define FLASH_BYTE_WRTIE    8
#endif

#if defined (STM32L072xx) || defined (STM32L082xx)
    #include "stm32l0xx_hal.h"
    #include "stm32l0xx.h"

    #define FLASH_BYTE_WRTIE    4
    #define FLASH_BYTE_EMPTY    0x00   
#endif

#include "main.h"
#include "string.h"
#include "stdio.h"

#ifndef USER_UTIL_H
#define USER_UTIL_H


/*===================Define=========================*/
#define TRUE 	        1
#define FALSE 	        0
#define PENDING	        2
#define ERROR           3

#define true			1
#define false			0
#define pending			2
#define error			3
#define waiting			4

#define DEBUG_ENABLED   1       //Cho phep DEBUG printf

#define DBLEVEL_L       1       /*!< just essential traces */
#define DBLEVEL_M       2       /*!< functional traces */
#define DBLEVEL_H       3       /*!< all traces */

#define _VLEVEL_DEBUG   DBLEVEL_M

//#define ENABLE_TEST_HARDWARE

#define pi 3.14159265358979323846


#define UTILS_INIT_CRITICAL_SECTION()

/**
  * @brief macro used to enter the critical section
  */
#define UTILS_ENTER_CRITICAL_SECTION() uint32_t primask_bit= __get_PRIMASK();\
  __disable_irq()

/**
  * @brief macro used to exit the critical section
  */
#define UTILS_EXIT_CRITICAL_SECTION()  __set_PRIMASK(primask_bit)

/*!
 * \brief Returns the minimum value between a and b
 *
 * \param [IN] a 1st value
 * \param [IN] b 2nd value
 * \retval minValue Minimum value
 */
#ifndef MIN
    #define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#endif

/*!
 * \brief Returns the maximum value between a and b
 *
 * \param [IN] a 1st value
 * \param [IN] b 2nd value
 * \retval maxValue Maximum value
 */
#ifndef MAX
    #define MAX( a, b ) ( ( ( a ) > ( b ) ) ? ( a ) : ( b ) )
#endif

      
#define TS_OFF 0         /*!< Log without TimeStamp */
#define TS_ON 1          /*!< Log with TimeStamp */
#define T_REG_OFF  0     /*!< Log without bitmask */
      
#define APP_PPRINTF(...)  do{ } while( UTIL_ADV_TRACE_OK \
                              != UTIL_ADV_TRACE_COND_FSend(VLEVEL_ALWAYS, T_REG_OFF, TS_OFF, __VA_ARGS__) ) /* Polling Mode */
#define APP_TPRINTF(...)   do{ {UTIL_ADV_TRACE_COND_FSend(VLEVEL_ALWAYS, T_REG_OFF, TS_ON, __VA_ARGS__);} }while(0); /* with timestamp */
#define APP_PRINTF(...)   do{ {UTIL_ADV_TRACE_COND_FSend(VLEVEL_ALWAYS, T_REG_OFF, TS_OFF, __VA_ARGS__);} }while(0);


#if defined (DEBUG_ENABLED) && (DEBUG_ENABLED == 1)
    #define APP_LOG(TS,VL,...)   do{ {UTIL_ADV_TRACE_COND_FSend(VL, T_REG_OFF, TS, __VA_ARGS__);} }while(0);
#elif defined (DEBUG_ENABLED) && (DEBUG_ENABLED == 0)
    #define APP_LOG(TS,VL,...)
#else

#endif /* DEBUG_ENABLED */

/*===================Struct, Var=========================*/
typedef struct 
{
	uint8_t			*Data_a8;					// string
	uint16_t		Length_u16;
} sData;

typedef struct 
{
	uint8_t hour;  // read hour
	uint8_t min;   // read minute
	uint8_t sec;   // read second
    uint8_t day;   // read day
    uint8_t date;  // read date
    uint8_t month; // read month
    uint8_t year;  // read year
    uint32_t SubSeconds;
} ST_TIME_FORMAT;


typedef enum
{
    _POWER_MODE_SAVE = 0,
    _POWER_MODE_ONLINE,
}sKind_Mode_Modem;


extern uint8_t (*pFunc_UTIL_Log_To_Mem) (uint8_t *pData, uint16_t Length);
extern void (*pFunc_UTIL_Debug_Send) (uint8_t *pData, uint16_t Length);

/*===================extern struct, Var=========================*/
extern uint32_t        RtCountSystick_u32;
extern ST_TIME_FORMAT  sRTC;
/*===================Function=========================*/
uint32_t    Cal_Time(uint32_t Millstone_Time, uint32_t Systick_now);
uint32_t    Cal_Time_s(uint32_t Millstone_Time, uint32_t Systick_now);
uint8_t     Check_Time_Out(uint32_t Millstone_Time, uint32_t Time_Period_ms);
uint8_t     Check_Time_Out_2(uint32_t Millstone_Time, uint32_t Time_Period_ms);

float       Convert_FloatPoint_2Float (uint32_t Float_Point);
int32_t     Convert_float_2int (uint32_t Float_Point_IEEE, uint8_t scale);
int32_t     Convert_uint_2int (uint32_t Num);

void        Scale_String_Dec (sData *StrDec, uint8_t Scale);
int16_t     Convert_uint16_2int16 (uint16_t Num);
int64_t     Convert_uint64_2int64 (uint64_t Num);
uint8_t 	BBC_Cacul (uint8_t* Buff, uint16_t length);
uint8_t 	Check_BBC (sData* Str);

void        Reset_Buff(sData *str);
void        Convert_Int64_To_StringDec (sData* Payload, int64_t Data, uint8_t Scale);
void        Convert_Uint64_To_StringDec (sData* Payload, uint64_t Data, uint8_t Scale);
void 		Convert_Hex_To_String_Hex(sData *sTaget, sData *sSource);
void        Convert_String_Hex_To_Hex (sData *sSource, int Pos_Begin, sData *sTarget, uint16_t MaxLength);
uint8_t     Convert_String_To_Hex (uint8_t NumString);
uint32_t    Convert_String_To_Dec(uint8_t *pData, uint8_t lenData);

void        UTIL_MEM_set( void *dst, uint8_t value, uint16_t size );
void        UTIL_MEM_cpy( void *dst, const void *src, uint16_t size );
void        UTIL_MEM_cpyr( void *dst, const void *src, uint16_t size );
uint8_t     Convert_Hex_To_StringHex (uint8_t Hex);

void        UTIL_Log (uint8_t Level, uint8_t *pData, uint16_t Length);
void        UTIL_Log_Str (uint8_t Level, const char *str);

void        UTIL_Printf (uint8_t Level, uint8_t *pData, uint16_t Length);
void        UTIL_Printf_Str (uint8_t Level, const char *str);
void        UTIL_Printf_Hex (uint8_t Level, uint8_t *pData, uint16_t Length);
void        UTIL_Printf_Dec (uint8_t Level, int32_t Number);


double      UTIL_Rad_2Deg(double rad); 
double      UTIL_Deg_2Rad (double deg);
double      UTIL_Cacul_Distance (double lat1, double lon1, double lat2, double lon2, char unit); 

double      UTIL_Convert_String_To_Double (uint8_t *pData, uint8_t length);
void        UTIL_Convert_Double_To_string (sData *pData, double Num, uint8_t Decimal);
double      UTIL_Convert_GNRMC_To_Location (double GNRMC_Val);
uint8_t     UTIL_Convert_IP_To_Buff (sData *sIP, uint8_t *aIP_BUF);
uint32_t    UTIL_Get_Num_From_Str(sData *strRecei, uint16_t *Pos);

#endif
