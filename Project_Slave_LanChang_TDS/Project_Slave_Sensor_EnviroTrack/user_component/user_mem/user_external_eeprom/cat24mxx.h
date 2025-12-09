

#ifndef CAT24MXX_H
#define CAT24MXX_H _CAT24MXX_H

#include "user_util.h"

#include "event_driven.h"
#include "queue_p.h"
#include "user_mem_comm.h"



/*============== Define ====================*/
#define USING_EX_EEPROM


#define CAT24_START_LOG             0
#define CAT24_END_LOG               16384 /* Cat24 128kbs */

#define I2C_CAT24                   hi2c1

#define NUMBER_RETRY_BUSY           20
#define TIME_RETRY_BUSY             10
#define NUMBER_RETRY_R_W            20

/* =============== Drivers ============== */
#define I2C_CAT24Mxx_ADDR_7BIT          0xA0        /* Don't care operation bit (R/W bit) - 1010A1A2A30b */
#define I2C_CAT24Mxx_SIZE               0x20000     /* Bytes */
#define I2C_CAT24Mxx_MAX_BUFF           64          /* Bytes */

/*=========== Define =================*/

#define CAT_SIZE_DATA_TSVH	            384        
#define CAT_SIZE_DATA_EVENT	            64         
#define CAT_SIZE_DATA_LOG	            64   
#define CAT_SIZE_DATA_GPS	            256   

#define	CAT_MAX_MESS_TSVH_SAVE			18
#define	CAT_MAX_MESS_EVENT_SAVE			10
#define CAT_MAX_MESS_LOG_SAVE           130 
#define CAT_MAX_MESS_GPS_SAVE           2  

#define CAT_BYTE_EMPTY                  0xFF


/* ================================ Address config DCU ==================================== */

//LOG Mess TSVH (16384 - 800) / 64 = 243: 100pack + 10Pack + 120pack
#define	CAT_ADDR_MESS_A_START   		64   // 832	            //50: 128 byte    
#define	CAT_ADDR_MESS_A_STOP			7232
//LOG Mess EVENT
#define	CAT_ADDR_EVENT_START   		    7296            //10 packet + 1 du
#define	CAT_ADDR_EVENT_STOP			    7936
//LOG Mess LOG
#define	CAT_ADDR_LOG_START   		    8000            //130 packet + 1 du
#define	CAT_ADDR_LOG_STOP			    16384

#define	CAT_ADDR_GPS_START   		    0            //130 packet + 1 du
#define	CAT_ADDR_GPS_STOP			    768

//Define Variable
#define	EEPROM_MAX_RETRY_WRITE          10

/*========== Var struct ===============*/

typedef enum
{
	_EVENT_CAT24Mxx_WRITE = 0,
	_EVENT_CAT24Mxx_READ,
	_EVENT_CAT24Mxx_END, // don't use
}eEVENT_CAT24;


/* ===================================== Struct =========================================== */
typedef struct
{
    uint8_t status_u8;
    uint8_t Count_err_u8;
    uint8_t Count_retry_u8;
}sFROM_Manager_Struct;


extern sFROM_Manager_Struct	sFROM_Manager;
//
extern sEvent_struct        sEventCAT24[];
extern sMemoryVariable      sCAT24Var;


/* ===================================== Function =========================================== */
uint8_t CAT24Mxx_Write_Array (uint16_t writeAddr, uint8_t* dataBuff, uint16_t dataLength);
uint8_t CAT24Mxx_Read_Array (uint16_t readAddr, uint8_t* dataBuff, uint16_t dataLength);


int32_t TestEEPROM(void); /* Test EEPROM */
uint8_t CAT24Mxx_Erase (void);
uint8_t CAT24Mxx_Write_Byte (uint16_t writeAddr, uint8_t *dataBuff);
uint8_t CAT24Mxx_Write_Word (uint16_t writeAddr, uint16_t *dataBuff);
uint8_t CAT24Mxx_Write_DoubleWord (uint16_t writeAddr, uint32_t *dataBuff);
uint8_t CAT24Mxx_Write_QuadWord (uint16_t writeAddr, uint64_t *dataBuff);
uint8_t CAT24Mxx_Read_Byte (uint16_t writeAddr, uint8_t *dataBuff);
uint8_t CAT24Mxx_Read_Word (uint16_t writeAddr, uint16_t *dataBuff);
uint8_t CAT24Mxx_Read_DoubleWord (uint16_t writeAddr, uint32_t *dataBuff);
uint8_t CAT24Mxx_Read_QuadWord (uint16_t writeAddr, uint64_t *dataBuff);
uint8_t CAT24Mxx_Check_Busy(uint8_t  addrA16bit, uint8_t Retry, uint8_t Timeout);
//*================ Func ============================*/

void    CAT24Mxx_Init (void);
uint8_t CAT24Mxx_Task (void);



#endif
