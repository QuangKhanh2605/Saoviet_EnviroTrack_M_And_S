/*
 * S25FL.h
 *
 *  Created on: Dec 15, 2021
 *      Author: lenovo
 */

#ifndef FLASH_EXT_S25FL_H_
#define FLASH_EXT_S25FL_H_

#include <stdint.h>
#include "spi.h"

#include "user_util.h"
#include "event_driven.h"
#include "queue_p.h"

#include "user_mem_comm.h"



/*============ Define =============*/ 
#define USING_EX_FLASH	
     
#define SKIP				0x00

#define LOW    	 			0x00  /* Chip Select line low */
#define HIGH    			0x01  /* Chip Select line high */

#define S25FL_READ          0x03
#define S25FL_WRITE         0x02
#define S25FL_READ_STATUS   0x05
#define S25FL_WREN          0x06
#define S25FL_SECTOR_ERASE  0x20

#define S25FL_BLOCK_ERASE   0xD8
#define S25FL_CHIP_ERASE    0xC7
#define S25FL_JEDEC         0x9F  // 0x90
#define S25FL_WRITE_DISABLE 0x04

#define S25FL_VOLATILE_WREN 0x50
#define S25FL_RSTEN         0x66
#define S25FL_RST           0x99

#define WIP_Flag   		    0x03 // 0x01 vs 0x02 /* Write In Progress (WIP) flag */
#define WEL_Flag   		    0x02 /* Chua co check Flag WEL */
#define DUMMY_BYTE 		    0xA5

/* Define Size */
#define S25FL_PAGE_SIZE 	256
#define S25FL_SECTOR_SIZE 	4096

#define SPI_NSS_GPIO_Port       ETH_SCSn_GPIO_Port  
#define SPI_NSS_Pin             ETH_SCSn_Pin  

#define FLASH_ON_OFF_GPIO_Port  ETH_ON_OFF_GPIO_Port
#define FLASH_ON_OFF_Pin        ETH_ON_OFF_Pin

#define FLASH_RESET_GPIO_Port   ETH_ON_OFF_GPIO_Port //EFLASH_HOLD_GPIO_Port
#define FLASH_RESET_Pin         ETH_ON_OFF_Pin  //EFLASH_HOLD_Pin

#define FLASH_POWER_OFF     HAL_GPIO_WritePin(FLASH_ON_OFF_GPIO_Port, FLASH_ON_OFF_Pin, GPIO_PIN_SET)    
#define FLASH_POWER_ON      HAL_GPIO_WritePin(FLASH_ON_OFF_GPIO_Port, FLASH_ON_OFF_Pin, GPIO_PIN_RESET)      


#define hSPI                    hspi3   // hspi2

#define TIMEOUT_ER_SECTOR				1000
#define TIMEOUT_WR_PAGE					100
#define TIMEOUT_WR_ENABLE				1000

#define FLASH_CMD_TIMEOUT               (200)
#define FLASH_CMD_TIME 	                (1)
#define FLASH_WRITE_TIME 	            5
#define FLASH_ERASE_TIME                100
#define FLASH_ERASE_ALL_TIME            120000

/*============ Address ============*/
#define FLASH_S25FL_BASE 	0x00000000
#define FLASH_S25FL_END 	0x01000000    //16 Mbyte = 128 Mbit

#define FLASH_ADDR_TEST 		        0x000000		//Sector 0

#define FLASH_ADDR_TSVH_A_START 		0x001000		//Sector 1    //  Neu 10p 1 ban tin. can 1080 sec
#define FLASH_ADDR_TSVH_A_STOP			0x871000		//Sector 2161 // 4 thang = 2880 hour. 1 record = 256 byte.

#define FLASH_ADDR_TSVH_B_START 		FLASH_ADDR_TSVH_A_START		
#define FLASH_ADDR_TSVH_B_STOP			FLASH_ADDR_TSVH_A_STOP		

#define FLASH_ADDR_TSVH_OFFSET			FLASH_ADDR_TSVH_B_START - FLASH_ADDR_TSVH_A_START

#define FLASH_ADDR_EVENT_A_START 		0x871000		//Sector 1081
#define FLASH_ADDR_EVENT_A_STOP			0x875000		//Sector 1085  //16kb  -> 64 byte * 200 rec (thua ra)

#define FLASH_ADDR_EVENT_B_START 		FLASH_ADDR_EVENT_A_START
#define FLASH_ADDR_EVENT_B_STOP			FLASH_ADDR_EVENT_A_STOP	

#define FLASH_ADDR_EVENT_OFFSET   		FLASH_ADDR_EVENT_B_START - FLASH_ADDR_EVENT_A_START	

#define FLASH_ADDR_LOG_START 			0x875000		//Sector 1085		
#define FLASH_ADDR_LOG_STOP				0x895000		//Sector 1101  //256 kbyte = 4000 rec. moi rec 64 byte

#define FLASH_ADDR_GPS_A_START 			FLASH_S25FL_BASE	
#define FLASH_ADDR_GPS_A_STOP			FLASH_S25FL_BASE	

#define FLASH_ADDR_GPS_B_START 			FLASH_S25FL_BASE		
#define FLASH_ADDR_GPS_B_STOP			FLASH_S25FL_BASE	

#define FLASH_ADDR_GPS_OFFSET   		FLASH_ADDR_GPS_B_START -  FLASH_ADDR_GPS_A_START 

/*--------- Max size record ----------*/
#define FLASH_MAX_RECORD_TSVH           17280
#define FLASH_MAX_RECORD_EVENT          100
#define FLASH_MAX_RECORD_LOG            2000
#define FLASH_MAX_RECORD_GPS            10000

#define FLASH_SIZE_DATA_TSVH            256
#define FLASH_SIZE_DATA_EVENT           64
#define FLASH_SIZE_DATA_LOG             64
#define FLASH_SIZE_DATA_GPS             256

#define FLASH_MAX_RETRY_ITEM            3
#define FLASH_MAX_RETRY_SKIP            6


#define FLASH_MAX_RETRY_TEST            3    

/*===================Var struct======================================*/
                

typedef enum
{
	_EVENT_FLASH_SEND_BYTE = 0,
	_EVENT_FLASH_WRITE_BUFF,
	_EVENT_FLASH_READ_BUFF,

	_EVENT_FLASH_END, // don't use
}Event_Flash_TypeDef;

typedef enum
{

	_FLASH_CMD_READ_STATUS = 0,
	_FLASH_CMD_CHECK_WRITE_ENABLE,
	_FLASH_CMD_CHECK_ERASE_ENABLE,

	_FLASH_CMD_CHECK_WRITE_END,
	_FLASH_CMD_CHECK_ERASE_END,

	_FLASH_CMD_ERASE_SECTOR,
	_FLASH_CMD_WRITE_DATA,
	_FLASH_CMD_READ_DATA,

	_FLASH_CMD_ACTIVE_CS,
	_FLASH_CMD_DEACTIVE_CS,

	_FLASH_CMD_ENABLE,
    
    _FLASH_CMD_WRITE_DELAY,
    _FLASH_CMD_ERASE_DELAY,
    _FLASH_CMD_READ_CHECK_WRITE,
    
    _FLASH_CMD_ERASE_CHIP,
    _FLASH_CMD_ERASE_CHIP_DELAY,
    _FLASH_CMD_ERASE_CHIP_END,
    
	_FLASH_CMD_END, // don't use
}Cmd_Flash_TypeDef;


typedef uint8_t (*fp_flash_cmd_callback) (void);

typedef struct
{
	uint8_t cmd_name;
	uint8_t cmd_byte;
	uint8_t cmd_check;
	fp_flash_cmd_callback callback_success;
	fp_flash_cmd_callback callback_failure;
}sFlash_CMD_struct;


typedef struct
{
    uint8_t         StepInit_u8;
    uint8_t         Pending_u8;
    uint8_t         Status_u8;
    
    uint32_t        PosFindData_u32;
    uint8_t         FlagMark_u8;
    uint8_t         IsFullArea_u8;
    uint8_t         IsEmptyArea_u8;
}sFlashInitIndexRecordVar;


extern sEvent_struct        sEventExFlash[];
extern sFlash_CMD_struct	aFlashCmd[];

extern Struct_Queue_Type    qFlashStep;
extern Struct_Queue_Type    qFlashRead;
extern Struct_Queue_Type    qFlashWrite;

extern sMemoryVariable      sS25FLvar;
extern sFlashInitIndexRecordVar        sS25FLInit;




/*=========== Function =============*/
void    S25FL_ChipSelect(uint8_t State);
uint8_t S25FL_Send_Byte(uint8_t byte);

void    S25FL_Send_Address (uint32_t address);

/*========== Func external flash ===========*/
void    eFlash_Init (void);
uint8_t eFlash_Task (void);

void    eFlash_Send_Address(uint32_t address);

uint8_t eFlash_Push_Step_To_Queue(uint8_t step);
uint8_t eFlash_Push_Block_To_Queue(uint8_t *block_flash_step, uint8_t size);
uint8_t eFlash_Get_Step_From_Queue(uint8_t Type);

/*========== Func test ===========*/

void    eFlash_S25FL_Enable(void);
uint8_t eFlash_S25FL_Erase_Sector(uint32_t SectorAddr);
uint8_t eFlash_S25FL_CheckWriteEnable(uint32_t timeout);
uint8_t eFlash_S25FL_WaitForWriteEnd(uint32_t timeout);
uint8_t eFlash_S25FL_PageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
uint8_t eFlash_S25FL_BufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
uint8_t eFlash_S25FL_BufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);

/*====== Func Handle record =========*/
void    eFlash_Default_Struct_Init_Pos (void);
void    eFlash_Push_Step_Erase_Sector (void);
void    eFlash_Push_Step_Erase_Chip (void);

uint8_t eFlash_Test_Write (void);

void    eFlash_Check_Data_Init_Pos (uint32_t Addr, uint8_t *pData, uint16_t Length, uint8_t PartAorB);
void    eFlash_Erase_Sector (uint32_t Addr);

void eFlash_Test (void);

#endif /* FLASH_EXT_S25FL_H_ */






