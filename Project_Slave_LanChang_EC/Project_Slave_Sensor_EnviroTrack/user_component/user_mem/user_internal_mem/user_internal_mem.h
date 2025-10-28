/*
    8/2021
    Thu vien xu ly Uart

    + Note:
        ++ Define MCU trong IDE IAR
        ++ Define Board phan cung: doi voi cung 1 loai MCU
*/

#ifndef USER_INTERNAL_ONFLASH_H
#define USER_INTERNAL_ONFLASH_H

#include "user_util.h"
#include "onchipflash.h"

#include "event_driven.h"
#include "queue_p.h"

#include "user_mem_comm.h"



//===============Define address=================

#define USING_INTERNAL_MEM

#define BOARD_READ_PULSE

#if defined (STM32L433xx)
    #define ON_FLASH_BASE               0x08000000
    #define	ADDR_FLASH_END			    0x0803F800
    /*========Add Firmware: Main + Update=========== */
    #define	ADDR_MAIN_PROGRAM			0x08004000	        //56 Page cho chuong trinh chinh = 112k
    #define	ADDR_UPDATE_PROGRAM			0x08020000	        //56 page -> End Program = 0x0803C000

    /*========Add Record Mess ================ */
    #define	ADDR_MESS_A_START   		0x0803C000	         //k dung
    #define	ADDR_MESS_A_STOP			0x0803C000

    #define	ADDR_EVENT_START   		    0x0803C000	         //k dung
    #define	ADDR_EVENT_STOP			    0x0803C000

    #define	ADDR_LOG_START   		    0x0803C000	         //k dung
    #define	ADDR_LOG_STOP			    0x0803C000           
    
    #define	ADDR_GPS_START   		    0x0803C000	         //k dung
    #define	ADDR_GPS_STOP			    0x0803C000           

    /*=======Add Index Mess================ */
    #define	ADDR_INDEX_REC 		        0x0803C800
    /* ============= Addr Meter config======== */       
    #define	ADDR_THRESH_MEAS			0x0803D000	
    #define	ADDR_METER_NUMBER			0x0803D800
    /* ============= Addr Modem config ======== */
    #define	ADDR_SERVER_INFOR			0x0803E000
    #define	ADDR_FREQ_ACTIVE    	    0x0803E800
    #define ADDR_DCUID         			0x0803F000      
    
    #define ADDR_FLAG_HAVE_NEW_FW       0x0803F800

    #define ADDR_INFOR_SLAVE_MODBUS_RTU 0x08020000
    #define ADDR_CALIB_TEMPERATURE      0x08020800
    #define ADDR_TEMPERATURE_ALARM      0x08021000
    #define ADDR_CALIB_DAC              0x08021800

    #define	FLASH_MAX_MESS_TSVH_SAVE	0
    #define	FLASH_MAX_MESS_EVENT_SAVE   0
    #define FLASH_MAX_MESS_LOG_SAVE     0

    #define MAX_SIZE_FIRMWARE           112     //Kb
    #define FIRMWARE_SIZE_PAGE	        MAX_SIZE_FIRMWARE*1024/FLASH_PAGE_SIZE

    #define SIZE_DATA_TSVH	            128        
    #define SIZE_DATA_EVENT	            64         
    #define SIZE_DATA_LOG	            64         
#endif



#if defined (STM32L452xx) 
    #define ON_FLASH_BASE               0x08000000
    #define	ADDR_FLASH_END			    0x0807F800
    /*========Add Firmware: Main + Update=========== */
    #define	ADDR_MAIN_PROGRAM			0x08004000	        //90 Page cho chuong trinh chinh = 180k
    #define	ADDR_UPDATE_PROGRAM			0x08032000	        //90 page -> End Program = 0x0805F000

    /*========Add Record Mess ================ */
    #define	ADDR_MESS_A_START   		0x0807A000	         //k dung
    #define	ADDR_MESS_A_STOP			0x0807A000

    #define	ADDR_EVENT_START   		    0x0807A000	         //k dung
    #define	ADDR_EVENT_STOP			    0x0807A000

    #define	ADDR_LOG_START   		    0x0807A000	         //k dung
    #define	ADDR_LOG_STOP			    0x0807A000           
    
    #define	ADDR_GPS_START   		    0x0807A000	         //k dung
    #define	ADDR_GPS_STOP			    0x0807A000           

    /*=======Add Index Mess================ */
    #define	ADDR_INDEX_REC 		        0x0807A800
    /* ============= Addr Meter config======== */  

    #define	ADDR_PRESS_INFOR			0x0807C000
    #define	ADDR_MODBUS_INFOR			0x0807C800
    #define	ADDR_THRESH_MEAS			0x0807D000	
    #define	ADDR_METER_NUMBER			0x0807D800 
    /* ============= Addr Modem config ======== */
    #define	ADDR_SERVER_INFOR			0x0807E000
    #define	ADDR_FREQ_ACTIVE    	    0x0807E800
    #define ADDR_DCUID         			0x0807F000      
    
    #define ADDR_FLAG_HAVE_NEW_FW       0x0807F800

    #define	FLASH_MAX_MESS_TSVH_SAVE	0
    #define	FLASH_MAX_MESS_EVENT_SAVE   0
    #define FLASH_MAX_MESS_LOG_SAVE     0

    #define MAX_SIZE_FIRMWARE           180     //Kb
    #define FIRMWARE_SIZE_PAGE	        MAX_SIZE_FIRMWARE*1024/FLASH_PAGE_SIZE

    #define SIZE_DATA_TSVH	            128        
    #define SIZE_DATA_EVENT	            64         
    #define SIZE_DATA_LOG	            64         
#endif





#if defined (STM32L496xx) 
    #define ON_FLASH_BASE               0x08000000
    #define	ADDR_FLASH_END			    0x080FF800
    /*========Add Firmware: Main + Update=========== */
//    #define	ADDR_MAIN_PROGRAM			0x08004000	        //90 Page cho chuong trinh chinh = 180k
//    #define	ADDR_UPDATE_PROGRAM			0x08032000	        //90 page -> End Program = 0x0805F000
    #define	ADDR_MAIN_PROGRAM			0x08004000	        //150 Page cho chuong trinh chinh = 256k
    #define	ADDR_UPDATE_PROGRAM			0x080B4800	        //150 page -> End Program = 0x080FF800
    /*========Add Record Mess ================ */
    #define	ADDR_MESS_A_START   		0x08060000	         //92k = 184 rec * 512
    #define	ADDR_MESS_A_STOP			0x08077000

    #define	ADDR_EVENT_START   		    0x08077000	         //4k = 64 rec * 64
    #define	ADDR_EVENT_STOP			    0x08078000

    #define	ADDR_LOG_START   		    0x08078000	         //8k = 128 rec * 64
    #define	ADDR_LOG_STOP			    0x0807A000           
    
    #define	ADDR_GPS_START   		    0x0807A000	         //k dung
    #define	ADDR_GPS_STOP			    0x0807A000           

    /*=======Add Index Mess================ */
    #define	ADDR_INDEX_REC 		        0x0807A800
    /* ============= Addr Meter config======== */  

    #define	ADDR_PRESS_INFOR			0x0807C000
    #define	ADDR_MODBUS_INFOR			0x0807C800
    #define	ADDR_THRESH_MEAS			0x0807D000	
    #define	ADDR_METER_NUMBER			0x0807D800 
    /* ============= Addr Modem config ======== */
    #define	ADDR_SERVER_INFOR			0x0807E000
    #define	ADDR_FREQ_ACTIVE    	    0x0807E800
    #define ADDR_DCUID         			0x0807F000      
    
    #define ADDR_FLAG_HAVE_NEW_FW       0x0807F800
    
    #define ADDR_TIME_WARNING_SENSOR    0x08080000
    
    #define ADDR_CONST_PH_COMPENSATION  0x08080800    
    #define ADDR_CALIB_CHLORINE         0x08081000
    #define ADDR_CALIBPINT_CHLORINE     0x08081800

    #define ADDR_OFFSET_MEASURE         0x0808F000

    #define	FLASH_MAX_MESS_TSVH_SAVE	180
    #define	FLASH_MAX_MESS_EVENT_SAVE   16
    #define FLASH_MAX_MESS_LOG_SAVE     120

    #define MAX_SIZE_FIRMWARE           300     //Kb
    #define FIRMWARE_SIZE_PAGE	        MAX_SIZE_FIRMWARE*1024/FLASH_PAGE_SIZE

    #define SIZE_DATA_TSVH	            512        
    #define SIZE_DATA_EVENT	            64         
    #define SIZE_DATA_LOG	            64         
#endif


/*=========== define cho chip L151 ==================*/

#if defined (STM32L151xC)
    #define ON_FLASH_BASE               0x08000000
    #define ADDR_FLASH_END              0x0803FF00 
    /*========Add Firmware: Main + Update=========== */
    #define	ADDR_MAIN_PROGRAM			0x08002000	        //47 Page cho chuong trinh chinh
    #define	ADDR_UPDATE_PROGRAM			0x08019800	        //47 page -> End Program = 0x08031000

    #define ADDR_FLAG_HAVE_NEW_FW       0x08031800

    /*========Add Record Mess ================ */
    #define	ADDR_MESS_A_START   		0x08032000	         //8 K
    #define	ADDR_MESS_A_STOP			0x08034100

    #define	ADDR_EVENT_START   		    0x08035000	         //8 K
    #define	ADDR_EVENT_STOP			    0x08037100

    #define	ADDR_LOG_START   		    0x08038000	         //8 K
    #define	ADDR_LOG_STOP			    0x0803A100           //64*128 = 8192 = 8k .

    /*=======Add Index Mess================ */
    #define	ADDR_INDEX_LOG_SEND 		0x0803CA00      
    #define	ADDR_INDEX_LOG_SAVE 	    0x0803CB00

    #define	ADDR_INDEX_EVENT_SEND 		0x0803CC00      
    #define	ADDR_INDEX_EVENT_SAVE 	    0x0803CD00

    #define	ADDR_INDEX_TSVH_SEND 		0x0803CE00      
    #define	ADDR_INDEX_TSVH_SAVE 		0x0803CF00  
    
    #define	ADDR_INDEX_GPS_SEND 		0x0803D000      
    #define	ADDR_INDEX_GPS_SAVE 		0x0803D100    

    /* ============= Addr Meter config======== */
    //Setting Wm
    #define	ADDR_METER_NUMBER			0x0803DB00

    #define	ADDR_LEVEL_CALIB			0x0803DC00	        
    #define	ADDR_THRESH_MEAS			0x0803DE00
    #define ADDR_SLAVE_ID               0x0803DF00

    //Setting Emeter
    #define ADDR_PF_SETTING             0x0803F200        
    #define ADDR_VOL_SETTING         	0x0803F300        
    #define ADDR_CT_SETTING             0x0803F400        
    #define ADDR_TIME_SETTING           0x0803F500        
    #define ADDR_RELAY_SETTING          0x0803F600    

    /* ============= Add Modem config ======== */
    #define	ADDR_SERVER_INFOR			0x0803FA00
    #define	ADDR_SERVER_BACKUP			0x0803FB00
    #define	ADDR_FREQ_ACTIVE    	    0x0803FC00
    #define ADDR_DCUID         			0x0803FD00       
    
    /*======= Define Value===========*/   
    #define	FLASH_MAX_MESS_TSVH_SAVE	30
    #define	FLASH_MAX_MESS_EVENT_SAVE   30
    #define FLASH_MAX_MESS_LOG_SAVE     128

    #define MAX_SIZE_FIRMWARE           94     //Kb
    #define FIRMWARE_SIZE_PAGE	        MAX_SIZE_FIRMWARE*1024/FLASH_PAGE_SIZE

    #define SIZE_DATA_TSVH	            128        
    #define SIZE_DATA_EVENT	            64         
    #define SIZE_DATA_LOG	            64   

#endif



#if defined (STM32L072xx) || defined (STM32L082xx)
    #define ON_FLASH_BASE               0x08000000
    #define ADDR_FLASH_END              0x08020000  

    /*========Add Firmware: Main + Update=========== */
#ifdef DEVICE_TYPE_GATEWAY
    #define	ADDR_MAIN_PROGRAM			0x08004000	        //42 Page cho chuong trinh chinh
//    #define	ADDR_UPDATE_PROGRAM			0x08019000	        //42 page -> End Program = 0x0802E000
//
//    #define ADDR_FLAG_HAVE_NEW_FW       0x0802E080

    /*========Add Record Mess ================ */
    #define	ADDR_MESS_A_START   		0x0801E000	         //3*256k -> Max 6 record
    #define	ADDR_MESS_A_STOP			0x0801E300

    #define	ADDR_EVENT_START   		    0x0801E300	         //1*256k -> Max 4 record
    #define	ADDR_EVENT_STOP			    0x0801E400

    #define	ADDR_LOG_START   		    0x0801E400	         //3*256k -> Max 4 record
    #define	ADDR_LOG_STOP			    0x0801E500           
#else
    #define	ADDR_MAIN_PROGRAM			0x08004000
//    #define	ADDR_UPDATE_PROGRAM			0x0800D000	  
//    
//    #define ADDR_FLAG_HAVE_NEW_FW       0x0800F000

    /*========Add Record Mess ================ */
    #define	ADDR_MESS_A_START   		0x08010000 	         
    #define	ADDR_MESS_A_STOP			0x0801F000

    #define	ADDR_EVENT_START   		    0x08010000	        
    #define	ADDR_EVENT_STOP			    0x08015000

    #define	ADDR_LOG_START   		    0x08016000	         
    #define	ADDR_LOG_STOP			    0x0801C000           //6 sector = 6*4096/64 = 384 record     
#endif

    /*=======Add Index Mess================ */
    #define	ADDR_INDEX_LOG_SEND 		0x0801E680
    #define	ADDR_INDEX_LOG_SAVE 	    0x0801E700

    #define	ADDR_INDEX_EVENT_SEND 		0x0801E780
    #define	ADDR_INDEX_EVENT_SAVE 	    0x0801E800

    #define	ADDR_INDEX_TSVH_SEND 		0x0801E880
    #define	ADDR_INDEX_TSVH_SAVE 		0x0801E900

    /* ============= Addr Meter config======== */
    #define	ADDR_FLAG_RESET		        0x0801ED00
    #define	ADDR_METER_NUMBER			0x0801ED80

    #define	ADDR_LEVEL_CALIB			0x0801EE00	        
    #define	ADDR_THRESH_MEAS			0x0801EE80	

    /* ============= Addr Modem config ======== */
    #define	ADDR_SERVER_INFOR			0x0801F500
    #define	ADDR_SERVER_BACKUP			0x0801F580
    #define	ADDR_FREQ_ACTIVE    	    0x0801F600
    #define ADDR_DCUID         			0x0801F680       
    /*============= Addr User Modbus ===========*/
    #define ADDR_INFOR_SLAVE_MODBUS_RTU 0x0801F700
    #define ADDR_CALIB_TEMPERATURE      0x0801F780
    #define ADDR_TEMPERATURE_ALARM      0x0801F800

#ifdef DEVICE_TYPE_GATEWAY
    #define	FLASH_MAX_MESS_TSVH_SAVE	2
    #define	FLASH_MAX_MESS_EVENT_SAVE   2
    #define FLASH_MAX_MESS_LOG_SAVE     2

    #define SIZE_DATA_TSVH	            128        
    #define SIZE_DATA_EVENT	            64         
    #define SIZE_DATA_LOG	            64    

    #define MAX_SIZE_FIRMWARE           84     //Kb
    #define FIRMWARE_SIZE_PAGE	        MAX_SIZE_FIRMWARE*1024/FLASH_PAGE_SIZE
#else
    #define	FLASH_MAX_MESS_TSVH_SAVE	30
    #define	FLASH_MAX_MESS_EVENT_SAVE   30
    #define FLASH_MAX_MESS_LOG_SAVE     128

    #define SIZE_DATA_TSVH	            128        
    #define SIZE_DATA_EVENT	            64         
    #define SIZE_DATA_LOG	            64    

    #define MAX_SIZE_FIRMWARE           84     //Kb
    #define FIRMWARE_SIZE_PAGE	        MAX_SIZE_FIRMWARE*1024/FLASH_PAGE_SIZE
#endif

#endif



/*============== Inline Flash ===================*/
#define _ON_FLASH_IS_NEW_PAGE(address)     \
                    ((address - ON_FLASH_BASE) % FLASH_PAGE_SIZE)
                           
typedef enum
{
	_EVENT_ON_FLASH_WRITE_BUFF = 0,
	_EVENT_ON_FLASH_READ_BUFF,

	_EVENT_ON_FLASH_END, // don't use
}eEVENT_ON_FLASH;



extern sEvent_struct        sEventOnChipFlash[];
extern sMemoryVariable       sOnFlash; 


/*======================== Function ======================*/

void        OnFlash_Init (void);
uint8_t     OnFlash_Task (void);    



#endif
