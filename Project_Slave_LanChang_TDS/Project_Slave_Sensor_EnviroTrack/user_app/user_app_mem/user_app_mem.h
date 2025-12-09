


#ifndef USER_APP_MEM_H_
#define USER_APP_MEM_H_


#include "user_util.h"
#include "user_define.h"

#include "user_internal_mem.h"
#include "cat24mxx.h"

/*=== Memory using save record ===*/
//#define MEM_REC_EX_EEPROM  
#define MEM_REC_IN_FLASH    
//#define MEM_REC_EX_FLASH    


#define USING_APP_MEM

/*=== Time Read again mess from flash if: read before not success (not increase index)*/
#define MAX_TIME_RETRY_READ     10000   
#define MAX_BYTE_CUT_GPS        18    
#define MEM_MAX_TIMEOUT_READ    24*3600    //1 ngay
/*================ Var struct =================*/
typedef enum
{
    _EVENT_MEM_POWER_ON,
    _EVENT_MEM_TEST_WRITE,
    _EVENT_MEM_ERASE,
    _EVENT_MEM_INIT_POS_REC,
    
    _EVENT_MEM_CHECK_NEW_TSVH,  
    _EVENT_MEM_CHECK_NEW_EVENT,  
    _EVENT_MEM_CHECK_NEW_GPS,  

    _EVENT_MEM_CTRL_WRITE, 
    
    _EVENT_MEM_CHECK_RQ_AT,    
    
	_EVENT_END_MEM,
}eKindsEventAppMem;

typedef enum
{
    _MEM_MQTT_INFOR,
    
}eKindSubParam;


typedef enum
{
    _MEM_DATA_TSVH_A,
    _MEM_DATA_TSVH_B,
    _MEM_DATA_EVENT_A,
    _MEM_DATA_EVENT_B,
    _MEM_DATA_LOG,
    _MEM_DATA_GPS_A,
    _MEM_DATA_GPS_B,
    _MEM_DATA_OTHER,
}sKindFlashTypeData;



typedef enum
{
    _MEM_READ_NEW_MESS,
    _MEM_READ_RESP_AT,
    _MEM_READ_INIT,
}sKindReadFlash;

typedef enum
{
    __EXTERNAL_EEPROM,
    __INTERNAL_FLASH,
    __EXTERNAL_FALSH,
}eMemType;




typedef struct
{
    uint32_t AddA_u32;
    uint32_t AddB_u32;
    
    uint32_t Offset_u32;
   
    uint32_t AddAStart_u32;
    uint32_t AddAStop_u32;
    
    uint32_t AddBStart_u32;
    uint32_t AddBStop_u32;
    
    uint32_t AddIndex_u32;      //Add luu index
    
    uint16_t SizeRecord_u16;     //Size record
    uint16_t MaxRecord_u16;      //Max record Save in Flash

    uint16_t IndexSend_u16;      //vi tri doc ra va gui
    uint16_t IndexSave_u16;      //vi tri luu tiep theo  
    
    uint8_t  cReaded_u8;
} sRecordMemoryManager;

/* queue write to mem information
    - aData : chua du lieu
    - Addr
    - Length data
*/
typedef struct
{
    uint8_t         aData[MAX_MEM_DATA];
    uint32_t        Addr_u32;
    uint16_t        Length_u16;             //Length data
    uint8_t         TypeData_u8;
    uint8_t         Type_u8;
}sQueueMemWriteManager;


typedef struct
{
    uint32_t        Addr_u32;        //Address
    uint16_t        Length_u16;      //Length data
    uint8_t         TypeData_u8;
    uint8_t         Type_u8;
}sQueueMemReadManager;

typedef struct
{
    uint8_t     Status_u8;
    uint8_t     Port_u8;
    uint16_t    IndexStart_u16;
    uint16_t    IndexStop_u16;
    uint32_t    Landmark_u32;
    sRecordMemoryManager    *pRecord;   //Record infor request
}sRequestATcmd;


//
typedef struct
{
    uint8_t     Status_u8;
    uint8_t     Pending_u8;
    uint32_t    cHardReset_u32;
    
    uint8_t     PendingInit_u8;
    sRequestATcmd   sReqATcmd;
    uint8_t     cPending_u8;
    uint8_t     RecMemType_u8;
    
    uint8_t     *pModePower_u8;
    
    uint8_t     ( *pReset_MCU ) (void);
    void        ( *pRespond ) (uint8_t portNo, uint8_t *data, uint16_t length, uint8_t ack);
    void        ( *pRespond_Str) (uint8_t portNo, const char *str, uint8_t ack);
}sAppMemVariable;


extern sEvent_struct        sEventAppMem []; 
extern sAppMemVariable      sAppMem;

extern sRecordMemoryManager     sRecTSVH;
extern sRecordMemoryManager     sRecEvent;
extern sRecordMemoryManager     sRecLog;
extern sRecordMemoryManager     sRecGPS;

/*================ Func =================*/
void    AppMem_Init(void);
uint8_t AppMem_Task(void);

void    Appmem_Check_Rec_Init_Data (uint32_t Addr, uint8_t *pData, uint16_t Length);

void    AppMem_Erase_New_Sector (uint32_t Addr);

void    AppMem_Cb_Write_OK (uint32_t Addr);
void    AppMem_Cb_Read_OK (uint8_t Kind, uint32_t Addr, uint8_t *pData, uint16_t length);
void    AppMem_Cb_Wrtie_ERROR (uint32_t Addr, uint8_t *pdata, uint16_t length);

void    AppMem_Cb_Erase_Chip_OK(void);
void    AppMem_Cb_Erase_Chip_Fail (void);

void    AppMem_Process_Req_AT (uint8_t type, uint8_t *pData, uint16_t Length);
void    AppMem_Record_Invalid_Handle (uint8_t TypeData);

uint8_t AppMem_Inc_Index_Send_1 (sRecordMemoryManager *sRec, uint8_t Num);
uint8_t AppMem_Inc_Index_Send_2 (sRecordMemoryManager *sRec, uint8_t Num);
uint8_t AppMem_Inc_Index_Save (sRecordMemoryManager *sRec);

void    AppMem_Forward_Mess (uint32_t Addr, uint8_t *pData, uint16_t Length);

void    AppMem_PowerOff (void);
void    AppMem_PowerOn(void);

uint8_t AppMem_Is_New_Record (void);
void    AppMem_Read_New_Mess_OK (uint8_t TypeData, uint8_t *pData, uint16_t Length);

void    AppMem_Init_Index_Record (void);
void    AppMem_Save_Index_Rec (void);
//Edit
uint8_t AppMem_Get_Type_From_Addr (uint32_t addr);
uint32_t AppMem_Cacul_Addr_Mess (uint8_t type, uint8_t RW);

//cb at serial cmd
void    AppMem_SER_Get_Last_For_LOG (sData *strRecei, uint16_t Pos);
void    AppMem_SER_Get_Last_For_OPERA (sData *strRecei, uint16_t Pos);
void    AppMem_SER_Get_Last_For_EVENT (sData *strRecei, uint16_t Pos);
void    AppMem_SER_Get_Last_For_GPS (sData *strRecei, uint16_t Pos);
uint8_t AppMem_Get_Req_AT (uint8_t type, uint16_t NumRec);
void    AppMem_SER_rErase_Memory (sData *strRecei, uint16_t Pos);
void    AppMem_SER_Get_Index_Log (sData *strRecei, uint16_t Pos);
//

void    AppMem_Send_To_Queue_Read (sQueueMemReadManager *qRead);
void    AppMem_Send_To_Queue_Write (sQueueMemWriteManager *qWrite);

uint8_t AppMem_Queue_Write_Empty (void);
uint8_t AppMem_Queue_Read_Empty (void);

uint8_t AppMem_Rec_Status (void);
uint8_t AppMem_Is_Write_To_PartAB (void);

void    AppMem_Write_Data (uint8_t MemType, uint8_t Typedata, uint32_t Addr, 
                        uint8_t *pData, uint16_t Length, uint16_t MAX_LENGTH);
uint8_t AppMem_Read_Data (uint8_t MemType, uint8_t Typedata, uint8_t Kind, uint32_t Addr, uint16_t Length);

uint8_t AppMem_Test_Write(void);


#endif

