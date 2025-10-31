
#ifndef USER_MEM_COMM_H_
#define USER_MEM_COMM_H_


#include "user_util.h"

#define MAX_MEM_DATA           1024


#define MEM_CACUL_INDEX(Addr, AddrStart, SizeRec)   (Addr - AddrStart) / SizeRec 
/*======== Struct var ===========*/

typedef enum
{
    __MEM_READ,
    __MEM_WRITE,
}eMemoryProcessType;


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
}sKindTypeRqReadFlash;


typedef struct
{
    uint32_t        AddA_u32;
    uint32_t        AddB_u32;
    uint32_t        Offset_u32;
   
    uint32_t        AddAStart_u32;
    uint32_t        AddAStop_u32;
    
    uint32_t        AddBStart_u32;
    uint32_t        AddBStop_u32;
    
    uint32_t        AddIndexSend_u32;               //Add luu index send
    uint32_t        AddIndexSave_u32;
    
    sData           sPayload;
    
    uint16_t        SizeRecord_u16;                 //Size record
    uint16_t        MaxRecord_u16;                  //Max record Save in Flash
    
    uint16_t        CountMessReaded_u16;
    
    uint16_t        IndexSend_u16;                  //vi tri doc ra va gui
    uint16_t        IndexSave_u16;                  //vi tri luu tiep theo  
} sRecordMemoryManager;


typedef struct
{
    uint8_t         aData[MAX_MEM_DATA];
    uint32_t        Addr_u32;
    uint8_t         TypeData_u8;            //TypeData: TSVHAB, EVENTAB, LOG, Other (write to Addr)
    uint16_t        Length_u16;             //Length data
}sQueueMemWriteManager;


typedef struct
{
    uint32_t        Addr_u32;               //Address
    uint16_t        Length_u8;              //Length data
    uint8_t         TypeData_u8;            //
    uint8_t         TypeRq_u8;
}sQueueMemReadManager;


typedef struct
{
    uint8_t         aData[MAX_MEM_DATA];         //Data write
    uint32_t        Addr_u32;           //Address
    uint16_t        Length_u16;         //pData Read
    uint8_t         Pending_u8;         //Pending or not
    uint8_t         TypeData_u8;
}sMemHandleWrite;


typedef struct
{
    uint32_t        Addr_u32;           //Address
    sData           aData;               //pData Read
    uint8_t         Pending_u8;         //Pending or not
    uint8_t         TypeData_u8;
    uint8_t         TypeRq_u8;          //1: Sim Read New mes  2: Sim RQ AT   3: Terminal Rq AT
}sMemHandleRead;

typedef struct
{
    uint32_t    Addr_u32;
    uint8_t     Status_u8;
}sMemHandleErase;
          

typedef struct
{
    sMemHandleRead      sHRead;         //Handle Read
    sMemHandleWrite     sHWrite;        //Handle write
    sMemHandleErase     sHErase;        //Hanlde erase
    
    uint8_t             Status_u8;              //false: Init Pending, true: OK ; ERROR: Flash Error
    uint8_t             Retry_u8;               //OK, ERROR

    void    ( *pMem_Write_OK ) (uint8_t TypeData);
    void    ( *pMem_Read_OK ) (uint8_t TypeData, uint32_t Addr, uint8_t TypeReq);
    void    ( *pMem_Wrtie_ERROR ) (uint8_t TypeData);
    void    ( *pMem_Erase_Chip_OK ) (void);
    void    ( *pMem_Erase_Chip_Fail ) (void);
    uint32_t ( *pMem_Get_Addr ) (uint8_t TypeData, uint8_t RW);
    void    ( *pMem_Forward_Data_To_Sim ) (uint8_t TypeData, uint8_t *pData, uint16_t Length);
}sMemoryManager;


#endif






