
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


typedef struct
{
    uint8_t         aData[MAX_MEM_DATA];
    uint32_t        Addr_u32;
    uint16_t        Length_u16;             //Length data
}sMemWriteInfor;


typedef struct
{
    uint32_t        Addr_u32;        //Address
    uint16_t        Length_u16;      //Length data
}sMemReadInfor;


typedef struct
{
    uint8_t         Status_u8;
    sMemWriteInfor  sHWrite; 
    sMemWriteInfor  sHRead;
    
    uint8_t         wPending_u8;
    uint8_t         rPending_u8;
    
    void    ( *pMem_Write_OK ) (uint32_t Addr);
    void    ( *pMem_Read_OK ) (uint32_t Addr, uint8_t *pData, uint16_t length);
    void    ( *pMem_Read_Fail ) (uint32_t Addr);
    
    void    ( *pMem_Wrtie_ERROR ) (uint32_t Addr, uint8_t *pdata, uint16_t length);
    void    ( *pMem_Erase_Chip_OK ) (void);
    void    ( *pMem_Erase_Chip_Fail ) (void);
}sMemoryVariable;






#endif






