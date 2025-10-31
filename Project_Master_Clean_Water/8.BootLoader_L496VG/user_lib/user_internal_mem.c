/*
    8/2021
    Thu vien xu ly Uart
*/
#include "user_internal_mem.h"


static uint8_t _Cb_On_Flash_Write_Buff (uint8_t event);
static uint8_t _Cb_On_Flash_Read_Buff (uint8_t event);


/*======== struct ===============*/
sEvent_struct sEventOnChipFlash[] =
{
	{ _EVENT_ON_FLASH_WRITE_BUFF, 	    0, 0, 500,  			_Cb_On_Flash_Write_Buff  },
	{ _EVENT_ON_FLASH_READ_BUFF, 	    0, 0, 500,  			_Cb_On_Flash_Read_Buff   },
};


static uint8_t      aDATA_READ[MAX_MEM_DATA];
sMemoryVariable     sOnFlash;


/*======================== Function ======================*/
/*------------ Func Callback --------------*/
static uint8_t _Cb_On_Flash_Write_Buff (uint8_t event)
{
    if (_ON_FLASH_IS_NEW_PAGE (sOnFlash.sHWrite.Addr_u32) == 0)
        Erase_Firmware(sOnFlash.sHWrite.Addr_u32, 1);
      
    OnchipFlash_Write_Buff (sOnFlash.sHWrite.Addr_u32, sOnFlash.sHWrite.aData, sOnFlash.sHWrite.Length_u16); 

    //Write OK: tang index save
    if (sOnFlash.pMem_Write_OK != NULL)
        sOnFlash.pMem_Write_OK(sOnFlash.sHWrite.Addr_u32);
    
    sOnFlash.wPending_u8 = false;
    
    return 1;
}


static uint8_t _Cb_On_Flash_Read_Buff (uint8_t event)
{
    OnchipFlashReadData(sOnFlash.sHRead.Addr_u32, aDATA_READ, sOnFlash.sHRead.Length_u16);

    if (sOnFlash.pMem_Read_OK != NULL)
        sOnFlash.pMem_Read_OK(sOnFlash.sHRead.Addr_u32, aDATA_READ, sOnFlash.sHRead.Length_u16);
    
    sOnFlash.rPending_u8 = false;
    
    return 1;
}
/*------------- Func Handle----------------*/

void OnFlash_Init (void)
{
    sOnFlash.Status_u8 = true;
}



uint8_t OnFlash_Task (void)
{
    uint8_t i = 0;
	uint8_t Result = false;

	for (i = 0; i < _EVENT_ON_FLASH_END; i++)
	{
		if (sEventOnChipFlash[i].e_status == 1)
		{
            Result = true;
            
			if ((sEventOnChipFlash[i].e_systick == 0) ||
					((HAL_GetTick() - sEventOnChipFlash[i].e_systick)  >=  sEventOnChipFlash[i].e_period))
			{
                sEventOnChipFlash[i].e_status = 0; 
				sEventOnChipFlash[i].e_systick = HAL_GetTick();
				sEventOnChipFlash[i].e_function_handler(i);
			}
		}
	}

	return Result;
}

