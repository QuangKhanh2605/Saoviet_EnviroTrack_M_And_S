/*
 * S25FL.c
 *
 *  Created on: Dec 15, 2021
 *      Author: lenovo
 */

#include "S25FL216K.h"
#include "gpio.h"


/*========= Func callback event ==========*/
static uint8_t _Cb_Event_Flash_Send_BYTE(uint8_t event);
static uint8_t _Cb_Event_Flash_Write_Buff(uint8_t event);
static uint8_t _Cb_Event_Flash_Read_Buff(uint8_t event);

/*========= Func callback cmd ==========*/
static uint8_t _Cb_Flash_Cmd_Failure(void);
static uint8_t _Cb_Flash_Erase_End_Fail(void);
static uint8_t _Cb_Read_Status_Success(void);
static uint8_t _Cb_Check_Write_Enable_Success(void);
static uint8_t _Cb_Check_Erase_Enable_Success(void);
static uint8_t _Cb_Check_Write_END_Success(void);
static uint8_t _Cb_Check_Erase_END_Success(void);
static uint8_t _Cb_Erase_Sector_Success(void);
static uint8_t _Cb_Write_Data_Success(void);
static uint8_t _Cb_Read_Data_Success(void);
static uint8_t _Cb_Active_Cs_Success(void);
static uint8_t _Cb_Deactive_Cs_Success(void);
static uint8_t _Cb_Enable_Success(void);
static uint8_t _Cb_Erase_Delay_Success(void);
static uint8_t _Cb_Read_Check_write(void);
static uint8_t _Cb_Erase_Chip_Success(void);
static uint8_t _Cb_Erase_Chip_End(void);
static uint8_t _Cb_Write_Delay_Success(void);
static uint8_t _Cb_Erase_Chip_Delay(void);

/*=========== Var struct =================*/
sEvent_struct sEventExFlash[] =
{
	{ _EVENT_FLASH_SEND_BYTE, 	0, 0, FLASH_CMD_TIME,   _Cb_Event_Flash_Send_BYTE   },
	{ _EVENT_FLASH_WRITE_BUFF, 	0, 0, 500,  			_Cb_Event_Flash_Write_Buff  },
	{ _EVENT_FLASH_READ_BUFF, 	0, 0, 500,  			_Cb_Event_Flash_Read_Buff   },
};



sFlash_CMD_struct	aFlashCmd[] =
{
	{ _FLASH_CMD_READ_STATUS,  		    S25FL_READ_STATUS, 	SKIP, 	   	_Cb_Read_Status_Success,            _Cb_Flash_Cmd_Failure},
	{ _FLASH_CMD_CHECK_WRITE_ENABLE,    DUMMY_BYTE, 		WEL_Flag, 	_Cb_Check_Write_Enable_Success,     _Cb_Flash_Cmd_Failure},
	{ _FLASH_CMD_CHECK_ERASE_ENABLE,    DUMMY_BYTE, 		WEL_Flag, 	_Cb_Check_Erase_Enable_Success,     _Cb_Flash_Cmd_Failure},
    
	{ _FLASH_CMD_CHECK_WRITE_END, 	    DUMMY_BYTE, 		WIP_Flag, 	_Cb_Check_Write_END_Success,        _Cb_Flash_Cmd_Failure},
	{ _FLASH_CMD_CHECK_ERASE_END, 	    DUMMY_BYTE, 		WIP_Flag, 	_Cb_Check_Erase_END_Success,        _Cb_Flash_Cmd_Failure},
    
	{ _FLASH_CMD_ERASE_SECTOR, 		    S25FL_SECTOR_ERASE, SKIP, 		_Cb_Erase_Sector_Success,           _Cb_Flash_Cmd_Failure},
	{ _FLASH_CMD_WRITE_DATA, 		    S25FL_WRITE, 		SKIP, 		_Cb_Write_Data_Success,             _Cb_Flash_Cmd_Failure},
	{ _FLASH_CMD_READ_DATA, 			S25FL_READ, 		SKIP, 		_Cb_Read_Data_Success,              _Cb_Flash_Cmd_Failure},
    
	{ _FLASH_CMD_ACTIVE_CS, 	  		SKIP, 		 	 	SKIP, 	    _Cb_Active_Cs_Success,              _Cb_Flash_Cmd_Failure},
	{ _FLASH_CMD_DEACTIVE_CS,  		    SKIP, 	 	     	SKIP, 	    _Cb_Deactive_Cs_Success,            _Cb_Flash_Cmd_Failure},
	{ _FLASH_CMD_ENABLE,  	  		    S25FL_WREN, 	 	SKIP, 	    _Cb_Enable_Success,                 _Cb_Flash_Cmd_Failure},
    
    { _FLASH_CMD_WRITE_DELAY, 	        SKIP, 		        SKIP, 	    _Cb_Write_Delay_Success,            _Cb_Flash_Cmd_Failure},
    { _FLASH_CMD_ERASE_DELAY, 	        SKIP, 		        SKIP, 	    _Cb_Erase_Delay_Success,            _Cb_Flash_Cmd_Failure},
    { _FLASH_CMD_READ_CHECK_WRITE, 	    S25FL_READ, 		SKIP, 	    _Cb_Read_Check_write,               _Cb_Flash_Cmd_Failure},
    
    { _FLASH_CMD_ERASE_CHIP, 	        S25FL_CHIP_ERASE, 	SKIP, 	    _Cb_Erase_Chip_Success,             _Cb_Flash_Cmd_Failure},
    { _FLASH_CMD_ERASE_CHIP_DELAY, 	    SKIP, 		        SKIP, 	    _Cb_Erase_Chip_Delay,               _Cb_Flash_Cmd_Failure},
    { _FLASH_CMD_ERASE_CHIP_END, 	    DUMMY_BYTE, 	    WIP_Flag, 	_Cb_Erase_Chip_End,                 _Cb_Flash_Erase_End_Fail},
};



static uint8_t aFlash_Erase_Sector_BlockCmd[] =
{
	_FLASH_CMD_ACTIVE_CS, _FLASH_CMD_ENABLE, _FLASH_CMD_DEACTIVE_CS,	\
	_FLASH_CMD_ACTIVE_CS, _FLASH_CMD_READ_STATUS, _FLASH_CMD_CHECK_ERASE_ENABLE, _FLASH_CMD_DEACTIVE_CS,	\
	_FLASH_CMD_ACTIVE_CS, _FLASH_CMD_ERASE_SECTOR, _FLASH_CMD_DEACTIVE_CS, \
	_FLASH_CMD_ACTIVE_CS, _FLASH_CMD_ERASE_DELAY, _FLASH_CMD_READ_STATUS, _FLASH_CMD_CHECK_ERASE_END, _FLASH_CMD_DEACTIVE_CS
};

static uint8_t aFlash_Write_Data_BlockCmd[] =
{
	_FLASH_CMD_ACTIVE_CS, _FLASH_CMD_ENABLE, _FLASH_CMD_DEACTIVE_CS,	\
	_FLASH_CMD_ACTIVE_CS, _FLASH_CMD_READ_STATUS, _FLASH_CMD_CHECK_WRITE_ENABLE, _FLASH_CMD_DEACTIVE_CS,	\
	_FLASH_CMD_ACTIVE_CS, _FLASH_CMD_WRITE_DATA, _FLASH_CMD_DEACTIVE_CS, \
	_FLASH_CMD_ACTIVE_CS, _FLASH_CMD_WRITE_DELAY, _FLASH_CMD_READ_STATUS, _FLASH_CMD_CHECK_WRITE_END, _FLASH_CMD_DEACTIVE_CS, \
    _FLASH_CMD_ACTIVE_CS, _FLASH_CMD_READ_CHECK_WRITE, _FLASH_CMD_DEACTIVE_CS,
};


static uint8_t aFlash_Read_Data_BlockCmd[] =
{
	_FLASH_CMD_ACTIVE_CS, _FLASH_CMD_READ_DATA, _FLASH_CMD_DEACTIVE_CS,
};


static uint8_t aFlash_Erase_Chip_BlockCmd[] =
{
	_FLASH_CMD_ACTIVE_CS, _FLASH_CMD_ENABLE, _FLASH_CMD_DEACTIVE_CS,	\
	_FLASH_CMD_ACTIVE_CS, _FLASH_CMD_READ_STATUS, _FLASH_CMD_CHECK_ERASE_ENABLE, _FLASH_CMD_DEACTIVE_CS,	\
	_FLASH_CMD_ACTIVE_CS, _FLASH_CMD_ERASE_CHIP, _FLASH_CMD_DEACTIVE_CS, \
	_FLASH_CMD_ACTIVE_CS, _FLASH_CMD_ERASE_CHIP_DELAY, _FLASH_CMD_READ_STATUS, _FLASH_CMD_ERASE_CHIP_END, _FLASH_CMD_DEACTIVE_CS
};



/*Chua data read tu flash*/
static uint8_t aDATA_S25FL_READ[MAX_MEM_DATA];

static uint8_t aQS25FLStepControl[100];

sMemoryVariable sS25FLvar;

sFlashInitIndexRecordVar  sS25FLInit;

Struct_Queue_Type   qS25FLStep;

static uint8_t Retry_u8 = 0;

/*=========== Function =============*/
void S25FL_ChipSelect(uint8_t State)
{
	uint16_t	i = 0;

	for (i = 0; i<1000; i++);

	if (State == LOW)
		HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_SET);

	for (i = 0; i<1000; i++);
}




uint8_t S25FL_Send_Byte(uint8_t byte)
{
	uint8_t retVal=0;

	HAL_SPI_TransmitReceive(&hSPI, &byte, &retVal, 1, 1000); 

	return retVal;
}


void S25FL_Send_Address (uint32_t address)
{
    /* Send SectorAddr high nibble address byte */
	S25FL_Send_Byte((address & 0xFF0000) >> 16);
	  /* Send SectorAddr medium nibble address byte */
	S25FL_Send_Byte((address & 0xFF00) >> 8);
	  /* Send SectorAddr low nibble address byte */
	S25FL_Send_Byte(address & 0xFF);
}

/*========= Func callback event ==========*/

static uint8_t _Cb_Event_Flash_Send_BYTE(uint8_t event)
{
	uint8_t flash_status = 0;
	uint8_t flag_check = 0;
	uint8_t flash_cmd = 0;

    sEventExFlash[_EVENT_FLASH_SEND_BYTE].e_period = FLASH_CMD_TIME;
    
	flash_cmd = eFlash_Get_Step_From_Queue(0);

	if (flash_cmd >= _FLASH_CMD_END) 
    {
		return 0;
	}
    
    //Lenh nao cmd send ! skip se send
	if (aFlashCmd[flash_cmd].cmd_byte != SKIP) // ignore with SKIP command
	{
		if ( (flash_cmd == _FLASH_CMD_CHECK_WRITE_ENABLE) || (flash_cmd == _FLASH_CMD_CHECK_ERASE_ENABLE) )
			flag_check = aFlashCmd[_FLASH_CMD_CHECK_WRITE_ENABLE].cmd_check;

		flash_status = S25FL_Send_Byte(aFlashCmd[flash_cmd].cmd_byte);
        
		if ( (aFlashCmd[flash_cmd].cmd_check != SKIP ) && ( (flash_status & aFlashCmd[flash_cmd].cmd_check) != flag_check) )
		{
			aFlashCmd[flash_cmd].callback_failure();
			return 0;
		}
	}
    
    fevent_active( sEventExFlash, event);
    
	if (aFlashCmd[flash_cmd].callback_success() == true)
        eFlash_Get_Step_From_Queue(1); // clear

	return 1;
}

static uint8_t _Cb_Event_Flash_Write_Buff(uint8_t event)
{ 
    if ( (sS25FLvar.sHWrite.Addr_u32 % S25FL_SECTOR_SIZE) == 0 )
    {
        //Xoa sector chuan bi ghi vao
        sS25FLvar.sHErase.Status_u8 = false;
        sS25FLvar.sHErase.Addr_u32 = sS25FLvar.sHWrite.Addr_u32;
            
        eFlash_Push_Block_To_Queue(aFlash_Erase_Sector_BlockCmd, sizeof(aFlash_Erase_Sector_BlockCmd)); 
    }
    
    eFlash_Push_Block_To_Queue(aFlash_Write_Data_BlockCmd, sizeof(aFlash_Write_Data_BlockCmd));

	return 1;
}



static uint8_t _Cb_Event_Flash_Read_Buff(uint8_t event)
{
    eFlash_Push_Block_To_Queue(aFlash_Read_Data_BlockCmd, sizeof(aFlash_Read_Data_BlockCmd));

	return 1;
}


/*========== Func Callback Cmd Flash ===========*/

static uint8_t _Cb_Flash_Cmd_Failure(void)
{
    /*
        Func: Callnack send fail cmd
            + set period 1s: to excute again
            + over retry -> clear step to push again : hien tai chi co write ms fail
            + Tiep tuc fail -> tim cach de bo qua item tu queue (writr or read)
    */
    uint8_t flash_cmd = 0;
    flash_cmd = eFlash_Get_Step_From_Queue(0);
    
    UTIL_Printf_Str( DBLEVEL_M, "u_ex_flash: flash cmd failure: ");
    UTIL_Printf_Dec( DBLEVEL_M, flash_cmd);
    UTIL_Printf_Str( DBLEVEL_M, "\r\n");  
    
    Retry_u8++;
    
    if (Retry_u8 <= FLASH_MAX_RETRY_ITEM)
    {
        sEventExFlash[_EVENT_FLASH_SEND_BYTE].e_period = FLASH_CMD_TIMEOUT;
        fevent_enable( sEventExFlash, _EVENT_FLASH_SEND_BYTE);
    } else if (Retry_u8 <= FLASH_MAX_RETRY_SKIP)
    {
        sEventExFlash[_EVENT_FLASH_SEND_BYTE].e_period = FLASH_CMD_TIMEOUT;
        fevent_enable( sEventExFlash, _EVENT_FLASH_SEND_BYTE);
        
        qQueue_Clear(&qS25FLStep);
        
        sS25FLvar.wPending_u8 = false;
        sS25FLvar.rPending_u8 = false;
        S25FL_ChipSelect(HIGH);
    } else
    {
        //Thuc hien item tiep theo
        qQueue_Clear(&qS25FLStep);
        sS25FLvar.wPending_u8 = false;
        sS25FLvar.rPending_u8 = false;
        
        //Neu truoc do status true: dang hoat dong bt: tang index save
        if (sS25FLvar.Status_u8 == true)
        {
            UTIL_Log_Str( DBLEVEL_M, "u_ex_flash: skip item queue write!\r\n");
            
            if (sS25FLvar.pMem_Wrtie_ERROR != NULL)
                sS25FLvar.pMem_Wrtie_ERROR(sS25FLvar.sHWrite.Addr_u32,
                                           sS25FLvar.sHWrite.aData, sS25FLvar.sHWrite.Length_u16);
        }
        //active Init Pos Flash again
        sS25FLvar.Status_u8 = false;
        Retry_u8 = 0;
        S25FL_ChipSelect(HIGH);
    }

	return 1;
}

static uint8_t _Cb_Flash_Erase_End_Fail(void)
{
    //Count Retry: tao timeout: 300000
    static uint16_t CountRetry = 0;
    
    CountRetry++;
        
    if (CountRetry <= 300)
    {
        //Tiep tuc thuc hien lai lenh do: Set period = 1000ms
        sEventExFlash[_EVENT_FLASH_SEND_BYTE].e_period = FLASH_CMD_TIMEOUT;
        fevent_enable( sEventExFlash, _EVENT_FLASH_SEND_BYTE);
        
    } else
    {
        //Erase chip Fail
        if (sS25FLvar.pMem_Erase_Chip_Fail != NULL)
            sS25FLvar.pMem_Erase_Chip_Fail();
        //Bo qua lenh nay:
        qQueue_Receive(&qS25FLStep, NULL, 1); 
    }
    
    return 1;
}



static uint8_t _Cb_Read_Status_Success(void)
{
	UTIL_Printf_Str( DBLEVEL_H, "read status ok \r\n");
    
	return 1;
}

static uint8_t _Cb_Check_Write_Enable_Success(void)
{
	UTIL_Printf_Str( DBLEVEL_H, "write enable ok \r\n");

	return 1;
}

static uint8_t _Cb_Check_Erase_Enable_Success(void)
{
	UTIL_Printf_Str(DBLEVEL_H, "erase enable ok \r\n");

	return 1;
}

static uint8_t _Cb_Check_Write_END_Success(void)
{ 
	UTIL_Printf_Str(DBLEVEL_M, "write end ok \r\n");
    
	return 1;
}

static uint8_t _Cb_Check_Erase_END_Success(void)
{
    sS25FLvar.sHErase.Status_u8 = true;
	UTIL_Printf_Str(DBLEVEL_M, "erase end ok \r\n");
    
	return 1;
}


static uint8_t _Cb_Erase_Sector_Success(void)
{
	uint32_t w_flash_address = 0;

    UTIL_Printf_Str( DBLEVEL_M, "erase sector ok \r\n" );
    
    w_flash_address = sS25FLvar.sHErase.Addr_u32;
    eFlash_Send_Address(w_flash_address);
        
	return 1;
}

static uint8_t _Cb_Write_Data_Success(void)
{
	uint16_t i = 0;

    UTIL_Printf_Str( DBLEVEL_M, "u_ex_flash: write process: " );
    
    eFlash_Send_Address( sS25FLvar.sHWrite.Addr_u32 );
    
    for (i = 0; i < sS25FLvar.sHWrite.Length_u16; i++)
        S25FL_Send_Byte(sS25FLvar.sHWrite.aData[i]);

    UTIL_Printf_Hex ( DBLEVEL_H, (uint8_t *) sS25FLvar.sHWrite.aData,  sS25FLvar.sHWrite.Length_u16);
    UTIL_Printf_Str( DBLEVEL_M, "\r\n" );
    
	return 1;
}

static uint8_t _Cb_Read_Data_Success(void)
{
	uint16_t i = 0;
  
    eFlash_Send_Address(sS25FLvar.sHRead.Addr_u32);
    
    for (i = 0; i < sS25FLvar.sHRead.Length_u16; i++)
        aDATA_S25FL_READ[i] =  S25FL_Send_Byte(DUMMY_BYTE);
        
    //Thuc hien item tiep theo
    sS25FLvar.rPending_u8 = false;
    
    if (sS25FLvar.pMem_Read_OK != NULL)
        sS25FLvar.pMem_Read_OK( sS25FLvar.sHRead.Kind_u8, sS25FLvar.sHRead.Addr_u32, aDATA_S25FL_READ, sS25FLvar.sHRead.Length_u16);
    
	return 1;
}

static uint8_t _Cb_Active_Cs_Success(void)
{
	S25FL_ChipSelect(LOW);

	return 1;
}

static uint8_t _Cb_Deactive_Cs_Success(void)
{
	S25FL_ChipSelect(HIGH);

	return 1;
}

static uint8_t _Cb_Enable_Success(void)
{
	UTIL_Printf_Str (DBLEVEL_H, "enable cmd ok \r\n");

	return 1;
}

static uint8_t _Cb_Write_Delay_Success(void)
{
	UTIL_Printf_Str (DBLEVEL_H, "write delay\r\n");
    //Delay 50ms sau Erase
    sEventExFlash[_EVENT_FLASH_SEND_BYTE].e_period = FLASH_WRITE_TIME;
    fevent_enable( sEventExFlash, _EVENT_FLASH_SEND_BYTE);
    
	return 1;
}

static uint8_t _Cb_Erase_Delay_Success(void)
{
	UTIL_Printf_Str (DBLEVEL_H, "erase delay\r\n");
    //Delay 50ms sau Erase
    sEventExFlash[_EVENT_FLASH_SEND_BYTE].e_period = FLASH_ERASE_TIME;
    fevent_enable( sEventExFlash, _EVENT_FLASH_SEND_BYTE);
    
	return 1;
}

static uint8_t _Cb_Read_Check_write(void)
{
    /*
        Read data at Addr write before: and caompare with data write
    */
    uint8_t TempByteRead = 0;
    
    eFlash_Send_Address(sS25FLvar.sHWrite.Addr_u32);

    for (uint16_t i = 0; i < sS25FLvar.sHWrite.Length_u16; i++)
    {
        TempByteRead = S25FL_Send_Byte(DUMMY_BYTE);
        if (sS25FLvar.sHWrite.aData[i] != TempByteRead )
        {
            UTIL_Log_Str (DBLEVEL_M, "u_ex_flash: read again compare error!\r\n");
            //thuc hien ham cmd fail to retry
            _Cb_Flash_Cmd_Failure();
            
            return 0;
        }
    }
    //Reset count retry
    Retry_u8 = 0;
    //Thuc hien item tiep theo
    sS25FLvar.wPending_u8 = false;
    
    if (sS25FLvar.Status_u8 == false)
    {
        //Dang test flash 
        sS25FLInit.Status_u8 = true;
    } else
    {
        //Thuc hien callback to control record
        if (sS25FLvar.pMem_Write_OK != NULL)
            sS25FLvar.pMem_Write_OK(sS25FLvar.sHWrite.Addr_u32);
    }
        
	return 1;
}

static uint8_t _Cb_Erase_Chip_Success(void)
{
    UTIL_Printf_Str( DBLEVEL_H, "u_ex_flash_: erase chip process...\r\n" );

	return 1;
}

static uint8_t _Cb_Erase_Chip_Delay(void)
{
    UTIL_Printf_Str (DBLEVEL_H, "erase chip delay\r\n");
    //Delay 50ms sau Erase
    sEventExFlash[_EVENT_FLASH_SEND_BYTE].e_period = FLASH_ERASE_ALL_TIME;
    fevent_enable( sEventExFlash, _EVENT_FLASH_SEND_BYTE);
    
	return 1;
}


static uint8_t _Cb_Erase_Chip_End(void)
{
    //Erase chip OK
    if (sS25FLvar.pMem_Erase_Chip_OK != NULL)
        sS25FLvar.pMem_Erase_Chip_OK();

	return 1;
}   



/*========= Func Handle =============*/
void eFlash_Init(void)
{
	S25FL_ChipSelect(HIGH);
    
    //Init Queue Sim Step
    qQueue_Create (&qS25FLStep, 100, sizeof (uint8_t), (uint8_t *) &aQS25FLStepControl); 

    //On power and Pull Up hold PIN  
    HAL_GPIO_WritePin(FLASH_ON_OFF_GPIO_Port, FLASH_ON_OFF_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(FLASH_RESET_GPIO_Port, FLASH_RESET_Pin, GPIO_PIN_SET);
    
    sS25FLvar.Status_u8 = false;
    sS25FLInit.Status_u8 = false;
}



uint8_t eFlash_Task (void)
{
	uint8_t i = 0;
	uint8_t Result = false;

	for (i = 0; i < _EVENT_FLASH_END; i++)
	{
		if (sEventExFlash[i].e_status == 1)
		{
            Result = true;
            
			if ((sEventExFlash[i].e_systick == 0) ||
					((HAL_GetTick() - sEventExFlash[i].e_systick)  >=  sEventExFlash[i].e_period))
			{
                sEventExFlash[i].e_status = 0; 
				sEventExFlash[i].e_systick = HAL_GetTick();
				sEventExFlash[i].e_function_handler(i);
			}
		}
	}

	return Result;
}

/*=========Func handle=====*/

void eFlash_Send_Address(uint32_t address)
{
    S25FL_Send_Address(address);
}


/* */
uint8_t eFlash_Push_Step_To_Queue(uint8_t step)
{
	if (step >= _FLASH_CMD_END)
		return 0;

	if (qQueue_Send(&qS25FLStep, (uint8_t *) &step, _TYPE_SEND_TO_END) == 0)
		return 0;

	if (sEventExFlash[_EVENT_FLASH_SEND_BYTE].e_status == 0)
		fevent_active(sEventExFlash, _EVENT_FLASH_SEND_BYTE);

	return 1;
}

uint8_t eFlash_Push_Block_To_Queue(uint8_t *block_flash_step, uint8_t size)
{
	uint8_t i = 0;

	for (i = 0; i < size; i++)
	{
		if (eFlash_Push_Step_To_Queue(block_flash_step[i]) == 0)
			return 0;
	}

	return 1;
}

/*
 * Type = 0 - don't clear queue
        1: clear
 * */
uint8_t eFlash_Get_Step_From_Queue(uint8_t Type)
{
	uint8_t flash_step = 0;

	if (qGet_Number_Items(&qS25FLStep) == 0)
		return _FLASH_CMD_END;
    
	if (qQueue_Receive(&qS25FLStep, (uint8_t *) &flash_step, Type) == 0)
        return _FLASH_CMD_END;

	return flash_step;
}






/*========= Function test ==========*/

void eFlash_S25FL_Enable(void)
{
    /* Select the FLASH: Chip Select low */
    S25FL_ChipSelect(LOW);	
    
    /* Send "Write Enable" instruction */
    S25FL_Send_Byte(S25FL_WREN);
    
    /* Deselect the FLASH: Chip Select high */
    S25FL_ChipSelect(HIGH);	
}


uint8_t eFlash_S25FL_Erase_Sector(uint32_t SectorAddr)
{
	uint8_t Retry = 0;
    /* Send write enable instruction */
	do{	
        eFlash_S25FL_Enable();
		if(eFlash_S25FL_CheckWriteEnable(TIMEOUT_WR_ENABLE) == 1)
		{
            /* Sector Erase */ 
            /* Select the FLASH: Chip Select low */
            S25FL_ChipSelect(LOW);		
            /* Send Sector Erase instruction  */
            S25FL_Send_Byte(S25FL_SECTOR_ERASE);
            /* Send SectorAddr high nibble address byte */
            S25FL_Send_Byte((SectorAddr & 0xFF0000) >> 16);
            /* Send SectorAddr medium nibble address byte */
            S25FL_Send_Byte((SectorAddr & 0xFF00) >> 8);
            /* Send SectorAddr low nibble address byte */
            S25FL_Send_Byte(SectorAddr & 0xFF);
            /* Deselect the FLASH: Chip Select high */
            S25FL_ChipSelect(HIGH);
            /* Wait the end of Flash writing */
            if(eFlash_S25FL_WaitForWriteEnd(TIMEOUT_ER_SECTOR) == 1)
                return 1;
            else
                return 0;
		}
		Retry++;
	}while(Retry < 3);
    
	return 0;
}


uint8_t eFlash_S25FL_WaitForWriteEnd(uint32_t timeout) 
{
    uint8_t   FLASH_Status = 0;
    uint32_t	Start_check = 0;
    
    Start_check = RtCountSystick_u32;
    
    /* Select the FLASH: Chip Select low */
    S25FL_ChipSelect(LOW);
  	
    /* Send "Read Status Register" instruction */
    S25FL_Send_Byte(S25FL_READ_STATUS);
    
    /* Loop as long as the memory is busy with a write cycle */ 		
    do
    { 
        /* Send a dummy byte to generate the clock needed by the FLASH 
        and put the value of the status register in FLASH_Status variable */
        FLASH_Status = S25FL_Send_Byte(DUMMY_BYTE); 
		
        if (Check_Time_Out(Start_check,timeout) == 1)
        {
            S25FL_ChipSelect(HIGH);
            return 0;
        }
        if ((FLASH_Status & WIP_Flag) != 0)                    //Bit so 0: WIP   ; Bit so 1: WEL:  1: cho phep program,,,, 0 K cho phep
            HAL_Delay(1);
    } while ((FLASH_Status & WIP_Flag) != 0);     /* Write in progress */  
    
    /* Deselect the FLASH: Chip Select high */
    S25FL_ChipSelect(HIGH);
    
    return 1;
}



uint8_t eFlash_S25FL_CheckWriteEnable(uint32_t timeout)
{
    uint8_t     FLASH_Status = 0;
	uint32_t	Start_check = 0;
	
	Start_check = RtCountSystick_u32;
    
    /* Select the FLASH: Chip Select low */
    S25FL_ChipSelect(LOW);
  	
    /* Send "Read Status Register" instruction */
    S25FL_Send_Byte(S25FL_READ_STATUS);
    
    /* Loop as long as the memory is busy with a write cycle */ 		
    do
    { 
        /* Send a dummy byte to generate the clock needed by the FLASH 
        and put the value of the status register in FLASH_Status variable */
        FLASH_Status = S25FL_Send_Byte(DUMMY_BYTE); 
		
        if (Check_Time_Out(Start_check,timeout) == 1)
        {
            S25FL_ChipSelect(HIGH);
            return 0;
        }
        if((FLASH_Status & WEL_Flag) != WEL_Flag)   
            HAL_Delay(1);
    }while((FLASH_Status & WEL_Flag) != WEL_Flag); /* Write in progress */     //Neu 1: cho phep write    0: k cho phep
    /* Deselect the FLASH: Chip Select high */
    S25FL_ChipSelect(HIGH);
    
    return 1;
}



uint8_t eFlash_S25FL_PageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    //	uint8_t Retry = 0;
	uint8_t mTempReadback[256];
	
    //	do{
    /* Enable the write access to the FLASH */
    eFlash_S25FL_Enable(); // Check enable thanh cong
    if(eFlash_S25FL_CheckWriteEnable(TIMEOUT_WR_ENABLE) == 1)
    {		
        /* Select the FLASH: Chip Select low */
        S25FL_ChipSelect(LOW);	   
        /* Send "Write to Memory " instruction */    
        S25FL_Send_Byte(S25FL_WRITE);	
        /* Send WriteAddr high nibble address byte to write to */
        S25FL_Send_Byte((WriteAddr & 0xFF0000) >> 16);
        /* Send WriteAddr medium nibble address byte to write to */
        S25FL_Send_Byte((WriteAddr & 0xFF00) >> 8);  
        /* Send WriteAddr low nibble address byte to write to */
        S25FL_Send_Byte(WriteAddr & 0xFF);             
        
        /* while there is data to be written on the FLASH */
        HAL_SPI_TransmitReceive(&hSPI, pBuffer, mTempReadback, NumByteToWrite, 100);
        
        //            /* Deselect the FLASH: Chip Select high */
        //            Retry = 3;
    } else
        return 0;
    
    S25FL_ChipSelect(HIGH);
    //		Retry++;
    //	}while(Retry<3);
    //    
    //    if(Flash_S25FL_CheckWriteEnable(TIMEOUT_WR_ENABLE) != 1)
    //		return 0;
    /* Wait the end of Flash writing */
    if(eFlash_S25FL_WaitForWriteEnd(TIMEOUT_WR_PAGE) != 1)
        return 0;
    else
        return 1;
}


uint8_t eFlash_S25FL_BufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{  
    if (eFlash_S25FL_WaitForWriteEnd(TIMEOUT_WR_ENABLE) == 1)
    {
        if (eFlash_S25FL_PageWrite(pBuffer, WriteAddr, NumByteToWrite) != 1)
        {
            UTIL_Printf_Str (DBLEVEL_H, "u_ex_flash: write fail!\r\n");
            return 0;
        }
    }
         
    UTIL_Printf_Str (DBLEVEL_H, "u_ex_flash: write OK!\r\n");
    
    return 1;
}

            

uint8_t eFlash_S25FL_BufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
	uint8_t mAdress[3];
	uint16_t mNumByteToRead = NumByteToRead;
	uint8_t Retry = 0;
	
	mAdress[0] = (uint8_t)((ReadAddr & 0xFF0000) >> 16);
	mAdress[1] = (uint8_t)((ReadAddr & 0x00FF00) >> 8);
	mAdress[2] = (uint8_t)(ReadAddr & 0x0000FF);
	
	do
	{
		if(eFlash_S25FL_WaitForWriteEnd(TIMEOUT_WR_ENABLE) == 1)
		{
            /* Select the FLASH: Chip Select low */
            S25FL_ChipSelect(LOW);
            
            /* Send "Read from Memory " instruction */
            S25FL_Send_Byte(S25FL_READ);	 
            // Send addr
            /* Send ReadAddr high nibble address byte to read from */
            S25FL_Send_Byte(mAdress[0]);
            /* Send ReadAddr medium nibble address byte to read from */
            S25FL_Send_Byte(mAdress[1]);
            /* Send ReadAddr low nibble address byte to read from */
            S25FL_Send_Byte(mAdress[2]);   
			
            while (mNumByteToRead--) /* while there is data to be read */
            {
                /* Read a byte from the FLASH */			
                *pBuffer = S25FL_Send_Byte(DUMMY_BYTE);
                /* Point to the next location where the byte read will be saved */
                pBuffer++;
            }
            
            /* Deselect the FLASH: Chip Select high */
            S25FL_ChipSelect(HIGH);
            return 1;
        }
		Retry++;
	} while (Retry<3);
    
	return 0;
}	
           



/*====== Func Handle record =========*/
void eFlash_Default_Struct_Init_Pos (void)
{
    sS25FLInit.FlagMark_u8 = true;
    sS25FLInit.IsEmptyArea_u8 = false;
    sS25FLInit.IsFullArea_u8 = false;
    sS25FLInit.Status_u8 = false;
}

/*
    Func: Push cac lenh xoa sector
*/
void eFlash_Push_Step_Erase_Sector (void)
{
    eFlash_Push_Block_To_Queue(aFlash_Erase_Sector_BlockCmd, sizeof(aFlash_Erase_Sector_BlockCmd));
}



/*
    Func: Push cac lenh xoa sector
*/
void eFlash_Push_Step_Erase_Chip (void)
{
    eFlash_Push_Block_To_Queue(aFlash_Erase_Chip_BlockCmd, sizeof(aFlash_Erase_Chip_BlockCmd));
}



/*========== Func Test Write ==========*/
uint8_t eFlash_Test_Write (void)
{
    uint8_t aTEST_FLASH[16] = {"1234567890123456"};
    static uint8_t Retry = 0;
    
    //Ghi vao vung nho test: 1 chuoi -> doc ra kiem tra
    if (sS25FLInit.Status_u8 == true)
    {
        UTIL_Printf_Str( DBLEVEL_M, "u_ex_flash: test flash write ok\r\n" );
        Retry = 0;
        sS25FLInit.Status_u8 = false; 

        return true;
    }
    
    //Qua so lan retry test flash: set status = ERROR. 10s quay lai soft reset va check lai
    if ( (Retry >= FLASH_MAX_RETRY_TEST) && (sS25FLvar.wPending_u8 != true) )
    {
        UTIL_Printf_Str( DBLEVEL_M, "u_ex_flash: flash error!\r\n" );
        
        sS25FLvar.Status_u8 = ERROR;
        //Set lai thoi gian kiem tra lai
        Retry = 0;
        
        return true;
    }
    
    if (sS25FLvar.wPending_u8 != true)
    {
        sS25FLvar.wPending_u8 = true;
        
        Retry++;
        
        for (uint16_t i = 0; i < 16; i++)         
            sS25FLvar.sHWrite.aData[i] = aTEST_FLASH[i];
                
        sS25FLvar.sHWrite.Length_u16   = 16;
        sS25FLvar.sHWrite.Addr_u32     = FLASH_ADDR_TEST;
        
        fevent_active( sEventExFlash, _EVENT_FLASH_WRITE_BUFF);
    }
    
    return false;
}


/*
    Func: Check Data read Init Pos
        PartAorB: 0: Part A, 1: part B
*/
void eFlash_Check_Data_Init_Pos (uint32_t Addr, uint8_t *pData, uint16_t Length, uint8_t PartAorB)
{
    uint16_t  i = 0, j = 0;
    
    if (PartAorB == 0)
    {
        //Kiem tra xem data khac empty k? (! 0xFF)
        for (i = 0; i < Length; i++)
            if (pData[i]== 0xFF)
                j++;
        
        if (j == Length)
        {
            sS25FLInit.PosFindData_u32 = Addr;
            //Empty Area -> Finish check
            sS25FLInit.Status_u8 = true;
        }
    } else
    {
        //Kiem tra xem data khac empty k? (! 0xFF) : sau do partB check tiep vi tri o sau 
        for (i = 0; i < Length; i++)
            if (pData[i]== 0xFF)
                j++;
        
        if (j == Length)
        {
            sS25FLInit.IsFullArea_u8 = true;
            if (sS25FLInit.FlagMark_u8 == true)
            {
                //Ket thuc va set Pos Part B
                sS25FLInit.PosFindData_u32 = Addr;
                sS25FLInit.FlagMark_u8 = false;
            }
        } else
        {
            sS25FLInit.IsEmptyArea_u8 = true;
            sS25FLInit.FlagMark_u8 = true;
        }
    }
}
   

void eFlash_Erase_Sector (uint32_t Addr)
{
    sS25FLvar.sHErase.Status_u8 = false;
    sS25FLvar.sHErase.Addr_u32 = Addr;
    eFlash_Push_Step_Erase_Sector();
}










uint8_t aTEST_WRITE[256];
uint8_t aTEST_READ[256];
uint8_t TestResult = true;

void eFlash_Test (void)
{
    uint16_t i = 0;
    uint32_t AddrTest = 0x001000;
        
	while (AddrTest < 0x871000)
	{
        UTIL_MEM_set(&aTEST_WRITE[0], 0, sizeof(aTEST_WRITE));
        
        for (i = 0; i < 256; i++)
            aTEST_WRITE[i] = '1';
        
        eFlash_S25FL_BufferWrite(aTEST_WRITE, AddrTest, 256 );
        
        //doc ra check
		if (eFlash_S25FL_BufferRead(aTEST_READ, AddrTest, 256) == 1)
		{
            //in ra man hinh
            UTIL_Printf( DBLEVEL_M, aTEST_READ, 256 );
            
			for (i = 0; i < 256; i++)
            {
				if (aTEST_READ[i] != aTEST_WRITE[i])
                {
                    TestResult = false;
                    UTIL_Printf_Str( DBLEVEL_M, "\r\nu_ex_flash: compare error!\r\n" );
                }
            }
        }
        
        HAL_Delay(1000);
    }
}










