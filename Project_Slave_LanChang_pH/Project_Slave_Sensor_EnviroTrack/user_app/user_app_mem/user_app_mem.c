

#include "user_app_mem.h"


#ifdef MEM_REC_IN_FLASH
    sRecordMemoryManager     sRecTSVH = 
    {
        .AddAStart_u32      = ADDR_MESS_A_START,   
        .AddAStop_u32       = ADDR_MESS_A_STOP,
        
        .Offset_u32         = 0,
        
        .AddBStart_u32      = ADDR_MESS_A_START,   
        .AddBStop_u32       = ADDR_MESS_A_STOP,
            
        .AddIndex_u32       = ADDR_INDEX_REC,
        
        .MaxRecord_u16      = FLASH_MAX_MESS_TSVH_SAVE, 
        .SizeRecord_u16     = SIZE_DATA_TSVH, 
    };
    
    sRecordMemoryManager     sRecEvent = 
    {
        .AddAStart_u32      = ADDR_EVENT_START,
        .AddAStop_u32       = ADDR_EVENT_STOP,
            
        .Offset_u32         = 0,
        
        .AddBStart_u32      = ADDR_EVENT_START,   
        .AddBStop_u32       = ADDR_MESS_A_STOP,
        
        .AddIndex_u32       = ADDR_INDEX_REC,
        
        .MaxRecord_u16      = FLASH_MAX_MESS_EVENT_SAVE,
        .SizeRecord_u16     = SIZE_DATA_EVENT,
    };
    
    //Struct Log data
    sRecordMemoryManager     sRecLog =    
    {
        .AddAStart_u32      = ADDR_LOG_START,   
        .AddAStop_u32       = ADDR_LOG_STOP,
        
        .Offset_u32         = 0,
        
        .AddBStart_u32      = ADDR_LOG_START,   
        .AddBStop_u32       = ADDR_LOG_STOP,
            
        .AddIndex_u32       = ADDR_INDEX_REC,
        
        .MaxRecord_u16      = FLASH_MAX_MESS_LOG_SAVE,     
        .SizeRecord_u16     = SIZE_DATA_LOG,
    };
    
    //Struct GPS Data
    sRecordMemoryManager     sRecGPS =    
    {
        .AddAStart_u32      = ADDR_GPS_START,   
        .AddAStop_u32       = ADDR_GPS_STOP,
        
        .Offset_u32         = 0,
        
        .AddBStart_u32      = ADDR_GPS_START,   
        .AddBStop_u32       = ADDR_GPS_STOP,
            
        .AddIndex_u32       = ADDR_INDEX_REC, 
        
        .SizeRecord_u16     = CAT_SIZE_DATA_GPS,
        .MaxRecord_u16      = CAT_MAX_MESS_GPS_SAVE,     
        
        .IndexSend_u16      = 0,
        .IndexSave_u16      = 0,
    };
#endif
    
#ifdef MEM_REC_EX_EEPROM
    sRecordMemoryManager     sRecTSVH = 
    {
        .AddAStart_u32      = CAT_ADDR_MESS_A_START,                      
        .AddAStop_u32       = CAT_ADDR_MESS_A_STOP,
        
        .Offset_u32         = 0,
        
        .AddBStart_u32      = CAT_ADDR_MESS_A_START,                      
        .AddBStop_u32       = CAT_ADDR_MESS_A_STOP,
        
        .AddIndex_u32       = ADDR_INDEX_REC,

        .MaxRecord_u16      = CAT_MAX_MESS_TSVH_SAVE,  
        .SizeRecord_u16     = CAT_SIZE_DATA_TSVH, 
    };
    // Struct Event Data
    sRecordMemoryManager     sRecEvent = 
    {
        .AddAStart_u32      = CAT_ADDR_EVENT_START,
        .AddAStop_u32       = CAT_ADDR_EVENT_STOP,
        
        .Offset_u32         = 0,
        
        .AddBStart_u32      = CAT_ADDR_EVENT_START,
        .AddBStop_u32       = CAT_ADDR_EVENT_STOP,
        
        .AddIndex_u32       = ADDR_INDEX_REC,         

        .MaxRecord_u16      = CAT_MAX_MESS_EVENT_SAVE,
        .SizeRecord_u16     = CAT_SIZE_DATA_EVENT, 
    };
    //Struct Log data: luu trong onflash nhung index luu trong exmem
    sRecordMemoryManager     sRecLog = 
    {
        .AddAStart_u32      = CAT_ADDR_LOG_START,   
        .AddAStop_u32       = CAT_ADDR_LOG_STOP,
        
        .Offset_u32         = 0,
        
        .AddBStart_u32      = CAT_ADDR_LOG_START,   
        .AddBStop_u32       = CAT_ADDR_LOG_STOP,
            
        .AddIndex_u32       = ADDR_INDEX_REC, 

        .MaxRecord_u16      = CAT_MAX_MESS_LOG_SAVE,
        .SizeRecord_u16     = CAT_SIZE_DATA_LOG,
    };
    
    //Struct GPS Data
    sRecordMemoryManager     sRecGPS =    
    {
        .AddAStart_u32      = CAT_ADDR_GPS_START,   
        .AddAStop_u32       = CAT_ADDR_GPS_STOP,
        
        .Offset_u32         = 0,
        
        .AddBStart_u32      = CAT_ADDR_GPS_START,   
        .AddBStop_u32       = CAT_ADDR_GPS_STOP,
            
        .AddIndex_u32       = ADDR_INDEX_REC, 

        .SizeRecord_u16     = CAT_SIZE_DATA_GPS,
        .MaxRecord_u16      = CAT_MAX_MESS_GPS_SAVE,     
        
        .IndexSend_u16      = 0,
        .IndexSave_u16      = 0,
    };
#endif

#ifdef MEM_REC_EX_FLASH
     
    sRecordMemoryManager     sRecTSVH = 
    {
        .AddA_u32           = FLASH_ADDR_TSVH_A_START,
        .AddB_u32           = FLASH_ADDR_TSVH_B_START,
        
        .Offset_u32         = FLASH_ADDR_TSVH_OFFSET,
            
        .AddAStart_u32      = FLASH_ADDR_TSVH_A_START,   
        .AddAStop_u32       = FLASH_ADDR_TSVH_A_STOP,
        
        .AddBStart_u32      = FLASH_ADDR_TSVH_B_START,   
        .AddBStop_u32       = FLASH_ADDR_TSVH_B_STOP,
            
        .AddIndex_u32       = ADDR_INDEX_REC, 
        
        .SizeRecord_u16     = FLASH_SIZE_DATA_TSVH, 
        .MaxRecord_u16      = FLASH_MAX_RECORD_TSVH, 
        
        .IndexSend_u16      = 0,
        .IndexSave_u16      = 0,
    };
    
    sRecordMemoryManager     sRecEvent = 
    {
        .AddA_u32           = FLASH_ADDR_EVENT_A_START,
        .AddB_u32           = FLASH_ADDR_EVENT_B_START,
        
        .Offset_u32         = FLASH_ADDR_EVENT_OFFSET,
            
        .AddAStart_u32      = FLASH_ADDR_EVENT_A_START,   
        .AddAStop_u32       = FLASH_ADDR_EVENT_A_STOP,
        
        .AddBStart_u32      = FLASH_ADDR_EVENT_B_START,   
        .AddBStop_u32       = FLASH_ADDR_EVENT_B_STOP,
            
        .AddIndex_u32       = ADDR_INDEX_REC,
        
        .SizeRecord_u16     = FLASH_SIZE_DATA_EVENT, 
        .MaxRecord_u16      = FLASH_MAX_RECORD_EVENT, 
        
        .IndexSend_u16      = 0,
        .IndexSave_u16      = 0,
    };
    
    sRecordMemoryManager     sRecLog = 
    {
        .AddA_u32           = FLASH_ADDR_LOG_START,
        .AddB_u32           = FLASH_ADDR_LOG_START,
        
        .Offset_u32         = 0,
            
        .AddAStart_u32      = FLASH_ADDR_LOG_START,   
        .AddAStop_u32       = FLASH_ADDR_LOG_STOP,
        
        .AddBStart_u32      = FLASH_ADDR_LOG_START,   
        .AddBStop_u32       = FLASH_ADDR_LOG_STOP,
            
        .AddIndex_u32       = ADDR_INDEX_REC,
        
        .SizeRecord_u16     = FLASH_SIZE_DATA_LOG, 
        .MaxRecord_u16      = FLASH_MAX_RECORD_LOG, 
        
        .IndexSend_u16      = 0,
        .IndexSave_u16      = 0,
    };

    //Struct GPS Data
    sRecordMemoryManager     sRecGPS =    
    {
        .AddAStart_u32      = FLASH_ADDR_GPS_A_START,   
        .AddAStop_u32       = FLASH_ADDR_GPS_A_STOP,
        
        .Offset_u32         = FLASH_ADDR_GPS_OFFSET,
        
        .AddBStart_u32      = FLASH_ADDR_GPS_B_START,   
        .AddBStop_u32       = FLASH_ADDR_GPS_B_STOP,
            
        .AddIndex_u32       = ADDR_INDEX_REC, 
        
        .SizeRecord_u16     = FLASH_SIZE_DATA_GPS,
        .MaxRecord_u16      = FLASH_MAX_RECORD_GPS,     
        
        .IndexSend_u16      = 0,
        .IndexSave_u16      = 0,
    };
    
#endif 
    
    
/*
    - Quan ly Index: 
        + Luu Part A, Part B
        + Luu index vao flash noi        
*/
static uint8_t _Cb_mem_Power_On(uint8_t event);
static uint8_t _Cb_mem_Test_Write(uint8_t event);
static uint8_t _Cb_mem_Init_Pos_Rec(uint8_t event);
static uint8_t _Cb_mem_Check_New_Tsvh (uint8_t event);
static uint8_t _Cb_mem_Check_New_Event (uint8_t event);
static uint8_t _Cb_mem_Check_New_Gps (uint8_t event);
static uint8_t _Cb_mem_Ctrl_Write (uint8_t event);

static uint8_t _Cb_mem_Check_Request_At (uint8_t event);
static uint8_t _Cb_mem_Erase (uint8_t event);

/*================ Var struct =================*/  
sEvent_struct sEventAppMem[] = 
{
    { _EVENT_MEM_POWER_ON,  		    1, 0, 100,      _Cb_mem_Power_On },
    { _EVENT_MEM_TEST_WRITE,  		    0, 0, 5,        _Cb_mem_Test_Write }, 
    { _EVENT_MEM_ERASE,  	            0, 0, 0,        _Cb_mem_Erase },
    { _EVENT_MEM_INIT_POS_REC,  		0, 0, 5,        _Cb_mem_Init_Pos_Rec }, 
    
    { _EVENT_MEM_CHECK_NEW_TSVH,  		0, 0, 1000,     _Cb_mem_Check_New_Tsvh },   
    { _EVENT_MEM_CHECK_NEW_EVENT,  		0, 0, 1000,     _Cb_mem_Check_New_Event },   
    { _EVENT_MEM_CHECK_NEW_GPS,  		0, 0, 1000,     _Cb_mem_Check_New_Gps },   
    
    { _EVENT_MEM_CTRL_WRITE,  		    0, 0, 1000,     _Cb_mem_Ctrl_Write },   
    
    { _EVENT_MEM_CHECK_RQ_AT,  		    0, 0, 6000,     _Cb_mem_Check_Request_At },
};    

sAppMemVariable       sAppMem;

static sQueueMemWriteManager    sQMemWrite[10];
static sQueueMemReadManager     sQMemRead[20];

Struct_Queue_Type   qMemRead;
Struct_Queue_Type   qMemWrite;

/*================= Function ======================*/

uint8_t AppMem_Task(void)
{
    uint8_t i = 0;
	uint8_t Result = false;

	for (i = 0; i < _EVENT_END_MEM; i++)
	{
		if (sEventAppMem[i].e_status == 1)
		{
            if ( (i != _EVENT_MEM_CHECK_NEW_TSVH) && \
                 (i != _EVENT_MEM_CHECK_NEW_EVENT) && \
                 (i != _EVENT_MEM_CHECK_NEW_GPS) )
                Result = true;

			if ((sEventAppMem[i].e_systick == 0) ||
					((HAL_GetTick() - sEventAppMem[i].e_systick)  >=  sEventAppMem[i].e_period))
			{
                sEventAppMem[i].e_status = 0;  //Disable event
				sEventAppMem[i].e_systick = HAL_GetTick();
				sEventAppMem[i].e_function_handler(i);
			}
		}
	}
    
    Result |= OnFlash_Task(); 
    Result |= CAT24Mxx_Task(); 
//    Result |= eFlash_Task(); 
    
	return Result;
}



   
void AppMem_Init(void)
{
    qQueue_Create (&qMemWrite, 10, sizeof (sQueueMemWriteManager), (sQueueMemWriteManager *) &sQMemWrite);  
    qQueue_Create (&qMemRead, 20, sizeof (sQueueMemReadManager), (sQueueMemReadManager *) &sQMemRead);    
    
#ifdef MEM_REC_IN_FLASH
    sAppMem.RecMemType_u8 = __INTERNAL_FLASH;
	AppMem_Init_Index_Record();

    OnFlash_Init();
     
    sOnFlash.pMem_Write_OK = AppMem_Cb_Write_OK;
    sOnFlash.pMem_Read_OK = AppMem_Cb_Read_OK;
#endif


#ifdef MEM_REC_EX_EEPROM
    sAppMem.RecMemType_u8 = __EXTERNAL_EEPROM;
    AppMem_Init_Index_Record();
    
    CAT24Mxx_Init();
    
    sCAT24Var.pMem_Write_OK = AppMem_Cb_Write_OK;
    sCAT24Var.pMem_Read_OK = AppMem_Cb_Read_OK;
    sCAT24Var.pMem_Wrtie_ERROR = AppMem_Cb_Wrtie_ERROR;
    sCAT24Var.pMem_Erase_Chip_OK = AppMem_Cb_Erase_Chip_OK;
#endif

#ifdef MEM_REC_EX_FLASH
    sAppMem.RecMemType_u8 = __EXTERNAL_FALSH;
    eFlash_Init();
    
    AppMem_Init_Index_Record();
            
    fevent_enable( sEventAppMem, _EVENT_MEM_POWER_ON);
    
    sS25FLvar.pMem_Write_OK = AppMem_Cb_Write_OK;
    sS25FLvar.pMem_Read_OK  = AppMem_Cb_Read_OK;   

    sS25FLvar.pMem_Wrtie_ERROR = AppMem_Cb_Wrtie_ERROR;
    sS25FLvar.pMem_Erase_Chip_OK = AppMem_Cb_Erase_Chip_OK;
    sS25FLvar.pMem_Erase_Chip_Fail = AppMem_Cb_Erase_Chip_Fail ; 
#endif

#ifdef USING_AT_CONFIG    
    CheckList_AT_CONFIG[_ERASE_MEMORY].CallBack = AppMem_SER_rErase_Memory;
    CheckList_AT_CONFIG[_QUERY_LAST_FOR_LOG].CallBack = AppMem_SER_Get_Last_For_LOG;
    CheckList_AT_CONFIG[_QUERY_LAST_FOR_OPERA].CallBack = AppMem_SER_Get_Last_For_OPERA;
    CheckList_AT_CONFIG[_QUERY_LAST_FOR_EVENT].CallBack = AppMem_SER_Get_Last_For_EVENT;
    CheckList_AT_CONFIG[_QUERY_LAST_FOR_GPS].CallBack = AppMem_SER_Get_Last_For_GPS;  
    CheckList_AT_CONFIG[_QUERY_INDEX_LOG].CallBack = AppMem_SER_Get_Index_Log;   
#endif    
}



/*--------------Func Callback --------------*/

static uint8_t _Cb_mem_Power_On(uint8_t event)
{
#ifdef MEM_REC_EX_FLASH
    static uint8_t Step = 0;
#endif
    fevent_disable( sEventAppMem, _EVENT_MEM_CHECK_NEW_TSVH);    
    fevent_disable( sEventAppMem, _EVENT_MEM_CHECK_NEW_EVENT);
    fevent_disable( sEventAppMem, _EVENT_MEM_CHECK_NEW_GPS);
    fevent_disable( sEventAppMem, _EVENT_MEM_CTRL_WRITE);

#ifdef MEM_REC_EX_EEPROM
    sCAT24Var.wPending_u8 = false;
    sCAT24Var.rPending_u8 = false;
#endif
    
#ifdef MEM_REC_IN_FLASH
    sOnFlash.wPending_u8 = false;
    sOnFlash.rPending_u8 = false;
#endif

#ifdef MEM_REC_EX_FLASH
    sS25FLvar.Status_u8 = false;
    sS25FLvar.wPending_u8 = false;
    sS25FLvar.rPending_u8 = false;
        
    if ( (*sAppMem.pModePower_u8 == _POWER_MODE_SAVE) 
        && ( sAppMem.cHardReset_u32 > MEM_MAX_HARD_RESET) )
    {
        return 1;
    }
    
    switch (Step)
    {
        case 0:
            sAppMem.cHardReset_u32++;
            sEventAppMem[_EVENT_MEM_POWER_ON].e_period = 500;
            AppMem_PowerOff();
            break;
        case 1:
            UTIL_Printf_Str( DBLEVEL_M, "u_app_mem: power on\r\n" );
            sEventAppMem[_EVENT_MEM_POWER_ON].e_period = 100;
            AppMem_PowerOn();
            break;
        case 2:
            Step = 0;
            fevent_active( sEventAppMem, _EVENT_MEM_TEST_WRITE);
            return 1;
        default:
            Step = 0;
            break;     
    }
    
    Step++;
    fevent_enable( sEventAppMem, event);
#else
    fevent_active( sEventAppMem, _EVENT_MEM_TEST_WRITE);
#endif

    
	return 1;
}



static uint8_t _Cb_mem_Test_Write(uint8_t event)
{
    if (AppMem_Test_Write() == true)
    {
        if (AppMem_Rec_Status() == ERROR)
        {
            UTIL_Printf_Str( DBLEVEL_M, "u_app_mem: mem write error!\r\n" );
            //Quay lai power on lai eflash sau 5s
            sEventAppMem[_EVENT_MEM_POWER_ON].e_period = 10000;
            fevent_enable( sEventAppMem, _EVENT_MEM_POWER_ON);
            return 1;
        }
        
        sAppMem.cHardReset_u32 = 0;
            
        fevent_active( sEventAppMem, _EVENT_MEM_INIT_POS_REC);
        
        fevent_enable( sEventAppMem, _EVENT_MEM_CHECK_NEW_TSVH);    
        fevent_enable( sEventAppMem, _EVENT_MEM_CHECK_NEW_EVENT);
        fevent_enable( sEventAppMem, _EVENT_MEM_CHECK_NEW_GPS);
        fevent_enable( sEventAppMem, _EVENT_MEM_CTRL_WRITE);
        return 1;
    }
    
    fevent_enable( sEventAppMem, event);
    
	return 1;
}

/*
    Func Init Pos Record in Flash
        + Check Write en: to determine Flash OK
        + Read Rec from start to find Pos

        + Tao 2 case dau:
            ++ HARD RESET
            ++ TEST WRITE
        + Cac case tiep theo Init Data: TSVHA, TSVHB, EventA, EventB, LOG
        
*/
static uint8_t _Cb_mem_Init_Pos_Rec(uint8_t event)
{
#ifdef MEM_REC_EX_FLASH
    sAppMem.KindRead_u8 = _MEM_READ_INIT;
    
    if (AppMem_Is_Write_To_PartAB() != true)
    {
        sS25FLvar.Status_u8 = true;
        return 1;
    }
    
    switch (sS25FLInit.StepInit_u8)
    {
        case _MEM_DATA_TSVH_A:
            if (eFlash_Check_Pos_PartA(&sRecTSVH) == true)
            {
                UTIL_Printf_Str( DBLEVEL_M, "u_app_mem: check addr tsvh a ok!\r\n" );
                sS25FLInit.StepInit_u8++;
            }
            break;
        case _MEM_DATA_TSVH_B:
            if (eFlash_Check_Pos_PartB(&sRecTSVH) == true)
            {
                UTIL_Printf_Str( DBLEVEL_M, "u_app_mem: check addr tsvh b ok!\r\n" );
                sS25FLInit.StepInit_u8++;
            }
            break;
        case _MEM_DATA_EVENT_A:
            if (eFlash_Check_Pos_PartA(&sRecEvent) == true)
            {
                UTIL_Printf_Str( DBLEVEL_M, "u_app_mem: check addr event a ok!\r\n" );
                sS25FLInit.StepInit_u8++;
            }
            break;
        case _MEM_DATA_EVENT_B:
            if (eFlash_Check_Pos_PartB(&sRecEvent) == true)
            {
                UTIL_Printf_Str( DBLEVEL_M, "u_app_mem: check addr event b ok!\r\n" );
                sS25FLInit.StepInit_u8++;
            }
            break;
        case _MEM_DATA_LOG:
            if (eFlash_Check_Pos_PartA(&sRecLog) == true)
            {
                UTIL_Printf_Str( DBLEVEL_M, "u_app_mem: check addr log a ok!\r\n" );
                sS25FLInit.StepInit_u8++;
            }
            break;
        case _MEM_DATA_GPS_A:
            if (eFlash_Check_Pos_PartA(&sRecGPS) == true)
            {
                UTIL_Printf_Str( DBLEVEL_M, "u_app_mem: check addr gps a ok!\r\n" );
                sS25FLInit.StepInit_u8++;
            }
            break;
        case _MEM_DATA_GPS_B:
            if (eFlash_Check_Pos_PartB(&sRecGPS) == true)
            {
                UTIL_Printf_Str( DBLEVEL_M, "u_app_mem: check addr gps b ok!\r\n" );
                sS25FLInit.StepInit_u8++;
            }
            break;
        default:
            sS25FLInit.StepInit_u8 = 0;
            sS25FLvar.Status_u8 = true;  //ket thuc
            return 1;
    }

    fevent_enable( sEventAppMem, event);
#endif
    
	return 1;
}




static uint8_t _Cb_mem_Check_New_Tsvh (uint8_t event)
{      
    static uint16_t MAX_COUNT_PENDING = 0;
    
    if (AppMem_Rec_Status() == true)
    {            
        MAX_COUNT_PENDING = MAX_TIME_RETRY_READ / sEventAppMem[_EVENT_MEM_CHECK_NEW_TSVH].e_period;
        
        /*  + K co item nao trong queue set pending ve false   */
        if ( ( AppMem_Queue_Read_Empty() == true) && (sAppMem.Pending_u8 == true) )
        {
            sAppMem.cPending_u8++;
            if (sAppMem.cPending_u8 >= MAX_COUNT_PENDING ) 
            {
                sAppMem.cPending_u8 = 0;
                sAppMem.Pending_u8 = false;
                //Um mark all mess
                mUnmark_Mess_Share_Buff();
            }  
        }

        if ( sRecTSVH.IndexSend_u16 != sRecTSVH.IndexSave_u16 )
        {
            if (sAppMem.Pending_u8 == false)
            {
                UTIL_Printf_Str( DBLEVEL_M, "u_app_mem: pending new mess TSVH\r\n" );
                sAppMem.cPending_u8 = 0;
                mUnmark_Mess_Share_Buff();
                
                if (AppMem_Read_Data(sAppMem.RecMemType_u8, _MEM_DATA_TSVH_B, \
                                     _MEM_READ_NEW_MESS, 0, sRecTSVH.SizeRecord_u16) == true)
                    sAppMem.Pending_u8 = true;
            } 
        }
    }
    
    fevent_enable( sEventAppMem, event);

    return 1;
}


static uint8_t _Cb_mem_Check_New_Event (uint8_t event)
{
    if ( (AppMem_Rec_Status() == true) && (sRecEvent.IndexSend_u16 != sRecEvent.IndexSave_u16) )
    { 
        if (sAppMem.Pending_u8 == false)
        {
            UTIL_Printf_Str( DBLEVEL_M, "u_app_mem: pending new mess EVENT\r\n" );
            sAppMem.cPending_u8 = 0;
            mUnmark_Mess_Share_Buff();
                
            if (AppMem_Read_Data(sAppMem.RecMemType_u8,
                                 _MEM_DATA_EVENT_B, _MEM_READ_NEW_MESS, 0, sRecEvent.SizeRecord_u16) == true)
                sAppMem.Pending_u8 = true;
        } 
    }
    
    fevent_enable( sEventAppMem, event);
    
    return 1;
}


static uint8_t _Cb_mem_Check_New_Gps (uint8_t event)
{
    if ( (AppMem_Rec_Status() == true) && (sRecGPS.IndexSend_u16 != sRecGPS.IndexSave_u16))
    { 
        if (sAppMem.Pending_u8 == false)
        {
            UTIL_Printf_Str( DBLEVEL_M, "u_app_mem: pending new mess GPS\r\n" );
            sAppMem.cPending_u8 = 0;
            mUnmark_Mess_Share_Buff();
            
            if (AppMem_Read_Data(sAppMem.RecMemType_u8, _MEM_DATA_GPS_B,
                                 _MEM_READ_NEW_MESS, 0, sRecGPS.SizeRecord_u16) == true)
                sAppMem.Pending_u8 = true;
        } 
    }
    
    fevent_enable( sEventAppMem, event);
    
    return 1;
}

static uint8_t _Cb_mem_Ctrl_Write (uint8_t event)
{
    sQueueMemWriteManager   sqWriteTemp;
    
    if ( (AppMem_Rec_Status() == true)
        && (qGet_Number_Items (&qMemWrite) != 0) )
    {
        UTIL_Printf_Str( DBLEVEL_H, "u_ex_eeprom: have item write to flash\r\n");
        //Get Item
        qQueue_Receive(&qMemWrite, (sQueueMemWriteManager *) &sqWriteTemp, 0);
        
        switch (sqWriteTemp.Type_u8)
        {
            case __EXTERNAL_EEPROM:
            #ifdef MEM_REC_EX_EEPROM
                //check status
                if ( (sCAT24Var.Status_u8 == true)
                    && (sCAT24Var.wPending_u8 != true) )
                {
                    //lay addr, length: 
                    if (AppMem_Cacul_Addr_Mess(sqWriteTemp.TypeData_u8, __MEM_WRITE) != 0xFFFFFFFF)  
                    {
                        sCAT24Var.sHWrite.Addr_u32 = AppMem_Cacul_Addr_Mess(sqWriteTemp.TypeData_u8, __MEM_WRITE);
                    } else
                    {
                        sCAT24Var.sHWrite.Addr_u32 = sqWriteTemp.Addr_u32;
                    }
                    
                    sCAT24Var.sHWrite.Length_u16 = sqWriteTemp.Length_u16;
                    UTIL_MEM_cpy(sCAT24Var.sHWrite.aData, sqWriteTemp.aData, sqWriteTemp.Length_u16);
                    //
                    sCAT24Var.wPending_u8 = true;
                    fevent_active( sEventCAT24, _EVENT_CAT24Mxx_WRITE);
                }
            #endif
                break;
            case __INTERNAL_FLASH:
            #ifdef MEM_REC_IN_FLASH
                //check status
                if ( (sOnFlash.Status_u8 == true)
                    && (sOnFlash.wPending_u8 != true) )
                {
                    //lay addr, length: 
                    if (AppMem_Cacul_Addr_Mess(sqWriteTemp.TypeData_u8, __MEM_WRITE) != 0xFFFFFFFF)  
                    {
                        sOnFlash.sHWrite.Addr_u32 = AppMem_Cacul_Addr_Mess(sqWriteTemp.TypeData_u8, __MEM_WRITE);
                    } else
                    {
                        sOnFlash.sHWrite.Addr_u32 = sqWriteTemp.Addr_u32;
                    }
                    
                    sOnFlash.sHWrite.Length_u16 = sqWriteTemp.Length_u16;
                    UTIL_MEM_cpy(sOnFlash.sHWrite.aData, sqWriteTemp.aData, sqWriteTemp.Length_u16);
                    //
                    sOnFlash.wPending_u8 = true;
                    fevent_active( sEventOnChipFlash, _EVENT_ON_FLASH_WRITE_BUFF);
                }
            #endif
                break;
            case __EXTERNAL_FALSH:
            #ifdef MEM_REC_EX_FLASH
                //check status
                if ( (sS25FLvar.Status_u8 == true)
                    && (sS25FLvar.wPending_u8 != true) )
                {
                    //lay addr, length: 
                    if (AppMem_Cacul_Addr_Mess(sqWriteTemp.TypeData_u8, __MEM_WRITE) != 0xFFFFFFFF)  
                    {
                        sS25FLvar.sHWrite.Addr_u32 = AppMem_Cacul_Addr_Mess(sqWriteTemp.TypeData_u8, __MEM_WRITE);
                    } else
                    {
                        sS25FLvar.sHWrite.Addr_u32 = sqWriteTemp.Addr_u32;
                    }
                    
                    sS25FLvar.sHWrite.Length_u16 = sqWriteTemp.Length_u16;
                    UTIL_MEM_cpy(sS25FLvar.sHWrite.aData, sqWriteTemp.aData, sqWriteTemp.Length_u16);
                    //
                    sS25FLvar.wPending_u8 = true;
                    fevent_active( sEventExFlash, _EVENT_FLASH_WRITE_BUFF);
                }
            #endif
                break;
            default:
                
                break;
        }
        
        fevent_enable( sEventAppMem, event);
    }
    
    return 1;
}


static uint8_t _Cb_mem_Check_Request_At (uint8_t event)
{    
    VLevelDebug = DBLEVEL_MESS;
    UTIL_ADV_TRACE_SetVerboseLevel(DBLEVEL_MESS);
   
    if ( (AppMem_Rec_Status() == true) && (sAppMem.sReqATcmd.Status_u8 == true) )
    {
        if (sAppMem.sReqATcmd.IndexStart_u16 != sAppMem.sReqATcmd.IndexStop_u16)
        {           
            sAppMem.sReqATcmd.Landmark_u32 = RtCountSystick_u32;
            
            AppMem_Read_Data( sAppMem.RecMemType_u8, _MEM_DATA_OTHER, _MEM_READ_RESP_AT,
                              sAppMem.sReqATcmd.pRecord->AddAStart_u32 + sAppMem.sReqATcmd.IndexStart_u16 * sAppMem.sReqATcmd.pRecord->SizeRecord_u16,
                              sAppMem.sReqATcmd.pRecord->SizeRecord_u16);
                            
            //Tang index 
            if (sAppMem.sReqATcmd.IndexStart_u16++ >= sAppMem.sReqATcmd.pRecord->MaxRecord_u16)
            {
                sAppMem.sReqATcmd.IndexStart_u16 = 0;
            }
        } else
        {
            //Ket thuc
            sAppMem.sReqATcmd.Status_u8 = false;

            VLevelDebug = DBLEVEL_M;
            UTIL_ADV_TRACE_SetVerboseLevel(DBLEVEL_M);
            
            return 1;
        }
    }
    
    //qua timeout ma k doc them ban tin -> ket thuc
    if (Check_Time_Out(sAppMem.sReqATcmd.Landmark_u32, 30000) == true)
    {
        sAppMem.sReqATcmd.IndexStart_u16 = sAppMem.sReqATcmd.IndexStop_u16;
    }
    
    if (sAppMem.sReqATcmd.Port_u8 == _AT_REQUEST_SERIAL) 
    {
        sEventAppMem[event].e_period = 500;
    } else
    {
        sEventAppMem[event].e_period = 6000;
    }
    
    fevent_enable( sEventAppMem, event);
    
	return 1;
}




static uint8_t _Cb_mem_Erase (uint8_t event)
{
#ifdef MEM_REC_EX_FLASH
    eFlash_Push_Step_Erase_Chip();
#endif
    
#ifdef MEM_REC_EX_EEPROM
    if (CAT24Mxx_Erase() == 0)
    {
        UTIL_Printf_Str( DBLEVEL_M, "u_app_mem: erase eeprom ok. reset mcu!\r\n" );
        
        AppMem_Cb_Erase_Chip_OK();
    }
#endif
  
#ifdef MEM_REC_IN_FLASH
    UTIL_Printf_Str( DBLEVEL_M, "u_app_mem: erase ok. reset mcu!\r\n" );
    AppMem_Cb_Erase_Chip_OK();
#endif

	return 1;
}


/*--------------Func handle --------------*/

/*
    Func: Call back external flash write
        + Increase Index and Addr: Active Event Check New Mess Modem
*/
void AppMem_Cb_Write_OK (uint32_t Addr)
{
    qQueue_Receive(&qMemWrite, NULL, 1);
    fevent_active( sEventAppMem, _EVENT_MEM_CTRL_WRITE);
        
    switch ( AppMem_Get_Type_From_Addr(Addr) )
    {
        case _MEM_DATA_TSVH_A:
            UTIL_Printf_Str( DBLEVEL_M, "u_app_mem: increase addr tsvh a\r\n");
            //Increase AddA_u32 TSVH: If using sim....
            if (AppMem_Inc_Index_Save(&sRecTSVH) == false)
                AppMem_Write_Data(sAppMem.RecMemType_u8, _MEM_DATA_TSVH_B, 0, 
                                  (uint8_t *) "Full Data Save!\r\n", 17, sRecTSVH.SizeRecord_u16);
            //Active Event check new Mess
            fevent_active( sEventAppMem, _EVENT_MEM_CHECK_NEW_TSVH);
            break;
        case _MEM_DATA_TSVH_B:
            UTIL_Printf_Str( DBLEVEL_M, "u_app_mem: increase addr tsvh b\r\n");
            //Increase AddB_u32 TSVH: If using sim....
            AppMem_Inc_Index_Send_1(&sRecTSVH, 1);
            //eable Event check new Mess: to period 5s send next
            fevent_active( sEventAppMem, _EVENT_MEM_CHECK_NEW_TSVH);
            break;
        case _MEM_DATA_EVENT_A:
            UTIL_Printf_Str( DBLEVEL_M, "u_app_mem: increase addr event a\r\n");
            //Increase AddA_u32 Event: If using sim....
            if (AppMem_Inc_Index_Save(&sRecEvent) == false)
                AppMem_Write_Data(sAppMem.RecMemType_u8, _MEM_DATA_EVENT_B, 0, 
                                  (uint8_t *) "Full Data Save!\r\n", 17, sRecEvent.SizeRecord_u16);
            //Active Event check new Mess
            fevent_active( sEventAppMem, _EVENT_MEM_CHECK_NEW_EVENT);
            break;
        case _MEM_DATA_EVENT_B:
            UTIL_Printf_Str( DBLEVEL_M, "u_app_mem: increase addr event b\r\n");
            //Increase AddB_u32 Event: If using sim....
            AppMem_Inc_Index_Send_1(&sRecEvent, 1);
            //eable Event check new Mess: to period 5s send next
            fevent_active( sEventAppMem, _EVENT_MEM_CHECK_NEW_EVENT);
            break;
        case _MEM_DATA_LOG:
            UTIL_Printf_Str( DBLEVEL_M, "u_app_mem: increase addr log a\r\n");
            AppMem_Inc_Index_Save(&sRecLog);
            break;
        case _MEM_DATA_GPS_A:
            UTIL_Printf_Str( DBLEVEL_M, "u_app_mem: increase addr gps a\r\n");
            if (AppMem_Inc_Index_Save(&sRecGPS) == false)
                AppMem_Write_Data(sAppMem.RecMemType_u8, _MEM_DATA_GPS_B, 0, 
                                  (uint8_t *) "Full Data Save!\r\n", 17, sRecGPS.SizeRecord_u16);
            
            fevent_active( sEventAppMem, _EVENT_MEM_CHECK_NEW_GPS);
            break;
        case _MEM_DATA_GPS_B:
            UTIL_Printf_Str( DBLEVEL_M, "u_app_mem: increase addr gps b\r\n");
            //Increase AddB_u32 Event: If using sim....
            AppMem_Inc_Index_Send_1(&sRecGPS, 1);
            //eable Event check new Mess: to period 5s send next
            fevent_active( sEventAppMem, _EVENT_MEM_CHECK_NEW_GPS);
            break;
        default:
            break;
    }
}


/*
    Func: Call back external flash read
        + Push Record to qFlashSim and active event Check new mess
*/

void AppMem_Cb_Read_OK (uint8_t Kind, uint32_t Addr, uint8_t *pData, uint16_t length)
{
    uint16_t RealLength = 0;  
    sData   sDataRead;
    
    qQueue_Receive(&qMemRead, NULL, 1);
    
    //Tro vao data read
    sDataRead.Data_a8 = pData;
    sDataRead.Length_u16 = length;
    
    RealLength = (*(sDataRead.Data_a8 + 1) << 8) + *(sDataRead.Data_a8 + 2);

    switch (Kind)
    {
        case _MEM_READ_NEW_MESS:      
            AppMem_Read_New_Mess_OK(AppMem_Get_Type_From_Addr(Addr), sDataRead.Data_a8, RealLength);
            break;
        case _MEM_READ_RESP_AT:
            AppMem_Process_Req_AT(AppMem_Get_Type_From_Addr(Addr), sDataRead.Data_a8, RealLength);
            break;
        case _MEM_READ_INIT:
            Appmem_Check_Rec_Init_Data(Addr, sDataRead.Data_a8, sDataRead.Length_u16);
            break;
        default:
            break;
    }
}



/*
    Func: App sim push data to write to extenal flash
        + Type Data: 0: TSVH ,1: Event  2: LOG
        + PartAorB: Xac dinh dia chi A hay B
        + pData
        + Length
*/
void AppMem_Write_Data (uint8_t MemType, uint8_t Typedata, uint32_t Addr, 
                        uint8_t *pData, uint16_t Length, uint16_t MAX_LENGTH)
{
    sQueueMemWriteManager  qFlashTemp;
    uint16_t Count = 0, i = 0;
    uint8_t TempCrc = 0;
      
    qFlashTemp.Type_u8 = MemType;
    qFlashTemp.TypeData_u8 = Typedata;
    qFlashTemp.Addr_u32 = Addr;
    
    qFlashTemp.aData[Count++] = BYTE_TEMP_FIRST;
    qFlashTemp.aData[Count++] = (Length >> 8) & 0xFF;
    qFlashTemp.aData[Count++] = Length & 0xFF;
        
    for (i = 0; i < Length; i++)
        qFlashTemp.aData[Count++] =  pData[i];
    
    //Gioi han size luu vao
    if (Count > (MAX_LENGTH - 1) )
        Count = MAX_LENGTH - 1;
    
    // caculator crc
    Count++;
	for (i = 0; i < (Count - 1); i++)
		TempCrc ^= qFlashTemp.aData[i];

    qFlashTemp.aData[Count-1] = TempCrc;   
    qFlashTemp.Length_u16 = Count;
    
    //Send to queue fwrite
    AppMem_Send_To_Queue_Write(&qFlashTemp);
}



uint8_t AppMem_Read_Data (uint8_t MemType, uint8_t Typedata, uint8_t Kind, uint32_t Addr, uint16_t Length)
{
    uint8_t Result = false;
    static uint32_t LandMark_u32 = 0; 
    
    switch (MemType)
    {
        case __EXTERNAL_EEPROM:
        #ifdef MEM_REC_EX_EEPROM
            if ( (sCAT24Var.Status_u8 == true)
                && sCAT24Var.rPending_u8 == false) )
            {
                sCAT24Var.rPending_u8 = true;
                LandMark_u32 = RtCountSystick_u32;
                //lay addr, length: 
                if (AppMem_Cacul_Addr_Mess(Typedata, __MEM_READ) != 0xFFFFFFFF)   
                {
                    sCAT24Var.sHRead.Addr_u32 = AppMem_Cacul_Addr_Mess(Typedata, __MEM_READ);
                } else
                {
                    sCAT24Var.sHRead.Addr_u32 = Addr;
                }
                
                sCAT24Var.sHRead.Length_u16 = Length;
                sCAT24Var.sHRead.Kind_u8 = Kind;
                
                fevent_active( sEventCAT24, _EVENT_CAT24Mxx_READ);
                Result = true;
            }
        #endif
            break;
        case __INTERNAL_FLASH:
        #ifdef MEM_REC_IN_FLASH
            if ( (sOnFlash.Status_u8 == true)
                && (sOnFlash.rPending_u8 == false) )
            {
                sOnFlash.rPending_u8 = true;
                LandMark_u32 = RtCountSystick_u32;
                
                //lay addr, length: 
                if (AppMem_Cacul_Addr_Mess(Typedata, __MEM_READ) != 0xFFFFFFFF)   
                {
                    sOnFlash.sHRead.Addr_u32 = AppMem_Cacul_Addr_Mess(Typedata, __MEM_READ);
                } else
                {
                    sOnFlash.sHRead.Addr_u32 = Addr;
                }
                
                sOnFlash.sHRead.Length_u16 = Length;
                sOnFlash.sHRead.Kind_u8 = Kind;
                
                fevent_active( sEventOnChipFlash, _EVENT_ON_FLASH_READ_BUFF);
                Result = true;
            }
        #endif
            break;
        case __EXTERNAL_FALSH:
        #ifdef MEM_REC_EX_FLASH
            if ( (sS25FLvar.Status_u8 == true)
                   && (sS25FLvar.rPending_u8 == false) ) 
            {
                sS25FLvar.rPending_u8 = true;
                LandMark_u32 = RtCountSystick_u32;
                //lay addr, length: 
                if (AppMem_Cacul_Addr_Mess(Typedata, __MEM_READ) != 0xFFFFFFFF)   
                {
                    sS25FLvar.sHRead.Addr_u32 = AppMem_Cacul_Addr_Mess(Typedata, __MEM_READ);
                } else
                {
                    sS25FLvar.sHRead.Addr_u32 = Addr;
                }
                
                sS25FLvar.sHRead.Length_u16 = Length;
                sS25FLvar.sHRead.Kind_u8 = Kind;
                
                fevent_active( sEventExFlash, _EVENT_FLASH_READ_BUFF);
                Result = true;
            }
        #endif
            break;
        default: 
            break;
    }
    
    if (Check_Time_Out(LandMark_u32, MEM_MAX_TIMEOUT_READ) == true)
    {
        LandMark_u32 = RtCountSystick_u32;
        fevent_active( sEventAppMem, _EVENT_MEM_POWER_ON);
    }
    
    return Result;
}


/*
    Func: check data read in Init flash
        
*/
void Appmem_Check_Rec_Init_Data (uint32_t Addr, uint8_t *pData, uint16_t Length)
{  
#ifdef MEM_REC_EX_FLASH
    switch (sS25FLInit.StepInit_u8)
    {
        case _MEM_DATA_TSVH_A:
        case _MEM_DATA_EVENT_A:
        case _MEM_DATA_LOG:
            eFlash_Check_Data_Init_Pos(Addr, pData, Length, 0);
            break;
        case _MEM_DATA_TSVH_B:
        case _MEM_DATA_EVENT_B:
            eFlash_Check_Data_Init_Pos(Addr, pData, Length, 1);
            break;
        default:
            break;
    }
    
    fevent_active( sEventAppMem, _EVENT_MEM_INIT_POS_REC);
#endif
}



/*
    Func: Cb write fail: Do chua dc xoa vung nho. Active Check Pos Init
*/
void AppMem_Cb_Wrtie_ERROR (uint32_t Addr, uint8_t *pdata, uint16_t length)
{
    qQueue_Receive(&qMemWrite, NULL, 1);
    fevent_active( sEventAppMem, _EVENT_MEM_CTRL_WRITE);
    //Forward packet to sim
    AppMem_Forward_Mess(Addr, pdata, length);
    
    //Neu quan ly Index -> Van tang vi ti write
    
    if (AppMem_Is_Write_To_PartAB() != true)
    {
        switch ( AppMem_Get_Type_From_Addr(Addr) )
        {
            case _MEM_DATA_TSVH_A:
                UTIL_Printf_Str( DBLEVEL_M, "u_app_mem: w fail -> inc tsvh a\r\n");
                AppMem_Inc_Index_Save(&sRecTSVH);
                break;
            case _MEM_DATA_EVENT_A:
                UTIL_Printf_Str( DBLEVEL_M, "u_app_mem: w fail -> inc event a\r\n");
                AppMem_Inc_Index_Save(&sRecEvent);
                break;
            case _MEM_DATA_LOG:
                UTIL_Printf_Str( DBLEVEL_M, "u_app_mem: w fail -> inc log a\r\n");
                AppMem_Inc_Index_Save(&sRecLog);
                break;
            case _MEM_DATA_GPS_A:
                UTIL_Printf_Str( DBLEVEL_M, "u_app_mem: w fail -> inc gps a\r\n");
                AppMem_Inc_Index_Save(&sRecGPS);
                break;
            default:
                break;
        }
    }
    
    fevent_active( sEventAppMem, _EVENT_MEM_POWER_ON);
}

/*
    Func: Cb write fail: Do chua dc xoa vung nho. Active Check Pos Init
*/
void AppMem_Cb_Erase_Chip_OK(void)
{
    sRecTSVH.IndexSend_u16 = 0;
    sRecTSVH.IndexSave_u16 = 0;
    sRecEvent.IndexSend_u16 = 0;
    sRecEvent.IndexSave_u16 = 0;
    sRecLog.IndexSend_u16 = 0;
    sRecLog.IndexSave_u16 = 0;
    sRecGPS.IndexSend_u16 = 0;
    sRecGPS.IndexSave_u16 = 0;
    
    AppMem_Save_Index_Rec();

    //Reset MCU
    if (sAppMem.pReset_MCU != NULL)
        sAppMem.pReset_MCU();
}

/*
    Func: Cb write fail: Do chua dc xoa vung nho. Active Check Pos Init
*/
void AppMem_Cb_Erase_Chip_Fail (void)
{
    //Try again?
    UTIL_Log_Str( DBLEVEL_M, "u_app_mem: cb erase chip fail!\r\n" );
}



void AppMem_Erase_New_Sector (uint32_t Addr)
{
#ifdef MEM_REC_EX_FLASH
    if (AppMem_Is_Write_To_PartAB() == true)
    {
        if ( (Addr % S25FL_SECTOR_SIZE) == 0 )
        {
            eFlash_Erase_Sector(Addr);
        }
    }
#endif 
}


/*
    Func: Procses RespAT
        + Check Req Port: Server |Serial
            ++ Server: -> Copy to buff at resp server
            ++ Terminal: Print ra uart
*/

/*
    Func: Procses RespAT
        + Check Req Port: Server |Serial
            ++ Server: -> Copy to buff at resp server
            ++ Terminal: Print ra uart
*/
void AppMem_Process_Req_AT (uint8_t type, uint8_t *pData, uint16_t Length)
{     
//    uint16_t i = 0;
    sData   sDataReal;
//    uint8_t TempCrc = 0;
    uint8_t IsValidData = true;
    sRecordMemoryManager *pRecord;   
    
    static char strTSVH[] = {" TSVH: "};
    static char strEvent[] = {" EVENT: "};
    static char strGPS[] = {" GPS: "};
    static char strLog[] = {" LOG: "};
    char *pMessType = NULL;
    
    switch (type)
    {
        case _MEM_DATA_TSVH_A:
        case _MEM_DATA_TSVH_B:
            pRecord = &sRecTSVH;
            pMessType = strTSVH;
            break;
        case _MEM_DATA_EVENT_A:
        case _MEM_DATA_EVENT_B:
             pRecord = &sRecEvent;
             pMessType = strEvent;
            break;
        case _MEM_DATA_GPS_A:
        case _MEM_DATA_GPS_B:
            pRecord = &sRecGPS;
            pMessType = strGPS;
            break;
        case _MEM_DATA_LOG:
            pRecord = &sRecLog;
            pMessType = strLog;
            break;
        default:
            break;
    }

    //Check Crc
    if (Length < pRecord->SizeRecord_u16)
    {
        //Tro vao data real
        sDataReal.Data_a8 = pData + 3;  //1byte 0xAA + 2byte length
        sDataReal.Length_u16 = Length;
        
//        for (i = 0; i < (Length + 3); i++)
//        {
//            TempCrc ^= *(pData + i);
//        }
//        
//        if ( TempCrc != *(pData + Length + 3) )
//            IsValidData = false;
    } else
    {
        IsValidData = false;
    }
    
    //Copy sang buff App sim Data
    if (IsValidData == true)
    {
        if (sAppMem.pRespond != NULL)
        {
            if (sAppMem.sReqATcmd.Port_u8 == _AT_REQUEST_SERIAL)
            {
                UTIL_Printf_Str( DBLEVEL_MESS, "\r\nrecord" );
                UTIL_Printf_Str( DBLEVEL_MESS, (const char *) pMessType );
                if (type == _MEM_DATA_LOG)
                {
                    UTIL_Printf( DBLEVEL_MESS, (uint8_t *) sDataReal.Data_a8, sDataReal.Length_u16);
                } else
                {
                    UTIL_Printf_Hex( DBLEVEL_MESS, (uint8_t *) sDataReal.Data_a8, sDataReal.Length_u16);
                }
                UTIL_Printf_Str( DBLEVEL_MESS, "-end\r\n" );
            } else
            {
                sAppMem.pRespond(sAppMem.sReqATcmd.Port_u8, pData, Length, 0);
            }            
        }
    } else
    {
        sAppMem.pRespond(sAppMem.sReqATcmd.Port_u8, "u_app_mem: invalid record!", strlen("u_app_mem: invalid record!"), 0);
    }
}



/*
    Func: Check Mess read from flash invalid
        + Increase Index send
        + Or MEM_REC_EX_FLASH: Push data to write to part B
    +note: Neu trong queue ton tai nhieu read: sau do invalid thi
            Can check xem bang indexsave truoc khi++
*/
void AppMem_Record_Invalid_Handle (uint8_t TypeData)
{
    switch (TypeData)
    {
        case _MEM_DATA_TSVH_A:
        case _MEM_DATA_TSVH_B:
            if (AppMem_Inc_Index_Send_2(&sRecTSVH, 1) == false)
                AppMem_Write_Data(sAppMem.RecMemType_u8, _MEM_DATA_TSVH_B, 0, 
                                  (uint8_t *) "Invalid Data\r\n", 14, sRecTSVH.SizeRecord_u16);
            break;
        case _MEM_DATA_EVENT_A:
        case _MEM_DATA_EVENT_B:
            if (AppMem_Inc_Index_Send_2(&sRecEvent, 1) == false)
                AppMem_Write_Data(sAppMem.RecMemType_u8, _MEM_DATA_EVENT_B, 0, 
                                  (uint8_t *) "Invalid Data\r\n", 14, sRecEvent.SizeRecord_u16);
            break;
        case _MEM_DATA_GPS_A:
        case _MEM_DATA_GPS_B:
            if (AppMem_Inc_Index_Send_2(&sRecGPS, 1) == false)
                AppMem_Write_Data(sAppMem.RecMemType_u8, _MEM_DATA_GPS_B, 0, 
                                  (uint8_t *) "Invalid Data\r\n", 14, sRecGPS.SizeRecord_u16);
            break;
        default:
            break;
    }
}





uint8_t AppMem_Inc_Index_Save (sRecordMemoryManager *sRec)
{
    uint8_t Result = true;
    
    sRec->IndexSave_u16 = (sRec->IndexSave_u16 + 1) % sRec->MaxRecord_u16;
    sRec->AddA_u32 = sRec->AddAStart_u32 + sRec->IndexSave_u16 * sRec->SizeRecord_u16;
    
    if (AppMem_Is_Write_To_PartAB() == true)
    {
        //Xoa sector tiep theo: Neu nhu dung quan ly AB
        AppMem_Erase_New_Sector(sRec->AddA_u32);
    } else
    {
        AppMem_Save_Index_Rec();   
    }
    
    if (sRec->IndexSave_u16 == sRec->IndexSend_u16)
    {
        if (AppMem_Is_Write_To_PartAB() == true)
        {
            Result = false; 
        } else
        {
            //Tang index send
            sRec->IndexSend_u16 = (sRec->IndexSend_u16 + 1) % sRec->MaxRecord_u16;
            sRec->AddB_u32 = sRec->AddBStart_u32 + sRec->IndexSend_u16 * sRec->SizeRecord_u16;
            AppMem_Save_Index_Rec(); 
        }
    }
    
    return Result;
}




uint8_t AppMem_Inc_Index_Send_1 (sRecordMemoryManager *sRec, uint8_t Num)
{       
    sAppMem.Pending_u8 = false; 
    
    if (sRec->IndexSend_u16 != sRec->IndexSave_u16)
    {
        sRec->IndexSend_u16 = (sRec->IndexSend_u16 + Num) % sRec->MaxRecord_u16;
        sRec->AddB_u32 = sRec->AddBStart_u32 + sRec->IndexSend_u16 * sRec->SizeRecord_u16;
        
        if (AppMem_Is_Write_To_PartAB() == true)
        {
            AppMem_Erase_New_Sector(sRec->AddB_u32);
        } else
        {
            AppMem_Save_Index_Rec(); 
        }
    }
    
    return true;
}
                
                
                
uint8_t AppMem_Inc_Index_Send_2 (sRecordMemoryManager *sRec, uint8_t Num)
{       
    uint8_t Result = true;
    
    if (sRec->IndexSend_u16 != sRec->IndexSave_u16)
    {
        if (AppMem_Is_Write_To_PartAB() == true)
        {
            Result = false;  
        } else
        {
            sAppMem.Pending_u8 = false;
            
            sRec->IndexSend_u16 = (sRec->IndexSend_u16 + Num) % sRec->MaxRecord_u16;
            sRec->AddB_u32 = sRec->AddBStart_u32 + sRec->IndexSend_u16 * sRec->SizeRecord_u16;
            AppMem_Save_Index_Rec(); 
        } 
    }
    
    return Result;
}


/*
    Func: Chuyen tiep du lieu ghi vao khi mem error
*/
void AppMem_Forward_Mess (uint32_t Addr, uint8_t *pData, uint16_t Length)
{
    uint8_t MessType = DATA_TSVH_OPERA;
    
    switch (AppMem_Get_Type_From_Addr(Addr))   
    {
        case _MEM_DATA_TSVH_A:
        #if (TSVH_OPERA_OR_FLASH == TSVH_OPERA)
            MessType = DATA_TSVH_OPERA;
        #elif (TSVH_OPERA_OR_FLASH == TSVH_FLASH)
            MessType = DATA_TSVH_FLASH;
        #else 
            MessType = DATA_TSVH_MULTI;
        #endif

            break;
        case _MEM_DATA_EVENT_A:
            MessType = DATA_EVENT;
            break; 
        case _MEM_DATA_GPS_A:
            MessType = DATA_GPS;
            break; 
        default:
            return;
    
    }
    
    UTIL_Printf_Str( DBLEVEL_H, "u_app_mem: mem forward data\r\n" );
    
    //Copy vao buff sim: Bo qua 3 byte dau va byte cuoi crc     
    mGet_Data_From_Mem(MessType, (pData + 3), Length - 4);
    
    #ifdef USING_APP_SIM
    if (sEventAppSim[_EVENT_SIM_SEND_MESS].e_status == 0)
        fevent_active( sEventAppSim, _EVENT_SIM_SEND_MESS);
    #endif
    
    #ifdef USING_APP_ETHERNET
        if (sEventAppEth[_EVENT_ETH_SEND_DATA].e_status == 0)
            fevent_active( sEventAppEth, _EVENT_ETH_SEND_DATA);
    #endif
}


/*
    Func: ngat han nguon cua flash ngoai
        + Power Off power
        + Cau hinh chan SPI ve output: xuat muc 0 
*/

void AppMem_PowerOff(void)
{    
#ifdef MEM_REC_EX_FLASH
    FLASH_POWER_OFF;
    
    HAL_GPIO_WritePin(FLASH_RESET_GPIO_Port, FLASH_RESET_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_RESET);
#endif
}


/*
    Func: Power on again ex flash
*/

void AppMem_PowerOn(void)
{
#ifdef MEM_REC_EX_FLASH
    HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(FLASH_RESET_GPIO_Port, FLASH_RESET_Pin, GPIO_PIN_SET);
    FLASH_POWER_ON;
    MX_SPI2_Init();
#endif
}




/*
    Func: Kiem tra xem con record can gui di k
*/

uint8_t AppMem_Is_New_Record (void)
{
    if ( ( sRecTSVH.IndexSend_u16 != sRecTSVH.IndexSave_u16 ) ||
        ( sRecEvent.IndexSend_u16 != sRecEvent.IndexSave_u16 ) )
        return true;
    
    return false;
}


/*
    //Kiem tra record format:   
        + Check Length, crc
        + If valid: Push to sim
            Invalid notify invalid
*/
void AppMem_Read_New_Mess_OK (uint8_t type, uint8_t *pData, uint16_t Length)
{
    uint8_t MessType = DATA_TSVH_OPERA;   
    uint16_t i = 0;
    sData   sDataReal;
    uint8_t TempCrc = 0;
    uint8_t IsValidData = true;
    sRecordMemoryManager *pRecord;    
        
    switch (type)
    {
        case _MEM_DATA_TSVH_A:
        case _MEM_DATA_TSVH_B:
        #if (TSVH_OPERA_OR_FLASH == TSVH_OPERA)
            MessType = DATA_TSVH_OPERA;
        #elif (TSVH_OPERA_OR_FLASH == TSVH_FLASH)
            MessType = DATA_TSVH_FLASH;
        #else
            MessType = DATA_TSVH_MULTI;
        #endif
            
            pRecord = &sRecTSVH;
            break;
        case _MEM_DATA_EVENT_A:
        case _MEM_DATA_EVENT_B:
            MessType = DATA_EVENT;
            pRecord = &sRecEvent;
            break;
        case _MEM_DATA_GPS_A:
        case _MEM_DATA_GPS_B:
            MessType = DATA_GPS;
            pRecord = &sRecGPS;
            break;
        default:
            break;
    }

    //Check Crc
    if (Length < pRecord->SizeRecord_u16)
    {
        //Tro vao data real
        sDataReal.Data_a8 = pData + 3;  //1byte 0xAA + 2byte length
        sDataReal.Length_u16 = Length;
        
        for (i = 0; i < (Length + 3); i++)
        {
            TempCrc ^= *(pData + i);
        }
        
        if ( TempCrc != *(pData + Length + 3) )
            IsValidData = false;
    } else
    {
        IsValidData = false;
    }
    
    //Copy sang buff App sim Data
    if (IsValidData == true)
    {
        UTIL_Printf_Str( DBLEVEL_M, "u_app_mem: data read from memory:\r\n" );
        UTIL_Printf_Hex( DBLEVEL_H, (uint8_t *) sDataReal.Data_a8, sDataReal.Length_u16);
        UTIL_Printf_Str( DBLEVEL_H, "\r\n" );
        
        mGet_Data_From_Mem(MessType, sDataReal.Data_a8, sDataReal.Length_u16);
    #ifdef USING_APP_SIM
        if (sEventAppSim[_EVENT_SIM_SEND_MESS].e_status == 0)
            fevent_active( sEventAppSim, _EVENT_SIM_SEND_MESS);
    #endif
    
    #ifdef USING_APP_ETHERNET
        if (sEventAppEth[_EVENT_ETH_SEND_DATA].e_status == 0)
            fevent_active( sEventAppEth, _EVENT_ETH_SEND_DATA);
    #endif
    } else
    {
        UTIL_Log_Str( DBLEVEL_M, "u_app_mem: check invalid read record!\r\n" );
        AppMem_Record_Invalid_Handle(type);
    }      
}






void AppMem_Init_MQTT_Infor (void)
{
    

}


void AppMem_Read_MQTT_Infor (void)
{
    

}




/*
    Func: Init Record Index
        + Index Send, index save
*/
void AppMem_Init_Index_Record (void)
{
    uint8_t TempU8 = 0;
    uint8_t aTEMP[24] = {0};
    uint16_t TempU16 = 0;
    
    //Doc vi tri send va luu ra
    TempU8 = *(__IO uint8_t*) ADDR_INDEX_REC;
    
	if (TempU8 != FLASH_BYTE_EMPTY)
	{
        OnchipFlashReadData ((ADDR_INDEX_REC + 2), aTEMP, 24);   
        //TSVH send
        TempU16 = (aTEMP[0] << 8) | aTEMP[1];
        sRecTSVH.IndexSend_u16 = MIN(TempU16, sRecTSVH.MaxRecord_u16);
        
        //TSVH Save
        TempU16 = (aTEMP[2] << 8) | aTEMP[3];        
        sRecTSVH.IndexSave_u16 = MIN(TempU16, sRecTSVH.MaxRecord_u16);
        
        //Event send
        TempU16 = (aTEMP[4] << 8) | aTEMP[5];
        sRecEvent.IndexSend_u16 = MIN(TempU16, sRecEvent.MaxRecord_u16);

        //Event Save
        TempU16 = (aTEMP[6] << 8) | aTEMP[7];
        sRecEvent.IndexSave_u16 = MIN(TempU16, sRecEvent.MaxRecord_u16);
        
        //GPS send
        TempU16 = (aTEMP[8] << 8) | aTEMP[9];
        sRecGPS.IndexSend_u16 = MIN(TempU16, sRecGPS.MaxRecord_u16);

        //GPS Save
        TempU16 = (aTEMP[10] << 8) | aTEMP[11];
        sRecGPS.IndexSave_u16 = MIN(TempU16, sRecGPS.MaxRecord_u16);
        
        //log send
        TempU16 = (aTEMP[12] << 8) | aTEMP[13];
        sRecLog.IndexSend_u16 = MIN(TempU16, sRecLog.MaxRecord_u16);
        
        //log Save
        TempU16 = (aTEMP[14] << 8) | aTEMP[15];
        sRecLog.IndexSave_u16 = MIN(TempU16, sRecLog.MaxRecord_u16);
    } else
    {
    	AppMem_Save_Index_Rec();
    }
    
    //Tinh lai Adr theo Index
    sRecTSVH.AddA_u32 = sRecTSVH.AddAStart_u32 + sRecTSVH.IndexSave_u16 * sRecTSVH.SizeRecord_u16;
    sRecTSVH.AddB_u32 = sRecTSVH.AddBStart_u32 + sRecTSVH.IndexSend_u16 * sRecTSVH.SizeRecord_u16;
    
    sRecEvent.AddA_u32 = sRecEvent.AddAStart_u32 + sRecEvent.IndexSave_u16 * sRecEvent.SizeRecord_u16;
    sRecEvent.AddB_u32 = sRecEvent.AddBStart_u32 + sRecEvent.IndexSend_u16 * sRecEvent.SizeRecord_u16;
    
    sRecLog.AddA_u32 = sRecLog.AddAStart_u32 + sRecLog.IndexSave_u16 * sRecLog.SizeRecord_u16;
    sRecLog.AddB_u32 = sRecLog.AddBStart_u32 + sRecLog.IndexSend_u16 * sRecLog.SizeRecord_u16;
    
    sRecGPS.AddA_u32 = sRecGPS.AddAStart_u32 + sRecGPS.IndexSave_u16 * sRecGPS.SizeRecord_u16;
    sRecGPS.AddB_u32 = sRecGPS.AddBStart_u32 + sRecGPS.IndexSend_u16 * sRecGPS.SizeRecord_u16;
}


void AppMem_Save_Index_Rec (void)
{
    uint8_t aTEMP[24] = {0};
    
    aTEMP[0] = BYTE_TEMP_FIRST;
    aTEMP[1] = 12;

    aTEMP[2] = (sRecTSVH.IndexSend_u16 >> 8) & 0xFF;
    aTEMP[3] = sRecTSVH.IndexSend_u16 & 0xFF;
    
    aTEMP[4] = (sRecTSVH.IndexSave_u16 >> 8) & 0xFF;
    aTEMP[5] = sRecTSVH.IndexSave_u16 & 0xFF;
    
    aTEMP[6] = (sRecEvent.IndexSend_u16 >> 8) & 0xFF;
    aTEMP[7] = sRecEvent.IndexSend_u16 & 0xFF;
    
    aTEMP[8] = (sRecEvent.IndexSave_u16 >> 8) & 0xFF;
    aTEMP[9] = sRecEvent.IndexSave_u16 & 0xFF;
    
    aTEMP[10] = (sRecGPS.IndexSend_u16 >> 8) & 0xFF;
    aTEMP[11] = sRecGPS.IndexSend_u16 & 0xFF;
    
    aTEMP[12] = (sRecGPS.IndexSave_u16 >> 8) & 0xFF;
    aTEMP[13] = sRecGPS.IndexSave_u16 & 0xFF;
    
    aTEMP[14] = (sRecLog.IndexSend_u16 >> 8) & 0xFF;
    aTEMP[15] = sRecLog.IndexSend_u16 & 0xFF;
    
    aTEMP[16] = (sRecLog.IndexSave_u16 >> 8) & 0xFF;
    aTEMP[17] = sRecLog.IndexSave_u16 & 0xFF;

    Erase_Firmware(ADDR_INDEX_REC, 1);
    OnchipFlashWriteData(ADDR_INDEX_REC, &aTEMP[0], 24);
}
    
/*
    Func: tinh dia chi luu record theo loai ban tin
        - type: TSVH A,B  ; LOG ....
        - RW: dia chi cho viec doc hay la ghi
*/
uint32_t AppMem_Cacul_Addr_Mess (uint8_t type, uint8_t RW)
{
    uint32_t AddrGet_u32 = 0;
    
    switch (type)   
    {
        case _MEM_DATA_TSVH_A: 
            AddrGet_u32 = sRecTSVH.AddA_u32;
            break;
        case _MEM_DATA_TSVH_B: 
            if (RW == __MEM_READ)
                AddrGet_u32 = sRecTSVH.AddB_u32 - sRecTSVH.Offset_u32;
            else
                AddrGet_u32 = sRecTSVH.AddB_u32;
            break;
        case _MEM_DATA_EVENT_A:
            AddrGet_u32 = sRecEvent.AddA_u32;
            break; 
        case _MEM_DATA_EVENT_B:
            if (RW == __MEM_READ)
                AddrGet_u32 = sRecEvent.AddB_u32 - sRecEvent.Offset_u32;
            else
                AddrGet_u32 = sRecEvent.AddB_u32;
            break; 
        case _MEM_DATA_LOG:
            AddrGet_u32 = sRecLog.AddA_u32;
            break; 
        case _MEM_DATA_GPS_A:
            AddrGet_u32 = sRecGPS.AddA_u32;
            break; 
        case _MEM_DATA_GPS_B:
            if (RW == __MEM_READ)
                AddrGet_u32 = sRecGPS.AddB_u32 - sRecGPS.Offset_u32;
            else
                AddrGet_u32 = sRecGPS.AddB_u32;
            break; 
        case _MEM_DATA_OTHER:
            AddrGet_u32 = 0xFFFFFFFF;
            break; 
        default:
            break;
    }
    
    return AddrGet_u32;
}





/*
    Func: tinh dia chi luu record theo loai ban tin
        - addr: 
*/
uint8_t AppMem_Get_Type_From_Addr (uint32_t addr)
{
    uint8_t type = 0;
    
    if ( (sRecTSVH.AddAStart_u32 <= addr) && (addr < sRecTSVH.AddAStop_u32) )
    {
        type = _MEM_DATA_TSVH_A;
    } else if ( (sRecTSVH.AddBStart_u32 <= addr) && (addr < sRecTSVH.AddBStop_u32) )
    {
        type = _MEM_DATA_TSVH_B;
    } else if ( (sRecEvent.AddAStart_u32 <= addr) && (addr < sRecEvent.AddAStop_u32) )
    {
        type = _MEM_DATA_EVENT_A;
    } else if ( (sRecEvent.AddBStart_u32 <= addr) && (addr < sRecEvent.AddBStop_u32) )
    {
        type = _MEM_DATA_EVENT_B;
    } else if ( (sRecGPS.AddAStart_u32 <= addr) && (addr < sRecGPS.AddAStop_u32) )
    {
        type = _MEM_DATA_GPS_A;
    } else if ( (sRecGPS.AddBStart_u32 <= addr) && (addr < sRecGPS.AddBStop_u32) )
    {
        type = _MEM_DATA_GPS_B;
    } else if ( (sRecLog.AddAStart_u32 <= addr) && (addr < sRecLog.AddAStop_u32) )
    {
        type = _MEM_DATA_LOG;
    }
       
    return type;
}


void AppMem_Send_To_Queue_Read (sQueueMemReadManager *qRead)
{
    qQueue_Send(&qMemRead, (sQueueMemReadManager *) qRead, _TYPE_SEND_TO_END); 
}


void AppMem_Send_To_Queue_Write (sQueueMemWriteManager *qWrite)
{
    //check mem ok hay loi
    if (AppMem_Rec_Status() == true)
    {
        qQueue_Send(&qMemWrite, (sQueueMemWriteManager *) qWrite, _TYPE_SEND_TO_END); 
        
        fevent_active( sEventAppMem, _EVENT_MEM_CTRL_WRITE);
    } else
    {
        //forword to sim
        AppMem_Forward_Mess(AppMem_Cacul_Addr_Mess(qWrite->TypeData_u8, __MEM_WRITE), qWrite->aData, qWrite->Length_u16);
    }
}



uint8_t AppMem_Queue_Write_Empty (void)
{
    if (qGet_Number_Items(&qMemWrite) == 0)
    {
        return true;
    }
    
    return false;
}

uint8_t AppMem_Queue_Read_Empty (void)
{
    if (qGet_Number_Items(&qMemRead) == 0)
    {
        return true;
    }
    
    return false;
}


/*
    Func: check bo nho dung de luu rec:
*/
uint8_t AppMem_Rec_Status (void)
{
#ifdef MEM_REC_EX_EEPROM
    return sCAT24Var.Status_u8;
#endif
    
#ifdef MEM_REC_IN_FLASH
    return sOnFlash.Status_u8;
#endif

#ifdef MEM_REC_EX_FLASH
    return sS25FLvar.Status_u8;
#endif
}


uint8_t AppMem_Is_Write_To_PartAB (void)
{
#ifdef MEM_REC_EX_EEPROM
    return false;
#endif
    
#ifdef MEM_REC_IN_FLASH
    return false;
#endif
    
#ifdef MEM_REC_EX_FLASH
    return false;
#endif
}


#ifdef USING_AT_CONFIG
void AppMem_SER_rErase_Memory (sData *strRecei, uint16_t Pos)
{
    fevent_enable( sEventAppMem, _EVENT_MEM_ERASE);
    sAppMem.pRespond_Str(PortConfig, "PENDING...", 0);
}


void AppMem_SER_Get_Last_For_LOG (sData *strRecei, uint16_t Pos)
{        
    AppMem_Get_Req_AT(_MEM_DATA_LOG, (uint16_t) UTIL_Get_Num_From_Str(strRecei, &Pos));
}

void AppMem_SER_Get_Last_For_OPERA (sData *strRecei, uint16_t Pos)
{
    AppMem_Get_Req_AT(_MEM_DATA_TSVH_A, (uint16_t) UTIL_Get_Num_From_Str(strRecei, &Pos));
}

void AppMem_SER_Get_Last_For_EVENT (sData *strRecei, uint16_t Pos)
{
    AppMem_Get_Req_AT(_MEM_DATA_EVENT_A, (uint16_t) UTIL_Get_Num_From_Str(strRecei, &Pos));
}

void AppMem_SER_Get_Last_For_GPS (sData *strRecei, uint16_t Pos)
{
    AppMem_Get_Req_AT(_MEM_DATA_GPS_A, (uint16_t) UTIL_Get_Num_From_Str(strRecei, &Pos));
}

void AppMem_SER_Get_Index_Log (sData *strRecei, uint16_t Pos)
{    
    char aData[128] = {0};
    
    sprintf(aData, "tsvh: %d, %d, %d\r\n", sRecTSVH.IndexSend_u16, sRecTSVH.IndexSave_u16, sRecTSVH.MaxRecord_u16);
    sprintf(aData + strlen(aData), "event: %d, %d, %d\r\n", sRecEvent.IndexSend_u16, sRecEvent.IndexSave_u16, sRecEvent.MaxRecord_u16);
    sprintf(aData + strlen(aData), "log: %d, %d, %d\r\n", sRecLog.IndexSend_u16, sRecLog.IndexSave_u16, sRecLog.MaxRecord_u16);
    
    sAppMem.pRespond_Str(PortConfig, aData, 0);
}

/*
    Func: Set Infor index read parameter:
        + Read n rec
*/

uint8_t AppMem_Get_Req_AT (uint8_t type, uint16_t NumRec)
{   
    if ( ( NumRec != 0) && (AppMem_Rec_Status() == true) )
    {
        switch (type)
        {
            case _MEM_DATA_TSVH_A:
                sAppMem.sReqATcmd.pRecord = &sRecTSVH;
                break;
            case _MEM_DATA_EVENT_A:
                sAppMem.sReqATcmd.pRecord = &sRecEvent;
                break;
            case _MEM_DATA_LOG:
                sAppMem.sReqATcmd.pRecord = &sRecLog;
                break;
            case _MEM_DATA_GPS_A:
                sAppMem.sReqATcmd.pRecord = &sRecGPS;
                break;
            default:
                break;
        }
        
        if (NumRec < sAppMem.sReqATcmd.pRecord->MaxRecord_u16)
        {
            sAppMem.sReqATcmd.Landmark_u32 = RtCountSystick_u32;
            sAppMem.sReqATcmd.Status_u8 = true;
            sAppMem.sReqATcmd.Port_u8 = PortConfig;
            sAppMem.sReqATcmd.IndexStop_u16 = sAppMem.sReqATcmd.pRecord->IndexSave_u16;
                
            if (sAppMem.sReqATcmd.IndexStop_u16 >= NumRec)
            {
                sAppMem.sReqATcmd.IndexStart_u16 = sAppMem.sReqATcmd.IndexStop_u16 - NumRec;
            } else
            {
                sAppMem.sReqATcmd.IndexStart_u16 = (sAppMem.sReqATcmd.pRecord->MaxRecord_u16 \
                                                    + sAppMem.sReqATcmd.IndexStop_u16 - NumRec);
            }
            
            //active event check read resp at
            fevent_active( sEventAppMem, _EVENT_MEM_CHECK_RQ_AT);
            UTIL_Printf_Str( DBLEVEL_M, "\r\nOK\r\n" );
            
            return 1;
        }
    }
    
    UTIL_Printf_Str( DBLEVEL_M, "u_app_mem: get request at error!\r\n" );
    
    return 0;
}


#endif



uint8_t AppMem_Test_Write(void)
{
#ifdef MEM_REC_EX_EEPROM
    return true;
#endif
    
#ifdef MEM_REC_IN_FLASH
    return true;
#endif
    
#ifdef MEM_REC_EX_FLASH
    return eFlash_Test_Write();
#endif
}


              