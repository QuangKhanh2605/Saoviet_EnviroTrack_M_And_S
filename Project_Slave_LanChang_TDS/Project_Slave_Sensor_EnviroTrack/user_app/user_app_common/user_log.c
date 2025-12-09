
#include "user_log.h"
#include "user_define.h"

#include "queue_p.h"


/*==================var struct======================*/


/*==================Function======================*/
/*
    Func: Luu vao flash noi
*/
uint8_t LOG_Save_Record (uint8_t *pData, uint16_t Length)
{
    uint8_t aTEMP[128] = {0};
    uint8_t i = 0, Count = 0, TempCrc = 0;

    if (Length >= 40) //9 byte: 64 - (3 + 15 + 1 + 1)- 4 = 40
        Length = 40;

    //Dong goi theo 1.ST.Log
#ifdef USING_APP_MEM
    sprintf((char *) aTEMP, "%d.%d/%d %d:%d:%d.", sRecLog.IndexSave_u16, sRTC.month, sRTC.date, 
                                                sRTC.hour, sRTC.min, sRTC.sec);
#else
    sprintf((char *) aTEMP, "%d/%d %d:%d:%d.", sRTC.month, sRTC.date, sRTC.hour, sRTC.min, sRTC.sec);
#endif
    
    Count = strlen((char *) aTEMP);

    for (i = 0; i < Length; i++)
        aTEMP[Count++] = *(pData + i);
    
    Count++;
	for (i = 0; i < (Count - 1); i++)
		TempCrc ^= aTEMP[i];

    aTEMP[Count-1] = TempCrc;
    
#ifdef USING_APP_MEM
    AppMem_Write_Data(sAppMem.RecMemType_u8, _MEM_DATA_LOG, 0,
                      &aTEMP[0], Count, sRecLog.SizeRecord_u16);
#endif
    
    return 1;
}











