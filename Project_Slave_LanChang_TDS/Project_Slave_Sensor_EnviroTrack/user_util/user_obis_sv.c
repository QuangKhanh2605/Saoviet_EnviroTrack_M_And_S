

#include "user_obis_sv.h"




/*============ Func =================*/




/*
    Func:            
*/
void SV_Protocol_Packet_Data (uint8_t *pTarget, uint16_t *LenTarget, uint8_t Obis, 
                                        void *pData, uint8_t LenData, uint8_t Scale)
{
    uint16_t Pos = *LenTarget, i= 0;
    ST_TIME_FORMAT *sRTCTemp;
    uint8_t *TempU8;
    
    if (LenData == 0)
        return;
          
    switch (Obis)
    {
        case OBIS_TIME_DEVICE:
            pTarget[Pos++] = Obis;
            pTarget[Pos++] = LenData;
    
            sRTCTemp = pData;

            pTarget[Pos++] = sRTCTemp->year;
            pTarget[Pos++] = sRTCTemp->month;
            pTarget[Pos++] = sRTCTemp->date;
            pTarget[Pos++] = sRTCTemp->hour;
            pTarget[Pos++] = sRTCTemp->min;
            pTarget[Pos++] = sRTCTemp->sec;
            break;
        case OBIS_ENVI_TEMP_1:
        case OBIS_ENVI_HUMI_1: 
        case OBIS_ENVI_TEMP_2:
        case OBIS_ENVI_HUMI_2:
        case OBIS_EMET_CUR:
        case OBIS_DEV_VOL1:
        case OBIS_DEV_VOL2:
        case OBIS_RSSI_1:
        
        case OBIS_FREQ_SEND:
        case OBIS_WM_PULSE:
        case OBIS_WM_FLOW:
        case OBIS_WM_PRESSURE: 
        case OBIS_WM_LEVEL_UNIT:
        case OBIS_WM_LEVEL_VAL_SENSOR: 
        case OBIS_WM_LEVEL_VAL_REAL:
        case OBIS_CONTACT_STATUS:
        case OBIS_NUM_CHANNEL_WM:
        case OBIS_TYPE: 
        
        case OBIS_WM_DIG_FLOW:
        case OBIS_WM_PULSE_FORWARD:
        case OBIS_WM_PULSE_REVERSE:
           
        case OBIS_SETT_PULSE_FACTOR:
        case OBIS_SETT_PULSE_START:
        case OBIS_SETT_LI_IN_MIN: 
        case OBIS_SETT_LI_IN_MAX: 
        case OBIS_SETT_LI_IN_UNIT: 
        case OBIS_SETT_LI_OUT_MIN: 
        case OBIS_SETT_LI_OUT_MAX: 
        case OBIS_SETT_LI_OUT_UNIT: 
        case OBIS_SETT_PRESS_FACTOR:  
        case OBIS_SETT_PRESS_TYPE:   
            pTarget[Pos++] = Obis;
            pTarget[Pos++] = LenData;
            
            SV_Sub_Packet_Integer(pTarget, &Pos, pData, LenData);
            //
            if (Scale != 0xAA)
                pTarget[Pos++] = Scale;
            
            break; 
        case OBIS_GPS_LOC:
        case OBIS_SERI_SENSOR: 
        case OBIS_SER_ADD:
        case OBIS_SERI_SIM: 
        case OBIS_FW_VERSION:
        case OBIS_NET_STATUS:
        case OBIS_MODBUS_RTU:
            pTarget[Pos++] = Obis;
            pTarget[Pos++] = LenData;
            
            TempU8 = (uint8_t *) pData;
            for (i = 0; i < LenData; i++)
                pTarget[Pos++] = *(TempU8 + i);
            break; 
        default:
            break; 
    }
    
    *LenTarget = Pos ;
}


void SV_Sub_Packet_Integer (uint8_t *pTarget, uint16_t *LenTarget, void *Data, uint8_t LenData)
{
    uint16_t Pos = *LenTarget;
    
    uint8_t  *TempU8;
    uint16_t *TempU16;
    uint32_t *TempU32;
    uint64_t *TempU64;
    
    switch (LenData)
    {
        case 1:    
            TempU8 = (uint8_t *) Data;
            pTarget[Pos++] = *TempU8;
            break;
        case 2:
            TempU16 = (uint16_t *) Data;
            pTarget[Pos++] = ((*TempU16) >> 8) & 0xFF;
            pTarget[Pos++] =  (*TempU16) & 0xFF;
            break;
        case 4:
            TempU32 = (uint32_t *) Data;
            pTarget[Pos++] = ((*TempU32) >> 24) & 0xFF;
            pTarget[Pos++] = ((*TempU32) >> 16) & 0xFF;
            pTarget[Pos++] = ((*TempU32) >> 8) & 0xFF;
            pTarget[Pos++] =  (*TempU32) & 0xFF;
            break;
        case 8:
            TempU64 = (uint64_t *) Data;
            pTarget[Pos++] = ((*TempU64) >> 56) & 0xFF;
            pTarget[Pos++] = ((*TempU64) >> 48) & 0xFF;
            pTarget[Pos++] = ((*TempU64) >> 40) & 0xFF;
            pTarget[Pos++] = ((*TempU64) >> 32) & 0xFF;
            pTarget[Pos++] = ((*TempU64) >> 24) & 0xFF;
            pTarget[Pos++] = ((*TempU64) >> 16) & 0xFF;
            pTarget[Pos++] = ((*TempU64) >> 8) & 0xFF;
            pTarget[Pos++] =  (*TempU64) & 0xFF;
            break;
        default:
            break;
    }
    
    *LenTarget = Pos ;
}
