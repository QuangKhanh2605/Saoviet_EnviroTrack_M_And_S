

#ifndef USER_OBIS_SV_H
#define USER_OBIS_SV_H

#include "user_util.h"


/*======== Define =================*/
#define PAYLOAD_HEX_FORMAT
//#define PAYLOAD_JSON_FORMAT

#define OBIS_TIME_DEVICE    0x01
#define OBIS_TIME_EMET      0x02
#define OBIS_MODE           0x03
#define OBIS_DEV_VOL1       0x04
#define OBIS_DEV_VOL2       0x05
#define OBIS_RSSI_1         0x06
#define OBIS_RSSI_2         0x07
#define OBIS_FREQ_SEND      0x08
#define OBIS_SERI_DEV       0x09
#define OBIS_SERI_SENSOR    0x0A
#define OBIS_SERI_SIM       0x0B
#define OBIS_SER_ADD        0x0C
#define OBIS_GPS_LOC        0x0D
#define OBIS_FW_VERSION     0x0F
#define OBIS_NET_STATUS     0x10
#define OBIS_TYPE           0x11

#define OBIS_DCU_TYPE               0x11
#define OBIS_INDEX_OBIS_TSVH        0x15

#define OBIS_EMET_VOL               0x20
#define OBIS_EMET_VOL_A             0x21
#define OBIS_EMET_VOL_B             0x22
#define OBIS_EMET_VOL_C             0x23

#define OBIS_EMET_CUR               0x24
#define OBIS_EMET_CUR_A             0x25
#define OBIS_EMET_CUR_B             0x26
#define OBIS_EMET_CUR_C             0x27

#define OBIS_EMET_FREQ              0x28

#define OBIS_EMET_FACTOR            0x29
#define OBIS_EMET_FACTOR_A          0x2A
#define OBIS_EMET_FACTOR_B          0x2B
#define OBIS_EMET_FACTOR_C          0x2C

#define OBIS_EMET_ACT_W             0x2D
#define OBIS_EMET_ACT_W_A           0x2E
#define OBIS_EMET_ACT_W_B           0x2F
#define OBIS_EMET_ACT_W_C           0x30

#define OBIS_EMET_ACT_Wh            0x31
#define OBIS_EMET_ACT_Wh_A          0x32
#define OBIS_EMET_ACT_Wh_B          0x33
#define OBIS_EMET_ACT_Wh_C          0x34


#define OBIS_WM_PULSE               0x60
#define OBIS_WM_FLOW                0x61
#define OBIS_WM_PRESSURE            0x62

#define OBIS_WM_LEVEL_UNIT          0x63
#define OBIS_WM_LEVEL_VAL_SENSOR    0x64
#define OBIS_WM_LEVEL_VAL_REAL      0x65

#define OBIS_NUM_CHANNEL_WM         0x67
#define OBIS_CHANNEL_WM_CONFIG      0x68

#define OBIS_WM_PULSE_FORWARD       0x69
#define OBIS_WM_PULSE_REVERSE       0x6A
#define OBIS_WM_DIG_FLOW            0x6B

#define OBIS_ENVI_TEMP_1            0x70
#define OBIS_ENVI_TEMP_2            0x71
#define OBIS_ENVI_HUMI_1            0x72
#define OBIS_ENVI_HUMI_2            0x6E

/*================ Obis Control Oxy ================*/
#define OBIS_ENVI_STATE_MACHINE     0xB4

#define OBIS_ENVI_OXY_TEMPERATURE   0x70
#define OBIS_ENVI_PH_WATER          0x73
#define OBIS_ENVI_OXY_MG_L          0x74
#define OBIS_ENVI_SALINITY          0x75    //(ppt)
#define OBIS_ENVI_EC                0x7F    //(uS/cm)
#define OBIS_ENVI_OXY_PERCENT       0x8B
#define OBIS_ENVI_SALINITY_UNIT     0x8C    //(%)
#define OBIS_ENVI_CLO_DU            0x8D
#define OBIS_ENVI_NTU               0x8E

#define OBIS_CONTACT_STATUS         0x90
#define OBIS_MODBUS_RTU             0xA0


#define OBIS_SETT_PULSE_FACTOR      0xC0
#define OBIS_SETT_PULSE_START       0xC1

#define OBIS_SETT_LI_IN_MIN         0xC4
#define OBIS_SETT_LI_IN_MAX         0xC5
#define OBIS_SETT_LI_IN_UNIT        0xC6

#define OBIS_SETT_LI_OUT_MIN        0xC7
#define OBIS_SETT_LI_OUT_MAX        0xC8
#define OBIS_SETT_LI_OUT_UNIT       0xC9

#define OBIS_SETT_PRESS_FACTOR      0xCA
#define OBIS_SETT_PRESS_TYPE        0xCB

#define OBIS_WARNING_SENSOR_CONNECT 0xD9
#define OBIS_WARNING_SENSOR_THREAD  0xD6


void SV_Protocol_Packet_Data (uint8_t *pTarget, uint16_t *LenTarget, uint8_t Obis, 
                                        void *pData, uint8_t LenData, uint8_t Scale);
void SV_Sub_Packet_Integer (uint8_t *pTarget, uint16_t *LenTarget, void *Data, uint8_t LenData);


#endif