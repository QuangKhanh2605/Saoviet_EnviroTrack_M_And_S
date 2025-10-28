

#include "user_app_sim2.h"
#include "user_define.h"

#include "user_string.h"
#include "user_modem.h"
#include "math.h"


/*================ static function =================*/  

static uint8_t _Cb_Event_Sim_GPS_Get (uint8_t event);
static uint8_t _Cb_Event_Sim_Control (uint8_t event);
static uint8_t _Cb_Event_Sim_Check_Active (uint8_t event);
static uint8_t _Cb_Event_Sim_GPS_Packet (uint8_t event);

/*================ Var struct =================*/  
sEvent_struct sEventAppSim2[] = 
{
    { _EVENT_SIM2_CONTROL,  		    0, 0, 10,       _Cb_Event_Sim_Control }, 
    { _EVENT_SIM2_GPS_GET,  		    0, 0, 5000,     _Cb_Event_Sim_GPS_Get },  
    { _EVENT_SIM2_GPS_PACK,  		    0, 0, 0,        _Cb_Event_Sim_GPS_Packet }, 
    
    { _EVENT_SIM2_CHECK_ACTIVE,  		1, 0, 10000,    _Cb_Event_Sim_Check_Active},  
};


static void AppSim2_Process_URC_Callback (uint8_t Type);
static void AppSim2_Cb_Power_On (void);

sFuncCallbackHandlerSim     sAppSim2CallBack = 
{
    .pSim_Common_Reset_MCU      = Reset_Chip,
    .pSim_Common_Power_On       = AppSim2_Cb_Power_On,
    .pSim_Common_Handler_AT     = NULL,
    .pSim_Common_Handler_URC    = AppSim2_Process_URC_Callback,
    .pSim_Common_Config_Update  = NULL,
    .pSim_Common_Get_Fw_Version = NULL,
};

uint8_t     aGPS_PAYLOAD[MAX_GPS_LENGTH_PAYLOAD];

sAppSim2Variable     sAppSim2Var =
{
    .sGPS.sPayload = {aGPS_PAYLOAD, 0},
};

/*================ Func =================*/

static uint8_t _Cb_Event_Sim_Control (uint8_t event)
{
    /*
        Control Sim: By status variable
    */
    switch (sSimCommon2.Step_Control_u8)
    {
        case _SIM_INIT:
            //enable event get GPS
            fevent_enable(sEventAppSim2, _EVENT_SIM2_GPS_GET);
            break;
        case _SIM_PSM:
            fevent_active(sEventAppSim2, _EVENT_SIM2_GPS_PACK);
            //active Power Off module sim
            fevent_active(sEventSim2, _EVENT_SIM2_POWER_OFF);
            //disable event get gps va check active
            fevent_disable(sEventAppSim2, _EVENT_SIM2_CHECK_ACTIVE);
            fevent_disable(sEventAppSim2, _EVENT_SIM2_GPS_GET);
            break;
        case _SIM_POWER_OFF:
            //active Event Power On
            fevent_active(sEventSim2, _EVENT_SIM2_TURN_ON);
            fevent_disable(sEventAppSim2, _EVENT_SIM2_CHECK_ACTIVE);
            fevent_disable(sEventAppSim2, _EVENT_SIM2_GPS_GET);
            break;
        case _SIM_HARD_RS:
            //active Event hard rs module
            fevent_active(sEventAppSim2, _EVENT_SIM2_GPS_PACK);
            fevent_active(sEventSim2, _EVENT_SIM2_HARD_RESET);
            fevent_disable(sEventAppSim2, _EVENT_SIM2_CHECK_ACTIVE);
            fevent_disable(sEventAppSim2, _EVENT_SIM2_GPS_GET);
            break;
        default:
            break;
    }
    
    return 1;
}




static uint8_t _Cb_Event_Sim_GPS_Get (uint8_t event)
{
    //Get GPS avarage 5s
    AppSim2_GPS_Analys_Data();
    //Neu sample >= MAX_SAMPLE -> active Packet
    if (sAppSim2Var.sGPS.Index5s_u8 >= MAX_SAMPLE)
    {
        fevent_active(sEventAppSim2, _EVENT_SIM2_GPS_PACK);
    }
    //Reset data gps sample
    AppSim2_Default_Struct_GPS();
        
    //Kiem tra mode hoat dong xem co can chuyen sang che do moi chua
    if (sSimVar2.ModePower_u8 == _POWER_MODE_SAVE)
    {
        sSimCommon2.Step_Control_u8 = _SIM_PSM;
        fevent_active(sEventAppSim2, _EVENT_SIM2_CONTROL);
    } else
    {
        fevent_enable(sEventAppSim2, event);
    }
    
    return 1;
}


static uint8_t _Cb_Event_Sim_GPS_Packet (uint8_t event)
{
    //Tinh toan tu cac mau du lieu gps: va dong goi luu ban tin vao bo nho
    UTIL_Printf_Str( DBLEVEL_M, "u_app_sim2: packet gps payload!\r\n" );
    AppSim2_GPS_Packet_Payload (AppSim2_GPS_Moving() ); 
    
    //Reset gia tri toa do 5s
    for (uint8_t i = 0; i < MAX_SAMPLE; i++)
    {
        sAppSim2Var.sGPS.aLocation5s[i].Lat = 0;
        sAppSim2Var.sGPS.aLocation5s[i].Long = 0;
    }
    
    sAppSim2Var.sGPS.Index5s_u8 = 0;
    
    return 1;
}

/*
    Func: cb event check module active
        + neu trong 10s k nhan duoc chuoi $GNRMC, truyen sang:-> Module can on lai
        + trong 10 phut k nhan dc data GPS -> Power ON lai

        + Note: sua lai tu power on lai -> thanh hard reset module
*/
static uint8_t _Cb_Event_Sim_Check_Active (uint8_t event)
{
    if (sSimCommon2.Step_Control_u8 == _SIM_INIT)
    {
        if ( (sAppSim2Var.sGPS.Status_u8 == false)
            || (Check_Time_Out(sAppSim2Var.sGPS.LandMarkDisConnect_u32, 600000) == true) )
        {
            sAppSim2Var.sGPS.LandMarkDisConnect_u32 = RtCountSystick_u32;
            //PowerOn lai module
            sSimCommon2.Step_Control_u8 = _SIM_HARD_RS;
            //Power on again
            fevent_active(sEventAppSim2, _EVENT_SIM2_CONTROL);
        }
        
        //Set lai status
        if (sAppSim2Var.sGPS.Status_u8 == true)
        {
            sAppSim2Var.sGPS.Status_u8 = pending;
            sSimVar2.CountSoftReset_u8 = 0;
        } else
        {
            sAppSim2Var.sGPS.Status_u8 = false;
            sSimVar2.CountSoftReset_u8++;
        }
        //Neu hard rs qua 30p -> Power ON lai
        if (sSimVar2.CountSoftReset_u8 >= 180)
        {
            //PowerOn lai module
            sSimCommon2.Step_Control_u8 = _SIM_POWER_OFF;
            //Power on again
            fevent_active(sEventAppSim2, _EVENT_SIM2_CONTROL);
            UTIL_Printf_Str( DBLEVEL_M, "u_app_sim2: need power on sim2!\r\n" );
        }
    }
    
    fevent_enable(sEventAppSim2, event); 
          
    return 1;
}

/*============== Function Handler ====================*/   
/*
    Func Init App SIM
*/

void AppSim2_Init (void)
{
    Sim2_Init();
    
    sSimComm2FuncCallBack = &sAppSim2CallBack;
    //Mode power of modem
    sSimVar2.ModePower_u8 = &sModem.ModeSimPower_u8; 
    
    sSimCommon2.Step_Control_u8 = _SIM_INIT;
    fevent_enable(sEventAppSim2, _EVENT_SIM2_CHECK_ACTIVE); 
    fevent_enable(sEventAppSim2, _EVENT_SIM2_GPS_GET); 
    
//    fevent_active(sEventAppSim2, _EVENT_SIM2_CONTROL);   // Ban dieu khien power on
}


 
uint8_t AppSim2_Task(void)
{
	uint8_t i = 0;
	uint8_t Result = 0;

	for (i = 0; i < _EVENT_END_SIM2; i++)
	{
		if (sEventAppSim2[i].e_status == 1)
		{
            if (i != _EVENT_SIM2_CHECK_ACTIVE)
                Result = 1;
            
			if ((sEventAppSim2[i].e_systick == 0) ||
					((HAL_GetTick() - sEventAppSim2[i].e_systick)  >=  sEventAppSim2[i].e_period))
			{
                sEventAppSim2[i].e_status = 0; 
				sEventAppSim2[i].e_systick = HAL_GetTick();
				sEventAppSim2[i].e_function_handler(i);
			}
		}
	}
    
    Result |= Sim2_Task();

	return Result;
}

/*
    Func: callback event power on sim
        + L506, L511, EC200U: Push step sim
        + N10F: PowerOn -> Control Sim
*/
static void AppSim2_Cb_Power_On (void)
{
    fevent_active(sEventAppSim2, _EVENT_SIM2_CONTROL);
    fevent_enable(sEventAppSim2, _EVENT_SIM2_CHECK_ACTIVE); 
}



static void AppSim2_Process_URC_Callback (uint8_t Type)
{
    switch (Type)
    {
        case _N10F_URC_GPS: 
            AppSim2_Process_GPS_AtResp(&sUartDebug);
            break;
        default:
            break;
    }
}


/*
    Func: Cat du lieu GPS
        +  $GNGGA,045406.000,2101.75436,N,10547.12246,E,1,10,1.4,106.7,M,-28.0,M,,*67
        +  $GNRMC,045406.000,A,2101.75436,N,10547.12246,E,1.36,162.77,191023,,,A,V*05
        Dùng dinh dang GNRMC de cat data
    
*/

uint8_t AppSim2_Process_GPS_AtResp(sData *strSim)
{
    uint16_t i = 0;
    double LatTemp = 0, LongTemp = 0;
    int PosFind = 0;
    uint8_t CountPh = 0;
    sData strCheck = {(uint8_t *) aN10FUrc[_N10F_URC_GPS].at_response, strlen(aN10FUrc[_N10F_URC_GPS].at_response)};
    uint8_t aTempData[20] = {0};
    uint8_t Length = 0;
    uint8_t SignLat = 1, SignLong = 1;
    
    //Cat du lieu ra
    PosFind = Find_String_V2(&strCheck, strSim);
    if (PosFind >= 0)
    {
        sAppSim2Var.sGPS.Status_u8 = true;
            
        for (i = PosFind; i < strSim->Length_u16; i++)
        {
            if (*(strSim->Data_a8 + i) == ',')
            {
                CountPh++;
            }
            
            if (CountPh == 3)
                break;
        }
        //bat dau lattude
        i++;
        while (*(strSim->Data_a8 + i) != ',')
        {
            aTempData[Length++] = *(strSim->Data_a8 + i);
            
            if ( (Length >= 20) || (i >= strSim->Length_u16) )
                return false;
            i++;
        }
        //
        if (Length == 0)
            return false;
        
        LatTemp = UTIL_Convert_String_To_Double(aTempData, Length);
        
        //vi tri tiep theo la N,
        if ( (LatTemp == 0xFFFFFFFF) ||
             (*(strSim->Data_a8 + i + 2) != ',') || 
             ( (*(strSim->Data_a8 + i + 1) != 'N') && (*(strSim->Data_a8 + i + 1) != 'S') ) )
        {
            return false;
        } else
        {
            if (*(strSim->Data_a8 + i + 1) == 'S')
            {
                SignLat = -1;
            }
        }
  
        i += 2;
        //Cat data long
        Length = 0;
        UTIL_MEM_set (aTempData, 0, sizeof(aTempData));
        
        i++;
        while (*(strSim->Data_a8 + i) != ',')
        {
            aTempData[Length++] = *(strSim->Data_a8 + i);
            
            if ( (Length >= 20) || (i >= strSim->Length_u16) )
                return false;
            i++;
        }
        //
        if (Length == 0)
            return false;
        
        LongTemp = UTIL_Convert_String_To_Double(aTempData, Length);
        
        //vi tri tiep theo la E,
        if ( (LongTemp == 0xFFFFFFFF) ||
             (*(strSim->Data_a8 + i + 2) != ',') || 
             ( (*(strSim->Data_a8 + i + 1) != 'E') && (*(strSim->Data_a8 + i + 1) != 'W') ) )
        {
            return false;
        } else
        {
            if (*(strSim->Data_a8 + i + 1) == 'W')
            {
                SignLong = -1;
            }
        }
        
        //Luu gia tri vao mang
        if (sAppSim2Var.sGPS.Index_u8 < TOTAL_GPS_ANALYS)
        {
            sAppSim2Var.sGPS.aLocation[sAppSim2Var.sGPS.Index_u8].Lat = UTIL_Convert_GNRMC_To_Location (SignLat * LatTemp);
            sAppSim2Var.sGPS.aLocation[sAppSim2Var.sGPS.Index_u8].Long = UTIL_Convert_GNRMC_To_Location (SignLong * LongTemp);
            sAppSim2Var.sGPS.Index_u8++;
        } else
        {
            UTIL_Printf_Str( DBLEVEL_M, "u_app_sim2: full array gps!\r\n" );
        }
    } else
    {
        return false;
    }
     
    
    return true;
}



void AppSim2_Default_Struct_GPS (void)
{
    sAppSim2Var.sGPS.Index_u8 = 0;
}

/*
    Func: Phan tich mau GPS: trong 5s
        + Neu k co data: get lat, long = 0,0
        + Neu co data sample: lay toa do tai diem gan nhat voi trung binh
*/
void AppSim2_GPS_Analys_Data (void)
{
    double Distance[TOTAL_GPS_ANALYS] = {0};
    uint16_t i = 0;
    double DistAverage = 0, TempDist = 0, MinDist = 0;
    uint8_t PosFind = 0;
    
    if (sAppSim2Var.sGPS.Index_u8 > 0)
    {
        //Tim gps gan avarage nhat
        for (i = 0; i < sAppSim2Var.sGPS.Index_u8 ; i++)
        {        
            Distance[i] = 1000 * UTIL_Cacul_Distance(sAppSim2Var.sGPS.aLocation[i].Lat, sAppSim2Var.sGPS.aLocation[i].Long, 
                                                     0, 0, 'K');
            
            DistAverage += Distance[i];
        }
        //Tinh trung binh
        DistAverage = DistAverage / (sAppSim2Var.sGPS.Index_u8);
        //
        MinDist = DistAverage;
        for (i = 0; i < sAppSim2Var.sGPS.Index_u8 ; i++)
        {
            //Tinh khoang cách voi gia tri trung binh
            if (DistAverage > Distance[i])
            {
                TempDist = DistAverage - Distance[i];
            } else
            {
                TempDist = Distance[i] - DistAverage;
            }
            
            //Check xem co phai gia tri nho nhat k?
            if (TempDist < MinDist)
            {
                MinDist = TempDist;
                PosFind = i;
            }
        }
        //Lay diem gps gan avarage 
        if (sAppSim2Var.sGPS.Index5s_u8 < MAX_SAMPLE)
        {
            sAppSim2Var.sGPS.aLocation5s[sAppSim2Var.sGPS.Index5s_u8].Lat = sAppSim2Var.sGPS.aLocation[PosFind].Lat;
            sAppSim2Var.sGPS.aLocation5s[sAppSim2Var.sGPS.Index5s_u8].Long = sAppSim2Var.sGPS.aLocation[PosFind].Long;
            
            if ( (sAppSim2Var.sGPS.sLocaOrigin.Lat == 0) && (sAppSim2Var.sGPS.sLocaOrigin.Long == 0) )
            {
                sAppSim2Var.sGPS.sLocaOrigin.Lat = sAppSim2Var.sGPS.aLocation5s[sAppSim2Var.sGPS.Index5s_u8].Lat;
                sAppSim2Var.sGPS.sLocaOrigin.Long = sAppSim2Var.sGPS.aLocation5s[sAppSim2Var.sGPS.Index5s_u8].Long;
            }
            
            sAppSim2Var.sGPS.Index5s_u8++;
        }
        
        sAppSim2Var.sGPS.DataOK_u8 = true;
        sAppSim2Var.sGPS.LandMarkDisConnect_u32 = RtCountSystick_u32;
    } else
    {
        if (sAppSim2Var.sGPS.DataOK_u8 == true)
        {
            sAppSim2Var.sGPS.aLocation5s[sAppSim2Var.sGPS.Index5s_u8].Lat = 0;
            sAppSim2Var.sGPS.aLocation5s[sAppSim2Var.sGPS.Index5s_u8].Long = 0;
            sAppSim2Var.sGPS.Index5s_u8++;
        }
        
        sAppSim2Var.sGPS.DataOK_u8 = false;
    }
}


/*
    Func: Check GPS dang di chuyen k
    return:
        + Tra ve 0xFF: neu dang di chuyen
        + Tra ve vi tri gan nhat: neu k di chuyen
*/
uint8_t AppSim2_GPS_Moving (void)
{
    double Distance[MAX_SAMPLE] = {0};
    uint16_t i = 0;
    uint8_t CountMore50m = 0;
    uint8_t CountPosError = 0;
//    double DistAverage = 0, TempDist = 0, MinDist = 0;
//    uint8_t PosFind = 0;
                                               
    //Lan luot tinh toan khoang cach
    for (i = 0; i < sAppSim2Var.sGPS.Index5s_u8 ; i++)
    {        
        if ( (sAppSim2Var.sGPS.aLocation5s[i].Lat == 0) && (sAppSim2Var.sGPS.aLocation5s[i].Long == 0) )
        {
            Distance[i] = -1;   //Nhung diem error danh dau gia tri dis = -1
            CountPosError++;
            continue;
        }
    
        Distance[i] = 1000 * UTIL_Cacul_Distance(sAppSim2Var.sGPS.aLocation5s[i].Lat, sAppSim2Var.sGPS.aLocation5s[i].Long, 
                                                 sAppSim2Var.sGPS.sLocaOrigin.Lat, sAppSim2Var.sGPS.sLocaOrigin.Long, 'K'); 
    }
    
    //so sanh
    for (i = 0; i < sAppSim2Var.sGPS.Index5s_u8; i++)
    {
        if (Distance[i] > DISTANCE_GPS_MOVE)
            CountMore50m++;
    }
        
    if ( CountMore50m > ( (sAppSim2Var.sGPS.Index5s_u8 - CountPosError) / 2 ) )   
    {
        //Neu qua nua vi tri > 50m: Xem dang di chuyen
        return true;  
    } 
//    else
//    {
//        //Neu chua dc 1 nua vi tri > 50m. Xe dang dung yen: 
//        // Tinh trung binh va tim vi tri gan nhat de dong goi ban tin
//        for (i = 0; i < sAppSim2Var.sGPS.Index5s_u8; i++)
//        {
//            if (Distance[i] == -1)
//                continue;
//            
//            DistAverage += Distance[i];
//        }
//        DistAverage = DistAverage / (sAppSim2Var.sGPS.Index5s_u8 - CountPosError);
//        
//        //Tim gia tri gan voi gia tri trung binh    
//        MinDist = DistAverage;
//        for (i = 0; i < sAppSim2Var.sGPS.Index5s_u8 ; i++)
//        {
//            if (Distance[i] == -1)
//            {
//                continue;
//            }
//            //Tinh khoang cách voi gia tri trung binh
//            if (DistAverage > Distance[i])
//            {
//                TempDist = DistAverage - Distance[i];
//            } else
//            {
//                TempDist = Distance[i] - DistAverage;
//            }
//            
//            //Check xem co phai gia tri nho nhat k?
//            if (TempDist < MinDist)
//            {
//                MinDist = TempDist;
//                PosFind = i;
//            }
//        }
//        
//        return PosFind;
//    }
    return false;
}


/*
    Func: Dong goi GPS Record to save
        + Type: moving: Packet all data
                Not moving: if 0,0 -> packet 0,0
                            else packet origin location
*/

void AppSim2_GPS_Packet_Payload (uint8_t Type)
{
    uint16_t  i = 0;
    
    /*Neu k co toa do nao : ket thuc*/
    if (sAppSim2Var.sGPS.Index5s_u8 == 0)
        return;
    
    Reset_Buff(&sAppSim2Var.sGPS.sPayload);
    //Them Stime vao truoc
    *(sAppSim2Var.sGPS.sPayload.Data_a8 + sAppSim2Var.sGPS.sPayload.Length_u16++) = OBIS_TIME_DEVICE;   // sTime
    *(sAppSim2Var.sGPS.sPayload.Data_a8 + sAppSim2Var.sGPS.sPayload.Length_u16++) = 0x06;
    *(sAppSim2Var.sGPS.sPayload.Data_a8 + sAppSim2Var.sGPS.sPayload.Length_u16++) = sRTC.year;
    *(sAppSim2Var.sGPS.sPayload.Data_a8 + sAppSim2Var.sGPS.sPayload.Length_u16++) = sRTC.month;
    *(sAppSim2Var.sGPS.sPayload.Data_a8 + sAppSim2Var.sGPS.sPayload.Length_u16++) = sRTC.date;
    *(sAppSim2Var.sGPS.sPayload.Data_a8 + sAppSim2Var.sGPS.sPayload.Length_u16++) = sRTC.hour;
    *(sAppSim2Var.sGPS.sPayload.Data_a8 + sAppSim2Var.sGPS.sPayload.Length_u16++) = sRTC.min;
    *(sAppSim2Var.sGPS.sPayload.Data_a8 + sAppSim2Var.sGPS.sPayload.Length_u16++) = sRTC.sec;
    
    if (Type == true)
    {        
        for (i = 0; i < sAppSim2Var.sGPS.Index5s_u8; i++)
        {
            //Ngan cach giua cac toa do la dau ;
            if (i != 0)
                *(sAppSim2Var.sGPS.sPayload.Data_a8 + sAppSim2Var.sGPS.sPayload.Length_u16++) = ';';
            //Data cua tung vi tri
            UTIL_Convert_Double_To_string(&sAppSim2Var.sGPS.sPayload, sAppSim2Var.sGPS.aLocation5s[i].Lat, 7);
            *(sAppSim2Var.sGPS.sPayload.Data_a8 + sAppSim2Var.sGPS.sPayload.Length_u16++) = ',';
            UTIL_Convert_Double_To_string(&sAppSim2Var.sGPS.sPayload, sAppSim2Var.sGPS.aLocation5s[i].Long, 7);
            
            if (sAppSim2Var.sGPS.sPayload.Length_u16 >= (MAX_GPS_LENGTH_PAYLOAD - 30) )
            {
                break;
            }
        }
    #ifdef USING_APP_MEM
        AppMem_Write_Data(_MEM_DATA_GPS_A, sAppSim2Var.sGPS.sPayload.Data_a8, sAppSim2Var.sGPS.sPayload.Length_u16, sRecGPS.SizeRecord_u16);
    #endif
        
        //Lay lai toa do goc: Toa do cuoi cung (neu di chuyen): tim vi tri toa do khac 0 tu cuoi
        for (i = sAppSim2Var.sGPS.Index5s_u8; i > 0; i--)
        {
            if ( (sAppSim2Var.sGPS.aLocation5s[i - 1].Lat != 0) || (sAppSim2Var.sGPS.aLocation5s[i - 1].Long != 0 ) )
                break;
        }
        
        sAppSim2Var.sGPS.sLocaOrigin.Lat = sAppSim2Var.sGPS.aLocation5s[i - 1].Lat;
        sAppSim2Var.sGPS.sLocaOrigin.Long = sAppSim2Var.sGPS.aLocation5s[i - 1].Long;
    } else
    {
        //Dong goi tat ca gps là toa do gan nhat
        for (i = 0; i < sAppSim2Var.sGPS.Index5s_u8; i++)
        {
            //Ngan cach giua cac toa do la dau ;
            if (i != 0)
                *(sAppSim2Var.sGPS.sPayload.Data_a8 + sAppSim2Var.sGPS.sPayload.Length_u16++) = ';';
            
            if ( (sAppSim2Var.sGPS.aLocation5s[i].Lat == 0) && (sAppSim2Var.sGPS.aLocation5s[i].Long == 0) )
            {
                //Data tai vi tri mat ket noi van giu nguyen
                *(sAppSim2Var.sGPS.sPayload.Data_a8 + sAppSim2Var.sGPS.sPayload.Length_u16++) = '0';
                *(sAppSim2Var.sGPS.sPayload.Data_a8 + sAppSim2Var.sGPS.sPayload.Length_u16++) = ',';
                *(sAppSim2Var.sGPS.sPayload.Data_a8 + sAppSim2Var.sGPS.sPayload.Length_u16++) = '0';
            } else
            {
                //Data cua tung vi tri
                UTIL_Convert_Double_To_string(&sAppSim2Var.sGPS.sPayload, sAppSim2Var.sGPS.sLocaOrigin.Lat, 7);
                *(sAppSim2Var.sGPS.sPayload.Data_a8 + sAppSim2Var.sGPS.sPayload.Length_u16++) = ',';
                UTIL_Convert_Double_To_string(&sAppSim2Var.sGPS.sPayload, sAppSim2Var.sGPS.sLocaOrigin.Long, 7);
            }
            
            if (sAppSim2Var.sGPS.sPayload.Length_u16 >= (MAX_GPS_LENGTH_PAYLOAD - 30) )
            {
                break;
            }
        }
        
    #ifdef USING_APP_MEM
        AppMem_Write_Data(_MEM_DATA_GPS_A, sAppSim2Var.sGPS.sPayload.Data_a8, sAppSim2Var.sGPS.sPayload.Length_u16, sRecGPS.SizeRecord_u16);
    #endif
    }    
}





