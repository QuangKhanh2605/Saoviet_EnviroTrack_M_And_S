

#include "user_sim_common.h"

#include "user_string.h"
#include "user_time.h"

/*=========== Var =====================*/
sFuncCallbackSim     sCbSimComm;

/*============== Struct var ==========*/

static uint8_t aSIM_ID[SIM_ID_LENGTH] = {"123456789"};
static uint8_t aSERIAL_MODULESIM[SIM_ID_LENGTH];

sSimInformation	sSimCommInfor = 
{
    .sSIMid = {&aSIM_ID[0], 0}, 
    .RSSI_u8 = 0,
    .Ber_u8  = 0,
    .sImei   = {&aSERIAL_MODULESIM[0], 0},
};


sSimVariable	    sSimCommVar;

sSimUpdateVariable    sSimFwUpdate;

sSimFileVariable         sFileSys = 
{
    .aNAME = {"/SVWM_Firmware.bin"},
    .LengthName_u8 = 18,
    
    .strHandle = {&sFileSys.aTEMP_STR_HANDLE[0], 0},
    .Handle_u32 = 1,
};


/*=========== Function =====================*/

/*
    Func: Get Serial SIM card 
*/
uint8_t Sim_Common_Get_Sim_ID (sData *strRecv)
{
	uint16_t 		Pos,var;
	uint8_t			aRes_Data[SIM_ID_LENGTH] = {0};
    sData	        sRes = {&aRes_Data[0], 0};
    uint8_t         MarkCompare = 0;    
    
	for (Pos = 1; Pos < strRecv->Length_u16; ++Pos) 
		if ((strRecv->Data_a8[Pos-1] == '8') && (strRecv->Data_a8[Pos] == '9')) 
            break;
    
	for (var = (Pos-1); var < strRecv->Length_u16; var++) 
		if ((strRecv->Data_a8[var] < '0') || (strRecv->Data_a8[var] > '9')) 
            break;
    
	sRes.Data_a8 = &strRecv->Data_a8[Pos-1];
	sRes.Length_u16 = var - Pos + 1;
    
	if (sRes.Length_u16 > SIM_ID_LENGTH) 
        sRes.Length_u16 = SIM_ID_LENGTH;
    
	if (sRes.Length_u16 != sSimCommInfor.sSIMid.Length_u16)
    { 
        MarkCompare = 1;
	} else
    {
        //Compare new Sim to Mark Send Sim ID
        for (var = 0; var < sRes.Length_u16; ++var) 
        {
            if (sSimCommInfor.sSIMid.Data_a8[var] != sRes.Data_a8[var])
            {
                MarkCompare = 1;   //Co su khac nhau
                break;
            }
        } 
    }
    //If Mark == 1: Is new sim ID
    if (MarkCompare == 1)
    {
        for (var = 0; var < sRes.Length_u16; ++var) 
            sSimCommInfor.sSIMid.Data_a8[var] = sRes.Data_a8[var];

        sSimCommInfor.sSIMid.Length_u16 = sRes.Length_u16;
        //Call Func Handler from SIM
        
        if (sCbSimComm.pHandle_AT != NULL)
            sCbSimComm.pHandle_AT (_SIM_COMM_SIM_CARD);
    }
    
    return 1;
}

/*
    Func: Get CSQ
        + Convert to Dec (RSSI)
        + Convert to %
*/

uint8_t Sim_Common_Get_CSQ (sData *strRecv)
{
	uint16_t Pos_Str = 0;
       
    sSimCommInfor.RSSI_u8 = *(strRecv->Data_a8 + Pos_Str) - 0x30;
    sSimCommInfor.Ber_u8  = *(strRecv->Data_a8 + Pos_Str + 2) - 0x30;  
    
    if (*(strRecv->Data_a8 + Pos_Str + 2) == 0x2C) 
    {
        sSimCommInfor.RSSI_u8 = ((*(strRecv->Data_a8 + Pos_Str))<<4) + \
                                                         ((*(strRecv->Data_a8 + Pos_Str + 1)) & 0x0F);
        sSimCommInfor.Ber_u8 = *(strRecv->Data_a8 + Pos_Str + 3) - 0x30;           // ber 0->7
    }
	/* convert signal strength */
	switch (sSimCommInfor.RSSI_u8) 
    {
		case 0x00:
			sSimCommInfor.RSSI_u8 = 113;
			break;
		case 0x01:
			sSimCommInfor.RSSI_u8 = 111;	
			break;
		case 0x31:
			sSimCommInfor.RSSI_u8 = 51;
			break;
        case 0x99:
			sSimCommInfor.RSSI_u8 = 0;
			break;
		default: //2 den 30/ 2 per step: 0x19 ->19  sau do - 2 = 17
			sSimCommInfor.RSSI_u8 = 113 - ( ( (sSimCommInfor.RSSI_u8) / 16 ) * 10 + sSimCommInfor.RSSI_u8 % 16 ) * 2;
			break;
	}
    
    return 1;
}


/*
    Func: Get Imei Module SIM
*/

uint8_t Sim_Common_Get_Imei(sData *strRecv)
{
    uint16_t i = 0;
    
    for (i = 0; i < strRecv->Length_u16; i++)
        if ((*(strRecv->Data_a8 + i) >= 0x30) && (*(strRecv->Data_a8 + i) <= 0x39)) 
            break;
    
    if (i < strRecv->Length_u16)
    {
        Reset_Buff(&sSimCommInfor.sImei);
        while ((*(strRecv->Data_a8 + i) >= 0x30) && (*(strRecv->Data_a8 + i) <= 0x39)) 
        {
            *(sSimCommInfor.sImei.Data_a8 + sSimCommInfor.sImei.Length_u16++) = *(strRecv->Data_a8 + i);
            i++;
            if (sSimCommInfor.sImei.Length_u16 >= SIM_ID_LENGTH) 
                break;
        }
    }
    
    return 1;
}


/*
    Func: Get sTime from BTS
*/

uint8_t Sim_Common_Get_CCLK (sData *strRecv)
{
	int             Pos_Fix = 0;
	uint16_t        Temp_Get = 0;
    uint16_t        Temp_year = 0;
    uint8_t         count = 0;
    uint8_t         aRTCtemp[8] = {0};
    static  uint8_t Received_u8 = false;
    
    if ( (strRecv->Length_u16 > (Pos_Fix + 14))
        && (Received_u8 == false) )
	{
        Pos_Fix += 1;   //dau khoang trong va nhay ++ de tro vao year
        
        //tim vi tri bat dau so decima
        Temp_year = *(strRecv->Data_a8 + Pos_Fix);
        while ((0x30 > Temp_year) || (Temp_year > 0x39))
        {
            Pos_Fix++;
            Temp_year = *(strRecv->Data_a8 + Pos_Fix);
        }
        
        //bat dau vi tri co so: year month date hour min sec
        while (count < 6)
        {
            Temp_Get  = 0;
            Temp_year = *(strRecv->Data_a8 + Pos_Fix);
            while ((0x30 <=  Temp_year) && (Temp_year <= 0x39))
            {
                Temp_Get = Temp_Get* 10 + Temp_year - 0x30;
                Pos_Fix++;
                Temp_year = *(strRecv->Data_a8 + Pos_Fix);
            }
                    
            aRTCtemp[count++] = Temp_Get % 100;               //nam chi lay 2 chu so sau
            
            if (Pos_Fix >= strRecv->Length_u16) 
                return 1;
            
            Pos_Fix++; // dau /
        }
        
        sSimCommInfor.sTime.year   = aRTCtemp[0] % 100;
        sSimCommInfor.sTime.month  = aRTCtemp[1];
        sSimCommInfor.sTime.date   = aRTCtemp[2];
        sSimCommInfor.sTime.hour   = aRTCtemp[3];  
        sSimCommInfor.sTime.min    = aRTCtemp[4];
        sSimCommInfor.sTime.sec    = aRTCtemp[5];
        
        if (Check_update_Time (&sSimCommInfor.sTime) == 0)
            return 1;

        //Convert lai day. 1/1/2012 la chu nhat. Thu 2 - cn: 2-8
        sSimCommInfor.sTime.day = ((HW_RTC_GetCalendarValue_Second (sSimCommInfor.sTime, 1) / SECONDS_IN_1DAY) + 6) % 7 + 1;
       
        if (sCbSimComm.pHandle_AT != NULL)
            sCbSimComm.pHandle_AT(_SIM_COMM_GET_STIME);
        
        Received_u8 = true;
    }
    
    return 1;
}




uint8_t Sim_Common_CPIN_Ready (sData *strRecv)
{
	sSimCommVar.CallReady_u8     = TRUE;
    sSimCommVar.State_u8   = _SIM_NETWORK;
    
    return 1;
}


uint8_t Sim_Common_Tranparent(sData *strRecv)
{
    sSimCommVar.DataMode_u8      = 1;

    return 1;
}


uint8_t Sim_Common_Non_Tranparent(sData *strRecv)
{
    sSimCommVar.DataMode_u8      = 0;
    
    return 1;
}


uint8_t Sim_Common_Check_Attached (sData *strRecv)
{
	char *p = NULL;

	p = strstr((char*) strRecv->Data_a8, "+CGATT: 1");
	if (p != NULL)
	{
		sSimCommVar.State_u8  = _SIM_SETTING;
        sSimCommVar.NetReady_u8      = TRUE;
        return 1;
	}

	return 0;
}



uint8_t Sim_Common_SMS_Send_1 (sData *uart_string)
{
    if (sCbSimComm.pHandle_AT != NULL)
        sCbSimComm.pHandle_AT (_SIM_COMM_SMS_SEND_1);   
    
    return 1;
}

uint8_t Sim_Common_SMS_Send_2 (sData *uart_string)
{
    if (sCbSimComm.pHandle_AT != NULL)
        sCbSimComm.pHandle_AT (_SIM_COMM_SMS_SEND_2);  
    
    return 1;
}

uint8_t Sim_Common_SMS_Send_3 (sData *uart_string)
{
    if (sCbSimComm.pHandle_AT != NULL)
        sCbSimComm.pHandle_AT (_SIM_COMM_SMS_SEND_3);  
    
    return 1;
}

uint8_t Sim_Common_Call_1 (sData *uart_string)
{
    if (sCbSimComm.pHandle_AT != NULL)
        sCbSimComm.pHandle_AT (_SIM_COMM_CALLING_1);    
    
    return 1;
}

uint8_t Sim_Common_Call_2 (sData *uart_string)
{
    if (sCbSimComm.pHandle_AT != NULL)
        sCbSimComm.pHandle_AT (_SIM_COMM_CALLING_2);  
    
    return 1;
}


// tcp mqtt
uint8_t Sim_Common_TCP_Close(sData *strRecv)
{
    uint8_t aCID = CID_SERVER;
    
    Sim_Common_Send_AT_Cmd( &aCID, 1 );
    
    return 1;
}

uint8_t Sim_Common_TCP_Open_1 (sData *strRecv)
{
    uint8_t aTEMP_STR[10] = {"0,\"TCP\",\""};

    aTEMP_STR[0] = CID_SERVER;
    
    Sim_Common_Send_AT_Cmd( &aTEMP_STR[0], 9 );
    Sim_Common_Send_AT_Cmd( sSimCommInfor.sServer.sIP->Data_a8, sSimCommInfor.sServer.sIP->Length_u16 );
    Sim_Common_Send_AT_Cmd( (uint8_t *) "\",", 2 );
    Sim_Common_Send_AT_Cmd( sSimCommInfor.sServer.sPort->Data_a8, sSimCommInfor.sServer.sPort->Length_u16 );
      
    sSimCommVar.State_u8        = _SIM_INIT_TCP;
    
    return 1;
}


uint8_t Sim_Common_Ping (sData *strRecv)
{
    if (sCbSimComm.pHandle_AT != NULL)
        sCbSimComm.pHandle_AT (_SIM_COMM_TCP_SEND_1);
    
    return 1;
}

uint8_t Sim_Common_TCP_Send_1 (sData *strRecv)
{
    if (sCbSimComm.pHandle_AT != NULL)
        sCbSimComm.pHandle_AT (_SIM_COMM_TCP_SEND_1);
    
    return 1;
}

uint8_t Sim_Common_TCP_Send_2 (sData *strRecv)
{
    if (sCbSimComm.pHandle_AT != NULL)
        sCbSimComm.pHandle_AT (_SIM_COMM_TCP_SEND_2);
    
    return 1;
}



uint8_t Sim_Common_MQTT_Conn_1 (sData *strRecv)
{
    if (sCbSimComm.pHandle_AT != NULL)
        sCbSimComm.pHandle_AT (_SIM_COMM_MQTT_CONN_1);
    
    return 1;
}

uint8_t Sim_Common_MQTT_Conn_2 (sData *strRecv)
{
//    if (sCbSimComm.pHandle_AT != NULL)
//        sCbSimComm.pHandle_AT (_SIM_COMM_MQTT_CONN_2);
    
    return 1;
}

uint8_t Sim_Common_MQTT_Sub_1 (sData *strRecv)
{
    if (sCbSimComm.pHandle_AT != NULL)
        sCbSimComm.pHandle_AT (_SIM_COMM_MQTT_SUB_1);
    
    return 1;
}

uint8_t Sim_Common_MQTT_Sub_2 (sData *strRecv)
{
    sSimCommVar.State_u8 = _SIM_CONN_MQTT;
    
    if (sCbSimComm.pHandle_AT != NULL)
        sCbSimComm.pHandle_AT (_SIM_COMM_MQTT_SUB_2);
    
    return 1;
}


uint8_t Sim_Common_MQTT_Pub_1 (sData *strRecv)
{
    if (sCbSimComm.pHandle_AT != NULL)
        sCbSimComm.pHandle_AT (_SIM_COMM_MQTT_PUB_1);
    
    return 1;
}

uint8_t Sim_Common_MQTT_Pub_2 (sData *strRecv)
{
    sSimCommVar.State_u8 = _SIM_CONN_MQTT;
    
    if (sCbSimComm.pHandle_AT != NULL)
        sCbSimComm.pHandle_AT (_SIM_COMM_MQTT_PUB_2);
    
    return 1;
}


/*==== htt[ func cb*/
uint8_t Sim_Common_HTTP_Head_1 (sData *strRecv)
{
    if (sCbSimComm.pHandle_AT != NULL)
        sCbSimComm.pHandle_AT (_SIM_COMM_HTTP_HEAD_1);
    
    return 1;
}

uint8_t Sim_Common_HTTP_Head_2 (sData *strRecv)
{    
    if (sCbSimComm.pHandle_AT != NULL)
        sCbSimComm.pHandle_AT (_SIM_COMM_HTTP_HEAD_2);
    
    return 1;
}

uint8_t Sim_Common_HTTP_Send_1 (sData *strRecv)
{
    if (sCbSimComm.pHandle_AT != NULL)
        sCbSimComm.pHandle_AT (_SIM_COMM_HTTP_SEND_1);
    
    return 1;
}

uint8_t Sim_Common_HTTP_Send_2 (sData *strRecv)
{
    if (sCbSimComm.pHandle_AT != NULL)
        sCbSimComm.pHandle_AT (_SIM_COMM_HTTP_SEND_2);
    
    return 1;
}


uint8_t Sim_Common_HTTP_Send_Ex (sData *strRecv)
{   
    sCbSimComm.pHandle_AT (_SIM_COMM_HTTP_SEND_EX);
    
    return 1;
}


/*================================ Func HTTP ===========================*/

uint8_t Sim_Common_Http_Read_Data (sData *uart_string)
{
    int             Pos = 0;
    sData           strHeader      = {(uint8_t*)"$HTTPREAD:", 10};
    sData           strStartHttp   = {(uint8_t*)"\r\n", 2};
    sData           strHeaderFw;
    
    sData           strFix;
    uint16_t        i = 0, j = 0;
    uint8_t         aNumbyte[10] = {0};
    uint8_t         lengthnum = 0;
    uint16_t        NumbyteRecei = 0;
    uint8_t         aTEMP_DATA[8] = {0};
                   
    Pos = Find_String_V2((sData*) &strHeader, uart_string);
	if (Pos >= 0)
	{
        //Get Number recv string
        for (i = Pos; i < uart_string->Length_u16; i++)
        {
            if (*(uart_string->Data_a8 + i) == ',')
            {
                i++;
                while ((*(uart_string->Data_a8 + i) >= 0x30) && (*(uart_string->Data_a8 + i) <= 0x39))
                {
                    aNumbyte[lengthnum++] = *(uart_string->Data_a8 + i) - 0x30;
                    i++;
                }
                
                break;
            }
        }
        //convert ra so byte nhan dc
        for (i = 0; i < lengthnum; i++)
             NumbyteRecei = NumbyteRecei *10 + aNumbyte[i];
        
        if (NumbyteRecei == 0) 
        {
            sSimFwUpdate.FailStatus_u8 = SEND_UPDATE_FIRMWARE_FAIL_LENGTH;
            sCbSimComm.pHandle_AT ( _SIM_COMM_HTTP_UPDATE_FAIL ); 
            return 0;
        }
        
        strFix.Data_a8      = uart_string->Data_a8 + Pos;
        strFix.Length_u16   = uart_string->Length_u16 - Pos;
        
        Pos = Find_String_V2((sData*)&strStartHttp, &strFix);  //check vi tri bat dau cua toan bo data http
        if (Pos >= 0)
        {   
            Pos  += strStartHttp.Length_u16;
            //
            strFix.Data_a8 = strFix.Data_a8 + Pos;
            if (NumbyteRecei > (strFix.Length_u16 - Pos - 6))
            {
                sSimFwUpdate.FailStatus_u8 = SEND_UPDATE_FIRMWARE_FAIL_LENGTH;
                sCbSimComm.pHandle_AT ( _SIM_COMM_HTTP_UPDATE_FAIL ); 
                return 0;
            }
            strFix.Length_u16 = NumbyteRecei;
            //
            sSimFwUpdate.CountByteTotal_u32 += NumbyteRecei;                          // tong so data ma ban tin nhan doc ra cua http
            sSimFwUpdate.AddOffset_u32 = sSimFwUpdate.CountByteTotal_u32;
            //Neu la Pack dau tien, Check 32 byte header xem co chu: ModemFirmware:SV1.1,0x0800C000
            if (sSimFwUpdate.CountPacket_u8 == 0)
            {
                strHeaderFw.Data_a8 = sCbSimComm.pGet_Fw_Version()->Data_a8;
                strHeaderFw.Length_u16 = LENGTH_HEADER_FW;
        
                Pos = Find_String_V2((sData*)&strHeaderFw, &strFix);  
                if (Pos >= 0)
                    sSimFwUpdate.IsFirmSaoViet_u8 = 1;
                //Lay byte cuoi cung ra: byte 32 cua Packdau tien. Sau header 31 byte
                sSimFwUpdate.LastCrcFile_u8 = *(strFix.Data_a8 + 31);  
                //Check Get Fw Sao Viet
                if (sSimFwUpdate.IsFirmSaoViet_u8 == 0)    //neu khong co header SV code thi return luon
                {
                    sSimFwUpdate.FailStatus_u8 = SEND_UPDATE_FIRMWARE_FAIL_FLASH;
                    sCbSimComm.pHandle_AT ( _SIM_COMM_HTTP_UPDATE_FAIL ); 
                    return 0;
                }
                //Fix string data
                strFix.Data_a8 += 32;
                strFix.Length_u16 -= 32;
            }
        
            //Cacal Crc
            for (i = 0; i < strFix.Length_u16; i++)
                sSimFwUpdate.LasCrcCal_u8 += *(strFix.Data_a8 + i);
                      
            //Increase Count Packet
            sSimFwUpdate.CountPacket_u8++;
            
            //Ghi data vao. Bu bao cho thanh boi cua FLASH_BYTE_WRTIE
            if ((strFix.Length_u16 % FLASH_BYTE_WRTIE) != 0)
                strFix.Length_u16 += ( FLASH_BYTE_WRTIE - (strFix.Length_u16 % FLASH_BYTE_WRTIE) );
            //write fw to Mem: Onchip flash or External mem
            for (i = 0; i < (strFix.Length_u16 / FLASH_BYTE_WRTIE); i++)
            {
                //Phai chia ra buffer rieng moi k loi
                for (j = 0; j < FLASH_BYTE_WRTIE; j++)
                    aTEMP_DATA[j] = *(strFix.Data_a8 + i * FLASH_BYTE_WRTIE + j); 
                //
                if(OnchipFlashWriteData(sSimFwUpdate.AddSave_u32, &aTEMP_DATA[0], FLASH_BYTE_WRTIE) != HAL_OK)
                {
                    sSimFwUpdate.FailStatus_u8 = SEND_UPDATE_FIRMWARE_FAIL_FLASH;
                    sCbSimComm.pHandle_AT ( _SIM_COMM_HTTP_UPDATE_FAIL ); 
                    return 0;
                } else
                    sSimFwUpdate.AddSave_u32 += FLASH_BYTE_WRTIE;
            }          
                 
            //kiem tra xem nhan du byte chua de bao ok
            if (sSimFwUpdate.CountByteTotal_u32 >= sSimFwUpdate.FirmLength_u32)  //Binh thuong la dau == //Test de >=           
            {
                if (sSimFwUpdate.LastCrcFile_u8 == sSimFwUpdate.LasCrcCal_u8)
                {
                    sSimFwUpdate.CountByteTotal_u32 += (FLASH_BYTE_WRTIE - sSimFwUpdate.CountByteTotal_u32 % FLASH_BYTE_WRTIE);
                    
                    sCbSimComm.pHandle_AT ( _SIM_COMM_HTTP_UPDATE_OK );
                    return 1;
                } else
                {
                    sSimFwUpdate.FailStatus_u8 = SEND_UPDATE_FIRMWARE_FAIL_LENGTH;
                    sCbSimComm.pHandle_AT ( _SIM_COMM_HTTP_UPDATE_FAIL ); 
                    return 0;
                }
            }
            
            //Continue
            sCbSimComm.pHandle_AT ( _SIM_COMM_HTTP_READ_OK );
            return 1;
        }
    }
    //Set Fail Setup Http
    sSimFwUpdate.FailStatus_u8 = SEND_UPDATE_FIRMWARE_FAIL_SETUP;
    sCbSimComm.pHandle_AT ( _SIM_COMM_HTTP_UPDATE_FAIL ); 
    
    return 0;
}




/*================================ Func URC ===========================*/
/*
    Func: Call Handler URC Reset SIM
*/
uint8_t Sim_Common_URC_Reset (sData *strRecv)
{
    sCbSimComm.phandle_URC(_SIM_COMM_URC_RESET_SIM);
    
    return 1;
}

/*
    Func: Call Handler URC LOST SIM
*/
uint8_t Sim_Common_URC_Lost_Card (sData *strRecv)
{
    sCbSimComm.phandle_URC(_SIM_COMM_URC_SIM_LOST);
    
    return 1;
}

/*
    Func: Call Handler URC LOST SIM
*/
uint8_t Sim_Common_URC_Closed (sData *strRecv)
{ 
    sCbSimComm.phandle_URC(_SIM_COMM_URC_CLOSED);  
    
    return 1;
}

uint8_t Sim_Common_URC_Call_Ready (sData *strRecv)
{
    sSimCommVar.CallReady_u8 = TRUE;
    
    return 1;
}

/*
    Func: Call Handler URC Error
*/
uint8_t Sim_Common_URC_Error (sData *strRecv)
{
    sCbSimComm.phandle_URC(_SIM_COMM_URC_ERROR);
    
    return 1;
}


/*
    Func: Call Handler URC Receive server
*/
uint8_t Sim_Common_URC_Recv_Server (sData *strRecv)
{
    sCbSimComm.phandle_URC(_SIM_COMM_URC_RECEIV_SERVER);
    
    return 1;
}

/*
    Func: Call Handler URC calling
*/
uint8_t Sim_Common_URC_Calling (sData *strRecv)
{
    sCbSimComm.phandle_URC(_SIM_COMM_URC_CALLING); 
    
    return 1;
}

uint8_t Sim_Common_URC_Call_Fail (sData *strRecv)
{    
    sCbSimComm.phandle_URC(_SIM_COMM_URC_CALL_FAIL); 
    
    return 1;
}


uint8_t Sim_Common_URC_Sms_Content (sData *strRecv)
{
    sCbSimComm.phandle_URC(_SIM_COMM_URC_SMS_CONTENT);
    
    return 1;
}

uint8_t Sim_Common_URC_Sms_Indi (sData *strRecv)
{
    sCbSimComm.phandle_URC(_SIM_COMM_URC_SMS_IND);
    
    return 1;
}



/*================================ Func Process ===========================*/

/*
    Func: Init Struct Update Fw variable
*/

void Sim_Common_Init_Struct_UpdateFw (void)
{
	sSimFwUpdate.CountByteTotal_u32 			= 0;
	sSimFwUpdate.Pending_u8 					= 0;
    //
    sSimFwUpdate.AddOffset_u32                  = 0;
    sSimFwUpdate.CountPacket_u8                 = 0;
    sSimFwUpdate.LastCrcFile_u8                 = 0;
    sSimFwUpdate.LasCrcCal_u8                   = 0;
    sSimFwUpdate.IsFirmSaoViet_u8               = FALSE;
}

/*
    Func: Init Update fw before send AT+HTTP Read
        + Init Struct var
        + Erase area New Fw
*/

uint8_t Sim_Common_Init_UpdateFw (void)
{      
    if ( sCbSimComm.pConfig_Update == NULL)
        return 0;
    
    sCbSimComm.pConfig_Update();
    
    Sim_Common_Init_Struct_UpdateFw();
    Sim_Common_Init_Struct_FileSys();

    return 1;
}


//
void Sim_Common_Send_AT_Cmd (uint8_t *string, uint16_t length) 
{
    if (length < 64)
    {
        UTIL_Printf( DBLEVEL_M, string, length );
    } else
    {
        UTIL_Printf( DBLEVEL_M, string, 64 );
    }
    
    if (sCbSimComm.pSend != NULL)
        sCbSimComm.pSend(string, length);
}




void Sim_Common_Init_Struct_FileSys (void)
{
    sFileSys.Handle_u32 = 1;
    UTIL_MEM_set ( sFileSys.aTEMP_STR_HANDLE, 0, sizeof (sFileSys.aTEMP_STR_HANDLE) );
    sFileSys.strHandle.Length_u16 = 0;
}

/*
    Func: Clear Queue step
*/

void Sim_Common_Queue_Clear (Struct_Queue_Type *pQueue)
{
    qQueue_Clear(pQueue);
}



uint8_t Sim_Common_Analys_Error (void)
{
    if (sSimCommVar.RxPinReady_u8 == FALSE)  
    {
        UTIL_Printf_Str( DBLEVEL_M, "u_sim: error power or rx pin uart\r\n" );
        return _SIM_COMM_ERROR_PIN_RX;
    }
    
    if (sSimCommVar.TxPinReady_u8 == FALSE)  
    {
        UTIL_Printf_Str( DBLEVEL_M, "u_sim: error tx pin uart\r\n" );
        return _SIM_COMM_ERROR_PIN_TX;
    }
    
    if (sSimCommVar.CallReady_u8 == FALSE)  
    {
        UTIL_Printf_Str( DBLEVEL_M, "u_sim: error card sim\r\n" );
        return _SIM_COMM_ERROR_CARD;
    }
    
    if (sSimCommVar.NetReady_u8 == FALSE)  
    {
        UTIL_Printf_Str( DBLEVEL_M, "u_sim: error network\r\n" );
        return _SIM_COMM_ERROR_ATTACH;
    }
    
    if (sSimCommVar.ServerReady_u8 == FALSE)  
    {
        UTIL_Printf_Str( DBLEVEL_M, "u_sim: error connect server\r\n" );
        return _SIM_COMM_ERROR_TCP;
    }
    
    return _SIM_COMM_ERROR_NONE;
}

sSimInformation *pSim_Common_Get_Infor (void)
{
    return &sSimCommInfor;
}


/**/

void Sim_Common_Init (sSimVariable *sSimCommVar)
{
    sSimCommVar->CallReady_u8           = FALSE;

    sSimCommVar->State_u8               = _SIM_START; 
    
    sSimCommVar->RxPinReady_u8          = FALSE;
    sSimCommVar->TxPinReady_u8          = FALSE;
    sSimCommVar->NetReady_u8            = FALSE;
    sSimCommVar->ServerReady_u8         = FALSE;
}






