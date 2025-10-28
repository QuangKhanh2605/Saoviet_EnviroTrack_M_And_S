
#include "user_app_ethernet.h"
#include "socket.h"

#include "user_modem.h"

#include "user_string.h"
#include "spi.h"


/*================== Static func ===================*/
static void wizchip_write(uint8_t wb);
static uint8_t wizchip_read(void);
static void  wizchip_deselect(void);
static void  wizchip_select(void);


static uint8_t _Cb_Check_Bus (uint8_t event);
static uint8_t _Cb_Hard_Reset (uint8_t event);

static uint8_t _Cb_Check_PHY_LINK (uint8_t event);
static uint8_t _Cb_Running_DHCP (uint8_t event);
static uint8_t _Cb_DNS_Process (uint8_t event);

static uint8_t _Cb_Socket_Control (uint8_t event);

static uint8_t _Cb_Send_Ping (uint8_t event);
static uint8_t _Cb_Send_Data (uint8_t event);

static uint8_t _Cb_Send (uint8_t event);
static uint8_t _Cb_Send_OK (uint8_t event);
static uint8_t _Cb_Send_Timeout (uint8_t event);
static uint8_t _Cb_Recv_Data (uint8_t event);

static uint8_t _Cb_Control_Sim (uint8_t event);
static uint8_t _Cb_INTn_Process (uint8_t event);
static uint8_t _Cb_Power_Off (uint8_t event);

// Func callback cmd
static uint8_t _Cb_Pasv_Port_OK (sData *pData);
static uint8_t _Cb_Get_OK (sData *pData);
static uint8_t _Cb_List_OK (sData *pData);
static uint8_t _Cb_Get_Size_OK (sData *pData); 
static uint8_t _Cb_URC_Ready (sData *pData);
static uint8_t _Cb_Success_Def (sData *pData);
static uint8_t _Cb_Mqtt_Sub_OK (sData *pData);
static uint8_t _Cb_Mqtt_Pub_OK (sData *pData);

static uint8_t _fMQTT_Connect (void);
static uint8_t _fMQTT_Subcribe (void);
static uint8_t _fMQTT_Publish (void);

static uint8_t _fSet_User (void);
static uint8_t _fSet_Pass (void);
static uint8_t _fSet_Pasv_Port (void);
static uint8_t _fSet_Get_File (void);
static uint8_t _fSet_Type_ASCII (void);
static uint8_t _fSet_Type_BIN (void);
static uint8_t _fSet_CWD (void);
static uint8_t _fQuery_List (void);
static uint8_t _fQuery_Size_File (void);

//cmd recv
static uint8_t _Cb_URC_Server (sData *pData);
/*===================== Struct var ===================== */

uint8_t gDATABUF[DATA_BUF_SIZE];
uint8_t aRECEIVE_DATA[DATA_BUF_SIZE];
uint16_t LengthRecv = 0;

/* Network Configuration default */
wiz_NetInfo gWIZNETINFO =
{
    .mac = {0x00, 0x08, 0xdc, 0xab, 0xcd, 0xff},
    .ip  = {192, 168, 1, 101},
    .sn  = {255, 255, 255, 0},
    .gw  = {192, 168, 1, 1},
    .dns = {8, 8, 8, 8},
    .dhcp = NETINFO_DHCP
};

/*===================== Extern struct var ===================== */

sEvent_struct sEventAppEth[] =
{    
    { _EVENT_ETH_CONTROL_SIM, 	    1, 0, 1000, 		_Cb_Control_Sim },
	{ _EVENT_ETH_HARD_RESET, 		1, 0, 1000, 	    _Cb_Hard_Reset },           
	{ _EVENT_ETH_CHECK_BUS, 	    0, 0, 1000, 		_Cb_Check_Bus }, 
	{ _EVENT_ETH_PHY_LINK, 		    0, 0, 1000, 		_Cb_Check_PHY_LINK },
    
	{ _EVENT_ETH_DNS_PROCESS, 		0, 0, 50, 			_Cb_DNS_Process },
    { _EVENT_ETH_RUN_DHCP, 		    0, 0, 0, 			_Cb_Running_DHCP },   //Must set period = 0
    { _EVENT_ETH_SOCK_CTRL, 	    0, 0, 10, 			_Cb_Socket_Control },
    
    { _EVENT_ETH_SEND_PING, 		0, 0, 30000, 		_Cb_Send_Ping },    
    { _EVENT_ETH_SEND_DATA, 		0, 0, 1000, 		_Cb_Send_Data },     //Quet send data lien tuc

    { _EVENT_ETH_SEND, 		        0, 0, 1000, 	    _Cb_Send },
    { _EVENT_ETH_SEND_OK, 		    0, 0, 1000, 	    _Cb_Send_OK },
    { _EVENT_ETH_SEND_TIMEOUT, 		0, 0, 5000, 		_Cb_Send_Timeout },
    { _EVENT_ETH_RECV_DATA, 		0, 0, 0, 		    _Cb_Recv_Data },
    
    { _EVENT_ETH_INTn_PROCESS, 		0, 0, 1, 			_Cb_INTn_Process },
    
    { _EVENT_ETH_POWER_OFF, 		0, 0, 0, 			_Cb_Power_Off }, 
};

sEthernetVariable sAppEthVar = 
{
    .cSoftReset_u16 = 0xFF,
};


/*DNS client */
uint8_t aDNS_2nd[4]    = {208, 67, 222, 222};      	// Secondary DNS server IP

//cmd
uint8_t aETH_MQTT_CONN[2] = 
{
	_ETH_MQTT_CONN,
    _ETH_MQTT_SUB,
}; 

uint8_t aETH_FTP_CONN[7] = 
{
	_ETH_FTP_SET_USER,
    _ETH_FTP_SET_PASS,
    _ETH_FTP_SET_TYPE_BIN,
    _ETH_FTP_SET_DIR,
    _ETH_FTP_SET_PASV_PORT,
    _ETH_FTP_QUER_SIZE,
    _ETH_FTP_SET_GET_FILE,
}; 


static uint8_t ConACK[4] = {MY_CONNACK, 0x02, 0x00, 0x00};
static uint8_t SubACK[6] = {MY_SUBACK, 0x04, 0x00, 0x0a, 0x00, 0x00};
static uint8_t PubACK[4] = {MY_PUBACK, 0x02, 0x00, 0x01};
static uint8_t PingResp[2] = {MY_PINGRESP, 0x00};


sCommandEth     sEthCmd[] = 
{
    {   _ETH_MQTT_CONN,             _fMQTT_Connect,     _Cb_Success_Def,    (char *) ConACK     },  
    {   _ETH_MQTT_SUB,              _fMQTT_Subcribe,    _Cb_Mqtt_Sub_OK,    (char *) SubACK     },
    {   _ETH_MQTT_PUB_NACK,         _fMQTT_Publish,     _Cb_Mqtt_Pub_OK,    NULL                },
    {   _ETH_MQTT_PUB_ACK,          _fMQTT_Publish,     _Cb_Mqtt_Pub_OK,    (char *) PubACK     },
    {   _ETH_MQTT_PING,             _fMQTT_Publish,     _Cb_Mqtt_Pub_OK,    (char *) PubACK   },
    
    {   _ETH_FTP_SET_USER,          _fSet_User,         _Cb_Success_Def,    "331"     },
    {   _ETH_FTP_SET_PASS,          _fSet_Pass,         _Cb_Success_Def,    "230"     },
    
    {   _ETH_FTP_SET_TYPE_ASCII,    _fSet_Type_ASCII,   _Cb_Success_Def,    "200"     },
	{   _ETH_FTP_SET_TYPE_BIN,      _fSet_Type_BIN,     _Cb_Success_Def,    "200"     },
    
	{   _ETH_FTP_QUER_SER_DIR,      _fQuery_List,       _Cb_List_OK,        "150"     },
    
    {   _ETH_FTP_QUER_SIZE,         _fQuery_Size_File,  _Cb_Get_Size_OK,    "213"     },
    
    {   _ETH_FTP_SET_DIR,           _fSet_CWD,          _Cb_Success_Def,    "250"     },
    {   _ETH_FTP_SET_PASV_PORT,     _fSet_Pasv_Port,    _Cb_Pasv_Port_OK,   "227"     },
    {   _ETH_FTP_SET_GET_FILE,      _fSet_Get_File,     _Cb_Get_OK,         "150"     },  // 150
};

sCommandEth sEthResCode[] = 
{
    {   _ETH_MQTT_URC_SERVER,       NULL,               _Cb_URC_Server,     "AMI/"    },
    
    {   _ETH_FTP_URC_READY,         NULL,               _Cb_URC_Ready,      "220"    },
    {   _ETH_FTP_URC_CLOSING,       NULL,               NULL,               "226"    },
};


static uint8_t aQEthStepControl[ETH_MAX_CMD_QUEUE];
Struct_Queue_Type qEthStep;

static uint8_t Pending_u8 = false;
static uint32_t LandMarkSend_u32 = 0;

sEthFTPvariable sFTPvar;

/*========================Function ===============================*/
//Func Callback Event
static uint8_t _Cb_Hard_Reset (uint8_t event)
{
    uint8_t i = 0;
    static uint16_t cLastHardReset_u16 = 0;
    
    //Disable all event > PHY
    for (i = _EVENT_ETH_HARD_RESET + 1; i < _EVENT_APP_ETH_END; i++)
        fevent_disable(sEventAppEth, i);
    
    if (*sAppEthVar.pModePower_u8 == _POWER_MODE_SAVE)
    {
        if ( (abs(cLastHardReset_u16 - sAppEthVar.cHardReset_u16) >= 2) 
        #ifdef USING_APP_SIM
            || ( (sSimCommVar.ServerReady_u8 == true) && (AppMem_Is_New_Record() != true) )
        #endif
            )
        {
            cLastHardReset_u16 = sAppEthVar.cHardReset_u16;
            //power off
            fevent_active(sEventAppEth, _EVENT_ETH_POWER_OFF);
            
            return 1;
        }
        
        if (AppEth_W5500_Hard_Reset() == true)
        {
            UTIL_Printf_Str ( DBLEVEL_M, "u_app_eth: w5500 start!\r\n" );
            
            sAppEthVar.cHardReset_u16++;
            AppEth_Init_Default();
            fevent_active(sEventAppEth, _EVENT_ETH_CHECK_BUS);
        } else
        {
            fevent_enable(sEventAppEth, event);
        }
    } else
    {
        //Cho thiet bi reset cach nhau 10 phut
        sAppEthVar.cSoftReset_u16++;
        if (sAppEthVar.cSoftReset_u16 <= 5)  //120
        {
            AppEth_Init_Default();
            fevent_active(sEventAppEth, _EVENT_ETH_CHECK_BUS);
            return 1;
        }

        if (AppEth_W5500_Hard_Reset() == true)
        {
            UTIL_Printf_Str ( DBLEVEL_M, "u_app_eth: w5500 start!\r\n" );
            
            sAppEthVar.cSoftReset_u16 = 0;
            sAppEthVar.cHardReset_u16++;
            AppEth_Init_Default();
            fevent_active(sEventAppEth, _EVENT_ETH_CHECK_BUS);
        } else
        {
            fevent_enable(sEventAppEth, event);
        }
        

    }

	return 1;
}


static uint8_t _Cb_Check_Bus (uint8_t event)
{
    static uint8_t RetryBus = 0;
    
    /*Test SPI by Read Net Config default? Doc version
     * + Read Version: default = 0x04
     * + Read PHY Status If connect ethernet: Default 191 (0x10111111b)
     */
    UTIL_Printf_Str ( DBLEVEL_H, "u_app_eth: test bus spi\r\n" );
    APP_LOG(TS_OFF, DBLEVEL_H, "u_app_eth: version w5500: %d\r\n", getVERSIONR()); 
    
    sAppEthVar.Status_u8 = _ETH_CHECK_SPI;
    
    if ( (getVERSIONR() == 0x04)
        && (getVERSIONR() == 0x04) 
        && (getVERSIONR() == 0x04))
    {
        //Init
        AppEth_W5500_Init();
        //Init default Net
        AppEth_Net_Config();
        //active event
        sAppEthVar.PHYstatus_u8 = false;
        fevent_active(sEventAppEth, _EVENT_ETH_PHY_LINK);
        RetryBus = 0;
    } else
    {
        if (RetryBus++ >= ETH_MAX_RETRY_BUS)
        {
            RetryBus = 0;
            fevent_active(sEventAppEth, _EVENT_ETH_HARD_RESET);
        } else
        {
            fevent_enable(sEventAppEth, event);
        }
    }
    
	return 1;
}

/*
    Func: check Phy link
        + 1s chay vao check 1 lan
        + doc lien tiep 5 lan -> neu fail lien tiep 5 lan -> PHY fail
        + 
*/
static uint8_t _Cb_Check_PHY_LINK (uint8_t event)
{
    static uint8_t CountFalse = 0;
    uint8_t i = 0;

    if ( AppEth_Get_PHYStatus() == false )
    {
        if (CountFalse++ >= ETH_MAX_FAIL_PHY)
        {
            CountFalse = 0; 
            
            sAppEthVar.cHardReset_u16++;
            sAppEthVar.Status_u8 = _ETH_CHECK_PHY;
            
            UTIL_Printf_Str ( DBLEVEL_M, "u_app_eth: phy link remove\r\n" );
            
            sAppEthVar.PHYstatus_u8 = false;

            //Disable all event > PHY
            for (i = _EVENT_ETH_PHY_LINK + 1; i < _EVENT_APP_ETH_END; i++)
                fevent_disable(sEventAppEth, i);
            
            fevent_active(sEventAppEth, _EVENT_ETH_HARD_RESET); 
        }
    } else
    {
        CountFalse = 0;
        
        if (sAppEthVar.PHYstatus_u8 == false)
        {
            UTIL_Printf_Str ( DBLEVEL_M, "u_app_eth: phy link become ready\r\n" );
             
            sAppEthVar.Status_u8 = _ETH_CHECK_PHY;
            
            //doi mode connect neu co req
            if (*sAppEthVar.pModeConnFuture_u8 != 0)
            {
                *sAppEthVar.pModeConnLast_u8 = *sAppEthVar.pModeConnNow_u8;    //Luu lai mode cuoi cung
                *sAppEthVar.pModeConnNow_u8 = *sAppEthVar.pModeConnFuture_u8;
                *sAppEthVar.pModeConnFuture_u8 = 0; 
            }
            
            Modem_Set_Server_Infor_To_App();
            
            //Start DHCP or DNS
            if ( (*sAppEthVar.pModeConnNow_u8 == MODE_CONNECT_DATA_MAIN)
                || (*sAppEthVar.pModeConnNow_u8  == MODE_CONNECT_DATA_BACKUP) )
            {
                if (*sAppEthVar.sServer.DomainOrIp_u8== __SERVER_DOMAIN)
                {
                    fevent_active(sEventAppEth, _EVENT_ETH_DNS_PROCESS);
                } else
                {
                    DHCP_stop();
                    fevent_active(sEventAppEth, _EVENT_ETH_RUN_DHCP);
                }
                
                //reset retry update
                sFTPvar.Retry_u8 = 0;
            } else
            {          
                if (*sAppEthVar.sUpdate.DomainOrIp_u8== __SERVER_DOMAIN)
                {
                    fevent_active(sEventAppEth, _EVENT_ETH_DNS_PROCESS);
                } else
                {
                    DHCP_stop();
                    fevent_active(sEventAppEth, _EVENT_ETH_RUN_DHCP);
                }
            }
        }
        
        sAppEthVar.PHYstatus_u8 = true;
        
        //check timeout running
        if (Check_Time_Out (sAppEthVar.LandMarkRun_u32, 120000) == true)
        {
            fevent_active(sEventAppEth, _EVENT_ETH_HARD_RESET);
            return 1;
        }
    }
        
    fevent_enable(sEventAppEth, event);

    return 1;
}



static uint8_t _Cb_DNS_Process (uint8_t event)
{
	static uint8_t StepDNS_u8 = 0, DNSResult = false;
	int32_t ret = -1;
    uint8_t *pIP;
    uint8_t *sIP;
    
    sAppEthVar.Status_u8 = _ETH_DNS_RUNNING;
    sAppEthVar.LandMarkRun_u32 = RtCountSystick_u32;
    
    if ( (*sAppEthVar.pModeConnNow_u8 == MODE_CONNECT_DATA_MAIN)
            || (*sAppEthVar.pModeConnNow_u8  == MODE_CONNECT_DATA_BACKUP) )
    {
        sIP = sAppEthVar.sServer.sIP->Data_a8;
        pIP = sAppEthVar.sServer.aIP;
    } else
    {
        sIP = sAppEthVar.sUpdate.pFirmIP->Data_a8;
        pIP = sAppEthVar.sUpdate.aIP;
    }
    
    
	switch (StepDNS_u8)
	{
		case 0:
			DNS_init(SOCK_DNS, gDATABUF);
			break;
		case 1:
			ret = DNS_run (gWIZNETINFO.dns, sIP, pIP);
			if (ret > 0)
			{
				UTIL_Printf_Str ( DBLEVEL_H, "> 1st DNS Respond\r\n" );
				StepDNS_u8 = 10;
				DNSResult = true;
			}
			break;
		case 2:
			ret = DNS_run (aDNS_2nd, sIP, pIP);
			if (ret > 0)
			{
				UTIL_Printf_Str ( DBLEVEL_H, "> 2nd DNS Respond\r\n" );
				DNSResult = true;
				StepDNS_u8 = 10;
			} else if(ret == -1)
			{
				UTIL_Printf_Str ( DBLEVEL_H, "> MAX_DOMAIN_NAME is too small. Should be redefined it.\r\n" );
			} else
			{
				UTIL_Printf_Str ( DBLEVEL_H, "> DNS Failed\r\n" );
			}

			break;
		default:
			StepDNS_u8 = 0;
			if (DNSResult == true)
			{
                APP_LOG(TS_OFF, DBLEVEL_M, "> Convert DNS to IP: %d.%d.%d.%d \r\n",
                                            pIP[0], pIP[1],
                                            pIP[2], pIP[3]); 

                DNSResult = false;
				fevent_active(sEventAppEth, _EVENT_ETH_RUN_DHCP);
				return 1;
			}

			break;
	}

	fevent_enable(sEventAppEth, event);
	StepDNS_u8++;

	return 1;
}


static uint8_t _Cb_Running_DHCP (uint8_t event)
{
    uint8_t DHCPResult = 0;
	static uint8_t stepDHCP_u8 = 0;
    static uint32_t LandMarkRunDhcp_u32 = 0;
    static uint8_t Retry = 0;

    sAppEthVar.Status_u8 = _ETH_DHCP_RUNNING;
    sAppEthVar.LandMarkRun_u32 = RtCountSystick_u32;
    //Tat event duoi
    fevent_disable(sEventAppEth, _EVENT_ETH_SOCK_CTRL);
    
	switch (stepDHCP_u8)
	{
		case 0:
            if (Retry++ >= ETH_MAX_RETRY_DHCP)
            {
                Retry = 0;
                fevent_active(sEventAppEth, _EVENT_ETH_HARD_RESET);
                return 1;
            }
            
			/* DHCP client Initialization */
			if (gWIZNETINFO.dhcp == NETINFO_DHCP)
			{
				UTIL_Printf_Str ( DBLEVEL_H, "u_app_eth: DHCP Init\r\n" );
				DHCP_init (SOCKET_DEMO, gDATABUF);
				// if you want different action instead default ip assign, update, conflict.
				// if cbfunc == 0, act as default.
				reg_dhcp_cbfunc (AppEth_Cb_IP_Assign, AppEth_Cb_IP_Assign, AppEth_Cb_IP_Conflict);
			} else
			{
				// Static
				UTIL_Printf_Str ( DBLEVEL_H, "u_app_eth: static network\r\n" );
				AppEth_Display_Net_Conf();
			}
			stepDHCP_u8++;
            LandMarkRunDhcp_u32 = RtCountSystick_u32;
			break;
		case 1:
			DHCPResult = AppEth_DHCP_Start();
			//
			if (DHCPResult == DHCP_IP_LEASED)
			{
                stepDHCP_u8 = 0;
                Retry = 0;
                
                APP_LOG(TS_OFF, DBLEVEL_M, "u_app_eth: DHCP ip leased %d.%d.%d.%d\r\n",  
                                                gWIZNETINFO.ip[0], gWIZNETINFO.ip[1], gWIZNETINFO.ip[2], gWIZNETINFO.ip[3] );

				//Active event Open Socket
				fevent_active(sEventAppEth, _EVENT_ETH_SOCK_CTRL);
                sAppEthVar.ReConnect_u8 = false;
                AppEth_FTP_Init_Default();
                return 1;
			} else if ( (DHCPResult == DHCP_FAILED) || (DHCPResult == DHCP_STOPPED) )
            {
                stepDHCP_u8 = 0;
				UTIL_Printf_Str ( DBLEVEL_H, "u_app_eth: DHCP failed\r\n" );
            }
            
            //check timeout
            if (Check_Time_Out(LandMarkRunDhcp_u32, 10000) == true)
            {
                stepDHCP_u8 = 0;
				UTIL_Printf_Str ( DBLEVEL_M, "u_app_eth: DHCP timeout\r\n" );
            }
			break;
		default:
			break;
	}

    fevent_enable(sEventAppEth, event);
    
    return 1;
}

uint8_t g_send_buf[DATA_BUF_SIZE];
uint8_t g_recv_buf[DATA_BUF_SIZE];

uint8_t data_buf [DATA_BUF_SIZE];

uint8_t flag_sent_http_request = DISABLE;

uint8_t Domain_name[] = "up.saovietgroup.com.vn";
uint8_t URI[] = "/WMGE/SVWM_GSM_ETH_V1_1_6.bin";



static uint8_t _Cb_Socket_Control (uint8_t event)
{ 
    //http client 
    
//    uint16_t i;
//    uint16_t len = 0;
//  
//    httpc_init(SOCKET_DEMO, sAppEthVar.sServer.aIP, 80, g_send_buf, g_recv_buf);
//    
//    while (1)
//    {
//        httpc_connection_handler();
//        
//        if (httpc_isSockOpen)
//        {
//            if (httpc_connect() == HTTPC_TRUE)
//                httpc_isConnected = HTTPC_TRUE;
//        }
//        // HTTP client example
//        if (httpc_isConnected)
//        {
//		    if (!flag_sent_http_request)
//            {
//                // Send: HTTP request
//                request.method = (uint8_t *)HTTP_GET;
//                request.uri = (uint8_t *)URI;
//                request.host = (uint8_t *)Domain_name;
//                
//                // HTTP client example #1: Function for send HTTP request (header and body fields are integrated)
//                {
//                    httpc_send(&request, g_recv_buf, g_send_buf, 0);
//                }
//                
//                // HTTP client example #2: Separate functions for HTTP request - default header + body
//				  {
////					  httpc_send_header(&request, g_recv_buf, NULL, len);
////					  httpc_send_body(g_send_buf, len); // Send HTTP requset message body
//				  }
//
//				  // HTTP client example #3: Separate functions for HTTP request with custom header fields - default header + custom header + body
//				  {
//					  //httpc_add_customHeader_field(tmpbuf, "Custom-Auth", "auth_method_string"); // custom header field extended - example #1
//					  //httpc_add_customHeader_field(tmpbuf, "Key", "auth_key_string"); // custom header field extended - example #2
//					  //httpc_send_header(&request, g_recv_buf, tmpbuf, len);
//					  //httpc_send_body(g_send_buf, len);
//				  }
//                  
//                flag_sent_http_request = ENABLE;
//            }
//            
//		    // Recv: HTTP response
//		    if (httpc_isReceived > 0)
//		    {
//                len = httpc_recv(g_recv_buf, httpc_isReceived);
//                
//                APP_LOG(TS_OFF, DBLEVEL_M," >> HTTP Response - Received len: %d\r\n", len);
//                APP_LOG(TS_OFF, DBLEVEL_M,"======================================================\r\n");
//                
//                for(i = 0; i < len; i++) 
//                    APP_LOG(TS_OFF, DBLEVEL_M, "%c", g_recv_buf[i]);
//                
//                APP_LOG(TS_OFF, DBLEVEL_M,"\r\n");
//                APP_LOG(TS_OFF, DBLEVEL_M,"======================================================\r\n");
//            }
//        }
//    }
    
    uint8_t SockCtrlState = 0;
    uint8_t SockDataState = 0;
    static uint16_t AnyPortCtrl = 50000;
    static uint16_t AnyPortData = 35000;        
    static uint32_t LandMarkEstab_u32 = 0;
    uint8_t rReConn_u8 = false;
    
    sAppEthVar.LandMarkRun_u32 = RtCountSystick_u32;
    //Ctrl socket run
    switch (*sAppEthVar.pModeConnNow_u8)
    {
        case _ETH_MODE_CONN_MAIN:
        case _ETH_MODE_CONN_BACKUP:
            sEventAppEth[_EVENT_ETH_SOCK_CTRL].e_period = 10;
            
            SockCtrlState = AppEth_Socket_Control (SOCKET_DEMO, sAppEthVar.sServer.aIP, 
                                                    *sAppEthVar.sServer.Port_u16, &AnyPortCtrl);
            
            if (SockCtrlState == SOCK_ESTABLISHED)
            {
                LandMarkEstab_u32 = RtCountSystick_u32;
                //Lan dau moi active lai Send
                if ( (!sAppEthVar.ReConnect_u8) || (sAppEthVar.Status_u8 < _ETH_TCP_CONNECT) )
                {
                    APP_LOG(TS_OFF, DBLEVEL_M, "u_app_eth: socket established: %d:%d:%d:%d : %d\r\n",  
                                                sAppEthVar.sServer.aIP[1], sAppEthVar.sServer.aIP[1], sAppEthVar.sServer.aIP[2],
                                                sAppEthVar.sServer.aIP[3], *sAppEthVar.sServer.Port_u16);

                    sAppEthVar.Status_u8 = _ETH_TCP_CONNECT;
                    sAppEthVar.ReConnect_u8 = true;
                    //Push step connect mqtt
                    qQueue_Clear(&qEthStep);
                    Pending_u8 = false;
                    AppEth_Push_Block_To_Queue( aETH_MQTT_CONN, sizeof(aETH_MQTT_CONN) );
                }
            } else if (SockCtrlState == SOCK_CLOSED)
            {
                if (Check_Time_Out(LandMarkEstab_u32, ETH_MAX_TIME_CTRL_SOCK) == true) 
                {
                    LandMarkEstab_u32 = RtCountSystick_u32;
                    rReConn_u8 = true;
                }
            }
            break;
        case _ETH_MODE_CONN_FTP:
            if (sFTPvar.Retry_u8 >= ETH_MAX_RETRY_UPDATE)
            {
                AppEth_Finish_Update(__FAIL_CONNECT);
                return true;
            }
            
            sEventAppEth[_EVENT_ETH_SOCK_CTRL].e_period = 0;
                
            SockCtrlState = AppEth_Socket_Control (CTRL_SOCK, sAppEthVar.sUpdate.aIP, 
                                                              *sAppEthVar.sUpdate.Port_u16, &AnyPortCtrl);
        
            if (SockCtrlState == SOCK_ESTABLISHED)
            {
                LandMarkEstab_u32 = RtCountSystick_u32;
                if (!sAppEthVar.ReConnect_u8)
                {
                    APP_LOG(TS_OFF, DBLEVEL_M, "u_app_eth: socket established: %d:%d:%d:%d : %d\r\n",  
                                                sAppEthVar.sUpdate.aIP[1], sAppEthVar.sUpdate.aIP[1], sAppEthVar.sUpdate.aIP[2],
                                                sAppEthVar.sUpdate.aIP[3], *sAppEthVar.sUpdate.Port_u16);    
                    
                    sAppEthVar.Status_u8 = _ETH_FTP_CONNECTED;
                    sAppEthVar.ReConnect_u8 = true;
                    
                    AppEth_Default_Struct_UpdateVar();
                    
                    qQueue_Clear(&qEthStep);
                    Pending_u8 = false;
                    AppEth_Push_Block_To_Queue( aETH_FTP_CONN, sizeof(aETH_FTP_CONN) ); 
                }
                
                //Polling recv
                if (getSn_RX_RSR(CTRL_SOCK) > 0)
                { 
                   fevent_active(sEventAppEth, _EVENT_ETH_RECV_DATA);
                } 
            } else if (SockCtrlState == SOCK_CLOSED)
            {
                if (Check_Time_Out(LandMarkEstab_u32, ETH_MAX_TIME_CTRL_SOCK) == true) 
                {
                    LandMarkEstab_u32 = RtCountSystick_u32;
                    AnyPortCtrl = 50000;
                    rReConn_u8 = true;
                }            
            }  
            
            //Data socket run
            if (sFTPvar.dsock_state == DATASOCK_READY)
            {
                if (sFTPvar.DataSockConnected_u8 != true)
                {
                    SockDataState = AppEth_Socket_Control (DATA_SOCK, sFTPvar.RemoteIP,
                                                           sFTPvar.RemotePort, &AnyPortData);
                } else
                {
                    SockDataState = getSn_SR(DATA_SOCK);
                }
                
                if (SockDataState == SOCK_ESTABLISHED)
                {
                    sFTPvar.DataSockConnected_u8 = true;
                }
                
                if (sFTPvar.DataSockConnected_u8 == true)
                {
                    //Neu chua lay xong fw ma mat ket noi: thu lai
                    if ( (AppEth_FTP_DataSock_Run() == false)
                        || (SockDataState != SOCK_ESTABLISHED))
                    {
                        close (CTRL_SOCK);
                        close (DATA_SOCK);
                        rReConn_u8 = true;
                    }
                }
            }
            
            //Neu 60s ma k connect duoc: se retry lai
            if (Check_Time_Out(sFTPvar.LandMark_u32, 60000) == true)
            {
                rReConn_u8 = true;
            }
            
            //Kiem tra xem update thanh cong khong
            if ((sAppEthVar.sUpdateVar.FwSizeGet_u32 != 0)
                && (sAppEthVar.sUpdateVar.Total_u8 >= sAppEthVar.sUpdateVar.FwSizeGet_u32))
            {
                //Check crc
                if (sAppEthVar.sUpdateVar.CrcCalcu_u8 == sAppEthVar.sUpdateVar.CrcFile_u8)
                {
                    AppEth_Finish_Update(__SUCCESSFUL);
                    return true;
                } else
                {
                    AppEth_Finish_Update(__FAIL_CRC);
                    return false;
                }
            }
            break;
        case _ETH_MODE_CONN_HTTP:
            //Neu 60s ma k connect duoc: se retry lai
            if (Check_Time_Out(sFTPvar.LandMark_u32, 60000) == true)
            {
                rReConn_u8 = true;
                sAppEthVar.Retry_u8 = ETH_MAX_SOCK_RETRY;
            }
            break;
        default:
            break;
    }
 
    if (rReConn_u8 == true)
    {
        sAppEthVar.Retry_u8++;
        if (sAppEthVar.Retry_u8 >= ETH_MAX_SOCK_RETRY)
        {
            fevent_active(sEventAppEth, _EVENT_ETH_HARD_RESET);
        } else
        {
            fevent_active(sEventAppEth, _EVENT_ETH_RUN_DHCP);
        }
        
        return true;
    }
    
    fevent_enable(sEventAppEth, event);
    
    
    return 1;
}


//Func Callback Event
static uint8_t _Cb_Send (uint8_t event)
{
    uint8_t cmd_u8 = AppEth_Get_Queue(0); 
    
    sAppEthVar.LandMarkRun_u32 = RtCountSystick_u32;
    
    if (cmd_u8 < _ETH_CMD_END)
    {
        Pending_u8 = true;
        LandMarkSend_u32 = RtCountSystick_u32;
        
        //send
        if (sEthCmd[cmd_u8].pFunc_Send != NULL)
        {
            if (sEthCmd[cmd_u8].pFunc_Send() == false)
            {   
                fevent_active(sEventAppEth, _EVENT_ETH_SEND_TIMEOUT);
                Pending_u8 = false;
                return 1;
            }
        }
        //cho timeout or excute success
        if (sEthCmd[cmd_u8].sResp == NULL)
        {
            sEthCmd[cmd_u8].pCb_Success(NULL);
            fevent_active(sEventAppEth, _EVENT_ETH_SEND_OK);
        } else
        {
            fevent_enable(sEventAppEth, _EVENT_ETH_SEND_TIMEOUT);
        }
    } else
    {
        Pending_u8 = false;
    }

	return 1;
}

static uint8_t _Cb_Send_OK (uint8_t event)
{
    //Nhay tiep lenh tiep theo
    AppEth_Get_Queue(1);
    fevent_disable(sEventAppEth, _EVENT_ETH_SEND_TIMEOUT);
    fevent_enable(sEventAppEth, _EVENT_ETH_SEND);

	return 1;
}

static uint8_t _Cb_Send_Timeout (uint8_t event)
{
    uint8_t cmd_u8 = AppEth_Get_Queue(0); 
    
    UTIL_Printf_Str ( DBLEVEL_M, "u_app_eth: send timeout\r\n");
    
    sAppEthVar.cHardReset_u16++;
    mSet_default_MQTT();
    
    switch (cmd_u8)
    {
        case _ETH_MQTT_CONN:
        case _ETH_MQTT_SUB:
            //chuyen xuong event DHCP
            qQueue_Clear(&qEthStep);
            Pending_u8 = false;
            fevent_active(sEventAppEth, _EVENT_ETH_RUN_DHCP);
            break;
        case _ETH_MQTT_PUB_ACK:
        case _ETH_MQTT_PING:
            //clear queue and push conn again
            qQueue_Clear(&qEthStep);
            Pending_u8 = false;
            AppEth_Push_Block_To_Queue( aETH_MQTT_CONN, sizeof(aETH_MQTT_CONN) );
            break;
        case _ETH_FTP_SET_USER:
        case _ETH_FTP_SET_PASS:
        case _ETH_FTP_SET_TYPE_ASCII:
        case _ETH_FTP_SET_TYPE_BIN:
        case _ETH_FTP_QUER_SER_DIR:
        case _ETH_FTP_QUER_SIZE:
        case _ETH_FTP_SET_DIR:
        case _ETH_FTP_SET_PASV_PORT:
        case _ETH_FTP_SET_GET_FILE:
            //chuyen xuong event DHCP
            send (CTRL_SOCK, (uint8_t *)"QUIT\r\n", strlen("QUIT\r\n"));
            qQueue_Clear(&qEthStep);
            Pending_u8 = false;
            fevent_active(sEventAppEth, _EVENT_ETH_RUN_DHCP);
            break;
        default:
            break;
    }
    
	return 1;
}

static uint8_t _Cb_Recv_Data (uint8_t event)
{ 
    int Pos = -1;
    uint8_t cmd_u8 = AppEth_Get_Queue(0); 
    uint16_t i = 0;
    uint16_t size = 0;
    
    sData   strSource = {&aRECEIVE_DATA[0], 0};
    sData   strFix = {NULL, 0};

    if ( (*sAppEthVar.pModeConnNow_u8 == MODE_CONNECT_DATA_MAIN)
          || (*sAppEthVar.pModeConnNow_u8  == MODE_CONNECT_DATA_BACKUP) )
    {
        if ((size = getSn_RX_RSR(SOCKET_DEMO)) > 0)
        {
            if (size > DATA_BUF_SIZE) 
                size = DATA_BUF_SIZE - 1;
            
            LengthRecv = recv (SOCKET_DEMO, aRECEIVE_DATA, size);
            strSource.Length_u16 = LengthRecv;
            
            if (LengthRecv != size)
            {
                return 0;
            }
        }
    } else
    {
        if ((size = getSn_RX_RSR(CTRL_SOCK)) > 0)
        { 
            memset(aRECEIVE_DATA, 0, sizeof(aRECEIVE_DATA));
            
            if (size > sizeof(aRECEIVE_DATA)) 
                size = sizeof(aRECEIVE_DATA) - 1;
            
            LengthRecv = recv(CTRL_SOCK, aRECEIVE_DATA, size);

            if (LengthRecv != size)
            {
                return 0;
            }
            
            aRECEIVE_DATA[LengthRecv] = '\0';
            strSource.Length_u16 = LengthRecv + 1;
        }
    }

    if (LengthRecv > 0)
    {
        UTIL_Printf_Str ( DBLEVEL_M, "u_app_eth: intn recv: \r\n");          
        UTIL_Printf ( DBLEVEL_M, strSource.Data_a8, strSource.Length_u16);
        UTIL_Printf_Str(DBLEVEL_M, "\r\n");

//        if (sMQTT.Status_u8 == PENDING)
//        {            
            strFix.Data_a8 = (uint8_t*) sEthCmd[cmd_u8].sResp;
            strFix.Length_u16 = strlen( sEthCmd[cmd_u8].sResp );   
    
            if (strFix.Data_a8 != NULL)
            {
                Pos = Find_String_V2(&strFix, &strSource);
                if (Pos >= 0)
                {
                    sEthCmd[cmd_u8].pCb_Success(&strSource);
                    fevent_active(sEventAppEth, _EVENT_ETH_SEND_OK);
                }
            }
//        }
        
        //Check Recv Server
        for (i = _ETH_MQTT_URC_SERVER; i <= _ETH_MQTT_URC_SERVER; i++) 
        {
            strFix.Data_a8 = (uint8_t *) sEthResCode[i].sResp;
            strFix.Length_u16 = strlen(sEthResCode[i].sResp);
            //
            Pos = Find_String_V2(&strFix, &strSource);
            if (Pos >= 0)
            {
                sEthResCode[i].pCb_Success(&strSource);
            }
        }
    }
    
    //Clear
    LengthRecv = 0;
    memset(aRECEIVE_DATA, 0, sizeof(aRECEIVE_DATA));

	return 1;
}


static uint8_t _Cb_Send_Ping (uint8_t event)
{
    if (sAppEthVar.Status_u8 != _ETH_MQTT_CONNECTED)
    {
        return 1;
    }
    
    if (*sAppEthVar.pModePower_u8 == _POWER_MODE_ONLINE)
    {
        sMQTT.aMARK_MESS_PENDING[DATA_PING_2] = TRUE;
        fevent_enable(sEventAppEth, event);   
        fevent_active(sEventAppEth, _EVENT_ETH_SEND_DATA);   
    }
    
    return 1;
}



static uint8_t _Cb_Send_Data (uint8_t event)
{
    if (sAppEthVar.Status_u8 != _ETH_MQTT_CONNECTED)
    {
        return 1;
    }
    
    if (AppEth_Send_Mess() == true)
    {        
        fevent_enable(sEventAppEth, _EVENT_ETH_SEND_PING);
    } else
    {
        if (*sAppEthVar.pModePower_u8 == _POWER_MODE_ONLINE)
        {
            if (sEventAppEth[_EVENT_ETH_SEND_PING].e_status == 0)
                fevent_enable(sEventAppEth, _EVENT_ETH_SEND_PING);   
        } else
        {
            #ifdef USING_APP_MEM
                //Neu con ban tin moi: Enable lai check new rec
                if (AppMem_Is_New_Record() != true)
                {
                    fevent_active(sEventAppEth, _EVENT_ETH_POWER_OFF);
                    return 1;
                }
            #else
                fevent_active(sEventAppEth, _EVENT_ETH_POWER_OFF);
                return 1;
            #endif
        }
    }
    
    fevent_enable(sEventAppSim, event);
    
    return 1;
}



uint8_t AppEth_Send_Mess (void)
{
    uint16_t i = 0;
    uint8_t Result = FALSE;
    static uint32_t LandMarkSendMess_u32 = 0;

    if (Check_Time_Out (LandMarkSendMess_u32, 120000) == true)
    {
        sMQTT.Status_u8 = false;
    } 
    //Kiem tra xem co ban tin nao can gui di khong
    for (i = TOPIC_NOTIF; i < END_MQTT_SEND; i++)
    {
        if (sMQTT.aMARK_MESS_PENDING[i] == TRUE)
        {
            if (i == DATA_PING)  //bo qua ping cua sim
                continue;
            
            Result = TRUE;
            //
            if (sMQTT.Status_u8 != PENDING)
            {
                //Danh dau Mess type hien tai ->neu OK clear di
                sMQTT.MessType_u8 = i;
                //Dong goi ban tin vao buff aPAYLOAD bang cach thuc hien callback
                if (sMark_MessageSend_Type[i].CallBack(i) == TRUE)
                {                           
                    sMQTT.Status_u8 = PENDING;
                    LandMarkSendMess_u32 = RtCountSystick_u32;
                    
                    //Push cmd Pub or Ping
                    if (sMQTT.MessType_u8 == DATA_PING_2)
                    {
                        AppEth_Push_Cmd_To_Queue(_ETH_MQTT_PING);
                    } else
                    {
                        if (sMQTT.PulishQos != 0)
                        {
                            AppEth_Push_Cmd_To_Queue(_ETH_MQTT_PUB_ACK);
                        } else
                        {
                            AppEth_Push_Cmd_To_Queue(_ETH_MQTT_PUB_NACK);
                        }
                    }
                } else
                    sMQTT.aMARK_MESS_PENDING[i] = FALSE;
            } 
        }
    }
    
    return Result;
}


static uint8_t _Cb_Control_Sim (uint8_t event)
{
#ifdef USING_APP_SIM
    if ( ( (*sAppEthVar.pModeConnNow_u8 != MODE_CONNECT_DATA_MAIN)
            && (*sAppEthVar.pModeConnNow_u8  != MODE_CONNECT_DATA_BACKUP) )
        || (*sAppEthVar.pModePower_u8 == _POWER_MODE_SAVE) )
    {
        if (sAppEthVar.Status_u8 < _ETH_TCP_CONNECT)
        {
            AppSim_Restart_If_PSM();
        } else
        {
            //Power Off SIM
            if (sSimCommVar.State_u8 != _SIM_POWER_OFF)
                fevent_active(sEventAppSim, _EVENT_SIM_POWER_OFF);
        }
    }

    fevent_enable(sEventAppEth, event);
#endif

    return 1;
}

static uint8_t _Cb_INTn_Process (uint8_t event)
{
    static uint8_t Retry = 0;
    
    if (AppEth_IRQ_Handler() == false)
    {
        Retry++;
        if ( Retry < 5)
        {
            fevent_enable(sEventAppEth, event);
            return 1;
        }
    } 
    
    Retry = 0;
        
    return 1;
}


static uint8_t _Cb_Power_Off (uint8_t event)
{
    UTIL_Printf_Str ( DBLEVEL_M, "u_app_eth: power off!\r\n"); 
    
    //Set trang thai
    sAppEthVar.Status_u8 = _ETH_POWER_OFF;
    
    W5500_OFF;
    
    //Disable all event
    for (uint8_t i = _EVENT_ETH_CONTROL_SIM; i < _EVENT_ETH_POWER_OFF; i++)
        fevent_disable(sEventAppEth, i);
    
    if (*sAppEthVar.pModePower_u8 == _POWER_MODE_ONLINE)
    { 
        fevent_active(sEventAppEth, _EVENT_ETH_HARD_RESET); 
        fevent_enable(sEventAppEth, _EVENT_ETH_CONTROL_SIM); 
    }

    return 1;
}




/*===================== Funcion ===================== */

static void  wizchip_select(void)
{
	W5500_SCS_LOW;
}

static void  wizchip_deselect(void)
{
	W5500_SCS_HIGH;
}

static uint8_t wizchip_read(void)
{
	uint8_t retVal = 0;
    uint8_t aDUMMY_BYTE = 0xA5;

	HAL_SPI_TransmitReceive(&hspi3, &aDUMMY_BYTE, &retVal, 1, 1000);

	return retVal;
}

static void wizchip_write(uint8_t wb)
{
	uint8_t retVal = 0;

	HAL_SPI_TransmitReceive(&hspi3, &wb, &retVal, 1, 1000); 
}



/*============== Func Handler App Enthernet=============*/

/*
Func: Hard reset module
+ Active low RST atleast 500ms
+ Check PHY regis
*/

uint8_t AppEth_W5500_Hard_Reset(void)
{
	static uint8_t EthStepHardRS_u8 = 0;
	uint8_t Result = false;

	switch (EthStepHardRS_u8)
	{
		case 0:
			UTIL_Printf_Str ( DBLEVEL_M, "\r\nu_app_eth: hard reset w5500...\r\n" );
			W5500_OFF;
			EthStepHardRS_u8++;
			break;
		case 1:
			W5500_ON;
            EthStepHardRS_u8++;
			break;
        case 2:
			W5500_RST_LOW;
			EthStepHardRS_u8++;
			break;
		case 3:
			Result = true;
			W5500_RST_HIGH;
			EthStepHardRS_u8 = 0;
			break;
		default:
            EthStepHardRS_u8 = 0;
			break;
	}

	return Result;
}


void AppEth_Init(void)
{
    //Set callback handler SPI: select, deselect, read, write: 
	reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);   
	reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);   
    
    sMQTT.aMARK_MESS_PENDING[SEND_SERVER_TIME_PENDING] = TRUE;
    
    //creat queue cmd
    qQueue_Create (&qEthStep, ETH_MAX_CMD_QUEUE, sizeof (uint8_t), (uint8_t *) &aQEthStepControl);
}
    


uint8_t AppEth_Task(void)
{
	uint8_t Result = 0;
	uint8_t i = 0;

	for (i = 0; i < _EVENT_APP_ETH_END; i++)
	{
		if (sEventAppEth[i].e_status == 1)
		{
            if (i != _EVENT_ETH_CONTROL_SIM)
                Result = 1;
            
			if ((sEventAppEth[i].e_systick == 0) 
                || ((HAL_GetTick() - sEventAppEth[i].e_systick) >= sEventAppEth[i].e_period))
			{
                sEventAppEth[i].e_status = 0;   //Disable event
				sEventAppEth[i].e_systick = HAL_GetTick();
				sEventAppEth[i].e_function_handler(i);
			}
		}
	}

	return Result;
}




void AppEth_W5500_Init(void)
{
	uint8_t memsize[2][8] = 
    { 
        { 2, 2, 2, 2, 2, 2, 2, 2 }, 
        { 2, 2, 2, 2, 2, 2, 2, 2 }, 
    };
    
	UTIL_Printf_Str ( DBLEVEL_H, "u_app_eth: Set Tx/RX size\r\n" );
	/* wizchip initialize*/
	if (ctlwizchip(CW_INIT_WIZCHIP, (void*) memsize) == -1) 
    {
		UTIL_Printf_Str ( DBLEVEL_H, "u_app_eth: Error Set Tx/RX size\r\n" );
	}
    //New add other example
	UTIL_Printf_Str ( DBLEVEL_H, "u_app_eth: Set RTR, RCR\r\n" );
    setRTR(0x07d0);
	setRCR(4);
    
	UTIL_Printf_Str ( DBLEVEL_H, "u_app_eth: Set IRQ Regis\r\n" );
	setSIMR (0x0F);         //IRQ 4 sock 0 -> 3
	setSn_IMR (0, 0x1F);  //Mark All IRQ Sn
}



int32_t AppEth_Socket_Control (uint8_t sn, uint8_t* destip, uint16_t destport, uint16_t *any_port)
{
    int32_t Result = SOCK_CLOSING;
    static uint8_t StepConn_u8 = 0;
    static uint32_t LandMark_u32 = 0;
    
    // Check the W5500 Socket n status register (Sn_SR, The 'Sn_SR' controlled by Sn_CR command or Packet send/recv status)
    Result = getSn_SR(sn);
    
    switch (getSn_SR(sn))
    {
      case SOCK_ESTABLISHED:
            StepConn_u8 = 0;
            break;
      case SOCK_CLOSE_WAIT:
            StepConn_u8 = 0;
            disconnect(sn);            
            break;
      case SOCK_CLOSED:
            StepConn_u8 = 0;
            // TCP socket open with 'any_port' port number
            APP_LOG(TS_OFF, DBLEVEL_H, "u_app_eth: sock %d: start\r\n", sn);
            close(sn);
            if (socket(sn, Sn_MR_TCP, (*any_port)++, 0x00) != sn)
            {
                if (*any_port == 0xffff) 
                    *any_port = 50000;
            } 
            break;
      case SOCK_INIT :
            switch (StepConn_u8)
            {
                case 0: 
                    APP_LOG(TS_OFF, DBLEVEL_H, "u_app_eth: sock %d: opened\r\n", sn);
                    //	Try to TCP connect to the TCP server (destination)
                    if (connect(sn, destip, destport) != SOCK_OK) 
                    {
                        close(sn);
                        break;
                    }
                    
                    StepConn_u8++;
                    LandMark_u32 = RtCountSystick_u32;
                    break;
                default:
                    if (getSn_IR(sn) & Sn_IR_TIMEOUT)
                    {
                        setSn_IR(sn, Sn_IR_TIMEOUT);
                        APP_LOG(TS_OFF, DBLEVEL_M, "u_app_eth: %d:connect timeout!\r\n", sn);
                        StepConn_u8 = 0;
                    }
                    
                    if (Check_Time_Out(LandMark_u32, 10000) == true)
                    {
                        close(sn);
                    }
                    break;
            }            
            break;      
      default:
            close(sn);
            setSn_MR(sn, Sn_MR_TCP);
            break;
    }
    
    return Result;
}


uint8_t AppEth_IRQ_Handler(void)
{
	uint8_t IRQ_sign = 0;
    
    //Doc tiep thanh ghi SIR (ngat socket n) -> Xem co ngat S0?
    IRQ_sign = getSIR();
    IRQ_sign = IRQ_sign & (1 << SOCKET_DEMO);
    
    if (IRQ_sign != 0)  //Ngat o socket 0
    {
        IRQ_sign = getSn_IR(SOCKET_DEMO);   //Doc tiep S0_IR de check ngat gi
        setSn_IR (SOCKET_DEMO, IRQ_sign);   //Ghi gia tri 1 vao de xoa
        
        if (sAppEthVar.Status_u8 >= _ETH_MQTT_CONNECT)
        {
            //Recv IR
            if (IRQ_sign & Sn_IR_RECV)
            {
                fevent_active(sEventAppEth, _EVENT_ETH_RECV_DATA);
            }
            //TIMEOUT IR
            if (IRQ_sign & Sn_IR_TIMEOUT)
            {
                close(SOCKET_DEMO);
                fevent_active(sEventAppEth, _EVENT_ETH_RUN_DHCP);
            }
        }
    } else
    {
        //thanh ghi SIR cacs bit = '1' cho den khi cac bit Sn_IR duoc xoa. Nen UnMark cac Sn_IR khac k?
        return true;
    }
  
    return false;
}


/*=============== Func callback ======================*/
/*
    Set Register Network: GAR, IPR...
*/
void AppEth_Net_Config (void)
{
	UTIL_Printf_Str ( DBLEVEL_H, "u_app_eth: set network!\r\n");
	ctlnetwork(CN_SET_NETINFO, (void*) &gWIZNETINFO);
}

/*
    Func: Get Net Config
        - Get Net Config to Struct
        - Dislay main debug
*/
void AppEth_Display_Net_Conf(void)
{
    #ifdef _MAIN_DEBUG_
        uint8_t tmpstr[6] = {0,};
    #endif

	ctlnetwork(CN_GET_NETINFO, (void*) &gWIZNETINFO);
}



uint8_t AppEth_Get_PHYStatus(void)
{
	int8_t tmp = 0;

    ctlwizchip(CW_GET_PHYLINK, (void*) &tmp); 
    
    if (tmp == PHY_LINK_ON )
        return true;

    return false;
}


/*
 * 	Func: Check Detect gateway
 * 	Return:
 * 			0:
* 			1: Detect OK
* 			2: Pending
 *
 * */
uint8_t AppEth_Detect_Gateway (uint8_t sn)
{
	static uint8_t StepDetGw = 0;

	uint8_t ip_adde[4] = {0};
    uint8_t SnIRValue = 0;
    uint8_t SHARValue[6] = {0};

    static uint32_t LandMarkProcess = 0;

    switch (StepDetGw)
    {
    	case 0:
    		UTIL_Printf_Str ( DBLEVEL_M, "u_app_eth: detect gateway function\r\n" );

			ip_adde[0]= gWIZNETINFO.ip[0] + 1;
			ip_adde[1]= gWIZNETINFO.ip[1] + 1;
			ip_adde[2]= gWIZNETINFO.ip[2] + 1;
			ip_adde[3]= gWIZNETINFO.ip[3] + 1;

			setSn_DIPR(sn, ip_adde);
			setSn_MR(sn, Sn_MR_TCP);
			setSn_CR(sn, Sn_CR_OPEN);
            
            StepDetGw++;
            break;
        case 1:
			if (getSn_SR(sn) != SOCK_INIT)
			{
				setSn_CR(sn, Sn_CR_CLOSE);
				UTIL_Printf_Str ( DBLEVEL_M, "u_app_eth: detect gateway init fail\r\n" );

				StepDetGw = 0;
				return false;
			}

			setSn_CR(sn, Sn_CR_CONNECT);
            
			LandMarkProcess = RtCountSystick_u32;
			StepDetGw++;
    		break;
    	case 2:
    		//Check timeout to break while
			if (Check_Time_Out(LandMarkProcess, 5000) == true)
			{
                UTIL_Printf_Str ( DBLEVEL_M, "u_app_eth: detect gateway timeout2\r\n" );
				StepDetGw = 0;
				return false;
			}

			//Get Interrupt Socket
			SnIRValue = getSn_IR(sn);
			if (SnIRValue != 0)
				setSn_IR(sn, SnIRValue);     //write '1' to clear bit Int
            
			if (SnIRValue & Sn_IR_TIMEOUT)
			{
				//Timeout IR
                setSn_CR(sn, Sn_CR_CLOSE);
				UTIL_Printf_Str ( DBLEVEL_M, "u_app_eth: detect gateway timeout\r\n" );
				StepDetGw = 0;
				return false;
			} else
			{
				//If not IR timeout -> check SHAR
				getSn_DHAR(sn, SHARValue);
				if (SHARValue[0] != 0xFF)
				{
					//Close socket detect gw
					setSn_CR (sn, Sn_CR_CLOSE);
					UTIL_Printf_Str ( DBLEVEL_M, "u_app_eth: detect gateway OK\r\n" );

					StepDetGw = 0;
					return true;
				}
			}
            
            
    		break;
    	default:
    		StepDetGw = 0;
    		break;
    }

    return pending;
}

/*
    Call back for ip assign
*/

void AppEth_Cb_IP_Assign (void)
{
    getIPfromDHCP(gWIZNETINFO.ip);
    getGWfromDHCP(gWIZNETINFO.gw);
    getSNfromDHCP(gWIZNETINFO.sn);
    getDNSfromDHCP(gWIZNETINFO.dns);
    gWIZNETINFO.dhcp = NETINFO_DHCP;
    /* Network initialization */
    AppEth_Net_Config();

    AppEth_Display_Net_Conf();
}


/*
    Call back for ip Conflict
        + Quay lai hard reset module
*/

void AppEth_Cb_IP_Conflict (void)
{
    UTIL_Printf_Str ( DBLEVEL_H, "u_app_eth: dhcp ip conflict\r\n" );   
    //active Hard reset 
    fevent_active(sEventAppEth, _EVENT_ETH_HARD_RESET);
}



int8_t AppEth_DHCP_Start (void)
{
    int8_t Result = -1;
    static uint8_t my_dhcp_retry = 0;
    /* DHCP IP allocation and check the DHCP lease time (for IP renewal) */
    if (gWIZNETINFO.dhcp == NETINFO_DHCP)
    {
        Result = DHCP_run();

        switch (Result)
        {
            case DHCP_IP_ASSIGN:
            case DHCP_IP_CHANGED:
                /* If this block empty, act with default_ip_assign & default_ip_update */
                //
                // This example calls my_ip_assign in the two case.
                //
                // Add to ...
                //
                break;
            case DHCP_IP_LEASED:
                //
                // TODO: insert user's code here
                //
                break;
            case DHCP_FAILED:
                my_dhcp_retry++;
                if (my_dhcp_retry > ETH_MAX_DHCP_RETRY)
                {
                    DHCP_stop();      
                    my_dhcp_retry = 0;
                }
                break;
            case DHCP_STOPPED:
                break;
            default:
                break;
        }
    }

    return Result;
}


/*
    Func: Restart module sim neu nhu sim dang o che do PSM
*/
void AppEth_Restart_If_PSM(void)
{
    if ( sAppEthVar.Status_u8 == _ETH_POWER_OFF )   //co truong hop loi o day
    {
        sAppEthVar.Status_u8  = _ETH_START;
        fevent_active(sEventAppEth, _EVENT_ETH_HARD_RESET);
        fevent_enable(sEventAppEth, _EVENT_ETH_CONTROL_SIM);
    }
}



//Function handle

uint8_t AppEth_Get_Queue(uint8_t Type)
{
	uint8_t step = 0;

	if (qGet_Number_Items(&qEthStep) == 0)
		return _ETH_CMD_END;
    
	if (qQueue_Receive(&qEthStep, (uint8_t *) &step, Type) == 0)
        return _ETH_CMD_END;

	return step;
}



uint8_t AppEth_Push_Cmd_To_Queue(uint8_t cmd)
{
	if (cmd >= _ETH_CMD_END)
		return 0;

	if (qQueue_Send(&qEthStep, (uint8_t *) &cmd, _TYPE_SEND_TO_END) == 0)
		return 0;

    if ((Pending_u8 == false ) 
        || (Check_Time_Out(LandMarkSend_u32, 80000) == true) )
    {
		fevent_active(sEventAppEth, _EVENT_ETH_SEND);
    }

	return 1;
}


uint8_t AppEth_Push_Block_To_Queue(uint8_t *blockCmd, uint8_t size)
{
	uint8_t i = 0;

	for (i = 0; i < size; i++)
	{
		if (AppEth_Push_Cmd_To_Queue (blockCmd[i]) == 0)
			return 0;
	}

	return 1;
}



//Func callback cmd
static uint8_t _Cb_Success_Def (sData *pData)
{
    
    return 1;
}


static uint8_t _Cb_Mqtt_Sub_OK (sData *pData)
{
    UTIL_Printf_Str( DBLEVEL_M, "u_app_eth: mqtt connect ok!\r\n" );
    
    sMQTT.aMARK_MESS_PENDING[DATA_HANDSHAKE] = TRUE;
    
    //active send pub
    mSet_default_MQTT(); 
    fevent_active(sEventAppEth, _EVENT_ETH_SEND_DATA);
    sAppEthVar.Status_u8 = _ETH_MQTT_CONNECTED;

    return 1;
}

static uint8_t _Cb_Mqtt_Pub_OK (sData *pData)
{
    UTIL_Printf_Str( DBLEVEL_M, "u_app_eth: mqtt publish ok!\r\n" );
    
    //Set status ve true
    sAppEthVar.Status_u8 = _ETH_MQTT_CONNECTED;
    sMQTT.Status_u8 = TRUE;   
        
    switch (sMQTT.MessType_u8)
    {
        case DATA_TSVH_MULTI:
        case DATA_TSVH_FLASH:
        case DATA_TSVH_OPERA:
        #ifdef USING_APP_MEM
            if (AppMem_Inc_Index_Send_2(&sRecTSVH, 1) == false)
            {
                //set lai timeout cho qua trinh luu phan B truoc khi doc tiep ban tin
                sAppMem.cPending_u8 = 0;  
                AppMem_Write_Data(sAppMem.RecMemType_u8, _MEM_DATA_TSVH_B, 0, 
                                  sMQTT.sDataFromMem.Data_a8, sMQTT.sDataFromMem.Length_u16, sRecTSVH.SizeRecord_u16);
            }
        #endif
            //Unmark mess 
            mUnmark_Mess_Share_Buff(); 
            break;
        case DATA_EVENT:
        #ifdef USING_APP_MEM
            if (AppMem_Inc_Index_Send_2(&sRecEvent, 1) == false)
            {
                //set lai timeout cho qua trinh luu phan B truoc khi doc tiep ban tin
                sAppMem.cPending_u8 = 0;  
                AppMem_Write_Data(sAppMem.RecMemType_u8, _MEM_DATA_EVENT_B, 0,
                                  sMQTT.sDataFromMem.Data_a8, sMQTT.sDataFromMem.Length_u16, sRecEvent.SizeRecord_u16);
            }
        #endif
            //Unmark mess 
            mUnmark_Mess_Share_Buff();
            break;
        case DATA_GPS:
        #ifdef USING_APP_MEM
            if (AppMem_Inc_Index_Send_2(&sRecGPS, 1) == false)
            {
                //set lai timeout cho qua trinh luu phan B truoc khi doc tiep ban tin
                sAppMem.cPending_u8 = 0;  
                AppMem_Write_Data(sAppMem.RecMemType_u8, _MEM_DATA_GPS_B, 0, 
                                  sMQTT.sDataFromMem.Data_a8, sMQTT.sDataFromMem.Length_u16, sRecGPS.SizeRecord_u16);
            }
        #endif
            //Unmark mess 
            mUnmark_Mess_Share_Buff();
            break;
        case SEND_SAVE_BOX_OK:
            sMQTT.aMARK_MESS_PENDING[sMQTT.MessType_u8] = FALSE; 
            
            if (sAppEthVar.pReq_Save_Box != NULL)
                sAppEthVar.pReq_Save_Box();
            break;
        case SEND_SHUTTING_DOWN:
            if (sAppEthVar.pReset_MCU_Imediate != NULL)
                sAppEthVar.pReset_MCU_Imediate();
            break;
        case SEND_UPDATE_FIRM_OK:
        case SEND_UPDATE_FIRM_FAIL:
            sMQTT.aMARK_MESS_PENDING[sMQTT.MessType_u8] = FALSE; 
            
            if (sAppEthVar.pReset_MCU_Imediate != NULL)
                sAppEthVar.pReset_MCU_Imediate();
            break;
        default:
            sMQTT.aMARK_MESS_PENDING[sMQTT.MessType_u8] = FALSE;  
            break;
    }
    
    sAppEthVar.cHardReset_u16 = 0;
    sAppEthVar.LandMarkConnMQTT_u32 = RtCountSystick_u32;
    
    fevent_enable(sEventAppEth, _EVENT_ETH_SEND_DATA);
    
    return 1;
}

static uint8_t _Cb_Pasv_Port_OK (sData *pData)
{
    if (pportc((char *) pData->Data_a8) == -1)
    {
        APP_LOG(TS_OFF, DBLEVEL_M, "Bad port syntax\r\n");
    }
    else
    {
        APP_LOG(TS_OFF, DBLEVEL_M, "Go Open Data Sock...\r\n ");
        
        sFTPvar.dsock_mode = PASSIVE_MODE;
        sFTPvar.dsock_state = DATASOCK_READY;
    }

    return 1;
}


static uint8_t _Cb_Get_OK (sData *pData)
{
    //bat dau cho lay firmware
    sFTPvar.DataGetStart_u8 = true;  

    return 1;
}


static uint8_t _Cb_Get_Size_OK (sData *pData)
{
    //
    sData strcheck;
    int Pos = -1;
    uint16_t PosFind = 0;
    uint32_t TempSize = 0;
    
    strcheck.Data_a8 = (uint8_t*) sEthCmd[_ETH_FTP_QUER_SIZE].sResp;
    strcheck.Length_u16 = strlen( sEthCmd[_ETH_FTP_QUER_SIZE].sResp );   

    Pos = Find_String_V2(&strcheck, pData);
    if (Pos >= 0)
    {
        PosFind = Pos + strlen( sEthCmd[_ETH_FTP_QUER_SIZE].sResp);
        //lay gia tri so sau do
        TempSize = UTIL_Get_Num_From_Str(pData, &PosFind);
        if (TempSize == 0xFFFFFFFF)
        {
            APP_LOG(TS_OFF, DBLEVEL_M, "u_app_eth: get size file error!\r\n ");
            return 0;
        }
        
        sAppEthVar.sUpdateVar.FwSizeGet_u32 = TempSize;
    }
    
    return 1;
}


static uint8_t _Cb_List_OK (sData *pData)
{

    return 1;
}



static uint8_t _Cb_URC_Ready (sData *pData)
{
    //bat dau truyen lenh
    sFTPvar.CtrlReady_u8 = true;  

    return 1;
}

static uint8_t _Cb_URC_Server (sData *pData)
{
    uint8_t var = 0;
    int PosFind = 0;
    
    for (var = REQUEST_RESET; var < END_MQTT_RECEI; ++var)
    {
        PosFind = Find_String_V2 ((sData*) &sMark_MessageRecei_Type[var].sKind, pData);
        
        if ((PosFind >= 0) && (sMark_MessageRecei_Type[var].CallBack != NULL))
            sMark_MessageRecei_Type[var].CallBack(pData, PosFind);
    }

    return 1;
}


static uint8_t _fMQTT_Connect (void)
{
    uint8_t aCLIENT[40] = {0};
    sData ClientEth = {aCLIENT, 0};
    sData sEth = {(uint8_t *) "Eth", 3};
      
    sAppEthVar.Status_u8 = _ETH_MQTT_CONNECT;
    
    Copy_String_2(&ClientEth, sMQTT.sModemID);
    Copy_String_2(&ClientEth, &sEth);
    
    
    mConnect_MQTT( (char *) ClientEth.Data_a8,
                   (char *) sAppEthVar.sServer.sMQTTUserName->Data_a8, 
                   (char *)sAppEthVar.sServer.sMQTTPassword->Data_a8,
                   *sAppEthVar.sServer.MQTTKeepAlive_u32 );

    //Send via Ethernet
    UTIL_Printf (DBLEVEL_M,  sMQTT.str.Data_a8, sMQTT.str.Length_u16);
    
    if (send (SOCKET_DEMO, sMQTT.str.Data_a8, sMQTT.str.Length_u16) == -1)
        return false;
    
    sMQTT.Status_u8 = PENDING;
    
    return true;
}

static uint8_t _fMQTT_Subcribe (void)
{
    sAppEthVar.Status_u8 = _ETH_MQTT_SUBCRIBE;
    
    mSubcribe_MQTT();
    //Send via Ethernet
    UTIL_Printf (DBLEVEL_M,  sMQTT.str.Data_a8, sMQTT.str.Length_u16);
    
    if (send (SOCKET_DEMO, sMQTT.str.Data_a8, sMQTT.str.Length_u16) == -1)
        return false;
    
    sMQTT.Status_u8 = PENDING;
    
    return true;
}

static uint8_t _fMQTT_Publish (void)
{
    //Send qua ethernet
    UTIL_Printf_Str(DBLEVEL_M, "\r\nu_app_eth: message tranfer: \r\n");
    UTIL_Printf (DBLEVEL_M, sMQTT.str.Data_a8, sMQTT.str.Length_u16);
    UTIL_Printf_Str(DBLEVEL_M, "\r\n");
                    
    if (send (SOCKET_DEMO, sMQTT.str.Data_a8, sMQTT.str.Length_u16) == -1)
        return false;
    
    return true;
}


static uint8_t _fSet_User (void)
{
    char dat[50]={0};
    
    sprintf(dat,"USER %s\r\n", sAppEthVar.sUpdate.pFirmUser->Data_a8);   // saoviet
    
    APP_LOG(TS_OFF, DBLEVEL_M, "u_ftp: set user: \"%s\"", dat);

    send(CTRL_SOCK, (uint8_t *)dat, strlen(dat));
    
    return 1;
}


static uint8_t _fSet_Pass (void)
{
    char dat[50]={0};

    sprintf(dat,"PASS %s\r\n", sAppEthVar.sUpdate.pFirmPass->Data_a8);   // sv123@
    APP_LOG(TS_OFF, DBLEVEL_M,"u_ftp: set pasword: \"%s\"", dat);
    
    send(CTRL_SOCK, (uint8_t *)dat, strlen(dat));
    
    return 1;
}

static uint8_t _fSet_Type_ASCII (void)
{
    char dat[50]={0};

    sprintf(dat,"TYPE %c\r\n", TransferAscii);
    APP_LOG(TS_OFF, DBLEVEL_M,"u_ftp: type: \"%s\"", dat);
    
    sFTPvar.type = ASCII_TYPE;
    send(CTRL_SOCK, (uint8_t *)dat, strlen(dat));
    
    return 1;
}

static uint8_t _fSet_Type_BIN (void)
{
    char dat[50]={0};

    sprintf(dat,"TYPE %c\r\n", TransferBinary);
    APP_LOG(TS_OFF, DBLEVEL_M,"u_ftp: type: \"%s\"", dat);
    
    sFTPvar.type = ASCII_TYPE;
    send(CTRL_SOCK, (uint8_t *)dat, strlen(dat));
    
    return 1;
}


static uint8_t _fQuery_List (void)
{
    char dat[50]={0};

    sprintf(dat,"LIST\r\n");
    APP_LOG(TS_OFF, DBLEVEL_M,"u_ftp: List: \"%s\"", dat);
    send(CTRL_SOCK, (uint8_t *)dat, strlen(dat));
                            
    return 1;
}

static uint8_t _fQuery_Size_File (void)
{
    char dat[50]={0};

    if (sAppEthVar.sUpdate.pFirmPath->Length_u16 == 0)
    {
        sprintf(dat,"SIZE /%s\r\n", (char *) sAppEthVar.sUpdate.pFirmName->Data_a8);
    } else
    {
        sprintf(dat,"SIZE /%s/%s\r\n", (char *) sAppEthVar.sUpdate.pFirmPath->Data_a8,
                                        (char *) sAppEthVar.sUpdate.pFirmName->Data_a8);
    }
    
    APP_LOG(TS_OFF, DBLEVEL_M, "u_ftp: size file: \"%s\"", dat);
    
    send(CTRL_SOCK, (uint8_t *)dat, strlen(dat));
                            
    return 1;
}

static uint8_t _fSet_CWD (void)
{
    char dat[50]={0};
    
    sprintf(dat,"CWD /%s\r\n", (char *) sAppEthVar.sUpdate.pFirmPath->Data_a8);
    APP_LOG(TS_OFF, DBLEVEL_M, "> path file: %s\r\n", dat);
    
    send(CTRL_SOCK, (uint8_t *)dat, strlen(dat));
    
    return 1;
}



static uint8_t _fSet_Pasv_Port (void)
{
    wiz_NetInfo gWIZNETINFO;
    char dat[50]={0};
    uint16_t  local_port = 35000;
    
    if (sFTPvar.dsock_mode == PASSIVE_MODE)
    {
        sprintf(dat, "PASV\r\n");
    } else
    {                      
        ctlnetwork(CN_GET_NETINFO, (void*) &gWIZNETINFO);   
        
        sprintf(dat, "PORT %d,%d,%d,%d,%d,%d\r\n",
                    gWIZNETINFO.ip[0], gWIZNETINFO.ip[1],
                    gWIZNETINFO.ip[2], gWIZNETINFO.ip[3], 
                    (uint8_t)(local_port>>8), (uint8_t)(local_port&0x00ff));
    }
    
    APP_LOG(TS_OFF, DBLEVEL_M, "u_ftp: %s", dat);
    
    send (CTRL_SOCK, (uint8_t *)dat, strlen(dat));
                            
    return 1;
}



static uint8_t _fSet_Get_File (void)
{
    char dat[80]={0};
    
    sprintf(dat,"RETR /%s/%s\r\n", (char *) sAppEthVar.sUpdate.pFirmPath->Data_a8,
                                        (char *) sAppEthVar.sUpdate.pFirmName->Data_a8);
    
    APP_LOG(TS_OFF, DBLEVEL_M, "u_ftp: \"%s\"", dat);
    
    send(CTRL_SOCK, (uint8_t *)dat, strlen(dat));
                            
    return 1;
}


//end cb cmd ethernet

void AppEth_Init_Default (void)
{
    sAppEthVar.Status_u8 = _ETH_START;
    sAppEthVar.PHYstatus_u8 = false;
    sAppEthVar.ReConnect_u8 = false;
    sAppEthVar.Retry_u8 = 0;
}


void AppEth_FTP_Init_Default (void)
{
    sFTPvar.DataSockConnected_u8 = false;
    sFTPvar.DataGetStart_u8 = false;
    sFTPvar.CtrlReady_u8 = false;
    sFTPvar.dsock_state = DATASOCK_IDLE;

    sFTPvar.LandMark_u32 = RtCountSystick_u32;
    sFTPvar.Retry_u8++;
}



int pportc(char * arg)
{
	int i;
	char* tok=0;
    
	strtok(arg,"(");
    
	for (i = 0; i < 4; i++)
	{
		if (i==0) 
            tok = strtok(NULL,",\r\n");
		else	 
            tok = strtok(NULL,",");
        
		sFTPvar.RemoteIP[i] = (uint8_t)atoi(tok);
        
		if (!tok)
        {
			APP_LOG(TS_OFF, DBLEVEL_M, "bad pport : %s\r\n", arg);
			return -1;
		}
	}
    
	sFTPvar.RemotePort = 0;
	for (i = 0; i < 2; i++)
    {
		tok = strtok(NULL,",\r\n");
        
		sFTPvar.RemotePort <<= 8;
		sFTPvar.RemotePort += atoi(tok);
        
		if (!tok)
        {
			APP_LOG(TS_OFF, DBLEVEL_M, "bad pport : %s\r\n", arg);
			return -1;
		}
	}
    
	APP_LOG(TS_OFF, DBLEVEL_M, "ip : %d.%d.%d.%d, port : %d\r\n", 
                    sFTPvar.RemoteIP[0], sFTPvar.RemoteIP[1],
                    sFTPvar.RemoteIP[2], sFTPvar.RemoteIP[3], sFTPvar.RemotePort);
	
    return 0;
}



uint8_t AppEth_FTP_DataSock_Run (void)
{       
    long ret = 0;
	uint32_t remain_datasize = 0;
    uint32_t recv_byte = 0;
    uint32_t LandMark_u32 = RtCountSystick_u32;
    
    if (sFTPvar.DataGetStart_u8 == true)
    {
        if ((remain_datasize = getSn_RX_RSR(DATA_SOCK)) > 0)
        {
            while (1)
            {
                if (remain_datasize > DATA_BUF_SIZE)
                    recv_byte = DATA_BUF_SIZE;
                else
                    recv_byte = remain_datasize;
                
                ret = recv(DATA_SOCK, aRECEIVE_DATA, recv_byte);
                
                if (ret < 0)
                    return false;
                
                UTIL_Printf( DBLEVEL_M, aRECEIVE_DATA, ret);
                
                if (AppEth_Save_Firmware(aRECEIVE_DATA, ret) == false)
                    return false;    
                
                remain_datasize -= ret;
                
                if (remain_datasize <= 0)
                    break;
                
                if (Check_Time_Out(LandMark_u32, 5000) == true)
                    return false;
            }
        } 
    }
    
    return true;
}

void AppEth_Default_Struct_UpdateVar (void)
{
    //Xoa vung nho update
    Erase_Firmware(ADDR_UPDATE_PROGRAM, FIRMWARE_SIZE_PAGE); 
    
    //Set default cho cac bien
    sAppEthVar.sUpdateVar.Addr_u32 = ADDR_UPDATE_PROGRAM;
    sAppEthVar.sUpdateVar.HeaderValid_u8 = false;
    sAppEthVar.sUpdateVar.cPacket_u8 = 0;
    sAppEthVar.sUpdateVar.Total_u8 = 0;
    sAppEthVar.sUpdateVar.FwSizeGet_u32 = 0;
    sAppEthVar.sUpdateVar.CrcFile_u8 = 0;
    sAppEthVar.sUpdateVar.CrcCalcu_u8 = 0;
    sAppEthVar.sUpdateVar.Status_u8 = 0;
}

uint8_t AppEth_Save_Firmware (uint8_t *pData, uint16_t length)
{
    static uint8_t aTEMP_BYTE[8] = {0};
    static uint8_t cByte = 0;
    sData strHeader;
    int Pos = -1;
    sData sSource = {pData, length};
    uint16_t i = 0, nByteAdd = 0;
    uint8_t aTEMP_DATA[8] = {0};
    
    if (sAppEthVar.sUpdateVar.cPacket_u8 == 0)
    {
        UTIL_MEM_set(aTEMP_BYTE, 0, 8);
        cByte = 0;
        
        //Check header fw sao viet
        strHeader.Data_a8 = sAppEthVar.pGet_Fw_Version()->Data_a8;
        strHeader.Length_u16 = LENGTH_HEADER_FW;

        Pos = Find_String_V2((sData*)&strHeader, &sSource);  
        if (Pos >= 0)
            sAppEthVar.sUpdateVar.HeaderValid_u8 = true;
          
        if (sAppEthVar.sUpdateVar.HeaderValid_u8 == false)  
        {
            AppEth_Finish_Update(__FAIL_HEADER);
            
            return false;
        }
        
        //Lay byte cuoi cung ra: byte 32 cua Packdau tien. Sau header 31 byte
        sAppEthVar.sUpdateVar.CrcFile_u8 = *(sSource.Data_a8 + 31);
        //Fix string data
        sSource.Data_a8 += 32;
        sSource.Length_u16 -= 32;
        
        sAppEthVar.sUpdateVar.Total_u8 += 32;
    }

    //Cacal Crc
    for (i = 0; i < sSource.Length_u16; i++)
        sAppEthVar.sUpdateVar.CrcCalcu_u8 += *(sSource.Data_a8 + i);
              
    //Increase Count Packet
    sAppEthVar.sUpdateVar.cPacket_u8++;
    sAppEthVar.sUpdateVar.Total_u8 += sSource.Length_u16;
    
    //Neu so byte lon hon size: cong them cac byte le
    if ( sAppEthVar.sUpdateVar.Total_u8 >= sAppEthVar.sUpdateVar.FwSizeGet_u32 )
    {
        if ((sAppEthVar.sUpdateVar.Total_u8 % FLASH_BYTE_WRTIE) != 0)
        {
            nByteAdd = 2 * FLASH_BYTE_WRTIE - (sAppEthVar.sUpdateVar.Total_u8 % FLASH_BYTE_WRTIE);   //so byte de chan
            for (i = 0; i < nByteAdd; i++)
                *(sSource.Data_a8 + sSource.Length_u16++) = FLASH_BYTE_EMPTY;
            
            sAppEthVar.sUpdateVar.Total_u8 += nByteAdd;
        }
    }

    while (sSource.Length_u16 >= FLASH_BYTE_WRTIE)
    {
        //Lay file le truoc do truoc
        if (cByte != 0)
        {
            for (i = 0; i < cByte; i++)
                aTEMP_DATA[i] = aTEMP_BYTE[i];
            
            UTIL_MEM_set(aTEMP_BYTE, 0, 8);
        }
        
        for (i = cByte; i < FLASH_BYTE_WRTIE; i++)
        {
            aTEMP_DATA[i] = *(sSource.Data_a8++);
        }
        
        sSource.Length_u16 -= (FLASH_BYTE_WRTIE - cByte);
        cByte = 0;
        
        if(OnchipFlashWriteData(sAppEthVar.sUpdateVar.Addr_u32, &aTEMP_DATA[0], FLASH_BYTE_WRTIE) != HAL_OK)
        {
            AppEth_Finish_Update(__FAIL_FLASH);
            return false;
        } else
        {
            sAppEthVar.sUpdateVar.Addr_u32 += FLASH_BYTE_WRTIE;
        }
    }
    
    //copy toan bo byte le vao buff temp
    for (i = 0; i < sSource.Length_u16; i++)
        aTEMP_BYTE[cByte++] = *(sSource.Data_a8 + i);
    
    return true;
}


void AppEth_Finish_Update (uint8_t status)
{
    switch (status)
    {
        case __SUCCESSFUL:
            // Update thanh cong
        #ifdef USING_INTERNAL_MEM
            Erase_Firmware(ADDR_FLAG_HAVE_NEW_FW, 1);
            //Ghi Flag update va Size firm vao Inflash
            HAL_FLASH_Unlock();
            HAL_Delay(10);
            
        #if (FLASH_BYTE_WRTIE == 8)
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 
                              ADDR_FLAG_HAVE_NEW_FW, 0xAA);
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 
                              ADDR_FLAG_HAVE_NEW_FW + FLASH_BYTE_WRTIE, sAppEthVar.sUpdateVar.Total_u8);
        #else
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 
                              ADDR_FLAG_HAVE_NEW_FW, 0xAA);
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 
                              ADDR_FLAG_HAVE_NEW_FW + FLASH_BYTE_WRTIE, sAppEthVar.sUpdateVar.Total_u8);
        #endif
            
            HAL_Delay(10);
            HAL_FLASH_Lock();
        #endif
            
            sAppEthVar.sUpdateVar.Status_u8 = status;
            sMQTT.aMARK_MESS_PENDING[SEND_UPDATE_FIRM_OK] = TRUE;
            //Init connect again
            *sAppEthVar.pModeConnNow_u8 = *sAppEthVar.pModeConnLast_u8;
            fevent_active(sEventAppEth, _EVENT_ETH_HARD_RESET);
            break;
        case __FAIL_CONNECT:
        case __FAIL_LENGTH:
        case __FAIL_HEADER:
        case __FAIL_FLASH:
            sAppEthVar.sUpdateVar.Status_u8 = status;
            sMQTT.aMARK_MESS_PENDING[SEND_UPDATE_FIRM_FAIL] = TRUE;
            *sAppEthVar.pModeConnNow_u8 = *sAppEthVar.pModeConnLast_u8;
            fevent_active(sEventAppEth, _EVENT_ETH_HARD_RESET);
            break;
        default:
            break;
    }
}


