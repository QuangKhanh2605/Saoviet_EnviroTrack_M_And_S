/*
 * siml506.c
 *
 *  Created on: Feb 16, 2022
 *      Author: chien
 */


#include "string.h"

#include "user_l511.h"
#include "user_string.h"

     
/* ========= handler Sim driver ============*/
/* define static functions */
static uint8_t at_callback_success(sData *uart_string);

/*========== Callback AT ===================*/
static uint8_t _CbAT_SET_APN (sData *uart_string);
static uint8_t _CbAT_APN_ACTIVE (sData *uart_string);
static uint8_t _CbAT_OPEN_TCP_2 (sData *uart_string);
static uint8_t _CbAT_GET_CLOCK (sData *uart_string);
static uint8_t _CbAT_CHECK_RSSI(sData *uart_string);
//File sys callback
static uint8_t _CbSYS_DEL_FILE(sData *uart_string);
//FTP Callback
static uint8_t _CbFTP_SEND_IP(sData *uart_string);
static uint8_t _CbFTP_GET_FILE1(sData *uart_string);
static uint8_t _CbFTP_GET_FILE2(sData *uart_string);
static uint8_t _CbFTP_GET_FILE3(sData *uart_string);
static uint8_t _CbFTP_READ_1(sData *uart_string);
static uint8_t _CbFTP_READ_2(sData *uart_string);
//Callback HTTP
static uint8_t _CbHTTP_SETURL_1(sData *uart_string);
static uint8_t _CbHTTP_SETURL_2(sData *uart_string);

static uint8_t _CbHTTP_FIRM_LENGTH(sData *uart_string);
static uint8_t _CbHTTP_READ_1(sData *uart_string);
static uint8_t _CbHTTP_READ_2(sData *uart_string);

static uint8_t _CbURC_CLOSED (sData *uart_string);

/*============= Var struct ====================*/
//Block step control SIM
uint8_t aL511_BLOCK_INIT[9] = {
	_L511_AT_ECHO,
    _L511_AT_ECHO,
	_L511_AT_AT,
//	_L511_AT_MAN_LOG,
    _L511_AT_BAUD_RATE,
    _L511_AT_GET_IMEI,
   
//    _L511_CALLING_LINE,
//    _L511_SMS_FORMAT_TEXT,
//    _L511_SMS_NEW_MESS_IND,
    
//	_L511_NET_RA_AUTO,
	_L511_NET_CEREG,
	
	_L511_AT_GET_ID,
	_L511_AT_CHECK_RSSI,
	_L511_AT_CHECK_SIM,  
};

  
#ifdef MQTT_PROTOCOL
uint8_t aL511_BLOCK_NETWORK[6] = 
{
	_L511_NET_CHECK_ATTACH,
//    _L511_SMS_READ_UNREAD,
//    _L511_SMS_DEL_ALL,
    _L511_AT_CHECK_RSSI,
	_L511_NET_SYN_TIME_ZONE,
	_L511_NET_GET_RTC,
	_L511_AT_SET_APN_1,
    _L511_AT_SET_APN_2,
};
#else
uint8_t aL511_BLOCK_NETWORK[12] = 
{
	_L511_NET_CHECK_ATTACH,
    _L511_AT_CHECK_RSSI,
    _L511_SMS_READ_UNREAD,
    _L511_SMS_DEL_ALL,
    _L511_AT_CHECK_RSSI,
	_L511_NET_SYN_TIME_ZONE,
	_L511_NET_GET_RTC,
    
    _L511_AT_SET_APN_1,
    _L511_AT_SET_APN_2,
    _L511_AT_APN_AUTH_1,
    _L511_AT_APN_AUTH_2,
    _L511_AT_TEST_0,
}; 
#endif


#ifdef USING_TRANSPARENT_MODE
    uint8_t aL511_BLOCK_CONNECT[13] = 
    {
        _L511_TCP_CLOSE_1,
        _L511_TCP_CLOSE_2,
        _L511_TCP_NETCLOSE,
        _L511_TCP_TRANS_SETUP, //1
        _L511_TCP_TRANS,       //2
        _L511_TCP_NETOPEN,     //3
        _L511_TCP_GET_IP_SIM,      //4
        _L511_TCP_CONNECT_1,   //5
        _L511_TCP_CONNECT_2,   //6
        
        _L511_MQTT_CONN_1,        //7
        _L511_MQTT_CONN_2,        //8

        _L511_MQTT_SUB_1,         //9
        _L511_MQTT_SUB_2,         //10
    };

    uint8_t aL511_BLOCK_PUB[2] =
    {
        _L511_MQTT_PUB_1,
        _L511_MQTT_PUB_2,
    };

    uint8_t aL511_BLOCK_PUB_FB[2] =
    {
        _L511_MQTT_PUB_FB_1,
        _L511_MQTT_PUB_FB_2,
    };
    
    uint8_t aL511_BLOCK_PING[2] =
    {
        _L511_MQTT_PUB_FB_1,
        _L511_MQTT_PUB_PING,
    };
#else
    #ifdef MQTT_PROTOCOL
        uint8_t aL511_BLOCK_CONNECT[17] = 
        {
            _L511_TCP_CLOSE_1,
            _L511_TCP_CLOSE_2,
            _L511_TCP_NETCLOSE,
            _L511_TCP_TRANS_SETUP, //1
            _L511_TCP_TRANS,       //2
            _L511_TCP_NETOPEN,     //3
            _L511_TCP_GET_IP_SIM,      //4
            _L511_TCP_CONNECT_1,   //5
            _L511_TCP_CONNECT_2,   //6
            
            _L511_MQTT_CONN_1,        //7
            _L511_TCP_SEND_CMD_1,
            _L511_TCP_SEND_CMD_2,
            _L511_MQTT_CONN_2,        //8

            _L511_MQTT_SUB_1,         //9
            _L511_TCP_SEND_CMD_1,
            _L511_TCP_SEND_CMD_2,
            _L511_MQTT_SUB_2,         //10
        };
    #endif
        
    #ifdef HTTP_PROTOCOL
        uint8_t aL511_BLOCK_CONNECT[6] = 
        {          
            _L511_AT_ACTIVE_APN,
            _L511_AT_GET_IP_APN,
               
            _L511_HTTP_CLOSE,
            _L511_HTTP_OPEN,
            _L511_HTTP_SET_URL_1,
            _L511_HTTP_SET_URL_2,
        };
        
        uint8_t aL511_BLOCK_HTTP_POST[9] = 
        {
//            _L511_HTTP_CLOSE,
//            _L511_HTTP_OPEN,
//            _L511_HTTP_SET_URL_1,
//            _L511_HTTP_SET_URL_2,
                        
             _L511_HTTP_SET_HEADER_1,
            _L511_HTTP_SET_HEADER_2,

            _L511_HTTP_REQ_POST_1,
            
            _L511_HTTP_POST_DATA_1,
            _L511_HTTP_POST_DATA_2,
            _L511_HTTP_POST_DATA_3,
            
            _L511_HTTP_POST_SEND,
    
            _L511_HTTP_POST_DAT_END,
            _L511_HTTP_POST_SEND_2,
            
//            //Send type 2
//            _L511_HTTP_SET_HEADER_1,
//            _L511_HTTP_SET_HEADER_2,
//           
//            _L511_HTTP_POST_DATA_EX1,
//            _L511_HTTP_POST_DATA_EX2,
//            _L511_HTTP_REQ_POST_3,            
        };
    #endif
        
    #ifdef HTTPS_PROTOCOL        
        
        uint8_t aL511_BLOCK_CONNECT[2] = 
        {
            _L511_AT_ACTIVE_APN,
            _L511_AT_GET_IP_APN,
        };
        
        uint8_t aL511_BLOCK_HTTP_POST[5] = 
        {
            _L511_HTTPS_ACQUIRE,
            _L511_HTTPS_OPEN_SES_1,
            _L511_HTTPS_OPEN_SES_2,
            _L511_HTTPS_CLOSE_SES,
            _L511_HTTPS_STOP,
        };
    #endif

    uint8_t aL511_BLOCK_PUB[4] =
    {
        _L511_MQTT_PUB_1,
        _L511_TCP_SEND_CMD_1,
        _L511_TCP_SEND_CMD_2,
        _L511_TCP_SEND_CMD_3,
    };

    uint8_t aL511_BLOCK_PUB_FB[4] =
    {
        _L511_MQTT_PUB_FB_1,
        _L511_TCP_SEND_CMD_1,
        _L511_TCP_SEND_CMD_2,
        _L511_MQTT_PUB_FB_2,
    };
    
    uint8_t aL511_BLOCK_PING[4] =
    {
        _L511_MQTT_PUB_FB_1,
        _L511_TCP_SEND_CMD_1,
        _L511_TCP_SEND_CMD_2,
        _L511_MQTT_PUB_PING,
    };
    
#endif


uint8_t aL511_BLOCK_HTTP_HEAD[2] = 
{
    _L511_HTTP_SET_HEADER_1,
    _L511_HTTP_SET_HEADER_2,
};

uint8_t  aL511_BLOCK_CSQ[3] = 
{
    _L511_TCP_OUT_DATAMODE,
    _L511_AT_CHECK_RSSI,
    _L511_TCP_IN_DATAMODE,
};

#ifdef MQTT_PROTOCOL
    uint8_t aL511_BLOCK_DIS_CONN[4] = 
    {
        _L511_TCP_OUT_DATAMODE,
        _L511_TCP_CLOSE_1,
        _L511_TCP_CLOSE_2,
        _L511_TCP_NETCLOSE,
    };
#endif
    
#ifdef HTTP_PROTOCOL
    uint8_t aL511_BLOCK_DIS_CONN[1] = 
    {
        _L511_HTTP_CLOSE,
    };
#endif

#ifdef HTTPS_PROTOCOL
    uint8_t aL511_BLOCK_DIS_CONN[1] = 
    {
        _L511_HTTPS_STOP,
    };
#endif
    

uint8_t aL511_BLOCK_CLOCK[3] = 
{
    _L511_TCP_OUT_DATAMODE,
    _L511_NET_GET_RTC,
    _L511_TCP_IN_DATAMODE,
};

uint8_t aL511_BLOCK_HTTP_INIT[17] = 
{
    _L511_NET_CHECK_ATTACH,
    _L511_AT_CHECK_RSSI,
    _L511_AT_SET_APN_1,
    _L511_AT_SET_APN_2,
    _L511_AT_TEST_0,
    _L511_AT_GET_IP_APN,
    _L511_AT_ACTIVE_APN,
    _L511_AT_GET_IP_APN,
    
    _L511_TCP_PING_TEST1,
    _L511_TCP_PING_TEST2,
    _L511_HTTP_CLOSE,
    _L511_HTTP_OPEN,
    _L511_HTTP_SET_URL_1,
    _L511_HTTP_SET_URL_2,
    _L511_HTTP_RECV_LOCAL,
    _L511_HTTP_REQUEST_GET,
    _L511_HTTP_LENGTH,
};

uint8_t aL511_BLOCK_HTTP_READ[2] = 
{
    _L511_HTTP_READ_1,
    _L511_HTTP_READ_2,
};



//Read SMS
uint8_t aL511_BLOCK_READ_SMS[2] = 
{
    _L511_TCP_OUT_DATAMODE,
//    _L511_SMS_READ_UNREAD,
//    _L511_SMS_DEL_ALL,
    _L511_TCP_IN_DATAMODE,
};  


uint8_t aL511_BLOCK_SEND_SMS[2] = 
{
    _L511_TCP_OUT_DATAMODE,
    _L511_TCP_IN_DATAMODE,
};  

uint8_t aL511_BLOCK_CALLING[2] = 
{
    _L511_TCP_OUT_DATAMODE,
    _L511_TCP_IN_DATAMODE,
};  



/*========= Struct List AT Sim ==================*/
static char DataConack_8a[] = { MY_CONNACK, 0x02, 0x00 };		 //20020000
static char DataSuback_8a[] = { MY_SUBACK, 0x04, 0x00};          //9004000a00
static char DataPuback_8a[] = { MY_PUBACK, 0x02, 0x00};	 //40020001  , 0x1A 
static char DataPingResp_8a[] = { MY_PINGRESP, 0x00};	 // Ð<00>

const sCommand_Sim_Struct aSimL511Step[] =
{  
    //Cmd Pre Init
	{	_L511_AT_AT,			    at_callback_success,        "OK", 			    "AT\r" 	        },
	{	_L511_AT_ECHO,			    at_callback_success,        "OK", 			    "ATE0\r" 	 	},
	{   _L511_AT_MAN_LOG,    	    at_callback_success,        "OK",	            "AT+MLOGK=5\r"  },
    //Cmd Init
    {   _L511_AT_BAUD_RATE, 		at_callback_success,        "OK",			    "AT+IPR=115200\r"	},
    {	_L511_AT_GET_ID,			Sim_Common_Get_Sim_ID,      "CCID:", 	        "AT+ICCID\r" 	},
	{	_L511_AT_CHECK_RSSI,		_CbAT_CHECK_RSSI,           "+CSQ: ", 		    "AT+CSQ\r"	 	},
	{   _L511_AT_GET_IMEI, 		    Sim_Common_Get_Imei,        "OK", 		        "AT+CGSN\r"		},
	{	_L511_AT_CHECK_SIM,		    Sim_Common_CPIN_Ready,      "OK", 			    "AT+CPIN?\r"   	},
    {	_L511_AT_SET_APN_1,         _CbAT_SET_APN,              NULL,	            "AT+QICSGP=1,1,\""		},// AT+CGDCONT=1,"IP","V3G2057"  - "AT+CGDCONT=1,\"IP\",\"",19}
	{	_L511_AT_SET_APN_2,         at_callback_success,        "OK",	            "\"\r"         }, 

    {   _L511_AT_TEST_0, 		    at_callback_success,        "OK",               "AT+CGATT=1\r" 	},
    {   _L511_AT_GET_IP_APN,        _CbAT_APN_ACTIVE,           "+CGPADDR: 1,",     "AT+CGPADDR=1\r"	},
    {   _L511_AT_ACTIVE_APN, 	    at_callback_success,        "OK",               "AT+CGACT=1,1\r"	},
//   //SMS
//    {   _L511_SMS_FORMAT_TEXT,	at_callback_success,        "OK",               "AT+CMGF=1\r"	},  
//    {   _L511_SMS_NEW_MESS_IND,	at_callback_success,        "OK",               "AT+CNMI=2,1\r"	    },
//    {   _L511_SMS_READ_UNREAD,  	at_callback_success,        "\r\nOK",           "AT+CMGL=\"REC UNREAD\"\r"	    },
//    {   _L511_SMS_READ_ALL,  	    at_callback_success,        "\r\nOK",           "AT+CMGL=\"ALL\"\r"	        },
//    {   _L511_SMS_DEL_ALL,	    at_callback_success,        "OK",               "AT+CMGD=1,4\r"	},
//    //Calling
//    {   _L511_CALLING_LINE, 		at_callback_success,        "OK",               "AT+CLIP=1\r"   },
//    //Cmd Set 2G 3G 4G
//	{	_L511_NET_RA_AUTO,          at_callback_success,        "OK",			    "AT+CNMP=2\r"	},
//	{   _L511_NET_RA_2G,            at_callback_success,        "OK",			    "AT+CNMP=13\r"  },
//	{   _L511_NET_RA_3G,            at_callback_success,        "OK",			    "AT+CNMP=14\r"  },
//	{  	_L511_NET_RA_4G,            at_callback_success,        "OK",			    "AT+CNMP=38\r"  },
//
//	{	_L511_NET_CHECK_RA,         at_callback_success,        "+CNMP:",       	"AT+CNMP?\r"    },
//	{	_L511_NET_CHECK_BAND,       at_callback_success,        "+CNBP:",           "AT+CNBP?\r"    },
//	{	_L511_NET_CHECK_CSPI,       at_callback_success,        "+CPSI:",           "AT+CPSI?\r"    },
    //Cmd Check Network
	{	_L511_NET_CEREG, 	        at_callback_success,        "OK",			    "AT+CEREG=0\r"	},
	{	_L511_NET_CHECK_ATTACH,	    Sim_Common_Check_Attached,  "OK",	            "AT+CGATT?\r"	},
    //Cmd Config
	{	_L511_NET_SYN_TIME_ZONE,    at_callback_success,        "OK",			    "AT+CTZR=1\r"	},
	{	_L511_NET_GET_RTC, 		    _CbAT_GET_CLOCK,            "+CCLK:",		    "AT+CCLK?\r"	},
    //Cmd Data mode and command mode
#ifdef USING_TRANSPARENT_MODE
    {   _L511_TCP_OUT_DATAMODE, 	Sim_Common_Non_Tranparent,  NULL,               "+++" 			}, 
    {   _L511_TCP_TRANS, 		    Sim_Common_Tranparent,      "OK",               "AT+CIPMODE=1\r" 	 	}, 
    {   _L511_TCP_IN_DATAMODE,      Sim_Common_Tranparent,      "CONNECT",          "ATO\r"		    },
#else
    {   _L511_TCP_OUT_DATAMODE, 	Sim_Common_Non_Tranparent,  NULL, 			    NULL            }, 
    {   _L511_TCP_TRANS, 		    Sim_Common_Tranparent,      "OK",               "AT+CIPMODE=0\r", 		}, 
    {   _L511_TCP_IN_DATAMODE,      Sim_Common_Tranparent,      NULL,		        NULL            },
#endif   
    
    {   _L511_TCP_TRANS_SETUP,      at_callback_success,        "OK",               "AT+MCIPCFGPL=0,1,0,0,0\r",    	}, 
    //Cmd TCP 
    {   _L511_TCP_GET_IP_SIM,	    at_callback_success,        "+IPADDR:",         "AT+IPADDR\r"   },         
    {	_L511_TCP_NETOPEN,	        at_callback_success,        "+NETOPEN:SUCCESS", "AT+NETOPEN\r"	},
    {   _L511_TCP_NETCLOSE,         at_callback_success,        "OK",               "AT+NETCLOSE\r"	},

    {	_L511_TCP_CLOSE_1,	 	    Sim_Common_TCP_Close,       NULL,	            "AT+CIPCLOSE="  	},
    {	_L511_TCP_CLOSE_2,	 	    at_callback_success,        "OK",	            "\r"  	}, 

    {	_L511_TCP_CONNECT_1,	    Sim_Common_TCP_Open_1,      NULL,               "AT+CIPOPEN="  	},
#ifdef USING_TRANSPARENT_MODE
    {	_L511_TCP_CONNECT_2,	    _CbAT_OPEN_TCP_2,           "CONNECT",          ",0\r"          }, 
#else
    {	_L511_TCP_CONNECT_2,	    _CbAT_OPEN_TCP_2,           "+CIPOPEN:",        ",0\r"          },
#endif        
    {   _L511_TCP_SEND_CMD_1, 	    Sim_Common_TCP_Send_1,      NULL,               "AT+CIPSEND="  	},
    {   _L511_TCP_SEND_CMD_2, 	    Sim_Common_TCP_Send_2,      ">",                "\r"	 		},
    {   _L511_TCP_SEND_CMD_3, 	    Sim_Common_MQTT_Pub_2,      "+CIPSEND:SUCCESS",	NULL	        },
    
    //Mqtt AT
	{   _L511_MQTT_CONN_1, 		    Sim_Common_MQTT_Conn_1,     NULL,	            NULL            },
	{   _L511_MQTT_CONN_2, 		    Sim_Common_MQTT_Conn_2,     DataConack_8a,      NULL       	    },  //4,"20020000"

	{   _L511_MQTT_SUB_1, 		    Sim_Common_MQTT_Sub_1,      NULL,	            NULL        	},
	{   _L511_MQTT_SUB_2, 		    Sim_Common_MQTT_Sub_2,      DataSuback_8a,      NULL  			},

	{   _L511_MQTT_PUB_1, 		    Sim_Common_MQTT_Pub_1,      NULL,				NULL           	},
	{   _L511_MQTT_PUB_2, 		    Sim_Common_MQTT_Pub_2,      NULL,	   			NULL        	},  //"4,40020001"  "SEND OK"

	{   _L511_MQTT_PUB_FB_1,        Sim_Common_MQTT_Pub_1,      NULL,				NULL          	},
	{   _L511_MQTT_PUB_FB_2, 	    Sim_Common_MQTT_Pub_2,      DataPuback_8a,      NULL   			},  // 4,"40020001"
    
    {   _L511_MQTT_PUB_PING, 	    Sim_Common_MQTT_Pub_2,      DataPingResp_8a,    NULL   			},  
    //Ping
    {   _L511_TCP_PING_TEST1,       Sim_Common_Ping,            NULL,               "AT+MPING=\""  	},    
    {   _L511_TCP_PING_TEST2,       at_callback_success,        "+MPING:3",         "\",1,4\r"     	},
    
    // File	 
    {   _L511_SYS_DEL_FILE_1,       _CbSYS_DEL_FILE,            NULL,               "AT+MFSDEL=\"" 	},
    {   _L511_SYS_DEL_FILE_2,       at_callback_success,        "OK",               "\"\r"        	},
    {   _L511_SYS_LIST_FILE,        at_callback_success,        "OK",               "AT+MFSLS\r"   	},    
    // FTP		
    {   _L511_FTP_SERVER_1, 	    _CbFTP_SEND_IP,             NULL,               "AT+CFTPSERV=\""  	},
    {   _L511_FTP_SERVER_2, 	    at_callback_success,        "OK",               "\"\r" 		  	    },
    {   _L511_FTP_PORT,             at_callback_success,        "OK",               "AT+CFTPPORT=21\r"	},    
        
    {   _L511_FTP_USERNAME, 	    at_callback_success,        "OK",               "AT+CFTPUN=\"admin\"\r" 	},
    {   _L511_FTP_PASSWORD, 	    at_callback_success,        "OK",               "AT+CFTPPW=\"admin\"\r"		},
    {   _L511_FTP_TYPE, 		    at_callback_success,        "OK",               "AT+CFTPTYPE=\"I\"\r"   	},
    
    {   _L511_FTP_LIST_FILE,        at_callback_success,        "+CFTPLIST:SUCCESS", "AT+CFTPLIST=\"/\"\r"   	},
    {   _L511_FTP_GET_FILE_1,       at_callback_success,        "+CFTPGET:",        "AT+CFTPGET=\"/Test/FIRM_HCMCHES2.bin\",10\r"  	},
    
    {   _L511_FTP_GET_FILE1,        _CbFTP_GET_FILE1,           NULL,               "AT+CFTPGETFILE=\"/"      	},
    {   _L511_FTP_GET_FILE2,        _CbFTP_GET_FILE2,           NULL,               "\",\""            	},
    {   _L511_FTP_GET_FILE3,        _CbFTP_GET_FILE3,           "+CFTPGETFILE:SUCCESS",     "\",0\r"	},
    
    {   _L511_FTP_READ_1,           _CbFTP_READ_1,              NULL,               "AT+CFTPRDFILE=\""	},    
    {   _L511_FTP_READ_2,           _CbFTP_READ_2,              "+CFTPRDFILE:SUCCESS",      "\r"        },
    
   
    // HTTP	
    {   _L511_HTTP_OPEN,            at_callback_success,        "OK",               "AT$HTTPOPEN\r"     },
    {   _L511_HTTP_CLOSE,           at_callback_success,        "OK",               "AT$HTTPCLOSE\r"  	},
    {   _L511_HTTP_CLOSE_2,         at_callback_success,        NULL,               "\r"  	},
    {   _L511_HTTP_RECV_LOCAL,      at_callback_success,        "OK",               "AT$HTTPTYPE=1\r" 	},
    {   _L511_HTTP_RECV_TE,         at_callback_success,        "OK",               "AT$HTTPTYPE=0\r" 	},
    
    {   _L511_HTTP_SET_HEADER_1,    Sim_Common_HTTP_Head_1,     NULL,               "AT$HTTPRQH="    	},
    {   _L511_HTTP_SET_HEADER_2,    Sim_Common_HTTP_Head_2,     "OK",               "\r"                },
    
    {   _L511_HTTP_SET_URL_1,       _CbHTTP_SETURL_1,           NULL,               "AT$HTTPPARA="    	},
    {   _L511_HTTP_SET_URL_2,       _CbHTTP_SETURL_2,           "OK",               "\r"                },

    {   _L511_HTTP_REQ_POST_1,      at_callback_success,        "OK",               "AT$HTTPACTION=1\r"     },
    {   _L511_HTTP_REQ_POST_3,      Sim_Common_MQTT_Pub_2,      "OK",               "AT$HTTPACTION=3\r"     },
    {   _L511_HTTP_REQUEST_GET,     at_callback_success,        "OK",               "AT$HTTPACTION=0,1\r"   },
    
    {   _L511_HTTP_POST_DATA_1,     Sim_Common_HTTP_Send_1,     NULL,               "AT$HTTPDATA="      },
    {   _L511_HTTP_POST_DATA_2,     Sim_Common_HTTP_Send_2,     ">>",               "\r"                },
    {   _L511_HTTP_POST_DATA_3,     at_callback_success,        "OK",               NULL                },
    
    {   _L511_HTTP_POST_DATA_EX1,   Sim_Common_HTTP_Send_Ex,    NULL,               "AT$HTTPDATAEX="      },
    {   _L511_HTTP_POST_DATA_EX2,   at_callback_success,        "OK",               "\r"                },
    
    {   _L511_HTTP_POST_DAT_END,    at_callback_success,        "OK",               "AT$HTTPDATA=0\r"     },
    {   _L511_HTTP_POST_SEND,       at_callback_success,        "OK",               "AT$HTTPSEND\r"     },
    
    {   _L511_HTTP_POST_SEND_2,     Sim_Common_MQTT_Pub_2,      "$HTTPRECV:",       "AT$HTTPSEND\r"     },  //  
    
    {   _L511_HTTP_LENGTH,          _CbHTTP_FIRM_LENGTH,        "$HTTPREAD:1",      "AT$HTTPREAD=1\r"   },    
    {   _L511_HTTP_READ_RES,        at_callback_success,        "OK",               "AT$HTTPREAD=0,0\r"  	},
    
    {   _L511_HTTP_READ_1,          _CbHTTP_READ_1,             NULL,               "AT$HTTPREAD=0,"  	},
    {   _L511_HTTP_READ_2,          _CbHTTP_READ_2,             "\r\nOK\r\n",       "\r"               	},
    
	// ...
	{	_L511_END, 			        at_callback_success,        NULL,		        NULL,	            },
};

      
const sCommand_Sim_Struct aL511Urc[] =
{
	{_L511_URC_RESET_SIM900,		Sim_Common_URC_Reset,       "NORMAL POWER DOWN",        NULL 	},// OK
	{_L511_URC_SIM_LOST, 		    Sim_Common_URC_Lost_Card,   "SIM CRASH",		 		NULL    },
	{_L511_URC_SIM_REMOVE, 		    Sim_Common_URC_Lost_Card,   "SIM REMOVED",		 		NULL	},
	{_L511_URC_CLOSED,			    _CbURC_CLOSED,              "+SERVER DISCONNECTED:", 	NULL	},   //+SERVER DISCONNECTED:0
	{_L511_URC_PDP_DEACT, 		    _CbURC_CLOSED,              "+NETWORK DISCONNECTED",    NULL	},   //+NETWORK DISCONNECTED:0
	{_L511_URC_CALL_READY, 		    Sim_Common_URC_Call_Ready,  "Call Ready",				NULL	},
    {_L511_URC_CALLING, 		    Sim_Common_URC_Calling,     "+CLIP:",                   NULL    },
    {_L511_URC_SMS_CONTENT, 		Sim_Common_URC_Sms_Content, "+CMGL:",                   NULL	},
    {_L511_URC_SMS_INDICATION1, 	Sim_Common_URC_Sms_Indi,    "+CMTI:",                   NULL	},
    {_L511_URC_SMS_INDICATION2, 	Sim_Common_URC_Sms_Indi,    "+CMT:",                    NULL	},
    
	{_L511_URC_ERROR, 			    Sim_Common_URC_Error,       "ERROR",					NULL	},
    {_L511_URC_RECEIV_SERVER, 	    Sim_Common_URC_Recv_Server, "AMI/",			            NULL	},     
};


/*=============== Call back =================*/
static uint8_t at_callback_success(sData *uart_string)
{
    
        
	return 1;
}

/*==============Call back at sim ===============*/

static uint8_t _CbAT_SET_APN (sData *uart_string)
{
    Sim_Common_Send_AT_Cmd( sSimCommInfor.sAPNInfor->sName.Data_a8, sSimCommInfor.sAPNInfor->sName.Length_u16 );
    Sim_Common_Send_AT_Cmd( (uint8_t*) "\",\"", 3 );
    Sim_Common_Send_AT_Cmd( sSimCommInfor.sAPNInfor->sUsername.Data_a8, sSimCommInfor.sAPNInfor->sUsername.Length_u16 );
    Sim_Common_Send_AT_Cmd( (uint8_t*) "\",\"", 3 );
    Sim_Common_Send_AT_Cmd( sSimCommInfor.sAPNInfor->sPassword.Data_a8, sSimCommInfor.sAPNInfor->sPassword.Length_u16 );
    
    return 1;
}


static uint8_t _CbAT_APN_ACTIVE (sData *uart_string)
{
    //Test tam
#ifndef MQTT_PROTOCOL
    sSimCommVar.State_u8 = _SIM_CONN_MQTT;
#endif
    sSimCommVar.HttpHeaderIndex_u8 = 0;
    return 1;
}



static uint8_t _CbAT_CHECK_RSSI(sData *uart_string)
{
    sData   strCheck = {(uint8_t*) aSimL511Step[_L511_AT_CHECK_RSSI].at_response, 
                                        strlen(aSimL511Step[_L511_AT_CHECK_RSSI].at_response)};
    sData strFind;
	int Pos = Find_String_V2(&strCheck, uart_string);
    
	if (Pos >= 0) 
    { 
        Pos += strCheck.Length_u16;
        strFind.Data_a8 = uart_string->Data_a8 + Pos;
        strFind.Length_u16 = uart_string->Length_u16 - Pos;
            
        Sim_Common_Get_CSQ (&strFind);
    }
    
    return 1;
}

static uint8_t _CbAT_GET_CLOCK (sData *uart_string)
{
	sData  strCheck = {(uint8_t*) aSimL511Step[_L511_NET_GET_RTC].at_response, 
                                                strlen(aSimL511Step[_L511_NET_GET_RTC].at_response)};
    sData strFind;
	int Pos = Find_String_V2(&strCheck, uart_string);
    
	if (Pos >= 0) 
    { 
        Pos += strCheck.Length_u16;
        strFind.Data_a8 = uart_string->Data_a8 + Pos;
        strFind.Length_u16 = uart_string->Length_u16 - Pos;
            
        Sim_Common_Get_CCLK (&strFind);
    }
   
    return 1;
}


static uint8_t _CbAT_OPEN_TCP_2 (sData *uart_string)
{
    int Posfix = 0;
    
#ifdef USING_TRANSPARENT_MODE
    sData StConnectOk = {(uint8_t*)"CONNECT \r\n",10};
#else
    sData StConnectOk = {(uint8_t*)"+CIPOPEN: SUCCESS", 17}; 
#endif
    Posfix = Find_String_V2((sData*) &StConnectOk, uart_string);
	if (Posfix >= 0)
    {
        sSimCommVar.State_u8 = _SIM_OPENED_TCP; 
        return 1;
    }

    return 0;
}



/*================== Call back File SYS, FTP va HTTP====================*/
//File sys callback
static uint8_t _CbSYS_DEL_FILE(sData *uart_string)
{
    Sim_Common_Send_AT_Cmd( (uint8_t *)&sFileSys.aNAME[1], sFileSys.LengthName_u8 - 1 );
    
    return 1;
}


//FTP Callback
static uint8_t _CbFTP_SEND_IP(sData *uart_string)
{
    Sim_Common_Send_AT_Cmd( sSimFwUpdate.sUpdateInfor.pFirmIP->Data_a8, sSimFwUpdate.sUpdateInfor.pFirmIP->Length_u16 );
    
    return 1;
}



static uint8_t _CbFTP_GET_FILE1(sData *uart_string)
{
    Sim_Common_Send_AT_Cmd( sSimFwUpdate.sUpdateInfor.pFirmPath->Data_a8, sSimFwUpdate.sUpdateInfor.pFirmPath->Length_u16 );
    
    if (sSimFwUpdate.sUpdateInfor.pFirmPath->Length_u16 != 0) 
        Sim_Common_Send_AT_Cmd( (uint8_t*) "/",1 );
    
    Sim_Common_Send_AT_Cmd( sSimFwUpdate.sUpdateInfor.pFirmName->Data_a8, sSimFwUpdate.sUpdateInfor.pFirmName->Length_u16 ); 

    return 1;
}

static uint8_t _CbFTP_GET_FILE2(sData *uart_string)
{
    Sim_Common_Send_AT_Cmd( (uint8_t *)&sFileSys.aNAME[1], sFileSys.LengthName_u8 - 1 );

    return 1;
}

static uint8_t _CbFTP_GET_FILE3(sData *uart_string)
{
//    int16_t     index =0;
//    int16_t     Pos_Str = -1;
//    uint8_t     statusM = 0;
//    uint8_t     dataMemory[8] = {0};
//    uint8_t     count = 0;
//    sData    strCheck = {(uint8_t*) aSimL511Step[_L511_FTP_GET_FILE3].at_response, strlen(aSimL511Step[_L511_FTP_GET_FILE3].at_response)};
//    
//	Pos_Str = Find_String_V2((sData*) &strCheck, uart_string);
//    if (Pos_Str >= 0)
//    {
//        for (index = Pos_Str; index < uart_string->Length_u16; index++)
//        {
//            if (*(uart_string->Data_a8 + index) == ',')
//                statusM = 1;
//            
//            if (*(uart_string->Data_a8 + (index + 1)) == '\r')
//            {
//                statusM = 0;
//                break;
//            }
//            if (statusM == 1)
//            {
//                dataMemory[count] = *(uart_string->Data_a8 + (index + 1));
//                count++;
//            }
//            if (count == 8)
//                break;
//        }
//        sSimFwUpdate.FirmLength_u32 = (uint32_t) UtilStringToInt2(dataMemory, count);
//    }
    
    return 1;
}



static uint8_t _CbFTP_READ_1(sData *uart_string)
{
//    uint8_t     aNumbyte[10] = {0};
//    uint8_t     aLengread[10] = {0};
//    sData    strPos      = {&aNumbyte[0], 0};
//    sData    strlenread  = {&aLengread[0], 0};
//    //chuyen offset ra string de truyen vao sim
//    Convert_Uint64_To_StringDec(&strPos, sSimFwUpdate.CountByteTotal_u32, 0);
//    
//    if (sSimFwUpdate.FirmLength_u32 >= (sSimFwUpdate.CountByteTotal_u32 + 1024))
//        Convert_Uint64_To_StringDec(&strlenread, 1024, 0);
//    else 
//        Convert_Uint64_To_StringDec(&strlenread, (sSimFwUpdate.FirmLength_u32 - sSimFwUpdate.CountByteTotal_u32), 0);
//      
//    Sim_Common_Send_AT_Cmd((uint8_t*) &aFIRM_FILE_NAME[0], LENGTH_FIRM_NAME, 100);
//    Sim_Common_Send_AT_Cmd((uint8_t*) "\",", 2,100);
//    Sim_Common_Send_AT_Cmd((uint8_t*) strPos.Data_a8, strPos.Length_u16,100);
//    Sim_Common_Send_AT_Cmd((uint8_t*) ",", 1,100);
//    Sim_Common_Send_AT_Cmd((uint8_t*) strlenread.Data_a8, strlenread.Length_u16,100);
    
    return 1;
}

static uint8_t _CbFTP_READ_2(sData *uart_string)
{
    //Tuong tu nhu Http Read
    
    return 1;
}

//Callback HTTP

// https:\/\/a.nel.cloudflare.com\/report\/v3?s=RVcnmYwkD6RaH6XWO9ChlqMOChrTrd7qP9ecE9ufonbhMGbPKullRhExQb6Ivfn1OJp5aquSMUENt3AlRFeXPh1JvH4Ru23tY%2B285JlixO42I6raA9atJ34YbySf2Mb4f9HqWqI%3D

// "https://thingsboard.cloud/api/v1/integrations/http/68f538c0-67f8-184f-cfa5-d81404c5e08c"}; 
// {"http://up.saovietgroup.com.vn:80/chien"}; 
    
// {"https://demo-api.ilotusland.asia/iot/core/public/push-data/c47f0701-9c99-4db9-99af-5c55932c264f"};  //  
// https://n8n.ilotusland.com/webhook/e370f0f4-96bc-4134-adb6-da658813d60a
static uint8_t _CbHTTP_SETURL_1(sData *uart_string)
{   
    Sim_Common_Send_AT_Cmd( sSimFwUpdate.sUpdateInfor.pUrlHTTP->Data_a8, sSimFwUpdate.sUpdateInfor.pUrlHTTP->Length_u16 );
    Sim_Common_Send_AT_Cmd( (uint8_t*) ",", 1 );
    Sim_Common_Send_AT_Cmd( sSimFwUpdate.sUpdateInfor.pFirmPort->Data_a8, sSimFwUpdate.sUpdateInfor.pFirmPort->Length_u16 );
    
    return 1;
}

static uint8_t _CbHTTP_SETURL_2(sData *uart_string)
{
    return 1;
}



static uint8_t _CbHTTP_FIRM_LENGTH(sData *uart_string)
{
    int16_t     index =0;
    int16_t     Pos_Str = -1;
    uint8_t     statusM = 0;
    uint8_t     dataMemory[8] = {0};
    uint8_t     count = 0;
    sData       strCheck = {(uint8_t*) aSimL511Step[_L511_HTTP_LENGTH].at_response, strlen(aSimL511Step[_L511_HTTP_LENGTH].at_response)};
    
	Pos_Str = Find_String_V2((sData*) &strCheck, uart_string);
    if (Pos_Str >= 0)
    {
        for (index = Pos_Str; index < uart_string->Length_u16; index++)
        {
            if (*(uart_string->Data_a8 + index) == ',')
                statusM = 1;

            if (*(uart_string->Data_a8 + (index + 1)) == '\r')
            {
                statusM = 0;
                break;
            }
            if (statusM == 1)
            {
                dataMemory[count] = *(uart_string->Data_a8 + (index + 1));
                count++;
            }
            if (count == 8)
                break;
        }
        sSimFwUpdate.FirmLength_u32 = (uint32_t) UtilStringToInt2(dataMemory, count);
    }      

    sSimCommVar.State_u8 = _SIM_CONN_HTTP; 
    //Init Struct Update Fw
    if (Sim_Common_Init_UpdateFw() == false)
        return 0;
    
    return 1;
}

static uint8_t _CbHTTP_READ_1(sData *uart_string)
{
    char aOFFSET[20] = {0};
    //chuyen offset ra string de truyen vao sim
    UtilIntToString(sSimFwUpdate.AddOffset_u32, aOFFSET);

    Sim_Common_Send_AT_Cmd( (uint8_t *) aOFFSET, strlen(aOFFSET) );

    return 1;
}



static uint8_t _CbHTTP_READ_2(sData *uart_string)
{
    return Sim_Common_Http_Read_Data (uart_string);
}



//+SERVER DISCONNECTED:0
static uint8_t _CbURC_CLOSED (sData *uart_string)
{    
    sData strCheck = {(uint8_t*) aL511Urc[_L511_URC_CLOSED].at_response,
                                    strlen(aL511Urc[_L511_URC_CLOSED].at_response)};
    
	int PosFind = Find_String_V2((sData*) &strCheck, uart_string);
    if (PosFind >= 0)
    {
        PosFind += strCheck.Length_u16;
        //check vi tri tiep theo la CID disconnect
        if (*(uart_string->Data_a8 + PosFind + 1) == CID_SERVER)
        {
            Sim_Common_URC_Closed(uart_string);
        }
    }
 
    return 1;
}



sCommand_Sim_Struct * L511_Get_AT_Cmd (uint8_t step)
{
    return (sCommand_Sim_Struct *) &aSimL511Step[step];
}


uint32_t L511_Get_Delay_AT (uint8_t Step)
{
    uint32_t Time = 0;
    
    switch (Step)
    {
        case _L511_HTTP_POST_SEND_2:
        case _L511_TCP_OUT_DATAMODE:
            Time = 1000;
            break;
        case _L511_TCP_TRANS:
        case _L511_HTTP_READ_2:
        case _L511_FTP_READ_2:
            Time = 100;
            break;
        default:
            break;
    }
    
    return Time;
}
       

/*
    Func: Get Timeout follow Cmd
        
*/

uint32_t L511_Get_Timeout_Follow_Cmd (uint8_t sim_step)
{
    uint32_t time = 0;
        
    switch(sim_step)
    {
        case _L511_TCP_CONNECT_2:
        case _L511_MQTT_CONN_2:
        case _L511_MQTT_SUB_2:
        case _L511_MQTT_PUB_2:
        case _L511_HTTP_LENGTH:
            time = SIM_TCP_TIMEOUT;
            break;
        case _L511_NET_CHECK_ATTACH:
            time = SIM_CMD_TIMEOUT2;
            break;
        default:
            time = SIM_CMD_TIMEOUT;
            break;
    }
    
    return time;    
}

/*
    Func: check xem co bo qua lenh loi k
        return: true:
                    + thuc hien tiep cac lenh tiep theo
                    + bo qua bao nhieu lenh tiep theo lien quan
                false: thuc hien failure
*/
           
uint8_t L511_Is_Ignore_Failure (uint8_t Step, uint8_t (*pJum_Next_At)(uint8_t Type))
{
    return false;
}



/*
    Func: Check step allow check URC
        + 1: Allow
        + 0: Not allow
*/
uint8_t L511_Is_Step_Check_URC (uint8_t sim_step)
{
    if ((sim_step != _L511_FTP_READ_1) && (sim_step != _L511_FTP_READ_2)
        && (sim_step != _L511_HTTP_READ_1) && (sim_step != _L511_HTTP_READ_2)) 
    {
        return TRUE;
    }
    
    return FALSE;
}


uint8_t L511_Check_Step_Skip_Error (uint8_t step)
{
    
    return false;
}


uint8_t L511_Is_Send_Hex (uint8_t step)
{
    if ( ( step == _L511_TCP_SEND_CMD_3)
         || (step == _L511_MQTT_CONN_2) 
         || (step == _L511_MQTT_SUB_2)
         || (step == _L511_MQTT_PUB_2) 
         || (step == _L511_MQTT_PUB_FB_2) 
         || (step == _L511_MQTT_PUB_PING) 
         || (step == _L511_HTTP_POST_DATA_3) 
        )
        return true;
    
    return false;
}
