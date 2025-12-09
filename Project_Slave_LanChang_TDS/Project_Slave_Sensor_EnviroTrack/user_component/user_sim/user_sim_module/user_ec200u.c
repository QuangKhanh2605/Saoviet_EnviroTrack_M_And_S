/*
 * siml506.c
 *
 *  Created on: Feb 16, 2022
 *      Author: chien
 */


#include "string.h"

#include "user_ec200u.h"
#include "user_string.h"

     
/* ========= handler Sim driver ============*/
/* define static functions */
static uint8_t at_callback_success(sData *uart_string);

/*========== Callback AT ===================*/
static uint8_t _CbAT_SET_APN (sData *uart_string);
static uint8_t _CbAT_APN_AUTH (sData *uart_string);
static uint8_t _CbAT_APN_ACTIVE (sData *uart_string);

static uint8_t _CbAT_CHECK_RSSI(sData *uart_string);
static uint8_t _CbAT_GET_CLOCK (sData *uart_string);

static uint8_t _CbAT_OPEN_TCP_2 (sData *uart_string);

//File sys callback
static uint8_t _CbSYS_DEL_FILE(sData *uart_string);
static uint8_t _CbSYS_FILE_OPEN_1(sData *uart_string);
static uint8_t _CbSYS_FILE_OPEN_2(sData *uart_string);
static uint8_t _CbSYS_FILE_CLOSE_1(sData *uart_string);
static uint8_t _CbSYS_FILE_SEEK_1(sData *uart_string);
static uint8_t _CbSYS_FILE_POSITION_1(sData *uart_string);
static uint8_t _CbSYS_FILE_READ_1(sData *uart_string);
static uint8_t _CbSYS_FILE_READ_2(sData *uart_string);

//FTP Callback
static uint8_t _CbFTP_SEND_IP(sData *uart_string);
static uint8_t _CbFTP_SEND_PATH(sData *uart_string);

static uint8_t _CbFTP_GET_FILE_1(sData *uart_string);
static uint8_t _CbFTP_GET_FILE_2(sData *uart_string);

static uint8_t _CbFTP_READ_1(sData *uart_string);
static uint8_t _CbFTP_READ_2(sData *uart_string);
//Callback HTTP
static uint8_t _CbHTTP_SETURL_1(sData *uart_string);
static uint8_t _CbHTTP_SETURL_2(sData *uart_string);

static uint8_t _CbHTTP_GET_REQUEST(sData *uart_string);
static uint8_t _CbHTTP_STORE_FILE1(sData *uart_string);

static uint8_t _CbHTTP_READ_1(sData *uart_string);
static uint8_t _CbHTTP_READ_2(sData *uart_string);

//AUDIO
static uint8_t _Cb_AUDIO_FINISH(sData *uart_string);

//GPS callback
static uint8_t _Cb_GPS_POSITION_INF(sData *uart_string);
static uint8_t _CbURC_CLOSED (sData *uart_string);

/*============= Var struct ====================*/
//Block step control SIM
uint8_t aEC200_BLOCK_INIT[17] = 
{
	_EC200_AT_ECHO,               //0
    _EC200_AT_ECHO,               //1
	_EC200_AT_AT,                 //2
	
	_EC200_AT_BAUD_RATE,          //3
	_EC200_AT_CHECK_RSSI,         //4
	_EC200_AT_GET_IMEI,           //5
    _EC200_AT_DTR_MODE,           //6
    
    _EC200_CALLING_LINE,          //7
    _EC200_CALLING_CONN,          //8    
    _EC200_SMS_FORMAT_TEXT,       //9    
    _EC200_SMS_NEW_MESS_IND,      //10
    _EC200_GPS_CFG_AUTO_RUN,      //11
    
    _EC200_NET_CEREG,             //12
    _EC200_NET_TIME_ZONE,
//    _EC200_NET_GET_RTC,           //
    _EC200_AT_GET_ID,             //13
    _EC200_AT_CHECK_SIM,          //14
    _EC200_AT_URC_PORT,           //15    
};

  


uint8_t aEC200_BLOCK_NETWORK[9] = 
{
	_EC200_NET_CHECK_ATTACH,      //0
    _EC200_SMS_READ_UNREAD,       //1
    _EC200_SMS_DEL_ALL,           //2
    _EC200_AT_CHECK_RSSI,         //3
	_EC200_NET_GET_RTC,           //4
	_EC200_AT_SET_APN_1,          //5
    _EC200_AT_SET_APN_2,          //6
    _EC200_GPS_TURN_ON,           //7
    _EC200_AT_CHECK_RSSI,         //8
    
//    _EC200_GPS_CFG_TOKEN,         //7
//    _EC200_GPS_QUERY_TOKEN,       //8
//    _EC200_GPS_CFG_FORMAT,        //9
//    _EC200_GPS_GET_LOCATION,      //10    
//    _EC200_GPS_TURN_OFF,          //11   
};
   

uint8_t aEC200_BLOCK_GNSS[2] = 
{
    _EC200_AT_CHECK_RSSI,
    _EC200_GPS_POSTION_INF,
//    _EC200_GPS_GET_LOCATION,
};

uint8_t aEC200_BLOCK_GNSS_DELL[1] = 
{
    _EC200_GPS_DEL_CUR_DATA,
};


uint8_t aEC200_BLOCK_GNSS_ON[1] = 
{
    _EC200_GPS_TURN_ON,
};


#ifdef USING_TRANSPARENT_MODE
    uint8_t aEC200_BLOCK_CONNECT[7] = 
    {
        _EC200_TCP_IP_SIM,     //0
        _EC200_TCP_CONNECT_1,  //1
        _EC200_TCP_CONNECT_2,  //2
        
        _EC200_MQTT_CONN_1,        //3
        _EC200_MQTT_CONN_2,        //4

        _EC200_MQTT_SUB_1,         //5
        _EC200_MQTT_SUB_2,         //6
    };

    uint8_t aEC200_BLOCK_PUB[2] =
    {
        _EC200_MQTT_PUB_1,
        _EC200_MQTT_PUB_2,
    };

    uint8_t aEC200_BLOCK_PUB_FB[2] =
    {
        _EC200_MQTT_PUB_FB_1,
        _EC200_MQTT_PUB_FB_2,
    };
    
    uint8_t aEC200_BLOCK_PING[2] =
    {
        _EC200_MQTT_PUB_FB_1,
        _EC200_MQTT_PUB_PING,
    };  
#else
    uint8_t aEC200_BLOCK_CONNECT[11] = 
    {
        _EC200_TCP_CONNECT_1,     //0
        _EC200_TCP_CONNECT_2,     //1
        _EC200_TCP_IP_SIM,        //2
        
        _EC200_MQTT_CONN_1,       //3
        _EC200_TCP_SEND_CMD_1,    //4
        _EC200_TCP_SEND_CMD_2,    //5
        _EC200_MQTT_CONN_2,       //6

        _EC200_MQTT_SUB_1,        //7
        _EC200_TCP_SEND_CMD_1,    //8
        _EC200_TCP_SEND_CMD_2,    //9
        _EC200_MQTT_SUB_2,        //10
    };

    uint8_t aEC200_BLOCK_PUB[4] =
    {
        _EC200_MQTT_PUB_1,
        _EC200_TCP_SEND_CMD_1,
        _EC200_TCP_SEND_CMD_2,
        _EC200_TCP_SEND_CMD_3,
    };

    uint8_t aEC200_BLOCK_PUB_FB[4] =
    {
        _EC200_MQTT_PUB_FB_1,
        _EC200_TCP_SEND_CMD_1,
        _EC200_TCP_SEND_CMD_2,
        _EC200_MQTT_PUB_FB_2,
    };
    
    uint8_t aEC200_BLOCK_PING[4] =
    {
        _EC200_MQTT_PUB_FB_1,
        _EC200_TCP_SEND_CMD_1,
        _EC200_TCP_SEND_CMD_2,
        _EC200_MQTT_PUB_PING,
    };        
#endif

uint8_t  aEC200_BLOCK_CSQ[3] = 
{
    _EC200_TCP_OUT_DATAMODE,
    _EC200_AT_CHECK_RSSI,
    _EC200_TCP_IN_DATAMODE,
};


uint8_t aEC200_BLOCK_DIS_CONN[4] = 
{
    _EC200_TCP_OUT_DATAMODE,
    _EC200_TCP_IP_SIM,
	_EC200_TCP_CLOSE_1,
    _EC200_TCP_CLOSE_2,
};

uint8_t aEC200_BLOCK_CLOCK[3] = 
{
    _EC200_TCP_OUT_DATAMODE,
    _EC200_NET_GET_RTC,
    _EC200_TCP_IN_DATAMODE,
};

uint8_t aEC200_BLOCK_HTTP_INIT[19] = 
{
    _EC200_NET_CHECK_ATTACH,      //0
    _EC200_AT_CHECK_RSSI,         //1
    _EC200_AT_SET_APN_1,          //2
    _EC200_AT_SET_APN_2,          //3
//    _EC200_AT_APN_AUTH_1,
//    _EC200_AT_APN_AUTH_2,
    _EC200_AT_TEST_0,             //4
    _EC200_AT_ACTIVE_APN,         //5
    
    _EC200_HTTP_CFG_CONTEXT,      //6
    _EC200_HTTP_CFG_HEADER2,      //7
    _EC200_HTTP_SET_URL_1,        //8
    _EC200_HTTP_SET_URL_2,        //9
    _EC200_HTTP_SET_URL_3,        //10
         
    _EC200_HTTP_REQUEST_GET,      //11
    _EC200_HTTP_STORE_FILE1,      //12
    _EC200_HTTP_STORE_FILE2,      //13
        
    _EC200_SYS_FILE_LIST,         //14
    _EC200_SYS_FILE_OPEN1,        //15
    _EC200_SYS_FILE_OPEN2,        //16
    
    _EC200_SYS_FILE_SEEK1,        //17
    _EC200_SYS_FILE_SEEK2,        //18
};




uint8_t aEC200_BLOCK_HTTP_READ[2] = 
{
    _EC200_SYS_FILE_READ1,
    _EC200_SYS_FILE_READ2,

//    _EC200_SYS_FILE_POS1,
//    _EC200_SYS_FILE_POS2,
};



//Read SMS
uint8_t aEC200_BLOCK_READ_SMS[4] = 
{
    _EC200_TCP_OUT_DATAMODE,
    _EC200_SMS_READ_UNREAD,
    _EC200_SMS_DEL_ALL,
    _EC200_TCP_IN_DATAMODE,
};  

//Read SMS
uint8_t aEC200_BLOCK_SEND_SMS[5] = 
{
    _EC200_TCP_OUT_DATAMODE,
    _EC200_SMS_SEND_1,
    _EC200_SMS_SEND_2,
    _EC200_SMS_SEND_3,
    _EC200_TCP_IN_DATAMODE,
};  

//Read SMS
uint8_t aEC200_BLOCK_CALLING[6] = 
{
    _EC200_TCP_OUT_DATAMODE,
    _EC200_CALLING_ESTAB_1,
    _EC200_CALLING_ESTAB_2,
    _EC200_AUDIO_FAR_END,
    _EC200_CALLING_DISCONN,
    _EC200_TCP_IN_DATAMODE,
};  

uint8_t aEC200_BLOCK_FILE_CLOSE[2] = 
{
    _EC200_SYS_FILE_CLOSE1,
    _EC200_SYS_FILE_CLOSE2,
};
    


uint8_t aEC200_BLOCK_FTP_INIT[16] = 
{
    _EC200_FTP_CONTEXT,           //0
    _EC200_FTP_ACCOUNT,           //1
    _EC200_FTP_FILE_TYPE,         //2    
    _EC200_FTP_TRANS_MODE,        //3
    _EC200_FTP_RESP_TIME,         //4
    
    _EC200_FTP_LOGIN_1,           //5
    _EC200_FTP_LOGIN_2,           //6
                
    _EC200_FTP_PATH_1,            //7
    _EC200_FTP_PATH_2,            //8
    
    _EC200_FTP_GET_TO_RAM_1,      //9
    _EC200_FTP_GET_TO_RAM_2,      //10
    
    _EC200_SYS_FILE_LIST,         //11
    _EC200_SYS_FILE_OPEN1,        //12
    _EC200_SYS_FILE_OPEN2,        //13
    
    _EC200_SYS_FILE_SEEK1,        //14
    _EC200_SYS_FILE_SEEK2,        //15
};


uint8_t aEC200_BLOCK_FTP_READ[2] = 
{
    _EC200_SYS_FILE_READ1,
    _EC200_SYS_FILE_READ2,
};


/*========= Struct List AT Sim ==================*/
static char DataConack_8a[] = { MY_CONNACK, 0x02, 0x00 };		 //20020000
static char DataSuback_8a[] = { MY_SUBACK, 0x04, 0x00};          //9004000a00
static char DataPuback_8a[] = { MY_PUBACK, 0x02, 0x00};	         //40020001  , 0x1A 
static char DataPingResp_8a[] = { MY_PINGRESP, 0x00};	 // Ð<00>
static char EndChar[2] = {0x1A, 0};
    
const sCommand_Sim_Struct aEC200Step[] =
{
    //Cmd Pre Init
	{	_EC200_AT_AT,			    at_callback_success,        "OK",                   "AT\r"              },
	{	_EC200_AT_ECHO,				at_callback_success,        "OK", 		            "ATE0\r" 	 	    },
    //Cmd Init
    {   _EC200_AT_BAUD_RATE, 		at_callback_success,        "OK",                   "AT+IPR=115200\r"	},
    {	_EC200_AT_GET_ID,			Sim_Common_Get_Sim_ID,      "CCID: ",               "AT+QCCID\r"     	},
	{	_EC200_AT_CHECK_RSSI,		_CbAT_CHECK_RSSI,           "+CSQ: ",               "AT+CSQ\r\n" 	 	},
	{   _EC200_AT_GET_IMEI, 		Sim_Common_Get_Imei,        "OK",                   "AT+CGSN=1\r"		},
	{	_EC200_AT_CHECK_SIM,		Sim_Common_CPIN_Ready,      "OK",                   "AT+CPIN?\r"  	    },
    {	_EC200_AT_SET_APN_1,      	_CbAT_SET_APN,              NULL,                   "AT+CGDCONT=1,\"IP\",\""		}, 
	{	_EC200_AT_SET_APN_2,        at_callback_success,        "OK",                   "\"\r"         		}, 
    {	_EC200_AT_APN_AUTH_1,	    _CbAT_APN_AUTH,             NULL,                   "AT+QICSGP=1,1,\""	},     
    {   _EC200_AT_APN_AUTH_2,       at_callback_success,        "OK",                   ",1\r" 	        },
    
    {   _EC200_AT_DTR_MODE, 	    at_callback_success,        "OK",                   "AT&D1\r"			},
    {   _EC200_AT_TEST_0, 		    at_callback_success,        "OK",                   "AT+CGATT=1\r"	   	},
    {   _EC200_AT_GET_IP_APN,       _CbAT_APN_ACTIVE,           "+QIACT: 1,",           "AT+QIACT?\r"	},
    {   _EC200_AT_ACTIVE_APN, 	    at_callback_success,        "OK",                   "AT+QIACT=1\r"  	},
    {   _EC200_AT_URC_PORT, 	    at_callback_success,        "OK",                   "AT+QURCCFG=\"urcport\",\"uart1\"\r"  	}, 
   //SMS
    {   _EC200_SMS_FORMAT_TEXT,	    at_callback_success,        "OK",                   "AT+CMGF=1\r"       },  
    {   _EC200_SMS_NEW_MESS_IND,	at_callback_success,        "OK",                   "AT+CNMI=2,1\r"     },   // 1,2,0,1,0
    {   _EC200_SMS_READ_UNREAD,  	at_callback_success,        "\r\nOK",               "AT+CMGL=\"REC UNREAD\"\r"      },
    
    {   _EC200_SMS_SEND_1,  	    Sim_Common_SMS_Send_1,      NULL,                   "AT+CMGS=\""        },
    {   _EC200_SMS_SEND_2,  	    Sim_Common_SMS_Send_2,      ">",                    "\"\r"              },
    {   _EC200_SMS_SEND_3,  	    Sim_Common_SMS_Send_3,      "+CMGS:",               EndChar             },
    
    {   _EC200_SMS_READ_ALL,  	    at_callback_success,        "\r\nOK",               "AT+CMGL=\"ALL\"\r" },
    {   _EC200_SMS_DEL_ALL,	        at_callback_success,        "OK",                   "AT+CMGD=1,4\r"     },
    //Calling
    {   _EC200_CALLING_LINE, 		at_callback_success,        "OK",                   "AT+CLIP=1\r"       },
    {   _EC200_CALLING_CONN, 		at_callback_success,        "OK",                   "AT+COLP=1\r"       },
    
    {   _EC200_CALLING_ESTAB_1, 	Sim_Common_Call_1,          NULL,                   "ATD"       },
    {   _EC200_CALLING_ESTAB_2, 	Sim_Common_Call_2,          "+COLP: ",              ";\r"       },
    {   _EC200_CALLING_DISCONN, 	at_callback_success,        "OK",                   "ATH0\r"       },
    
    //Cmd Check Network
    {	_EC200_NET_TIME_ZONE, 	    at_callback_success,        "OK",	                "AT+CTZU=0\r"		},
    
	{	_EC200_NET_CEREG, 			at_callback_success,        "OK",	                "AT+CEREG=0\r"		},
	{	_EC200_NET_CHECK_ATTACH,	Sim_Common_Check_Attached,  "OK",                   "AT+CGATT?\r"		},
    //Cmd clock
	{	_EC200_NET_GET_RTC, 		_CbAT_GET_CLOCK,            "+CCLK:",	            "AT+CCLK?\r"		},
    //Cmd Data mode and command mode
#ifdef USING_TRANSPARENT_MODE
    {   _EC200_TCP_OUT_DATAMODE,    Sim_Common_Non_Tranparent,  NULL,                   "+++"			  	}, 
    {   _EC200_TCP_IN_DATAMODE,     Sim_Common_Tranparent,      "CONNECT",              "ATO\r"		        },
#else
    {   _EC200_TCP_OUT_DATAMODE, 	Sim_Common_Non_Tranparent,  NULL, 			        NULL	            }, 
    {   _EC200_TCP_IN_DATAMODE,     Sim_Common_Tranparent,      NULL,		            NULL     	        },
#endif   
    //Cmd TCP 
    {   _EC200_TCP_IP_SIM,	        at_callback_success,        "OK",                   "AT+QISTATE?\r"  },         
	{	_EC200_TCP_CLOSE_1,	 	    Sim_Common_TCP_Close,       NULL,	                "AT+QICLOSE="  	},
    {	_EC200_TCP_CLOSE_2,	 	    at_callback_success,        "OK",	                "\r"  	}, 
  
    {	_EC200_TCP_CONNECT_1,	    Sim_Common_TCP_Open_1,      NULL,                   "AT+QIOPEN=1,"     	},
#ifdef USING_TRANSPARENT_MODE
    {	_EC200_TCP_CONNECT_2,	    _CbAT_OPEN_TCP_2,           "CONNECT",              ",0,2\r"            },   // Tranparent; AT+QIOPEN=1,0,"TCP","220.180.239.212",8009,0,2
#else
    {	_EC200_TCP_CONNECT_2,	    _CbAT_OPEN_TCP_2,           "+QIOPEN:",             ",0,1\r"            },   // Push mode: AT+QIOPEN=1,0,"TCP","220.180.239.212",8009,0,1 
#endif        
    {   _EC200_TCP_SEND_CMD_1, 	    Sim_Common_TCP_Send_1,      NULL,                   "AT+QISEND="      	},
    {   _EC200_TCP_SEND_CMD_2,      Sim_Common_TCP_Send_2,      ">",                    "\r"	 		   	},
    {   _EC200_TCP_SEND_CMD_3, 	    Sim_Common_MQTT_Pub_2,      "SEND OK",	            NULL            	},
    
    //Mqtt AT
	{   _EC200_MQTT_CONN_1, 		Sim_Common_MQTT_Conn_1,     NULL,	                NULL   	            },
	{   _EC200_MQTT_CONN_2, 		Sim_Common_MQTT_Conn_2,     DataConack_8a,          NULL	    		},  //4,"20020000"

	{   _EC200_MQTT_SUB_1, 		    Sim_Common_MQTT_Sub_1,      NULL,	                NULL                },
	{   _EC200_MQTT_SUB_2, 		    Sim_Common_MQTT_Sub_2,      DataSuback_8a,          NULL       			},

	{   _EC200_MQTT_PUB_1, 		    Sim_Common_MQTT_Pub_1,      NULL,				    NULL             	},
	{   _EC200_MQTT_PUB_2, 		    Sim_Common_MQTT_Pub_2,      NULL,	   			    NULL  		     	},  //"4,40020001"  "SEND OK"
    
	{   _EC200_MQTT_PUB_FB_1, 	    Sim_Common_MQTT_Pub_1,      NULL,				    NULL             	},
	{   _EC200_MQTT_PUB_FB_2, 	    Sim_Common_MQTT_Pub_2,      DataPuback_8a,          NULL       	        },  // 4,"40020001"
    
    {   _EC200_MQTT_PUB_PING, 	    Sim_Common_MQTT_Pub_2,      DataPingResp_8a,        NULL       	        }, 
    //Ping
    {   _EC200_TCP_PING_TEST1,      Sim_Common_Ping,            NULL,                   "AT+QPING=1,\""    	},    
    {   _EC200_TCP_PING_TEST2,      at_callback_success,        "+QPING: 0,4",          "\",1,4\r"        	},
    
    // File	 
    {   _EC200_SYS_FILE_DEL1,       _CbSYS_DEL_FILE,            NULL,                   "AT+QFDEL=\""     	},
    {   _EC200_SYS_FILE_DEL2,       at_callback_success,        "OK",                   "\"\r"          	},
    
    {   _EC200_SYS_FILE_DEL_ALL,    at_callback_success,        "OK",                   "AT+QFDEL=\"*\"\r"  }, 
    {   _EC200_SYS_FILE_LIST,       at_callback_success,        "OK",                   "AT+QFLST=\"*\"\r"  },    
    
    {   _EC200_SYS_FILE_OPEN1,      _CbSYS_FILE_OPEN_1,         NULL,                   "AT+QFOPEN=\""      },    
    {   _EC200_SYS_FILE_OPEN2,      _CbSYS_FILE_OPEN_2,         "+QFOPEN:",              "\",2\r"           },                //mode 2: readonly
        
    {   _EC200_SYS_FILE_CLOSE1,     _CbSYS_FILE_CLOSE_1,        NULL,                   "AT+QFCLOSE="       },    
    {   _EC200_SYS_FILE_CLOSE2,     at_callback_success,        "OK",                   "\r"                },   
    
    {   _EC200_SYS_FILE_SEEK1,      _CbSYS_FILE_SEEK_1,         NULL,                   "AT+QFSEEK="        },    
    {   _EC200_SYS_FILE_SEEK2,      at_callback_success,        "OK",                   ",0,0\r"            },   
    
    {   _EC200_SYS_FILE_POS1,       _CbSYS_FILE_POSITION_1,     NULL,                    "AT+QFPOSITION="    },   
    {   _EC200_SYS_FILE_POS2,       at_callback_success,        "OK",                    "\r"                },   
    
    {   _EC200_SYS_FILE_READ1,      _CbSYS_FILE_READ_1,         NULL,                   "AT+QFREAD="        },    
    {   _EC200_SYS_FILE_READ2,      _CbSYS_FILE_READ_2,         "\r\nOK\r\n",           ",1024\r"           },   
    // FTP		
    {   _EC200_FTP_CONTEXT, 	    at_callback_success,        "OK",                   "AT+QFTPCFG=\"contextid\",1\r"		},
    {   _EC200_FTP_ACCOUNT, 	    at_callback_success,        "OK",                   "AT+QFTPCFG=\"account\",\"admin\",\"admin\"\r"		    },
            
    {   _EC200_FTP_FILE_TYPE, 	    at_callback_success,        "OK",                   "AT+QFTPCFG=\"filetype\",0\r"		},
    {   _EC200_FTP_TRANS_MODE,      at_callback_success,        "OK",                   "AT+QFTPCFG=\"transmode\",1\r"		},
    {   _EC200_FTP_RESP_TIME, 		at_callback_success,        "OK",                   "AT+QFTPCFG=\"rsptimeout\",60\r"	},
    
    {   _EC200_FTP_LOGIN_1, 	    _CbFTP_SEND_IP,             NULL,                   "AT+QFTPOPEN=\""   	},
    {   _EC200_FTP_LOGIN_2, 	    at_callback_success,        "OK",                   "\"\r"            	},
    
    {   _EC200_FTP_PATH_1,          _CbFTP_SEND_PATH,           NULL,                   "AT+QFTPCWD=\"/"    },
    {   _EC200_FTP_PATH_2,          at_callback_success,        "+CFTPGET:",            "\"\r"  	        },
    
    {   _EC200_FTP_FILE_DEL_1,      at_callback_success,        NULL,                   "AT+QFTPDEL=\""     },
    {   _EC200_FTP_FILE_DEL_2,      at_callback_success,        "OK",                   "\"\r"              },
    
    {   _EC200_FTP_FILE_SIZE_1,     at_callback_success,        NULL,                   "AT+QFTPSIZE=\""    },
    {   _EC200_FTP_FILE_SIZE_2,     at_callback_success,        "+QFTPSIZE:0, ",        "\"\r"              },
    
    {   _EC200_FTP_GET_TO_RAM_1,    _CbFTP_GET_FILE_1,          NULL,                   "AT+QFTPGET=\""     },
    {   _EC200_FTP_GET_TO_RAM_2,    _CbFTP_GET_FILE_2,          "+QFTPGET: 0,",         "\"\r"              },
    
    {   _EC200_FTP_READ_1,          _CbFTP_READ_1,              NULL,                   "AT+CFTPRDFILE=\"" 	},    
    {   _EC200_FTP_READ_2,          _CbFTP_READ_2,              "+CFTPRDFILE:SUCCESS",  "\r"               	},
    
    //SSL Config
    {   _EC200_HTTP_CFG_SSL_VER,    at_callback_success,        "OK",                   "AT+QSSLCFG=\"sslversion\",1,4\r"     	},
    {   _EC200_HTTP_CFG_SSL_CIP,    at_callback_success,        "OK",                   "AT+QSSLCFG=\"ciphersuite\",1,0x0005\r"	},
    {   _EC200_HTTP_CFG_SSL_LEV,    at_callback_success,        "OK",                   "AT+QSSLCFG=\"seclevel\",1,0\r"     	},
    
    {   _EC200_HTTP_CFG_SSL_CA,     at_callback_success,        "OK",                   "AT+QSSLCFG=\"cacert\",1,\"UFS:cacert.pem\"\r"     	        },
    {   _EC200_HTTP_CFG_SSL_CLC,    at_callback_success,        "OK",                   "AT+QSSLCFG=\"clientcert\",1,\"UFS:clientcert.pem\"\r"     	},
    {   _EC200_HTTP_CFG_SSL_CLK,    at_callback_success,        "OK",                   "AT+QSSLCFG=\"clientkey\",1,\"UFS:clientkey.pem\"\r"     	},    
    
    // HTTP	
    {   _EC200_HTTP_CFG_CONTEXT,    at_callback_success,        "OK",                   "AT+QHTTPCFG=\"contextid\",1\r"   	},
    {   _EC200_HTTP_CFG_HEADER1,    at_callback_success,        "OK",                   "AT+QHTTPCFG=\"requestheader\",0\r"  	},
    {   _EC200_HTTP_CFG_HEADER2,    at_callback_success,        "OK",                   "AT+QHTTPCFG=\"responseheader\",0\r"   	},
    {   _EC200_HTTP_CFG_SSL_CTX,    at_callback_success,        "OK",                   "AT+QHTTPCFG=\"sslctxid\",1\r"     	},
    
    {   _EC200_HTTP_SET_URL_1,      _CbHTTP_SETURL_1,           NULL,                   "AT+QHTTPURL="    	},
    {   _EC200_HTTP_SET_URL_2,      _CbHTTP_SETURL_2,           "CONNECT",              ",60\r"             },
    {   _EC200_HTTP_SET_URL_3,      at_callback_success,        "OK",                   NULL                },
    
    {   _EC200_HTTP_POST_UART_1,    Sim_Common_HTTP_Send_1,     NULL,                   "AT+QHTTPPOST="  	},
    {   _EC200_HTTP_POST_UART_2,    Sim_Common_HTTP_Send_2,     "CONNECT",              ",80,80\r"  	    },
    {   _EC200_HTTP_POST_UART_3,    Sim_Common_MQTT_Pub_2,      "\r\nOK",               NULL  	            },
        
    {   _EC200_HTTP_REQUEST_GET,    _CbHTTP_GET_REQUEST,        "+QHTTPGET:",           "AT+QHTTPGET=80\r"  	},  
    {   _EC200_HTTP_STORE_FILE1,    _CbHTTP_STORE_FILE1,        NULL,                   "AT+QHTTPREADFILE=\"UFS:"  	},
    {   _EC200_HTTP_STORE_FILE2,    at_callback_success,        "+QHTTPREADFILE: 0",    "\",80\r"  	},
    
    {   _EC200_HTTP_READ_1,         _CbHTTP_READ_1,             NULL,                   "AT$HTTPREAD=0,"    },
    {   _EC200_HTTP_READ_2,         _CbHTTP_READ_2,             "\r\nOK\r\n",           "\r",               },
    //GPS: GLOCA
    {   _EC200_GPS_CFG_TOKEN,       at_callback_success,        "OK",                   "AT+QLBSCFG=\"token\",\"V7Sk323082828KtB\"\r"    	},
    {   _EC200_GPS_CFG_FORMAT,      at_callback_success,        "OK",                   "AT+QLBSCFG=\"latorder\",1\r"   },
    {   _EC200_GPS_QUERY_TOKEN,     at_callback_success,        "OK",                   "AT+QLBSCFG=\"token\"\r"    	},
    {   _EC200_GPS_GET_LOCATION,    at_callback_success,        "OK",                   "AT+QLBS=0\r"       }, 
    //GPS: GNSS
    {   _EC200_GPS_CFG_OUT_PORT,    at_callback_success,        "OK",                   "AT+QGPSCFG=\"outport\",\"usbnmea\"\r"  }, // "AT+QGPSCFG=\"outport\",\"usbnmea\"\r"
    {   _EC200_GPS_CFG_AUTO_RUN,    at_callback_success,        "OK",                   "AT+QGPSCFG=\"autogps\",1\r"   }, 
    {   _EC200_GPS_DEL_CUR_DATA,    at_callback_success,        "OK",                   "AT+QGPSDEL=1\r"   },
    {   _EC200_GPS_TURN_ON,         at_callback_success,        "OK",                   "AT+QGPS=1\r"    	},
    {   _EC200_GPS_POSTION_INF,     _Cb_GPS_POSITION_INF,       "OK",                   "AT+QGPSLOC=2\r"    },
    {   _EC200_GPS_TURN_OFF,        at_callback_success,        "OK",                   "AT+QGPSEND\r"       },   
    
    //Audio 
    {   _EC200_AUDIO_FAR_END,       _Cb_AUDIO_FINISH,           "+QPSND: 0",             "AT+QPSND=1,\"test3.wav\",0,1,0\r"       },  
    
	// ...
	{	_EC200_END, 			    at_callback_success,        NULL,		             NULL,	            },
};

      
const sCommand_Sim_Struct aEC200Urc[] =
{
	{   _EC200_URC_RESET_SIM900,	Sim_Common_URC_Reset,       "NORMAL POWER DOWN",        NULL    },// OK
	{   _EC200_URC_SIM_LOST, 		Sim_Common_URC_Lost_Card,   "SIM CRASH",	            NULL 	},
	{   _EC200_URC_SIM_REMOVE, 	    Sim_Common_URC_Lost_Card,   "SIM REMOVED",		        NULL	},
	{   _EC200_URC_CLOSED,			_CbURC_CLOSED,              "+QIURC: \"closed\"",       NULL	},   
	{   _EC200_URC_PDP_DEACT, 		_CbURC_CLOSED,              "+NETWORK DISCONNECTED",    NULL	},   
	{   _EC200_URC_CALL_READY, 		Sim_Common_URC_Call_Ready,  "Call Ready",		        NULL    },
    {   _EC200_URC_CALLING, 		Sim_Common_URC_Calling,     "+CLIP:",                   NULL	},
    {   _EC200_URC_CALL_FAIL, 		Sim_Common_URC_Call_Fail,   "NO CARRIER",               NULL	}, 
    {   _EC200_URC_SMS_CONTENT, 	Sim_Common_URC_Sms_Content, "+CMGL:",                   NULL	},
    {   _EC200_URC_SMS_INDICATION1, Sim_Common_URC_Sms_Indi,    "+CMTI:",                   NULL	},
    {   _EC200_URC_SMS_INDICATION2, Sim_Common_URC_Sms_Indi,    "+CMT:",                    NULL	},
    
	{   _EC200_URC_ERROR, 			Sim_Common_URC_Error,       "ERROR",                    NULL	},
    {   _EC200_URC_RECEIV_SERVER, 	Sim_Common_URC_Recv_Server, "AMI/",			            NULL    },     
};


/*=============== Call back =================*/
static uint8_t at_callback_success(sData *uart_string)
{
    
        
	return 1;
}

/*==============Call back at sim ===============*/

static uint8_t _CbAT_SET_APN (sData *uart_string)
{
    Sim_Common_Send_AT_Cmd(sSimCommInfor.sAPNInfor->sName.Data_a8, sSimCommInfor.sAPNInfor->sName.Length_u16);
    
    return 1;
}

static uint8_t _CbAT_APN_AUTH (sData *uart_string)
{
    Sim_Common_Send_AT_Cmd( sSimCommInfor.sAPNInfor->sName.Data_a8, sSimCommInfor.sAPNInfor->sName.Length_u16 );
    Sim_Common_Send_AT_Cmd( (uint8_t*) "\",\"", 3 );
    Sim_Common_Send_AT_Cmd( sSimCommInfor.sAPNInfor->sUsername.Data_a8, sSimCommInfor.sAPNInfor->sUsername.Length_u16 );
    Sim_Common_Send_AT_Cmd( (uint8_t*) "\",\"", 3 );
    Sim_Common_Send_AT_Cmd( sSimCommInfor.sAPNInfor->sPassword.Data_a8, sSimCommInfor.sAPNInfor->sPassword.Length_u16 );
    Sim_Common_Send_AT_Cmd( (uint8_t*) "\"", 1 );
    
    return 1;
}


static uint8_t _CbAT_APN_ACTIVE (sData *uart_string)
{

    return 1;
}  



static uint8_t _CbAT_CHECK_RSSI(sData *uart_string)
{
    sData   strCheck = {(uint8_t*) aEC200Step[_EC200_AT_CHECK_RSSI].at_response, 
                                        strlen(aEC200Step[_EC200_AT_CHECK_RSSI].at_response)};
    sData strFind;
	int Pos = Find_String_V2(&strCheck, uart_string);
    
	if (Pos >= 0) 
    { 
        Pos += strCheck.Length_u16;
        strFind.Data_a8 = uart_string->Data_a8 + Pos ;
        strFind.Length_u16 = uart_string->Length_u16 - Pos;
            
        Sim_Common_Get_CSQ (&strFind);
    }
    
    return 1;
}

static uint8_t _CbAT_GET_CLOCK (sData *uart_string)
{
	sData  strCheck = {(uint8_t*) aEC200Step[_EC200_NET_GET_RTC].at_response, 
                                                strlen(aEC200Step[_EC200_NET_GET_RTC].at_response)};
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
    
    Posfix = Find_String_V2((sData*) &StConnectOk, uart_string);
	if (Posfix >= 0)
    {
        sSimCommVar.State_u8   = _SIM_OPENED_TCP;
        return 1;
    }
#else
    sData StConnectOk = {(uint8_t*)"+QIOPEN:", 8};    // +QIOPEN: 0,0   //check so '0' phia sau

    Posfix = Find_String_V2((sData*) &StConnectOk, uart_string);
	if (Posfix >= 0)
    {
        Posfix += StConnectOk.Length_u16;
        
        if (*(uart_string->Data_a8 + Posfix + 3) == '0')
        {
            sSimCommVar.State_u8   = _SIM_OPENED_TCP;
            return 1;
        }
    }
#endif
    
    return 0;
}

 

/*================== Call back File SYS, FTP va HTTP====================*/
//File sys callback
static uint8_t _CbSYS_DEL_FILE(sData *uart_string)
{
    Sim_Common_Send_AT_Cmd( (uint8_t *)&sFileSys.aNAME[0], sFileSys.LengthName_u8 );
    
    return 1;
}


static uint8_t _CbSYS_FILE_OPEN_1(sData *uart_string)
{
    Sim_Common_Send_AT_Cmd( (uint8_t *)&sFileSys.aNAME[0], sFileSys.LengthName_u8 );

    return 1;
}

static uint8_t _CbSYS_FILE_OPEN_2(sData *uart_string)
{
    //get file handle: +QFOPEN: <filehandle>   +QFOPEN: 20
    int Pos_Str = -1;
    uint16_t i = 0;
    uint16_t Temp = 0;
    
    sData strCheck = {(uint8_t*) aEC200Step[_EC200_SYS_FILE_OPEN2].at_response, strlen(aEC200Step[_EC200_SYS_FILE_OPEN2].at_response)};
    
	Pos_Str = Find_String_V2((sData*) &strCheck, uart_string);
    if (Pos_Str >= 0)
    {
        Pos_Str+= strCheck.Length_u16 + 1;
        for (i = Pos_Str; i < uart_string->Length_u16; i++)
        {
            if (*(uart_string->Data_a8 + i) == 0x0D)
                break;
                  
            if ( (*(uart_string->Data_a8 + i) < 0x30) || (*(uart_string->Data_a8 + i) > 0x39) )
            {
                return 0;
            }
            
             Temp = Temp * 10 + *(uart_string->Data_a8 + i) - 0x30;          
        }
        //Get File Handle
        sFileSys.Handle_u32 = Temp;
        //Convert to string
        Reset_Buff(&sFileSys.strHandle);
        UtilIntToString (sFileSys.Handle_u32, (char *) sFileSys.strHandle.Data_a8);
        sFileSys.strHandle.Length_u16 = strlen((char *) sFileSys.strHandle.Data_a8);
    }
        
    return 1;
}


static uint8_t _CbSYS_FILE_CLOSE_1(sData *uart_string)
{
    Sim_Common_Send_AT_Cmd( sFileSys.strHandle.Data_a8, sFileSys.strHandle.Length_u16 );

    return 1;
}

static uint8_t _CbSYS_FILE_SEEK_1(sData *uart_string)
{
    //<filehandle>
    Sim_Common_Send_AT_Cmd( sFileSys.strHandle.Data_a8, sFileSys.strHandle.Length_u16 );
    
    return 1;
}

static uint8_t _CbSYS_FILE_POSITION_1(sData *uart_string)
{
    //<filehandle>
    Sim_Common_Send_AT_Cmd( sFileSys.strHandle.Data_a8, sFileSys.strHandle.Length_u16 );
    
    return 1;
}
 

static uint8_t _CbSYS_FILE_READ_1(sData *uart_string)
{
    //<filehandle>[,<length>]
    Sim_Common_Send_AT_Cmd( sFileSys.strHandle.Data_a8, sFileSys.strHandle.Length_u16 );

    return 1;
}

/*
    Header fw: SVMM_LO_GW:SV_1_1_1,0x08004000@  
*/
static uint8_t _CbSYS_FILE_READ_2(sData *uart_string)
{
    /*
        Xu ly lay data fw ow day:
            + Lay data fw sau: CONNECT 1024\r\n va truoc \r\nOK\r\n
            + Kiem tra du 1024 byte?
            + Kiem tra header FW SV
            + Tinh crc va kiem tra
    */  
    int             Pos = 0;
    sData           strConnect      = {(uint8_t*)"\r\nCONNECT ", 10};
    sData           strHeaderFw;
    
    sData           strFix;
    uint16_t        i = 0, j = 0;
    uint16_t        NumbyteRecei = 0;
    uint8_t         aTEMP_DATA[8] = {0};
                   
    Pos = Find_String_V2((sData*) &strConnect, uart_string);
	if (Pos >= 0)
	{
        Pos += strConnect.Length_u16;
        //Get Number recv string
        for (i = Pos; i < uart_string->Length_u16; i++)
        {
            if ((*(uart_string->Data_a8 + i) < 0x30) || (*(uart_string->Data_a8 + i) > 0x39))
            {
                break;
            }
            
            NumbyteRecei = NumbyteRecei * 10 + *(uart_string->Data_a8 + i) - 0x30;
        }
        //phia tren i da tro den \r. can + 2 de tro den dau fw
        Pos = i + 2;   
        
        strFix.Data_a8 = uart_string->Data_a8 + Pos;
        strFix.Length_u16 = uart_string->Length_u16 - Pos;
        //check num byte recev
        if ( (NumbyteRecei == 0) || (NumbyteRecei > strFix.Length_u16 - 6) )    //6 byte cuoi \r\nOK\r\n
        {
            sSimFwUpdate.FailStatus_u8 = SEND_UPDATE_FIRMWARE_FAIL_LENGTH;
            sCbSimComm.pHandle_AT ( _SIM_COMM_HTTP_UPDATE_FAIL ); 
            return 0;
        }
        strFix.Length_u16 = NumbyteRecei;  //lay length dung bang length thuc te
            
        sSimFwUpdate.CountByteTotal_u32 += NumbyteRecei; 
        
        //Neu la Pack dau tien, Check 32 byte header xem co chu: ModemFirmware:SV1.1,0x0800C000
        strHeaderFw.Data_a8 = sCbSimComm.pGet_Fw_Version()->Data_a8;
        strHeaderFw.Length_u16 = LENGTH_HEADER_FW;
        
        if (sSimFwUpdate.CountPacket_u8 == 0)
        {
            Pos = Find_String_V2( (sData*)&strHeaderFw, &strFix );  
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
    //Set Fail Setup Http
    sSimFwUpdate.FailStatus_u8 = SEND_UPDATE_FIRMWARE_FAIL_SETUP;
    sCbSimComm.pHandle_AT ( _SIM_COMM_HTTP_UPDATE_FAIL ); 
    
    return 0;
}


//FTP Callback
static uint8_t _CbFTP_SEND_IP(sData *uart_string)
{
    Sim_Common_Send_AT_Cmd( sSimFwUpdate.sUpdateInfor.pFirmIP->Data_a8, sSimFwUpdate.sUpdateInfor.pFirmIP->Length_u16 );
    
    return 1;
}


//FTP Callback
static uint8_t _CbFTP_SEND_PATH(sData *uart_string)
{
    Sim_Common_Send_AT_Cmd( sSimFwUpdate.sUpdateInfor.pFirmPath->Data_a8, sSimFwUpdate.sUpdateInfor.pFirmPath->Length_u16 );
    
    return 1;
}

static uint8_t _CbFTP_GET_FILE_1(sData *uart_string)
{
    Sim_Common_Send_AT_Cmd( sSimFwUpdate.sUpdateInfor.pFirmName->Data_a8, sSimFwUpdate.sUpdateInfor.pFirmName->Length_u16 ); 
    Sim_Common_Send_AT_Cmd( (uint8_t *) "\",\"UFS:", 7 ); 
    Sim_Common_Send_AT_Cmd( (uint8_t *)&sFileSys.aNAME[0], sFileSys.LengthName_u8 );
    
    return 1;
}


static uint8_t _CbFTP_GET_FILE_2(sData *uart_string)
{
    int16_t     index =0;
    int16_t     Pos_Str = -1;
    uint8_t     statusM = 0;
    uint8_t     dataMemory[8] = {0};
    uint8_t     count = 0;
    sData    strCheck = {(uint8_t*) aEC200Step[_EC200_FTP_GET_TO_RAM_2].at_response, strlen(aEC200Step[_EC200_FTP_GET_TO_RAM_2].at_response)};
    
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

    sSimCommVar.State_u8   = _SIM_CONN_HTTP;
    //Init Struct Update Fw
    if (Sim_Common_Init_UpdateFw() == false)
        return 0;
    
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

static uint8_t _CbHTTP_SETURL_1(sData *uart_string)
{   
    char aTEMP[20] = {0};
    
    UtilIntToString (sSimFwUpdate.sUpdateInfor.pUrlHTTP->Length_u16, aTEMP);
    
    Sim_Common_Send_AT_Cmd( (uint8_t *) aTEMP, strlen(aTEMP) );

    return 1;
}

static uint8_t _CbHTTP_SETURL_2(sData *uart_string)
{ 
    sSimCommVar.pSender.Data_a8 = sSimFwUpdate.sUpdateInfor.pUrlHTTP->Data_a8;
    sSimCommVar.pSender.Length_u16 = sSimFwUpdate.sUpdateInfor.pUrlHTTP->Length_u16;  
    
    return 1;
}


//+QHTTPGET: 0,200,601710   : lay length tu phan hoi nay
static uint8_t _CbHTTP_GET_REQUEST(sData *uart_string)
{
    int16_t     index =0;
    int16_t     Pos_Str = -1;
    uint8_t     statusM = 0;
    uint8_t     dataMemory[8] = {0};
    uint8_t     count = 0;
    sData       strCheck = {(uint8_t*) aEC200Step[_EC200_HTTP_REQUEST_GET].at_response, strlen(aEC200Step[_EC200_HTTP_REQUEST_GET].at_response)};
    
	Pos_Str = Find_String_V2((sData*) &strCheck, uart_string);
    
    if (Pos_Str >= 0)
    {
        Pos_Str += strlen(aEC200Step[_EC200_HTTP_REQUEST_GET].at_response) + 1;   
        //tro den err va check '0'
        if (*(uart_string->Data_a8 + Pos_Str) != '0' )
        {
            return 0;
        }
        
        Pos_Str += 2;  //qua dau ',' dau tien
        
        //Lay length fw
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

    sSimCommVar.State_u8   = _SIM_CONN_HTTP; 
    //Init Struct Update Fw
    if (Sim_Common_Init_UpdateFw() == false)
        return 0;
    
    return 1;
}

static uint8_t _CbHTTP_STORE_FILE1(sData *uart_string)
{
    Sim_Common_Send_AT_Cmd( (uint8_t *)&sFileSys.aNAME[0], sFileSys.LengthName_u8 );
    
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


static uint8_t _Cb_AUDIO_FINISH(sData *uart_string)
{
    if (sCbSimComm.pHandle_AT != NULL)
        sCbSimComm.pHandle_AT(_SIM_COMM_AUDIO_OK);

    return 1;
}



// +QGPSLOC: <UTC>,<latitude>,<longitude>,
static uint8_t _Cb_GPS_POSITION_INF(sData *uart_string)
{
    sData strCheck = {(uint8_t *) "+QGPSLOC: ", 10};
    int PosFind = -1;
    uint8_t Result = false, CountPh = 0;
    uint16_t PosStart = 0, i = 0;
    uint16_t PosEnd = 0;
    
    PosFind = Find_String_V2((sData*) &strCheck, uart_string);
    if (PosFind >= 0)
    {
        PosFind += strCheck.Length_u16;
        //Tim kiem day phay tiep theo: la vi tri bat dau
        for (i = PosFind; i < uart_string->Length_u16; i++)
        {
            if (*(uart_string->Data_a8 + i) == ',')
            {
                CountPh++;
               
                if (CountPh == 1)
                {
                    //Vi tri dau phay dau tien: start
                    PosStart = i + 1;
                } else if (CountPh == 3)
                {
                    //Vi tri dau phay thu 3: Stop va ket thuc
                    PosEnd = i;
                    Result = true;
                    break;
                }
            }
        }
          
        if ( (Result == true) && (sCbSimComm.pGet_GPS_OK != NULL) )
        {
            sCbSimComm.pGet_GPS_OK((uart_string->Data_a8 + PosStart), PosEnd - PosStart);
        }
    }
    
    if (sCbSimComm.pGet_GPS_ERROR != NULL)
        sCbSimComm.pGet_GPS_ERROR();
    
    return 1;
}


static uint8_t _CbURC_CLOSED (sData *uart_string)
{    
    sData strCheck = {(uint8_t*) aEC200Urc[_EC200_URC_CLOSED].at_response, 
                                    strlen(aEC200Urc[_EC200_URC_CLOSED].at_response)};
    
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

/*
    Func: Get Timeout follow Cmd
        
*/

uint32_t EC200_Get_Timeout_Follow_Cmd (uint8_t sim_step)
{
    uint32_t time = 0;
    
    switch(sim_step)
    {
        case _EC200_TCP_CONNECT_2:
        case _EC200_MQTT_CONN_2:
        case _EC200_MQTT_SUB_2:
        case _EC200_MQTT_PUB_2:
        case _EC200_HTTP_STORE_FILE2:
            time = SIM_TCP_TIMEOUT;
            break;
        case _EC200_AUDIO_FAR_END:
            time = SIM_AUDIO_TIMEOUT;
            break;
        case _EC200_CALLING_ESTAB_2:
            time = SIM_CALL_TIMEOUT;
            break;
        case _EC200_NET_CHECK_ATTACH:
            time = SIM_CMD_TIMEOUT2;
            break;
        default:
            time = SIM_CMD_TIMEOUT;
            break;
    }
    
    return time;    
}


sCommand_Sim_Struct * EC200_Get_AT_Cmd (uint8_t step)
{
    return (sCommand_Sim_Struct *) &aEC200Step[step];
}


/*
    Func: Get Delay between 2 at cmd
*/
uint32_t EC200_Get_Delay_AT (uint8_t Step)
{
    uint32_t Time = 0;
    
    switch (Step)
    {
        case _EC200_TCP_OUT_DATAMODE:
            Time = 1000;
            break;
        case _EC200_HTTP_READ_2:
        case _EC200_FTP_READ_2:
            Time = 100;
            break;
        default:
            break;
    }
    
    return Time;
}


/*
    Func: check xem co bo qua lenh loi k
        return: true:
                    + thuc hien tiep cac lenh tiep theo
                    + bo qua bao nhieu lenh tiep theo lien quan
                false: thuc hien failure
*/
           
uint8_t EC200_Is_Ignore_Failure (uint8_t Step, uint8_t (*pJum_Next_At)(uint8_t Type))
{
    uint8_t nCmdRelate = 0, i = 0;
    uint8_t Result = false;
    
    switch (Step)
    {
        case _EC200_CALLING_ESTAB_2:
            Result = true;
            nCmdRelate = 3;
            break;
        case _EC200_AUDIO_FAR_END:
            Result = true;
            nCmdRelate = 2;
            break;
        case _EC200_CALLING_DISCONN:  
            Result = true;
            nCmdRelate = 1;
            break;
        default:
            break;
    }
    
    for (i = 0; i < nCmdRelate; i++)
    {
        pJum_Next_At(1);
    }

    return Result;
}


/*
    Func: Check step allow check URC
        + 1: Allow
        + 0: Not allow
*/
uint8_t EC200_Is_Step_Check_URC (uint8_t sim_step)
{
    if ((sim_step != _EC200_FTP_READ_1) && (sim_step != _EC200_FTP_READ_2)
        && (sim_step != _EC200_HTTP_READ_1) && (sim_step != _EC200_HTTP_READ_2)  
        && (sim_step != _EC200_SYS_FILE_READ1) && (sim_step != _EC200_SYS_FILE_READ2))  
    {
        return TRUE;
    }
    
    return FALSE;
}


uint8_t EC200_Check_Step_Skip_Error (uint8_t step)
{
    if ( step == _EC200_GPS_POSTION_INF )
    {
        sCbSimComm.pGet_GPS_ERROR (); 
    }
    
    if ( ( step == _EC200_GPS_POSTION_INF) || (step == _EC200_GPS_TURN_ON) || (step == _EC200_GPS_DEL_CUR_DATA) )
        return true;
    
    return false;
}



uint8_t EC200_Is_Send_Hex (uint8_t step)
{
    if ( ( step == _EC200_TCP_SEND_CMD_3)
         || (step == _EC200_MQTT_CONN_2) 
         || (step == _EC200_MQTT_SUB_2)
         || (step == _EC200_MQTT_PUB_2) 
         || (step == _EC200_MQTT_PUB_FB_2) 
         || (step == _EC200_MQTT_PUB_PING) 
         || (step == _EC200_HTTP_SET_URL_3) 
         || (step == _EC200_HTTP_POST_UART_3) 
        )
        return true;
    
    return false;
}



