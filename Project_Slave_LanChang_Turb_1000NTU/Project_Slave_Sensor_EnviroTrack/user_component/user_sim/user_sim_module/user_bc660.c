/*
 * user_bc66.c
 *
 *  Created on: Dec 31, 2021
 *      Author: Chien
 */

#include "string.h"

#include "user_bc660.h"
#include "user_string.h"

/*===============define======================*/
static uint8_t _Cb_AT_Success(sData *uart_string);
static uint8_t _Cb_AT_CHECK_RSSI(sData *uart_string);
static uint8_t _Cb_AT_GET_CLOCK (sData *uart_string);
static uint8_t _Cb_AT_TCP_OPEN_2 (sData *uart_string);
static uint8_t _Cb_URC_CLOSED (sData *uart_string);
static uint8_t _Cb_NET_Get_Eng (sData *uart_string);

static uint8_t _Cb_AT_PSM_Enable (sData *uart_string);
static uint8_t _Cb_URC_WAKEUP(sData *str_Receive);

/*========= var struct ==============*/

uint8_t aBC660_BLOCK_INIT[13] =
{
	//Pre Init
	_BC660_AT_AT,			    //0
	_BC660_AT_ECHO,		        //1
	_BC660_AT_ECHO,		        //2
	_BC660_DEEP_SLEEP_DIS,		//3
	_BC660_AT_BAUD_RATE,		//4
    _BC660_AT_REVISION,         //5
    _BC660_AT_GET_ID,			//6
    _BC660_CONF_EVENT_SLEEP,    //7
    _BC660_CONF_WAKE_PIN,       //8
	_BC660_AT_CHECK_SIM,		//9
	_BC660_AT_CHECK_RSSI,		//10
    
	_BC660_AT_SET_APN,			//11
	_BC660_NET_SET_BAND_3,      //12
};

uint8_t aBC660_BLOCK_NETWORK[5] = 
{
	_BC660_NET_CHECK_ATTACH,	//0
	//Setting
	_BC660_AT_EN_WAKE_URC,		//1
	_BC660_TCP_CONF_DATA,	    //2
	_BC660_eDRX_DIS,		    //3
	_BC660_AT_DIS_URC,	        //4
};

uint8_t aBC660_BLOCK_CONNECT[15] = 
{
    //Init TCP/IP
	_BC660_NET_CHECK_ATTACH,	//0
    _BC660_DEEP_SLEEP_DIS,		//1
    _BC660_PSM_DIS,             //2
    _BC660_NET_GET_ENG,         //3  _BC660_AT_CHECK_RSSI,		
	_BC660_NET_GET_RTC,			//4
	//Open TCP socket
	_BC660_TCP_OPEN_1,		    //5
	_BC660_TCP_OPEN_2,			//6

	_BC660_MQTT_CONN_1,         //7
    _BC660_TCP_SEND_CMD_1,      //8
    _BC660_TCP_SEND_CMD_2,      //9
	_BC660_MQTT_CONN_2,         //10

	_BC660_MQTT_SUB_1,          //11
    _BC660_TCP_SEND_CMD_1,      //12
    _BC660_TCP_SEND_CMD_2,      //13
	_BC660_MQTT_SUB_2,          //14
};

 uint8_t aBC660_BLOCK_PUB[4] =
{
    _BC660_MQTT_PUB_1,
    _BC660_TCP_SEND_CMD_1,
    _BC660_TCP_SEND_CMD_2,
    _BC660_TCP_SEND_CMD_3,
};

uint8_t aBC660_BLOCK_PUB_FB[4] =
{
    _BC660_MQTT_PUB_FB_1,
    _BC660_TCP_SEND_CMD_1,
    _BC660_TCP_SEND_CMD_2,
    _BC660_MQTT_PUB_FB_2,
};

uint8_t aBC660_BLOCK_PING[4] =
{
    _BC660_MQTT_PUB_FB_1,
    _BC660_TCP_SEND_CMD_1,
    _BC660_TCP_SEND_CMD_2,
    _BC660_MQTT_PUB_PING,
};        

uint8_t  aBC660_BLOCK_CSQ[1] = 
{
    _BC660_AT_CHECK_RSSI,
};


uint8_t aBC660_BLOCK_DIS_CONN[2] = 
{
	_BC660_TCP_CLOSE_1,
    _BC660_TCP_CLOSE_2,
};

uint8_t aBC660_BLOCK_CLOCK[1] = 
{
    _BC660_AT_CHECK_RSSI,
};

uint8_t aBC660_BLOCK_HTTP_INIT[1] = 
{
    _BC660_AT_CHECK_RSSI,
};




uint8_t aBC660_BLOCK_HTTP_READ[1] = 
{
    _BC660_AT_CHECK_RSSI,
};



//Read SMS
uint8_t aBC660_BLOCK_READ_SMS[1] = 
{
    _BC660_AT_CHECK_RSSI,
};  

//Read SMS
uint8_t aBC660_BLOCK_SEND_SMS[1] = 
{
    _BC660_AT_CHECK_RSSI,
};  

//Read SMS
uint8_t aBC660_BLOCK_CALLING[1] = 
{
    _BC660_AT_CHECK_RSSI,
};  

uint8_t aBC660_BLOCK_FILE_CLOSE[1] = 
{
    _BC660_AT_CHECK_RSSI,
};
    


uint8_t aBC660_BLOCK_FTP_INIT[1] = 
{
    _BC660_AT_CHECK_RSSI,
};


uint8_t aBC660_BLOCK_FTP_READ[1] = 
{
    _BC660_AT_CHECK_RSSI,
};


uint8_t aBC660_BLOCK_LOW_POWER[2] =
{
//    _BC660_AT_SET_CREG,
    _BC660_DEEP_SLEEP_EN, 
    _BC660_PSM_EN,
//    _BC660_AT_CHECK_CREG,
};



/*=====================Var struct============================*/
static char DataConack_8a[] = { MY_CONNACK, 0x02, 0x00 };		 //20020000
static char DataSuback_8a[] = { MY_SUBACK, 0x04, 0x00};          //9004000a00
static char DataPuback_8a[] = { MY_PUBACK, 0x02, 0x00};	         //40020001  , 0x1A 
static char DataPingResp_8a[] = { MY_PINGRESP, 0x00};	         // D000>


const sCommand_Sim_Struct aBC660Step[] =
{
	{   _BC660_AT_AT, 		        _Cb_AT_Success,             "OK", 			"AT\r"   	        },
    {   _BC660_AT_ECHO, 	        _Cb_AT_Success,			    "OK",			"ATE0\r"	        },
    
	// pre Setup
    {   _BC660_AT_REVISION, 		_Cb_AT_Success,		        "OK",           "AT+CGMR\r"	        },	
    {   _BC660_AT_GET_ID, 		    Sim_Common_Get_Sim_ID,      "+QCCID:",		"AT+QCCID\r"        },
	{   _BC660_AT_BAUD_RATE, 		_Cb_AT_Success,	            "OK",			"AT+IPR=115200\r"	},
	
	{   _BC660_AT_CHECK_SIM,        Sim_Common_CPIN_Ready,	    "OK",           "AT+CPIN?\r"	    },
	{   _BC660_AT_CHECK_RSSI, 		_Cb_AT_CHECK_RSSI,          "CSQ:",         "AT+CSQ\r"	        },
	{   _BC660_AT_SET_APN, 		    _Cb_AT_Success,		        "OK",           "AT+QCGDEFCONT=\"IP\",\"nbiot\"\r"	},	//v-internet    nbiot
    
    //config
    {   _BC660_AT_SOFT_RESET,       _Cb_AT_Success,             "RDY",          "AT+QRST=1\r"	    },
	
    {   _BC660_AT_SAVE_SETT, 		_Cb_AT_Success ,            "OK",           "AT&W\r"            },
	{   _BC660_AT_POWER_OFF, 		_Cb_AT_Success,		        "OK",			"AT+QPOWD=0\r"	    },
    
    {   _BC660_AT_EN_WAKE_URC,      _Cb_AT_Success,             "OK",           "AT+QNBIOTEVENT=1,1\r"		},
    
    {   _BC660_AT_EN_URC, 	        _Cb_AT_Success,		        "OK",			"AT+CSCON=1\r" 		},
	{   _BC660_AT_DIS_URC,          _Cb_AT_Success,		        "OK",			"AT+CSCON=0\r"		},
    
    {   _BC660_DEEP_SLEEP_DIS,  	_Cb_AT_Success,		        "OK",		    "AT+QSCLK=0\r"	    },
    {   _BC660_DEEP_SLEEP_EN,       _Cb_AT_Success,		        "OK",           "AT+QSCLK=1\r"  	},
    {   _BC660_LIGHT_SLEEP,         _Cb_AT_Success,		        "OK",           "AT+QSCLK=2\r" 		},
    
    {   _BC660_CONF_EVENT_SLEEP,	_Cb_AT_Success,	    	    "OK",           "AT+QCFG=\"dsevent\",1\r"       },
    {   _BC660_CONF_WAKE_PIN,	    _Cb_AT_Success,	            "OK",           "AT+QCFG=\"wakeupRXD\",1\r"     },
    
    //net
    {   _BC660_NET_CHECK_ATTACH, 	Sim_Common_Check_Attached,	"OK",		    "AT+CGATT?\r" 		},
	{   _BC660_NET_SET_BAND_3, 	    _Cb_AT_Success,	            "OK",			"AT+QBAND=2,1,3\r"  },
	{   _BC660_NET_SET_BAND_ALL, 	_Cb_AT_Success,	            "OK",			"AT+QBAND=0\r"  	},

    {   _BC660_NET_GET_ENG, 	    _Cb_NET_Get_Eng,	        "OK",			"AT+QENG=0\r"  	    },    // +QENG: 0,1769,9,400,"00288516",-103,-9,-94,5,3,"A924",0,14,2
	{   _BC660_NET_GET_RTC, 		_Cb_AT_GET_CLOCK,	        "+CCLK:", 		"AT+CCLK?\r"   	    },

    {   _BC660_PSM_DIS,  	        _Cb_AT_Success,	            "OK",           "AT+CPSMS=0\r"	    },
    {   _BC660_PSM_EN,  	        _Cb_AT_PSM_Enable,	         "OK",           "AT+CPSMS=1,,,\"00111100\",\"00000001\"\r"    }, //3h va 4s
    
    {   _BC660_eDRX_EN,    	        _Cb_AT_Success,             "OK",           "AT+CEDRXS=1,5,\"0101\"\r"      },    //ENABLE eDRX
    {   _BC660_eDRX_DIS,            _Cb_AT_Success,	            "OK",           "AT+CEDRXS=0\r"	    },
    
    // TCP
    {   _BC660_TCP_CONF_DATA,	    _Cb_AT_Success,             "OK",           "AT+QICFG=\"dataformat\",0,0\r" },
    
    {   _BC660_AT_SET_CREG, 		_Cb_AT_Success,             "OK",           "AT+CEREG=5\r"      },
    {   _BC660_AT_CHECK_CREG, 		_Cb_AT_Success,             "CEREG: 0,|1|5", "AT+CEREG?\r"      }, 

    {   _BC660_TCP_CONF_MODE_ACC,	_Cb_AT_Success,	            "OK",           "AT+QISWTMD=0,0\r"	},
    {   _BC660_TCP_CONF_MODE_PUSH,	_Cb_AT_Success,             "OK",           "AT+QISWTMD=0,1\r"	},

    {   _BC660_TCP_OPEN_1, 		    Sim_Common_TCP_Open_1,	    NULL,           "AT+QIOPEN=0,"  	    },
    {   _BC660_TCP_OPEN_2, 		    _Cb_AT_TCP_OPEN_2,  		"+QIOPEN:",     ",0,1\r"	            },

    {	_BC660_TCP_CLOSE_1,	 	    Sim_Common_TCP_Close,       NULL,	        "AT+QICLOSE="  	},
    {	_BC660_TCP_CLOSE_2,	 	    _Cb_AT_Success,             "OK",	        "\r"  	}, 
        
    {   _BC660_TCP_SEND_CMD_1, 	    Sim_Common_TCP_Send_1,      NULL,           "AT+QISEND="      	},
    {   _BC660_TCP_SEND_CMD_2,      Sim_Common_TCP_Send_2,      ">",            "\r"	 		   	},
    {   _BC660_TCP_SEND_CMD_3, 	    Sim_Common_MQTT_Pub_2,      "SEND OK",	    NULL            	},
    
    //mqtt
	{   _BC660_MQTT_CONN_1, 		Sim_Common_MQTT_Conn_1,     NULL,	        NULL                },
	{   _BC660_MQTT_CONN_2, 		Sim_Common_MQTT_Conn_2,	    DataConack_8a,  NULL                },  //4,"20020000"

	{   _BC660_MQTT_SUB_1, 		    Sim_Common_MQTT_Sub_1,      NULL,	        NULL   	            },
	{   _BC660_MQTT_SUB_2, 		    Sim_Common_MQTT_Sub_2,	    DataSuback_8a,  NULL	            },

	{   _BC660_MQTT_PUB_1, 		    Sim_Common_MQTT_Pub_1,      NULL,			NULL  	            },
	{   _BC660_MQTT_PUB_2, 		    Sim_Common_MQTT_Pub_2,	    NULL,   		NULL 	            },  //"4,40020001"  "SEND OK"

	{   _BC660_MQTT_PUB_FB_1, 	    Sim_Common_MQTT_Pub_1,      NULL,			NULL                },
	{   _BC660_MQTT_PUB_FB_2, 	    Sim_Common_MQTT_Pub_2,	   	DataPuback_8a,  NULL  	            },  // 4,"40020001"

    {   _BC660_MQTT_PUB_PING, 	    Sim_Common_MQTT_Pub_2,      DataPingResp_8a, NULL       	    }, 
    
	{   _BC660_END, 		        _Cb_AT_Success ,            NULL,		    NULL                },
};



const sCommand_Sim_Struct aBC660Urc[] =
{
	{   _BC660_URC_RESET_SIM900,        _Cb_AT_Success,             "NORMAL POWER DOWN",        NULL    }, // OK +QNBIOTEVENT: "ENTER PSM"
	{   _BC660_URC_CLOSED,			    _Cb_URC_CLOSED,             "+QIURC: \"closed\"",       NULL	},  
	{   _BC660_URC_PDP_DEACT,		    _Cb_AT_Success, 	        "+SERVER DISCONNECTED",     NULL	},
	{   _BC660_URC_ERROR, 			    Sim_Common_URC_Error,	    "ERROR",		   	        NULL	},
	{   _BC660_URC_WAKEUP,	 		    _Cb_URC_WAKEUP,	            "+QATWAKEUP",				NULL	},
	{   _BC660_URC_DEEP_SLEEP, 		    _Cb_AT_Success,	            "ENTER DEEPSLEEP",	  	    NULL	},
    
    {   _BC660_URC_RECEIV_SERVER, 	    Sim_Common_URC_Recv_Server,	"AMI/",			        NULL	},   //"AMI/"   
};



/*====================Function call back============================*/

static uint8_t _Cb_AT_Success(sData *uart_string)
{
    
        
	return 1;
}

static uint8_t _Cb_AT_CHECK_RSSI(sData *uart_string)
{
    sData   strCheck = {(uint8_t*) aBC660Step[_BC660_AT_CHECK_RSSI].at_response, 
                                        strlen(aBC660Step[_BC660_AT_CHECK_RSSI].at_response)};
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



static uint8_t _Cb_AT_GET_CLOCK (sData *uart_string)
{
	sData  strCheck = {(uint8_t*) aBC660Step[_BC660_NET_GET_RTC].at_response, 
                                                strlen(aBC660Step[_BC660_NET_GET_RTC].at_response)};
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


// +QENG: 0,1769,9,400,"00288516",-103,-9,-94,5,3,"A924",0,14,2

static uint8_t _Cb_NET_Get_Eng (sData *uart_string)
{
    sData strCheck = {(uint8_t*) "+QENG: 0", strlen("+QENG: 0")};
    
	int Pos = Find_String_V2(&strCheck, uart_string);
    char aTEMP[20] = {0};
    uint8_t length = 0;
    
    
	if (Pos >= 0) 
    { 
        Pos += strCheck.Length_u16 + 1;
        
        //EarFcn
        length = UTIL_Cut_String((char *) (uart_string->Data_a8 + Pos), aTEMP, uart_string->Length_u16 - Pos);
        if (length > 0)
            sSimCommInfor.EarFcn_u16 = (uint16_t) UtilStringToInt(aTEMP);
        
        //EarFcn offset
        Pos += length + 1;
        UTIL_MEM_set(aTEMP, 0, sizeof(aTEMP));
        length = UTIL_Cut_String((char *) (uart_string->Data_a8 + Pos), aTEMP, uart_string->Length_u16 - Pos);
        if (length > 0)
            sSimCommInfor.EarFcnOffs_u16 = (uint16_t) UtilStringToInt(aTEMP);
        
        //npci
        Pos += length + 1;
        UTIL_MEM_set(aTEMP, 0, sizeof(aTEMP));
        length = UTIL_Cut_String((char *) (uart_string->Data_a8 + Pos), aTEMP, uart_string->Length_u16 - Pos);
        if (length > 0)
            sSimCommInfor.nCellID_u16 = (uint16_t) UtilStringToInt(aTEMP);
        
        //cell id
        Pos += length + 1;
        UTIL_MEM_set(aTEMP, 0, sizeof(aTEMP));
        length = UTIL_Cut_String((char *) (uart_string->Data_a8 + Pos), aTEMP, uart_string->Length_u16 - Pos);
        if (length > 0)
        {
            UTIL_MEM_set(sSimCommInfor.aCELL_ID, 0, sizeof(sSimCommInfor.aCELL_ID));
            UTIL_MEM_cpy(sSimCommInfor.aCELL_ID, aTEMP, strlen(aTEMP));
        }
        //RSRP
        Pos += length + 1;
        UTIL_MEM_set(aTEMP, 0, sizeof(aTEMP));
        length = UTIL_Cut_String((char *) (uart_string->Data_a8 + Pos), aTEMP, uart_string->Length_u16 - Pos);
        if (length > 0)
            sSimCommInfor.scRSRP_i16 = (uint16_t) UtilStringToInt(aTEMP);
        
        //RSRQ
        Pos += length + 1;
        UTIL_MEM_set(aTEMP, 0, sizeof(aTEMP));
        length = UTIL_Cut_String((char *) (uart_string->Data_a8 + Pos), aTEMP, uart_string->Length_u16 - Pos);
        if (length > 0)
            sSimCommInfor.scRSRQ_i16 = (uint16_t) UtilStringToInt(aTEMP);
        
        //RSSI
        Pos += length + 1;
        UTIL_MEM_set(aTEMP, 0, sizeof(aTEMP));
        length = UTIL_Cut_String((char *) (uart_string->Data_a8 + Pos), aTEMP, uart_string->Length_u16 - Pos);
        if (length > 0)
            sSimCommInfor.scRSSI_i16 = (uint16_t) UtilStringToInt(aTEMP);
        
        //bo qua snr
        Pos += length + 1;
        UTIL_MEM_set(aTEMP, 0, sizeof(aTEMP));
        length = UTIL_Cut_String((char *) (uart_string->Data_a8 + Pos), aTEMP, uart_string->Length_u16 - Pos); 
        
        //Band
        Pos += length + 1;
        UTIL_MEM_set(aTEMP, 0, sizeof(aTEMP));
        length = UTIL_Cut_String((char *) (uart_string->Data_a8 + Pos), aTEMP, uart_string->Length_u16 - Pos);
        if (length > 0)
            sSimCommInfor.Band_u8 = (uint16_t) UtilStringToInt(aTEMP);
    }
    
    
    return 1;
}



static uint8_t _Cb_AT_PSM_Enable (sData *uart_string)
{
    UTIL_Log_Str(DBLEVEL_M, "u_bc660: go to psm mode!\r\n" );

    sSimCommVar.State_u8 = _SIM_PSM;
    
    return 1;
}


static uint8_t _Cb_AT_TCP_OPEN_2 (sData *uart_string)
{
    int Posfix = 0;

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
    
    return 0;
}


static uint8_t _Cb_URC_CLOSED (sData *uart_string)
{    
    sData strCheck = {(uint8_t*) aBC660Urc[_BC660_URC_CLOSED].at_response, 
                                    strlen(aBC660Urc[_BC660_URC_CLOSED].at_response)};
    
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



static uint8_t _Cb_URC_WAKEUP(sData *str_Receive)
{
    sSimCommVar.Wakeup_u8 = true;

	return 1;
}


/*========== function handle==========*/

sCommand_Sim_Struct * BC660_Get_AT_Cmd (uint8_t step)
{
    return (sCommand_Sim_Struct *) &aBC660Step[step];
}


uint32_t BC660_Get_Delay_AT (uint8_t Step)
{    
    return 0;
}
       

/*
    Func: Get Timeout follow Cmd
        
*/

uint32_t BC660_Get_Timeout_Follow_Cmd (uint8_t sim_step)
{
    uint32_t time = 0;
        
    switch(sim_step)
    {
        case _BC660_TCP_OPEN_2:
        case _BC660_MQTT_CONN_2:
        case _BC660_MQTT_SUB_2:
        case _BC660_MQTT_PUB_2:
            time = SIM_TCP_TIMEOUT;
            break;
        case _BC660_NET_CHECK_ATTACH:
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
           
uint8_t BC660_Is_Ignore_Failure (uint8_t Step, uint8_t (*pJum_Next_At)(uint8_t Type))
{
    return false;
}



/*
    Func: Check step allow check URC
        + 1: Allow
        + 0: Not allow
*/
uint8_t BC660_Is_Step_Check_URC (uint8_t sim_step)
{    
    return false;
}


uint8_t BC660_Check_Step_Skip_Error (uint8_t step)
{
    
    return false;
}



uint8_t BC660_Is_Send_Hex (uint8_t step)
{
    if ( ( step == _BC660_TCP_SEND_CMD_3)
         || (step == _BC660_MQTT_CONN_2) 
         || (step == _BC660_MQTT_SUB_2)
         || (step == _BC660_MQTT_PUB_2) 
         || (step == _BC660_MQTT_PUB_FB_2) 
         || (step == _BC660_MQTT_PUB_PING) 
        )
        return true;
    
    return false;
}


