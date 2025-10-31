

#include "user_at_serial.h"
#include "user_string.h"
#include "onchipflash.h"

/*======================== Structs var======================*/

struct_CheckList_AT CheckList_AT_CONFIG[] =
{
//			id ENUM		    kind process		  str Received
		{_SET_DEV_SERIAL, 		NULL,		    {(uint8_t*)"at+seri=",8}},  //ex:
		{_QUERY_DEV_SERIAL,		NULL,		    {(uint8_t*)"at+seri?",8}},
        
        {_QUERY_SIM_ID,		    NULL,		    {(uint8_t*)"at+simseri?",11}},
        
		{_SET_SERVER_MAIN,	 	NULL,	        {(uint8_t*)"at+mqttmain=",12}},     //Ex: at+mqttmain=124.158.5.154,1883,syswmsv,sv@wmsv568
		{_QUERY_SERVER_MAIN, 	NULL,	        {(uint8_t*)"at+mqttmain?",12}},
        
        {_SET_SERVER_BACKUP,	NULL,	        {(uint8_t*)"at+mqttbackup=",14}},   //Ex: at+mqttbackup=mqtt.saovietgroup.com.vn,7525,syswmsv,sv@wmsv568
		{_QUERY_SERVER_BACKUP, 	NULL,	        {(uint8_t*)"at+mqttbackup?",14}},
        
        {_REQ_SWITCH_SERVER, 	NULL,	        {(uint8_t*)"at+swserver",11}},

        {_SET_DUTY_READ_DATA,	NULL,	        {(uint8_t*)"at+duty=",8}},          //ex: at+duty=03x30 at+duty=03x30
        {_QUERY_DUTY_READ_DATA,	NULL,	        {(uint8_t*)"at+duty?",8}},
        
        {_SET_FREQ_ONLINE,	    NULL,	        {(uint8_t*)"at+freqmin=",11}},      // at+freqmin=5
        {_QUERY_FREQ_ONLINE,	NULL,	        {(uint8_t*)"at+freqmin?",11}},
        
        {_SET_DEFAULT_PULSE, 	NULL,	        {(uint8_t*)"at+resetpulse?",14}},
        {_SET_SAVE_BOX, 	    NULL,	        {(uint8_t*)"at+savebox?",11}},      //ex:at+savebox
        {_SET_NUM_WAKEUP, 	    NULL,	        {(uint8_t*)"at+numwake=",11}},     //ex: at+numwake=05
        {_READ_OLD_DATA, 	    NULL,	        {(uint8_t*)"at+readlog=",11}},     //ex: at+numwake=05
        
        {_SET_FLOW_THRESH, 		NULL,		    {(uint8_t*)"at+flowthresh=",14}},  // at+flowthresh=1234,12345
		{_QUERY_FLOW_THRESH, 	NULL,	        {(uint8_t*)"at+flowthresh?",14}},
        
        {_SET_QUAN_THRESH, 	    NULL,		    {(uint8_t*)"at+quanthresh=",14}},   // at+quanthresh=1234,12345
		{_QUERY_QUAN_THRESH, 	NULL,	        {(uint8_t*)"at+quanthresh?",14}},
        
        {_SET_BAT_THRESH, 		NULL,		    {(uint8_t*)"at+batthresh=",13}},    // at+batthresh=12
		{_QUERY_BAT_THRESH, 	NULL,	        {(uint8_t*)"at+batthresh?",13}},
        
        {_SET_PULSE_VALUE, 		NULL,		    {(uint8_t*)"at+pulse=",9}},         // at+pulse=20
        {_QUERY_PULSE_VALUE, 	NULL,           {(uint8_t*)"at+pulse?",9}},
    
        {_SET_RTC, 		        NULL,		    {(uint8_t*)"at+rtc=",7}},
        {_QUERY_RTC, 			NULL,	        {(uint8_t*)"at+rtc?",7}},

        {_SET_LEVEL_DEBUG,	    NULL,	        {(uint8_t*)"at+debug=",9}},
        {_QUERY_LEVEL_DEBUG,    NULL,	        {(uint8_t*)"at+debug?",9}},
        
        {_RESET_MODEM, 	        NULL,           {(uint8_t*)"at+mreset?",10}},
        {_ERASE_MEMORY, 	    NULL,           {(uint8_t*)"at+erasemem?",12}},
        
        {_QUERY_FIRM_VER,       NULL,	        {(uint8_t*)"at+fwversion?",13}},    
        {_QUERY_UP_FIRM_HTTP,   NULL,	        {(uint8_t*)"at+userurl=",11}},      
        {_QUERY_UP_FIRM_FTP,    NULL,	        {(uint8_t*)"at+userftp=",11}},
        
        {_QUERY_INDEX_LOG,      NULL,	        {(uint8_t*)"at+logindex?",12}},     //Vi tri index log
        {_TEST_LOG,             NULL,	        {(uint8_t*)"at+testlog?",11}},      //Ghi vao 1 ban tin log
        {_QUERY_READ_ALL_LOG,   NULL,	        {(uint8_t*)"at+readlog?",11}},      //Doc tat ca cac log
        {_QUERY_READ_LAST_LOG,  NULL,	    	{(uint8_t*)"at+rdlastlog?",13}},     //Doc n long gan nhat
        {_QUERY_LAST_FOR_LOG,   NULL,	        {(uint8_t*)"at+rdlastforlog=",16}},  // at+rdlastforlog=(200)
        
        {_QUERY_LAST_FOR_OPERA, NULL,	        {(uint8_t*)"at+rdlastforopera=",18}}, // at+rdlastforopera=(200)
        {_QUERY_LAST_FOR_EVENT, NULL,	        {(uint8_t*)"at+rdlastforevent=",18}}, 
        
        {_SET_CALIB_LEVEL,      NULL,	        {(uint8_t*)"at+levelcalib=",14}},   //at+levelcalib=-14
        {_QUERY_CALIB_LEVEL,    NULL,	        {(uint8_t*)"at+levelcalib?",14}}, 
        
        {_SET_LEVEL_THRESH, 	NULL,		    {(uint8_t*)"at+levelthresh=",15}},  // at+levelthresh=1234,12345
		{_QUERY_LEVEL_THRESH, 	NULL,	        {(uint8_t*)"at+levelthresh?",15}},
        
        {_QUERY_GPS_LOC,        NULL,			{(uint8_t*)"at+gps?",7}},
        {_QUERY_LAST_FOR_GPS,   NULL,	        {(uint8_t*)"at+rdlastforgps=",16}}, 
        
        {_SET_LIST_ID_SLAVE,	NULL,	        {(uint8_t*)"at+slave=",9}},
        {_QUERY_LIST_ID_SLAVE,	NULL,	        {(uint8_t*)"at+slave?",9}}, 
        
        {_SET_EX_BAT_THRESH,	NULL,	        {(uint8_t*)"at+exbatthresh=",15}},
        {_QUERY_EX_BAT_THRESH,	NULL,	        {(uint8_t*)"at+exbatthresh?",15}},
        
        {_SET_POWER_ON_TEMH,	NULL,	        {(uint8_t*)"at+ontemh?",10}},
        {_SET_POWER_OFF_TEMH,	NULL,	        {(uint8_t*)"at+offtemh?",11}},
                
        {_SET_NUM_CHANNEL,	    NULL,	        {(uint8_t*)"at+wmchannel=",13}},
        {_QUERY_NUM_CHANNEL,	NULL,	        {(uint8_t*)"at+wmchannel?",13}},
        
        {_SET_MODBUS_INFOR,	    NULL,	        {(uint8_t*)"at+modinfor=",12}},
        {_QUERY_MODBUS_INFOR,	NULL,	        {(uint8_t*)"at+modinfor?",12}},
        
        {_SET_PRESS_INFOR,	    NULL,	        {(uint8_t*)"at+pressinfor=",14}},  //at+pressinfor=0,400,2000,0,10000
        {_QUERY_PRESS_INFOR,	NULL,	        {(uint8_t*)"at+pressinfor?",14}},
        
        {_GET_STATE_SENSOR,	    NULL,	        {(uint8_t*)"at+statesensor?",15}},
        {_GET_MEASURE_VALUE,    NULL,           {(uint8_t*)"at+measurevalue?",16}},
        {_GET_MEASURE_FILTER,   NULL,           {(uint8_t*)"at+measurefilter?",17}},
        
        {_GET_FREQ_WARNING_SENSOR, NULL,        {(uint8_t*)"at+freqwarningsensor?",21}},
        {_SET_FREQ_WARNING_SENSOR, NULL,        {(uint8_t*)"at+freqwarningsensor=",21}},
        
        {_GET_OFFSET_CLO,       NULL,           {(uint8_t*)"at+offsetclo?",13}},
        {_SET_OFFSET_CLO,       NULL,           {(uint8_t*)"at+offsetclo=",13}},
        
        {_GET_OFFSET_PH,        NULL,           {(uint8_t*)"at+offsetph?",12}},
        {_SET_OFFSET_PH,        NULL,           {(uint8_t*)"at+offsetph=",12}},
        
        {_GET_OFFSET_NTU,       NULL,           {(uint8_t*)"at+offsetntu?",13}},
        {_SET_OFFSET_NTU,       NULL,           {(uint8_t*)"at+offsetntu=",13}},
        
        {_END_AT_CMD,	        NULL,           {(uint8_t*)"at+end",6}},
};

uint8_t PortConfig = 0;

/*======================== Function ======================*/


