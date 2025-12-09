

#ifndef _FTPC_H_
#define _FTPC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <limits.h>
#include <stdarg.h>
#include <stdlib.h>
#include "socket.h"
    
#include "user_util.h"

/* If you need this header, use it. */
#define LINELEN		100
#define _MAX_SS		1024

#define CTRL_SOCK	2
#define DATA_SOCK	3

/* FTP Responses */
#define R_150	150		/* File status ok; opening data conn */
#define R_200	200		/* 'Generic' command ok */
#define R_220	220		/* Service ready for new user. */
#define R_226	226		/* Closing data connection.  File transfer/abort successful */
#define R_227	227		/* Entering passive mode (h1,h2,h3,h4,p1,p2) */
#define R_230	230		/* User logged in, proceed */
#define R_331	331		/* User name okay, need password. */
#define R_250	250     /* Request file action ok*/		 

#define TransferAscii		'A'
#define TransferBinary		'I'

#define MAX_TIME_FTP        5000
        
enum ftpc_type 
{
	ASCII_TYPE,
	IMAGE_TYPE,
};

enum ftpc_datasock_state
{
	DATASOCK_IDLE,
	DATASOCK_READY,
	DATASOCK_START,
};

enum ftpc_datasock_mode
{
	PASSIVE_MODE,
	ACTIVE_MODE
};


typedef struct 
{
	enum ftpc_type  type;		/* Transfer type */

	enum ftpc_datasock_state    dsock_state;
	enum ftpc_datasock_mode     dsock_mode;
    
    uint8_t     RemoteIP[4];
    uint16_t    RemotePort;
      
    uint8_t     DataSockConnected_u8;
    uint8_t     DataGetStart_u8;
    uint8_t     CtrlReady_u8;
    uint32_t    LandMark_u32;
    uint8_t     Retry_u8;
}sEthFTPvariable;

/*========== Function ===============*/



#ifdef __cplusplus
}
#endif

#endif // _FTPC_H_
