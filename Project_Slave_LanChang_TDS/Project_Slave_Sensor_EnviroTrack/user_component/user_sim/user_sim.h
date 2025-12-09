/*
 * user_sim.h
 *
 *  Created on: Dec 31, 2021
 *      Author: Chien
 */

#ifndef USER_SIM_H_
#define USER_SIM_H_

#include "user_util.h"

#include "queue_p.h"
#include "event_driven.h"

#include "user_sim_common.h"
#include "user_sim_config.h"



//#define SIM_L506_LIB
#define SIM_EC200U_LIB  
//#define SIM_L511_LIB  
//#define SIM_BC660_LIB  


/*======= Define Type SIM: SIM_L506_LIB =============*/  
#ifdef SIM_L506_LIB
    #include "user_l506.h"
    #define SIM_STEP_END                                _L506_END
    #define SIM_STEP_OUT_DATA_MODE                      _L506_TCP_OUT_DATAMODE
    #define SIM_STEP_ATTACH                             _L506_NET_CHECK_ATTACH

    #define SIM_GET_AT_CMD(sim_step)                    L506_Get_AT_Cmd(sim_step) 
     
    #define SIM_GET_TIMEOUT(sim_step)                   L506_Get_Timeout_Follow_Cmd(sim_step)    
    #define SIM_GET_DELAY_AT(sim_step)                  L506_Get_Delay_AT(sim_step) 
    #define SIM_IS_IGNORE_FAILURE(sim_step, pJum_AT)    L506_Is_Ignore_Failure(sim_step, pJum_AT)

    #define SIM_IS_STEP_CHECK_URC(sim_step)             L506_Is_Step_Check_URC(sim_step)  
    #define SIM_IS_STEP_SKIP_ERROR(sim_step)            L506_Check_Step_Skip_Error(sim_step) 
    #define SIM_IS_SEND_HEX(sim_step)                   L506_Is_Send_Hex(sim_step) 

     /*Block step*/
    #define aSIM_BLOCK_STEP_INIT                        aL506_BLOCK_INIT
    #define aSIM_BLOCK_STEP_NETWORK                     aL506_BLOCK_NETWORK
    #define aSIM_BLOCK_STEP_CONNECT                     aL506_BLOCK_CONNECT
    #define aSIM_BLOCK_STEP_DIS_CONN                    aL506_BLOCK_DIS_CONN

    #define aSIM_BLOCK_STEP_PUB_FB                      aL506_BLOCK_PUB_FB
    #define aSIM_BLOCK_STEP_PUB                         aL506_BLOCK_PUB
    #define aSIM_BLOCK_STEP_PING                        aL506_BLOCK_PING

    #define aSIM_BLOCK_STEP_CSQ                         aL506_BLOCK_CSQ
    #define aSIM_BLOCK_STEP_READ_SMS                    aL506_BLOCK_READ_SMS
    #define aSIM_BLOCK_STEP_SEND_SMS                    aL506_BLOCK_SEND_SMS
    #define aSIM_BLOCK_STEP_CALL                        aL506_BLOCK_CALLING
    #define aSIM_BLOCK_STEP_CLOCK                       aL506_BLOCK_CLOCK

    #define aSIM_BLOCK_STEP_HTTP_INIT                   aL506_BLOCK_HTTP_INIT
    #define aSIM_BLOCK_STEP_HTTP_READ                   aL506_BLOCK_HTTP_READ

    #define aSIM_BLOCK_STEP_FTP_INIT                    aL506_BLOCK_FTP_INIT
    #define aSIM_BLOCK_STEP_FTP_READ                    aL506_BLOCK_FTP_READ
     
    #define aSIM_BLOCK_STEP_HTTP_HEAD                   aL506_BLOCK_HTTP_HEAD  
    #define aSIM_BLOCK_STEP_HTTP_POST                   aL506_BLOCK_HTTP_POST  

    #define sSIM_URC                                    aL506Urc
    #define SIM_URC_FIRST                               _L506_URC_RESET_SIM900
    #define SIM_URC_END                                 _L506_URC_END
#endif

/*======= Define Type SIM: SIM_EC200U LIB =============*/  
#ifdef SIM_EC200U_LIB
    #include "user_ec200u.h"
    #define SIM_STEP_END                                _EC200_END
    #define SIM_STEP_OUT_DATA_MODE                      _EC200_TCP_OUT_DATAMODE
    #define SIM_STEP_ATTACH                             _EC200_NET_CHECK_ATTACH
    #define SIM_STEP_CALLING                            _EC200_CALLING_ESTAB_2
     
    #define SIM_GET_AT_CMD(sim_step)                    EC200_Get_AT_Cmd(sim_step) 
     
    #define SIM_GET_TIMEOUT(sim_step)                   EC200_Get_Timeout_Follow_Cmd(sim_step)    
    #define SIM_GET_DELAY_AT(sim_step)                  EC200_Get_Delay_AT(sim_step) 
    #define SIM_IS_IGNORE_FAILURE(sim_step, pJum_AT)    EC200_Is_Ignore_Failure(sim_step, pJum_AT)

    #define SIM_IS_STEP_CHECK_URC(sim_step)             EC200_Is_Step_Check_URC(sim_step)
    #define SIM_IS_STEP_SKIP_ERROR(sim_step)            EC200_Check_Step_Skip_Error(sim_step)
    #define SIM_IS_SEND_HEX(sim_step)                   EC200_Is_Send_Hex(sim_step) 

    /*Block step*/
    #define aSIM_BLOCK_STEP_INIT                        aEC200_BLOCK_INIT
    #define aSIM_BLOCK_STEP_NETWORK                     aEC200_BLOCK_NETWORK
    #define aSIM_BLOCK_STEP_CONNECT                     aEC200_BLOCK_CONNECT
    #define aSIM_BLOCK_STEP_DIS_CONN                    aEC200_BLOCK_DIS_CONN

    #define aSIM_BLOCK_STEP_PUB_FB                      aEC200_BLOCK_PUB_FB
    #define aSIM_BLOCK_STEP_PUB                         aEC200_BLOCK_PUB
    #define aSIM_BLOCK_STEP_PING                        aEC200_BLOCK_PING

    #define aSIM_BLOCK_STEP_CSQ                         aEC200_BLOCK_CSQ
    #define aSIM_BLOCK_STEP_READ_SMS                    aEC200_BLOCK_READ_SMS
    #define aSIM_BLOCK_STEP_SEND_SMS                    aEC200_BLOCK_SEND_SMS
    #define aSIM_BLOCK_STEP_CALL                        aEC200_BLOCK_CALLING
    #define aSIM_BLOCK_STEP_CLOCK                       aEC200_BLOCK_CLOCK

    #define aSIM_BLOCK_STEP_HTTP_INIT                   aEC200_BLOCK_HTTP_INIT
    #define aSIM_BLOCK_STEP_HTTP_READ                   aEC200_BLOCK_HTTP_READ

    #define aSIM_BLOCK_STEP_FILE_CLOSE                  aEC200_BLOCK_FILE_CLOSE

    #define aSIM_BLOCK_STEP_GNSS                        aEC200_BLOCK_GNSS
    #define aSIM_BLOCK_STEP_GNSS_DELL                   aEC200_BLOCK_GNSS_DELL
    #define aSIM_BLOCK_STEP_GNSS_ON                     aEC200_BLOCK_GNSS_ON

    #define aSIM_BLOCK_STEP_FTP_INIT                    aEC200_BLOCK_FTP_INIT
    #define aSIM_BLOCK_STEP_FTP_READ                    aEC200_BLOCK_FTP_READ
     
    #define aSIM_BLOCK_STEP_HTTP_HEAD                   aEC200_BLOCK_HTTP_HEAD  
    #define aSIM_BLOCK_STEP_HTTP_POST                   aEC200_BLOCK_HTTP_POST  

    #define sSIM_URC                                    aEC200Urc
    #define SIM_URC_FIRST                               _EC200_URC_RESET_SIM900
    #define SIM_URC_END                                 _EC200_URC_END
    //End block step
#endif

/*======= Define Type SIM: SIM_L511 LIB =============*/  
#ifdef SIM_L511_LIB
    #include "user_l511.h"
    #define SIM_STEP_END                                _L511_END
    #define SIM_STEP_OUT_DATA_MODE                      _L511_TCP_OUT_DATAMODE
    #define SIM_STEP_ATTACH                             _L511_NET_CHECK_ATTACH
     
    #define SIM_GET_AT_CMD(sim_step)                    L511_Get_AT_Cmd(sim_step) 
     
    #define SIM_GET_TIMEOUT(sim_step)                   L511_Get_Timeout_Follow_Cmd(sim_step)    
    #define SIM_GET_DELAY_AT(sim_step)                  L511_Get_Delay_AT(sim_step) 
    #define SIM_IS_IGNORE_FAILURE(sim_step, pJum_AT)    L511_Is_Ignore_Failure(sim_step, pJum_AT)

    #define SIM_IS_STEP_CHECK_URC(sim_step)             L511_Is_Step_Check_URC(sim_step)  
    #define SIM_IS_STEP_SKIP_ERROR(sim_step)            L511_Check_Step_Skip_Error(sim_step) 
    #define SIM_IS_SEND_HEX(sim_step)                   L511_Is_Send_Hex(sim_step) 

    /*Block step*/
    #define aSIM_BLOCK_STEP_INIT                        aL511_BLOCK_INIT
    #define aSIM_BLOCK_STEP_NETWORK                     aL511_BLOCK_NETWORK
    #define aSIM_BLOCK_STEP_CONNECT                     aL511_BLOCK_CONNECT
    #define aSIM_BLOCK_STEP_DIS_CONN                    aL511_BLOCK_DIS_CONN

    #define aSIM_BLOCK_STEP_PUB_FB                      aL511_BLOCK_PUB_FB
    #define aSIM_BLOCK_STEP_PUB                         aL511_BLOCK_PUB
    #define aSIM_BLOCK_STEP_PING                        aL511_BLOCK_PING

    #define aSIM_BLOCK_STEP_CSQ                         aL511_BLOCK_CSQ
    #define aSIM_BLOCK_STEP_READ_SMS                    aL511_BLOCK_READ_SMS
    #define aSIM_BLOCK_STEP_SEND_SMS                    aL511_BLOCK_SEND_SMS
    #define aSIM_BLOCK_STEP_CALL                        aL511_BLOCK_CALLING
    #define aSIM_BLOCK_STEP_CLOCK                       aL511_BLOCK_CLOCK

    #define aSIM_BLOCK_STEP_HTTP_INIT                   aL511_BLOCK_HTTP_INIT
    #define aSIM_BLOCK_STEP_HTTP_READ                   aL511_BLOCK_HTTP_READ

    #define aSIM_BLOCK_STEP_FILE_CLOSE                  aL511_BLOCK_FILE_CLOSE

    #define aSIM_BLOCK_STEP_GNSS                        aL511_BLOCK_GNSS
    #define aSIM_BLOCK_STEP_GNSS_DELL                   aL511_BLOCK_GNSS_DELL
    #define aSIM_BLOCK_STEP_GNSS_ON                     aL511_BLOCK_GNSS_ON

    #define aSIM_BLOCK_STEP_FTP_INIT                    aL511_BLOCK_FTP_INIT
    #define aSIM_BLOCK_STEP_FTP_READ                    aL511_BLOCK_FTP_READ

    #define aSIM_BLOCK_STEP_HTTP_HEAD                   aL511_BLOCK_HTTP_HEAD  
    #define aSIM_BLOCK_STEP_HTTP_POST                   aL511_BLOCK_HTTP_POST  

    #define sSIM_URC                                    aL511Urc
    #define SIM_URC_FIRST                               _L511_URC_RESET_SIM900
    #define SIM_URC_END                                 _L511_URC_END
#endif



#ifdef SIM_BC660_LIB
    #include "user_bc660.h"
    #define SIM_STEP_END                                _BC660_END
    #define SIM_STEP_OUT_DATA_MODE                      _BC660_END
    #define SIM_STEP_ATTACH                             _BC660_NET_CHECK_ATTACH
    #define SIM_STEP_CALLING                            _BC660_END
     
    #define SIM_GET_AT_CMD(sim_step)                    BC660_Get_AT_Cmd(sim_step) 
     
    #define SIM_GET_TIMEOUT(sim_step)                   BC660_Get_Timeout_Follow_Cmd(sim_step)    
    #define SIM_GET_DELAY_AT(sim_step)                  BC660_Get_Delay_AT(sim_step) 
    #define SIM_IS_IGNORE_FAILURE(sim_step, pJum_AT)    BC660_Is_Ignore_Failure(sim_step, pJum_AT)

    #define SIM_IS_STEP_CHECK_URC(sim_step)             BC660_Is_Step_Check_URC(sim_step)
    #define SIM_IS_STEP_SKIP_ERROR(sim_step)            BC660_Check_Step_Skip_Error(sim_step)
    #define SIM_IS_SEND_HEX(sim_step)                   BC660_Is_Send_Hex(sim_step) 

    /*Block step*/
    #define aSIM_BLOCK_STEP_INIT                        aBC660_BLOCK_INIT
    #define aSIM_BLOCK_STEP_NETWORK                     aBC660_BLOCK_NETWORK
    #define aSIM_BLOCK_STEP_CONNECT                     aBC660_BLOCK_CONNECT
    #define aSIM_BLOCK_STEP_DIS_CONN                    aBC660_BLOCK_DIS_CONN

    #define aSIM_BLOCK_STEP_PUB_FB                      aBC660_BLOCK_PUB_FB
    #define aSIM_BLOCK_STEP_PUB                         aBC660_BLOCK_PUB
    #define aSIM_BLOCK_STEP_PING                        aBC660_BLOCK_PING

    #define aSIM_BLOCK_STEP_CSQ                         aBC660_BLOCK_CSQ
    #define aSIM_BLOCK_STEP_READ_SMS                    aBC660_BLOCK_READ_SMS
    #define aSIM_BLOCK_STEP_SEND_SMS                    aBC660_BLOCK_SEND_SMS
    #define aSIM_BLOCK_STEP_CALL                        aBC660_BLOCK_CALLING
    #define aSIM_BLOCK_STEP_CLOCK                       aBC660_BLOCK_CLOCK

    #define aSIM_BLOCK_STEP_HTTP_INIT                   aBC660_BLOCK_HTTP_INIT
    #define aSIM_BLOCK_STEP_HTTP_READ                   aBC660_BLOCK_HTTP_READ

    #define aSIM_BLOCK_STEP_FILE_CLOSE                  aBC660_BLOCK_FILE_CLOSE

    #define aSIM_BLOCK_STEP_FTP_INIT                    aBC660_BLOCK_FTP_INIT
    #define aSIM_BLOCK_STEP_FTP_READ                    aBC660_BLOCK_FTP_READ
     
    #define aSIM_BLOCK_STEP_HTTP_HEAD                   aBC660_BLOCK_HTTP_HEAD  
    #define aSIM_BLOCK_STEP_HTTP_POST                   aBC660_BLOCK_HTTP_POST  
     
    #define aSIM_BLOCK_STEP_PSM                         aBC660_BLOCK_LOW_POWER  

    #define sSIM_URC                                    aBC660Urc
    #define SIM_URC_FIRST                               _BC660_URC_RESET_SIM900
    #define SIM_URC_END                                 _BC660_URC_END
    //End block step
#endif




/*================Define================*/
#define SIM_MAX_ITEM_QUEUE  50

#define SIM_CMD_RETRY		2
#define SIM_CMD_FREQ		50     
#define SIM_MAX_CGATT  	    10
         
#define MAX_PING_TO_CONFIG  1
#define MAX_LENGTH_AT_SERVER    40

#define STEP_INVALID  	    0xFF
         
/*===============struct, var================*/
typedef enum
{
	_EVENT_SIM_AT_SEND = 0,
    _EVENT_SIM_UART_RECEIVE,
	_EVENT_SIM_AT_SEND_OK,
	_EVENT_SIM_AT_SEND_TIMEOUT,
    
	_EVENT_SIM_END, // don't use
}Event_sim_TypeDef;


typedef struct 
{
    uint8_t     Retry_u8;                //So lan retry 1 lenh AT
    uint8_t     ATProcessed_u8;          //So step AT thuc hien 1 lenh
    
    uint8_t     IsPause_u8;
    uint8_t     IsMatch_u8;              //1: Resp TRUE  ; 0: Resp Fail
    
    uint8_t     Pending_u8;
    uint32_t    LandMark_u32;
    
    void        (*pSim_Active_DTR_PIN) (void);
    uint8_t     (*pSim_Check_Ignore_Failure) (uint8_t Step, uint8_t (*pJum_Next_At)(uint8_t Type));
}StructSimVariable;


/*============ extern Struct var ==================*/
extern StructSimVariable    sSimVar;
extern sEvent_struct        sEventSim[];
extern Struct_Queue_Type    qSimStep;
extern sData 	sUartSim;
/*==============Function==================*/

void 		Sim_Init(void);
uint8_t 	Sim_Task(void);

void        Sim_Init_Uart (void);
void        Sim_Init_RX_Mode(void);
void        Sim_Stop_RX_Mode (void);
void        Sim_Rx_Callback (uint16_t Size);
void        Sim_Reinit_Uart (void);

uint8_t 	Sim_Push_Step_To_Queue(uint8_t sim_step);
uint8_t 	Sim_Push_Block_To_Queue(uint8_t *block_sim_step, uint8_t size);

uint8_t     Sim_Back_Current_AT (uint8_t NumAT);
uint8_t 	Sim_Check_Response(uint8_t sim_step);
uint8_t     Sim_Get_Step_From_Queue(uint8_t Type);

void        Sim_Disable_All_Event (void);
void        Sim_Clear (void);

uint8_t     Sim_Active_DTR_Pin (void);


#endif /* USER_SIM_H_ */
