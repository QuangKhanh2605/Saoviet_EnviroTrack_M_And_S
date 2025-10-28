#include "user_connect_sensor.h"
#include "user_define.h"

static uint8_t aUART_485SS_DATA [512];
sData   sUart485SS= {(uint8_t *) &aUART_485SS_DATA[0], 0};

static uint16_t OldPosDMA485SS = 0;
static uint8_t aUART_RS485SS_DMA_RX [100];
static uint8_t Uart485SSRxByte = 0;

/*--- variable recv or pending*/
uint8_t Rs485SSStatus_u8 = false;

/*=========================== Function ===========================*/
void RS485SS_Init_Data (void)
{
    Reset_Buff(&sUart485SS);
    Rs485SSStatus_u8  = PENDING;
}

void RS485SS_Init_Uart (void)
{
    MX_UART_RS485SS_Init();
}

void RS485SS_Init_RX_Mode(void)
{
#if (UART_RS485SS_MODE == UART_RS485SS_IT)
    //    __HAL_UART_ENABLE_IT(&uart_rs485, UART_IT_RXNE);
    HAL_UART_Receive_IT(&uart_rs485SS, &Uart485SSRxByte, 1);
#else
    HAL_UARTEx_ReceiveToIdle_DMA(&uart_rs485SS, aUART_RS485SS_DMA_RX, sizeof(aUART_RS485SS_DMA_RX));
    __HAL_DMA_DISABLE_IT(&uart_dma_rs485SS, DMA_IT_HT); 
    OldPosDMA485SS = 0;
#endif
    
    HAL_NVIC_SetPriority(USART_RS485SS_IRQn, USART_RS485SS_Priority, 0);
    HAL_NVIC_EnableIRQ(USART_RS485SS_IRQn);
}

void RS485SS_Stop_RX_Mode (void)
{
#if (UART_RS485SS_MODE == UART_RS485SS_IT)
//    __HAL_UART_DISABLE_IT(&uart_rs485SS, UART_IT_RXNE);   
#else
    HAL_UART_DMAStop(&uart_rs485SS);
#endif
}

void RS485SS_Rx_Callback (uint16_t Size)
{
#if (UART_RS485SS_MODE == UART_RS485SS_IT)
    *(sUart485SS.Data_a8 + sUart485SS.Length_u16++) = Uart485SSRxByte;

    if (sUart485SS.Length_u16 >= (sizeof(aUART_485SS_DATA) - 1))
        sUart485SS.Length_u16 = 0;

    HAL_UART_Receive_IT(&uart_rs485SS, &Uart485SSRxByte, 1);

    Rs485SSStatus_u8 = TRUE;
#else
    uint16_t NbByteRec = 0, i = 0;
    uint16_t OldPosData = OldPosDMA485SS;
       
    if (uart_dma_rs485SS.Init.Mode == DMA_NORMAL)   //sua o day
    {
        if ( (sUart485SS.Length_u16 + Size) >= sizeof(aUART_485SS_DATA) )
                sUart485SS.Length_u16 = 0;
        
        for (i = 0; i < Size; i++)
            *(sUart485SS.Data_a8 + sUart485SS.Length_u16 + i) = aUART_RS485SS_DMA_RX[i]; 

        sUart485SS.Length_u16 += Size ;
    } else
    {
        //Chia 2 truong hop
        if (Size > OldPosData)
        {
            NbByteRec = Size - OldPosData;
            
            //Kiem tra length cua buff nhan
            if ( (sUart485SS.Length_u16 + NbByteRec) >= sizeof(aUART_485SS_DATA) )
                sUart485SS.Length_u16 = 0;
        
            for (i = 0; i < NbByteRec; i++)
                *(sUart485SS.Data_a8 + sUart485SS.Length_u16 + i) = aUART_RS485SS_DMA_RX[OldPosData + i]; 
            
            sUart485SS.Length_u16 += NbByteRec;
        } else
        {
            NbByteRec = sizeof(aUART_RS485SS_DMA_RX) - OldPosData;

            //Kiem tra length cua buff nhan
            if ( (sUart485SS.Length_u16 + Size + NbByteRec) >= sizeof(aUART_485SS_DATA) )
                sUart485SS.Length_u16 = 0;
            
            for (i = 0; i < NbByteRec; i++)
                *(sUart485SS.Data_a8 + sUart485SS.Length_u16 + i) = aUART_RS485SS_DMA_RX[OldPosData + i]; 
            
            for (i = 0; i < Size; i++)
                *(sUart485SS.Data_a8 + sUart485SS.Length_u16 + i) = aUART_RS485SS_DMA_RX[i]; 
            
            sUart485SS.Length_u16 += (Size + NbByteRec) ;
        }
        
        //Cap nhat lai Old Position
        OldPosDMA485SS = Size;
    }
          
    if (uart_dma_rs485SS.Init.Mode == DMA_NORMAL)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&uart_rs485SS, aUART_RS485SS_DMA_RX, sizeof(aUART_RS485SS_DMA_RX));
        __HAL_DMA_DISABLE_IT(&uart_dma_rs485SS, DMA_IT_HT);
    }
        
    Rs485SSStatus_u8 = TRUE;
#endif
}


