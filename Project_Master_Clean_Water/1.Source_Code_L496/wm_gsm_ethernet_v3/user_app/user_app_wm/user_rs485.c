
/*
    - Thiet bi doc level chat long
        + Day ket noi:
            ++ Red:    Vcc 12-36V
            ++ Green:  Gnd
            ++ Yellow: RS485B
            ++ Blue: RS485A

    - Thiet bi doc 485 dong ho nuoc
            A: Tx+
            B: Tx-
*/      

#include "user_rs485.h"
#include "user_modbus_rtu.h"


void (*pRS485_Send_Data) (uint8_t *pData, uint16_t Length);

static uint8_t aUART_485_DATA [512];
sData   sUart485 = {(uint8_t *) &aUART_485_DATA[0], 0};

static uint16_t OldPosDMA485 = 0;
static uint8_t aUART_RS485_DMA_RX [100];
static uint8_t Uart485RxByte = 0;


static uint8_t aUART_485_2_DATA [512];
sData   sUart485_2 = {(uint8_t *) &aUART_485_2_DATA[0], 0};

static uint16_t OldPosDMA485_2 = 0;
static uint8_t aUART_RS485_2_DMA_RX [100];
static uint8_t Uart485_2RxByte = 0;

/*--- variable recv or pending*/
uint8_t Rs485Status_u8 = false;
uint8_t Rs485_2Status_u8 = false;

/*======================== Funcion ======================*/
void RS485_Init_Data (void)
{
    Reset_Buff(&sUart485);
    Rs485Status_u8  = PENDING;
}


void RS485_Init_Uart (void)
{
    MX_UART_RS485_Init();
}


void RS485_Init_RX_Mode(void)
{
#if (UART_RS485_MODE == UART_RS485_IT)
    //    __HAL_UART_ENABLE_IT(&uart_rs485, UART_IT_RXNE);
    HAL_UART_Receive_IT(&uart_rs485, &Uart485RxByte, 1);
#else
    HAL_UARTEx_ReceiveToIdle_DMA(&uart_rs485, aUART_RS485_DMA_RX, sizeof(aUART_RS485_DMA_RX));
    __HAL_DMA_DISABLE_IT(&uart_dma_rs485, DMA_IT_HT); 
    OldPosDMA485 = 0;
#endif
    
    HAL_NVIC_SetPriority(USART_RS485_IRQn, USART_RS485_Priority, 0);
    HAL_NVIC_EnableIRQ(USART_RS485_IRQn);
}

void RS485_Stop_RX_Mode (void)
{
#if (UART_RS485_MODE == UART_RS485_IT)
//    __HAL_UART_DISABLE_IT(&uart_rs485, UART_IT_RXNE);   
#else
    HAL_UART_DMAStop(&uart_rs485);
#endif
}


void RS485_Rx_Callback (uint16_t Size)
{
#if (UART_RS485_MODE == UART_RS485_IT)
    *(sUart485.Data_a8 + sUart485.Length_u16++) = Uart485RxByte;

    if (sUart485.Length_u16 >= (sizeof(aUART_485_DATA) - 1))
        sUart485.Length_u16 = 0;

    HAL_UART_Receive_IT(&uart_rs485, &Uart485RxByte, 1);

    Rs485Status_u8 = TRUE;
#else
    uint16_t NbByteRec = 0, i = 0;
    uint16_t OldPosData = OldPosDMA485;
       
    if (uart_dma_rs485.Init.Mode == DMA_NORMAL)   //sua o day
    {
        if ( (sUart485.Length_u16 + Size) >= sizeof(aUART_485_DATA) )
                sUart485.Length_u16 = 0;
        
        for (i = 0; i < Size; i++)
            *(sUart485.Data_a8 + sUart485.Length_u16 + i) = aUART_RS485_DMA_RX[i]; 

        sUart485.Length_u16 += Size ;
    } else
    {
        //Chia 2 truong hop
        if (Size > OldPosData)
        {
            NbByteRec = Size - OldPosData;
            
            //Kiem tra length cua buff nhan
            if ( (sUart485.Length_u16 + NbByteRec) >= sizeof(aUART_485_DATA) )
                sUart485.Length_u16 = 0;
        
            for (i = 0; i < NbByteRec; i++)
                *(sUart485.Data_a8 + sUart485.Length_u16 + i) = aUART_RS485_DMA_RX[OldPosData + i]; 
            
            sUart485.Length_u16 += NbByteRec;
        } else
        {
            NbByteRec = sizeof(aUART_RS485_DMA_RX) - OldPosData;

            //Kiem tra length cua buff nhan
            if ( (sUart485.Length_u16 + Size + NbByteRec) >= sizeof(aUART_485_DATA) )
                sUart485.Length_u16 = 0;
            
            for (i = 0; i < NbByteRec; i++)
                *(sUart485.Data_a8 + sUart485.Length_u16 + i) = aUART_RS485_DMA_RX[OldPosData + i]; 
            
            for (i = 0; i < Size; i++)
                *(sUart485.Data_a8 + sUart485.Length_u16 + i) = aUART_RS485_DMA_RX[i]; 
            
            sUart485.Length_u16 += (Size + NbByteRec) ;
        }
        
        //Cap nhat lai Old Position
        OldPosDMA485 = Size;
    }
          
    if (uart_dma_rs485.Init.Mode == DMA_NORMAL)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&uart_rs485, aUART_RS485_DMA_RX, sizeof(aUART_RS485_DMA_RX));
        __HAL_DMA_DISABLE_IT(&uart_dma_rs485, DMA_IT_HT);
    }
        
    Rs485Status_u8 = TRUE;
#endif
}


/*
    FuncTest: Master Read
*/

void RS485_Modbus_Read_Value (uint8_t SlaveID, uint16_t Addr, uint16_t nRegis)
{
    uint8_t aFrame[48] = {0};
    sData   strFrame = {(uint8_t *) &aFrame[0], 0};
    
    ModRTU_Master_Read_Frame(&strFrame, SlaveID, FUN_READ_BYTE, Addr, nRegis);

    HAL_GPIO_WritePin(DE_GPIO_PORT, DE_GPIO_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
    // Send
    RS485_Init_Data();
    HAL_UART_Transmit(&uart_rs485, strFrame.Data_a8, strFrame.Length_u16, 1000); 
    
    //Dua DE ve Receive
    HAL_GPIO_WritePin(DE_GPIO_PORT, DE_GPIO_PIN, GPIO_PIN_RESET);
}



/*
    Kiem tra format modbus
        return: 
*/

uint8_t RS485_Modbus_Check_Format (uint8_t SlaveID, uint16_t nRegis,
                                   sData *pSource, sData *Content)
{
    uint16_t CrcCalcu = 0;
    uint8_t aCRC_GET[2] = {0};
    uint16_t Pos = 0;
    uint8_t ModFunc = 0, ModLength = 0, ModSlave = 0;
    
    if (pSource->Length_u16 < 4)
        return false;
    
    CrcCalcu = ModRTU_CRC(pSource->Data_a8, pSource->Length_u16 - 2);
            
    aCRC_GET[0] = (uint8_t) (CrcCalcu & 0x00FF);
    aCRC_GET[1] = (uint8_t) ( (CrcCalcu >> 8) & 0x00FF );
    
    if ( (aCRC_GET[0] != *(pSource->Data_a8 + pSource->Length_u16 - 2)) 
        || (aCRC_GET[1] != *(pSource->Data_a8 + pSource->Length_u16 - 1)) )
        return false;
            
    ModSlave = *(pSource->Data_a8 + Pos++);
    ModFunc = *(pSource->Data_a8 + Pos++);
            
    switch (ModFunc)
    {
        case FUN_READ_BYTE:
            ModLength = *(pSource->Data_a8 + Pos++);
            //check frame
            if ( (ModSlave != SlaveID) || (ModLength != (nRegis * 2)))
                return false;
            
            //tro content vao data
            Content->Data_a8 = pSource->Data_a8 + Pos;
            Content->Length_u16 = ModLength;   
            break;
        default:
            break;
    }
    
    return true;
}



/*--------- RS485 2 --------------*/
void RS485_2_Init_Data (void)
{
    Reset_Buff(&sUart485_2);
    Rs485_2Status_u8  = PENDING;
}

void RS485_2_Init_Uart (void)
{
    MX_UART_RS485_2_Init();
}


void RS485_2_Init_RX_Mode(void)
{
#if (UART_RS485_2_MODE == UART_RS485_2_IT)
    HAL_UART_Receive_IT(&uart_rs485_2, &Uart485_2RxByte, 1);
#else
    HAL_UARTEx_ReceiveToIdle_DMA(&uart_rs485_2, aUART_RS485_2_DMA_RX, sizeof(aUART_RS485_2_DMA_RX));
    __HAL_DMA_DISABLE_IT(&uart_dma_rs485_2, DMA_IT_HT); 
    OldPosDMA485 = 0;
#endif
    
    HAL_NVIC_SetPriority(USART_RS485_2_IRQn, USART_RS485_2_Priority, 0);
    HAL_NVIC_EnableIRQ(USART_RS485_2_IRQn);
}

void RS485_2_Stop_RX_Mode (void)
{
#if (UART_RS485_2_MODE == UART_RS485_2_IT)
#else
    HAL_UART_DMAStop(&uart_rs485_2);
#endif
}

void RS485_2_Modbus_Read_Value (uint8_t SlaveID, uint16_t Addr, uint16_t nRegis)
{
    uint8_t aFrame[48] = {0};
    sData   strFrame = {(uint8_t *) &aFrame[0], 0};
    
    ModRTU_Master_Read_Frame(&strFrame, SlaveID, FUN_READ_BYTE, Addr, nRegis);

    HAL_GPIO_WritePin(DE_2_GPIO_PORT, DE_2_GPIO_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
    // Send
    RS485_2_Init_Data();
    HAL_UART_Transmit(&uart_rs485_2, strFrame.Data_a8, strFrame.Length_u16, 1000); 
    
    //Dua DE ve Receive
    HAL_GPIO_WritePin(DE_2_GPIO_PORT, DE_2_GPIO_PIN, GPIO_PIN_RESET);
}




void RS485_2_Rx_Callback (uint16_t Size)
{
#if (UART_RS485_2_MODE == UART_RS485_2_IT)
    *(sUart485_2.Data_a8 + sUart485_2.Length_u16++) = Uart485_2RxByte;

    if (sUart485_2.Length_u16 >= (sizeof(aUART_485_2_DATA) - 1))
        sUart485_2.Length_u16 = 0;

    HAL_UART_Receive_IT(&uart_rs485_2, &Uart485_2RxByte, 1);

    Rs485_2Status_u8 = TRUE;
#else
    uint16_t NbByteRec = 0, i = 0;
    uint16_t OldPosData = OldPosDMA485_2;
       
    if (uart_dma_rs485_2.Init.Mode == DMA_NORMAL)   //sua o day
    {
        if ( (sUart485_2.Length_u16 + Size) >= sizeof(aUART_485_2_DATA) )
                sUart485_2.Length_u16 = 0;
        
        for (i = 0; i < Size; i++)
            *(sUart485_2.Data_a8 + sUart485_2.Length_u16 + i) = aUART_RS485_2_DMA_RX[i]; 

        sUart485_2.Length_u16 += Size ;
    } else
    {
        //Chia 2 truong hop
        if (Size > OldPosData)
        {
            NbByteRec = Size - OldPosData;
            
            //Kiem tra length cua buff nhan
            if ( (sUart485_2.Length_u16 + NbByteRec) >= sizeof(aUART_485_2_DATA) )
                sUart485_2.Length_u16 = 0;
        
            for (i = 0; i < NbByteRec; i++)
                *(sUart485_2.Data_a8 + sUart485_2.Length_u16 + i) = aUART_RS485_2_DMA_RX[OldPosData + i]; 
            
            sUart485_2.Length_u16 += NbByteRec;
        } else
        {
            NbByteRec = sizeof(aUART_RS485_2_DMA_RX) - OldPosData;

            //Kiem tra length cua buff nhan
            if ( (sUart485_2.Length_u16 + Size + NbByteRec) >= sizeof(aUART_485_2_DATA) )
                sUart485_2.Length_u16 = 0;
            
            for (i = 0; i < NbByteRec; i++)
                *(sUart485_2.Data_a8 + sUart485_2.Length_u16 + i) = aUART_RS485_2_DMA_RX[OldPosData + i]; 
            
            for (i = 0; i < Size; i++)
                *(sUart485_2.Data_a8 + sUart485_2.Length_u16 + i) = aUART_RS485_2_DMA_RX[i]; 
            
            sUart485_2.Length_u16 += (Size + NbByteRec) ;
        }
        
        //Cap nhat lai Old Position
        OldPosDMA485_2 = Size;
    }
          
    if (uart_dma_rs485_2.Init.Mode == DMA_NORMAL)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&uart_rs485_2, aUART_RS485_2_DMA_RX, sizeof(aUART_RS485_2_DMA_RX));
        __HAL_DMA_DISABLE_IT(&uart_dma_rs485_2, DMA_IT_HT);
    }
        
    Rs485_2Status_u8 = TRUE;
#endif
}

