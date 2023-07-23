#include "uart.h"
#include "SEQueue.h"

/**********************************************************************
* 名    称：USART1_IRQHandler()
* 功    能：USART1中断
* 入口参数：
* 出口参数：
* 全局变量：
-----------------------------------------------------------------------
* 说明：
***********************************************************************/
void USART1_IRQHandler(void)
{
    unsigned char  u8tmp = 0;
    if(RESET != USART_GetITStatus(USART1, USART_IT_RXNE))
    {
        u8tmp = (unsigned char)USART_ReceiveData(USART1);
        EnSEQueue(&g_Queue[0],u8tmp); //
    }
    USART_ClearFlag(USART1,USART_IT_RXNE);
}

/**********************************************************************
* 名    称：USART_Configuration()
* 功    能：串口初始化函数
* 入口参数：
* 出口参数：
***********************************************************************/
void USART_Configuration(void)
{
/* USART1 and USART1 configuration ------------------------------------------------------*/
    /* USART and USART1 configured as follow:
            - BaudRate = 9600 baud
            - Word Length = 8 Bits
            - One Stop Bit
            - No parity
            - Hardware flow control disabled (RTS and CTS signals)
            - Receive and transmit enabled
    */

    USART_InitTypeDef USART_InitStructure;
    //USART_ClockInitTypeDef  USART_InitClock;                        //定义串口初始化时钟结构体

    USART_InitStructure.USART_BaudRate = 9600;//9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    //USART_InitClock.USART_Clock = USART_Clock_Disable;                 //串口时钟禁止
    //USART_InitClock.USART_CPOL = USART_CPOL_Low;                         //时钟下降沿有效
    //USART_InitClock.USART_CPHA = USART_CPHA_2Edge;                    //数据在第二个时钟沿捕捉
    //USART_InitClock.USART_LastBit = USART_LastBit_Disable;            //最后数据位的时钟脉冲不输出到SCLK引脚

    //USART_ClockInit(USART1,&USART_InitClock);//初始化USART1外围时钟，按照 USART_ClockInitStruct 内的参数.

    /* Configure USART1 */
    USART_Init(USART1, &USART_InitStructure);

    /* Enable USART1 Receive and Transmit interrupts */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /* Enable the USART1 */
    USART_Cmd(USART1, ENABLE);
}

void USART1_Receive(uint8_t chbyte)
{
//    uint16_t i=0;
//    uint8_t u8checksum = 0x00;      //
}

//void Delay(int nCount);
void USART1_SendBuf(const uint8_t* u8buf, const uint16_t u16len)
{
    unsigned short i = 0;

    if(NULL == u8buf)
    {
        return;
    }

    for(i = 0; i < u16len; i++)
    {
        USART_SendData(USART1, (uint8_t)u8buf[i]);
        while(RESET == USART_GetFlagStatus(USART1, USART_FLAG_TXE));
    }
}
