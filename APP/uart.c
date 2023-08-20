#include "uart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "SEQueue.h"
#include "motor_control.h"
#include <stdlib.h>

#define SCOM_HEADER_LENGTH                  6
#define SCOM1_HEADER_LENGTH                 6 //

// #define SCOM0_MIN_LENGTH    8
#define SCOM1_MIN_LENGTH    8
// #define SCOM2_MIN_LENGTH    8
// #define SCOM3_MIN_LENGTH    8
// #define SCOM4_MIN_LENGTH    8
// #define SCOM5_MIN_LENGTH    8
// #define SCOM6_MIN_LENGTH    8
// #define SCOM7_MIN_LENGTH    8

// #define USART1_REC_LEN      2000
#define USART1_REC_LEN      200
// #define USART2_REC_LEN      2000
// #define USART3_REC_LEN      100
// #define USART4_REC_LEN      200
// #define USART5_REC_LEN      300
// #define USART6_REC_LEN      500
// #define USART7_REC_LEN      100


#define COMMA_LEN_MAX                       25 // 协议中，逗号的最大个数

#define CHARTOHEX(ch)  ((ch) >= 'A' ? ((ch) - 'A')+10 : (ch) - '0')
#define HEXTOCHAR(hx)  (uint8_t)((hx) >= 0x0A ? (hx) + 'A' - 0x0A : (hx) + '0')

#define FSYN_OFF 0x00
#define FSYN_ON  0xFF


#define PrintfLog(...)

enum UART_FRAME_ID_TYPE/*UART帧ID 枚举*/
{
    NULL_ID = 0x00,
    //UART0_Rx ,
    ANGST_ID,
    ANGRD_ID,
    PIDST_ID,
    PIDRD_ID,
    ANCAL_ID,  /* 这只角度偏差指令 */
};

/* 数据帧类型 */
typedef enum
{
    FRAME_NULL = 0x00,
    USER_DEF,             //
    BD40_TYPE,            //
    BD21_TYPE,            //
    IMU_TYPE,
    IC_PRM_TYPE,
} FRAME_TPYE_DEF;

/* 数据帧结构体 */
typedef struct
{
    char u8Header[SCOM_HEADER_LENGTH]; //帧头缓存
    uint8_t u8commSYN;                 //串口同步标志
    uint8_t u8cmdtype;                 //命令字类型
    uint16_t iCommCount;               //接收计数
    uint16_t iRecvLength;              //数据总长度
    uint16_t lastCmdHeadPos;           //最后一个帧头的位置
    uint8_t lastCmdHeadType;           //最后一个帧头的类型
    FRAME_TPYE_DEF frameType;          //数据帧类型
} FRAME_DATA;

static uint8_t g_u8COM1_RX_BUF[USART1_REC_LEN] = {0};

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
            - BaudRate = 115200 baud
            - Word Length = 8 Bits
            - One Stop Bit
            - No parity
            - Hardware flow control disabled (RTS and CTS signals)
            - Receive and transmit enabled
    */
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    /* 使能时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    /*********************初始化串口IO配置**********************************/
    /* Configure USART1 Tx (PA.09) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /* Configure USART1 Rx (PA.10) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //USART_ClockInitTypeDef  USART_InitClock;                        //定义串口初始化时钟结构体
    USART_InitStructure.USART_BaudRate = 115200; // 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    /* Configure USART1 */
    USART_Init(USART1, &USART_InitStructure);

    /* Enable USART1 Receive and Transmit interrupts */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /* Enable the USART1 */
    USART_Cmd(USART1, ENABLE);
}

/***************************************
功能:串口1输出一个字节
输入:c:串口输出的字符
输出:NA
***************************************/
void USART1_Putc(const uint8_t c)
{
    USART_SendData(USART1, c);
    /* Loop until the end of transmission */
    while(RESET == USART_GetFlagStatus(USART1, USART_FLAG_TXE));
}

#include <stdio.h>
int fputc(int ch, FILE* f)
{
    USART1_Putc(ch);
    return ch;
}

void USART1_SendBuf(const uint8_t* u8buf, const uint16_t uint16_tlen)
{
    unsigned short i = 0;

    if(NULL == u8buf)
    {
        return;
    }

    for(i = 0; i < uint16_tlen; i++)
    {
        USART1_Putc(u8buf[i]);
    }
}

/***************************************
功能:串口1发送字符串
输入:buf:输入字符串
输出:NA
***************************************/
void USART1_SendString(const char* buf)
{
    uint16_t i = 0;
    uint16_t u16len = 0;

    if (NULL == buf)
    {
        return;
    }

    while (NULL != *(buf+i)) //获取输入log字符串长度
    {
        i++;
        u16len++;
        if(100 <= u16len) //输出太长
        {
            return;
        }
    }
    for (i = 0;i < u16len; i++)
    {
        USART1_Putc(buf[i]);
    }
}

/*************************************************
功能：获取逗号的位置和数量
输入：Rxbuf: 数据帧
      len:帧长度
输出：commabuf:逗号位置buf
      返回值:逗号的总个数
*************************************************/
static uint8_t GetCommaPosAndNumber(const uint8_t* Rxbuf, const uint16_t len, uint8_t* commabuf)
{
    uint8_t commaSum = 0;
    uint8_t i = 0;

    if( (NULL == Rxbuf) || (NULL == commabuf))
    {
        return 0;
    }

    for(i = 0; i < len; i++)
    {
        if(',' == Rxbuf[i])
        {
            if(commaSum >= COMMA_LEN_MAX)
            {
                return 0;
            }
            commabuf[commaSum++] = i;
        }
    }

    return commaSum;
}

/*************************************************
功能：处理接收到的指令     GDSZ 惯导设置
输入：Rxbuf: 命令
      len:帧长度
      $GDSZ,aa,bb,,*hh
      aa:数据频率, bb:初始对准时间
输出：无
*************************************************/
static void ANGSTCmd_Progress(const uint8_t* Rxbuf, uint16_t len)
{
    uint8_t commabuf[COMMA_LEN_MAX] = {0};
    float tempdata = 0;
    char sbuf[20] = {0};
    uint8_t commaNumber = 0;  //
    uint16_t i = 0, buflen = 0;

    commaNumber = GetCommaPosAndNumber(Rxbuf,len,commabuf);
    if(commaNumber > 2)
    {
        return;
    }

    for(i = 0; i < commaNumber; i++)        // 逗号都遍历一遍,
    {
        if(i != (commaNumber - 1))              // 不是最后一个逗号
        {
            if(commabuf[i]+1 == commabuf[i+1])  // 两个逗号挨着，说明数据为空，赋值为0
            {
                tempdata = 0.0;
                buflen = 0;
            }
            else
            {
                memset(sbuf, 0, 20);
                buflen = commabuf[i+1] - commabuf[i] - 1;
                memcpy(sbuf, &Rxbuf[commabuf[i]+1], buflen);
                tempdata = atof(sbuf);
            }
        }
        else //最后一个逗号
        {
            if(commabuf[i]+1 == (len-5)) // 逗号之后 为'*'
            {
                tempdata = 0.0;
                buflen = 0;
            }
            else
            {
                memset(sbuf, 0, 20);
                buflen = len-5 - commabuf[i] - 1;
                memcpy(sbuf, &Rxbuf[commabuf[i]+1], buflen);
                tempdata = atof(sbuf);
            }

        }

        switch(i)
        {
            case 0: // 设置目标角度 单位:度/100 tempdata 范围
                MotorSetControlAngle(tempdata);
                break;
            case 1: // 初始对准时间，限定了几个指定的

                break;
            default:
                break;
        }
    }

    printf("Recieve ANGST cmd!\r\n");
}

/**
 * 读取角度
*/
static void ANGRDCmd_Progress(const uint8_t* Rxbuf, uint16_t len)
{
    MotorPrintAngle();
}

static void PIDSTCmd_Progress(const uint8_t* Rxbuf, uint16_t len)
{
    uint8_t commabuf[COMMA_LEN_MAX] = {0};
    float tempdata = 0;
    char sbuf[20] = {0};
    uint8_t commaNumber = 0;  //
    uint16_t i = 0, buflen = 0;
    double coeff[3] = {0};

    commaNumber = GetCommaPosAndNumber(Rxbuf,len,commabuf);
    if(commaNumber > 2)
    {
        return;
    }

    for(i = 0; i < commaNumber; i++)        // 逗号都遍历一遍,
    {
        if(i != (commaNumber - 1))              // 不是最后一个逗号
        {
            if(commabuf[i]+1 == commabuf[i+1])  // 两个逗号挨着，说明数据为空，赋值为0
            {
                tempdata = 0.0;
                buflen = 0;
            }
            else
            {
                memset(sbuf, 0, 20);
                buflen = commabuf[i+1] - commabuf[i] - 1;
                memcpy(sbuf, &Rxbuf[commabuf[i]+1], buflen);
                tempdata = atof(sbuf);
            }
        }
        else //最后一个逗号
        {
            if(commabuf[i]+1 == (len-5)) // 逗号之后 为'*'
            {
                tempdata = 0.0;
                buflen = 0;
            }
            else
            {
                memset(sbuf, 0, 20);
                buflen = len-5 - commabuf[i] - 1;
                memcpy(sbuf, &Rxbuf[commabuf[i]+1], buflen);
                tempdata = atof(sbuf);
            }

        }

        switch(i)
        {
            case 0: // 设置目标角度 单位:度/100 tempdata 范围
                coeff[0] = (double)tempdata;
                break;
            case 1: // 初始对准时间，限定了几个指定的
                coeff[1] = (double)tempdata;
                break;
            case 2: // 初始对准时间，限定了几个指定的
                coeff[2] = (double)tempdata;
                break;
            default:
                break;
        }
    }

    MotorControlSetPidCoeff((const double*) coeff);
}

static void PIDRDCmd_Progress(const uint8_t* Rxbuf, uint16_t len)
{
    MotorPrintPidCoeff();
}

/**
 * @brief 解析ANCAL指令并保存到flash中, ANCAL指令格式 $ANCAL,xx*AA
 *
*/
static void ANCALCmd_Progress(const uint8_t* Rxbuf, uint16_t len)
{
    uint8_t commabuf[COMMA_LEN_MAX] = {0};
    float tempdata = 0;
    char sbuf[20] = {0};
    uint8_t commaNumber = 0;  //
    uint16_t i = 0, buflen = 0;
    int16_t offset;

    commaNumber = GetCommaPosAndNumber(Rxbuf,len,commabuf);
    if(commaNumber > 2)
    {
        return;
    }

    for(i = 0; i < commaNumber; i++)        // 逗号都遍历一遍,
    {
        if(i != (commaNumber - 1))              // 不是最后一个逗号
        {
            if(commabuf[i]+1 == commabuf[i+1])  // 两个逗号挨着，说明数据为空，赋值为0
            {
                tempdata = 0.0;
                buflen = 0;
            }
            else
            {
                memset(sbuf, 0, 20);
                buflen = commabuf[i+1] - commabuf[i] - 1;
                memcpy(sbuf, &Rxbuf[commabuf[i]+1], buflen);
                tempdata = atof(sbuf);
            }
        }
        else //最后一个逗号
        {
            if(commabuf[i]+1 == (len-5)) // 逗号之后 为'*'
            {
                tempdata = 0.0;
                buflen = 0;
            }
            else
            {
                memset(sbuf, 0, 20);
                buflen = len-5 - commabuf[i] - 1;
                memcpy(sbuf, &Rxbuf[commabuf[i]+1], buflen);
                tempdata = atof(sbuf);
            }

        }

        switch(i)
        {
            case 0: // 设置目标角度 单位:度/100 tempdata 范围
                offset = (int16_t)tempdata;
                MotorControlSetAngleOffset(offset, SET_UPDAE_FLASH);
                break;
            default:
                break;
        }
    }
}

/*************************************************
功能：串口接收到完整帧后进行数据处理
输入：Rxbuf: 数据帧
      len:帧长度
      cmdtype:帧类型
输出：无
*************************************************/
static void SCOM1_Process_cmd(uint8_t* Rxbuf, uint16_t len,uint8_t cmdtype)
{
    if(NULL == Rxbuf)
    {
        //PrintfLog(UART_MODULE,ERROR_LEVEL,"Uart1 Data is NULL!",0);
        return;
    }

    switch(cmdtype)
    {
        case ANGST_ID:
            ANGSTCmd_Progress(Rxbuf, len);
            break;
        case ANGRD_ID:
            ANGRDCmd_Progress(Rxbuf, len);
            break;
        case PIDST_ID:
            PIDSTCmd_Progress(Rxbuf, len);
            break;
        case PIDRD_ID:
            PIDRDCmd_Progress(Rxbuf, len);
            break;
        case ANCAL_ID:
            ANCALCmd_Progress(Rxbuf, len);
            break;
        default:
            USART1_SendBuf((unsigned char*)Rxbuf, len);
            break;
    }

    memset(g_u8COM1_RX_BUF, 0, USART1_REC_LEN);
}

void USART1_Receive(uint8_t chbyte)
{
    static FRAME_DATA s_framdata = {0};
    uint16_t i=0;
    uint8_t u8checksum = 0x00;                //接收校验和

    for(i = 0; i<SCOM1_HEADER_LENGTH; i++) //获取串口的前5个字节
    {
        if((SCOM1_HEADER_LENGTH - 1) == i)
        {
            s_framdata.u8Header[i] = chbyte;
        }
        else
        {
           s_framdata.u8Header[i] = s_framdata.u8Header[i+1];
        }
    }

    if (s_framdata.u8commSYN == FSYN_ON)
    {
        g_u8COM1_RX_BUF[s_framdata.iCommCount++] = chbyte;

        if(BD21_TYPE == s_framdata.frameType) //数据帧格式为用户自定义的类型
        {
            if( (0x0D == g_u8COM1_RX_BUF[s_framdata.iCommCount-2])&&(0x0A == g_u8COM1_RX_BUF[s_framdata.iCommCount-1]) )//
            {
                s_framdata.iRecvLength = s_framdata.iCommCount;
                for(i = 1; i < s_framdata.iCommCount-5; i++)        //
                {
                    u8checksum = (g_u8COM1_RX_BUF[i] ^ u8checksum) & 0xFF;
                }
                //校验和判断
                if( ( HEXTOCHAR((u8checksum >> 4)&0x0F) == g_u8COM1_RX_BUF[s_framdata.iCommCount-4])  \
                    &&( HEXTOCHAR(u8checksum & 0x0F) == g_u8COM1_RX_BUF[s_framdata.iCommCount-3]))
                {
                    //校验和通过
                    SCOM1_Process_cmd(g_u8COM1_RX_BUF, s_framdata.iCommCount, s_framdata.u8cmdtype);
                    memset(&s_framdata, 0, sizeof(FRAME_DATA)); //各状态清零
                    return;
                }
                else
                {
                    memset(&s_framdata, 0, sizeof(FRAME_DATA)); //各状态清零
                    memset(g_u8COM1_RX_BUF, 0, USART1_REC_LEN);
                    return;
                }
            }
        }
        else
        {
            memset(&s_framdata,0,sizeof(FRAME_DATA)); //各状态清零
            memset(g_u8COM1_RX_BUF,0,USART1_REC_LEN);
            return;
        }
    }
    //
    else if(0 == strncmp(s_framdata.u8Header,"$ANGST", SCOM1_HEADER_LENGTH))
    {
        s_framdata.u8commSYN = FSYN_ON;
        s_framdata.u8cmdtype = ANGST_ID;
        s_framdata.iCommCount = SCOM1_HEADER_LENGTH;
        s_framdata.frameType = BD21_TYPE;
        memcpy(g_u8COM1_RX_BUF,s_framdata.u8Header,SCOM1_HEADER_LENGTH);
    }
    //ANGRD
    else if(0 == strncmp(s_framdata.u8Header,"$ANGRD", SCOM1_HEADER_LENGTH))
    {
        s_framdata.u8commSYN = FSYN_ON;
        s_framdata.u8cmdtype = ANGRD_ID;
        s_framdata.iCommCount = SCOM1_HEADER_LENGTH;
        s_framdata.frameType = BD21_TYPE;
        memcpy(g_u8COM1_RX_BUF, s_framdata.u8Header, SCOM1_HEADER_LENGTH);
    }
    else if(0 == strncmp(s_framdata.u8Header,"$PIDST", SCOM1_HEADER_LENGTH))
    {
        s_framdata.u8commSYN = FSYN_ON;
        s_framdata.u8cmdtype = PIDST_ID;
        s_framdata.iCommCount = SCOM1_HEADER_LENGTH;
        s_framdata.frameType = BD21_TYPE;
        memcpy(g_u8COM1_RX_BUF, s_framdata.u8Header, SCOM1_HEADER_LENGTH);
    }
    else if(0 == strncmp(s_framdata.u8Header,"$PIDRD", SCOM1_HEADER_LENGTH))
    {
        s_framdata.u8commSYN = FSYN_ON;
        s_framdata.u8cmdtype = PIDRD_ID;
        s_framdata.iCommCount = SCOM1_HEADER_LENGTH;
        s_framdata.frameType = BD21_TYPE;
        memcpy(g_u8COM1_RX_BUF, s_framdata.u8Header, SCOM1_HEADER_LENGTH);
    }
    else if(0 == strncmp(s_framdata.u8Header,"$ANCAL", SCOM1_HEADER_LENGTH))
    {
        s_framdata.u8commSYN = FSYN_ON;
        s_framdata.u8cmdtype = ANCAL_ID;
        s_framdata.iCommCount = SCOM1_HEADER_LENGTH;
        s_framdata.frameType = BD21_TYPE;
        memcpy(g_u8COM1_RX_BUF, s_framdata.u8Header, SCOM1_HEADER_LENGTH);
    }
}


