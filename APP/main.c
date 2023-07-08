/**
  ******************************************************************************
  * @file    Project/Template/main.c
  * @author  MCD Application Team
  * @version V3.0.0
  * @date    04/06/2009
  * @brief   Main program body
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "core_cm3.h"
#include "stdio.h"
#include "SEGGER_RTT.h"

#define             ENCODE_RESOLUTION               2000
#define             ENCODE_TIM_DIV                  4
#define             ENCODE_CNT_PERROUND             (ENCODE_RESOLUTION*ENCODE_TIM_DIV)

#define             ENCODE_COUNT2ANGLE              (36000.0/ENCODE_CNT_PERROUND)               //角度乘100 减少误差
#define             TIM_ENCODE_COUNT_INIT           (32768)

unsigned char *P_RXD;//接收数据指针
unsigned int Num_RXD=0;//要打印字节区位码的字节数
unsigned char TxBuffer[64]={0,2,3,};//串口发送缓冲区
unsigned char RxBuffer[64]; //串口接收缓冲区
unsigned char Key0=0;
unsigned char Key0_Value=0;
unsigned char Key0_State=0;
unsigned char LED0_State=0;
unsigned char t;
unsigned char JG;//数据比较结果

uint32_t Signal_Fre = 0;
uint32_t g_Time3Count = 0;
double g_deltaAngle = 0;
double g_curAngle = 0;
/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/

//void Delay(int nCount);
int fputc(int ch, FILE *f)
{
  ITM_SendChar(ch);
  return ch;
}


void TIM2_Encoder_Init(u16 arr,u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef  TIM_ICInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);        //使能定时器2时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;         //浮空输入
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;         //浮空输入
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period=arr;
    TIM_TimeBaseStructure.TIM_Prescaler=psc;
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);

    TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising); //编码器模式
    TIM_ICStructInit(&TIM_ICInitStructure);
    //TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICFilter=6;
    //TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;
    //TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;
    //TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;
    TIM_ICInit(TIM2,&TIM_ICInitStructure);
    TIM_ClearFlag(TIM2,TIM_FLAG_Update);//清除更新标志位
    //TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
    TIM_ClearITPendingBit(TIM2,TIM_IT_Update); //清除中断标志位
    TIM_SetCounter(TIM2,TIM_ENCODE_COUNT_INIT);
    TIM_Cmd(TIM2,ENABLE);
}

/****************************TIM3初始化*********************************************/
void TIM3_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period=arr;
    TIM_TimeBaseStructure.TIM_Prescaler=psc;
    TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);

    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

    NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM3,ENABLE);
}

/**
 * 0.05s 一次中断
*/
void TIM3_IRQHandler()
{
    double deltaAngle;
    int deltaCount;
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
    {
        g_Time3Count++;
        Signal_Fre = TIM_GetCounter(TIM2);
        TIM_SetCounter(TIM2,TIM_ENCODE_COUNT_INIT);
        deltaCount = Signal_Fre - TIM_ENCODE_COUNT_INIT;
        deltaAngle = deltaCount * ENCODE_COUNT2ANGLE;
        g_curAngle += deltaAngle;
        if (g_curAngle < 0) {
            g_curAngle += 36000;
        } else if (g_curAngle >= 36000) {
            g_curAngle -= 36000;
        }
        if (g_Time3Count%10 == 0) {
            SEGGER_RTT_printf(0,"g_curAngle=%d \r\n",(int)g_curAngle);
        }

        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //清除TIMx更新中断标志
    }
}


/**
  * @brief  Main program.
  * @param  None
  * @retval : None
  */
int main(void)
{
    /* Setup STM32 system (clock, PLL and Flash configuration) */
    SystemInit();

    RCC_Configuration();

    NVIC_Configuration();

    GPIO_Configuration();

    SysTick_init();                              //延时初始化

    SPI2_Init();                                //初始化SPI硬件口

    USART_Configuration();  //USART1配置

    TIM2_Encoder_Init(65535,0); //定时器2编码器模式
    TIM3_Int_Init(4999,719);//0.05s定时器溢出中断

    /* Infinite loop */
    while (1)
    {
        if (g_Time3Count%20 == 0) {
            LED0_ON();//LED亮
        } else if (g_Time3Count%10 == 0) {
            LED0_OFF();//LED亮
        }
    }
}

static bool g_isFirstZPulse = TRUE;
static double g_ZPaseAngle = 0;
void ENCODE_ZPaseProcess(void)
{
    double angleDelta;
    if (g_isFirstZPulse == TRUE) {
        g_isFirstZPulse = FALSE;
        g_ZPaseAngle = g_curAngle;
        SEGGER_RTT_printf(0,"first Z pulse:g_ZPaseAngle=%d\r\n",(int)g_ZPaseAngle);
        return;
    }

    angleDelta = g_curAngle - g_ZPaseAngle;
    if (angleDelta > 18000) {
        angleDelta -= 36000;
    } else if (angleDelta < -18000) {
        angleDelta += 36000;
    }
    SEGGER_RTT_printf(0,"Z pulse:angleDelta=%d, g_curAngle:%d.\r\n",(int)angleDelta, (int)g_curAngle);
    g_curAngle = g_ZPaseAngle;
}

void EXTI2_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line2) == SET)
    {
        //这里写中断处理的一些内容：
        EXTI_ClearITPendingBit(EXTI_Line2);
        ENCODE_ZPaseProcess();
    }
}
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {

  }
}
#endif



/**
  * @}
  */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
