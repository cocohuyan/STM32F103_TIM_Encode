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
#include "core_cm3.h"
#include "stdio.h"
#include "SEGGER_RTT.h"

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
    TIM_SetCounter(TIM2,32768);
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

/*********************************TIM3中断处理***************************************/
void TIM3_IRQHandler()
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
    {
        g_Time3Count++;
        Signal_Fre = TIM_GetCounter(TIM2);
        Signal_Fre = (int)Signal_Fre * 20 / 4;  //定时器0.05s中断一次，并且编码器模式是4倍频，计算出编码器的速度
        TIM_SetCounter(TIM2,32768);

        SEGGER_RTT_printf(0,"Signal_Fre=%d \r\n",Signal_Fre);
        // SEGGER_RTT_printf(0,"TIM3IRQ \r\n");
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
