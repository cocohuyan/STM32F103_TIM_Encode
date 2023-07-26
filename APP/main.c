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
#include "SEQueue.h"
#include "pwm.h"
#include "systick.h"
#include "uart.h"
#include "calibration.h"
#include "encode.h"
#include "motor_control.h"

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

/****************************TIM3初始化*********************************************/
void TIM3_Int_Init(u16 arr,u16 psc)
{
#if 0
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
#endif /* 0 */
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_ICInitTypeDef TIM_ICInitStruct;
    //1、开启时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  //由于代码需要TIM2输出PWM，因此，输入捕获定时器需要换一个，此处换为TIM3，TIM3是APB1的外设
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //GPIO的使用需要查看引脚定义
    //2、配置GPIO

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  //上拉输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;		//GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //3、配置时基单元
    TIM_InternalClockConfig(TIM3);   //选择内部时钟

    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数
    TIM_TimeBaseInitStructure.TIM_Period = 65535 - 1;		//ARR周期，给最大值，防止溢出，16位计数器可以满量程计数
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;		//PSC预分频器，这个值决定测周法的标准频率fc，72M/预分频 = 1Mhz，就是计数器自增的频率，就是计数标准频率
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);  //把以上参数配置为TIM3的时基单元
    //4、初始化输入捕获单元

    TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;  //选用TIM3的通道1
    TIM_ICInitStruct.TIM_ICFilter = 0x0;   //用于选择输入捕获的滤波器
    TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising; //对应边沿检测极性选择这一部分
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;  //此处不分频，不分频就是每次触发都有效，2分频就是每个一次有效一次，以此类推
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI; //此处选择直连通道，配置数据选择器，可以选择直连通道或者交叉通道
    TIM_PWMIConfig(TIM3,&TIM_ICInitStruct);  //该函数只支持通道1、2
    //5、配置TRGI的触发源为TI1FP1
    TIM_SelectInputTrigger(TIM3,TIM_TS_TI1FP1);
    //6、配置从模式为Reset，给计数器清零，以便重新计数
    TIM_SelectSlaveMode(TIM3,TIM_SlaveMode_Reset);
    //最后一步
    TIM_Cmd(TIM3,ENABLE);
}

/*
    自此，初始化后，整个电路就能全自动测量，当我们要查看频率时，需要读取CCR，进行计算，
    */
    //当需要查看频率时，需要读取CCR，进行计算
uint32_t IC_GetFreq(void)
{
    //fc=72/（PSC + 1）= 1MHZ
    //执行公式fx = fc / N
    //N 就是读取CCR的值
    //TIM_GetCapture1读取定时器3的通道1的CCR的值
    //输出比较模式下，CCR是只写的，要用setCompare写入
    //输入捕获模式下，CCR是只读的，要用GetCapture读出
    uint16_t ccr;
    ccr = TIM_GetCapture1(TIM3);
    TIM_SetCompare1(TIM3, 0);
    if (ccr == 0 ) {
        return 0;
    }
    return 1000000 / ccr;
}

//获取占空比的函数
uint32_t IC_GetDuty(void)
{
    //高电平的计数值存在CCR2里
    //整个周期的计数值存在CCR1里，占空比=CCR2/CCR1即可
    //显然该数是0~1，要显示整数，给他乘以100
    uint32_t duty;
    uint16_t ccr1, ccr2;
    ccr1 = TIM_GetCapture1(TIM3);
    ccr2 = TIM_GetCapture2(TIM3);

    if (ccr1 == 0) {
        return 0;
    }

    duty = ccr2 * 100 / ccr1; // TIM_GetCapture2捕获CCR2，TIM_GetCapture1捕获CCR1
    TIM_SetCompare1(TIM3, 0);
    TIM_SetCompare2(TIM3, 0);
    return duty;
    //少一个数有没有可能是因为计数器CNT是从0开始计数的呢？？？
}

/**
 * 0.05s 一次中断
*/
void TIM3_IRQHandler()
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //清除TIMx更新中断标志
    }
}

uint16_t comparedata = 8000;
uint32_t lasttick = 0;
/**
  * @brief  Main program.
  * @param  None
  * @retval : None
  */
int main(void)
{
    uint8_t info = 0;
    uint32_t freq;
    uint32_t duty;
    uint32_t j;
    char txMsg[64] = {0};
    /* Setup STM32 system (clock, PLL and Flash configuration) */
    SystemInit();
    RCC_Configuration();
    NVIC_Configuration();
    GPIO_Configuration();
    SysTick_init();                             //延时初始化
    // SPI2_Init();                                //初始化SPI硬件口
    USART_Configuration();                      //USART1配置

    TIM1_PWM_Init();
    TIM2_Encoder_Init(65535,0); // 定时器2编码器模式
    TIM3_Int_Init(4999,719);      // 0.05s定时器溢出中断

    MotorControlInit();
    /* Infinite loop */
    while (1) {
        uint32_t tick = GetSystick();
        if (tick%1000 == 0 && tick !=lasttick) {
            lasttick = tick;
            freq = IC_GetFreq();
            duty = IC_GetDuty();
            SEGGER_RTT_printf(0,"-freq: %d!\r\n", freq);
            SEGGER_RTT_printf(0,"-duty: %d!\r\n", duty);
            j = snprintf(txMsg,64, "-duty: %d!\r\n", duty);
            USART1_SendBuf((const uint8_t*)txMsg, j);
            j = snprintf(txMsg,64, "-freq: %d!\r\n", freq);
            USART1_SendBuf((const uint8_t*)txMsg, j);
        }

        if (DeSEQueue(&g_Queue[0], &info)) {   //从队列中出队
            USART1_Receive(info);
        }
    }
}

void ENCODE_ZPaseProcess(void)
{
    TIM_SetCounter(TIM2,TIM_ENCODE_COUNT_INIT);
    if (MotorGetControlStatus() == STATE_NO_CALIB) {
        MotorSetControlStatus(STATE_STOP);
    }
    SEGGER_RTT_printf(0,"---------------Z pulse--------------!\r\n");
}

void EXTI2_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line2) == SET)
    {
        //这里写中断处理的一些内容：

        ENCODE_ZPaseProcess();
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}

void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) == SET)
    {
        //这里写中断处理的一些内容：
        ENCODE_ZPaseProcess();
        EXTI_ClearITPendingBit(EXTI_Line0);
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
