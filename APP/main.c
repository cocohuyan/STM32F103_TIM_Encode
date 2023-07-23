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
    /* Setup STM32 system (clock, PLL and Flash configuration) */
    SystemInit();
    RCC_Configuration();
    NVIC_Configuration();
    GPIO_Configuration();
    SysTick_init();                             //延时初始化
    // SPI2_Init();                                //初始化SPI硬件口
    USART_Configuration();                      //USART1配置

    TIM1_PWM_Init();
    //TIM2_Encoder_Init(65535,0); //定时器2编码器模式
    TIM3_Int_Init(4999,719);//0.05s定时器溢出中断

    MotorControlInit();
    /* Infinite loop */
    while (1) {
        uint32_t tick = GetSystick();
        if (tick%5000 == 0 && tick !=lasttick) {
            lasttick = tick;
            //TIM_SetCompare2(TIM1, comparedata);
            comparedata += 1000;
            SEGGER_RTT_printf(0,"TIM_SetCompare1 %d!\r\n", comparedata);
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
