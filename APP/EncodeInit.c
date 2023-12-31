//*****************主函数*****************//
#include "sys.h"
#include "delay.h"
#include "timer.h"
//#include "usart.h"
#include "SEGGER_RTT.h"

int16_t Signal_Fre = 0;

int main_test(void)
{
    delay_init();             //延时函数初始化
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);      //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
  //uart_init(115200);     //串口初始化为115200

    TIM8_PWM_Init(899,3);     //PWM频率=72000000/900/4=20Khz
    TIM2_Encoder_Init(65535,0); //定时器2编码器模式
    TIM3_Int_Init(4999,719);//0.05s定时器溢出中断

    TIM8_PWMStopGPIO_Init();
    delay_ms(10);
    SEGGER_RTT_printf(0,"Test \r\n");

  while(1)
    {

        TIM_SetCompare1(TIM8,450);
      TIM_SetCompare2(TIM8,0);

        delay_ms(10);
    }
}

//**********定时器配置****************//
#include "timer.h"
//#include "usart.h"
#include "SEGGER_RTT.h"

extern int16_t Signal_Fre;




/********************TIM8初始化***********************/
//TIM8 PWM部分初始化
//CH1:PC6
//CH1N:PA7
//CH2:PC7
//CH2N:PB0
//CH3:PC8
//CH3N:PB1
//BKIN:PA7
//arr：自动重装值
//psc：时钟预分频数
void TIM8_PWM_Init(u16 arr,u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);    //使能定时器8时钟

     RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC , ENABLE);  //使能GPIO外设功能模块时钟

  //设置该引脚为复用输出功能,输出TIM8 CH1的PWM脉冲波形    GPIOC.6
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //TIM_CH1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIO

    //设置该引脚为复用输出功能,输出TIM8 CH1N的PWM脉冲波形    GPIOA.7
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //TIM_CH1N
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO

    //设置该引脚为复用输出功能,输出TIM8 CH2的PWM脉冲波形    GPIOC.7
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //TIM_CH2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIO

    //设置该引脚为复用输出功能,输出TIM8 CH1N的PWM脉冲波形    GPIOB.0
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //TIM_CH2N
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO

  //设置该引脚为复用输出功能,输出TIM8 CH3的PWM脉冲波形    GPIOC.8
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //TIM_CH3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIO

    //设置该引脚为复用输出功能,输出TIM8 CH3N的PWM脉冲波形    GPIOB.1
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; //TIM_CH3N
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO

  //刹车引脚PA6
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //BKIN
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO

   //初始化TIM1
    TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

    //初始化TIM1 PWM模式
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
     TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; //互补比较输出使能
    TIM_OCInitStructure.TIM_Pulse = 0;  //占空比 CCR的值
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
    //TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High; //互补输出极性:TIM输出比较极性高
    //TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low ; //互补输出极性:TIM输出比较极性低
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_OC1Init(TIM8, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM8
    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);  //使能TIM1在CCR上的预装载寄存器

    TIM_OCInitStructure.TIM_Pulse = 0;  //占空比 CCR的值
    TIM_OC2Init(TIM8, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM8
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);  //使能TIM1在CCR上的预装载寄存器

    //TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;  //刹车引脚恢复后引脚输出状态
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;  //刹车引脚不能恢复后引脚输出状态
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;   //刹车使能
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;   //刹车引脚高电平刹车
    TIM_BDTRInitStructure.TIM_DeadTime = 36; //死区时间36:500ns

    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;

    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    //TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Disable;
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    //TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Disable;
    TIM_BDTRConfig(TIM8,&TIM_BDTRInitStructure);

    TIM_Cmd(TIM8, ENABLE);  //使能TIM8
    TIM_CtrlPWMOutputs(TIM8,ENABLE);  //主输出使能，当使用通用定时器时，这个不需要

}


void TIM8_PWMStopGPIO_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_Cmd(TIM8, DISABLE);  //失能TIM8

    //设置该引脚为复用输出功能,输出TIM8 CH1的PWM脉冲波形    GPIOC.6
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //TIM_CH1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIO
    GPIO_ResetBits(GPIOC,GPIO_Pin_6);                         //PC.6 输出低

    //设置该引脚为复用输出功能,输出TIM8 CH1N的PWM脉冲波形    GPIOA.7
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //TIM_CH1N
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO
    GPIO_ResetBits(GPIOA,GPIO_Pin_7);                         //PA.7 输出低

    //设置该引脚为复用输出功能,输出TIM8 CH2的PWM脉冲波形    GPIOC.7
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //TIM_CH2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIO
    GPIO_ResetBits(GPIOC,GPIO_Pin_7);                         //PC.7 输出低

    //设置该引脚为复用输出功能,输出TIM8 CH1N的PWM脉冲波形    GPIOB.0
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //TIM_CH2N
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO
    GPIO_ResetBits(GPIOB,GPIO_Pin_0);                         //PB.0 输出低
}


void TIM8_PWMStartGPIO_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_Cmd(TIM8, ENABLE);  //使能TIM8

    //设置该引脚为复用输出功能,输出TIM8 CH1的PWM脉冲波形    GPIOC.6
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //TIM_CH1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIO

    //设置该引脚为复用输出功能,输出TIM8 CH1N的PWM脉冲波形    GPIOA.7
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //TIM_CH1N
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO

    //设置该引脚为复用输出功能,输出TIM8 CH2的PWM脉冲波形    GPIOC.7
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //TIM_CH2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIO

    //设置该引脚为复用输出功能,输出TIM8 CH1N的PWM脉冲波形    GPIOB.0
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //TIM_CH2N
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO
}


