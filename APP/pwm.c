#include "pwm.h"

#define PWM_PERIOD_COUNT      50000
/* 配置TIM1 PWM模式下 服用引脚I/O */
static void TIM1_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* TIM1 clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1|RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/* 配置TIM1输出PWM信号的模式， 如周期、极性、占空比 */
static void TIM1_Mode_Config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    /* PWM信号电平跳变值 */
    //u16 CCR1_Val = 50000;
    //u16 CCR2_Val = 22500;
    u16 CCR3_Val = 22000;
    //u16 CCR4_Val = 125;

/* -----------------------------------------------------------------------
    TIM1 Configuration: generate 2 PWM signals with 2 different duty cycles:
    TIM1CLK = 72 MHz, Prescaler = 0x0, TIM1 counter clock = 36 MHz
    TIM3 ARR Register = 50000 => TIM1 Frequency = TIM1 counter clock/(ARR + 1)
    TIM3 Frequency = 1440Hz.
    TIM1 Channel1 duty cycle = (TIM1_CCR1/ TIM1_ARR)* 100
    TIM1 Channel2 duty cycle = (TIM1_CCR2/ TIM1_ARR)* 100

  ----------------------------------------------------------------------- */

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD_COUNT;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    /* PWM1 Mode configuration: Channel1 */
    /*TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;//
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);*/

    /* PWM1 Mode configuration: Channel2
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;//
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);*/


    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;//
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);



    TIM_ARRPreloadConfig(TIM1, ENABLE);

    /* TIM1 enable counter */
    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

/**
 * 输入 duty:范围 0 ~ 100， 单位 1/100
*/
void PWM_SetDuty(uint8_t duty)
{
    uint16_t compare2;
    if (duty > 100) {
        return;
    }

    compare2 = (100 - duty) * PWM_PERIOD_COUNT/100;
    TIM_SetCompare3(TIM1, compare2);
}

/* 初始化TIM1 */
void TIM1_PWM_Init(void)
{
    TIM1_GPIO_Config();
    TIM1_Mode_Config();
}
