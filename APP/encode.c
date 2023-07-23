#include "encode.h"
#include "SEGGER_RTT.h"

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


/**
 * 输出的角度范围：0-36000
*/
uint16_t Encode_UpdateAngle(void)
{
    uint16_t counter;
    int angle;

    counter =  TIM_GetCounter(TIM2);
    angle = ((int)(counter - TIM_ENCODE_COUNT_INIT)%ENCODE_CNT_PERROUND) * ENCODE_COUNT2ANGLE;

    if ((angle < 0) && (angle >= -36000)) {
        angle += 36000;
    } else if (angle < -36000 || angle > 36000) {
        SEGGER_RTT_printf(0,"Encode_UpdateAngle angle error %d!\r\n", angle);
    }

    return (uint16_t)angle;
}
