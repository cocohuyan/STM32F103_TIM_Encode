#include "encode.h"
#include "SEGGER_RTT.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "motor_control.h"

void TIM2_Encoder_Init(u16 arr,u16 psc)
{
#if 1
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

    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //编码器模式
    TIM_ICStructInit(&TIM_ICInitStructure);
    //TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICFilter=6;
    //TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;
    //TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;
    //TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);//清除更新标志位
    //TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update); //清除中断标志位
    TIM_SetCounter(TIM2, TIM_ENCODE_COUNT_INIT);
    TIM_Cmd(TIM2, ENABLE);
#endif
    // GPIO_InitTypeDef GPIO_InitStructure;
    // //TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    // TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    // TIM_ICInitTypeDef TIM_ICInitStruct;
    // //TIM_ICInitTypeDef TIM_ICInitStructure;

    // RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);        //使能定时器2时钟
    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //浮空输入
    // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    // GPIO_Init(GPIOA,&GPIO_InitStructure);

    // //3、配置时基单元
    // TIM_InternalClockConfig(TIM2);   //选择内部时钟

    // TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    // TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数
    // TIM_TimeBaseInitStructure.TIM_Period = 65535 - 1;               //ARR周期，给最大值，防止溢出，16位计数器可以满量程计数
    // TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;               //PSC预分频器，这个值决定测周法的标准频率fc，72M/预分频 = 1Mhz，就是计数器自增的频率，就是计数标准频率
    // TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    // TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);             //把以上参数配置为TIM2的时基单元

    // //4、初始化输入捕获单元

    // TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;  //选用TIM2的通道2
    // TIM_ICInitStruct.TIM_ICFilter = 0x1;   //用于选择输入捕获的滤波器
    // TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising; //对应边沿检测极性选择这一部分
    // TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;  //此处不分频，不分频就是每次触发都有效，2分频就是每个一次有效一次，以此类推
    // TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI; //此处选择直连通道，配置数据选择器，可以选择直连通道或者交叉通道
    // TIM_ICInit(TIM2, &TIM_ICInitStruct);
    // //5、配置TRGI的触发源为TI1FP1
    // TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);
    // //6、配置从模式为Reset，给计数器清零，以便重新计数
    // TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
    // //
    // TIM_Cmd(TIM2,ENABLE);
}

//uint32_t IC_GetFreq(void)
//{
//    //fc=72/（PSC + 1）= 1MHZ
//    //执行公式fx = fc / N
//    //N 就是读取CCR的值
//    //TIM_GetCapture1读取定时器3的通道1的CCR的值
//    //输出比较模式下，CCR是只写的，要用setCompare写入
//    //输入捕获模式下，CCR是只读的，要用GetCapture读出
//    return 1000000 / (TIM_GetCapture2(TIM2));
//}


/**
 * 输出的角度范围：0-36000
*/
uint16_t Encode_UpdateAngle(void)
{
    uint16_t counter;
    int angle;

    counter =  TIM_GetCounter(TIM2);

    /* 检查counter值 如果即将溢出需要重新设置 To do */

    /* angle范围 -36000 ~ 36000 */
    angle = ((int)(counter - TIM_ENCODE_COUNT_INIT)%ENCODE_CNT_PERROUND) * ENCODE_COUNT2ANGLE;

    MotorAngleCalibrateBoardOffset(&angle);

    if ((angle < 0) && (angle >= -36000)) {
        angle += 36000;
    } else if (angle < -36000 || angle > 36000) {
        SEGGER_RTT_printf(0,"Encode_UpdateAngle angle error %d!\r\n", angle);
    }

    return (uint16_t)angle;
}
