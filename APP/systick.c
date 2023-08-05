#include "stm32f10x.h"
#include "system_stm32f10x.h"
#include "systick.h"
#include "motor_control.h"

static uint32_t g_Systick = 0;
static unsigned int TimingDelay;
uint16_t g_TIM2Counter = 0;
/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval : None
  */
static void TimingDelay_Decrement(void)
{
    if (TimingDelay != 0x00)
    {
        TimingDelay--;
    }
}

void SysTick_Handler(void)     //一毫秒系统中断
{
    TimingDelay_Decrement();

    g_Systick++;
    if (g_Systick%10 == 0) {
        MotorTickProcess();
    }

}

/**
 * @brief  Inserts a delay time.
 * @param nTime: specifies the delay time length, in milliseconds.
 * @retval : None
 */
void delay_ms(unsigned int nTime)
{
    TimingDelay = nTime;

    while(TimingDelay != 0);
}

uint32_t GetSystick(void)
{
    return g_Systick;
}

void SysTick_init(void)
{
    SysTick_Config(SystemFrequency / 1000);
}
