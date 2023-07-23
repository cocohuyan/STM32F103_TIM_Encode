#ifndef _SYSTICK_H_
#define _SYSTICK_H_

#include "stm32f10x.h"

void SysTick_init(void);
void delay_ms(unsigned int nTime);
uint32_t GetSystick(void);

#endif /* _SYSTICK_H_*/
