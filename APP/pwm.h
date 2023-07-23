#ifndef _PWM_H_
#define _PWM_H_

#include "stm32f10x.h"

void TIM1_PWM_Init(void);
void PWM_SetDuty(uint8_t duty);

#endif /* _PWM_H_ */
