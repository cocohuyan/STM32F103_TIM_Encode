#ifndef _ENCODE_H_
#define _ENCODE_H_

#include "stm32f10x.h"

#define             ENCODE_RESOLUTION               2000
#define             ENCODE_TIM_DIV                  4
#define             ENCODE_CNT_PERROUND             (ENCODE_RESOLUTION*ENCODE_TIM_DIV)

#define             ENCODE_COUNT2ANGLE              (4.5) // (36000.0/ENCODE_CNT_PERROUND)  角度乘100 减少误差
#define             TIM_ENCODE_COUNT_INIT           (0x8000)

void TIM2_Encoder_Init(uint16_t arr,uint16_t psc);
uint16_t Encode_UpdateAngle(void);

#endif /* _ENCODE_H_ */
