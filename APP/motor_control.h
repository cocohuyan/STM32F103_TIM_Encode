#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

#include "stm32f10x.h"

typedef enum
{
    STATE_STOP,
    STATE_FINISH,
    STATE_RUNNING,
    STATE_OVERLOAD,
    STATE_STALL,
    STATE_NO_CALIB
} State_t;

State_t MotorGetControlStatus(void);
void MotorSetControlStatus(State_t state);
void MotorTickProcess(void);
void MotorControlInit(void);

#endif /* _MOTOR_CONTROL_H_ */
