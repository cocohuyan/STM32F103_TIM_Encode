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

typedef enum
{
    SET_FROM_FLASH,     //从flash中获取参数并设置
    SET_UPDAE_FLASH,    //更新flashc中保存的参数
} SetParaType_t;

/**
 * PID 输出output为占空比
*/
typedef struct {
    bool kpValid, kiValid, kdValid;
    double kp, ki, kd;
    double angleError, angleErrorLast;
    double outputKp, outputKi, outputKd;
    int32_t integralRound;
    int32_t integralRemainder;
    double output;                               // pid计算输出的转速
} PID_t;

typedef struct  {
    uint16_t realAngle;      // 测量当前的实际角度 范围 0-36000
    int16_t deltaAngle;      // 编码器初始0度 与实际0度的偏差  -18000 - 18000
    uint16_t _Angle;         // 设置的目标角度 范围 0-36000
    State_t state;
    PID_t pid;
} Controller_t;

State_t MotorGetControlStatus(void);
void MotorSetControlStatus(State_t state);
void MotorTickProcess(void);
void MotorControlInit(void);
void MotorSetControlAngle(uint16_t angle);
void MotroPrintDebugInfo(void);
void MotorUpdateRealAngle(uint16_t angle);
void MotorPrintPidCoeff(void);
void MotorPrintAngle(void);
void MotorControlSetPidCoeff(const double* coeff);
void MotorControlAngleCheck(void);
void MotorAngleCalibrateBoardOffset(int* angle);
void MotorControlSetAngleOffset(int16_t offset, SetParaType_t type);
Controller_t* MotorGetControlPara(void);

#endif /* _MOTOR_CONTROL_H_ */
