#include "motor_control.h"
#include "stm32f10x_gpio.h"
#include "calibration.h"
#include "encode.h"
#include "pwm.h"

#define PWM_DUTY_MAX         84
#define PWM_DUTY_MIN         55
#define PWM_DUTY_CALI        80

#define PID_ANGLE_MAX        36000
#define PID_ANGLE_ERROR_MAX  18000
#define PID_ANGLE_ERROR_MIN  -18000

#define MOTOR_VELOCITY_MAX   1000
#define MOTOR_VELOCITY_MIN   208
/**
 * PID 输出output为占空比
*/
typedef struct {
    bool kpValid, kiValid, kdValid;
    int32_t kp, ki, kd;
    int32_t angleError, angleErrorLast;
    int32_t outputKp, outputKi, outputKd;
    int32_t integralRound;
    int32_t integralRemainder;
    int32_t output;                               // pid计算输出的转速
} PID_t;

typedef struct  {
    uint16_t realAngle;
    uint16_t _Angle;
    State_t state;
    PID_t pid;
} Controller_t;

Controller_t g_control;

static void MotorCalcuDutyToOutput(void)
{
    static bool direction;
    uint32_t velocity;
    uint16_t duty;

    if (g_control.pid.output > 10) {
        velocity = g_control.pid.output;
        direction = TRUE;
    } else if (g_control.pid.output < -10) {
        velocity = -g_control.pid.output;
        direction = FALSE;
    } else {
        g_control.state = STATE_STOP;
        PWM_SetDuty(100);
        return;
    }

    // pid输出速度根据电机能力 限制在合理范围内，
    if (velocity > MOTOR_VELOCITY_MAX) {
        velocity = MOTOR_VELOCITY_MAX;
    }

    // 根据手册 电压5V时，空转速度 1300
    // 这里简单假定 占空比 100 - 0 对应速度 0-1000
    duty = 100 - velocity/10;
    // 目前占空比在一定范围内，所以软件暂时限制在合理范围内 电机才可正常运转可能和电源功率不足有关
    if (duty > PWM_DUTY_MAX) {
        duty = PWM_DUTY_MAX;
    } else if (duty < PWM_DUTY_MIN) {
        duty = PWM_DUTY_MIN;
    }

    if (direction) {
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
    } else {
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);
    }
    PWM_SetDuty(duty);
}

static void MotorPidControlPosition(void)
{
    g_control.realAngle = Encode_UpdateAngle();
    g_control.pid.angleErrorLast = g_control.pid.angleError;            // 保存上次的误差
    g_control.pid.angleError = g_control._Angle - g_control.realAngle;  // 更新误差

    //误差确保在 -18000 ~ 18000 之间
    if (g_control.pid.angleError > PID_ANGLE_ERROR_MAX) {
        g_control.pid.angleError -= PID_ANGLE_MAX;
    } else if (g_control.pid.angleError < PID_ANGLE_ERROR_MAX) {
        g_control.pid.angleError += PID_ANGLE_MAX;
    }

    g_control.pid.outputKp = g_control.pid.kp * g_control.pid.angleError;  // Outputkp = kp * error
    g_control.pid.outputKi += g_control.pid.ki * g_control.pid.angleError; // Outputki = ki * (error(k) + ... + error(1))
    if (g_control.pid.outputKi > PID_ANGLE_ERROR_MAX) {
        g_control.pid.outputKi = PID_ANGLE_ERROR_MAX;
    } else if (g_control.pid.outputKi < PID_ANGLE_ERROR_MIN) {
        g_control.pid.outputKi = PID_ANGLE_ERROR_MIN;
    }

    g_control.pid.outputKd = g_control.pid.kd * (g_control.pid.angleError - g_control.pid.angleErrorLast);

    g_control.pid.output = g_control.pid.outputKp + g_control.pid.outputKi + g_control.pid.outputKd;

    // 根据输出的速度 转换为占空比并控制电机
    MotorCalcuDutyToOutput();
}

void MotorTickProcess(void)
{

    switch (g_control.state) {
        case STATE_NO_CALIB:
            PWM_SetDuty(PWM_DUTY_CALI);  //开机先转动电机 获取Z相脉冲. 校准初始位置
            break;
        case STATE_STOP:
            PWM_SetDuty(100);
            break;
        case STATE_RUNNING:
            MotorPidControlPosition();
            break;
        default:
            break;
    }
}

State_t MotorGetControlStatus(void)
{
    return g_control.state;
}

void MotorSetControlStatus(State_t state)
{
    g_control.state = state;
}

void MotorControlInit(void)
{
    g_control.state = STATE_NO_CALIB;
    g_control.pid.kp = 10;
    g_control.pid.ki = 0.01;
    g_control.pid.kd = -0.1;
}

void MotorSetControlAngle(uint16_t angle)
{
    g_control._Angle = angle;

    MotorSetControlStatus(STATE_RUNNING);
}
