#include "motor_control.h"
#include <stdio.h>
#include "stm32f10x_gpio.h"
#include "calibration.h"
#include "encode.h"
#include "pwm.h"
#include "stmflash.h"

#define PWM_DUTY_MAX         84
#define PWM_DUTY_MIN         55
#define PWM_DUTY_CALI        80

#define PID_ANGLE_MAX        36000
#define PID_ANGLE_ERROR_MAX  18000
#define PID_ANGLE_ERROR_MIN  -18000

#define MOTOR_VELOCITY_MAX   1000
#define MOTOR_VELOCITY_MIN   208

Controller_t g_control;

static void MotorCalcuDutyToOutput(void)
{
    static bool direction;
    double velocity;
    uint16_t duty;

    if (g_control.pid.output > 0.9) {
        velocity = g_control.pid.output;
        direction = FALSE;
    } else if (g_control.pid.output < -0.9) {
        velocity = -g_control.pid.output;
        direction = TRUE;
    } else/* if ((g_control.pid.outputKp < 0.3) && (g_control.pid.outputKp > -0.3)) */{
        PWM_SetDuty(100);
        printf("switch to stop STATE:_Angle:%d, realAngle:%d\r\n", g_control._Angle, g_control.realAngle);
        g_control.state = STATE_STOP;
        g_control.pid.outputKp = 0;
        g_control.pid.outputKi = 0;
        g_control.pid.outputKd = 0;
        g_control.pid.angleError = 0;
        g_control.pid.angleErrorLast = 0;
        g_control.pid.output = 0;

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
    } else if (g_control.pid.angleError < PID_ANGLE_ERROR_MIN) {
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

    g_control.pid.output = (g_control.pid.outputKp + g_control.pid.outputKi/5000 + g_control.pid.outputKd)/100;

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

/**
 * 设置Pid的 kp ki kd参数; 调整控制效果
*/
void MotorControlSetPidCoeff(const double* coeff)
{
    g_control.pid.kp = coeff[0];
    g_control.pid.ki = coeff[1];
    g_control.pid.kd = coeff[2];
}

/**
 * 设置板子角度偏移
*/
void MotorControlSetAngleOffset(int16_t offset, SetParaType_t type)
{
    if (offset > 18000 || offset < -18000) {
        printf("MotorControlSetAngleOffset para err %d!\r\n", offset);
        return;
    }

    g_control.deltaAngle = offset;
    printf("MotorControlSetAngleOffset para %d!\r\n", offset);

    if (type == SET_UPDAE_FLASH) {
        BoardConfigParaSave();
    }
}

Controller_t* MotorGetControlPara(void)
{
    return &g_control;
}

void MotorControlInit(void)
{
    g_control.state = STATE_NO_CALIB;
    g_control.pid.kp = 2.5;
    g_control.pid.ki = 0.00001;
    g_control.pid.kd = 0;
}

void MotorSetControlAngle(uint16_t angle)
{
    if (angle >= 36000) {
        printf("MotorSetControlAngle para err! angle:%d", angle);
        return;
    }
    g_control._Angle = angle;

    MotorSetControlStatus(STATE_RUNNING);
}

void MotroPrintDebugInfo(void)
{
    //printf("pid_t para: kp:%d, ki:%d, kd:%d\r\n", g_control.pid.kp, g_control.pid.ki, g_control.pid.kd);
    // printf("pid_t out : output:%f, outputKp:%f, outputKi:%f,  outputKd:%f\r\n", g_control.pid.output, \
    //        g_control.pid.outputKp, g_control.pid.outputKi, g_control.pid.outputKd);
    printf("pid_t:_Angle:%d, realAngle:%d\r\n", g_control._Angle, g_control.realAngle);
}

/**
 * 打印当前所处角度
*/
void MotorPrintAngle(void)
{
    printf("$ANGDA,%d,%d*5A\r\n", g_control._Angle, g_control.realAngle);
}

/**
 * 打印PID参数
*/
void MotorPrintPidCoeff(void)
{
    printf("$PIDDA,%f,%f,%f*5A\r\n", g_control.pid.kp, g_control.pid.ki, g_control.pid.kd);
}

void MotorUpdateRealAngle(uint16_t angle)
{
    if (angle >= 36000) {
        printf("MotorUpdateRealAngle para err! angle:%d", angle);
        return;
    }
    g_control.realAngle = angle;
}


void MotorControlAngleCheck(void)
{
    uint16_t angle;
    angle = Encode_UpdateAngle();
    MotorUpdateRealAngle(angle);
    if (angle - g_control._Angle > 200 || angle - g_control._Angle < -200) {
        MotorSetControlStatus(STATE_RUNNING);
    }
}

/**
 * @brief 编码器计算的角度根据设置的 角度偏差deltaAngle 进行校准，默认偏差为0
*/
void MotorAngleCalibrateBoardOffset(int* angle)
{
    int tempAngle = *angle;
    if (tempAngle > 36000 || tempAngle < -36000) {
        printf("MotorAngleCalibrateBoardOffset input angle err: %d", tempAngle);
        return;
    }

    if (g_control.deltaAngle > 18000 || g_control.deltaAngle < -18000) {
        printf("MotorAngleCalibrateBoardOffset deltaangle err: %d", g_control.deltaAngle);
        g_control.deltaAngle = 0;
        return;
    }

    tempAngle += g_control.deltaAngle;
    *angle = tempAngle;
}
