#include "calibration.h"

static CALI_Status EncoderCaliStatus = CALI_STATUS_NOINIT;

CALI_Status GetCalibrationStatus(void)
{
    return EncoderCaliStatus;
}

void SetCalibrationStatus(CALI_Status status)
{
    EncoderCaliStatus = status;
}

/**
 * 校准处理: 转动电机直到接收到 Z相脉冲并将此时的位置识别为0度. 电机位置固定在0度;
 *
*/
void CalibrationProcess(void)
{
    /* 转动电机 */

}
