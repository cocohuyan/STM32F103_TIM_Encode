#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_

#include "stm32f10x.h"

typedef enum
{
    CALI_STATUS_NOINIT = 0,
    CALI_STATUS_INITOK
} CALI_Status;

CALI_Status GetCalibrationStatus(void);
void SetCalibrationStatus(CALI_Status status);

#endif /* _CALIBRATION_H_ */
