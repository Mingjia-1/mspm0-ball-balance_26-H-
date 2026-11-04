/** @file wheel_speed_controller.h @brief Independent encoder PI loops. */
#ifndef CONTROL_WHEEL_SPEED_CONTROLLER_H_
#define CONTROL_WHEEL_SPEED_CONTROLLER_H_

#include <stdint.h>
#include "bsp/drive_base.h"

typedef struct {
    int32_t integral;
    int32_t leftPulseSum;
    int32_t rightPulseSum;
    int32_t leftTarget;
    int32_t rightTarget;
    int32_t leftActual;
    int32_t rightActual;
    int16_t leftDuty;
    int16_t rightDuty;
    int32_t rightIntegral;
    uint8_t windowTicks;
} WheelSpeedController;

void WheelSpeedController_reset(WheelSpeedController *controller);
void WheelSpeedController_stop(WheelSpeedController *controller);
void WheelSpeedController_step(WheelSpeedController *controller,
    DriveBaseEncoderCounts delta, int32_t leftTarget, int32_t rightTarget);

#endif
