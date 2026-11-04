/** @file line_controller.h @brief Continuous black-line controller for H track. */
#ifndef CONTROL_LINE_CONTROLLER_H_
#define CONTROL_LINE_CONTROLLER_H_

#include <stdint.h>
#include "bsp/line_sensor_8ch.h"

typedef struct {
    int32_t filteredError;
    int32_t previousError;
    int32_t steering;
    uint16_t lostTicks;
} LineController;

typedef struct {
    int32_t leftTarget;
    int32_t rightTarget;
    int32_t error;
    int32_t steering;
    uint8_t lineLost;
    uint8_t fault;
} LineControllerOutput;

void LineController_reset(LineController *controller);
LineControllerOutput LineController_step(LineController *controller,
    LineSensorReading sensor, int32_t requestedBaseSpeed);

#endif
