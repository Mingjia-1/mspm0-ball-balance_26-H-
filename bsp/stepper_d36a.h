/**
 * @file stepper_d36a.h
 * @brief D36A STEP/DIR/EN axis-1 motion engine.
 */
#ifndef STEPPER_D36A_H
#define STEPPER_D36A_H

#include <stdbool.h>
#include <stdint.h>

void StepperD36a_init(void);
void StepperD36a_tick10ms(void);
void StepperD36a_timerIsr(void);
void StepperD36a_enable(void);
void StepperD36a_disable(void);
void StepperD36a_emergencyStop(void);

void StepperD36a_setDirectionLevelForUp(bool high);
bool StepperD36a_directionKnown(void);
bool StepperD36a_directionLevelForUp(void);
void StepperD36a_setPositionSteps(int32_t positionSteps);
int32_t StepperD36a_positionSteps(void);
void StepperD36a_setTargetSteps(
    int32_t targetSteps, int32_t maxRatePps, int32_t accelerationPps2);
void StepperD36a_runDirection(
    bool directionLevel, int32_t stepCount, int32_t maxRatePps,
    int32_t accelerationPps2);
bool StepperD36a_busy(void);

#endif
