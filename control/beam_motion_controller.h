/**
 * @file beam_motion_controller.h
 * @brief Manual-zero or optional switch-homed crank-link position control.
 */
#ifndef BEAM_MOTION_CONTROLLER_H
#define BEAM_MOTION_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BEAM_MOTION_IDLE = 0,
    BEAM_MOTION_MANUAL_DIR_TEST_OUT,
    BEAM_MOTION_MANUAL_DIR_TEST_RETURN,
    BEAM_MOTION_DIRECTION_PENDING,
    BEAM_MOTION_RELEASE_A,
    BEAM_MOTION_RELEASE_B,
    BEAM_MOTION_PROBE_A,
    BEAM_MOTION_RETURN_A,
    BEAM_MOTION_PROBE_B,
    BEAM_MOTION_BACKOFF,
    BEAM_MOTION_TO_LEVEL,
    BEAM_MOTION_READY,
    BEAM_MOTION_FAULT
} BeamMotionState;

typedef struct {
    BeamMotionState state;
    uint32_t stateStartMs;
    int32_t probeStartSteps;
    uint8_t activeProbeLevel;
} BeamMotionController;

void BeamMotionController_init(BeamMotionController *controller);
void BeamMotionController_startHoming(
    BeamMotionController *controller, uint32_t nowMs);
void BeamMotionController_tick10ms(
    BeamMotionController *controller, uint32_t nowMs);
void BeamMotionController_setLiftUm(
    BeamMotionController *controller, int32_t liftUm);
void BeamMotionController_stop(BeamMotionController *controller);
bool BeamMotionController_ready(const BeamMotionController *controller);
bool BeamMotionController_faulted(const BeamMotionController *controller);
int32_t BeamMotionController_liftUm(const BeamMotionController *controller);
const char *BeamMotionController_stateText(
    const BeamMotionController *controller);

#endif

