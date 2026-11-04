/**
 * @file beam_motion_controller.c
 * @brief No-limit manual zero/direction test plus optional switch homing.
 */
#include "control/beam_motion_controller.h"

#include "bsp/limit_switch.h"
#include "bsp/stepper_d36a.h"
#include "config/beam_setup_config.h"
#include "config/control_config.h"
#include "config/hardware_config.h"

static void enterState(BeamMotionController *controller,
    BeamMotionState state, uint32_t nowMs)
{
    controller->state = state;
    controller->stateStartMs = nowMs;
}

static void fault(BeamMotionController *controller, uint32_t nowMs)
{
    StepperD36a_emergencyStop();
    enterState(controller, BEAM_MOTION_FAULT, nowMs);
}

/*
 * The ball controller continues to command endpoint lift in micrometres.
 * Near the manually levelled crank mid-point, arc length is r*theta, giving
 * steps = lift * steps_per_revolution / circumference. Normal commands are
 * only +/-3.2 mm on a 28.55 mm crank, so the sine-vs-arc error stays below
 * roughly 0.3 percent while avoiding floating point in the 10 ms loop.
 */
static int32_t liftUmToSteps(int32_t liftUm)
{
    int32_t numerator = liftUm * H_STEPPER_STEPS_PER_REV;

    if (numerator >= 0) {
        numerator += H_CRANK_CIRCUMFERENCE_UM / 2L;
    } else {
        numerator -= H_CRANK_CIRCUMFERENCE_UM / 2L;
    }
    return numerator / H_CRANK_CIRCUMFERENCE_UM;
}

static int32_t stepsToLiftUm(int32_t steps)
{
    return (steps * H_CRANK_CIRCUMFERENCE_UM) /
        H_STEPPER_STEPS_PER_REV;
}

static void startProbe(BeamMotionController *controller,
    uint8_t rawDirectionLevel, BeamMotionState state, uint32_t nowMs)
{
    controller->activeProbeLevel = rawDirectionLevel;
    controller->probeStartSteps = StepperD36a_positionSteps();
    StepperD36a_runDirection(rawDirectionLevel != 0U,
        H_BEAM_DIRECTION_PROBE_STEPS, H_STEPPER_HOME_PPS,
        H_STEPPER_HOME_ACCEL_PPS2);
    enterState(controller, state, nowMs);
}

static void acceptDownDirection(BeamMotionController *controller,
    uint8_t downDirectionLevel, uint32_t nowMs)
{
    StepperD36a_emergencyStop();
    StepperD36a_setDirectionLevelForUp(downDirectionLevel == 0U);
    StepperD36a_setPositionSteps(0);
    StepperD36a_setTargetSteps(H_BEAM_HOME_BACKOFF_STEPS,
        H_STEPPER_HOME_PPS, H_STEPPER_HOME_ACCEL_PPS2);
    enterState(controller, BEAM_MOTION_BACKOFF, nowMs);
}

static void acceptUpDirection(BeamMotionController *controller,
    uint8_t upDirectionLevel, uint32_t nowMs)
{
    StepperD36a_emergencyStop();
    StepperD36a_setDirectionLevelForUp(upDirectionLevel != 0U);
    StepperD36a_setPositionSteps(H_BEAM_HOME_BACKOFF_STEPS);
    StepperD36a_setTargetSteps(H_BEAM_LEVEL_OFFSET_STEPS,
        H_STEPPER_RUN_MAX_PPS, H_STEPPER_RUN_ACCEL_PPS2);
    enterState(controller, BEAM_MOTION_TO_LEVEL, nowMs);
}

void BeamMotionController_init(BeamMotionController *controller)
{
    controller->state = BEAM_MOTION_IDLE;
    controller->stateStartMs = 0U;
    controller->probeStartSteps = 0;
    controller->activeProbeLevel = 0U;
}

void BeamMotionController_startHoming(
    BeamMotionController *controller, uint32_t nowMs)
{
    StepperD36a_emergencyStop();

#if H_BEAM_USE_HOME_SWITCH
    StepperD36a_setPositionSteps(0);
    if (LimitSwitch_homeActive()) {
        StepperD36a_runDirection(false, H_BEAM_HOME_BACKOFF_STEPS,
            H_STEPPER_HOME_PPS, H_STEPPER_HOME_ACCEL_PPS2);
        enterState(controller, BEAM_MOTION_RELEASE_A, nowMs);
    } else {
        startProbe(controller, 0U, BEAM_MOTION_PROBE_A, nowMs);
    }
#else
    /*
     * With no absolute sensor, the operator's physically level beam is the
     * only available reference. Logical coordinates are centred here; the
     * direct crank then operates only inside the symmetric local soft range.
     */
    StepperD36a_setPositionSteps(H_BEAM_LEVEL_OFFSET_STEPS);
#if H_STEPPER_UP_DIRECTION_VERIFIED
    StepperD36a_setDirectionLevelForUp(
        H_STEPPER_DIR_LEVEL_FOR_UP != 0U);
    enterState(controller, BEAM_MOTION_READY, nowMs);
#else
    /*
     * Commissioning-only motion: raw DIR=0 moves exactly 1 mm, then raw DIR=1
     * returns the same number of pulses.  Closed-loop start remains locked.
     */
    StepperD36a_runDirection(false, H_STEPPER_DIRECTION_TEST_STEPS,
        H_STEPPER_DIRECTION_TEST_PPS,
        H_STEPPER_DIRECTION_TEST_ACCEL_PPS2);
    enterState(controller, BEAM_MOTION_MANUAL_DIR_TEST_OUT, nowMs);
#endif
#endif
}

void BeamMotionController_tick10ms(
    BeamMotionController *controller, uint32_t nowMs)
{
    StepperD36a_tick10ms();

    switch (controller->state) {
    case BEAM_MOTION_MANUAL_DIR_TEST_OUT:
        if (!StepperD36a_busy()) {
            StepperD36a_runDirection(true,
                H_STEPPER_DIRECTION_TEST_STEPS,
                H_STEPPER_DIRECTION_TEST_PPS,
                H_STEPPER_DIRECTION_TEST_ACCEL_PPS2);
            enterState(controller,
                BEAM_MOTION_MANUAL_DIR_TEST_RETURN, nowMs);
        } else if ((uint32_t)(nowMs - controller->stateStartMs) >
                   H_STEPPER_HOME_TIMEOUT_MS) {
            fault(controller, nowMs);
        }
        break;

    case BEAM_MOTION_MANUAL_DIR_TEST_RETURN:
        if (!StepperD36a_busy()) {
            StepperD36a_emergencyStop();
            StepperD36a_setPositionSteps(H_BEAM_LEVEL_OFFSET_STEPS);
            enterState(controller, BEAM_MOTION_DIRECTION_PENDING, nowMs);
        } else if ((uint32_t)(nowMs - controller->stateStartMs) >
                   H_STEPPER_HOME_TIMEOUT_MS) {
            fault(controller, nowMs);
        }
        break;

    case BEAM_MOTION_RELEASE_A:
        if (!LimitSwitch_homeActive()) {
            acceptUpDirection(controller, 0U, nowMs);
        } else if (!StepperD36a_busy()) {
            StepperD36a_runDirection(true,
                2L * H_BEAM_HOME_BACKOFF_STEPS,
                H_STEPPER_HOME_PPS, H_STEPPER_HOME_ACCEL_PPS2);
            enterState(controller, BEAM_MOTION_RELEASE_B, nowMs);
        } else if ((uint32_t)(nowMs - controller->stateStartMs) >
                   H_STEPPER_HOME_TIMEOUT_MS) {
            fault(controller, nowMs);
        }
        break;

    case BEAM_MOTION_RELEASE_B:
        if (!LimitSwitch_homeActive()) {
            acceptUpDirection(controller, 1U, nowMs);
        } else if (!StepperD36a_busy() ||
                   ((uint32_t)(nowMs - controller->stateStartMs) >
                    H_STEPPER_HOME_TIMEOUT_MS)) {
            fault(controller, nowMs);
        }
        break;

    case BEAM_MOTION_PROBE_A:
        if (LimitSwitch_homeActive()) {
            acceptDownDirection(controller, 0U, nowMs);
        } else if (!StepperD36a_busy()) {
            StepperD36a_runDirection(true,
                H_BEAM_DIRECTION_PROBE_STEPS,
                H_STEPPER_HOME_PPS, H_STEPPER_HOME_ACCEL_PPS2);
            enterState(controller, BEAM_MOTION_RETURN_A, nowMs);
        } else if ((uint32_t)(nowMs - controller->stateStartMs) >
                   H_STEPPER_PROBE_TIMEOUT_MS) {
            fault(controller, nowMs);
        }
        break;

    case BEAM_MOTION_RETURN_A:
        if (!StepperD36a_busy()) {
            startProbe(controller, 1U, BEAM_MOTION_PROBE_B, nowMs);
        } else if ((uint32_t)(nowMs - controller->stateStartMs) >
                   H_STEPPER_RETURN_TIMEOUT_MS) {
            fault(controller, nowMs);
        }
        break;

    case BEAM_MOTION_PROBE_B:
        if (LimitSwitch_homeActive()) {
            acceptDownDirection(controller, 1U, nowMs);
        } else if (!StepperD36a_busy() ||
                   ((uint32_t)(nowMs - controller->stateStartMs) >
                    H_STEPPER_PROBE_TIMEOUT_MS)) {
            fault(controller, nowMs);
        }
        break;

    case BEAM_MOTION_BACKOFF:
        if (!StepperD36a_busy()) {
            if (LimitSwitch_homeActive()) {
                fault(controller, nowMs);
            } else {
                StepperD36a_setTargetSteps(H_BEAM_LEVEL_OFFSET_STEPS,
                    H_STEPPER_RUN_MAX_PPS, H_STEPPER_RUN_ACCEL_PPS2);
                enterState(controller, BEAM_MOTION_TO_LEVEL, nowMs);
            }
        } else if ((uint32_t)(nowMs - controller->stateStartMs) >
                   H_STEPPER_HOME_TIMEOUT_MS) {
            fault(controller, nowMs);
        }
        break;

    case BEAM_MOTION_TO_LEVEL:
        if (!StepperD36a_busy()) {
            enterState(controller, BEAM_MOTION_READY, nowMs);
        } else if ((uint32_t)(nowMs - controller->stateStartMs) >
                   H_STEPPER_HOME_TIMEOUT_MS) {
            fault(controller, nowMs);
        }
        break;

    default:
        break;
    }
}

void BeamMotionController_setLiftUm(
    BeamMotionController *controller, int32_t liftUm)
{
    int32_t liftSteps;
    int32_t targetSteps;

    if (controller->state != BEAM_MOTION_READY) return;
    liftSteps = liftUmToSteps(liftUm);
    if (liftSteps > H_BEAM_CONTROL_RANGE_STEPS) {
        liftSteps = H_BEAM_CONTROL_RANGE_STEPS;
    } else if (liftSteps < -H_BEAM_CONTROL_RANGE_STEPS) {
        liftSteps = -H_BEAM_CONTROL_RANGE_STEPS;
    }
    targetSteps = H_BEAM_LEVEL_OFFSET_STEPS + liftSteps;
    StepperD36a_setTargetSteps(targetSteps,
        H_STEPPER_SERVO_MAX_PPS, H_STEPPER_SERVO_ACCEL_PPS2);
}
void BeamMotionController_stop(BeamMotionController *controller)
{
    StepperD36a_emergencyStop();
    controller->state = BEAM_MOTION_IDLE;
}

bool BeamMotionController_ready(const BeamMotionController *controller)
{
    return controller->state == BEAM_MOTION_READY;
}

bool BeamMotionController_faulted(const BeamMotionController *controller)
{
    return controller->state == BEAM_MOTION_FAULT;
}

int32_t BeamMotionController_liftUm(const BeamMotionController *controller)
{
    (void)controller;
    return stepsToLiftUm(
        StepperD36a_positionSteps() - H_BEAM_LEVEL_OFFSET_STEPS);
}

const char *BeamMotionController_stateText(
    const BeamMotionController *controller)
{
    switch (controller->state) {
    case BEAM_MOTION_IDLE: return "BEAM IDLE";
    case BEAM_MOTION_MANUAL_DIR_TEST_OUT: return "DIR0 TEST";
    case BEAM_MOTION_MANUAL_DIR_TEST_RETURN: return "DIR RETURN";
    case BEAM_MOTION_DIRECTION_PENDING: return "REPORT DIR";
    case BEAM_MOTION_RELEASE_A:
    case BEAM_MOTION_RELEASE_B: return "HOME REL";
    case BEAM_MOTION_PROBE_A:
    case BEAM_MOTION_RETURN_A:
    case BEAM_MOTION_PROBE_B: return "HOME DIR";
    case BEAM_MOTION_BACKOFF: return "HOME BACK";
    case BEAM_MOTION_TO_LEVEL: return "TO LEVEL";
    case BEAM_MOTION_READY: return "READY";
    case BEAM_MOTION_FAULT: return "DRIVE ERR";
    default: return "BEAM ERR";
    }
}
