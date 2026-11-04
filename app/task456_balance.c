/**
 * @file task456_balance.c
 * @brief Visual ball hold plus bounded JY61P vehicle-pitch feedforward.
 */
#include "app/task456_balance.h"

#include "app/jy61p_link.h"
#include "app/vision_link.h"
#include "config/control_config.h"
#include "control/beam_motion_controller.h"

#define TASK456_NUMBER_4 (4U)
#define TASK456_NUMBER_5 (5U)
#define TASK456_NUMBER_6 (6U)

typedef enum {
    TASK456_IDLE = 0,
    TASK456_HOMING,
    TASK456_ARMED,
    TASK456_RUNNING,
    TASK456_VISION_FAULT,
    TASK456_BEAM_FAULT
} Task456State;

typedef enum {
    TASK456_HOLD_TRACK = 0,
    TASK456_HOLD_BREAKAWAY,
    TASK456_HOLD_STABLE,
    TASK456_HOLD_GUARD
} Task456HoldPhase;

typedef struct {
    int32_t filteredVelocityMmPerSecond;
    int32_t lastOutputUm;
    int32_t progressAnchorMm;
    int32_t breakawayStartMm;
    uint32_t progressElapsedMs;
    uint32_t breakawayElapsedMs;
    int8_t direction;
    Task456HoldPhase phase;
} Task456HoldController;

static Task456State g_state;
static BeamMotionController g_beam;
static Task456HoldController g_hold;
static uint8_t g_taskNumber;
static int32_t g_positionMm;
static int32_t g_velocityMmPerSecond;
static int32_t g_targetMm;
static int32_t g_imuZeroMilliDegrees;
static int32_t g_imuAngleMilliDegrees;
static uint32_t g_lastVisionReceivedAtMs;
static uint32_t g_controlSampleDeltaMs;
static uint32_t g_visionLostStartMs;
static uint32_t g_runStartMs;
static uint8_t g_visionFresh;
static uint8_t g_visionValid;
static uint8_t g_visionCalibrated;
static uint8_t g_visionUsable;
static uint8_t g_newMeasurement;
static uint8_t g_imuUsable;
static uint8_t g_haveImuZero;

static int32_t absolute32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t clamp32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static int32_t sign32(int32_t value)
{
    return (value > 0) ? 1L : ((value < 0) ? -1L : 0L);
}

static void holdControllerReset(
    Task456HoldController *controller, int32_t positionMm)
{
    controller->filteredVelocityMmPerSecond = 0;
    controller->lastOutputUm = 0;
    controller->progressAnchorMm = positionMm;
    controller->breakawayStartMm = positionMm;
    controller->progressElapsedMs = 0U;
    controller->breakawayElapsedMs = 0U;
    controller->direction = 0;
    controller->phase = TASK456_HOLD_STABLE;
}

static int32_t holdControllerStep(Task456HoldController *controller,
    int32_t targetMm, int32_t positionMm, int32_t velocityMmPerSecond,
    uint8_t newMeasurement, uint32_t sampleDeltaMs)
{
    int32_t errorMm = targetMm - positionMm;
    int32_t direction = sign32(errorMm);
    int32_t filteredVelocity =
        controller->filteredVelocityMmPerSecond;
    int32_t accelerationCommand;
    int32_t commandUm;
    int32_t outputDeltaUm;
    int32_t projectedPositionMm;
    uint8_t guardActive = 0U;

    if (newMeasurement != 0U) {
        controller->filteredVelocityMmPerSecond +=
            (velocityMmPerSecond -
             controller->filteredVelocityMmPerSecond) / 2L;
        filteredVelocity = controller->filteredVelocityMmPerSecond;
    }

    projectedPositionMm = positionMm +
        filteredVelocity * H_TASK456_GUARD_LOOKAHEAD_MS / 1000L;
    if ((filteredVelocity >= H_TASK456_GUARD_TRIGGER_SPEED_MM_S) &&
        (projectedPositionMm >= H_TASK456_GUARD_BOUNDARY_MM)) {
        commandUm = H_TASK456_COMMAND_MAX_UM;
        guardActive = 1U;
    } else if ((filteredVelocity <=
                -H_TASK456_GUARD_TRIGGER_SPEED_MM_S) &&
               (projectedPositionMm <=
                -H_TASK456_GUARD_BOUNDARY_MM)) {
        commandUm = -H_TASK456_COMMAND_MAX_UM;
        guardActive = 1U;
    }

    if (guardActive != 0U) {
        controller->phase = TASK456_HOLD_GUARD;
        controller->progressAnchorMm = positionMm;
        controller->progressElapsedMs = 0U;
        controller->breakawayElapsedMs = 0U;
    } else {
        if (direction != controller->direction) {
            controller->direction = (int8_t)direction;
            controller->progressAnchorMm = positionMm;
            controller->breakawayStartMm = positionMm;
            controller->progressElapsedMs = 0U;
            controller->breakawayElapsedMs = 0U;
            controller->phase = TASK456_HOLD_TRACK;
        }

        if (newMeasurement != 0U) {
            int32_t signedProgressMm =
                (positionMm - controller->progressAnchorMm) * direction;
            int32_t alignedSpeedMmPerSecond =
                filteredVelocity * direction;

            if ((direction == 0) ||
                ((absolute32(errorMm) <=
                  H_TASK456_POSITION_DEADBAND_MM) &&
                 (absolute32(filteredVelocity) <=
                  H_TASK456_HOLD_SPEED_MM_S))) {
                controller->progressAnchorMm = positionMm;
                controller->breakawayStartMm = positionMm;
                controller->progressElapsedMs = 0U;
                controller->breakawayElapsedMs = 0U;
                controller->phase = TASK456_HOLD_STABLE;
            } else if (controller->phase ==
                       TASK456_HOLD_BREAKAWAY) {
                controller->breakawayElapsedMs += sampleDeltaMs;
                signedProgressMm =
                    (positionMm - controller->breakawayStartMm) *
                    direction;
                if ((signedProgressMm >=
                     H_TASK456_BREAKAWAY_EXIT_PROGRESS_MM) ||
                    (alignedSpeedMmPerSecond >=
                     H_TASK456_BREAKAWAY_EXIT_SPEED_MM_S)) {
                    controller->progressAnchorMm = positionMm;
                    controller->progressElapsedMs = 0U;
                    controller->breakawayElapsedMs = 0U;
                    controller->phase = TASK456_HOLD_TRACK;
                }
            } else if ((signedProgressMm >=
                        H_TASK456_MIN_PROGRESS_MM) ||
                       (alignedSpeedMmPerSecond >=
                        H_TASK456_BREAKAWAY_EXIT_SPEED_MM_S)) {
                controller->progressAnchorMm = positionMm;
                controller->progressElapsedMs = 0U;
                controller->phase = TASK456_HOLD_TRACK;
            } else {
                controller->progressElapsedMs += sampleDeltaMs;
                if ((controller->progressElapsedMs >=
                     H_TASK456_PROGRESS_WINDOW_MS) &&
                    (absolute32(filteredVelocity) <=
                     H_TASK456_STALL_SPEED_MM_S)) {
                    controller->breakawayStartMm = positionMm;
                    controller->breakawayElapsedMs = 0U;
                    controller->phase = TASK456_HOLD_BREAKAWAY;
                }
            }
        }

        accelerationCommand =
            H_TASK456_POSITION_KP_NUM * errorMm /
                H_TASK456_POSITION_KP_DEN -
            H_TASK456_VELOCITY_KD_NUM * filteredVelocity /
                H_TASK456_VELOCITY_KD_DEN;
        accelerationCommand = clamp32(accelerationCommand,
            -H_TASK456_ACCEL_LIMIT_MM_S2,
            H_TASK456_ACCEL_LIMIT_MM_S2);
        /* Positive lift accelerates the ball toward negative position. */
        commandUm =
            -accelerationCommand * H_TASK456_ACCEL_TO_LIFT_UM;

        if (controller->phase == TASK456_HOLD_BREAKAWAY) {
            int32_t breakawayUm = H_TASK456_BREAKAWAY_MIN_UM +
                (int32_t)(controller->breakawayElapsedMs *
                          H_TASK456_BREAKAWAY_RAMP_UM_PER_100MS / 100U);
            if (breakawayUm > H_TASK456_BREAKAWAY_MAX_UM) {
                breakawayUm = H_TASK456_BREAKAWAY_MAX_UM;
            }
            if ((direction > 0) && (commandUm > -breakawayUm)) {
                commandUm = -breakawayUm;
            } else if ((direction < 0) &&
                       (commandUm < breakawayUm)) {
                commandUm = breakawayUm;
            }
        } else if ((absolute32(errorMm) <=
                    H_TASK456_POSITION_DEADBAND_MM) &&
                   (absolute32(filteredVelocity) <=
                    H_TASK456_HOLD_SPEED_MM_S)) {
            commandUm = 0;
            controller->phase = TASK456_HOLD_STABLE;
        } else {
            controller->phase = TASK456_HOLD_TRACK;
        }
    }

    commandUm = clamp32(commandUm,
        -H_TASK456_COMMAND_MAX_UM, H_TASK456_COMMAND_MAX_UM);
    outputDeltaUm = clamp32(
        commandUm - controller->lastOutputUm,
        -H_TASK456_COMMAND_SLEW_UM_PER_TICK,
        H_TASK456_COMMAND_SLEW_UM_PER_TICK);
    controller->lastOutputUm += outputDeltaUm;
    return controller->lastOutputUm;
}

static const char *holdControllerPhaseText(
    const Task456HoldController *controller)
{
    switch (controller->phase) {
    case TASK456_HOLD_TRACK: return "HOLD TRACK";
    case TASK456_HOLD_BREAKAWAY: return "HOLD BREAK";
    case TASK456_HOLD_STABLE: return "HOLD";
    case TASK456_HOLD_GUARD: return "HOLD GUARD";
    default: return "HOLD ERR";
    }
}
static void updateVision(void)
{
    VisionBallObservation observation;
    uint8_t isNewFrame;

    g_visionFresh = 0U;
    g_visionValid = 0U;
    g_visionCalibrated = 0U;
    g_visionUsable = 0U;
    g_newMeasurement = 0U;
    if (!VisionLink_getObservation(&observation)) return;

    g_visionFresh = 1U;
    g_visionValid = observation.valid;
    g_visionCalibrated =
        ((observation.flags & VISION_FLAG_POSITION_CALIBRATED) != 0U) ?
        1U : 0U;
    if (observation.valid != 0U) {
        g_positionMm = observation.positionMm;
        g_velocityMmPerSecond = observation.velocityMmPerSecond;
    }
    if ((g_visionValid == 0U) || (g_visionCalibrated == 0U)) return;

    /* Match task 3: a valid calibrated frame is control-usable. Do not add a
     * second confidence gate here, because the camera/protocol already owns
     * target validation and the extra gate falsely blocked tasks 4/5/6. */
    g_visionUsable = 1U;
    isNewFrame =
        (observation.receivedAtMs != g_lastVisionReceivedAtMs) ? 1U : 0U;
    if (isNewFrame != 0U) {
        if (g_lastVisionReceivedAtMs == UINT32_MAX) {
            g_controlSampleDeltaMs = H_TASK456_CONTROL_PERIOD_MS;
        } else {
            g_controlSampleDeltaMs =
                observation.receivedAtMs - g_lastVisionReceivedAtMs;
            if ((g_controlSampleDeltaMs == 0U) ||
                (g_controlSampleDeltaMs > VISION_PROTOCOL_TIMEOUT_MS)) {
                g_controlSampleDeltaMs = H_TASK456_CONTROL_PERIOD_MS;
            }
        }
        g_lastVisionReceivedAtMs = observation.receivedAtMs;
        g_newMeasurement = 1U;
    }
}
static int32_t selectedImuAngle(const Jy61pObservation *observation)
{
#if H_TASK456_IMU_ANGLE_AXIS == H_TASK456_IMU_AXIS_ROLL
    return observation->rollMilliDegrees;
#else
    return observation->pitchMilliDegrees;
#endif
}

static void updateImu(void)
{
    Jy61pObservation observation;

    g_imuUsable = 0U;
    if (!Jy61pLink_getObservation(&observation)) return;
    if (observation.angleValid == 0U) return;
    g_imuAngleMilliDegrees =
        H_TASK456_IMU_ANGLE_SIGN * selectedImuAngle(&observation);
    g_imuUsable = 1U;
}

static int32_t angleFeedforwardUm(void)
{
    int32_t deltaMilliDegrees;
    int32_t feedforwardUm;

    if ((g_imuUsable == 0U) || (g_haveImuZero == 0U)) return 0;
    deltaMilliDegrees = clamp32(
        g_imuAngleMilliDegrees - g_imuZeroMilliDegrees,
        -H_TASK456_ANGLE_INPUT_LIMIT_MDEG,
        H_TASK456_ANGLE_INPUT_LIMIT_MDEG);
    /* Raise/lower the actuator opposite to the vehicle pitch disturbance. */
    feedforwardUm = -deltaMilliDegrees *
        H_TASK456_ANGLE_TO_LIFT_NUM / H_TASK456_ANGLE_TO_LIFT_DEN;
    return clamp32(feedforwardUm,
        -H_TASK456_ANGLE_FF_MAX_UM,
        H_TASK456_ANGLE_FF_MAX_UM);
}

static int32_t task4StartAccelerationFeedforwardUm(uint32_t nowMs)
{
    uint32_t elapsedMs;
    uint32_t decayMs;

    if ((g_taskNumber != TASK456_NUMBER_4) ||
        (g_state != TASK456_RUNNING)) return 0;
    elapsedMs = nowMs - g_runStartMs;
    if (elapsedMs >= H_TASK4_ACCEL_RAMP_MS) return 0;
    if (elapsedMs <= H_TASK4_START_FF_HOLD_MS) {
        return -H_TASK4_START_DOWN_FF_UM;
    }
    decayMs = H_TASK4_ACCEL_RAMP_MS - H_TASK4_START_FF_HOLD_MS;
    /* Negative lift lowers the right beam end as requested for launch. */
    return -(int32_t)(H_TASK4_START_DOWN_FF_UM *
        (int32_t)(H_TASK4_ACCEL_RAMP_MS - elapsedMs) /
        (int32_t)decayMs);
}
void Task456Balance_init(void)
{
    g_state = TASK456_IDLE;
    g_taskNumber = TASK456_NUMBER_4;
    g_positionMm = 0;
    g_velocityMmPerSecond = 0;
    g_targetMm = 0;
    g_imuZeroMilliDegrees = 0;
    g_imuAngleMilliDegrees = 0;
    g_lastVisionReceivedAtMs = UINT32_MAX;
    g_controlSampleDeltaMs = H_TASK456_CONTROL_PERIOD_MS;
    g_visionLostStartMs = 0U;
    g_runStartMs = 0U;
    g_visionFresh = 0U;
    g_visionValid = 0U;
    g_visionCalibrated = 0U;
    g_visionUsable = 0U;
    g_newMeasurement = 0U;
    g_imuUsable = 0U;
    g_haveImuZero = 0U;
    BeamMotionController_init(&g_beam);
    holdControllerReset(&g_hold, 0);
}

void Task456Balance_arm(uint8_t taskNumber, uint32_t nowMs)
{
    Task456Balance_stop();
    g_taskNumber = taskNumber;
    g_targetMm = H_TASK456_CENTER_TARGET_MM;
    g_lastVisionReceivedAtMs = UINT32_MAX;
    g_controlSampleDeltaMs = H_TASK456_CONTROL_PERIOD_MS;
    g_visionLostStartMs = 0U;
    g_runStartMs = 0U;
    g_haveImuZero = 0U;
    holdControllerReset(&g_hold, g_positionMm);
    BeamMotionController_startHoming(&g_beam, nowMs);
    g_state = TASK456_HOMING;
}

void Task456Balance_start(uint32_t nowMs)
{
    if (!Task456Balance_readyToStart()) return;
    if (g_taskNumber == TASK456_NUMBER_6) {
        g_targetMm = clamp32(g_positionMm,
            -H_TASK456_TARGET_LIMIT_MM, H_TASK456_TARGET_LIMIT_MM);
    } else {
        g_targetMm = H_TASK456_CENTER_TARGET_MM;
    }
    holdControllerReset(&g_hold, g_positionMm);
    g_visionLostStartMs = 0U;
    g_runStartMs = nowMs;
    g_state = TASK456_RUNNING;
}

void Task456Balance_tick10ms(uint32_t nowMs)
{
    int32_t liftUm;

    BeamMotionController_tick10ms(&g_beam, nowMs);
    updateVision();
    updateImu();
    if ((g_imuUsable != 0U) && (g_haveImuZero == 0U)) {
        g_imuZeroMilliDegrees = g_imuAngleMilliDegrees;
        g_haveImuZero = 1U;
    }

    if (BeamMotionController_faulted(&g_beam)) {
        g_state = TASK456_BEAM_FAULT;
        return;
    }
    if (g_state == TASK456_HOMING) {
        if (BeamMotionController_ready(&g_beam)) g_state = TASK456_ARMED;
        return;
    }
    if (g_state == TASK456_ARMED) {
        BeamMotionController_setLiftUm(&g_beam, 0);
        if (g_imuUsable != 0U) {
            g_imuZeroMilliDegrees = g_imuAngleMilliDegrees;
            g_haveImuZero = 1U;
        }
        if ((g_taskNumber == TASK456_NUMBER_6) &&
            (g_visionUsable != 0U) &&
            (absolute32(g_velocityMmPerSecond) <=
             H_TASK456_SETTLE_VELOCITY_MM_S) &&
            (absolute32(g_positionMm) <= H_TASK456_TARGET_LIMIT_MM)) {
            g_targetMm = g_positionMm;
        }
        return;
    }
    if (g_state != TASK456_RUNNING) return;

    if (g_visionUsable == 0U) {
        BeamMotionController_setLiftUm(&g_beam, 0);
        if (g_visionLostStartMs == 0U) g_visionLostStartMs = nowMs;
        if ((uint32_t)(nowMs - g_visionLostStartMs) >=
            H_TASK456_VISION_ABORT_MS) {
            g_state = TASK456_VISION_FAULT;
        }
        return;
    }
    g_visionLostStartMs = 0U;
    liftUm = holdControllerStep(&g_hold, g_targetMm,
        g_positionMm, g_velocityMmPerSecond,
        g_newMeasurement, g_controlSampleDeltaMs);
    liftUm += angleFeedforwardUm();
    liftUm += task4StartAccelerationFeedforwardUm(nowMs);
    liftUm = clamp32(liftUm,
        -H_TASK456_COMMAND_MAX_UM, H_TASK456_COMMAND_MAX_UM);
    BeamMotionController_setLiftUm(&g_beam, liftUm);
}

void Task456Balance_stop(void)
{
    BeamMotionController_stop(&g_beam);
    g_runStartMs = 0U;
    g_state = TASK456_IDLE;
}

bool Task456Balance_readyToStart(void)
{
    if ((g_state != TASK456_ARMED) || (g_visionUsable == 0U) ||
        (absolute32(g_velocityMmPerSecond) >
         H_TASK456_SETTLE_VELOCITY_MM_S)) return false;
    if (g_taskNumber == TASK456_NUMBER_6) {
        return absolute32(g_targetMm) <= H_TASK456_TARGET_LIMIT_MM;
    }
    return absolute32(g_positionMm - H_TASK456_CENTER_TARGET_MM) <=
        H_TASK456_CENTER_START_WINDOW_MM;
}

bool Task456Balance_faulted(void)
{
    return (g_state == TASK456_VISION_FAULT) ||
        (g_state == TASK456_BEAM_FAULT);
}

int32_t Task456Balance_ballPositionMm(void)
{
    return g_positionMm;
}

int32_t Task456Balance_ballVelocityMmPerSecond(void)
{
    return g_velocityMmPerSecond;
}

int32_t Task456Balance_targetMm(void)
{
    return g_targetMm;
}

int32_t Task456Balance_liftUm(void)
{
    return BeamMotionController_liftUm(&g_beam);
}

const char *Task456Balance_stateText(void)
{
    if (g_state == TASK456_IDLE) return "T456 IDLE";
    if (g_state == TASK456_HOMING) {
        return BeamMotionController_stateText(&g_beam);
    }
    if (g_state == TASK456_VISION_FAULT) return "VISION ERR";
    if (g_state == TASK456_BEAM_FAULT) return "BEAM ERR";
    if (g_visionFresh == 0U) return "UART LOST";
    if (g_visionValid == 0U) return "NO BALL";
    if (g_visionCalibrated == 0U) return "UNCAL";
    if (g_visionUsable == 0U) return "VISION WAIT";
    if (g_state == TASK456_ARMED) {
        if (absolute32(g_velocityMmPerSecond) >
            H_TASK456_SETTLE_VELOCITY_MM_S) return "BALL SPD";
        if ((g_taskNumber != TASK456_NUMBER_6) &&
            (absolute32(g_positionMm) >
             H_TASK456_CENTER_START_WINDOW_MM)) return "CENTER";
        if (absolute32(g_targetMm) >
            H_TASK456_TARGET_LIMIT_MM) return "TGT RANGE";
        return (g_haveImuZero != 0U) ? "READY" : "READY NOIMU";
    }
    if (g_imuUsable == 0U) return "RUN NOIMU";
    return holdControllerPhaseText(&g_hold);
}