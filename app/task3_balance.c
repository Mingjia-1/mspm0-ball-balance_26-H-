/**
 * @file task3_balance.c
 * @brief Vision trajectory sequence: O -> +50 waypoint -> -50 mm hold.
 */
#include "app/task3_balance.h"

#include "app/vision_link.h"
#include "config/control_config.h"
#include "control/ball_controller.h"
#include "control/beam_motion_controller.h"

static Task3BalanceState g_state;
static BeamMotionController g_beam;
static BallController g_ball;
static uint32_t g_stableMs;
static uint32_t g_visionLostStartMs;
static uint32_t g_lowConfidenceMs;
static uint32_t g_lastVisionReceivedAtMs;
static uint32_t g_controlSampleDeltaMs;
static int32_t g_positionMm;
static int32_t g_velocityMmPerSecond;
static uint8_t g_visionFresh;
static uint8_t g_visionValid;
static uint8_t g_visionCalibrated;
static uint8_t g_visionConfidence;
static uint8_t g_controlSampleUsable;
static uint8_t g_confidenceAccepted;
static uint8_t g_visionUsable;

static int32_t absolute32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static void updateVision(void)
{
    VisionBallObservation observation;
    uint8_t isNewFrame;

    g_visionFresh = 0U;
    g_controlSampleUsable = 0U;
    g_visionUsable = 0U;
    if (!VisionLink_getObservation(&observation)) return;

    isNewFrame =
        (observation.receivedAtMs != g_lastVisionReceivedAtMs) ? 1U : 0U;
    if (isNewFrame != 0U) {
        if (g_lastVisionReceivedAtMs == UINT32_MAX) {
            g_controlSampleDeltaMs = H_TASK3_CONTROL_PERIOD_MS;
        } else {
            g_controlSampleDeltaMs =
                observation.receivedAtMs - g_lastVisionReceivedAtMs;
            if ((g_controlSampleDeltaMs == 0U) ||
                (g_controlSampleDeltaMs > VISION_PROTOCOL_TIMEOUT_MS)) {
                g_controlSampleDeltaMs = H_TASK3_CONTROL_PERIOD_MS;
            }
        }
        g_lastVisionReceivedAtMs = observation.receivedAtMs;
    }

    g_visionFresh = 1U;
    g_visionValid = observation.valid;
    g_visionCalibrated =
        ((observation.flags & VISION_FLAG_POSITION_CALIBRATED) != 0U) ?
        1U : 0U;
    g_visionConfidence = observation.confidence;
    if (observation.valid != 0U) {
        g_positionMm = observation.positionMm;
        g_velocityMmPerSecond = observation.velocityMmPerSecond;
    } else {
        g_positionMm = 0;
        g_velocityMmPerSecond = 0;
    }
    /* A cached frame keeps the link healthy but must not advance control
     * filters, friction confirmation or stability timers more than once. */
    g_controlSampleUsable =
        ((g_visionValid != 0U) &&
         (g_visionCalibrated != 0U) && (isNewFrame != 0U)) ? 1U : 0U;
    if ((g_visionValid == 0U) || (g_visionCalibrated == 0U)) {
        g_confidenceAccepted = 0U;
        g_lowConfidenceMs = 0U;
    } else if (g_controlSampleUsable != 0U) {
        g_confidenceAccepted = 1U;
        g_lowConfidenceMs = 0U;
    } else if (g_confidenceAccepted != 0U) {
        if (g_lowConfidenceMs < H_TASK3_LOW_CONFIDENCE_HOLD_MS) {
            g_lowConfidenceMs += H_TASK3_CONTROL_PERIOD_MS;
        }
        if (g_lowConfidenceMs >= H_TASK3_LOW_CONFIDENCE_HOLD_MS) {
            g_confidenceAccepted = 0U;
            g_lowConfidenceMs = 0U;
        }
    }
    g_visionUsable = g_confidenceAccepted;
}

static bool stableAt(
    int32_t targetMm, int32_t windowMm, uint32_t requiredMs)
{
    if ((absolute32(g_positionMm - targetMm) <= windowMm) &&
        (absolute32(g_velocityMmPerSecond) <=
         H_TASK3_SETTLE_VELOCITY_MM_S) &&
        BallController_settled(&g_ball)) {
        if (g_stableMs < requiredMs) {
            g_stableMs += g_controlSampleDeltaMs;
            if (g_stableMs > requiredMs) g_stableMs = requiredMs;
        }
    } else {
        g_stableMs = 0U;
    }
    return g_stableMs >= requiredMs;
}

static const char *armedStateText(void)
{
    const VisionProtocolStats *stats = VisionLink_getProtocolStats();

    if (g_visionFresh == 0U) {
        if ((stats != 0) && (stats->acceptedFrames != 0U)) {
            return "UART LOST";
        }
        if (VisionLink_receivedByteCount() == 0U) {
            return "UART NONE";
        }
        if ((stats != 0) && (stats->crcErrors != 0U)) {
            return "UART CRC";
        }
        if ((stats != 0) &&
            ((stats->formatErrors != 0U) ||
             (stats->semanticErrors != 0U))) {
            return "UART FMT";
        }
        return "UART DATA";
    }
    if (g_visionValid == 0U) return "NO BALL";
    if (g_visionCalibrated == 0U) return "UNCAL";
    if (g_confidenceAccepted == 0U) {
        return "LOW CONF";
    }
    if (absolute32(g_positionMm) > H_TASK3_START_WINDOW_MM) {
        return "CENTER";
    }
    if (absolute32(g_velocityMmPerSecond) >
        H_TASK3_SETTLE_VELOCITY_MM_S) {
        return "BALL SPD";
    }
    return "READY";
}

void Task3Balance_init(void)
{
    g_state = TASK3_BALANCE_IDLE;
    g_stableMs = 0U;
    g_visionLostStartMs = 0U;
    g_lowConfidenceMs = 0U;
    g_lastVisionReceivedAtMs = UINT32_MAX;
    g_controlSampleDeltaMs = H_TASK3_CONTROL_PERIOD_MS;
    g_positionMm = 0;
    g_velocityMmPerSecond = 0;
    g_visionFresh = 0U;
    g_visionValid = 0U;
    g_visionCalibrated = 0U;
    g_visionConfidence = 0U;
    g_controlSampleUsable = 0U;
    g_confidenceAccepted = 0U;
    g_visionUsable = 0U;
    BeamMotionController_init(&g_beam);
    BallController_reset(&g_ball);
}

void Task3Balance_arm(uint32_t nowMs)
{
    BallController_reset(&g_ball);
    g_stableMs = 0U;
    g_visionLostStartMs = 0U;
    g_lowConfidenceMs = 0U;
    g_lastVisionReceivedAtMs = UINT32_MAX;
    g_controlSampleDeltaMs = H_TASK3_CONTROL_PERIOD_MS;
    g_confidenceAccepted = 0U;
    BeamMotionController_startHoming(&g_beam, nowMs);
    g_state = TASK3_BALANCE_HOMING;
}

void Task3Balance_start(uint32_t nowMs)
{
    if (!Task3Balance_readyToStart()) return;
    BallController_reset(&g_ball);
    BallController_setMove(
        &g_ball, g_positionMm, H_TASK3_POSITIVE_TARGET_MM);
    (void)nowMs;
    g_stableMs = 0U;
    g_visionLostStartMs = 0U;
    g_state = TASK3_BALANCE_TO_POSITIVE;
}

void Task3Balance_tick10ms(uint32_t nowMs)
{
    int32_t liftUm;

    BeamMotionController_tick10ms(&g_beam, nowMs);
    updateVision();

    if (BeamMotionController_faulted(&g_beam)) {
        g_state = TASK3_BALANCE_HOME_FAULT;
        return;
    }

    if (g_state == TASK3_BALANCE_HOMING) {
        if (BeamMotionController_ready(&g_beam)) {
            g_state = TASK3_BALANCE_ARMED;
        }
        return;
    }

    if ((g_state != TASK3_BALANCE_TO_POSITIVE) &&
        (g_state != TASK3_BALANCE_TO_NEGATIVE) &&
        (g_state != TASK3_BALANCE_COMPLETE)) return;

    if (g_visionUsable == 0U) {
        BeamMotionController_setLiftUm(&g_beam, 0);
        g_stableMs = 0U;
        if (g_visionLostStartMs == 0U) g_visionLostStartMs = nowMs;
        if ((uint32_t)(nowMs - g_visionLostStartMs) >=
            H_TASK3_VISION_ABORT_MS) {
            g_state = TASK3_BALANCE_VISION_FAULT;
        }
        return;
    }
    g_visionLostStartMs = 0U;
    liftUm = BallController_step(
        &g_ball, g_positionMm, g_velocityMmPerSecond,
        g_controlSampleUsable, g_controlSampleDeltaMs,
        BeamMotionController_liftUm(&g_beam));
    BeamMotionController_setLiftUm(&g_beam, liftUm);

    /* Time and actuator motion run at 10 ms. Only a genuinely new camera
     * sample may update stability or advance the target sequence. */
    if (g_controlSampleUsable == 0U) return;

    if (g_state == TASK3_BALANCE_TO_POSITIVE) {
        /* Entering the scored +5 cm window is the visual waypoint. The
         * next trajectory immediately commands negative velocity, so braking
         * and reversal are continuous instead of waiting for the ball to stop. */
        if (BallController_waypointReached(&g_ball)) {
            BallController_setMove(
                &g_ball, g_positionMm, H_TASK3_NEGATIVE_TARGET_MM);
            g_stableMs = 0U;
            g_state = TASK3_BALANCE_TO_NEGATIVE;
        }
    } else if ((g_state == TASK3_BALANCE_TO_NEGATIVE) &&
               stableAt(H_TASK3_NEGATIVE_TARGET_MM,
                   H_TASK3_FINAL_WINDOW_MM,
                   H_TASK3_FINAL_STABLE_MS)) {
        /* Completion is visual: the controller may retain a small lift to
         * reject pipe bias or a local depression. */
        g_state = TASK3_BALANCE_COMPLETE;
    }
}

void Task3Balance_stop(void)
{
    BeamMotionController_stop(&g_beam);
    g_state = TASK3_BALANCE_IDLE;
}

bool Task3Balance_readyToStart(void)
{
    return (g_state == TASK3_BALANCE_ARMED) &&
        (g_visionUsable != 0U) &&
        (absolute32(g_positionMm) <= H_TASK3_START_WINDOW_MM) &&
        (absolute32(g_velocityMmPerSecond) <=
         H_TASK3_SETTLE_VELOCITY_MM_S);
}

bool Task3Balance_complete(void)
{
    return g_state == TASK3_BALANCE_COMPLETE;
}

bool Task3Balance_faulted(void)
{
    return (g_state == TASK3_BALANCE_VISION_FAULT) ||
        (g_state == TASK3_BALANCE_HOME_FAULT) ||
        BallController_stuck(&g_ball);
}

int32_t Task3Balance_ballPositionMm(void)
{
    return g_positionMm;
}

int32_t Task3Balance_ballVelocityMmPerSecond(void)
{
    return g_velocityMmPerSecond;
}

int32_t Task3Balance_liftUm(void)
{
    return BeamMotionController_liftUm(&g_beam);
}

const char *Task3Balance_stateText(void)
{
    switch (g_state) {
    case TASK3_BALANCE_IDLE: return "T3 IDLE";
    case TASK3_BALANCE_HOMING:
        return BeamMotionController_stateText(&g_beam);
    case TASK3_BALANCE_ARMED: return armedStateText();
    case TASK3_BALANCE_TO_POSITIVE: return BallController_phaseText(&g_ball);
    case TASK3_BALANCE_TO_NEGATIVE: return BallController_phaseText(&g_ball);
    case TASK3_BALANCE_COMPLETE: return
        BallController_settled(&g_ball) ? "DONE" : "RECOVER";
    case TASK3_BALANCE_VISION_FAULT: return "VISION ERR";
    case TASK3_BALANCE_HOME_FAULT: return "HOME ERR";
    default: return "T3 ERR";
    }
}
