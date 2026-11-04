/** @file wheel_speed_controller.c @brief 40 Hz dual PI with speed feed-forward. */
#include "control/wheel_speed_controller.h"
#include "config/control_config.h"

static int32_t absolute32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t clamp32(int32_t value, int32_t low, int32_t high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static int16_t updatePi(int32_t target, int32_t measured, int32_t *integral)
{
    int32_t error;
    int32_t candidate;
    int32_t feedForward;
    int32_t unsaturated;
    int32_t output;

    if (target <= 0) {
        *integral = 0;
        return 0;
    }

    error = target - measured;
    candidate = clamp32(*integral + error,
        -H_SPEED_INTEGRAL_LIMIT, H_SPEED_INTEGRAL_LIMIT);
    feedForward = H_SPEED_FF_BASE_PERCENT +
        (H_SPEED_FF_NUMERATOR * target) / H_SPEED_FF_DIVISOR;
    unsaturated = feedForward +
        (H_SPEED_KP_NUMERATOR * error) / H_SPEED_KP_DIVISOR +
        candidate / H_SPEED_KI_DIVISOR;
    output = clamp32(unsaturated, H_SPEED_MIN_OUTPUT_PERCENT,
        H_SPEED_MAX_OUTPUT_PERCENT);

    /* Conditional integration prevents wind-up at either PWM limit. */
    if ((unsaturated == output) ||
        ((unsaturated > output) && (error < 0)) ||
        ((unsaturated < output) && (error > 0))) {
        *integral = candidate;
    }
    return (int16_t)output;
}

static int32_t normalizedSpeedCount(int32_t pulseSum)
{
    const int32_t windowMs =
        (int32_t)H_SPEED_WINDOW_TICKS * (int32_t)H_CONTROL_PERIOD_MS;
    int32_t referenceEdges;

    referenceEdges =
        (pulseSum * H_SPEED_REFERENCE_WINDOW_MS + windowMs / 2L) / windowMs;
    return (referenceEdges + H_ENCODER_EDGES_PER_SPEED_COUNT / 2L) /
        H_ENCODER_EDGES_PER_SPEED_COUNT;
}

void WheelSpeedController_reset(WheelSpeedController *controller)
{
    controller->integral = 0;
    controller->rightIntegral = 0;
    controller->leftPulseSum = 0;
    controller->rightPulseSum = 0;
    controller->leftTarget = 0;
    controller->rightTarget = 0;
    controller->leftActual = 0;
    controller->rightActual = 0;
    controller->leftDuty = 0;
    controller->rightDuty = 0;
    controller->windowTicks = 0U;
}

void WheelSpeedController_stop(WheelSpeedController *controller)
{
    WheelSpeedController_reset(controller);
    DriveBase_stop();
}

void WheelSpeedController_step(WheelSpeedController *controller,
    DriveBaseEncoderCounts delta, int32_t leftTarget, int32_t rightTarget)
{
    controller->leftTarget = clamp32(leftTarget, 0L, H_SPEED_TARGET_MAX);
    controller->rightTarget = clamp32(rightTarget, 0L, H_SPEED_TARGET_MAX);
    controller->leftPulseSum += absolute32(delta.left);
    controller->rightPulseSum += absolute32(delta.right);
    controller->windowTicks++;

    if ((controller->leftTarget == 0) && (controller->rightTarget == 0)) {
        WheelSpeedController_stop(controller);
        return;
    }
    if (controller->windowTicks < H_SPEED_WINDOW_TICKS) return;

    controller->leftActual =
        normalizedSpeedCount(controller->leftPulseSum);
    controller->rightActual =
        normalizedSpeedCount(controller->rightPulseSum);
    controller->leftDuty = updatePi(controller->leftTarget,
        controller->leftActual, &controller->integral);
    controller->rightDuty = updatePi(controller->rightTarget,
        controller->rightActual, &controller->rightIntegral);
    DriveBase_setDuty(controller->leftDuty, controller->rightDuty);
    controller->leftPulseSum = 0;
    controller->rightPulseSum = 0;
    controller->windowTicks = 0U;
}
