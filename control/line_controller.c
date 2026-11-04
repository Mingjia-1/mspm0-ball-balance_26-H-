/** @file line_controller.c @brief 200 Hz centroid PD for the curved H track. */
#include "control/line_controller.h"
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

static void setRecoveryOutput(LineController *controller,
    LineControllerOutput *output, int32_t error,
    int32_t leftTarget, int32_t rightTarget, int32_t steering)
{
    controller->filteredError = error;
    controller->previousError = error;
    controller->steering = steering;
    output->leftTarget = leftTarget;
    output->rightTarget = rightTarget;
    output->error = error;
    output->steering = steering;
}

void LineController_reset(LineController *controller)
{
    controller->filteredError = 0;
    controller->previousError = 0;
    controller->steering = 0;
    controller->lostTicks = 0U;
}

LineControllerOutput LineController_step(LineController *controller,
    LineSensorReading sensor, int32_t requestedBaseSpeed)
{
    LineControllerOutput output = {0, 0, controller->filteredError, 0, 0U, 0U};
    int32_t error;
    int32_t magnitude;
    int32_t base;
    int32_t bendEntrySpeed;
    int32_t curveSpeed;
    int32_t edgeSpeed;
    int32_t derivative;
    int32_t proportional;
    int32_t derivativeTerm;
    int32_t desiredSteering;
    int32_t steeringDelta;

    if (sensor.visible == 0U) {
        controller->lostTicks++;
        output.lineLost = 1U;
        if (controller->lostTicks > H_LOST_LINE_TIMEOUT_TICKS) {
            output.fault = 1U;
            return output;
        }

        /*
         * Continue the most recent turn for short gaps in the tape.  Both
         * wheels remain forward, so recovery cannot become an in-place spin.
         */
        if (controller->filteredError < 0) {
            output.leftTarget = 5L;
            output.rightTarget = 40L;
            output.steering = -H_LINE_MAX_STEERING;
        } else {
            output.leftTarget = 40L;
            output.rightTarget = 5L;
            output.steering = H_LINE_MAX_STEERING;
        }
        return output;
    }

    controller->lostTicks = 0U;
    error = sensor.error - H_LINE_TARGET_ERROR;
    if (absolute32(error) <= H_CENTER_DEADBAND) error = 0L;
    magnitude = absolute32(error);

    /*
     * D7/D8 and D1/D2 are immediate recovery zones.  The raw mask bypasses
     * filtering and slew limiting, so an outer sensor changes the requested
     * wheel speeds on this 5 ms tick.  The inner wheel is never stopped.
     */
    if (((sensor.activeMask & 0x80U) != 0U) &&
        ((sensor.activeMask & 0x18U) == 0U)) {
        setRecoveryOutput(controller, &output, error,
            44L, 5L, H_LINE_MAX_STEERING);
        return output;
    }
    if (((sensor.activeMask & 0x40U) != 0U) &&
        ((sensor.activeMask & 0x18U) == 0U)) {
        setRecoveryOutput(controller, &output, error,
            40L, 7L, 28L);
        return output;
    }
    if (((sensor.activeMask & 0x01U) != 0U) &&
        ((sensor.activeMask & 0x18U) == 0U)) {
        setRecoveryOutput(controller, &output, error,
            5L, 44L, -H_LINE_MAX_STEERING);
        return output;
    }
    if (((sensor.activeMask & 0x02U) != 0U) &&
        ((sensor.activeMask & 0x18U) == 0U)) {
        setRecoveryOutput(controller, &output, error,
            7L, 40L, -28L);
        return output;
    }

    /*
     * Large displacement and a side crossing must be acted on immediately.
     * Near centre, a light 3:1 filter suppresses one-sample GPIO chatter.
     */
    if ((magnitude >= H_CURVE_ERROR) ||
        ((error > 0) && (controller->filteredError < 0)) ||
        ((error < 0) && (controller->filteredError > 0))) {
        controller->filteredError = error;
    } else {
        controller->filteredError =
            (controller->filteredError + 3L * error) / 4L;
    }

    derivative = controller->filteredError - controller->previousError;
    controller->previousError = controller->filteredError;
    proportional =
        (H_LINE_KP_NUMERATOR * controller->filteredError) /
        H_LINE_KP_DIVISOR;
    derivativeTerm = clamp32(
        (H_LINE_KD_NUMERATOR * derivative) / H_LINE_KD_DIVISOR,
        -H_LINE_D_MAX, H_LINE_D_MAX);
    desiredSteering = clamp32(proportional + derivativeTerm,
        -H_LINE_MAX_STEERING, H_LINE_MAX_STEERING);

    steeringDelta = clamp32(desiredSteering - controller->steering,
        -H_LINE_STEERING_SLEW, H_LINE_STEERING_SLEW);
    controller->steering += steeringDelta;

    if (requestedBaseSpeed == H_TASK2_BASE_SPEED) {
        bendEntrySpeed = H_TASK2_BEND_ENTRY_SPEED;
        curveSpeed = H_TASK2_CURVE_SPEED;
        edgeSpeed = H_TASK2_EDGE_SPEED;
    } else {
        bendEntrySpeed = H_BALANCE_BEND_ENTRY_SPEED;
        curveSpeed = H_BALANCE_CURVE_SPEED;
        edgeSpeed = H_BALANCE_EDGE_SPEED;
    }
    base = requestedBaseSpeed;
    if (magnitude >= H_EDGE_ERROR) base = edgeSpeed;
    else if (magnitude >= H_CURVE_ERROR) base = curveSpeed;
    else if (magnitude >= H_BEND_ENTRY_ERROR) base = bendEntrySpeed;

    output.leftTarget = clamp32(base + controller->steering,
        H_MIN_WHEEL_SPEED, H_MAX_WHEEL_SPEED);
    output.rightTarget = clamp32(base - controller->steering,
        H_MIN_WHEEL_SPEED, H_MAX_WHEEL_SPEED);
    output.error = error;
    output.steering = controller->steering;
    return output;
}
