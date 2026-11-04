/**
 * @file ball_controller.c
 * @brief Vision-based velocity trajectory and adaptive friction controller.
 *
 * The controller never assumes that logical actuator zero means the beam is
 * physically level. Position and velocity close the loop. A bounded probe
 * overcomes local static friction; once motion starts, the probe is removed
 * and its threshold is learned separately for each travel direction.
 */
#include "control/ball_controller.h"

#include "config/control_config.h"

static int32_t clamp32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static int32_t absolute32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t sign32(int32_t value)
{
    return (value > 0) ? 1L : ((value < 0) ? -1L : 0L);
}

static uint32_t integerSqrt32(uint32_t value)
{
    uint32_t result = 0U;
    uint32_t bit = 1UL << 30;

    while (bit > value) bit >>= 2;
    while (bit != 0U) {
        if (value >= result + bit) {
            value -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }
    return result;
}

static int32_t learnedForDirection(
    const BallController *controller, int32_t direction)
{
    return (direction > 0) ?
        controller->learnedPositiveUm : controller->learnedNegativeUm;
}

static void learnBreakaway(BallController *controller,
    int32_t direction, int32_t observedUm)
{
    int32_t *learned;

    observedUm = clamp32(observedUm,
        H_TASK3_LEARNED_FRICTION_MIN_UM,
        H_TASK3_LEARNED_FRICTION_MAX_UM);
    learned = (direction > 0) ?
        &controller->learnedPositiveUm : &controller->learnedNegativeUm;
    *learned = (3L * (*learned) + observedUm) / 4L;
}
void BallController_reset(BallController *controller)
{
    controller->startMm = 0;
    controller->targetMm = 0;
    controller->moveDirection = 0;
    controller->filteredVelocityMmPerSecond = 0;
    controller->referenceVelocityMmPerSecond = 0;
    controller->lastOutputUm = 0;
    controller->previousFilteredVelocityMmPerSecond = 0;
    controller->disturbanceAccelerationMmPerSecond2 = 0;
    controller->guardBoundaryMm = H_TASK3_GUARD_BOUNDARY_MM;
    controller->learnedPositiveUm = H_TASK3_LEARNED_FRICTION_INITIAL_UM;
    controller->learnedNegativeUm = H_TASK3_LEARNED_FRICTION_INITIAL_UM;
    controller->progressAnchorMm = 0;
    controller->breakawayStartMm = 0;
    controller->progressElapsedMs = 0U;
    controller->breakawayElapsedMs = 0U;
    controller->stallFrames = 0U;
    controller->probeDirection = 0;
    controller->guardDirection = 0;
    controller->waypointReached = 0U;
    controller->guardActive = 0U;
    controller->phase = BALL_CONTROL_HOLD;
}

void BallController_setMove(
    BallController *controller, int32_t startMm, int32_t targetMm)
{
    controller->startMm = startMm;
    controller->targetMm = targetMm;
    controller->moveDirection = sign32(targetMm - startMm);
    /* Preserve measured velocity, disturbance estimate and actuator command.
     * Task start calls reset first; waypoint reversal must remain continuous. */
    controller->referenceVelocityMmPerSecond = 0;
    controller->progressAnchorMm = startMm;
    controller->breakawayStartMm = startMm;
    controller->progressElapsedMs = 0U;
    controller->breakawayElapsedMs = 0U;
    controller->stallFrames = 0U;
    controller->probeDirection = (int8_t)controller->moveDirection;
    controller->waypointReached = 0U;
    controller->phase = (controller->moveDirection == 0) ?
        BALL_CONTROL_HOLD : BALL_CONTROL_TRACK;
}

void BallController_setHold(BallController *controller, int32_t targetMm)
{
    BallController_setMove(controller, targetMm, targetMm);
    controller->waypointReached = 1U;
    controller->phase = BALL_CONTROL_HOLD;
}

void BallController_setGuardBoundary(
    BallController *controller, int32_t boundaryMm)
{
    if (boundaryMm < H_TASK3_GUARD_BOUNDARY_MM) {
        boundaryMm = H_TASK3_GUARD_BOUNDARY_MM;
    }
    controller->guardBoundaryMm = boundaryMm;
}

int32_t BallController_step(BallController *controller,
    int32_t positionMm, int32_t velocityMmPerSecond,
    uint8_t newMeasurement, uint32_t sampleDeltaMs,
    int32_t actualLiftUm)
{
    int32_t errorMm = controller->targetMm - positionMm;
    int32_t remainingMm = absolute32(errorMm);
    int32_t desiredDirection = sign32(errorMm);
    int32_t filteredVelocity =
        controller->filteredVelocityMmPerSecond;
    int32_t referenceSpeed;
    int32_t profileRemainingMm;
    int32_t actuatorLagDistanceMm;
    int32_t positionTerm;
    int32_t accelerationCommand;
    int32_t accelerationLimit;
    int32_t commandUm;
    int32_t commandLimitUm;
    int32_t outputDeltaUm;
    int32_t projectedPositionMm;
    int32_t slewLimitUm = H_TASK3_COMMAND_SLEW_UM_PER_TICK;
    if (controller->phase == BALL_CONTROL_STUCK) {
        controller->referenceVelocityMmPerSecond = 0;
        controller->lastOutputUm = 0;
        return 0;
    }

    if (newMeasurement != 0U) {
        int32_t oldFilteredVelocity =
            controller->filteredVelocityMmPerSecond;
        controller->filteredVelocityMmPerSecond +=
            (velocityMmPerSecond -
             controller->filteredVelocityMmPerSecond) / 2L;
        filteredVelocity = controller->filteredVelocityMmPerSecond;
        if ((sampleDeltaMs >= 10U) && (sampleDeltaMs <= 200U) &&
            (controller->guardActive == 0U) &&
            (absolute32(filteredVelocity) >=
             H_TASK3_OBSERVER_MIN_SPEED_MM_S) &&
            (controller->phase != BALL_CONTROL_PROBE)) {
            int32_t measuredAcceleration =
                (filteredVelocity - oldFilteredVelocity) * 1000L /
                (int32_t)sampleDeltaMs;
            int32_t modelAcceleration =
                -actualLiftUm * H_TASK3_MODEL_ACCEL_NUM /
                H_TASK3_MODEL_ACCEL_DEN;
            int32_t disturbanceSample = clamp32(
                measuredAcceleration - modelAcceleration,
                -H_TASK3_OBSERVER_SAMPLE_LIMIT_MM_S2,
                H_TASK3_OBSERVER_SAMPLE_LIMIT_MM_S2);
            controller->disturbanceAccelerationMmPerSecond2 +=
                (disturbanceSample -
                 controller->disturbanceAccelerationMmPerSecond2) /
                H_TASK3_OBSERVER_FILTER_DIVISOR;
            controller->disturbanceAccelerationMmPerSecond2 = clamp32(
                controller->disturbanceAccelerationMmPerSecond2,
                -H_TASK3_OBSERVER_LIMIT_MM_S2,
                H_TASK3_OBSERVER_LIMIT_MM_S2);
        }
        controller->previousFilteredVelocityMmPerSecond =
            filteredVelocity;
        /*
         * A camera frame may jump from one side of the 10 mm waypoint
         * window to the other. Crossing the near edge therefore counts as
         * reaching it; requiring one sample inside the window can leave the
         * state machine accelerating toward an already passed target.
         */
        if (((controller->moveDirection > 0) &&
             (positionMm >=
              controller->targetMm - H_TASK3_WAYPOINT_WINDOW_MM)) ||
            ((controller->moveDirection < 0) &&
             (positionMm <=
              controller->targetMm + H_TASK3_WAYPOINT_WINDOW_MM)) ||
            (remainingMm <= H_TASK3_WAYPOINT_WINDOW_MM)) {
            controller->waypointReached = 1U;
        }

        projectedPositionMm = positionMm +
            filteredVelocity * H_TASK3_GUARD_LOOKAHEAD_MS / 1000L;
        if ((filteredVelocity >= H_TASK3_GUARD_TRIGGER_SPEED_MM_S) &&
            (projectedPositionMm >= controller->guardBoundaryMm)) {
            controller->guardActive = 1U;
            controller->guardDirection = 1;
        } else if ((filteredVelocity <=
                    -H_TASK3_GUARD_TRIGGER_SPEED_MM_S) &&
                   (projectedPositionMm <=
                    -controller->guardBoundaryMm)) {
            controller->guardActive = 1U;
            controller->guardDirection = -1;
        }

        if ((controller->guardActive != 0U) &&
            ((filteredVelocity * controller->guardDirection) <=
             -H_TASK3_GUARD_RELEASE_SPEED_MM_S)) {
            controller->guardActive = 0U;
            controller->guardDirection = 0;
        }
    }

    if (controller->guardActive != 0U) {
        /* The direct crank reacts quickly. Keep the
         * normal full guard for every other trajectory, but ramp and limit the
         * left-boundary brake while executing the +5 -> -5 move. */
        if ((controller->moveDirection < 0) &&
            (controller->guardDirection < 0)) {
            commandUm = -H_TASK3_NEGATIVE_GUARD_LIFT_UM;
            outputDeltaUm = clamp32(
                commandUm - controller->lastOutputUm,
                -H_TASK3_NEGATIVE_GUARD_SLEW_UM_PER_TICK,
                H_TASK3_NEGATIVE_GUARD_SLEW_UM_PER_TICK);
            controller->lastOutputUm += outputDeltaUm;
        } else {
            controller->lastOutputUm =
                controller->guardDirection * H_TASK3_GUARD_LIFT_UM;
        }
        controller->referenceVelocityMmPerSecond = 0;
        controller->stallFrames = 0U;
        controller->phase = BALL_CONTROL_GUARD;
        return controller->lastOutputUm;
    }

    if (desiredDirection != controller->probeDirection) {
        controller->probeDirection = (int8_t)desiredDirection;
        controller->stallFrames = 0U;
        controller->progressAnchorMm = positionMm;
        controller->breakawayStartMm = positionMm;
        controller->progressElapsedMs = 0U;
        controller->breakawayElapsedMs = 0U;
        if (controller->phase == BALL_CONTROL_PROBE) {
            controller->phase = BALL_CONTROL_TRACK;
        }
    }

    if (newMeasurement != 0U) {
        int32_t signedProgressMm;
        int32_t alignedSpeedMmPerSecond =
            filteredVelocity * desiredDirection;

        if ((desiredDirection == 0) ||
            (remainingMm <= H_TASK3_POSITION_DEADBAND_MM)) {
            controller->progressAnchorMm = positionMm;
            controller->breakawayStartMm = positionMm;
            controller->progressElapsedMs = 0U;
            controller->breakawayElapsedMs = 0U;
            if (controller->phase == BALL_CONTROL_PROBE) {
                controller->phase = BALL_CONTROL_TRACK;
            }
        } else if (controller->phase == BALL_CONTROL_PROBE) {
            controller->breakawayElapsedMs += sampleDeltaMs;
            signedProgressMm =
                (positionMm - controller->breakawayStartMm) *
                desiredDirection;
            if ((signedProgressMm >=
                 H_TASK3_BREAKAWAY_EXIT_PROGRESS_MM) ||
                (alignedSpeedMmPerSecond >=
                 H_TASK3_BREAKAWAY_EXIT_SPEED_MM_S)) {
                learnBreakaway(controller, desiredDirection,
                    absolute32(controller->lastOutputUm));
                controller->phase =
                    (controller->waypointReached != 0U) ?
                    BALL_CONTROL_CAPTURE : BALL_CONTROL_TRACK;
                controller->progressAnchorMm = positionMm;
                controller->progressElapsedMs = 0U;
                controller->breakawayElapsedMs = 0U;
            } else if (controller->breakawayElapsedMs >=
                       H_TASK3_BREAKAWAY_TIMEOUT_MS) {
                controller->referenceVelocityMmPerSecond = 0;
                controller->lastOutputUm = 0;
                controller->phase = BALL_CONTROL_STUCK;
                return 0;
            }
        } else {
            signedProgressMm =
                (positionMm - controller->progressAnchorMm) *
                desiredDirection;
            if ((signedProgressMm >= H_TASK3_MIN_PROGRESS_MM) ||
                (alignedSpeedMmPerSecond >=
                 H_TASK3_BREAKAWAY_EXIT_SPEED_MM_S)) {
                controller->progressAnchorMm = positionMm;
                controller->progressElapsedMs = 0U;
            } else {
                controller->progressElapsedMs += sampleDeltaMs;
                if ((controller->progressElapsedMs >=
                     H_TASK3_PROGRESS_WINDOW_MS) &&
                    (absolute32(filteredVelocity) <=
                     H_TASK3_STALL_SPEED_MM_S)) {
                    controller->breakawayStartMm = positionMm;
                    controller->breakawayElapsedMs = 0U;
                    controller->phase = BALL_CONTROL_PROBE;
                }
            }
        }
    }
    actuatorLagDistanceMm =
        absolute32(filteredVelocity) * H_TASK3_ACTUATOR_LAG_MS / 1000L;
    profileRemainingMm = remainingMm - actuatorLagDistanceMm;
    if (profileRemainingMm < 0) profileRemainingMm = 0;
    referenceSpeed = (int32_t)integerSqrt32((uint32_t)(
        2L * H_TASK3_PROFILE_BRAKE_ACCEL_MM_S2 *
        profileRemainingMm));
    if (referenceSpeed > H_TASK3_PROFILE_MAX_SPEED_MM_S) {
        referenceSpeed = H_TASK3_PROFILE_MAX_SPEED_MM_S;
    }
    if ((controller->waypointReached != 0U) &&
        (referenceSpeed > H_TASK3_CAPTURE_MAX_SPEED_MM_S)) {
        /* Once the camera has captured this target, disturbances may only
         * trigger a low-speed recovery, never another full-speed run. */
        referenceSpeed = H_TASK3_CAPTURE_MAX_SPEED_MM_S;
    }
    controller->referenceVelocityMmPerSecond =
        desiredDirection * referenceSpeed;
    /*
     * Visual PD with separate term limits. Long travel must not let P wind
     * up to a high launch speed; D starts unloading the actuator as soon as the
     * camera reports motion. Static friction is handled only by PROBE below.
     */
    positionTerm =
        (H_TASK3_POSITION_KP_NUM * errorMm) /
            H_TASK3_POSITION_KP_DEN;
    positionTerm = clamp32(positionTerm,
        -H_TASK3_POSITION_P_LIMIT_MM_S2,
        H_TASK3_POSITION_P_LIMIT_MM_S2);
    accelerationCommand = positionTerm +
        (H_TASK3_VELOCITY_KD_NUM *
         (controller->referenceVelocityMmPerSecond - filteredVelocity)) /
            H_TASK3_VELOCITY_KD_DEN;

    /*
     * During the long +5 -> -5 traverse, D may unload the left-driving
     * tilt to level but must not create a right-driving acceleration before
     * the ball enters the final braking window. This preserves inertia
     * without allowing the early reversal seen on the real pipe.
     */
    if ((controller->moveDirection < 0) &&
        (remainingMm > H_TASK3_NEGATIVE_BRAKE_WINDOW_MM) &&
        (accelerationCommand > 0)) {
        accelerationCommand = 0;
    }
    if (controller->moveDirection < 0) {
        accelerationLimit = (controller->waypointReached != 0U) ?
            H_TASK3_NEGATIVE_CAPTURE_ACCEL_LIMIT_MM_S2 :
            H_TASK3_NEGATIVE_DRIVE_ACCEL_LIMIT_MM_S2;
    } else {
        accelerationLimit = (controller->waypointReached != 0U) ?
            H_TASK3_CAPTURE_ACCEL_LIMIT_MM_S2 :
            H_TASK3_ACCEL_LIMIT_MM_S2;
    }
    accelerationCommand = clamp32(accelerationCommand,
        -accelerationLimit, accelerationLimit);

    /* Positive lift accelerates toward negative ball position. */
    commandUm = -accelerationCommand * H_TASK3_ACCEL_TO_LIFT_UM +
        controller->disturbanceAccelerationMmPerSecond2 *
        H_TASK3_DISTURBANCE_TO_LIFT_UM;
    if ((desiredDirection != 0) &&
        ((accelerationCommand * desiredDirection) > 0)) {
        int32_t dynamicUm =
            learnedForDirection(controller, desiredDirection) *
            H_TASK3_DYNAMIC_FRICTION_PERCENT / 100L;
        commandUm += (desiredDirection > 0) ? -dynamicUm : dynamicUm;
    }
    if ((controller->moveDirection < 0) &&
        (remainingMm > H_TASK3_NEGATIVE_BRAKE_WINDOW_MM) &&
        (commandUm < 0)) {
        /* Observer compensation must obey the same no-early-reversal rule. */
        commandUm = 0;
    }
    if (controller->moveDirection < 0) {
        int32_t driveLimitUm =
            (controller->waypointReached != 0U) ?
            H_TASK3_NEGATIVE_CAPTURE_COMMAND_MAX_UM :
            H_TASK3_NEGATIVE_DRIVE_COMMAND_MAX_UM;
        if (commandUm > driveLimitUm) commandUm = driveLimitUm;
        if (commandUm < -H_TASK3_NEGATIVE_REVERSE_BRAKE_MAX_UM) {
            commandUm = -H_TASK3_NEGATIVE_REVERSE_BRAKE_MAX_UM;
        }
    }

    if (controller->phase == BALL_CONTROL_PROBE) {
        int32_t probeUm = H_TASK3_PROBE_MIN_UM +
            (int32_t)(controller->breakawayElapsedMs *
                      H_TASK3_PROBE_RAMP_UM_PER_100MS / 100U);
        int32_t learnedUm =
            learnedForDirection(controller, desiredDirection);

        if (probeUm < learnedUm) probeUm = learnedUm;
        if (controller->moveDirection < 0) {
            commandLimitUm = (controller->waypointReached != 0U) ?
                H_TASK3_NEGATIVE_CAPTURE_PROBE_MAX_UM :
                H_TASK3_NEGATIVE_DRIVE_PROBE_MAX_UM;
        } else {
            commandLimitUm = (controller->waypointReached != 0U) ?
                H_TASK3_CAPTURE_PROBE_MAX_UM : H_TASK3_PROBE_MAX_UM;
        }
        if (probeUm > commandLimitUm) {
            probeUm = commandLimitUm;
        }
        if ((desiredDirection > 0) && (commandUm > -probeUm)) {
            commandUm = -probeUm;
        } else if ((desiredDirection < 0) && (commandUm < probeUm)) {
            commandUm = probeUm;
        }
        controller->phase = BALL_CONTROL_PROBE;
    } else {
        controller->phase = (controller->waypointReached != 0U) ?
            BALL_CONTROL_CAPTURE : BALL_CONTROL_TRACK;
    }

    if ((remainingMm <= H_TASK3_POSITION_DEADBAND_MM) &&
        (absolute32(filteredVelocity) <= H_TASK3_HOLD_SPEED_MM_S)) {
        commandUm = 0;
        controller->stallFrames = 0U;
        controller->phase = BALL_CONTROL_HOLD;
    }

    if (controller->moveDirection < 0) {
        if (controller->phase == BALL_CONTROL_PROBE) {
            commandLimitUm = (controller->waypointReached != 0U) ?
                H_TASK3_NEGATIVE_CAPTURE_PROBE_MAX_UM :
                H_TASK3_NEGATIVE_DRIVE_PROBE_MAX_UM;
        } else {
            commandLimitUm = (controller->waypointReached != 0U) ?
                H_TASK3_NEGATIVE_CAPTURE_COMMAND_MAX_UM :
                H_TASK3_NEGATIVE_DRIVE_COMMAND_MAX_UM;
        }
    } else {
        commandLimitUm = (controller->waypointReached != 0U) ?
            H_TASK3_CAPTURE_COMMAND_MAX_UM : H_TASK3_COMMAND_MAX_UM;
    }
    commandUm = clamp32(commandUm,
        -commandLimitUm, commandLimitUm);
    if (commandUm == 0) {
        slewLimitUm = H_TASK3_RETURN_SLEW_UM_PER_TICK;
    }
    if ((controller->moveDirection < 0) &&
        (slewLimitUm > H_TASK3_NEGATIVE_COMMAND_SLEW_UM_PER_TICK)) {
        slewLimitUm = H_TASK3_NEGATIVE_COMMAND_SLEW_UM_PER_TICK;
    }
    outputDeltaUm = clamp32(commandUm - controller->lastOutputUm,
        -slewLimitUm, slewLimitUm);
    controller->lastOutputUm += outputDeltaUm;
    return controller->lastOutputUm;
}

bool BallController_waypointReached(const BallController *controller)
{
    return controller->waypointReached != 0U;
}

bool BallController_settled(const BallController *controller)
{
    return controller->phase == BALL_CONTROL_HOLD;
}

bool BallController_stuck(const BallController *controller)
{
    return controller->phase == BALL_CONTROL_STUCK;
}

const char *BallController_phaseText(const BallController *controller)
{
    switch (controller->phase) {
    case BALL_CONTROL_TRACK: return "TRACK";
    case BALL_CONTROL_PROBE: return "PROBE";
    case BALL_CONTROL_CAPTURE: return "CAPTURE";
    case BALL_CONTROL_GUARD: return "GUARD";
    case BALL_CONTROL_HOLD: return "HOLD";
    case BALL_CONTROL_STUCK: return "STUCK";
    default: return "BALL ERR";
    }
}

