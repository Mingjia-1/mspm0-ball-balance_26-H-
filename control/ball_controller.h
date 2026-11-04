/**
 * @file ball_controller.h
 * @brief Vision trajectory controller with adaptive stick-slip handling.
 */
#ifndef BALL_CONTROLLER_H
#define BALL_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BALL_CONTROL_TRACK = 0,
    BALL_CONTROL_PROBE,
    BALL_CONTROL_CAPTURE,
    BALL_CONTROL_GUARD,
    BALL_CONTROL_HOLD,
    BALL_CONTROL_STUCK
} BallControlPhase;

typedef struct {
    int32_t startMm;
    int32_t targetMm;
    int32_t moveDirection;
    int32_t filteredVelocityMmPerSecond;
    int32_t referenceVelocityMmPerSecond;
    int32_t lastOutputUm;
    int32_t previousFilteredVelocityMmPerSecond;
    int32_t disturbanceAccelerationMmPerSecond2;
    int32_t guardBoundaryMm;
    int32_t learnedPositiveUm;
    int32_t learnedNegativeUm;
    int32_t progressAnchorMm;
    int32_t breakawayStartMm;
    uint32_t progressElapsedMs;
    uint32_t breakawayElapsedMs;
    uint16_t stallFrames;
    int8_t probeDirection;
    int8_t guardDirection;
    uint8_t waypointReached;
    uint8_t guardActive;
    BallControlPhase phase;
} BallController;

void BallController_reset(BallController *controller);
void BallController_setMove(
    BallController *controller, int32_t startMm, int32_t targetMm);
void BallController_setHold(BallController *controller, int32_t targetMm);
void BallController_setGuardBoundary(
    BallController *controller, int32_t boundaryMm);
int32_t BallController_step(BallController *controller,
    int32_t positionMm, int32_t velocityMmPerSecond,
    uint8_t newMeasurement, uint32_t sampleDeltaMs,
    int32_t actualLiftUm);
bool BallController_waypointReached(const BallController *controller);
bool BallController_settled(const BallController *controller);
bool BallController_stuck(const BallController *controller);
const char *BallController_phaseText(const BallController *controller);

#endif