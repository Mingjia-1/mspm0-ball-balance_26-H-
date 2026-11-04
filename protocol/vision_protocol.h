/**
 * @file vision_protocol.h
 * @brief MaixCAM BALL_ROD_POSITION V3 stream parser and safety gate.
 */
#ifndef VISION_PROTOCOL_H
#define VISION_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define VISION_PROTOCOL_FRAME_SIZE             (27U)
#define VISION_PROTOCOL_PAYLOAD_SIZE           (18U)
#define VISION_PROTOCOL_TIMEOUT_MS             (120U)
#define VISION_PROTOCOL_MIN_CONFIDENCE         (60U)
#define VISION_PROTOCOL_PREDICTION_MAX_AGE_MS  (110U)
#define VISION_PROTOCOL_POSITION_LIMIT_MM      (150)
#define VISION_PROTOCOL_VELOCITY_LIMIT_MM_S    (3000)

#define VISION_FLAG_POSITION_CALIBRATED        (1U << 0)
#define VISION_FLAG_TARGET_STABLE              (1U << 1)
#define VISION_FLAG_TARGET_CLIPPED             (1U << 2)
#define VISION_FLAG_MULTIPLE_CANDIDATES        (1U << 3)
#define VISION_FLAG_SELECTED_BY_CONTINUITY     (1U << 4)
#define VISION_FLAG_POSITION_FILTERED          (1U << 5)
#define VISION_FLAG_ROI_ENABLED                (1U << 6)
#define VISION_FLAG_POSITION_PREDICTED         (1U << 7)

typedef struct {
    uint16_t sequence;
    uint8_t valid;
    uint8_t confidence;
    int16_t positionMm;
    int16_t velocityMmPerSecond;
    uint16_t centerXPixels;
    uint16_t centerYPixels;
    uint16_t boxWidthPixels;
    uint16_t boxHeightPixels;
    uint8_t ballCount;
    uint8_t flags;
    uint16_t sourceAgeMs;
    uint32_t receivedAtMs;
    uint8_t transportFresh;
    uint8_t controlUsable;
} VisionBallObservation;

typedef struct {
    uint32_t acceptedFrames;
    uint32_t crcErrors;
    uint32_t formatErrors;
    uint32_t semanticErrors;
    uint32_t duplicateFrames;
    uint32_t estimatedDroppedFrames;
} VisionProtocolStats;

typedef struct {
    uint8_t frame[VISION_PROTOCOL_FRAME_SIZE];
    uint8_t frameLength;
    uint8_t haveSequence;
    uint8_t haveObservation;
    uint16_t lastSequence;
    VisionBallObservation latest;
    VisionProtocolStats stats;
} VisionProtocol;

void VisionProtocol_reset(VisionProtocol *protocol);
bool VisionProtocol_consumeByte(
    VisionProtocol *protocol, uint8_t byte, uint32_t nowMs);
bool VisionProtocol_getObservation(const VisionProtocol *protocol,
    uint32_t nowMs, VisionBallObservation *observation);
const VisionProtocolStats *VisionProtocol_getStats(
    const VisionProtocol *protocol);
uint16_t VisionProtocol_crc16CcittFalse(
    const uint8_t *data, uint16_t length);

#endif
