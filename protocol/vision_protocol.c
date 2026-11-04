/**
 * @file vision_protocol.c
 * @brief Robust parser for the fixed 27-byte BALL_ROD_POSITION V3 frame.
 */
#include "protocol/vision_protocol.h"

#include <stddef.h>
#include <string.h>

#define VISION_HEADER_0        (0xA5U)
#define VISION_HEADER_1        (0x5AU)
#define VISION_VERSION         (0x03U)
#define VISION_MESSAGE_TYPE    (0x03U)

static uint16_t readU16Le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int16_t readI16Le(const uint8_t *data)
{
    return (int16_t)readU16Le(data);
}

static int32_t absolute16(int16_t value)
{
    return (value < 0) ? -(int32_t)value : (int32_t)value;
}

uint16_t VisionProtocol_crc16CcittFalse(
    const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;

    for (index = 0U; index < length; index++) {
        uint8_t bit;
        crc ^= (uint16_t)data[index] << 8;
        for (bit = 0U; bit < 8U; bit++) {
            crc = ((crc & 0x8000U) != 0U) ?
                (uint16_t)((crc << 1) ^ 0x1021U) :
                (uint16_t)(crc << 1);
        }
    }
    return crc;
}

void VisionProtocol_reset(VisionProtocol *protocol)
{
    if (protocol == NULL) return;
    (void)memset(protocol, 0, sizeof(*protocol));
}

static void resynchronize(VisionProtocol *protocol)
{
    uint8_t index;

    for (index = 1U; index + 1U < VISION_PROTOCOL_FRAME_SIZE; index++) {
        if ((protocol->frame[index] == VISION_HEADER_0) &&
            (protocol->frame[index + 1U] == VISION_HEADER_1)) {
            uint8_t remaining =
                (uint8_t)(VISION_PROTOCOL_FRAME_SIZE - index);
            (void)memmove(protocol->frame, &protocol->frame[index], remaining);
            protocol->frameLength = remaining;
            return;
        }
    }

    if (protocol->frame[VISION_PROTOCOL_FRAME_SIZE - 1U] ==
        VISION_HEADER_0) {
        protocol->frame[0] = VISION_HEADER_0;
        protocol->frameLength = 1U;
    } else {
        protocol->frameLength = 0U;
    }
}

static bool frameFormatIsValid(VisionProtocol *protocol)
{
    uint16_t expectedCrc;
    uint16_t receivedCrc;

    if ((protocol->frame[2] != VISION_VERSION) ||
        (protocol->frame[3] != VISION_MESSAGE_TYPE) ||
        (protocol->frame[6] != VISION_PROTOCOL_PAYLOAD_SIZE)) {
        protocol->stats.formatErrors++;
        return false;
    }

    expectedCrc = VisionProtocol_crc16CcittFalse(
        &protocol->frame[2], 23U);
    receivedCrc = readU16Le(&protocol->frame[25]);
    if (expectedCrc != receivedCrc) {
        protocol->stats.crcErrors++;
        return false;
    }
    return true;
}

static bool observationIsPlausible(const VisionBallObservation *observation)
{
    if (observation->valid > 1U) return false;
    if (observation->confidence > 100U) return false;
    if (observation->sourceAgeMs > 1000U) return false;

    if (observation->valid == 0U) return true;
    if (absolute16(observation->positionMm) >
        VISION_PROTOCOL_POSITION_LIMIT_MM) return false;
    if (absolute16(observation->velocityMmPerSecond) >
        VISION_PROTOCOL_VELOCITY_LIMIT_MM_S) return false;
    return true;
}

static void decodeObservation(VisionProtocol *protocol, uint32_t nowMs)
{
    VisionBallObservation observation;
    uint16_t sequence;

    (void)memset(&observation, 0, sizeof(observation));
    sequence = readU16Le(&protocol->frame[4]);
    observation.sequence = sequence;
    observation.valid = protocol->frame[7];
    observation.confidence = protocol->frame[8];
    observation.positionMm = readI16Le(&protocol->frame[9]);
    observation.velocityMmPerSecond = readI16Le(&protocol->frame[11]);
    observation.centerXPixels = readU16Le(&protocol->frame[13]);
    observation.centerYPixels = readU16Le(&protocol->frame[15]);
    observation.boxWidthPixels = readU16Le(&protocol->frame[17]);
    observation.boxHeightPixels = readU16Le(&protocol->frame[19]);
    observation.ballCount = protocol->frame[21];
    observation.flags = protocol->frame[22];
    observation.sourceAgeMs = readU16Le(&protocol->frame[23]);
    observation.receivedAtMs = nowMs;
    observation.transportFresh = 1U;

    if (!observationIsPlausible(&observation)) {
        protocol->stats.semanticErrors++;
        return;
    }

    if ((protocol->haveSequence != 0U) &&
        (sequence == protocol->lastSequence)) {
        protocol->stats.duplicateFrames++;
        return;
    }

    if (protocol->haveSequence != 0U) {
        uint16_t advance = (uint16_t)(sequence - protocol->lastSequence);
        if ((advance > 1U) && (advance < 0x8000U)) {
            protocol->stats.estimatedDroppedFrames +=
                (uint32_t)(advance - 1U);
        }
    }

    if (observation.valid == 0U) {
        observation.positionMm = 0;
        observation.velocityMmPerSecond = 0;
    }
    observation.controlUsable =
        ((observation.valid != 0U) &&
         ((observation.flags & VISION_FLAG_POSITION_CALIBRATED) != 0U) &&
         (observation.confidence >= VISION_PROTOCOL_MIN_CONFIDENCE) &&
         (((observation.flags & VISION_FLAG_POSITION_PREDICTED) == 0U) ||
          (observation.sourceAgeMs <=
           VISION_PROTOCOL_PREDICTION_MAX_AGE_MS))) ?
        1U : 0U;

    protocol->lastSequence = sequence;
    protocol->haveSequence = 1U;
    protocol->haveObservation = 1U;
    protocol->latest = observation;
    protocol->stats.acceptedFrames++;
}

bool VisionProtocol_consumeByte(
    VisionProtocol *protocol, uint8_t byte, uint32_t nowMs)
{
    uint32_t acceptedBefore;

    if (protocol == NULL) return false;
    acceptedBefore = protocol->stats.acceptedFrames;

    if (protocol->frameLength == 0U) {
        if (byte == VISION_HEADER_0) {
            protocol->frame[0] = byte;
            protocol->frameLength = 1U;
        }
        return false;
    }

    if (protocol->frameLength == 1U) {
        if (byte == VISION_HEADER_1) {
            protocol->frame[1] = byte;
            protocol->frameLength = 2U;
        } else if (byte != VISION_HEADER_0) {
            protocol->frameLength = 0U;
        }
        return false;
    }

    protocol->frame[protocol->frameLength++] = byte;
    if (protocol->frameLength < VISION_PROTOCOL_FRAME_SIZE) return false;

    if (frameFormatIsValid(protocol)) {
        decodeObservation(protocol, nowMs);
        protocol->frameLength = 0U;
    } else {
        resynchronize(protocol);
    }
    return protocol->stats.acceptedFrames != acceptedBefore;
}

bool VisionProtocol_getObservation(const VisionProtocol *protocol,
    uint32_t nowMs, VisionBallObservation *observation)
{
    if ((protocol == NULL) || (observation == NULL) ||
        (protocol->haveObservation == 0U)) {
        if (observation != NULL) {
            (void)memset(observation, 0, sizeof(*observation));
        }
        return false;
    }

    *observation = protocol->latest;
    if ((uint32_t)(nowMs - observation->receivedAtMs) >
        VISION_PROTOCOL_TIMEOUT_MS) {
        observation->valid = 0U;
        observation->positionMm = 0;
        observation->velocityMmPerSecond = 0;
        observation->transportFresh = 0U;
        observation->controlUsable = 0U;
        return false;
    }
    return true;
}

const VisionProtocolStats *VisionProtocol_getStats(
    const VisionProtocol *protocol)
{
    return (protocol != NULL) ? &protocol->stats : NULL;
}
