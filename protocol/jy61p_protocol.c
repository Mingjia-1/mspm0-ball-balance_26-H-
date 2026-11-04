/**
 * @file jy61p_protocol.c
 * @brief Parses standard JY61P angle (0x53) and gyro (0x52) frames.
 */
#include "protocol/jy61p_protocol.h"

#include <string.h>

static int16_t readI16Le(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int32_t angleMilliDegrees(int16_t raw)
{
    return ((int32_t)raw * 180000L) / 32768L;
}

static int32_t gyroMilliDegreesPerSecond(int16_t raw)
{
    return ((int32_t)raw * 2000000L) / 32768L;
}

static uint8_t checksum(const uint8_t *frame)
{
    uint8_t sum = 0U;
    uint8_t i;
    for (i = 0U; i < 10U; i++) sum = (uint8_t)(sum + frame[i]);
    return sum;
}

void Jy61pProtocol_reset(Jy61pProtocol *protocol)
{
    memset(protocol, 0, sizeof(*protocol));
}

bool Jy61pProtocol_consumeByte(
    Jy61pProtocol *protocol, uint8_t byte, uint32_t nowMs)
{
    uint8_t type;

    if (protocol->length == 0U) {
        if (byte != 0x55U) return false;
        protocol->frame[0] = byte;
        protocol->length = 1U;
        return false;
    }

    protocol->frame[protocol->length++] = byte;
    if (protocol->length < JY61P_FRAME_SIZE) return false;
    protocol->length = 0U;

    if (checksum(protocol->frame) != protocol->frame[10]) {
        protocol->checksumErrors++;
        if (protocol->frame[10] == 0x55U) {
            protocol->frame[0] = 0x55U;
            protocol->length = 1U;
        }
        return false;
    }

    type = protocol->frame[1];
    if (type == 0x53U) {
        protocol->latest.rollMilliDegrees =
            angleMilliDegrees(readI16Le(&protocol->frame[2]));
        protocol->latest.pitchMilliDegrees =
            angleMilliDegrees(readI16Le(&protocol->frame[4]));
        protocol->latest.yawMilliDegrees =
            angleMilliDegrees(readI16Le(&protocol->frame[6]));
        protocol->latest.angleReceivedAtMs = nowMs;
        protocol->latest.angleValid = 1U;
    } else if (type == 0x52U) {
        protocol->latest.gyroXMilliDegreesPerSecond =
            gyroMilliDegreesPerSecond(readI16Le(&protocol->frame[2]));
        protocol->latest.gyroYMilliDegreesPerSecond =
            gyroMilliDegreesPerSecond(readI16Le(&protocol->frame[4]));
        protocol->latest.gyroZMilliDegreesPerSecond =
            gyroMilliDegreesPerSecond(readI16Le(&protocol->frame[6]));
        protocol->latest.gyroReceivedAtMs = nowMs;
        protocol->latest.gyroValid = 1U;
    } else {
        protocol->formatErrors++;
        return false;
    }

    protocol->acceptedFrames++;
    return true;
}

bool Jy61pProtocol_getObservation(
    const Jy61pProtocol *protocol, uint32_t nowMs, Jy61pObservation *observation)
{
    if (observation == 0) return false;
    *observation = protocol->latest;
    observation->angleValid =
        ((protocol->latest.angleValid != 0U) &&
         ((uint32_t)(nowMs - protocol->latest.angleReceivedAtMs) <=
          JY61P_TIMEOUT_MS)) ? 1U : 0U;
    observation->gyroValid =
        ((protocol->latest.gyroValid != 0U) &&
         ((uint32_t)(nowMs - protocol->latest.gyroReceivedAtMs) <=
          JY61P_TIMEOUT_MS)) ? 1U : 0U;
    return (observation->angleValid != 0U) ||
        (observation->gyroValid != 0U);
}
