/**
 * @file jy61p_protocol.h
 * @brief JY61P 0x55/11-byte frame parser and vehicle attitude result.
 */
#ifndef JY61P_PROTOCOL_H
#define JY61P_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define JY61P_FRAME_SIZE       (11U)
#define JY61P_TIMEOUT_MS       (250U)

typedef struct {
    int32_t rollMilliDegrees;
    int32_t pitchMilliDegrees;
    int32_t yawMilliDegrees;
    int32_t gyroXMilliDegreesPerSecond;
    int32_t gyroYMilliDegreesPerSecond;
    int32_t gyroZMilliDegreesPerSecond;
    uint32_t angleReceivedAtMs;
    uint32_t gyroReceivedAtMs;
    uint8_t angleValid;
    uint8_t gyroValid;
} Jy61pObservation;

typedef struct {
    uint8_t frame[JY61P_FRAME_SIZE];
    uint8_t length;
    Jy61pObservation latest;
    uint32_t acceptedFrames;
    uint32_t checksumErrors;
    uint32_t formatErrors;
} Jy61pProtocol;

void Jy61pProtocol_reset(Jy61pProtocol *protocol);
bool Jy61pProtocol_consumeByte(
    Jy61pProtocol *protocol, uint8_t byte, uint32_t nowMs);
bool Jy61pProtocol_getObservation(
    const Jy61pProtocol *protocol, uint32_t nowMs, Jy61pObservation *observation);

#endif
