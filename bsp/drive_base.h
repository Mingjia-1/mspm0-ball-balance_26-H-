/** @file drive_base.h @brief TB6612 chassis and encoder edge counters. */
#ifndef BSP_DRIVE_BASE_H_
#define BSP_DRIVE_BASE_H_

#include <stdint.h>

typedef struct {
    int32_t left;
    int32_t right;
} DriveBaseEncoderCounts;

void DriveBase_init(void);
void DriveBase_setDuty(int16_t leftPercent, int16_t rightPercent);
void DriveBase_stop(void);
void DriveBase_brake(void);
void DriveBase_resetEncoders(void);
DriveBaseEncoderCounts DriveBase_getEncoderCounts(void);
DriveBaseEncoderCounts DriveBase_takeEncoderDelta(void);
int32_t DriveBase_getAverageDistanceEdges(void);

#endif
