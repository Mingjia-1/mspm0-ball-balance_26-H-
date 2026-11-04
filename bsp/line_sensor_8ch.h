/** @file line_sensor_8ch.h @brief 1 kHz majority-filtered eight-channel input. */
#ifndef BSP_LINE_SENSOR_8CH_H_
#define BSP_LINE_SENSOR_8CH_H_

#include <stdint.h>

typedef struct {
    uint8_t rawMask;
    uint8_t activeMask;
    uint8_t activeCount;
    uint8_t visible;
    uint8_t marker;
    int32_t error;
} LineSensorReading;

void LineSensor8ch_init(void);
void LineSensor8ch_tick1ms(void);
LineSensorReading LineSensor8ch_read(void);

#endif
