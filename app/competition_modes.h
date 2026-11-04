/** @file competition_modes.h @brief H-task 2/3/4/5/6 UI and finish logic. */
#ifndef APP_COMPETITION_MODES_H_
#define APP_COMPETITION_MODES_H_

#include <stdint.h>
#include "bsp/line_sensor_8ch.h"

typedef enum {
    COMPETITION_TASK_2 = 2,
    COMPETITION_TASK_3 = 3,
    COMPETITION_TASK_4 = 4,
    COMPETITION_TASK_5 = 5,
    COMPETITION_TASK_6 = 6
} CompetitionTask;

typedef enum {
    COMPETITION_STATE_SELECT = 0,
    COMPETITION_STATE_ARMED,
    COMPETITION_STATE_RUNNING,
    COMPETITION_STATE_BRAKING,
    COMPETITION_STATE_PAUSED,
    COMPETITION_STATE_COMPLETE,
    COMPETITION_STATE_TIMEOUT,
    COMPETITION_STATE_LINE_FAULT,
    COMPETITION_STATE_CONTROL_FAULT,
    COMPETITION_STATE_ESTOP
} CompetitionState;

typedef struct {
    CompetitionTask task;
    CompetitionState state;
    uint32_t startMs;
    uint32_t elapsedMs;
    uint8_t startLineExited;
    uint8_t exitStableTicks;
    uint8_t markerStableTicks;
    uint8_t startEvent;
} CompetitionModes;

void CompetitionModes_init(CompetitionModes *modes);
void CompetitionModes_handleKeys(CompetitionModes *modes, uint8_t keyEvents,
    uint32_t nowMs, LineSensorReading sensor, uint8_t balanceReady);
void CompetitionModes_step(CompetitionModes *modes, uint32_t nowMs,
    LineSensorReading sensor, int32_t distanceMm);
void CompetitionModes_brakeComplete(CompetitionModes *modes, uint32_t nowMs);
void CompetitionModes_task3Complete(CompetitionModes *modes, uint32_t nowMs);
void CompetitionModes_controlFault(CompetitionModes *modes);
void CompetitionModes_lineFault(CompetitionModes *modes);
uint8_t CompetitionModes_takeStartEvent(CompetitionModes *modes);
uint8_t CompetitionModes_shouldDrive(const CompetitionModes *modes);
uint8_t CompetitionModes_shouldBrake(const CompetitionModes *modes);
int32_t CompetitionModes_baseSpeed(const CompetitionModes *modes);
const char *CompetitionModes_stateText(const CompetitionModes *modes);

#endif
