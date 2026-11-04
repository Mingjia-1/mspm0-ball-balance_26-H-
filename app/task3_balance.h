/**
 * @file task3_balance.h
 * @brief Vision trajectory task 3: O -> +50 waypoint -> -50 mm hold.
 */
#ifndef APP_TASK3_BALANCE_H_
#define APP_TASK3_BALANCE_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    TASK3_BALANCE_IDLE = 0,
    TASK3_BALANCE_HOMING,
    TASK3_BALANCE_ARMED,
    TASK3_BALANCE_TO_POSITIVE,
    TASK3_BALANCE_TO_NEGATIVE,
    TASK3_BALANCE_COMPLETE,
    TASK3_BALANCE_VISION_FAULT,
    TASK3_BALANCE_HOME_FAULT
} Task3BalanceState;

void Task3Balance_init(void);
void Task3Balance_arm(uint32_t nowMs);
void Task3Balance_start(uint32_t nowMs);
void Task3Balance_tick10ms(uint32_t nowMs);
void Task3Balance_stop(void);

bool Task3Balance_readyToStart(void);
bool Task3Balance_complete(void);
bool Task3Balance_faulted(void);
int32_t Task3Balance_ballPositionMm(void);
int32_t Task3Balance_ballVelocityMmPerSecond(void);
int32_t Task3Balance_liftUm(void);
const char *Task3Balance_stateText(void);

#endif
