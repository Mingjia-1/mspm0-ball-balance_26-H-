/**
 * @file task456_balance.h
 * @brief Moving-vehicle ball hold for H tasks 4, 5 and 6.
 */
#ifndef APP_TASK456_BALANCE_H_
#define APP_TASK456_BALANCE_H_

#include <stdbool.h>
#include <stdint.h>

void Task456Balance_init(void);
void Task456Balance_arm(uint8_t taskNumber, uint32_t nowMs);
void Task456Balance_start(uint32_t nowMs);
void Task456Balance_tick10ms(uint32_t nowMs);
void Task456Balance_stop(void);

bool Task456Balance_readyToStart(void);
bool Task456Balance_faulted(void);
int32_t Task456Balance_ballPositionMm(void);
int32_t Task456Balance_ballVelocityMmPerSecond(void);
int32_t Task456Balance_targetMm(void);
int32_t Task456Balance_liftUm(void);
const char *Task456Balance_stateText(void);

#endif