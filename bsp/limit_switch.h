/**
 * @file limit_switch.h
 * @brief Debounced fail-safe lower home switch for the T8 screw.
 */
#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include <stdbool.h>

void LimitSwitch_init(void);
void LimitSwitch_tick1ms(void);
bool LimitSwitch_homeActive(void);
bool LimitSwitch_homeRaw(void);

#endif
