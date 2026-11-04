/** @file user_input.h @brief Debounced active-low four-key event input. */
#ifndef BSP_USER_INPUT_H_
#define BSP_USER_INPUT_H_

#include <stdint.h>

typedef enum {
    USER_KEY_START_PAUSE = 0x01U,
    USER_KEY_MODE = 0x02U,
    USER_KEY_CONFIRM = 0x04U,
    USER_KEY_ESTOP_RESET = 0x08U
} UserKeyEvent;

void UserInput_init(void);
void UserInput_tick1ms(void);
uint8_t UserInput_takeEvents(void);

#endif
