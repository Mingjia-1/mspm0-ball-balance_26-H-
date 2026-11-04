/** @file user_input.c @brief 15 ms integrator debounce with press events. */
#include "bsp/user_input.h"
#include "config/control_config.h"
#include "ti_msp_dl_config.h"

typedef struct {
    uint8_t counter;
    uint8_t pressed;
} DebouncedKey;

static DebouncedKey g_keys[4];
static volatile uint8_t g_events;

static uint8_t rawPressed(uint8_t index)
{
    switch (index) {
    case 0U:
        return (DL_GPIO_readPins(SYSTEM_IO_KEY1_N_PORT,
            SYSTEM_IO_KEY1_N_PIN) == 0U) ? 1U : 0U;
    case 1U:
        return (DL_GPIO_readPins(SYSTEM_IO_KEY2_N_PORT,
            SYSTEM_IO_KEY2_N_PIN) == 0U) ? 1U : 0U;
    case 2U:
        return (DL_GPIO_readPins(SYSTEM_IO_KEY3_N_PORT,
            SYSTEM_IO_KEY3_N_PIN) == 0U) ? 1U : 0U;
    default:
        return (DL_GPIO_readPins(SYSTEM_IO_KEY4_N_PORT,
            SYSTEM_IO_KEY4_N_PIN) == 0U) ? 1U : 0U;
    }
}

void UserInput_init(void)
{
    uint8_t i;
    for (i = 0U; i < 4U; i++) {
        g_keys[i].counter = 0U;
        g_keys[i].pressed = 0U;
    }
    g_events = 0U;
}

void UserInput_tick1ms(void)
{
    uint8_t i;
    for (i = 0U; i < 4U; i++) {
        uint8_t raw = rawPressed(i);
        if (raw != 0U) {
            if (g_keys[i].counter < H_BUTTON_DEBOUNCE_MS) g_keys[i].counter++;
        } else if (g_keys[i].counter > 0U) {
            g_keys[i].counter--;
        }

        if ((g_keys[i].pressed == 0U) &&
            (g_keys[i].counter >= H_BUTTON_DEBOUNCE_MS)) {
            g_keys[i].pressed = 1U;
            g_events |= (uint8_t)(1U << i);
        } else if ((g_keys[i].pressed != 0U) &&
                   (g_keys[i].counter == 0U)) {
            g_keys[i].pressed = 0U;
        }
    }
}

uint8_t UserInput_takeEvents(void)
{
    uint8_t events;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    events = g_events;
    g_events = 0U;
    if (primask == 0U) __enable_irq();
    return events;
}
