/**
 * @file limit_switch.c
 * @brief PB4 normally-closed lower-limit input with integrator debounce.
 */
#include "bsp/limit_switch.h"

#include <stdint.h>

#include "config/hardware_config.h"
#include "ti_msp_dl_config.h"

static volatile uint8_t g_homeCounter;
static volatile uint8_t g_homeActive;

bool LimitSwitch_homeRaw(void)
{
    uint8_t high = (DL_GPIO_readPins(
        BEAM_IO_HOME_NC_PORT, BEAM_IO_HOME_NC_PIN) != 0U) ? 1U : 0U;

#if H_HOME_SWITCH_ACTIVE_HIGH
    return high != 0U;
#else
    return high == 0U;
#endif
}

void LimitSwitch_init(void)
{
    bool raw = LimitSwitch_homeRaw();
    g_homeCounter = raw ? H_HOME_DEBOUNCE_MS : 0U;
    g_homeActive = raw ? 1U : 0U;
}

void LimitSwitch_tick1ms(void)
{
    if (LimitSwitch_homeRaw()) {
        if (g_homeCounter < H_HOME_DEBOUNCE_MS) g_homeCounter++;
    } else if (g_homeCounter > 0U) {
        g_homeCounter--;
    }

    if (g_homeCounter >= H_HOME_DEBOUNCE_MS) {
        g_homeActive = 1U;
    } else if (g_homeCounter == 0U) {
        g_homeActive = 0U;
    }
}

bool LimitSwitch_homeActive(void)
{
    return g_homeActive != 0U;
}
