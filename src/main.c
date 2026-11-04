/** @file main.c @brief MSPM0G3507 H-task firmware entry. */
#include "app/jy61p_link.h"
#include "app/system_app.h"
#include "app/vision_link.h"
#include "ti_msp_dl_config.h"

void SysTick_Handler(void)
{
    SystemApp_tick1ms();
    VisionLink_tick1ms();
    Jy61pLink_tick1ms();
}

int main(void)
{
    SYSCFG_DL_init();
    SystemApp_init();
    VisionLink_init();
    Jy61pLink_init();
    DL_SYSTICK_config(CPUCLK_FREQ / 1000U);

    for (;;) {
        SystemApp_process();
        VisionLink_process();
        Jy61pLink_process();
    }
}
