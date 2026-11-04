/** @file oled_display.h @brief SSD1306 task, state and timer display. */
#ifndef BSP_OLED_DISPLAY_H_
#define BSP_OLED_DISPLAY_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t taskNumber;
    const char *stateText;
    uint32_t elapsedMs;
    int32_t distanceMm;
    int32_t lineError;
} OledDisplayModel;

bool OledDisplay_init(void);
void OledDisplay_update(const OledDisplayModel *model);
bool OledDisplay_isOnline(void);

#endif
