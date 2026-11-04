/**
 * @file vision_link.h
 * @brief Application-facing MaixCAM link service.
 */
#ifndef APP_VISION_LINK_H_
#define APP_VISION_LINK_H_

#include <stdbool.h>
#include <stdint.h>

#include "protocol/vision_protocol.h"

void VisionLink_init(void);
void VisionLink_tick1ms(void);
void VisionLink_process(void);
bool VisionLink_getObservation(VisionBallObservation *observation);
const VisionProtocolStats *VisionLink_getProtocolStats(void);
uint32_t VisionLink_receivedByteCount(void);
uint32_t VisionLink_takeUartOverflowCount(void);

#endif
