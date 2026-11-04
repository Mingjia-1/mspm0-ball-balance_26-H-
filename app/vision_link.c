/**
 * @file vision_link.c
 * @brief Connects UART bytes to the V3 parser without disturbing line control.
 */
#include "app/vision_link.h"

#include "bsp/vision_uart.h"

static volatile uint32_t g_visionMilliseconds;
static VisionProtocol g_visionProtocol;

void VisionLink_init(void)
{
    g_visionMilliseconds = 0U;
    VisionProtocol_reset(&g_visionProtocol);
    VisionUart_init();
}

void VisionLink_tick1ms(void)
{
    g_visionMilliseconds++;
}

void VisionLink_process(void)
{
    uint8_t byte;
    uint32_t nowMs = g_visionMilliseconds;

    while (VisionUart_readByte(&byte)) {
        (void)VisionProtocol_consumeByte(
            &g_visionProtocol, byte, nowMs);
    }
}

bool VisionLink_getObservation(VisionBallObservation *observation)
{
    return VisionProtocol_getObservation(
        &g_visionProtocol, g_visionMilliseconds, observation);
}

const VisionProtocolStats *VisionLink_getProtocolStats(void)
{
    return VisionProtocol_getStats(&g_visionProtocol);
}

uint32_t VisionLink_receivedByteCount(void)
{
    return VisionUart_receivedByteCount();
}

uint32_t VisionLink_takeUartOverflowCount(void)
{
    return VisionUart_takeOverflowCount();
}
