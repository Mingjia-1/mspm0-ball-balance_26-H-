/**
 * @file jy61p_link.c
 * @brief Connects UART0 bytes to the standard JY61P frame parser.
 */
#include "app/jy61p_link.h"

#include <stdint.h>

#include "bsp/jy61p_uart.h"

static volatile uint32_t g_jy61pMilliseconds;
static Jy61pProtocol g_jy61pProtocol;

void Jy61pLink_init(void)
{
    g_jy61pMilliseconds = 0U;
    Jy61pProtocol_reset(&g_jy61pProtocol);
    Jy61pUart_init();
}

void Jy61pLink_tick1ms(void)
{
    g_jy61pMilliseconds++;
}

void Jy61pLink_process(void)
{
    uint8_t byte;
    uint32_t nowMs = g_jy61pMilliseconds;

    while (Jy61pUart_readByte(&byte)) {
        (void)Jy61pProtocol_consumeByte(
            &g_jy61pProtocol, byte, nowMs);
    }
}

bool Jy61pLink_getObservation(Jy61pObservation *observation)
{
    return Jy61pProtocol_getObservation(
        &g_jy61pProtocol, g_jy61pMilliseconds, observation);
}
