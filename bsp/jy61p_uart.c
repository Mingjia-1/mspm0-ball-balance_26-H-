/**
 * @file jy61p_uart.c
 * @brief UART0 RX ISR and SPSC ring buffer for JY61P frames.
 */
#include "bsp/jy61p_uart.h"

#include "ti_msp_dl_config.h"

#define JY61P_UART_RING_SIZE    (128U)
#define JY61P_UART_RING_MASK    (JY61P_UART_RING_SIZE - 1U)

static volatile uint8_t g_rxRing[JY61P_UART_RING_SIZE];
static volatile uint16_t g_rxHead;
static volatile uint16_t g_rxTail;
static volatile uint32_t g_overflowCount;

static void pushFromIsr(uint8_t byte)
{
    uint16_t next = (uint16_t)((g_rxHead + 1U) & JY61P_UART_RING_MASK);

    if (next == g_rxTail) {
        g_overflowCount++;
        return;
    }
    g_rxRing[g_rxHead] = byte;
    g_rxHead = next;
}

void Jy61pUart_init(void)
{
    g_rxHead = 0U;
    g_rxTail = 0U;
    g_overflowCount = 0U;
    NVIC_ClearPendingIRQ(JY61P_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(JY61P_UART_INST_INT_IRQN);
}

bool Jy61pUart_readByte(uint8_t *byte)
{
    if ((byte == 0) || (g_rxTail == g_rxHead)) return false;
    *byte = g_rxRing[g_rxTail];
    g_rxTail = (uint16_t)((g_rxTail + 1U) & JY61P_UART_RING_MASK);
    return true;
}

uint32_t Jy61pUart_takeOverflowCount(void)
{
    uint32_t count;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    count = g_overflowCount;
    g_overflowCount = 0U;
    if (primask == 0U) __enable_irq();
    return count;
}

void JY61P_UART_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(JY61P_UART_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        do {
            pushFromIsr(
                (uint8_t)DL_UART_Main_receiveData(JY61P_UART_INST));
        } while (!DL_UART_Main_isRXFIFOEmpty(JY61P_UART_INST));
        break;
    default:
        break;
    }
}
