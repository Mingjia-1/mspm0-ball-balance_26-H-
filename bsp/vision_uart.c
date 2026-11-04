/**
 * @file vision_uart.c
 * @brief UART1 RX ISR and single-producer/single-consumer ring buffer.
 */
#include "bsp/vision_uart.h"

#include "ti_msp_dl_config.h"

#define VISION_UART_RING_SIZE    (256U)
#define VISION_UART_RING_MASK    (VISION_UART_RING_SIZE - 1U)

static volatile uint8_t g_rxRing[VISION_UART_RING_SIZE];
static volatile uint16_t g_rxHead;
static volatile uint16_t g_rxTail;
static volatile uint32_t g_receivedByteCount;
static volatile uint32_t g_overflowCount;

static void pushFromIsr(uint8_t byte)
{
    uint16_t next;

    g_receivedByteCount++;
    next = (uint16_t)((g_rxHead + 1U) & VISION_UART_RING_MASK);
    if (next == g_rxTail) {
        g_overflowCount++;
        return;
    }
    g_rxRing[g_rxHead] = byte;
    g_rxHead = next;
}

void VisionUart_init(void)
{
    g_rxHead = 0U;
    g_rxTail = 0U;
    g_receivedByteCount = 0U;
    g_overflowCount = 0U;
    NVIC_ClearPendingIRQ(VISION_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(VISION_UART_INST_INT_IRQN);
}

bool VisionUart_readByte(uint8_t *byte)
{
    uint32_t primask;
    bool received = false;

    if (byte == 0) return false;

    /*
     * The interrupt-driven ring remains the primary path. If an RX interrupt
     * is ever missed during reset/startup, poll the UART data register as a
     * recovery path so the link cannot remain permanently at UART NONE.
     */
    primask = __get_PRIMASK();
    __disable_irq();
    if (g_rxTail != g_rxHead) {
        *byte = g_rxRing[g_rxTail];
        g_rxTail = (uint16_t)((g_rxTail + 1U) & VISION_UART_RING_MASK);
        received = true;
    } else if (!DL_UART_Main_isRXFIFOEmpty(VISION_UART_INST)) {
        *byte = (uint8_t)DL_UART_Main_receiveData(VISION_UART_INST);
        g_receivedByteCount++;
        received = true;
    }
    if (primask == 0U) __enable_irq();
    return received;
}

uint32_t VisionUart_receivedByteCount(void)
{
    return g_receivedByteCount;
}

uint32_t VisionUart_takeOverflowCount(void)
{
    uint32_t count;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    count = g_overflowCount;
    g_overflowCount = 0U;
    if (primask == 0U) __enable_irq();
    return count;
}

void VISION_UART_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(VISION_UART_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(VISION_UART_INST)) {
                pushFromIsr(
                    (uint8_t)DL_UART_Main_receiveData(VISION_UART_INST));
            }
            break;
        default:
            break;
    }
}
