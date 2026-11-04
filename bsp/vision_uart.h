/**
 * @file vision_uart.h
 * @brief Interrupt-driven byte transport for the MaixCAM Pro UART link.
 */
#ifndef VISION_UART_H
#define VISION_UART_H

#include <stdbool.h>
#include <stdint.h>

void VisionUart_init(void);
bool VisionUart_readByte(uint8_t *byte);
uint32_t VisionUart_receivedByteCount(void);
uint32_t VisionUart_takeOverflowCount(void);

#endif
