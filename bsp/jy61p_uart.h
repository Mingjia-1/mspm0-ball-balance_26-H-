/**
 * @file jy61p_uart.h
 * @brief Interrupt-driven UART0 transport for the vehicle-mounted JY61P.
 */
#ifndef JY61P_UART_H
#define JY61P_UART_H

#include <stdbool.h>
#include <stdint.h>

void Jy61pUart_init(void);
bool Jy61pUart_readByte(uint8_t *byte);
uint32_t Jy61pUart_takeOverflowCount(void);

#endif
