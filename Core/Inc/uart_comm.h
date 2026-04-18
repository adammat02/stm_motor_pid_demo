/**
 * @file uart_comm.h
 * @brief UART line-based communication interface.
 *
 * Receives data byte-by-byte via interrupt and buffers complete lines
 * terminated by '\n' or '\r'. Provides a simple API to check for and
 * retrieve received lines.
 *
 */

#ifndef UART_COMM_H
#define UART_COMM_H

#include "usart.h"
#include <stdbool.h>

/** Maximum length of a received line including the null terminator. */
#define RX_BUFF_LEN 100

/**
 * @brief Initialize UART and start interrupt-driven reception.
 *
 * @param huart  Pointer to the HAL UART handle to use.
 */
void uart_init(UART_HandleTypeDef *huart);

/**
 * @brief Process one received byte.
 *
 * Must be called from HAL_UART_RxCpltCallback(). Accumulates bytes into
 * an internal buffer and sets the line-ready flag on '\n' or '\r'.
 * Incoming bytes are ignored while a previous line has not been consumed.
 */
void uart_rx_byte_callback(void);

/**
 * @brief Transmit a null-terminated string over UART (blocking).
 *
 * @param str  Null-terminated string to send.
 */
void uart_send_str(const char *str);

/**
 * @brief Check whether a complete line has been received.
 *
 * @return true if a line is ready to be read with uart_get_line().
 */
bool uart_is_line(void);

/**
 * @brief Copy the received line into @p out and clear the ready flag.
 *
 * Does nothing if no line is available. The copied string is always
 * null-terminated.
 *
 * @param out  Destination buffer.
 * @param len  Size of @p out in bytes.
 */
void uart_get_line(char *out, size_t len);

#endif
