#ifndef UART_COMM_H
#define UART_COMM_H

#include "usart.h"
#include <stdbool.h>

#define RX_BUFF_LEN 100

void uart_init(UART_HandleTypeDef *huart);

void uart_rx_byte_callback(void);

void uart_send_str(const char *str);

bool uart_is_line(void);

void uart_get_line(char *out, uint16_t len);

#endif