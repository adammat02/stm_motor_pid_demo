#include "uart_comm.h"
#include <string.h>

volatile static char line[RX_BUFF_LEN];
volatile static bool line_ready = false;

static UART_HandleTypeDef *uart;
static uint8_t rx_byte;
static char rx_buff[RX_BUFF_LEN];
static uint8_t idx = 0;

void uart_init(UART_HandleTypeDef *huart)
{
  uart = huart;
  HAL_UART_Receive_IT(uart, &rx_byte, 1);
}

void uart_rx_byte_callback()
{
  if (line_ready)
    return;

  if (rx_byte == '\n' || rx_byte == '\r')
  {
    if (idx > 0)
    {
      rx_buff[idx] = '\0';
      memcpy((void*)line, rx_buff, idx + 1);
      line_ready = true;
      idx = 0;
    }
  }
  else
  {
    if (idx < RX_BUFF_LEN - 1)
    {
      rx_buff[idx++] = rx_byte;
    }
    else
    {
      idx = 0;
    }
  }

  HAL_UART_Receive_IT(uart, &rx_byte, 1);
}

void uart_send_str(const char *str)
{
  HAL_UART_Transmit(uart, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

bool uart_is_line()
{
  return line_ready;
}

void uart_get_line(char *out, size_t len)
{
  if (!line_ready)
    return;
  memcpy((void*)out,(void*)line, len);
  line_ready = false;
}