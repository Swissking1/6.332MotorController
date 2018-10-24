#ifndef __UART_H
#define __UART_H

#include <stdint.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_uart.h>

typedef struct UART_Struct {
  UART_HandleTypeDef handle;
} UART_Struct;

void uart_init(struct UART_Struct *self);
HAL_StatusTypeDef uart_transmit(struct UART_Struct *self, uint8_t *data, int delay);
HAL_StatusTypeDef uart_receive(struct UART_Struct *self, uint8_t *data, int delay);
HAL_StatusTypeDef uart_transmit_char(struct UART_Struct *self, uint8_t *data, int delay);

#endif //__UART_H
