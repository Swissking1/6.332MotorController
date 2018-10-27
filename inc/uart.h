
#ifndef __UART_H
#define __UART_H

#include <stdint.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_uart.h>


void uart_init(void);
HAL_StatusTypeDef uart_transmit(uint8_t *data , int delay);
HAL_StatusTypeDef uart_receive(char *data,int delay);
HAL_StatusTypeDef uart_transmit_char(char data, int delay);

#endif //__UART_H
