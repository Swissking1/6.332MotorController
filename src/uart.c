#include "uart.h"
#include "stm32f4xx_hal.h"
#include <string.h>

extern void _Error_Handler(char *, int);

static UART_HandleTypeDef uart;

static void uart_gpio_init(void) {
  // Peripheral clock enable
    __HAL_RCC_USART3_CLK_ENABLE();

    // USART3 GPIO Configuration
    // PC5      ------> USART3_RX
    // PB10     ------> USART3_TX
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void uart_init() {
  uart_gpio_init();

  uart.Instance = USART3;
  uart.Init.BaudRate = 115200;
  uart.Init.WordLength = UART_WORDLENGTH_8B;
  uart.Init.StopBits = UART_STOPBITS_1;
  uart.Init.Parity = UART_PARITY_NONE;
  uart.Init.Mode = UART_MODE_TX_RX;
  uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  uart.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&(uart)) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }
}

HAL_StatusTypeDef uart_transmit(uint8_t *data, int delay) {
  return HAL_UART_Transmit(&(uart), data, strlen(data), delay);
}

HAL_StatusTypeDef uart_transmit_char(char data, int delay) {
  return HAL_UART_Transmit(&(uart), &data, 1, delay);
}

HAL_StatusTypeDef uart_receive(char *data, int delay) {
  return HAL_UART_Receive(&(uart), data, sizeof(data), delay);
}
