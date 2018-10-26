#include "uart.h"

void uart_init(struct UART_Struct *self) {

	self->handle.Instance = USART3;
	self->handle.Init.BaudRate = 115200;
	self->handle.Init.WordLength = UART_WORDLENGTH_8B;
	self->handle.Init.StopBits = UART_STOPBITS_1;
	self->handle.Init.Parity = UART_PARITY_NONE;
	self->handle.Init.Mode = UART_MODE_TX_RX;
	self->handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	self->handle.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&(self->handle)) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
}

HAL_StatusTypeDef uart_transmit(struct UART_Struct *self, uint8_t *data, size_t size, int delay) {
	return HAL_UART_Transmit(&(self->handle), data, size, delay);
}

HAL_StatusTypeDef uart_transmit_char(struct UART_Struct *self, uint8_t *data, int delay) {
	return HAL_UART_Transmit(&(self->handle), data, 1, delay);
}

HAL_StatusTypeDef uart_receive(struct UART_Struct *self, uint8_t *data, size_t size, int delay) {
	return HAL_UART_Receive(&(self->handle), data, size, delay);
}
