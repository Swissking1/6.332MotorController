#include <stm32f4xx_hal.h>
#include "main.h"

extern void _Error_Handler(char *, int);

void HAL_MspInit(void) {

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);

  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);

  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);

  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);

  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);

  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);

  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart) {

  if(huart->Instance==USART3) {
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PC5     ------> USART3_RX
    PB10     ------> USART3_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8);
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_9);
  }
  else if(huart->Instance==USART6) {
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB14
    PB9
    */
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_14);
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_9);
  }
}
