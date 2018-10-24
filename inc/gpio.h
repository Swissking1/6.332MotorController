#ifndef __GPIO_H_
#define __GPIO_H_

#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_gpio.h>
#include <stm32f4xx_hal_gpio_ex.h>

#define LED1_PIN GPIO_PIN_0
#define LED1_PORT GPIOB

void Led_Init(void);

#endif //__GPIO_H_
