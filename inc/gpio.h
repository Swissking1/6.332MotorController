#ifndef __GPIO_H_
#define __GPIO_H_

#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_gpio.h>
#include <stm32f4xx_hal_gpio_ex.h>

/* Quick HAL GPIO overview
 *
 * Possible pin values are:
 *    - GPIO_PIN_SET   which sets a pin to logic HIGH
 *    - GPIO_PIN_RESET which sets a pin to logic LOW
 *
 * HAL GPIO functions:
 *     - HAL_GPIO_ReadPin(GPIO(name)); // returns {GPIO_PIN_RESET, GPIO_PIN_SET}
 *     - HAL_GPIO_WritePin(GPIO(name), value);
 *     - HAL_GPIO_TogglePin(GPIO(name));
 *
 * Nice Macros:
 *     - READ_PIN(name); // returns bool
 */

/// MACRO DEFINITIONS

#define PIN(name) name ## _PIN
#define PORT(name) name ## _PORT
#define GPIO(name) (PORT(name)), (PIN(name))
#define READ_PIN(name) (HAL_GPIO_ReadPin(GPIO(name)) == GPIO_PIN_SET)

#define GPIO_BEGIN_INIT()       \
  GPIO_InitTypeDef gpioinit;    \
  __HAL_RCC_GPIOA_CLK_ENABLE(); \
  __HAL_RCC_GPIOB_CLK_ENABLE(); \
  __HAL_RCC_GPIOC_CLK_ENABLE();

#define DGPIO_INIT_OUT(name, begin)     \
  gpioinit.Pin   = PIN(name);           \
  gpioinit.Mode  = GPIO_MODE_OUTPUT_PP; \
  gpioinit.Pull  = GPIO_PULLUP;         \
  gpioinit.Speed = GPIO_SPEED_FAST;     \
  HAL_GPIO_Init(PORT(name), &gpioinit); \
  HAL_GPIO_WritePin(GPIO(name), begin);


#define DGPIO_INIT_IN(name, type)   \
  gpioinit.Pin   = PIN(name);       \
  gpioinit.Mode  = GPIO_MODE_INPUT; \
  gpioinit.Pull  = type;            \
  gpioinit.Speed = GPIO_SPEED_FAST; \
  HAL_GPIO_Init(PORT(name), &gpioinit);

// Input types:
//  - GPIO_NOPULL
//  - GPIO_PULLUP
//  - GPIO_PULLDOWN


/// PIN DEFINITIONS
// LEDs

#define BMSLED1_PIN                        GPIO_PIN_12
#define BMSLED1_PORT                       GPIOC

#define BMSLED2_PIN                        GPIO_PIN_10
#define BMSLED2_PORT                       GPIOC

#define BMSLED3_PIN                        GPIO_PIN_15
#define BMSLED3_PORT                       GPIOA

#define LED1_PIN                        GPIO_PIN_0
#define LED1_PORT                       GPIOB

#define LED2_PIN                        GPIO_PIN_7
#define LED2_PORT                       GPIOB

#define LED3_PIN                        GPIO_PIN_14
#define LED3_PORT                       GPIOB

#define EN1_PIN                        GPIO_PIN_10
#define EN1_PORT                       GPIOC

#define EN2_PIN                        GPIO_PIN_11
#define EN2_PORT                       GPIOC

#define EN3_PIN                        GPIO_PIN_12
#define EN3_PORT                       GPIOC

#endif //__GPIO_H_
