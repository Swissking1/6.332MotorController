#ifndef __MAIN_H__
#define __MAIN_H__

#include "uart.h"
#include "gpio.h"

void _Error_Handler(char *, int);
static void MX_TIM1_Init(void);
void Set_PWM_Duty_Cycle(uint8_t frac);

static uint32_t uhPrescalerValue = 0; // counter prescaler value

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

//void blink(void);

#endif // __MAIN_H__

