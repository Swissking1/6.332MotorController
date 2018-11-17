#ifndef __MAIN_H__
#define __MAIN_H__

#include "uart.h"
#include "gpio.h"

void _Error_Handler(char *, int);
void MX_TIM1_Init(void);
void Set_PWM_Duty_Cycle(uint8_t frac, int channel);


#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

#endif // __MAIN_H__

