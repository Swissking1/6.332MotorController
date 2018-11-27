#ifndef __MAIN_H__
#define __MAIN_H__

#include "uart.h"
#include "gpio.h"
#include "pwm.h"

void _Error_Handler(char *, int);

uint32_t Encoder_Read(void);
void Encoder_Start(void);
void Encoder_Stop(void);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

#endif // __MAIN_H__

