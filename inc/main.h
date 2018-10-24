#ifndef __MAIN_H__
#define __MAIN_H__

#include "uart.h"
#include "gpio.h"
//#include "mpu9265_spi.h"

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

//void blink(void);

#endif // __MAIN_H__

