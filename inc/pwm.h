#ifndef __PWM_H
#define __PWM_H


#include <stdint.h>

void Set_PWM_Duty_Cycle(uint32_t frac, int channel);
void MX_TIM1_Init(void);

#endif // ifndef __PWM_H


