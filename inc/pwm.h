#ifndef __PWM_H
#define __PWM_H


#include <stdint.h>

//void PWM_Output_Init();
void Set_PWM_Duty_Cycle(uint8_t frac, int channel);
void MX_TIM1_Init(void);

#endif // ifndef __PWM_H


