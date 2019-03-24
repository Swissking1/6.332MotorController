#ifndef __MAIN_H__
#define __MAIN_H__

#include "uart.h"
#include "gpio.h"
#include "pwm.h"
#include <math.h>
#include <stdbool.h>

void _Error_Handler(char *, int);

uint32_t Encoder_Read(void);
void Encoder_Start(void);
void Encoder_Stop(void);
float Get_Mech_Pos(void);
float Get_Elec_Pos(void);

void abc(float d, float q, float* a, float*b,float*c,float theta);
void dq0(float a, float b, float c, float* d,float* q, float theta);


void adc_init(void);
void adc_read(uint32_t chan, uint32_t *data);
int _write(int file, char *data,int len);

static void MX_GPIO_Init(void);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

#define SLO_PRINTF(...) {                    \
  static uint32_t last_print = 0;            \
  do {                                       \
    if (HAL_GetTick() - last_print > 1000) { \
      last_print = HAL_GetTick();            \
      printf(__VA_ARGS__);                   \
    }                                        \
   } while (0);                              \
}

#endif // __MAIN_H__

