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
//void abc(float d, float q, float* a, float*b,float*c);
void dq0(float a, float b, float c, float* d,float* q);
/*
int32_t clarke(int32_t ia, int32_t ib);
int32_t park(int32_t i_alpha, int32_t i_beta,bool flag);
*/
int32_t inverse_park(int32_t vq_set, int32_t vd_set, bool flag);
int32_t inverse_park(int32_t vq_set, int32_t vd_set, bool flag);
int32_t inverse_clarke(int32_t v_alpha, int32_t v_beta,bool flag);


int _write(int file, char *data,int len);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

#endif // __MAIN_H__

