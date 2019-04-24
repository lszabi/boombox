#ifndef UTILS_H
#define UTILS_H

#include "stm32f10x_lib.h"

void RCC_Configuration(void);
void NVIC_Configuration(void);

void delay_ms(vu32);
void delay_us(vu32);

#endif // UTILS_H
