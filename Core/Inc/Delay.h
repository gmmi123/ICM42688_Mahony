#ifndef __Delay_H
#define __Delay_H

#include "stm32f4xx.h"                  // Device header

void delay_init(void);
void delay_us(uint32_t nus);
#endif
