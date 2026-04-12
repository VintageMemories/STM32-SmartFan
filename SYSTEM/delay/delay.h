#ifndef __DELAY_H
#define __DELAY_H

#include "sys.h"

extern volatile uint32_t g_msTicks;

void delay(uint32_t nus);
void delay_ms(uint32_t nms);
void delay_us(uint32_t nus);

#endif

