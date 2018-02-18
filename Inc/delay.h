#ifndef DELAY_H
#define DELAY_H

#include <stdint.h>
#include "stm32f1xx_hal.h"


void delay_ms(uint32_t ms);
void delay_us(uint32_t us);
uint32_t get_timeout(uint32_t start_tick);


#endif 

