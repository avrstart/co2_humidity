#include "delay.h"


void delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

void delay_us(uint32_t us)
{
    volatile uint32_t nCount; 
     
    nCount=(HAL_RCC_GetSysClockFreq()/10000000)*us; 
    for (; nCount!=0; nCount--); 
}


uint32_t get_timeout(uint32_t start_tick) 
{
    int32_t current_tick = HAL_GetTick() - start_tick;
    
    if(current_tick > 0)
    {
        return current_tick;
    }
    else {
        return (UINT32_MAX + current_tick);
    }
}



