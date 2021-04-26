#include "delay.h"
 
static volatile uint32_t ticks;
 
void delayInit() 
{ 
  SystemCoreClockUpdate(); 
	SysTick_Config(SystemCoreClock / 3000000); // Overflow every 1 us 
} 

void delayDecrement(void)
{
	if (ticks != 0x00) 
 		ticks--;
}

void delayUs(uint32_t nTime) 
{ 
 	ticks = nTime; 
 	while(ticks); 
}
 
void delayMs(uint32_t nTime) 
{ 
	while(nTime--)  
 		delayUs(1000); 
} 
