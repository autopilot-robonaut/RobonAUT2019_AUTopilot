#include "timer.h"

uint32_t get_timer_us(){
	uint32_t time = 0;
	time = Timer7_UPPER * 60000 + __HAL_TIM_GET_COUNTER(&htim7);
	return time;
}

unsigned long getRunTimeCounterValue(void)
{
	return get_timer_us();
}
