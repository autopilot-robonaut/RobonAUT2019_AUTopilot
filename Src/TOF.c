#include "TOF.h"

#define TOF1_period 50


uint8_t dataReady;
uint8_t rangeStatus;
uint16_t TOF1_distance,TOF2_distance;

uint8_t ultra_timer = 1;
uint8_t ultra_overrun = 0;
uint32_t ultra_echo_time = 0;
float ultra_distance_mm=0;

uint32_t time_old_TOF,time_new_TOF,time_elapsed_TOF;


void Start_TOF_Task(void const * argument)
{
  /* USER CODE BEGIN Start_TOF_Task */
	
	/* Wait for boot */
	
	
	
  /* Infinite loop */
  for(;;)
  {
    osDelay(60);
//		time_old_TOF = get_timer_us();	
//		
//		/*
//		 * Elore nézo Ultrahang távolsag adata 
//		 * BEGIN
//		 */
//		HAL_TIM_Base_Start_IT(&htim13);
//		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);

//		ulTaskNotifyTake( pdTRUE,60);
//		/*
//		 * Elore nézo Ultrahang távolsag adata 
//		 * END
//		 */
//		
//	
//		if(ultra_overrun < 4)
//		{
//			ultra_distance_mm = ultra_echo_time*0.0345f;
//		}
//		ultra_overrun = 0;
//		time_elapsed_TOF = get_timer_us() - time_old_TOF;
  }
  /* USER CODE END Start_TOF_Task */
}

