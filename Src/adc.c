#include "adc.h"

uint32_t szenzor_sor_front[48],szenzor_sor_back[48];
uint32_t adc_buffer[9];

uint32_t time_old_adc,time_new_adc,time_elapsed_adc;

int8_t led_driver_counter,analog_multi_counter;

uint8_t spi_data_tcrt[8]={0x11,0x11,0x22,0x22,0x44,0x44,0x88,0x88};

uint32_t SHARP_side_voltage,SHARP_front_voltage;
float SHARP_side_mm,SHARP_side_mm_new,SHARP_front_mm,SHARP_front_mm_new;

float SHARP_array[10],SHARP_side_array[10];
uint32_t SHARP_array_index;

uint32_t multiplexer_switch[16]={
	0xC3000000,0x00000100,0x01000200,0x00000100,
	0x03004000,0x00000100,0x01000200,0x00000100,
	0x43008000,0x00000100,0x01000200,0x00000100,
	0x03004000,0x00000100,0x01000200,0x00000100
};

void Start_ADC_Task(void const * argument)
{
  /* USER CODE BEGIN Start_ADC_Task */
	
	GPIOC->BSRR = 0x03000000;
	
  /* Infinite loop */
  for(;;)
  {

		time_old_adc = get_timer_us();
		
		for(led_driver_counter=0;led_driver_counter<4;led_driver_counter++)
		{
			// Toggle TCRT5000 LEDs
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi2,(uint8_t *)&spi_data_tcrt[led_driver_counter*2],2,0xFFFF); //DMA-val nem volt idonyerés

			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);			
			
			// Start 300 us timer
			HAL_TIM_Base_Start_IT(&htim11);
			
			SHARP_front_voltage = iaverage(SHARP_array,10);
			SHARP_side_voltage = iaverage(SHARP_side_array,10);
			SHARP_side_mm = 337620.0f*powf(SHARP_side_voltage,-1);
			SHARP_front_mm = 1049857.072f*powf(SHARP_front_voltage,-1.05845);
			
			// Wait for TIM11 timer(300 us)
			ulTaskNotifyTake( pdTRUE,5 );
			HAL_TIM_Base_Stop_IT(&htim11);			
			
			for(analog_multi_counter=0;analog_multi_counter<4;analog_multi_counter++)
			{
				
				// Analóg multiplexálás
				analog_multi_switch(led_driver_counter*4+analog_multi_counter);
				Delay_us(2);
				
				// Elso szenzorsor értékei
				HAL_ADC_Start(&hadc2);
				HAL_ADC_Start(&hadc3);
				HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)&adc_buffer, 9);

				ulTaskNotifyTake( pdTRUE,5);
				
				// Érték rendezés							
				szenzor_sor_front[led_driver_counter+4*analog_multi_counter] = adc_buffer[0];
				szenzor_sor_front[16+led_driver_counter+4*analog_multi_counter] = adc_buffer[1];
				szenzor_sor_front[32+led_driver_counter+4*analog_multi_counter] = adc_buffer[2];
				szenzor_sor_back[led_driver_counter+4*analog_multi_counter] = adc_buffer[3];
				szenzor_sor_back[16+led_driver_counter+4*analog_multi_counter] = adc_buffer[4];
				szenzor_sor_back[32+led_driver_counter+4*analog_multi_counter] = adc_buffer[5];
				
				SHARP_array[SHARP_array_index] =adc_buffer[7];
				SHARP_side_array[SHARP_array_index] = adc_buffer[6];
				SHARP_array_index++;
				if(SHARP_array_index == 10)SHARP_array_index=0;
				
			}
		}
		xTaskNotify(Calculate_TaskHandle,0,eNoAction);
		// Egy ciklus beolvasási ideje
		time_elapsed_adc = get_timer_us() - time_old_adc;		
  }
  /* USER CODE END Start_ADC_Task */
}

void Delay_us(uint32_t delay) {
	delay = delay*60;
	while (delay--);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	vTaskNotifyGiveFromISR( ADC_TaskHandle, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
	xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR( ADC_TaskHandle, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void analog_multi_switch(uint8_t state)
{
	GPIOB->BSRR = multiplexer_switch[state];
}

