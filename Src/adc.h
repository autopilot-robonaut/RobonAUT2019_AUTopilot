#include "robonaut_config.h"

void Start_ADC_Task(void const * argument);
void Delay_us(uint32_t delay);
void analog_multi_switch(uint8_t state);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
