#include "pwm_in.h"

// Captured Value
uint32_t PWMINValue = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    // Get the Input Capture value 
    PWMINValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
  }
}
