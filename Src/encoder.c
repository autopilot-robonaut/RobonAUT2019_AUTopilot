#include "encoder.h"

#define ENC_TO_METER_const 0.0000071816f*2.0f
#define ENC_TO_MM_const (ENC_TO_METER_const*1000)

float speed;

uint16_t current_encoder_val,last_encoder_val,current_encoder_val_unchanged;
float pos_x=0,pos_y=0;

float encoder_upper_value;

float encoder_full_value,encoder_full_value_before;

uint32_t encoder_time_before,encoder_time_now;
float encoder_time_between;

uint8_t speed_index;
float last_elozes_korr_x,last_elozes_korr_y;

void Start_Encoder_Task(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		encoder_time_now = get_timer_us();
		current_encoder_val = TIM3->CNT;
		current_encoder_val_unchanged = current_encoder_val;
		
		current_encoder_val &= 0xC000; 
		last_encoder_val &= 0xC000;
		if(current_encoder_val == 0 && last_encoder_val == 0xC000) encoder_upper_value++;
		if(current_encoder_val == 0xC000 && last_encoder_val == 0) encoder_upper_value--;
		
		encoder_full_value = encoder_upper_value* 65535.0f + (float) current_encoder_val_unchanged;
		
		// Másodpercben az eltelt ido két mérés között
		encoder_time_between = (float)(encoder_time_now - encoder_time_before)*0.000001f;
		
		// M/s avagy mm/ms sebesség érték
		if((ENC_TO_METER_const * (encoder_full_value - encoder_full_value_before))/encoder_time_between!=0)
		{
			speed=0.8f*speed+0.2f*(ENC_TO_METER_const * (encoder_full_value - encoder_full_value_before))/encoder_time_between;			
		}
		else {speed=0;}
		
		pos_x += cosf(yaw)*ENC_TO_MM_const * (encoder_full_value - encoder_full_value_before);
		pos_y += sinf(yaw)*ENC_TO_MM_const * (encoder_full_value - encoder_full_value_before);
		elozes_posx-=last_elozes_korr_x;
		elozes_posy-=last_elozes_korr_y;
		elozes_posx += cosf(szog_hiba_SI)*ENC_TO_MM_const * (encoder_full_value - encoder_full_value_before)+ cosf(szog_hiba_SI)*245;
		elozes_posy += sinf(szog_hiba_SI)*ENC_TO_MM_const * (encoder_full_value - encoder_full_value_before)+sinf(szog_hiba_SI)*245;
		last_elozes_korr_x=cosf(szog_hiba_SI)*245;
		last_elozes_korr_y=sinf(szog_hiba_SI)*245;
		encoder_full_value_before = encoder_full_value;
		encoder_time_before = encoder_time_now;
		last_encoder_val = current_encoder_val_unchanged;
		
		osDelay(1);
  }
  /* USER CODE END 5 */ 
}

float enc_mm_value_get(){
	return (encoder_full_value*ENC_TO_MM_const);
}

