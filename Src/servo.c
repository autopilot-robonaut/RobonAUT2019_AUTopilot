#include "servo.h"

// 0 ha automatában megy, és bármi más érték esetén RC távirányítóval mükszik
#define MANUAL_SERVO 0

#define SERVO_MAX_PWM 1900
#define SERVO_MIN_PWM 1000
#define SERVO_MID_PWM 1525

#define SERVO_ARRAY_SIZE 5

float servo_pwm=SERVO_MID_PWM;
float servo_pwm_array[SERVO_ARRAY_SIZE];
uint16_t servo_array_index,for_index_servo;
uint8_t egyenesen_megyunk = FALSE,egyenesen_megyunk_temp = FALSE;

int32_t SERVO_szog_P = 0,SERVO_I,SERVO_vonal_P = 500;

//State Space 
float kszi=0.9, Tservo, tbeallas,dbeallas,svalos,skomplex,korientacio,kkozep,speedsi,L = 0.265 ,deltap,phi;
float dbeallasm,dbeallaso;


void Start_Servo_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Servo_Task */
  /* Infinite loop */
  for(;;)
  {
		
		switch(state_game)
		{
			case LABYRINTH:
				dbeallas=0.5f*speed+0.375f;
				if(dbeallas < 0.9f) dbeallas = 0.9f;
				else if(dbeallas > 2.0f) dbeallas = 2.0f;
				allapotszabalyozo();
				break;
			
			case LANE_CHANGE:
				dbeallas=0.375f*speed+0.375f;
				if(dbeallas < 0.9f) dbeallas = 0.9f;
				else if(dbeallas > 2.0f) dbeallas = 2.0f;
				allapotszabalyozo();
				break;
			
			case OVERTAKING:
				switch(OT_STRAIGHT)
				{
					case OT_START:
						dbeallas=0.2f*speed+0.2f;
						if(dbeallas < 0.4f) dbeallas = 0.4f;
						else if(dbeallas > 3.0f) dbeallas = 3.0f;
						break;
					case OT_STRAIGHT:
						dbeallas=0.5f*speed+1.5f;
						if(dbeallas < 1.5f) dbeallas = 1.5f;
						else if(dbeallas > 3.0f) dbeallas = 3.0f;
						break;
					case OT_BACK:
						dbeallas=0.5f*speed+1.5f;
						if(dbeallas < 1.5f) dbeallas = 1.5f;
						else if(dbeallas > 3.0f) dbeallas = 3.0f;
						break;
				}
				/*dbeallas=0.5f*speed+1.5f;
				if(dbeallas < 1.5f) dbeallas = 1.5f;
				else if(dbeallas > 3.0f) dbeallas = 3.0f;*/
				allapotszabalyozo();
				break;
			
			case SAFETYCAR:
				dbeallas=0.375f*speed+0.375f;
				if(dbeallas < 0.9f) dbeallas = 0.9f;
				else if(dbeallas > 2.0f) dbeallas = 2.0f;
				allapotszabalyozo();
				break;
			
			case SPEED_RACE:
				dbeallas=dbeallasm*speed+dbeallaso;
				if(dbeallas < 0.9f) dbeallas = 0.9f;
				else if(dbeallas > 1.5f) dbeallas = 1.5f; // 2 volt
				allapotszabalyozo();			
				break;
				
			default:
				break;
		}
		
		
		if(servo_pwm>SERVO_MAX_PWM) servo_pwm=SERVO_MAX_PWM;
		else if(servo_pwm<SERVO_MIN_PWM)servo_pwm=SERVO_MIN_PWM;
		
		if(MANUAL_SERVO) servo_pwm = PWMINValue;
		
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, servo_pwm);  
    
		osDelay(5);
  }
  /* USER CODE END Start_Servo_Task */
}

void allapotszabalyozo(void)
{
		speedsi=speed;
		if(speedsi != 0)
		{
			tbeallas=dbeallas/speedsi;
			Tservo=tbeallas*kszi/3;
			skomplex=(1/Tservo)*sqrt(1-pow(kszi,2));
			svalos=-(1/Tservo)*kszi;
			kkozep=-(L/pow(speedsi,2))*(pow(svalos,2)+pow(skomplex,2));
			korientacio=(L/speedsi)*(2*svalos-speedsi*kkozep);
			phi=-kkozep*fo_vonal_hiba_SI-korientacio*szog_hiba_SI;
			servo_pwm=phi*57.2957795f*420/30+SERVO_MID_PWM;//rad to deg 57.2957795
		}
		
		servo_pwm_array[servo_array_index] = servo_pwm-SERVO_MID_PWM;
		servo_array_index++;
		if(servo_array_index >= SERVO_ARRAY_SIZE) servo_array_index = 0;
		
		egyenesen_megyunk_temp = TRUE;
		
		for(for_index_servo = 0;for_index_servo < SERVO_ARRAY_SIZE;for_index_servo++)
		{
			if(fabs(servo_pwm_array[for_index_servo]) > 40)
			{
				egyenesen_megyunk_temp = FALSE;
			}
		}
		
		
		egyenesen_megyunk = egyenesen_megyunk_temp;
		
}

