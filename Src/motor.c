#include "motor.h"

#define RADIOS_INDITAS TRUE
#define ENGEDELYEZES FALSE

// 0 ha automatában megy, és bármi más érték esetén RC távirányítóval mükszik
#define MANUAL_MOTOR 0
// 0 ha nem akarunk motor parancsot kiadni, és bármi más érték esetén normális mukodés
#define SWITCH_OFF_MOTOR 0

#define MAX_VALTASOK_SZAMA 25

#define MAX_MOTOR_PWM 2100
#define MIN_MOTOR_PWM 900

#define MAX_I_ERROR 500.0f

#define MOTOR_PWM_MID 1500
#define MOTOR_PWM_FWD 1570

#define SAFETY_KOVETES_tav	800			//mm-ben
#define SAFETY_KOVETES_P 		0.001f

uint16_t motor_pwm;
float MOTOR_P = 0,MOTOR_I = 0;
float MOTOR_P_flash = 0,MOTOR_I_flash = 0;//80,2 volt flash elott
float speed_error,desired_speed = 0.0f,actual_speed,accumulated_speed_error,accumulated_distance_error,distance_error;
uint32_t MOTOR_I_old_time,MOTOR_I_new_time;
float MOTOR_I_dt_s;
float MOTOR_P_ERROR,MOTOR_I_ERROR;
uint8_t Motor_Brake = 0,BRAKE_GYORS_LASSU = 0;
uint8_t max_speed = 0;

uint16_t safety_kovetesi_tavolsag = SAFETY_KOVETES_tav;
float safety_P,safety_I;

char RI_receive_message[5];

uint8_t RI_started = FALSE;
uint8_t first_run_motor=TRUE;

//SPEED_RACE
uint32_t enc_mm_now_motor;
uint32_t elozo_resz_valtas=0;

//Safety car
float safety_tavolsag_mm;

uint32_t state_speed_array[8][5][4] = { 
                     { {20000,5,1,0}, {0, 0,0,0}, {0, 0,0,0}, {0, 0,0,0}, {0, 0,0,0} },
                     { {0, 0,0, 0}, {0, 0,0, 0}, {0, 0,0, 0}, {0, 0,0, 0}, {0, 0,0, 0} },
                     { {0, 0,0, 0}, {0, 0,0, 0}, {0, 0,0, 0}, {0, 0,0, 0}, {0, 0,0, 0} },
                     { {0, 0,0, 0}, {0, 0,0, 0}, {0, 0,0, 0}, {0, 0,0, 0}, {0, 0,0, 0} },
                     { {0, 0,0, 0}, {0, 0,0, 0}, {0, 0,0, 0}, {0, 0,0, 0}, {0, 0,0, 0} },
                     { {0, 0,0, 0}, {0, 0,0, 0}, {0, 0,0, 0}, {0, 0,0, 0}, {0, 0,0, 0} },
                     { {0, 0,0, 0}, {0, 0,0, 0}, {0, 0,0, 0}, {0, 0,0, 0}, {0, 0,0, 0} },
                     { {0, 0,0, 0}, {0, 0,0, 0}, {0, 0,0, 0}, {0, 0,0, 0}, {0, 0,0, 0} }
                 };

uint8_t ugyes_gyors_valtas=FALSE,ugyes_lassu_valtas=FALSE;

void Start_Motor_Task(void const * argument)
{
	/*
	 * WAIT FOR RADIO START
	 */
	//ulTaskNotifyTake( pdTRUE,3000);
	if(RADIOS_INDITAS == TRUE )
	{
		do{
			osDelay(10);
			HAL_UART_Receive_IT(&huart1,(uint8_t*)RI_receive_message,1);
			ulTaskNotifyTake( pdTRUE,portMAX_DELAY);
		}while(RI_receive_message[0] != 0x30U && state_game != SAFETYCAR && state_game != SPEED_RACE  );
		
		if(state_game == SPEED_RACE)
		{
			osDelay(2000);
		}
	}

	RI_started = TRUE;
	/*
	 * WAIT FOR RADIO START
	 */
	
	
  /* USER CODE BEGIN Start_Motor_Task */
  /* Infinite loop */
	MOTOR_I_old_time = get_timer_us();
  for(;;)
  {
			
		enc_mm_now_motor = enc_mm_value_get();
		
		MOTOR_I_new_time = get_timer_us();
		MOTOR_I_dt_s = (MOTOR_I_new_time - MOTOR_I_old_time)* 0.000001f; // Másodpercben az idokulonbseg
		MOTOR_I_old_time = MOTOR_I_new_time;

		switch(state_game)
		{
			case LABYRINTH:
				MOTOR_P=40;
				MOTOR_I=120;
				if(ugyessegi_lassaban)
				{
					desired_speed = 0.90f;
					ugyes_lassu_valtas = TRUE;
				}
				else
				{
					if(szog_hiba_SI<0.1f) 
					{
						desired_speed = 1.6f;
						ugyes_gyors_valtas = TRUE;
					}
					else 
					{
						desired_speed = 1.2f;
						ugyes_lassu_valtas = TRUE;
					}						
				}
				if(ugyes_lassu_valtas && ugyes_gyors_valtas ){
					Motor_Brake = 100;
					accumulated_speed_error = 0;
				}
				if((speed-desired_speed) < 0.05f )
				{
					ugyes_lassu_valtas = FALSE;
					ugyes_gyors_valtas = FALSE;
					Motor_Brake = 0;
				}
				break;
			
			case LANE_CHANGE:
				MOTOR_P=40;
				MOTOR_I=120;
				if(atsoroltunk_mar || atsorolunk)	desired_speed = 0.8f;
				else desired_speed = 1.5f;
				first_run_motor=TRUE;
				
				break;
			
			case OVERTAKING:
				MOTOR_P=40;
				MOTOR_I=120;
				Motor_Brake = 0;
				switch(OT_STRAIGHT)
				{
					case OT_START:
						desired_speed = 1.3f;
						break;
					case OT_STRAIGHT:
						if(szog_hiba_SI<0.1f)	desired_speed = 3.5f;
						else desired_speed = 1.5f;
						break;
					case OT_BACK:
						desired_speed = 1.5f;
						break;
				}					
				
				break;
			
			case SAFETYCAR:
				MOTOR_P=40;
				MOTOR_I=120;
				safety_P=0.004f;
				safety_I = 0.00080f;
				safety_tavolsag_mm=SHARP_front_mm;
				
				distance_error = safety_tavolsag_mm-550.0f ;
				accumulated_distance_error += MOTOR_I_dt_s * distance_error*safety_I;
			
				if(accumulated_distance_error > 1.6f) accumulated_distance_error = 1.6f;
				else if(accumulated_distance_error < -1.6f) accumulated_distance_error = -1.6f;
				

				desired_speed = safety_P*distance_error + accumulated_distance_error;
				if(desired_speed > 1.7f && akt_vonalallapot == FAST) desired_speed = 1.7f;
				if(desired_speed > 1.05f && akt_vonalallapot == SLOW) desired_speed = 1.05f;
				if(elozes_allapot == OT_BACK)
				{
					switch(akt_vonalallapot)
					{
						case SLOW:
							desired_speed = 1.2f;
							break;
						case FAST:
							desired_speed = 1.6f;
							break;
					}
				}
				if(elozes_allapot != OT_BACK && SHARP_front_mm < 300)
				{
					if(accumulated_distance_error  < 0)
					{
						accumulated_distance_error = 0;
					}
					desired_speed = 0;
					Motor_Brake = 100;
				}
				else
				{
					Motor_Brake = 0;
				}	
				
				
				if(first_run_motor && SHARP_front_mm < 600)
				{
					if(speed<0.01f) 
					{
						first_run_motor=FALSE;
					}
					else
					{
						first_run_motor=TRUE;
					}
					accumulated_distance_error = 0;
					desired_speed = 0;
					Motor_Brake = 100;
					
				}
				
				break;
			
			case SPEED_RACE:	
				MOTOR_P=20;
				MOTOR_I=60;		
				if(first_run_motor){				
					first_run_motor=FALSE;
					desired_speed=palya[current_szakasz][reszek_szama_ind][1];
					max_speed=palya[current_szakasz][reszek_szama_ind][2];
					Motor_Brake = palya[current_szakasz][reszek_szama_ind][3];
					if(palya[current_szakasz][reszek_szama_ind][3] !=0) BRAKE_GYORS_LASSU = 1;
					else BRAKE_GYORS_LASSU = 0;
					
				}
				if(BRAKE_GYORS_LASSU && (speed-desired_speed) < 0.5f ){
					BRAKE_GYORS_LASSU = 0;
					Motor_Brake = 0;
					accumulated_speed_error = 50;
				}
				
				// Elso FASTítás(4 SLOW szakaszon)
				if(valtasokszama == 7)
				{
					palya[0][0][1]=4.5; // 1 FAST
					palya[1][0][1]=2; // 2 SLOW
					palya[2][0][1]=6.0; // 2 FAST
					palya[3][0][1]=2; // 2 SLOW
					palya[4][0][1]=5.0;		// 3 FAST
					palya[5][0][1]=1.8; // 3 SLOW
					palya[6][0][1]=3.5; // 4 FAST
					palya[7][0][1]=1.8; // 4 SLOW
				}
				
				// Második FASTítás(4 SLOW szakaszon)
				if(valtasokszama == 15)
				{
					palya[0][0][1]=5.5; 	// 1 FAST
					palya[1][0][1]=2; // 2 SLOW
					palya[2][0][1]=6.5;  // 2 FAST
					palya[3][0][1]=2; // 2 SLOW
					palya[4][0][1]=5.5;  // 3 FAST
					palya[5][0][1]=1.8; // 3 SLOW
					palya[6][0][1]=3.8;  // 4 FAST
					palya[7][0][1]=1.8; // 4 SLOW
				}
				
				if( (enc_mm_now_motor-elozo_resz_valtas) > palya[current_szakasz][reszek_szama_ind][0])
				{
					elozo_resz_valtas=enc_mm_now_motor;
					reszek_szama_ind++;
					desired_speed=palya[current_szakasz][reszek_szama_ind][1];
					max_speed=palya[current_szakasz][reszek_szama_ind][2];
					Motor_Brake = palya[current_szakasz][reszek_szama_ind][3];
					if(palya[current_szakasz][reszek_szama_ind][3] !=0) BRAKE_GYORS_LASSU = 1;
					else BRAKE_GYORS_LASSU = 0;
				}
				break;
				
			default:
				break;
		}
		
		/*
		 * MOTOR PI szabályzó algoritmusa
		 */
		actual_speed = speed;
		speed_error = desired_speed - actual_speed;
		

		
		accumulated_speed_error += MOTOR_I_dt_s * speed_error*MOTOR_I;
		if(accumulated_speed_error > MAX_I_ERROR) accumulated_speed_error = MAX_I_ERROR;
		else if(accumulated_speed_error < -MAX_I_ERROR) accumulated_speed_error = -MAX_I_ERROR;
		
		MOTOR_P_ERROR = MOTOR_P * speed_error;
		MOTOR_I_ERROR = accumulated_speed_error;
		
		motor_pwm = MOTOR_P_ERROR + MOTOR_I_ERROR + MOTOR_PWM_FWD; // MOTOR_PWM_FWD miatt mindig lesz fogaysztása a motornak

		// Korlátozás
		if(motor_pwm >= MAX_MOTOR_PWM) motor_pwm = MAX_MOTOR_PWM;
		if(motor_pwm <= MIN_MOTOR_PWM) motor_pwm = MIN_MOTOR_PWM;
		
		/*
		 * MOTOR PI szabályzó vége
		 */
		
		/*
		 * Vészfékezés vagy egyszeruen egy fékezés megvalósítása: 0 - 100 érték, ahol a 100 a max fékezés
		 * Itt lehet egy sebességet nézni és ha nulla, akkor megálltunk és nem kell tovább fékezni
		 */
		
		if(Motor_Brake > 0 && Motor_Brake <= 100) motor_pwm = MOTOR_PWM_MID - ((MOTOR_PWM_MID - MIN_MOTOR_PWM)/100 * Motor_Brake);
		
		/*
		 * Vészfékezés vége
		 */
		 if(max_speed)motor_pwm = MAX_MOTOR_PWM;
		/*
		 * RC távirányító engedélyezés, végso kódból kivenni a versenyen !!!
		*/
		if(ENGEDELYEZES)
		{
			if(PWMINValue < 1550 && PWMINValue > 1450) {
				motor_pwm = MIN_MOTOR_PWM; 
				accumulated_speed_error = 0;
			}
			else if(PWMINValue < 1450 || PWMINValue > 2500) {
				motor_pwm = MOTOR_PWM_MID;	 
				accumulated_speed_error = 0;
			}
		}
		
		/*
		 * RC távirányító engedélyezés, végso kódból kivenni a versenyen !!! END
		*/
		

		
		if(valtasokszama >= MAX_VALTASOK_SZAMA) 
		{
			motor_pwm = MIN_MOTOR_PWM;
			star_valtozik = TRUE;
		}
			
		if(MANUAL_MOTOR) motor_pwm = PWMINValue;
		if(SWITCH_OFF_MOTOR) motor_pwm = MOTOR_PWM_MID;
		
		// PWM jel kiadása
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor_pwm);  //Timer 1 -> Motor PWM kiadás, kitöltési tényezojének a megváltoztatása

		osDelay(8);
  }
  /* USER CODE END Start_Motor_Task */
}





