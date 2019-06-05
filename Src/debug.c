#include "debug.h"

#define max_get_message_size 2200
#define UART_COM TRUE
#define DEBUG_SWITCH TRUE
#define ARDUINO_SWITCH TRUE


float flash_data[FLASH_DATA_SIZE];
uint32_t adc_vago[96];
uint32_t flash_addr;
float flash_data_tmp;
uint8_t first_flash=0;
uint32_t addr_write = FLASH_USER_START_ADDR;

char debug_data[64];
uint32_t time_old_debug,time_new_debug,time_elapsed_debug;

char pcWriteBuffer[512];
char debug_message[4000];
char receive_message[max_get_message_size];
char receive_data[1];
char temp_msg[max_get_message_size];
uint8_t caseselect;
uint32_t msg_index,receive_index,tmp_index,tmp_whole_index;
uint32_t flash;

char arduino_message[200];

float motor_p_test,motor_i_test;

uint8_t first_run,first_run_arduino;
uint8_t brake_arduino=0;

uint8_t feny_allapota = FALSE,star_valtozik = TRUE,jirany_valtozik = TRUE,fek_valtozik = FALSE,birany_valtozik = TRUE;
uint8_t star_vege = FALSE;

uint32_t star_start_time = INT32_MAX;

void Start_Debug_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Debug_Task */
	first_run = 1;
	first_run_arduino = 1;
  /* Infinite loop */
  for(;;)
  {
		if(ARDUINO_SWITCH)
		{
			if(first_run_arduino)
			{
				osDelay(300);
				first_run_arduino = 0;
			}
			if(next_dir_change || jirany_valtozik || birany_valtozik)
			{
				if(next_dir_cross==1 ||jirany_valtozik) 
				{
					jirany_valtozik = FALSE;
					HAL_UART_Transmit_DMA(&huart5, (uint8_t*)arduino_message, sprintf(arduino_message, "jjindexQ"));
				}
				else
				{
					birany_valtozik = FALSE;
					HAL_UART_Transmit_DMA(&huart5, (uint8_t*)arduino_message, sprintf(arduino_message, "bbindexQ"));
				}
				next_dir_change = FALSE;
				ulTaskNotifyTake( pdTRUE,3000);
			}
			if((brake_arduino == 0 && Motor_Brake>0)) {

				HAL_UART_Transmit_DMA(&huart5, (uint8_t*)arduino_message, sprintf(arduino_message, "ffekQ"));
				brake_arduino=1;
				ulTaskNotifyTake( pdTRUE,3000);
			}
			else if((brake_arduino == 1 && Motor_Brake ==0)) {
				HAL_UART_Transmit_DMA(&huart5, (uint8_t*)arduino_message, sprintf(arduino_message, "ffekQ"));
				brake_arduino=0;
				ulTaskNotifyTake( pdTRUE,3000);
			}
			
			if(star_valtozik)
			{
				star_start_time = get_timer_us();
			}
			
			if(star_valtozik || star_vege) 
			{
				star_valtozik = FALSE;
				star_vege = FALSE;
				HAL_UART_Transmit_DMA(&huart5, (uint8_t*)arduino_message, sprintf(arduino_message, "sstarQ"));
				ulTaskNotifyTake( pdTRUE,3000);
			}
			
			if(( (int32_t) get_timer_us() - (int32_t) star_start_time) >10000000)
			{
				star_vege = TRUE;
				star_start_time = INT32_MAX;
			}
		}
		
		if(DEBUG_SWITCH)
		{
			//
			xSemaphoreTake(xMutexUART, portMAX_DELAY);
			// Adat-ra várás
			if( first_run !=1)
			{
				//osDelay(10);
				tmp_whole_index = 0;
				if(UART_COM)HAL_UART_Receive_DMA(&huart2,(uint8_t*)receive_message,1);
				else HAL_UART_Receive_DMA(&huart5,(uint8_t*)receive_message,1);
				ulTaskNotifyTake( pdTRUE,3000);
				caseselect =receive_message[0];
			}
			else
			{
				tmp_whole_index = 0;
				caseselect = 48;
				first_run = 0;
			}
			
			// Ido mérés kezdete, inicializálás
			msg_index = 0;
			time_old_debug = get_timer_us();
			
			
			// Run-time statistics
	//		vTaskGetRunTimeStats(pcWriteBuffer);
			
			// Saját üzenetek betöltése egy nagy karakter tömb-be,ami egybe lesz kiküldve -> 2000 bájt a limit!!!
			//UART2_message(debug_data, sprintf(debug_data, "CASE %d \n",caseselect));
			
			// Eset választás, a 0->48-nak felel meg (ASCII karaktertábla)
			switch(caseselect){
				case 48:
	//				// Run-time statistics
	//				UART2_message(pcWriteBuffer, 512);
	//				UART2_message(debug_data, sprintf(debug_data, "\n"));				
				
					// Labirintus adatok
					print_table();
					// Egyéb üzenetek
					UART2_message(debug_data, sprintf(debug_data, "CTBJ %d\n",cross_type_b_j));			
					UART2_message(debug_data, sprintf(debug_data, "TDEB %d\n",time_elapsed_debug));
					UART2_message(debug_data, sprintf(debug_data, "SPWM %d\n",__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_4)));
					UART2_message(debug_data, sprintf(debug_data, "MPWM %d\n",__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1)));
					UART2_message(debug_data, sprintf(debug_data, "SPEED %5.5f\n", speed));
					UART2_message(debug_data, sprintf(debug_data, "KDT %d\n", keresztezodes_dontes_fajta));
					UART2_message(debug_data, sprintf(debug_data, "IKS %d\n", konwncross));
					UART2_message(debug_data, sprintf(debug_data, "ENCF %4.2f\n", enc_mm_value_get()));
					UART2_message(debug_data, sprintf(debug_data, "PWMIN %d\n", PWMINValue));
					UART2_message(debug_data, sprintf(debug_data, "VSZ %d\n", vonalhiba_kulonbseg));
					UART2_message(debug_data, sprintf(debug_data, "LS %d\n", current_szakasz));
					UART2_message(debug_data, sprintf(debug_data, "THREELS %4.2f\n", HATSO_VONAL_SINCE));
					UART2_message(debug_data, sprintf(debug_data, "LM %4.2f\n", fo_vonal_hiba_SI));
					UART2_message(debug_data, sprintf(debug_data, "LM2 %4.2f\n", masodik_vonal_hiba));
					UART2_message(debug_data, sprintf(debug_data, "SZOGH %4.4f\n", szog_hiba_SI));
					UART2_message(debug_data, sprintf(debug_data, "CROSSE %d\n", elore_count));
					UART2_message(debug_data, sprintf(debug_data, "CROSSB %d\n", balra_count));
					UART2_message(debug_data, sprintf(debug_data, "CROSSJ %d\n", jobbra_count));
					UART2_message(debug_data, sprintf(debug_data, "TOF1 %4.2f\n", SHARP_front_mm));
					UART2_message(debug_data, sprintf(debug_data, "ULTRAF %4.2f\n", ultra_distance_mm));
					UART2_message(debug_data, sprintf(debug_data, "TOF2 %4.2f\n", SHARP_side_mm));
					UART2_message(debug_data, sprintf(debug_data, "TTOF %d\n", time_elapsed_TOF));
					UART2_message(debug_data, sprintf(debug_data, "VSTATE %d\n", state_game));
					UART2_message(debug_data, sprintf(debug_data, "DSPEED %4.2f\n", desired_speed));
					UART2_message(debug_data, sprintf(debug_data, "ESTATE %d\n", elozes_allapot));
					UART2_message(debug_data, sprintf(debug_data, "MBRAKE %d\n", Motor_Brake));
					UART2_message(debug_data, sprintf(debug_data, "EGYENESKEZD %4.2f\n", egyenes_elozes_kezdete));
					
					UART2_message(debug_data, sprintf(debug_data, "VALTASOKSZ %df\n", valtasokszama));
					
					
					UART2_message(debug_data, sprintf(debug_data, "ONELS %d\n", current_szakasz));
					UART2_message(debug_data, sprintf(debug_data, "TWOLS %d\n", reszek_szama_ind));
					
					UART2_message(debug_data, sprintf(debug_data, "EMEGYUNK %d\n", egyenesen_megyunk));
					
					// Inerciális szenzor értékei
					UART2_message(debug_data, sprintf(debug_data, "ITEMP %4.2f\n",szagatott_type));

					UART2_message(debug_data, sprintf(debug_data, "MAGX %d\n",szaggatott));
					UART2_message(debug_data, sprintf(debug_data, "USZMM %4.2f\n",utolso_szakasz_mm));
					
//					UART2_message(debug_data, sprintf(debug_data, "GYROCALIB %d\n",gyro_calib_u8));
//					UART2_message(debug_data, sprintf(debug_data, "ACCELCALIB %d\n",accel_calib_u8));
//					UART2_message(debug_data, sprintf(debug_data, "MAGCALIB %d\n",mag_calib_u8));
//					UART2_message(debug_data, sprintf(debug_data, "OP MODE %d\n",operation_mode_u8));
					UART2_message(debug_data, sprintf(debug_data, "LASTK %d\n",last_keresztezodes));
					UART2_message(debug_data, sprintf(debug_data, "LASTKX %d\n",last_lab_x));
					UART2_message(debug_data, sprintf(debug_data, "LASTKY %d\n",last_lab_y));
					UART2_message(debug_data, sprintf(debug_data, "SYSTEMCALIB %d\n",system_calib_u8));
					UART2_message(debug_data, sprintf(debug_data, "YAW %4.4f\n",yaw*(180/PI)));
					UART2_message(debug_data, sprintf(debug_data, "YAWT %4.4f\n",yaw_tmp*(180/PI)));
					UART2_message(debug_data, sprintf(debug_data, "COMRES %d\n",comres));
					UART2_message(debug_data, sprintf(debug_data, "SZA %4.2f\n",szagatott_atlag));
					//UART2_message(debug_data, sprintf(debug_data, "TINE %d\n",time_elapsed_inertial));
					UART2_message(debug_data, sprintf(debug_data, "PERROR %4.2f\n",MOTOR_P_ERROR));
					UART2_message(debug_data, sprintf(debug_data, "IERROR %4.2f\n",MOTOR_I_ERROR));
					UART2_message(debug_data, sprintf(debug_data, "MINDIST %d\n",min_dist));
					
					// ADC értékek kiküldése
					
					for(int8_t i = 0;i<48;i++){
						UART2_message(debug_data, sprintf(debug_data, "AF %d %d\n",i,szenzor_sor_front[i]));
						UART2_message(debug_data, sprintf(debug_data, "AB %d %d\n",i,szenzor_sor_back[i]));
					}
					UART2_message(debug_data, sprintf(debug_data, "TADC %d\n",time_elapsed_adc));
					UART2_message(debug_data, sprintf(debug_data, "TADCP %d\n",time_elapsed_adc_process));


					// MOTOR paraméterek
					UART2_message(debug_data, sprintf(debug_data, "MOTORP %4.2f\n",MOTOR_P));
					UART2_message(debug_data, sprintf(debug_data, "MOTORI %4.2f\n",MOTOR_I));
					UART2_message(debug_data, sprintf(debug_data, "Dbeallm %4.4f\n",dbeallasm));
					UART2_message(debug_data, sprintf(debug_data, "Dbeallo %4.4f\n",dbeallaso));
					//UART2_message(debug_data, sprintf(debug_data, "RIMSG %d\n",RI_receive_message[0]));s
					//UART2_message(debug_data, sprintf(debug_data, "RISTART %d\n",RI_started));
					
					//Pos infok
					UART2_message(debug_data, sprintf(debug_data, "POS %4.2f %4.2f\n",pos_x,pos_y));
					
					
					break;
					

				case 49:
					UART2_message(debug_data, sprintf(debug_data, "SPEED %5.5f\n", speed));
					break;
				
				case 50:
					if(UART_COM)HAL_UART_Receive_DMA(&huart2,(uint8_t*)receive_message,max_get_message_size);
					else HAL_UART_Receive_DMA(&huart5,(uint8_t*)receive_message,max_get_message_size);
					ulTaskNotifyTake( pdTRUE,portMAX_DELAY );
					UART2_message(receive_message,1000);		
					for(uint8_t ind=0;ind<FLASH_DATA_SIZE;ind++)
					{
						message_space_separation(max_get_message_size);
						if(tmp_index != 0) flash_addr = atol(temp_msg);
						message_space_separation(max_get_message_size);
						if(tmp_index != 0) flash_data_tmp = atof(temp_msg);	
						flash_data[flash_addr]=flash_data_tmp;					
					}
					Flash_variable(); // refresh variables		
					for(uint8_t ind=0;ind<96;ind++)
					{
						message_space_separation(max_get_message_size);
						if(tmp_index != 0) flash_addr = atol(temp_msg);
						message_space_separation(max_get_message_size);
						if(tmp_index != 0) flash_data_tmp = atol(temp_msg);	
						adc_vago[ind]=flash_data_tmp;					
					}				
					HAL_FLASH_Unlock();
					FLASH_Erase_Sector(FLASH_SECTOR_6,VOLTAGE_RANGE_3);
					HAL_FLASH_Lock();
					osDelay(10);
					addr_write= FLASH_USER_START_ADDR;
					if(first_flash==0){Flash_WriteWord(1);first_flash=1;}				
					for(uint8_t ind=0;ind<FLASH_DATA_SIZE;ind++)
					{
						Flash_WriteWord(*(uint32_t *)&flash_data[ind]);
						osDelay(1);
					}
					for(uint8_t ind=0;ind<96;ind++)
					{
						Flash_WriteWord(adc_vago[ind]);
						osDelay(1);
					}			
					//Check_Flash();
					
					/*
					message_space_separation(max_get_message_size);
					if(tmp_index != 0) motor_i_test = atof(temp_msg);
					
					UART2_message(debug_data, sprintf(debug_data, "SETMOTORP %3.3f\n",motor_p_test));
					UART2_message(debug_data, sprintf(debug_data, "SETMOTORI %3.3f\n",motor_i_test));
					*/
					break;
			}
				
			// Zárás jelzése
			UART2_message(debug_data, sprintf(debug_data, "BYTESENT %d\n",msg_index+2));
			UART2_message(debug_data, sprintf(debug_data, "\n\r"));
			
			// DMA-val uart üzenet kiküldése
			//print_table();
			if(UART_COM) HAL_UART_Transmit_DMA(&huart2, (uint8_t*)debug_message,  msg_index);
			else HAL_UART_Transmit_DMA(&huart5, (uint8_t*)debug_message,  msg_index);
			xSemaphoreGive(xMutexUART);		
			// Eltelt ido mérése és várakozás
			time_elapsed_debug =  get_timer_us() - time_old_debug;
			ulTaskNotifyTake( pdTRUE,portMAX_DELAY );
		}
		else
		{
			osDelay(100);
		}	
  }
	
  /* USER CODE END Start_Debug_Task */
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART5){
		xHigherPriorityTaskWokenDebug = pdFALSE;
		vTaskNotifyGiveFromISR(Debug_TaskHandle, &xHigherPriorityTaskWokenDebug );
		portYIELD_FROM_ISR(xHigherPriorityTaskWokenDebug);
	}	
	if (huart->Instance == USART2){
		xHigherPriorityTaskWokenDebug = pdFALSE;
		vTaskNotifyGiveFromISR(Debug_TaskHandle, &xHigherPriorityTaskWokenDebug );
		portYIELD_FROM_ISR(xHigherPriorityTaskWokenDebug);
	}
}
 
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART5){
		xHigherPriorityTaskWokenDebug = pdFALSE;
		vTaskNotifyGiveFromISR(Debug_TaskHandle, &xHigherPriorityTaskWokenDebug );
		portYIELD_FROM_ISR(xHigherPriorityTaskWokenDebug);
	}	
	if (huart->Instance == USART2){
		xHigherPriorityTaskWokenDebug = pdFALSE;
		vTaskNotifyGiveFromISR(Debug_TaskHandle, &xHigherPriorityTaskWokenDebug );
		portYIELD_FROM_ISR(xHigherPriorityTaskWokenDebug);
	}
	if (huart->Instance == USART1){
		xHigherPriorityTaskWokenDebugGet = pdFALSE;
		vTaskNotifyGiveFromISR(Motor_TaskHandle, &xHigherPriorityTaskWokenDebugGet );
		portYIELD_FROM_ISR(xHigherPriorityTaskWokenDebugGet);
	}
}

void UART2_message(char *debug_data,int32_t size)
{
	for(int16_t i = 0;i<size;i++)
	{
		debug_message[msg_index] = debug_data[i];
		msg_index++;
	}
}

void message_space_separation(int32_t size)
{
	tmp_index = 0;
	while(receive_message[tmp_whole_index] != 32 && tmp_whole_index < size){
		if(receive_message[tmp_whole_index] != 13){
			temp_msg[tmp_index] = receive_message[tmp_whole_index];
			tmp_index++;	
		}
		tmp_whole_index++;
	}
	tmp_whole_index++;
}

void Flash_variable(void)
{
	MOTOR_P_flash= flash_data[0];
	MOTOR_I_flash= flash_data[1];
	desired_speed = flash_data[2];
	dbeallasm = flash_data[3];
	dbeallaso = flash_data[4];
}

uint8_t Flash_WriteWord(uint32_t flash_data)
{
	uint8_t status = 1;
	HAL_FLASH_Unlock();

	if (addr_write < FLASH_USER_END_ADDR)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr_write, flash_data) == HAL_OK)
		{
			if(!(Flash_Read(addr_write) == flash_data)){
				status = 0;
			}
			addr_write = addr_write + 4;
		}
		else
		{
			status = 0;
		}
	}
	else{}
	HAL_FLASH_Lock();
	return status;
}

uint32_t Flash_Read(uint32_t addr)
{
	volatile uint32_t flash_word = *(volatile uint32_t*) addr;
	return flash_word;
}
