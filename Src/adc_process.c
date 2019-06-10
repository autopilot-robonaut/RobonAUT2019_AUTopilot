#include "adc_process.h"

#define KEZDO_GYORSASSAGI_ALLAPOT FAST
#define KEZDO_VERSENY_ALLAPOT LABYRINTH
 
#define MAX_VONAL_SZAM_GYORS 4
#define VONAL_TAVOLSAG_SZENZOR_SZAM_GYORS 2

#define MAX_VONAL_SZAM_UGYES 3
#define VONAL_TAVOLSAG_SZENZOR_SZAM_UGYES 2

#define FOV_szures 0.0f

#define TCRT5000_tavolsag 6.35f
#define SZENZORSOR_tavolsag 265.0f

#define ABLAK_MERET 3 // 7-es volt

#define CROSS_E_ARRAY 3
#define ELOZUNK_SAFTY TRUE




uint32_t elso_ugyessegi_res[48],hatso_ugyessegi_res[48];
uint32_t elso_ugyessegi_sum[48][ABLAK_MERET],hatso_ugyessegi_sum[48][ABLAK_MERET];
uint32_t elso_sum_tmp,hatso_sum_tmp;
uint8_t ablak_ind=0;

uint16_t vagoertek = 2000;

uint8_t fb_values[48];
uint8_t spi_data_fb[6]={0x00,0x00,0x00,0x00,0x00,0x00};

uint32_t fbled_time_before,fbled_time_now;

float fo_vonal_hiba,szog_hiba_SI,fo_vonal_hiba_mem,hatso_fo_vonal_hiba,hatso_fo_vonal_hiba_mem,fo_vonal_hiba_SI;
uint32_t max_ertek,max_index;
uint32_t VONALAK_SZAMA = 0,ELSO_VONALAK_SZAMA_MEM = 1,HATSO_VONALAK_SZAMA_MEM = 1;
float NO_LINE_SINCE = 0, THREE_LINE_SINCE = 0,TWO_LINE_SINCE = 0,ONE_LINE_SINCE = 0,FOUR_LINE_SINCE=0;
float HATSO_VONAL_SINCE=0;
uint32_t akt_vonalak_szama_elol;

uint32_t valtasokszama=0;

//Ügyességi keresztezodések
uint32_t last_cross_time=0;
float masodik_vonal_hiba;
uint8_t state_game = KEZDO_VERSENY_ALLAPOT;
uint32_t cross_type_b=0,cross_type_j=0,balra_count,jobbra_count=0,elore_count=0;
float cross_type_e=0;
float last_cross_encoder=0;
float vonalak[MAX_VONAL_SZAM_GYORS],hatso_vonalak[MAX_VONAL_SZAM_GYORS],vonal_hibak[MAX_VONAL_SZAM_GYORS];
uint32_t elso_szenzor_sor_process[48],hatso_szenzor_sor_process[48];
int8_t szaggatott=-1;

//Ügyessegi
float cross_type_e_array[CROSS_E_ARRAY];
float szagatott_type;
float cross_type_e_average;
float safety_elozes_elott_mm = 0;
uint8_t cross_type_e_array_index = 0;
int8_t next_dir_cross=-1;
uint8_t kovetni_kell=FALSE;
uint8_t next_dir_change=FALSE;

uint8_t blink=1;

enum keresztezodes last_keresztezodes;

uint8_t lehet_uj_keresztezodes = TRUE,vege_az_egyenesnek = FALSE,vege_a_szetvalasnak = FALSE,atsorolas_hatul_nullaztuk = FALSE;
uint8_t keresztezodes_dontes_fajta;

//SPEED_RACE
float utolso_szakasz_mm=0;
float palya[SZAKASZOK_SZAMA][MAX_RESZEK_SZAMA_SZAKASZBAN][ADATOK_SZAMA]={{
	{100000,4,-1,0}		,{-1,-1,-1,-1}			,{-1,-1,-1,-1}},   			// 1 FAST SZAKASZ
	{{100000,1.8,-1,100}	,{20000,1.7,-1,100}	,{-1,-1,-1,-1}},  		// 1 SLOW SZAKASZ
	{{100000,4,-1,0}	,{-1,-1,-1,-1}			,{-1,-1,-1,-1}},  			// 2 FAST SZAKASZ
	{{100000,1.8,-1,100}	,{7000,1.7,-1,100}	,{20000,1.5,-1,100}},		// 2 SLOW SZAKASZ
	{{100000,5,-1,0}	,{-1,-1,-1,-1}			,{-1,-1,-1,-1}},  			// 3 FAST SZAKASZ
	{{100000,1.8,-1,100}	,{20000,1.7,-1,100}	,{-1,-1,-1,-1}},			// 3 SLOW SZAKASZ
	{{100000,3.5,-1,0}	,{-1,-1,-1,-1}			,{-1,-1,-1,-1}},   			// 4 FAST SZAKASZ
	{{100000,1.5,-1,100}	,{7000,1.7,-1,100}	,{20000,1.5,-1,100}}};	// 4 SLOW SZAKASZ

//		float palya[SZAKASZOK_SZAMA][MAX_RESZEK_SZAMA_SZAKASZBAN][ADATOK_SZAMA]={{{100000,10,-1,0},
//		{-1,-1,-1,-1},{-1,-1,-1,-1}},{{500,10,-1,0},{20000,2.0,-1,100},{-1,-1,-1,-1}},
//		{{100000,10,-1,0},{-1,-1,-1,-1},{-1,-1,-1,-1}},{{500,10,-1,0},{7000,1.9,-1,100},
//		{20000,1.7,0,100}},{{100000,10,-1,0},{-1,-1,-1,-1},{-1,-1,-1,-1}},{{500,10,-1,0},
//		{20000,2.0,-1,100},{-1,-1,-1,-1}},{{100000,10,1,0},{-1,-1,-1,-1},{-1,-1,-1,-1}},
//		{{500,10,-1,0},{7000,1.9,-1,100},{20000,1.7,-1,100}}};
uint8_t reszek_szama_ind=0;
int32_t szakaszok[SZAKASZOK_SZAMA]={5000,4500,6500,7500,7000,7000,6000,4000};
uint8_t current_szakasz=0;
uint8_t current_szakasz_safty=0;
uint8_t cross_type_b_j=0;
//int8_t cross_dir_from=-1;


float enc_mm_before,enc_mm_now,enc_mm_between,elozo_vonal_valtas_mm;
float last_j_b_cross=1000000;
uint8_t last_j_b_dir=0;
enum section_state akt_vonalallapot = KEZDO_GYORSASSAGI_ALLAPOT;

uint8_t vonal_tipus_memoria[10];

uint32_t time_old_adc_process,time_new_adc_process,time_elapsed_adc_process;

uint8_t ugyessegi_lassaban = FALSE,iranyfeny_vilagithat=FALSE,uj_keresztezodes_vilagithat=FALSE;
float szagatott_atlag=0;
float szog_keresztezodes_elott=0,yaw_tmp;

//Átsorolás
uint8_t ugyessegin_vagyunk = TRUE,atsorolunk = FALSE,atsorolas_hatul_nullaztunk=FALSE;
uint8_t nem_latunk_vonalat_egy_ideig_elol = FALSE,nem_latunk_vonalat_egy_ideig_hatul = FALSE,atsoroltunk_mar = FALSE;
float enc_ugyessegi_vege,enc_gyorsassagi_kezdete;
float NO_LINE_SINCE_HATUL = 0;
float atsorolasi_hiba_mm;

//OVERTAKING
float elozes_elotti_szog=PI;
float elozes_posx=0;
float elozes_posy=0;
enum overtaking_state elozes_allapot = OT_START;
float egyenes_elozes_kezdete=0;
float kirantas_szog_kulonbseg;
uint8_t vonalhiba_kulonbseg=0;

uint8_t gyorsasagival_kezdunk = FALSE;

void Start_Calculate_Task(void const * argument){
  /* USER CODE BEGIN Start_Calculate_Task */
	
	uint8_t init_led=0;
	HAL_SPI_Transmit(&hspi3,&init_led,1,0xFFFF);
	HAL_SPI_Transmit(&hspi3,&init_led,1,0xFFFF);
	HAL_SPI_Transmit(&hspi3,&init_led,1,0xFFFF);
	HAL_SPI_Transmit(&hspi3,&init_led,1,0xFFFF);
	HAL_SPI_Transmit(&hspi3,&init_led,1,0xFFFF);
	HAL_SPI_Transmit(&hspi3,&init_led,1,0xFFFF);
	osDelay(500);
	for(uint8_t wait_index = 0;wait_index < 5;wait_index++)
	{
		ulTaskNotifyTake( pdTRUE,50);
	}

	if(SHARP_front_mm < 500)
	{
		state_game=SAFETYCAR;
		blink = 2;
	}
	
	osDelay(3000);
	if(gyorsasagival_kezdunk)
	{
		state_game=SPEED_RACE;
		blink = 3;
	}
	xTaskNotify(Motor_TaskHandle,0,eNoAction);
	//state_game = LANE_CHANGE;
  /* Infinite loop */
	if(state_game == SAFETYCAR)
	{
		ONE_LINE_SINCE=1250;
	}
  for(;;)
  {
		// Megvárni a az ADC kiolvasást
		ulTaskNotifyTake( pdTRUE,50);
		//adc_proc_ugyessegi();
		time_old_adc_process = get_timer_us();
		
		/*
		 * Vonal információk kinyerése a szenzor adatokból
		 */
		switch(state_game)
		{
			case LABYRINTH:
				ugyessegi_adc_process(MAX_VONAL_SZAM_UGYES,VONAL_TAVOLSAG_SZENZOR_SZAM_UGYES);
				break;
			
			case LANE_CHANGE:
				atsorolas_adc_process(MAX_VONAL_SZAM_UGYES,VONAL_TAVOLSAG_SZENZOR_SZAM_UGYES);
				break;
			
			case OVERTAKING:			
				elozes_adc_process(MAX_VONAL_SZAM_GYORS,VONAL_TAVOLSAG_SZENZOR_SZAM_GYORS);
				break;
			
			case SAFETYCAR:
				safetycar_adc_process(MAX_VONAL_SZAM_GYORS,VONAL_TAVOLSAG_SZENZOR_SZAM_GYORS);
				break;
			
			case SPEED_RACE:
				gyorsassagi_adc_process(MAX_VONAL_SZAM_GYORS,VONAL_TAVOLSAG_SZENZOR_SZAM_GYORS);				
				break;
				
			default:
				break;
		}	
		/*
		 * Vonal információk kinyerése a szenzor adatokból VÉGE
		 */
		
		/* 
		 * Feedback LED vilagitas BEGIN
		 */
		
		fbled_time_now = get_timer_us();
		// 100 msec frissitjük a Feedback LEDeket -> Nem terheli le a procit
		if(fbled_time_now > (fbled_time_before+20000)){
			if(blink)
			{
				uint8_t blink_mem=255;
				HAL_SPI_Transmit(&hspi3,&blink_mem,1,0xFFFF);
				HAL_SPI_Transmit(&hspi3,&blink_mem,1,0xFFFF);
				HAL_SPI_Transmit(&hspi3,&blink_mem,1,0xFFFF);
				HAL_SPI_Transmit(&hspi3,&blink_mem,1,0xFFFF);
				HAL_SPI_Transmit(&hspi3,&blink_mem,1,0xFFFF);
				HAL_SPI_Transmit(&hspi3,&blink_mem,1,0xFFFF);
				osDelay(300);
				blink_mem=0;
				HAL_SPI_Transmit(&hspi3,&blink_mem,1,0xFFFF);
				HAL_SPI_Transmit(&hspi3,&blink_mem,1,0xFFFF);
				HAL_SPI_Transmit(&hspi3,&blink_mem,1,0xFFFF);
				HAL_SPI_Transmit(&hspi3,&blink_mem,1,0xFFFF);
				HAL_SPI_Transmit(&hspi3,&blink_mem,1,0xFFFF);
				HAL_SPI_Transmit(&hspi3,&blink_mem,1,0xFFFF);
				osDelay(300);
				blink--;
			}
			else
			{
				// Elso szenzosor feldolgozása
				for(uint8_t j = 0;j<48;j++){
					if(elso_szenzor_sor_process[j]==1)
					{
						fb_values[j] = 1;
					}
					else
					{
						fb_values[j] = 0;
					}
				}
				/*
				if(iranyfeny_vilagithat)
				{
					switch(last_keresztezodes)
					{
						case 0:
							break;
						case 1:
							for(uint8_t j = 0;j<6;j++){
								fb_values[j] = 1;
							}
							break;
						case 2:
							for(uint8_t j = 42;j<48;j++){
								fb_values[j] = 1;
							}
							break;
					}
				}
					if(newcross && uj_keresztezodes_vilagithat)
					{
						for(uint8_t j = 38;j<40;j++){
								fb_values[j] = 1;
						}
						for(uint8_t j = 8;j<10;j++){
								fb_values[j] = 1;
						}
					}
				*/
				
				if(comres>-1)
				{
					// SPI adat eloallitása
					spi_data_fb[0]=0;spi_data_fb[1]=0;spi_data_fb[2]=0;spi_data_fb[3]=0;spi_data_fb[4]=0;spi_data_fb[5]=0;
					for(uint8_t fbcount=0;fbcount<48;fbcount++){
						if(fbcount<8){
							spi_data_fb[5]+=fb_values[fbcount]*pow(2,fbcount);
						}
						else if(fbcount<16){
							spi_data_fb[4]+=fb_values[fbcount]*pow(2,fbcount-8);
						}
						else if(fbcount<24){
							spi_data_fb[3]+=fb_values[fbcount]*pow(2,fbcount-16);
						}
						else if(fbcount<32){
							spi_data_fb[2]+=fb_values[fbcount]*pow(2,fbcount-24);
						}
						else if(fbcount<40){
							spi_data_fb[1]+=fb_values[fbcount]*pow(2,fbcount-32);
						}
						else{
							spi_data_fb[0]+=fb_values[fbcount]*pow(2,fbcount-40);
						}
					}
					// SPI adat kiküldése
					HAL_SPI_Transmit(&hspi3,(uint8_t *)&spi_data_fb[0],6,0xFFFF);
					fbled_time_before = fbled_time_now;

				}
				else 
				{
					uint8_t nincs_yaw=255;
					HAL_SPI_Transmit(&hspi3,&nincs_yaw,1,0xFFFF);
					HAL_SPI_Transmit(&hspi3,&nincs_yaw,1,0xFFFF);
					HAL_SPI_Transmit(&hspi3,&nincs_yaw,1,0xFFFF);
					HAL_SPI_Transmit(&hspi3,&nincs_yaw,1,0xFFFF);
					HAL_SPI_Transmit(&hspi3,&nincs_yaw,1,0xFFFF);
					HAL_SPI_Transmit(&hspi3,&nincs_yaw,1,0xFFFF);
				}
			}
		}
		/* 
		 * Feedback LED vilagitas END
		 */
		
		time_elapsed_adc_process = get_timer_us() - time_old_adc_process;	
		
  }
  /* USER CODE END Start_Calculate_Task */
}

void search_max(uint32_t *values,uint32_t size,uint32_t *max_ertek,uint32_t *max_index,uint8_t type){
	*max_ertek = 0;
	*max_index = 0;
	
	for(uint32_t index = 0;index < size;index++){
		if(type == 0){
			if((values[index] > *max_ertek) && (values[index] > adc_vago[index])){
				*max_ertek = values[index];
				*max_index = index;
			}
		}
		else
		{
			if((values[index] > *max_ertek) && (values[index] > adc_vago[48+index])){
				*max_ertek = values[index];
				*max_index = index;
			}
		}
	}
}

void Swap(float *a,float *b){
	float temp_a;
	temp_a = *a;
	*a = *b;
	*b = temp_a;
}

void gyorsassagi_adc_process(uint8_t max_vonalszam,uint8_t max_vonaltavolsag_szam){
		memcpy(elso_szenzor_sor_process,szenzor_sor_front,sizeof(float)*48);
		memcpy(hatso_szenzor_sor_process,szenzor_sor_back,sizeof(float)*48);

	
		VONALAK_SZAMA = 0;
		for(uint16_t vonal_index = 0;vonal_index < max_vonalszam;vonal_index++) {
			search_max(elso_szenzor_sor_process,48,&max_ertek,&max_index,0);
			if(max_ertek >= adc_vago[max_index])
			{
				VONALAK_SZAMA++;
				vonalak[vonal_index] = (float)max_index;
				elso_szenzor_sor_process[max_index] = 1;
				for(uint8_t index = 0;index < max_vonaltavolsag_szam;index++){
					if((max_index+index+1) <= 47) elso_szenzor_sor_process[max_index+index+1] = 0;
					if((int8_t)((int8_t)max_index-(int8_t)index-1) >= 0) elso_szenzor_sor_process[max_index-index-1] = 0;
				}
			}
		}
		
		akt_vonalak_szama_elol = VONALAK_SZAMA;
		
		// Encoder értékbol mm különbséget kinyerni
		enc_mm_now = enc_mm_value_get();
		enc_mm_between = enc_mm_now - enc_mm_before;
		enc_mm_before = enc_mm_now;
		
		// Vonal váltások detektálása		
		if(VONALAK_SZAMA == 3  && (enc_mm_now - elozo_vonal_valtas_mm )> szakaszok[current_szakasz] && ONE_LINE_SINCE > 1200 && THREE_LINE_SINCE >30){
			THREE_LINE_SINCE = 0;
			ONE_LINE_SINCE = 0;
			utolso_szakasz_mm=enc_mm_now-elozo_vonal_valtas_mm;
			elozo_vonal_valtas_mm = enc_mm_now;			
			valtasokszama++;
			//palyaprogramozas miatt
			current_szakasz++;
			if(current_szakasz==SZAKASZOK_SZAMA) current_szakasz=0;
			reszek_szama_ind=0;
			elozo_resz_valtas=enc_mm_now;
			desired_speed=palya[current_szakasz][reszek_szama_ind][1];
			max_speed=palya[current_szakasz][reszek_szama_ind][2];
			Motor_Brake = palya[current_szakasz][reszek_szama_ind][3];
			if(palya[current_szakasz][reszek_szama_ind][3] !=0) BRAKE_GYORS_LASSU = 1;
			else BRAKE_GYORS_LASSU = 0;
			// Motor I tag nullazása, sebességváltáskor
		}
		
		// Elso vonal tipusa és fo vonal kiszamolasa
		if(VONALAK_SZAMA == 1){		
			ONE_LINE_SINCE += enc_mm_between;	
			if(ONE_LINE_SINCE>50)THREE_LINE_SINCE=0;
			NO_LINE_SINCE=0;

			fo_vonal_hiba = vonalak[0] - 24;
		}
		else if(VONALAK_SZAMA == 2){
			TWO_LINE_SINCE += enc_mm_between;
			NO_LINE_SINCE=0;
			// Vonal hibák kiszámolása
			vonal_hibak[0] = vonalak[0] - 24;
			vonal_hibak[1] = vonalak[1] - 24;
			fo_vonal_hiba = (vonal_hibak[0]+vonal_hibak[1])/2.0f;
		}
		else if(VONALAK_SZAMA == 3){
			THREE_LINE_SINCE += enc_mm_between;
			NO_LINE_SINCE=0;
			
			// Vonal hibák kiszámolása
			vonal_hibak[0] = vonalak[0] - 24;
			vonal_hibak[1] = vonalak[1] - 24;
			vonal_hibak[2] = vonalak[2] - 24;

			
			// atlagnal kovetjuk
			fo_vonal_hiba = (vonal_hibak[0]+vonal_hibak[1]+vonal_hibak[2])/3.0f;
		}
		else if(VONALAK_SZAMA ==4){
			vonal_hibak[0] = vonalak[0] - 24;
			vonal_hibak[1] = vonalak[1] - 24;
			vonal_hibak[2] = vonalak[2] - 24;
			vonal_hibak[3] = vonalak[3] - 24;
			FOUR_LINE_SINCE+=enc_mm_between;
			NO_LINE_SINCE=0;
			fo_vonal_hiba = (vonal_hibak[0]+vonal_hibak[1]+vonal_hibak[2]+vonal_hibak[3])/4.0f;
		}
		else {
			NO_LINE_SINCE += enc_mm_between;
			if(NO_LINE_SINCE>3000){Motor_Brake=100;}
		}
		
		fo_vonal_hiba_mem = fo_vonal_hiba;
		ELSO_VONALAK_SZAMA_MEM = VONALAK_SZAMA;
		
		// Hátsó szenzorsor feldolgozása
		VONALAK_SZAMA = 0;
		for(uint16_t vonal_index = 0;vonal_index < max_vonalszam;vonal_index++){
			search_max(hatso_szenzor_sor_process,48,&max_ertek,&max_index,1);
			if(max_ertek >= adc_vago[48+max_index])
			{
				VONALAK_SZAMA++;
				vonalak[vonal_index] = (float)max_index;
				hatso_szenzor_sor_process[max_index] = 0;
				for(uint8_t index = 0;index < max_vonaltavolsag_szam;index++){
					if((max_index+index+1) <= 47) hatso_szenzor_sor_process[max_index+index+1] = 0;
					if((int8_t)((int8_t)max_index-(int8_t)index-1) >= 0) hatso_szenzor_sor_process[max_index-index-1] = 0;
				}
			}
		}
		
		
		if(VONALAK_SZAMA == 1){		
			// Egyszeru
			hatso_fo_vonal_hiba = vonalak[0] - 24;
		}
		else if(VONALAK_SZAMA == 2){
			// Vonal hibák kiszámolása
			vonal_hibak[0] = vonalak[0] - 24;
			vonal_hibak[1] = vonalak[1] - 24;
			
			hatso_fo_vonal_hiba = (vonal_hibak[0] + vonal_hibak[1])/2;
		}
		else if(VONALAK_SZAMA == 3){
			// Vonal hibák kiszámolása
			vonal_hibak[0] = vonalak[0] - 24;
			vonal_hibak[1] = vonalak[1] - 24;
			vonal_hibak[2] = vonalak[2] - 24;

			// Kozepso vonal kivalasztasa
			hatso_fo_vonal_hiba = (vonal_hibak[0]+vonal_hibak[1]+vonal_hibak[2])/3.0f;
		}
		else if(VONALAK_SZAMA ==4){
			vonal_hibak[0] = vonalak[0] - 24;
			vonal_hibak[1] = vonalak[1] - 24;
			vonal_hibak[2] = vonalak[2] - 24;
			vonal_hibak[3] = vonalak[3] - 24;
			FOUR_LINE_SINCE+=enc_mm_between;
			NO_LINE_SINCE=0;
			hatso_fo_vonal_hiba = (vonal_hibak[0]+vonal_hibak[1]+vonal_hibak[2]+vonal_hibak[3])/4.0f;
		}
		
		hatso_fo_vonal_hiba_mem = hatso_fo_vonal_hiba;
		
		// Vonal hiba és szöghiba SI-ben(m,radian)
		fo_vonal_hiba_SI = fo_vonal_hiba*TCRT5000_tavolsag*0.001f;
		if(ELSO_VONALAK_SZAMA_MEM !=0 && VONALAK_SZAMA != 0) szog_hiba_SI = ((fo_vonal_hiba - hatso_fo_vonal_hiba)*TCRT5000_tavolsag)/SZENZORSOR_tavolsag;
}

void safetycar_adc_process(uint8_t max_vonalszam,uint8_t max_vonaltavolsag_szam){
		memcpy(elso_szenzor_sor_process,szenzor_sor_front,sizeof(float)*48);
		memcpy(hatso_szenzor_sor_process,szenzor_sor_back,sizeof(float)*48);

	
		VONALAK_SZAMA = 0;
		for(uint16_t vonal_index = 0;vonal_index < max_vonalszam;vonal_index++) {
			search_max(elso_szenzor_sor_process,48,&max_ertek,&max_index,0);
			if(max_ertek >= adc_vago[max_index])
			{
				VONALAK_SZAMA++;
				vonalak[vonal_index] = (float)max_index;
				elso_szenzor_sor_process[max_index] = 1;
				for(uint8_t index = 0;index < max_vonaltavolsag_szam;index++){
					if((max_index+index+1) <= 47) elso_szenzor_sor_process[max_index+index+1] = 0;
					if((int8_t)((int8_t)max_index-(int8_t)index-1) >= 0) elso_szenzor_sor_process[max_index-index-1] = 0;
				}
			}
		}
		
		akt_vonalak_szama_elol = VONALAK_SZAMA;
		
		// Encoder értékbol mm különbséget kinyerni
		enc_mm_now = enc_mm_value_get();
		enc_mm_between = enc_mm_now - enc_mm_before;
		enc_mm_before = enc_mm_now;
		
		// Vonal váltások detektálása		
		if(VONALAK_SZAMA == 3 && ONE_LINE_SINCE > 1200 && THREE_LINE_SINCE >30 ){
			THREE_LINE_SINCE = 0;
			ONE_LINE_SINCE = 0;
			utolso_szakasz_mm=enc_mm_now-elozo_vonal_valtas_mm;
			elozo_vonal_valtas_mm = enc_mm_now;			
			valtasokszama++;
			current_szakasz_safty++;

			// Motor I tag nullazása, sebességváltáskor
			//accumulated_speed_error = 0;
			
			switch(akt_vonalallapot)
			{
				case SLOW:
					akt_vonalallapot = FAST;
					break;
				case FAST:
					akt_vonalallapot = SLOW;
				  BRAKE_GYORS_LASSU = 1;

					break;
				default:
					break;
			}
		}
		
		// Elso vonal tipusa és fo vonal kiszamolasa
		if(VONALAK_SZAMA == 1){		
			ONE_LINE_SINCE += enc_mm_between;	
			if(ONE_LINE_SINCE>50)THREE_LINE_SINCE=0;
			NO_LINE_SINCE=0;

			fo_vonal_hiba = vonalak[0] - 24;
		}
		else if(VONALAK_SZAMA == 2){
			TWO_LINE_SINCE += enc_mm_between;
			NO_LINE_SINCE=0;
			// Vonal hibák kiszámolása
			vonal_hibak[0] = vonalak[0] - 24;
			vonal_hibak[1] = vonalak[1] - 24;
			fo_vonal_hiba = (vonal_hibak[0]+vonal_hibak[1])/2.0f;
		}
		else if(VONALAK_SZAMA == 3){
			THREE_LINE_SINCE += enc_mm_between;
			NO_LINE_SINCE=0;
			
			// Vonal hibák kiszámolása
			vonal_hibak[0] = vonalak[0] - 24;
			vonal_hibak[1] = vonalak[1] - 24;
			vonal_hibak[2] = vonalak[2] - 24;
			
			
			// atlagnal kovetjuk
			fo_vonal_hiba = (vonal_hibak[0]+vonal_hibak[1]+vonal_hibak[2])/3.0f;
		}
		else if(VONALAK_SZAMA ==4){
			vonal_hibak[0] = vonalak[0] - 24;
			vonal_hibak[1] = vonalak[1] - 24;
			vonal_hibak[2] = vonalak[2] - 24;
			vonal_hibak[3] = vonalak[3] - 24;
			FOUR_LINE_SINCE+=enc_mm_between;
			NO_LINE_SINCE=0;
			fo_vonal_hiba = (vonal_hibak[0]+vonal_hibak[1]+vonal_hibak[2]+vonal_hibak[3])/4.0f;
		}
		else {
			NO_LINE_SINCE += enc_mm_between;
			if(NO_LINE_SINCE>3000){Motor_Brake=100;}
		}
		
		fo_vonal_hiba_mem = fo_vonal_hiba;
		ELSO_VONALAK_SZAMA_MEM = VONALAK_SZAMA;
		
		// Hátsó szenzorsor feldolgozása
		VONALAK_SZAMA = 0;
		for(uint16_t vonal_index = 0;vonal_index < max_vonalszam;vonal_index++){
			search_max(hatso_szenzor_sor_process,48,&max_ertek,&max_index,1);
			if(max_ertek >= adc_vago[48+max_index])
			{
				VONALAK_SZAMA++;
				vonalak[vonal_index] = (float)max_index;
				hatso_szenzor_sor_process[max_index] = 0;
				for(uint8_t index = 0;index < max_vonaltavolsag_szam;index++){
					if((max_index+index+1) <= 47) hatso_szenzor_sor_process[max_index+index+1] = 0;
					if((int8_t)((int8_t)max_index-(int8_t)index-1) >= 0) hatso_szenzor_sor_process[max_index-index-1] = 0;
				}
			}
		}
		
		
		if(VONALAK_SZAMA == 1){		
			// Egyszeru
			hatso_fo_vonal_hiba = vonalak[0] - 24;
		}
		else if(VONALAK_SZAMA == 2){
			// Vonal hibák kiszámolása
			vonal_hibak[0] = vonalak[0] - 24;
			vonal_hibak[1] = vonalak[1] - 24;
			
			hatso_fo_vonal_hiba = (vonal_hibak[0] + vonal_hibak[1])/2;
		}
		else if(VONALAK_SZAMA == 3){
			// Vonal hibák kiszámolása
			vonal_hibak[0] = vonalak[0] - 24;
			vonal_hibak[1] = vonalak[1] - 24;
			vonal_hibak[2] = vonalak[2] - 24;
			
			// Kozepso vonal kivalasztasa
			hatso_fo_vonal_hiba = (vonal_hibak[0]+vonal_hibak[1]+vonal_hibak[2])/3.0f;
		}
		else if(VONALAK_SZAMA ==4){
			vonal_hibak[0] = vonalak[0] - 24;
			vonal_hibak[1] = vonalak[1] - 24;
			vonal_hibak[2] = vonalak[2] - 24;
			vonal_hibak[3] = vonalak[3] - 24;
			FOUR_LINE_SINCE+=enc_mm_between;
			NO_LINE_SINCE=0;
			hatso_fo_vonal_hiba = (vonal_hibak[0]+vonal_hibak[1]+vonal_hibak[2]+vonal_hibak[3])/4.0f;
		}
		
		hatso_fo_vonal_hiba_mem = hatso_fo_vonal_hiba;
		if(current_szakasz_safty>7 && SHARP_front_mm>600)
		{
			valtasokszama=0;
			state_game=SPEED_RACE;
			first_run_motor = TRUE;
		}
		if(ELOZUNK_SAFTY && current_szakasz_safty == 4  && safety_elozes_elott_mm ==0 )
		{
			safety_elozes_elott_mm = enc_mm_now;
		}
		if(ELOZUNK_SAFTY && current_szakasz_safty == 4 && fabs(fo_vonal_hiba)<3 && fabs(szog_hiba_SI)<0.05 && (enc_mm_now - safety_elozes_elott_mm) < 3000 && (enc_mm_now - safety_elozes_elott_mm) > 1000)
		{
			elozes_elotti_szog=yaw;
			state_game=OVERTAKING;
			elozes_posx=0;
			elozes_posy=0;
			birany_valtozik = TRUE;
		}
		
		// Vonal hiba és szöghiba SI-ben(m,radian)
		fo_vonal_hiba_SI = fo_vonal_hiba*TCRT5000_tavolsag*0.001f;
		if(ELSO_VONALAK_SZAMA_MEM !=0 && VONALAK_SZAMA != 0) szog_hiba_SI = ((fo_vonal_hiba - hatso_fo_vonal_hiba)*TCRT5000_tavolsag)/SZENZORSOR_tavolsag;
}

void ugyessegi_adc_process(uint8_t max_vonalszam,uint8_t max_vonaltavolsag_szam){
		memcpy(elso_szenzor_sor_process,szenzor_sor_front,sizeof(float)*48);
		memcpy(hatso_szenzor_sor_process,szenzor_sor_back,sizeof(float)*48);
		for(uint8_t i=0;i<48;i++) {
			elso_ugyessegi_sum[i][ablak_ind]=elso_szenzor_sor_process[i];
			hatso_ugyessegi_sum[i][ablak_ind]=hatso_szenzor_sor_process[i];
		}
		ablak_ind++;
		if(ablak_ind==ABLAK_MERET) ablak_ind=0;

		for(uint8_t i=0;i<48;i++) {
			elso_sum_tmp=0; hatso_sum_tmp=0;
			for(uint8_t j=0;j<ABLAK_MERET;j++)
			{
				elso_sum_tmp+=elso_ugyessegi_sum[i][j];
				hatso_sum_tmp+=hatso_ugyessegi_sum[i][j];
			}
			elso_szenzor_sor_process[i]=elso_sum_tmp/ABLAK_MERET;
			hatso_szenzor_sor_process[i]=hatso_sum_tmp/ABLAK_MERET;
		}
		VONALAK_SZAMA = 0;
		for(uint16_t vonal_index = 0;vonal_index < max_vonalszam;vonal_index++) {
			search_max(elso_szenzor_sor_process,48,&max_ertek,&max_index,0);
			if(max_ertek >= adc_vago[max_index])
			{
				VONALAK_SZAMA++;
				vonalak[vonal_index] = (float)max_index;
				elso_szenzor_sor_process[max_index] = 1;
				for(uint8_t index = 0;index < max_vonaltavolsag_szam;index++){
					if((max_index+index+1) <= 47) elso_szenzor_sor_process[max_index+index+1] = 0;
					if((int8_t)((int8_t)max_index-(int8_t)index-1) >= 0) elso_szenzor_sor_process[max_index-index-1] = 0;
				}
			}
		}
		
		akt_vonalak_szama_elol = VONALAK_SZAMA;
		
		// Encoder értékbol mm különbséget kinyerni
		enc_mm_now = enc_mm_value_get();
		enc_mm_between = enc_mm_now - enc_mm_before;
		enc_mm_before = enc_mm_now;
		
		if(VONALAK_SZAMA == 3){
			VONALAK_SZAMA--;
			if (vonalak[0]> vonalak[1]) Swap(&vonalak[0],&vonalak[1]);
			if (vonalak[1]> vonalak[2]) Swap(&vonalak[1],&vonalak[2]);
			if (vonalak[0]> vonalak[1]) Swap(&vonalak[0],&vonalak[1]);
			elso_szenzor_sor_process[ (uint8_t) vonalak[1]]=0;
			vonalak[0] = vonalak[0];
			vonalak[1] = vonalak[2];
			
			akt_vonalak_szama_elol = VONALAK_SZAMA;
		}	
		// Elso vonal tipusa és fo vonal kiszamolasa
		if(VONALAK_SZAMA == 1){		
			ONE_LINE_SINCE += enc_mm_between;
			if(ONE_LINE_SINCE > 500){
				iranyfeny_vilagithat = FALSE;
				uj_keresztezodes_vilagithat = FALSE;
			}
			if(ONE_LINE_SINCE > 200) 
			{
				//szagatott_type=0;
				kovetni_kell = FALSE;
				vege_az_egyenesnek = FALSE;
				vege_a_szetvalasnak = FALSE;
				lehet_uj_keresztezodes = TRUE;
				
				ugyessegi_lassaban = FALSE;
				cross_type_b_j = 0;
			}
			if(ONE_LINE_SINCE > 350)
			{
				if(szagatott_type>250)
				{
					szagatott_atlag=iaverage(cross_type_e_array,CROSS_E_ARRAY);
					if(szagatott_atlag<110)
					{
						szaggatott=1;
					}
					else
					{
						szaggatott=2;
						
					}
					szagatott_type=0;
				}
			}
			if(ONE_LINE_SINCE > 50) 
			{
				if(cross_type_e>20)
				{
					cross_type_e_array[cross_type_e_array_index] = cross_type_e;
					cross_type_e_array_index++;
					if(cross_type_e_array_index == CROSS_E_ARRAY){
						cross_type_e_array_index=0;						
					}					
				}									
				cross_type_e=0;cross_type_b=0; cross_type_j=0;

			}
			NO_LINE_SINCE=0;
			fo_vonal_hiba = vonalak[0] - 24;
		}
		
		else if(VONALAK_SZAMA == 2){
			TWO_LINE_SINCE += enc_mm_between;
			NO_LINE_SINCE=0;
			// Vonal hibák kiszámolása
			vonal_hibak[0] = vonalak[0] - 24;
			vonal_hibak[1] = vonalak[1] - 24;
			
			// Elözo állapothoz képest kisebb vonal hiba lesz a fovonalhibank
			if(abs((int)(vonal_hibak[0]-fo_vonal_hiba_mem)) > abs((int)(vonal_hibak[1]-fo_vonal_hiba_mem)) )
			{
				fo_vonal_hiba = vonal_hibak[1];
				masodik_vonal_hiba=vonal_hibak[0];
			}
			else
			{
				fo_vonal_hiba = vonal_hibak[0];
				masodik_vonal_hiba=vonal_hibak[1];
			}
			
			//Keresztezodes
			if(lehet_uj_keresztezodes)
			{
				
				// ha nem elorol erkezunk, hanem jobbrol vagy balrol (nem keverhetjük össze a sávváltással)
				vonalhiba_kulonbseg=abs( (int) (fo_vonal_hiba-masodik_vonal_hiba) );
				if(abs( (int) (fo_vonal_hiba-masodik_vonal_hiba) )>9)
				{
					ugyessegi_lassaban = TRUE;
					vege_az_egyenesnek=TRUE;
					if( (masodik_vonal_hiba	+ fo_vonal_hiba) < 0 ) // jobb
					{
						cross_type_j++;
					}
					else //bal
					{
						cross_type_b++; 			
					}				
				}
				else //ha elorol erkeznk vagy savvaltas van
				{
					// még ki kell találni
					vege_a_szetvalasnak = TRUE;
					szagatott_type +=enc_mm_between;
					cross_type_e+=enc_mm_between;			
				}
				
				if((cross_type_j>3 || cross_type_b>3 ) && cross_type_b_j==0)
				{
					szog_keresztezodes_elott=yaw;// szog_hiba_SI volt es azzal mukodott de logikailag ez a helyes
					if(cross_type_b>cross_type_j) 
					{
						cross_type_b_j=2;
						vege_az_egyenesnek = FALSE;
					}
					else if(cross_type_j>=cross_type_b)
					{
						cross_type_b_j=1;
						vege_az_egyenesnek = FALSE;
					}
				}
				if(vege_a_szetvalasnak && cross_type_b_j )
				{			
					
					cross_type_b=0; cross_type_j=0;		cross_type_e=0;	szagatott_type = 0;
					lehet_uj_keresztezodes = FALSE;
					vege_a_szetvalasnak = FALSE;	
					last_j_b_cross=enc_mm_now;
					last_j_b_dir=cross_type_b_j;
					cross_type_b_j=0;
				}
				
				if( cross_type_e>20)
				{
					ugyessegi_lassaban = TRUE;
				}
				
				if( cross_type_e>170 && vege_az_egyenesnek && cross_type_b_j==0) //170
				{
					last_keresztezodes = FWD;
					vege_az_egyenesnek = FALSE;
					lehet_uj_keresztezodes = FALSE;
					iranyfeny_vilagithat = TRUE;
					
					uj_keresztezodes_vilagithat = TRUE;
					
					cross_type_b=0; cross_type_j=0;		cross_type_e=0; szagatott_type = 0;
					//last_cross_time=time_old_adc_process;		
					elore_count++;
										
					next_dir_cross=labirintus(0);
					next_dir_change=TRUE;
					// Túl nagy vonal hiba esetén, elvetjük az új vonalhibát: IMPLEMENTÁLÁSra vár!!!!!
					
					
					
					// FWD helyzetbol jövünk és döntöttünk már, hogy merre megyünk
					if(next_dir_cross==1) // 
					{
						if(vonal_hibak[0]<vonal_hibak[1]) fo_vonal_hiba=vonal_hibak[0];
						else fo_vonal_hiba=vonal_hibak[1];						
					}
					else
					{
						if(vonal_hibak[0]>vonal_hibak[1]) fo_vonal_hiba=vonal_hibak[0];
						else fo_vonal_hiba=vonal_hibak[1];
					}
					kovetni_kell=TRUE;
				}
				
			}
			// Jobbról jöttünk, felulirjuk a vonalat,amit kövessünk
			if((cross_type_b_j == 1) || (enc_mm_now-last_j_b_cross<490 && last_j_b_dir == 1 && last_j_b_cross < 10000000000) )
			{
				// Bal oldali vonalat kövessük
				if(vonal_hibak[0]>vonal_hibak[1]) fo_vonal_hiba=vonal_hibak[0];
				else fo_vonal_hiba=vonal_hibak[1];
			}
			// Balról jöttünk
			else if((cross_type_b_j == 2) || (enc_mm_now-last_j_b_cross<490 && last_j_b_dir == 2 && last_j_b_cross < 10000000000))
			{
				// Jobb oldali vonalat kövessük
				if(vonal_hibak[0]>vonal_hibak[1]) fo_vonal_hiba=vonal_hibak[1];
				else fo_vonal_hiba=vonal_hibak[0];
			}
			
			if(kovetni_kell)
			{
				if(next_dir_cross==1)
				{
					if(vonal_hibak[0]<vonal_hibak[1]) fo_vonal_hiba=vonal_hibak[0];
					else fo_vonal_hiba=vonal_hibak[1];						
				}
				else
				{
					if(vonal_hibak[0]>vonal_hibak[1]) fo_vonal_hiba=vonal_hibak[0];
					else fo_vonal_hiba=vonal_hibak[1];
				}
				
			}
			
			ONE_LINE_SINCE=0;
			

		}
		else {
			NO_LINE_SINCE += enc_mm_between;
			if(NO_LINE_SINCE>3000){Motor_Brake=100;}
		}
		

		
		// Kocsi eleje korrekció, elorébb van a keresztezodésben
		if(enc_mm_now-last_j_b_cross>490){
			uj_keresztezodes_vilagithat = TRUE;
			// Hagyományos döntés
			if( fabs(szog_keresztezodes_elott-yaw)<0.349f) // ez radianba kell valtani
			{
				keresztezodes_dontes_fajta = 1;
				next_dir_cross=labirintus(last_j_b_dir);
				if(last_j_b_dir==2)
				{					
						last_keresztezodes = LEFT;
						balra_count++;		
				}
				else 
				{					
						last_keresztezodes = RIGHT;
						jobbra_count++;	
				}
			}
			// Szög alapján döntés
			else
			{
				keresztezodes_dontes_fajta = 0;
				if(szog_keresztezodes_elott-PI/2<yaw && szog_keresztezodes_elott+PI/2>yaw) yaw_tmp=yaw;
				else if(szog_keresztezodes_elott-PI/2<yaw) yaw_tmp=yaw-2*PI;
				else yaw_tmp=yaw+2*PI;
				if(szog_keresztezodes_elott-yaw_tmp>0)
				{
					last_keresztezodes = RIGHT;
					jobbra_count++;	
					next_dir_cross=labirintus(1);
				}
				else
				{
					last_keresztezodes = LEFT;
					balra_count++;		
					next_dir_cross=labirintus(2);
				}
			}
			last_j_b_cross=10000000000;
		}
		
		
		// Memória változók
		fo_vonal_hiba_mem = fo_vonal_hiba;
		ELSO_VONALAK_SZAMA_MEM = VONALAK_SZAMA;
		
		// Hátsó szenzorsor feldolgozása
		VONALAK_SZAMA = 0;
		for(uint16_t vonal_index = 0;vonal_index < max_vonalszam;vonal_index++)
		{
			search_max(hatso_szenzor_sor_process,48,&max_ertek,&max_index,1);
			if(max_ertek >= adc_vago[48+max_index])
			{
				VONALAK_SZAMA++;
				vonalak[vonal_index] = (float)max_index;
				hatso_szenzor_sor_process[max_index] = 0;
				for(uint8_t index = 0;index < max_vonaltavolsag_szam;index++){
					if((max_index+index+1) <= 47) hatso_szenzor_sor_process[max_index+index+1] = 0;
					if((int8_t)((int8_t)max_index-(int8_t)index-1) >= 0) hatso_szenzor_sor_process[max_index-index-1] = 0;
				}
			}
		}
		
		if(VONALAK_SZAMA == 3){
			VONALAK_SZAMA--;
			if (vonalak[0]> vonalak[1]) Swap(&vonalak[0],&vonalak[1]);
			if (vonalak[1]> vonalak[2]) Swap(&vonalak[1],&vonalak[2]);
			if (vonalak[0]> vonalak[1]) Swap(&vonalak[0],&vonalak[1]);
			vonalak[0] = vonalak[0];
			vonalak[1] = vonalak[2];
		}			
		if(VONALAK_SZAMA == 1){	
			HATSO_VONAL_SINCE += enc_mm_between;
			// Egyszeru
			hatso_fo_vonal_hiba = vonalak[0] - 24;
		}
		else if(VONALAK_SZAMA == 2){
			HATSO_VONAL_SINCE = 0;
			// Vonal hibák kiszámolása
			vonal_hibak[0] = vonalak[0] - 24;
			vonal_hibak[1] = vonalak[1] - 24;
			
			// Elözo állapothoz képest kisebb vonal hiba lesz a fovonalhibank
			if(abs((int)(vonal_hibak[0]-hatso_fo_vonal_hiba_mem)) > abs((int)(vonal_hibak[1]-hatso_fo_vonal_hiba_mem)) )
			{
				hatso_fo_vonal_hiba = vonal_hibak[1];
			}
			else
			{
				hatso_fo_vonal_hiba = vonal_hibak[0];
			}
			
			// Jobbról jöttünk, felulirjuk a vonalat,amit kövessünk
			if((cross_type_b_j == 1) || (enc_mm_now-last_j_b_cross<490 && last_j_b_dir == 1 && last_j_b_cross < 10000000000) )
			{
				// Bal oldali vonalat kövessük
				if(vonal_hibak[0]>vonal_hibak[1]) hatso_fo_vonal_hiba=vonal_hibak[0];
				else hatso_fo_vonal_hiba=vonal_hibak[1];
			}
			// Balról jöttünk
			else if((cross_type_b_j == 2) || (enc_mm_now-last_j_b_cross<490 && last_j_b_dir == 2 && last_j_b_cross < 10000000000))
			{
				// Jobb oldali vonalat kövessük
				if(vonal_hibak[0]>vonal_hibak[1]) hatso_fo_vonal_hiba=vonal_hibak[1];
				else hatso_fo_vonal_hiba=vonal_hibak[0];
			}
			
			// Elorol jöttünk , tudjuk az irányt merre akarunk menni
			if(kovetni_kell)
			{
					if(next_dir_cross==1)
					{
						if(vonal_hibak[0]<vonal_hibak[1]) hatso_fo_vonal_hiba=vonal_hibak[0];
						else hatso_fo_vonal_hiba=vonal_hibak[1];						
					}
					else
					{
						if(vonal_hibak[0]>vonal_hibak[1]) hatso_fo_vonal_hiba=vonal_hibak[0];
						else hatso_fo_vonal_hiba=vonal_hibak[1];
					}
					
			}
		}
		


		
		hatso_fo_vonal_hiba_mem = hatso_fo_vonal_hiba;
		
		// Vonal hiba és szöghiba SI-ben(m,radian)
		fo_vonal_hiba_SI = fo_vonal_hiba*TCRT5000_tavolsag*0.001f;
		if(ELSO_VONALAK_SZAMA_MEM !=0 && VONALAK_SZAMA != 0) szog_hiba_SI = ((fo_vonal_hiba - hatso_fo_vonal_hiba)*TCRT5000_tavolsag)/SZENZORSOR_tavolsag;
		
		cross_99=path_99();
		if(( (data_table[current_cross][11]==99 && last_keresztezodes!=FWD) || 
			(cross_99==-1 &&last_keresztezodes ==FWD && (data_table[current_cross][11]==1 ||data_table[current_cross][11]==2))) 
			&& num_of_unkonown_road == 0 &&ONE_LINE_SINCE>200 && HATSO_VONAL_SINCE > 10)
		{
			state_game = LANE_CHANGE;
			jirany_valtozik = TRUE;
			HATSO_VONAL_SINCE = 0;
			cross_type_e = 0;
		}
}

void atsorolas_adc_process(uint8_t max_vonalszam,uint8_t max_vonaltavolsag_szam){
		memcpy(elso_szenzor_sor_process,szenzor_sor_front,sizeof(float)*48);
		memcpy(hatso_szenzor_sor_process,szenzor_sor_back,sizeof(float)*48);
		for(uint8_t i=0;i<48;i++) {
			elso_ugyessegi_sum[i][ablak_ind]=elso_szenzor_sor_process[i];
			hatso_ugyessegi_sum[i][ablak_ind]=hatso_szenzor_sor_process[i];
		}
		ablak_ind++;
		if(ablak_ind==ABLAK_MERET) ablak_ind=0;

		for(uint8_t i=0;i<48;i++) {
			elso_sum_tmp=0; hatso_sum_tmp=0;
			for(uint8_t j=0;j<ABLAK_MERET;j++)
			{
				elso_sum_tmp+=elso_ugyessegi_sum[i][j];
				hatso_sum_tmp+=hatso_ugyessegi_sum[i][j];
			}
			elso_szenzor_sor_process[i]=elso_sum_tmp/ABLAK_MERET;
			hatso_szenzor_sor_process[i]=hatso_sum_tmp/ABLAK_MERET;
		}
		VONALAK_SZAMA = 0;
		for(uint16_t vonal_index = 0;vonal_index < max_vonalszam;vonal_index++) {
			search_max(elso_szenzor_sor_process,48,&max_ertek,&max_index,0);
			if(max_ertek >= adc_vago[max_index])
			{
				VONALAK_SZAMA++;
				vonalak[vonal_index] = (float)max_index;
				elso_szenzor_sor_process[max_index] = 1;
				for(uint8_t index = 0;index < max_vonaltavolsag_szam;index++){
					if((max_index+index+1) <= 47) elso_szenzor_sor_process[max_index+index+1] = 0;
					if((int8_t)((int8_t)max_index-(int8_t)index-1) >= 0) elso_szenzor_sor_process[max_index-index-1] = 0;
				}
			}
		}
		
		akt_vonalak_szama_elol = VONALAK_SZAMA;
		
		// Encoder értékbol mm különbséget kinyerni
		enc_mm_now = enc_mm_value_get();
		enc_mm_between = enc_mm_now - enc_mm_before;
		enc_mm_before = enc_mm_now;
		
	
		// Elso vonal tipusa és fo vonal kiszamolasa
		if(VONALAK_SZAMA == 1){		
			ONE_LINE_SINCE += enc_mm_between;
			if(ONE_LINE_SINCE > 200) 
			{
				cross_type_e = 0;
				lehet_uj_keresztezodes = TRUE;
				ugyessegi_lassaban = FALSE;
			}
			NO_LINE_SINCE=0;
			// Egyszeru
			fo_vonal_hiba = FOV_szures*fo_vonal_hiba+ (1-FOV_szures) *( vonalak[0] - 24);
		}
		
		else if(VONALAK_SZAMA == 2){
			TWO_LINE_SINCE += enc_mm_between;
			NO_LINE_SINCE=0;
			// Vonal hibák kiszámolása
			vonal_hibak[0] = vonalak[0] - 24;
			vonal_hibak[1] = vonalak[1] - 24;
			
			fo_vonal_hiba = (vonal_hibak[0]+vonal_hibak[1])/2;
			
			
			//Sávváltás érzékelés
			if(lehet_uj_keresztezodes && atsoroltunk_mar == FALSE)
			{
				cross_type_e+=enc_mm_between;
			}
			
			ONE_LINE_SINCE=0;
		}
		else if(VONALAK_SZAMA == 3){
			vonal_hibak[0] = vonalak[0] - 24;
			vonal_hibak[1] = vonalak[1] - 24;
			vonal_hibak[2] = vonalak[2] - 24;
			
			fo_vonal_hiba = (vonal_hibak[0]+vonal_hibak[1]+vonal_hibak[2])/3;
			
						//Sávváltás érzékelés
			if(lehet_uj_keresztezodes && atsoroltunk_mar == FALSE)
			{
				cross_type_e+=enc_mm_between;
			}
			
			NO_LINE_SINCE=0;
			ONE_LINE_SINCE=0;
		}
		else {
			NO_LINE_SINCE += enc_mm_between;
			nem_latunk_vonalat_egy_ideig_elol = TRUE;
		}


		
		fo_vonal_hiba_mem = fo_vonal_hiba;
		ELSO_VONALAK_SZAMA_MEM = VONALAK_SZAMA;
		
		// Hátsó szenzorsor feldolgozása
		VONALAK_SZAMA = 0;
		for(uint16_t vonal_index = 0;vonal_index < max_vonalszam;vonal_index++)
		{
			search_max(hatso_szenzor_sor_process,48,&max_ertek,&max_index,1);
			if(max_ertek >= adc_vago[48+max_index])
			{
				VONALAK_SZAMA++;
				vonalak[vonal_index] = (float)max_index;
				hatso_szenzor_sor_process[max_index] = 0;
				for(uint8_t index = 0;index < max_vonaltavolsag_szam;index++){
					if((max_index+index+1) <= 47) hatso_szenzor_sor_process[max_index+index+1] = 0;
					if((int8_t)((int8_t)max_index-(int8_t)index-1) >= 0) hatso_szenzor_sor_process[max_index-index-1] = 0;
				}
			}
		}
		
		
		if(VONALAK_SZAMA == 1){		
			HATSO_VONAL_SINCE+=enc_mm_between;
			// Egyszeru
			hatso_fo_vonal_hiba = vonalak[0] - 24;
			
			//NO_LINE_SINCE_HATUL = 0;
		}

		else if(VONALAK_SZAMA == 2){
			HATSO_VONAL_SINCE+=enc_mm_between;
			// Vonal hibák kiszámolása
			vonal_hibak[0] = vonalak[0] - 24;
			vonal_hibak[1] = vonalak[1] - 24;
			hatso_fo_vonal_hiba = (vonal_hibak[0]+vonal_hibak[1])/2;
			
			//NO_LINE_SINCE_HATUL = 0;
		}
		else if(VONALAK_SZAMA == 3){
			HATSO_VONAL_SINCE+=enc_mm_between;
			vonal_hibak[0] = vonalak[0] - 24;
			vonal_hibak[1] = vonalak[1] - 24;
			vonal_hibak[2] = vonalak[2] - 24;
			//NO_LINE_SINCE_HATUL = 0;
			hatso_fo_vonal_hiba = (vonal_hibak[0]+vonal_hibak[1]+vonal_hibak[2])/3;
		}
		else {
			NO_LINE_SINCE_HATUL += enc_mm_between;
			HATSO_VONAL_SINCE = 0;
			nem_latunk_vonalat_egy_ideig_hatul = TRUE;
		}

		
		hatso_fo_vonal_hiba_mem = hatso_fo_vonal_hiba;
		if(NO_LINE_SINCE_HATUL > 10 && !atsorolas_hatul_nullaztuk)
		{
			atsorolas_hatul_nullaztuk = TRUE;
			HATSO_VONAL_SINCE = 0;
		}
		
		if((enc_mm_now - enc_ugyessegi_vege) > 1000 /*&& fo_vonal_hiba>5 */&& atsorolunk && HATSO_VONAL_SINCE > 20)
		{
			atsorolunk = FALSE;
			cross_type_e = 0;
			lehet_uj_keresztezodes = FALSE;
			atsoroltunk_mar = TRUE;
			enc_gyorsassagi_kezdete = enc_mm_now;
		}
		
		if(cross_type_e >= 50 && ugyessegin_vagyunk)
		{
			enc_ugyessegi_vege = enc_mm_now;
			NO_LINE_SINCE_HATUL = 0;
		}
		
		if((cross_type_e >= 50||atsorolunk) && (ugyessegin_vagyunk||atsorolunk) )
		{
			atsorolunk = TRUE;
			ugyessegin_vagyunk = FALSE;
			atsorolasi_hiba_mm = enc_mm_now - enc_ugyessegi_vege;
			fo_vonal_hiba = -18 + atsorolasi_hiba_mm*0.02f;
			if(fo_vonal_hiba > 0) fo_vonal_hiba = 0;
			hatso_fo_vonal_hiba = -18 + atsorolasi_hiba_mm*0.02f;
			if(hatso_fo_vonal_hiba > 0) hatso_fo_vonal_hiba = 0;
		}
		
		if(SHARP_front_mm<= 500 && atsoroltunk_mar )
		{
			state_game = SAFETYCAR;
			Motor_Brake = 100;
			HATSO_VONAL_SINCE = 0;
			ONE_LINE_SINCE = 1250;
		}

		
		// Vonal hiba és szöghiba SI-ben(m,radian)
		fo_vonal_hiba_SI = fo_vonal_hiba*TCRT5000_tavolsag*0.001f;
		if(ELSO_VONALAK_SZAMA_MEM !=0 && VONALAK_SZAMA != 0) szog_hiba_SI = ((fo_vonal_hiba - hatso_fo_vonal_hiba)*TCRT5000_tavolsag)/SZENZORSOR_tavolsag;

}
void elozes_adc_process(uint8_t max_vonalszam,uint8_t max_vonaltavolsag_szam) {
		memcpy(elso_szenzor_sor_process,szenzor_sor_front,sizeof(float)*48);
		memcpy(hatso_szenzor_sor_process,szenzor_sor_back,sizeof(float)*48);

	
		VONALAK_SZAMA = 0;
		for(uint16_t vonal_index = 0;vonal_index < max_vonalszam;vonal_index++) {
			search_max(elso_szenzor_sor_process,48,&max_ertek,&max_index,0);
			if(max_ertek >= adc_vago[max_index])
			{
				VONALAK_SZAMA++;
				vonalak[vonal_index] = (float)max_index;
				elso_szenzor_sor_process[max_index] = 1;
				for(uint8_t index = 0;index < max_vonaltavolsag_szam;index++){
					if((max_index+index+1) <= 47) elso_szenzor_sor_process[max_index+index+1] = 0;
					if((int8_t)((int8_t)max_index-(int8_t)index-1) >= 0) elso_szenzor_sor_process[max_index-index-1] = 0;
				}
			}
		}
		
		akt_vonalak_szama_elol = VONALAK_SZAMA;
		
		// Encoder értékbol mm különbséget kinyerni
		enc_mm_now = enc_mm_value_get();
		enc_mm_between = enc_mm_now - enc_mm_before;
		enc_mm_before = enc_mm_now;
		
		if(elozes_elotti_szog-3*PI/2<yaw && elozes_elotti_szog+3*PI/2>yaw) yaw_tmp=yaw;
		else if(elozes_elotti_szog-3*PI/2<yaw) yaw_tmp=yaw-2*PI;
		else yaw_tmp=yaw+2*PI;
		

		if(elozes_allapot == OT_START && fabs(elozes_posy)>500)
		{
			//egyenes_elozes_kezdete = enc_mm_now;
			elozes_allapot = OT_STRAIGHT;
		}

		if(elozes_allapot == OT_STRAIGHT || elozes_allapot == OT_START)
		{
			fo_vonal_hiba_SI = (elozes_posy + 800.0f)*0.001f;
		}
		else
		{
			fo_vonal_hiba_SI = (elozes_posy)*0.001f;
			
		}
		szog_hiba_SI = yaw_tmp -elozes_elotti_szog;
		
		if(elozes_posx>4500 && elozes_allapot == OT_STRAIGHT)
		{
			elozes_allapot = OT_BACK;
			//accumulated_speed_error = 0;
			ONE_LINE_SINCE = 0;
			TWO_LINE_SINCE = 0;
			THREE_LINE_SINCE = 0;
			FOUR_LINE_SINCE = 0;
			HATSO_VONAL_SINCE=0;
		}	
		if(fabs(elozes_posy)<5  && elozes_allapot == OT_BACK)
		{
			ONE_LINE_SINCE=1250;
			state_game = SAFETYCAR;	
			accumulated_speed_error = 150;			
		}			
		
		
		// Elso vonal tipusa és fo vonal kiszamolasa
		if(VONALAK_SZAMA == 1){		
			ONE_LINE_SINCE += enc_mm_between;	
			if(ONE_LINE_SINCE>50)THREE_LINE_SINCE=0;
			NO_LINE_SINCE=0;
			fo_vonal_hiba = vonalak[0] - 24;
		}
		else if(VONALAK_SZAMA == 2){
			TWO_LINE_SINCE += enc_mm_between;
			NO_LINE_SINCE=0;
			// Vonal hibák kiszámolása
			vonal_hibak[0] = vonalak[0] - 24;
			vonal_hibak[1] = vonalak[1] - 24;
			fo_vonal_hiba = (vonal_hibak[0]+vonal_hibak[1])/2.0f;
		}
		else if(VONALAK_SZAMA == 3){
			THREE_LINE_SINCE += enc_mm_between;
			NO_LINE_SINCE=0;
			
			// Vonal hibák kiszámolása
			vonal_hibak[0] = vonalak[0] - 24;
			vonal_hibak[1] = vonalak[1] - 24;
			vonal_hibak[2] = vonalak[2] - 24;
			
			// atlagnal kovetjuk
			fo_vonal_hiba = (vonal_hibak[0]+vonal_hibak[1]+vonal_hibak[2])/3.0f;
		}
		else if(VONALAK_SZAMA ==4){
			vonal_hibak[0] = vonalak[0] - 24;
			vonal_hibak[1] = vonalak[1] - 24;
			vonal_hibak[2] = vonalak[2] - 24;
			vonal_hibak[3] = vonalak[3] - 24;
			FOUR_LINE_SINCE+=enc_mm_between;
			NO_LINE_SINCE=0;
			fo_vonal_hiba = (vonal_hibak[0]+vonal_hibak[1]+vonal_hibak[2]+vonal_hibak[3])/4.0f;
		}
		else {
			NO_LINE_SINCE += enc_mm_between;
		}
		
		fo_vonal_hiba_mem = fo_vonal_hiba;
		ELSO_VONALAK_SZAMA_MEM = VONALAK_SZAMA;
		
		// Hátsó szenzorsor feldolgozása
		VONALAK_SZAMA = 0;
		for(uint16_t vonal_index = 0;vonal_index < max_vonalszam;vonal_index++){
			search_max(hatso_szenzor_sor_process,48,&max_ertek,&max_index,1);
			if(max_ertek >= adc_vago[48+max_index])
			{
				VONALAK_SZAMA++;
				vonalak[vonal_index] = (float)max_index;
				hatso_szenzor_sor_process[max_index] = 0;
				for(uint8_t index = 0;index < max_vonaltavolsag_szam;index++){
					if((max_index+index+1) <= 47) hatso_szenzor_sor_process[max_index+index+1] = 0;
					if((int8_t)((int8_t)max_index-(int8_t)index-1) >= 0) hatso_szenzor_sor_process[max_index-index-1] = 0;
				}
			}
		}
		
		
		if(VONALAK_SZAMA == 1){		
			HATSO_VONAL_SINCE+=enc_mm_between;
			// Egyszeru
			hatso_fo_vonal_hiba = vonalak[0] - 24;
		}
		else if(VONALAK_SZAMA == 2){
			HATSO_VONAL_SINCE+=enc_mm_between;
			// Vonal hibák kiszámolása
			vonal_hibak[0] = vonalak[0] - 24;
			vonal_hibak[1] = vonalak[1] - 24;
			
			hatso_fo_vonal_hiba = (vonal_hibak[0] + vonal_hibak[1])/2;
		}
		else if(VONALAK_SZAMA == 3){
			HATSO_VONAL_SINCE+=enc_mm_between;
			// Vonal hibák kiszámolása
			vonal_hibak[0] = vonalak[0] - 24;
			vonal_hibak[1] = vonalak[1] - 24;
			vonal_hibak[2] = vonalak[2] - 24;
			
			// Kozepso vonal kivalasztasa
			hatso_fo_vonal_hiba = (vonal_hibak[0]+vonal_hibak[1]+vonal_hibak[2])/3.0f;
		}
		else if(VONALAK_SZAMA ==4){
			HATSO_VONAL_SINCE+=enc_mm_between;
			vonal_hibak[0] = vonalak[0] - 24;
			vonal_hibak[1] = vonalak[1] - 24;
			vonal_hibak[2] = vonalak[2] - 24;
			vonal_hibak[3] = vonalak[3] - 24;
			FOUR_LINE_SINCE+=enc_mm_between;
			NO_LINE_SINCE=0;
			hatso_fo_vonal_hiba = (vonal_hibak[0]+vonal_hibak[1]+vonal_hibak[2]+vonal_hibak[3])/4.0f;
		}
		
		hatso_fo_vonal_hiba_mem = hatso_fo_vonal_hiba;
}
uint8_t labirintus(uint8_t direction){	
	uint8_t dir=data_process(direction,pos_x, pos_y, enc_mm_now-last_cross_encoder, szaggatott);
	last_cross_encoder=enc_mm_now;
	//last_cross_time=time_old_adc_process;
	//Ha valamelyik infot tovabb tudjuk terjeszteni, akkor megcsináljuk
	uint8_t flow=1;
	while(flow) flow=follow_the_flow();
	szaggatott=-1;	
	return dir;
}

float iaverage(float *iarray,uint32_t arraysize){
	float favg = 0;
	for(uint16_t index = 0; index < arraysize;index++){
		favg += iarray[index];
	}
	favg = favg/arraysize;
	return favg;
}
