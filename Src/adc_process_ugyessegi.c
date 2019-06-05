#include "adc_process_ugyessegi.h"

#define ABLAK_MERET 5
/*
uint32_t elso_ugyessegi_res[48],hatso_ugyessegi_res[48];
uint32_t elso_ugyessegi_sum[48][ABLAK_MERET],hatso_ugyessegi_sum[48][ABLAK_MERET];
uint32_t elso_sum_tmp,hatso_sum_tmp;
uint8_t ablak_ind=0;*/
/*
uint8_t fb_values[48];
uint8_t spi_data_fb[6]={0x00,0x00,0x00,0x00,0x00,0x00};

uint32_t fbled_time_before,fbled_time_now;*/
void adc_proc_ugyessegi()
{	
	
	/*
	for(uint8_t i=0;i<48;i++)
	{
		elso_ugyessegi_sum[i][ablak_ind]=szenzor_sor_front[i];
		hatso_ugyessegi_sum[i][ablak_ind]=szenzor_sor_back[i];
	}
	ablak_ind++;
	if(ablak_ind==ABLAK_MERET) ablak_ind=0;

	for(uint8_t i=0;i<48;i++)
	{
		elso_sum_tmp=0; hatso_sum_tmp=0;
		for(uint8_t j=1;j<ABLAK_MERET;j++)
		{
			elso_sum_tmp+=elso_ugyessegi_sum[i][j];
			hatso_sum_tmp+=hatso_ugyessegi_sum[i][j];
		}
		if(elso_sum_tmp>((ABLAK_MERET-1)*adc_vago[i])) {elso_ugyessegi_res[i]=elso_sum_tmp/ABLAK_MERET;}
		else elso_ugyessegi_res[i]=0;
		if(hatso_sum_tmp>((ABLAK_MERET-1)*adc_vago[48+i])) {hatso_ugyessegi_res[i]=hatso_sum_tmp/ABLAK_MERET;}
		else hatso_ugyessegi_res[i]=0;
	}
		
		
		fbled_time_now = get_timer_us();
		// 10 msec frissitjük a Feedback LEDeket -> Nem terheli le a procit
		if(1 || fbled_time_now > (fbled_time_before+10000))
		{
			// Elso szenzosor feldolgozása
				for(uint8_t j = 0;j<48;j++){
					if(elso_ugyessegi_res[j]> 0)
					{
						fb_values[j] = 1;
					}
					else
					{
						fb_values[j] = 0;
					}
				}
				
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

	*/
}




