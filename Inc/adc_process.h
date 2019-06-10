#include "robonaut_config.h"

#define SZAKASZOK_SZAMA 8
#define MAX_RESZEK_SZAMA_SZAKASZBAN 3
#define ADATOK_SZAMA 5

void Start_Calculate_Task(void const * argument);
void search_max(uint32_t *values,uint32_t size,uint32_t *max_ertek,uint32_t *max_index,uint8_t type);
void Swap(float *a,float *b);

void ugyessegi_adc_process(uint8_t max_vonalszam,uint8_t max_vonaltavolsag_szam);
void gyorsassagi_adc_process(uint8_t max_vonalszam,uint8_t max_vonaltavolsag_szam);
void atsorolas_adc_process(uint8_t max_vonalszam,uint8_t max_vonaltavolsag_szam);
void safetycar_adc_process(uint8_t max_vonalszam,uint8_t max_vonaltavolsag_szam);
void elozes_adc_process(uint8_t max_vonalszam,uint8_t max_vonaltavolsag_szam);

uint8_t labirintus(uint8_t irany);
float iaverage(float *iarray,uint32_t arraysize);
