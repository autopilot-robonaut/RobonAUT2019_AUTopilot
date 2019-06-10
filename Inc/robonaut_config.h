#ifndef __RobonuAUT_CONF_H
#define __RobonuAUT_CONF_H

#define ULONG_MAX  4294967295UL
//#define PI 3.14159265f

#define TRUE  1
#define FALSE 0

#define ARM_MATH_CM4

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
//#include "math.h"
#include "arm_math.h"
#include <stdint.h>
#include <stdlib.h> 

#include "timer.h"
#include "debug.h"
#include "motor.h"
#include "adc.h"
#include "adc_process.h"
#include "pwm_in.h"
#include "servo.h"
#include "encoder.h"
#include "bno055.h"
#include "inertial.h"
#include "TOF.h"
#include "labirintus.h"


//A funkciókhoz handlerek
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc1;

extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern DMA_HandleTypeDef hdma_tim3_ch4_up;

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

extern osThreadId Encoder_TaskHandle;
extern osThreadId ADC_TaskHandle;
extern osThreadId Debug_TaskHandle;
extern osThreadId Calculate_TaskHandle;
extern osThreadId Motor_TaskHandle;
extern osThreadId Get_Debug_TaskHandle;
extern osThreadId Gyroscope_TaskHandle;
extern osThreadId Servo_TaskHandle;


//ISR wait-hez kapcsolódik
extern BaseType_t xHigherPriorityTaskWoken;
extern BaseType_t xHigherPriorityTaskWokenDebug;
extern BaseType_t xHigherPriorityTaskWokenGPIO;
extern BaseType_t xHigherPriorityTaskWokenDebugGet;

// Mutexek
extern SemaphoreHandle_t xMutexI2C;
extern SemaphoreHandle_t xMutexUART;

enum section_state{
	SLOW = 0,
	FAST = 1,
};

enum keresztezodes{
	FWD = 0,
	LEFT = 1,
	RIGHT = 2,
};

enum game_state{
	LABYRINTH = 0,
	LANE_CHANGE = 1,
	OVERTAKING = 2,
	SAFETYCAR = 3,
	SPEED_RACE = 4,
};

enum overtaking_state{
	OT_START = 0,
	OT_STRAIGHT = 1,
	OT_BACK = 2,
};

//Saját globális változók
// Timer változók
extern uint16_t Timer7_UPPER;

// ADC változók
extern uint32_t adc_buffer[9],time_elapsed_adc;
extern uint32_t szenzor_sor_front[48],szenzor_sor_back[48];
extern uint32_t SHARP_side_voltage,SHARP_front_voltage;
extern float SHARP_side_mm,SHARP_front_mm;


// ADC Process változók
extern float fo_vonal_hiba_SI,szog_hiba_SI,fo_vonal_hiba,masodik_vonal_hiba;
extern enum section_state akt_vonalallapot;
extern uint32_t time_elapsed_adc_process;
extern uint32_t akt_vonalak_szama_elol;
extern uint32_t adc_vago[96];
extern uint32_t valtasokszama;
extern float FOUR_LINE_SINCE,THREE_LINE_SINCE,HATSO_VONAL_SINCE;
extern uint8_t state_game;
extern uint32_t elso_szenzor_sor_process[48],hatso_szenzor_sor_process[48];
extern float TWO_LINE_SINCE,ONE_LINE_SINCE;
extern int8_t next_dir_cross;
extern uint32_t cross_type_b,cross_type_j,cross_type_s,balra_count,jobbra_count,elore_count;
extern float szagatott_type;
extern float cross_type_e;
extern int8_t szaggatott;
extern float utolso_szakasz_mm;
extern float palya[SZAKASZOK_SZAMA][MAX_RESZEK_SZAMA_SZAKASZBAN][ADATOK_SZAMA];
extern uint8_t reszek_szama_ind;
extern int32_t szakaszok[SZAKASZOK_SZAMA];
extern uint8_t current_szakasz;
extern uint8_t ugyessegi_lassaban;
extern float szagatott_atlag;
extern enum overtaking_state elozes_allapot;
extern float egyenes_elozes_kezdete;
extern uint8_t atsoroltunk_mar,atsorolunk;
extern uint8_t keresztezodes_dontes_fajta;
extern uint8_t vonalhiba_kulonbseg;
extern uint32_t valtasokszama;

//OVERTAKING
extern float yaw_tmp;
extern float elozes_posx,elozes_posy;

// PWM változók
extern uint32_t PWMINValue;

// Enkóder változók
extern float speed;
extern float encoder_full_value;
extern float encoder_upper_value;
extern float encoder_time_between;
extern float pos_x,pos_y;

extern float kirantas_szog_kulonbseg;


// Inerciális szenzor értékek
extern float yaw;
extern u8 mag_calib_u8,accel_calib_u8,gyro_calib_u8,system_calib_u8,operation_mode_u8;
extern s32 comres;
extern float yaw_tmp;

//State Space 
extern float kszi, Tservo, tbeallas,dbeallas,svalos,skomplex,korientacio,kkozep,speedsi,L,deltap;
extern float servo_pwm,szervo_kozep;

// TOF változók
extern uint8_t ultra_timer,ultra_overrun;
extern float ultra_distance_mm;
extern uint32_t ultra_echo_time;
extern uint32_t time_elapsed_TOF;

//Motor valtozok
extern float MOTOR_P,MOTOR_I;
extern float accumulated_speed_error;
extern uint8_t Motor_Brake,BRAKE_GYORS_LASSU;
extern float desired_speed;
extern float MOTOR_P_ERROR,MOTOR_I_ERROR;
extern char RI_receive_message[5];
extern uint8_t RI_started;
extern uint8_t first_run_motor;
extern uint8_t egyenesen_megyunk;
extern uint8_t max_speed;
extern uint32_t elozo_resz_valtas;
extern float dbeallasm,dbeallaso;
extern float MOTOR_P_flash,MOTOR_I_flash;

//Flash
extern float flash_data[FLASH_DATA_SIZE];
extern uint32_t addr_write;

//Labirintus változók
extern int32_t last_lab_x,last_lab_y;
extern enum keresztezodes last_keresztezodes;
extern uint8_t newcross,konwncross;
extern int8_t cross_99;
extern uint8_t current_cross;
extern uint8_t num_of_unkonown_road;
extern int32_t data_table[NODES_NUM][NODES_DATA_NUM];
extern uint32_t min_dist;
extern uint8_t next_dir_change;
extern uint8_t cross_type_b_j;
extern uint8_t gyorsasagival_kezdunk;
extern uint8_t feny_allapota,star_valtozik,jirany_valtozik,fek_valtozik,birany_valtozik;


#endif /* __RobonuAUT_CONF_H */
