/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define ADC3_FRONT_Pin GPIO_PIN_0
#define ADC3_FRONT_GPIO_Port GPIOC
#define TCRT_MOSI_Pin GPIO_PIN_1
#define TCRT_MOSI_GPIO_Port GPIOC
#define ADC2_FRONT_Pin GPIO_PIN_2
#define ADC2_FRONT_GPIO_Port GPIOC
#define ADC1_FRONT_Pin GPIO_PIN_3
#define ADC1_FRONT_GPIO_Port GPIOC
#define ADC3_BACK_Pin GPIO_PIN_0
#define ADC3_BACK_GPIO_Port GPIOA
#define ADC2_BACK_Pin GPIO_PIN_1
#define ADC2_BACK_GPIO_Port GPIOA
#define PC_TX_Pin GPIO_PIN_2
#define PC_TX_GPIO_Port GPIOA
#define PC_RX_Pin GPIO_PIN_3
#define PC_RX_GPIO_Port GPIOA
#define ADC1_BACK_Pin GPIO_PIN_4
#define ADC1_BACK_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define ENCA_Pin GPIO_PIN_6
#define ENCA_GPIO_Port GPIOA
#define ENCB_Pin GPIO_PIN_7
#define ENCB_GPIO_Port GPIOA
#define ENC_INDEX_Pin GPIO_PIN_4
#define ENC_INDEX_GPIO_Port GPIOC
#define AN_IN_Pin GPIO_PIN_5
#define AN_IN_GPIO_Port GPIOC
#define FBLED_MOSI_Pin GPIO_PIN_0
#define FBLED_MOSI_GPIO_Port GPIOB
#define AN_Multi_IN_Pin GPIO_PIN_1
#define AN_Multi_IN_GPIO_Port GPIOB
#define PWM_Servo_Pin GPIO_PIN_2
#define PWM_Servo_GPIO_Port GPIOB
#define TCRT_SCK_Pin GPIO_PIN_10
#define TCRT_SCK_GPIO_Port GPIOB
#define MOTOR_EN_Pin GPIO_PIN_12
#define MOTOR_EN_GPIO_Port GPIOB
#define MULTIP_1_Pin GPIO_PIN_14
#define MULTIP_1_GPIO_Port GPIOB
#define MULTIP_2_Pin GPIO_PIN_15
#define MULTIP_2_GPIO_Port GPIOB
#define TOF_IR_3_Pin GPIO_PIN_6
#define TOF_IR_3_GPIO_Port GPIOC
#define TOF_IR_3_EXTI_IRQn EXTI9_5_IRQn
#define TOF_IR_2_Pin GPIO_PIN_7
#define TOF_IR_2_GPIO_Port GPIOC
#define TOF_IR_2_EXTI_IRQn EXTI9_5_IRQn
#define SHARP_MULTIP_1_Pin GPIO_PIN_8
#define SHARP_MULTIP_1_GPIO_Port GPIOC
#define SHARP_MULTIP_2_Pin GPIO_PIN_9
#define SHARP_MULTIP_2_GPIO_Port GPIOC
#define PWM_Motor_Pin GPIO_PIN_8
#define PWM_Motor_GPIO_Port GPIOA
#define RI_TX_Pin GPIO_PIN_9
#define RI_TX_GPIO_Port GPIOA
#define RI_RX_Pin GPIO_PIN_10
#define RI_RX_GPIO_Port GPIOA
#define ULTRA_TRIG_Pin GPIO_PIN_11
#define ULTRA_TRIG_GPIO_Port GPIOA
#define ULTRA_ECHO_Pin GPIO_PIN_12
#define ULTRA_ECHO_GPIO_Port GPIOA
#define ULTRA_ECHO_EXTI_IRQn EXTI15_10_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define PWM_IN_Pin GPIO_PIN_15
#define PWM_IN_GPIO_Port GPIOA
#define FBLED_SCK_Pin GPIO_PIN_10
#define FBLED_SCK_GPIO_Port GPIOC
#define TOF_IR_1_Pin GPIO_PIN_11
#define TOF_IR_1_GPIO_Port GPIOC
#define TOF_IR_1_EXTI_IRQn EXTI15_10_IRQn
#define BT_TX_Pin GPIO_PIN_12
#define BT_TX_GPIO_Port GPIOC
#define BT_RX_Pin GPIO_PIN_2
#define BT_RX_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LE_1_Pin GPIO_PIN_4
#define LE_1_GPIO_Port GPIOB
#define INERC_IT_Pin GPIO_PIN_5
#define INERC_IT_GPIO_Port GPIOB
#define INERC_IT_EXTI_IRQn EXTI9_5_IRQn
#define MULTIP_4_Pin GPIO_PIN_8
#define MULTIP_4_GPIO_Port GPIOB
#define MULTIP_3_Pin GPIO_PIN_9
#define MULTIP_3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
extern SemaphoreHandle_t xMutexI2C;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
