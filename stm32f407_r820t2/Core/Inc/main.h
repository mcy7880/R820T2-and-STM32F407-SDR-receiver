/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define N_cos_sin    33 //see comments in main.c
#define Step_cos_sin 10

#define N_asin 150

//IQ filters coefficients for FM
#define b0__105kHz 5.59802632778882980346679687500000e-04
#define b1__105kHz 2.79901339672505855560302734375000e-03
#define b2__105kHz 5.59802679345011711120605468750000e-03
#define b3__105kHz 5.59802679345011711120605468750000e-03
#define b4__105kHz 2.79901339672505855560302734375000e-03
#define b5__105kHz 5.59802632778882980346679687500000e-04

#define a0__105kHz -3.78060984611511230468750000000000e+00
#define a1__105kHz 6.29705619812011718750000000000000e+00
#define a2__105kHz -5.68249320983886718750000000000000e+00
#define a3__105kHz 2.76533651351928710937500000000000e+00
#define a4__105kHz -5.81376254558563232421875000000000e-01

//IQ filters coefficients for AM and CW
#define b0__15kHz 3.95331483105110237374901771545410e-08
#define b1__15kHz 1.97665755763409833889454603195190e-07
#define b2__15kHz 3.95331511526819667778909206390381e-07
#define b3__15kHz 3.95331511526819667778909206390381e-07
#define b4__15kHz 1.97665755763409833889454603195190e-07
#define b5__15kHz 3.95331483105110237374901771545410e-08

#define a0__15kHz -4.90738058090209960937500000000000e+00
#define a1__15kHz 9.64835929870605468750000000000000e+00
#define a2__15kHz -9.49977493286132812500000000000000e+00
#define a3__15kHz 4.68406248092651367187500000000000e+00
#define a4__15kHz -9.25264954566955566406250000000000e-01

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
typedef enum
{
	DEMOD_FM = 0,
	DEMOD_AM,
	OUT_IQ,
	DEMOD_CW
}Output_demod_type_enum;
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_LD4_Pin GPIO_PIN_12
#define LED_LD4_GPIO_Port GPIOD
#define TEST_Pin GPIO_PIN_15
#define TEST_GPIO_Port GPIOD
#define CS43L22_reset_Pin GPIO_PIN_4
#define CS43L22_reset_GPIO_Port GPIOD
#define SCL_Pin GPIO_PIN_6
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_9
#define SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
