/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include <string.h>
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t rxchar, txchar;

extern DAC_HandleTypeDef hdac;

//ADC scaling coefficients y=A_ADC_scale*ADC_value + B_ADC_scale   0...4095 -> +/- 1.000
const float A_ADC_scale = 2.0/4095.0;
const float B_ADC_scale = -1.0;

//DAC scaling coefficients for IQ
const float A_DAC_scale_IQ = (4095.0/2.0)/0.6;
const float B_DAC_scale_IQ = 4095.0/2.0;

//DAC scaling coefficients for AM
const float A_DAC_scale_AM = 6200.0;
const float B_DAC_scale_AM = 2048.0;

//DAC scaling coefficients for FM
const float A_DAC_scale_FM = (4095.0/2.0)*M_2_PI;
const float B_DAC_scale_FM = 4095.0/2.0;

extern float sine_arr[];
extern float cosine_arr[];
extern float asin_arr[];
uint8_t cnt; //look-up tables entry counter for sine_arr and cosine_arr

const float A_asin_arr_scale = (N_asin - 1.0)/2.0;
const float B_asin_arr_scale = (N_asin - 1.0)/2.0;

//IIR filters were designed in Matlab's Filter Designer Tool. It's Chebyshev’s Type I filter and it was implemented with direct form II.
//It would be much more better with high order FIR filters but it's impossible to implement it due to hardware limitation. Generally speaking IIR filter provides
//non linear phase response and it tends to non constant group delay and obviously distortions.

//fc=105 kHz for FM and fc=15 kHz for AM and CW
//Numerator filter coefficients
float b[6];
float a[5];

//IIR filters delay registers for I and Q filters
float Z_I[5];
float Z_Q[5];
float Z_audio[4];


//Numerator filter coefficients for AM audio filter 200...4.5 kHz - after decimation
const float b_AM[] = {2.92117078788578510284423828125000e-03, 0.00000000000000000000000000000000e+00, -5.84234157577157020568847656250000e-03, 0.00000000000000000000000000000000e+00, 2.92117078788578510284423828125000e-03};

//Denominator filter coefficientsfor AM audio filter 200...4.5 kHz - after decimation
const float a_AM[] = {-3.87497282028198242187500000000000e+00, 5.64039278030395507812500000000000e+00, -3.65578103065490722656250000000000e+00, 8.90361666679382324218750000000000e-01};


//Numerator filter coefficients for FM audio filter fc=10 kHz - after decimation
const float b_FM[] = {1.80640220642089843750000000000000e-02, 3.61280441284179687500000000000000e-02, 1.80640220642089843750000000000000e-02};

//Denominator filter coefficients for FM audio filter fc=10 kHz - after decimation
const float a_FM[] = {-1.64560496807098388671875000000000e+00, 7.26677656173706054687500000000000e-01};


//Numerator filter coefficients for CW audio filter fc=100 Hz - after decimation
float b_CW[] = {4.93644665766623802483081817626953e-06, 9.87289331533247604966163635253906e-06, 4.93644665766623802483081817626953e-06};

//Denominator filter coefficients for CW audio filter fc=100 Hz - after decimation
const float a_CW[] = {-1.99434518814086914062500000000000e+00, 9.94365453720092773437500000000000e-01};


//IIR filters delay registers for audio filters AM, FM and CW
float Z1_audio, Z2_audio, Z3_audio, Z4_audio;

//current and previous I and Q values
float I, Q, I_Z1, I_Z2, Q_Z1, Q_Z2;

extern uint32_t v_in_samples[];
const float K = 0.5;

//variables and constants for CW
uint16_t CW_trig_upper_level = 32;
uint8_t CW_trig_lower_level = 30; //0..255
bool CW_triggered = false;
uint8_t CW_decim_cnt;
const float CW_A_COEFF = 500.0;

//variables and constants for simple digital AGC for AM - it's more like automatic scaling rather than actual AGC
float module, module_max_tmp, AM_AGC_sig;
uint32_t AM_mod_max_cnt;

#define AM_max_cnt_sample 60000
const float AM_AGC_coeff = 0.7;

Output_demod_type_enum Demod_Type = DEMOD_FM; //demodulation type

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_spi3_tx;
extern UART_HandleTypeDef huart5;
/* USER CODE BEGIN EV */
extern uint8_t RX_buffer[RX_BUFLEN];
extern uint8_t *RX_wptr, *RX_rptr;
extern uint8_t TX_buffer[TX_BUFLEN];
extern uint8_t *TX_wptr, *TX_rptr;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi3_tx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */

  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */

  /* USER CODE END UART5_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void HAL_ADC_ConvHalfCpltCallback (ADC_HandleTypeDef * hadc)
{
	GPIOD->BSRR = 1<<15; //calculation time measurement

	uint8_t n, k;
	float sig_in, tmp, I_tmp, Q_tmp;
	for (n = 0;n < 4; n++) //processing first 4 samples from ADC
	{
		sig_in = A_ADC_scale*v_in_samples[n] + B_ADC_scale; //scaling from 0...4095 to +/-1.000
		I_tmp = sig_in*cosine_arr[cnt]; //multiplication by sine and cosine before LPF
		Q_tmp = sig_in*sine_arr[cnt];
		if (++cnt == N_cos_sin) cnt = 0;

		//I low pass filter
		for (k = 0;k<5;k++) I_tmp -= Z_I[k]*a[k];
		if (n == 3) //calculating filter's output - only in the final iteration because 4 samples from ADC gives 1 sample for detectors due to downsampling
		{
			I_Z2 = I_Z1;
			I_Z1 = I;
			I = I_tmp*b[0] + Z_I[0]*b[1] + Z_I[1]*b[2] + Z_I[2]*b[3] + Z_I[3]*b[4] + Z_I[4]*b[5];
		}
		Z_I[4] = Z_I[3];
		Z_I[3] = Z_I[2];
		Z_I[2] = Z_I[1];
		Z_I[1] = Z_I[0];
		Z_I[0] = I_tmp;

		//Q low pass filter
		for (k = 0;k<5;k++) Q_tmp -= Z_Q[k]*a[k];
		if (n == 3) //calculating filter's output - only in the final iteration because 4 samples from ADC gives 1 sample for detectors due to downsampling
		{
			Q_Z2 = Q_Z1;
			Q_Z1 = Q;
			Q = Q_tmp*b[0] + Z_Q[0]*b[1] + Z_Q[1]*b[2] + Z_Q[2]*b[3] + Z_Q[3]*b[4] + Z_Q[4]*b[5];
		}
		Z_Q[4] = Z_Q[3];
		Z_Q[3] = Z_Q[2];
		Z_Q[2] = Z_Q[1];
		Z_Q[1] = Z_Q[0];
		Z_Q[0] = Q_tmp;
	}

	float phase;
	int32_t DAC_value;

	switch(Demod_Type)
	{

	//FM discriminator
	case DEMOD_FM: //it's described in NUMERICAL FM DEMODULATION ENHANCEMENTS by Andrew J. Noga   https://apps.dtic.mil/sti/pdfs/ADA311269.pdf
			phase = K*(I_Z1*(Q - Q_Z2) - (I - I_Z2)*Q_Z1) / (I_Z1*I_Z1 + Q_Z1*Q_Z1);

			if (phase > 1.0) phase = 1.0;
			if (phase < -1.0) phase = -1.0;

			phase = asin_arr[(uint16_t) (A_asin_arr_scale*phase + B_asin_arr_scale)];

			//audio low pass filter for FM
			tmp = phase - (Z1_audio*a_FM[0] + Z2_audio*a_FM[1]);
			phase = tmp*b_FM[0] + Z1_audio*b_FM[1] + Z2_audio*b_FM[2];
			Z2_audio = Z1_audio;
			Z1_audio = tmp;

			DAC->DHR12R1 = A_DAC_scale_FM*phase + B_DAC_scale_FM; //scaling
		break;

	//AM detector
	case DEMOD_AM: //simple AM detector based on module
			module = sqrtf(I*I + Q*Q);

			//digital AGC - more likely automatic scaling
			if (module > module_max_tmp) module_max_tmp = module;
			AM_mod_max_cnt++;
			if (AM_mod_max_cnt == AM_max_cnt_sample)
			{
				AM_mod_max_cnt = 0;
				AM_AGC_sig = AM_AGC_coeff / module_max_tmp;
				module_max_tmp = -1.0;
			}

			//audio band pass filter for AM
			tmp = module;
			module = 0;
			for (k = 0;k<4;k++)
			{
				tmp -= Z_audio[k]*a_AM[k];
				module += Z_audio[k]*b_AM[k+1];
			}
			module += tmp*b_AM[0];
			Z_audio[3] = Z_audio[2];
			Z_audio[2] = Z_audio[1];
			Z_audio[1] = Z_audio[0];
			Z_audio[0] = tmp;

			DAC_value = A_DAC_scale_AM*module*AM_AGC_sig + B_DAC_scale_AM;
			if (DAC_value > 4095) DAC_value = 4095;
			if (DAC_value < 0) DAC_value = 0;
			DAC->DHR12R1 = DAC_value;
		break;

	//IQ output - just for testing and educational purposes
	case OUT_IQ:
			DAC_value = A_DAC_scale_IQ*I + B_DAC_scale_IQ;
			if (DAC_value > 4095) DAC_value = 4095;
			if (DAC_value < 0) DAC_value = 0;
			DAC->DHR12R1 = DAC_value;

			DAC_value = A_DAC_scale_IQ*Q + B_DAC_scale_IQ;
			if (DAC_value > 4095) DAC_value = 4095;
			if (DAC_value < 0) DAC_value = 0;
			DAC->DHR12R2 = DAC_value;
		break;

	//simple CW - based on comparator with hysteresis
	case DEMOD_CW:
		module = sqrtf(I*I + Q*Q);

		tmp = module - (Z1_audio*a_CW[0] + Z2_audio*a_CW[1]);
		module = (tmp*b_CW[0] + Z1_audio*b_CW[1] + Z2_audio*b_CW[2])*CW_A_COEFF;
		Z2_audio = Z1_audio;
		Z1_audio = tmp;

		if ( (module > CW_trig_upper_level) && (!CW_triggered) )
		{
			CW_triggered = true;
			DAC->DHR12R1 = 0;
		}

		if (CW_triggered)
		{
			CW_decim_cnt++;
			if (CW_decim_cnt == 107)
			{
				DAC->DHR12R1 ^= 0xFFF;
				CW_decim_cnt = 0;
			}

			if (module < CW_trig_lower_level) CW_triggered = false;
		}
		break;

	default:
		break;
	}

	GPIOD->BSRR = 1<<31; //calculation time measurement
}

void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef * hadc)
{
	GPIOD->BSRR = 1<<15; //calculation time measurement

	uint8_t n, k;
	float sig_in, tmp, I_tmp, Q_tmp;
	for (n = 4;n < 8; n++) //processing second 4 samples from ADC - the same principle of operation like in previous 4 samples
	{
		sig_in = A_ADC_scale*v_in_samples[n] + B_ADC_scale;
		I_tmp = sig_in*cosine_arr[cnt];
		Q_tmp = sig_in*sine_arr[cnt];
		if (++cnt == N_cos_sin) cnt = 0;

		for (k = 0;k<5;k++) I_tmp -= Z_I[k]*a[k];
		if (n == 7)
		{
			I_Z2 = I_Z1;
			I_Z1 = I;
			I = I_tmp*b[0] + Z_I[0]*b[1] + Z_I[1]*b[2] + Z_I[2]*b[3] + Z_I[3]*b[4] + Z_I[4]*b[5];
		}
		Z_I[4] = Z_I[3];
		Z_I[3] = Z_I[2];
		Z_I[2] = Z_I[1];
		Z_I[1] = Z_I[0];
		Z_I[0] = I_tmp;

		for (k = 0;k<5;k++) Q_tmp -= Z_Q[k]*a[k];
		if (n == 7)
		{
			Q_Z2 = Q_Z1;
			Q_Z1 = Q;
			Q = Q_tmp*b[0] + Z_Q[0]*b[1] + Z_Q[1]*b[2] + Z_Q[2]*b[3] + Z_Q[3]*b[4] + Z_Q[4]*b[5];
		}
		Z_Q[4] = Z_Q[3];
		Z_Q[3] = Z_Q[2];
		Z_Q[2] = Z_Q[1];
		Z_Q[1] = Z_Q[0];
		Z_Q[0] = Q_tmp;
	}

	float phase;
	int32_t DAC_value;

	switch(Demod_Type)
	{
	case DEMOD_FM:
			phase = K*(I_Z1*(Q - Q_Z2) - (I - I_Z2)*Q_Z1) / (I_Z1*I_Z1 + Q_Z1*Q_Z1);

			if (phase > 1.0) phase = 1.0;
			if (phase < -1.0) phase = -1.0;

			phase = asin_arr[(uint16_t) (A_asin_arr_scale*phase + B_asin_arr_scale)];

			tmp = phase - (Z1_audio*a_FM[0] + Z2_audio*a_FM[1]);
			phase = tmp*b_FM[0] + Z1_audio*b_FM[1] + Z2_audio*b_FM[2];
			Z2_audio = Z1_audio;
			Z1_audio = tmp;

			DAC->DHR12R1 = A_DAC_scale_FM*phase + B_DAC_scale_FM;
		break;

	case DEMOD_AM:
			module = sqrtf(I*I + Q*Q);

			if (module > module_max_tmp) module_max_tmp = module;
			AM_mod_max_cnt++;
			if (AM_mod_max_cnt == AM_max_cnt_sample)
			{
				AM_mod_max_cnt = 0;
				AM_AGC_sig = AM_AGC_coeff / module_max_tmp;
				module_max_tmp = -1.0;
			}

			tmp = module;
			module = 0;
			for (k = 0;k<4;k++)
			{
				tmp -= Z_audio[k]*a_AM[k];
				module += Z_audio[k]*b_AM[k+1];
			}
			module += tmp*b_AM[0];
			Z_audio[3] = Z_audio[2];
			Z_audio[2] = Z_audio[1];
			Z_audio[1] = Z_audio[0];
			Z_audio[0] = tmp;

			DAC_value = A_DAC_scale_AM*module*AM_AGC_sig + B_DAC_scale_AM;
			if (DAC_value > 4095) DAC_value = 4095;
			if (DAC_value < 0) DAC_value = 0;
			DAC->DHR12R1 = DAC_value;
		break;

	case OUT_IQ:
			DAC_value = A_DAC_scale_IQ*I + B_DAC_scale_IQ;
			if (DAC_value > 4095) DAC_value = 4095;
			if (DAC_value < 0) DAC_value = 0;
			DAC->DHR12R1 = DAC_value;

			DAC_value = A_DAC_scale_IQ*Q + B_DAC_scale_IQ;
			if (DAC_value > 4095) DAC_value = 4095;
			if (DAC_value < 0) DAC_value = 0;
			DAC->DHR12R2 = DAC_value;
		break;

	case DEMOD_CW:
		module = sqrtf(I*I + Q*Q);

		tmp = module - (Z1_audio*a_CW[0] + Z2_audio*a_CW[1]);
		module = (tmp*b_CW[0] + Z1_audio*b_CW[1] + Z2_audio*b_CW[2])*CW_A_COEFF;
		Z2_audio = Z1_audio;
		Z1_audio = tmp;

		if ( (module > CW_trig_upper_level) && (!CW_triggered) )
		{
			CW_triggered = true;
			DAC->DHR12R1 = 0;
		}

		if (CW_triggered)
		{
			CW_decim_cnt++;
			if (CW_decim_cnt == 107)
			{
				DAC->DHR12R1 ^= 0xFFF;
				CW_decim_cnt = 0;
			}

			if (module < CW_trig_lower_level) CW_triggered = false;
		}
		break;

	default:
		break;
	}

	GPIOD->BSRR = 1<<31; //calculation time measurement
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t rxchar_tmp = rxchar;
	HAL_UART_Receive_IT(huart, &rxchar, 1);

	/* check if there's room in the buffer */
	if((RX_wptr != RX_rptr-1) &&
	   (RX_wptr - RX_rptr != (RX_BUFLEN-1)))
	{
		/* Yes - Queue the new char */
		*RX_wptr++ = rxchar_tmp;

		/* Wrap pointer */
		if((RX_wptr - &RX_buffer[0])>=RX_BUFLEN)
			RX_wptr = &RX_buffer[0];
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	/* check if there's data in the buffer */
	if(TX_rptr != TX_wptr)
	{
		/* get data out of the buffer */
		txchar = *TX_rptr++;
		//HAL_UART_Transmit_IT(huart, TX_rptr++, 1);

		/* wrap the pointer */
		if((TX_rptr - &TX_buffer[0])>=TX_BUFLEN)
			TX_rptr = &TX_buffer[0];

		/* send the data */
		HAL_UART_Transmit_IT(huart, &txchar, 1);
	}
}
/* USER CODE END 1 */
