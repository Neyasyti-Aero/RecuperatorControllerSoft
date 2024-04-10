/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "ds18b20.h"
#include "temperature.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEMPERATURE_SENSORS_COUNT 4
uint8_t TEMPERATURE_SENSORS_ID[TEMPERATURE_SENSORS_COUNT][8] = {
{0x28, 0x35, 0x15, 0x28, 0x0D, 0x00, 0x00, 0xCA},
{0x28, 0xA6, 0x01, 0x28, 0x0D, 0x00, 0x00, 0x41},
{0x28, 0xC8, 0x6E, 0x28, 0x0D, 0x00, 0x00, 0xEF},
{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
};

#define RECIEVE_COMMAND_UART huart3
#define SEND_RESPONSE_UART huart3


// Servo Defines
#define SERVO1_MIN_VALUE 1000
#define SERVO1_MAX_VALUE 2000
#define SERVO1_PWM_TIM htim2
#define SERVO1_PWM_TIM_INSTANCE SERVO1_PWM_TIM.Instance
#define SERVO1_PWM_CHANNEL TIM_CHANNEL_2

#define SERVO2_MIN_VALUE 1000
#define SERVO2_MAX_VALUE 2000
#define SERVO2_PWM_TIM htim2
#define SERVO2_PWM_TIM_INSTANCE SERVO2_PWM_TIM.Instance
#define SERVO2_PWM_CHANNEL TIM_CHANNEL_3

#define SERVO_SPEED_TIM htim6
#define SERVO_SPEED_TIM_INSTANCE SERVO_SPEED_TIM.Instance
#define SERVO_SPEED_TIM_CYCLES_COUNT 50 // lower value <=> faster turning

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
DS18B20 temperatureSensors[TEMPERATURE_SENSORS_COUNT];

uint8_t g_rxdata[256];
uint8_t g_rxbyte[1];
uint8_t g_iRxBufferPos;
uint8_t g_txdata[256];
uint8_t g_bReadyToParse;
uint8_t g_bNeedToRespond;

float g_fValveCommand[2];
float g_fEngineCommand[2];
int g_bRelayCommand;

uint32_t g_iServoCycleCount = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define MOTOR1 0
#define MOTOR2 1

// Motor timers defines

#define MOTOR1_GP_TIM htim4
#define MOTOR1_PWM_TIM htim1
#define MOTOR1_PWM_TIM_AF GPIO_AF1_TIM1
#define MOTOR1_PWM_TIM_INSTANCE MOTOR1_PWM_TIM.Instance
#define MOTOR1_GP_TIM_INSTANCE MOTOR1_GP_TIM.Instance

#define MOTOR2_GP_TIM htim5
#define MOTOR2_PWM_TIM htim8
#define MOTOR2_PWM_TIM_AF GPIO_AF3_TIM8
#define MOTOR2_PWM_TIM_INSTANCE MOTOR2_PWM_TIM.Instance
#define MOTOR2_GP_TIM_INSTANCE MOTOR2_GP_TIM.Instance

#define PWM_TIM_PRESCALER 1
#define PWM_PERIOD 875 // 3500 === 24 kHz
#define PWM_START_PULSE 190 // 1 microsecond = pulse 120, 900 <=> 8000 mA
#define PWM_START_CYCLE_COUNT 500 // 1 cycle = 0.000025 second, 40 kHz, commutation-related
#define PWM_START_TO_NORMAL_TRANSITION_CYCLE_COUNT 150
#define PWM_START_DEADTIME 0.05f // 0 ... 1, commutation-related
#define PWM_START_COMPLEMENTARY_DEADTIME 50 // pwm-related, https://hasanyavuz.ozderya.net/?p=437
#define ENGINE_MAX_CYCLE_COUNT 18

// Motor 1 commutation defines

#define M1_U_CH TIM_CHANNEL_3
#define M1_U_H &MOTOR1_PWM_TIM, M1_U_CH
#define M1_U_PLUS_ON PinToAF(M1_U_L, &MOTOR1_PWM_TIM); HAL_TIMEx_PWMN_Start(M1_U_H); HAL_TIM_PWM_Start(M1_U_H)
#define M1_U_PLUS_OFF HAL_TIM_PWM_Stop(M1_U_H); HAL_TIMEx_PWMN_Stop(M1_U_H); PinToPP(M1_U_L)

#define M1_V_CH TIM_CHANNEL_2
#define M1_V_H &MOTOR1_PWM_TIM, M1_V_CH
#define M1_V_PLUS_ON PinToAF(M1_V_L, &MOTOR1_PWM_TIM); HAL_TIMEx_PWMN_Start(M1_V_H); HAL_TIM_PWM_Start(M1_V_H)
#define M1_V_PLUS_OFF HAL_TIM_PWM_Stop(M1_V_H); HAL_TIMEx_PWMN_Stop(M1_V_H); PinToPP(M1_V_L)

#define M1_W_CH TIM_CHANNEL_1
#define M1_W_H &MOTOR1_PWM_TIM, M1_W_CH
#define M1_W_PLUS_ON PinToAF(M1_W_L, &MOTOR1_PWM_TIM); HAL_TIMEx_PWMN_Start(M1_W_H); HAL_TIM_PWM_Start(M1_W_H)
#define M1_W_PLUS_OFF HAL_TIM_PWM_Stop(M1_W_H); HAL_TIMEx_PWMN_Stop(M1_W_H); PinToPP(M1_W_L)

#define M1_U_L M1_U_L_GPIO_Port, M1_U_L_Pin
#define M1_U_MINUS_ON HAL_GPIO_WritePin(M1_U_L, GPIO_PIN_SET)
#define M1_U_MINUS_OFF HAL_GPIO_WritePin(M1_U_L, GPIO_PIN_RESET)

#define M1_V_L M1_V_L_GPIO_Port, M1_V_L_Pin
#define M1_V_MINUS_ON HAL_GPIO_WritePin(M1_V_L, GPIO_PIN_SET)
#define M1_V_MINUS_OFF HAL_GPIO_WritePin(M1_V_L, GPIO_PIN_RESET)

#define M1_W_L M1_W_L_GPIO_Port, M1_W_L_Pin
#define M1_W_MINUS_ON HAL_GPIO_WritePin(M1_W_L, GPIO_PIN_SET)
#define M1_W_MINUS_OFF HAL_GPIO_WritePin(M1_W_L, GPIO_PIN_RESET)

// Motor 2 commutation defines

#define M2_U_CH TIM_CHANNEL_3
#define M2_U_H &MOTOR2_PWM_TIM, M2_U_CH
#define M2_U_PLUS_ON PinToAF(M2_U_L, &MOTOR2_PWM_TIM);HAL_TIMEx_PWMN_Start(M2_U_H);HAL_TIM_PWM_Start(M2_U_H)
#define M2_U_PLUS_OFF HAL_TIM_PWM_Stop(M2_U_H);HAL_TIMEx_PWMN_Stop(M2_U_H);PinToPP(M2_U_L)

#define M2_V_CH TIM_CHANNEL_2
#define M2_V_H &MOTOR2_PWM_TIM, M2_V_CH
#define M2_V_PLUS_ON PinToAF(M2_V_L, &MOTOR2_PWM_TIM);HAL_TIMEx_PWMN_Start(M2_V_H);HAL_TIM_PWM_Start(M2_V_H)
#define M2_V_PLUS_OFF HAL_TIM_PWM_Stop(M2_V_H);HAL_TIMEx_PWMN_Stop(M2_V_H);PinToPP(M2_V_L)

#define M2_W_CH TIM_CHANNEL_1
#define M2_W_H &MOTOR2_PWM_TIM, M2_W_CH
#define M2_W_PLUS_ON PinToAF(M2_W_L, &MOTOR2_PWM_TIM);HAL_TIMEx_PWMN_Start(M2_W_H);HAL_TIM_PWM_Start(M2_W_H)
#define M2_W_PLUS_OFF HAL_TIM_PWM_Stop(M2_W_H);HAL_TIMEx_PWMN_Stop(M2_W_H);PinToPP(M2_W_L)

#define M2_U_L M2_U_L_GPIO_Port, M2_U_L_Pin
#define M2_U_MINUS_ON HAL_GPIO_WritePin(M2_U_L, GPIO_PIN_SET)
#define M2_U_MINUS_OFF HAL_GPIO_WritePin(M2_U_L, GPIO_PIN_RESET)

#define M2_V_L M2_V_L_GPIO_Port, M2_V_L_Pin
#define M2_V_MINUS_ON HAL_GPIO_WritePin(M2_V_L, GPIO_PIN_SET)
#define M2_V_MINUS_OFF HAL_GPIO_WritePin(M2_V_L, GPIO_PIN_RESET)

#define M2_W_L M2_W_L_GPIO_Port, M2_W_L_Pin
#define M2_W_MINUS_ON HAL_GPIO_WritePin(M2_W_L, GPIO_PIN_SET)
#define M2_W_MINUS_OFF HAL_GPIO_WritePin(M2_W_L, GPIO_PIN_RESET)

// ADC-BEMF

#define MOTOR1_ADC_SAMPLE_TIME ADC_SAMPLETIME_3CYCLES
#define MOTOR1_ADC hadc1
#define MOTOR1_ADC_INSTANCE MOTOR1_ADC.Instance
#define MOTOR1_ADC_DELAY_CHANNEL TIM_CHANNEL_4
#define M1_U_BEMF_CHANNEL ADC_CHANNEL_12
#define M1_V_BEMF_CHANNEL ADC_CHANNEL_11
#define M1_W_BEMF_CHANNEL ADC_CHANNEL_10

#define MOTOR2_ADC_SAMPLE_TIME ADC_SAMPLETIME_3CYCLES
#define MOTOR2_ADC hadc2
#define MOTOR2_ADC_INSTANCE MOTOR1_ADC.Instance
#define MOTOR2_ADC_DELAY_CHANNEL TIM_CHANNEL_4
#define M2_U_BEMF_CHANNEL ADC_CHANNEL_15
#define M2_V_BEMF_CHANNEL ADC_CHANNEL_14
#define M2_W_BEMF_CHANNEL ADC_CHANNEL_13

#define ADC_BEMF_DELAY_CYCLES_COUNT 85

// Modes

#define MOTOR_MODE_STOP 0
#define MOTOR_MODE_START 1
#define MOTOR_MODE_NORMAL 2

// globals

uint8_t g_iCurrentMode[2];
uint8_t g_iCurrentCommutationStep[2];
uint32_t g_iCurrentCycleNum[2];
uint32_t g_iTotalCycleCount[2]; // 1 cycle = 0.00005 second, 20 kHz
float g_fCommutationDeadtime[2];
uint8_t g_bNeedNewCommutation[2];

void PinToPP(GPIO_TypeDef *port, uint16_t pin)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(port, &GPIO_InitStruct);
}

void PinToAF(GPIO_TypeDef *port, uint16_t pin, TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);

	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	if (htim->Instance == MOTOR1_PWM_TIM_INSTANCE)
		GPIO_InitStruct.Alternate = MOTOR1_PWM_TIM_AF;
	else if (htim->Instance == MOTOR2_PWM_TIM_INSTANCE)
		GPIO_InitStruct.Alternate = MOTOR2_PWM_TIM_AF;
	HAL_GPIO_Init(port, &GPIO_InitStruct);
}

void LudwigVanBeethoven(uint8_t motorx)
{
	TIM_HandleTypeDef htim;
	if (motorx == MOTOR1)
	{
		htim = MOTOR1_PWM_TIM;
	}
	else if (motorx == MOTOR2)
	{
		htim = MOTOR2_PWM_TIM;
	}
	else
	{
		HAL_Delay(5000);
		return;
	}
	// u+ v- w0 first part
	// u+ v0 w- reprise
	// u0 v+ w- second part
	// u- v+ w0 refren
	// u- v0 w+	third part
	// u0 v- w+ refren
	
	if (motorx) { M2_V_MINUS_ON; } else { M1_V_MINUS_ON; }
	
	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 44998; // 622.25 Hz, D#
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);
	
	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 44998; // 622.25 Hz, D#
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 56693; // 493.88 Hz, H
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 47673; // 587.33 Hz, D
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 53512; // 523.25 Hz, C
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 63636; // 440.00 Hz, A
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250 * 3);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 64212; // 261.63 Hz, C
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 50966; // 329.63 Hz, E
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 63636; // 440.00 Hz, A
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 56693; // 493.88 Hz, H
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250 * 3);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 50966; // 329.63 Hz, E
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 40453; // 415.30 Hz, G#
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 56693; // 493.88 Hz, H
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 53512; // 523.25 Hz, C
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250 * 3);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 50966; // 329.63 Hz, E
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);
	
	if (motorx) { M2_V_MINUS_OFF; } else { M1_V_MINUS_OFF; }
	
	
	// Reprise
	
	
	if (motorx) { M2_W_MINUS_ON; } else { M1_W_MINUS_ON; }

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 44998; // 622.25 Hz, D#
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);
	
	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 44998; // 622.25 Hz, D#
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 56693; // 493.88 Hz, H
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 47673; // 587.33 Hz, D
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 53512; // 523.25 Hz, C
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 63636; // 440.00 Hz, A
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250 * 3);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 64212; // 261.63 Hz, C
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 50966; // 329.63 Hz, E
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 63636; // 440.00 Hz, A
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 56693; // 493.88 Hz, H
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250 * 3);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 50966; // 329.63 Hz, E
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 53512; // 523.25 Hz, C
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 56693; // 493.88 Hz, H
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);
	
	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 63636; // 440.00 Hz, A
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	HAL_Delay(250 * 3);
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	HAL_Delay(1);
	
	
	// Second Part
	
	
	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 56693; // 493.88 Hz, H
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 53512; // 523.25 Hz, C
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 47673; // 587.33 Hz, D
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250 * 3);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42857; // 392.00 Hz, G
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 40088; // 698.46 Hz, F
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 47673; // 587.33 Hz, D
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250 * 3);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 48105; // 349.23 Hz, F
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 47673; // 587.33 Hz, D
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 53512; // 523.25 Hz, C
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250 * 3);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 50966; // 329.63 Hz, E
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 47673; // 587.33 Hz, D
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 53512; // 523.25 Hz, C
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 56693; // 493.88 Hz, H
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250 * 3);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 50966; // 329.63 Hz, E
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);
	
	if (motorx) { M2_W_MINUS_OFF; } else { M1_W_MINUS_OFF; }
	
	
	// Refren


	if (motorx) { M2_U_MINUS_ON; } else { M1_U_MINUS_ON; }
	
	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 44998; // 622.25 Hz, D#
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);
	
	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 44998; // 622.25 Hz, D#
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 56693; // 493.88 Hz, H
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 47673; // 587.33 Hz, D
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 53512; // 523.25 Hz, C
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 63636; // 440.00 Hz, A
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250 * 3);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 64212; // 261.63 Hz, C
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 50966; // 329.63 Hz, E
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 63636; // 440.00 Hz, A
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 56693; // 493.88 Hz, H
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250 * 3);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 50966; // 329.63 Hz, E
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 40453; // 415.30 Hz, G#
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 56693; // 493.88 Hz, H
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 53512; // 523.25 Hz, C
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250 * 3);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 50966; // 329.63 Hz, E
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);
	
	
	// Reprise
	
	
	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 44998; // 622.25 Hz, D#
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);
	
	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 44998; // 622.25 Hz, D#
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 56693; // 493.88 Hz, H
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 47673; // 587.33 Hz, D
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 53512; // 523.25 Hz, C
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 63636; // 440.00 Hz, A
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250 * 3);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 64212; // 261.63 Hz, C
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 50966; // 329.63 Hz, E
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 63636; // 440.00 Hz, A
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 56693; // 493.88 Hz, H
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250 * 3);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 50966; // 329.63 Hz, E
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 53512; // 523.25 Hz, C
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 56693; // 493.88 Hz, H
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);
	
	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 63636; // 440.00 Hz, A
	if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
	HAL_Delay(250 * 3);
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	HAL_Delay(1);
	
	
	// Third part
	
	
	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 64212; // 261.63 Hz, C
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 48105; // 349.23 Hz, F
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42857; // 392.00 Hz, G
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 63636; // 440.00 Hz, A
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250 * 4);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 40088; // 698.46 Hz, F
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250 + 150);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(100);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250 * 2);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 47673; // 587.33 Hz, D
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250 * 2);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 30032; // 932.32 Hz, B
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250 + 150);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 31818; // 880.00 Hz, A
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(100);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 31818; // 880.00 Hz, A
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 35715; // 783.99 Hz, G
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 40088; // 698.46 Hz, F
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 47673; // 587.33 Hz, D
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 53512; // 523.25 Hz, C
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 60065; // 466.16 Hz, B
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250 * 2);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 63636; // 440.00 Hz, A
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250 * 2);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 60065; // 466.16 Hz, B
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(62);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 63636; // 440.00 Hz, A
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42857; // 392.00 Hz, G
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 63636; // 440.00 Hz, A
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 60065; // 466.16 Hz, B
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 53512; // 523.25 Hz, C
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250 * 4);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 47673; // 587.33 Hz, D
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 44998; // 622.25 Hz, D#
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250 * 3);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 40088; // 698.46 Hz, F
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 63636; // 440.00 Hz, A
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 53512; // 523.25 Hz, C
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250 * 4);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 47673; // 587.33 Hz, D
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250 + 150);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 56693; // 493.88 Hz, H
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(100);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 53512; // 523.25 Hz, C
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 35715; // 783.99 Hz, G
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42857; // 392.00 Hz, G
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 35715; // 783.99 Hz, G
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 63636; // 440.00 Hz, A
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 35715; // 783.99 Hz, G
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);
	
	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 56693; // 493.88 Hz, H
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);
	
	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 35715; // 783.99 Hz, G
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);
	
	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 53512; // 523.25 Hz, C
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 35715; // 783.99 Hz, G
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 47673; // 587.33 Hz, D
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 35715; // 783.99 Hz, G
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 35715; // 783.99 Hz, G
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 26756; // 1046.5 Hz, C
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 28347; // 987.77 Hz, H
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 31818; // 880.00 Hz, A
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 75);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 35715; // 783.99 Hz, G
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 40088; // 698.46 Hz, F
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 47673; // 587.33 Hz, D
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 75);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 35715; // 783.99 Hz, G
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 40088; // 698.46 Hz, F
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 47673; // 587.33 Hz, D
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);


	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 53512; // 523.25 Hz, C
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 35715; // 783.99 Hz, G
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42857; // 392.00 Hz, G
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 35715; // 783.99 Hz, G
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 63636; // 440.00 Hz, A
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 35715; // 783.99 Hz, G
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);
	
	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 56693; // 493.88 Hz, H
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);
	
	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 35715; // 783.99 Hz, G
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);
	
	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 53512; // 523.25 Hz, C
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 35715; // 783.99 Hz, G
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 47673; // 587.33 Hz, D
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 35715; // 783.99 Hz, G
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 35715; // 783.99 Hz, G
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 26756; // 1046.5 Hz, C
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 28347; // 987.77 Hz, H
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 31818; // 880.00 Hz, A
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 75);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 35715; // 783.99 Hz, G
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 40088; // 698.46 Hz, F
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 47673; // 587.33 Hz, D
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 75);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 35715; // 783.99 Hz, G
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 40088; // 698.46 Hz, F
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 47673; // 587.33 Hz, D
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 40088; // 698.46 Hz, F
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 44998; // 622.25 Hz, D#
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 56693; // 493.88 Hz, H
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);
	
	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 44998; // 622.25 Hz, D#
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 56693; // 493.88 Hz, H
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);
	
	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 44998; // 622.25 Hz, D#
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(125);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250 * 3);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 56693; // 493.88 Hz, H
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);
	
	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 44998; // 622.25 Hz, D#
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250 * 3);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 56693; // 493.88 Hz, H
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);
	
	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 44998; // 622.25 Hz, D#
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 44998; // 622.25 Hz, D#
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 44998; // 622.25 Hz, D#
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 44998; // 622.25 Hz, D#
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 44998; // 622.25 Hz, D#
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 56693; // 493.88 Hz, H
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 47673; // 587.33 Hz, D
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 53512; // 523.25 Hz, C
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 63636; // 440.00 Hz, A
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250 * 3);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);
	
	if (motorx) { M2_U_MINUS_OFF; } else { M1_U_MINUS_OFF; }
	
	
	// Refren
	
	
	if (motorx) { M2_V_MINUS_ON; } else { M1_V_MINUS_ON; }

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 64212; // 261.63 Hz, C
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 50966; // 329.63 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 63636; // 440.00 Hz, A
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 56693; // 493.88 Hz, H
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250 * 3);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 50966; // 329.63 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 40453; // 415.30 Hz, G#
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 56693; // 493.88 Hz, H
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 53512; // 523.25 Hz, C
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250 * 3);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 50966; // 329.63 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);
	
	
	// Reprise
	
	
	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 44998; // 622.25 Hz, D#
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);
	
	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 44998; // 622.25 Hz, D#
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 42471; // 659.26 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 56693; // 493.88 Hz, H
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 47673; // 587.33 Hz, D
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 53512; // 523.25 Hz, C
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 63636; // 440.00 Hz, A
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250 * 3);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 64212; // 261.63 Hz, C
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 50966; // 329.63 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 63636; // 440.00 Hz, A
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 56693; // 493.88 Hz, H
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250 * 3);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 50);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 4;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 50966; // 329.63 Hz, E
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 53512; // 523.25 Hz, C
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 56693; // 493.88 Hz, H
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);
	
	__HAL_TIM_SetCounter(&htim, 0);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, 100);
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = 2;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = 63636; // 440.00 Hz, A
	if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
	HAL_Delay(250 * 6);
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	HAL_Delay(1);

	if (motorx) { M2_V_MINUS_OFF; } else { M1_V_MINUS_OFF; }
}





void ProcessServos()
{
	if (g_iServoCycleCount > SERVO_SPEED_TIM_CYCLES_COUNT)
	{
		uint16_t desiredValue;
		uint16_t currentValue;
		
		desiredValue = (int)(0.01f * g_fValveCommand[0] * (SERVO1_MAX_VALUE - SERVO1_MIN_VALUE) + SERVO1_MIN_VALUE);
		currentValue = __HAL_TIM_GET_COMPARE(&SERVO1_PWM_TIM, SERVO1_PWM_CHANNEL);
		if (currentValue > desiredValue + 2)
			__HAL_TIM_SET_COMPARE(&SERVO1_PWM_TIM, SERVO1_PWM_CHANNEL, currentValue - 1);
		else if (currentValue < desiredValue - 2)
			__HAL_TIM_SET_COMPARE(&SERVO1_PWM_TIM, SERVO1_PWM_CHANNEL, currentValue + 1);
			
		
		desiredValue = (int)(0.01f * g_fValveCommand[1] * (SERVO2_MAX_VALUE - SERVO2_MIN_VALUE) + SERVO2_MIN_VALUE);
		currentValue = __HAL_TIM_GET_COMPARE(&SERVO2_PWM_TIM, SERVO2_PWM_CHANNEL);
		if (currentValue > desiredValue + 2)
			__HAL_TIM_SET_COMPARE(&SERVO2_PWM_TIM, SERVO2_PWM_CHANNEL, currentValue - 1);
		else if (currentValue < desiredValue - 2)
			__HAL_TIM_SET_COMPARE(&SERVO2_PWM_TIM, SERVO2_PWM_CHANNEL, currentValue + 1);
		
		g_iServoCycleCount = 0;
	}
	else
		g_iServoCycleCount++;
}

/*
Step 0: u+ v- w0
Step 1: u+ v0 w-
Step 2: u0 v+ w-
Step 3: u- v+ w0
Step 4: u- v0 w+
Step 5: u0 v- w+
*/
void DoCommutationNextStep(uint8_t motorx)
{
	switch (g_iCurrentCommutationStep[motorx])
	{
		case 0:
			if (g_iCurrentCycleNum[motorx] > g_iTotalCycleCount[motorx] || g_bNeedNewCommutation[motorx])
			{
				if (motorx) { M2_V_MINUS_OFF; } else { M1_V_MINUS_OFF; }
				g_iCurrentCycleNum[motorx] = 0;
				g_iCurrentCommutationStep[motorx] = 1;
				if (g_iCurrentMode[motorx] == MOTOR_MODE_NORMAL)
				{
					ADC_ChannelConfTypeDef sConfig = {0};
					
					sConfig.Channel = motorx ? M2_U_BEMF_CHANNEL : M1_U_BEMF_CHANNEL;
					sConfig.Rank = 1;
					sConfig.SamplingTime = motorx ? MOTOR2_ADC_SAMPLE_TIME : MOTOR1_ADC_SAMPLE_TIME;
					HAL_ADC_ConfigChannel(motorx ? &MOTOR2_ADC : &MOTOR1_ADC, &sConfig);
					
					sConfig.Channel = motorx ? M2_V_BEMF_CHANNEL : M1_V_BEMF_CHANNEL;
					sConfig.Rank = 2;
					HAL_ADC_ConfigChannel(motorx ? &MOTOR2_ADC : &MOTOR1_ADC, &sConfig);
				}
				if (motorx) { M2_W_MINUS_ON; } else { M1_W_MINUS_ON; }
			}
			break;	
		case 1:
			if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
			if (motorx) { M2_U_MINUS_ON; } else { M1_U_MINUS_ON; }
			if (g_iCurrentCycleNum[motorx] > g_iTotalCycleCount[motorx] || g_bNeedNewCommutation[motorx])
			{
				if (motorx) { M2_U_MINUS_OFF; } else { M1_U_MINUS_OFF; }
				g_iCurrentCycleNum[motorx] = 0;
				g_iCurrentCommutationStep[motorx] = 2;
				if (g_iCurrentMode[motorx] == MOTOR_MODE_NORMAL)
				{
					ADC_ChannelConfTypeDef sConfig = {0};
					
					sConfig.Channel = motorx ? M2_V_BEMF_CHANNEL : M1_V_BEMF_CHANNEL;
					sConfig.Rank = 1;
					sConfig.SamplingTime = motorx ? MOTOR2_ADC_SAMPLE_TIME : MOTOR1_ADC_SAMPLE_TIME;
					HAL_ADC_ConfigChannel(motorx ? &MOTOR2_ADC : &MOTOR1_ADC, &sConfig);
					
					sConfig.Channel = motorx ? M2_U_BEMF_CHANNEL : M1_U_BEMF_CHANNEL;
					sConfig.Rank = 2;
					HAL_ADC_ConfigChannel(motorx ? &MOTOR2_ADC : &MOTOR1_ADC, &sConfig);
				}
				if (motorx) { M2_V_PLUS_ON; } else { M1_V_PLUS_ON; }
			}
			break;
		case 2:
			if (g_iCurrentCycleNum[motorx] > g_iTotalCycleCount[motorx] || g_bNeedNewCommutation[motorx])
			{
				if (motorx) { M2_W_MINUS_OFF; } else { M1_W_MINUS_OFF; }
				g_iCurrentCycleNum[motorx] = 0;
				g_iCurrentCommutationStep[motorx] = 3;
				if (g_iCurrentMode[motorx] == MOTOR_MODE_NORMAL)
				{
					ADC_ChannelConfTypeDef sConfig = {0};
						
					sConfig.Channel = motorx ? M2_V_BEMF_CHANNEL : M1_V_BEMF_CHANNEL;
					sConfig.Rank = 1;
					sConfig.SamplingTime = motorx ? MOTOR2_ADC_SAMPLE_TIME : MOTOR1_ADC_SAMPLE_TIME;
					HAL_ADC_ConfigChannel(motorx ? &MOTOR2_ADC : &MOTOR1_ADC, &sConfig);
					
					sConfig.Channel = motorx ? M2_W_BEMF_CHANNEL : M1_W_BEMF_CHANNEL;
					sConfig.Rank = 2;
					HAL_ADC_ConfigChannel(motorx ? &MOTOR2_ADC : &MOTOR1_ADC, &sConfig);
				}
				if (motorx) { M2_U_MINUS_ON; } else { M1_U_MINUS_ON; }
			}
			break;
		case 3:
			if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
			if (motorx) { M2_V_MINUS_ON; } else { M1_V_MINUS_ON; }
			if (g_iCurrentCycleNum[motorx] > g_iTotalCycleCount[motorx] || g_bNeedNewCommutation[motorx])
			{
				if (motorx) { M2_V_MINUS_OFF; } else { M1_V_MINUS_OFF; }
				g_iCurrentCycleNum[motorx] = 0;
				g_iCurrentCommutationStep[motorx] = 4;
				if (g_iCurrentMode[motorx] == MOTOR_MODE_NORMAL)
				{
					ADC_ChannelConfTypeDef sConfig = {0};
					
					sConfig.Channel = motorx ? M2_W_BEMF_CHANNEL : M1_W_BEMF_CHANNEL;
					sConfig.Rank = 1;
					sConfig.SamplingTime = motorx ? MOTOR2_ADC_SAMPLE_TIME : MOTOR1_ADC_SAMPLE_TIME;
					HAL_ADC_ConfigChannel(motorx ? &MOTOR2_ADC : &MOTOR1_ADC, &sConfig);
					
					sConfig.Channel = motorx ? M2_V_BEMF_CHANNEL : M1_V_BEMF_CHANNEL;
					sConfig.Rank = 2;
					HAL_ADC_ConfigChannel(motorx ? &MOTOR2_ADC : &MOTOR1_ADC, &sConfig);
				}
				if (motorx) { M2_W_PLUS_ON; } else { M1_W_PLUS_ON; }
			}
			break;
		case 4:
			if (g_iCurrentCycleNum[motorx] > g_iTotalCycleCount[motorx] || g_bNeedNewCommutation[motorx])
			{
				if (motorx) { M2_U_MINUS_OFF; } else { M1_U_MINUS_OFF; }
				g_iCurrentCycleNum[motorx] = 0;
				g_iCurrentCommutationStep[motorx] = 5;
				if (g_iCurrentMode[motorx] == MOTOR_MODE_NORMAL)
				{
					ADC_ChannelConfTypeDef sConfig = {0};
					
					sConfig.Channel = motorx ? M2_W_BEMF_CHANNEL : M1_W_BEMF_CHANNEL;
					sConfig.Rank = 1;
					sConfig.SamplingTime = motorx ? MOTOR2_ADC_SAMPLE_TIME : MOTOR1_ADC_SAMPLE_TIME;
					HAL_ADC_ConfigChannel(motorx ? &MOTOR2_ADC : &MOTOR1_ADC, &sConfig);
					
					sConfig.Channel = motorx ? M2_U_BEMF_CHANNEL : M1_U_BEMF_CHANNEL;
					sConfig.Rank = 2;
					HAL_ADC_ConfigChannel(motorx ? &MOTOR2_ADC : &MOTOR1_ADC, &sConfig);
				}
				if (motorx) { M2_V_MINUS_ON; } else { M1_V_MINUS_ON; }
			}
			break;
		case 5:
			if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
			if (motorx) { M2_W_MINUS_ON; } else { M1_W_MINUS_ON; }
			if (g_iCurrentCycleNum[motorx] > g_iTotalCycleCount[motorx] || g_bNeedNewCommutation[motorx])
			{
				if (motorx) { M2_W_MINUS_OFF; } else { M1_W_MINUS_OFF; }
				g_iCurrentCycleNum[motorx] = 0;
				g_iCurrentCommutationStep[motorx] = 0;
				if (g_iCurrentMode[motorx] == MOTOR_MODE_NORMAL)
				{
					ADC_ChannelConfTypeDef sConfig = {0};
					
					sConfig.Channel = motorx ? M2_U_BEMF_CHANNEL : M1_U_BEMF_CHANNEL;
					sConfig.Rank = 1;
					sConfig.SamplingTime = motorx ? MOTOR2_ADC_SAMPLE_TIME : MOTOR1_ADC_SAMPLE_TIME;
					HAL_ADC_ConfigChannel(motorx ? &MOTOR2_ADC : &MOTOR1_ADC, &sConfig);
					
					sConfig.Channel = motorx ? M2_W_BEMF_CHANNEL : M1_W_BEMF_CHANNEL;
					sConfig.Rank = 2;
					HAL_ADC_ConfigChannel(motorx ? &MOTOR2_ADC : &MOTOR1_ADC, &sConfig);
				}
				if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
			}
			break;
	}
	g_bNeedNewCommutation[motorx] = 0;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	uint16_t powerVoltage = HAL_ADC_GetValue(hadc);
	uint16_t bemfVoltage = HAL_ADC_GetValue(hadc);

	// bad value filter
	// when voltage on powered <= voltage on open phase
	// when one of voltages is zero
	// when powered voltage is too close to zero
	if (powerVoltage <= bemfVoltage)
		return;
	if (bemfVoltage == 0)
		return;
	if (powerVoltage <= 677)
		return;

	uint8_t motorx = hadc->Instance == MOTOR1_ADC_INSTANCE ? MOTOR1 : MOTOR2;

	// 0, 2, 4 - upward bemf
	// 1, 3, 5 - downward bemf
	if (g_iCurrentCommutationStep[motorx] == 0 || g_iCurrentCommutationStep[motorx] == 2 || g_iCurrentCommutationStep[motorx] == 4)
	{
		if (bemfVoltage > powerVoltage / 2)
		{
			g_bNeedNewCommutation[MOTOR1] = 1;
			DoCommutationNextStep(motorx);
		}
	}
	else if (g_iCurrentCommutationStep[motorx] == 1 || g_iCurrentCommutationStep[motorx] == 3 || g_iCurrentCommutationStep[motorx] == 5)
	{
		if (bemfVoltage < powerVoltage / 2)
		{
			g_bNeedNewCommutation[MOTOR1] = 1;
			DoCommutationNextStep(motorx);
		}
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == MOTOR1_PWM_TIM_INSTANCE)
	{
		HAL_ADC_Start_IT(&MOTOR1_ADC);
		return;
	}
}

// GP timer callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	uint8_t motorx;
	if (htim->Instance == MOTOR1_GP_TIM_INSTANCE)
		motorx = MOTOR1;
	else if (htim->Instance == MOTOR2_GP_TIM_INSTANCE)
		motorx = MOTOR2;
	else if (htim->Instance == SERVO_SPEED_TIM_INSTANCE)
	{
		ProcessServos();
		return;
	}
	else
		return;
	
	g_iCurrentCycleNum[motorx]++;
	
	if (g_iCurrentCycleNum[motorx] > (1 - g_fCommutationDeadtime[motorx]) * g_iTotalCycleCount[motorx])
	{
		if (g_iCurrentMode[motorx] == MOTOR_MODE_START)
		{
			if (!g_iCurrentCommutationStep[motorx] && g_iTotalCycleCount[motorx] > PWM_START_TO_NORMAL_TRANSITION_CYCLE_COUNT)
			{
				g_iTotalCycleCount[motorx]--;
			}
			else if (g_iTotalCycleCount[motorx] <= PWM_START_TO_NORMAL_TRANSITION_CYCLE_COUNT)
			{
				g_iCurrentMode[motorx] = MOTOR_MODE_NORMAL;
			}
			
			// DoNextStep should be called once, AND RIGHT THERE!!!
			DoCommutationNextStep(motorx);
			
			if (g_iCurrentMode[motorx] == MOTOR_MODE_NORMAL) // transition to normal completed in this cycle
			{
				//__HAL_TIM_SetCompare(motorx ? &MOTOR2_PWM_TIM : &MOTOR1_PWM_TIM, motorx ? MOTOR2_ADC_DELAY_CHANNEL : MOTOR1_ADC_DELAY_CHANNEL, ADC_BEMF_DELAY_CYCLES_COUNT);
				//HAL_TIM_OC_Start_IT(motorx ? &MOTOR2_PWM_TIM : &MOTOR1_PWM_TIM, motorx ? MOTOR2_ADC_DELAY_CHANNEL : MOTOR1_ADC_DELAY_CHANNEL);
			}
		}
		else if (g_iCurrentMode[motorx] == MOTOR_MODE_NORMAL)
		{
			DoCommutationNextStep(motorx);
		}
	}
}

void StopMotorPWM(uint8_t motorx)
{
	TIM_HandleTypeDef htim = motorx ? MOTOR2_GP_TIM : MOTOR1_GP_TIM;
	HAL_TIM_Base_Stop_IT(&htim);
	HAL_TIM_OC_Stop_IT(motorx ? &MOTOR2_PWM_TIM : &MOTOR1_PWM_TIM, motorx ? MOTOR2_ADC_DELAY_CHANNEL : MOTOR1_ADC_DELAY_CHANNEL);
	
	if (motorx) { M2_U_PLUS_OFF; } else { M1_U_PLUS_OFF; }
	if (motorx) { M2_V_PLUS_OFF; } else { M1_V_PLUS_OFF; }
	if (motorx) { M2_W_PLUS_OFF; } else { M1_W_PLUS_OFF; }
	if (motorx) { M2_U_MINUS_ON; } else { M1_U_MINUS_ON; }
	if (motorx) { M2_U_MINUS_OFF; } else { M1_U_MINUS_OFF; }
	if (motorx) { M2_V_MINUS_ON; } else { M1_V_MINUS_ON; }
	if (motorx) { M2_V_MINUS_OFF; } else { M1_V_MINUS_OFF; }
	if (motorx) { M2_W_MINUS_ON; } else { M1_W_MINUS_ON; }
	if (motorx) { M2_W_MINUS_OFF; } else { M1_W_MINUS_OFF; }
	
	g_iCurrentMode[motorx] = MOTOR_MODE_STOP;
}

void SetComplementaryPWMDeadTime(TIM_HandleTypeDef *htim, uint8_t deadtime)
{
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = deadtime;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(htim, &sBreakDeadTimeConfig);
}

void StartMotorSpin(uint8_t motorx)
{
	TIM_HandleTypeDef htim;
	if (motorx == MOTOR1)
	{
		htim = MOTOR1_PWM_TIM;
	}
	else if (motorx == MOTOR2)
	{
		htim = MOTOR2_PWM_TIM;
	}
	else
	{
		return;
	}
	
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->PSC = PWM_TIM_PRESCALER;
	(motorx ? MOTOR2_PWM_TIM_INSTANCE : MOTOR1_PWM_TIM_INSTANCE)->ARR = PWM_PERIOD;
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, PWM_START_PULSE);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, PWM_START_PULSE);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, PWM_START_PULSE);
	g_iTotalCycleCount[motorx] = PWM_START_CYCLE_COUNT;
	g_fCommutationDeadtime[motorx] = PWM_START_DEADTIME;
	
	g_iCurrentCycleNum[motorx] = 0;
	g_iCurrentCommutationStep[motorx] = 0;
	if (motorx) { M2_U_PLUS_ON; } else { M1_U_PLUS_ON; }
	if (motorx) { M2_V_MINUS_ON; } else { M1_V_MINUS_ON; }
	
	SetComplementaryPWMDeadTime(&htim, PWM_START_COMPLEMENTARY_DEADTIME);
	
	htim = motorx ? MOTOR2_GP_TIM : MOTOR1_GP_TIM;
	HAL_TIM_Base_Start_IT(&htim);
	
	g_iCurrentMode[motorx] = MOTOR_MODE_START;
}

void ProcessMotor(uint8_t motorx)
{
		if (g_fEngineCommand[motorx] > 20.0f)
		{
			if (g_iCurrentMode[motorx] == MOTOR_MODE_STOP)
				StartMotorSpin(motorx);
			else if (g_iCurrentMode[motorx] == MOTOR_MODE_NORMAL)
				g_iTotalCycleCount[motorx] = (int)(0.01f * (100.0f - g_fEngineCommand[motorx]) * (PWM_START_TO_NORMAL_TRANSITION_CYCLE_COUNT - ENGINE_MAX_CYCLE_COUNT) + ENGINE_MAX_CYCLE_COUNT);
		}
		else if (g_fEngineCommand[motorx] < 15.0f)
		{
			if (g_iCurrentMode[motorx] == MOTOR_MODE_NORMAL)
				StopMotorPWM(motorx);
		}
}

void ParseRxValues()
{
	g_iRxBufferPos = 255;
	
	g_rxdata[127] = 0;

	sscanf((char*)g_rxdata, "START %f %f %f %f %i END", &g_fValveCommand[0], &g_fValveCommand[1], &g_fEngineCommand[0], &g_fEngineCommand[1], &g_bRelayCommand);
	
	g_iRxBufferPos = 0;
	g_bReadyToParse = 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (g_iRxBufferPos == 255)
	{
		HAL_UART_Receive_IT(&RECIEVE_COMMAND_UART, g_rxbyte, sizeof(g_rxbyte));
		return;
	}
	
	if (g_rxbyte[0] == 'S' || g_iRxBufferPos >= sizeof(g_rxdata))
		g_iRxBufferPos = 0;
	
	if (g_rxbyte[0] != 0x00 && g_rxbyte[0] != 0xFF && g_rxbyte[0] != 'C')
	{
		g_rxdata[g_iRxBufferPos] = g_rxbyte[0];
		g_iRxBufferPos++;
	}
	
	if (g_rxbyte[0] == 'D' && g_rxdata[0] == 'S')
		g_bReadyToParse = 1;
	
	if (g_rxbyte[0] == 'C')
		g_bNeedToRespond = 1;

	HAL_UART_Receive_IT(&RECIEVE_COMMAND_UART, g_rxbyte, sizeof(g_rxbyte));
}

void RespondToRequest()
{
	g_bNeedToRespond = 0;
	
	UpdateTemperatures(temperatureSensors, TEMPERATURE_SENSORS_COUNT);
	
	sprintf((char*)g_txdata, "START %f %f %f %f %f %f %f %f %d END", g_fValveCommand[0], g_fValveCommand[1], g_fEngineCommand[0], g_fEngineCommand[1], temperatureSensors[0].temperature, temperatureSensors[1].temperature, temperatureSensors[2].temperature, temperatureSensors[3].temperature, g_bRelayCommand);
	
	HAL_UART_Transmit_IT(&SEND_RESPONSE_UART, g_txdata, strlen((char*)g_txdata));
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	
	// Start sensors
	TemperatureInitSensors(temperatureSensors, TEMPERATURE_SENSORS_COUNT);
	
	// Start servos
	__HAL_TIM_SET_COMPARE(&SERVO1_PWM_TIM, SERVO1_PWM_CHANNEL, SERVO1_MIN_VALUE);
	HAL_TIM_PWM_Start(&SERVO1_PWM_TIM, SERVO1_PWM_CHANNEL);
	__HAL_TIM_SET_COMPARE(&SERVO2_PWM_TIM, SERVO2_PWM_CHANNEL, SERVO2_MIN_VALUE);
	HAL_TIM_PWM_Start(&SERVO2_PWM_TIM, SERVO2_PWM_CHANNEL);
	HAL_TIM_Base_Start_IT(&SERVO_SPEED_TIM);
	
	// Start recieving commands
	HAL_UART_Receive_IT(&RECIEVE_COMMAND_UART, g_rxbyte, sizeof(g_rxbyte));
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); // led D3
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); // led D2

	StopMotorPWM(MOTOR1); // used as initializer, do not comment out
	StopMotorPWM(MOTOR2); // used as initializer, do not comment out
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


	HAL_Delay(3000);
	LudwigVanBeethoven(MOTOR1);
	HAL_Delay(3000);
	LudwigVanBeethoven(MOTOR2);
	
  while (1)
  {
		if (g_bReadyToParse)
			ParseRxValues();
		
		if (g_bNeedToRespond)
			RespondToRequest();
		
		if (g_bRelayCommand)
			HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);
		
		ProcessMotor(MOTOR1);
		ProcessMotor(MOTOR2);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 2;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 63636;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 94;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 1100;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 25;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 25;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 83;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 100;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 2;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 63636;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_2_Pin|LED_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RELAY_Pin|RE_Pin|DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD10 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_2_Pin LED_3_Pin */
  GPIO_InitStruct.Pin = LED_2_Pin|LED_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY_Pin RE_Pin DE_Pin */
  GPIO_InitStruct.Pin = RELAY_Pin|RE_Pin|DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
