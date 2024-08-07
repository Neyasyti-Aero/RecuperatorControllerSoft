#include "main.h"

#include <stdio.h>
#include <string.h>
#include "ds18b20.h"
#include "temperature.h"

#define TEMPERATURE_SENSORS_COUNT 4
uint8_t TEMPERATURE_SENSORS_ID[TEMPERATURE_SENSORS_COUNT][8] = {
{0x28, 0x35, 0x15, 0x28, 0x0D, 0x00, 0x00, 0xCA},
{0x28, 0xA6, 0x01, 0x28, 0x0D, 0x00, 0x00, 0x41},
{0x28, 0xC8, 0x6E, 0x28, 0x0D, 0x00, 0x00, 0xEF},
{0x28, 0x4A, 0x38, 0x94, 0x91, 0x22, 0x01, 0x7C}
};

#define RECIEVE_COMMAND_UART huart1
#define SEND_RESPONSE_UART huart1

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

DS18B20 temperatureSensors[TEMPERATURE_SENSORS_COUNT];

uint8_t g_rxdata[256];
uint8_t g_rxbyte[1];
uint16_t g_iRxBufferPos;
uint8_t g_txdata[256];
uint8_t g_bReadyToParse;
uint8_t g_bNeedToRespond;

float g_fValveCommand[2];
float g_fEngineCommand[2];
float g_fEngineState[2];
int g_bRelayCommand;

uint32_t g_iServoCycleCount = 0;

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
#define PWM_PERIOD 4000 // 3500 === 24 kHz, 4200 === 20 kHz, 5250 === 16 kHz if HCLK === 168MHz, but 4000 === 20 kHz if HCLK === 160 MHz
#define PWM_START_PULSE 210 // 1 microsecond = pulse 120, 900 <=> 8000 mA
#define PWM_START_CYCLE_COUNT 700 // 1 cycle = 0.000025 second, 40 kHz, commutation-related
#define PWM_START_TO_NORMAL_TRANSITION_CYCLE_COUNT 350
#define PWM_START_DEADTIME 0.00f // 0 ... 1, commutation-related
#define PWM_START_COMPLEMENTARY_DEADTIME 50 // pwm-related, https://hasanyavuz.ozderya.net/?p=437
#define ENGINE_MAX_CYCLE_COUNT 76

// Motor 1 commutation defines

#define M1_U_CH TIM_CHANNEL_3
#define M1_U_H &MOTOR1_PWM_TIM, M1_U_CH
#define M1_U_PLUS_ON PinToAF(M1_U_L, &MOTOR1_PWM_TIM); PinToAF(M1_U_H_GPIO_Port, M1_U_H_Pin, &MOTOR1_PWM_TIM); HAL_TIM_PWM_Start(M1_U_H); HAL_TIMEx_PWMN_Start(M1_U_H)
#define M1_U_PLUS_OFF HAL_TIMEx_PWMN_Stop(M1_U_H); PinToPP(M1_U_L); HAL_TIM_PWM_Stop(M1_U_H); PinToPP(M1_U_H_GPIO_Port, M1_U_H_Pin); HAL_GPIO_WritePin(M1_U_H_GPIO_Port, M1_U_H_Pin, GPIO_PIN_RESET)

#define M1_V_CH TIM_CHANNEL_2
#define M1_V_H &MOTOR1_PWM_TIM, M1_V_CH
#define M1_V_PLUS_ON PinToAF(M1_V_L, &MOTOR1_PWM_TIM); PinToAF(M1_V_H_GPIO_Port, M1_V_H_Pin, &MOTOR1_PWM_TIM); HAL_TIM_PWM_Start(M1_V_H); HAL_TIMEx_PWMN_Start(M1_V_H)
#define M1_V_PLUS_OFF HAL_TIMEx_PWMN_Stop(M1_V_H); PinToPP(M1_V_L); HAL_TIM_PWM_Stop(M1_V_H); PinToPP(M1_V_H_GPIO_Port, M1_V_H_Pin); HAL_GPIO_WritePin(M1_V_H_GPIO_Port, M1_V_H_Pin, GPIO_PIN_RESET)

#define M1_W_CH TIM_CHANNEL_1
#define M1_W_H &MOTOR1_PWM_TIM, M1_W_CH
#define M1_W_PLUS_ON PinToAF(M1_W_L, &MOTOR1_PWM_TIM); PinToAF(M1_W_H_GPIO_Port, M1_W_H_Pin, &MOTOR1_PWM_TIM); HAL_TIM_PWM_Start(M1_W_H); HAL_TIMEx_PWMN_Start(M1_W_H)
#define M1_W_PLUS_OFF HAL_TIMEx_PWMN_Stop(M1_W_H); PinToPP(M1_W_L); HAL_TIM_PWM_Stop(M1_W_H); PinToPP(M1_W_H_GPIO_Port, M1_W_H_Pin); HAL_GPIO_WritePin(M1_W_H_GPIO_Port, M1_W_H_Pin, GPIO_PIN_RESET)

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
#define M2_U_PLUS_ON PinToAF(M2_U_L, &MOTOR2_PWM_TIM); PinToAF(M2_U_H_GPIO_Port, M2_U_H_Pin, &MOTOR2_PWM_TIM); HAL_TIM_PWM_Start(M2_U_H); HAL_TIMEx_PWMN_Start(M2_U_H)
#define M2_U_PLUS_OFF HAL_TIMEx_PWMN_Stop(M2_U_H); PinToPP(M2_U_L); HAL_TIM_PWM_Stop(M2_U_H); PinToPP(M2_U_H_GPIO_Port, M2_U_H_Pin); HAL_GPIO_WritePin(M2_U_H_GPIO_Port, M2_U_H_Pin, GPIO_PIN_RESET)

#define M2_V_CH TIM_CHANNEL_2
#define M2_V_H &MOTOR2_PWM_TIM, M2_V_CH
#define M2_V_PLUS_ON PinToAF(M2_V_L, &MOTOR2_PWM_TIM); PinToAF(M2_V_H_GPIO_Port, M2_V_H_Pin, &MOTOR2_PWM_TIM); HAL_TIM_PWM_Start(M2_V_H); HAL_TIMEx_PWMN_Start(M2_V_H);
#define M2_V_PLUS_OFF HAL_TIMEx_PWMN_Stop(M2_V_H); PinToPP(M2_V_L); HAL_TIM_PWM_Stop(M2_V_H); PinToPP(M2_V_H_GPIO_Port, M2_V_H_Pin); HAL_GPIO_WritePin(M2_V_H_GPIO_Port, M2_V_H_Pin, GPIO_PIN_RESET)

#define M2_W_CH TIM_CHANNEL_1
#define M2_W_H &MOTOR2_PWM_TIM, M2_W_CH
#define M2_W_PLUS_ON PinToAF(M2_W_L, &MOTOR2_PWM_TIM); PinToAF(M2_W_H_GPIO_Port, M2_W_H_Pin, &MOTOR2_PWM_TIM); HAL_TIM_PWM_Start(M2_W_H); HAL_TIMEx_PWMN_Start(M2_W_H)
#define M2_W_PLUS_OFF HAL_TIMEx_PWMN_Stop(M2_W_H); PinToPP(M2_W_L); HAL_TIM_PWM_Stop(M2_W_H); PinToPP(M2_W_H_GPIO_Port, M2_W_H_Pin); HAL_GPIO_WritePin(M2_W_H_GPIO_Port, M2_W_H_Pin, GPIO_PIN_RESET)

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

#define MOTOR1_ADC_SAMPLE_TIME ADC_SAMPLETIME_28CYCLES
#define MOTOR1_ADC hadc1
#define MOTOR1_ADC_INSTANCE MOTOR1_ADC.Instance
#define MOTOR1_ADC_DELAY_CHANNEL TIM_CHANNEL_4
#define M1_U_BEMF_CHANNEL ADC_CHANNEL_12
#define M1_V_BEMF_CHANNEL ADC_CHANNEL_11
#define M1_W_BEMF_CHANNEL ADC_CHANNEL_10

#define MOTOR2_ADC_SAMPLE_TIME ADC_SAMPLETIME_28CYCLES
#define MOTOR2_ADC hadc2
#define MOTOR2_ADC_INSTANCE MOTOR1_ADC.Instance
#define MOTOR2_ADC_DELAY_CHANNEL TIM_CHANNEL_4
#define M2_U_BEMF_CHANNEL ADC_CHANNEL_15
#define M2_V_BEMF_CHANNEL ADC_CHANNEL_14
#define M2_W_BEMF_CHANNEL ADC_CHANNEL_13

#define ADC_BEMF_DELAY_CYCLES_COUNT 30

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
uint8_t g_bADCReliable[2];
uint32_t g_iEnginePWMPulsesCount[2]; // number of pwm pulses since last commutation
uint32_t g_iEnginePWMPulsesTrigger[2];
uint8_t g_bBEMFCrossingPointDetected[2]; // was the zero-crossing point already detected during this commutation step or not

void PinToPP(GPIO_TypeDef *port, uint16_t pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(port, &GPIO_InitStruct);
}

void PinToAF(GPIO_TypeDef *port, uint16_t pin, TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);

	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	if (htim->Instance == MOTOR1_PWM_TIM_INSTANCE)
		GPIO_InitStruct.Alternate = MOTOR1_PWM_TIM_AF;
	else if (htim->Instance == MOTOR2_PWM_TIM_INSTANCE)
		GPIO_InitStruct.Alternate = MOTOR2_PWM_TIM_AF;
	HAL_GPIO_Init(port, &GPIO_InitStruct);
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
				g_iEnginePWMPulsesCount[motorx] = 0;
				g_iEnginePWMPulsesTrigger[motorx] = 0xFFFFFFFF;
				g_iCurrentCommutationStep[motorx] = 1;
				g_bBEMFCrossingPointDetected[motorx] = 0;
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
				g_iEnginePWMPulsesCount[motorx] = 0;
				g_iEnginePWMPulsesTrigger[motorx] = 0xFFFFFFFF;
				g_iCurrentCommutationStep[motorx] = 2;
				g_bBEMFCrossingPointDetected[motorx] = 0;
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
				g_iEnginePWMPulsesCount[motorx] = 0;
				g_iEnginePWMPulsesTrigger[motorx] = 0xFFFFFFFF;
				g_iCurrentCommutationStep[motorx] = 3;
				g_bBEMFCrossingPointDetected[motorx] = 0;
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
				g_iEnginePWMPulsesCount[motorx] = 0;
				g_iEnginePWMPulsesTrigger[motorx] = 0xFFFFFFFF;
				g_iCurrentCommutationStep[motorx] = 4;
				g_bBEMFCrossingPointDetected[motorx] = 0;
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
				g_iEnginePWMPulsesCount[motorx] = 0;
				g_iEnginePWMPulsesTrigger[motorx] = 0xFFFFFFFF;
				g_iCurrentCommutationStep[motorx] = 5;
				g_bBEMFCrossingPointDetected[motorx] = 0;
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
				g_iEnginePWMPulsesCount[motorx] = 0;
				g_iEnginePWMPulsesTrigger[motorx] = 0xFFFFFFFF;
				g_iCurrentCommutationStep[motorx] = 0;
				g_bBEMFCrossingPointDetected[motorx] = 0;
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
	if (bemfVoltage < powerVoltage / 4)
		return;
	if (powerVoltage <= 677)
		return;

	uint8_t motorx = hadc->Instance == MOTOR1_ADC_INSTANCE ? MOTOR1 : MOTOR2;
	g_bADCReliable[motorx] = 1;

	// 0, 2, 4 - upward bemf
	// 1, 3, 5 - downward bemf
	if (g_iCurrentCommutationStep[motorx] == 0 || g_iCurrentCommutationStep[motorx] == 2 || g_iCurrentCommutationStep[motorx] == 4)
	{
		if (bemfVoltage > 51 * powerVoltage / 100)
		{
			g_bNeedNewCommutation[motorx] = 1;
			g_bBEMFCrossingPointDetected[motorx] = 1;
			g_iEnginePWMPulsesTrigger[motorx] = 2 * g_iEnginePWMPulsesCount[motorx];
		}
	}
	else if (g_iCurrentCommutationStep[motorx] == 1 || g_iCurrentCommutationStep[motorx] == 3 || g_iCurrentCommutationStep[motorx] == 5)
	{
		if (bemfVoltage < 49 * powerVoltage / 100)
		{
			g_bNeedNewCommutation[motorx] = 1;
			g_bBEMFCrossingPointDetected[motorx] = 1;
			g_iEnginePWMPulsesTrigger[motorx] = 2 * g_iEnginePWMPulsesCount[motorx];
		}
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	uint8_t motorx = htim->Instance == MOTOR1_PWM_TIM_INSTANCE ? MOTOR1 : MOTOR2;
	g_iEnginePWMPulsesCount[motorx]++;
	if (g_iEnginePWMPulsesCount[motorx] >= g_iEnginePWMPulsesTrigger[motorx])
		DoCommutationNextStep(motorx);
	else if (!g_bBEMFCrossingPointDetected[motorx])
		HAL_ADC_Start_IT(htim->Instance == MOTOR1_PWM_TIM_INSTANCE ? &MOTOR1_ADC : &MOTOR2_ADC);
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
				// these two lines turns on the ADC
				//__HAL_TIM_SetCompare(motorx ? &MOTOR2_PWM_TIM : &MOTOR1_PWM_TIM, motorx ? MOTOR2_ADC_DELAY_CHANNEL : MOTOR1_ADC_DELAY_CHANNEL, ADC_BEMF_DELAY_CYCLES_COUNT);
				//HAL_TIM_OC_Start_IT(motorx ? &MOTOR2_PWM_TIM : &MOTOR1_PWM_TIM, motorx ? MOTOR2_ADC_DELAY_CHANNEL : MOTOR1_ADC_DELAY_CHANNEL);
			}
		}
		else if (g_iCurrentMode[motorx] == MOTOR_MODE_NORMAL && !g_bADCReliable[motorx])
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
	g_fEngineState[motorx] = 0.0f;
	g_bADCReliable[motorx] = 0;
	g_iEnginePWMPulsesTrigger[motorx] = 0xFFFFFFFF;
	g_iEnginePWMPulsesCount[motorx] = 0;
	g_bBEMFCrossingPointDetected[motorx] = 0;
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
	g_fEngineState[motorx] = 0.0f;
	g_bADCReliable[motorx] = 0;
	g_iEnginePWMPulsesTrigger[motorx] = 0xFFFFFFFF;
	g_iEnginePWMPulsesCount[motorx] = 0;
	g_bBEMFCrossingPointDetected[motorx] = 0;
}

void SetMotorCurrent(uint8_t motorx, uint8_t baccelerating)
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

	uint32_t pulse = PWM_START_PULSE;
	if (baccelerating)
		pulse = (uint32_t)(PWM_START_PULSE * (1.0f + 2.20f * (g_fEngineState[motorx] / 100.0f)));
	else if (g_fEngineState[motorx] > 96.0f)
		pulse = (uint32_t)(PWM_START_PULSE * (1.0f + 1.95f * (g_fEngineState[motorx] / 100.0f)));
	else if (g_fEngineState[motorx] > 92.0f)
		pulse = (uint32_t)(PWM_START_PULSE * (1.0f + 1.70f * (g_fEngineState[motorx] / 100.0f)));
	else if (g_fEngineState[motorx] > 86.0f)
		pulse = (uint32_t)(PWM_START_PULSE * (1.0f + 1.40f * (g_fEngineState[motorx] / 100.0f)));
	else if (g_fEngineState[motorx] > 65.0f)
		pulse = (uint32_t)(PWM_START_PULSE * (1.0f + 1.20f * (g_fEngineState[motorx] / 100.0f)));
	else if (g_fEngineState[motorx] > 55.0f)
		pulse = (uint32_t)(PWM_START_PULSE * (1.0f + 1.15f * (g_fEngineState[motorx] / 100.0f)));

	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_U_CH : M1_U_CH, pulse);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_V_CH : M1_V_CH, pulse);
	__HAL_TIM_SET_COMPARE(&htim, motorx ? M2_W_CH : M1_W_CH, pulse);
}

void ProcessMotor(uint8_t motorx)
{
	if (g_fEngineCommand[motorx] > 15.0f)
	{
		if (g_iCurrentMode[motorx] == MOTOR_MODE_STOP)
		{
			StartMotorSpin(motorx);
		}
		else if (g_iCurrentMode[motorx] == MOTOR_MODE_NORMAL)
		{
			if (g_fEngineCommand[motorx] >= g_fEngineState[motorx] - 0.1f && g_fEngineCommand[motorx] <= g_fEngineState[motorx] + 0.1f)
			{
				SetMotorCurrent(motorx, 0);
				g_fEngineState[motorx] = g_fEngineCommand[motorx];
			}
			g_iTotalCycleCount[motorx] = (int)(0.01f * (100.0f - g_fEngineState[motorx]) * (PWM_START_TO_NORMAL_TRANSITION_CYCLE_COUNT - ENGINE_MAX_CYCLE_COUNT) + ENGINE_MAX_CYCLE_COUNT);
			if (g_fEngineState[motorx] <= g_fEngineCommand[motorx] - 0.05f)
			{
				SetMotorCurrent(motorx, 1);
				g_fEngineState[motorx] += 0.00014f;
			}
			else if (g_fEngineState[motorx] >= g_fEngineCommand[motorx] + 0.05f)
			{
				SetMotorCurrent(motorx, 0);
				g_fEngineState[motorx] -= 0.00014f;
			}
		}
	}
	else if (g_fEngineCommand[motorx] < 10.0f)
	{
		if (g_iCurrentMode[motorx] == MOTOR_MODE_START || g_iCurrentMode[motorx] == MOTOR_MODE_NORMAL)
		{
			StopMotorPWM(motorx);
		}
	}
}

void MAX485Recieve()
{
	HAL_GPIO_WritePin(RE_GPIO_Port, RE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET);
}

void MAX485Transmit()
{
	HAL_GPIO_WritePin(RE_GPIO_Port, RE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_SET);
}

void ParseRxValues()
{
	g_iRxBufferPos = 255;

	// validation
	uint8_t spaceCount = 0;
	uint8_t dotCount = 0;
	uint8_t valid = 1;
	
	for (uint8_t i = 0; i < strlen((char*)g_rxdata); i++)
	{
		if (g_rxdata[i] == 0x20)
		{
			spaceCount++;
			continue;
		}
		
		if (g_rxdata[i] == 0x2E)
		{
			dotCount++;
			continue;
		}
		
		if (g_rxdata[i] > 0x54 || g_rxdata[i] < 0x30 || (g_rxdata[i] > 0x39 && g_rxdata[i] < 0x44 && g_rxdata[i] != 0x41) || (g_rxdata[i] > 0x45 && g_rxdata[i] < 0x52 && g_rxdata[i] != 0x4E))
		{
			valid = 0;
			break;
		}
		
		if (g_rxdata[i] == 0x2D && i > 0)
		{
			if (g_rxdata[i - 1] != 0x20)
			{
				valid = 0;
				break;
			}
		}
	}
	
	if (valid == 1 && (strstr((char*)g_rxdata, "START ") == NULL || strstr((char*)g_rxdata, " END") == NULL || spaceCount != 6 || dotCount != 4))
	{
		valid = 0;
	}
	
	if (valid == 0)
	{
		memset(g_rxdata, (uint8_t)0x00, sizeof(g_rxdata));
		g_iRxBufferPos = 0;
		g_bReadyToParse = 0;
		return;
	}
	
	sscanf((char*)g_rxdata, "START %f %f %f %f %i END", &g_fValveCommand[0], &g_fValveCommand[1], &g_fEngineCommand[0], &g_fEngineCommand[1], &g_bRelayCommand);
	
	memset(g_rxdata, (uint8_t)0x00, sizeof(g_rxdata));
	
	g_iRxBufferPos = 0;
	g_bReadyToParse = 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (g_iRxBufferPos == 255)
	{
		MAX485Recieve();
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

	MAX485Recieve();
	HAL_UART_Receive_IT(&RECIEVE_COMMAND_UART, g_rxbyte, sizeof(g_rxbyte));
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	MAX485Recieve();
}

void RespondToRequest()
{
	g_bNeedToRespond = 0;

	UpdateTemperatures(temperatureSensors, TEMPERATURE_SENSORS_COUNT);

	sprintf((char*)g_txdata, "START %f %f %f %f %f %f %f %f %d END", g_fValveCommand[0], g_fValveCommand[1], g_fEngineState[0], g_fEngineState[1], temperatureSensors[0].temperature, temperatureSensors[1].temperature, temperatureSensors[2].temperature, temperatureSensors[3].temperature, g_bRelayCommand);

	MAX485Transmit();
	HAL_UART_Transmit_IT(&SEND_RESPONSE_UART, g_txdata, strlen((char*)g_txdata));
}

int main(void)
{
	HAL_Init();

	SystemClock_Config();

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

	// Start sensors
	TemperatureInitSensors(temperatureSensors, TEMPERATURE_SENSORS_COUNT);

	// Start servos
	__HAL_TIM_SET_COMPARE(&SERVO1_PWM_TIM, SERVO1_PWM_CHANNEL, SERVO1_MIN_VALUE);
	HAL_TIM_PWM_Start(&SERVO1_PWM_TIM, SERVO1_PWM_CHANNEL);
	__HAL_TIM_SET_COMPARE(&SERVO2_PWM_TIM, SERVO2_PWM_CHANNEL, SERVO2_MIN_VALUE);
	HAL_TIM_PWM_Start(&SERVO2_PWM_TIM, SERVO2_PWM_CHANNEL);
	HAL_TIM_Base_Start_IT(&SERVO_SPEED_TIM);

	// Start recieving commands
	MAX485Recieve();
	HAL_UART_Receive_IT(&RECIEVE_COMMAND_UART, g_rxbyte, sizeof(g_rxbyte));

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); // led D3
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); // led D2

	StopMotorPWM(MOTOR1); // used as initializer, do not comment out
	StopMotorPWM(MOTOR2); // used as initializer, do not comment out

	HAL_Delay(3000);
	g_fEngineCommand[MOTOR1] = 0.0f;
	g_fEngineCommand[MOTOR2] = 0.0f;

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
	}
}

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 160;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_ADC1_Init(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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

	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_ADC2_Init(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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

	sConfig.Channel = ADC_CHANNEL_13;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_14;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_TIM1_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

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
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM2_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 79;
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

	HAL_TIM_MspPostInit(&htim2);

}

static void MX_TIM4_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 39;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 18;
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
}

static void MX_TIM5_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 39;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 18;
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
}

static void MX_TIM6_Init(void)
{
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 39;
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
}

static void MX_TIM8_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

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
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim8);
}

static void MX_USART1_UART_Init(void)
{
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
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
}

static void MX_USART2_UART_Init(void)
{
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
}

static void MX_USART3_UART_Init(void)
{
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
}

static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	HAL_GPIO_WritePin(GPIOD, LED_2_Pin|LED_3_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOB, RELAY_Pin|RE_Pin|DE_Pin, GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LED_2_Pin|LED_3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = RELAY_Pin|RE_Pin|DE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
	__disable_irq();
	while (1)
	{
	}
}
