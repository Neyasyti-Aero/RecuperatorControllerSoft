/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define M1_W_BEMF_Pin GPIO_PIN_0
#define M1_W_BEMF_GPIO_Port GPIOC
#define M1_V_BEMF_Pin GPIO_PIN_1
#define M1_V_BEMF_GPIO_Port GPIOC
#define M1_U_BEMF_Pin GPIO_PIN_2
#define M1_U_BEMF_GPIO_Port GPIOC
#define M2_W_BEMF_Pin GPIO_PIN_3
#define M2_W_BEMF_GPIO_Port GPIOC
#define P3_SERVO_1_Pin GPIO_PIN_1
#define P3_SERVO_1_GPIO_Port GPIOA
#define P4_SERVO_2_Pin GPIO_PIN_2
#define P4_SERVO_2_GPIO_Port GPIOA
#define M2_W_L_Pin GPIO_PIN_7
#define M2_W_L_GPIO_Port GPIOA
#define M2_V_BEMF_Pin GPIO_PIN_4
#define M2_V_BEMF_GPIO_Port GPIOC
#define M2_U_BEMF_Pin GPIO_PIN_5
#define M2_U_BEMF_GPIO_Port GPIOC
#define M1_W_L_Pin GPIO_PIN_8
#define M1_W_L_GPIO_Port GPIOE
#define M1_W_H_Pin GPIO_PIN_9
#define M1_W_H_GPIO_Port GPIOE
#define M1_V_L_Pin GPIO_PIN_10
#define M1_V_L_GPIO_Port GPIOE
#define M1_V_H_Pin GPIO_PIN_11
#define M1_V_H_GPIO_Port GPIOE
#define M1_U_L_Pin GPIO_PIN_12
#define M1_U_L_GPIO_Port GPIOE
#define M1_U_H_Pin GPIO_PIN_13
#define M1_U_H_GPIO_Port GPIOE
#define M2_V_L_Pin GPIO_PIN_14
#define M2_V_L_GPIO_Port GPIOB
#define M2_U_L_Pin GPIO_PIN_15
#define M2_U_L_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_12
#define LED_2_GPIO_Port GPIOD
#define LED_3_Pin GPIO_PIN_13
#define LED_3_GPIO_Port GPIOD
#define M2_W_H_Pin GPIO_PIN_6
#define M2_W_H_GPIO_Port GPIOC
#define M2_V_H_Pin GPIO_PIN_7
#define M2_V_H_GPIO_Port GPIOC
#define M2_U_H_Pin GPIO_PIN_8
#define M2_U_H_GPIO_Port GPIOC
#define RELAY_Pin GPIO_PIN_3
#define RELAY_GPIO_Port GPIOB
#define RE_Pin GPIO_PIN_5
#define RE_GPIO_Port GPIOB
#define DE_Pin GPIO_PIN_6
#define DE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
