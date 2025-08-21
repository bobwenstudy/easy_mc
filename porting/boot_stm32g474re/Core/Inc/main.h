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
#include "stm32g4xx_hal.h"

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
#define M1_CURR_AMPL_V_Pin GPIO_PIN_0
#define M1_CURR_AMPL_V_GPIO_Port GPIOA
#define M1_CURR_BUS_Pin GPIO_PIN_1
#define M1_CURR_BUS_GPIO_Port GPIOA
#define M1_CURR_AMPL_U_Pin GPIO_PIN_2
#define M1_CURR_AMPL_U_GPIO_Port GPIOA
#define M1_CURR_AMPL_W_Pin GPIO_PIN_6
#define M1_CURR_AMPL_W_GPIO_Port GPIOA
#define M1_ENCODER_Z_Pin GPIO_PIN_0
#define M1_ENCODER_Z_GPIO_Port GPIOB
#define M1_ENCODER_Z_EXTI_IRQn EXTI0_IRQn
#define DEBUG_IO_0_Pin GPIO_PIN_1
#define DEBUG_IO_0_GPIO_Port GPIOB
#define DEBUG_IO_1_Pin GPIO_PIN_2
#define DEBUG_IO_1_GPIO_Port GPIOB
#define M1_BUS_VOLTAGE_Pin GPIO_PIN_12
#define M1_BUS_VOLTAGE_GPIO_Port GPIOB
#define M1_PWM_UL_Pin GPIO_PIN_13
#define M1_PWM_UL_GPIO_Port GPIOB
#define M1_PWM_VL_Pin GPIO_PIN_14
#define M1_PWM_VL_GPIO_Port GPIOB
#define M1_PWM_WL_Pin GPIO_PIN_15
#define M1_PWM_WL_GPIO_Port GPIOB
#define M1_PWM_UH_Pin GPIO_PIN_8
#define M1_PWM_UH_GPIO_Port GPIOA
#define M1_PWM_VH_Pin GPIO_PIN_9
#define M1_PWM_VH_GPIO_Port GPIOA
#define M1_PWM_WH_Pin GPIO_PIN_10
#define M1_PWM_WH_GPIO_Port GPIOA
#define M1_OCP_Pin GPIO_PIN_11
#define M1_OCP_GPIO_Port GPIOA
#define M1_ENCODER_A_Pin GPIO_PIN_4
#define M1_ENCODER_A_GPIO_Port GPIOB
#define M1_ENCODER_B_Pin GPIO_PIN_5
#define M1_ENCODER_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_uart4_tx;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
