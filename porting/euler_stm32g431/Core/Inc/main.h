/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define Button3_Pin GPIO_PIN_13
#define Button3_GPIO_Port GPIOC
#define Button3_EXTI_IRQn EXTI15_10_IRQn
#define Button1_Pin GPIO_PIN_10
#define Button1_GPIO_Port GPIOC
#define Button1_EXTI_IRQn EXTI15_10_IRQn
#define Button2_Pin GPIO_PIN_11
#define Button2_GPIO_Port GPIOC
#define Button2_EXTI_IRQn EXTI15_10_IRQn
#define DEBUG_IO_0_Pin GPIO_PIN_5
#define DEBUG_IO_0_GPIO_Port GPIOB
#define M1_ENCODER_A_Pin GPIO_PIN_6
#define M1_ENCODER_A_GPIO_Port GPIOB
#define M1_ENCODER_B_Pin GPIO_PIN_7
#define M1_ENCODER_B_GPIO_Port GPIOB
#define M1_ENCODER_Z_Pin GPIO_PIN_8
#define M1_ENCODER_Z_GPIO_Port GPIOB
#define M1_ENCODER_Z_EXTI_IRQn EXTI9_5_IRQn
#define DEBUG_IO_1_Pin GPIO_PIN_9
#define DEBUG_IO_1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern COMP_HandleTypeDef hcomp1;

extern CORDIC_HandleTypeDef hcordic;

extern DAC_HandleTypeDef hdac1;
extern DAC_HandleTypeDef hdac3;

extern FDCAN_HandleTypeDef hfdcan1;

extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;
extern OPAMP_HandleTypeDef hopamp3;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
