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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern DMA_HandleTypeDef hdma_usart1_rx;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Track_L2_Pin GPIO_PIN_1
#define Track_L2_GPIO_Port GPIOA
#define SR04_Echo_Pin GPIO_PIN_4
#define SR04_Echo_GPIO_Port GPIOA
#define SR04_Trig_Pin GPIO_PIN_5
#define SR04_Trig_GPIO_Port GPIOA
#define Track_L1_Pin GPIO_PIN_0
#define Track_L1_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_10
#define AIN1_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_11
#define AIN2_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_12
#define BIN1_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_13
#define BIN2_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_11
#define OLED_SCL_GPIO_Port GPIOA
#define OLED_SDA_Pin GPIO_PIN_12
#define OLED_SDA_GPIO_Port GPIOA
#define Track_R1_Pin GPIO_PIN_5
#define Track_R1_GPIO_Port GPIOB
#define Track_R2_Pin GPIO_PIN_8
#define Track_R2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
