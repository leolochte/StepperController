/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STEP_Pin GPIO_PIN_14
#define STEP_GPIO_Port GPIOC
#define DIR_Pin GPIO_PIN_15
#define DIR_GPIO_Port GPIOC
#define RE_CLK_Pin GPIO_PIN_0
#define RE_CLK_GPIO_Port GPIOA
#define RE_DT_Pin GPIO_PIN_1
#define RE_DT_GPIO_Port GPIOA
#define RE_SWD_Pin GPIO_PIN_2
#define RE_SWD_GPIO_Port GPIOA
#define RE_SWD_EXTI_IRQn EXTI2_IRQn
#define SW7_C_Pin GPIO_PIN_3
#define SW7_C_GPIO_Port GPIOA
#define SW7_C_EXTI_IRQn EXTI3_IRQn
#define MICROSTEP_Pin GPIO_PIN_7
#define MICROSTEP_GPIO_Port GPIOA
#define RV1_Pin GPIO_PIN_0
#define RV1_GPIO_Port GPIOB
#define RV2_Pin GPIO_PIN_1
#define RV2_GPIO_Port GPIOB
#define D3_Pin GPIO_PIN_8
#define D3_GPIO_Port GPIOA
#define D4_Pin GPIO_PIN_11
#define D4_GPIO_Port GPIOA
#define D5_Pin GPIO_PIN_12
#define D5_GPIO_Port GPIOA
#define RST_Pin GPIO_PIN_15
#define RST_GPIO_Port GPIOA
#define SW7_U_Pin GPIO_PIN_4
#define SW7_U_GPIO_Port GPIOB
#define SW7_U_EXTI_IRQn EXTI4_IRQn
#define SW7_L_Pin GPIO_PIN_5
#define SW7_L_GPIO_Port GPIOB
#define SW7_L_EXTI_IRQn EXTI9_5_IRQn
#define SW7_D_Pin GPIO_PIN_6
#define SW7_D_GPIO_Port GPIOB
#define SW7_D_EXTI_IRQn EXTI9_5_IRQn
#define SW7_R_Pin GPIO_PIN_7
#define SW7_R_GPIO_Port GPIOB
#define SW7_R_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
