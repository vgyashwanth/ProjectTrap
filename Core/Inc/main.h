/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32c0xx_hal.h"

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
#define GNSS_RX2_Pin GPIO_PIN_5
#define GNSS_RX2_GPIO_Port GPIOA
#define GNSS_RST_Pin GPIO_PIN_6
#define GNSS_RST_GPIO_Port GPIOA
#define GNSS_EN_Pin GPIO_PIN_7
#define GNSS_EN_GPIO_Port GPIOA
#define LTE_PWR_KEY_Pin GPIO_PIN_1
#define LTE_PWR_KEY_GPIO_Port GPIOB
#define GNSS_TX2_Pin GPIO_PIN_8
#define GNSS_TX2_GPIO_Port GPIOA
#define GNSS_VBCKP_Pin GPIO_PIN_6
#define GNSS_VBCKP_GPIO_Port GPIOC
#define LTE_RST_Pin GPIO_PIN_15
#define LTE_RST_GPIO_Port GPIOA
#define LTE_DCD_Pin GPIO_PIN_3
#define LTE_DCD_GPIO_Port GPIOB
#define LTE_RI_Pin GPIO_PIN_4
#define LTE_RI_GPIO_Port GPIOB
#define LTE_TX1_Pin GPIO_PIN_6
#define LTE_TX1_GPIO_Port GPIOB
#define LTE_TX2_Pin GPIO_PIN_7
#define LTE_TX2_GPIO_Port GPIOB
#define LTE_PWR_EN_Pin GPIO_PIN_8
#define LTE_PWR_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
