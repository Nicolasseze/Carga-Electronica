/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TRIGGER_Pin GPIO_PIN_13
#define TRIGGER_GPIO_Port GPIOC
#define CONX_Pin GPIO_PIN_14
#define CONX_GPIO_Port GPIOC
#define ERROR2_Pin GPIO_PIN_15
#define ERROR2_GPIO_Port GPIOC
#define ETH_RST_Pin GPIO_PIN_3
#define ETH_RST_GPIO_Port GPIOA
#define ETH_CS_Pin GPIO_PIN_4
#define ETH_CS_GPIO_Port GPIOA
#define ETH_SCK_Pin GPIO_PIN_5
#define ETH_SCK_GPIO_Port GPIOA
#define ETH_MISO_Pin GPIO_PIN_6
#define ETH_MISO_GPIO_Port GPIOA
#define ETH_MOSI_Pin GPIO_PIN_7
#define ETH_MOSI_GPIO_Port GPIOA
#define LM35_Pin GPIO_PIN_0
#define LM35_GPIO_Port GPIOB
#define ERROR1_Pin GPIO_PIN_1
#define ERROR1_GPIO_Port GPIOB
#define UX_SCK_Pin GPIO_PIN_13
#define UX_SCK_GPIO_Port GPIOB
#define UX_MISO_Pin GPIO_PIN_14
#define UX_MISO_GPIO_Port GPIOB
#define UX_MOSI_Pin GPIO_PIN_15
#define UX_MOSI_GPIO_Port GPIOB
#define I_SCALE_Pin GPIO_PIN_9
#define I_SCALE_GPIO_Port GPIOA
#define USER_KEY_Pin GPIO_PIN_10
#define USER_KEY_GPIO_Port GPIOA
#define USER_LED_Pin GPIO_PIN_11
#define USER_LED_GPIO_Port GPIOA
#define V_SCALE_Pin GPIO_PIN_12
#define V_SCALE_GPIO_Port GPIOA
#define I2C_SCL_Pin GPIO_PIN_6
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_7
#define I2C_SDA_GPIO_Port GPIOB
#define ADC_RDY_Pin GPIO_PIN_8
#define ADC_RDY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
//Definicion Hardware
#define ADC_HW	ADS1115
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
