/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#define LED0_Pin GPIO_PIN_13
#define LED0_GPIO_Port GPIOC
#define NRF24_IRQ_Pin GPIO_PIN_14
#define NRF24_IRQ_GPIO_Port GPIOC
#define NRF24_CE_Pin GPIO_PIN_15
#define NRF24_CE_GPIO_Port GPIOC
#define BATTERY_Pin GPIO_PIN_0
#define BATTERY_GPIO_Port GPIOA
#define SW1_Pin GPIO_PIN_1
#define SW1_GPIO_Port GPIOA
#define PWM1_Pin GPIO_PIN_2
#define PWM1_GPIO_Port GPIOA
#define SUBS_CS_Pin GPIO_PIN_3
#define SUBS_CS_GPIO_Port GPIOA
#define SUBS_IRQ_Pin GPIO_PIN_4
#define SUBS_IRQ_GPIO_Port GPIOA
#define SUBS_IRQ_EXTI_IRQn EXTI4_IRQn
#define EEPROM_WR_CTRL_Pin GPIO_PIN_0
#define EEPROM_WR_CTRL_GPIO_Port GPIOB
#define IMU_IRQ_Pin GPIO_PIN_1
#define IMU_IRQ_GPIO_Port GPIOB
#define IMU_CS_Pin GPIO_PIN_2
#define IMU_CS_GPIO_Port GPIOB
#define MTR_1A_Pin GPIO_PIN_10
#define MTR_1A_GPIO_Port GPIOB
#define MTR_1B_Pin GPIO_PIN_12
#define MTR_1B_GPIO_Port GPIOB
#define MTR_2A_Pin GPIO_PIN_13
#define MTR_2A_GPIO_Port GPIOB
#define MTR_2B_Pin GPIO_PIN_14
#define MTR_2B_GPIO_Port GPIOB
#define SD_CS_Pin GPIO_PIN_15
#define SD_CS_GPIO_Port GPIOB
#define MTR_1_ENCA_Pin GPIO_PIN_8
#define MTR_1_ENCA_GPIO_Port GPIOA
#define MTR_1_ENCB_Pin GPIO_PIN_9
#define MTR_1_ENCB_GPIO_Port GPIOA
#define NRF24_CS_Pin GPIO_PIN_10
#define NRF24_CS_GPIO_Port GPIOA
#define MTR_2_ENCA_Pin GPIO_PIN_15
#define MTR_2_ENCA_GPIO_Port GPIOA
#define MTR_2_ENCB_Pin GPIO_PIN_3
#define MTR_2_ENCB_GPIO_Port GPIOB
#define MTR_1_PWM_Pin GPIO_PIN_4
#define MTR_1_PWM_GPIO_Port GPIOB
#define MTR_2_PWM_Pin GPIO_PIN_5
#define MTR_2_PWM_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
