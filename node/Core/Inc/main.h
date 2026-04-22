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
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lora.h"

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
#define PC0_LED_Pin GPIO_PIN_2
#define PC0_LED_GPIO_Port GPIOC
#define PC1_LED_Pin GPIO_PIN_3
#define PC1_LED_GPIO_Port GPIOC
#define Battery_Pin GPIO_PIN_0
#define Battery_GPIO_Port GPIOA
#define DBG_BTN1_Pin GPIO_PIN_6
#define DBG_BTN1_GPIO_Port GPIOA
#define DBG_BTN2_Pin GPIO_PIN_7
#define DBG_BTN2_GPIO_Port GPIOA
#define PC2_LED_Pin GPIO_PIN_4
#define PC2_LED_GPIO_Port GPIOC
#define LORA_MOSFET_Pin GPIO_PIN_0
#define LORA_MOSFET_GPIO_Port GPIOB
#define LORA_TGL_RELAY_Pin GPIO_PIN_8
#define LORA_TGL_RELAY_GPIO_Port GPIOC
#define TP_TX1_Pin GPIO_PIN_6
#define TP_TX1_GPIO_Port GPIOB
#define TP_RX1_Pin GPIO_PIN_7
#define TP_RX1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
void connected_test_all(void);
void go_to_sleep(void);
void get_timestamp(void);
void uart_set_rtc(void);
void set_time_and_date(RTC_TimeTypeDef *time, RTC_DateTypeDef *date);
void send_usb_ttl(uint8_t * message, uint8_t length, UART_HandleTypeDef * huart);
void connected_test_all(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
