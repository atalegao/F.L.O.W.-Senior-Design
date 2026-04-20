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
#include "lora.h"
#include "mesh.h"


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
#define LORA_TGL_Pin GPIO_PIN_1
#define LORA_TGL_GPIO_Port GPIOA
#define RA8875_CS_Pin GPIO_PIN_4
#define RA8875_CS_GPIO_Port GPIOA
#define LCD_RESET_Pin GPIO_PIN_0
#define LCD_RESET_GPIO_Port GPIOB
#define LCD_WAIT_Pin GPIO_PIN_1
#define LCD_WAIT_GPIO_Port GPIOB
#define LCD_INT_Pin GPIO_PIN_2
#define LCD_INT_GPIO_Port GPIOB
#define LCD_INT_EXTI_IRQn EXTI2_3_IRQn
#define WIFI_RST_Pin GPIO_PIN_8
#define WIFI_RST_GPIO_Port GPIOA
#define DBG_BTN1_Pin GPIO_PIN_15
#define DBG_BTN1_GPIO_Port GPIOA
#define DBG_BTN2_Pin GPIO_PIN_10
#define DBG_BTN2_GPIO_Port GPIOC
#define PB14_LED_Pin GPIO_PIN_11
#define PB14_LED_GPIO_Port GPIOC
#define PB13_LED_Pin GPIO_PIN_12
#define PB13_LED_GPIO_Port GPIOC
#define PB15_LED_Pin GPIO_PIN_2
#define PB15_LED_GPIO_Port GPIOD
#define PA15_LED_Pin GPIO_PIN_3
#define PA15_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define CS_DISABLE GPIO_PIN_SET //Pin set and reset for CS
#define CS_ENABLE GPIO_PIN_RESET
void connected_test_all(void);
void go_to_sleep(void);
void get_timestamp(void);
void uart_set_rtc(void);
void set_time_and_date(RTC_TimeTypeDef *time, RTC_DateTypeDef *date);
void send_usb_ttl(uint8_t * message, uint8_t length, UART_HandleTypeDef huart);
void connected_test_all(void);
void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback (UART_HandleTypeDef *huart);


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
