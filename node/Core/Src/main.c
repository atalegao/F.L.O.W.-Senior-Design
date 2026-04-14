/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <RH_RF95.h>
#include "mesh.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim21;

/* USER CODE BEGIN PV */
//usart2 is lora
//uart1 is USB
//lpuart1 is ultrasonic sensor
RTC_TimeTypeDef current_time;
RTC_DateTypeDef current_date;

uint8_t usb_buffer_rtc [7];
bool read_lora_fifo;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM21_Init(void);
static void MX_RNG_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool sendfifo_ready_norm = true;
int sendfifo_offset_norm = 0;
bool rx_ready = false;
uint8_t sendfifo_norm[FIFOSIZE_TX_NORM]; //array of data read from LoRa module
uint8_t receivefifo[FIFOSIZE_RX]; //array of data read from LoRa module
uint8_t sendfifo_send_message[FIFOSIZE_TX_SEND]; //array of data sending to LoRa module for an actual message send
uint8_t sendfifo_rec_message[FIFOSIZE_TX_REC]; //array of data sending to LoRa module for reading FIFO buffer
extern uint8_t * rx_data;
int sendfifo_offset_send = 0;
bool sendfifo_ready_send = true;
int sendfifo_offset_rec = 0;
bool sendfifo_ready_rec = true;
uint8_t rec_data [MESSAGE_LENGTH];
uint8_t self_addr [ADDR_LENGTH];

bool send_normal = false;
bool send_send = false;
bool send_rec = false; //great names I know

bool do_send = false;

uint8_t global_receive_mode_from_cad;
//1 means the lora timer is currently for receive mode timeout
//0 means the lora timer is currently for cad cycle

uint8_t receivefifo_usb_ttl [0];

uint8_t addr_any_direction [ADDR_LENGTH];
uint8_t addr_right_direction [ADDR_LENGTH];

bool isHub = false;

#define DO_SEND 1

#define DO_REC 0

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  HAL_Delay(2000);
  HAL_NVIC_DisableIRQ (SysTick_IRQn);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(SysTick_IRQn);//while this is added here, it needs to be at the bottom of the main function right before the loop

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_TIM21_Init();
  MX_RNG_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_DisableIRQ(TIM21_IRQn); //disable tim21, used for CAD cycle so it does not go off before init is done
  HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn); //disable tim6, used for lora send so it does not go off before init is done
  read_lora_fifo = false;
  receivefifo[0] = 0; //added
  HAL_GPIO_WritePin (LORA_TGL_RELAY_GPIO_Port, LORA_TGL_RELAY_Pin, GPIO_PIN_RESET);//Relay
  HAL_GPIO_WritePin(LORA_MOSFET_GPIO_Port, LORA_MOSFET_Pin, GPIO_PIN_RESET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin (LORA_TGL_RELAY_GPIO_Port, LORA_TGL_RELAY_Pin, GPIO_PIN_SET);//Relay
  HAL_GPIO_WritePin(LORA_MOSFET_GPIO_Port, LORA_MOSFET_Pin, GPIO_PIN_SET);
  HAL_Delay(1000);
  HAL_UART_Receive_IT(&hlpuart1, &rx_data, 1); //ultrasonic sensor data receive on lpuart1

  HAL_UART_Receive_DMA(&huart2, receivefifo, 1); //lora
  HAL_UART_Receive_DMA(&huart1, receivefifo_usb_ttl, 1); //usb-ttl
  HAL_Delay(1000);//added
  connected_test_all();
  ultrasonic_init(); 
  lora_init(hdma_usart2_tx, huart2);
  HAL_Delay(1000);
  //setup_lora_send_timer(&htim6); //set up lora send data timer
  HAL_NVIC_SetPriority(TIM21_IRQn, 2, 0); //start TIM21 since it was stopped before
  HAL_NVIC_EnableIRQ(TIM21_IRQn);
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

  //send_data[0] = 0xF0;
  //send_data[1] = 0x0F;
  HAL_TIM_Base_Start_IT(&htim21);
  //HAL_TIM_Base_Start_IT(&htim6);

  HAL_NVIC_DisableIRQ (SysTick_IRQn);//this has to be added here, else HAL_Delay will not work in TIM21 IRQ
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(SysTick_IRQn);

  mesh_init();
  self_addr[0] = 0x16;
  self_addr[1] = 0x17;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(DO_REC){
		  if(read_lora_fifo){ //
			  // start restart the CAD timer so it doesn't take the entire receive-timout time
			  HAL_TIM_Base_Stop_IT(&htim21);
			  change_lora_timer_period(0, &htim21); //0 means sleep time,
			  HAL_TIM_Base_Start_IT(&htim21);
			  //end restart the CAD timer so it doesn't take the entire receive-timout time
			  //lora_read_fifo_all(rec_data, 0x2, hdma_usart2_tx, huart2); //second input is length

			  //added below line (and commented out above line) to handle received message
			  mesh_main_rec(hdma_usart2_tx, huart2);

			  read_lora_fifo = false;
			  //
			  HAL_GPIO_WritePin(GPIOC, PC0_LED_Pin|PC1_LED_Pin|PC2_LED_Pin, GPIO_PIN_SET);
			  HAL_Delay(1000);
			  HAL_GPIO_WritePin(GPIOC, PC0_LED_Pin|PC1_LED_Pin|PC2_LED_Pin, GPIO_PIN_RESET);
		  }
	  }
	  if(DO_SEND){
		  HAL_NVIC_DisableIRQ(TIM21_IRQn); //disable tim21, used for CAD cycle so it does not go off before init is done
		  HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		  uint8_t message_id [4];
		  message_id[0] = 0x12;
		  message_id[1] = 0x13;
		  message_id[2] = 0x14;
		  message_id[3] = 0x15;
		  uint8_t dest_addr [ADDR_LENGTH];
		  dest_addr[0] = 0x16;
		  dest_addr[1] = 0x17;
		  uint8_t water_height [WATER_LENGTH];
		  water_height[0] = 0x18;
		  uint8_t battery_status [BATTERY_LENGTH];
		  battery_status[0] = 0x19;
		  uint8_t node_addr [ADDR_LENGTH];
		  node_addr[0] = 0x20;
		  node_addr[1] = 0x21;
		  uint8_t time [6];
		  time[0] = 0x22;
		  time[1] = 0x23;
		  time[2] = 0x24;
		  time[3] = 0x25;
		  time[4] = 0x26;
		  time[5] = 0x27;

		  self_addr[0] = 0x01;
		  self_addr[1] = 0xFF;
		  mesh_send_data(message_id, dest_addr, water_height, battery_status, node_addr, time, hdma_usart2_tx, huart2);
		  HAL_Delay(100);
		  HAL_GPIO_WritePin(GPIOC, PC0_LED_Pin|PC1_LED_Pin|PC2_LED_Pin, GPIO_PIN_SET);
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(GPIOC, PC0_LED_Pin|PC1_LED_Pin|PC2_LED_Pin, GPIO_PIN_RESET);
	  }
//	  if(read_lora_fifo){ //
//		  // start restart the CAD timer so it doesn't take the entire receive-timout time
//		  HAL_TIM_Base_Stop_IT(&htim21);
//		  change_lora_timer_period(0, &htim21); //0 means sleep time,
//		  HAL_TIM_Base_Start_IT(&htim21);
//		  //end restart the CAD timer so it doesn't take the entire receive-timout time
//		  lora_read_fifo_all(rec_data, 0x2, hdma_usart2_tx, huart2); //second input is length
//		  read_lora_fifo = false;
//		  //
//		  HAL_GPIO_WritePin(GPIOC, PC0_LED_Pin|PC1_LED_Pin|PC2_LED_Pin, GPIO_PIN_SET);
//		  HAL_Delay(1000);
//		  HAL_GPIO_WritePin(GPIOC, PC0_LED_Pin|PC1_LED_Pin|PC2_LED_Pin, GPIO_PIN_RESET);
//	  }
//	  //send message
//	  if(do_send & (!global_receive_mode_from_cad) & (!read_lora_fifo)){
//		 uint8_t data [2];
//		 data[0] = 0xF0;
//		 data[1] = 0x0F;
//		 lora_send(data, 2, hdma_usart2_tx, huart2);
//		 do_send = false;//end send
//		 HAL_GPIO_WritePin(GPIOC, PC2_LED_Pin, GPIO_PIN_SET);
//		 HAL_Delay(1000);
//		 HAL_GPIO_WritePin(GPIOC, PC2_LED_Pin, GPIO_PIN_RESET);
//		 uint32_t delay = random_number_gen();//random delay
//		 HAL_Delay(delay & 0xFF);
//	  }
	//go_to_sleep();//this should be the final line in the loop
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_RTC
                              |RCC_PERIPHCLK_USB;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00B07CB4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 209700;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_7B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM21 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM21_Init(void)
{

  /* USER CODE BEGIN TIM21_Init 0 */

  /* USER CODE END TIM21_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM21_Init 1 */

  /* USER CODE END TIM21_Init 1 */
  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 6399;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 9999;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM21_Init 2 */

  /* USER CODE END TIM21_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PC0_LED_Pin|PC1_LED_Pin|PC2_LED_Pin|LORA_TGL_RELAY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_MOSFET_GPIO_Port, LORA_MOSFET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TP_TX1_Pin|TP_RX1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC0
                           PC1 PC5 PC6 PC7
                           PC9 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_9|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PH1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0_LED_Pin PC1_LED_Pin PC2_LED_Pin LORA_TGL_RELAY_Pin */
  GPIO_InitStruct.Pin = PC0_LED_Pin|PC1_LED_Pin|PC2_LED_Pin|LORA_TGL_RELAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Battery_Pin PA1 PA4 PA5
                           PA8 PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = Battery_Pin|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DBG_BTN1_Pin DBG_BTN2_Pin */
  GPIO_InitStruct.Pin = DBG_BTN1_Pin|DBG_BTN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA_MOSFET_Pin TP_TX1_Pin TP_RX1_Pin */
  GPIO_InitStruct.Pin = LORA_MOSFET_Pin|TP_TX1_Pin|TP_RX1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB12 PB13
                           PB14 PB15 PB3 PB4
                           PB5 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void go_to_sleep(void){
	HAL_SuspendTick();
	HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	HAL_ResumeTick();
}

void get_timestamp(void){
	HAL_RTC_GetTime(&hrtc, &current_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &current_date, RTC_FORMAT_BIN);
}

void uart_set_rtc(void){
	//this should be called when the user indicates they are about to send rtc time and date data

	//use one buffer with size 7
	while (HAL_UART_Receive(&huart1, usb_buffer_rtc, 7, 120000) != HAL_OK){ //last is timeout in ms, 60000 is 1 minute
		//do nothing
	}

	RTC_TimeTypeDef set_time;
	RTC_DateTypeDef set_date;
	set_time.Hours = usb_buffer_rtc[0];// 24 hour time
	set_time.Minutes = usb_buffer_rtc[1];
	set_time.Seconds = usb_buffer_rtc[2];

	set_date.WeekDay = usb_buffer_rtc[3]; //monday = 1, tuesday = 2,...
	set_date.Month = usb_buffer_rtc[4]; //1 = January, 2 = February, ...
	set_date.Date = usb_buffer_rtc[5];//day
	set_date.Year = usb_buffer_rtc[6];//just 26 not 2006

}

void set_time_and_date(RTC_TimeTypeDef *time, RTC_DateTypeDef *date){
	if(HAL_RTC_SetTime(&hrtc, time, RTC_FORMAT_BIN) != HAL_OK){
		//error
	}
	if(HAL_RTC_SetDate(&hrtc, date, RTC_FORMAT_BIN) != HAL_OK){
		//error
	}
	HAL_UART_Receive_DMA(&huart1, receivefifo_usb_ttl, 1); //usb-ttl
	//turns on DMA for receive again since non-dma was used before
}

void send_usb_ttl(uint8_t * message, uint8_t length, UART_HandleTypeDef huart){
	HAL_UART_Transmit_DMA(&huart, message, length);
}



void connected_test_all(void){
	HAL_GPIO_WritePin(GPIOC, PC0_LED_Pin|PC1_LED_Pin|PC2_LED_Pin, GPIO_PIN_SET);//turn on all LEDs

	connected_test(hdma_usart2_tx, huart2);//check LoRa

	//check Ultrasonic
	//lpuart1

	//send something to USB-TTL (don't require a response because it might not be connected)
	uint8_t message [4];
	message[0] = 't';
	message[1] = 'e';
	message[2] = 's';
	message[3] = 't';
	send_usb_ttl(message, 4, huart1);

	//wait some time
	HAL_Delay(1000);

	HAL_GPIO_WritePin(GPIOC, PC0_LED_Pin|PC1_LED_Pin|PC2_LED_Pin, GPIO_PIN_RESET);//turn off all LEDs

}

void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart){
	if(huart->Instance == USART2){ //lora
		//normal code
		//receivefifo[0] = 0;
		rx_ready = true;
		//then call receive again
		HAL_UART_Receive_DMA(&huart2, receivefifo, 1);
		if(global_receive_mode_from_cad == 1){
			global_receive_mode_from_cad = 0;
			//this is for when CAD mode detected a preamble
			//this should not happen, figure out what to do here later
			if(receivefifo[0] == 0x49){
				//this is when the response is I (0x49)
				//this means this is actually the I response for a DIO interrupt
				//call the read FIFO and figure out response function
				//lora_read_fifo_all(rec_data, 0x2, hdma_usart1_tx, huart[0]); //second input is length
				//HAL_NVIC_SetPendingIRQ(1);//above line is called in this interrupt to deal with interrupt priorities
				int value;
				value = uart_read(); //dummy read to get rid of the I response
				read_lora_fifo = true;
			}
		}
	}
	//ADDED FOR ULTRASONIC
	else if(huart->Instance == hlpuart1.Instance){ // ultrasonic on LPUART1
		ultrasonic_process_rx(rx_data);
		HAL_UART_Receive_IT(&hlpuart1, &rx_data, 1);
	}
	else if(huart->Instance == USART1){ //usb-ttl
		if(receivefifo_usb_ttl[0] == 0xFF){ //special character to indicate setting RTC
			//set flag to update rtc
			//do not activate DMA
			//activate it when done with rtc stuff
		}
		else{//ignore
			HAL_UART_Receive_DMA(&huart1, receivefifo_usb_ttl, 1); //usb-ttl
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2){//lora
		if(send_normal){
			for (int i = 0;  i < FIFOSIZE_TX_NORM; i++){
				sendfifo_norm[i] = 0;
				}
			sendfifo_offset_norm = 0;
			sendfifo_ready_norm = true;
			send_normal = false;
		}
		else if(send_send){
			for (int i = 0;  i < FIFOSIZE_TX_SEND; i++){
				sendfifo_send_message[i] = 0;
			}
			sendfifo_offset_send = 0;
			sendfifo_ready_send = true;
			send_send = false;
		}
		else if(send_rec){
			for (int i = 0;  i < FIFOSIZE_TX_REC; i++){
				sendfifo_rec_message[i] = 0;
			}
			sendfifo_offset_rec = 0;
			sendfifo_ready_rec = true;
			send_rec = false;
		}
		else{
			//should not happen, error
			while (true){
				//infinite loop to know something is wrong
			}
		}
	}
	else if(huart->Instance == USART1){//usb-ttl
		//do nothing
	}

}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
