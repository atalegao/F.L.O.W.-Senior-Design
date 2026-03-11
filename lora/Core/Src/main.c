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
TIM_HandleTypeDef htim21;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
bool read_lora_fifo;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM21_Init(void);
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
int sendfifo_offset_send = 0;
bool sendfifo_ready_send = true;
int sendfifo_offset_rec = 0;
bool sendfifo_ready_rec = true;

bool send_normal;
bool send_send;
bool send_rec; //great names I know

uint8_t global_receive_mode_from_cad;
//1 means the lora timer is currently for receive mode timeout
//0 means the lora timer is currently for cad cycle



#ifdef CAD_BOL
  uint8_t rxdone = 0;
  uint8_t valid_header = 0;
  uint8_t crc_error = 0;
  bool clear = true; //this clears irq registers (need to for this, else LEDs would never reset)
  uint8_t rec_data [MESSAGE_LENGTH];
  bool rec_good = false;
  uint8_t send_data [MESSAGE_LENGTH];
  bool send_good = false;
#endif
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
  //change systick priority
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
  MX_USART1_UART_Init();
  MX_TIM21_Init();
  /* USER CODE BEGIN 2 */
  read_lora_fifo = false;
  HAL_GPIO_WritePin (GPIOC, 8, GPIO_PIN_SET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin (GPIOC, 8, GPIO_PIN_RESET);
  HAL_Delay(1000);
  HAL_UART_Receive_DMA(&huart1, receivefifo, 1);
  connected_test(hdma_usart1_tx, huart1);
  lora_init();
  //tx = C12, rx = D2
  //rxdone LED = C0, valid_header LED = C1, crc_error LED = C2 USE RESISTORS: 150 ohm
  //C3-10 are 8 bits for data
  //if(receive_bol){ //receiving
#ifdef CAD_BOL
  send_data[0] = 0xF0;
  send_data[1] = 0x0F;
  HAL_TIM_Base_Start_IT(&htim21);
#endif

#ifdef RECEIVE_BOL
	  uint8_t rxdone = 0;
	  uint8_t valid_header = 0;
	  uint8_t crc_error = 0;
	  bool clear = true; //this clears irq registers (need to for this, else LEDs would never reset)
	  uint8_t data [MESSAGE_LENGTH];
	  bool good = false;
  //}
#endif
  //end receiving
  //else{//sending
#ifdef SEND_BOL
	  uint8_t data [MESSAGE_LENGTH];
	  data[0] = 0xF0;
	  data[1] = 0x0F;
	  bool good = false;
#endif
  //}
	  HAL_NVIC_DisableIRQ (SysTick_IRQn);//this has to be added here, else HAL_Delay will not work in TIM21 IRQ
	  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(SysTick_IRQn);

//	  //this is for special fake interrupt for LoRa (Rx callback was calling a function that needed Tx callback, but they had same priority)
//	  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 3, 0);
//	  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(read_lora_fifo){ //
		  lora_read_fifo_all(rec_data, 0x2, hdma_usart1_tx, huart1); //second input is length
		  read_lora_fifo = false;
	  }
#ifdef RECEIVE_BOL
		  set_mode_continuous_receive();
		  HAL_Delay(1000);
		  //every 1 second, check irq flags (and clear)
		  good = check_irq_flags_receive(&rxdone,&valid_header, &crc_error, clear);
		  //set LEDs based on flags
		  if(good){
		  	//GPIOC->ODR = rxdone | (valid_header << 1) | (crc_error << 2);
		  	lora_read_fifo_all(data, 2); //get message from FIFO
		  }
		  else{
		  	//GPIOC->ODR = 0;
		  }
		  set_mode_sleep(); //this clears FIFO
		  set_mode_continuous_receive(); //this goes back to receive mode
#endif
	  //end receiving
#ifdef SEND_BOL
		  good = lora_send(data, 2, hdma_usart1_tx, huart1);
		  if(good){
			  //GPIOC->ODR = 1 | (1 << 1) | (0 << 2) | (data[0] << 3);
		  }
		  else{
		      //GPIOC->ODR = 0 | (0 << 1) | (1 << 2) | (data[0] << 3);
		   }
		  HAL_Delay(1000);
		  //GPIOC->ODR = 0;
		  HAL_Delay(1000);
		  lora_write_single(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags (can try adding this in send module as well)
		  //like reading register until get send done and then clear it
		  if(data[0] == 0xFF){
			  data[1] += 0x1;
		  }
		  data[0] += 0x1;
		  set_mode_sleep(hdma_usart1_tx, huart1); //this clears FIFO
#endif

#ifdef CAD_BOL
  //do nothing, wait for timer
#endif
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LoRa_power_GPIO_Port, LoRa_power_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LoRa_power_Pin */
  GPIO_InitStruct.Pin = LoRa_power_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LoRa_power_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart){
	//normal code
	//receivefifo[0] = 0;
	rx_ready = true;
	//then call receive again
	HAL_UART_Receive_DMA(&huart1, receivefifo, 1);
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

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(send_normal){
		for (int i = 0;  i < FIFOSIZE_TX_NORM; i++){
		        sendfifo[i] = 0;
		    }
		    sendfifo_offset = 0;
		    sendfifo_ready = true;
	}
	else if(send_send){
		for (int i = 0;  i < FIFOSIZE_TX_SEND; i++){
			sendfifo_send_message[i] = 0;
		}
		sendfifo_offset_send = 0;
		sendfifo_ready_send = true;
	}
	else if(send_rec){
		for (int i = 0;  i < FIFOSIZE_TX_REC; i++){
			sendfifo_rec_message[i] = 0;
		}
		sendfifo_offset_rec = 0;
		sendfifo_ready_rec = true;
	}
	else{
		//should not happen, error
		while (true){
			//infinite loop to know something is wrong
		}
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
