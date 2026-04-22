/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c  [RECEIVER NODE]
  * @brief          : Receives water level data from LoRa module over USART1,
  *                   checks against flood threshold, and triggers ESP8266
  *                   over USART2 to send a Telegram alert if exceeded.
  *
  * HARDWARE / UART ASSIGNMENTS:
  *   USART2  <->  LoRa module         (9600 baud) — receives WATER messages
  *   USART1  <->  ESP8266 WiFi module  (9600 baud) — sends FLOOD alert
  *   USART2  also used for debug printf via ST-Link virtual COM. DONT USE WHEN LORA CONNECTED
  *
  * LORA MESSAGE EXPECTED FORMAT (from sender node):
  *   "WATER:<node_id>:<distance_mm>\n"
  *   e.g. "WATER:1:142\n"
  *
  * MESSAGE SENT TO ESP8266 ON THRESHOLD BREACH:
  *   "FLOOD:<node_id>:<distance_mm>\n"
  *   e.g. "FLOOD:1:142\n"
  *
  * CUBEIDE IOC SETUP REQUIRED:
  *   - USART1: Asynchronous, 9600 baud
  *   - USART2: Asynchronous, 57600 baud, enable global interrupt in NVIC
  *   Both UARTs will be declared and initialised automatically by CubeMX.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Flood threshold in mm.
// Alert fires when water level reading is AT OR BELOW this value.
// Lower reading = water is closer to sensor = higher water level.
// Adjust once you know your sensor mounting height.
#define FLOOD_THRESHOLD_MM   150

// Cooldown between alerts — prevents spamming Telegram.
// Set to 10 minutes (in milliseconds).
#define ALERT_COOLDOWN_MS    (10 * 60 * 1000)

// UART receive buffer size
#define RX_BUF_SIZE          128

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// --- UART receive state ---
uint8_t          rxByte;
char             rxBuffer[RX_BUF_SIZE];
uint8_t          rxIndex = 0;
volatile uint8_t messageReady = 0;
char             parsedMessage[RX_BUF_SIZE];

// --- Alert cooldown state ---
uint32_t lastAlertTick      = 0;
uint8_t  alertCooldownActive = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void ESP_SendAlert(int nodeId, uint32_t distance);
void ProcessLoRaMessage(char* msg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Start interrupt-driven receive from LoRa module on USART2
  //HAL_UART_Receive_IT(&huart2, &rxByte, 1); REPLACE WITH LORA CODE IN HUB MAIN!

  // Safe to printf now — USART2 is initialised. DONT USE WITH LORA CONNECTED
  //printf("Receiver node booting...\r\n");

  HAL_Delay(3000); // give D1 Mini time to boot and connect to WiFi
  ESP_SendAlert(1, 120); // simulate node 1 reading 120mm (below 150mm threshold)

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Process any complete message received from LoRa
    if (messageReady) {
      messageReady = 0;
      ProcessLoRaMessage(parsedMessage);
    }

    // Check if alert cooldown period has expired
    if (alertCooldownActive) {
      if (HAL_GetTick() - lastAlertTick >= ALERT_COOLDOWN_MS) {
        alertCooldownActive = 0;
        printf("Alert cooldown expired, ready to alert again\r\n");
      }
    }
//    ESP_SendAlert(1, 120);
//        printf("Sent test alert\r\n");
//        HAL_Delay(3000);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  huart2.Init.BaudRate = 115200;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
//static void MX_GPIO_Init(void)
//{
//  /* USER CODE BEGIN MX_GPIO_Init_1 */
//
//  /* USER CODE END MX_GPIO_Init_1 */
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//
//  /* USER CODE BEGIN MX_GPIO_Init_2 */
//
//  /* USER CODE END MX_GPIO_Init_2 */
//}
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // PA2=USART2 TX (ST-Link), PA3=USART2 RX — debug printf
  GPIO_InitStruct.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PA9=USART1 TX (ESP8266), PA10=USART1 RX
  GPIO_InitStruct.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Alternate = GPIO_AF4_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

// -------------------------------------------------------
// UART RX interrupt callback
// Builds a complete '\n'-terminated line from LoRa module.
// Sets messageReady flag when a full line is received.
// -------------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    if (rxByte == '\n' || rxIndex >= RX_BUF_SIZE - 1)
    {
      rxBuffer[rxIndex] = '\0';

      // Trim trailing \r if present
      if (rxIndex > 0 && rxBuffer[rxIndex - 1] == '\r')
        rxBuffer[rxIndex - 1] = '\0';

      strcpy(parsedMessage, rxBuffer);
      messageReady = 1;
      rxIndex = 0;
    }
    else
    {
      rxBuffer[rxIndex++] = (char)rxByte;
    }

    // Re-arm interrupt for next byte
    HAL_UART_Receive_IT(&huart1, &rxByte, 1);
  }
}

// -------------------------------------------------------
// ProcessLoRaMessage()
// Parses incoming LoRa message and checks flood threshold.
//
// Expected format: "WATER:<nodeId>:<distance_mm>"
// e.g.             "WATER:1:142"
// -------------------------------------------------------
void ProcessLoRaMessage(char* msg)
{
  if (strncmp(msg, "WATER:", 6) == 0)
  {
    int      nodeId;
    uint32_t distance;

    if (sscanf(msg, "WATER:%d:%lu", &nodeId, &distance) == 2)
    {
      printf("Node %d: water level = %lu mm\r\n", nodeId, distance);

      // Lower distance = water is closer to sensor = higher water level
      if (distance <= FLOOD_THRESHOLD_MM && !alertCooldownActive)
      {
        printf("FLOOD THRESHOLD EXCEEDED — Node %d: %lu mm — sending alert\r\n",
               nodeId, distance);

        ESP_SendAlert(nodeId, distance);

        lastAlertTick       = HAL_GetTick();
        alertCooldownActive = 1;
      }
    }
  }
}

// -------------------------------------------------------
// ESP_SendAlert()
// Sends flood alert command to ESP8266 over USART1.
// ESP8266 parses this and fires the Telegram message.
//
// Format sent: "FLOOD:<nodeId>:<distance_mm>\n"
// e.g.         "FLOOD:1:142\n"
// -------------------------------------------------------
void ESP_SendAlert(int nodeId, uint32_t distance)
{
  char buf[64];
  snprintf(buf, sizeof(buf), "FLOOD:%d:%lu\n", nodeId, distance);
  HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
}

// -------------------------------------------------------
// __io_putchar()
// Routes printf to USART2 (ST-Link debug monitor). WILL NOT WORK WHEN CONNECTED TO LORA
// -------------------------------------------------------
//int __io_putchar(int ch)
//{
//  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 0xFFFF);
//  return ch;
//}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {}
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
