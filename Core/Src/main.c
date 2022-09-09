/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  UART_RX_0 = 48,
  UART_RX_1,
  UART_RX_2,
  UART_RX_NONE = 0xFF,
}UARTCommandTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RXBUFFERSIZE                     1

#define BLEUIO_NOT_READY_MSG	"\r\nBleuIO not ready yet.\r\n"

// Dongle commands
#define DONGLE_CMD_ATI "ATI\r\n"
#define DONGLE_CMD_AT_ADVSTART "AT+ADVSTART\r\n"
#define DONGLE_CMD_AT_ADVSTOP "AT+ADVSTOP\r\n"
#define DONGLE_SEND_LED_ON "AT+SPSSEND=LED ON\r\n"
#define DONGLE_SEND_LED_OFF "AT+SPSSEND=LED OFF\r\n"
#define DONGLE_SEND_LED_RED "AT+SPSSEND=LED RED\r\n"
#define DONGLE_SEND_LED_GREEN "AT+SPSSEND=LED GREEN\r\n"
#define DONGLE_SEND_LED_BLUE "AT+SPSSEND=LED BLUE\r\n"
#define DONGLE_SEND_LED_RAINBOW "AT+SPSSEND=LED RAINBOW\r\n"

#define MAX_LED 8
#define USE_BRIGHTNESS 1



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x30040200
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];  // for brightness

uint16_t pwmData[(24*MAX_LED)+50];

volatile int datasentflag;

static char uart_tx_buf[250];
int uart_buf_len;
bool isBleuIOReady;
uint8_t dongle_response[244];
static UARTCommandTypeDef uartStatus;
static uint8_t aRxBuffer[RXBUFFERSIZE];
static int RX_value;

bool isAdvertising;
bool isLEDOn;
bool isConnected;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
void handleUartInput(UARTCommandTypeDef cmd);
void dongle_interpreter(uint8_t * input);
void Set_LED (int LEDnum, int Red, int Green, int Blue);
void Set_Brightness (int brightness);
void WS2812_Send (void);
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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */
  isBleuIOReady = false;
  isAdvertising= false;
  isLEDOn= false;
  isConnected= false;
  uartStatus = UART_RX_NONE;

  // Starts uart recieve interrupt mode
  HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, RXBUFFERSIZE);

  // Turns on all LEDs on start up
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // Green
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET); // Yellow
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // Red
  HAL_Delay(1000);

  // Turns off all LEDs except Red
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);

  // Sends welcome message to uart
  uart_buf_len = sprintf(uart_tx_buf, "\r\nWelcome to STM32 BleuIO RGB LED Example!\r\nPress 0 to run the ATI command\r\nPress 1 to manually turn on LED\r\nPress 2 to manually turn off LED\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, HAL_MAX_DELAY);

  // Set LED Strip on and eight different colors (Rainbow Mode)
  Set_LED(0, 255, 0, 0);
  Set_LED(1, 0, 255, 0);
  Set_LED(2, 0, 0, 255);

  Set_LED(3, 46, 89, 128);

  Set_LED(4, 156, 233, 100);
  Set_LED(5, 102, 0, 235);
  Set_LED(6, 47, 38, 77);

  Set_LED(7, 255, 200, 0);
  Set_Brightness(40);
  WS2812_Send();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    // Simple handler for uart input
    handleUartInput(uartStatus);
    // Inteprets the dongle data
    dongle_interpreter(dongle_response);

    // Starts advertising as soon as the Dongle is ready.
    if(!isAdvertising && !isConnected && isBleuIOReady)
    {
    	HAL_Delay(200);
		Set_Brightness(0);
		WS2812_Send();
    	writeToDongle((uint8_t*)DONGLE_CMD_AT_ADVSTART);
    	isAdvertising = true;
    }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 64-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_STRIP_GPIO_Port, LED_STRIP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_OTG_FS_PWR_EN_GPIO_Port, USB_OTG_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_STRIP_Pin */
  GPIO_InitStruct.Pin = LED_STRIP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_STRIP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_OTG_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTG_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle == &huart3)
	{
		RX_value = (int)aRxBuffer[0];
		uartStatus = UART_RX_NONE;

		switch(RX_value)
		{
			case UART_RX_0:
			{
				uartStatus = UART_RX_0;
				break;
			}
			case UART_RX_1:
			{
				uartStatus = UART_RX_1;
				break;
			}
			case UART_RX_2:
			{
				uartStatus = UART_RX_2;
				break;
			}
			default:
			{
				uartStatus = UART_RX_NONE;
				break;
			}
		}
		// Resets uart recieve interrupt mode
		HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, RXBUFFERSIZE);
	}
}


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
	datasentflag=1;
}

/**
  * @brief Simple uart input handler
  * @retval None
  */
void handleUartInput(UARTCommandTypeDef cmd)
{
	switch(cmd)
	{
		case UART_RX_0:
		{
			// 0
			uart_buf_len = sprintf(uart_tx_buf, "\r\n(0 pressed)\r\n");
			HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, HAL_MAX_DELAY);
			if(isBleuIOReady)
			{
				writeToDongle((uint8_t*)DONGLE_CMD_ATI);
			} else
			{
				uart_buf_len = sprintf(uart_tx_buf, BLEUIO_NOT_READY_MSG);
				HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, HAL_MAX_DELAY);
			}
			uartStatus = UART_RX_NONE;
			break;
		}
		case UART_RX_1:
		{
			// 1
			uart_buf_len = sprintf(uart_tx_buf, "\r\n(1 pressed LED on!)\r\n");
			HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, HAL_MAX_DELAY);
			  Set_Brightness(40);
			  WS2812_Send();
			uartStatus = UART_RX_NONE;
			break;
		}
		case UART_RX_2:
		{
			// 2
			uart_buf_len = sprintf(uart_tx_buf, "\r\n(2 pressed LED off!)\r\n");
			HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, HAL_MAX_DELAY);
			  Set_Brightness(0);
			  WS2812_Send();

			uartStatus = UART_RX_NONE;
			break;
		}
		case UART_RX_NONE:
		{
			break;
		}
		default:
		{
			uartStatus = UART_RX_NONE;
			break;
		}
	}
}

/**
  * @brief Simple dongle interpreter
  * @retval None
  */
void dongle_interpreter(uint8_t * input)
{

	if(strlen((char *)input) != 0)
	{
		if(strstr((char *)input, "\r\nADVERTISING...") != NULL)
		{
			isAdvertising = true;
		}
		if(strstr((char *)input, "\r\nADVERTISING STOPPED.") != NULL)
		{
			isAdvertising = false;
		}
		if(strstr((char *)input, "\r\nCONNECTED") != NULL)
		{
			isConnected = true;
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
		}
		if(strstr((char *)input, "\r\nDISCONNECTED") != NULL)
		{
			isConnected = false;
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
		}
		if(strstr((char *)input, "L=0") != NULL)
		{
			isLEDOn = false;

			Set_Brightness(0);
			WS2812_Send();
			writeToDongle((uint8_t*)DONGLE_SEND_LED_OFF);

			uart_buf_len = sprintf(uart_tx_buf, "\r\nLED is %s\r\n", isLEDOn ? "on":"off");
			HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, HAL_MAX_DELAY);
		}
		if(strstr((char *)input, "L=1") != NULL)
		{
			isLEDOn = true;

			Set_Brightness(40);
			WS2812_Send();
			writeToDongle((uint8_t*)DONGLE_SEND_LED_ON);

			uart_buf_len = sprintf(uart_tx_buf, "\r\nLED is %s\r\n", isLEDOn ? "on":"off");
			HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, HAL_MAX_DELAY);
		}
		if(strstr((char *)input, "L=RED") != NULL)
		{

			// Set LEDs to rainbow colors
			Set_LED(0, 255, 0, 0);
			Set_LED(1, 255, 0, 0);
			Set_LED(2, 255, 0, 0);
			Set_LED(3, 255, 0, 0);
			Set_LED(4, 255, 0, 0);
			Set_LED(5, 255, 0, 0);
			Set_LED(6, 255, 0, 0);
			Set_LED(7, 255, 0, 0);

			if(isLEDOn)
			{
				Set_Brightness(40);
			}

			WS2812_Send();
			writeToDongle((uint8_t*)DONGLE_SEND_LED_RED);

			uart_buf_len = sprintf(uart_tx_buf, "\r\nLED is %s\r\n", isLEDOn ? "on":"off");
			HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, HAL_MAX_DELAY);
		}
		if(strstr((char *)input, "L=GREEN") != NULL)
		{


			// Set LEDs to rainbow colors
			Set_LED(0,  0, 255, 0);
			Set_LED(1,  0, 255, 0);
			Set_LED(2,  0, 255, 0);
			Set_LED(3,  0, 255, 0);
			Set_LED(4,  0, 255, 0);
			Set_LED(5,  0, 255, 0);
			Set_LED(6,  0, 255, 0);
			Set_LED(7,  0, 255, 0);

			if(isLEDOn)
			{
				Set_Brightness(40);
			}

			WS2812_Send();
			writeToDongle((uint8_t*)DONGLE_SEND_LED_GREEN);

			uart_buf_len = sprintf(uart_tx_buf, "\r\nLED is %s\r\n", isLEDOn ? "on":"off");
			HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, HAL_MAX_DELAY);
		}
		if(strstr((char *)input, "L=BLUE") != NULL)
		{

			// Set LEDs to rainbow colors
			Set_LED(0, 0, 0, 255);
			Set_LED(1, 0, 0, 255);
			Set_LED(2, 0, 0, 255);
			Set_LED(3, 0, 0, 255);
			Set_LED(4, 0, 0, 255);
			Set_LED(5, 0, 0, 255);
			Set_LED(6, 0, 0, 255);
			Set_LED(7, 0, 0, 255);

			if(isLEDOn)
			{
				Set_Brightness(40);
			}

			WS2812_Send();
			writeToDongle((uint8_t*)DONGLE_SEND_LED_BLUE);

			uart_buf_len = sprintf(uart_tx_buf, "\r\nLED is %s\r\n", isLEDOn ? "on":"off");
			HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, HAL_MAX_DELAY);
		}
		if(strstr((char *)input, "L=RAINBOW") != NULL)
		{

			// Set LEDs to rainbow colors
			Set_LED(0, 255, 0, 0);
			Set_LED(1, 0, 255, 0);
			Set_LED(2, 0, 0, 255);
			Set_LED(3, 46, 89, 128);
			Set_LED(4, 156, 233, 100);
			Set_LED(5, 102, 0, 235);
			Set_LED(6, 47, 38, 77);
			Set_LED(7, 255, 200, 0);

			if(isLEDOn)
			{
				Set_Brightness(40);
			}

			WS2812_Send();
			writeToDongle((uint8_t*)DONGLE_SEND_LED_RAINBOW);

			uart_buf_len = sprintf(uart_tx_buf, "\r\nLED is %s\r\n", isLEDOn ? "on":"off");
			HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, HAL_MAX_DELAY);
		}
	}
	memset(&dongle_response, 0, RSP_SIZE);
}


void Set_LED (int LEDnum, int Red, int Green, int Blue)
{
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = Green;
	LED_Data[LEDnum][2] = Red;
	LED_Data[LEDnum][3] = Blue;
}

#define PI 3.14159265

void Set_Brightness (int brightness)  // 0-45
{
#if USE_BRIGHTNESS

	if (brightness > 45) brightness = 45;
	for (int i=0; i<MAX_LED; i++)
	{
		LED_Mod[i][0] = LED_Data[i][0];
		for (int j=1; j<4; j++)
		{
			float angle = 90-brightness;  // in degrees
			angle = angle*PI / 180;  // in rad
			LED_Mod[i][j] = (LED_Data[i][j])/(tan(angle));
		}
	}

#endif

}

void WS2812_Send (void)
{
	uint32_t indx=0;
	uint32_t color;


	for (int i= 0; i<MAX_LED; i++)
	{
#if USE_BRIGHTNESS
		color = ((LED_Mod[i][1]<<16) | (LED_Mod[i][2]<<8) | (LED_Mod[i][3]));
#else
		color = ((LED_Data[i][1]<<16) | (LED_Data[i][2]<<8) | (LED_Data[i][3]));
#endif

		for (int i=23; i>=0; i--)
		{
			if (color&(1<<i))
			{
				pwmData[indx] = 60;  // 2/3 of 90
			}

			else pwmData[indx] = 30;  // 1/3 of 90

			indx++;
		}

	}

	for (int i=0; i<50; i++)
	{
		pwmData[indx] = 0;
		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);
	while (!datasentflag){};
	datasentflag = 0;
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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
