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
#include <string.h>
#include <stdlib.h>
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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define ADC_MAX_COUNT          4095.0f
#define ADC_VREF               3.3f
#define SENSOR_DIVIDER_RATIO   (20.0f / (9.79f + 20.0f))
#define SENSOR_V_MIN           0.5f
#define SENSOR_V_MAX           4.5f
#define PRESSURE_BAR_MIN       0.0f
#define PRESSURE_BAR_MAX       12.0f
#define AC_INPUT_RMS           220.0f

#define TIM1_CLK_HZ            72000000UL
#define PWM_FREQ_MIN_HZ        100U
#define PWM_FREQ_MAX_HZ        50000U
#define PWM_DEFAULT_FREQ_HZ    5000U
#define DUTY_MIN_PERCENT       0.0f
#define DUTY_MAX_PERCENT       95.0f
#define DUTY_DEFAULT_PERCENT   50.0f

#define TELEMETRY_PERIOD_MS    100U
#define UART_RX_BUFFER_SIZE    96U
#define UART_TX_BUFFER_SIZE    160U

static uint32_t adc_raw = 0;
static uint32_t send_tick = 0;
static uint32_t pwm_freq_hz = PWM_DEFAULT_FREQ_HZ;
static float duty_percent = DUTY_DEFAULT_PERCENT;
static float adc_voltage = 0.0f;
static float sensor_voltage = 0.0f;
static float pressure_bar = 0.0f;
static float estimated_output_rms = 0.0f;

static uint8_t uart_rx_byte = 0;
static char uart_rx_buffer[UART_RX_BUFFER_SIZE];
static uint16_t uart_rx_index = 0;
static volatile uint8_t uart_command_ready = 0;
static char uart_command_buffer[UART_RX_BUFFER_SIZE];
static char uart_tx_buffer[UART_TX_BUFFER_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, 10);
	return ch;
}

static void PWM_Set(uint32_t frequency_hz, float duty_pct);
static void ADC_ReadPressure(void);
static void UART_StartReceiveIT(void);
static void UART_ProcessCommand(const char *command);
static void UART_SendTelemetry(void);
static float clamp_float(float value, float min_value, float max_value);
static uint32_t clamp_u32(uint32_t value, uint32_t min_value, uint32_t max_value);
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

  /* Configure the System Power */
  SystemPower_Config();

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_ICACHE_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	PWM_Set(pwm_freq_hz, duty_percent);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	UART_StartReceiveIT();

	HAL_Delay(500);
	snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
			"READY,PWM_AC_CHOPPER,%.1f,%lu\r\n", duty_percent,
			(unsigned long) pwm_freq_hz);
	HAL_UART_Transmit(&huart1, (uint8_t*) uart_tx_buffer,
			strlen(uart_tx_buffer), 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (uart_command_ready) {
			__disable_irq();
			char command[UART_RX_BUFFER_SIZE];
			strncpy(command, uart_command_buffer, sizeof(command));
			command[sizeof(command) - 1] = '\0';
			uart_command_ready = 0;
			__enable_irq();
			UART_ProcessCommand(command);
		}

		ADC_ReadPressure();

		if ((HAL_GetTick() - send_tick) >= TELEMETRY_PERIOD_MS) {
			send_tick = HAL_GetTick();
			UART_SendTelemetry();
		}

		HAL_Delay(10);
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_5CYCLE;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

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
  htim1.Init.Period = 7199;
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
  HAL_TIMEx_EnableDeadTimePreload(&htim1);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 72;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_ENABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static float clamp_float(float value, float min_value, float max_value) {
	if (value < min_value) {
		return min_value;
	}
	if (value > max_value) {
		return max_value;
	}
	return value;
}

static uint32_t clamp_u32(uint32_t value, uint32_t min_value, uint32_t max_value) {
	if (value < min_value) {
		return min_value;
	}
	if (value > max_value) {
		return max_value;
	}
	return value;
}

static void PWM_Set(uint32_t frequency_hz, float duty_pct) {
	frequency_hz = clamp_u32(frequency_hz, PWM_FREQ_MIN_HZ, PWM_FREQ_MAX_HZ);
	duty_pct = clamp_float(duty_pct, DUTY_MIN_PERCENT, DUTY_MAX_PERCENT);

	uint32_t period = TIM1_CLK_HZ / frequency_hz;
	if (period < 2U) {
		period = 2U;
	}
	period -= 1U;

	uint32_t compare = (uint32_t) (((float) (period + 1U) * duty_pct) / 100.0f);
	if (compare > period) {
		compare = period;
	}

	pwm_freq_hz = frequency_hz;
	duty_percent = duty_pct;
	estimated_output_rms = AC_INPUT_RMS * (duty_percent / 100.0f);

	__HAL_TIM_DISABLE(&htim1);
	__HAL_TIM_SET_AUTORELOAD(&htim1, period);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compare);
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	__HAL_TIM_ENABLE(&htim1);
}

static void ADC_ReadPressure(void) {
	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
		adc_raw = HAL_ADC_GetValue(&hadc1);
	}
	HAL_ADC_Stop(&hadc1);

	adc_voltage = ((float) adc_raw / ADC_MAX_COUNT) * ADC_VREF;
	sensor_voltage = adc_voltage / SENSOR_DIVIDER_RATIO;
	pressure_bar = ((sensor_voltage - SENSOR_V_MIN)
			/ (SENSOR_V_MAX - SENSOR_V_MIN)) * PRESSURE_BAR_MAX;
	pressure_bar = clamp_float(pressure_bar, PRESSURE_BAR_MIN, PRESSURE_BAR_MAX);
}

static void UART_StartReceiveIT(void) {
	HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
}

static void UART_ProcessCommand(const char *command) {
	char local[UART_RX_BUFFER_SIZE];
	strncpy(local, command, sizeof(local));
	local[sizeof(local) - 1] = '\0';

	char *cmd = strtok(local, ", \t");
	if (cmd == NULL) {
		return;
	}

	if (strcmp(cmd, "GET") == 0) {
		UART_SendTelemetry();
		return;
	}

	if (strcmp(cmd, "STOP") == 0) {
		PWM_Set(pwm_freq_hz, 0.0f);
		HAL_UART_Transmit(&huart1, (uint8_t*) "OK,STOP\r\n", 9, 100);
		return;
	}

	if (strcmp(cmd, "SET") != 0) {
		HAL_UART_Transmit(&huart1, (uint8_t*) "ERR,UNKNOWN_CMD\r\n", 17, 100);
		return;
	}

	char *field = strtok(NULL, ", \t");
	char *value = strtok(NULL, ", \t");
	if (field == NULL || value == NULL) {
		HAL_UART_Transmit(&huart1, (uint8_t*) "ERR,BAD_FORMAT\r\n", 16, 100);
		return;
	}

	if (strcmp(field, "DUTY") == 0) {
		float new_duty = strtof(value, NULL);
		PWM_Set(pwm_freq_hz, new_duty);
		snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "OK,DUTY,%.2f\r\n",
				duty_percent);
		HAL_UART_Transmit(&huart1, (uint8_t*) uart_tx_buffer,
				strlen(uart_tx_buffer), 100);
		return;
	}

	if (strcmp(field, "FREQ") == 0) {
		uint32_t new_freq = (uint32_t) strtoul(value, NULL, 10);
		PWM_Set(new_freq, duty_percent);
		snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "OK,FREQ,%lu\r\n",
				(unsigned long) pwm_freq_hz);
		HAL_UART_Transmit(&huart1, (uint8_t*) uart_tx_buffer,
				strlen(uart_tx_buffer), 100);
		return;
	}

	if (strcmp(field, "BOTH") == 0) {
		char *freq_value = strtok(NULL, ", \t");
		if (freq_value == NULL) {
			HAL_UART_Transmit(&huart1, (uint8_t*) "ERR,BAD_FORMAT\r\n", 16, 100);
			return;
		}
		float new_duty = strtof(value, NULL);
		uint32_t new_freq = (uint32_t) strtoul(freq_value, NULL, 10);
		PWM_Set(new_freq, new_duty);
		snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "OK,BOTH,%.2f,%lu\r\n",
				duty_percent, (unsigned long) pwm_freq_hz);
		HAL_UART_Transmit(&huart1, (uint8_t*) uart_tx_buffer,
				strlen(uart_tx_buffer), 100);
		return;
	}

	HAL_UART_Transmit(&huart1, (uint8_t*) "ERR,UNKNOWN_FIELD\r\n", 19, 100);
}

static void UART_SendTelemetry(void) {
	uint32_t adc_mv = (uint32_t) ((adc_voltage * 1000.0f) + 0.5f);
	uint32_t sensor_mv = (uint32_t) ((sensor_voltage * 1000.0f) + 0.5f);
	uint32_t pressure_mbar = (uint32_t) ((pressure_bar * 1000.0f) + 0.5f);
	uint32_t duty_centipercent = (uint32_t) ((duty_percent * 100.0f) + 0.5f);
	uint32_t rms_centivolt = (uint32_t) ((estimated_output_rms * 100.0f) + 0.5f);

	snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
			"DATA,%lu,%u,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
			(unsigned long) HAL_GetTick(), (unsigned int) adc_raw,
			(unsigned long) adc_mv, (unsigned long) sensor_mv,
			(unsigned long) pressure_mbar, (unsigned long) duty_centipercent,
			(unsigned long) pwm_freq_hz, (unsigned long) rms_centivolt);
	HAL_UART_Transmit(&huart1, (uint8_t*) uart_tx_buffer,
			strlen(uart_tx_buffer), 100);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		if (uart_rx_byte == '\n' || uart_rx_byte == '\r') {
			if (uart_rx_index > 0) {
				uart_rx_buffer[uart_rx_index] = '\0';
				strncpy(uart_command_buffer, uart_rx_buffer,
						sizeof(uart_command_buffer));
				uart_command_buffer[sizeof(uart_command_buffer) - 1] = '\0';
				uart_command_ready = 1;
				uart_rx_index = 0;
			}
		} else if (uart_rx_index < (UART_RX_BUFFER_SIZE - 1U)) {
			uart_rx_buffer[uart_rx_index++] = (char) uart_rx_byte;
		} else {
			uart_rx_index = 0;
		}
		HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
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
	while (1) {
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
