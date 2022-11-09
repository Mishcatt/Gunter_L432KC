/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "touchsensing.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "soundSamples.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define GUNTER_SAMPLING_MIN 1667
#define GUNTER_SAMPLING_DEFAULT 833
// 72 MHz / 750 = 96 kHz
// 720 = 100 kHz
// 1440 = 50 kHz

enum gunterStates {
	GUNTER_BOOT,
	GUNTER_OFF,
	GUNTER_WENK,
	GUNTER_SIN,
	GUNTER_SQUARE,
	GUNTER_TRIANGLE,
	GUNTER_SAW,
	GUNTER_NOISE,
	GUNTER_TOUCH,
	GUNTER_MIDI,
	GUNTER_CV
};

enum buttons {
	BUTTON_NONE = 0b00000000,
	BUTTON_B = 0b00000001,
	BUTTON_A = 0b00000010,
	BUTTON_RIGHT = 0b00000100,
	BUTTON_LEFT = 0b00001000,
	BUTTON_DOWN = 0b00010000,
	BUTTON_UP = 0b00100000
};

enum adcNames {
	ADC_KNOB_1 = 0,	// Function
	ADC_KNOB_2 = 1,
	ADC_CV = 2,		// Control Voltage
	ADC_POT_1 = 3,	// Frequency
	ADC_POT_2 = 4,	// Frequency Shift
	ADC_POT_3 = 5,	// Delay
	ADC_POT_4 = 6
};

enum knobInput {
	KNOB_1_MIN = 949,
	KNOB_1_MAX = 2093,
	KNOB_2_MIN = 2094,
	KNOB_2_MAX = 2265,
	KNOB_3_MIN = 2266,
	KNOB_3_MAX = 2468,
	KNOB_4_MIN = 2469,
	KNOB_4_MAX = 2693,
	KNOB_5_MIN = 2711,
	KNOB_5_MAX = 2985,
	KNOB_6_MIN = 3006,
	KNOB_6_MAX = 3340,
	KNOB_7_MIN = 3375,
	KNOB_7_MAX = 3787,
	KNOB_8_MIN = 3833,
	KNOB_8_MAX = 0xfff
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;

RNG_HandleTypeDef hrng;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

TSC_HandleTypeDef htsc;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

const uint16_t sinTable[SIN_TABLE] = { SIN_TABLE_CONTENT };
const uint16_t wenkTable[WENK_TABLE] = { WENK_TABLE_CONTENT };

volatile uint16_t adcBuffer[7] = {0, 0, 0, 0, 0, 0, 0};
volatile uint8_t gunterState = GUNTER_OFF;
volatile uint8_t gunterADCReady = 0;
volatile uint32_t gunterSampling[2] = {GUNTER_SAMPLING_MIN, GUNTER_SAMPLING_MIN};
volatile float gunterShift = 1.0f;
volatile int16_t gunterDistortion = 0;
volatile float gunterClipping = 1.0f;
volatile int16_t gunterDiminish = 0;
volatile float gunterPercussion = 1.0f;
volatile uint16_t gunterPeriod = 1000;
volatile float gunterPeriod1000 = 0xfff / 1000;
volatile float gunterPeriod2000 = 0x1fff / 1000;
volatile uint16_t gunterPeriodShift = 1000;
volatile float gunterPeriodShift1000 = 0xfff / 1000;
volatile float gunterPeriodShift2000 = 0x1fff / 1000;
volatile uint16_t gunterTempPeriod = 1000;
volatile uint16_t gunterDivision = 1;
volatile uint16_t gunterTempDivision = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_ADC1_Init(void);
static void MX_RNG_Init(void);
static void MX_TSC_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

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
	//HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_ADC1_Init();
  MX_RNG_Init();
  MX_TSC_Init();
  MX_TOUCHSENSING_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

	//HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	HAL_DACEx_DualSetValue(&hdac1, DAC_ALIGN_12B_R, 0x000, 0x000);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	//HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	HAL_ADC_Start_DMA(&hadc1, adcBuffer, 7);
	//HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if (gunterADCReady) {
			gunterADCReady = 0;

			if (adcBuffer[ADC_KNOB_1] <= KNOB_1_MAX && adcBuffer[ADC_KNOB_1] >= KNOB_1_MIN) {
				gunterState = GUNTER_OFF;
			}
			else if (adcBuffer[ADC_KNOB_1] <= KNOB_2_MAX && adcBuffer[ADC_KNOB_1] >= KNOB_2_MIN) {
				gunterState = GUNTER_WENK;
			}
			else if (adcBuffer[ADC_KNOB_1] <= KNOB_3_MAX && adcBuffer[ADC_KNOB_1] >= KNOB_3_MIN) {
				gunterState = GUNTER_SIN;
			}
			else if (adcBuffer[ADC_KNOB_1] <= KNOB_4_MAX && adcBuffer[ADC_KNOB_1] >= KNOB_4_MIN) {
				gunterState = GUNTER_TRIANGLE;
			}
			else if (adcBuffer[ADC_KNOB_1] <= KNOB_5_MAX && adcBuffer[ADC_KNOB_1] >= KNOB_5_MIN) {
				gunterState = GUNTER_SQUARE;
			}
			else if (adcBuffer[ADC_KNOB_1] <= KNOB_6_MAX && adcBuffer[ADC_KNOB_1] >= KNOB_6_MIN) {
				gunterState = GUNTER_SAW;
			}
			else if (adcBuffer[ADC_KNOB_1] <= KNOB_7_MAX && adcBuffer[ADC_KNOB_1] >= KNOB_7_MIN) {
				gunterState = GUNTER_NOISE;
			}
			else if (adcBuffer[ADC_KNOB_1] <= KNOB_8_MAX && adcBuffer[ADC_KNOB_1] >= KNOB_8_MIN) {
				gunterState = GUNTER_CV;
			}

			gunterPeriod = (0xfc4 - adcBuffer[ADC_POT_1]); // TODO: Skalowanie?
			if (gunterPeriod < 3) gunterPeriod = 3;
			//gunterPeriod = 2;
			gunterPeriod1000 = 4096.0f / gunterPeriod;
			gunterPeriod2000 = 8192.0f / gunterPeriod;

			if (adcBuffer[ADC_POT_2] > 0x8ff) {
				gunterShift = ((0x10ff - adcBuffer[ADC_POT_2]) / 4096.0f) + 0.5f; // 0.5625 - 1.0
			}
			else if (adcBuffer[ADC_POT_2] < 0x700) {
				gunterShift = ((0xf00 - adcBuffer[ADC_POT_2]) / 4096.0f) + 0.5f; // 1.0 - 1.4375
			}
			else {
				gunterShift = 1.0f;
			}
			gunterShift = 1.0f;

			gunterPeriodShift = gunterPeriod * gunterShift;
			gunterPeriodShift1000 = 4096.0f / gunterPeriodShift;
			gunterPeriodShift2000 = 8192.0f / gunterPeriodShift;


			if (adcBuffer[ADC_POT_3] > 0x8ff) {
				gunterDistortion = adcBuffer[ADC_POT_3] - 0x8ff;
			}
			else if (adcBuffer[ADC_POT_3] < 0x700) {
				gunterDistortion = adcBuffer[ADC_POT_3] - 0x700;
			}
			else {
				gunterDistortion = 0;
			}
			gunterDistortion = 0;

			gunterClipping = -(gunterDistortion*0.01f) + 1.0f;

			if (adcBuffer[ADC_POT_2] > 0x8ff) {
				gunterDiminish = adcBuffer[ADC_POT_2] - 0x8ff;
			}
			else if (adcBuffer[ADC_POT_2] < 0x700) {
				gunterDiminish = adcBuffer[ADC_POT_2] - 0x700;
			}
			else {
				gunterDiminish = 0;
			}
			//gunterDiminish = adcBuffer[ADC_POT_2] - 0x7ff;
			if (gunterDiminish < 0) {
				gunterPercussion = 1.0f + (gunterDiminish*0.0000005f);
			} else {
				gunterPercussion = 1.0f;
			}

			if (gunterState == GUNTER_WENK) {
				if (adcBuffer[ADC_POT_1] > 0x7ff) {
					gunterTempDivision = 1024;
					gunterSampling[0] = (0x800 - (adcBuffer[ADC_POT_1] - 0x7ff)) * (GUNTER_SAMPLING_DEFAULT) * gunterTempDivision;
					gunterSampling[0] /= 0x800;
					while (gunterSampling[0] >= GUNTER_SAMPLING_MIN*2) {
						gunterSampling[0] /= 2;
						gunterTempDivision /= 2;
					}
				} else {
					gunterTempDivision = 2;
					gunterSampling[0] = (GUNTER_SAMPLING_DEFAULT + ((0x7ff - adcBuffer[ADC_POT_1]) * (0x7ff - adcBuffer[ADC_POT_1]) / 256)) * gunterTempDivision;
					if (gunterSampling[0] >= GUNTER_SAMPLING_MIN*2) {
						gunterSampling[0] /= 2;
						gunterTempDivision /= 2;
					}
				}
				gunterSampling[1] = gunterSampling[0] * gunterShift;
			}
			else if (gunterState == GUNTER_SIN) {
				gunterTempDivision = 1024;
				gunterSampling[0] = (0x1000 - adcBuffer[ADC_POT_1]) * GUNTER_SAMPLING_DEFAULT * gunterTempDivision;
				gunterSampling[0] /= 0x5800;
				while (gunterSampling[0] >= GUNTER_SAMPLING_MIN*2) {
					gunterSampling[0] /= 2;
					gunterTempDivision /= 2;
				}
				gunterSampling[1] = gunterSampling[0] * gunterShift;
			} else {
				gunterTempDivision = 1;
				gunterSampling[0] = GUNTER_SAMPLING_MIN;
				gunterSampling[1] = GUNTER_SAMPLING_MIN;
			}

			gunterDivision = gunterTempDivision;
			__HAL_TIM_SET_AUTORELOAD(&htim6, gunterSampling[0]);
			__HAL_TIM_SET_AUTORELOAD(&htim7, gunterSampling[1]);

			//HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		}

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RNG|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 12;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 7;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_256;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_8;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T7_TRGO;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  htim6.Init.Period = 8000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 8000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TSC Initialization Function
  * @param None
  * @retval None
  */
static void MX_TSC_Init(void)
{

  /* USER CODE BEGIN TSC_Init 0 */

  /* USER CODE END TSC_Init 0 */

  /* USER CODE BEGIN TSC_Init 1 */

  /* USER CODE END TSC_Init 1 */

  /** Configure the TSC peripheral
  */
  htsc.Instance = TSC;
  htsc.Init.CTPulseHighLength = TSC_CTPH_2CYCLES;
  htsc.Init.CTPulseLowLength = TSC_CTPL_2CYCLES;
  htsc.Init.SpreadSpectrum = DISABLE;
  htsc.Init.SpreadSpectrumDeviation = 1;
  htsc.Init.SpreadSpectrumPrescaler = TSC_SS_PRESC_DIV1;
  htsc.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV4;
  htsc.Init.MaxCountValue = TSC_MCV_8191;
  htsc.Init.IODefaultMode = TSC_IODEF_OUT_PP_LOW;
  htsc.Init.SynchroPinPolarity = TSC_SYNC_POLARITY_FALLING;
  htsc.Init.AcquisitionMode = TSC_ACQ_MODE_NORMAL;
  htsc.Init.MaxCountInterrupt = DISABLE;
  htsc.Init.ChannelIOs = TSC_GROUP2_IO2;
  htsc.Init.ShieldIOs = 0;
  htsc.Init.SamplingIOs = TSC_GROUP2_IO1;
  if (HAL_TSC_Init(&htsc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TSC_Init 2 */

  /* USER CODE END TSC_Init 2 */

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
  huart1.Init.BaudRate = 31250;
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
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) { // 96 kHz
	static uint16_t currentSample[2] = {0, 0};
	static uint16_t currentDistortion[2] = {0, 0};
	static uint32_t currentNoise[2] = {1, 1};
	static float currentVolume[2] = {1.0f, 1.0f};
	int16_t temp[2] = {0, 0};

	if (htim == &htim6) {
		//HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);

		if (gunterState == GUNTER_TRIANGLE) {
			if (currentSample[0] >= (gunterPeriod)) currentSample[0] = 0;
			if (currentSample[0] <= (gunterPeriod / 2)) {
				temp[0] = currentSample[0] * gunterPeriod2000;
			} else {
				temp[0] = 0x1fff - (currentSample[0] * gunterPeriod2000);
			}
		}

		else if (gunterState == GUNTER_SAW) {
			if (currentSample[0] >= gunterPeriod) currentSample[0] = 0;
			temp[0] = (gunterPeriod - currentSample[0]) * gunterPeriod1000;
		}

		else if (gunterState == GUNTER_NOISE) {
			if (currentSample[0] >= gunterPeriod) {
				currentSample[0] = 0;
			}
			if (currentSample[0] == 0) {
				currentNoise[0] = (214013 * currentNoise[0] + 2531011);
			}
			temp[0] = currentNoise[0] & 0xfff;
		}

		else if (gunterState == GUNTER_SQUARE) {
			if (currentSample[0] >= gunterPeriod) currentSample[0] = 0;
			if (currentSample[0] >= (gunterPeriod / 2)) {
				temp[0] = 0x000;
			} else {
				temp[0] = 0xfff;
			}
		}

		else if (gunterState == GUNTER_WENK) {
			if (currentSample[0] >= WENK_TABLE) currentSample[0] = 0;
			temp[0] = wenkTable[currentSample[0]];
		}

		else if (gunterState == GUNTER_SIN) {
			if (currentSample[0] >= SIN_TABLE) currentSample[0] = 0;
			temp[0] = sinTable[currentSample[0]];
		}

		else if (gunterState == GUNTER_OFF) {
			currentSample[0] = 0;
			temp[0] = 0;
		}

		currentSample[0] += gunterDivision;

		temp[0] -= 0x7ff;
		if (gunterDistortion < 0) {
			temp[0] *= gunterClipping;
		}
		if (temp[0] > 0x7ff) temp[0] = 0x7ff;
		else if (temp[0] < -0x7ff) temp[0] = -0x7ff;
		temp[0] += 0x7ff;
		temp[0] *= currentVolume[0];

		if (gunterPercussion == 1.0f || currentVolume[0] < 0.1f) {
			currentVolume[0] = 1.0f;
		} else {
			currentVolume[0] *= gunterPercussion;
		}

		if (currentDistortion[0] > 0) {
			currentDistortion[0]--;
		} else {
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, temp[0]);
			if (gunterDistortion > 0) {
				currentDistortion[0] = gunterDistortion >> 3;
			}
		}
	}

	else if (htim == &htim7) {
		//HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

		if (gunterState == GUNTER_TRIANGLE) {
			if (currentSample[1] >= gunterPeriodShift) currentSample[1] = 0;
			if (currentSample[1] <= (gunterPeriodShift / 2)) {
				temp[1] = currentSample[1] * gunterPeriodShift2000;
			} else {
				temp[1] = 0x1fff - (currentSample[1] * gunterPeriodShift2000);
			}
		}

		else if (gunterState == GUNTER_SAW) {
			if (currentSample[1] >= gunterPeriodShift) currentSample[1] = 0;
			temp[1] = (gunterPeriodShift - currentSample[1]) * gunterPeriodShift1000;
		}

		else if (gunterState == GUNTER_NOISE) {
			if (currentSample[1] >= gunterPeriod) {
				currentSample[1] = 0;
			}
			if (currentSample[1] == 0) {
				currentNoise[1] = (214013 * currentNoise[1] + 2531011);
			}
			temp[1] = currentNoise[1] & 0xfff;
		}

		else if (gunterState == GUNTER_SQUARE) {
			if (currentSample[1] >= gunterPeriodShift) currentSample[1] = 0;
			if (currentSample[1] >= (gunterPeriodShift / 2)) {
				temp[1] = 0x000;
			} else {
				temp[1] = 0xfff;
			}
		}

		else if (gunterState == GUNTER_WENK) {
			if (currentSample[1] >= WENK_TABLE) currentSample[1] = 0;
			temp[1] = wenkTable[currentSample[1]];
		}

		else if (gunterState == GUNTER_SIN) {
			if (currentSample[1] >= SIN_TABLE) currentSample[1] = 0;
			temp[1] = sinTable[currentSample[1]];
		}

		else if (gunterState == GUNTER_OFF) {
			currentSample[1] = 0;
			temp[1] = 0;
		}

		currentSample[1] += gunterDivision;

		temp[1] -= 0x7ff;
		if (gunterDistortion < 0) {
			temp[1] *= gunterClipping;
		}
		if (temp[1] > 0x7ff) temp[1] = 0x7ff;
		else if (temp[1] < -0x7ff) temp[1] = -0x7ff;
		temp[1] += 0x7ff;
		temp[1] *= currentVolume[1];

		if (gunterPercussion == 1.0f || currentVolume[1] < 0.1f) {
			currentVolume[1] = 1.0f;
		} else {
			currentVolume[1] *= gunterPercussion;
		}

		if (currentDistortion[1] > 0) {
			currentDistortion[1]--;
		} else {
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, temp[1]);
			if (gunterDistortion > 0) {
				currentDistortion[1] = gunterDistortion >> 3;
			}
		}
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	/*static uint32_t randomNumberBuffer = 0;
	static uint8_t randomShift = 0;
	static uint8_t randomBitsBuffer = 0;*/

	gunterADCReady = 1;

	//HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

	/*if (adcBuffer[ADC_KNOB_2] & 1) {
		randomNumberBuffer |= 1 << randomShift;
		randomBitsBuffer++;
	}
	randomShift++;

	if (randomShift > 31) {
		randomNumberBuffer = 0;
		randomShift = 0;
		randomBitsBuffer = 0;
	}*/
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
