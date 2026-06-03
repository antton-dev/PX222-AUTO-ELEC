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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define ADC_VREF        3.3f
#define ADC_MAX         4095.0f

#define ALPHA_NEUTRE    0.50f

/*
 * Sécurité pour les premiers essais.
 * On limite la commande entre 45 % et 55 %.
 */
#define ALPHA_MIN       0.40f
#define ALPHA_MAX       0.60f


/*
 * Correcteur PI issu du rapport :
 * R(p) = k(1 + wi / p)
 */
static float Kp = 1.00f;
static float Wi = 36.95f;

/*
 * Période d'échantillonnage visée : 1 ms.
 */
#define TE_S 0.001f

/*
 * Consigne de courant.
 * Pour les premiers essais à E0 = 20 V, rester faible.
 */
static float i_ref_A = 0.05f;

/*
 * Mémoire de l'intégrale.
 * Unité : A.s
 */
static float integrale = 0.0f;

/*
 * Limite logicielle de l'intégrale.
 * Protection supplémentaire contre l'emballement.
 */
#define INTEGRALE_MIN -0.020f
#define INTEGRALE_MAX  0.020f

/*
 * Variables visibles en debug.
 */
volatile float dbg_i_mes_A = 0.0f;
volatile float dbg_i_ref_A = 0.0f;
volatile float dbg_erreur_A = 0.0f;
volatile float dbg_alpha = 0.0f;

volatile float dbg_integrale = 0.0f;
volatile float dbg_u_pi = 0.0f;
volatile float dbg_alpha_unsat = 0.0f;
volatile float dbg_alpha_sat = 0.0f;

static void ResetPI(void)
{
  integrale = 0.0f;
}

static float offset_i_mes_A = 0.0f;

static float Saturate(float x, float min, float max)
{
  if (x < min) return min;
  if (x > max) return max;
  return x;
}

static void PWM_SetAlpha(float alpha)
{
  alpha = Saturate(alpha, 0.0f, 1.0f);

  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
  uint32_t ccr = (uint32_t)(alpha * (float)(arr + 1U));

  if (ccr > arr + 1U)
  {
    ccr = arr + 1U;
  }

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
}

static float ADC_ToVoltage(uint16_t adc)
{
  return ADC_VREF * ((float)adc / ADC_MAX);
}

typedef struct
{
  uint16_t mes_i_adc;
  uint16_t ref_i_adc;
} ADC_Values_t;

static ADC_Values_t ADC_ReadAll(void)
{
  ADC_Values_t values = {0};

  HAL_ADC_Start(&hadc1);

  if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
  {
    Error_Handler();
  }
  values.mes_i_adc = HAL_ADC_GetValue(&hadc1);

  if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
  {
    Error_Handler();
  }
  values.ref_i_adc = HAL_ADC_GetValue(&hadc1);

  HAL_ADC_Stop(&hadc1);

  return values;
}

static float ReadCurrent_A(void)
{
  ADC_Values_t adc = ADC_ReadAll();

  /*
   * Sujet : Mes_I donne 1 V pour 1 A.
   */
  float v_mes = ADC_ToVoltage(adc.mes_i_adc);
  float i_mes = v_mes;

  return i_mes - offset_i_mes_A;
}

static float ReadCurrentFiltered_A(void)
{
  float sum = 0.0f;

  for (int k = 0; k < 16; k++)
  {
    sum += ReadCurrent_A();
  }

  return sum / 16.0f;
}

static void CalibrateCurrentOffset(void)
{
  float sum = 0.0f;

  PWM_SetAlpha(ALPHA_NEUTRE);
  HAL_Delay(500);

  for (int k = 0; k < 200; k++)
  {
    ADC_Values_t adc = ADC_ReadAll();
    sum += ADC_ToVoltage(adc.mes_i_adc);
    HAL_Delay(1);
  }

  offset_i_mes_A = sum / 200.0f;
}
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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_TIM_MOE_ENABLE(&htim1);

  /*
   * Démarrage au neutre.
   */
  PWM_SetAlpha(ALPHA_NEUTRE);

  /*
   * Calibration de l'offset de mesure courant.
   * À faire avec alpha = 0.50 et courant supposé nul.
   */
  CalibrateCurrentOffset();


  ResetPI();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_tick = HAL_GetTick();

  while (1)
  {
    /*
     * Boucle de correction toutes les 1 ms.
     */
	  if ((HAL_GetTick() - last_tick) >= 1)
	  {
	    last_tick += 1;

	    float i_mes_A = ReadCurrentFiltered_A();

	    /*
	     * Erreur de courant.
	     * Mes_I est en 1 V = 1 A.
	     */
	    float erreur_A = i_ref_A - i_mes_A;

	    /*
	     * Calcul de la commande avant mise à jour éventuelle de l'intégrale.
	     * u_pi représente l'écart à ajouter autour du neutre alpha = 0.5.
	     */
	    float u_pi_avant = Kp * (erreur_A + Wi * integrale);
	    float alpha_avant = ALPHA_NEUTRE + u_pi_avant;

	    /*
	     * Anti-windup inspiré de votre rapport :
	     * on bloque l'intégrateur si la commande est saturée
	     * ET si l'erreur pousse encore plus dans le sens de la saturation.
	     */
	    uint8_t saturation_haute = (alpha_avant >= ALPHA_MAX);
	    uint8_t saturation_basse = (alpha_avant <= ALPHA_MIN);

	    uint8_t bloquer_integrale =
	        (saturation_haute && erreur_A > 0.0f) ||
	        (saturation_basse && erreur_A < 0.0f);

	    if (!bloquer_integrale)
	    {
	      integrale += erreur_A * TE_S;
	      integrale = Saturate(integrale, INTEGRALE_MIN, INTEGRALE_MAX);
	    }

	    /*
	     * Calcul PI final.
	     */
	    float u_pi = Kp * (erreur_A + Wi * integrale);

	    float alpha_unsat = ALPHA_NEUTRE + u_pi;
	    float alpha = Saturate(alpha_unsat, ALPHA_MIN, ALPHA_MAX);

	    PWM_SetAlpha(alpha);

	    /*
	     * Variables observables dans le debugger.
	     */
	    dbg_i_mes_A = i_mes_A;
	    dbg_i_ref_A = i_ref_A;
	    dbg_erreur_A = erreur_A;
	    dbg_integrale = integrale;
	    dbg_u_pi = u_pi;
	    dbg_alpha_unsat = alpha_unsat;
	    dbg_alpha_sat = alpha;
	    dbg_alpha = alpha;
	  }
  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim1.Init.Period = 2908;
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
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1454;
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
