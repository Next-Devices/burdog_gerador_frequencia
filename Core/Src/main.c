/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define FREQ_EXTRA_BAIXO   24850
#define FREQ_BAIXO         24300
#define FREQ_MEDIO         23500
#define FREQ_ALTO          22340
#define FREQ_VAZIO         26500

typedef enum {
	NIVEL_NENHUM = 0,
    NIVEL_EXTRA_BAIXO,
    NIVEL_BAIXO,
    NIVEL_MEDIO,
    NIVEL_ALTO
} NivelPressostato;

NivelPressostato nivelAtual = NIVEL_NENHUM;
uint32_t freqAlvo = FREQ_EXTRA_BAIXO;
uint32_t freqAtual = FREQ_VAZIO;

uint8_t enchendo = 0;
uint8_t esvaziando = 0;
uint32_t last_nivel_press_time = 0;
uint32_t last_encher_press_time = 0;
uint32_t last_esvaziar_press_time = 0;
const uint32_t DEBOUNCE_TIME = 200; // 200ms
const uint32_t FREQ_UPDATE_TIME = 50; // 50ms
uint32_t last_freq_update_time = 0;
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);

void PWM_SetFrequency(uint32_t freq);
void AtualizaLEDs(NivelPressostato nivel);
/* USER CODE BEGIN PFP */

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
  MX_TIM2_Init();
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  // Inicializa com frequência de "vazio"
  PWM_SetFrequency(FREQ_VAZIO);

  // Desliga todos os LEDs inicialmente
  HAL_GPIO_WritePin(extra_baixo_GPIO_Port, extra_baixo_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(baixo_GPIO_Port, baixo_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(medio_GPIO_Port, medio_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(alto_GPIO_Port, alto_Pin, GPIO_PIN_RESET);
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint32_t current_time = HAL_GetTick();

      // Lógica para o botão "selecionar nível"
      if (HAL_GPIO_ReadPin(nivel_GPIO_Port, nivel_Pin) == GPIO_PIN_RESET && (current_time - last_nivel_press_time) > DEBOUNCE_TIME) {
          last_nivel_press_time = current_time;

              nivelAtual = (nivelAtual + 1) % 5;

              switch (nivelAtual) {
                  case NIVEL_EXTRA_BAIXO: freqAlvo = FREQ_EXTRA_BAIXO; break;
                  case NIVEL_BAIXO:       freqAlvo = FREQ_BAIXO;       break;
                  case NIVEL_MEDIO:       freqAlvo = FREQ_MEDIO;       break;
                  case NIVEL_ALTO:        freqAlvo = FREQ_ALTO;        break;
                  case NIVEL_NENHUM:      freqAlvo = FREQ_VAZIO;       break;
                  default: break;
              }

              HAL_GPIO_WritePin(extra_baixo_GPIO_Port, extra_baixo_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(baixo_GPIO_Port, baixo_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(medio_GPIO_Port, medio_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(alto_GPIO_Port, alto_Pin, GPIO_PIN_RESET);
      }

      // Lógica para o botão "encher"
      if (HAL_GPIO_ReadPin(encher_GPIO_Port, encher_Pin) == GPIO_PIN_RESET && (current_time - last_encher_press_time) > DEBOUNCE_TIME) {
          last_encher_press_time = current_time;
          if (nivelAtual != NIVEL_NENHUM) {
              enchendo = 1;
              esvaziando = 0;
          }
      }

      // Lógica para o botão "esvaziar"
      if (HAL_GPIO_ReadPin(esvaziar_GPIO_Port, esvaziar_Pin) == GPIO_PIN_RESET && (current_time - last_esvaziar_press_time) > DEBOUNCE_TIME) {
          last_esvaziar_press_time = current_time;
          enchendo = 0;
          esvaziando = 1;
      }

      // Lógica de controle de frequência - ENCHENDO e ESVAZIANDO
      if ((enchendo || esvaziando) && (current_time - last_freq_update_time) > FREQ_UPDATE_TIME) {
          last_freq_update_time = current_time;

          if (enchendo) {
              if (freqAtual < freqAlvo) {
                  freqAtual += 10;
                  PWM_SetFrequency(freqAtual);
              } else {
                  enchendo = 0;
                  AtualizaLEDs(nivelAtual);
              }
          } else if (esvaziando) {
              if (freqAtual > FREQ_VAZIO) {
                  freqAtual -= 10;
                  PWM_SetFrequency(freqAtual);

                  // Lógica para desligar os LEDs conforme a frequência diminui
                  if (freqAtual < FREQ_ALTO) HAL_GPIO_WritePin(alto_GPIO_Port, alto_Pin, GPIO_PIN_RESET);
                  if (freqAtual < FREQ_MEDIO) HAL_GPIO_WritePin(medio_GPIO_Port, medio_Pin, GPIO_PIN_RESET);
                  if (freqAtual < FREQ_BAIXO) HAL_GPIO_WritePin(baixo_GPIO_Port, baixo_Pin, GPIO_PIN_RESET);
                  if (freqAtual < FREQ_EXTRA_BAIXO) HAL_GPIO_WritePin(extra_baixo_GPIO_Port, extra_baixo_Pin, GPIO_PIN_RESET);

              } else {
                  esvaziando = 0;
                  freqAtual = FREQ_VAZIO;
                  PWM_SetFrequency(FREQ_VAZIO);
                  nivelAtual = NIVEL_NENHUM;
                  HAL_GPIO_WritePin(extra_baixo_GPIO_Port, extra_baixo_Pin, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(baixo_GPIO_Port, baixo_Pin, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(medio_GPIO_Port, medio_Pin, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(alto_GPIO_Port, alto_Pin, GPIO_PIN_RESET);
              }
          }
      }
  }
  /* USER CODE END 3 */
}


void PWM_SetFrequency(uint32_t freq)
{
    uint32_t timer_clock = 72000000; // APB1 Timer clock (Blue Pill)
    uint32_t prescaler = 0;

    uint32_t arr = (timer_clock / (prescaler + 1)) / freq - 1;

    __HAL_TIM_SET_AUTORELOAD(&htim2, arr); //sobrescreve arr para chegar na frequencia desejada
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, arr/2); // duty 50%
}

void AtualizaLEDs(NivelPressostato nivel)
{
    // Desliga todos os LEDs primeiro
    HAL_GPIO_WritePin(extra_baixo_GPIO_Port, extra_baixo_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(baixo_GPIO_Port, baixo_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(medio_GPIO_Port, medio_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(alto_GPIO_Port, alto_Pin, GPIO_PIN_RESET);

    // Acende apenas o LED correspondente ao nível atingido
    switch (nivel) {
        case NIVEL_EXTRA_BAIXO:
            HAL_GPIO_WritePin(extra_baixo_GPIO_Port, extra_baixo_Pin, GPIO_PIN_SET);
            break;
        case NIVEL_BAIXO:
            HAL_GPIO_WritePin(baixo_GPIO_Port, baixo_Pin, GPIO_PIN_SET);
            break;
        case NIVEL_MEDIO:
            HAL_GPIO_WritePin(medio_GPIO_Port, medio_Pin, GPIO_PIN_SET);
            break;
        case NIVEL_ALTO:
            HAL_GPIO_WritePin(alto_GPIO_Port, alto_Pin, GPIO_PIN_SET);
            break;
        default:
            // Nenhum LED para NIVEL_NENHUM
            break;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  // inicia com aproximadamente 26.5k = 72e6/(2716)*1
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2716-1;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, alto_Pin|medio_Pin|baixo_Pin|extra_baixo_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : alto_Pin medio_Pin baixo_Pin extra_baixo_Pin */
  GPIO_InitStruct.Pin = alto_Pin|medio_Pin|baixo_Pin|extra_baixo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : esvaziar_Pin encher_Pin nivel_Pin */
  GPIO_InitStruct.Pin = esvaziar_Pin|encher_Pin|nivel_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
