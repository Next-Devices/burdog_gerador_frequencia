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
/* USER CODE BEGIN PD */
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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
uint8_t enchendo = 0;
uint8_t esvaziando = 0;
uint32_t last_nivel_press_time = 0;
uint32_t last_encher_press_time = 0;
uint32_t last_esvaziar_press_time = 0;
const uint32_t DEBOUNCE_TIME = 400; // 400ms
uint32_t last_freq_update_time = 0;
uint32_t lastBlinkTime = 0;
uint8_t ledState = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void PWM_SetFrequency(uint32_t freq);
void AtualizaLEDs(NivelPressostato nivel);
void PiscarLED(NivelPressostato nivel);

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
  MX_TIM3_Init();

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

   // Inicializa com frequência de "vazio"
   PWM_SetFrequency(FREQ_VAZIO);
   // Desliga todos os LEDs inicialmente
   HAL_GPIO_WritePin(GPIOA, extra_baixo_Pin, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOA, baixo_Pin, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOB, medio_Pin, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOB, alto_Pin, GPIO_PIN_RESET);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   while (1)
     {
         uint32_t current_time = HAL_GetTick();

         // Lógica para o botão "selecionar nível"
         if (HAL_GPIO_ReadPin(GPIOB, bt_nivel_Pin) == GPIO_PIN_RESET &&
             (current_time - last_nivel_press_time) > DEBOUNCE_TIME)
         {
             last_nivel_press_time = current_time;

             if (nivelAtual == NIVEL_NENHUM) {
                 nivelAtual = NIVEL_EXTRA_BAIXO;
             }
             else if (nivelAtual == NIVEL_EXTRA_BAIXO) {
                 nivelAtual = NIVEL_BAIXO;
             }
             else if (nivelAtual == NIVEL_BAIXO) {
                 nivelAtual = NIVEL_MEDIO;
             }
             else if (nivelAtual == NIVEL_MEDIO) {
                 nivelAtual = NIVEL_ALTO;
             }
             else { // se estava em ALTO, volta para EXTRA_BAIXO
                 nivelAtual = NIVEL_EXTRA_BAIXO;
             }


             // Atualiza a frequência alvo conforme nível escolhido
             switch (nivelAtual) {
                 case NIVEL_EXTRA_BAIXO: freqAlvo = FREQ_EXTRA_BAIXO; break;
                 case NIVEL_BAIXO:       freqAlvo = FREQ_BAIXO;       break;
                 case NIVEL_MEDIO:       freqAlvo = FREQ_MEDIO;       break;
                 case NIVEL_ALTO:        freqAlvo = FREQ_ALTO;        break;
                 default:                freqAlvo = FREQ_VAZIO;       break; // fallback
             }

             // Desliga todos os LEDs até atingir a frequência do nível
             HAL_GPIO_WritePin(GPIOA, extra_baixo_Pin, GPIO_PIN_RESET);
             HAL_GPIO_WritePin(GPIOA, baixo_Pin, GPIO_PIN_RESET);
             HAL_GPIO_WritePin(GPIOB, medio_Pin, GPIO_PIN_RESET);
             HAL_GPIO_WritePin(GPIOB, alto_Pin, GPIO_PIN_RESET);
         }

         // Lógica para o botão "encher"
         if (HAL_GPIO_ReadPin(GPIOB, bt_encher_Pin) == GPIO_PIN_RESET && (current_time - last_encher_press_time) > DEBOUNCE_TIME) {
        	 last_encher_press_time  = current_time;
             enchendo = 1;
             esvaziando = 0;
         }

         // Lógica para o botão "esvaziar"
         if (HAL_GPIO_ReadPin(bt_esvaziar_GPIO_Port, bt_esvaziar_Pin) == GPIO_PIN_RESET && (current_time - last_esvaziar_press_time) > DEBOUNCE_TIME) {
             last_esvaziar_press_time = current_time;
             enchendo = 0;
             esvaziando = 1;
         }


             if (enchendo) {
                 if (freqAtual > freqAlvo) {
                     freqAtual -= 100;  // vai diminuindo até o alvo
                     if (freqAtual < freqAlvo) freqAtual = freqAlvo;
                     PWM_SetFrequency(freqAtual);
                     PiscarLED(nivelAtual);
                 } else {
                     enchendo = 0;
                     PWM_SetFrequency(freqAtual);
                     AtualizaLEDs(nivelAtual);
                 }
             } else if (esvaziando) {
                 if (freqAtual < FREQ_VAZIO) {
                     freqAtual += 100;  // vai subindo até "vazio"
                     if (freqAtual > FREQ_VAZIO) freqAtual = FREQ_VAZIO;
                     PWM_SetFrequency(freqAtual);
                     PiscarLED(nivelAtual);
                 } else {
                     esvaziando = 0;
                     nivelAtual = NIVEL_NENHUM;
                     PWM_SetFrequency(FREQ_VAZIO);
                     AtualizaLEDs(nivelAtual);
                 }
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

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1810;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, extra_baixo_Pin|baixo_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, alto_Pin|medio_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : bt_nivel_Pin bt_encher_Pin */
  GPIO_InitStruct.Pin = bt_nivel_Pin|bt_encher_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : extra_baixo_Pin baixo_Pin */
  GPIO_InitStruct.Pin = extra_baixo_Pin|baixo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : bt_esvaziar_Pin */
  GPIO_InitStruct.Pin = bt_esvaziar_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(bt_esvaziar_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : alto_Pin medio_Pin */
  GPIO_InitStruct.Pin = alto_Pin|medio_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void PWM_SetFrequency(uint32_t freq)
{
    uint32_t timer_clock = 48000000; // APB1 Timer clock
    uint32_t prescaler = 0;

    uint32_t arr = (timer_clock / (prescaler + 1)) / freq - 1;

    __HAL_TIM_SET_AUTORELOAD(&htim3, arr); //sobrescreve arr para chegar na frequencia desejada
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, arr/2); // duty 50%
}

void AtualizaLEDs(NivelPressostato nivel)
{
    // Desliga todos os LEDs primeiro
	HAL_GPIO_WritePin(GPIOA, extra_baixo_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, baixo_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, medio_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, alto_Pin, GPIO_PIN_RESET);

    // Acende apenas o LED correspondente ao nível atingido
    switch (nivel) {
        case NIVEL_EXTRA_BAIXO:
            HAL_GPIO_WritePin(GPIOA, extra_baixo_Pin, GPIO_PIN_SET);
            break;
        case NIVEL_BAIXO:
            HAL_GPIO_WritePin(GPIOA, baixo_Pin, GPIO_PIN_SET);
            break;
        case NIVEL_MEDIO:
            HAL_GPIO_WritePin(GPIOB, medio_Pin, GPIO_PIN_SET);
            break;
        case NIVEL_ALTO:
            HAL_GPIO_WritePin(GPIOB, alto_Pin, GPIO_PIN_SET);
            break;
        default:
            // Nenhum LED para NIVEL_NENHUM
            break;
    }
}

void PiscarLED(NivelPressostato nivel)
{
    uint32_t current_time = HAL_GetTick();

    // Alterna a cada 500 ms
    if (current_time - lastBlinkTime >= 500) {
        lastBlinkTime = current_time;
        ledState = !ledState;
    }

    // Desliga todos os LEDs
    HAL_GPIO_WritePin(GPIOA, extra_baixo_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, baixo_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, medio_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, alto_Pin, GPIO_PIN_RESET);

    if (ledState) {
        AtualizaLEDs(nivel); // Acende apenas o nível selecionado
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
