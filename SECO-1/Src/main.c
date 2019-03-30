/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static int n_pulsos = 0;
static float n_pulsos_array[1200];
static float n_pulsos_media[1200];
static uint16_t cuenta_1ms = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  media(4, 10); // Llamamos a la funcion que realiza la media, con 4V, 10 repeticiones
  //mueve_motor(4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
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
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1199;
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
  sConfigOC.Pulse = 600;
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
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1199;
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
  sConfigOC.Pulse = 600;
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
  htim6.Init.Prescaler = 15;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 3999;
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
  huart2.Init.BaudRate = 38400;
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : Encoder_A_IT_Pin Encoder_B_IT_Pin */
  GPIO_InitStruct.Pin = Encoder_A_IT_Pin|Encoder_B_IT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
// Llama n_veces a la funcion mueve_motor con un voltaje determinado, realiza la media y las transmite por el puerto serie
void media (float voltaje, uint8_t n_veces) {
	for (uint8_t i=0; i< n_veces; i++){
		mueve_motor(voltaje);
		HAL_Delay(5000);
		for (uint16_t j=0; j<1200; j++){
			n_pulsos_media[j] = (n_pulsos_media[j]*i + n_pulsos_array[j]) / (i+1);
		}
	}
	tx_pulsos();
}

// Transmite por UART las medias de las posiciones guardadas
void tx_pulsos (void) {

	for (int i = 0; i <1200; i++ ) {
		tx_UART_int(&huart2, i, 10);
		uint8_t espacio[] = "    ";
		HAL_UART_Transmit(&huart2, espacio, sizeof(espacio), 10);
		tx_UART_float(&huart2, n_pulsos_media[i], 10);
	}
}

// Guarda las posiciones en un array cada 1 ms
void timer_1ms(void) {

	if (cuenta_1ms == 1200) {	// Cuando pasa el tiempo se detiene el timer
		HAL_TIM_Base_Stop_IT(&htim6);
		n_pulsos = 0;	// Resetamos los pulsos y el contador para posibles test consecutivos
		cuenta_1ms = 0;
		return;
	}

	if (cuenta_1ms == 600) {	// Pasado este tiempo se detiene el motor
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1); // Paramos las PWM
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	}

	n_pulsos_array[cuenta_1ms] = n_pulsos;	// Guardamos los pulsos detectados en el array
	cuenta_1ms++;	// Incrementamos el contador cada 1 ms
	}

void cuenta_pulsos(void) {	// Cuenta cada pulso recibido por los canales A y B del encoder

	static uint8_t estado_encoder = estado_00;  // Estado actual del encoder
	static uint8_t estado_anterior = estado_00; // Estado anterior del encoder

	estado_anterior = estado_encoder;	// Actualizamos el valor anterior del encoder

	// Asignamos el estado actual de encoder
	if (!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)) && !(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6))) {
		estado_encoder = estado_00;
	}

	if (!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)) && (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6))) {
		estado_encoder = estado_01;
	}

	if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)) && !(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6))) {
		estado_encoder = estado_10;
	}

	if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)) && (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6))) {
		estado_encoder = estado_11;
	}

	// Comparamos el valor actual del encoder con el anterior para conocer el sentido de giro
	switch (estado_anterior) {
		case estado_00:	if (estado_encoder == estado_01) { n_pulsos = n_pulsos + 1;}
						//if (estado_encoder == estado_10) { n_pulsos = n_pulsos - 1;}
						else { n_pulsos = n_pulsos - 1;}
						break;
		case estado_01:	if (estado_encoder == estado_11) { n_pulsos = n_pulsos + 1;}
						//if (estado_encoder == estado_00) { n_pulsos = n_pulsos - 1;}
						else { n_pulsos = n_pulsos - 1;}
						break;
		case estado_11:	if (estado_encoder == estado_10) { n_pulsos = n_pulsos + 1;}
						//if (estado_encoder == estado_01) { n_pulsos = n_pulsos - 1;}
						else { n_pulsos = n_pulsos - 1;}
						break;
		case estado_10:	if (estado_encoder == estado_00) { n_pulsos = n_pulsos + 1;}
						//if (estado_encoder == estado_11) { n_pulsos = n_pulsos - 1;}
						else { n_pulsos = n_pulsos - 1;}
						break;
	}
}

// Asigna un ciclo de trabajo correspondiente a un voltaje para mover el motor
void mueve_motor(float voltaje)
{
	uint8_t sentido;
		if (voltaje > 0) {
			sentido = HORARIO;
		}
		else {
			sentido = ANTIHORARIO;
			voltaje = -voltaje;
		}

		int duty_cycle = (int)(voltaje*100)-1;	// Pasamos de voltios a centesimas de voltio, que equivale al duty_cycle
	if (sentido == HORARIO) {

		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, htim2.Init.Period - duty_cycle);
		//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty_cycle);// Asignamos el duty cycle
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);	// Arrancamos el PWM 2
		HAL_TIM_Base_Start_IT(&htim6);				// Arrancamos el timer de 1 ms
	}
	if (sentido == ANTIHORARIO) {

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, htim3.Init.Period - duty_cycle); // Asignamos el duty cycle
		//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty_cycle);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);	// Arrancamos el PWM 3
		HAL_TIM_Base_Start_IT(&htim6);				// Arrancamos el timer de 1 ms
	}
}

// Funcion de depuración para transmitir un float por el puerto serie
void tx_UART_float(UART_HandleTypeDef *huart, float data, uint32_t Timeout) {
	int p_entera = data/1;
	int p_decimal = ((data - p_entera)*100)/1;

	tx_UART_int(huart, p_entera, 10);

	uint8_t punto[] = ".";
	HAL_UART_Transmit(huart, punto, 1, 10);

	if (p_decimal < 10) {
		uint8_t cero[] = "0";
		HAL_UART_Transmit(&huart2, cero, 1, 10);
	}

	tx_UART_int(huart, p_decimal, 10);

	uint8_t salto[] = "\r\n";
	HAL_UART_Transmit(huart, salto, 2, 10);

}

// Funcion de depuración para transmitir un int por el puerto serie
void tx_UART_int(UART_HandleTypeDef *huart, int data, uint32_t Timeout)
{

	int size = 1;
	uint8_t negativo = 0;

	if (data < 0) {	// Si los pulsos
		data = -data;
		negativo = 1;
	}

	int numero = data;

	while(numero > 9) {
	  numero =  numero/10;
	  size++;
	}

	char data_char[size];		// String de chars
	uint8_t data_tx[size];	// String de uint8_t

	sprintf(data_char,"%d", data);	// Cada numero del int en un char

	for(uint8_t i=0; i<size; i++ ) {			// Casting de char a uint8_t
		data_tx[i] = (uint8_t) data_char[i];
	}

	if (negativo) {		// Si el numero es negativo, transmite un "-" antes
		uint8_t menos[] = "-";
		HAL_UART_Transmit(huart, menos, 1, 10);
	}
	HAL_UART_Transmit(huart,data_tx,sizeof(data_tx), 10);	// TX por UART del array de uint8_t
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
