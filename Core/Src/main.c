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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
	int position,E,speed,speedr,speedl;
	char sensor_read[7];
	
	const float Kp = 0.00035; //set up the constants value
	const float Ki = 0.00015;
	const float Kd = 10;
	int32_t P = 0,I = 0,D = 0;
	int32_t error;
	int lastError = 0;
	int errors[10] = {0,0,0,0,0,0,0,0,0,0};
	int error_sum = 0;
	int last_end = 0;	// 0 -> Left, 1 -> Right 
	int last_endside = 0;	// 0 -> Left, 1 -> Right 
	int last_idle = 0;
	
	const uint8_t maxspeedr = 50;
	const uint8_t maxspeedl = 50;
	const uint8_t basespeedr = 45;
	const uint8_t basespeedl = 45;
	const uint8_t basespeed = 20;
	const int ARR = 10;
	
	int actives = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
//void delay_us(uint16_t us);
//void sharp_turn();
//int Sensor_Read();
//void motor_control(double pos_right, double pos_left);
//void forward_brake(int pos_right,int pos_left);
//void past_errors(int error);
//int errors_sum (int index,int abs); 
//void PID_control(); 
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim3,0);
	while(__HAL_TIM_GET_COUNTER(&htim3)<us);
}

void motor_control(double pos_right, double pos_left) 
{
//	__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_1, ARR*pos_right);
//	__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_2, 0);
//	__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_3, ARR*pos_left);
//	__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_4, 0);
	if(pos_right < 0)
	{
		__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_2, ARR*(-pos_right));
	} 
	else 
	{
		__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_1, ARR*pos_right);
		__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_2, 0);
	}
	if(pos_left < 0)
	{
		__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_4, ARR*(-pos_left));
	} 
	else 
	{
		__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_3, ARR*pos_left);
		__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_4, 0);
	}
}

void sharp_turn() 
{
	if(last_idle > 25)
	{
//		if(last_end == 1) motor_control(-35, 50);
//		else motor_control(50, -35);
		if(last_endside == 1) motor_control(-30, 50);
		else motor_control(50, -30);
	}
	else 
	{
//		if(last_end == 1) motor_control(-20, 50);
//		else motor_control(50, -20);
		if(last_endside == 1) motor_control(-20, 40);
		else motor_control(40, -20);
	}
}


int32_t Sensor_Read() 
{	
	// Threshold
	delay_us(5000);
	
	int32_t pos = 0;
  int active = 0;
	
	if (HAL_GPIO_ReadPin(sensor1_GPIO_Port,sensor1_Pin)==1) 
	{
		pos += 10000;
    active++;
		last_endside = 1;
  }
	
	if (HAL_GPIO_ReadPin(sensor2_GPIO_Port,sensor2_Pin)==0) 
	{
		pos += 20000;
    active++;
		last_end = 1; // 0 -> Left, 1 -> Right
	}
	if (HAL_GPIO_ReadPin(sensor3_GPIO_Port,sensor3_Pin)==0) 
	{
		pos += 30000;
    active++;
  }
	if (HAL_GPIO_ReadPin(sensor4_GPIO_Port,sensor4_Pin)==0) 
	{
		pos += 40000;
    active++;
  }
	if (HAL_GPIO_ReadPin(sensor5_GPIO_Port,sensor5_Pin)==0) 
	{
		pos += 50000;
    active++;
  }
	if (HAL_GPIO_ReadPin(sensor6_GPIO_Port,sensor6_Pin)==0) 
	{
		pos +=60000;
    active++;
		last_end = 0; // 0 -> Left, 1 -> Right
  }
	if (HAL_GPIO_ReadPin(sensor7_GPIO_Port,sensor7_Pin)==1) 
	{
		pos += 70000;
    active++;
		last_endside = 0;
  }

	position = pos/active;
  actives = active;
	
	if (actives == 0) last_idle++;
	else last_idle = 0;

	return pos/active;
}

void forward_brake(int pos_right,int pos_left) 
{
	if(actives == 0) sharp_turn();
	else motor_control(pos_right,pos_left);
}

void past_errors(int error) 
{
  for(int i=9;i>0;i--) errors[i]=errors[i-1];
  errors[0]=error;
}

int errors_sum (int index,int abs) 
{
  int sum = 0;
  for(int i=0;i<index;i++) 
  {
    if(abs==1&errors[i]<0) sum+=-errors[i]; 
    else sum+=errors[i];
  }
  return sum;
}

void PID_control() 
{
	int32_t position = Sensor_Read();	
  error = 40000 - position;
	past_errors(error);

  P = error;
  I = errors_sum(5, 0);
//	I = I + error;
  D = error - lastError;
	
  int motorspeed = P*Kp + I*Ki + D*Kd;
//  int motorspeed = (error*Kp) + (I + error)*Ki + (error - lastError)*Kd;
	
	lastError = error;
	
  int motorspeedl = basespeedl + motorspeed;
  int motorspeedr = basespeedr - motorspeed;
	
//	if(error == 20000){motorspeedr = -10; motorspeedl = 50;}
//	if(error == -20000){motorspeedr = 50; motorspeedl = -10;}
	
  if (motorspeedl > maxspeedl) motorspeedl = maxspeedl;
  if (motorspeedr > maxspeedr) motorspeedr = maxspeedr;
	
//	if(error==0) motorspeedr=motorspeedl=basespeed;
	E = error; speed = motorspeed; speedr = motorspeedr; speedl = motorspeedl; // debug
	forward_brake(motorspeedr, motorspeedl);
//	motor_control(motorspeedr,motorspeedl);
}

void debug_sensor()
{
	int read;
	for(char i=0;i<7;i++)
	{
		read = HAL_GPIO_ReadPin(GPIOA,(uint16_t)0x0002<<i);
		sensor_read[i]=read;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	_Bool ok = 0;
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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
	HAL_Delay(2000);
//	while(ok == 0)
//	{
//		if(HAL_GPIO_ReadPin(start_GPIO_Port,start_Pin) == 0) ok = 1;
//	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		PID_control(); 
//		Sensor_Read();
		debug_sensor();
//		if(HAL_GPIO_ReadPin(sensor1_GPIO_Port,sensor1_Pin)==1)
//		{ 
//			motor_control(-45, 45);
//			HAL_Delay(200);
//			motor_control(0,0);
//			HAL_Delay(50);
//		}
//		if(HAL_GPIO_ReadPin(sensor7_GPIO_Port,sensor7_Pin)==1)
//		{
//			motor_control(45, -45);
//			HAL_Delay(200);
//			motor_control(0,0);
//			HAL_Delay(50);
//		}
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 80-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_status_GPIO_Port, led_status_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : led_status_Pin */
  GPIO_InitStruct.Pin = led_status_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_status_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : start_Pin */
  GPIO_InitStruct.Pin = start_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(start_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : sensor1_Pin sensor2_Pin sensor3_Pin sensor4_Pin
                           sensor5_Pin sensor6_Pin sensor7_Pin */
  GPIO_InitStruct.Pin = sensor1_Pin|sensor2_Pin|sensor3_Pin|sensor4_Pin
                          |sensor5_Pin|sensor6_Pin|sensor7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

