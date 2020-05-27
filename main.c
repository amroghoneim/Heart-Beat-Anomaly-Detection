/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_x-cube-ai.h"
#include "stdio.h"
#include <cmath>

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

CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN 0 */
	
float test;
int count;
float min=999999;
float max= -1;
char reading[10];
uint16_t adcval;
char newline[2] = "\r\n";
float in_data[187];
float norm_in_data[187] = {0.823899,0.474843,0.163522,0.685535,0.204403,0.789308,0.374214,0.128931,0.650943,0.185535,0.820755,0.336478,0.125786,0.641509,0.188679,0.886792,0.336478,0.201258,0.578616,0.172956,0.823899,0.289308,0.295597,0.569182,0.166667,0.833333,0.267296,0.399371,0.534591,0.166667,0.886792,0.267296,0.506289,0.503145,0.160377,0.830189,0.238994,0.556604,0.490566,0.144654,0.817610,0.220126,0.588050,0.468553,0.150943,0.767296,0.226415,0.654088,0.418239,0.147799,0.738994,0.207547,0.764151,0.408805,0.128931,0.751572,0.213836,0.902516,0.421384,0.157233,0.679245,0.204403,0.823899,0.327044,0.141509,0.613208,0.188679,0.981132,0.355346,0.261006,0.650943,0.201258,0.921384,0.327044,0.273585,0.072327,0.000000,0.669811,0.210692,0.443396,0.562893,0.185535,0.902516,0.235849,0.531447,0.509434,0.166667,0.858491,0.232704,0.550314,0.465409,0.154088,0.783019,0.235849,0.616352,0.415094,0.138365,0.852201,0.235849,0.742138,0.411950,0.135220,0.808176,0.207547,0.823899,0.396226,0.110063,0.644654,0.176101,0.861635,0.355346,0.116352,0.657233,0.182390,0.921384,0.349057,0.179245,0.606918,0.185535,0.911950,0.323899,0.283019,0.550314,0.172956,0.886792,0.298742,0.405660,0.569182,0.172956,0.874214,0.270440,0.474843,0.503145,0.157233,0.877358,0.273585,0.550314,0.525157,0.157233,0.943396,0.286164,0.594340,0.468553,0.150943,0.918239,0.276730,0.638365,0.427673,0.135220,0.764151,0.220126,0.748428,0.396226,0.144654,0.701258,0.210692,0.827044,0.371069,0.125786,0.641509,0.179245,0.899371,0.349057,0.141509,0.644654,0.194969,0.874214,0.314465,0.232704,0.694969,0.201258,1.000000,0.352201,0.320755,0.556604,0.176101,0.883648,0.289308,0.415094,0.493711,0.154088,0.814465,0.251572,0.471698,0.433962,0.141509,0.833333};
float norm_in_data_f;
	char normalized[10];

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
	void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  /* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADC_ConvCpltCallback must be implemented in the user file.
   */
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	adcval = HAL_ADC_GetValue(&hadc1); //getting the ADC value
	adcval= ((float)adcval/4096.0) * 2048.0; //change to 11 bits
	if(count<187){  //only add the first 187 samples to the buffer in_data
				in_data[count] = adcval; 
				if(in_data[count]<min)  //get the minimum value
					min=in_data[count];
				if(in_data[count]>max)  //get the maximum value
					max=in_data[count];
				count++;
			}
//	sprintf(reading, "%hu\r\n", adcval); // copy the adc value to a reading buffer to display the readings if needed (python)
//  HAL_UART_Transmit(&huart2, (uint8_t*)reading, sizeof(reading)-4, HAL_MAX_DELAY); //transmit reading (python)
	//HAL_UART_Transmit(&huart2, (uint8_t*)newline, sizeof(newline), HAL_MAX_DELAY); //newline
}
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
  MX_ADC1_Init();
  MX_CRC_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_X_CUBE_AI_Init();
   /* USER CODE BEGIN 2 */
	
	//float test[1];
	char inference[10];
	char normal[17]={'N','o','r','m','a','l',',','g','o','o','d',' ','j','o','b','\r','\n'};
	char abnormal[10]={'A','b','n','o','r','m','a','l','\r','\n'};
	//static float in_data[187] = {1.00E+00,7.64E-01,4.64E-01,1.81E-01,1.18E-01,1.52E-01,1.25E-01,1.01E-01,1.02E-01,8.82E-02,7.82E-02,7.82E-02,7.25E-02,7.25E-02,8.25E-02,9.53E-02,1.14E-01,1.25E-01,1.42E-01,1.71E-01,2.01E-01,2.29E-01,2.59E-01,2.96E-01,3.40E-01,3.67E-01,3.81E-01,4.07E-01,4.31E-01,4.34E-01,4.40E-01,4.31E-01,4.28E-01,4.17E-01,3.95E-01,3.83E-01,3.70E-01,3.34E-01,3.20E-01,3.00E-01,2.65E-01,2.56E-01,2.46E-01,2.45E-01,2.43E-01,2.32E-01,2.59E-01,2.97E-01,3.24E-01,3.49E-01,3.39E-01,3.06E-01,2.63E-01,2.30E-01,2.28E-01,2.25E-01,2.23E-01,2.30E-01,2.23E-01,2.26E-01,2.35E-01,2.18E-01,2.46E-01,4.14E-01,6.46E-01,8.99E-01,7.47E-01,3.19E-01,3.13E-02,0.00E+00,1.11E-01,1.48E-01,1.27E-01,1.14E-01,1.04E-01,1.08E-01,1.14E-01,1.05E-01,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00,0.00E+00};  
	
	HAL_TIM_Base_Start(&htim2);
	HAL_ADC_Start_IT(&hadc1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(count>=187){  //if Counter exceeded 187 samples
			HAL_ADC_Stop_IT(&hadc1); //stop ADC interrupts so no more values are read
				for (int j=0; j<187;j++){
					norm_in_data[j]= (in_data[j]-min)/(max-min); //normailize the signals in in_data buffer
					//norm_in_data_f= (in_data[j]-min)/(max-min); //used for normalized transmission
				}
				test = MX_X_CUBE_AI_Process(norm_in_data); //use the AI_Process to get the prediction
				if(!isnan(test)){
					sprintf(inference, "%f\r\n", test); //copy the probability to inference array 
				//HAL_UART_Transmit(&huart2, (uint8_t*)inference, sizeof(inference), HAL_MAX_DELAY);
				if((inference[2]=='9') || (inference[0]=='1')) //check if the probability is >=0.9
					HAL_UART_Transmit(&huart2, (uint8_t*)normal, sizeof(normal), HAL_MAX_DELAY); //trannsmit normal
				else
					HAL_UART_Transmit(&huart2, (uint8_t*)abnormal, sizeof(abnormal), HAL_MAX_DELAY); //transmit abnormal
			}
//				sprintf(reading, "%f\r\n", norm_in_data_f); //copy normalized values to reading (python)
//				HAL_UART_Transmit(&huart2, (uint8_t*)reading, sizeof(reading), HAL_MAX_DELAY); //Transmit normalized reading (python)
				count =0; //reset counter
				min=999999; //reset minimum
				max=-1; //reset maximum
				HAL_ADC_Start_IT(&hadc1); //start ADC interrupt again.
			
			}	
				HAL_Delay(1);
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B; //bits/////////////////////
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
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
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  htim2.Init.Prescaler = 7; //8000  7999
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 7999; //7
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
