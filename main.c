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
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

//LIBRARY EKSTERNAL
#include "i2c-lcd.h"
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
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
// sensitivity = VACPeak / (ADCMax-ADCOffset)
float sensitivity[3] = {0.561879945777734205277419028052, 0.61556557733122668566801292477, 0.57695749099496997549138416828876};

uint16_t coba= 0;
double y_ink_test=0;

volatile uint16_t tfFlag = 0xFFFF;
volatile uint16_t adcBuff[3] = {0};
volatile uint16_t adcCount = 0;
volatile uint16_t adcVal[3][500] = {0};
uint8_t volt_unbalance = 0;

uint16_t maxValue[3];
double adcOffset[3] ={0};
double Vrms[3]={0};
double nilaiANN[3]={0};
double VNorm =0;

uint32_t tim12count = 0;
uint16_t timVal = 0;
uint16_t lastSMS = 0;

//GPS
uint8_t gpsBuff[100]={};
uint8_t flagGPS = 0;
char currentLat[12], currentLong[12];

//MENYIMPAN BUFFER
uint8_t tmpBuff [200]={0}; 
uint8_t printLen=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM12_Init(void);
/* USER CODE BEGIN PFP */
void printLCD(void);
void getVRMS(void);
double ANN(double voltageNorm);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
	//LCD
	HAL_Delay(50);
	lcd_init();
	HAL_Delay(50);
	
	//GPS
	//OUTPUT GPS = $GPRMC,132406.00,A,0717.15101,S,11248.02735,E,1.069,,071122,,,A*6E
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, gpsBuff, 100);
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	
//	printLen = snprintf((char*)tmpBuff, 100, "AT+CMGS=\"+6281362313006\"\r\nhttps://maps.google.com/?q=%s,%s", currentLat, currentLong);
//	HAL_UART_Transmit(&huart6, tmpBuff, printLen, HAL_MAX_DELAY);
//	printLen = snprintf((char*)tmpBuff, 100, "\nVR-N: %f\nVS-N: %f\nVT-N: %f%c", Vrms[0], Vrms[1], Vrms[2], 26);
//	HAL_UART_Transmit(&huart6, tmpBuff, printLen, HAL_MAX_DELAY);
//	HAL_UART_Receive(&huart6, tmpBuff, 20, 60000);
	
		//Timer
	HAL_TIM_Base_Start_IT(&htim3); //BERDETIK SETIAP 200us/0.2ms (ADC DALAM 20ms = 20/0.2 = 100)
	HAL_TIM_Base_Start(&htim12);//BERDETIK SETIAP 0.5ms
	timVal = __HAL_TIM_GetCounter(&htim12);
	
	//MULAI ADC DENGAN DMA
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuff, sizeof(adcBuff)/sizeof(adcBuff[0]));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	//PANGGIL SETIAP 0.5ms x 40 = 20ms
		if(__HAL_TIM_GetCounter(&htim12) - timVal >= 40){
			//STOP MEMASUKKAN ADC BARU
			tfFlag = 0x0000;
			
			++tim12count;
			HAL_Delay(200);
			
			//PEMROSESAN
			getVRMS();
			
			//ANN
			for(int i=0; i<3; i++){
				VNorm = (Vrms[i] - 0.2568) / 249.7209;
				nilaiANN[i] = ANN(VNorm);
			}
			
			printLCD();
			
			//TERJADI UNBALANCE
			if(volt_unbalance == 1){
					//WAKTU SEKARANG LEBIH (0.5ms x 60000 = 30000ms) DARI SMS SEBELUMNYA
				if(__HAL_TIM_GetCounter(&htim12) - lastSMS >= 60000){
					printLen = snprintf((char*)tmpBuff, 100, "AT+CMGS=\"+6281362313006\"\r\nhttps://maps.google.com/?q=%s,%s", currentLat, currentLong);
					HAL_UART_Transmit(&huart6, tmpBuff, printLen, HAL_MAX_DELAY);
					printLen = snprintf((char*)tmpBuff, 100, "\nVR-N: %f\nVS-N: %f\nVT-N: %f%c", Vrms[0], Vrms[1], Vrms[2], 26);
					HAL_UART_Transmit(&huart6, tmpBuff, printLen, HAL_MAX_DELAY);
					HAL_UART_Receive(&huart6, tmpBuff, 20, 60000);
					
					volt_unbalance = 0;
					lastSMS = __HAL_TIM_GetCounter(&htim12);
				}
			}
			
			//AMBIL DATA
			printLen = snprintf((char*)tmpBuff, 200, ",%u,%f,%f,%f,%d,%d,%d,%f,%f,%f,%f,%f,%f\r\n", tim12count, adcOffset[0], adcOffset[1], adcOffset[2], maxValue[0],  maxValue[1], maxValue[2], Vrms[0], Vrms[1], Vrms[2], nilaiANN[0], nilaiANN[1], nilaiANN[2]);
			HAL_UART_Transmit(&huart3, tmpBuff, printLen, HAL_MAX_DELAY);
			
			//RESTART INDEKS ADC, UPDATE WAKTU PEMBACAAN, MULAI MEMASUKKAN ADC
			adcCount = 0;
			timVal = __HAL_TIM_GetCounter(&htim12);
			tfFlag = 0xFFFF;
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 200-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 42000-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	//MULAI DMA KETIKA COUNTER TIM3 PENUH
  if (htim->Instance==TIM3)	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuff, sizeof(adcBuff)/sizeof(adcBuff[0]));
}
void printLCD(void){
	lcd_clear();
	lcd_send_cmd(0x80|0x00);//  lcd_setCursor(0,0);
	lcd_send_string("Nilai Tegangan");
	
	lcd_send_cmd(0x80|0x40);//  lcd_setCursor(0,1);
	printLen = snprintf((char*)tmpBuff, 50, "VR-N: %f", Vrms[0]);
	lcd_send_string((char*)tmpBuff);
	
	lcd_send_cmd(0x80|0x14);//	lcd_setCursor(0,2);
	printLen = snprintf((char*)tmpBuff, 50, "VS-N: %f", Vrms[1]);
  lcd_send_string((char*)tmpBuff);
	
		lcd_send_cmd(0x80|0x54);//	lcd_setCursor(0,3);
	printLen = snprintf((char*)tmpBuff, 50, "VT-N: %f", Vrms[2]);
  lcd_send_string((char*)tmpBuff);
}

void getVRMS(void){
//RESET NILAI MAX ADC
	for(int i =0; i<3; i++) maxValue[i] = 0;
	
	//RESET NILAI VRMS
	for(int i =0; i<3; i++) Vrms[i] = 0;
	
	//JUMLAHKAN ADC TIAP CHANNEL
	for(int i=0; i<adcCount; i++){
		for(int j=0; j<3; j++){
			//JUMLAH ADC TIAP CHANNEL
			adcOffset[j] += adcVal[j][i];
			
			//NILAI MAX
			if(adcVal[j][i] > maxValue[j]) maxValue[j] = adcVal[j][i];
		}
	}
	
	//RATA-RATA (OFFSET) ADC TIAP CHANNEL
	for(int i=0; i<3; i++) adcOffset[i] = adcOffset[i] / adcCount;
	
	//SQUARE SUM
	for(int i=0; i<adcCount; i++){
		for(int j=0; j<3; j++){
			Vrms[j] += pow(((double)(adcVal[j][i] - adcOffset[j]) * sensitivity[j]), 2);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
		}
	}
	
	//RATA-RATA SQUARE AVERAGE ADC TIAP CHANNEL
	for(int i=0; i<3; i++) Vrms[i] = sqrt(Vrms[i] / (double)adcCount);
	
	//TEMUKAN VRMS
//	for(int i=0; i<3; i++){
//		Vrms[i] = (maxValue[i] - adcOffset[i]) * sensitivity[i] / sqrt(2);
////		if(Vrms[i]<5)Vrms[i] = 0;
//	}
}

double ANN(double voltageNorm){
		double v_0j[16] = {-65.0433360137739, -0.925757485996498, 59.9720674460496, 48.0057784148022, -0.549024666656972, 1.33181512179937, -0.0678168217561989, -0.0127697020859943,
										 2.19535701184534, -2.07530301800864, -39.0229445178471, 4.04058949958256, 4.02614381383835, -1.94464723538935, -10.6716403725990, 1.49138708046781};
		double v_ij[16] = {69.6187747367864, 11.1760612459524, -64.4402240842946, -58.9438559423896, 11.1772456316080, 11.2027708766319, -11.2043542405409, -11.1964274004930,
										 11.2046723708285, 11.1765184611173, 48.3953319154966, 11.1999092002035, 11.2010875981166, 11.1231663100475, -11.1999996933773, -11.1019059150399};
		double w_0k = -2.92972035175016;
		double w_jk[16] = {-21.0924675450650, -1.99777612211494, 17.8700501592290, -21.8429685041816, -2.04153717964564, -2.79331688681383, -0.103010088794269, -0.261487572011259, -3.26413823077646, -1.80812905682893, 18.6112003331674, -2.64957342901986, -3.52996103316309, -1.50880960426936, -0.175060090284714, -1.16750521952207};
    
		double z_inj_test[16];
     y_ink_test = 0;
    double predict = 0;

    //INPUT LAYER KE HIDDEN LAYER
    for (int i = 0; i < 16; i++) z_inj_test[i] = 1 / (1 + exp(-(voltageNorm * v_ij[i] + v_0j[i])));

    //HIDDEN LATER KE OUTPUT LAYER
    for (int i = 0; i < 16; i++) y_ink_test += z_inj_test[i] * w_jk[i];
    y_ink_test += w_0k;

    predict = 1 / (1 + exp(-(y_ink_test)));
    return predict;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	//MASUKKAN ADC JIKA tfFlag BERNILAI 0XFFFF
	if(tfFlag==0xFFFF){
		adcVal[0][adcCount] = adcBuff[0];
		adcVal[1][adcCount] = adcBuff[1];
		adcVal[2][adcCount] = adcBuff[2];
		
		//PENGAMAN JIKA JUMLAH PEMBACAAN ADC MELEBIHI 503X
		if(++adcCount > 500)adcCount = 0;
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
  //USART2 = GPS
	if(huart->Instance == USART2){
		char latDD[3];
		char latMM[9];
		char longDD[4];
		char longMM[9];
		char ew[1], ns[1];
		int jGPS=0, iGPS = 0;
		char latitude[12], longitude[12];
		float finalLat=0, finalLong=0;
		
		//CONTOH OUTPUT GPRMC GPS
//		char str[100] = "$GPRMC,132406.00,A,0717.15101,S,11248.02735,E,1.069,,071122,,,A*6E";

//BACA TIAP KARAKTERNYA
		while(iGPS<Size){
		//JIKA KARAKTER BERNILAI KOMA
			if(gpsBuff[iGPS] == ','){
				//SETELAH KOMA PERTAMA MERUPAKAN VALIDITAS (V=invalid, A=valid)
				if(jGPS==1){
					if(gpsBuff[++iGPS] == 'V')return;
				}
				//SETELAH KOMA KEDUA MERUPAKAN LATITUDE
				else if(jGPS==2){
					int k=0;iGPS++;
					do{
						latitude[k] = gpsBuff[iGPS];
						iGPS++;k++;
					}while(gpsBuff[iGPS] != ',');
					jGPS++;
					continue;
				}
				//SETELAH KOMA KETIGA MERUPAKAN North/South
				else if(jGPS==3){
					int k=0;iGPS++;
					do{
						ns[k] = gpsBuff[iGPS];
						iGPS++;k++;
					}while(gpsBuff[iGPS] != ',');
					jGPS++;
					continue;
				}
				//SETELAH KOMA KEEMPAT MERUPAKAN LONGITUDE
				else if(jGPS==4){
					int k=0;iGPS++;
					do{
						longitude[k] = gpsBuff[iGPS];
						iGPS++;k++;
					}while(gpsBuff[iGPS] != ',');
					jGPS++;
					continue;
				}
				//SETELAH KOMA KELIMA MERUPAKAN East/West
				else if(jGPS==5){
					int k=0;iGPS++;
					do{
						ew[k] = gpsBuff[iGPS];
						iGPS++;k++;
					}while(gpsBuff[iGPS] != ',');
					jGPS++;
					continue;
				}
				
				//TAMBAH INDEKS KOMA
				jGPS++;
			}
			//TAMBAH INDEKS KARAKTER
			iGPS++;
		}
		
		//KONVERSI LATITUDE LONGITUDE KE DERAJAT
		strncpy(latDD, latitude, 2);
		strncpy(latMM, latitude+2, strlen(latitude)-2);
		strncpy(longDD, longitude, 3);
		strncpy(longMM, longitude+3, strlen(longitude)-3);
		finalLat = atof(latDD) + (atof(latMM) / 60);
		finalLong = atoi(longDD) + (atof(longMM) / 60);
		
		if(ns[0] == 'S') sprintf(currentLat, "-%f", finalLat);
		else sprintf(currentLat, "%f", finalLat);
		if(ew[0] == 'W') sprintf(currentLong, "-%f",finalLong);
		else sprintf(currentLong, "%f",finalLong);
		
		//SIAP MENERIMA GPS SELANJUTNYA
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, gpsBuff, 100);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
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
