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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l475e_iot01_qspi.h"
#define ARM_MATH_CM4
#include "arm_math.h"
#include "OPTIM.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ITM_Port32(n) (*((volatile unsigned long *)(0xE0000000+4*n)))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

QSPI_HandleTypeDef hqspi;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

// ---------------- IMPORTANT GLOBAL VARIABLES ------------------------------------------
uint16_t f_s_DAC = 44100; // HZ, sampling frequency for values TO the DAC SPEAKER
uint16_t f_s_mic = 10000; // Hz, sampling frequency for values FROM the DFSDM microphone

// states for game
uint8_t gameState = 0;			// 0: game init & generating tones, 1: playing tones for player/ waiting for player choice,
								// 2: recording from mic, 3: ready for DFT, 4: computing DFT and analysis, 5: report win/loss

#define numTones 8
#define micSamples 10000
float32_t frequencies[8] = {261.63, 293.66, 329.63, 349.23, 392.00, 440.00, 493.88, 523.25}; // C4, D4, E4, F4, G4, A4, B4, C5

uint32_t flashStartAddr = (uint32_t) 0x00000000;
uint16_t sineBuffer[22050]; // buffer in RAM to store 0.5s of wave values at a time during wave generation and to load DAC
uint32_t currentTone = 0;
uint32_t count1 = 0; //since notes are being played at 0.5s, when reading the notes, only change note after x amount counts, e.g change when count == 4, is 2 sec
uint32_t count2 = 0;
int32_t micBuffer[micSamples];
uint16_t counterLED = 0;
uint16_t counterLEDMax = 8820;	// for flashing every 0.2s with TIM2 which is at f=44.1kHz

// for DFT
uint32_t fft_buffer_size = 2048;
uint32_t fft_sampling_rate = 10000;
int frequency = 0;
struct str{
	float value;
	uint16_t index;
};
float allowedError = 0.2;


// other setup notes:
// BUTTON : blue push button
// LED_GREEN: active HIGH, so GPIO_SET turns it ON
// LED_RED: active LOW, so GPIO_RESET turns it ON
// DAC Speaker: f_s = 44.1kHz, circular DMA with half-word (takes 16-bit array inputs with 12 bit resolution). Use this to output tones for the user to choose from
// DFSDM mic: f_s = 10kHz, NORMAL DMA with word input (captures 32 bits of input from mic. We need to RSR by 20bits to scale to 12 bits then pass to FFT?)


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_DFSDM1_Init(void);
/* USER CODE BEGIN PFP */
void readFirstHalfFromFlash(void);
void readSecondHalfFromFlash(void);
void showPlayerWon(void);
void showPlayerLost(void);
void get_frequency(void);
int cmp(const void *a, const void *b);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int cmp(const void *a, const void *b)
{
    struct str *a1 = (struct str *)a;
    struct str *a2 = (struct str *)b;
    if ((*a1).value > (*a2).value)
        return -1;
    else if ((*a1).value < (*a2).value)
        return 1;
    else
        return 0;
}

void get_frequency(void){
	uint32_t fft_sampling_rate = 2048;

	float mic_buffer_float[fft_buffer_size];
	for (uint16_t i = 0; i < fft_buffer_size; i++) {
		mic_buffer_float[i] = (micBuffer[i+fft_buffer_size]);
	}

	int frequency1 = 500;
	int amplitude1 = 100;
	int frequency2 = 123;
	int amplitude2 = 90;
	float ts = (float) 1/fft_sampling_rate;
	for (int j = 0; j < fft_buffer_size; j++){
		mic_buffer_float[j] = (amplitude1*(arm_sin_f32(2*M_PI*frequency1*ts*j)));
		mic_buffer_float[j] += (amplitude2*(arm_sin_f32(2*M_PI*frequency2*ts*j)));
	}

	float fft_buffer_out[fft_buffer_size];
	arm_rfft_fast_instance_f32 fft_handler;
	arm_rfft_fast_init_f32(&fft_handler, fft_buffer_size);
	arm_rfft_fast_f32(&fft_handler, mic_buffer_float, fft_buffer_out, 0);

	int max = -1;
	int max2 = -1;
	int max_index = 0;
	int max2_index = 0;
	int N = fft_buffer_size;

	int mean = 0;
	for (uint16_t i = 0; i < fft_buffer_size; i++) {
		mean += (micBuffer[i+fft_buffer_size]);
	}
	mean = mean /(int) fft_buffer_size;
	int dc_offset = 0; //(int)(20*log10f(mean))*(mean);


	int frequency_point = 0;
	int magnitudes[fft_buffer_size/2];

	for (int i=1; i<N; i=i+2) {

		int real = fft_buffer_out[i];
		int imag = fft_buffer_out[i+1];
		int magnitude = sqrt(real*real+imag*imag);
		magnitude = (int)(20*log10f(magnitude)) - dc_offset;
		if (magnitude<0) magnitude = 0;

		magnitudes[frequency_point] = magnitude;
		frequency_point ++;

	}

	ITM_Port32(31) = 1;
  	for (uint32_t i = 0; i < 1000; i++){
		for (int i = 0 ; i < N/2; i++){
			int mag = magnitudes[i];
			if (mag > max2){
				max2_index = i;
				max2 = mag;
			}
//			if (magnitude < max && magnitude > max2){
//				max2_index = frequency_point;
//				max2 = magnitude;
//			}
		}
  	}
  	ITM_Port32(31) = 2;

  	for (uint32_t i = 0; i < 1000; i++)
  		asmMax(&magnitudes, N/2, &max, &max_index);

  	ITM_Port32(31) = 3;

	float T = fft_buffer_size / fft_sampling_rate;
	float frequencies[N/2];
	for (int i = 0 ; i < N/2 ; i ++){
		frequencies[i] = (float) i / T;
	}

	float max_f  = frequencies[max_index];
	float max2_f = frequencies[max2_index];

	frequency = max_f;
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
  MX_DMA_Init();
  MX_DFSDM1_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_QUADSPI_Init();

  /* USER CODE BEGIN 2 */

  // -------------- Initial stage: GAME SETUP ---------------------------------

  // red LED is initialized as ON (active low), so reset it back to OFF
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);

  // start TIM2 with global interrupts enabled
  	HAL_TIM_Base_Start_IT(&htim2);

  	// initialize hqspi flash
  	  if (BSP_QSPI_Init() != QSPI_OK) {
  		  Error_Handler();
  	  }

  	  // generate full octave (C4 to C5)

  	  uint16_t maxVal = (uint16_t) 4095 * 2 / 3;			// 12 bit resolution has range [0, 4095], we scale this to 2/3 of the range
  	  uint16_t amplitude = maxVal/2;
  	  float32_t f, omega;

  	  // erase all blocks in flash that we need
  	  uint32_t sizeNeeded = numTones * f_s_DAC; // Each block is 64kB. we need to store 8 tones * (44100/2) samples per tone * 2bytes/sample)
  	  for (uint32_t i = 0; i < sizeNeeded; i += 64000) {
  		  if (BSP_QSPI_Erase_Block(flashStartAddr + i) != QSPI_OK) {
  			  Error_Handler();
  		  }
  	  }

  	  // create 0.5s of each tone in RAM buffer and write to flash
  	  for (uint8_t i = 0; i < numTones; i++) {
  		  omega = 2 * 3.14159265 * (frequencies[i]/f_s_DAC); 		// omega = 2pi*f_rel where f_rel = f_wave / f_sampling
  		  for(uint16_t counter = 0; counter < f_s_DAC/2 ; counter++) {
  			  sineBuffer[counter] = (uint16_t) (amplitude * (arm_sin_f32(omega*counter) + 1));
  		  }

  		  	  // write tone to flash at the appropriate address
  			  // note: size for entire 0.5seconds is (f_s/2) samples x 2 bytes per sample = f_sample bytes
  			  // so, we increment flashStartAddr by i*f_s each time
  		  if (BSP_QSPI_Write((uint8_t *) sineBuffer, (uint32_t) flashStartAddr + i*f_s_DAC, sizeof(sineBuffer)) != QSPI_OK) {
  			  Error_Handler();
  		  }
  	  }

  	  // --------- 1st stage: play tones for player to choose from --------------
  	  // start DAC with first values from flash
  	  gameState = 1;
  	  readFirstHalfFromFlash();
  	  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, sineBuffer, f_s_DAC/2, DAC_ALIGN_12B_R);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // check if we are ready for DFT conversion
	  if (gameState == 3) {
		  // start DFT Conversion
		  gameState = 4;

		  // NOTE: this is to do DFT. however if we run it, the RAM buffers dont work anymore and we cannot read from Flash. need fix.
		  get_frequency();

		  // DFT is complete
		  gameState = 5;

	  } else if (gameState == 5) {
		  // once DFT is complete, check if player has won or lost depending on the frequency error
		  float targetFrequency = frequencies[(int) (currentTone / (f_s_DAC/2))];
		  int lowerBound =  (int) ((1.0-allowedError)*targetFrequency);
		  int higherBound = (int) ((1.0+allowedError)*targetFrequency);

		  if (frequency < higherBound && frequency > lowerBound) {
			  showPlayerWon();
		  } else {
			  showPlayerLost();
		  }

		  // change state to game complete
		  gameState = 6;

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
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
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 100;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 80;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1814;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/**
 * @brief Interrupt handler when half of DAC array has been converted
 * @retval None
 * @author Helen
 **/
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
	// first half has been converted
	// read new wave first half and put into buffer's first half
	if (gameState < 5) { // not 5 or 6
		readFirstHalfFromFlash();
	}
}

/**
 * @brief Interrupt handler when all of DAC array has been converted
 * @retval None
 * @author Helen
 **/
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
	// second half of array has been converted
	if (gameState < 5) { // not 5 or 6
		readSecondHalfFromFlash();
	} else if (gameState >= 5){ // 5 or 6
		// done playing winning/losing tone, game is finished
		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

		// NOTE: do NOT reset gameState
	}

}

/**
 * @brief Read next half array from flash into first half of sineBuffer in RAM
 * @retval None
 * @author Helen
 **/
void readFirstHalfFromFlash(void){
	if (BSP_QSPI_Read((uint8_t *) sineBuffer, (uint32_t) flashStartAddr + currentTone, sizeof(sineBuffer) / 2) != QSPI_OK) {
		Error_Handler();
	}
	count1 += 1;
	//since count is 4 here, this means 0.5*4 == 2 seconds, if count == 6, 0.5*6 = 3 seconds, etc.
	if (count1 == 4) {
		currentTone += f_s_DAC/2;
			if (currentTone >= numTones * f_s_DAC) {
				currentTone = 0;
			}
		count1 = 0;
	}
}

/**
 * @brief Read next half array from flash into second half of sineBuffer in RAM
 * @retval None
 * @author Helen
 **/
void readSecondHalfFromFlash(void) {
	if (BSP_QSPI_Read((uint8_t *) sineBuffer + f_s_DAC/2, (uint32_t) flashStartAddr + currentTone, sizeof(sineBuffer) / 2) != QSPI_OK) {
		Error_Handler();
	}
	count2 += 1;
	//since count is 4 here, this means 0.5*4 == 2 seconds, if count == 6, 0.5*6 = 3 seconds, etc.
	if (count2 == 4){
		currentTone += f_s_DAC/2;
			if (currentTone >= numTones * f_s_DAC) {
				currentTone = 0;
			}
		count2 = 0;
	}
}

/**
 * @brief Interrupt handler when DFSDM finish converting all values
 * @retval None
 * @author Krishna
 **/
void HAL_DFSDM_FilterRegConvCpltCallback (DFSDM_Filter_HandleTypeDef * hdfsdm_filter) {
	if (hdfsdm_filter == &hdfsdm1_filter0) {
		// stop recording
		HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
		// turn off flashing green light
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

		//Filter 8 MSB by shifting by 8 to discard channel info, and by 16 more for scaling
	    for(uint16_t i = 0; i < micSamples; i++){
	    	micBuffer[i] = (micBuffer[i] >> 20);

	    	// not needed anymore: store converted recording in first 10k elements of sineBuffer
	    	//sineBuffer[i] = (uint16_t) (micBuffer[i]);
	    }

	    // Test: playback of recording (note SPEAKER IS AT A WAY HIGHER SAMPLING FREQUENCY, we want to play this back at 10kHz sampling frequency...)
	    //HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, sineBuffer, micSamples, DAC_ALIGN_12B_R);

	    // --------- 3rd stage: ready for DFT conversion --------------
	    // TODO: HAND OFF TO THE DFT CONVERSION
	    gameState = 3;
	}
}

/**
 * @brief Interrupt handler for blue push button. Player pushes this when they have chosen their tone and to start recording
 * @retval None
 * @author Krishna
 */
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin) {
	if (GPIO_Pin == BUTTON_Pin) {
		// if we were playing the tones for the player to choose from
		if (gameState == 1) {
			// Stop playing tones
			HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

			// --------- 2nd stage: start recording  --------------
			gameState = 2;
			HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, micBuffer, micSamples);
		}


		// if we are ready to reset the game
		else if(gameState == 6) {
			// turn off LEDs
			HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

			// reset the variables
			currentTone = 0;
			count1 = 0;
			count2 = 0;
			frequency = 0;

			// restart the game
			gameState = 1;
			readFirstHalfFromFlash();
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, sineBuffer, f_s_DAC/2, DAC_ALIGN_12B_R);
		}

	}
}

/**
 * @brief Plays a tone to signify that the player won
 * @retval None
 * @author Helen
 */
void showPlayerWon (void) {
	// show GREEN LED to show that player won
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);

//	// read winning tone into sineBuffer which stores 0.5 seconds of sound in 22050 samples
//	// c4 for 1/3 of the 0.5s
//	if (BSP_QSPI_Read((uint8_t *) sineBuffer, (uint32_t) flashStartAddr, sizeof(sineBuffer) /3 ) != QSPI_OK) {
//		Error_Handler();
//	}
//
//	// c5 for 2/3 of the 0.5s
//	if (BSP_QSPI_Read((uint8_t *) sineBuffer + f_s_DAC/3, (uint32_t) flashStartAddr + 7*f_s_DAC, 2 * sizeof(sineBuffer) / 3) != QSPI_OK) {
//		Error_Handler();
//	}
//
//	//play winning tone
//	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, sineBuffer, f_s_DAC/2, DAC_ALIGN_12B_R);
}

/**
 * @brief Plays a tone to signify that the player lost
 * @retval None
 * @author Helen
 */
void showPlayerLost (void) {
	// show RED LED to show that player won
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);


//	// read losing tone into sineBuffer which stores 0.5 seconds of sound in 22050 samples
//	// c5 for 1/3 of the 0.5s
//	if (BSP_QSPI_Read((uint8_t *) sineBuffer, (uint32_t) flashStartAddr + 7*f_s_DAC, sizeof(sineBuffer) /3 ) != QSPI_OK) {
//		Error_Handler();
//	}
//
//	// c5 for 2/3 of the 0.5s
//	if (BSP_QSPI_Read((uint8_t *) sineBuffer + f_s_DAC/3, (uint32_t) flashStartAddr,  2 * sizeof(sineBuffer) / 3) != QSPI_OK) {
//		Error_Handler();
//	}

//	//play losing tone
//	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, sineBuffer, f_s_DAC/2, DAC_ALIGN_12B_R);
}

/**
 * @brief Interrupt handler for TIM2 to trigger LED changes depending on game state
 * @retval None
 * @author Helen
 **/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// verify timer is htim2
	if (htim == &htim2) {
		// flash LED if needed (only during mic recording)
		if (gameState == 2) {
			if (counterLED == counterLEDMax) {
				counterLED = 0;
				HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
			} else {
				counterLED++;
			}
		}
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

  // turn on red LED and halt debugger with breakpoint
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
  __BKPT();

//  while (1)
//  {
//  }
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
