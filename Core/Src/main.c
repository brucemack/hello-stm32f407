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
#include <math.h>
#include <stdint.h>

#define ARM_MATH_CM4

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
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Synthesizer
// Largest value for a 32-bit signed number
static float amp = 0x7fffffff;
// Sample frequency
static const float fs = 44100;
static const int lut_size = 256;
static float lut[256];
// The number of steps through the synthesis LUT on each cycle
static unsigned int lutStep = 0;
// Current pointer in the LUT
static unsigned int lutPtr = 0;
// Balance between I/Q
static float balance = 1.0;
static float gain = 0.75;

#define FILTER_BLOCK_SIZE 16
static const int blockSize = FILTER_BLOCK_SIZE;

// This where the DMA happens.  We multiply by 8 to address:
// - The fact that we have left and right data moving through at the same time
// - Each sample takes up two 16-bit words
// - There is space for two blocks.  The DMA works on one block while
//   the other block is being processed, and then we flip to the other side.
//
static uint16_t out_data[FILTER_BLOCK_SIZE * 8];

static void setSynthFreq(float freqHz) {
	float phasePerSample = (freqHz / fs) * 2.0 * 3.14159;
	lutStep = (phasePerSample / (2.0 * 3.1415926)) * lut_size;
}

float ramp = 0;


static void moveOut(int base) {

	int32_t ci;
	uint16_t hi = 0;
	uint16_t lo = 0;

	int out_ptr = base;
	float filterOut[FILTER_BLOCK_SIZE];

	// Take the synthesized cosine and load it into the filter input area
	for (int i = 0; i < blockSize; i++) {
		// Generate synthesized data by stepping through the cosine table
		// This handles the wrapping of the LUT pointer:
		lutPtr = (lutPtr + lutStep) & 0xff;
		filterOut[i] = amp * lut[lutPtr] * gain;

		//filterOut[i] = amp * ramp * gain;
		//ramp += 0.001;
		//if (ramp > 1.0) {
		//	ramp = 0;
		//}
	}

	// Generate the output signals
	for (int i = 0; i < blockSize; i++) {
		// LEFT CHANNEL
		ci = filterOut[i] * balance;
		hi = (ci >> 16) & 0xffff;
		lo = ci & 0xffff;
		out_data[out_ptr++] = hi;
		out_data[out_ptr++] = lo;
		// RIGHT CHANNEL
		ci = filterOut[i] * balance;
		hi = (ci >> 16) & 0xffff;
		lo = ci & 0xffff;
		out_data[out_ptr++] = hi;
		out_data[out_ptr++] = lo;
	}
}

// Called at the half-way point.  Fills in the first half of the DMA area.
void HAL_I2S_TxHalfCpltCallback (I2S_HandleTypeDef * hi2s) {
	moveOut(0);
}

// Called at the end point.  Fills in the second half of the DMA area.
void HAL_I2S_TxCpltCallback (I2S_HandleTypeDef * hi2s) {
	moveOut(blockSize * 4);
}

HAL_StatusTypeDef writeCodec(uint8_t addr, uint8_t data) {
	uint8_t devAddr = 0x30;
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(devAddr), addr, 1, &data, 1, 10);
	return status;
}

extern void CppMain_setup();

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
  MX_I2C1_Init();
  MX_I2S2_Init();
  /* USER CODE BEGIN 2 */

	HAL_StatusTypeDef status;
	uint8_t devAddr = 0x60 << 1;
	uint8_t addr = 0;
	uint8_t data = 0;

	// ----- SI5351 INIT --------------------------------------------------
	// Make sure the device is alive
	status = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(devAddr), 2, 2);
	if (status != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
		printf("Problem with Si5351");

	CppMain_setup();

	// ----- CODEC INIT ---------------------------------------------------

	// Harware reset the CODEC
	HAL_GPIO_WritePin(CODEC_RESET_GPIO_Port, CODEC_RESET_Pin, 1);
	HAL_GPIO_WritePin(CODEC_RESET_GPIO_Port, CODEC_RESET_Pin, 0);
	HAL_GPIO_WritePin(CODEC_RESET_GPIO_Port, CODEC_RESET_Pin, 1);

	HAL_Delay(2);

	// NOTE: TLV320AIC3204 uses standard 7-bit convention (no shifting needed)
	devAddr = 0x30;
	addr = 0;
	data = 0;

	// Make sure the device is alive
	status = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(devAddr), 2, 2);
	if (status != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
		printf("Problem");

	// Select page 0
	writeCodec(0, 0);
	// Software reset
	writeCodec(1, 1);

	HAL_Delay(2);

	// Select page 0
	writeCodec(0, 0);
  	// Configure MFP5 as GPIO input
	writeCodec(52, 0b00001000);

	// ===== Sanity check on CODEC =====
  	// Set GPIO to HI
	HAL_GPIO_WritePin(CODEC_MFP5_GPIO_Port, CODEC_MFP5_Pin, 1);
	// Read back
	addr = 52;
	data = 0;
  	status = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(devAddr), addr, 1, &data, 1, 100);
  	if (data != 10)
  		printf("Problem");
  	// Set GPIO to LO
	HAL_GPIO_WritePin(CODEC_MFP5_GPIO_Port, CODEC_MFP5_Pin, 0);
	// Read back
  	status = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(devAddr), addr, 1, &data, 1, 100);
  	if (data != 8)
  		printf("Problem");

  	// ===== DAC SETUP ======
	// Select page 0
	writeCodec(0x00, 0x00);
	// Clock settings register 1, Configure source of CODEC_CLKIN=MCLK
	writeCodec(0x04, 0b00000000);
	// PLL is powered down
	writeCodec(0x05, 0b00000000);
	// Setup NDAC (on, divide by 1)
	writeCodec(0x0b, 0x81);
	// Setup MDAC (on, divide by 2)
	writeCodec(0x0c, 0x82);
	// Set OSR of DAC to 128
	writeCodec(0x0d, 0x00);
	writeCodec(0x0e, 0x80);
	// Audio interface.  I2S mode, word length=32, BCLK is input, WCLK is input, DOUT not HI-Z
	writeCodec(0x1b, 0b00110000);
	// Data offset by 1 BCLK
	// TODO: RESEARCH THIS TO MAKE SURE IT IS RIGHT
	//writeCodec(0x1c, 0b00000001);
	// Default bit clock polarity, BDIV_CLKIN=DAC_CLK
	writeCodec(0x1d, 0b00000000);
	// BCLK-N divider powered down
	// TODO: CHECK THIS
	writeCodec(0x1e, 0b00000000);
	// Setup DAC mode = PRB_P8, which is:
	//   Interpolation filter B
	//   Stereo
	//   No IIR
	//   4 biquads
	//   DRC on
	//   3D off
	//   No beep generation
	writeCodec(0x3c, 0b00001000);
	// Select page 1
	writeCodec(0x00, 0x01);
	// Disable weak connection of AVdd with DVdd in presence of external AVdd supply
	writeCodec(0x01, 0b00001000);
	// DVdd/AVdd LDO nominally 1.72V, analog blocks enabled, Avdd LDO powered up
	writeCodec(0x02, 0b00000001);
	// Analog input quick-charging configuration  Input power-up time is 6.4ms (for ADC)
	// TODO: NEED TO STUDY THIS
	writeCodec(0x47, 0b00110010);
	// Reference power-up configuration register.  Reference powers up in 40ms when
	// analog blocks are on.
	writeCodec(0x7b, 0b00000001);
  	// Select page 1
	writeCodec(0x00, 0x01);
	// Soft routing 0, De-pop: 5 time constants, 6k resistance
	writeCodec(0x14, 0b00100101);
	// Route LDAC/RDAC to HPL/HPR
	writeCodec(0x0c, 0b00001000);
	writeCodec(0x0d, 0b00001000);
	// Route IN1L to HPL (BYPASS)
	//writeCodec(0x0c, 0b00000100);
	// Route IN1R to HPR (BYPASS)
	//writeCodec(0x0d, 0b00000100);
	// Set the DAC PTM mode to PTM_P3/P4
	writeCodec(0x03, 0x00);
	writeCodec(0x04, 0x00);
	// [VOLUME CONTROL]
  	// Unmute HPL/HPR driver, set 0dB gain
	writeCodec(0x10, 0b00111011);
	writeCodec(0x11, 0b00111011);
  	// Power configuration.  Output of HPL/HPR powered from LDOIN, range is 1.8V to 3.6V
	writeCodec(0x0a, 0b00000011);
	// Overcurrent detection on, debounced = 64ms, driver shut down
	writeCodec(0x0b, 0b00011001);
  	// Power up the HPL/HPR
	writeCodec(0x09, 0b00110000);

	HAL_Delay(2500);

  	// Select page 0
	writeCodec(0x00, 0x00);
	// [VOLUME CONTROL]
	// Left DAC digital volume
	writeCodec(0x41, 0b11110000);
	// [VOLUME CONTROL]
	// Right DAC digital volume
	writeCodec(0x42, 0b11110000);
  	// Power LDAC/RDAC, soft-stepping disabled
	writeCodec(0x3f, 0b11010110);
	// Unmute
	writeCodec(0x40, 0b00000000);

	// Read back status
  	// Select page 0
	writeCodec(0x00, 0x00);
	addr = 0x25;
  	status = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(devAddr), addr, 1, &data, 1, 100);
  	if (status != 0b10101010) {
  		printf("Problem");
  	}

  	// Select page 0
	writeCodec(0x00, 0x00);
	addr = 0x2c;
  	status = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(devAddr), addr, 1, &data, 1, 100);

	// Populate synthesizer LUT
	for (int i = 0; i < lut_size; i++) {
		float t = (float)i / (float)lut_size;
		float a = t * 2.0 * 3.1315926;
		lut[i] = cos(a);
	}

	setSynthFreq(4000);

    // Initialize DMA
    HAL_I2S_Transmit_DMA(&hi2s2, out_data, blockSize * 4);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 96;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_32B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CODEC_MFP5_Pin|CODEC_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_0_GPIO_Port, GPIO_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CODEC_MFP5_Pin CODEC_RESET_Pin */
  GPIO_InitStruct.Pin = CODEC_MFP5_Pin|CODEC_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_0_Pin */
  GPIO_InitStruct.Pin = GPIO_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_0_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
