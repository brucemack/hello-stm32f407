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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <stdint.h>

#define ARM_MATH_CM4
#include "arm_math.h"

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
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi3_rx;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2S3_Init(void);
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
// Balance between I/Q.  This was determined experimentally.
volatile float balance = 1.02;
static float gain = 0.75;
// Phase adjust (I/Q leakage).  This was determined
// experimentally
volatile float phaseAdjust = -0.08;

#define FILTER_TAP_NUM 63
// This is the number of samples that we process in each DMA cycle
#define FILTER_BLOCK_SIZE 16
static const int blockSize = FILTER_BLOCK_SIZE;

extern float hilbert_90_filter_taps[FILTER_TAP_NUM];
static float filter_state[FILTER_TAP_NUM + FILTER_BLOCK_SIZE - 1];
static arm_fir_instance_f32 filter_lp_0;

// Delay line that is equivalent in length to the group delay of the filter
#define DELAY_TAP_NUM 63
static float delayBuffer[(DELAY_TAP_NUM - 1) / 2];
static int delaySize = (DELAY_TAP_NUM - 1) / 2;
static int delayPtr = 0;

static int fix(int ptr) {
	return ptr % delaySize;
}

static void writeDelay(const float* d, int size) {
	for (int i = 0; i < size; i++) {
		delayBuffer[delayPtr] = d[i];
		delayPtr = fix(delayPtr + 1);
	}
}

static void readDelay(float* d, int size) {
	for (int i = 0; i < size; i++) {
		d[i] = delayBuffer[fix(delayPtr + i)];
	}
}

// This where the DMA happens.  We multiply by 8 to address:
// - The fact that we have left and right data moving through at the same time
// - Each sample takes up two 16-bit words
// - There is space for two blocks.  The DMA works on one block while
//   the other block is being processed, and then we flip to the other side.
//
static volatile uint16_t out_data[FILTER_BLOCK_SIZE * 8];
static volatile uint16_t in_data[FILTER_BLOCK_SIZE * 8];

// This keeps track of which output block is currently *not* being
// transferred to the I2S interface.  Buffer filling should happen
// on this block.
static volatile int outBlockFree = 0;

static void setSynthFreq(float freqHz) {
	float phasePerSample = (freqHz / fs) * 2.0 * 3.14159;
	lutStep = (phasePerSample / (2.0 * 3.1415926)) * lut_size;
}

// This reads half of the inbound data from the DMA buffer.  This is
// called by the DMA interrupt at the half-way point through the
// entire DMA buffer.
static void moveIn(int inBlock, int outBlockFree) {

	int in_ptr;
	if (inBlock == 0) {
		in_ptr = 0;
	} else {
		in_ptr = blockSize * 4;
	}

	int out_ptr;
	if (outBlockFree == 0) {
		out_ptr = 0;
	} else {
		out_ptr = blockSize * 4;
	}

	int32_t sampleI, sampleQ;
	uint16_t hi;
	uint16_t lo;
	float filterIn[FILTER_BLOCK_SIZE];
	float filterOut[FILTER_BLOCK_SIZE];
	float delayIn[FILTER_BLOCK_SIZE];
	float delayOut[FILTER_BLOCK_SIZE];
	uint16_t s;

	for (int i = 0; i < blockSize; i++) {

		// Read the left channel into the filter
		// Load the high end of the sample into the high end of the 32-bit number
		s = in_data[in_ptr++];
		sampleI = s;
		// Shift up
		sampleI = sampleI << 16;
		// Add low end of sample
		s = in_data[in_ptr++];
		sampleI |= s;

		// Read the right channel into the delay
		// Load the high end of the sample into the high end of the 32-bit number
		s = in_data[in_ptr++];
		sampleQ = s;
		// Shift up
		sampleQ = sampleQ << 16;
		// Add low end of sample
		s = in_data[in_ptr++];
		sampleQ |= s;

		// Allow a small amount of the Q signal into the I for phase adjust
		filterIn[i] = (float)sampleI + (float)sampleQ * phaseAdjust;
		delayIn[i] = (float)sampleQ + (float)sampleI * phaseAdjust;
	}

	// Apply the FIR filter
	// MEASUREMENT: This takes about 4,000 cycles.
	arm_fir_f32(&filter_lp_0, filterIn, filterOut, blockSize);

	// Move data out of the circular buffer (lagged)
	readDelay(delayOut, blockSize);
	// Move data into the circular buffer
	writeDelay(delayIn, blockSize);

	// Move things into the DMA outbound area
	for (int i = 0; i < blockSize; i++) {

		// Generate synthesized data by stepping through the cosine table
		// This handles the wrapping of the LUT pointer:
		//lutPtr = (lutPtr + lutStep) & 0xff;
		//sample = amp * lut[lutPtr] * gain;

		// Combine Hilbert with Delay
		// This looks like +USB (LSB is canceled)
		int32_t sample = (0.5 * balance * filterOut[i]) - (0.5 * delayOut[i]);

		// Transfer filtered sample to left output
		//sample = filterOut[i];
		hi = (sample >> 16) & 0xffff;
		lo = (sample & 0xffff);
		out_data[out_ptr++] = hi;
		out_data[out_ptr++] = lo;

		// Transfer delayed data to right output
		//sample = delayOut[i];
		hi = (sample >> 16) & 0xffff;
		lo = (sample & 0xffff);
		out_data[out_ptr++] = hi;
		out_data[out_ptr++] = lo;
	}
}

// Called at the half-way point.  First half of the DMA area is available
// for updating after this call.
void HAL_I2S_TxHalfCpltCallback (I2S_HandleTypeDef * hi2s) {
	outBlockFree = 0;
}

// Called at the end point.  Second half of the DMA area if available
// for updating after this call
void HAL_I2S_TxCpltCallback (I2S_HandleTypeDef * hi2s) {
	outBlockFree = 1;
}

// Called at the half-way point.  Fills in the first half of the DMA area.
void HAL_I2S_RxHalfCpltCallback (I2S_HandleTypeDef * hi2s) {
	moveIn(0, outBlockFree);
}

// Called at the end point.  Fills in the second half of the DMA area.
void HAL_I2S_RxCpltCallback (I2S_HandleTypeDef * hi2s) {
	moveIn(1, outBlockFree);
}

HAL_StatusTypeDef writeCodec(uint8_t addr, uint8_t data) {
	uint8_t devAddr = 0x30;
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(devAddr), addr, 1, &data, 1, 10);
	if (status != HAL_OK) {
		printf("Error on I2C write\r\n");
	}
	return status;
}

uint8_t readCodec(uint8_t addr) {
	uint8_t devAddr = 0x30;
	uint8_t data = 0;
	HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(devAddr), addr, 1, &data, 1, 10);
	return data;
}

extern void CppMain_setup();

#define WS_Pin GPIO_PIN_15
#define WS_Pin_Port GPIOA

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
  MX_USART1_UART_Init();
  MX_I2S3_Init();
  /* USER CODE BEGIN 2 */

	// Initialize the filter
	arm_fir_init_f32(&filter_lp_0, FILTER_TAP_NUM, hilbert_90_filter_taps, filter_state, FILTER_BLOCK_SIZE);

	HAL_StatusTypeDef status;
	uint8_t devAddr = 0x60 << 1;
	uint8_t addr = 0;
	uint8_t data = 0;

	// ----- SI5351 INIT --------------------------------------------------
	// Make sure the device is alive
	status = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(devAddr), 2, 2);
	if (status != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
		printf("Problem with Si5351\r\n");

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
		printf("Problem with CODEC\r\n");

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
  		printf("Problem\r\n");
  	// Set GPIO to LO
	HAL_GPIO_WritePin(CODEC_MFP5_GPIO_Port, CODEC_MFP5_Pin, 0);
	// Read back
  	status = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(devAddr), addr, 1, &data, 1, 100);
  	if (data != 8)
  		printf("Problem\r\n");

  	// ===== DAC SETUP ======
	// Select page 0
	writeCodec(0x00, 0x00);
	// Clock settings register 1, Configure source of CODEC_CLKIN=MCLK
	writeCodec(0x04, 0b00000000);
	// PLL is powered down (not used)
	writeCodec(0x05, 0b00000000);

	// Setup NDAC (on, divide by 1)
	writeCodec(0x0b, 0x81);
	// Setup MDAC (on, divide by 2)
	writeCodec(0x0c, 0x82);
	// Set OSR of DAC to 128
	writeCodec(0x0d, 0x00);
	writeCodec(0x0e, 0x80);

	// Setup NADC (on, divide by 1) [Pg. 99]
	writeCodec(0x12, 0x81);
	// Setup MADC (on, divide by 2) [Pg. 99]
	writeCodec(0x13, 0x82);
	// Set OSR of ADC to 128
	writeCodec(0x14, 0b10000000);

	// Audio interface.  I2S mode, word length=32, BCLK is input, WCLK is input, DOUT not HI-Z
	writeCodec(0x1b, 0b00110000);
	// Data offset by 0 BCLK (default)
	writeCodec(0x1c, 0b00000000);
	// Default bit clock polarity, BDIV_CLKIN=DAC_CLK
	writeCodec(0x1d, 0b00000000);
	// BCLK-N divider powered down
	writeCodec(0x1e, 0b00000000);
	// Audio interface setting register 5 -
	writeCodec(0x20, 0x00);
	// DOUT/MFP2 Control [Pg. 108]
	writeCodec(0x35, 0b00010010);
	// Setup DAC mode = PRB_P8, which is:
	//   Interpolation filter B
	//   Stereo
	//   No IIR
	//   4 biquads
	//   DRC on
	//   3D off
	//   No beep generation
	writeCodec(0x3c, 0b00001000);
	// Setup ADC mode.  Use PRB_R2 (First-Order IIR, AGC, Filter A, 6 biquads)
	writeCodec(0x3d, 0b00000010);

	// Select page 1
	writeCodec(0x00, 0x01);
	// Disable weak connection of AVdd with DVdd in presence of external AVdd supply
	writeCodec(0x01, 0b00001000);
	// DVdd/AVdd LDO nominally 1.72V, analog blocks enabled, Avdd LDO powered up
	writeCodec(0x02, 0b00000001);
	// Analog input quick-charging configuration  Input power-up time is 6.4ms (for ADC)
	writeCodec(0x47, 0b00110010);
	// Set MicPGA startup delay to 3.1ms
	writeCodec(0x47, 0x32);
	// Reference power-up configuration register.  Reference powers up in 40ms when
	// analog blocks are on.
	writeCodec(0x7b, 0b00000001);
	// ADC Power Tune PTM_R4
	writeCodec(0x3d, 0x00);

	// Select page 1
	writeCodec(0x00, 0x01);
	// Soft routing 0, De-pop: 5 time constants, 6k resistance
	writeCodec(0x14, 0b00100101);
	// Route LDAC/RDAC to HPL/HPR
	writeCodec(0x0c, 0b00001000);
	writeCodec(0x0d, 0b00001000);
	// Set the DAC PTM mode to PTM_P3/P4
	writeCodec(0x03, 0x00);
	writeCodec(0x04, 0x00);
  	// Unmute HPL/HPR driver (0b01), set 0dB gain
	// Here is a level control that is not supposed to be used as
	// the regular volume control.
	// Range: 0b111010 -> 0b011101
	writeCodec(0x10, 0b00111010);
	writeCodec(0x11, 0b00111010);
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
	writeCodec(0x41, 0);
	// [VOLUME CONTROL]
	// Right DAC digital volume
	writeCodec(0x42, 0);
  	// Power LDAC/RDAC, soft-stepping disabled
	writeCodec(0x3f, 0b11010110);
	// Unmute
	writeCodec(0x40, 0b00000000);

	// Configure ADC [Pg. 90 for example]
  	// Select page 1
	writeCodec(0x00, 0x01);
	// Route IN1L to left Mic PGA with 20K input impedance [Pg. 135]
	writeCodec(0x34, 0x80);
	// Route Common Mode to LEFT_M with impedance of 20K
	writeCodec(0x36, 0x80);
	// Route IN1R to right Mic PGA with input impedance of 20K [Pg. 135]
	writeCodec(0x37, 0x80);
	// Route Common Mode to RIGHT_M with impedance of 20K
	writeCodec(0x39, 0x80);

	// Unmute left MICPGA, set gain to 6dB (given that input impedance is 20k => 0dB)
	writeCodec(0x3b, 0x0c);
	// Unmute right MICPGA, set gain to 6dB (given that input impedance is 20k => 0dB)
	writeCodec(0x3c, 0x0c);

	// Page 0
	writeCodec(0x00, 0x00);
	// Power up left and right ADC channels, digital mic off, 1 gain step per ADC clock [Pg. 115]
	writeCodec(0x51, 0b11000000);
	// Unmute left and right digital volume controls.  ADC fine gain adjust = 0dB [Pg. 115]
	writeCodec(0x52, 0b00000000);

	// ADC channel volume control - left (should be 0dB) [Pg. 116]
	writeCodec(0x53, 0b00100110);
	// ADC channel volume control - right (should be 0dB)
	writeCodec(0x54, 0b00100110);

	// *******************************************
	// TEMPORARY: Turn on loopback.  This routes ADC output to DAC input. [Pg. 101]
	//writeCodec(0x00, 0x00);
	//data = readCodec(0x1d);
	//data |= 0b00010000;
	//writeCodec(0x1d, data);
	// *******************************************

	// Populate synthesizer LUT
	for (int i = 0; i < lut_size; i++) {
		float t = (float)i / (float)lut_size;
		float a = t * 2.0 * 3.1315926;
		lut[i] = cos(a);
	}

	setSynthFreq(2000);

    // Initialize DMA
	// IPORTANT: Because of a bug in STM32F407, we must always enable the
	// slave I2S port before the master.
	HAL_GPIO_WritePin(GPIOA, WS_Pin, GPIO_PIN_SET);
    HAL_I2S_Receive_DMA(&hi2s3, in_data, blockSize * 4);
    // This is the master:
    HAL_I2S_Transmit_DMA(&hi2s2, out_data, blockSize * 4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  CppMain_loop();
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
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 72;
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
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_SLAVE_RX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_32B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
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
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
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
  HAL_GPIO_WritePin(GPIOD, GPIO_0_Pin|GPIO_1_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : GPIO_0_Pin GPIO_1_Pin */
  GPIO_InitStruct.Pin = GPIO_0_Pin|GPIO_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
