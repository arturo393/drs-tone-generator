/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "i2c1_master.h"
#include "max2871.h"
#include "m24c64.h"
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
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// PINS MAX2871
/*
 #define CLOCKPIN A0
 #define DATAPIN A1
 #define LE A2
 #define CE A3
 */

float AC[56];
float EEPROM_float;

// OLED 128x64 with SH1106 Controller
// E.G. DM-OLED13-625
/*
 #define OLED_MOSI  29
 #define OLED_CLK   27
 #define OLED_DC    35
 #define OLED_CS    37
 #define OLED_RESET 31
 Adafruit_SH1106 display(OLED_MOSI,OLED_CLK,OLED_DC,OLED_RESET,OLED_CS);
 */
/*
 #if (SH1106_LCDHEIGHT != 64)
 #error("Height incorrect, please fix Adafruit_SH1106.h!");
 #endif

 unsigned int ActiveDisplay = 1;  // AMPLITUDE
 boolean ShowCursor = true;
 unsigned int CursorPosition = 2;

 // Menue Intem to be displayed
 unsigned int MenueItem = 0;

 // STATE OF THE ROTARY ENCODER
 const int RE2 = 49;  // PRESSED
 const int RE1 = 47;
 const int RE0 = 45;
 unsigned long StartMilli;
 unsigned long DurationMilli;
 boolean PressedLong = false;
 */
// "INTERRUPT" VARIABLES
unsigned int RE_now = 0;
unsigned int RE_old = 0;
unsigned int RE_xor = 0;
unsigned int RE_one = 0;
unsigned int RE_two = 0;
unsigned int RE_eval = 0x0;  // 0 = LEFT(-), 1=RIGHT(+), 2=PRESSED
unsigned long RE_time0 = 0;
unsigned long RE_time1 = 0;
unsigned long RE_time2 = 0;

const int EEPROM_ADR = 0x50;
unsigned long *sumaptr;
unsigned long *muxptr;

MAX2871_t *ppl_ptr;
uint8_t addrList[5] = { 0 };
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	MAX2871_t ppl;
	ppl.FreqOUTold = 0;
	ppl_ptr = &ppl;
	ppl_ptr->ATTBYTE = 0;
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
	MX_I2C1_Init();
	MX_SPI2_Init();
	MX_USART1_UART_Init();
	MX_CRC_Init();
	/* USER CODE BEGIN 2 */
	i2c1MasterInit();
	max2871Init(&ppl);
	uint8_t addrList[5] = { 0 };
	uint8_t freq_init[FREQ_OUT_SIZE] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
			0x77, 0x88 };
	m24c64WriteNBytes(BASE_ADDR, freq_init, FREQ_OUT_ADDR, FREQ_OUT_SIZE);
	uint8_t buffer[10] = { 0 };

	m24c64ReadNBytes(BASE_ADDR, buffer, FREQ_OUT_ADDR, FREQ_OUT_SIZE);

	for (int i = FREQ_OUT_SIZE - 1; i >= 0; i--)
		ppl.FreqOut |= (buffer[i] << (i * 8));

	m24c64ReadNBytes(BASE_ADDR, buffer, FREQ_OUT_ADDR + FREQ_OUT_SIZE,
	LACT_SIZE);
	ppl.LACT = buffer[0] + buffer[1] / 2.0;

	if (ppl.FreqOut == -1) {
		ppl.FreqOut = 4000000000;
		ppl.LACT = 12;
	}

	HAL_Delay(200);
	max2871CalculateRegisterValues(&ppl);

	max2871RegisterInit(&hspi2, &ppl);
	uint8_t IODIRA = 0x00;

	buffer[0] = 0x00;
	buffer[1] = IODIRA;
	m24c64WriteNBytes(BASE_ADDR, buffer,
			FREQ_OUT_ADDR + FREQ_OUT_SIZE + LACT_SIZE, MCPADR_SIZE);
	uint8_t IODIRB = 0x01;
	buffer[0] = 0x00;
	buffer[1] = IODIRB;
	m24c64WriteNBytes(BASE_ADDR, buffer,
			FREQ_OUT_ADDR + FREQ_OUT_SIZE + LACT_SIZE, MCPADR_SIZE);

	// ATTENUATION NEEDED FOR LMAX OUTPUT - CHECK THAT
	// no se que es esto
	AC[0] = 19; //  100 MHz
	AC[1] = 19; //  300 MHz
	AC[2] = 20; //  500 MHz
	AC[3] = 18.5; //  700 MHz
	AC[4] = 18.5; //  900 MHz
	AC[5] = 18.5; // 1100 MHz
	AC[6] = 18.5; // 1300 MHz
	AC[7] = 17; // 1500 MHz
	AC[8] = 17; // 1700 MHz
	AC[9] = 16.5; // 1900 MHz
	AC[10] = 16; // 2100 MHz
	AC[11] = 15; // 2300 MHz
	AC[12] = 13; // 2500 MHz
	AC[13] = 13; // 2700 MHz
	AC[14] = 11; // 2900 MHz
	AC[15] = 9.5; // 3100 MHz
	AC[16] = 10; // 3300 MHz
	AC[17] = 9; // 3500 MHz
	AC[18] = 8.5; // 3700 MHz
	AC[19] = 7; // 3900 MHz
	AC[20] = 6; // 4100 MHz
	AC[21] = 4; //4300 MHz
	AC[22] = 5.5; //4500 MHz
	AC[23] = 5;  //4700 MHz
	AC[24] = 2;  //4900 MHz
	AC[25] = 0;  //5100 MHz
	AC[26] = 0;  //5300 MHz
	AC[27] = 0;  //5500 MHz
	AC[28] = 0;  //5700 MHz
	AC[29] = 0;  //5900 MHz
	AC[30] = 0;  //6100 MHz
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	int Valor_0;
	int Valor_1;
	int Valor_2;
	int Valor_3;
	int Valor_4;
	int Valor_5;
	int Valor_6;
	int Valor_7;
	int Valor_8;
	int Valor_9;
	unsigned long suma_current = -1;
	unsigned long suma_read;
	unsigned long suma_new = 0;
	unsigned long mux;
	muxptr = &mux;
	unsigned long FreqBase = 150000000;
	ppl.FreqOut = suma_read + FreqBase;
	unsigned long ultima_suma = HAL_GetTick();
	unsigned long togg = HAL_GetTick();
	int toggtime;
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		//toggtime=HAL_GetTick()-togg;
		HAL_GPIO_WritePin(GPIOA, MAX_RF_ENABLE_Pin, GPIO_PIN_SET);
		//if ((HAL_GetTick() - togg) > 1000) {

			//HAL_GPIO_TogglePin(MAX_RF_ENABLE_GPIO_Port, MAX_RF_ENABLE_Pin);
			//togg = HAL_GetTick();
		//}

		Valor_0 = HAL_GPIO_ReadPin(SW_0_GPIO_Port, SW_0_Pin) ? 0 : 12500;
		Valor_1 = HAL_GPIO_ReadPin(SW_1_GPIO_Port, SW_1_Pin) ? 0 : 25000;
		Valor_2 = HAL_GPIO_ReadPin(SW_2_GPIO_Port, SW_2_Pin) ? 0 : 50000;
		Valor_3 = HAL_GPIO_ReadPin(SW_3_GPIO_Port, SW_3_Pin) ? 0 : 100000;
		Valor_4 = HAL_GPIO_ReadPin(SW_4_GPIO_Port, SW_4_Pin) ? 0 : 200000;
		Valor_5 = HAL_GPIO_ReadPin(SW_5_GPIO_Port, SW_5_Pin) ? 0 : 400000;
		Valor_6 = HAL_GPIO_ReadPin(SW_6_GPIO_Port, SW_6_Pin) ? 0 : 800000;
		Valor_7 = HAL_GPIO_ReadPin(SW_7_GPIO_Port, SW_7_Pin) ? 0 : 1600000;
		Valor_8 = HAL_GPIO_ReadPin(SW_8_GPIO_Port, SW_8_Pin) ? 0 : 3200000;
		Valor_9 = HAL_GPIO_ReadPin(SW_9_GPIO_Port, SW_9_Pin) ? 0 : 6400000;

		suma_read = (Valor_0) + (Valor_1) + (Valor_2) + (Valor_3) + (Valor_4)
				+ (Valor_5) + (Valor_6) + (Valor_7) + (Valor_8) + (Valor_9);

		if (suma_read != suma_new) {
			ultima_suma = HAL_GetTick();
			suma_new = suma_read;
		}

		if ((HAL_GetTick() - ultima_suma) > 1000) {

			if (suma_new != suma_current) {
				ppl.FreqOut = suma_read + FreqBase;
				max2871Program(&hspi2, &ppl);
				suma_current = suma_new;
			}
		}

		//mux = HAL_GPIO_ReadPin(GPIOA, MAX_MUX_Pin);
		//	max2871Read();
	}

//max2871Read();
	/* USER CODE END WHILE */
	/* USER CODE BEGIN 3 */
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void) {

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
	if (HAL_CRC_Init(&hcrc) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00303D5B;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_1LINE;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 7;
	hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

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
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, RS485_CTRL_Pin | LED_2_Pin | LED_3_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
	MAX_LE_Pin | MAX_CE_Pin | MAX_MUX_Pin | MAX_RF_ENABLE_Pin | LED_1_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : RS485_CTRL_Pin LED_2_Pin LED_3_Pin */
	GPIO_InitStruct.Pin = RS485_CTRL_Pin | LED_2_Pin | LED_3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : TEST_INPUT_1_Pin TEST_INPUT_2_Pin SW_2_Pin */
	GPIO_InitStruct.Pin = TEST_INPUT_1_Pin | TEST_INPUT_2_Pin | SW_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : MAX_LE_Pin MAX_CE_Pin MAX_MUX_Pin MAX_RF_ENABLE_Pin
	 LED_1_Pin */
	GPIO_InitStruct.Pin = MAX_LE_Pin | MAX_CE_Pin | MAX_MUX_Pin
			| MAX_RF_ENABLE_Pin | LED_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : MAX_LOCK_DETECTOR_Pin SW_1_Pin SW_3_Pin SW_4_Pin
	 SW_5_Pin */
	GPIO_InitStruct.Pin = MAX_LOCK_DETECTOR_Pin | SW_1_Pin | SW_3_Pin | SW_4_Pin
			| SW_5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : SW_0_Pin SW_6_Pin SW_7_Pin SW_8_Pin
	 SW_9_Pin */
	GPIO_InitStruct.Pin = SW_0_Pin | SW_6_Pin | SW_7_Pin | SW_8_Pin | SW_9_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
