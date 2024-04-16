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
#include "led.h"
#include "uart1.h"
#include "rs485.h"
#include "module.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define FREQ_STEP 12500
#define HS16_CLK 16000000
#define BAUD_RATE 115200
#define SW_DEBOUNCE 500

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

IWDG_HandleTypeDef hiwdg;

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
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

MAX2871_t *ppl_ptr;
UART1_t *uart1_ptr;
Tone_uhf_t *uhf_ptr;
RS485_t *rs485_ptr;

unsigned long getFreqSum(unsigned long FreqBase) {
	unsigned long suma_read;

	suma_read = 0;
	suma_read += HAL_GPIO_ReadPin(SW_0_GPIO_Port, SW_0_Pin) ? 0 : FREQ_STEP;
	suma_read += HAL_GPIO_ReadPin(SW_1_GPIO_Port, SW_1_Pin) ? 0 : FREQ_STEP * 2;
	suma_read += HAL_GPIO_ReadPin(SW_2_GPIO_Port, SW_2_Pin) ? 0 : FREQ_STEP * 4;
	suma_read += HAL_GPIO_ReadPin(SW_3_GPIO_Port, SW_3_Pin) ? 0 : FREQ_STEP * 8;
	suma_read +=
			HAL_GPIO_ReadPin(SW_4_GPIO_Port, SW_4_Pin) ? 0 : FREQ_STEP * 16;
	suma_read +=
			HAL_GPIO_ReadPin(SW_5_GPIO_Port, SW_5_Pin) ? 0 : FREQ_STEP * 32;
	suma_read +=
			HAL_GPIO_ReadPin(SW_6_GPIO_Port, SW_6_Pin) ? 0 : FREQ_STEP * 64;
	suma_read +=
			HAL_GPIO_ReadPin(SW_7_GPIO_Port, SW_7_Pin) ? 0 : FREQ_STEP * 128;
	suma_read +=
			HAL_GPIO_ReadPin(SW_8_GPIO_Port, SW_8_Pin) ? 0 : FREQ_STEP * 256;
	suma_read +=
			HAL_GPIO_ReadPin(SW_9_GPIO_Port, SW_9_Pin) ? 0 : FREQ_STEP * 512;
	return suma_read;
}

void USART1_IRQHandler(void) {
	uart1_read_to_frame(uart1_ptr);
}

void freqOutCmdUpdate(const UART1_t *uart1, MAX2871_t *ppl) {
	unsigned long receiveValue;
	receiveValue = 0;
	receiveValue = uart1->rxBuffer[4] << 24;
	receiveValue |= uart1->rxBuffer[5] << 16;
	receiveValue |= uart1->rxBuffer[6] << 8;
	receiveValue |= uart1->rxBuffer[7];
	if ((receiveValue > FREQ_OUT_MIN) && (receiveValue < FREQ_OUT_MAX)) {
		ppl->freqOut = receiveValue;
		sprintf(uart1->tx, "New Frequency Out: %u\n", receiveValue);
		uart1_send_frame(uart1->tx, TX_BUFFLEN);
		ppl->freqOutUpdate = true;
	} else {
		sprintf(uart1->tx, "OUT OF RANGE \n");
		uart1_send_frame(uart1->tx, TX_BUFFLEN);
	}
}

void freqBaseCmdUpdate(const UART1_t *uart1, MAX2871_t *ppl) {
	unsigned long receiveValue;
	receiveValue = 0;
	receiveValue = uart1->rxBuffer[4] << 24;
	receiveValue |= uart1->rxBuffer[5] << 16;
	receiveValue |= uart1->rxBuffer[6] << 8;
	receiveValue |= uart1->rxBuffer[7];
	if ((receiveValue > FREQ_BASE_MIN) && (receiveValue < FREQ_BASE_MAX)) {
		ppl->freqBase = receiveValue;
		ppl->freqOut = ppl->freqSumRead + ppl->freqBase;
		ppl->freqOutUpdate = true;
		sprintf(uart1->tx, "New Base Frequency: %u\n", receiveValue);
		uart1_send_frame(uart1->tx, TX_BUFFLEN);
	} else {
		sprintf(uart1->tx, "OUT OF RANGE \n");
		uart1_send_frame(uart1->tx, TX_BUFFLEN);
	}
}

void printParameters(const UART1_t *uart1, MAX2871_t *ppl) {
	char *tx;
	tx = (char*) uart1->tx;
	unsigned long out = (unsigned long) ppl->freqOut;
	unsigned long base = (unsigned long) ppl->freqBase;
	sprintf(tx, "Frequency: %lu\nBase Frequency: %lu\n", out, base);
	if (ppl->register4.APWR == 0)
		strcat(tx, "Power out: -4 [dBm]\n");
	if (ppl->register4.APWR == 1)
		strcat(tx, "Power out: -1 [dBm]\n");
	if (ppl->register4.APWR == 2)
		strcat(tx, "Power out: +2 [dBm]\n");
	if (ppl->register4.APWR == 3)
		strcat(tx, "Power out: +5 [dBm]\n");
	if (ppl->hibridMode == 0)
		strcat(tx, "SWITCH INITIALIZATION\n");
	if (ppl->hibridMode == 1)
		strcat(tx, "EEPROM INITIALIZATION\n");
	uart1_send_frame(tx, TX_BUFFLEN);
}

void ParametersCmd(const UART1_t *uart1, MAX2871_t *ppl) {
	unsigned long receiveValue;
	receiveValue = 0;
	receiveValue = uart1->rxBuffer[4] << 24;
	receiveValue |= uart1->rxBuffer[5] << 16;
	receiveValue |= uart1->rxBuffer[6] << 8;
	receiveValue |= uart1->rxBuffer[7];
	if (receiveValue == 0) {
		printParameters(uart1, ppl);
	}
}

void setModeCmd(const UART1_t *uart1, MAX2871_t *ppl) {
	unsigned long receiveValue;
	receiveValue = 0;
	receiveValue = uart1->rxBuffer[4] << 24;
	receiveValue |= uart1->rxBuffer[5] << 16;
	receiveValue |= uart1->rxBuffer[6] << 8;
	receiveValue |= uart1->rxBuffer[7];
	ppl->hibridMode = receiveValue;
	if (ppl->hibridMode == 0) { // Modo switch
		HIBRID_MODE_OFF_LED();
		sprintf(uart1->tx, "SWITCH INITIALIZATION\n");
		uart1_send_frame(uart1->tx, TX_BUFFLEN);

	}
	if (ppl->hibridMode == 1) { // Modo hibrido
		HIBRID_MODE_ON_LED();
		sprintf(uart1->tx, "EEPROM INITIALIZATION\n");
		uart1_send_frame(uart1->tx, TX_BUFFLEN);
	}

}

void powerOutCmdUpdate(const UART1_t *uart1, MAX2871_t *ppl) {
	unsigned long receiveValue;
	receiveValue = 0;
	receiveValue = uart1->rxBuffer[4] << 24;
	receiveValue |= uart1->rxBuffer[5] << 16;
	receiveValue |= uart1->rxBuffer[6] << 8;
	receiveValue |= uart1->rxBuffer[7];
	if (receiveValue == 0) {
		//Power out -4dBm
		ppl->register4.APWR = 0x0UL;
		ppl->freqOutUpdate = true;
		sprintf(uart1->tx, "PdBm out = -4[dBm] \n");
		uart1_send_frame(uart1->tx, TX_BUFFLEN);
	}
	if (receiveValue == 1) {
		//Power out -1dBm
		ppl->register4.APWR = 0x1UL;
		ppl->freqOutUpdate = true;
		sprintf(uart1->tx, "PdBm out = -1[dBm] \n");
		uart1_send_frame(uart1->tx, TX_BUFFLEN);
	}
	if (receiveValue == 2) {
		//Power out +2dBm
		ppl->register4.APWR = 0x2UL;
		ppl->freqOutUpdate = true;
		sprintf(uart1->tx, "PdBm out = +2[dBm] \n");
		uart1_send_frame(uart1->tx, TX_BUFFLEN);
	}
	if (receiveValue == 3) {
		//Power out +5dBm
		ppl->register4.APWR = 0x3UL;
		ppl->freqOutUpdate = true;
		sprintf(uart1->tx, "PdBm out = +5[dBm] \n");
		uart1_send_frame(uart1->tx, TX_BUFFLEN);
	}
}

void freqOutRs485Update(const UART1_t *uart1, RS485_t *rs485, MAX2871_t *ppl) {
	unsigned long receiveValue;
	switch (rs485->cmd) {
	case SET_PARAMETER_FREQOUT: //cmd = 31
		freqOutCmdUpdate(uart1, ppl);
		rs485->cmd = NONE;
		break;
	case SET_PARAMETERS: //cmd = 32
		ParametersCmd(uart1, ppl);
		rs485->cmd = NONE;
		break;
	case SET_PARAMETER_FREQBASE: //cmd = 33
		freqBaseCmdUpdate(uart1, ppl);
		rs485->cmd = NONE;
		break;
	case QUERY_PARAMETER_PdBm: //cmd = 34
		powerOutCmdUpdate(uart1, ppl);
		rs485->cmd = NONE;
		break;
	case SET_MODE: //cmd = 35
		setModeCmd(uart1, ppl);
		save(MODE_ADDR, (uint8_t*) (&ppl->hibridMode), 0,
		FREQ_OUT_SIZE);
		rs485->cmd = NONE;
		break;
	default:
		rs485->cmd = NONE;
		break;
	}
}

void freqOutSWUpdate(const UART1_t *uart1, MAX2871_t *ppl) {

	ppl->freqSumRead = getFreqSum(ppl->freqBase);
	if (ppl->freqSumRead != ppl->freqSumNew) {
		ppl->lastFreqSumReadTick = HAL_GetTick();
		FREQ_CHANGING_ON_LED();
		ppl->freqSumNew = ppl->freqSumRead;
	}

	if ((HAL_GetTick() - ppl->lastFreqSumReadTick) > SW_DEBOUNCE) {
		if (ppl->freqSumNew != ppl->freqSumCurrent) {
			ppl->freqOut = ppl->freqSumNew + ppl->freqBase;
			sprintf(uart1->tx, "New Frequency Out: %u\n", ppl->freqOut);
			uart1_send_frame(uart1->tx, TX_BUFFLEN);
			ppl->freqOutUpdate = true;
			FREQ_CHANGING_OFF_LED();
			ppl->freqSumCurrent = ppl->freqSumNew;
		}
	}
}

void getParametersFromEeprom(MAX2871_t *ppl) {
	ppl->freqBase = getULFromEeprom(FREQ_BASE_ADDR);
	if ((ppl->freqBase < FREQ_BASE_MIN) || (ppl->freqBase > FREQ_BASE_MAX))
		ppl->freqBase = FREQ_BASE_DEFAULT;

	ppl->register4.APWR = getULFromEeprom(POUT_ADDR);
	if ((ppl->register4.APWR < 0x0UL) || (ppl->register4.APWR > 0x3UL))
		ppl->register4.APWR = 0x1UL;

	ppl->hibridMode = getULFromEeprom(MODE_ADDR);
	if ((ppl->hibridMode < 0) || (ppl->hibridMode > 1))
		ppl->hibridMode = 0;

	ppl->freqSumNew = getFreqSum(ppl->freqBase);
	ppl->freqSumRead = ppl->freqSumNew;
	ppl->freqSumCurrent = ppl->freqSumNew;
	ppl->freqOut = FREQ_BASE_DEFAULT + ppl->freqSumCurrent;
	if (ppl->hibridMode == 1) {
		HIBRID_MODE_ON_LED();
		ppl->freqOut = getULFromEeprom(FREQ_OUT_ADDR);
		if ((ppl->freqOut < FREQ_OUT_MIN) || (ppl->freqOut > FREQ_OUT_MAX))
			ppl->freqOut = FREQ_BASE_DEFAULT + ppl->freqSumCurrent;
	}
}

void saveParameters(MAX2871_t *ppl) {
	save(FREQ_OUT_ADDR, (uint8_t*) (&ppl->freqOut), 0,
	FREQ_OUT_SIZE);
	save(FREQ_BASE_ADDR, (uint8_t*) (&ppl->freqBase), 0,
	FREQ_OUT_SIZE);
	save(POUT_ADDR, (uint8_t*) (&ppl->register4.APWR), 0,
	FREQ_OUT_SIZE);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	RS485_t rs485;
	MAX2871_t ppl;
	LED_t led;
	UART1_t uart1;
	Tone_uhf_t uhf;
	ppl_ptr = &ppl;
	uart1_ptr = &uart1;
	uhf_ptr = &uhf;
	rs485_ptr = &rs485;
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
	//MX_USART1_UART_Init();
	MX_CRC_Init();
	//MX_IWDG_Init();
	/* USER CODE BEGIN 2 */
	toneUhfInit(UHF_TONE, ID0, &uhf);
	rs485Init(&rs485);
	ledInit(&led);
	i2c1MasterInit();
	uart1Init(HS16_CLK, BAUD_RATE, &uart1);
	max2871Init(&ppl);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	max2871RegisterInit(&hspi2, &ppl); //
	getParametersFromEeprom(&ppl);
	max2871ProgramFreqOut(&hspi2, &ppl);
	HAL_GPIO_WritePin(GPIOA, MAX_RF_ENABLE_Pin, GPIO_PIN_SET);
	printParameters(&uart1, &ppl);

	while (1) {

		freqOutSWUpdate(&uart1, &ppl);
		rs485Uart1Decode(&rs485, &uart1);
		freqOutRs485Update(&uart1, &rs485, &ppl);

		if (ppl.freqOutUpdate) {
			ppl.freqOutUpdate = false;
			max2871ProgramFreqOut(&hspi2, &ppl);
			save(POUT_ADDR, (uint8_t*) (&ppl.register4.APWR), 0,
			FREQ_OUT_SIZE);

			if (ppl.hibridMode == 1)
				saveParameters(&ppl);
		}

		led_enable_kalive(&led);
		//HAL_IWDG_Refresh(&hiwdg);
	}
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void) {

	/* USER CODE BEGIN IWDG_Init 0 */

	/* USER CODE END IWDG_Init 0 */

	/* USER CODE BEGIN IWDG_Init 1 */

	/* USER CODE END IWDG_Init 1 */
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
	hiwdg.Init.Window = 4095;
	hiwdg.Init.Reload = 1249;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN IWDG_Init 2 */

	/* USER CODE END IWDG_Init 2 */

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
