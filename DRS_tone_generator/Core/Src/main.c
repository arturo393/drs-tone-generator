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
#define FREQ_BASE_MIN 142500000UL
#define FREQ_BASE_MAX 148412500UL
#define FREQ_OUT_MIN 142500000UL
#define FREQ_OUT_MAX 161200000UL
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

MAX2871_t *ppl_ptr;
UART1_t *uart1_ptr;
Tone_uhf_t *uhf_ptr;
RS485_t *rs485_ptr;

void getFreqOutFromEeprom(uint8_t buffer[10], MAX2871_t *ppl) {
	m24c64ReadNBytes(BASE_ADDR, buffer, FREQ_OUT_ADDR, FREQ_OUT_SIZE);
	for (int i = FREQ_OUT_SIZE - 1; i >= 0; i--)
		ppl->FreqOut |= (buffer[i] << (i * 8));
}

unsigned long getFreqOut(unsigned long FreqBase) {
	unsigned long suma_read;
	suma_read = FreqBase;
	suma_read += HAL_GPIO_ReadPin(SW_0_GPIO_Port, SW_0_Pin) ? 0 : FREQ_STEP;
	suma_read += HAL_GPIO_ReadPin(SW_1_GPIO_Port, SW_1_Pin) ? 0 : FREQ_STEP * 2;
	suma_read += HAL_GPIO_ReadPin(SW_2_GPIO_Port, SW_2_Pin) ? 0 : FREQ_STEP * 4;
	suma_read += HAL_GPIO_ReadPin(SW_3_GPIO_Port, SW_3_Pin) ? 0 : FREQ_STEP * 8;
	suma_read += HAL_GPIO_ReadPin(SW_4_GPIO_Port, SW_4_Pin) ? 0 : FREQ_STEP * 16;
	suma_read += HAL_GPIO_ReadPin(SW_5_GPIO_Port, SW_5_Pin) ? 0 : FREQ_STEP * 32;
	suma_read += HAL_GPIO_ReadPin(SW_6_GPIO_Port, SW_6_Pin) ? 0 : FREQ_STEP * 64;
	suma_read += HAL_GPIO_ReadPin(SW_7_GPIO_Port, SW_7_Pin) ? 0 : FREQ_STEP * 128;
	suma_read += HAL_GPIO_ReadPin(SW_8_GPIO_Port, SW_8_Pin) ? 0 : FREQ_STEP * 256;
	suma_read += HAL_GPIO_ReadPin(SW_9_GPIO_Port, SW_9_Pin) ? 0 : FREQ_STEP * 512;
	return suma_read;
}

void USART1_IRQHandler(void) {
	uart1_read_to_frame(uart1_ptr);
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
	ppl_ptr = &ppl;
	LED_t led;
	uint8_t buffer[10] = { 0 };
	UART1_t uart1;
	Tone_uhf_t uhf;
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
	//MX_I2C1_Init();
	MX_SPI2_Init();
	//MX_USART1_UART_Init();
	//MX_CRC_Init();
	/* USER CODE BEGIN 2 */
	toneUhfInit(UHF_TONE, ID0, &uhf);

	rs485_init(&rs485);
	led_init(&led);
	max2871Init(&ppl);
	max2871RegisterInit(&hspi2, &ppl);
	i2c1MasterInit();
	uart1_init(HS16_CLK, BAUD_RATE, &uart1);
	getFreqOutFromEeprom(buffer, &ppl);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	unsigned long freqOutCurrent = -1;
	unsigned long freqOutRead;
	unsigned long freqOutNew = 0;
	unsigned long lastReadTick = HAL_GetTick();
	unsigned long FreqBase = 145000000;
	unsigned long ON_OFF;
	unsigned long FreqOutCh;
	unsigned long FreqBaseCh;
	unsigned long PdBmCh;

	HAL_GPIO_WritePin(GPIOA, MAX_RF_ENABLE_Pin, GPIO_PIN_SET);

	while (1) {

		led_enable_kalive(&led);
		freqOutRead = getFreqOut(FreqBase);

		if (freqOutRead != freqOutNew) {
			sprintf(uart1.tx_buffer, "Frequency: %u\n", freqOutRead);
			uart1_send_frame(uart1.tx_buffer, TX_BUFFLEN);
			Change_end_off();
			lastReadTick = HAL_GetTick();
			HAL_GPIO_WritePin(GPIOA, MAX_RF_ENABLE_Pin, GPIO_PIN_RESET);
			Freq_changing_on();
			freqOutNew = freqOutRead;
		}

		if ((HAL_GetTick() - lastReadTick) > 100) {
			if (freqOutNew != freqOutCurrent) {
				ppl.FreqOut = freqOutRead;
				max2871Program(&hspi2, &ppl);
				HAL_GPIO_WritePin(GPIOA, MAX_RF_ENABLE_Pin, GPIO_PIN_SET);
				Freq_changing_off();
				Change_end_on();
				freqOutCurrent = freqOutNew;
			}
		}

		rs485_update_status_by_uart(&rs485, &uart1);

		switch (rs485.cmd) {
		case QUERY_PARAMETER_FREQOUT: //cmd = 31
			FreqOutCh = 0;
			FreqOutCh = uart1.rx_buffer[4] << 24;
			FreqOutCh |= uart1.rx_buffer[5] << 16;
			FreqOutCh |= uart1.rx_buffer[6] << 8;
			FreqOutCh |= uart1.rx_buffer[7];

			if(FreqOutCh > FREQ_OUT_MIN & FreqOutCh < FREQ_OUT_MAX){
		    ppl.FreqOut = FreqOutCh;

		    sprintf(uart1.tx_buffer, "New Frequency: %u\n", FreqOutCh);
		    uart1_send_frame(uart1.tx_buffer, TX_BUFFLEN);

			max2871Program(&hspi2, &ppl);
			}
			rs485.cmd = NONE;
			break;

		case QUERY_PARAMETER_ON_OFF: //cmd = 32
			ON_OFF = 0;
			ON_OFF = uart1.rx_buffer[4] << 24;
			ON_OFF |= uart1.rx_buffer[5] << 16;
			ON_OFF |= uart1.rx_buffer[6] << 8;
			ON_OFF |= uart1.rx_buffer[7];

			if(ON_OFF == 0){
		    HAL_GPIO_WritePin(GPIOA, MAX_RF_ENABLE_Pin, GPIO_PIN_RESET);
		    sprintf(uart1.tx_buffer, "RFA DISABLED \n");
		    uart1_send_frame(uart1.tx_buffer, TX_BUFFLEN);
			}
			if(ON_OFF == 1){
		    HAL_GPIO_WritePin(GPIOA, MAX_RF_ENABLE_Pin, GPIO_PIN_SET);
		    sprintf(uart1.tx_buffer, "RFA ENABLED \n");
		    uart1_send_frame(uart1.tx_buffer, TX_BUFFLEN);
			}
			rs485.cmd = NONE;
			break;

		case QUERY_PARAMETER_FREQBASE: //cmd = 33
			FreqBaseCh = 0;
			FreqBaseCh = uart1.rx_buffer[4] << 24;
			FreqBaseCh |= uart1.rx_buffer[5] << 16;
			FreqBaseCh |= uart1.rx_buffer[6] << 8;
			FreqBaseCh |= uart1.rx_buffer[7];

			if(FreqBaseCh > FREQ_BASE_MIN & FreqBaseCh < FREQ_BASE_MAX){
				FreqBase = FreqBaseCh;
				sprintf(uart1.tx_buffer, "New Base Frequency: %u\n", FreqBaseCh);
				uart1_send_frame(uart1.tx_buffer, TX_BUFFLEN);
			}
			rs485.cmd = NONE;
			break;

		case QUERY_PARAMETER_PdBm: //cmd = 34
			PdBmCh = 0;
			PdBmCh = uart1.rx_buffer[4] << 24;
			PdBmCh |= uart1.rx_buffer[5] << 16;
			PdBmCh |= uart1.rx_buffer[6] << 8;
			PdBmCh |= uart1.rx_buffer[7];

			if(PdBmCh == 0){     //Potencia de la salida en -4dBm
			ppl.register4.APWR = 0x0UL;
			max2871Program(&hspi2, &ppl);
			}
			if(PdBmCh == 1){     //Potencia de la salida en -1dBm
			ppl.register4.APWR = 0x1UL;
			max2871Program(&hspi2, &ppl);
			}
			if(PdBmCh == 2){     //Potencia de la salida en +2dBm
			ppl.register4.APWR = 0x2UL;
			max2871Program(&hspi2, &ppl);
		    }
			if(PdBmCh == 3){     //Potencia de la salida en +5dBm
			ppl.register4.APWR = 0x3UL;
			max2871Program(&hspi2, &ppl);
			}
			rs485.cmd = NONE;
			break;

		default:
			rs485.cmd = NONE;
			break;
		}

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
