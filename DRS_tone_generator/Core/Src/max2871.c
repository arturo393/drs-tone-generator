/*
 * max2871.c
 *
 *  Created on: Jan 25, 2023
 *      Author: artur
 */

#include "max2871.h"

void max2871Init(MAX2871_t *ppl) {
	//composition of MAX2971 Registers

	ppl->FreqOut = 0;
	ppl->FreqOUTold = 0;
	ppl->FMIN = 23499999;
	ppl->FMAX = 6000000001;
	ppl->LMAX = 12;
	ppl->LMIN = 12 - 31.5;
	ppl->LACT = 12.0;
	ppl->ATT = 0.0;
	ppl->CAL = 0.0;
	ppl->DIVA = 0x0UL;

	ppl->register0.INT = 0x0UL;     //Enables fractional-N mode
	ppl->register0.NDIV = 0x0UL;
	ppl->register0.FRAC = 0x0UL;
	ppl->register0.ADDR0 = 0x0UL;

	ppl->register1.CPL = 0x3UL;     //Charge pump liniarity 30%
	ppl->register1.CPT = 0x00UL;    //Charge pump test mode  normal mode
	ppl->register1.PHASE = 0x1UL;   //Phase Value (recomened)
	ppl->register1.MODULUS = 0xFA0UL; //4000 for max resolution
	ppl->register1.ADDR1  =   0x1UL;

	ppl->register2.LDS = 0x1UL; 	// 1 if fPFD > 32 MHz
	ppl->register2.SDN = 0x0UL; 	//noise mode  Low-noise mode
	ppl->register2.MUX = 0x6UL;		//MUX pin configuration  Digital lock detect
	ppl->register2.DBR = 0x0UL; 	//reference doubler is disabled
	ppl->register2.RDIV2 = 0x0UL; 	//reference divide-by-2 is disabled
	ppl->register2.RCNT = 0x0UL; 	// reference divide Value is unused
	ppl->register2.REG4DB = 0x0UL; 	//double buffer mode disabled
	ppl->register2.CP = 0x00UL; 	//charge pump current  0.32 mA (1.36/RSET * (1 + CP[3:0]) RSET  5k1)
	ppl->register2.LDF = 0x0UL;		// lock detect function  Frac-N lock detect
	ppl->register2.LDP = 0x0UL;  	// lock detect precision  10ns
	ppl->register2.PDP = 0x1UL; 	//phase detector polarity set positive
	ppl->register2.SHDN = 0x0UL;
	ppl->register2.TRI = 0x0UL;
	ppl->register2.RST = 0x0UL;
	ppl->register2.ADDR2 = 0x2UL;

	ppl->register3.VCO_MS = 0x0UL;		// VCO manual selection: unused
	ppl->register3.VAS_SHDN = 0x0UL; 	// VAS enabled
	ppl->register3.RETUNE = 0x1UL; 	    // VAS temperature compensation enabled
	ppl->register3.CSM = 0x0UL; 	    // Cycle slip mode disabled
	ppl->register3.MUTEDEL = 0x0UL; 	// mute delay mode disabled
	ppl->register3.CDM = 0x1UL; 	    // Fast-lock mode enabled
	ppl->register3.CDIV = 0x0UL;		// clock divider value unused
	ppl->register3.ADDR3 = 0x3UL;

	ppl->register4.RES = 0x3UL;		    // Reserved
	ppl->register4.SDLDO = 0x0UL; 	    // LDO enabled
	ppl->register4.SDDIV = 0x0UL; 	    // VCO Divider enabled
	ppl->register4.SDREF = 0x0UL; 	    // Reference input enabled
	ppl->register4.FB = 0x1UL; 	        // VCO to N counter mode is NOT divided
	ppl->register4.BS = 0x30FFUL; 	    // shoud be choosen so that fPFD/BS  50kH or less
	ppl->register4.SDVCO = 0x0UL;		// VCO enabled
	ppl->register4.MTLD = 0x0UL;        // RFOUT Mute until Lock detect mode disabled
	ppl->register4.BDIV = 0x0UL;		// RFOUTB is divided (so it's the same as RFOUTA)
	ppl->register4.RFB_EN = 0x0UL; 	    // RFOUTB enabled (modificado 6-02-23)
	ppl->register4.BPWR = 0x3UL; 	    // RFOUTB  5 dBm
	ppl->register4.RFA_EN = 0x1UL; 	    // RFOUTA enabled
	ppl->register4.APWR = 0x3UL; 	    // RFOUTA  5dBm
	ppl->register4.ADDR4 = 0x4UL;

	ppl->register5.VAS_DLY = 0x3UL; 	// 0x0 if VAS_TEMP  0, 0x3 if VAS_TEMP  1
	ppl->register5.SDPLL = 0x0UL;		// PLL enabled
	ppl->register5.F01 = 0x1UL;         // if F  0 then int
	ppl->register5.LD = 0x3UL;		    // Lock-Detect pin function  HIGH
	ppl->register5.MUX_MSB = 0x0UL; 	// MSB of MUX
	ppl->register5.ADCS = 0x0UL; 	    // ADC normal operation (ADC isn't used)
	ppl->register5.ADCM = 0x0UL; 	    // ADC disabled
	ppl->register5.ADDR5 = 0x5UL;

	ppl->MCPADR = 0x27;
	ppl->ATTREGADR = 0x12; // ATTENUATOR, MCP REG A
	ppl->CALREGADR = 0x13; // CALIBRATION, MCP REG B
	ppl->CALBYTE = 0x00;
	ppl->ATTBYTE = 0x00;

}

void max2871GpioInit() {
}

unsigned long dataIn = 0;
void max2871Read() {
	dataIn = 0;
	HAL_GPIO_WritePin(GPIOA, MAX_LE_Pin, GPIO_PIN_RESET);

	for (int i = 0; i < 29; i++) {
		HAL_GPIO_WritePin(GPIOA, MAX_SCK_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(GPIOA, MAX_SCK_Pin, GPIO_PIN_RESET);

	}
	HAL_GPIO_WritePin(GPIOA, MAX_DATA_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOA, MAX_SCK_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MAX_SCK_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOA, MAX_SCK_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MAX_SCK_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOA, MAX_DATA_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOA, MAX_SCK_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MAX_SCK_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOA, MAX_LE_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOA, MAX_SCK_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MAX_SCK_Pin, GPIO_PIN_RESET);

	for (int i = 32; i >= 0; i--) {

		bool output = HAL_GPIO_ReadPin(GPIOA, MAX_MUX_Pin);

		dataIn |= HAL_GPIO_ReadPin(GPIOA, MAX_MUX_Pin) ? 1 << i : 0 << i;

		HAL_GPIO_WritePin(GPIOA, MAX_SCK_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, MAX_SCK_Pin, GPIO_PIN_RESET);
	}
}

void max2871Write(SPI_HandleTypeDef *hspi2, unsigned long data)
//Writes 32 Bit value to register of MAX2871
{
    uint8_t buffer[4]={0};

    buffer[0] = (data & 0xFF000000) >> 24;
    buffer[1] = (data & 0x00FF0000) >> 16;
    buffer[2] = (data & 0x0000FF00) >> 8;
    buffer[3] = (data & 0x000000FF);

	HAL_GPIO_WritePin(GPIOA, MAX_CE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MAX_LE_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_Transmit(hspi2, buffer, 4, 100);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, MAX_CE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MAX_LE_Pin, GPIO_PIN_SET);
	HAL_Delay(50);
}

void max2871CalculateRegisterValues(MAX2871_t *ppl) //calculates values of NDIV, FRAC & DIVA
{
	double rest;
	unsigned long FREQREF = 50000000.0;  // FREQREF X 5
	unsigned long RESOL = 4000.0;
	if (ppl->FreqOut >= 3000000000) {
		ppl->DIVA = 0;
		ppl->register0.NDIV = ppl->FreqOut / FREQREF;
		rest = ppl->FreqOut % FREQREF;
		ppl->register0.FRAC = rest / FREQREF * RESOL;
	} else if ((ppl->FreqOut < 3000000000) && (ppl->FreqOut >= 1500000000)) {
		ppl->DIVA = 1;
		ppl->register0.NDIV = ppl->FreqOut * 2 / FREQREF;
		rest = ppl->FreqOut * 2 % FREQREF;
		ppl->register0.FRAC = rest /FREQREF * RESOL;
	} else if ((ppl->FreqOut < 1500000000) && (ppl->FreqOut >= 750000000)) {
		ppl->DIVA = 2;
		ppl->register0.NDIV = ppl->FreqOut * 4 / FREQREF;
		rest = ppl->FreqOut * 4 % FREQREF;
		ppl->register0.FRAC = rest / FREQREF * RESOL;
	} else if ((ppl->FreqOut < 750000000) && (ppl->FreqOut >= 375000000)) {
		ppl->DIVA = 3;
		ppl->register0.NDIV = ppl->FreqOut * 8 / FREQREF;
		rest = ppl->FreqOut * 8 % FREQREF;
		ppl->register0.FRAC = rest / FREQREF * RESOL;
	} else if ((ppl->FreqOut < 375000000) && (ppl->FreqOut >= 187500000)) {
		ppl->DIVA = 4;
		ppl->register0.NDIV = ppl->FreqOut * 16 / FREQREF;
		rest = ppl->FreqOut * 16 % FREQREF;
		ppl->register0.FRAC = rest / FREQREF * RESOL;
	} else if ((ppl->FreqOut < 187500000) && (ppl->FreqOut >= 93750000)) {
		ppl->DIVA = 5;
		ppl->register0.NDIV = ppl->FreqOut * 32 / FREQREF;
		rest = ppl->FreqOut * 32 % FREQREF;
		ppl->register0.FRAC = rest / FREQREF * RESOL;
	} else if ((ppl->FreqOut < 93750000) && (ppl->FreqOut >= 46875000)) {
		ppl->DIVA = 6;
		ppl->register0.NDIV = ppl->FreqOut * 64 / FREQREF;
		rest = ppl->FreqOut * 64 % FREQREF;
		ppl->register0.FRAC = rest / FREQREF * RESOL;
	} else {
		ppl->DIVA = 7;
		ppl->register0.NDIV = ppl->FreqOut * 128 / FREQREF;
		rest = ppl->FreqOut * 128 % FREQREF;
		ppl->register0.FRAC = rest / FREQREF * RESOL;
	}
}

unsigned long max2871RegisterInit(SPI_HandleTypeDef *hspi2, MAX2871_t *ppl) {

	unsigned long composedRegisterValue;

	for (int i = 0; i < 2; i++) {
		composedRegisterValue = ppl->register5.VAS_DLY << 29| ppl->register5.SDPLL << 25 | ppl->register5.F01 << 24
		| ppl->register5.LD << 22 | ppl->register5.MUX_MSB << 18 | ppl->register5.ADCS << 6 | ppl->register5.ADCM << 3 | ppl->register5.ADDR5;

		max2871Write(hspi2, composedRegisterValue);
		HAL_Delay(20);

		composedRegisterValue = ppl->register4.RES << 29| ppl->register4.SDLDO << 28 | ppl->register4.SDDIV << 27
		| ppl->register4.SDREF << 26 | ppl->register4.FB << 23 | ppl->DIVA << 20 | ppl->register4.BS << 12 | ppl->register4.SDVCO << 11
		| ppl->register4.MTLD << 10 | ppl->register4.BDIV << 9 | ppl->register4.RFB_EN << 8 | ppl->register4.BPWR << 6 | ppl->register4.RFA_EN << 5
		| ppl->register4.APWR << 3 | ppl->register4.ADDR4;

		max2871Write(hspi2, composedRegisterValue);

		composedRegisterValue = ppl->register3.VCO_MS << 26| ppl->register3.VAS_SHDN << 25 | ppl->register3.RETUNE << 24
		| ppl->register3.CSM << 18 | ppl->register3.MUTEDEL << 17 | ppl->register3.CDM << 15 | ppl->register3.CDIV << 3 | ppl->register3.ADDR3;

		max2871Write(hspi2, composedRegisterValue);

		composedRegisterValue = ppl->register2.LDS << 31| ppl->register2.SDN << 29 | ppl->register2.MUX << 26 | ppl->register2.DBR << 25
		| ppl->register2.RDIV2 << 24 | ppl->register2.RCNT << 14 | ppl->register2.REG4DB << 13 | ppl->register2.CP << 9 | ppl->register2.LDF << 8
		| ppl->register2.LDP << 7 | ppl->register2.PDP << 6 | ppl->register2.SHDN << 5 | ppl->register2.TRI << 4 | ppl->register2.RST << 3 | ppl->register2.ADDR2;

		max2871Write(hspi2, composedRegisterValue);
		composedRegisterValue = ppl->register1.CPL << 29| ppl->register1.CPT << 27 | ppl->register1.PHASE << 15
		| ppl->register1.MODULUS << 3 | ppl->register1.ADDR1;

		max2871Write(hspi2, composedRegisterValue);

		composedRegisterValue = ppl->register0.INT << 31
				| ppl->register0.NDIV << 15 | ppl->register0.FRAC << 3
				| ppl->register0.ADDR0;
		max2871Write(hspi2, composedRegisterValue);

	}

	return composedRegisterValue;
}
void max2871Program(SPI_HandleTypeDef *hspi2, MAX2871_t *ppl) // compose register value of register 0 and 4
{

	unsigned long composedRegisterValue;

	max2871CalculateRegisterValues(ppl);

	composedRegisterValue = ppl->register0.INT << 31 | ppl->register0.NDIV << 15
			| ppl->register0.FRAC << 3 | ppl->register0.ADDR0;

	max2871Write(hspi2, composedRegisterValue);

    HAL_Delay(1300); // New delay

	composedRegisterValue = ppl->register4.RES << 29|ppl->register4.SDLDO << 28|ppl->register4.SDDIV << 27
	| ppl->register4.SDREF << 26|ppl->register4.FB << 23|ppl->DIVA << 20|ppl->register4.BS << 12
	|ppl->register4.SDVCO << 11|ppl->register4.MTLD << 10|ppl->register4.BDIV << 9|ppl->register4.RFB_EN << 8
	|ppl->register4.BPWR << 6|ppl->register4.RFA_EN << 5|ppl->register4.APWR << 3|ppl->register4.ADDR4;

	max2871Write(hspi2, composedRegisterValue);
}
