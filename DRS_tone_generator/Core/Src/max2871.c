/*
 * max2871.c
 *
 *  Created on: Jan 25, 2023
 *      Author: artur
 */

#include "max2871.h"

unsigned long getRegister0InitValue(MAX2871_t *ppl) {

	ppl->register0.INT = 0x0UL;   // Enables fractional-N mode
	ppl->register0.NDIV = 0x0UL; // Sets integer part (N-divider) of the feedback divider factor
	ppl->register0.FRAC = 0x0UL;  // Sets fractional value
	ppl->register0.ADDR0 = 0x0UL; // Register address bits

	return ppl->register0.INT << 31 | ppl->register0.NDIV << 15
			| ppl->register0.FRAC << 3 | ppl->register0.ADDR0;
}

unsigned long getRegister1InitValue(MAX2871_t *ppl) {

	ppl->register1.CPL = 0x3UL;        // Charge pump linearity 30%
	ppl->register1.CPT = 0x00UL;       // Charge pump test mode  normal mode
	ppl->register1.PHASE = 0x1UL;      // Phase Value (recommended)
	ppl->register1.MODULUS = 0xFA0UL;  // 4000 for max resolution
	ppl->register1.ADDR1 = 0x1UL;      // Register address bits

	return ppl->register1.CPL << 29 | ppl->register1.CPT << 27
			| ppl->register1.PHASE << 15 | ppl->register1.MODULUS << 3
			| ppl->register1.ADDR1;
}

unsigned long getRegister2InitValue(MAX2871_t *ppl) {

	ppl->register2.LDS = 0x1UL;    // 1 if fPFD > 32 MHz
	ppl->register2.SDN = 0x0UL;    // Noise mode  Low-noise mode
	ppl->register2.MUX = 0x6UL;    // MUX pin configuration  Digital lock detect
	ppl->register2.DBR = 0x0UL;    // Reference doubler is disabled
	ppl->register2.RDIV2 = 0x0UL;  // Reference divide-by-2 is disabled
	ppl->register2.RCNT = 0x0UL;   // Reference divide Value is unused
	ppl->register2.REG4DB = 0x0UL; // Double buffer mode disabled
	ppl->register2.CP = 0x00UL; // Charge pump current  0.32 mA (1.36/RSET * (1 + CP[3:0]) RSET  5k1)
	ppl->register2.LDF = 0x0UL;    // Lock detect function  Frac-N lock detect
	ppl->register2.LDP = 0x0UL;    // Lock detect precision  10ns
	ppl->register2.PDP = 0x1UL;    // Phase detector polarity set positive
	ppl->register2.SHDN = 0x0UL;   // Sets power-down mode.
	ppl->register2.TRI = 0x0UL;    // Sets charge-pump three-state mode
	ppl->register2.RST = 0x0UL;    // Sets counter reset mode
	ppl->register2.ADDR2 = 0x2UL;  // Register address bits

	return ppl->register2.LDS << 31 | ppl->register2.SDN << 29
			| ppl->register2.MUX << 26 | ppl->register2.DBR << 25
			| ppl->register2.RDIV2 << 24 | ppl->register2.RCNT << 14
			| ppl->register2.REG4DB << 13 | ppl->register2.CP << 9
			| ppl->register2.LDF << 8 | ppl->register2.LDP << 7
			| ppl->register2.PDP << 6 | ppl->register2.SHDN << 5
			| ppl->register2.TRI << 4 | ppl->register2.RST << 3
			| ppl->register2.ADDR2;
}

unsigned long getRegister3InitValue(MAX2871_t *ppl) {

	ppl->register3.VCO_MS = 0x0UL;   // VCO manual selection: unused
	ppl->register3.VAS_SHDN = 0x0UL; // VAS enabled
	ppl->register3.RETUNE = 0x1UL;   // VAS temperature compensation enabled
	ppl->register3.CSM = 0x0UL;      // Cycle slip mode disabled
	ppl->register3.MUTEDEL = 0x0UL;  // Mute delay mode disabled
	ppl->register3.CDM = 0x1UL;      // Fast-lock mode enabled
	ppl->register3.CDIV = 0x0UL;     // Clock divider value unused
	ppl->register3.ADDR3 = 0x3UL;    // Register address bits

	return ppl->register3.VCO_MS << 26 | ppl->register3.VAS_SHDN << 25
			| ppl->register3.RETUNE << 24 | ppl->register3.CSM << 18
			| ppl->register3.MUTEDEL << 17 | ppl->register3.CDM << 15
			| ppl->register3.CDIV << 3 | ppl->register3.ADDR3;
}

unsigned long getRegister4InitValue(MAX2871_t *ppl) {

	ppl->register4.RES = 0x3UL;    // Reserved
	ppl->register4.SDLDO = 0x0UL;  // LDO enabled
	ppl->register4.SDDIV = 0x0UL;  // VCO Divider enabled
	ppl->register4.SDREF = 0x0UL;  // Reference input enabled
	ppl->register4.FB = 0x1UL;     // VCO to N counter mode is NOT divided
	ppl->register4.BS = 0x30FFUL; // Should be chosen so that fPFD/BS  50kH or less
	ppl->register4.SDVCO = 0x0UL;  // VCO enabled
	ppl->register4.MTLD = 0x0UL;   // RFOUT Mute until Lock detect mode disabled
	ppl->register4.BDIV = 0x0UL; // RFOUTB is divided (so it's the same as RFOUTA)
	ppl->register4.RFB_EN = 0x0UL; // RFOUTB disabled
	ppl->register4.BPWR = 0x3UL;   // RFOUTB  5 dBm
	ppl->register4.RFA_EN = 0x1UL; // RFOUTA enabled
	ppl->register4.APWR = 0x0UL; // Sets RFOUTA single-ended output power   (00 = -4dBm)
								 //(01 = -1dBm)
								 //(10 = +2dBm)
								 //(11 = +5dBm)
	ppl->register4.DIVA = 0x0UL;
	ppl->register4.ADDR4 = 0x4UL;  // Register address bits

	return ppl->register4.RES << 29 | ppl->register4.SDLDO << 28
			| ppl->register4.SDDIV << 27 | ppl->register4.SDREF << 26
			| ppl->register4.FB << 23 | ppl->register4.DIVA << 20
			| ppl->register4.BS << 12 | ppl->register4.SDVCO << 11
			| ppl->register4.MTLD << 10 | ppl->register4.BDIV << 9
			| ppl->register4.RFB_EN << 8 | ppl->register4.BPWR << 6
			| ppl->register4.RFA_EN << 5 | ppl->register4.APWR << 3
			| ppl->register4.ADDR4;
}

unsigned long getRegister5InitValue(MAX2871_t *ppl) {

	ppl->register5.VAS_DLY = 0x3UL;  // 0x0 if VAS_TEMP  0, 0x3 if VAS_TEMP  1
	ppl->register5.SDPLL = 0x0UL;    // PLL enabled
	ppl->register5.F01 = 0x1UL;      // If F  0 then int
	ppl->register5.LD = 0x1UL;       // Lock-Detect pin function  HIGH
	ppl->register5.MUX_MSB = 0x0UL;  // MSB of MUX
	ppl->register5.ADCS = 0x0UL;     // ADC normal operation (ADC isn't used)
	ppl->register5.ADCM = 0x0UL;     // ADC disabled
	ppl->register5.ADDR5 = 0x5UL;    // Register address bits

	return ppl->register5.VAS_DLY << 29 | ppl->register5.SDPLL << 25
			| ppl->register5.F01 << 24 | ppl->register5.LD << 22
			| ppl->register5.MUX_MSB << 18 | ppl->register5.ADCS << 6
			| ppl->register5.ADCM << 3 | ppl->register5.ADDR5;
}

void max2871Init(MAX2871_t *ppl) {
	// Composition of MAX2971 Registers
	ppl->freqSumRead = 0x0UL;
	ppl->freqBase = 0x0UL;
	ppl->freqOutUpdate = false;
	ppl->lastFreqSumReadTick = HAL_GetTick();
}

void max2871Write(SPI_HandleTypeDef *hspi2, unsigned long data)
// Writes 32 Bit value to register of MAX2871
{
	uint8_t buffer[4] = { 0 };

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

void max2871CalculateRegister0Values(MAX2871_t *ppl) // Calculates values of NDIV, FRAC & DIVA
{
	double rest;
	unsigned long FreqRef = 50000000.0;  // FreqRef * 5
	unsigned long Resol = 4000.0;

	if (ppl->freqOut >= 3000000000) {
		ppl->register4.DIVA = 0;
		ppl->register0.NDIV = ppl->freqOut / FreqRef;
		rest = ppl->freqOut % FreqRef;
		ppl->register0.FRAC = rest / FreqRef * Resol;
	} else if ((ppl->freqOut < 3000000000) && (ppl->freqOut >= 1500000000)) {
		ppl->register4.DIVA = 1;
		ppl->register0.NDIV = ppl->freqOut * 2 / FreqRef;
		rest = ppl->freqOut * 2 % FreqRef;
		ppl->register0.FRAC = rest / FreqRef * Resol;
	} else if ((ppl->freqOut < 1500000000) && (ppl->freqOut >= 750000000)) {
		ppl->register4.DIVA = 2;
		ppl->register0.NDIV = ppl->freqOut * 4 / FreqRef;
		rest = ppl->freqOut * 4 % FreqRef;
		ppl->register0.FRAC = rest / FreqRef * Resol;
	} else if ((ppl->freqOut < 750000000) && (ppl->freqOut >= 375000000)) {
		ppl->register4.DIVA = 3;
		ppl->register0.NDIV = ppl->freqOut * 8 / FreqRef;
		rest = ppl->freqOut * 8 % FreqRef;
		ppl->register0.FRAC = rest / FreqRef * Resol;
	} else if ((ppl->freqOut < 375000000) && (ppl->freqOut >= 187500000)) {
		ppl->register4.DIVA = 4;
		ppl->register0.NDIV = ppl->freqOut * 16 / FreqRef;
		rest = ppl->freqOut * 16 % FreqRef;
		ppl->register0.FRAC = rest / FreqRef * Resol;
	} else if ((ppl->freqOut < 187500000) && (ppl->freqOut >= 93750000)) {
		ppl->register4.DIVA = 5;
		ppl->register0.NDIV = ppl->freqOut * 32 / FreqRef;
		rest = ppl->freqOut * 32 % FreqRef;
		ppl->register0.FRAC = rest / FreqRef * Resol;
	} else if ((ppl->freqOut < 93750000) && (ppl->freqOut >= 46875000)) {
		ppl->register4.DIVA = 6;
		ppl->register0.NDIV = ppl->freqOut * 64 / FreqRef;
		rest = ppl->freqOut * 64 % FreqRef;
		ppl->register0.FRAC = rest / FreqRef * Resol;
	} else {
		ppl->register4.DIVA = 7;
		ppl->register0.NDIV = ppl->freqOut * 128 / FreqRef;
		rest = ppl->freqOut * 128 % FreqRef;
		ppl->register0.FRAC = rest / FreqRef * Resol;
	}
}

void max2871RegisterInit(SPI_HandleTypeDef *hspi2, MAX2871_t *ppl) {

	for (int i = 0; i < 2; i++) {
		max2871Write(hspi2, getRegister5InitValue(ppl));
		HAL_Delay(20);
		max2871Write(hspi2, getRegister4InitValue(ppl));
		max2871Write(hspi2, getRegister3InitValue(ppl));
		max2871Write(hspi2, getRegister2InitValue(ppl));
		max2871Write(hspi2, getRegister1InitValue(ppl));
		max2871Write(hspi2, getRegister0InitValue(ppl));
	}
}

void waitForLock() {
	GPIO_PinState lock = GPIO_PIN_SET;
	unsigned long t_ini = HAL_GetTick();
	unsigned long test = 0;
	while (lock == GPIO_PIN_SET && (test < 2000)) {
		test = HAL_GetTick() - t_ini;
		lock = HAL_GPIO_ReadPin(MAX_LOCK_DETECTOR_GPIO_Port,
		MAX_LOCK_DETECTOR_Pin);
	}
}

void writeRegister0(MAX2871_t *ppl, SPI_HandleTypeDef *hspi2) {
	unsigned long registerValue;
	registerValue = ppl->register0.INT << 31 | ppl->register0.NDIV << 15
			| ppl->register0.FRAC << 3 | ppl->register0.ADDR0;
	max2871Write(hspi2, registerValue);
}

void writeRegister4(MAX2871_t *ppl, SPI_HandleTypeDef *hspi2) {
	unsigned long registerValue = 0;
	registerValue = ppl->register4.RES << 29 | ppl->register4.SDLDO << 28
			| ppl->register4.SDDIV << 27 | ppl->register4.SDREF << 26
			| ppl->register4.FB << 23 | ppl->register4.DIVA << 20
			| ppl->register4.BS << 12 | ppl->register4.SDVCO << 11
			| ppl->register4.MTLD << 10 | ppl->register4.BDIV << 9
			| ppl->register4.RFB_EN << 8 | ppl->register4.BPWR << 6
			| ppl->register4.RFA_EN << 5 | ppl->register4.APWR << 3
			| ppl->register4.ADDR4;
	max2871Write(hspi2, registerValue);
}

void max2871ProgramFreqOut(SPI_HandleTypeDef *hspi2, MAX2871_t *ppl) // Compose register value of register 0 and 4
{
	double rest;
	HAL_GPIO_WritePin(GPIOA, MAX_RF_ENABLE_Pin, GPIO_PIN_RESET);
	ppl->register4.RFA_EN = 0x0UL; // Disabled
	writeRegister4(ppl, hspi2);
	ppl->register0.NDIV = ppl->freqOut * 32 / (unsigned long) FREQ_REF;
	rest = ppl->freqOut * 32 % (unsigned long) FREQ_REF;
	ppl->register0.FRAC = rest / (unsigned long) FREQ_REF * RESOLUTION;
	writeRegister0(ppl, hspi2);
	waitForLock();
	ppl->register4.DIVA = 5;
	ppl->register4.RFA_EN = 0X1UL; // Enabled
	writeRegister4(ppl, hspi2);
	HAL_GPIO_WritePin(GPIOA, MAX_RF_ENABLE_Pin, GPIO_PIN_SET);
}
