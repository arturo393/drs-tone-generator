/*
 * max2871.c
 *
 *  Created on: Jan 25, 2023
 *      Author: artur
 */

#include "max2871.h"

void max2871Init(MAX2871_t *ppl) {

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
	ppl->register0.INT = 0x0UL; //Enables fractional-N mode
	ppl->register0.NDIV = 0x0UL;
	ppl->register0.FRAC = 0x0UL;
	ppl->register0.ADDR0 = 0x0UL;

	ppl->MCPADR = 0x27;
	ppl->ATTREGADR = 0x12; // ATTENUATOR, MCP REG A
	ppl->CALREGADR = 0x13; // CALIBRATION, MCP REG B
	ppl->CALBYTE = 0x00;
	ppl->ATTBYTE = 0x00;

}

void max2871GpioInit() {
}

void max2871Write(unsigned long data)
//Writes 32 Bit value to register of MAX2871
{

	LE_LOW();

	shiftOut(MAX_DATA_Pin, MAX_SCK_Pin, 1, ((data & 0xFF000000) >> 24));
	shiftOut(MAX_DATA_Pin, MAX_SCK_Pin, 1, ((data & 0x00FF0000) >> 16));
	shiftOut(MAX_DATA_Pin, MAX_SCK_Pin, 1, ((data & 0x0000FF00) >> 8));
	shiftOut(MAX_DATA_Pin, MAX_SCK_Pin, 1, (data & 0x000000FF));

	LE_HIGH();
	HAL_Delay(50);

}

void shiftOut(uint32_t DATAPIN, uint32_t CLOCKPIN, uint32_t bitorder, uint32_t val) {
	uint8_t i;

	for (i = 0; i < 8; i++) {
		if (bitorder == 0)
			HAL_GPIO_WritePin(MAX_DATA_GPIO_Port, MAX_DATA_Pin,
					!!(val & (1 << i)));
		else
			HAL_GPIO_WritePin(MAX_DATA_GPIO_Port, MAX_DATA_Pin,
					!!(val & (1 << (7 - i))));

		HAL_GPIO_WritePin(MAX_SCK_GPIO_Port, MAX_SCK_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MAX_SCK_GPIO_Port, MAX_SCK_Pin, GPIO_PIN_RESET);
	}
}

void max2871CalculateRegisterValues(MAX2871_t *ppl) //calculates values of NDIV, FRAC & DIVA
{
	double rest;
	if (ppl->FreqOut >= 3000000000) {
		ppl->DIVA = 0;
		ppl->register0.NDIV = ppl->FreqOut / 50000000;
		rest = ppl->FreqOut % 50000000;
		ppl->register0.FRAC = rest / 50000000.0 * 4000.0;
	} else if ((ppl->FreqOut < 3000000000) && (ppl->FreqOut >= 1500000000)) {
		ppl->DIVA = 1;
		ppl->register0.NDIV = ppl->FreqOut * 2 / 50000000;
		rest = ppl->FreqOut * 2 % 50000000;
		ppl->register0.FRAC = rest / 50000000.0 * 4000.0;
	} else if ((ppl->FreqOut < 1500000000) && (ppl->FreqOut >= 750000000)) {
		ppl->DIVA = 2;
		ppl->register0.NDIV = ppl->FreqOut * 4 / 50000000;
		rest = ppl->FreqOut * 4 % 50000000;
		ppl->register0.FRAC = rest / 50000000.0 * 4000.0;
	} else if ((ppl->FreqOut < 750000000) && (ppl->FreqOut >= 375000000)) {
		ppl->DIVA = 3;
		ppl->register0.NDIV = ppl->FreqOut * 8 / 50000000;
		rest = ppl->FreqOut * 8 % 50000000;
		ppl->register0.FRAC = rest / 50000000.0 * 4000.0;
	} else if ((ppl->FreqOut < 375000000) && (ppl->FreqOut >= 187500000)) {
		ppl->DIVA = 4;
		ppl->register0.NDIV = ppl->FreqOut * 16 / 50000000;
		rest = ppl->FreqOut * 16 % 50000000;
		ppl->register0.FRAC = rest / 50000000.0 * 4000.0;
	} else if ((ppl->FreqOut < 187500000) && (ppl->FreqOut >= 93750000)) {
		ppl->DIVA = 5;
		ppl->register0.NDIV = ppl->FreqOut * 32 / 50000000;
		rest = ppl->FreqOut * 32 % 50000000;
		ppl->register0.FRAC = rest / 50000000.0 * 4000.0;
	} else if ((ppl->FreqOut < 93750000) && (ppl->FreqOut >= 46875000)) {
		ppl->DIVA = 6;
		ppl->register0.NDIV = ppl->FreqOut * 64 / 50000000;
		rest = ppl->FreqOut * 64 % 50000000;
		ppl->register0.FRAC = rest / 50000000.0 * 4000.0;
	} else {
		ppl->DIVA = 7;
		ppl->register0.NDIV = ppl->FreqOut * 128 / 50000000;
		rest = ppl->FreqOut * 128 % 50000000;
		ppl->register0.FRAC = rest / 50000000.0 * 4000.0;
	}
}

unsigned long max2871RegisterInit(MAX2871_t *ppl) {

	unsigned long composedRegisterValue;

	for (int i = 0; i < 2; i++) {
		composedRegisterValue = VAS_DLY << 29| SDPLL << 25 | F01 << 24
		| LD << 22 | MUX << 18 | ADCS << 6 | ADCM << 3 | ADDR5;

		max2871Write(composedRegisterValue);
		HAL_Delay(20);

		composedRegisterValue =
		RES << 29| SDLDO << 28 | SDDIV << 27
		| SDREF << 26 | FB << 23 | ppl->DIVA << 20 | BS << 12 | SDVCO << 11
		| MTLD << 10 | BDIV << 9 | RFB_EN << 8 | BPWR << 6 | RFA_EN << 5
		| APWR << 3 | ADDR4;

		max2871Write(composedRegisterValue);

		composedRegisterValue =
		VCO_MS << 26| VAS_SHDN << 25 | VAS_TEMP << 24
		| CSM << 18 | MUTEDEL << 17 | CDM << 15 | CDIV << 3 | ADDR3;

		max2871Write(composedRegisterValue);

		composedRegisterValue =
		LDS << 31| SDN << 29 | MUX << 26 | DBR << 25
		| RDIV2 << 24 | RCNT << 14 | REG4DB << 13 | CP << 9 | LDF << 8
		| LDP << 7 | PDP << 6 | SHDN << 5 | TRI << 4 | RST << 3 | ADDR2;

		max2871Write(composedRegisterValue);
		composedRegisterValue = CPL << 29| CPT << 27 | PHASE << 15
		| MODULUS << 3 | ADDR1;

		max2871Write(composedRegisterValue);

		composedRegisterValue = ppl->register0.INT << 31
				| ppl->register0.NDIV << 15 | ppl->register0.FRAC << 3
				| ppl->register0.ADDR0;
		max2871Write(composedRegisterValue);

	}

	return composedRegisterValue;
}

void max2871Program(MAX2871_t *ppl) // compose register value of register 0 and 4
{

	unsigned long composedRegisterValue = 0;
	max2871CalculateRegisterValues(ppl);

	composedRegisterValue = ppl->register0.INT << 31 | ppl->register0.NDIV << 15
			| ppl->register0.FRAC << 3 | ppl->register0.ADDR0;

	max2871Write(composedRegisterValue);

	composedRegisterValue = RES << 29|SDLDO << 28|SDDIV << 27
	| SDREF << 26|FB << 23|ppl->DIVA << 20|BS << 12
	|SDVCO << 11|MTLD << 10|BDIV << 9|RFB_EN << 8
	|BPWR << 6|RFA_EN << 5|APWR << 3|ADDR4;

	max2871Write(composedRegisterValue);

}
