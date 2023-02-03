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

	ppl->register1.CPL = 0x3UL; //Charge pump liniarity 30%
	ppl->register1.CPT = 0x00UL; //Charge pump test mode  normal mode
	ppl->register1.PHASE = 0x1UL; //Phase Value (recomened)
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
	ppl->register2.LDF = 0x0UL;		// lock dtect function  Frac-N lock detect
	ppl->register2.LDP = 0x0UL;  	// lock detect precision  10ns
	ppl->register2.PDP = 0x1UL; 	//phase detector polarity set poitive
	ppl->register2.SHDN = 0x0UL;
	ppl->register2.TRI = 0x0UL;
	ppl->register2.RST = 0x0UL;
	ppl->register2.ADDR2 = 0x2UL;

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

    buffer[0] = (data & 0xFF000000) >> 24; //ORIGINAL OJO
    buffer[1] = (data & 0x00FF0000) >> 16;
    buffer[2] = (data & 0x0000FF00) >> 8;
    buffer[3] = (data & 0x000000FF);

	HAL_GPIO_WritePin(GPIOA, MAX_CE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MAX_LE_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	//shiftOut(MAX_DATA_Pin, MAX_SCK_Pin, 1, ((data & 0xFF000000) >> 24));
	//shiftOut(MAX_DATA_Pin, MAX_SCK_Pin, 1, ((data & 0x00FF0000) >> 16));
	//shiftOut(MAX_DATA_Pin, MAX_SCK_Pin, 1, ((data & 0x0000FF00) >> 8));
    //shiftOut(MAX_DATA_Pin, MAX_SCK_Pin, 1, (data & 0x000000FF));
	HAL_SPI_Transmit(hspi2, buffer, 4, 100);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, MAX_CE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MAX_LE_Pin, GPIO_PIN_SET);
	HAL_Delay(50);
}

/*void shiftOut(uint16_t DATAPIN, uint16_t CLOCKPIN, bool MSBFIRST,
		uint8_t command) {

	for (int i = 0; i < 8; i++) {
		bool output = false;
		if (MSBFIRST) {
			output = command & 0b10000000;
			command = command << 1;
		} else {
			output = command & 0b00000001;
			command = command >> 1;
		}
		HAL_GPIO_WritePin(GPIOA, DATAPIN, output);
		HAL_GPIO_WritePin(GPIOA, CLOCKPIN, GPIO_PIN_SET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOA, CLOCKPIN, GPIO_PIN_RESET);
		HAL_Delay(1);
	}
}*/

/*void shiftOut(uint32_t DATAPIN, uint32_t CLOCKPIN, uint32_t bitorder, uint32_t val) {
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
 }*/

void max2871CalculateRegisterValues(MAX2871_t *ppl) //calculates values of NDIV, FRAC & DIVA
{
	double rest;
	unsigned long FREQREF = 10000000.0;  //REF GENERADOR X 5
	unsigned long RESOL = 800.0;        // RESOL = FREQREF/12500
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
		composedRegisterValue = VAS_DLY << 29| SDPLL << 25 | F01 << 24 //Reserva los espacios necesarios para cada variable
		| LD << 22 | ppl->register2.MUX << 18 | ADCS << 6 | ADCM << 3 | ADDR5;        //ADDR5 (direccion) = 3 bits, ADCM (modo del ADC) = 3 bits, SDPLL (setea el PLL a modo shutdown) = 1 bit
                                                                       //ADCS (inicia el ADC) = 1 bit, MUX (setea el modo del pin MUX) = 1 bit, VAS_DLY (Delay para el VCO) = 2 bits
		max2871Write(hspi2, composedRegisterValue);                    //LD (setea la funcion deteccion de bloqueo) = 2 bits, F01 (setea la forma de los enteros para F) = 1 bit
		HAL_Delay(20);

		composedRegisterValue = RES << 29| SDLDO << 28 | SDDIV << 27        //SDLDO (setea a shutdown el VCO LDO) = 1 bit, SDDIV (setea a shutdown el VCO Divider) = 1 bit
		| SDREF << 26 | FB << 23 | ppl->DIVA << 20 | BS << 12 | SDVCO << 11 //SDREF (setea a shutdown la referencia de entrada) = 1 bit, FB (setea el VCO a modo feedback) = 1 bit
		| MTLD << 10 | BDIV << 9 | RFB_EN << 8 | BPWR << 6 | RFA_EN << 5    //DIVA (divide por x la referencia de salida segun se programe) = 3 bits, SDVCO (pone el VCO en modo shutdown) =1 bit
		| APWR << 3 | ADDR4;

		max2871Write(hspi2, composedRegisterValue);

		composedRegisterValue = VCO_MS << 26| VAS_SHDN << 25 | RETUNE << 24
		| CSM << 18 | MUTEDEL << 17 | CDM << 15 | CDIV << 3 | ADDR3;

		max2871Write(hspi2, composedRegisterValue);

		composedRegisterValue = ppl->register2.LDS << 31| SDN << 29 | MUX << 26 | DBR << 25
		| RDIV2 << 24 | RCNT << 14 | REG4DB << 13 | CP << 9 | LDF << 8
		| LDP << 7 | PDP << 6 | SHDN << 5 | TRI << 4 | RST << 3 | ADDR2;

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

	unsigned long composedRegisterValue; // = 0;
	max2871CalculateRegisterValues(ppl);

	composedRegisterValue = ppl->register0.INT << 31 | ppl->register0.NDIV << 15
			| ppl->register0.FRAC << 3 | ppl->register0.ADDR0;

	max2871Write(hspi2, composedRegisterValue);

    HAL_Delay(6000); //new delay

	composedRegisterValue = RES << 29|SDLDO << 28|SDDIV << 27
	| SDREF << 26|FB << 23|ppl->DIVA << 20|BS << 12
	|SDVCO << 11|MTLD << 10|BDIV << 9|RFB_EN << 8
	|BPWR << 6|RFA_EN << 5|APWR << 3|ADDR4;

	max2871Write(hspi2, composedRegisterValue);
}
