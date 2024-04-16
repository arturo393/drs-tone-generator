/*
 * max2871.h
 *
 *  Created on: Jan 25, 2023
 *      Author: artur
 */

#ifndef INC_MAX2871_H_
#define INC_MAX2871_H_

#include "main.h"
#include "stdbool.h"

#define LE_LOW()  CLEAR_BIT(GPIOA->ODR, GPIO_ODR_OD1)
#define LE_HIGH() SET_BIT(GPIOA->ODR, GPIO_ODR_OD1)

#define FREQ_OUT_SIZE 4
#define FREQ_BASE_MIN 142500000UL
#define FREQ_BASE_MAX 148412500UL
#define FREQ_OUT_MIN 142500000UL
#define FREQ_OUT_MAX 161200000UL
#define FREQ_BASE_DEFAULT 149500000
#define FREQ_REF 50000000.0
#define RESOLUTION 4000.0


typedef struct REGISTER0 {
	//Register 0

	unsigned long INT;
	unsigned long NDIV;
	unsigned long FRAC;
	unsigned long ADDR0;
} REGISTER0_t;

typedef struct REGISTER1 {
	//Register 1

	unsigned long CPL;
	unsigned long CPT;
	unsigned long PHASE;
	unsigned long MODULUS;
	unsigned long ADDR1;
} REGISTER1_t;

typedef struct REGISTER2 {
	//Register 2

	unsigned long LDS;
	unsigned long SDN;
	unsigned long MUX;
	unsigned long DBR;
	unsigned long RDIV2;
	unsigned long RCNT;
	unsigned long REG4DB;
	unsigned long CP;
	unsigned long LDF;
	unsigned long LDP;
	unsigned long PDP;
	unsigned long SHDN;
	unsigned long TRI;
	unsigned long RST;
	unsigned long ADDR2;
} REGISTER2_t;

typedef struct REGISTER3 {
	//Register 3

	unsigned long VCO_MS;
	unsigned long VAS_SHDN;
	unsigned long RETUNE;
	unsigned long CSM;
	unsigned long MUTEDEL;
	unsigned long CDM;
	unsigned long CDIV;
	unsigned long ADDR3;
} REGISTER3_t;

typedef struct REGISTER4 {
	//Register 4

	unsigned long RES;
	unsigned long SDLDO;
	unsigned long SDDIV;
	unsigned long SDREF;
	unsigned long FB;
	unsigned long DIVA;
	unsigned long BS;
	unsigned long SDVCO;
	unsigned long MTLD;
	unsigned long BDIV;
    unsigned long RFB_EN;
	unsigned long BPWR;
	unsigned long RFA_EN;
	unsigned long APWR;
	unsigned long ADDR4;
} REGISTER4_t;

typedef struct REGISTER5 {
	//Register 5

	unsigned long VAS_DLY;
	unsigned long SDPLL;
	unsigned long F01;
	unsigned long LD;
	unsigned long MUX_MSB;
	unsigned long ADCS;
	unsigned long ADCM;
	unsigned long ADDR5;
} REGISTER5_t;

typedef struct MAX2871 {

	unsigned long long freqOut;
	unsigned long long freqSumCurrent;
	unsigned long long freqSumRead;
	unsigned long long freqSumNew;
	unsigned long long freqBase;
	unsigned long long lastFreqSumReadTick;
    bool freqOutUpdate;
    unsigned long hibridMode;

	REGISTER0_t register0;
	REGISTER1_t register1;
	REGISTER2_t register2;
	REGISTER3_t register3;
	REGISTER4_t register4;
	REGISTER5_t register5;
} MAX2871_t;

void max2871Init();
void max2871CalculateRegister0Values(MAX2871_t *ppl); // Calculates values of NDIV, FRAC & DIVA
void max2871RegisterInit(SPI_HandleTypeDef *hspi2, MAX2871_t *ppl);
void max2871Write(SPI_HandleTypeDef *hspi2, unsigned long data);
void max2871ProgramFreqOut(SPI_HandleTypeDef *hspi2, MAX2871_t *ppl);

#endif /* INC_MAX2871_H_ */
