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

#define FREQ_OUT_SIZE 8
#define LACT_SIZE 2
#define MCPADR_SIZE 2

typedef struct REGISTER0 {
	//Register 0

	unsigned long INT;     //Enables fractional-N mode
	unsigned long NDIV;    // Integer part from N-Divider
	unsigned long FRAC;
	unsigned long ADDR0;
} REGISTER0_t;

typedef struct REGISTER1 {
	//Register 1

	unsigned long CPL;     //Charge pump liniarity 30%
	unsigned long CPT;     //Charge pump test mode  normal mode
	unsigned long PHASE;   //Phase Value (recomened)
	unsigned long MODULUS; //4000 for max resolution
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
	//charge pump output high-impedance mode disabled
	unsigned long RST;
	// counter reset mode  normal operation
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
	unsigned long long FreqOut;
	unsigned long long FreqOUTold;
	unsigned long long FMIN;
	unsigned long long FMAX;
	float LMAX;
	float LMIN;
	float LACT;
	float ATT;
	float CAL;
	unsigned long DIVA;
	REGISTER0_t register0;
	REGISTER1_t register1;
	REGISTER2_t register2;
	REGISTER3_t register3;
	REGISTER4_t register4;
	REGISTER5_t register5;

	int MCPADR;
	int ATTREGADR;
	int CALREGADR;
	unsigned int CALBYTE;
	unsigned int ATTBYTE;

} MAX2871_t;

void max2871Init();
void max2871CalculateRegisterValues(MAX2871_t *ppl); //calculates values of NDIV, FRAC & DIVA
unsigned long max2871RegisterInit(SPI_HandleTypeDef *hspi2, MAX2871_t *ppl);
void max2871Read();
void max2871Write(SPI_HandleTypeDef *hspi2, unsigned long data);
void max2871Program(SPI_HandleTypeDef *hspi2, MAX2871_t *ppl);
#endif /* INC_MAX2871_H_ */
