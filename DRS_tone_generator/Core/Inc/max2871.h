/*
 * max2871.h
 *
 *  Created on: Jan 25, 2023
 *      Author: artur
 */

#ifndef INC_MAX2871_H_
#define INC_MAX2871_H_

#include "main.h"

#define LE_LOW()  CLEAR_BIT(GPIOA->ODR, GPIO_ODR_OD1)
#define LE_HIGH() SET_BIT(GPIOA->ODR, GPIO_ODR_OD1)

//composition of MAX2871 Registers

//Register 1
#define CPL  0x3UL //Charge pump liniarity 30%
#define CPT  0x00UL //Charge pump test mode  normal mode
#define PHASE  0x1UL //Phase Value (recomened)
#define MODULUS  0xFA0UL //4000 for max resolution
#define ADDR1  0x1UL

//Register 2
#define LDS  0x1UL
// 1 if fPFD > 32 MHz
#define SDN  0x0UL
//noise mode  Low-noise mode
#define MUX  0x6UL
//MUX pin configuration  Digital lock detect
#define DBR  0x0UL
//reference doubler is disabled
#define RDIV2  0x0UL
//reference divide-by-2 is disabled
#define RCNT  0x0UL
// reference divide Value is unused
#define REG4DB  0x0UL
//double buffer mode disabled
#define CP  0x00UL
//charge pump current  0.32 mA (1.36/RSET * (1 + CP[3:0]) RSET  5k1)
#define LDF  0x0UL
// lock dtect function  Frac-N lock detect
#define LDP  0x0UL
// lock detect precision  10ns
#define PDP  0x1UL
//phase detector polarity set poitive
#define SHDN  0x0UL
//power down mode  normal mode
#define TRI  0x0UL
//charge pump output high-impedance mode disabled
#define RST  0x0UL
// counter reset mode  normal operation
#define ADDR2  0x2UL

//Register 3
#define VCO_MS  0x0UL
// VCO maual selction: unused
#define VAS_SHDN  0x0UL
//VAS enabled
#define VAS_TEMP  0x1UL
//VAS temperature compensation enabled
#define CSM  0x0UL
//Cycle slip mode disabled
#define MUTEDEL  0x0UL
//mute delay mode disabled
#define CDM  0x1UL
// Fast-lock mode enabled
#define CDIV  0x0UL
// clock divider value unused
#define ADDR3  0x3UL

//Register 4
#define RES  0x3UL
//Reserved
#define SDLDO  0x0UL
//LDO endabled
#define SDDIV  0x0UL
//VCO Divider enabled
#define SDREF  0x0UL
//Reference input enabled
#define FB  0x1UL
//VCO to N counter mode is NOT divided

#define BS  0x30FFUL
//shoud be choosen so that fPFD/BS  50kH or less
#define SDVCO  0x0UL
//VCO enabled
#define MTLD  0x0UL
//RFOUT Mute until Lock detet mode disabled
#define BDIV  0x0UL
//RFOUTB is divided (so it's the same as RFOUTA)
#define RFB_EN  0x1UL
//RFOUTB enabled
#define BPWR  0x3UL
//RFOUTB  5 dBm
#define RFA_EN  0x1UL
//RFOUTA enabled
#define APWR  0x3UL
//RFOUTA  5dBm
#define ADDR4  0x4UL

//Register 5
#define VAS_DLY  0x3UL
//0x0 if VAS_TEMP  0, 0x3 if VAS_TEMP  1
#define SDPLL  0x0UL
// PLL enabled
#define F01  0x1UL
// if F  0 then int
#define LD  0x3UL
//Lock-Detect pin function  HIGH
#define MUX_MSB  0x0UL
//MSB of MUX
#define ADCS  0x0UL
//ADC normal operation (ADC isn't used)
#define ADCM  0x0UL
//ADC disabled
#define ADDR5  0x5UL

#define FREQ_OUT_SIZE 8
#define LACT_SIZE 2
#define MCPADR_SIZE 2

typedef struct REGISTER0 {
	//Register 0
	unsigned long INT; //Enables fractional-N mode
	unsigned long NDIV;
	unsigned long FRAC;
	unsigned long ADDR0;
} REGISTER0_t;

typedef struct REGISTER1 {
	//Register 0
	unsigned long INT; //Enables fractional-N mode
	unsigned long NDIV;
	unsigned long FRAC;
	unsigned long ADDR0;
} REGISTER1_t;

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

	int MCPADR;
	int ATTREGADR;
	int CALREGADR;
	unsigned int CALBYTE;
	unsigned int ATTBYTE;

} MAX2871_t;

void max2871Init();
void max2871Write(unsigned long data);
void shiftOut(uint32_t DATAPIN, uint32_t CLOCKPIN, uint32_t MSBFIRST,
		uint32_t data);
void max2871CalculateRegisterValues(MAX2871_t *ppl); //calculates values of NDIV, FRAC & DIVA
unsigned long max2871RegisterInit(MAX2871_t *ppl);
void max2871Program(MAX2871_t *ppl);
void shiftOut(uint32_t DATAPIN, uint32_t CLOCKPIN, uint32_t, uint32_t data);
#endif /* INC_MAX2871_H_ */
