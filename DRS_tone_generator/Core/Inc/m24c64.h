/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : m24c64.h
 * @brief          : Header for m24c64.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 *
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __M24C64_H
#define M24C64_H

#ifdef __cplusplus
extern "C" {
#endif


#include "main.h"
#include "i2c1_master.h"
#include "string.h"

#define CHIP_ADDR 0xa0
#define PAGE_SIZE 8
#define PAGE_NUM 32
#define IS_READY 0xaa
#define PADDRPOSITION 3

// EEPROM ADDRESS (8bits)
#define EEPROM_ADDR 0xA0
//#define EEPROM_ADDR 0x50

#define BASE_ADDR 0x03

/*
@page is the number of the start page. Range from 0 to PAGE_NUM-1
@offset is the start byte offset in the page. Range from 0 to PAGE_SIZE-1
@data is the pointer to the data to write in bytes
@size is the size of the data
*/
typedef enum{
	FREQ_OUT_ADDR = BASE_ADDR,
	FREQ_BASE_ADDR,
	POUT_ADDR,
	MODE_ADDR
}M24C64_ADDR_t;


typedef struct M24C64 {
	M24C64_ADDR_t  addrs;
	 uint8_t data[PAGESIZE];
} M24C64_t;

void m24c64_page_read(uint8_t address,uint8_t page, uint8_t *data);
void m24c64ReadNBytes(uint8_t page, uint8_t *data, uint8_t offset,uint8_t size);
void m24c64WriteNBytes(uint8_t page, uint8_t *data, uint8_t offset,uint8_t size);
void m24c64_init_16uvalue(M24C64_ADDR_t addr,uint16_t value);
void m24c64_store_16uvalue(M24C64_ADDR_t addr,uint16_t value);
unsigned long getULFromEeprom(uint8_t page);


#ifdef __cplusplus
}
#endif

#endif /* M24C64_H */
