#include <m24c64.h>

void m24c64_page_read(uint8_t address, uint8_t page, uint8_t *data) {
	uint8_t buff[2] = { 0 };
	uint16_t MemAddress = page << PADDRPOSITION;

	buff[0] = MemAddress >> 8;
	buff[1] = MemAddress & 0xff;

	i2c1MasterByteTx(CHIP_ADDR, buff, 2);
	i2c1MasterFrameRx(CHIP_ADDR, data, 32);
}

void readPage(uint8_t page, uint8_t *data, uint8_t offset, uint8_t size) {
	uint8_t buff[1] = { 0 };
	uint16_t MemAddress = page << PADDRPOSITION | offset;

	//buff[0] = MemAddress >> 8;
	buff[0] = MemAddress & 0xff;

	i2c1MasterByteTx(CHIP_ADDR, buff, 1);
	HAL_Delay(5);
	i2c1MasterFrameRx(CHIP_ADDR, data, size);
}

void save(uint8_t page, uint8_t *data, uint8_t offset,
		uint8_t size) {
	uint8_t buff[PAGE_SIZE + 1];
	uint8_t read[PAGE_SIZE];

	readPage(page, read, offset, size);
	bool notEqual = false;

	for (int i = 0; i < size; i++)
		if (data[i] != read[i]) {
			notEqual = true;
			break;
		}

	if (notEqual) {
		//buff[0] = (page << PADDRPOSITION | offset) >> 8;
		buff[0] = (page << PADDRPOSITION | offset) & 0xff;
		for (int i = 0; i < size; i++) {
			buff[i + 1] = data[i];
		}
		i2c1MasterByteTx(CHIP_ADDR, buff, size + 1);
	}
	HAL_Delay(6);
}

void m24c64_init_16uvalue(M24C64_ADDR_t addr, uint16_t value) {
	uint8_t buff[2];
	readPage(BASE_ADDR, buff, addr, 1);
	if (!(buff[0] == IS_READY)) {
		buff[0] = value >> 8;
		buff[1] = value & 0xff;
		save(BASE_ADDR, buff, addr + 1, 2);
	}
}

void m24c64_store_16uvalue(M24C64_ADDR_t addr, uint16_t value) {
	uint8_t buff[2];
	buff[0] = value >> 8;
	buff[1] = value & 0xff;
	save(BASE_ADDR, buff, addr + 1, 2);
	buff[0] = addr;
	save(BASE_ADDR, buff, addr, 1);
}

unsigned long getULFromEeprom(uint8_t page) {
	//uint8_t size = sizeof(unsigned long);
	uint8_t buffer[4] = { 0 };
	unsigned long readValue = 0;
	readPage(page, buffer, 0, 4);
	for (int i = 0; i < 4; i++) {
		readValue |= (buffer[i] << ((i) * 8));
	}
	return readValue;
}

