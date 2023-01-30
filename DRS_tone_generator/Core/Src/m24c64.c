#include <m24c64.h>

void m24c64_page_read(uint8_t address, uint8_t page, uint8_t *data) {
	uint8_t buff[2] = { 0 };
	uint16_t MemAddress = page << PADDRPOSITION;

	buff[0] = MemAddress >> 8;
	buff[1] = MemAddress & 0xff;

	i2c1MasterByteTx(CHIP_ADDR, buff, 2);
	i2c1MasterFrameRx(CHIP_ADDR, data, 32);
}

void m24c64ReadNBytes(uint8_t page, uint8_t *data, uint8_t offset, uint8_t size) {
	uint8_t buff[2] = { 0 };
	uint16_t MemAddress = page << PADDRPOSITION | offset;

	buff[0] = MemAddress >> 8;
	buff[1] = MemAddress & 0xff;

	i2c1MasterByteTx(CHIP_ADDR, buff, 2);
	HAL_Delay(5);
	i2c1MasterFrameRx(CHIP_ADDR, data, size);
}

void m24c64WriteNBytes(uint8_t page, uint8_t *data, uint8_t offset, uint8_t size) {
	uint8_t buff[size + 2];
	uint8_t read[size];

	m24c64ReadNBytes(page, read, offset, size);

	if (strncmp((const char*) data, (const char*) read, (size_t) size)) {
		buff[0] = (page << PADDRPOSITION | offset) >> 8;
		buff[1] = (page << PADDRPOSITION | offset) & 0xff;
		for (int i = 0; i < size; i++) {
			buff[i + 2] = data[i];
		}
		i2c1MasterByteTx(CHIP_ADDR, buff, size + 2);
	}
	HAL_Delay(6);
}

void m24c64_init_16uvalue(M24C64_ADDR_t addr, uint16_t value) {
	uint8_t buff[2];
	m24c64ReadNBytes(BASE_ADDR, buff, addr, 1);
	if (!(buff[0] == IS_READY)) {
		buff[0] = value >> 8;
		buff[1] = value & 0xff;
		m24c64WriteNBytes(BASE_ADDR, buff, addr + 1, 2);
	}
}

void m24c64_store_16uvalue(M24C64_ADDR_t addr, uint16_t value) {
	uint8_t buff[2];
	buff[0] = value >> 8;
	buff[1] = value & 0xff;
	m24c64WriteNBytes(BASE_ADDR, buff, addr + 1, 2);
	buff[0] = addr;
	m24c64WriteNBytes(BASE_ADDR, buff, addr, 1);
}
