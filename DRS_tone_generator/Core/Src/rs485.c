/*
 * rs485.c
 *
 *  Created on: Sep 28, 2022
 *      Author: sigmadev
 */
#include "rs485.h"
#define MINIMUN_FRAME_LEN 6

uint16_t crc_get(uint8_t *buffer, uint8_t buff_len) {
	uint8_t b;
	uint8_t i;
	uint16_t generator = 0x1021; //divisor is 16bit
	uint16_t crc = 0;			 // CRC value is 16bit

	for (b = 0; b < buff_len; b++) {
		crc ^= ((uint16_t) (buffer[b] << 8)); // move byte into MSB of 16bit CRC
		for (i = 0; i < 8; i++) {
			if ((crc & 0x8000) != 0) // test for MSB = bit 15
				crc = ((uint16_t) ((crc << 1) ^ generator));
			else
				crc <<= 1;
		}
	}
	return crc;
}

void rs485Init(RS485_t *r) {
	r->len = 0;
	r->status = DONE;
	r->cmd = NONE;
	/* PB9 DE485 as output  */
	SET_BIT(GPIOB->MODER, GPIO_MODER_MODE9_0);
	CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODE9_1);

}
Rs485_status_t rs485_check_frame(RS485_t *r, UART1_t *u) {

	if (u->rxCount > (MINIMUN_FRAME_LEN)) {
		if (u->rxBuffer[0] == LTEL_START_MARK) {
			if (u->rxBuffer[u->rxCount - 1] == LTEL_END_MARK)
				return VALID_FRAME;
			else
				return START_READING;
		} else
			return NOT_VALID_FRAME;
	} else

		return WAITING;
}

Rs485_status_t rs485_check_CRC_module(UART1_t *uart1) {
	unsigned long crc_cal;
	unsigned long crc_save;
	crc_save = uart1->rxBuffer[8] << 8;
	crc_save |= uart1->rxBuffer[9];
	crc_cal = crc_get(&(uart1->rxBuffer[1]), 7);
	if (crc_cal == crc_save)
		return DATA_OK;
	return CRC_ERROR;
}

Rs485_status_t rs485_check_valid_module(UART1_t *uart1) {
	if (uart1->rxBuffer[1] == UHF_TONE) {
		if (uart1->rxBuffer[2] == ID0) {
			for (int i = 3; i < uart1->rxCount; i++)
				if (uart1->rxBuffer[i] == LTEL_END_MARK)
					return VALID_MODULE;
		} else
			return WRONG_MODULE_ID;
	} else
		return WRONG_MODULE_FUNCTION;
	return WRONG_MODULE_FUNCTION;
}

void rs485_set_query_frame(RS485_t *r, Module_pa_t *module) {

	/*uint8_t crc_frame[2];
	 uint16_t crc;
	 r->frame[0] = LTEL_START_MARK;
	 r->frame[1] = module->function;
	 r->frame[2] = module->id;
	 r->frame[3] = r->cmd;

	 if (module->function == LOW_NOISE_AMPLIFIER) {
	 switch (r->cmd) {
	 case QUERY_PARAMETER_LTEL:
	 r->frame[4] = 0x00;
	 r->frame[5] = 0x05;
	 r->frame[6] = 0x00;
	 r->frame[7] = module->att;
	 r->frame[8] = module->gain;
	 r->frame[9] = module->pout;
	 r->frame[10] = module->voltage;
	 break;
	 case QUERY_PARAMETER_SIGMA:
	 r->frame[4] = 0x06;
	 r->frame[5] = module->pout;
	 r->frame[6] = module->att;
	 r->frame[7] = module->gain;
	 r->frame[8] = module->current;
	 r->frame[9] = module->voltage;
	 r->frame[10] = module->pin;
	 default:
	 r->frame[0] = 0;
	 }
	 crc = crc_get(&(r->frame[1]), 10);
	 memcpy(crc_frame, &crc, 2);
	 r->frame[11] = crc_frame[0];
	 r->frame[12] = crc_frame[1];
	 r->frame[13] = LTEL_END_MARK;

	 } else if (module->function == POWER_AMPLIFIER) {

	 switch (r->cmd) {
	 case QUERY_PARAMETER_LTEL:
	 r->frame[4] = 0x00;
	 r->frame[5] = 0x08;
	 r->frame[6] = module->enable;
	 r->frame[7] = 0x00;
	 r->frame[8] = module->temperature;
	 r->frame[9] = module->gain;
	 r->frame[10] = module->vswr;
	 r->frame[11] = module->att;
	 r->frame[12] = module->pout;
	 r->frame[13] = module->pin;

	 break;
	 case QUERY_PARAMETER_SIGMA:
	 r->frame[4] = 0x00;
	 r->frame[5] = 0x08;
	 r->frame[6] = module->enable;
	 r->frame[7] = 0x00;
	 r->frame[8] = module->temperature;
	 r->frame[9] = module->gain;
	 r->frame[10] = module->vswr;
	 r->frame[11] = module->att;
	 r->frame[12] = module->pout;
	 r->frame[13] = module->pin;
	 default:
	 r->frame[0] = 0;
	 }
	 crc = crc_get(&(r->frame[1]), 10);
	 memcpy(crc_frame, &crc, 2);
	 r->frame[13 + 1] = crc_frame[0];
	 r->frame[13 + 2] = crc_frame[1];
	 r->frame[13 + 3] = LTEL_END_MARK;
	 }*/
}

void rs485_parameters_cmd_action(uint8_t *frame) {

}

void rs485Uart1Decode(RS485_t *rs485, UART1_t *uart1) {
	switch (rs485->status) {
	case VALID_MODULE:
		rs485->status = rs485_check_CRC_module(uart1);
		break;
	case DATA_OK:
		rs485->cmd = uart1->rxBuffer[3];
		uart1_send_str("DATA OK\r\n");
		rs485->status = DONE;
		break;
	case START_READING:
		rs485->status = WAITING;
		if (uart1_clean_by_timeout(uart1, "START_READING"))
			rs485->status = DONE;
		break;
	case VALID_FRAME:
		rs485->status = rs485_check_valid_module(uart1);
		break;
	case NOT_VALID_FRAME:
		HAL_Delay(50);
	    uart1_send_str("NOT VALID FRAME\r\n");
		uart1_clean_buffer(uart1);
		rs485->status = DONE;
		break;
	case WRONG_MODULE_ID:
		uart1_send_str("WRONG MODULE ID\r\n");
		uart1_clean_buffer(uart1);
		rs485->status = DONE;
		break;
	case WRONG_MODULE_FUNCTION:
		uart1_send_str("WRONG MODULE FUNCTION\r\n");
		uart1_clean_buffer(uart1);
		rs485->status = DONE;
		break;
	case CRC_ERROR:
		uart1_send_str("CRC ERROR\r\n");
		uart1_clean_buffer(uart1);
		rs485->status = DONE;
		break;
	case WAITING:
		rs485->status = rs485_check_frame(rs485, uart1);
		uart1_clean_by_timeout(uart1, "WAITING");
		break;
	case DONE:
		uart1_send_str("DONE\r\n");
		uart1_clean_buffer(uart1);
		rs485->status = WAITING;
		break;
	default:
		rs485->status = DONE;
		uart1_clean_buffer(uart1);
		break;
	}
}