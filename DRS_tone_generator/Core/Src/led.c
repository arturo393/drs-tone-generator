/*
 * led.c
 *
 *  Created on: Sep 26, 2022
 *      Author: sigmadev
 */
#include "led.h"

void led_init(LED_t *led) {
	/*CURRENT NORMAL LED PA7 (A) as output SUMA CHANGING*/
	SET_BIT(GPIOA->MODER, GPIO_MODER_MODE7_0);
	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE7_1);
	/*CURRENT NORMAL LED PB0 (B)  as output SUMA END*/
	SET_BIT(GPIOB->MODER, GPIO_MODER_MODE0_0);
	CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODE0_1);
	/*CURRENT NORMAL LED PB1 (SR) as output SYSTEM R */
	SET_BIT(GPIOB->MODER, GPIO_MODER_MODE1_0);
	CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODE1_1);
	led_reset(led);
}
void led_off(void) {

}

void led_enable_kalive(LED_t *l) {
	if (HAL_GetTick() - l->ka_counter > LED_KA_STATE_TIMEOUT)
		l->ka_counter = HAL_GetTick();
	else {
		if (HAL_GetTick() - l->ka_counter > LED_KA_ON_TIMEOUT)
			sys_rp_led_off();
		else
			sys_rp_led_on();
	}

}
void led_i2c_toggle(LED_t *l) {
	if (READ_BIT(GPIOB->ODR, GPIO_ODR_OD1))
		i2c1_irq_led_off();
	else
		i2c1_irq_led_on();
}

void led_reset(LED_t *l) {
	l->ch_counter = 0;
	l->cl_counter = 0;
	l->cn_counter = 0;
	l->ka_counter = HAL_GetTick();
	l->sysrp_counter = 0;
	l->th_counter = 0;
	l->tok_counter = 0;
}

