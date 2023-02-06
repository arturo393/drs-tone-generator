
/*
 * led.h
 *
 *  Created on: Sep 26, 2022
 *      Author: sigmadev
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include "main.h"

#define LED_KA_STATE_TIMEOUT  1000
#define LED_KA_ON_TIMEOUT  50
#define LED_MAX_CURRENT 600
#define LED_MIN_CURRENT 100
#define LED_MAX_TEMPERATURE 75
#define sys_rp_led_on() SET_BIT(GPIOB->ODR,GPIO_ODR_OD1)
#define sys_rp_led_off() CLEAR_BIT(GPIOB->ODR,GPIO_ODR_OD1)

#define suma_end_on() SET_BIT(GPIOA->ODR,GPIO_ODR_OD7)
#define suma_end_off() CLEAR_BIT(GPIOA->ODR,GPIO_ODR_OD7)

#define suma_changing_on() SET_BIT(GPIOB->ODR,GPIO_ODR_OD0)
#define suma_changing_off() CLEAR_BIT(GPIOB->ODR,GPIO_ODR_OD0)

typedef struct LED{
	uint32_t ka_counter;
	uint32_t cl_counter;
	uint32_t cn_counter;
	uint32_t ch_counter;
	uint32_t sysrp_counter;
	uint32_t tok_counter;
	uint32_t th_counter;
}LED_t;

void led_init(LED_t *led);
void led_off(void);
void led_enable_kalive(LED_t *l);
void led_reset(LED_t *l);
void led_current_update(int16_t current);
uint8_t led_temperature_update(uint8_t tempearture);
void led_i2c_toggle(LED_t *l);

#endif /* INC_LED_H_ */
