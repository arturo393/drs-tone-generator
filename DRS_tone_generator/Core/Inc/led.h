
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
#define SYS_RP_LED_ON() SET_BIT(GPIOB->ODR,GPIO_ODR_OD1)
#define SYS_RP_LED_OFF() CLEAR_BIT(GPIOB->ODR,GPIO_ODR_OD1)

#define HIBRID_MODE_ON_LED() SET_BIT(GPIOA->ODR,GPIO_ODR_OD7)
#define HIBRID_MODE_OFF_LED() CLEAR_BIT(GPIOA->ODR,GPIO_ODR_OD7)

#define FREQ_CHANGING_ON_LED() SET_BIT(GPIOB->ODR,GPIO_ODR_OD0)
#define FREQ_CHANGING_OFF_LED() CLEAR_BIT(GPIOB->ODR,GPIO_ODR_OD0)

typedef struct LED{
	uint32_t kaCounter;
	uint32_t clCounter;
	uint32_t cnCounter;
	uint32_t chCounter;
	uint32_t sysrpCounter;
	uint32_t tokCounter;
	uint32_t thCounter;
}LED_t;

void ledInit(LED_t *led);
void led_off(void);
void led_enable_kalive(LED_t *l);
void led_reset(LED_t *l);
void led_current_update(int16_t current);
uint8_t led_temperature_update(uint8_t tempearture);
void led_i2c_toggle(LED_t *l);

#endif /* INC_LED_H_ */
