/*
 * led.c - f030 breakout LED setup
 */

#include "led.h"

/*
 * Turn on LED
 */
void led_on(uint16_t LED)
{
	HAL_GPIO_WritePin(LED_LD4_GPIO_Port, LED, GPIO_PIN_SET);
}

/*
 * Turn off LED
 */
void led_off(uint16_t LED)
{
	HAL_GPIO_WritePin(LED_LD4_GPIO_Port, LED, GPIO_PIN_RESET);
}

/*
 * Toggle LED
 */
void led_toggle(uint16_t LED)
{
	HAL_GPIO_TogglePin(LED_LD4_GPIO_Port, LED);
}

