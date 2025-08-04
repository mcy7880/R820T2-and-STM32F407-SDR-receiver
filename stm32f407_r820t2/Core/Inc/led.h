/*
 * led.c - f030 breakout LED setup
 */

#ifndef __led__
#define __led__

#include <stdint.h>
#include "main.h"

#define LED1 LED_LD4_Pin

void led_on(uint16_t LED);
void led_off(uint16_t LED);
void led_toggle(uint16_t LED);

#endif
