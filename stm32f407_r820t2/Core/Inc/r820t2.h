/*
 * r820t2.h - R820T2 downconverter driver for STM32F030
 * 02-12-2017 E. Brombaugh
 */

#ifndef __r820t2__
#define __r820t2__

#include <stdint.h>
#include "main.h"

extern uint32_t r820t_freq;
extern uint32_t r820t_xtal_freq;
extern uint32_t r820t_if_freq;

void R820T2_init(I2C_HandleTypeDef i2c_handle);
int32_t R820T2_PLL_lock_check(void);
void R820T2_i2c_write_reg(uint8_t reg, uint8_t data);
void R820T2_i2c_read_raw(uint8_t *data, uint8_t sz);
uint8_t R820T2_i2c_read_reg_uncached(uint8_t reg);
uint8_t R820T2_i2c_read_reg_cached(uint8_t reg);
void R820T2_set_freq(uint32_t freq);
void R820T2_set_lna_gain(uint8_t gain_index);
void R820T2_set_mixer_gain(uint8_t gain_index);
void R820T2_set_vga_gain_VGA_code(uint8_t gain_index);
void R820T2_set_vga_gain_VAGC_pin(float gain_dB);
void R820T2_config_VAGC_pin(uint8_t value);
void R820T2_set_lna_agc(uint8_t value);
void R820T2_set_mixer_agc(uint8_t value);
void R820T2_set_if_bandwidth(uint8_t bw);

#endif
