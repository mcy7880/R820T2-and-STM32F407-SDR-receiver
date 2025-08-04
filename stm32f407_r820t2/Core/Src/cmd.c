/*
 * cmd.c - Command parsing routines for STM32F303 breakout SPI to ice5 FPGA
 * 05-11-16 E. Brombaugh
 * 02-08-2025 Maciej Fajfer - porting to STM32F407, modifications, improvements and Software Defined Radio code
 */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "cmd.h"
#include "usart.h"
#include "printf.h"
#include "r820t2.h"
#include "MY_CS43L22.h"
#include "led.h"

#define MAX_ARGS 5

extern I2C_HandleTypeDef hi2c3;
extern float I, Q;

/* locals we use here */
char cmd_buffer[80];
char *cmd_wptr;
const char *cmd_commands[] = 
{
	"help",
	"read",
	"write",
    "dump",
    "freq",
    "lna_gain",
    "mixer_gain",
    "vga_gain",
    "lna_agc_ena",
    "mixer_agc_ena",
    "bandwidth",
    "init",
    "volume",
	"mute",
	"unmute",
	"demod_type",
	"tune",
	"scan",
	NULL
};

const char *demod_type_param[] = {"AM", "FM", "IQ", "CW", NULL};

extern Output_demod_type_enum Demod_Type;
extern uint16_t CW_trig_upper_level;
extern uint8_t CW_trig_lower_level;
extern float b[];
extern float a[];

/* reset buffer & display the prompt */
void cmd_prompt(void)
{
	/* reset input buffer */
	cmd_wptr = &cmd_buffer[0];

	/* prompt user */
	UART_printf("\r\nCommand>");
}

static char wait_for_char_SCAN(double* step)
{
	char rxchar_loc;
	UART_printf(" -> u/d - up/down ; s - stop\r\n\r\n");
	while ( ((rxchar_loc = usart_getc()) == EOF) || ((rxchar_loc != 'u') && (rxchar_loc != 'd') && (rxchar_loc != 's')) );
	if (rxchar_loc == 'u') *step = fabs(*step);
	if (rxchar_loc == 'd') *step *= -1;
	usart_flush_RX_buffer();
	return rxchar_loc;
}

void set_IQ_filters_coeff(float* b, float* a, Output_demod_type_enum Demod_Type)
{
	if ( (Demod_Type == DEMOD_FM) || (Demod_Type == OUT_IQ) )
	{
		//set coefficients for FM and fc=105 kHz
		b[0] = b0__105kHz;
		b[1] = b1__105kHz;
		b[2] = b2__105kHz;
		b[3] = b3__105kHz;
		b[4] = b4__105kHz;
		b[5] = b5__105kHz;
		a[0] = a0__105kHz;
		a[1] = a1__105kHz;
		a[2] = a2__105kHz;
		a[3] = a3__105kHz;
		a[4] = a4__105kHz;
	}
	else
	{
		//set coefficients for AM or CW and fc=15 kHz
		b[0] = b0__15kHz;
		b[1] = b1__15kHz;
		b[2] = b2__15kHz;
		b[3] = b3__15kHz;
		b[4] = b4__15kHz;
		b[5] = b5__15kHz;
		a[0] = a0__15kHz;
		a[1] = a1__15kHz;
		a[2] = a2__15kHz;
		a[3] = a3__15kHz;
		a[4] = a4__15kHz;
	}
}

static float calculate_mean_module()
{
	float module = 0;
	uint8_t i;
	for (i = 0; i < 120; i++) //calculating mean module value for scan and tune commands
	{
		module += sqrtf(I*I + Q*Q); //it's poor solution but that's not enough computing power for doing it real time in ADC's callbacks
		HAL_Delay(1);
	}
	return 20.0*log10f(module/120.0);
}

/* process command line after <cr> */
void cmd_proc(void)
{
	char *token, *argv[MAX_ARGS];
	int argc, cmd, reg;
	unsigned long data, i;

	/* parse out three tokens: cmd arg arg */
	argc = 0;
	token = strtok(cmd_buffer, " ");
	while(token != NULL && argc < MAX_ARGS)
	{
		argv[argc++] = token;
		token = strtok(NULL, " ");
	}

	/* figure out which command it is */
	if(argc > 0)
	{
		cmd = 0;
		while(cmd_commands[cmd] != NULL)
		{
			if(strcmp(argv[0], cmd_commands[cmd])==0)
				break;
			cmd++;
		}
	
		/* Can we handle this? */
		if(cmd_commands[cmd] != NULL)
		{
			UART_printf("\r\n");

			/* Handle commands */
			switch(cmd)
			{
				case 0:		/* Help */
					UART_printf("help - this message\r\n");
					UART_printf("read <addr> - read reg (R820T2)\r\n");
					UART_printf("write <addr> <data> - write reg, data (R820T2)\r\n");
					UART_printf("dump - dump all regs (R820T2)\r\n");
					UART_printf("freq <frequency> - Set freq in MHz\r\n");
					UART_printf("lna_gain <gain> - Set gain of LNA [0 - 15]\r\n");
					UART_printf("mixer_gain <gain> - Set gain Mixer [0 - 15]\r\n");
					UART_printf("vga_gain <gain> - Set gain VGA [1 - 49.3 dB]\r\n");
					UART_printf("lna_agc_ena <state> - Enable LNA AGC [0 / 1]\r\n");
					UART_printf("mixer_agc_ena <state> - Enable Mixer AGC [0 / 1]\r\n");
					UART_printf("bandwidth <bw> - Set IF bandwidth [1 - 15]\r\n");
                    UART_printf("init - default SDR state\r\n");
                    UART_printf("volume <vol> - audio volume for CS43L22 [0 - 100]\r\n");
                    UART_printf("mute - muting of CS43L22\r\n");
                    UART_printf("unmute - unmuting of CS43L22\r\n");
                    UART_printf("demod_type <type> <CW upper lvl> <CW hyst> - Set demodulator type [AM/FM/IQ/CW]\r\n");
                    UART_printf("tune <start_freq> <step> - Manual tune from start_freq [MHz] with step [MHz]\r\n");
                    UART_printf("scan <start_freq> <step> <mod_thres> <Mute> - Scan from start_freq [MHz] with step [MHz], mod_thres [dB] and Mute [0/1]\r\n");
					break;
	
				case 1: 	/* read */
					if(argc < 2)
						UART_printf("read - missing arg(s)\r\n");
					else
					{
						reg = (int)strtoul(argv[1], NULL, 0) & 0x3f;
						data = R820T2_i2c_read_reg_uncached(reg);
						UART_printf("read: 0x%02X = 0x%02lX\r\n", reg, data);
					}
					break;
	
				case 2: 	/* write */
					if(argc < 3)
						UART_printf("write - missing arg(s)\r\n");
					else
					{
						reg = (int)strtoul(argv[1], NULL, 0) & 0x3f;
						data = strtoul(argv[2], NULL, 0);
						R820T2_i2c_write_reg(reg, data);
						UART_printf("write: 0x%02X 0x%02lX\r\n", reg, data);
					}
					break;
		
				case 3: 	/* dump */
                    UART_printf("dump:\r\n");
                    for(i=0;i<0x20;i++)
                    {
                        reg = R820T2_i2c_read_reg_uncached(i);
                        UART_printf("%3d : 0x%02X\r\n", i, reg&0xff);
                        HAL_Delay(5);
                    }
                    break;
                
                case 4:     /* freq */
					if(argc < 2)
						UART_printf("freq - missing arg(s)\r\n");
					else
					{
						double frequency = atof(argv[1]);
						R820T2_set_freq( (uint32_t) (frequency*1.0E6));
						UART_printf("freq:  %.6f MHz", frequency);

						int32_t PLL_status = R820T2_PLL_lock_check();
						if (PLL_status == -1)
							UART_printf(" -> PLL has not locked\r\n");
						else
							UART_printf(" -> PLL has locked\r\n");
					}
                    break;
                    
                case 5:     /* lna gain */
					if(argc < 2)
						UART_printf("lna_gain - missing arg(s)\r\n");
					else
					{
						data = (int)strtoul(argv[1], NULL, 0) & 0x0f;
						R820T2_set_lna_gain(data);;
						UART_printf("lna_gain:  %ld\r\n", data);
					}
                    break;
                
                case 6:     /* mixer gain */
					if(argc < 2)
						UART_printf("mixer_gain - missing arg(s)\r\n");
					else
					{
						data = (int)strtoul(argv[1], NULL, 0) & 0x0f;
						R820T2_set_mixer_gain(data);;
						UART_printf("mixer_gain:  %ld\r\n", data);
					}
                    break;
                
                case 7:     /* vga gain */
					if(argc < 2)
						UART_printf("vga_gain - missing arg(s)\r\n");
					else
					{
						/*data = (int)strtoul(argv[1], NULL, 0) & 0x0f;
						R820T2_set_vga_gain_VGA_code(data);;
						UART_printf("vga_gain:  %ld\r\n", data);*/

						float vga_gain_dB = atof(argv[1]);
						R820T2_set_vga_gain_VAGC_pin(vga_gain_dB);
						if (vga_gain_dB < 1.0) vga_gain_dB = 1.0;
						if (vga_gain_dB > 50.0) vga_gain_dB = 50.0;
						UART_printf("vga_gain:  %.1f dB\r\n", truncf(vga_gain_dB/0.1)*0.1); //calculating actual and truncated gain
					}
                    break;
                
                case 8:     /* lna agc enable */
					if(argc < 2)
						UART_printf("lna_agc_ena - missing arg(s)\r\n");
					else
					{
						data = (int)strtoul(argv[1], NULL, 0) & 0x01;
						R820T2_set_lna_agc(data);;
						UART_printf("lna_agc_ena:  %ld\r\n", data);
					}
                    break;
                
                case 9:     /* mixer agc enable */
					if(argc < 2)
						UART_printf("lna_agc_ena - missing arg(s)\r\n");
					else
					{
						data = (int)strtoul(argv[1], NULL, 0) & 0x01;
						R820T2_set_mixer_agc(data);;
						UART_printf("mixer_agc_ena:  %ld\r\n", data);
					}
                    break;
                
                case 10:     /* IF Bandwidth */
					if(argc < 2)
						UART_printf("bandwidth - missing arg(s)\r\n");
					else
					{
						data = (int)strtoul(argv[1], NULL, 0) & 0x0f;
						R820T2_set_if_bandwidth(data);;
						UART_printf("bandwidth:  %ld\r\n", data);
					}
                    break;
                
				case 11: 	/* init */
                    UART_printf("init\r\n");
                    R820T2_init(hi2c3);
                    R820T2_set_mixer_agc(1);
					R820T2_set_lna_agc(1);
					//R820T2_set_vga_gain_VGA_code(14);
					R820T2_set_vga_gain_VAGC_pin(48.0); //dB
					Demod_Type = DEMOD_FM;

					CS43_SetVolume(CS43_default_vol);
					CS43_Unmute();
                    break;

				case 12:     /* audio volume for CS43L22 */
					if(argc < 2)
						UART_printf("volume - missing arg(s)\r\n");
					else
					{
						data = (int)strtoul(argv[1], NULL, 0);
						if (data > 100) data = 100;
						CS43_SetVolume(data);
						UART_printf("volume:  %ld\r\n", data);
					}
					break;

				case 13: 	/* CS43L22 mute */
                    UART_printf("CS43L22 muted\r\n");
                    CS43_Mute();
                    break;

				case 14: 	/* CS43L22 unmute */
                    UART_printf("CS43L22 unmuted\r\n");
                    CS43_Unmute();
                    break;

				case 15: 	/* demodulator type */
					if(argc < 2)
						UART_printf("demod_type - missing arg(s)\r\n");
					else
					{
						uint8_t type = 0;
						while(demod_type_param[type] != NULL)
						{
							if(strcmp(argv[1], demod_type_param[type])==0)
								break;
							type++;
						}

						if (demod_type_param[type] != NULL)
						{
							switch(type)
							{
								case 0: //AM
									Demod_Type = DEMOD_AM;
									set_IQ_filters_coeff(b, a, Demod_Type);
									UART_printf("demod_type: AM\r\n");
								break;

								case 1: //FM
									Demod_Type = DEMOD_FM;
									set_IQ_filters_coeff(b, a, Demod_Type);
									UART_printf("demod_type: FM\r\n");
								break;

								case 2: //IQ
									Demod_Type = OUT_IQ;
									set_IQ_filters_coeff(b, a, Demod_Type);
									UART_printf("demod_type: IQ\r\n");
								break;

								case 3: //CW
									if(argc < 4)
										UART_printf("demod_type CW - missing arg(s)\r\n");
									else
									{
										CW_trig_lower_level = (int)strtoul(argv[2], NULL, 0) & 0xFF; //trigger lower level
										CW_trig_upper_level = CW_trig_lower_level + ((int)strtoul(argv[3], NULL, 0) & 0xFF); //trigger lower level + hyst
										Demod_Type = DEMOD_CW;
										set_IQ_filters_coeff(b, a, Demod_Type);
										UART_printf("demod_type: CW %d %d\r\n", CW_trig_lower_level, CW_trig_upper_level-CW_trig_lower_level);
									}
								break;

								default:
								break;
							}
						}
						else
							UART_printf("demod_type - unknown type param\r\n");

					}

					break;

				case 16: 	/* tune */
					if(argc < 3)
						UART_printf("tune - missing arg(s)\r\n");
					else
					{
						double frequency = atof(argv[1]);
						double step = fabs(atof(argv[2]));
						char rxchar_loc;
						UART_printf("start_freq: %.6f MHz ; step: %.6f MHz\r\n\r\n", frequency, step);

						do {
							R820T2_set_freq( (uint32_t) (frequency*1.0E6));
							UART_printf("%.6f MHz", frequency);

							int32_t PLL_status = R820T2_PLL_lock_check();
							if (PLL_status == -1)
							{
								UART_printf(" -> PLL has not locked\r\n");
								break;
							}

							UART_printf(" ; Mod: %.2f\r\n -> n/p - next/prev step ; s - stop\r\n", calculate_mean_module());

							while ( ((rxchar_loc = usart_getc()) == EOF) || ((rxchar_loc != 'n') && (rxchar_loc != 'p') && (rxchar_loc != 's')) );
							usart_flush_RX_buffer();

							if (rxchar_loc == 'p') frequency -= step;
							if (rxchar_loc == 'n') frequency += step;
							if (frequency < 25.0) frequency = 25.0;

						} while( (rxchar_loc == 'n') || (rxchar_loc == 'p') );
					}
					break;

				case 17: 	/* scan */
					if(argc < 5)
						UART_printf("scan - missing arg(s)\r\n");
					else
					{
						double frequency = atof(argv[1]);
						double step = fabs(atof(argv[2]));
						float module_threshold = atof(argv[3]);
						uint8_t Mute_Dis_Ena = atof(argv[4]);

						char rxchar_loc;
						UART_printf("start_freq: %.6f MHz ; step: %.6f MHz ; Mod_thresh: %.2f ; s - stop ; p - pause\r\n\r\n", frequency, step, module_threshold);

						if (Mute_Dis_Ena == 1) CS43_Mute();
						while(1)
						{
							R820T2_set_freq( (uint32_t) (frequency*1.0E6));
							UART_printf("SCANNING: %.6f MHz", frequency);

							int32_t PLL_status = R820T2_PLL_lock_check();
							if (PLL_status == -1)
							{
								UART_printf(" -> PLL has not locked\r\n");
								break;
							}

							HAL_Delay(200);
							float module = calculate_mean_module();
							UART_printf(" ; Mod: %.2f\r\n", module);
							led_toggle(LED1);

							if((rxchar_loc = usart_getc())!= EOF)
							{
								if (rxchar_loc == 's') break;
								if (rxchar_loc == 'p')
								{
									if (Mute_Dis_Ena == 1) CS43_Unmute();
									UART_printf("***SCANNING PAUSED***");
									if (wait_for_char_SCAN(&step) == 's') break;
								}
							}

							if (module > module_threshold)
							{
								if (Mute_Dis_Ena == 1) CS43_Unmute();
								UART_printf("***SCANNING HAS STOPPED***");
								if (wait_for_char_SCAN(&step) == 's') break;
							}

							if (Mute_Dis_Ena == 1) CS43_Mute();

							frequency += step;
							if (frequency < 25.0) frequency = 25.0;

						}
					}

					CS43_Unmute();
					break;

				default:	/* shouldn't get here */
					break;
			}
		}
		else
			UART_printf(" Unknown command\r\n");
	}
}
void R820T2_set_if_bandwidth(uint8_t bw);
	
void init_cmd(void)
{
	/* prompt */
	cmd_prompt();
}

void cmd_parse(char ch)
{
	/* accumulate chars until cr, handle backspace */
	if(ch == '\b')
	{
		/* check for buffer underflow */
		if(cmd_wptr - &cmd_buffer[0] > 0)
		{
			UART_printf("\b \b");		/* Erase & backspace */
			cmd_wptr--;		/* remove previous char */
		}
	}
	else if(ch == '\r')
	{
		*cmd_wptr = '\0';	/* null terminate, no inc */
		cmd_proc();
		cmd_prompt();
	}
	else
	{
		/* check for buffer full (leave room for null) */
		if(cmd_wptr - &cmd_buffer[0] < 254)
		{
			*cmd_wptr++ = ch;	/* store to buffer */
			usart_putc(NULL, ch);   /* echo */
		}
	}
}
