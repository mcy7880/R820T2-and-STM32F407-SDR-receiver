/*
 * R820T2.c - R820T2 Clock Generator driver for STM32F030
 * 01-08-2017 E. Brombaugh
 * 02-08-2025 Maciej Fajfer - porting to STM32F407, modifications, improvements and Software Defined Radio code
 */

#include "r820t2.h"
#include <math.h>
#include "printf.h"
#include "main.h"

/*
 * I2C stuff
 * R820T can't works on the same I2C bus with CS43L22 due to addresses conflict! It's strange but true.
 */
#define R820T2_I2C_ADDRESS   ((0x1A)<<1)	// 0011010+RW - 0x34 (w), 0x35 (r)
#define FLAG_TIMEOUT         ((uint32_t)0x1000)
#define LONG_TIMEOUT         ((uint32_t)(10 * FLAG_TIMEOUT))

/*
 * Freq calcs
 */
#define XTAL_FREQ 28800000
#define CALIBRATION_LO 88000

/* registers */
#define R820T2_NUM_REGS     0x20
#define R820T2_WRITE_START	5

I2C_HandleTypeDef hi2cx;

/* initial values from airspy */
/* initial freq @ 128MHz -> ~5MHz IF due to xtal mismatch */
static const uint8_t r82xx_init_array[R820T2_NUM_REGS] =
{
	0x00, 0x00, 0x00, 0x00,	0x00,		/* 00 to 04 */
    /* 05 */ 0x90, // LNA manual gain mode, init to 0
    /* 06 */ 0x80,
    /* 07 */ 0x60,
    /* 08 */ 0x80, // Image Gain Adjustment
    /* 09 */ 0x40, // Image Phase Adjustment
    /* 0A */ 0xA8, // Channel filter [0..3]: 0 = widest, f = narrowest - Optimal. Don't touch!
    /* 0B */ 0x0F, // High pass filter - Optimal. Don't touch!
    /* 0C */ 0x40, // VGA control by code, init at 0
    /* 0D */ 0x63, // LNA AGC settings: [0..3]: Lower threshold; [4..7]: High threshold
    /* 0E */ 0x75,
    /* 0F */ 0xF8, // Filter Widest, LDO_5V OFF, clk out OFF,
    /* 10 */ 0x7C,
    /* 11 */ 0x83,
    /* 12 */ 0x80,
    /* 13 */ 0x00,
    /* 14 */ 0x0F,
    /* 15 */ 0x00,
    /* 16 */ 0xC0,
    /* 17 */ 0x30,
    /* 18 */ 0x48,
    /* 19 */ 0xCC,
    /* 1A */ 0x60,
    /* 1B */ 0x00,
    /* 1C */ 0x54,
    /* 1D */ 0xAE,
    /* 1E */ 0x0A,
    /* 1F */ 0xC0
};

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

/*
 * Tuner frequency ranges
 * Kanged & modified from airspy firmware to include freq for scanning table
 * "Copyright (C) 2013 Mauro Carvalho Chehab"
 * https://stuff.mit.edu/afs/sipb/contrib/linux/drivers/media/tuners/r820t.c
 */
struct r820t_freq_range
{
  uint16_t freq;
  uint8_t open_d;
  uint8_t rf_mux_ploy;
  uint8_t tf_c;
};

const struct r820t_freq_range freq_ranges[] =
{
  {
  /* 0 MHz */ 0,
  /* .open_d = */     0x08, /* low */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */     0xdf, /* R27[7:0]  band2,band0 */
  }, {
  /* 50 MHz */ 50,
  /* .open_d = */     0x08, /* low */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */     0xbe, /* R27[7:0]  band4,band1  */
  }, {
  /* 55 MHz */ 55,
  /* .open_d = */     0x08, /* low */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */     0x8b, /* R27[7:0]  band7,band4 */
  }, {
  /* 60 MHz */ 60,
  /* .open_d = */     0x08, /* low */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */     0x7b, /* R27[7:0]  band8,band4 */
  }, {
  /* 65 MHz */ 65,
  /* .open_d = */     0x08, /* low */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */     0x69, /* R27[7:0]  band9,band6 */
  }, {
  /* 70 MHz */ 70,
  /* .open_d = */     0x08, /* low */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */     0x58, /* R27[7:0]  band10,band7 */
  }, {
  /* 75 MHz */ 75,
  /* .open_d = */     0x00, /* high */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */     0x44, /* R27[7:0]  band11,band11 */
  }, {
  /* 80 MHz */ 80,
  /* .open_d = */     0x00, /* high */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */     0x44, /* R27[7:0]  band11,band11 */
  }, {
  /* 90 MHz */ 90,
  /* .open_d = */     0x00, /* high */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */     0x34, /* R27[7:0]  band12,band11 */
  }, {
  /* 100 MHz */ 100,
  /* .open_d = */     0x00, /* high */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */     0x34, /* R27[7:0]  band12,band11 */
  }, {
  /* 110 MHz */ 110,
  /* .open_d = */     0x00, /* high */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */     0x24, /* R27[7:0]  band13,band11 */
  }, {
  /* 120 MHz */ 120,
  /* .open_d = */     0x00, /* high */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */     0x24, /* R27[7:0]  band13,band11 */
  }, {
  /* 140 MHz */ 140,
  /* .open_d = */     0x00, /* high */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */     0x14, /* R27[7:0]  band14,band11 */
  }, {
  /* 180 MHz */ 180,
  /* .open_d = */     0x00, /* high */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */     0x13, /* R27[7:0]  band14,band12 */
  }, {
  /* 220 MHz */ 220,
  /* .open_d = */     0x00, /* high */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */     0x13, /* R27[7:0]  band14,band12 */
  }, {
  /* 250 MHz */ 250,
  /* .open_d = */     0x00, /* high */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */     0x11, /* R27[7:0]  highest,highest */
  }, {
  /* 280 MHz */ 280,
  /* .open_d = */     0x00, /* high */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */     0x00, /* R27[7:0]  highest,highest */
  }, {
  /* 310 MHz */ 310,
  /* .open_d = */     0x00, /* high */
  /* .rf_mux_ploy = */  0x41, /* R26[7:6]=1 (bypass)  R26[1:0]=1 (middle) */
  /* .tf_c = */     0x00, /* R27[7:0]  highest,highest */
  }, {
  /* 450 MHz */ 450,
  /* .open_d = */     0x00, /* high */
  /* .rf_mux_ploy = */  0x41, /* R26[7:6]=1 (bypass)  R26[1:0]=1 (middle) */
  /* .tf_c = */     0x00, /* R27[7:0]  highest,highest */
  }, {
  /* 588 MHz */ 588,
  /* .open_d = */     0x00, /* high */
  /* .rf_mux_ploy = */  0x40, /* R26[7:6]=1 (bypass)  R26[1:0]=0 (highest) */
  /* .tf_c = */     0x00, /* R27[7:0]  highest,highest */
  }, {
  /* 650 MHz */ 650,
  /* .open_d = */     0x00, /* high */
  /* .rf_mux_ploy = */  0x40, /* R26[7:6]=1 (bypass)  R26[1:0]=0 (highest) */
  /* .tf_c = */     0x00, /* R27[7:0]  highest,highest */
  }
};

/*
 * local vars
 */
uint8_t r820t_regs[R820T2_NUM_REGS];
uint32_t r820t_freq;
uint32_t r820t_xtal_freq;
uint32_t r820t_if_freq;

/*
 * exception handler for I2C timeout
 */
void R820T2_TIMEOUT_UserCallback(void)
{
	UART_printf("\r\nFailed to communicate with R820T2\n\r");
  /* Block communication and all processes */
  while (1)
  {   
  }
}

/*
 * Send a block of data to the R820T2 via I2C
 */
void R820T2_i2c_write(uint8_t reg, uint8_t *data, uint8_t sz)
{
	
#if 0
    uint16_t i;
    UART_printf("R820T2_i2c_write(%d, %d, %d)\n\r", reg, data, sz);
	for(i=0;i<sz;i++)
    {
        printf("%2d : 0x%02X\n\r", i, data[i]&0xff);
        HAL_Delay(10);
    }
#endif
    
    /* limit to legal addresses */
    if((reg+sz) > R820T2_NUM_REGS)
    {
        return;
    }

    if (HAL_I2C_Mem_Write(&hi2cx, R820T2_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, sz, 150) != HAL_OK)
    {
    	R820T2_TIMEOUT_UserCallback();
    }
}

/*
 * Write single R820T2 reg via I2C
 */
void R820T2_i2c_write_reg(uint8_t reg, uint8_t data)
{
    /* check for legal reg */
    if(reg>=R820T2_NUM_REGS)
        return;
    
    /* update cache */
    r820t_regs[reg] = data;
    
    /* send via I2C */
    R820T2_i2c_write(reg, &r820t_regs[reg], 1);
}

/*
 * Write single R820T2 reg via I2C with mask vs cached
 */
void R820T2_i2c_write_cache_mask(uint8_t reg, uint8_t data, uint8_t mask)
{
    /* check for legal reg */
    if(reg>=R820T2_NUM_REGS)
        return;
    
    /* mask vs cached reg */
    data = (data & mask) | (r820t_regs[reg] & ~mask);
    r820t_regs[reg] = data;
    
    /* send via I2C */
    R820T2_i2c_write(reg, &r820t_regs[reg], 1);
}

/*
 * Nybble-level bit reverse table for register readback
 */
const uint8_t bitrev_lut[16] = { 0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
      0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf };

/*
 * Read R820T2 regs via I2C
 */
void R820T2_i2c_read_raw(uint8_t *data, uint8_t sz)
{
    uint8_t value;

	/* check for legal reg */
	if(sz>R820T2_NUM_REGS)
		return;

	if (HAL_I2C_Master_Receive(&hi2cx, R820T2_I2C_ADDRESS, data, sz, 150) != HAL_OK)
	{
		R820T2_TIMEOUT_UserCallback();
	}
#if 1
	while(sz--)
	{
		/* Get data */
		value = *data;

		/* bit reverse into destination */
		*data++ = (bitrev_lut[value & 0xf] << 4) | bitrev_lut[value >> 4];
	}
#endif
}

/*
 * Read R820T2 reg - uncached
 */
uint8_t R820T2_i2c_read_reg_uncached(uint8_t reg)
{
    uint8_t sz = reg+1;
    uint8_t *data = r820t_regs;

    /* check for legal read */
    if(sz>R820T2_NUM_REGS)
        return 0;
    
    /* get all regs up to & including desired reg */
    R820T2_i2c_read_raw(data, sz);

    /* return desired */
    return r820t_regs[reg];
}

/*
 * Read R820T2 reg - cached
 */
uint8_t R820T2_i2c_read_reg_cached(uint8_t reg)
{
    /* check for legal read */
    if(reg>R820T2_NUM_REGS)
        return 0;
    
    /* return desired */
    return r820t_regs[reg];
}

/*
 * r820t tuning logic
 */
#ifdef OPTIM_SET_MUX
int r820t_set_mux_freq_idx = -1; /* Default set to invalid value in order to force set_mux */
#endif

/*
 * Update Tracking Filter
 * Kanged & Modified from airspy firmware
 * 
 * "inspired by Mauro Carvalho Chehab set_mux technique"
 * https://stuff.mit.edu/afs/sipb/contrib/linux/drivers/media/tuners/r820t.c
 * part of r820t_set_mux() (set tracking filter)
 */
static void R820T2_set_tf(uint32_t freq)
{
    const struct r820t_freq_range *range;
    unsigned int i;

    /* Get Freq in MHz */
    freq = (uint32_t)((uint64_t)freq * 4295 >> 32); // fast approach
    
    /* Scan array for proper range */
    for(i=0;i<ARRAY_SIZE(freq_ranges)-1;i++)
    {
        if (freq < freq_ranges[i + 1].freq)
            break;
    }
    range = &freq_ranges[i];

    /* Open Drain */
    R820T2_i2c_write_cache_mask(0x17, range->open_d, 0x08);

    /* RF_MUX,Polymux */
    R820T2_i2c_write_cache_mask(0x1a, range->rf_mux_ploy, 0xc3);

    /* TF BAND */
    R820T2_i2c_write_reg(0x1b, range->tf_c);

    /* XTAL CAP & Drive */
    R820T2_i2c_write_cache_mask(0x10, 0x08, 0x0b);

    R820T2_i2c_write_cache_mask(0x08, 0x00, 0x3f);
 
    R820T2_i2c_write_cache_mask(0x09, 0x00, 0x3f);
}

/*
 * Update LO PLL
 */
void R820T2_set_pll(uint32_t freq)
{
    const uint32_t vco_min = 1770000000;
    const uint32_t vco_max = 3900000000;
    uint32_t pll_ref = (r820t_xtal_freq >> 1);
    uint32_t pll_ref_2x = r820t_xtal_freq;

    uint32_t vco_exact;
    uint32_t vco_frac;
    uint32_t con_frac;
    uint32_t div_num;
    uint32_t n_sdm;
    uint16_t sdm;
    uint8_t ni;
    uint8_t si;
    uint8_t nint;

    /* Calculate vco output divider */
    for (div_num = 0; div_num < 5; div_num++)
    {
        vco_exact = freq << (div_num + 1);
        if (vco_exact >= vco_min && vco_exact <= vco_max)
        {
            break;
        }
    }

    /* Calculate the integer PLL feedback divider */
    vco_exact = freq << (div_num + 1);
    nint = (uint8_t) ((vco_exact + (pll_ref >> 16)) / pll_ref_2x);
    vco_frac = vco_exact - pll_ref_2x * nint;

    nint -= 13;
    ni = (nint >> 2);
    si = nint - (ni << 2);

    /* Set the vco output divider */
    R820T2_i2c_write_cache_mask(0x10, (uint8_t) (div_num << 5), 0xe0);

    /* Set the PLL Feedback integer divider */
    R820T2_i2c_write_reg(0x14, (uint8_t) (ni + (si << 6)));

    /* Update Fractional PLL */
    if (vco_frac == 0)
    {
        /* Disable frac pll */
        R820T2_i2c_write_cache_mask(0x12, 0x08, 0x08);
    }
    else
    {
        /* Compute the Sigma-Delta Modulator */
        vco_frac += pll_ref >> 16;
        sdm = 0;
        for(n_sdm = 0; n_sdm < 16; n_sdm++)
        {
            con_frac = pll_ref >> n_sdm;
            if (vco_frac >= con_frac)
            {
                sdm |= (uint16_t) (0x8000 >> n_sdm);
                vco_frac -= con_frac;
                if (vco_frac == 0)
                    break;
            }
        }

        /*
        actual_freq = (((nint << 16) + sdm) * (uint64_t) pll_ref_2x) >> (div_num + 1 + 16);
        delta = freq - actual_freq
        if (actual_freq != freq)
        {
            fprintf(stderr,"Tunning delta: %d Hz", delta);
        }
        */
        
        /* Update Sigma-Delta Modulator */
        R820T2_i2c_write_reg(0x15, (uint8_t)(sdm & 0xff));
        R820T2_i2c_write_reg(0x16, (uint8_t)(sdm >> 8));

        /* Enable frac pll */
        R820T2_i2c_write_cache_mask(0x12, 0x00, 0x08);
    }
}

/*
 * Update Tracking Filter and LO to frequency
 */
void R820T2_set_freq(uint32_t freq)
{
  uint32_t lo_freq = freq + r820t_if_freq;

  R820T2_set_tf(freq);
  R820T2_set_pll(lo_freq);
  r820t_freq = freq;
}

/*
 * Update LNA Gain
 */
void R820T2_set_lna_gain(uint8_t gain_index)
{
  R820T2_i2c_write_cache_mask(0x05, gain_index, 0x0f);
}

/*
 * Update Mixer Gain
 */
void R820T2_set_mixer_gain(uint8_t gain_index)
{
  R820T2_i2c_write_cache_mask(0x07, gain_index, 0x0f);
}

/*
 * Update VGA Gain - IF vga gain controlled by vga_code[3:0] in R12
 */
void R820T2_set_vga_gain_VGA_code(uint8_t gain_index)
{
  R820T2_i2c_write_cache_mask(0x0c, gain_index, 0x0f);
}

//lookup table for IF VGA gain - calculated based on curve taken from R820T2 datasheet
const uint16_t TIM_CCR_reg_lookup_table[] = {
1509,1560,1610,1659,1693,1727,1760,1794,1828,1854,1875,1897,1919,1940,
1962,1984,2005,2035,2076,2117,2157,2198,2227,2254,2282,2310,2338,2365,
2384,2393,2403,2412,2422,2431,2441,2450,2460,2469,2479,2488,2498,2507,
2517,2527,2538,2549,2559,2570,2580,2591,2601,2612,2623,2633,2644,2654,
2665,2677,2691,2704,2718,2732,2745,2759,2773,2786,2800,2814,2827,2841,
2855,2868,2882,2896,2909,2923,2937,2950,2964,2978,2991,3006,3020,3034,
3048,3063,3077,3091,3106,3120,3134,3149,3163,3175,3187,3199,3211,3223,
3236,3248,3260,3272,3284,3296,3308,3320,3332,3344,3356,3369,3381,3393,
3405,3417,3429,3441,3453,3465,3477,3488,3499,3511,3522,3533,3545,3556,
3567,3578,3590,3601,3612,3624,3634,3645,3656,3666,3677,3688,3699,3709,
3720,3731,3741,3752,3763,3774,3784,3795,3805,3816,3827,3837,3848,3858,
3869,3879,3890,3901,3911,3922,3932,3943,3954,3965,3975,3986,3997,4007,
4018,4029,4040,4050,4061,4072,4085,4097,4110,4123,4136,4148,4161,4174,
4186,4199,4212,4224,4237,4249,4261,4273,4285,4297,4309,4321,4334,4346,
4358,4370,4382,4393,4405,4416,4427,4439,4450,4461,4472,4484,4495,4506,
4517,4529,4540,4552,4564,4576,4588,4601,4613,4625,4637,4649,4661,4673,
4685,4697,4707,4718,4728,4739,4750,4760,4771,4781,4792,4803,4813,4824,
4834,4845,4855,4864,4874,4883,4893,4902,4912,4921,4931,4940,4950,4959,
4969,4978,4988,4996,5004,5013,5021,5030,5038,5047,5055,5064,5072,5081,
5089,5098,5106,5114,5123,5130,5137,5144,5151,5159,5166,5173,5180,5187,
5194,5201,5208,5216,5223,5230,5237,5244,5251,5258,5265,5273,5280,5287,
5294,5301,5308,5315,5322,5330,5337,5344,5351,5358,5365,5373,5381,5389,
5397,5405,5413,5421,5429,5437,5445,5453,5461,5469,5477,5485,5493,5500,
5508,5516,5523,5531,5538,5546,5553,5561,5569,5576,5584,5591,5599,5606,
5614,5622,5629,5637,5646,5654,5663,5671,5680,5688,5697,5705,5714,5722,
5731,5739,5747,5756,5764,5773,5781,5790,5798,5807,5815,5824,5832,5841,
5849,5857,5866,5874,5883,5891,5900,5911,5922,5934,5945,5956,5968,5979,
5990,6001,6013,6024,6035,6047,6058,6070,6082,6094,6107,6119,6131,6143,
6155,6167,6179,6191,6203,6216,6228,6241,6254,6267,6279,6292,6305,6317,
6330,6343,6355,6368,6382,6395,6409,6423,6436,6450,6464,6477,6491,6505,
6518,6532,6546,6559,6573,6587,6600,6614,6628,6641,6655,6669,6682,6697,
6712,6728,6743,6759,6774,6790,6805,6821,6836,6852,6868,6885,6902,6919,
6936,6953,6969,6986,7003,7020,7039,7059,7080,7101,7121,7142,7163,7184,
7204,7225,7246,7266,7287,7308,7328,7349,7370,7397,7423,7450,7476,7503,
7529,7572,7623,7674,7725,7792,7859,7968,8191};

#define N_TIM_CCR_lookup_table 484
#define VGA_gain_max 49.300
const float A_TIM_CCR_reg_scale = -N_TIM_CCR_lookup_table/(1.0 - VGA_gain_max);
const float B_TIM_CCR_reg_scale = N_TIM_CCR_lookup_table/(1.0 - VGA_gain_max);

/*
 * 02-08-2025 - Maciej Fajfer
 * Update VGA Gain - IF vga gain controlled by vagc pin
 * 1...49.3 dB
 * ~0.1 dB steps
 */
void R820T2_set_vga_gain_VAGC_pin(float gain_dB)
{
  int16_t index_value = A_TIM_CCR_reg_scale*gain_dB + B_TIM_CCR_reg_scale;
  if (index_value < 0) index_value = 0;
  if (index_value > N_TIM_CCR_lookup_table-1) index_value = N_TIM_CCR_lookup_table;
  TIM4->CCR2 = TIM_CCR_reg_lookup_table[index_value];
}

/*
 * 02-08-2025 - Maciej Fajfer
 * Enable/Disable VAGC pin
 * value=0 -> Disable VAGC pin
 * value=1 -> Enable VAGC pin
 */
void R820T2_config_VAGC_pin(uint8_t value)
{
  if (value == 0)
	  value = 0x40;
  else
	  value = 0x50;
  R820T2_i2c_write_cache_mask(0x0C, value, 0xFF);
}

/*
 * Enable/Disable LNA AGC
 */
void R820T2_set_lna_agc(uint8_t value)
{
  value = value != 0 ? 0x00 : 0x10;
  R820T2_i2c_write_cache_mask(0x05, value, 0x10);
}

/*
 * Enable/Disable Mixer AGC
 */
void R820T2_set_mixer_agc(uint8_t value)
{
  value = value != 0 ? 0x10 : 0x00;
  R820T2_i2c_write_cache_mask(0x07, value, 0x10);
}

/*
 * Set IF Bandwidth - doesn't work properly
 * bw=15 -> widest
 * bw=0  -> narrowest
 */
void R820T2_set_if_bandwidth(uint8_t bw)
{
    const uint8_t modes[] = { 0xE0, 0x80, 0x60, 0x00 };
    uint8_t a = 0xB0 | (0x0F-(bw & 0x0F));
    uint8_t b = 0x0F | modes[(bw & 0x3) >> 4];
    R820T2_i2c_write_reg(0x0A, a);
    R820T2_i2c_write_reg(0x0B, b);
}

/*
 * Calibrate 
 * Kanged from airspy firmware
 * "inspired by Mauro Carvalho Chehab calibration technique"
 * https://stuff.mit.edu/afs/sipb/contrib/linux/drivers/media/tuners/r820t.c
 * part of r820t_set_tv_standard()
 */
int32_t R820T2_calibrate(void)
{
  int32_t i, cal_code;

  for (i = 0; i < 5; i++)
  {
    /* Set filt_cap */
    R820T2_i2c_write_cache_mask(0x0b, 0x08, 0x60);

    /* set cali clk =on */
    R820T2_i2c_write_cache_mask(0x0f, 0x04, 0x04);

    /* X'tal cap 0pF for PLL */
    R820T2_i2c_write_cache_mask(0x10, 0x00, 0x03);

    /* freq used for calibration */
    R820T2_set_pll(CALIBRATION_LO * 1000);

    /* Start Trigger */
    R820T2_i2c_write_cache_mask(0x0b, 0x10, 0x10);

    HAL_Delay(2);

    /* Stop Trigger */
    R820T2_i2c_write_cache_mask(0x0b, 0x00, 0x10);

    /* set cali clk =off */
    R820T2_i2c_write_cache_mask(0x0f, 0x00, 0x04);

    /* Check if calibration worked */
    cal_code = R820T2_i2c_read_reg_uncached(0x04) & 0x0f;
    if (cal_code && cal_code != 0x0f)
      return 0;
  }

  /* cal failed */
  return -1;
}

/*
 * 02-08-2025 - Maciej Fajfer
 * Checking PLL locked/unlocked
------------------------------------------------------------------------------------
R2		[7]						1
0x02	[6]		VCO_LOCK		0: PLL has not locked, 1: PLL has locked
		[5:0]	VCO_INDICATOR	VCO band
	 							000000: min (1.75 GHz), 111111: max (3.6 GHz)
------------------------------------------------------------------------------------
*/
int32_t R820T2_PLL_lock_check(void)
{
	uint8_t reg_R2, i = 20;

	while(i != 0) //20 attempts for PLL lock check
	{
		reg_R2 = R820T2_i2c_read_reg_uncached(0x02);
		if ( (reg_R2 & 0x40) != 0) return 1; //PLL has locked
		i--;
	}
	return -1; //PLL has not locked
}

/*
 * Initialize the R820T
 */
void R820T2_init(I2C_HandleTypeDef i2c_handle)
{
	hi2cx = i2c_handle;
	uint8_t i;

    /* initialize some operating parameters */
    r820t_xtal_freq = XTAL_FREQ; //28.8 MHz
    r820t_if_freq = 4500000; //4.5 MHz - required for IF amplifier
    r820t_freq = 106100000; //106.1 MHz
    
    /* initialize the device */
    for(i=R820T2_WRITE_START;i<R820T2_NUM_REGS;i++)
        R820T2_i2c_write_reg(i, r82xx_init_array[i]);

    /* Calibrate */
    R820T2_calibrate();
    
    /* set freq after calibrate */
    R820T2_set_freq(r820t_freq);
}


