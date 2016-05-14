/*
 * arch/arm/mach-tegra/lge/panel-l-540p-4-7.c
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <mach/dc.h>
#include <mach/iomap.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/tegra_pwm_bl.h>
#include <linux/regulator/consumer.h>
#include <linux/pwm_backlight.h>
#include <linux/leds.h>
#include <linux/ioport.h>
#include <video/mipi_display.h>

#ifdef CONFIG_MFD_RT8542
#include <linux/mfd/rt8542.h>
#include <linux/rt8542-bled.h>
#endif /* CONFIG_MFD_RT8542 */

#ifdef CONFIG_BACKLIGHT_MAX8831
#include <linux/mfd/max8831.h>
#include <linux/max8831_backlight.h>
#endif

#ifdef CONFIG_BACKLIGHT_LM3528
#include <linux/lm3528.h>
#endif
#include "../gpio-names.h"
#include "../board-panel.h"
#include "../board.h"
#include "../devices.h"
#include <mach/board_lge.h>

#if defined(CONFIG_MACH_PDA)
#define PDA_DSI_PANEL_RESET         0
#define PDA_DC_CTRL_MODE            (TEGRA_DC_OUT_CONTINUOUS_MODE | TEGRA_DC_OUT_INITIALIZED_MODE)
#define PRISM_LOG	0
#else
#define DSI_PANEL_RESET         0
#define DC_CTRL_MODE            (TEGRA_DC_OUT_CONTINUOUS_MODE | \
                                                               TEGRA_DC_OUT_INITIALIZED_MODE)
#define GPIO_DSV_EN	(49) /* TEGRA_GPIO_PG1 */
#endif

static struct regulator *vdd_lcd_s_1v8;
static struct regulator *vdd_lcd_s_2v8;

static struct platform_device *disp_device; //Backlight PWM Prism
static bool dsi_l_540p_4_7_reg_requested;
static bool dsi_l_540p_4_7_gpio_requested;
static bool is_bl_powered;
static struct tegra_dsi_out dsi_l_540p_4_7_pdata;
static unsigned *g_prism_enable;
static u8 *g_prism_aggr;

#ifdef CONFIG_MFD_RT8542
static tegra_dc_bl_output dsi_l_540p_4_7_rt8542_bl_exponential_curve ={
  //12.1mA

	0, 6, 6, 6, 6, 6, 6, 6, 6, 6,
	6, 6, 6, 6, 6, 6, 7, 7, 7, 7,
	7, 8, 8, 8, 8, 8, 9, 9, 9, 9,
	9, 10, 10, 10, 10, 10, 11, 11, 11, 11,
	11, 11, 11, 11, 12, 12, 12, 12, 12, 12,
	13, 13, 13, 13, 13, 14, 14, 14, 14, 14,
	15, 15, 15, 17, 17, 18, 18, 19, 20, 21,
	21, 21, 22, 22, 22, 23, 23, 24, 24, 25,
	25, 26, 26, 28, 28, 29, 29, 30, 30, 31,
	31, 32, 32, 33, 33, 34, 34, 35, 35, 36,
	36, 37, 37, 38, 38, 39, 39, 40, 40, 41,
	41, 41, 42, 43, 45, 47, 49, 50, 51, 51,
	52, 53, 54, 54, 55, 56, 57, 58, 59, 60,
	62, 64, 65, 66, 67, 68, 69, 70, 71, 72,
	73, 75, 77, 79, 81, 83, 85, 87, 88, 89,
	89, 89, 90, 90, 90, 91, 91, 91, 92, 92,
	93, 95, 97, 99, 101, 103, 104, 105, 107, 108,
	110, 111, 112, 113, 115, 116, 117, 119, 120, 122,
	123, 124, 125, 126, 127, 129, 130, 131, 133, 134,
	135, 137, 138, 139, 140, 141, 142, 144, 145, 146,
	147, 148, 149, 154, 156, 157, 159, 161, 164, 167,
	170, 172, 174, 176, 178, 180, 181, 183, 185, 187,
	189, 191, 193, 194, 195, 197, 199, 201, 203, 204,
	205, 205, 206, 207, 209, 211, 212, 214, 216, 218,
	222, 223, 226, 228, 230, 232, 235, 237, 240, 242,
	245, 247, 249, 251, 253, 255,
};
static tegra_dc_bl_output dsi_l_540p_4_7_rt8542_bl_response_curve = {
//tuning table for 12.1mA current
#if 1
	0, 1, 2, 3, 5, 6, 7, 8, 9, 10,
	12, 13,	13, 14,	15, 16,	17, 18,	19, 20,
	21, 22,	23, 24,	25, 26,	27, 27,	28, 29,
	30, 31,	32, 33,	34, 35,	36, 37,	38, 39,
	39, 40,	41, 42,	42, 43,	44, 44,	45, 46,
	46, 47,	48, 49,	50, 51,	52, 53,	54, 55,
	56, 57,	58, 59,	60, 60,	61, 62,	63, 64,
	65, 66, 67, 68,	69, 70,	71, 72,	73, 74,
	75, 75,	76, 77,	78, 79,	80, 81,	82, 83,
	83, 84,	85, 86,	87, 88,	89, 90,	91, 92,
	93, 94,	95, 96,	97, 98,	99, 100, 101, 102,
	103, 104, 105, 106, 107, 108, 109, 110, 111,112,
	113, 114, 115, 116, 117, 118, 118, 119, 120, 121,
	122, 123, 124, 125, 126, 127, 128, 129, 130, 131,
	132, 133, 134, 135, 137, 138, 139, 140, 141, 142,
	143, 144, 145, 146, 147, 148, 149, 150, 151, 152,
	153, 154, 155, 156, 157, 158, 159, 160, 161, 162,
	163, 164, 165, 166, 167, 168, 169, 171, 172, 173,
	174, 175, 176, 177, 178, 179, 180, 181, 182, 183,
	184, 185, 187, 188, 189, 190, 191, 192, 193, 194,
	195, 196, 197, 198, 199, 200, 201, 202, 203, 204,
	205, 206, 208, 209, 210, 211, 212, 213, 214, 215,
	216, 217, 218, 220, 221, 222, 223, 224, 225, 226,
	227, 229, 230, 231, 232, 233, 234, 235, 236, 237,
	238, 240, 241, 242, 243, 244, 245, 246, 247, 248,
	249, 251, 252, 253, 254, 255,
#else //linear table for initial tuning
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
	11, 12, 13, 14, 15, 16,	17, 18,	19, 20,
	21, 22,	23, 24,	25, 26,	27, 28,	29, 30,
	31, 32,	33, 34,	35, 36,	37, 38,	39, 40,
	41, 42,	43, 44,	45, 46,	47, 48,	49, 50,
	51, 52,	53, 54,	55, 56,	57, 58,	59, 60,
	61, 62,	63, 64,	65, 66,	67, 68,	69, 70,
	71, 72,	73, 74,	75, 76,	77, 78,	79, 80,
	81, 82,	83, 84,	85, 86,	87, 88,	89, 90,
	91, 92,	93, 94,	95, 96,	97, 98,	99, 100,
	101,102,103,104,105,106,107,108,109,110,
	111,112,113,114,115,116,117,118,119,120,
	121,122,123,124,125,126,127,128,129,130,
	131,132,133,134,135,136,137,138,139,140,
	141,142,143,144,145,146,147,148,149,150,
	151,152,153,154,155,156,157,158,159,160,
	161,162,163,164,165,166,167,168,169,170,
	171,172,173,174,175,176,177,178,179,180,
	181,182,183,184,185,186,187,188,189,190,
	191,192,193,194,195,196,197,198,199,200,
	201,202,203,204,205,206,207,208,209,210,
	211,212,213,214,215,216,217,218,219,220,
	221,222,223,224,225,226,227,228,229,230,
	231,232,233,234,235,236,237,238,239,240,
	241,242,243,244,245,246,247,248,249,250,
	251,252,253,254,255
#endif
};
#endif
#ifdef CONFIG_BACKLIGHT_MAX8831
static tegra_dc_bl_output dsi_l_540p_4_7_max8831_bl_response_curve = {
	0, 2, 5, 7, 10, 13, 15, 18,
	20, 23, 26, 27, 29, 30, 31, 33,
	34, 36, 37, 39, 40, 41, 42, 44,
	45, 46, 47, 48, 50, 51, 52, 53,
	54, 55, 56, 57, 58, 59, 60, 61,
	62, 63, 64, 65, 66, 67, 68, 69,
	70, 71, 73, 74, 75, 76, 78, 79,
	80, 82, 83, 84, 86, 86, 87, 88,
	89, 89, 90, 91, 92, 92, 93, 94,
	95, 96, 97, 98, 99, 100, 101, 102,
	103, 104, 105, 106, 107, 107, 108, 109,
	110, 111, 112, 112, 113, 114, 114, 115,
	115, 116, 117, 117, 118, 119, 120, 121,
	121, 122, 123, 124, 125, 126, 127, 128,
	129, 130, 131, 132, 133, 134, 135, 136,
	136, 138, 139, 140, 141, 142, 143, 144,
	145, 146, 147, 148, 149, 150, 151, 152,
	153, 154, 155, 155, 156, 157, 158, 159,
	161, 162, 163, 164, 165, 166, 167, 167,
	167, 167, 168, 168, 168, 168, 168, 169,
	169, 170, 171, 172, 172, 173, 174, 175,
	176, 177, 178, 179, 180, 181, 182, 183,
	184, 184, 185, 186, 187, 188, 189, 190,
	191, 192, 193, 194, 195, 195, 196, 197,
	198, 199, 200, 201, 202, 203, 204, 205,
	206, 206, 207, 207, 208, 208, 209, 209,
	210, 211, 211, 212, 213, 213, 214, 215,
	216, 216, 217, 218, 219, 220, 221, 222,
	223, 224, 225, 226, 227, 228, 229, 230,
	231, 232, 233, 235, 236, 237, 238, 239,
	240, 241, 242, 243, 244, 245, 246, 247,
	248, 249, 250, 251, 252, 253, 254, 255
};
#endif

#ifdef CONFIG_BACKLIGHT_LM3528
static tegra_dc_bl_output dsi_l_540p_4_7_lm3528_bl_response_curve = {
	0, 26, 55, 72, 83, 93, 100, 107,
	112, 117, 121, 125, 129, 132, 135, 138,
	141, 143, 146, 148, 150, 152, 154, 156,
	157, 159, 161, 162, 164, 165, 167, 168,
	169, 171, 172, 173, 174, 175, 176, 177,
	179, 180, 181, 182, 182, 183, 184, 185,
	186, 187, 188, 189, 189, 190, 191, 192,
	192, 193, 194, 195, 195, 196, 197, 197,
	198, 199, 199, 200, 200, 201, 202, 202,
	203, 203, 204, 205, 205, 206, 206, 207,
	207, 208, 208, 209, 209, 210, 210, 211,
	211, 212, 212, 212, 213, 213, 214, 214,
	215, 215, 216, 216, 216, 217, 217, 218,
	218, 218, 219, 219, 220, 220, 220, 221,
	221, 221, 222, 222, 223, 223, 223, 224,
	224, 224, 225, 225, 225, 226, 226, 226,
	227, 227, 227, 228, 228, 228, 228, 229,
	229, 229, 230, 230, 230, 231, 231, 231,
	231, 232, 232, 232, 233, 233, 233, 233,
	234, 234, 234, 234, 235, 235, 235, 236,
	236, 236, 236, 237, 237, 237, 237, 238,
	238, 238, 238, 239, 239, 239, 239, 240,
	240, 240, 240, 240, 241, 241, 241, 241,
	242, 242, 242, 242, 242, 243, 243, 243,
	243, 244, 244, 244, 244, 244, 245, 245,
	245, 245, 245, 246, 246, 246, 246, 246,
	247, 247, 247, 247, 247, 248, 248, 248,
	248, 248, 249, 249, 249, 249, 249, 250,
	250, 250, 250, 250, 250, 251, 251, 251,
	251, 251, 252, 252, 252, 252, 252, 252,
	253, 253, 253, 253, 253, 253, 254, 254,
	254, 254, 254, 254, 255, 255, 255, 255
};
#endif

static p_tegra_dc_bl_output dsi_l_540p_4_7_bl_response_curve;

static u8 revise_aggr[] = { 255, 253, 251, 251, 251, 248 };
static int __maybe_unused dsi_l_540p_4_7_bl_notify(struct device *unused,
							int brightness)
{

	int cur_sd_brightness = atomic_read(&sd_brightness);
#if PRISM_LOG
	int exp_br, cal_br, last_br, ori_br = brightness;
	int ori_sd_br = cur_sd_brightness;
#endif

	/*                                                               */
	brightness = dsi_l_540p_4_7_rt8542_bl_exponential_curve[brightness];
#if PRISM_LOG
	exp_br = brightness;
#endif

	/* SD brightness is a percentage */
	if (*g_prism_enable && *g_prism_aggr <= 5) {
		cur_sd_brightness = (cur_sd_brightness * 255) /
			revise_aggr[*g_prism_aggr];
		brightness = (brightness * cur_sd_brightness) / 255;
	}/* else
		pr_info("%s prism %s, aggressiveness %d\n", __func__,
			*g_prism_enable ? "enabled" : "disabled",
			*g_prism_aggr);
	*/
#if PRISM_LOG
	cal_br = brightness;
#endif

	/* Apply any backlight response curve */
	if (brightness > 255)
		pr_info("Error: SD Cal Brightness > 255!\n");
	else
		brightness = dsi_l_540p_4_7_bl_response_curve[brightness];
#if PRISM_LOG
	last_br = brightness;
	pr_info("##@%s prism %s, aggr(%d), ori(%d), sd_br(%d, %d), exp(%d), " \
		"cal(%d), last_br(%d)\n", __func__,
		*g_prism_enable ? "enabled" : "disabled", *g_prism_aggr,
		ori_br, cur_sd_brightness, ori_sd_br, exp_br, cal_br, last_br);
#endif
	return brightness;
}
static bool __maybe_unused dsi_l_540p_4_7_check_bl_power(void)
{
	return is_bl_powered;
}

#ifdef CONFIG_MFD_RT8542
static struct rt8542_bled_data rt8542_bled_data = {
	.BLED_CONTROL1 = {
		.bitfield = {
			.BLED_MAX_CURRENT = BLED_MAX_8P4mA, //BLED_MAX_12P1mA,//BLED_MAX_20mA,
			.BLED_PWM_CONFIG = PWM_ACTIVE_HIGH,
			.BLED_MAPPING_MODE = BLED_MAPPING_LINEAR,
			.BLED_OVP = BLED_OVP_32V,
		},
	},
	.BLED_CONTROL2 = {
		.bitfield = {
			.BLED_RAMPUP_RATE = BLED_RAMPUP_4mS,
			.BLED_RAMPDOWN_RATE = BLED_RAMPDOWN_32uS,
			.BLED_SW_FREQ = BLED_FREQ_1MHZ,
		}
	},
	.BLED_EXP_BRIGHT = {
		.bitfield = {
			.BLED_EXP_BRIGHTNESS = 0,
		},
	},
	.BLED_LIN_BRIGHT = {
		.bitfield = {
#if defined(CONFIG_BACKLIGHT_PWM_PRISM) || defined(CONFIG_BACKLIGHT_PWM_CABC)
			.BLED_LIN_BRIGHTNESS = 0x79, //127,
#else
			.BLED_LIN_BRIGHTNESS = 126,//100,
#endif
		},
	},
	.FLASH_CONTROL2 = {
		.bitfield = {
#if defined(CONFIG_BACKLIGHT_PWM_PRISM) || defined(CONFIG_BACKLIGHT_PWM_CABC)
			.PWM_ENABLE = 1,
#else
			.PWM_ENABLE = 0,
#endif
		},
	},
}; /* rt8542_bled_data end */

static struct rt8542_fled_data rt8542_fled_data = {
	.FLED_CURR_CONTROL = {
		.bitfield = {
			.FLED_STROBE_CURRENT = FLED_STROBE_618mA, //Set camera flash current
			.FLED_TORCH_CURRENT = FLED_TORCH_112mA, //Set camera torch current
		},
	},
	.FLED_CONTROL1 = {
		.bitfield = {
			.FLED_STROBE_TIMEOUT = 0x1a, //Set camera flash timeout
			.FLED_CURRENT_LIMIT = FLED_CURR_2P5A,
			.FLED_SW_FREQ = FLED_FREQ_2MHZ,
		},
	},
	.FLED_VIN_MONITOR = {
		.bitfield = {
			.FLED_VIN_MONITOR = FLED_VIN_2P7V,
		},
	},
	.FLED_CONTROL2 = {
		.bitfield = {
			.VIN_MONITOR_ENABLE = 1,
			.VIN_MONITOR_MODE = VIN_MONITOR_TORCH,
			.TX_ENABLE = 1,
			.TX_POLARITY = TX_ACTIVE_HIGH,
			/* external strobe enable flag */
			.STROBE_ENABLE = 1,
			.STROBE_POLARITY = STROBE_ACTIVE_HIGH,
		},
	},
	.strobe_gpio = TEGRA_GPIO_PS1,
}; /* rt8542_fled_data end */

static struct rt8542_ledch_data rt8542_ledch_data = {
	.CH_ENABLE = {
		.bitfield = {
#if defined(CONFIG_MACH_PDA)
			.BL_EN = 1,
#endif
			.BLED2_EN = 1,
			.BLED1_EN = 1,
			.FLED2_EN = 1,
			.FLED1_EN = 1,
		},
	},
}; /* rt8542_ledch_data end */

static struct rt8542_platform_data rt8542_platform_data = {
	.bled_data = &rt8542_bled_data,
	.fled_data = &rt8542_fled_data,
	.ledch_data = &rt8542_ledch_data,
	.en_pin = TEGRA_GPIO_PG3, //-1,
};

static struct i2c_board_info __initdata rt8542_device_info[] = {
	{
		I2C_BOARD_INFO(RT8542_DEVICE_NAME, 0x39),
		.platform_data = &rt8542_platform_data,
	},
};
#if defined(CONFIG_BACKLIGHT_PWM_PRISM)
static int dsi_l_540p_4_7_check_fb(struct device *dev, struct fb_info *info)
{
	return info->device == &disp_device->dev;
}

static struct platform_pwm_backlight_data dsi_l_540p_4_7_bl_data = {
	.pwm_id         = 0,
	.max_brightness = 255,
	.dft_brightness = 126,	//224,
	.pwm_period_ns  = 1000000,
	.notify         = dsi_l_540p_4_7_bl_notify,
	/* Only toggle backlight on fb blank notifications for disp1 */
	.check_fb       = dsi_l_540p_4_7_check_fb,
};

static struct platform_device __maybe_unused
	dsi_l_540p_4_7_bl_device = {
		.name   = "pwm-backlight",
		.id     = -1,
		.dev    = {
		.platform_data = &dsi_l_540p_4_7_bl_data,
	},
};

static struct platform_device __maybe_unused
		*dsi_l_540p_4_7_bl_devices[] __initdata = {
	    &tegra_pwfm0_device,
	    &dsi_l_540p_4_7_bl_device,
};
#endif
#endif /* CONFIG_MFD_RT8542 */

#ifdef CONFIG_BACKLIGHT_MAX8831
/*
	Sharp uses I2C max8831 blacklight device
*/
static struct led_info dsi_l_540p_4_7_max8831_leds[] = {
	[MAX8831_ID_LED3] = {
		.name = "max8831:red:pluto",
	},
	[MAX8831_ID_LED4] = {
		.name = "max8831:green:pluto",
	},
	[MAX8831_ID_LED5] = {
		.name = "max8831:blue:pluto",
	},
};

static struct platform_max8831_backlight_data dsi_l_540p_4_7_max8831_bl_data = {
	.id	= -1,
	.name	= "pluto_display_bl",
	.max_brightness	= MAX8831_BL_LEDS_MAX_CURR,
	.dft_brightness	= 100,
	.notify	= dsi_l_540p_4_7_bl_notify,
	.is_powered = dsi_l_540p_4_7_check_bl_power,
};

static struct max8831_subdev_info dsi_l_540p_4_7_max8831_subdevs[] = {
	{
		.id = MAX8831_ID_LED3,
		.name = "max8831_led_bl",
		.platform_data = &dsi_l_540p_4_7_max8831_leds[MAX8831_ID_LED3],
		.pdata_size = sizeof(
				dsi_l_540p_4_7_max8831_leds[MAX8831_ID_LED3]),
	}, {
		.id = MAX8831_ID_LED4,
		.name = "max8831_led_bl",
		.platform_data = &dsi_l_540p_4_7_max8831_leds[MAX8831_ID_LED4],
		.pdata_size = sizeof(
				dsi_l_540p_4_7_max8831_leds[MAX8831_ID_LED4]),
	}, {
		.id = MAX8831_ID_LED5,
		.name = "max8831_led_bl",
		.platform_data = &dsi_l_540p_4_7_max8831_leds[MAX8831_ID_LED5],
		.pdata_size = sizeof(
				dsi_l_540p_4_7_max8831_leds[MAX8831_ID_LED5]),
	}, {
		.id = MAX8831_BL_LEDS,
		.name = "max8831_display_bl",
		.platform_data = &dsi_l_540p_4_7_max8831_bl_data,
		.pdata_size = sizeof(dsi_l_540p_4_7_max8831_bl_data),
	},
};

static struct max8831_platform_data dsi_l_540p_4_7_max8831 = {
	.num_subdevs = ARRAY_SIZE(dsi_l_540p_4_7_max8831_subdevs),
	.subdevs = dsi_l_540p_4_7_max8831_subdevs,
};

static __maybe_unused struct i2c_board_info dsi_l_540p_4_7_i2c_led_info = {
	.type		= "max8831",
	.addr		= 0x4d,
	.platform_data	= &dsi_l_540p_4_7_max8831,
};


static unsigned int dsi_s_pluto_edp_states[] = {
	1130, 1017, 904, 791, 678, 565, 452, 339, 226, 113, 0
};
static unsigned int dsi_s_pluto_edp_brightness[] = {
	255, 230, 204, 179, 153, 128, 102, 77, 51, 10, 0
};
static unsigned int dsi_s_pda_edp_states[] = {
	720, 644, 523, 490, 442, 427, 395, 363, 330, 299, 0
};
static unsigned int dsi_s_pda_edp_brightness[] = {
	255, 230, 204, 170, 140, 128, 102, 77, 51, 10, 0
};
static unsigned int dsi_s_atlantis_edp_states[] = {
	720, 644, 523, 490, 442, 427, 395, 363, 330, 299, 0
};
static unsigned int dsi_s_atlantis_edp_brightness[] = {
	255, 230, 204, 170, 140, 128, 102, 77, 51, 10, 0
};
#endif

#ifdef CONFIG_BACKLIGHT_LM3528
static struct lm3528_platform_data lm3528_pdata = {
	.dft_brightness	= 200,
	.is_powered = dsi_l_540p_4_7_check_bl_power,
	.notify = dsi_l_540p_4_7_bl_notify,
};

static __maybe_unused struct i2c_board_info
	lm3528_dsi_l_540p_4_7_i2c_led_info = {
	.type		= "lm3528_display_bl",
	.addr		= 0x36,
	.platform_data	= &lm3528_pdata,
};
#endif
static int __init dsi_l_540p_4_7_register_bl_dev(void)
{
#if defined(CONFIG_MACH_PDA)
#if defined(CONFIG_BACKLIGHT_PWM_PRISM)
      int err;
	err = platform_add_devices(dsi_l_540p_4_7_bl_devices,
			ARRAY_SIZE(dsi_l_540p_4_7_bl_devices));
	if (err) {
		pr_err("disp1 bl device registration failed");
		return err;
	}
#endif
	dsi_l_540p_4_7_bl_response_curve = dsi_l_540p_4_7_rt8542_bl_response_curve;
/*                                                                                            
                         
                         
                          
                                                                                             */
	if(lge_get_board_revno() == HW_REV_PDA_B)
		i2c_register_board_info(1, rt8542_device_info, 1);
	else //rev C
		i2c_register_board_info(5, rt8542_device_info, 1);
	return 0;
#else
	struct i2c_board_info *bl_info;
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	switch (board_info.board_id) {
	case BOARD_E1670: /* Atlantis ERS */
	case BOARD_E1671: /* Atlantis POP Socket */
	case BOARD_E1740: /* Atlantis FFD */
		lm3528_pdata.edp_states = dsi_s_atlantis_edp_states;
		lm3528_pdata.edp_brightness = dsi_s_atlantis_edp_brightness;
		bl_info = &lm3528_dsi_l_540p_4_7_i2c_led_info;
		dsi_l_540p_4_7_bl_response_curve =
			dsi_l_540p_4_7_lm3528_bl_response_curve;
		break;
	case BOARD_E1680: /* Ceres ERS */
	case BOARD_E1681: /* Ceres DSC Socket */
	case BOARD_E1690: /* Ceres FFD */
		dsi_l_540p_4_7_max8831_bl_data.edp_states =
			dsi_s_pda_edp_states;
		dsi_l_540p_4_7_max8831_bl_data.edp_brightness =
				dsi_s_pda_edp_brightness;
		bl_info = &dsi_l_540p_4_7_i2c_led_info;
		dsi_l_540p_4_7_bl_response_curve =
			dsi_l_540p_4_7_max8831_bl_response_curve;
		break;
	case BOARD_E1580: /* Pluto */
	/* fall through */
	default:
		dsi_l_540p_4_7_max8831_bl_data.edp_states =
			dsi_s_pluto_edp_states;
		dsi_l_540p_4_7_max8831_bl_data.edp_brightness =
				dsi_s_pluto_edp_brightness;
		bl_info = &dsi_l_540p_4_7_i2c_led_info;
		dsi_l_540p_4_7_bl_response_curve =
			dsi_l_540p_4_7_max8831_bl_response_curve;
		break;
	}

	return i2c_register_board_info(1, bl_info, 1);
#endif
}

//Backlight PWM Prism
static void dsi_l_540p_4_7_set_disp_device(
	struct platform_device *display_device)
{
	disp_device = display_device;
}

#if defined(CONFIG_MACH_PDA)
struct tegra_dc_mode dsi_l_540p_4_7_modes_tovis_revB[] = {
	/* 540x960@60Hz  TOVIS*/
	{
		.pclk = 36427680,
		.h_ref_to_sync = 2, //mdss-dsi-h-sync-pulse 0
		.v_ref_to_sync = 1,
		.h_sync_width = 10, //mdss-dsi-h-pulse-width
		.v_sync_width = 4, //mdss-dsi-v-pulse-width
		.h_back_porch = 47,
		.h_front_porch = 20,
		.v_back_porch = 8,
		.v_front_porch = 12,
		.h_active = 540,
		.v_active = 960,
	},
};

struct tegra_dc_mode dsi_l_540p_4_7_modes_tovis[] = {
	/* 540x960@60Hz  TOVIS*/
	{
		.pclk = 37540800,
		.h_ref_to_sync = 0, //mdss-dsi-h-sync-pulse 0
		.v_ref_to_sync = 1,
		.h_sync_width = 40, //mdss-dsi-h-pulse-width
		.v_sync_width = 2, //mdss-dsi-v-pulse-width
		.h_back_porch = 32,
		.h_front_porch = 20,
		.v_back_porch = 15,
		.v_front_porch = 13,
		.h_active = 540,
		.v_active = 960,
	},
};
struct tegra_dc_mode dsi_l_540p_4_7_modes_tianma[] = {
	/* 540x960@60Hz TIANMA*/
	{
		.pclk = 37507440,
		.h_ref_to_sync = 0, //mdss-dsi-h-sync-pulse 2 -> 0
		.v_ref_to_sync = 1,
		.h_sync_width = 44, //mdss-dsi-h-pulse-width
		.v_sync_width = 4, //mdss-dsi-v-pulse-width
		.h_back_porch = 30,
		.h_front_porch = 20,
		.v_back_porch = 15,
		.v_front_porch = 7,
		.h_active = 540,
		.v_active = 960,
	},
};

#else
struct tegra_dc_mode dsi_l_540p_4_7_modes[] = {
	/* 540x960@60Hz  TOVIS*/
	{
		.pclk = 36427680,
		.h_ref_to_sync = 2, //mdss-dsi-h-sync-pulse 0
		.v_ref_to_sync = 1,
		.h_sync_width = 10, //mdss-dsi-h-pulse-width
		.v_sync_width = 4, //mdss-dsi-v-pulse-width
		.h_back_porch = 47,
		.h_front_porch = 20,
		.v_back_porch = 8,
		.v_front_porch = 12,
		.h_active = 540,
		.v_active = 960,
	},
};
#endif
static int dsi_l_540p_4_7_reg_get(void)
{
	int err = 0;

	if (dsi_l_540p_4_7_reg_requested)
		return 0;

	vdd_lcd_s_1v8 = regulator_get(NULL, "vdd_lcd_1v8_s");
	if (IS_ERR_OR_NULL(vdd_lcd_s_1v8)) {
		pr_err("vdd_lcd_1v8_s regulator get failed\n");
		err = PTR_ERR(vdd_lcd_s_1v8);
		vdd_lcd_s_1v8 = NULL;
		goto fail;
	}

	vdd_lcd_s_2v8 = regulator_get(NULL, "avdd_lcd");
	if (IS_ERR_OR_NULL(vdd_lcd_s_2v8)) {
		pr_err("vdd_lcd_2v8_s regulator get failed\n");
		err = PTR_ERR(vdd_lcd_s_2v8);
		vdd_lcd_s_2v8 = NULL;
		goto fail;
	}

	dsi_l_540p_4_7_reg_requested = true;
	return 0;
fail:
	return err;
}

static int dsi_l_540p_4_7_gpio_get(void)
{
	int err = 0;

	if (dsi_l_540p_4_7_gpio_requested)
		return 0;

	err = gpio_request(dsi_l_540p_4_7_pdata.dsi_panel_rst_gpio, "panel rst");
	if (err < 0) {
		pr_err("panel reset gpio request failed\n");
		goto fail;
	}
#if defined(CONFIG_MACH_PDA)
	err = gpio_request(dsi_l_540p_4_7_pdata.dsi_panel_dsv_en_gpio, "DSV_en");
#else
	err = gpio_request(GPIO_DSV_EN, "DSV_en");
#endif
	if (err < 0) {
		pr_err("DSV_en gpio request failed\n");
		goto fail;
	}
/*
	err = gpio_request(dsi_l_540p_4_7_pdata.dsi_panel_bl_en_gpio, "bl_en");
	if (err < 0) {
		pr_err("bl enable gpio request failed\n");
		goto fail;
	}
*/
#if defined(CONFIG_BACKLIGHT_PWM_PRISM)
	err = gpio_request(dsi_l_540p_4_7_pdata.dsi_panel_bl_pwm_gpio, "bl_pwm");
	if (err < 0) {
		pr_err("bl pwm gpio request failed\n");
		goto fail;
	}
#endif
	dsi_l_540p_4_7_gpio_requested = true;
	return 0;
fail:
	return err;
}

static int dsi_l_540p_4_7_enable(struct device *dev)
{
	int err = 0;

	printk("##@%s\n", __func__);
	err = dsi_l_540p_4_7_reg_get();
	if (err < 0) {
		pr_err("dsi regulator get failed\n");
		goto fail;
	}

	err = dsi_l_540p_4_7_gpio_get();
	if (err < 0) {
		pr_err("dsi gpio request failed\n");
		goto fail;
	}

#if defined(CONFIG_MACH_PDA)
	if(lge_get_board_revno() == HW_REV_PDA_B){
		//pr_err("DSV : dsi_panel_dsv_en_gpio -> 0\n");
		gpio_direction_output(dsi_l_540p_4_7_pdata.dsi_panel_dsv_en_gpio, 0);
	}
#else
	/* enble DSV_EN */
	/* DNI: DSV_EN is not required for current panel
		if required, it should be set to 1
	*/
	gpio_direction_output(GPIO_DSV_EN, 0);
#endif

	if (vdd_lcd_s_1v8) {
		err = regulator_enable(vdd_lcd_s_1v8);
		if (err < 0) {
			pr_err("vdd_lcd_1v8_s regulator enable failed\n");
			goto fail;
		}
	}
	udelay(10);

	if (vdd_lcd_s_2v8) {
		err = regulator_enable(vdd_lcd_s_2v8);
		if (err < 0) {
			pr_err("vdd_lcd_2v8_s regulator enable failed\n");
			goto fail;
		}
	}
	mdelay(10);

#if defined(CONFIG_MACH_PDA)
#if PDA_DSI_PANEL_RESET
	//pr_err("Reset : dsi_panel_rst_gpio\n");
	gpio_direction_output(dsi_l_540p_4_7_pdata.dsi_panel_rst_gpio, 1);
	msleep(1);
	gpio_direction_output(dsi_l_540p_4_7_pdata.dsi_panel_rst_gpio, 0);
	msleep(100);
	gpio_direction_output(dsi_l_540p_4_7_pdata.dsi_panel_rst_gpio, 1);
	msleep(10);
#endif
#else
#if DSI_PANEL_RESET
	/* TODO: check rst high timing. */
	gpio_direction_output(dsi_l_540p_4_7_pdata.dsi_panel_rst_gpio, 1);
	///sjoo// usleep_range(1000,1500);
	msleep(10);
	gpio_direction_output(dsi_l_540p_4_7_pdata.dsi_panel_rst_gpio, 0);
	///sjoo// usleep_range(1000,1500);
	msleep(20);
	gpio_direction_output(dsi_l_540p_4_7_pdata.dsi_panel_rst_gpio, 1);
	///sjoo// usleep_range(10000, 11000);
	msleep(20);

#endif
#endif

	/* BL on */
//	gpio_direction_output(dsi_l_540p_4_7_pdata.dsi_panel_bl_en_gpio, 1);
//	rt8542_bl_enable(true);
	is_bl_powered = true;

	return 0;
fail:
	return err;
}

/* init. code for Tianma panel */
static u8 set_ext_tianma[] = {0xB9, 0xFF, 0x83, 0x89}; // 5ms delay
static u8 set_mipi_tianma[] = {0xBA, 0x41, 0x93}; // 10ms delay
static u8 set_tianma[] = {0xC6, 0x08};
static u8 set_pwr_opt_tianma[] = {0xDE, 0x05, 0x58, 0x10}; // 10ms delay
static u8 set_pwr1_tianma[] = {0xB1, 0x00, 0x00, 0x07, 0x00, 0x00, 0x10, 0x11, 0x8F, 0xEF, \
                       0x2C, 0x34, 0x3F, 0x3F, 0x42, 0x01, 0x32, 0xF7, 0x20, 0x80}; // 5ms delay
static u8 set_display_tianma[] = {0xB2, 0x00, 0x00, 0x78, 0x0F, 0x05, 0x3F, 0x20}; // 5ms delay
static u8 set_cyc_tianma[] = {0xB4, 0x80, 0x08, 0x00, 0x32, 0x10, 0x02, 0x32, 0x10, 0x07, \
                       0x32, 0x10, 0x00, 0x37, 0x05, 0x40, 0x0B, 0x37, 0x05, 0x38, \
                       0x14, 0xFF, 0x53, 0x0A}; // 5ms delay
static u8 set_gip_tianma[] = {0xD5, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x60, \
                       0x00, 0x99, 0x88, 0x88, 0x88, 0x88, 0x23, 0x88, 0x01, 0x88, \
                       0x67, 0x88, 0x45, 0x45, 0x67, 0x88, 0x01, 0x88, 0x88, 0x88, \
                       0x88, 0x99, 0x88, 0x88, 0x88, 0x54, 0x88, 0x76, 0x88, 0x10, \
                       0x88, 0x32, 0x54, 0x76, 0x88, 0x10, 0x88, 0x88, 0x88}; // 5ms delay
static u8 set_gamma_tianma[] = {0xE0, 0x00, 0x12, 0x16, 0x29, 0x2F, 0x3F, 0x2A, 0x47, 0x07, \
				0x0E, 0x12, 0x16, 0x18, 0x14, 0x14, 0x17, 0x19, 0x00, 0x12, \
				0x15, 0x2C, 0x31, 0x3F, 0x2A, 0x46, 0x07, 0x0D, 0x11, 0x14, \
				0x16, 0x13, 0x14, 0x17, 0x19}; // 5ms delay
static u8 set_src_opt_tianma[] = {0xC0, 0x43, 0x17};
static u8 set_dgc_tianma[] = {0xC1, 0x01, 0x00, 0x08, 0x10, 0x1A, 0x20, 0x29, 0x30, 0x38, \
				0x40, 0x48, 0x50, 0x58, 0x5F, 0x68, 0x71, 0x7B, 0x81, 0x88, \
				0x91, 0x99, 0x9F, 0xA7, 0xB1, 0xB8, 0xBF, 0xC5, 0xCB, 0xD1, \
				0xDB, 0xE3, 0xE9, 0xF1, 0xF9, 0x10, 0x65, 0xCA, 0xE0, 0x67, \
				0xC0, 0x2C, 0xA1, 0xC0, 0x00, 0x02, 0x0B, 0x15, 0x1C, 0x23, \
				0x2D, 0x34, 0x3B, 0x42, 0x4A, 0x52, 0x5A, 0x62, 0x6A, 0x73, \
				0x7B, 0x82, 0x8A, 0x91, 0x99, 0xA0, 0xA9, 0xB3, 0xB9, 0xC1, \
				0xC8, 0xCE, 0xD6, 0xDF, 0xE5, 0xED, 0xF4, 0x10, 0x71, 0x20, \
				0x44, 0x57, 0xEC, 0xE9, 0x52, 0x00, 0x06, 0x0F, 0x17, 0x1E, \
				0x25, 0x2D, 0x34, 0x3C, 0x44, 0x4B, 0x53, 0x5A, 0x62, 0x6A, \
				0x73, 0x7C, 0x83, 0x89, 0x92, 0x99, 0xA2, 0xAB, 0xB4, 0xBD, \
				0xC5, 0xCB, 0xD1, 0xDA, 0xE4, 0xE9, 0xF2, 0xFA, 0xFF, 0x10, \
				0x65, 0xCA, 0x60, 0x67, 0xC8, 0x2C, 0xAB, 0xC0};
static u8 set_chesel_tianma[] = {0xE6, 0x00};
static u8 set_chemode_sta_tianma[] = {0xE3, 0x01};
static u8 set_che_tianma[] = {0xE5, 0x00, 0x0C, 0x15, 0x07, 0x04, 0x00, 0x80, 0x20, 0x00, \
						0x10, 0x00, 0x00, 0x08, 0x06, 0x04, 0x00, 0x80, 0x0E};
static u8 set_cs_ring_tianma[] = {0xE2, 0x40, 0x00, 0x00, 0x05, 0x05, 0x04, 0x04, 0x04, 0x04, \
						0x04, 0x08, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10};
static u8 set_vcom_tianma[] = {0xB6, 0x00, 0x83, 0x00, 0x83}; // 5ms delay
static u8 set_gip_forward_tianma[] = {0xCC, 0x02};
static u8 set_te_func_tianma[] = {0xB7, 0x00, 0x00, 0x50};

static u8 set_sequence1_tianma[] = {0xB0, 0x00, 0x01, 0x00, 0x00};
static u8 set_sequence2_tianma[] = {0xCF, 0x00, 0x1D, 0x03};
static u8 set_vgh_en_tianma[] = {0xB0, 0x00, 0x01, 0x08, 0x04}; // 10ms delay
static u8 set_vgl_en_tianma[] = {0xB0, 0x00, 0x01, 0x18, 0x04}; // 25ms delay
static u8 set_vcl_en_tianma[] = {0xB0, 0x00, 0x01, 0x1C, 0x04}; // 5ms delay
static u8 set_vgh_vgl_pump1_tianma[] = {0xCF, 0x00, 0x15, 0x03}; // 5ms delay
static u8 set_vgh_vgl_pump2_tianma[] = {0xCF, 0x00, 0x01, 0x03};
static u8 set_vsp_en_tianma[] = {0xB0, 0x00, 0x01, 0x3C, 0x04};
static u8 set_vsn_en1_tianma[] = {0xB0, 0x00, 0x01, 0x7C, 0x04};

static u8 set_vsn_en2_tianma[] = {0xCF, 0x00, 0x00, 0x03};
static u8 set_gip_src_en_tianma[] = {0xB0, 0x00, 0x01, 0x7C, 0x0E}; // 5ms delay
static u8 set_sequence3_tianma[] = {0xB0, 0x00, 0x01, 0x7C, 0x0F}; // 5ms delay
static u8 set_pwr2_tianma[] = {0xB1, 0x00, 0x00, 0x07, 0xE1, 0x94, 0x10, 0x11, 0x8F, 0xEF, \
						0x26, 0x2E, 0x3F, 0x3F, 0x42, 0x01, 0x32, 0xF7, 0x20, 0x80}; // 5ms delay

/* init. code for Tovis panel */
static u8 set_ext_tovis[] = {0xB9, 0xFF, 0x83, 0x89};
static u8 set_mipi_tovis[] = {0xBA, 0x41, 0x93, 0x00, 0x16, 0xA4, 0x00, 0x18};
static u8 set_pwr_tovis[] = {0xB1, 0x00, 0x00, 0x07, 0xE4, 0x50, 0x10, 0x11, 0xB1, 0xF1, \
						0x25, 0x2D, 0x1F, 0x1F, 0x42, 0x01, 0x5A, 0xF7, 0x00, 0xE6};
static u8 set_display_tovis[] = {0xB2, 0x00, 0x00, 0x78, 0x0E, 0x0C, 0x3F, 0x80};
static u8 set_cyc_tovis[] = {0xB4, 0x80, 0x0C, 0x00, 0x32, 0x10, 0x06, 0x54, 0x13, 0xD4, \
						0x32, 0x10, 0x00, 0x37, 0x35, 0x36, 0x08, 0x37, 0x01, 0x40, \
						0x02, 0x58, 0x58, 0x02, 0x00, 0x40, 0x00, 0x40, 0x14, 0x46, \
						0x50, 0x0A};
static u8 set_gip_tovis[] = {0xD5, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x3C, \
						0x08, 0x88, 0x88, 0x01, 0x01, 0x23, 0x45, 0x67, 0xAA, 0xBB, \
						0x45, 0x88, 0x88, 0x67, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, \
						0x88, 0x88, 0x54, 0x32, 0x10, 0x76, 0x54, 0xAA, 0xBB, 0x10, \
						0x88, 0x88, 0x76, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x3C, \
						0x03};
static u8 set_gamma_tovis[] = {0xE0, 0x00, 0x0B, 0x13, 0x34, 0x39, 0x3F, 0x21, 0x48, 0x07, \
						0x0C, 0x0D, 0x12, 0x14, 0x13, 0x16, 0x1F, 0x1F, 0x00, 0x0B, \
						0x13, 0x34, 0x39, 0x3F, 0x21, 0x48, 0x07, 0x0C, 0x0D, 0x12, \
						0x14, 0x13, 0x16, 0x1F, 0x1F};
static u8 set_dgc_tovis[] = {0xC1, 0x01, 0x00, 0x12, 0x21, 0x2C, 0x36, 0x3E, 0x47, 0x4F, \
						0x57, 0x5F, 0x65, 0x6D, 0x74, 0x7A, 0x81, 0x87, 0x8D, 0x94, \
						0x9A, 0xA0, 0xA6, 0xAC, 0xB2, 0xB9, 0xBF, 0xC6, 0xCB, 0xCF, \
						0xD6, 0xDF, 0xE5, 0xF1, 0xF9, 0x01, 0x79, 0xED, 0xB7, 0xC3, \
						0x99, 0xF3, 0xEE, 0x00, 0x00, 0x0C, 0x1C, 0x28, 0x31, 0x39, \
						0x42, 0x4A, 0x53, 0x5B, 0x61, 0x68, 0x6F, 0x76, 0x7D, 0x83, \
						0x89, 0x8F, 0x96, 0x9C, 0xA2, 0xA9, 0xB0, 0xB7, 0xBE, 0xC7, \
						0xCB, 0xD1, 0xD8, 0xE1, 0xE6, 0xF3, 0xFA, 0x24, 0x40, 0x4D, \
						0xA6, 0xD3, 0xFF, 0xEF, 0xAD, 0x00, 0x03, 0x1C, 0x25, 0x2E, \
						0x39, 0x43, 0x4B, 0x53, 0x5B, 0x64, 0x6B, 0x71, 0x79, 0x7F, \
						0x86, 0x8B, 0x91, 0x98, 0x9F, 0xA6, 0xAD, 0xB3, 0xBB, 0xC4, \
						0xC9, 0xCD, 0xD3, 0xDC, 0xE3, 0xEC, 0xF5, 0xFB, 0xFF, 0x2F, \
						0x9B, 0x41, 0x87, 0xD5, 0xBD, 0xEA, 0x82, 0xC0};
static u8 set_panel_tovis[] = {0xCC, 0x02};
static u8 set_vcom_tovis[] = {0xB6, 0x00, 0xA1, 0x00, 0xA1};
static u8 set_gain_tovis[] = {0xE5, 0x00, 0x0C, 0x15, 0x07, 0x00, 0x00, 0x80, 0x20, 0x80, \
						0x10, 0x00, 0x00, 0x08, 0x06, 0x04, 0x00, 0x80, 0x0E};
static u8 set_hue_gain_tovis[] = {0xE2, 0x40, 0x00, 0x00, 0x05, 0x05, 0x0C, 0x0C, 0x0C, 0x04, \
						0x04, 0x08, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10};

/* tovis init cmd*/
static u8 set_ext[] = {0xB9, 0xFF, 0x83, 0x89};
static u8 set_hs_ldol[] = {0xBA, 0x01, 0x92, 0x00, 0x16, 0xC4, 0x00, 0x18, \
	0xFF, 0x02, 0x21, 0x03, 0x21, 0x23, 0x25, 0x20, 0x00, 0x35, 0x40};
static u8 set_pwr_opt[] = {0xDE, 0x05, 0x58};
static u8 set_pwr[] = {0xB1, 0x00, 0x00, 0x07, 0xE3, 0x91, 0x10, 0x11, 0x6F, \
	0x0C, 0x1D, 0x25, 0x1E, 0x1E, 0x41, 0x01, 0x58, 0xF7, 0x00, 0xC0};
static u8 set_cyc[] = {0xB4, 0x80, 0x14, 0x00, 0x32, 0x10, 0x07, 0x32, 0x10, \
	0x00, 0x00, 0x00, 0x00, 0x17, 0x0A, 0x40, 0x0B, 0x13, 0x00, 0x4B, \
	0x14, 0x53, 0x53, 0x0A};
static u8 set_display[] = {0xB2, 0x00, 0x00, 0x78, 0x09, 0x0A, 0x00, 0x60};
static u8 set_gip[] = {0xD5, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, \
	0x20, 0x00, 0x99, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, \
	0x23, 0x88, 0x01, 0x01, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, \
	0x99, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x10, 0x88, 0x32, \
	0x10, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88};
static u8 set_gamma[] = {0xE0, 0x05, 0x11, 0x14, 0x37, 0x3F, 0x3F, 0x20, \
	0x4F, 0x08, 0x0E, 0x0D, 0x12, 0x14, 0x12, 0x14, 0x1D, 0x1C, 0x05, \
	0x11, 0x14, 0x37, 0x3F, 0x3F, 0x20, 0x4F, 0x08, 0x0E, 0x0D, 0x12, \
	0x14, 0x12, 0x14, 0x1D, 0x1C};
static u8 set_dgc[] = {0xC1, 0x01, 0x00, 0x04, 0x07, 0x0D, 0x1A, 0x1E, 0x25, \
	0x2C, 0x35, 0x3D, 0x46, 0x4F, 0x58, 0x60, 0x6A, 0x74, 0x7D, 0x85, \
	0x8E, 0x98, 0xA0, 0xAA, 0xB2, 0xBF, 0xC8, 0xCB, 0xD4, 0xE0, 0xE5, \
	0xF1, 0xF5, 0xFB, 0xFF, 0x80, 0x2F, 0x37, 0x2F, 0x35, 0x91, 0x24, \
	0x6A, 0xC0, 0x00, 0x04, 0x07, 0x0D, 0x1A, 0x1E, 0x25, 0x2C, 0x35, \
	0x3D, 0x46, 0x4F, 0x58, 0x60, 0x6A, 0x74, 0x7D, 0x85, 0x8E, 0x98, \
	0xA0};
static u8 set_cmd1[] = {0xc1, 0xAA, 0xB2, 0xBF, 0xC8, 0xCB, 0xD4, 0xE0, 0xE5, \
	0xF1, 0xF5, 0xFB, 0xFF, 0x80, 0x2F, 0x37, 0x2F, 0x35, 0x91, 0x24, \
		0x6A, 0xC0, 0x00, 0x04, 0x07, 0x0D, 0x19, 0x1D, 0x25, 0x2C, \
		0x35, 0x3D, 0x46, 0x4F, 0x58, 0x60, 0x6A, 0x74, 0x7D, 0x85, \
		0x8E, 0x98, 0xA0, 0xAA, 0xB2, 0xBF, 0xC8, 0xCB, 0xD4, 0xE0, \
		0xE5, 0xF1, 0xF5, 0xFB, 0xFF, 0x80, 0x2F, 0x37, 0x2F, 0x35, \
		0x91, 0x24, 0x6A, 0xC0};

static u8 set_cmd2[] = {0xC1, 0x01, 0x00, 0x04, 0x07, 0x0D, 0x1A, 0x1E, 0x25, \
	0x2C, 0x35, 0x3D, 0x46, 0x4F, 0x58, 0x60, 0x6A, 0x74, 0x7D, 0x85, \
		0x8E, 0x98, 0xA0, 0xAA, 0xB2, 0xBF, 0xC8, 0xCB, 0xD4, 0xE0, \
		0xE5, 0xF1, 0xF5, 0xFB, 0xFF, 0x80, 0x2F, 0x37, 0x2F, 0x35, \
		0x91, 0x24, 0x6A, 0xC0, 0x00, 0x04, 0x07, 0x0D, 0x1A, 0x1E, \
		0x25, 0x2C, 0x35, 0x3D, 0x46, 0x4F, 0x58, 0x60, 0x6A, 0x74, \
		0x7D, 0x85, 0x8E, 0x98, 0xA0};

static u8 set_vcom[] = {0xB6, 0x00, 0x86, 0x00, 0x86};
static u8 set_clock[] = {0xCB, 0x07, 0x07};
static u8 set_pwr2[] = {0xB1, 0x00, 0x00, 0x07, 0xE3, 0x91, 0x10, 0x11, 0x6F, \
	0x0C, 0x1D, 0x25, 0x1E, 0x1E, 0x41, 0x01, 0x58, 0xF7, 0x00, 0x00};

#if defined(CONFIG_BACKLIGHT_PWM_CABC)
static u8 set_cabc[] = {0xC9, 0x0F, 0x00, 0x1E, 0x02, 0x00, 0x80, 0x00, 0x01, 0x3E};
#endif

static struct tegra_dsi_cmd dsi_l_540p_4_7_sleep_cmd[] = {
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_SET_DISPLAY_OFF, 0x0),
	DSI_DLY_MS(50),
	DSI_GPIO_SET(DSI_PANEL_DSV_GPIO, 0),
	DSI_GPIO_SET(DSI_PANEL_RST_GPIO, 0),
	DSI_DLY_MS(20),
/* in case power off, sleep in is not required as per panel spec */
#if 0
	DSI_DLY_MS(10),
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_ENTER_SLEEP_MODE, 0x0),
	DSI_DLY_MS(10), /*                         */
#endif
};
/*tovis*/
static struct tegra_dsi_cmd dsi_l_540p_4_7_init_cmd[] = {
	DSI_DLY_MS(5),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_ext),
	DSI_DLY_MS(2),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_hs_ldol),
	DSI_DLY_MS(2),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_pwr_opt),
	DSI_DLY_MS(2),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_pwr),
	DSI_DLY_MS(2),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_cyc),
	DSI_DLY_MS(2),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_display),
	DSI_DLY_MS(2),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_gip),
	DSI_DLY_MS(2),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_gamma),
	DSI_DLY_MS(2),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_dgc),
	DSI_DLY_MS(2),
	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, set_cmd1),
	DSI_DLY_MS(2),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_cmd2),
	DSI_DLY_MS(2),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_vcom),
	DSI_DLY_MS(2),
	DSI_CMD_SHORT(DSI_GENERIC_SHORT_WRITE_2_PARAMS, 0xCC, 0x0E),
	DSI_DLY_MS(2),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_clock),
	DSI_DLY_MS(2),
#if defined(CONFIG_BACKLIGHT_PWM_CABC)
	// Set CABC
	DSI_CMD_SHORT(DSI_DCS_WRITE_1_PARAM, 0x51, 0xFF), // Set DBV value
	DSI_DLY_MS(2),
	DSI_CMD_SHORT(DSI_DCS_WRITE_1_PARAM, 0x53, 0x24), // Turn on back light control
	DSI_DLY_MS(2),
	DSI_CMD_SHORT(DSI_DCS_WRITE_1_PARAM, 0x55, 0x01), // Set CABC UI mode
	DSI_DLY_MS(2),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_cabc), // Set CABC control
	DSI_DLY_MS(5),
#endif
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_EXIT_SLEEP_MODE, 0x0),
	DSI_DLY_MS(150),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_pwr2),
	DSI_DLY_MS(2),
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_SET_DISPLAY_ON, 0x0),
	DSI_DLY_MS(10),
};

#if defined(CONFIG_MACH_PDA)
/*tianma*/
static struct tegra_dsi_cmd dsi_l_540p_4_7_init_cmd_tianma[] = {
	DSI_GPIO_SET(DSI_PANEL_RST_GPIO, 1),
	DSI_DLY_MS(1),
	DSI_GPIO_SET(DSI_PANEL_RST_GPIO, 0),
	DSI_DLY_MS(100),
	DSI_GPIO_SET(DSI_PANEL_RST_GPIO, 1),
	DSI_DLY_MS(10),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_ext_tianma),
	DSI_DLY_MS(5),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_mipi_tianma),
	DSI_DLY_MS(10),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_tianma),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_pwr_opt_tianma),
	DSI_DLY_MS(10),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_pwr1_tianma),
	DSI_DLY_MS(5),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_display_tianma),
	DSI_DLY_MS(5),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_cyc_tianma),
	DSI_DLY_MS(5),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_gip_tianma),
	DSI_DLY_MS(5),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_gamma_tianma),
	DSI_DLY_MS(5),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_src_opt_tianma),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_dgc_tianma),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_chesel_tianma),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_chemode_sta_tianma),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_che_tianma),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_cs_ring_tianma),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_vcom_tianma),
	DSI_DLY_MS(5),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_gip_forward_tianma),
	DSI_DLY_MS(5),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_te_func_tianma),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_sequence1_tianma),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_sequence2_tianma),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_vgh_en_tianma),
	DSI_DLY_MS(10),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_vgl_en_tianma),
	DSI_DLY_MS(25),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_vcl_en_tianma),
	DSI_DLY_MS(5),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_vgh_vgl_pump1_tianma),
	DSI_DLY_MS(5),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_vgh_vgl_pump2_tianma),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_vsp_en_tianma),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_vsn_en1_tianma),
};

static struct tegra_dsi_cmd dsi_l_540p_4_7_init_cmd2_tianma[] = {
	DSI_DLY_MS(5),
	DSI_GPIO_SET(DSI_PANEL_DSV_GPIO, 1),
	DSI_DLY_MS(5),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_vsn_en2_tianma),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_gip_src_en_tianma),
	DSI_DLY_MS(5),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_sequence3_tianma),
	DSI_DLY_MS(5),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_pwr2_tianma),
	DSI_DLY_MS(5),
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_EXIT_SLEEP_MODE, 0x0),
	DSI_DLY_MS(5),
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_SET_DISPLAY_ON, 0x0),
	DSI_DLY_MS(20),
};

/*tovis*/
static struct tegra_dsi_cmd dsi_l_540p_4_7_init_cmd_tovis[] = {
	DSI_GPIO_SET(DSI_PANEL_RST_GPIO, 1),
	DSI_DLY_MS(1),
	DSI_GPIO_SET(DSI_PANEL_RST_GPIO, 0),
	DSI_DLY_MS(1),
	DSI_GPIO_SET(DSI_PANEL_RST_GPIO, 1),
	DSI_DLY_MS(50),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_ext_tovis),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_mipi_tovis),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_pwr_tovis),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_display_tovis),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_cyc_tovis),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_gip_tovis),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_gamma_tovis),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_dgc_tovis),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_panel_tovis),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_vcom_tovis),
	DSI_CMD_SHORT(DSI_DCS_WRITE_1_PARAM, 0xE6, 0x00),
	DSI_CMD_SHORT(DSI_DCS_WRITE_1_PARAM, 0xE3, 0x01),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_gain_tovis),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_hue_gain_tovis),
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_EXIT_SLEEP_MODE, 0x0),
};

static struct tegra_dsi_cmd dsi_l_540p_4_7_init_cmd2_tovis[] = {
	DSI_DLY_MS(40),
	DSI_GPIO_SET(DSI_PANEL_DSV_GPIO, 1),
	DSI_DLY_MS(80),
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_SET_DISPLAY_ON, 0x0),
};
#endif

static struct tegra_dsi_out dsi_l_540p_4_7_pdata = {
	.n_data_lanes = 2,

	.refresh_rate = 60,
	.video_data_type = TEGRA_DSI_VIDEO_TYPE_VIDEO_MODE,
	.video_clock_mode = TEGRA_DSI_VIDEO_CLOCK_CONTINUOUS,
	.video_burst_mode = TEGRA_DSI_VIDEO_NONE_BURST_MODE,
	.controller_vs = DSI_VS_1,
	.pixel_format = TEGRA_DSI_PIXEL_FORMAT_24BIT_P,
	.virtual_channel = TEGRA_DSI_VIRTUAL_CHANNEL_0,

	.panel_send_dc_frames = true, /* Dummy 2 frame display added for protecting to suspend/resume noise*/
#if defined(CONFIG_MACH_PDA)
	.panel_reset = PDA_DSI_PANEL_RESET,
#else
	.panel_reset = DSI_PANEL_RESET,
#endif
	.power_saving_suspend = true,

	.dsi_init_cmd = dsi_l_540p_4_7_init_cmd,
	.n_init_cmd = ARRAY_SIZE(dsi_l_540p_4_7_init_cmd),

	.dsi_suspend_cmd = dsi_l_540p_4_7_sleep_cmd,
	.n_suspend_cmd = ARRAY_SIZE(dsi_l_540p_4_7_sleep_cmd),
};

static int dsi_l_540p_4_7_disable(void)
{
	/* delay between sleep in and reset low */
	printk("##@%s\n", __func__);

#if !defined(CONFIG_MACH_PDA)
	/* disable DSV_EN */
	gpio_direction_output(GPIO_DSV_EN, 0);
#if DSI_PANEL_RESET
	msleep(30);
	gpio_set_value(dsi_l_540p_4_7_pdata.dsi_panel_rst_gpio, 0);
	///sjoo// usleep_range(3000, 5000);
	msleep(10);
#endif
#endif
	is_bl_powered = false;
	///sjoo// usleep_range(3000, 5000);

	if (vdd_lcd_s_1v8)
		regulator_disable(vdd_lcd_s_1v8);

	if (vdd_lcd_s_2v8)
		regulator_disable(vdd_lcd_s_2v8);

	usleep_range(300, 500);
	return 0;
}

static void dsi_l_540p_4_7_dc_out_init(struct tegra_dc_out *dc)
{
#if defined(CONFIG_MACH_PDA)
	if (lge_get_board_revno() == HW_REV_PDA_B) {
		dc->modes = dsi_l_540p_4_7_modes_tovis_revB;
		dc->n_modes = ARRAY_SIZE(dsi_l_540p_4_7_modes_tovis_revB);
		dc->flags = PDA_DC_CTRL_MODE;

		dsi_l_540p_4_7_pdata.dsi_init_cmd2 = NULL;
		dsi_l_540p_4_7_pdata.n_init_cmd2 = 0;
	} else {
		gpio_request(DSI_PANEL_MAKER_ID_GPIO, "lcd_maker_id");
		gpio_direction_input(DSI_PANEL_MAKER_ID_GPIO);

		if (gpio_get_value(DSI_PANEL_MAKER_ID_GPIO)) {
			printk("\n[Display] %s:TIANMA panel will be initialized.\n", __func__);
			dc->modes = dsi_l_540p_4_7_modes_tianma;
			dc->n_modes = ARRAY_SIZE(dsi_l_540p_4_7_modes_tianma);
			dc->flags = PDA_DC_CTRL_MODE;

			dsi_l_540p_4_7_pdata.dsi_init_cmd = dsi_l_540p_4_7_init_cmd_tianma;
			dsi_l_540p_4_7_pdata.n_init_cmd = ARRAY_SIZE(dsi_l_540p_4_7_init_cmd_tianma);

			dsi_l_540p_4_7_pdata.dsi_init_cmd2 = dsi_l_540p_4_7_init_cmd2_tianma;
			dsi_l_540p_4_7_pdata.n_init_cmd2 = ARRAY_SIZE(dsi_l_540p_4_7_init_cmd2_tianma);
		} else {
			printk("\n[Display] %s:TOVIS panel will be initialized.\n", __func__);
			dc->modes = dsi_l_540p_4_7_modes_tovis;
			dc->n_modes = ARRAY_SIZE(dsi_l_540p_4_7_modes_tovis);
			dc->flags = PDA_DC_CTRL_MODE;

			dsi_l_540p_4_7_pdata.dsi_init_cmd = dsi_l_540p_4_7_init_cmd_tovis;
			dsi_l_540p_4_7_pdata.n_init_cmd = ARRAY_SIZE(dsi_l_540p_4_7_init_cmd_tovis);

			dsi_l_540p_4_7_pdata.dsi_init_cmd2 = dsi_l_540p_4_7_init_cmd2_tovis;
			dsi_l_540p_4_7_pdata.n_init_cmd2 = ARRAY_SIZE(dsi_l_540p_4_7_init_cmd2_tovis);
		}
	}
#else
	dc->modes = dsi_l_540p_4_7_modes;
	dc->n_modes = ARRAY_SIZE(dsi_l_540p_4_7_modes);
	dc->flags = DC_CTRL_MODE;
#endif
	dc->dsi = &dsi_l_540p_4_7_pdata;
	dc->parent_clk = "pll_d_out0";
	dc->enable = dsi_l_540p_4_7_enable;
	dc->disable = dsi_l_540p_4_7_disable;
	dc->width = 58;
	dc->height = 103;

}
static void dsi_l_540p_4_7_fb_data_init(struct tegra_fb_data *fb)
{
#if defined(CONFIG_MACH_PDA)
/*                                                                                            
                         
                         
                          
                                                                                             */
	if(lge_get_board_revno() == HW_REV_PDA_B){
		fb->xres = dsi_l_540p_4_7_modes_tovis[0].h_active;
		fb->yres = dsi_l_540p_4_7_modes_tovis[0].v_active;
	}else{
		fb->xres = dsi_l_540p_4_7_modes_tianma[0].h_active;
		fb->yres = dsi_l_540p_4_7_modes_tianma[0].v_active;
	}
#else
	fb->xres = dsi_l_540p_4_7_modes[0].h_active;
	fb->yres = dsi_l_540p_4_7_modes[0].v_active;
#endif
}

static void dsi_l_540p_4_7_sd_settings_init
(struct tegra_dc_sd_settings *settings)
{
	struct board_info bi;
	struct board_info board_info;
	tegra_get_display_board_info(&bi);
	tegra_get_board_info(&board_info);

	g_prism_enable = &settings->enable;
	g_prism_aggr = &settings->aggressiveness;

#ifdef CONFIG_MACH_PDA
	settings->bl_device_name = "pwm-backlight";
#else
	if ((bi.board_id == BOARD_E1563) || (board_info.board_id == BOARD_E1740))
		settings->bl_device_name = "lm3528_display_bl";
	else
		settings->bl_device_name = "max8831_display_bl";
#endif
}

static void dsi_l_540p_4_7_cmu_init(struct tegra_dc_platform_data *pdata)
{
	pdata->cmu = NULL;
}

struct tegra_panel __initdata dsi_l_540p_4_7 = {
	.init_sd_settings = dsi_l_540p_4_7_sd_settings_init,
	.init_dc_out = dsi_l_540p_4_7_dc_out_init,
	.init_fb_data = dsi_l_540p_4_7_fb_data_init,
	.register_bl_dev = dsi_l_540p_4_7_register_bl_dev,
	.init_cmu_data = dsi_l_540p_4_7_cmu_init,
	.set_disp_device = dsi_l_540p_4_7_set_disp_device, //Backlight PWM Prism
};
EXPORT_SYMBOL(dsi_l_540p_4_7);
