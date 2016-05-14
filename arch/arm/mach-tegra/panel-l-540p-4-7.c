/*
 * arch/arm/mach-tegra/panel-l-540p-4-7.c
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
#include <linux/mfd/max8831.h>
#include <linux/max8831_backlight.h>
#include <linux/leds.h>
#include <linux/ioport.h>
#include <linux/lm3528.h>
#include <video/mipi_display.h>
#include <linux/rt8542-bled.h>

#ifdef CONFIG_MFD_RT8542
#include <linux/mfd/rt8542.h>
#endif /* CONFIG_MFD_RT8542 */

#include "gpio-names.h"
#include "board-panel.h"
#include "board.h"

#define DSI_PANEL_RESET         1

#define DC_CTRL_MODE            TEGRA_DC_OUT_CONTINUOUS_MODE
#define GPIO_DSV_EN	(49) /* TEGRA_GPIO_PG1 */

static struct regulator *vdd_lcd_s_1v8;
static struct regulator *vdd_lcd_s_2v8;

static bool dsi_l_540p_4_7_reg_requested;
static bool dsi_l_540p_4_7_gpio_requested;
static bool is_bl_powered;
static struct tegra_dsi_out dsi_l_540p_4_7_pdata;

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

static p_tegra_dc_bl_output dsi_l_540p_4_7_bl_response_curve;

static int __maybe_unused dsi_l_540p_4_7_bl_notify(struct device *unused,
							int brightness)
{

	pr_info("##@%s bright(%d)\n", __func__, brightness);
#if 0
	int cur_sd_brightness = atomic_read(&sd_brightness);

	/* SD brightness is a percentage */
	brightness = (brightness * cur_sd_brightness) / 255;

	/* Apply any backlight response curve */
	if (brightness > 255)
		pr_info("Error: Brightness > 255!\n");
	else
		brightness = dsi_l_540p_4_7_bl_response_curve[brightness];
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
			.BLED_MAX_CURRENT = BLED_MAX_20mA,
			.BLED_PWM_CONFIG = PWM_ACTIVE_HIGH,
			.BLED_MAPPING_MODE = BLED_MAPPING_LINEAR,
			.BLED_OVP = BLED_OVP_40V,
		},
	},
	.BLED_CONTROL2 = {
		.bitfield = {
			.BLED_RAMPDOWN_RATE = BLED_RAMPDOWN_32uS,
			.BLED_RAMPUP_RATE = BLED_RAMPUP_32uS,
			.BLED_SW_FREQ = BLED_FREQ_500KHZ,
		}
	},
	.BLED_EXP_BRIGHT = {
		.bitfield = {
			.BLED_EXP_BRIGHTNESS = 0,
		},
	},
	.BLED_LIN_BRIGHT = {
		.bitfield = {
			.BLED_LIN_BRIGHTNESS = 0,
		},
	},
	.FLASH_CONTROL2 = {
		.bitfield = {
			.PWM_ENABLE = 0,
		},
	},
}; /* rt8542_bled_data end */

static struct rt8542_fled_data rt8542_fled_data = {
	.FLED_CURR_CONTROL = {
		.bitfield = {
			.FLED_STROBE_CURRENT = FLED_STROBE_750mA,
			.FLED_TORCH_CURRENT = FLED_TORCH_84mA,
		},
	},
	.FLED_CONTROL1 = {
		.bitfield = {
			.FLED_STROBE_TIMEOUT = 0x0f,
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
			.STROBE_ENABLE = 0,
			.STROBE_POLARITY = STROBE_ACTIVE_HIGH,
		},
	},
}; /* rt8542_fled_data end */

static struct rt8542_ledch_data rt8542_ledch_data = {
	.CH_ENABLE = {
		.bitfield = {
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
	.en_pin = -1,
};

static struct i2c_board_info __initdata rt8542_device_info[] = {
	{
		I2C_BOARD_INFO(RT8542_DEVICE_NAME, 0x39),
		.platform_data = &rt8542_platform_data,
	},
};
#endif /* CONFIG_MFD_RT8542 */

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

static unsigned int dsi_s_pluto_edp_states[] = {
	1130, 1017, 904, 791, 678, 565, 452, 339, 226, 113, 0
};
static unsigned int dsi_s_pluto_edp_brightness[] = {
	255, 230, 204, 179, 153, 128, 102, 77, 51, 10, 0
};
static unsigned int dsi_s_ceres_edp_states[] = {
	720, 644, 523, 490, 442, 427, 395, 363, 330, 299, 0
};
static unsigned int dsi_s_ceres_edp_brightness[] = {
	255, 230, 204, 170, 140, 128, 102, 77, 51, 10, 0
};
static unsigned int dsi_s_atlantis_edp_states[] = {
	720, 644, 523, 490, 442, 427, 395, 363, 330, 299, 0
};
static unsigned int dsi_s_atlantis_edp_brightness[] = {
	255, 230, 204, 170, 140, 128, 102, 77, 51, 10, 0
};

static int __init dsi_l_540p_4_7_register_bl_dev(void)
{
	struct i2c_board_info *bl_info;
	struct board_info board_info;

#if defined(CONFIG_MACH_PD_A)
	i2c_register_board_info(1, rt8542_device_info, 1);
	return 0;
#endif

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
			dsi_s_ceres_edp_states;
		dsi_l_540p_4_7_max8831_bl_data.edp_brightness =
				dsi_s_ceres_edp_brightness;
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
}

struct tegra_dc_mode dsi_l_540p_4_7_modes[] = {
	/* 540x960@60Hz */
	{
		.pclk = 36427680,
		.h_ref_to_sync = 2,
		.v_ref_to_sync = 1,
		.h_sync_width = 10,
		.v_sync_width = 4,
		.h_back_porch = 47,
		.v_back_porch = 8,
		.h_active = 540,
		.v_active = 960,
		.h_front_porch = 20,
		.v_front_porch = 12,
	},
};
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

	err = gpio_request(GPIO_DSV_EN, "DSV_en");
	if (err < 0) {
		pr_err("DSV_en gpio request failed\n");
		goto fail;
	}

	err = gpio_request(dsi_l_540p_4_7_pdata.dsi_panel_bl_en_gpio, "bl_en");
	if (err < 0) {
		pr_err("bl enable gpio request failed\n");
		goto fail;
	}

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

	/* enble DSV_EN */
	/* DNI: DSV_EN is not required for current panel
		if required, it should be set to 1
	*/
	gpio_direction_output(GPIO_DSV_EN, 0);

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
	/* BL on */
	gpio_direction_output(dsi_l_540p_4_7_pdata.dsi_panel_bl_en_gpio, 1);
	is_bl_powered = true;

	return 0;
fail:
	return err;
}

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


static struct tegra_dsi_cmd dsi_l_540p_4_7_sleep_cmd[] = {
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_SET_DISPLAY_OFF, 0x0),
	DSI_DLY_MS(50),
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_ENTER_SLEEP_MODE, 0x0),
	DSI_DLY_MS(120),
};

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
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_EXIT_SLEEP_MODE, 0x0),
	DSI_DLY_MS(150),
	DSI_CMD_LONG(MIPI_DSI_DCS_LONG_WRITE, set_pwr2),
	DSI_DLY_MS(2),
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_SET_DISPLAY_ON, 0x0),
	DSI_DLY_MS(10),
};

static struct tegra_dsi_out dsi_l_540p_4_7_pdata = {
	.n_data_lanes = 2,

	.refresh_rate = 60,
	.video_data_type = TEGRA_DSI_VIDEO_TYPE_VIDEO_MODE,
	.video_clock_mode = TEGRA_DSI_VIDEO_CLOCK_CONTINUOUS,
	.video_burst_mode = TEGRA_DSI_VIDEO_NONE_BURST_MODE,
	.controller_vs = DSI_VS_1,
	.pixel_format = TEGRA_DSI_PIXEL_FORMAT_24BIT_P,
	.virtual_channel = TEGRA_DSI_VIRTUAL_CHANNEL_0,

	.panel_send_dc_frames = true, /* added for suspend/resume */

	.panel_reset = DSI_PANEL_RESET,
	.power_saving_suspend = true,

	.dsi_init_cmd = dsi_l_540p_4_7_init_cmd,
	.n_init_cmd = ARRAY_SIZE(dsi_l_540p_4_7_init_cmd),

	.dsi_suspend_cmd = dsi_l_540p_4_7_sleep_cmd,
	.n_suspend_cmd = ARRAY_SIZE(dsi_l_540p_4_7_sleep_cmd),
};

static int dsi_l_540p_4_7_disable(void)
{
	/* delay between sleep in and reset low */
	///sjoo// msleep(100);

	/* BL off */
	gpio_direction_output(dsi_l_540p_4_7_pdata.dsi_panel_bl_en_gpio, 0);

	/* disable DSV_EN */
	gpio_direction_output(GPIO_DSV_EN, 0);

	msleep(30);
	gpio_set_value(dsi_l_540p_4_7_pdata.dsi_panel_rst_gpio, 0);
	///sjoo// usleep_range(3000, 5000);
	msleep(10);

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
	dc->dsi = &dsi_l_540p_4_7_pdata;
	dc->parent_clk = "pll_d_out0";
	dc->modes = dsi_l_540p_4_7_modes;
	dc->n_modes = ARRAY_SIZE(dsi_l_540p_4_7_modes);
	dc->enable = dsi_l_540p_4_7_enable;
	dc->disable = dsi_l_540p_4_7_disable;
	dc->width = 58;
	dc->height = 103;
	dc->flags = DC_CTRL_MODE;
}
static void dsi_l_540p_4_7_fb_data_init(struct tegra_fb_data *fb)
{
	fb->xres = dsi_l_540p_4_7_modes[0].h_active;
	fb->yres = dsi_l_540p_4_7_modes[0].v_active;
}

static void dsi_l_540p_4_7_sd_settings_init
(struct tegra_dc_sd_settings *settings)
{
	struct board_info bi;
	struct board_info board_info;
	tegra_get_display_board_info(&bi);
	tegra_get_board_info(&board_info);

	if ((bi.board_id == BOARD_E1563) || (board_info.board_id == BOARD_E1740))
		settings->bl_device_name = "lm3528_display_bl";
	else
		settings->bl_device_name = "max8831_display_bl";
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
};
EXPORT_SYMBOL(dsi_l_540p_4_7);
