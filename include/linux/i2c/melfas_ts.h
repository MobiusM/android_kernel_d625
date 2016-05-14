/*
 * include/linux/melfas_ts.h - platform data structure for MCS Series sensor
 *
 * Copyright (C) 2010 Melfas, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LINUX_MELFAS_TS_H
#define _LINUX_MELFAS_TS_H
/*           
                                                
                                   */
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#define MELFAS_DRIVER_NAME "melfas-ts"

#define MAX_NUM_OF_BUTTON	4

#define TOUCH_VDD_VTG_MIN_UV	2950000
#define TOUCH_VDD_VTG_MAX_UV	2950000
#define TOUCH_LPM_LOAD_UA	10
#define TOUCH_ACTIVE_LOAD_UA	10000

#define TOUCH_I2C_VTG_MIN_UV	1800000
#define TOUCH_I2C_VTG_MAX_UV	1800000
#define TOUCH_I2C_LPM_LOAD_UA	10
#define TOUCH_I2C_LOAD_UA	10000

#define TS_I2C_SCL		93
#define TS_I2C_SDA		94
#define TS_X_MAX		540
#define TS_Y_MAX		960
#define TS_GPIO_IRQ		105
#define TS_MAKER_ID		38 //TEGRA_GPIO_PE6

#define MELFAS_TS_I2C_ADDR	0x48

struct melfas_tsi_platform_data {
	uint32_t version;
	int x_max;
	int y_max;
	int max_pressure;
	int max_width;

	int gpio_scl;
	int gpio_sda;
	bool i2c_pull_up;

	int i2c_int_gpio;
	int touch_id_gpio;
#if defined(CONFIG_MACH_PDA)
	int touch_en_gpio;
#endif
	int gpio_ldo;
	u32 irq_flag;

	unsigned char num_of_finger;
	unsigned char num_of_button;
	unsigned short button[MAX_NUM_OF_BUTTON];
};

#endif /* _LINUX_MELFAS_TS_H */
