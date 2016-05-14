/*
 * rt8542.h - RT8542 flash/torch kernel driver
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.

 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */
/*
 *  include/linux/mfd/rt8542.h
 *  Include header file for Richtek RT8542 Core file
 *
 *  Copyright (C) 2013 Richtek Electronics
 *  cy_huang <cy_huang@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_MFD_RT8542_H
#define __LINUX_MFD_RT8542_H

#define RT8542_DEVICE_NAME	"RT8542"

#define RT8542_RESET_MASK	0x80
#define RT8542_CHEN_MASK	0x7F

#define BLCTL1_INIT_VAL	(BLED_OVP_40V << 5 | \
		BLED_MAPPING_LINEAR << 4 | BLED_MAX_12P1mA)

enum {
	RT8542_REG_DEVID,
	RT8542_REG_VENID,
	RT8542_REG_BLCTL1,
	RT8542_REG_BLCTL2,
	RT8542_REG_BLEXPCTL,
	RT8542_REG_BLLINCTL,
	RT8542_REG_FLCURCTL,
	RT8542_REG_FLCTL1,
	RT8542_REG_FLVINMON,
	RT8542_REG_FLCTL2,
	RT8542_REG_CHEN,
	RT8542_REG_ERRMESSAGE,
};


//#ifdef CONFIG_RT8542_BLED
enum {
	BLED_MAX_5P1mA,
	BLED_MAX_8P4mA,
	BLED_MAX_12P1mA,
	BLED_MAX_15P4mA,
	BLED_MAX_20mA,
	BLED_MAX_22P3mA,
	BLED_MAX_25P6mA,
	BLED_MAX_29P3mA,
};

enum {
	PWM_ACTIVE_HIGH,
	PWM_ACTIVE_LOW,
};

enum {
	BLED_MAPPING_EXPON,
	BLED_MAPPING_LINEAR,
};

enum {
	BLED_OVP_16V,
	BLED_OVP_24V,
	BLED_OVP_32V,
	BLED_OVP_40V,
};

enum {
	BLED_RAMPDOWN_32uS,
	BLED_RAMPDOWN_4mS,
	BLED_RAMPDOWN_8mS,
	BLED_RAMPDOWN_16mS,
	BLED_RAMPDOWN_32mS,
	BLED_RAMPDOWN_65mS,
	BLED_RAMPDOWN_131mS,
	BLED_RAMPDOWN_262mS,
};

enum {
	BLED_RAMPUP_32uS,
	BLED_RAMPUP_4mS,
	BLED_RAMPUP_8mS,
	BLED_RAMPUP_16mS,
	BLED_RAMPUP_32mS,
	BLED_RAMPUP_65mS,
	BLED_RAMPUP_131mS,
	BLED_RAMPUP_262mS,
};

enum {
	BLED_FREQ_500KHZ,
	BLED_FREQ_1MHZ,
	BLED_FREQ_1P33MHZ,
	BLED_FREQ_2MHZ,
};

struct rt8542_bled_data {
	union {
		struct {
			unsigned char BLED_MAX_CURRENT:3;
			unsigned char BLED_PWM_CONFIG:1;
			unsigned char BLED_MAPPING_MODE:1;
			unsigned char BLED_OVP:2;
			unsigned char Resv:1;
		}bitfield;
		unsigned char val;
	}BLED_CONTROL1;
	union {
		struct {
			unsigned char BLED_RAMPUP_RATE:3;
			unsigned char BLED_RAMPDOWN_RATE:3;
			unsigned char BLED_SW_FREQ:2;
		}bitfield;
		unsigned char val;
	}BLED_CONTROL2;
	union {
		struct {
			unsigned char BLED_EXP_BRIGHTNESS:7;
			unsigned char Resv:1;
		}bitfield;
		unsigned char val;
	}BLED_EXP_BRIGHT;
	union {
		struct {
			unsigned char BLED_LIN_BRIGHTNESS:7;
			unsigned char Resv:1;
		}bitfield;
		unsigned char val;
	}BLED_LIN_BRIGHT;
	union {
		struct {
			unsigned char Resv1:6;
			unsigned char PWM_ENABLE:1;
			unsigned char Resv2:1;
		}bitfield;
		unsigned char val;
	}FLASH_CONTROL2;
};
//#endif /* #ifdef CONFIG_RT8542_BLED */

//#ifdef CONFIG_RT8542_FLED
enum {
	FLED_STROBE_56mA,
	FLED_STROBE_103mA,
	FLED_STROBE_150mA,
	FLED_STROBE_196mA,
	FLED_STROBE_243mA,
	FLED_STROBE_290mA,
	FLED_STROBE_337mA,
	FLED_STROBE_384mA,
	FLED_STROBE_431mA,
	FLED_STROBE_478mA,
	FLED_STROBE_525mA,
	FLED_STROBE_571mA,
	FLED_STROBE_618mA,
	FLED_STROBE_665mA,
	FLED_STROBE_712mA,
	FLED_STROBE_750mA,
};

enum {
	FLED_TORCH_28mA,
	FLED_TORCH_56mA,
	FLED_TORCH_84mA,
	FLED_TORCH_112mA,
	FLED_TORCH_140mA,
	FLED_TORCH_168mA,
	FLED_TORCH_196mA,
	FLED_TORCH_225mA,
};

enum {
	FLED_CURR_1P7A,
	FLED_CURR_1P9A,
	FLED_CURR_2P5A,
	FLED_CURR_3P1A,
};

enum {
	FLED_FREQ_2MHZ,
	FLED_FREQ_4MHZ,
};

enum {
	FLED_VIN_2P5V,
	FLED_VIN_2P6V,
	FLED_VIN_2P7V,
	FLED_VIN_2P8V,
	FLED_VIN_2P9V,
	FLED_VIN_3P0V,
	FLED_VIN_3P1V,
	FLED_VIN_3P2V,
};

enum {
	VIN_MONITOR_TORCH,
	VIN_MONITOR_STANDBY,
};

enum {
	TX_ACTIVE_LOW,
	TX_ACTIVE_HIGH,
};

enum {
	STROBE_ACTIVE_LOW,
	STROBE_ACTIVE_HIGH,
};

struct rt8542_fled_data {
	union {
		struct {
			unsigned char FLED_STROBE_CURRENT:4;
			unsigned char FLED_TORCH_CURRENT:3;
			unsigned char Resv:1;
		}bitfield;
		unsigned char val;
	}FLED_CURR_CONTROL;
	union {
		struct {
			unsigned char FLED_STROBE_TIMEOUT:5;
			unsigned char FLED_CURRENT_LIMIT:2;
			unsigned char FLED_SW_FREQ:1;
		}bitfield;
		unsigned char val;
	}FLED_CONTROL1;
	union {
		struct {
			unsigned char FLED_VIN_MONITOR:3;
			unsigned char Resv:5;
		}bitfield;
		unsigned char val;
	}FLED_VIN_MONITOR;
	union {
		struct {
			unsigned char VIN_MONITOR_ENABLE:1;
			unsigned char VIN_MONITOR_MODE:1;
			unsigned char TX_ENABLE:1;
			unsigned char TX_POLARITY:1;
			unsigned char STROBE_ENABLE:1;
			unsigned char STROBE_POLARITY:1;
			unsigned char Resv:2;
		}bitfield;
		unsigned char val;
	}FLED_CONTROL2;
	int strobe_gpio;
};
//#endif /* #ifdef CONFIG_RT8542_FLED */

struct rt8542_ledch_data {
	union {
		struct {
#if defined(CONFIG_MACH_PDA)
			unsigned char BL_EN:1;
			unsigned char FL_EN:1;
			unsigned char TORCH_EN:1;
#else
			unsigned char Resv1:3;
#endif
			unsigned char BLED2_EN:1;
			unsigned char BLED1_EN:1;
			unsigned char FLED2_EN:1;
			unsigned char FLED1_EN:1;
			unsigned char Resv2:1;
		}bitfield;
		unsigned char val;
	}CH_ENABLE;
};

struct rt8542_platform_data {
	struct rt8542_bled_data *bled_data;
	struct rt8542_fled_data *fled_data;
	struct rt8542_ledch_data *ledch_data;
	int en_pin;
};

struct rt8542_chip {
	struct i2c_client *i2c;
	struct device *dev;
	struct rt8542_platform_data *pdata;
	int suspend;
	int en_pin;
	int chip_en;
	int bled_en;
	int fled_en;
	struct mutex io_lock;
};

extern int rt8542_reg_block_read(struct i2c_client *, int, int, void *);
extern int rt8542_reg_read(struct i2c_client *, int);
extern int rt8542_reg_write(struct i2c_client *, int, unsigned char);
extern int rt8542_assign_bits(struct i2c_client *, int, unsigned char, unsigned char);
extern int rt8542_set_bits(struct i2c_client *, int, unsigned char);
extern int rt8542_clr_bits(struct i2c_client *, int, unsigned char);

extern int rt8542_core_init(struct rt8542_chip *, struct rt8542_platform_data *);
extern int rt8542_core_deinit(struct rt8542_chip *);
extern int rt8542_chip_enable(struct i2c_client *client, int enable);

#ifdef CONFIG_MFD_RT_SHOW_INFO
#define RTINFO(format, args...) \
	printk(KERN_INFO "%s:%s() line-%d: " format, RT8542_DEVICE_NAME,__FUNCTION__,__LINE__, ##args)
#else
#define RTINFO(format,args...)
#endif /* CONFIG_MFD_RT_SHOW_INFO */

#endif /* __LINUX_MFD_RT8542_H */
