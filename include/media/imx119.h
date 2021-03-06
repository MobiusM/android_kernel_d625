/**
 * Copyright (c) 2012-2013 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef __IMX119_H__
#define __IMX119_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define IMX119_IOCTL_SET_MODE		_IOW('o', 1, struct imx119_mode)
#define IMX119_IOCTL_GET_STATUS		_IOR('o', 2, __u8)
#define IMX119_IOCTL_SET_FRAME_LENGTH	_IOW('o', 3, __u32)
#define IMX119_IOCTL_SET_COARSE_TIME	_IOW('o', 4, __u32)
#define IMX119_IOCTL_SET_GAIN		_IOW('o', 5, __u16)
#define IMX119_IOCTL_GET_FUSEID		_IOR('o', 6, struct nvc_fuseid)
#define IMX119_IOCTL_SET_GROUP_HOLD	_IOW('o', 7, struct imx119_ae)

/* IMX119 registers */
#define IMX119_GROUP_PARAM_HOLD			(0x0104)
#define IMX119_COARSE_INTEGRATION_TIME_15_8	(0x0202)
#define IMX119_COARSE_INTEGRATION_TIME_7_0	(0x0203)
#define IMX119_ANA_GAIN_GLOBAL			(0x0205)
#define IMX119_FRAME_LEN_LINES_15_8		(0x0340)
#define IMX119_FRAME_LEN_LINES_7_0		(0x0341)
#define IMX119_FUSE_ID_REG		(0x3500)

#define NUM_OF_FRAME_LEN_REG		2
#define NUM_OF_COARSE_TIME_REG		2
#define NUM_OF_GAIN_REG			9
struct imx119_mode {
	int xres;
	int yres;
	__u32 frame_length;
	__u32 coarse_time;
	__u16 gain;
	__u16 dgain;
};

struct imx119_ae {
	__u32 frame_length;
	__u8  frame_length_enable;
	__u32 coarse_time;
	__u8  coarse_time_enable;
	__s32 gain;
	__u32 dgain;
	__u8  gain_enable;
};

#ifdef __KERNEL__
struct imx119_power_rail {
	struct regulator *dvdd;
	struct regulator *avdd;
	struct regulator *iovdd;
};

struct imx119_platform_data {
	unsigned int cam2_gpio;
	bool ext_reg;
	const char *mclk_name; /* NULL for default */
	struct edp_client edpc_config;
	int (*power_on)(struct imx119_power_rail *pw);
	int (*power_off)(struct imx119_power_rail *pw);
};
#endif /* __KERNEL__ */

#endif  /* __IMX119_H__ */
