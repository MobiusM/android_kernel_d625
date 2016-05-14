/*
 * rt8542-fled.c - rt8542 flash/torch kernel driver
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.

 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */
/*
 *  drivers/video/backlight/rt8542-fled.c
 *  Driver for Richtek RT8542 flash led function
 *
 *  Copyright (C) 2013 Richtek Electronics
 *  cy_huang <cy_huang@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/mfd/rt8542.h>
#include <linux/rt8542-fled.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <media/nvc.h>
#include <media/nvc_torch.h>
#include <linux/gpio.h>

char *dname = "torch";

#define RT8542_FLASH_LEVELS 2
#define RT8542_FLASH_TIMERS 2
#define RT8542_TORCH_LEVELS 2
#define RT8542_TORCH_TIMERS 2

#define RT8542_FLASH_CAP_SIZE \
	(sizeof(struct nvc_torch_flash_capabilities_v1) \
	+ (sizeof(struct nvc_torch_lumi_level_v1) \
	* RT8542_FLASH_LEVELS))

#define RT8542_FLASH_TIMEOUT_SIZE \
	(sizeof(struct nvc_torch_timer_capabilities_v1) \
	+ (sizeof(struct nvc_torch_timeout_v1) \
	* RT8542_FLASH_TIMERS))

#define FLASH_EXT_CAPS_SIZE \
	(RT8542_FLASH_CAP_SIZE + RT8542_FLASH_TIMEOUT_SIZE)

#define RT8542_TORCH_CAP_SIZE \
	(sizeof(struct nvc_torch_torch_capabilities_v1) \
	+ (sizeof(struct nvc_torch_lumi_level_v1) \
	* RT8542_TORCH_LEVELS))

#define RT8542_TORCH_TIMEOUT_SIZE \
	(sizeof(struct nvc_torch_timer_capabilities_v1) \
	+ (sizeof(struct nvc_torch_timeout_v1) \
	* RT8542_TORCH_TIMERS))

#define TORCH_EXT_CAPS_SIZE \
	(RT8542_TORCH_CAP_SIZE + RT8542_TORCH_TIMEOUT_SIZE)

#define RT8542_F_LEVEL_OFF 0xFFFF

struct rt8542_fled_info {
	struct i2c_client *i2c;
	struct rt8542_chip *chip;
	int external_strobe_pin;
	int strobe_gpio;
	int suspend;
	int brightness;
	struct miscdevice miscdev;
	atomic_t in_use;
	struct nvc_torch_capability_query query;
	struct nvc_torch_flash_capabilities_v1 *flash_cap;
	struct nvc_torch_timer_capabilities_v1 *flash_timeouts;
	struct nvc_torch_torch_capabilities_v1 *torch_cap;
	struct nvc_torch_timer_capabilities_v1 *torch_timeouts;

	u8 flash_level;
	u8 torch_level;
	int power;
	struct nvc_torch_pin_state pinstate;
};

struct rt8542_fled_info *finfo;

static void rt8542_set_fl_brightness2(int brightness);
static inline int rt8542_fled_mode(struct i2c_client *i2c, \
			int external_strobe_pin, enum fled_mode fmode);
static inline int rt8542_fled_enable(struct i2c_client *i2c, int enable);

static int rt8542_param_write(struct rt8542_fled_info *fi, long arg)
{
	struct nvc_param params;
	struct nvc_torch_set_level_v1 level;
	struct nvc_torch_pin_state local_pinstate;
	int err = 0;

	if (copy_from_user(
		&params, (const void __user *)arg, sizeof(struct nvc_param))) {
		printk(KERN_ERR "%s %d err\n", __func__, __LINE__);
		return -EINVAL;
	}

	switch (params.param) {
	case NVC_PARAM_FLASH_LEVEL:
		printk(KERN_DEBUG "%s:%d NVC_PARAM_FLASH_LEVEL\n", \
			__func__, __LINE__);

		if (copy_from_user(&level, (const void __user *)params.p_value,
			sizeof(level))) {
			printk(KERN_ERR "%s %d err\n",	__func__, __LINE__);
			return -EINVAL;
		}

		if (RT8542_F_LEVEL_OFF == level.levels[0])
			rt8542_set_fl_brightness2(0);
		else
			rt8542_set_fl_brightness2(255);
		break;

	case NVC_PARAM_TORCH_LEVEL:
		printk(KERN_DEBUG "%s:%d NVC_PARAM_TORCH_LEVEL\n", \
			__func__, __LINE__);
		if (copy_from_user(&level, (const void __user *)params.p_value,
			sizeof(level))) {
			printk(KERN_ERR "%s %d err\n", __func__, __LINE__);
			return -EINVAL;
		}

		if (RT8542_F_LEVEL_OFF == level.levels[0])
			rt8542_set_fl_brightness2(0);
		else
			rt8542_set_fl_brightness2(127);
		break;

	case NVC_PARAM_FLASH_PIN_STATE:
		if (copy_from_user(&local_pinstate,
			(const void __user *)params.p_value,
			sizeof(local_pinstate))) {
			printk(KERN_ERR "%s %d err\n", __func__, __LINE__);
			err = -EINVAL;
			break;
		}
		break;
	}

	return err;
}

static int rt8542_param_read(struct rt8542_fled_info *fi, long arg)
{
	struct nvc_param params;
	const void *data_ptr = NULL;
	u32 data_size = 0;
	int err = 0;

	if (copy_from_user(&params, (const void __user *)arg,
				sizeof(struct nvc_param))) {
		printk(KERN_ERR "%s %d err\n", __func__, __LINE__);
		return -EINVAL;
	}

	switch (params.param) {
	case NVC_PARAM_TORCH_QUERY:
		printk(KERN_DEBUG "%s:%d NVC_PARAM_TORCH_QUERY\n", \
			__func__, __LINE__);
		data_ptr = &fi->query;
		data_size = sizeof(fi->query);
		break;

	case NVC_PARAM_FLASH_EXT_CAPS:
		printk(KERN_DEBUG "%s;%d EXT_FLASH_CAPS %d\n",
			__func__, __LINE__,  params.variant);
		if (params.variant >= fi->query.flash_num) {
			printk(KERN_ERR "%s:%d unsupported flash index.\n",
				__func__, __LINE__);
			err = -EINVAL;
			break;
		}
		data_ptr = fi->flash_cap;
		data_size = FLASH_EXT_CAPS_SIZE;
		break;

	case NVC_PARAM_TORCH_EXT_CAPS:
		printk(KERN_DEBUG "%s:%d EXT_TORCH_CAPS %d\n", \
			__func__, __LINE__, params.variant);
		if (params.variant >= fi->query.torch_num) {
			printk(KERN_ERR "%s:%d unsupported torch index.\n",
				__func__, __LINE__);
			err = -EINVAL;
			break;
		}
		data_ptr = fi->torch_cap;
		data_size = TORCH_EXT_CAPS_SIZE;
		break;

	case NVC_PARAM_FLASH_LEVEL:
		if (params.variant >= fi->query.flash_num) {
			printk(KERN_ERR "%s:%d unsupported flash index.\n", \
				__func__, __LINE__);
			err = -EINVAL;
			break;
		}
		data_ptr = &fi->flash_level;
		data_size = sizeof(fi->flash_level);
		printk(KERN_DEBUG "%s:%d flash_level(0x%x)\n", \
			__func__, __LINE__, fi->flash_level);
		break;

	case NVC_PARAM_TORCH_LEVEL:
		if (params.variant >= fi->query.torch_num) {
			printk(KERN_ERR "%s:%d unsupported torch index.\n", \
				__func__, __LINE__);
			err = -EINVAL;
			break;
		}
		data_ptr = &fi->torch_level;
		data_size = sizeof(fi->torch_level);
		printk(KERN_DEBUG "%s:%d torch_level(0x%x)\n", \
			__func__, __LINE__, fi->torch_level);
		break;

	case NVC_PARAM_FLASH_PIN_STATE:
		printk(KERN_DEBUG "%s:%d FLASH_PIN_STATE: %x & %x\n", \
			__func__, __LINE__, fi->pinstate.mask, \
			fi->pinstate.values);
		data_ptr = &fi->pinstate;
		data_size = sizeof(fi->pinstate);
		break;

	default:
		printk(KERN_ERR "%s:%d unsupported parameter: 0x%x\n",
			__func__, __LINE__, params.param);
		err = -EINVAL;
		break;
	}

	printk(KERN_DEBUG "%s:%d data size user %d vs local %d\n", \
		__func__, __LINE__, params.sizeofvalue, data_size);
	if (!err && params.sizeofvalue < data_size) {
		printk(KERN_ERR "%s:%d data size mismatch\n", \
			__func__, __LINE__);
		err = -EINVAL;
	}

	if (!err && copy_to_user((void __user *)params.p_value,
			 data_ptr, data_size)) {
		printk(KERN_ERR "%s:%d copy_to_user err line\n", \
			__func__, __LINE__);
		err = -EFAULT;
	}

	return err;
}

static long rt8542_nvc_ioctl(
	struct file *file, unsigned int cmd, unsigned long arg)
{
	struct rt8542_fled_info *fi = file->private_data;
	int pwr = fi->power;
	int err = 0;

	switch (cmd) {
	case NVC_IOCTL_PARAM_WR:
		err = rt8542_param_write(fi, arg);
		break;

	case NVC_IOCTL_PARAM_RD:
		err = rt8542_param_read(fi, arg);
		break;

	case NVC_IOCTL_PWR_WR:
		fi->power = (int)arg;
		printk(KERN_DEBUG "%s:%d NVC_IOCTL_PWR_WR (%d)\n", \
			__func__, __LINE__, fi->power);

		switch (fi->power) {
		case NVC_PWR_ERR:
		case NVC_PWR_OFF_FORCE:
		case NVC_PWR_OFF:
		case NVC_PWR_STDBY_OFF:
		case NVC_PWR_STDBY:
			rt8542_set_fl_brightness2(0);
			printk(KERN_DEBUG "%s:%d birghtness OFF (%d)\n", \
				__func__, __LINE__, fi->power);
			break;

		case NVC_PWR_COMM:
			break;
		case NVC_PWR_ON:
			break;
		default:
			printk(KERN_ERR "%s:%d default (%d)\n", \
				__func__, __LINE__, fi->power);
			break;
		}
		break;

	case NVC_IOCTL_PWR_RD:
		printk(KERN_DEBUG "%s:%d NVC_IOCTL_PWR_RD (%d) sizeof(%d)\n", \
			__func__, __LINE__, pwr, sizeof(pwr));
		if (copy_to_user(
			(void __user *)arg, (const void *)&pwr, sizeof(pwr))) {
			printk(KERN_ERR "%s:%d copy_to_user err line\n",
					__func__, __LINE__);
			err = -EFAULT;
		}
		break;

	default:
		printk(KERN_ERR "%s:%d error\n", __func__, __LINE__);
		break;
	}

	return err;
}

static int rt8542_fled_reg_init(struct i2c_client *i2c, \
				struct rt8542_fled_data *fdata)
{
	printk(KERN_DEBUG "\n");
	/* for debug fled platform data */
	printk(KERN_DEBUG "%s: fled current control->%02x\n", \
		__func__, fdata->FLED_CURR_CONTROL.val);
	printk(KERN_DEBUG "%s: fled control1->%02x\n", __func__, fdata->FLED_CONTROL1.val);
	printk(KERN_DEBUG "%s: fled vin monitor->%02x\n", \
		__func__, fdata->FLED_VIN_MONITOR.val);
	printk(KERN_DEBUG "%s: fled control2->%02x\n", __func__, fdata->FLED_CONTROL2.val);
	rt8542_reg_write(i2c, RT8542_REG_FLCURCTL, \
			 fdata->FLED_CURR_CONTROL.val);
	rt8542_reg_write(i2c, RT8542_REG_FLCTL1, fdata->FLED_CONTROL1.val);
	rt8542_reg_write(i2c, RT8542_REG_FLVINMON, fdata->FLED_VIN_MONITOR.val);
	rt8542_assign_bits(i2c, RT8542_REG_FLCTL2, RT8542_FLED_CTL2_MASK, \
				fdata->FLED_CONTROL2.val);

	return 0;
}

static int rt8542_nvc_open(struct inode *inode, struct file *file)
{
	struct miscdevice       *miscdev = file->private_data;
	struct rt8542_fled_info *fi;

	fi = container_of(miscdev, struct rt8542_fled_info, miscdev);
	if (!fi)
		return -ENODEV;

	if (atomic_xchg(&fi->in_use, 1))
		return -EBUSY;

	file->private_data = fi;
	rt8542_fled_reg_init(fi->i2c, fi->chip->pdata->fled_data);
	printk(KERN_DEBUG "%s\n", __func__);
	return 0;
}

static int rt8542_nvc_release(struct inode *inode, struct file *file)
{
	struct rt8542_fled_info *fi = file->private_data;

	file->private_data = NULL;
	WARN_ON(!atomic_xchg(&fi->in_use, 0));
	printk(KERN_DEBUG "%s\n", __func__);
	return 0;
}


static const struct file_operations rt8542_nvc_fileops = {
	.owner = THIS_MODULE,
	.open = rt8542_nvc_open,
	.unlocked_ioctl = rt8542_nvc_ioctl,
	.release = rt8542_nvc_release,
};

static inline int rt8542_fled_enable(struct i2c_client *i2c, int enable)
{
	if (enable) {
		rt8542_set_bits(i2c, RT8542_REG_CHEN, RT8542_FLED_EN_MASK);
		if (finfo->external_strobe_pin) {
			rt8542_set_bits(i2c, RT8542_REG_FLCTL2, \
					RT8542_FLED_STROBE_MASK);
			gpio_direction_output(finfo->strobe_gpio, 1);
		}
	} else {
		if (finfo->external_strobe_pin) {
			rt8542_clr_bits(i2c, RT8542_REG_FLCTL2, \
					RT8542_FLED_STROBE_MASK);
			gpio_direction_output(finfo->strobe_gpio, 0);
		}

		rt8542_clr_bits(i2c, RT8542_REG_CHEN, RT8542_FLED_EN_MASK);
	}

	return 0;
}

static inline int rt8542_fled_mode(struct i2c_client *i2c, \
			int external_strobe_pin, enum fled_mode fmode)
{
	if (fmode == FLED_MODE_TORCH) {
		rt8542_clr_bits(i2c, RT8542_REG_CHEN, RT8542_FLED_MODE_MASK);
		rt8542_clr_bits(i2c, RT8542_REG_FLCTL2, \
				RT8542_FLED_STROBE_MASK);
	} else if (fmode == FLED_MODE_STROBE) {
		rt8542_set_bits(i2c, RT8542_REG_CHEN, RT8542_FLED_MODE_MASK);
		if (external_strobe_pin)
			rt8542_set_bits(i2c, RT8542_REG_FLCTL2, \
					RT8542_FLED_STROBE_MASK);
	}

	return 0;
}

int rt8542_internal_set_fl_enable(int enable)
{
	if (enable && !finfo->chip->fled_en) {
		rt8542_fled_enable(finfo->i2c, 1);
		finfo->chip->fled_en = 1;
	} else {
		if (finfo->chip->fled_en) {
			rt8542_fled_enable(finfo->i2c, 0);
			finfo->chip->fled_en = 0;
		}
	}

	return 0;
}
EXPORT_SYMBOL(rt8542_internal_set_fl_enable);

int rt8542_internal_set_fl_mode(enum fled_mode fmode)
{
	if (fmode == FLED_MODE_TORCH) {
		rt8542_clr_bits(finfo->i2c, RT8542_REG_CHEN, \
				RT8542_FLED_MODE_MASK);
	} else if (fmode == FLED_MODE_STROBE) {
		rt8542_set_bits(finfo->i2c, RT8542_REG_CHEN, \
				RT8542_FLED_MODE_MASK);
	}

	return 0;
}
EXPORT_SYMBOL(rt8542_internal_set_fl_mode);

static void rt8542_set_fl_brightness(struct led_classdev *led_cdev, \
					enum led_brightness brightness)
{
	struct rt8542_fled_info *fi = dev_get_drvdata(led_cdev->dev->parent);

	fi->brightness = brightness;

	if (brightness) {
		switch (brightness) {
		case 255:
			rt8542_fled_mode(fi->i2c, fi->external_strobe_pin, \
						FLED_MODE_STROBE);
			break;
		case 127:
			rt8542_fled_mode(fi->i2c, fi->external_strobe_pin, \
						FLED_MODE_TORCH);
			break;
		default:
			pr_err("rt8542_fled: invalid brightness");
			break;
		}
		if (!fi->chip->fled_en) {
			rt8542_fled_enable(fi->i2c, 1);
			fi->chip->fled_en = 1;
		}
	} else {
		if (fi->chip->fled_en) {
			rt8542_fled_enable(fi->i2c, 0);
			fi->chip->fled_en = 0;
		}
	}
}

static void rt8542_set_fl_brightness2(int brightness)
{
	struct rt8542_fled_info *fi = finfo;

	fi->brightness = brightness;

	if (brightness) {
		switch (brightness) {
		case 255:
			rt8542_fled_mode(fi->i2c, fi->external_strobe_pin, \
						FLED_MODE_STROBE);
			break;
		case 127:
			rt8542_fled_mode(fi->i2c, fi->external_strobe_pin, \
						FLED_MODE_TORCH);
			break;
		default:
			pr_err("rt8542_fled: invalid brightness\n");
			break;
		}
		if (!fi->chip->fled_en) {
			rt8542_fled_enable(fi->i2c, 1);
			fi->chip->fled_en = 1;
		}
	} else {
		if (fi->chip->fled_en) {
			rt8542_fled_enable(fi->i2c, 0);
			fi->chip->fled_en = 0;
		}
	}
}

static struct led_classdev rt8542_fled_classdev = {
	.name 		= RT8542_DEVICE_NAME "-fled",
	.brightness	= 0,
	.max_brightness = RT8542_MAX_BRIGHTNESS,
	.brightness_set = rt8542_set_fl_brightness,
};

static int __devinit rt8542_fled_probe(struct platform_device *pdev)
{
	struct rt8542_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct rt8542_fled_info *fi;

	struct nvc_torch_flash_capabilities_v1	*pfcap = NULL;
	struct nvc_torch_lumi_level_v1		*plvls = NULL;
	struct nvc_torch_timer_capabilities_v1	*ptmcap = NULL;
	struct nvc_torch_timeout_v1		*ptimeout = NULL;
	struct nvc_torch_torch_capabilities_v1	*ptcap = NULL;
	int err;
	int i;

	fi = kzalloc(sizeof(*fi), GFP_KERNEL);
	if (!fi)
		return -ENOMEM;

	printk(KERN_DEBUG "##@%s\n", __func__);
	fi->i2c = chip->i2c;
	fi->chip = chip;
	fi->external_strobe_pin = \
		chip->pdata->fled_data->FLED_CONTROL2.bitfield.STROBE_ENABLE;
	if (fi->external_strobe_pin) {
		fi->strobe_gpio = chip->pdata->fled_data->strobe_gpio;
		printk(KERN_DEBUG "%s: strobe gpio:%d\n", \
			__func__, fi->strobe_gpio);
	}

	finfo = fi; // internal_use

	platform_set_drvdata(pdev, fi);

	if (fi->strobe_gpio) {
		err = gpio_request(fi->strobe_gpio, "camera_flash");
		if (err < 0)
			printk(KERN_ERR "%s: gpio_request failed %d\n", \
				__func__, fi->strobe_gpio);
	}

	if (led_classdev_register(&pdev->dev, &rt8542_fled_classdev)) {
		pr_err("%s: led class register fail\n", __func__);
		goto out;
	}

	fi->miscdev.name = dname;
	fi->miscdev.fops = &rt8542_nvc_fileops;
	fi->miscdev.minor = MISC_DYNAMIC_MINOR;
	if (misc_register(&fi->miscdev)) {
		printk(KERN_ERR "%s unable to register misc %s\n", \
				__func__, dname);
		return -ENODEV;
	}

	fi->pinstate.values = 0;
	fi->pinstate.mask = 0;
	fi->query.flash_num = 1;
	fi->query.torch_num = 1;
	fi->query.version = NVC_TORCH_CAPABILITY_VER_1;
	fi->query.led_attr = 0;

	fi->flash_cap = kzalloc(FLASH_EXT_CAPS_SIZE, GFP_KERNEL);
	if (!fi->flash_cap) {
		printk(KERN_ERR "%s failed to allocate memory for flash_cap\n", __func__);
		return -ENOMEM;
	}
	pfcap = fi->flash_cap;
	fi->flash_timeouts = (void *)(fi->flash_cap) + RT8542_FLASH_CAP_SIZE;
	ptmcap = fi->flash_timeouts;

	pfcap->version = NVC_TORCH_CAPABILITY_VER_1;
	pfcap->led_idx = 0;
	pfcap->attribute = 0;
	pfcap->granularity = 1000;
	pfcap->timeout_num = RT8542_FLASH_TIMERS;
	/* offset between flash_cap and timeout */
	pfcap->timeout_off = (void *)ptmcap - (void *)pfcap;
	pfcap->numberoflevels = RT8542_FLASH_LEVELS;

	plvls = pfcap->levels;
	for (i = 0; i < RT8542_FLASH_LEVELS; ++i) {
		if (0 == i) {
			plvls[0].guidenum = RT8542_F_LEVEL_OFF;
			plvls[0].luminance = 0;
		} else {
			plvls[i].guidenum = i;
			plvls[i].luminance = 15625 * i;
		}
	}

	ptmcap->timeout_num = RT8542_FLASH_TIMERS;
	ptimeout = ptmcap->timeouts;

	for (i = 0; i < RT8542_FLASH_TIMERS; ++i)
		ptimeout[i].timeout = 200;

	fi->torch_cap = kzalloc(TORCH_EXT_CAPS_SIZE, GFP_KERNEL);
	if (!fi->torch_cap) {
		printk(KERN_ERR "%s failed to allocate memory for torch_cap\n", __func__);
		return -ENOMEM;
	}
	ptcap = fi->torch_cap;
	fi->torch_timeouts = (void *)(fi->torch_cap) + RT8542_TORCH_CAP_SIZE;
	ptmcap = fi->torch_timeouts;

	ptcap->version = NVC_TORCH_CAPABILITY_VER_1;
	ptcap->led_idx = 0;
	ptcap->attribute = 0;
	ptcap->granularity = 1000;
	ptcap->timeout_num = RT8542_TORCH_TIMERS;
	ptcap->timeout_off = (void *)ptmcap - (void *)ptcap;
	ptcap->numberoflevels = RT8542_TORCH_LEVELS;

	printk(KERN_DEBUG "lumi");
	plvls = ptcap->levels;
	for (i = 0; i < RT8542_TORCH_LEVELS; ++i) {
		if (0 == i) {
			plvls[0].guidenum = RT8542_F_LEVEL_OFF;
			plvls[0].luminance = 0;
		} else {
			plvls[i].guidenum = i;
			plvls[i].luminance = 15625 * i;
		}
		printk(KERN_DEBUG "i(%d) guidenum(%d) luminance(%d)", \
			i, plvls[i].guidenum, plvls[i].luminance);
	}

	printk("timer\n");
	ptmcap->timeout_num = RT8542_TORCH_TIMERS;
	ptimeout = ptmcap->timeouts;
	for (i = 0; i < RT8542_TORCH_TIMERS; ++i) {
		ptimeout[i].timeout = 200 * i;
		printk(KERN_DEBUG "i(%d)timeout(%d)", i, ptimeout[i].timeout);
	}

	pr_info("rt8542-fled driver is successfully loaded\n");
	return 0;

out:
	gpio_free(fi->strobe_gpio);
	kfree(fi);
	return -EINVAL;
}

static int __devexit rt8542_fled_remove(struct platform_device *pdev)
{
	struct rt8542_fled_info *fi = platform_get_drvdata(pdev);

	led_classdev_unregister(&rt8542_fled_classdev);
	gpio_free(fi->strobe_gpio);
	misc_deregister(&fi->miscdev);
	kfree(fi);

	return 0;
}

static int rt8542_fled_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct rt8542_fled_info *fi = platform_get_drvdata(pdev);

	printk(KERN_DEBUG "%s\n", __func__);
	fi->suspend = 1;

	return 0;
}

static int rt8542_fled_resume(struct platform_device *pdev)
{
	struct rt8542_fled_info *fi = platform_get_drvdata(pdev);

	printk(KERN_DEBUG "%s\n", __func__);
	fi->suspend = 0;

	return 0;
}

static struct platform_driver rt8542_fled_driver =
{
	.driver = {
		.name = RT8542_DEVICE_NAME "-fled",
		.owner = THIS_MODULE,
	},
	.probe = rt8542_fled_probe,
	.remove = __devexit_p(rt8542_fled_remove),
	.suspend = rt8542_fled_suspend,
	.resume = rt8542_fled_resume,
};

static int __init rt8542_fled_init(void)
{
	return platform_driver_register(&rt8542_fled_driver);
}
module_init(rt8542_fled_init);

static void __exit rt8542_fled_exit(void)
{
	platform_driver_unregister(&rt8542_fled_driver);
}
module_exit(rt8542_fled_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("CY Huang <cy_huang@richtek.com");
MODULE_DESCRIPTION("FLED driver for RT8542");
MODULE_ALIAS("platform:" RT8542_DEVICE_NAME "-fled");
