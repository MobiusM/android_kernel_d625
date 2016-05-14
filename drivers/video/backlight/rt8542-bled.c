/*
 *  drivers/video/backlight/rt8542-bled.c
 *  Driver for Richtek RT8542 backlight led function
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
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/leds.h>
#include <linux/gpio.h>

#include <linux/mfd/rt8542.h>
#include <linux/rt8542-bled.h>
#include <linux/backlight.h>
#include <linux/fb.h>

//                                                                  
#define RT8542_DEBUG_LOG 1
//                                               

struct rt8542_bled_info {
	struct i2c_client *i2c;
	struct rt8542_chip *chip;
	int external_pwm;
	int brightness;
	int suspend;
};

struct rt8542_bled_info *binfo;

#if defined(CONFIG_BACKLIGHT_CABC_DEBUG_ENABLE)
static ssize_t rt8542_show_pwm(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    int data;
    data = rt8542_reg_read(binfo->i2c, RT8542_REG_FLCTL2);

    pr_info("[LCD]rt8542_show_pwm() pwm : 0x%x \n", data);

    data &= RT8542_BLED_PWM_MASK;

    if(data)
        return sprintf(buf, "%u\n", 1);
    else
        return sprintf(buf, "%u\n", 0);

}

static ssize_t rt8542_store_pwm(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
    int ret;
    unsigned int enable;

    ret = kstrtouint(buf, 10, &enable);
    if (ret)
        return -EINVAL;
    binfo->external_pwm = enable;
    if(enable)
        rt8542_set_bits(binfo->i2c, RT8542_REG_FLCTL2, RT8542_BLED_PWM_MASK);
    else
        rt8542_clr_bits(binfo->i2c, RT8542_REG_FLCTL2, RT8542_BLED_PWM_MASK);

    pr_info("[LCD]rt8542_store_pwm() pwm : %d \n", enable);

    return count;

}
#endif

static int rt8542_bled_pwm_enable(struct i2c_client *i2c, int enable)
{
    if(enable)
        rt8542_set_bits(i2c, RT8542_REG_FLCTL2, RT8542_BLED_PWM_MASK);
    else
        rt8542_clr_bits(i2c, RT8542_REG_FLCTL2, RT8542_BLED_PWM_MASK);
    return 0;
}
static int rt8542_bled_enable(struct i2c_client *i2c, int enable)
{
	if (enable)
		rt8542_set_bits(i2c, RT8542_REG_CHEN, RT8542_BLED_EN_MASK);
	else
		rt8542_clr_bits(i2c, RT8542_REG_CHEN, RT8542_BLED_EN_MASK);
	return 0;
}

#if 0
static int rt8542_get_bl_brightness(struct i2c_client *i2c)
{
	//u8 data = rt8542_reg_read(i2c, RT8542_REG_BLEXPCTL); //exponential mode
	u8 data = rt8542_reg_read(i2c, RT8542_REG_BLLINCTL); //linear mode
	return data & RT8542_BL_LEVEL_MASK;
}
#endif /* #if 0 */

int rt8542_internal_set_bl_enable(int enable)
{
	if (binfo->brightness)
	{
		if (!binfo->chip->bled_en)
		{
			rt8542_bled_enable(binfo->i2c, 1);
			binfo->chip->bled_en = 1;
		}
	}
	else
	{
		if (binfo->chip->bled_en)
		{
			rt8542_bled_enable(binfo->i2c, 0);
			binfo->chip->bled_en = 0;
		}
	}
	return 0;
}
EXPORT_SYMBOL(rt8542_internal_set_bl_enable);

//                                                                  
#if RT8542_DEBUG_LOG
#define RT8542_REG_LEN 12
void rt8542_read_all_reg(void)
{
	int reg, val;
	printk("===============================================\n");
	for (reg = 0; reg < RT8542_REG_LEN; reg ++) {
		val = rt8542_reg_read(binfo->i2c, reg);
		printk("[backlight] reg[0x%x] : [0x%x]\n", reg, val);
	}
	printk("===============================================\n");
}
#endif
//                                               

extern bool is_flash_on;
int rt8542_bl_enable(bool enable)
{
	if (!binfo)
		return 0;

	printk("[backlight] %s[%d][%d][%d][%d]\n",
			__func__, enable, binfo->chip->bled_en, binfo->chip->chip_en, is_flash_on);
	if (enable) {
		if (binfo->chip->bled_en == 0) {
			if (binfo->chip->chip_en == 0) {
				rt8542_chip_enable(binfo->i2c, 1);

				rt8542_bled_enable(binfo->i2c, 0);

				/* max current :8.4mA, OVP : 40V */
				rt8542_reg_write(binfo->i2c, RT8542_REG_BLCTL1, binfo->chip->pdata->bled_data->BLED_CONTROL1.val);
				rt8542_reg_write(binfo->i2c, RT8542_REG_BLCTL2, binfo->chip->pdata->bled_data->BLED_CONTROL2.val);
				/* brightness 0x79 for lmiting 16mA max current*/
				rt8542_internal_set_bl_brightness(binfo->chip->pdata->bled_data->BLED_LIN_BRIGHT.val);

				rt8542_bled_enable(binfo->i2c, 1);

				if (binfo->external_pwm) {
					rt8542_set_bits(binfo->i2c, RT8542_REG_FLCTL2, RT8542_BLED_PWM_MASK);
				}
			} else {
				/* max current :8.4mA, OVP : 40V */
				rt8542_reg_write(binfo->i2c, RT8542_REG_BLCTL1, binfo->chip->pdata->bled_data->BLED_CONTROL1.val);
				/* brightness 0x79 for lmiting 16mA max current*/
				rt8542_internal_set_bl_brightness(binfo->chip->pdata->bled_data->BLED_LIN_BRIGHT.val);

				rt8542_bled_enable(binfo->i2c, 1);

				if (binfo->external_pwm) {
					rt8542_set_bits(binfo->i2c, RT8542_REG_FLCTL2, RT8542_BLED_PWM_MASK);
				}
			}
			binfo->chip->bled_en = 1;
		}
//                                                                  
#if RT8542_DEBUG_LOG
		rt8542_read_all_reg();
#endif
//                                               
	} else {
		if (binfo->chip->bled_en == 1) {
			rt8542_bled_enable(binfo->i2c, 0);
			binfo->chip->bled_en = 0;
			rt8542_clr_bits(binfo->i2c, RT8542_REG_FLCTL2, RT8542_BLED_PWM_MASK);

			if (!is_flash_on) {
				rt8542_chip_enable(binfo->i2c, 0);
			}
		}
	}
}
EXPORT_SYMBOL(rt8542_bl_enable);

int rt8542_internal_set_bl_brightness(int brightness)
{
	u8 data = brightness & RT8542_BL_LEVEL_MASK;
	binfo->brightness = brightness;
	//                                                                                             
	{
		//rt8542_reg_write(binfo->i2c, RT8542_REG_BLEXPCTL, data); //exponential mode
		rt8542_reg_write(binfo->i2c, RT8542_REG_BLLINCTL, data); //linear mode
	}
	return 0;
}
EXPORT_SYMBOL(rt8542_internal_set_bl_brightness);

static void rt8542_set_bl_brightness(struct led_classdev *led_cdev, enum led_brightness brightness)
{
	struct rt8542_bled_info *bi = dev_get_drvdata(led_cdev->dev->parent);
#if defined(CONFIG_MACH_PDA)
	// calculate data(range 0x00 to 0x7F) based on brightness(range 0 to 255)
	int data = RT8542_MAX_BRIGHTNESS * (brightness *100/ANDROID_MAX_BRIGHTNESS);
	data /= 100;
	RTINFO("rt8542_set_bl_brightness() brightness:%d, data%d\n", brightness, data);
#else
	u8 data = brightness & RT8542_BL_LEVEL_MASK;
#endif
	bi->brightness = brightness;
	//                                                                                             
	{
		//rt8542_reg_write(bi->i2c, RT8542_REG_BLEXPCTL, data); //exponential mode
		rt8542_reg_write(bi->i2c, RT8542_REG_BLLINCTL, data); //linear mode
	}

	if (data)
	{
		if (!bi->chip->bled_en)
		{
			rt8542_bled_enable(bi->i2c, 1);
			bi->chip->bled_en = 1;
			if(bi->external_pwm)
				rt8542_bled_pwm_enable(bi->i2c,1);

			rt8542_reg_write(binfo->i2c, RT8542_REG_BLCTL1,binfo->chip->pdata->bled_data->BLED_CONTROL1.val);
		}
	}
	else
	{
		if (bi->chip->bled_en)
		{
			rt8542_bled_enable(bi->i2c, 0);
			bi->chip->bled_en = 0;
		}
	}
}

static struct led_classdev rt8542_bled_classdev = {
#if defined(CONFIG_MACH_PDA)
	.name 		= "lcd-backlight",
#else
	.name 		= RT8542_DEVICE_NAME "-bled",
#endif
	.brightness	= 0,
#if defined(CONFIG_MACH_PDA)
	.max_brightness = ANDROID_MAX_BRIGHTNESS,
#else
	.max_brightness = RT8542_MAX_BRIGHTNESS,
#endif
	.brightness_set = rt8542_set_bl_brightness,
};

#if defined(CONFIG_BACKLIGHT_CABC_DEBUG_ENABLE)
DEVICE_ATTR(bl_pwm, 0644, rt8542_show_pwm, rt8542_store_pwm);
#endif
static int rt8542_bled_reg_init(struct i2c_client *i2c, struct rt8542_bled_data *bdata)
{
	RTINFO("\n");
	// for debug bled platform data
	RTINFO("bled control1->%02x\n", bdata->BLED_CONTROL1.val);
	RTINFO("bled control2->%02x\n", bdata->BLED_CONTROL2.val);
	RTINFO("exp control->%02x\n", bdata->BLED_EXP_BRIGHT.val);
	RTINFO("lin control->%02x\n", bdata->BLED_LIN_BRIGHT.val);
	RTINFO("flash control2->%02x\n", bdata->FLASH_CONTROL2.val);
	rt8542_reg_write(i2c, RT8542_REG_BLCTL1, bdata->BLED_CONTROL1.val);
	rt8542_reg_write(i2c, RT8542_REG_BLCTL2, bdata->BLED_CONTROL2.val);
	rt8542_reg_write(i2c, RT8542_REG_BLEXPCTL, bdata->BLED_EXP_BRIGHT.val);
	//rt8542_reg_write(i2c, RT8542_REG_BLLINCTL, bdata->BLED_LIN_BRIGHT.val);
	rt8542_assign_bits(i2c, RT8542_REG_FLCTL2, RT8542_BLED_PWM_MASK, bdata->FLASH_CONTROL2.val);
	return 0;
}

static int __devinit rt8542_bled_probe(struct platform_device *pdev)
{
	struct rt8542_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct rt8542_bled_info *bi;
	int err;

	bi = kzalloc(sizeof(*bi), GFP_KERNEL);
	if (!bi)
		return -ENOMEM;

	bi->i2c = chip->i2c;
	bi->chip = chip;
	bi->external_pwm = chip->pdata->bled_data->FLASH_CONTROL2.bitfield.PWM_ENABLE;
	binfo = bi;

	platform_set_drvdata(pdev, bi);

	rt8542_bled_reg_init(bi->i2c, chip->pdata->bled_data);

	if (led_classdev_register(&pdev->dev, &rt8542_bled_classdev))
	{
		pr_err("%s: led class register fail\n", __func__);
		goto out;
	}
#if defined(CONFIG_BACKLIGHT_CABC_DEBUG_ENABLE)
	err = device_create_file(&pdev->dev,
			&dev_attr_bl_pwm);
#endif

	/* TODO: check initial code after bootloader BL implemented */
	rt8542_internal_set_bl_brightness(chip->pdata->bled_data->BLED_LIN_BRIGHT.val);
	rt8542_internal_set_bl_enable(1);

	pr_info("rt8542-bled driver is successfully loaded\n");
	return 0;
out:
	kfree(bi);
	return -EINVAL;
}

static int __devexit rt8542_bled_remove(struct platform_device *pdev)
{
	struct rt8542_bled_info *bi = platform_get_drvdata(pdev);

	led_classdev_unregister(&rt8542_bled_classdev);
	kfree(bi);
	return 0;
}

static int rt8542_bled_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct rt8542_bled_info *bi = platform_get_drvdata(pdev);
	RTINFO("\n");
	bi->suspend = 1;
	return 0;
}

static int rt8542_bled_resume(struct platform_device *pdev)
{
	struct rt8542_bled_info *bi = platform_get_drvdata(pdev);
	RTINFO("\n");
	bi->suspend = 0;
	return 0;
}

static struct platform_driver rt8542_bled_driver = 
{
	.driver = {
		.name = RT8542_DEVICE_NAME "-bled",
		.owner = THIS_MODULE,
	},
	.probe = rt8542_bled_probe,
	.remove = __devexit_p(rt8542_bled_remove),
	.suspend = rt8542_bled_suspend,
	.resume = rt8542_bled_resume,
};

static int __init rt8542_bled_init(void)
{
	return platform_driver_register(&rt8542_bled_driver);
}
module_init(rt8542_bled_init);

static void __exit rt8542_bled_exit(void)
{
	platform_driver_unregister(&rt8542_bled_driver);
}
module_exit(rt8542_bled_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("CY Huang <cy_huang@richtek.com");
MODULE_DESCRIPTION("BLED driver for RT8542");
MODULE_ALIAS("platform:" RT8542_DEVICE_NAME "-bled");
