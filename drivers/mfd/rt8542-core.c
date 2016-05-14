/*
 *  drivers/mfd/rt8542-core.c
 *  Driver for Richtek RT8542 Core PMIC
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
#include <linux/version.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>

#include <linux/mfd/rt8542.h>

#ifdef CONFIG_RT8542_BLED
static struct mfd_cell bled_devs[] = {
{
	.name = RT8542_DEVICE_NAME "-bled",
	.id = -1,
	.num_resources = 0,
},
};
#endif /* CONFIG_RT8542_BLED */

#ifdef CONFIG_RT8542_FLED
static struct mfd_cell fled_devs[] = {
{
	.name = RT8542_DEVICE_NAME "-fled",
	.id = -1,
	.num_resources = 0,
},
};
#endif /* CONFIG_RT8542_FLED */

#ifdef CONFIG_RT8542_DEBUGFS
static struct mfd_cell debug_devs[] = {
{
	.name = RT8542_DEVICE_NAME "-debug",
	.id = -1,
	.num_resources = 0,
},
};
#endif /* CONFIG_RT8542_DEBUGFS */

int __devinit rt8542_core_init(struct rt8542_chip *chip, struct rt8542_platform_data *pdata)
{
	int ret = 0;

	RTINFO("Start to initialize all device\n");

	#ifdef CONFIG_RT8542_BLED
	if (pdata && pdata->bled_data) {
	    RTINFO("mfd add bled dev\n");
		#if (LINUX_VERSION_CODE>=KERNEL_VERSION(3,6,0))
		ret = mfd_add_devices(chip->dev, 0, &bled_devs[0],
					ARRAY_SIZE(bled_devs),
					NULL, 0,NULL);
		#else
		ret = mfd_add_devices(chip->dev, 0, &bled_devs[0],
					ARRAY_SIZE(bled_devs),
					NULL, 0);
		#endif
		if (ret < 0) {
			dev_err(chip->dev, "Failed to add bled subdev\n");
			goto out_dev;
		}
	}
	#endif /* CONFIG_RT8542_BLED */

	#ifdef CONFIG_RT8542_FLED
	if (pdata && pdata->fled_data) {
	    RTINFO("mfd add fled dev\n");
		#if (LINUX_VERSION_CODE>=KERNEL_VERSION(3,6,0))
		ret = mfd_add_devices(chip->dev, 0, &fled_devs[0],
					ARRAY_SIZE(fled_devs),
					NULL, 0,NULL);
		#else
		ret = mfd_add_devices(chip->dev, 0, &fled_devs[0],
					ARRAY_SIZE(fled_devs),
					NULL, 0);
		#endif
		if (ret < 0) {
			dev_err(chip->dev, "Failed to add fled subdev\n");
			goto out_dev;
		}
	}
	#endif /* CONFIG_RT8542_FLED */

	#ifdef CONFIG_RT8542_DEBUGFS
	RTINFO("mfd add debug dev\n");
	#if (LINUX_VERSION_CODE>=KERNEL_VERSION(3,6,0))
	ret = mfd_add_devices(chip->dev, 0, &debug_devs[0],
			      ARRAY_SIZE(debug_devs),
			      NULL, 0, NULL);
	#else
	ret = mfd_add_devices(chip->dev, 0, &debug_devs[0],
			      ARRAY_SIZE(debug_devs),
			      NULL, 0);
	#endif /* LINUX_VERSION_CODE>=KERNL_VERSION(3,6,0) */
	if (ret < 0) {
		dev_err(chip->dev, "Failed to add debug subdev\n");
		goto out_dev;
	}
	#endif /* CONFIG_RT8542_DEBUGFS */

	RTINFO("Initialize all device successfully\n");
	return ret;
out_dev:
	mfd_remove_devices(chip->dev);
	return ret;
}
EXPORT_SYMBOL(rt8542_core_init);

int __devexit rt8542_core_deinit(struct rt8542_chip *chip)
{
	mfd_remove_devices(chip->dev);
	return 0;
}
EXPORT_SYMBOL(rt8542_core_deinit);
