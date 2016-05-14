/*
 * tps65132.c
 *
 * tps65132 chip family multi-function driver
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/err.h>

#include <linux/mfd/core.h>

/**
 * tps65132_reg_read: Read a single tps65132 register.
 *
 * @tps: Device to read from.
 * @reg: Register to read.
 * @val: Contians the value
 */

int tps65132_reg_read(struct i2c_client *i2c, int reg)
{
	int ret;
	ret = i2c_smbus_read_byte_data(i2c, reg);

	printk("tps65132_reg_read  reg = 0x%x, value = 0x%x\n",
            (unsigned int)reg,(unsigned int) ret);
	return ret;
}

EXPORT_SYMBOL_GPL(tps65132_reg_read);

/**
 * tps65132_reg_write: Write a single tps65132 register.
 *
 * @tps65132: Device to write to.
 * @reg: Register to write to.
 * @val: Value to write.
 * @level: Password protected level
 */

int tps65132_reg_write(struct i2c_client *i2c, int reg, unsigned char data)
{

	int ret;
	printk("tps65132_reg_write (client : 0x%x) reg = 0x%x, data = 0x%x\n",
           (unsigned int)i2c,(unsigned int)reg,(unsigned int)data);
	ret = i2c_smbus_write_byte_data(i2c, reg, data);
	return ret;
}
EXPORT_SYMBOL_GPL(tps65132_reg_write);

struct i2c_client *tps_i2c;

static int __devinit tps65132_probe(struct i2c_client *client,
				const struct i2c_device_id *ids)
{
	int  ret = 1;

	printk("##@%s\n", __func__);

	tps_i2c = client;

	ret = tps65132_reg_read(tps_i2c, 0x03);
	if(ret != 0x00)
		ret = tps65132_reg_write(tps_i2c, 0x03, 0x00);

//                                                                      
	ret = tps65132_reg_read(tps_i2c, 0x00);
	ret = tps65132_reg_read(tps_i2c, 0x01);

	ret = tps65132_reg_write(tps_i2c, 0x00, 0x0C);
	ret = tps65132_reg_write(tps_i2c, 0x01, 0x0C);
//                                               

	return ret;
}

static int __devexit tps65132_remove(struct i2c_client *client)
{
	int  ret = 1;

	printk("##@%s\n", __func__);

	tps_i2c = client;
	ret = tps65132_reg_write(tps_i2c, 0x03,0x03);
	return 0;
}

enum chips { tps65132 };

static const struct i2c_device_id tps65132_id_table[] = {
	{"tps65132", tps65132},
	{/* end of list */}
};
MODULE_DEVICE_TABLE(i2c, tps65132_id_table);

static struct i2c_driver tps65132_driver = {
	.driver		= {
		.name	= "tps65132",
	},
	.id_table	= tps65132_id_table,
	.probe		= tps65132_probe,
	.remove		= __devexit_p(tps65132_remove),
};

static int __init tps65132_init(void)
{
	return i2c_add_driver(&tps65132_driver);
}
subsys_initcall(tps65132_init);

static void __exit tps65132_exit(void)
{
	i2c_del_driver(&tps65132_driver);
}
module_exit(tps65132_exit);

MODULE_AUTHOR("AnilKumar Ch <anilkumar@ti.com>");
MODULE_DESCRIPTION("tps65132 chip family multi-function driver");
MODULE_LICENSE("GPL v2");
