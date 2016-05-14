/* drivers/mfd/rt8542-i2c.c
 * I2C Driver for Richtek RT8542
 * Multi function device - multi functional LED IC
 *
 * Copyright (C) 2013
 * Author: CY Huang <cy_huang@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#include <linux/mfd/rt8542.h>

static inline int rt8542_read_device(struct i2c_client *i2c,
				      int reg, int bytes, void *dest)
{
	int ret;
	if (bytes > 1)
		ret = i2c_smbus_read_i2c_block_data(i2c, reg, bytes, dest);
	else {
		ret = i2c_smbus_read_byte_data(i2c, reg);
		if (ret < 0)
			return ret;
		*(unsigned char *)dest = (unsigned char)ret;
	}
	RTINFO("rt8542_read_device reg = 0x%x, value = 0x%x\n",
            (unsigned int)reg,(unsigned int) ret);
	return ret;
}

int rt8542_reg_block_read(struct i2c_client *i2c, \
			int reg, int bytes, void *dest)
{
	return rt8542_read_device(i2c, reg, bytes, dest);
}
EXPORT_SYMBOL(rt8542_reg_block_read);

int rt8542_reg_read(struct i2c_client *i2c, int reg)
{
	struct rt8542_chip* chip = i2c_get_clientdata(i2c);
	int ret;
//	pr_debug("I2C Read (client : 0x%x) reg = 0x%x\n",
//           (unsigned int)i2c,(unsigned int)reg);
	mutex_lock(&chip->io_lock);
	ret = i2c_smbus_read_byte_data(i2c, reg);
	mutex_unlock(&chip->io_lock);

	RTINFO("RT8542 Read reg = 0x%x, value = 0x%x\n",
            (unsigned int)reg,(unsigned int) ret);
	return ret;
}
EXPORT_SYMBOL(rt8542_reg_read);

int rt8542_reg_write(struct i2c_client *i2c, int reg, unsigned char data)
{
	struct rt8542_chip* chip = i2c_get_clientdata(i2c);
	int ret;
	RTINFO("rt8542_reg_write (client : 0x%x) reg = 0x%x, data = 0x%x\n",
           (unsigned int)i2c,(unsigned int)reg,(unsigned int)data);
	mutex_lock(&chip->io_lock);
	ret = i2c_smbus_write_byte_data(i2c, reg, data);
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(rt8542_reg_write);

int rt8542_assign_bits(struct i2c_client *i2c, int reg,
		unsigned char mask, unsigned char data)
{
	struct rt8542_chip *chip = i2c_get_clientdata(i2c);
	unsigned char value;
	int ret;
	mutex_lock(&chip->io_lock);
	ret = rt8542_read_device(i2c, reg, 1, &value);
	if (ret < 0)
		goto out;
	value &= ~mask;
	value |= (data&mask);
	ret = i2c_smbus_write_byte_data(i2c,reg,value);
	RTINFO("rt8542_assign_bits reg = 0x%x, value = 0x%x\n",
            (unsigned int)reg,(unsigned int) value);
out:
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(rt8542_assign_bits);

int rt8542_set_bits(struct i2c_client *i2c, int reg,
		unsigned char mask)
{
	return rt8542_assign_bits(i2c,reg,mask,mask);
}
EXPORT_SYMBOL(rt8542_set_bits);

int rt8542_clr_bits(struct i2c_client *i2c, int reg,
		unsigned char mask)
{
	return rt8542_assign_bits(i2c,reg,mask,0);
}
EXPORT_SYMBOL(rt8542_clr_bits);

static int rt8542_chip_chinit(struct i2c_client *client, unsigned char ledch)
{
	RTINFO("ledch = %02x\n", ledch);
	rt8542_assign_bits(client, RT8542_REG_CHEN, RT8542_CHEN_MASK, ledch);
	return 0;
}

int rt8542_chip_enable(struct i2c_client *client, int enable)
{
	struct rt8542_chip *chip = i2c_get_clientdata(client);

	RTINFO("enable = %d\n", enable);
	if (enable)
	{
		if (chip->en_pin != -1)
			gpio_direction_output(chip->en_pin, 1);

		if (chip->pdata && chip->pdata->ledch_data)
			rt8542_chip_chinit(client, chip->pdata->ledch_data->CH_ENABLE.val);

		chip->chip_en = enable;
	}
	else
	{
		if (chip->en_pin != -1)
			gpio_direction_output(chip->en_pin, 0);

		chip->chip_en = enable;
	}
	return 0;
}

EXPORT_SYMBOL(rt8542_chip_enable);


#if 0
static int __devinit rt8542_chip_reset(struct i2c_client *client)
{
	RTINFO("\n");
	rt8542_set_bits(client, RT8542_REG_CHEN, RT8542_RESET_MASK);
	return 0;
}
#endif /* #if 0 */

static int __devinit rt8542_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct rt8542_platform_data *pdata = client->dev.platform_data;
	struct rt8542_chip *chip;
	int ret = 0;

	printk("##@%s\n", __func__ ) ;
	if (!pdata)
		return -EINVAL;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->i2c = client;
	chip->dev = &client->dev;
	ret = gpio_request(pdata->en_pin, "rt8542_chip_en");
	if (ret < 0)
		chip->en_pin = -1;
	else
		chip->en_pin = pdata->en_pin;
	chip->pdata = pdata;

	i2c_set_clientdata(client, chip);
	mutex_init(&chip->io_lock);

	rt8542_chip_enable(client, 1);
	#if 0
	rt8542_chip_reset(client);
	#endif /* #if 0 */
	if (pdata && pdata->ledch_data)
		rt8542_chip_chinit(client, pdata->ledch_data->CH_ENABLE.val);

	ret = rt8542_core_init(chip, pdata);
	if (ret < 0)
		dev_err(chip->dev, "rt8542_core_init_fail\n");
	else
		pr_info("RT8542 Initialize successfully\n");

	return ret;

}

static int __devexit rt8542_i2c_remove(struct i2c_client *client)
{
	struct rt8542_chip *chip = i2c_get_clientdata(client);
	rt8542_core_deinit(chip);
	if (chip->en_pin != -1)
		gpio_free(chip->en_pin);
	kfree(chip);
	return 0;
}

static void rt8542_i2c_shutdown(struct i2c_client *client)
{
	struct rt8542_chip *chip = i2c_get_clientdata(client);
	if (chip->chip_en)
		rt8542_chip_enable(client, 0);
}

static int rt8542_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct rt8542_chip *chip = i2c_get_clientdata(client);
//	if (chip->pdata && chip->pdata->ledch_data)
//		rt8542_chip_chinit(client, chip->pdata->ledch_data->CH_ENABLE.val);
//	if (chip->chip_en && !(chip->bled_en && chip->fled_en))
//		rt8542_chip_enable(client, 0);
	chip->suspend = 1;
	return 0;
}

static int rt8542_i2c_resume(struct i2c_client *client)
{
	struct rt8542_chip *chip = i2c_get_clientdata(client);
//	if (!chip->chip_en)
//		rt8542_chip_enable(client, 1);
	chip->suspend = 0;
	return 0;
}

static const struct i2c_device_id rt8542_id_table[] = {
	{ RT8542_DEVICE_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, rt8542_id_table);

static struct i2c_driver rt8542_driver = {
	.driver	= {
		.name	= RT8542_DEVICE_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= rt8542_i2c_probe,
	.remove		= __devexit_p(rt8542_i2c_remove),
	.shutdown	= rt8542_i2c_shutdown,
	.suspend	= rt8542_i2c_suspend,
	.resume		= rt8542_i2c_resume,
	.id_table	= rt8542_id_table,
};

static int __init rt8542_i2c_init(void)
{
	int ret;

	ret = i2c_add_driver(&rt8542_driver);
	if (ret != 0)
		pr_err("Failed to register RT8542 I2C driver: %d\n", ret);
	return ret;
}
module_init(rt8542_i2c_init);

static void __exit rt8542_i2c_exit(void)
{
	i2c_del_driver(&rt8542_driver);
}
module_exit(rt8542_i2c_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("I2C Driver for Richtek RT8542");
MODULE_AUTHOR("CY Huang <cy_huang@richtek.com>");
