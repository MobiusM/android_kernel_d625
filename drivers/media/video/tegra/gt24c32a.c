#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <media/gt24c32a.h>
#include <media/nvc.h>
#include <media/nvc_image.h>
#include <linux/edp.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#define DEBUG_EEPROM_DATA   1
#define EEPROM_PAGE_SIZE 256
#define MAX_RETRIES 3

static struct gt24c32a_info *info = NULL;

/*
This EEPROM has 4K capacity.
The range of reg is from 0 ~ 4095.
*/
static int gt24c32a_read(struct i2c_client *client, u16 reg, u16 length, u8 *data)
{
	int ret;
	struct i2c_msg msg;

	if (!client->adapter)
		return -ENODEV;

	msg.addr = 0x53;
	msg.buf = (u8 *)&reg;
	msg.flags = 0;
	msg.len = 1;

	ret = i2c_transfer(client->adapter, &msg, 1);

	if (ret != 1) {
		return -EIO;
	}

	msg.addr = 0x53;
	msg.buf = &data[reg];
	msg.flags = I2C_M_RD;
	msg.len = length;

	i2c_transfer(client->adapter, &msg, 1);

	if (ret != 1) {
		return -EIO;
	}
	return 0;
}

static int gt24c32a_write(struct i2c_client *client, u16 reg, u16 length, u8 *data)
{
	return 0;
}

static long gt24c32a_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct gt24c32a_info *info = (struct gt24c32a_info *)file->private_data;

	switch (cmd) {
	case GT24C32A_IOCTL_GET_E2PROM_DATA:
	{
#if DEBUG_EEPROM_DATA
		printk("%s: GT24C32A_IOCTL_GET_E2PROM_DATA: i2c client(name:%s, addr:0x%08x, flag:0x%08x)\n", __func__, info->i2c_client->name, info->i2c_client->addr, info->i2c_client->flags);
#endif
		if (copy_from_user(&info->reg_info,
			(const void __user *)arg,
			sizeof(struct gt24c32a_register_info)))
		{
			pr_err("%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}

		if(info->reg_info.reg_addr < 0 || info->reg_info.reg_addr > 4095)
		{
			pr_err("[Error] Requested reg_addr is not valid(r1) %s %d %x\n", __func__, __LINE__,info->reg_info.reg_addr);
			return -EFAULT;
		}
		if((info->reg_info.reg_addr +  info->reg_info.length) < 1 || (info->reg_info.reg_addr +  info->reg_info.length - 1) > 4095)
		{
			pr_err("[Error] Requested reg_addr is not valid(r2) %s %d %x\n", __func__, __LINE__,info->reg_info.reg_addr);
			return -EFAULT;
		}

#if DEBUG_EEPROM_DATA
		printk("[0] %s: Reading data from user level successful\n", __func__);
#endif
		if(gt24c32a_read(info->i2c_client,
			info->reg_info.reg_addr,
			info->reg_info.length,
			info->reg_info.e2prom_data) == 0)
		{
#if DEBUG_EEPROM_DATA
			printk("[1] %s: Reading success: 0x%0x,\
				transfer data to user level\n", __func__, info->reg_info.e2prom_data[0]);
#endif
			if (copy_to_user((void __user *) arg,
				 			&info->reg_info,
							sizeof(struct gt24c32a_register_info)))
			{
				pr_err("%s: 0x%x\n", __func__, __LINE__);
				return -EFAULT;
			}
#if DEBUG_EEPROM_DATA
			printk("[2] %s: Transfer success!!\n", __func__);
#endif

		}
		else
		{
			pr_err("%s GT24C32A_IOCTL_GET_E2PROM_DATA Failure\n", __func__);
			return -EFAULT;
		}
		break;
	}
	case GT24C32A_IOCTL_PUT_E2PROM_DATA:
	{
#if DEBUG_EEPROM_DATA
		pr_info("%s: GT24C32A_IOCTL_PUT_E2PROM_DATA\n", __func__);
#endif
		if (copy_from_user(&info->reg_info,
			(const void __user *)arg,
			sizeof(struct gt24c32a_register_info)))
		{
			pr_err("%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		if(info->reg_info.reg_addr < 0 || info->reg_info.reg_addr > 4095)
		{
			pr_err("[Error] Requested reg_addr is not valid(w1) %s %d %x\n", __func__, __LINE__,info->reg_info.reg_addr);
			return -EFAULT;
		}
		if((info->reg_info.reg_addr +  info->reg_info.length) < 1 || (info->reg_info.reg_addr +  info->reg_info.length - 1) > 4095)
		{
			pr_err("[Error] Requested reg_addr is not valid(w2) %s %d %x\n", __func__, __LINE__,info->reg_info.reg_addr);
			return -EFAULT;
		}

#if DEBUG_EEPROM_DATA
		printk("[0] %s: (W)Writing data from user level successful\n", __func__);
#endif
		if(gt24c32a_write(info->i2c_client,
			info->reg_info.reg_addr,
			info->reg_info.length,
			info->reg_info.e2prom_data) == 0)
		{
#if DEBUG_EEPROM_DATA
			printk("[1] %s: (W)Writing is successful\n", __func__);
#endif
		}
		else
		{
			return -EFAULT;
		}
		break;
	}
	default:
		printk("%s: 0x%x DEFAULT IOCTL\n", __func__, __LINE__);
		return -EINVAL;
	}
	return 0;
}

static int eeprom_regulator_get(struct gt24c32a_info *info,
	struct regulator **vreg, char vreg_name[])
{
	struct regulator *reg = NULL;
	int err = 0;

	reg = regulator_get(&info->i2c_client->dev, vreg_name);
	if (unlikely(IS_ERR(reg))) {
		dev_err(&info->i2c_client->dev, "%s %s ERR: %d\n",
			__func__, vreg_name, (int)reg);
		err = PTR_ERR(reg);
		reg = NULL;
	} else
		dev_dbg(&info->i2c_client->dev, "%s: %s\n",
			__func__, vreg_name);

	*vreg = reg;
	return err;
}

static int eeprom_power_get(struct gt24c32a_info *info)
{
	struct eeprom_power_rail *pw = &info->power;

	eeprom_regulator_get(info, &pw->dvdd, "vdig_eeprom");
	eeprom_regulator_get(info, &pw->avdd, "vana_eeprom");
	eeprom_regulator_get(info, &pw->iovdd, "vif_eeprom");

	return 0;
}

static int eeprom_power_on(void)
{
	int err;
	struct eeprom_power_rail *pw = &info->power;

	if (unlikely(WARN_ON(!pw || !pw->avdd || !pw->iovdd || !pw->dvdd)))
		return -EFAULT;

	err = regulator_enable(pw->dvdd);
	if (unlikely(err))
		goto eeprom_dvdd_fail;

	err = regulator_enable(pw->avdd);
	if (unlikely(err))
		goto eeprom_avdd_fail;

	err = regulator_enable(pw->iovdd);
	if (unlikely(err))
		goto eeprom_iovdd_fail;

	return 0;

	eeprom_iovdd_fail:
		regulator_disable(pw->dvdd);

	eeprom_dvdd_fail:
		regulator_disable(pw->avdd);

	eeprom_avdd_fail:
		regulator_disable(pw->iovdd);
		pr_err("%s FAILED\n", __func__);
	return -EINVAL;
}

static int gt24c32a_open(struct inode *inode, struct file *file)
{
	pr_info("gt24c32a: open!\n");

	eeprom_power_on();

	file->private_data = (struct file *)info;
	return 0;
}

int gt24c32a_release(struct inode *inode, struct file *file)
{
	struct eeprom_power_rail *pw = &info->power;
	pr_info("gt24c32a: release!\n");

	regulator_disable(pw->iovdd);
	regulator_disable(pw->avdd);
	regulator_disable(pw->dvdd);

	file->private_data = NULL;

	return 0;
}

static const struct file_operations gt24c32a_fileops = {
	.owner = THIS_MODULE,
	.open = gt24c32a_open,
	.unlocked_ioctl = gt24c32a_ioctl,
	.release = gt24c32a_release,
};

static struct miscdevice gt24c32a_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gt24c32a",
	.fops = &gt24c32a_fileops,
};

static int gt24c32a_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;

	pr_info("gt24c32a: probing sensor.\n");

	info = kzalloc(sizeof(struct gt24c32a_info), GFP_KERNEL);
	if (!info) {
		pr_err("gt24c32a: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&gt24c32a_device);
	if (err) {
		pr_err("gt24c32a: Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	info->i2c_client = client;
	i2c_set_clientdata(client, info);

	eeprom_power_get(info);

	pr_info("gt24c32a: probing sensor done.\n");

	return 0;
}

static int gt24c32a_remove(struct i2c_client *client)
{
	struct gt24c32a_info *info;
	info = i2c_get_clientdata(client);
	misc_deregister(&gt24c32a_device);

	if(info)
		kfree(info);
	return 0;
}

static const struct i2c_device_id gt24c32a_id[] = {
	{ "gt24c32a", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, gt24c32a_id);

static struct i2c_driver gt24c32a_i2c_driver = {
	.driver = {
		.name = "gt24c32a",
		.owner = THIS_MODULE,
	},
	.probe = gt24c32a_probe,
	.remove = gt24c32a_remove,
	.id_table = gt24c32a_id,
};

static int __init gt24c32a_init(void)
{
	pr_info("gt24c32a sensor driver loading\n");
	return i2c_add_driver(&gt24c32a_i2c_driver);
}

static void __exit gt24c32a_exit(void)
{
	i2c_del_driver(&gt24c32a_i2c_driver);
}

module_init(gt24c32a_init);
module_exit(gt24c32a_exit);

