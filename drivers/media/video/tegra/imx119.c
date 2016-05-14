/*
 * imx119.c - imx119 sensor driver
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.

 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.

 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/edp.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <media/nvc.h>
#include <media/imx119.h>
#include "nvc_utilities.h"

#ifdef CONFIG_DEBUG_FS
#include <media/nvc_debugfs.h>
#endif

#define IMX119_SIZEOF_I2C_BUF 16

struct imx119_reg {
	u16 addr;
	u16 val;
};

struct imx119_info {
	struct miscdevice		miscdev_info;
	int				mode;
	struct imx119_power_rail	power;
	struct nvc_fuseid		fuse_id;
	struct i2c_client		*i2c_client;
	struct imx119_platform_data	*pdata;
	struct edp_client		*edpc;
	unsigned			edp_state;
	atomic_t			in_use;
	struct clk			*mclk;
#ifdef CONFIG_DEBUG_FS
	struct nvc_debugfs_info debugfs_info;
#endif

};

#define IMX119_TABLE_WAIT_MS 0
#define IMX119_TABLE_END 1

#define IMX119_WAIT_MS 5
#define IMX119_FUSE_ID_SIZE 7
#define IMX119_FUSE_ID_DELAY 5

static struct regulator *imx119_ext_reg1;
static struct regulator *imx119_ext_reg2;

static struct imx119_reg mode_1280x1024[] = {
    {0x0100, 0x00},
    {IMX119_TABLE_WAIT_MS, 5},
    //PLL setting EXTCLK=12Mhz 396Mbps
    {0x0305, 0x02},
    {0x0307, 0x32},
    {0x3025, 0x0A},
    {0x302B, 0x4B},
    {0x3022, 0x02}, //rear divider 1
    //{IMX119_TABLE_WAIT_MS, 100}
    //Global(Initial) Setting
    {0x0112, 0x0A},
    {0x0113, 0x0A},
    //{0x0101, },
    {0x301C, 0x02},
    {0x302C, 0x85},
    {0x303A, 0xA4},
    {0x3108, 0x25},
    {0x310A, 0x27},
    {0x3122, 0x26},
    {0x3138, 0x26},
    {0x313A, 0x27},
    {0x316D, 0x0A},
    {0x308C, 0x00},
    {0x302E, 0x8C},
    {0x302F, 0x81},
    //Mode Setting FULL(All Pixel)
    {0x0340, 0x05}, //frame_length_lines
    {0x0341, 0x26}, //frame_length_lines
    {0x0342, 0x06}, //line_length
    {0x0343, 0x10}, //line_length
    {0x0346, 0x00}, //y_addr_start
    {0x0347, 0x00}, //y_addr_start
    {0x034A, 0x04}, //y_addr_end
    {0x034B, 0x0F}, //y_addr_end
    {0x034C, 0x05}, //x_output_size
    {0x034D, 0x10}, //x_output_size
    {0x034E, 0x04}, //y_output_size
    {0x034F, 0x10}, //y_output_size
    {0x0381, 0x01}, //x_even_inc
    {0x0383, 0x01}, //x_odd_inc
    {0x0385, 0x01}, //y_even_inc
    {0x0387, 0x01}, //y_odd_inc
    {0x3001, 0x00}, //HMODEADD
    {0x3016, 0x02}, //VMODEADD
    {0x3060, 0x30}, //CLPOWER_SMIA
    {0x30E8, 0x00}, //HADDAVE
    {0x3301, 0x01}, //RGCKREQSEL^M
    {0x308A, 0x43}, //Low Power
    {0x3305, 0x03}, //Mipi global timing
    {0x3309, 0x05}, //Mipi global timing
    {0x330B, 0x03}, //Mipi global timing
    {0x330D, 0x05}, //Mipi global timing
    {0x0101, 0x03}, /* image orientation */
    //Streaming
    {0x0100, 0x01},
    {IMX119_TABLE_WAIT_MS, 100},

    {IMX119_TABLE_END, 0x00}
};

static struct imx119_reg mode_144x176[] = {
    {0x0100, 0x00},
    {IMX119_TABLE_WAIT_MS, 100},
    //PLL setting EXTCLK=24Mhz 396Mbps
    {0x0305, 0x02},
    {0x0307, 0x21},
    {0x3025, 0x0A},
    {0x302B, 0x4B},
    {0x3022, 0x02}, //rear divider 1

    //Global(Initial) Setting
    {0x0112, 0x0A},
    {0x0113, 0x0A},
    //{0x0101, },
    {0x301C, 0x02},
    {0x302C, 0x85},
    {0x303A, 0xA4},
    {0x3108, 0x25},
    {0x310A, 0x27},
    {0x3122, 0x26},
    {0x3138, 0x26},
    {0x313A, 0x27},
    {0x316D, 0x0A},
    {0x308C, 0x00},
    {0x302E, 0x8C},
    {0x302F, 0x81},

    //Mode Setting FULL(All Pixel)
    {0x0340, 0x07}, //frame_length_lines
    {0x0341, 0x68}, //frame_length_lines
    {0x0342, 0x05}, //Line Length
    {0x0343, 0x70}, //Line Length
    {0x0344, 0x01}, //x_addr_start
    {0x0345, 0x68}, //x_addr_start
    {0x0346, 0x00}, //y_addr_start
    {0x0347, 0xA8}, //y_addr_start
    {0x0348, 0x03}, //x_addr_end
    {0x0349, 0xA7}, //x_addr_end
    {0x034A, 0x03}, //y_addr_end
    {0x034B, 0x67}, //y_addr_end
    {0x034C, 0x00}, //x_output_size
    {0x034D, 0x90}, //x_output_size
    {0x034E, 0x00}, //y_output_size
    {0x034F, 0xB0}, //y_output_size
    {0x0381, 0x05}, //x_even_inc
    {0x0383, 0x03}, //x_odd_inc
    {0x0385, 0x05}, //y_even_inc
    {0x0387, 0x03}, //y_odd_inc
    {0x3001, 0x80}, //HMODEADD
    {0x3016, 0x42}, //VMODEADD
    {0x3060, 0x30}, //CLPOWER_SMIA
    {0x30E8, 0x80}, //HADDAVE
    {0x3301, 0xC5}, //RGCKREQSEL^M
    {0x308A, 0x11}, //Low Power
    {0x3305, 0x00}, //Mipi global timing
    {0x3309, 0x02}, //Mipi global timing
    {0x330B, 0x00}, //Mipi global timing
    {0x330D, 0x02}, //Mipi global timing
    {0x0101, 0x03}, /* image orientation */
    //Streaming
    {0x0100, 0x01},
    {IMX119_TABLE_WAIT_MS, 100},
    {IMX119_TABLE_END, 0x00}
};

enum {
	IMX119_MODE_1280X1024,
	IMX119_MODE_144X176,
};

static struct imx119_reg *mode_table[] = {
	[IMX119_MODE_1280X1024] = mode_1280x1024,
	[IMX119_MODE_144X176] = mode_144x176,
};

static inline void
msleep_range(unsigned int delay_base)
{
	usleep_range(delay_base*1000, delay_base*1000+500);
}

static inline void
imx119_get_frame_length_regs(struct imx119_reg *regs, u32 frame_length)
{
	regs->addr = IMX119_FRAME_LEN_LINES_15_8;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = IMX119_FRAME_LEN_LINES_7_0;
	(regs + 1)->val = (frame_length) & 0xff;
}

static inline void
imx119_get_coarse_time_regs(struct imx119_reg *regs, u32 coarse_time)
{
	regs->addr = IMX119_COARSE_INTEGRATION_TIME_15_8;
	regs->val = (coarse_time >> 8) & 0xff;
	(regs + 1)->addr = IMX119_COARSE_INTEGRATION_TIME_7_0;
	(regs + 1)->val = (coarse_time) & 0xff;
}

static inline void
imx119_get_gain_reg(struct imx119_reg *regs, u16 gain, u32 dgain)
{
	regs->addr = IMX119_ANA_GAIN_GLOBAL;
	regs->val = gain & 0xFF;

	if (dgain >= 0x100) {
		(regs + 1)->addr = 0x020E;
		(regs + 1)->val = (dgain >> 8) & 0xFF;
		(regs + 2)->addr = 0x020F;
		(regs + 2)->val = ((dgain) & 0xFF);

		(regs + 3)->addr = 0x0210;
		(regs + 3)->val = (dgain >> 8) & 0xFF;
		(regs + 4)->addr = 0x0211;
		(regs + 4)->val = (dgain) & 0xFF;

		(regs + 5)->addr = 0x0212;
		(regs + 5)->val = (dgain >> 8) & 0xFF;
		(regs + 6)->addr = 0x0213;
		(regs + 6)->val = (dgain) & 0xFF;

		(regs + 7)->addr = 0x0214;
		(regs + 7)->val = (dgain >> 8) & 0xFF;
		(regs + 8)->addr = 0x0215;
		(regs + 8)->val = ((dgain) & 0xFF);
	}
}

static int
imx119_read_reg(struct i2c_client *client, u16 addr, u8 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[3] = {0x0, }; /*                                                                                  */

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

	*val = data[2];

	return 0;
}

static int
imx119_write_reg(struct i2c_client *client, u16 addr, u8 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[3];

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err == 1)
		return 0;

	dev_err(&client->dev, "%s:i2c write failed, %x = %x\n",
			__func__, addr, val);

	return err;
}

static int imx119_i2c_wr_blk(struct i2c_client *client, u8 *buf, int len)
{
	struct i2c_msg msg;
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = buf;
	if (i2c_transfer(client->adapter, &msg, 1) != 1)
		return -EIO;
	return 0;
}

static int
imx119_write_table(struct i2c_client *client,
			 const struct imx119_reg table[],
			 const struct imx119_reg override_list[],
			 int num_override_regs)
{
	int err;
	u8 i2c_transfer_buf[IMX119_SIZEOF_I2C_BUF];
	const struct imx119_reg *next;
	const struct imx119_reg *n_next;
	u8 *b_ptr = i2c_transfer_buf;
	u16 buf_count = 0;
	int i = 0;

	for (next = table; next->addr != IMX119_TABLE_END; next++) {
		if (next->addr == IMX119_TABLE_WAIT_MS) {
			msleep_range(next->val);
			continue;
		}

		if (!buf_count) {
			b_ptr = i2c_transfer_buf;
			*b_ptr++ = next->addr >> 8;
			*b_ptr++ = next->addr & 0xFF;
			buf_count = 2;
		}

		if (override_list[i].addr == next->addr) {
				*b_ptr++ = override_list[i].val;
				i++;
		}
		else
			*b_ptr++ = next->val;
		buf_count++;
		n_next = next + 1;
		if ((n_next->addr == next->addr + 1) &&
			(n_next->addr != IMX119_TABLE_WAIT_MS) &&
			(buf_count < IMX119_SIZEOF_I2C_BUF) &&
			(n_next->addr != IMX119_TABLE_END))
				continue;

		err = imx119_i2c_wr_blk(client, i2c_transfer_buf, buf_count);
		if (err) {
			pr_err("%s:imx119_write_table:%d", __func__, err);
			return err;
		}

		buf_count = 0;

	}

	return 0;
}

static void imx119_edp_lowest(struct imx119_info *info)
{
	if (!info->edpc)
		return;

	info->edp_state = info->edpc->num_states - 1;
	dev_dbg(&info->i2c_client->dev, "%s %d\n", __func__, info->edp_state);
	if (edp_update_client_request(info->edpc, info->edp_state, NULL)) {
		dev_err(&info->i2c_client->dev, "THIS IS NOT LIKELY HAPPEN!\n");
		dev_err(&info->i2c_client->dev,
			"UNABLE TO SET LOWEST EDP STATE!\n");
	}
}

static void imx119_edp_throttle(unsigned int new_state, void *priv_data);

static void imx119_edp_register(struct imx119_info *info)
{
	struct edp_manager *edp_manager;
	struct edp_client *edpc = &info->pdata->edpc_config;
	int ret;

	info->edpc = NULL;
	if (!edpc->num_states) {
		dev_notice(&info->i2c_client->dev,
			"%s: NO edp states defined.\n", __func__);
		return;
	}

	strncpy(edpc->name, "imx119", EDP_NAME_LEN - 1);
	edpc->name[EDP_NAME_LEN - 1] = 0;
	edpc->private_data = info;
	edpc->throttle = imx119_edp_throttle;

	dev_dbg(&info->i2c_client->dev, "%s: %s, e0 = %d, p %d\n",
		__func__, edpc->name, edpc->e0_index, edpc->priority);
	for (ret = 0; ret < edpc->num_states; ret++)
		dev_dbg(&info->i2c_client->dev, "e%d = %d mA",
			ret - edpc->e0_index, edpc->states[ret]);

	edp_manager = edp_get_manager("battery");
	if (!edp_manager) {
		dev_err(&info->i2c_client->dev,
			"unable to get edp manager: battery\n");
		return;
	}

	ret = edp_register_client(edp_manager, edpc);
	if (ret) {
		dev_err(&info->i2c_client->dev,
			"unable to register edp client\n");
		return;
	}

	info->edpc = edpc;
	/* set to lowest state at init */
	imx119_edp_lowest(info);
}

static int imx119_edp_req(struct imx119_info *info, unsigned new_state)
{
	unsigned approved;
	int ret = 0;

	if (!info->edpc)
		return 0;

	dev_dbg(&info->i2c_client->dev, "%s %d\n", __func__, new_state);
	ret = edp_update_client_request(info->edpc, new_state, &approved);
	if (ret) {
		dev_err(&info->i2c_client->dev, "E state transition failed\n");
		return ret;
	}

	if (approved > new_state) {
		dev_err(&info->i2c_client->dev, "EDP no enough current\n");
		return -ENODEV;
	}

	info->edp_state = approved;
	return 0;
}

static int
imx119_set_mode(struct imx119_info *info, struct imx119_mode *mode)
{
	struct device *dev = &info->i2c_client->dev;
	int sensor_mode;
	int err;
	struct imx119_reg reg_list[16];

	dev_info(dev, "%s: res [%ux%u] framelen %u coarsetime %u gain %u\n",
		__func__, mode->xres, mode->yres,
		mode->frame_length, mode->coarse_time, mode->gain);

	if ((mode->xres == 1280) && (mode->yres == 1024)) {
		sensor_mode = IMX119_MODE_1280X1024;
	} else if ((mode->xres == 144) && (mode->yres == 176)) {
		sensor_mode = IMX119_MODE_144X176;
	} else {
		dev_err(dev, "%s: invalid resolution to set mode %d %d\n",
			__func__, mode->xres, mode->yres);
		return -EINVAL;
	}

	/* request highest edp state */
	err = imx119_edp_req(info, 0);
	if (err) {
		dev_err(&info->i2c_client->dev,
			"%s: ERROR cannot set edp state! %d\n",	__func__, err);
		return err;
	}

	/*
	 * get a list of override regs for the asking frame length,
	 * coarse integration time, and gain.
	 */
	imx119_get_frame_length_regs(reg_list, mode->frame_length);
	imx119_get_coarse_time_regs(reg_list + 2, mode->coarse_time);
	imx119_get_gain_reg(reg_list + 4, mode->gain, mode->dgain);

	err = imx119_write_table(info->i2c_client, mode_table[sensor_mode],
			reg_list, 5);

	if (err)
		return err;

	info->mode = sensor_mode;
	dev_info(dev, "[imx119]: stream on.\n");
	return 0;
}

static int
imx119_get_status(struct imx119_info *info, u8 *dev_status)
{
	/* TBD */
	*dev_status = 0;
	return 0;
}

static int
imx119_set_frame_length(struct imx119_info *info,
				u32 frame_length,
				bool group_hold)
{
	struct imx119_reg reg_list[2];
	int i = 0;
	int ret;

	imx119_get_frame_length_regs(reg_list, frame_length);

	if (group_hold) {
		ret = imx119_write_reg(info->i2c_client,
					IMX119_GROUP_PARAM_HOLD, 0x01);
		if (ret)
			return ret;
	}

	for (i = 0; i < NUM_OF_FRAME_LEN_REG; i++) {
		ret = imx119_write_reg(info->i2c_client, reg_list[i].addr,
			reg_list[i].val);
		if (ret)
			return ret;
	}

	if (group_hold) {
		ret = imx119_write_reg(info->i2c_client,
					IMX119_GROUP_PARAM_HOLD, 0x0);
		if (ret)
			return ret;
	}

	return 0;
}

static int
imx119_set_coarse_time(struct imx119_info *info,
				u32 coarse_time,
				bool group_hold)
{
	int ret;

	struct imx119_reg reg_list[2];
	int i = 0;

	imx119_get_coarse_time_regs(reg_list, coarse_time);

	if (group_hold) {
		ret = imx119_write_reg(info->i2c_client,
					IMX119_GROUP_PARAM_HOLD,
					0x01);
		if (ret)
			return ret;
	}

	for (i = 0; i < NUM_OF_COARSE_TIME_REG; i++) {
		ret = imx119_write_reg(info->i2c_client, reg_list[i].addr,
			reg_list[i].val);
		if (ret)
			return ret;
	}

	if (group_hold) {
		ret = imx119_write_reg(info->i2c_client,
					IMX119_GROUP_PARAM_HOLD, 0x0);
		if (ret)
			return ret;
	}
	return 0;
}

static int
imx119_set_gain(struct imx119_info *info, u16 gain, u32 dgain, bool group_hold)
{
	int ret;
	int i;
	struct imx119_reg reg_list[16];

	imx119_get_gain_reg(reg_list, gain, dgain);

	if (group_hold) {
		ret = imx119_write_reg(info->i2c_client,
					IMX119_GROUP_PARAM_HOLD, 0x1);
		if (ret)
			return ret;
	}

	for (i = 0; i < NUM_OF_GAIN_REG; i++) {
		ret = imx119_write_reg(info->i2c_client, reg_list[i].addr,
			reg_list[i].val);
		if (ret)
			return ret;
		if (dgain < 0x100)
             		break;
	}

	if (group_hold) {
		ret = imx119_write_reg(info->i2c_client,
					IMX119_GROUP_PARAM_HOLD, 0x0);
		if (ret)
			return ret;
	}
	return 0;
}

static int
imx119_set_group_hold(struct imx119_info *info, struct imx119_ae *ae)
{
	int ret;
	int count = 0;
	bool groupHoldEnabled = false;

	if (ae->gain_enable)
		count++;
	if (ae->coarse_time_enable)
		count++;
	if (ae->frame_length_enable)
		count++;
	if (count >= 2)
		groupHoldEnabled = true;

	if (groupHoldEnabled) {
		ret = imx119_write_reg(info->i2c_client,
					IMX119_GROUP_PARAM_HOLD, 0x1);
		if (ret)
			return ret;
	}

	if (ae->gain_enable)
		imx119_set_gain(info, ae->gain, ae->dgain, false);
	if (ae->coarse_time_enable)
		imx119_set_coarse_time(info, ae->coarse_time, false);
	if (ae->frame_length_enable)
		imx119_set_frame_length(info, ae->frame_length, false);

	if (groupHoldEnabled) {
		ret = imx119_write_reg(info->i2c_client,
					IMX119_GROUP_PARAM_HOLD, 0x0);
		if (ret)
			return ret;
	}

	return 0;
}

static int imx119_get_fuse_id(struct imx119_info *info)
{
	int ret = 0;
	int i;
	u8 bak = 0;

	if (info->fuse_id.size)
		return 0;

	/*
	 * TBD 1: If the sensor does not have power at this point
	 * Need to supply the power, e.g. by calling power on function
	 */
	msleep_range(IMX119_FUSE_ID_DELAY);

	for (i = 0; i < IMX119_FUSE_ID_SIZE ; i++) {
		ret |= imx119_read_reg(info->i2c_client,
					IMX119_FUSE_ID_REG + i, &bak);
		info->fuse_id.data[i] = bak;
	}

	if (!ret)
		info->fuse_id.size = i;

	return ret;
}

static long
imx119_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	int err;
	struct imx119_info *info = file->private_data;
	struct device *dev = &info->i2c_client->dev;

	switch (cmd) {
	case IMX119_IOCTL_SET_MODE:
	{
		struct imx119_mode mode;
		if (copy_from_user(&mode,
			(const void __user *)arg,
			sizeof(struct imx119_mode))) {
			dev_err(dev, "%s:Failed to get mode from user.\n",
			__func__);
			return -EFAULT;
		}
		return imx119_set_mode(info, &mode);
	}
	case IMX119_IOCTL_SET_FRAME_LENGTH:
		return imx119_set_frame_length(info, (u32)arg, true);
	case IMX119_IOCTL_SET_COARSE_TIME:
		return imx119_set_coarse_time(info, (u32)arg, true);
	case IMX119_IOCTL_SET_GAIN:
		return imx119_set_gain(info, (u16)arg, 0x00, true);
	case IMX119_IOCTL_GET_STATUS:
	{
		u8 status;

		err = imx119_get_status(info, &status);
		if (err)
			return err;
		if (copy_to_user((void __user *)arg, &status, 1)) {
			dev_err(dev, "%s:Failed to copy status to user.\n",
			__func__);
			return -EFAULT;
		}
		return 0;
	}
	case IMX119_IOCTL_GET_FUSEID:
	{
		err = imx119_get_fuse_id(info);

		if (err) {
			dev_err(dev, "%s:Failed to get fuse id info.\n",
			__func__);
			return err;
		}
		if (copy_to_user((void __user *)arg,
				&info->fuse_id,
				sizeof(struct nvc_fuseid))) {
			dev_info(dev, "%s:Fail copy fuse id to user space\n",
				__func__);
			return -EFAULT;
		}
		return 0;
	}
	case IMX119_IOCTL_SET_GROUP_HOLD:
	{
		struct imx119_ae ae;
		if (copy_from_user(&ae, (const void __user *)arg,
				sizeof(struct imx119_ae))) {
			dev_info(dev, "%s:fail group hold\n", __func__);
			return -EFAULT;
		}
		return imx119_set_group_hold(info, &ae);
	}
	default:
		dev_err(dev, "%s:unknown cmd.\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static int imx119_get_extra_regulators(void)
{
	if (!imx119_ext_reg1) {
		imx119_ext_reg1 = regulator_get(NULL, "imx119_reg1");
		if (WARN_ON(IS_ERR(imx119_ext_reg1))) {
			pr_err("%s: can't get regulator imx119_reg1: %ld\n",
				__func__, PTR_ERR(imx119_ext_reg1));
			imx119_ext_reg1 = NULL;
			return -ENODEV;
		}
	}

	if (!imx119_ext_reg2) {
		imx119_ext_reg2 = regulator_get(NULL, "imx119_reg2");
		if (unlikely(WARN_ON(IS_ERR(imx119_ext_reg2)))) {
			pr_err("%s: can't get regulator imx119_reg2: %ld\n",
				__func__, PTR_ERR(imx119_ext_reg2));
			imx119_ext_reg2 = NULL;
			regulator_put(imx119_ext_reg1);
			return -ENODEV;
		}
	}

	return 0;
}

static void imx119_mclk_disable(struct imx119_info *info)
{
	dev_dbg(&info->i2c_client->dev, "%s: disable MCLK\n", __func__);
	clk_disable_unprepare(info->mclk);
}

static int imx119_mclk_enable(struct imx119_info *info)
{
	int err;
	unsigned long mclk_init_rate = 24000000;

	dev_dbg(&info->i2c_client->dev, "%s: enable MCLK with %lu Hz\n",
		__func__, mclk_init_rate);

	err = clk_set_rate(info->mclk, mclk_init_rate);
	if (!err)
		err = clk_prepare_enable(info->mclk);
	return err;
}

static int imx119_power_on(struct imx119_info *info)
{
	int err;
	struct imx119_power_rail *pw = &info->power;
	unsigned int cam2_gpio = info->pdata->cam2_gpio;

	if (unlikely(WARN_ON(!pw || !pw->avdd || !pw->iovdd || !pw->dvdd)))
		return -EFAULT;

	if (info->pdata->ext_reg) {
		if (imx119_get_extra_regulators())
			goto imx119_poweron_fail;

		err = regulator_enable(imx119_ext_reg1);
		if (unlikely(err))
			goto imx119_i2c_fail;

		err = regulator_enable(imx119_ext_reg2);
		if (unlikely(err))
			goto imx119_vcm_fail;
	}

	gpio_set_value(cam2_gpio, 0);

	err = regulator_enable(pw->dvdd);
	if (unlikely(err))
		goto imx119_dvdd_fail;

	err = regulator_enable(pw->iovdd);
	if (unlikely(err))
		goto imx119_iovdd_fail;

	err = regulator_enable(pw->avdd);
	if (unlikely(err))
		goto imx119_avdd_fail;

	usleep_range(1, 2);

	gpio_set_value(cam2_gpio, 1);

	return 0;

imx119_avdd_fail:
	if (info->pdata->ext_reg)
		regulator_disable(imx119_ext_reg2);

imx119_iovdd_fail:
	regulator_disable(pw->dvdd);

imx119_dvdd_fail:
	regulator_disable(pw->avdd);

imx119_vcm_fail:
	if (info->pdata->ext_reg)
		regulator_disable(imx119_ext_reg1);

imx119_i2c_fail:

imx119_poweron_fail:
	pr_err("%s failed.\n", __func__);
	return -ENODEV;
}

static int imx119_power_off(struct imx119_info *info)
{
	struct imx119_power_rail *pw = &info->power;
	unsigned int cam2_gpio = info->pdata->cam2_gpio;

	if (!info->i2c_client->dev.of_node) {
		if (info->pdata && info->pdata->power_off)
			info->pdata->power_off(&info->power);
		goto imx119_pwroff_end;
	}

	if (unlikely(WARN_ON(!pw || !pw->avdd || !pw->iovdd || !pw->dvdd)))
		return -EFAULT;

	gpio_set_value(cam2_gpio, 0);

	usleep_range(1, 2);

	regulator_disable(pw->avdd);
	regulator_disable(pw->iovdd);
	regulator_disable(pw->dvdd);

	if (info->pdata->ext_reg) {
		regulator_disable(imx119_ext_reg1);
		regulator_disable(imx119_ext_reg2);
	}

imx119_pwroff_end:
	imx119_edp_lowest(info);
	return 0;
}

static void imx119_edp_throttle(unsigned int new_state, void *priv_data)
{
	struct imx119_info *info = priv_data;

	imx119_power_off(info);
}

static int
imx119_open(struct inode *inode, struct file *file)
{
	int err;
	struct miscdevice	*miscdev = file->private_data;
	struct imx119_info	*info;

	info = container_of(miscdev, struct imx119_info, miscdev_info);
	/* check if the device is in use */
	if (atomic_xchg(&info->in_use, 1)) {
		dev_info(&info->i2c_client->dev, "%s:BUSY!\n", __func__);
		return -EBUSY;
	}

	file->private_data = info;

	err = imx119_mclk_enable(info);
	if (err < 0)
		return err;

	if (info->i2c_client->dev.of_node) {
		err = imx119_power_on(info);
	} else {
		if (info->pdata && info->pdata->power_on)
			err = info->pdata->power_on(&info->power);
		else {
			dev_err(&info->i2c_client->dev,
				"%s:no valid power_on function.\n", __func__);
			err = -EEXIST;
		}
	}
	if (err < 0)
		goto imx119_open_fail;

	return 0;

imx119_open_fail:
	imx119_mclk_disable(info);
	return err;
}

static int
imx119_release(struct inode *inode, struct file *file)
{
	struct imx119_info *info = file->private_data;

	imx119_power_off(info);

	imx119_mclk_disable(info);

	file->private_data = NULL;

	/* warn if device is already released */
	WARN_ON(!atomic_xchg(&info->in_use, 0));
	return 0;
}

static int imx119_power_put(struct imx119_power_rail *pw)
{
	if (likely(pw->dvdd))
		regulator_put(pw->dvdd);

	if (likely(pw->avdd))
		regulator_put(pw->avdd);

	if (likely(pw->iovdd))
		regulator_put(pw->iovdd);

	if (likely(imx119_ext_reg1))
		regulator_put(imx119_ext_reg1);

	if (likely(imx119_ext_reg2))
		regulator_put(imx119_ext_reg2);

	pw->dvdd = NULL;
	pw->avdd = NULL;
	pw->iovdd = NULL;
	imx119_ext_reg1 = NULL;
	imx119_ext_reg2 = NULL;

	return 0;
}

static int imx119_regulator_get(struct imx119_info *info,
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

static int imx119_power_get(struct imx119_info *info)
{
	struct imx119_power_rail *pw = &info->power;

	imx119_regulator_get(info, &pw->dvdd, "vdig_imx119"); /* digital 1.2v */
	imx119_regulator_get(info, &pw->avdd, "vana_imx119"); /* analog 2.7v */
	imx119_regulator_get(info, &pw->iovdd, "vif_imx119"); /* interface 1.8v */

	return 0;
}

static const struct file_operations imx119_fileops = {
	.owner = THIS_MODULE,
	.open = imx119_open,
	.unlocked_ioctl = imx119_ioctl,
	.release = imx119_release,
};

static struct miscdevice imx119_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "imx119",
	.fops = &imx119_fileops,
};

static struct of_device_id imx119_of_match[] = {
	{ .compatible = "nvidia,imx119", },
	{ },
};

MODULE_DEVICE_TABLE(of, imx119_of_match);

static struct imx119_platform_data *imx119_parse_dt(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	struct imx119_platform_data *board_info_pdata;
	const struct of_device_id *match;

	match = of_match_device(imx119_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_info_pdata = devm_kzalloc(&client->dev, sizeof(*board_info_pdata),
			GFP_KERNEL);
	if (!board_info_pdata) {
		dev_err(&client->dev, "Failed to allocate pdata\n");
		return NULL;
	}

	board_info_pdata->cam2_gpio = of_get_named_gpio(np, "cam2_gpios", 0);

	board_info_pdata->ext_reg = of_property_read_bool(np, "nvidia,ext_reg");

	return board_info_pdata;
}

static int
imx119_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct imx119_info *info;
	const char *mclk_name;
	int err = 0;

	pr_info("[imx119]: probing sensor.\n");

	info = devm_kzalloc(&client->dev,
		sizeof(struct imx119_info), GFP_KERNEL);
	if (!info) {
		pr_err("[imx119]:%s:Unable to allocate memory!\n", __func__);
		return -ENOMEM;
	}

	if (client->dev.of_node)
		info->pdata = imx119_parse_dt(client);
	else
		info->pdata = client->dev.platform_data;

	if (!info->pdata) {
		pr_err("[imx119]:%s:Unable to get platform data\n", __func__);
		return -EFAULT;
	}

	info->i2c_client = client;
	atomic_set(&info->in_use, 0);
	info->mode = -1;

	mclk_name = info->pdata->mclk_name ?
		    info->pdata->mclk_name : "default_mclk";
	info->mclk = devm_clk_get(&client->dev, mclk_name);
	if (IS_ERR(info->mclk)) {
		dev_err(&client->dev, "%s: unable to get clock %s\n",
			__func__, mclk_name);
		return PTR_ERR(info->mclk);
	}

	i2c_set_clientdata(client, info);

	imx119_power_get(info);

	imx119_edp_register(info);

	memcpy(&info->miscdev_info,
		&imx119_device,
		sizeof(struct miscdevice));

	err = misc_register(&info->miscdev_info);
	if (err) {
		imx119_power_put(&info->power);
		pr_err("[imx119]:%s:Unable to register misc device!\n",
		__func__);
	}

#ifdef CONFIG_DEBUG_FS
	info->debugfs_info.name = imx119_device.name;
	info->debugfs_info.i2c_client = info->i2c_client;
	info->debugfs_info.i2c_addr_limit = 0xFFFF;
	info->debugfs_info.i2c_rd8 = imx119_read_reg;
	info->debugfs_info.i2c_wr8 = imx119_write_reg;
	nvc_debugfs_init(&(info->debugfs_info));
#endif

	return err;
}

static int
imx119_remove(struct i2c_client *client)
{
	struct imx119_info *info = i2c_get_clientdata(client);
#ifdef CONFIG_DEBUG_FS
	nvc_debugfs_remove(&info->debugfs_info);
#endif
	imx119_power_put(&info->power);
	misc_deregister(&imx119_device);
	return 0;
}

static const struct i2c_device_id imx119_id[] = {
	{ "imx119", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, imx119_id);

static struct i2c_driver imx119_i2c_driver = {
	.driver = {
		.name = "imx119",
		.owner = THIS_MODULE,
	},
	.probe = imx119_probe,
	.remove = imx119_remove,
	.id_table = imx119_id,
};

static int __init
imx119_init(void)
{
	pr_info("[imx119] sensor driver loading\n");
	return i2c_add_driver(&imx119_i2c_driver);
}

static void __exit
imx119_exit(void)
{
	i2c_del_driver(&imx119_i2c_driver);
}

module_init(imx119_init);
module_exit(imx119_exit);
