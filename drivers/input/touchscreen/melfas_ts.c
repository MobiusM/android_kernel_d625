/* drivers/input/touchscreen/melfas_ts.c
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

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/i2c/melfas_ts.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>

#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#if defined(CONFIG_MACH_PDA)
#include <mach/board_lge.h>
#endif

#define MODE_CONTROL                    	0x01
#define TS_READ_START_ADDR              	0x10

#define TS_READ_START_ADDR			0x10
#define TS_READ_VERSION_ADDR			0xE1
#define TS_HW_REVISION_ADDR             	0xF1
#define TS_CORE_VERSION_ADDR            	0xF3
#define TS_PRIVATE_CUSTOM_VERSION_ADDR  	0xF4
#define TS_PUBLIC_CUSTOM_VERSION_ADDR   	0xF5

#define UNIVERSAL_CMD				0xA0
#define UNIVERSAL_CMD_RESULT_SIZE		0xAE
#define UNIVERSAL_CMD_RESULT			0xAF
#define UNIVCMD_ENTER_TEST_MODE			0x40
#define UNIVCMD_TEST_CM_DELTA			0x41
#define UNIVCMD_GET_PIXEL_CM_DELTA		0x42
#define UNIVERSAL_CMD_EXIT			0x4F

#define TS_READ_REGS_LEN			100
#define TS_READ_VERSION_INFO_LEN		3

#define MELFAS_MAX_TOUCH			10  /* ts->pdata->num_of_finger */
#define MELFAS_MAX_BTN				4
#define MELFAS_PACKET_SIZE			6

#define I2C_RETRY_CNT				10

#define PRESS_KEY				1
#define RELEASE_KEY				0

#define SET_DOWNLOAD_BY_GPIO			1

#define MIP_INPUT_EVENT_PACKET_SIZE		0x0F
#define MIP_INPUT_EVENT_INFORMATION		0x10

#define FW_VERSION_ADDR 	0

/* need to update define value when FW changed */
#define D1LK_TOVIS_VER 0xcb
#define D1LK_LGIT_VER 0x07

/*#include "mms136_download.h"*/

#define get_time_interval(a, b) (a >= b ? a-b : 1000000 + a - b)
struct timeval t_debug[2];

static volatile int init_values[20];
static volatile int irq_flag;
static volatile int tmp_flag[10];
static volatile int point_of_release;
static volatile int time_diff;
static volatile int pre_keycode;
static volatile int touch_prestate;
static volatile int btn_prestate;


static int num_tx_line = 21; //default value for d1lu,k,sk,v
static int num_rx_line = 12; //default value for d1lu,k,sk,v

/***************************************************************************
 * Debug Definitions
 ***************************************************************************/
enum {
	MELFAS_TS_DEBUG_PROBE = 1U << 0,
	MELFAS_TS_DEBUG_KEY_EVENT = 1U << 1,
	MELFAS_TS_DEBUG_TOUCH_EVENT = 1U << 2,
	MELFAS_TS_DEBUG_TOUCH_EVENT_ONETIME = 1U << 3,
	MELFAS_TS_DEBUG_EVENT_HANDLER = 1U << 4,
	MELFAS_TS_DEBUG_IRQ_HANDLER = 1U << 5,
	MELFAS_TS_DEBUG_TIME = 1U << 6,
};

static int melfas_ts_debug_mask = 9;
//static int melfas_ts_debug_mask = 0x0;

module_param_named(
	debug_mask, melfas_ts_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP
);
#define MELFAS_TS_DPRINTK(mask, level, message, ...) \
	do { \
		if ((mask) & melfas_ts_debug_mask) \
			printk(level message, ## __VA_ARGS__); \
	} while (0)

#define MELFAS_TS_DEBUG_PRINT_TOUCH_EVENT(temp) \
	do { \
		MELFAS_TS_DPRINTK(MELFAS_TS_DEBUG_TOUCH_EVENT, KERN_INFO, \
			"[TOUCH] %s   %d : x=%d y=%d p=%d \n", \
			temp, i, g_Mtouch_info[i].posX, g_Mtouch_info[i].posY, g_Mtouch_info[i].pressure); \
		if (tmp_flag[i] == 1) { \
			MELFAS_TS_DPRINTK(MELFAS_TS_DEBUG_TOUCH_EVENT_ONETIME, KERN_INFO, \
			"[TOUCH] %s   %d : x=%d y=%d p=%d \n", \
			temp, i, g_Mtouch_info[i].posX, g_Mtouch_info[i].posY, g_Mtouch_info[i].pressure); \
			if (!strcmp (temp, "Press")) \
				tmp_flag[i] = 0;\
		} \
	} while (0)

#define MELFAS_TS_DEBUG_PRINT_TIME() \
	do { \
		if (MELFAS_TS_DEBUG_TIME & melfas_ts_debug_mask) { \
			if (t_debug[0].tv_sec == 0	&& t_debug[0].tv_usec == 0) { \
				t_debug[0].tv_sec = t_debug[1].tv_sec; \
				t_debug[0].tv_usec = t_debug[1].tv_usec; \
			} else { \
				printk("Interrupt interval: %6luus\n", get_time_interval(t_debug[1].tv_usec, t_debug[0].tv_usec)); \
				t_debug[0].tv_sec = t_debug[1].tv_sec; \
				t_debug[0].tv_usec = t_debug[1].tv_usec; \
			} \
		} \
	} while (0)

/**************************************************************************/

enum {
	None = 0,
	TOUCH_SCREEN,
	TOUCH_KEY
};

struct muti_touch_info {
	int strength;
	int width;
	int posX;
	int posY;
	int pressure;
	int btn_touch;
};

struct btn_info {
	int key_code;
	int status;
};

struct melfas_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct melfas_tsi_platform_data *pdata;

	int version;
/*           
                                                
                                   */
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	struct regulator *vdd;
	struct regulator *vcc_i2c;

	char fw_update;
	char poweron;
};


struct melfas_ts_data *gts;
extern int mms_flash_fw_file(struct i2c_client *client, struct melfas_tsi_platform_data *pdata);
/*           
                                                
                                   */
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#elif defined (CONFIG_HAS_EARLYSUSPEND)
static void melfas_ts_early_suspend(struct early_suspend *h);
static void melfas_ts_late_resume(struct early_suspend *h);
#endif

static void melfas_ts_suspend(struct melfas_ts_data *ts);
static void melfas_ts_resume(struct melfas_ts_data *ts);

static void release_all_fingers(struct melfas_ts_data *ts);
static void melfas_ts_sw_reset(struct melfas_ts_data *ts);

static int melfas_power_on(struct melfas_ts_data *ts, bool on);

static struct muti_touch_info g_Mtouch_info[MELFAS_MAX_TOUCH];
static struct btn_info g_btn_info[MELFAS_MAX_BTN];

static int check_abs_time(void)
{
	time_diff = 0;

	if (!point_of_release)
		return 0;

	time_diff = jiffies_to_msecs(jiffies) - point_of_release;
	if (time_diff > 0)
		return time_diff;
	else
		return 0;
}

static void melfas_ts_event_handler(struct melfas_ts_data *ts)
{
	int ret = 0, i;
	uint8_t buf[TS_READ_REGS_LEN];
	int touchType = 0, touchState = 0, touchID = 0, pressed_type = 0;
	int posX = 0, posY = 0, width = 0, strength = 10;
	int keyID = 0, reportID = 0;
	uint8_t read_num = 0, pressed_count = 0;
	static int is_mix_event;

	MELFAS_TS_DPRINTK(MELFAS_TS_DEBUG_EVENT_HANDLER, KERN_INFO,
			"[TOUCH] melfas_ts_event_handler \n");

	if (ts == NULL) {
		MELFAS_TS_DPRINTK(MELFAS_TS_DEBUG_EVENT_HANDLER, KERN_ERR,
				"[TOUCH] melfas_ts_event_handler TS is NULL\n");
		goto err_free_irq;
	}

	buf[0] = MIP_INPUT_EVENT_PACKET_SIZE;
	ret = i2c_master_send(ts->client, buf, 1);
	ret = i2c_master_recv(ts->client, &read_num, 1);
	/* touch ic reset for ESD defense  */
	if (ret < 0) {
		melfas_ts_sw_reset(ts);
		goto err_free_irq;
	}

	if (read_num == 0) {
		MELFAS_TS_DPRINTK(MELFAS_TS_DEBUG_EVENT_HANDLER, KERN_ERR,
				"[TOUCH] melfas_ts_event_handler: read number 0 \n");
		goto err_free_irq;
	} else if (read_num > MELFAS_MAX_TOUCH*MELFAS_PACKET_SIZE) {
		MELFAS_TS_DPRINTK(MELFAS_TS_DEBUG_EVENT_HANDLER, KERN_ERR,
				"[TOUCH] melfas_ts_event_handler: read number is out of range\n");
		goto err_free_irq;
	}

	buf[0] = MIP_INPUT_EVENT_INFORMATION;
	ret = i2c_master_send(ts->client, buf, 1);
	ret = i2c_master_recv(ts->client, &buf[0], read_num);

	/* touch ic reset for ESD defense
	     if reportID is -0x0F, meflas touch IC need sw reset */
	reportID = (buf[0] & 0x0F);
	if (reportID == 0x0F) {
		printk(KERN_ERR "[TOUCH] ESD 0x0F : ");
		melfas_ts_sw_reset(ts);
		goto err_free_irq;
	}

	for (i = 0; i < read_num; i = i + 6) {
		if (ret < 0) {
			MELFAS_TS_DPRINTK(MELFAS_TS_DEBUG_EVENT_HANDLER, KERN_ERR,
				"[TOUCH] melfas_ts_event_handler: i2c failed\n");
			goto err_free_irq;
		} else {
			touchType  =  ((buf[i] & 0x60) >> 5);				/* Touch Screen, Touch Key */
			touchState = ((buf[i] & 0x80) == 0x80);				/* touchAction = (buf[0]>>7)&&0x01;*/
			reportID = (buf[i] & 0x0F);					/* Touch Screen -> n.th finger input
											Touch Key -> n.th touch key area. */
			posX = (uint16_t) (buf[i + 1] & 0x0F) << 8 | buf[i + 2];	/* X position (0 ~ 4096, 12 bit) */
			posY = (uint16_t) (buf[i + 1] & 0xF0) << 4 | buf[i + 3];	/* Y position (0 ~ 4096, 12 bit) */
			width = buf[i + 4];
			strength = buf[i + 5];

			if (touchType == TOUCH_KEY)
				keyID = reportID;
			else if (touchType == TOUCH_SCREEN) {
				touchID = reportID-1;
				pressed_type = TOUCH_SCREEN;
			}

			if (touchID > ts->pdata->num_of_finger-1)
				goto err_free_irq;

			if (touchType == TOUCH_SCREEN && touchID < MELFAS_MAX_TOUCH) {
				g_Mtouch_info[touchID].posX = posX;
				g_Mtouch_info[touchID].posY = posY;
				g_Mtouch_info[touchID].width = width;

				g_Mtouch_info[touchID].strength = strength;
				g_Mtouch_info[touchID].pressure = strength;
				g_Mtouch_info[touchID].btn_touch = touchState;

				if (btn_prestate && touch_prestate == 0) {
					input_report_key(ts->input_dev, pre_keycode, 0xff);
					btn_prestate = 0;
				}
			} else if (touchType == TOUCH_KEY) {
				g_btn_info[keyID].key_code = ts->pdata->button[keyID-1];
				g_btn_info[keyID].status = touchState;

				if (keyID > ts->pdata->num_of_button || keyID == 0) {
					MELFAS_TS_DPRINTK(MELFAS_TS_DEBUG_EVENT_HANDLER, KERN_ERR, "[TOUCH] Touchkey ID error \n");
				} else if (is_mix_event == 1) {
					input_report_key(ts->input_dev, pre_keycode, 0xff);
					is_mix_event = 0;
					btn_prestate = touchState;
				} else{
					if (touch_prestate) {
						btn_prestate = touchState;
					} else if (check_abs_time() > 0 && check_abs_time() < 100) {
						btn_prestate = touchState;
						point_of_release = 0;
					} else if (btn_prestate != touchState) {
						if (touchState == PRESS_KEY) {
							pre_keycode = ts->pdata->button[keyID-1];
							input_report_key(ts->input_dev, ts->pdata->button[keyID-1], PRESS_KEY);
						} else {
							input_report_key(ts->input_dev, ts->pdata->button[keyID-1], RELEASE_KEY);
						}
						btn_prestate = touchState;
					}
				}

				if ((read_num > 6) && (pressed_type == TOUCH_SCREEN)) {
					if (touchState && (touch_prestate == 0))
						is_mix_event = 1;
					touchType = TOUCH_SCREEN;
				}

				MELFAS_TS_DPRINTK(MELFAS_TS_DEBUG_KEY_EVENT, KERN_INFO,
					"[TOUCH] melfas_ts_event_handler: keyID : %d, touchState: %d\n",
					keyID, touchState);
				break;
			}

		}
	}

	if (touchType == TOUCH_SCREEN) {
		for (i = 0; i < ts->pdata->num_of_finger; i++) {
			if (g_Mtouch_info[i].btn_touch == -1)
				continue;

			if (g_Mtouch_info[i].btn_touch == 0) {
				g_Mtouch_info[i].btn_touch = -1;
				tmp_flag[i] = 1;
				MELFAS_TS_DEBUG_PRINT_TOUCH_EVENT("Release");
				continue;
			}
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, g_Mtouch_info[i].posX);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, g_Mtouch_info[i].posY);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, g_Mtouch_info[i].width);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, g_Mtouch_info[i].strength);
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
			input_mt_sync(ts->input_dev);
			MELFAS_TS_DEBUG_PRINT_TOUCH_EVENT("Press");
			touch_prestate = 1;
			pressed_count++;
		}
		if (pressed_count == 0) {
			input_mt_sync(ts->input_dev);
			touch_prestate = 0;
			point_of_release = jiffies_to_msecs(jiffies);
		}
	}
	input_sync(ts->input_dev);

	MELFAS_TS_DEBUG_PRINT_TIME();
	return;
err_free_irq:
	return;
}

static irqreturn_t melfas_ts_irq_handler(int irq, void *handle)
{
	struct melfas_ts_data *ts = (struct melfas_ts_data *)handle;

	irq_flag = 1;

	MELFAS_TS_DPRINTK(MELFAS_TS_DEBUG_IRQ_HANDLER, KERN_INFO, "melfas_ts_irq_handler\n");

	if (MELFAS_TS_DEBUG_TIME & melfas_ts_debug_mask)
	    do_gettimeofday(&t_debug[1]);

	melfas_ts_event_handler(ts);

	irq_flag = 0;
	return IRQ_HANDLED;
}

static ssize_t
mms136_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	uint8_t verbuf[TS_READ_VERSION_INFO_LEN];
	int len;

	verbuf[0] = TS_READ_VERSION_ADDR;
	i2c_master_send(ts->client, &verbuf[0], 1);
	i2c_master_recv(ts->client, &verbuf[0], TS_READ_VERSION_INFO_LEN);

	ts->version = verbuf[FW_VERSION_ADDR];

	len = snprintf(buf, PAGE_SIZE, "%d\n", ts->version);
	return len;
}

static ssize_t
mms136_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	int len;
	len = snprintf(buf, PAGE_SIZE, "\nMMS-136 Device Status\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "=============================\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "irq num       is %d\n", ts->client->irq);
	len += snprintf(buf + len, PAGE_SIZE - len, "gpio_irq num  is %d(level=%d)\n", ts->pdata->i2c_int_gpio, gpio_get_value(ts->pdata->i2c_int_gpio));
	len += snprintf(buf + len, PAGE_SIZE - len, "gpio_scl num  is %d\n", ts->pdata->gpio_scl);
	len += snprintf(buf + len, PAGE_SIZE - len, "gpio_sda num  is %d\n", ts->pdata->gpio_sda);
	return len;
}

static ssize_t
mms136_fw_upgrade_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	int cmd, ret;
	uint8_t verbuf[TS_READ_VERSION_INFO_LEN];

	if (sscanf(buf, "%d", &cmd) != 1)
		return -EINVAL;
	switch (cmd) {
	case 0:
		printk(KERN_ERR "melfas mms_flash_fw_file() \n");

		disable_irq(ts->client->irq);
		ts->fw_update = 1;

		mms_flash_fw_file(ts->client, ts->pdata);

		ts->fw_update = 0;
		enable_irq(ts->client->irq);

		melfas_ts_sw_reset(ts);

		mdelay(200);

		memset(verbuf, 0x0, TS_READ_VERSION_INFO_LEN);

		verbuf[0] = TS_READ_VERSION_ADDR;
		ret = i2c_master_send(ts->client, &verbuf[0], 1);
		ret = i2c_master_recv(ts->client, &verbuf[0], TS_READ_VERSION_INFO_LEN);

		ts->version = verbuf[FW_VERSION_ADDR];

		printk(KERN_ERR "[TOUCH] = Melfas Version Info =\n");
		printk(KERN_ERR "[TOUCH] Bootloader Version :: %d Core Version :: (%X), Config Version :: %d\n", verbuf[0], verbuf[1], verbuf[2]);

		if (irq_flag == 1) {
			printk("enable_irq\n");
			enable_irq(ts->client->irq);
			irq_flag = 0;
		}
		break;

	default:
		printk(KERN_ERR "usage: echo [1|2|3] > fw_upgrade\n");
		printk(KERN_ERR "  - 0: firmware upgrade(ISP) with firmware image\n");
		break;
	}
	return count;
}

static ssize_t
mms136_power_control_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	int cmd;

	if (sscanf(buf, "%d", &cmd) != 1)
		return -EINVAL;

	switch (cmd) {
	case 1: /* touch power on */
		melfas_power_on(ts, true);
		break;
	case 2: /*touch power off */
		melfas_power_on(ts, false);
		break;
	case 3:
		melfas_power_on(ts, false);
		msleep(50);
		melfas_power_on(ts, true);
		break;
	default:
		printk(KERN_INFO "usage: echo [1|2|3] > control\n");
		printk(KERN_INFO "  - 1: power on\n");
		printk(KERN_INFO "  - 2: power off\n");
		printk(KERN_INFO "  - 3: power reset\n");
		break;
	}
	return count;
}

static ssize_t
mms136_irq_control_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	int cmd, ret;

	if (sscanf(buf, "%d", &cmd) != 1)
		return -EINVAL;

	switch (cmd) {
	case 1: /* interrupt pin high */
		ret = gpio_direction_input(ts->pdata->i2c_int_gpio);
		if (ret < 0) {
			printk(KERN_ERR "%s: gpio input direction fail\n", __FUNCTION__);
			break;
		}
		gpio_set_value(ts->pdata->i2c_int_gpio, 1);
		printk(KERN_INFO "MMS-136 INTR GPIO pin high\n");
		break;
	case 2: /* interrupt pin LOW */
		ret = gpio_direction_input(ts->pdata->i2c_int_gpio);
		if (ret < 0) {
			printk(KERN_ERR "%s: gpio input direction fail\n", __FUNCTION__);
			break;
		}
		gpio_set_value(ts->pdata->i2c_int_gpio, 0);
		printk(KERN_INFO "MMS-136 INTR GPIO pin low\n");
		break;
	default:
		printk(KERN_INFO "usage: echo [1|2|3|4] > control\n");
		printk(KERN_INFO "  - 1: interrupt pin high\n");
		printk(KERN_INFO "  - 2: interrupt pin low\n");
		break;
	}
	return count;
}

static ssize_t
mms136_reg_control_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	int cmd, ret, reg_addr, length, i;
	uint8_t reg_buf[TS_READ_REGS_LEN];
	if (sscanf(buf, "%d, 0x%x, %d", &cmd, &reg_addr, &length) != 3)
		return -EINVAL;
	switch (cmd) {
	case 1:
		reg_buf[0] = reg_addr;
		ret = i2c_master_send(ts->client, reg_buf, 1);
		if (ret < 0) {
			printk(KERN_ERR "i2c master send fail\n");
			break;
		}
		ret = i2c_master_recv(ts->client, reg_buf, length);
		if (ret < 0) {
			printk(KERN_ERR "i2c master recv fail\n");
			break;
		}
		for (i = 0; i < length; i++) {
			printk(KERN_INFO "0x%x", reg_buf[i]);
		}
		printk(KERN_INFO "\n 0x%x register read done\n", reg_addr);
		break;
	case 2:
		reg_buf[0] = reg_addr;
		reg_buf[1] = length;
		ret = i2c_master_send(ts->client, reg_buf, 2);
		if (ret < 0) {
			printk(KERN_ERR "i2c master send fail\n");
			break;
		}
		printk(KERN_INFO "\n 0x%x register write done\n", reg_addr);
		break;
	default:
		printk(KERN_INFO "usage: echo [1(read)|2(write)], [reg address], [length|value] > reg_control\n");
		printk(KERN_INFO "  - Register Set or Read\n");
		break;
	}
	return count;
}

static ssize_t
mms136_cmdelta_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	char c = '-';
	int len, i, j, t;
	uint8_t write_buf[5];
	uint8_t read_buf[10];
	uint8_t read_size = 0;
	uint16_t cmdata = 0;
	int flag = 0;

	if (irq_flag == 0) {
		printk("disable_irq_nosync\n");
		disable_irq_nosync(ts->client->irq);
		irq_flag = 1;
	}
	write_buf[0] = UNIVERSAL_CMD;
	write_buf[1] = UNIVCMD_ENTER_TEST_MODE;
	i2c_master_send(ts->client, write_buf, 2);

	while (gpio_get_value(ts->pdata->i2c_int_gpio)) {
		flag++;
		if (flag == 30) {
			flag = 0;
			break;
		}
		msleep(100);
	}
	flag = 0;

	write_buf[0] = UNIVERSAL_CMD_RESULT_SIZE;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, read_buf, 1);
	printk("TEST MODE ENTER =%d \n", read_buf[0]);

	write_buf[0] = UNIVERSAL_CMD;
	write_buf[1] = UNIVCMD_TEST_CM_DELTA;
	i2c_master_send(ts->client, write_buf, 2);

	while (gpio_get_value(ts->pdata->i2c_int_gpio)) {
		flag++;
		if (flag == 30) {
			flag = 0;
			break;
		}
		msleep(100);
	}
	flag = 0;

	write_buf[0] = UNIVERSAL_CMD_RESULT_SIZE;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, read_buf, 1);
	printk("CM DELTA TEST =%d \n", read_buf[0]);

	len = snprintf(buf, PAGE_SIZE, "Touch Firmware Version is %d\n", ts->version);
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "%5c", c);
	for (j = 0; j < num_tx_line; j++)
		len += snprintf(buf + len, PAGE_SIZE - len, "%5d", j);
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
	len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
	len += snprintf(buf + len, PAGE_SIZE - len, "------------------------\n");

	/* read touch screen cmdelta */
	for (i = 0; i < num_rx_line ; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "%2d : ", i);
		for (j = 0; j < num_tx_line; j++) {
			write_buf[0] = UNIVERSAL_CMD;
			write_buf[1] = UNIVCMD_GET_PIXEL_CM_DELTA;
			write_buf[2] = j;
			write_buf[3] = i;
			i2c_master_send(ts->client, write_buf, 4);

			while (gpio_get_value(ts->pdata->i2c_int_gpio)) {
				flag++;
				if (flag == 100) {
					flag = 0;
					break;
				}
				udelay(100);
			}

			write_buf[0] = UNIVERSAL_CMD_RESULT_SIZE;
			i2c_master_send(ts->client, write_buf, 1);
			i2c_master_recv(ts->client, &read_size, 1);

			write_buf[0] = UNIVERSAL_CMD_RESULT;
			i2c_master_send(ts->client, write_buf, 1);
			i2c_master_recv(ts->client, read_buf, read_size);

			cmdata = read_buf[1];
			cmdata = ((cmdata << 8) | read_buf[0]);
			len += snprintf(buf + len, PAGE_SIZE - len, "%5d", cmdata);
		}
		len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	}

	/* read touch key cmdelta */
	len += snprintf(buf + len, PAGE_SIZE - len, "key: ");
	for (t = 0; t < MELFAS_MAX_BTN; t++) //Model Dependent
	{
		write_buf[0] = UNIVERSAL_CMD;
		write_buf[1] = 0x4A;
		write_buf[2] = t; //KEY CH.
		write_buf[3] = 0; //Dummy Info
		i2c_master_send(ts->client, write_buf, 4);

		while (gpio_get_value(ts->pdata->i2c_int_gpio)) {
			flag++;
			if (flag == 100) {
				flag = 0;
				break;
			}
			udelay(100);
		}

		write_buf[0] = UNIVERSAL_CMD_RESULT_SIZE;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, &read_size, 1);

		write_buf[0] = UNIVERSAL_CMD_RESULT;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, read_buf, read_size);

		cmdata = read_buf[1];
		cmdata = ((cmdata << 8) | read_buf[0]);
		len += snprintf(buf + len, PAGE_SIZE - len, "%5d", cmdata);
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "\n===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");

	write_buf[0] = UNIVERSAL_CMD;
	write_buf[1] = UNIVERSAL_CMD_EXIT;

	i2c_master_send(ts->client, write_buf, 2);

	if (irq_flag == 1) {
		printk("enable_irq\n");
		enable_irq(ts->client->irq);
		irq_flag = 0;
	}
	return len;
}

static ssize_t
mms136_key_intensity_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	char c = '-';
	int len, i, j;
	uint8_t write_buf[10];
	uint8_t read_buf[25];
	int8_t cmdata = 0;

	if (irq_flag == 0) {
		printk("disable_irq_nosync\n");
		disable_irq_nosync(ts->client->irq);
		irq_flag = 1;
	}

	len = snprintf(buf, PAGE_SIZE, "Touch Firmware Version is %d\n", ts->version);
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "%5c", c);
	for (j = 0; j < num_tx_line+1; j++)
		len += snprintf(buf + len, PAGE_SIZE - len, "%5d", j);
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
	len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
	len += snprintf(buf + len, PAGE_SIZE - len, "------------------------\n");

	for (i = 0; i < num_rx_line; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "%2d : ", i);
		j = 0;
		write_buf[0] = 0xB0;
		write_buf[1] = 0x1A;
		write_buf[2] = j;  /*Exciting CH.*/
		write_buf[3] = i;  /*Sensing CH.*/
		write_buf[4] = 0; /*Reserved*/
		write_buf[5] = 0x08; /*Flag*/
		i2c_master_send(ts->client, write_buf, 6);

		write_buf[0] = 0xBF;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, read_buf, num_tx_line+1);

		for (j = 0; j < ts->pdata->num_of_finger; j++) {
			cmdata = (int8_t) read_buf[j];
			len += snprintf(buf + len, PAGE_SIZE - len, "%5d", cmdata);
		}
		len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");

	if (irq_flag == 1) {
		printk("enable_irq\n");
		enable_irq(ts->client->irq);
		irq_flag = 0;
	}
	return len;
}


static ssize_t
mms136_intensity_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	char c = '-';
	int len, i, j;
	uint8_t write_buf[10];
	uint8_t read_buf[25];
	int8_t cmdata = 0;

	if (irq_flag == 0) {
		printk("disable_irq_nosync\n");
		disable_irq_nosync(ts->client->irq);
		irq_flag = 1;
	}

	len = snprintf(buf, PAGE_SIZE, "Touch Firmware Version is %d\n", ts->version);
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "%5c", c);
	for (j = 0; j < num_tx_line+1; j++)
		len += snprintf(buf + len, PAGE_SIZE - len, "%5d", j);
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
	len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
	len += snprintf(buf + len, PAGE_SIZE - len, "------------------------\n");

	for (i = 0; i < num_rx_line; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "%2d : ", i);
		j = 0;
		write_buf[0] = 0xB0;
		write_buf[1] = 0x1A;
		write_buf[2] = j;  /*Exciting CH.*/
		write_buf[3] = i;  /*Sensing CH.*/
		write_buf[4] = 0; /*Reserved*/
		write_buf[5] = 0x04; /*Flag*/
		i2c_master_send(ts->client, write_buf, 6);

		write_buf[0] = 0xBF;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, read_buf, num_tx_line+1);

		for (j = 0; j < num_tx_line+1; j++) {
			cmdata = (int8_t) read_buf[j];
			len += snprintf(buf + len, PAGE_SIZE - len, "%5d", cmdata);
		}
		len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");

	if (irq_flag == 1) {
		printk("enable_irq\n");
		enable_irq(ts->client->irq);
		irq_flag = 0;
	}
	return len;
}

static ssize_t
mms136_all_version_show(struct device *dev, struct device_attribute *attr,
char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	int ret, len;
	uint8_t version_buf[6];

	version_buf[0] = TS_READ_VERSION_ADDR;
	ret = i2c_master_send(ts->client, &version_buf[0], 1);
	ret = i2c_master_recv(ts->client, &version_buf[0], TS_READ_VERSION_INFO_LEN);

	len = snprintf(buf, PAGE_SIZE, "Melfas Version Info\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "============================\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "Firmware Version : %d\n", version_buf[0]);
	len += snprintf(buf + len, PAGE_SIZE - len, "Hardware Version : %d\n", version_buf[1]);
	len += snprintf(buf + len, PAGE_SIZE - len, "Compatibility Group : %c\n", version_buf[2]);
	len += snprintf(buf + len, PAGE_SIZE - len, "Core Firmware Version : %d\n", version_buf[3]);
	len += snprintf(buf + len, PAGE_SIZE - len, "Private Custom Version : %d\n", version_buf[4]);
	len += snprintf(buf + len, PAGE_SIZE - len, "Public Custom Version : %d\n", version_buf[5]);
	len += snprintf(buf + len, PAGE_SIZE - len, "============================\n");

	return len;
}

static ssize_t
set_resolution_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	int cmd, ret = 0, i;

	if (sscanf(buf, "%d", &cmd) != 1)
		return -EINVAL;

	if (cmd > 0) {
		printk("[TOUCH] Reinitialize Touch input device\n");
		input_unregister_device(ts->input_dev);
		ts->input_dev = input_allocate_device();

		if (!ts->input_dev) {
			printk("[TOUCH] Not enough memory\n");
			return ret;
		}

		ts->input_dev->name = "melfas-ts";

		ts->input_dev->evbit[0] = BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);

		for (i = 0; i < ts->pdata->num_of_button; i++)
			ts->input_dev->keybit[BIT_WORD(ts->pdata->button[i])] |= BIT_MASK(ts->pdata->button[i]);

		input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0,  ts->pdata->x_max * cmd, 0, 0);
		input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0,  ts->pdata->y_max * cmd, 0, 0);
		input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 40, 0, 0);
		input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
		input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, MELFAS_MAX_TOUCH - 1, 0, 0);

		ret = input_register_device(ts->input_dev);
		if (ret) {
			MELFAS_TS_DPRINTK(MELFAS_TS_DEBUG_PROBE, KERN_ERR,
					  "[TOUCH] Failed to register device\n");
			return ret;
		}
	} else {
		printk(KERN_INFO "usage: echo [1|2|3|4] > resolution\n");
		printk(KERN_INFO "  - 1: X 1\n");
		printk(KERN_INFO "  - 2: X 2\n");
	}
	return count;
}

static struct device_attribute mms136_device_attrs[] = {
	__ATTR(status,  S_IRUGO | S_IWUSR, mms136_status_show, NULL),
	__ATTR(version, S_IRUGO | S_IWUSR, mms136_version_show, NULL),
	__ATTR(fw_upgrade, S_IRUGO | S_IWUSR, NULL, mms136_fw_upgrade_store),
	__ATTR(power_control, S_IRUGO | S_IWUSR, NULL, mms136_power_control_store),
	__ATTR(irq_control, S_IRUGO | S_IWUSR, NULL, mms136_irq_control_store),
	__ATTR(reg_control, S_IRUGO | S_IWUSR, NULL, mms136_reg_control_store),
	__ATTR(cmdelta, S_IRUGO | S_IWUSR, mms136_cmdelta_show, NULL),
	__ATTR(intensity, S_IRUGO | S_IWUSR, mms136_intensity_show, NULL),
	__ATTR(key_intensity, S_IRUGO | S_IWUSR, mms136_key_intensity_show, NULL),
	__ATTR(all_version, S_IRUGO | S_IWUSR, mms136_all_version_show, NULL),
	__ATTR(resolution, S_IRUGO | S_IWUSR, NULL, set_resolution_store),
};

static int melfas_power_on(struct melfas_ts_data *ts, bool on)
{
	int retval = 0;

	pr_info("##@%s pwr(%d)\n", __func__, on);

	if (on == false)
		goto power_off;

	if (ts->poweron)
		return 0;


	if(ts->vdd) {
		retval = regulator_enable(ts->vdd);
		//printk(KERN_ERR "%d [TOUCH] On : regulator_enable(VDD) = %d \n", __LINE__, retval);
		if (retval < 0) {
			dev_err(&ts->client->dev, "[TOUCH] Regulator vdd enable failed retval = %d\n", retval);
		}
	}

	if(ts->vcc_i2c) {
		retval = regulator_enable(ts->vcc_i2c);
		//printk(KERN_ERR "%d [TOUCH] On : regulator_enable(I2C) = %d \n", __LINE__, retval);
		if (retval < 0) {
			dev_err(&ts->client->dev, "[TOUCH] Regulator i2c enable failed retval = %d\n", retval);
		}
	}

#if defined(CONFIG_MACH_PDA)
/*                                                                                            
                         
                         
                          
                                                                                             */
	if(lge_get_board_revno() >= HW_REV_PDA_C){
		retval = gpio_direction_output(ts->pdata->touch_en_gpio, 1);
		if (retval < 0) {
			dev_err(&ts->client->dev,
				"[TOUCH] unable to set direction for gpio [%d]\n",ts->pdata->touch_en_gpio);
		}
	}
#endif
	ts->poweron = 1;

	goto exit;


power_off :

	if (ts->poweron == 0)
		return 0;

	if(ts->vdd) {
		regulator_disable(ts->vdd);
		//dev_err(&ts->client->dev, "[TOUCH] regulator_disable(VDD) \n");
	}

	if(ts->vcc_i2c) {
		regulator_disable(ts->vcc_i2c);
		//dev_err(&ts->client->dev, "[TOUCH] regulator_disable(I2C) \n");
	}
#if defined(CONFIG_MACH_PDA)
/*                                                                                            
                         
                         
                          
                                                                                             */
	if(lge_get_board_revno() >= HW_REV_PDA_C){
		retval = gpio_direction_output(ts->pdata->touch_en_gpio,0);
		if (retval < 0) {
			dev_err(&ts->client->dev, "[TOUCH] Failed to gpio out\n");
		}
	}
#endif

	ts->poweron = 0;

exit :
	printk(KERN_ERR "[TOUCH] power delay 30ms \n");
	msleep(30);

	return retval;
}

int melfas_poweron(char on)
{
	return melfas_power_on(gts, (bool)on);
}
EXPORT_SYMBOL(melfas_poweron);

static int melfas_regulator_configure(struct melfas_ts_data *ts, bool on)
{
	int retval = 0;

	if (on == false)
		goto hw_shutdown;

	if(ts->vdd == NULL) {
		ts->vdd = regulator_get(&ts->client->dev, "vdd_touch");
		if (IS_ERR(ts->vdd)) {
			dev_err(&ts->client->dev, "[TOUCH] %s: Failed to get vdd_touch regulator\n", __func__);
			return PTR_ERR(ts->vdd);
		}
		dev_err(&ts->client->dev, "[TOUCH] regulator_get(VDD) \n");
	}
#if defined(CONFIG_MACH_PDA)
	if(ts->vcc_i2c == NULL) {
		ts->vcc_i2c = regulator_get(&ts->client->dev, "vdd_dis_ts");
		if (IS_ERR(ts->vcc_i2c)) {
			dev_err(&ts->client->dev, "[TOUCH] %s: Failed to get vdd_dis_ts regulator\n", __func__);
			return PTR_ERR(ts->vcc_i2c);
		}
		//dev_err(&ts->client->dev, "[TOUCH] regulator_get(VDD) \n");
	}
#else
	ts->vcc_i2c = NULL;
#endif

	if(ts->vdd) {
		retval = regulator_set_voltage(ts->vdd, TOUCH_VDD_VTG_MIN_UV, TOUCH_VDD_VTG_MAX_UV);
		if (retval)
			dev_err(&ts->client->dev, "[TOUCH] regulator_set_voltage(VDD) failed retval=%d\n", retval);
		else
			dev_err(&ts->client->dev, "[TOUCH] regulator_set_voltage(VDD) \n");
	}

	if(ts->vcc_i2c) {
		retval = regulator_set_voltage(ts->vcc_i2c, TOUCH_I2C_VTG_MIN_UV, TOUCH_I2C_VTG_MAX_UV);
		if (retval)
			dev_err(&ts->client->dev, "[TOUCH] regulator_set_voltage(I2C) failed retval=%d\n", retval);
		else
			dev_err(&ts->client->dev, "[TOUCH] regulator_set_voltage(I2C) \n");
	}

	return 0;

hw_shutdown :
	if(ts->vdd) {
		regulator_put(ts->vdd);
		dev_err(&ts->client->dev, "[TOUCH] regulator_put(VDD) \n");
	}

	if(ts->vcc_i2c) {
		regulator_put(ts->vcc_i2c);
		dev_err(&ts->client->dev, "[TOUCH] regulator_put(I2C) \n");
	}
	return retval;
}

static int melfas_parse_dt(struct device *dev, struct melfas_tsi_platform_data *melfas_pdata)
{
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 temp_val, num_buttons;
	u32 button_map[MAX_NUM_OF_BUTTON];
	int rc, i;

	dev_info(dev, "[TOUCH] %s\n", __func__);

	melfas_pdata->i2c_pull_up = of_property_read_bool(np, "melfas,i2c-pull-up");

	// x,y cordination
	rc = of_property_read_u32(np, "melfas,panel-x", &temp_val);
	if (rc && (rc != -EINVAL)) {
		return rc;
	} else {
		melfas_pdata->x_max = temp_val;
	}
	rc = of_property_read_u32(np, "melfas,panel-y", &temp_val);
	if (rc && (rc != -EINVAL)) {
		return rc;
	} else {
		melfas_pdata->y_max = temp_val;
	}

	rc = of_property_read_u32(np, "melfas,num-of-finger", &temp_val);
	if (rc && (rc != -EINVAL)) {
		return rc;
	} else {
		if(temp_val > MELFAS_MAX_TOUCH)
			temp_val = MELFAS_MAX_TOUCH;
		melfas_pdata->num_of_finger= (unsigned char) temp_val;
	}

	melfas_pdata->i2c_int_gpio = of_get_named_gpio_flags(np, "melfas,i2c_int_gpio", 0, &melfas_pdata->irq_flag); //irq_flag active low or high

	prop = of_find_property(np, "melfas,button-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		printk("[TOUCH] %s : num_buttons = %d\n", __func__, num_buttons);
		if (num_buttons < MAX_NUM_OF_BUTTON) {
			rc = of_property_read_u32_array(np, "melfas,button-map", button_map, num_buttons);
			if (rc) {
				dev_err(dev, "[TOUCH] Unable to read key codes\n");
				return rc;
			}
			for(i=0; i<num_buttons; i++) {
				melfas_pdata->button[i] = (unsigned short) button_map[i];
				printk("[TOUCH] %s : button[%d] = %d, button_map[%d] = %d\n", __func__, i, melfas_pdata->button[i], i, button_map[i]);
			}
			melfas_pdata->num_of_button = (unsigned char)num_buttons;
		}
	}

	return 0;

}

static int melfas_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct melfas_ts_data *ts=NULL;
	struct melfas_tsi_platform_data *platform_data=NULL;
	int ret = 0, i;

	uint8_t buf[TS_READ_VERSION_INFO_LEN] = {0};
	irq_flag = 0;

	printk("##@%s\n", __func__);
	dev_info(&client->dev, "[TOUCH] %s Start!!!\n", __func__);

	for(i=0; i<20; i++)
		init_values[i] = 1;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "[TOUCH] %s : failed to i2c functionality check\n", __func__);
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct melfas_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		dev_err(&client->dev, "[TOUCH] failed to allocate melfas_ts_data\n");
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	if(client->dev.of_node) {
		dev_dbg(&client->dev, "[TOUCH] %s : of node exist \n", __func__);
		platform_data = devm_kzalloc(&client->dev, sizeof(*platform_data), GFP_KERNEL);
		if(!platform_data) {
			dev_err(&client->dev, "[TOUCH] %s : Fail to allocate memory\n", __func__);
			return -ENOMEM;
		}

		ret = melfas_parse_dt(&client->dev, platform_data);
		if(ret) {
			dev_err(&client->dev, "[TOUCH] %s : device tree parsing failed on melfas_parse_dt()\n", __func__);
			return ret;
		}
	} else {
		platform_data = client->dev.platform_data;
	}

	ts->pdata = platform_data;
	ts->client = client;
	i2c_set_clientdata(client, ts);
	gts = ts;

	if(lge_get_board_revno() >= HW_REV_PDA_C){
		/* gpio init : chip enable */
		ret = gpio_request(ts->pdata->touch_en_gpio, "melfas_en");
		if (ret < 0){
			pr_err("%s not able to get gpio\n", __func__);
			return ret;
		}
		ret = gpio_direction_output(ts->pdata->touch_en_gpio, 0x0);
		if (ret < 0){
			pr_err("%s not able to get gpio\n", __func__);
			return ret;
		}
	}

	/* gpio init : maker_id */
	ret = gpio_request(ts->pdata->touch_id_gpio, "TOUCH_PANEL_MAKERID");
	if (unlikely(ret < 0))
		pr_err("%s not able to get MAKER ID gpio\n", __func__);
	gpio_direction_input(ts->pdata->touch_id_gpio);


	ret = melfas_regulator_configure(ts, true);
	if (ret < 0) {
		dev_err(&client->dev, "[TOUCH] Failed to configure regulators\n");
		goto err_reg_configure;
	}

	dev_err(&ts->client->dev, "[TOUCH] Power Control : probe %d\n", __LINE__);
	ret = melfas_power_on(ts, true);
	if (ret < 0) {
		dev_err(&client->dev, "[TOUCH] Failed to power on\n");
		goto err_power_device;
	}

#if 0
	if (gpio_is_valid(platform_data->touch_id_gpio)) {
		/* configure touchscreen irq gpio */
		ret = gpio_request(platform_data->touch_id_gpio, "mms136_touch_id_gpio");
		if (ret) {
			dev_err(&client->dev, "unable to request gpio [%d]\n",
						platform_data->touch_id_gpio);
			goto err_irq_gpio_req;
		}
		ret = gpio_direction_input(platform_data->touch_id_gpio);
		if (ret) {
			dev_err(&client->dev,
				"unable to set direction for gpio [%d]\n",
				platform_data->touch_id_gpio);
		}
	} else {
		dev_err(&client->dev, "irq gpio not provided\n");
	}

	ret = gpio_get_value(platform_data->touch_id_gpio);
	printk("%s : gpio_get_value = %d\n", __func__, ret);
	if(ret == 0) {
		platform_data->max_x = 320;
		platform_data->max_y = 480;
	}
#endif

#if 0
	buf[0] = TS_READ_VERSION_ADDR;

	for (i = 0; i < I2C_RETRY_CNT; i++) {
		ret = i2c_master_send(ts->client, &buf[0], 1);
		if (ret >= 0) {
			printk(KERN_ERR "[TOUCH] i2c_master_send() ok [%d]\n", ret);
			break;
		} else {
			MELFAS_TS_DPRINTK(MELFAS_TS_DEBUG_PROBE, KERN_ERR, "[TOUCH] i2c_master_send() failed[%d]\n", ret);
			if (i == I2C_RETRY_CNT-1) {
				ret = mms100_download(check_type, embedded_img); /*check FW status */
				if (ret == 0) {
					ret = mms100_download(isp_type, embedded_img);
					printk(KERN_ERR "[TOUCH] touch fw update success \n");
				}
				else {
					MELFAS_TS_DPRINTK(MELFAS_TS_DEBUG_PROBE, KERN_ERR, "[TOUCH] no touch panel \n");

					//return ret ;
				}
			}
		}
	}
#endif

	buf[0] = TS_READ_VERSION_ADDR;
	ret = i2c_master_send(ts->client, &buf[0], 1);
	ret = i2c_master_recv(ts->client, &buf[0], TS_READ_VERSION_INFO_LEN);

	printk(KERN_ERR "[TOUCH] = Melfas Version Info =\n");
	printk(KERN_ERR "[TOUCH] Bootloader Version :: %d Core Version :: (%X), Config Version :: %d\n", buf[0], buf[1], buf[2]);


	ts->input_dev = input_allocate_device();
	if (!ts->input_dev) {
		dev_err(&client->dev, "[TOUCH] %s : Not enough memory, input allocate device Failed\n", __func__);
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = MELFAS_DRIVER_NAME;
	ts->input_dev->evbit[0] = BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);

	for (i = 0; i < ts->pdata->num_of_button; i++) {
		ts->input_dev->keybit[BIT_WORD(ts->pdata->button[i])] |= BIT_MASK(ts->pdata->button[i]);
	}
	/*           
                                                                             
                                    */
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0,  ts->pdata->x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0,  ts->pdata->y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 40, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, MELFAS_MAX_TOUCH - 1, 0, 0);

	ret = input_register_device(ts->input_dev);
	if (ret) {
		dev_err(&client->dev,
			"[TOUCH] %s : Failed to register input device\n", __func__);
		ret = -ENOMEM;
		goto err_input_register_device_failed;
	}

/*           
                                                
                                   */
#if defined(CONFIG_FB)
	ts->fb_notif.notifier_call = fb_notifier_callback;
	if((ret= fb_register_client(&ts->fb_notif)))
		dev_err(&client->dev, "[TOUCH] %s: Unable to register fb_notifier: %d\n", __func__, ret);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
		ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
		ts->early_suspend.suspend = melfas_ts_early_suspend;
		ts->early_suspend.resume = melfas_ts_late_resume;
		register_early_suspend(&ts->early_suspend);
#endif


	if (ts->client->irq) {
		dev_dbg(&client->dev, "[TOUCH] %s : trying to request irq: %s-%d\n", 
					__func__, ts->client->name, ts->client->irq);
		ret = request_threaded_irq(client->irq, NULL, melfas_ts_irq_handler, IRQF_ONESHOT | IRQF_TRIGGER_LOW, ts->client->name, ts);
		/* due to comming from touch irq should make irq be disable */
		disable_irq(client->irq);

		if (ret > 0) {
			dev_err(&client->dev,
				"[TOUCH] %s : Can't allocate irq %d, ret %d\n", __func__, ts->client->irq, ret);
			ret = -EBUSY;
			goto err_request_irq;
		}
	}

	for (i = 0; i < ARRAY_SIZE(mms136_device_attrs); i++) {
		ret = device_create_file(&client->dev, &mms136_device_attrs[i]);
		if (ret) {
			goto err_request_irq;
		}
	}

	for (i = 0; i < MELFAS_MAX_TOUCH; i++) {
		g_Mtouch_info[i].btn_touch = -1;
		tmp_flag[i] = 1;
	}

	dev_dbg(&client->dev, "[TOUCH] %s : start touch name: %s, irq: %d\n",
				__func__, ts->client->name, ts->client->irq);
	return 0;

err_request_irq:
	printk(KERN_ERR "[TOUCH] : err_request_irq failed\n");
	free_irq(client->irq, ts);
err_input_register_device_failed:
	printk(KERN_ERR "[TOUCH] : err_input_register_device failed\n");
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
	printk(KERN_ERR "[TOUCH] : err_input_dev_alloc failed\n");
err_alloc_data_failed:
	printk(KERN_ERR "[TOUCH] : err_alloc_data failed_\n");
err_check_functionality_failed:
	printk(KERN_ERR "[TOUCH] : err_check_functionality failed_\n");
err_power_device:
	melfas_regulator_configure(ts, false);
err_reg_configure:
	input_free_device(ts->input_dev);
	ts->input_dev = NULL;

	return ret;
}

static int melfas_ts_remove(struct i2c_client *client)
{
	struct melfas_ts_data *ts = i2c_get_clientdata(client);
/*           
                                                
                                   */
#if defined(CONFIG_FB)
	if (fb_unregister_client(&ts->fb_notif))
		dev_err(&(ts->input_dev->dev),	"[TOUCH] %s: Error occurred while unregistering fb_notifier.\n", __func__);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#endif
	free_irq(client->irq, ts);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static void release_all_fingers(struct melfas_ts_data *ts)
{
	int i;
	for (i = 0; i < ts->pdata->num_of_finger; i++) {
		if (g_Mtouch_info[i].btn_touch < 0)
			continue;

		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, g_Mtouch_info[i].posX);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, g_Mtouch_info[i].posY);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, g_Mtouch_info[i].width);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
		input_mt_sync(ts->input_dev);

		g_Mtouch_info[i].posX = 0;
		g_Mtouch_info[i].posY = 0;
		g_Mtouch_info[i].strength = 0;
		g_Mtouch_info[i].btn_touch = -1;
		tmp_flag[i] = 1;
	}
	input_sync(ts->input_dev);
	touch_prestate = 0;
}

static void release_all_keys(struct melfas_ts_data *ts)
{
	int i;

	for (i = 0; i < MELFAS_MAX_BTN; i++) {
		dev_dbg(&ts->client->dev, "%s : g_btn_info status : %d, key_code : %d\n", __func__, g_btn_info[i].status , g_btn_info[i].key_code);
		if (g_btn_info[i].status <= 0)
			continue;
		input_report_key(ts->input_dev, g_btn_info[i].key_code, RELEASE_KEY);
	}
	btn_prestate = RELEASE_KEY;
	input_sync(ts->input_dev);
}

static void melfas_ts_sw_reset(struct melfas_ts_data *ts)
{
	dev_dbg(&ts->client->dev, "%s\n", __func__);

	dev_err(&ts->client->dev, "[TOUCH] Power Control : reset %d\n", __LINE__);
	release_all_fingers(ts);
	release_all_keys(ts);
	melfas_power_on(ts, false);
	melfas_power_on(ts, true);
}

/*           
                                                
                                   */
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct melfas_ts_data *ts = container_of(self, struct melfas_ts_data, fb_notif);
	if(evdata && evdata->data && event == FB_EVENT_BLANK && ts && ts->client) {
		blank = evdata->data;
		if(*blank == FB_BLANK_UNBLANK) {
			melfas_ts_resume(ts);
			dev_dbg(&(ts->input_dev->dev), "[TOUCH] %s: Resume\n", __func__);
		} else if (*blank == FB_BLANK_POWERDOWN) {
			melfas_ts_suspend(ts);
			dev_dbg(&(ts->input_dev->dev), "[TOUCH] %s: Suspend\n", __func__);
		}
	}
	return 0;
}

#elif defined (CONFIG_HAS_EARLYSUSPEND)
static void melfas_ts_early_suspend(struct early_suspend *h)
{
	struct melfas_ts_data *ts;
	ts = container_of(h, struct melfas_ts_data, early_suspend);
	melfas_ts_suspend_func(ts);
}

static void melfas_ts_late_resume(struct early_suspend *h)
{
	struct melfas_ts_data *ts;
	ts = container_of(h, struct melfas_ts_data, early_suspend);
	melfas_ts_resume_func(ts);
}
#endif

static void melfas_ts_suspend(struct melfas_ts_data *ts)
{
	int ret=0;

	if(ts->fw_update == 1)
		return;

	disable_irq(ts->client->irq);
	ret = melfas_power_on(ts, false);
	if (ret < 0)
		printk(KERN_ERR "[TOUCH] Failed to power on\n");
}

static void melfas_ts_resume(struct melfas_ts_data *ts)
{
	int ret=0;

	dev_err(&ts->client->dev, "[TOUCH] Power Control : resume %d\n", __LINE__);

	if(ts->fw_update == 1)
		return;

	ret = melfas_power_on(ts, true);
	if (ret < 0)
		printk(KERN_ERR "[TOUCH] Failed to power on\n");
	enable_irq(ts->client->irq);
}

#if (!defined(CONFIG_HAS_EARLYSUSPEND) && !defined(CONFIG_FB))
static void melfas_ts_suspend_func(struct melfas_ts_data *ts)
{
	int ret = 0;

	dev_dbg(&ts->client->dev, "[TOUCH] %s\n", __func__);

	ret = melfas_power_on(ts, false);
	if (ret < 0)
		dev_err(&ts->client->dev, "[TOUCH] %s : touch suspend failed\n", __func__);

	/* move release timing */
	release_all_fingers(ts);
	release_all_keys(ts);
}

static void melfas_ts_resume_func(struct melfas_ts_data *ts)
{
	int ret = 0;

	dev_dbg(&ts->client->dev, "[TOUCH] %s\n", __func__);

	ret = melfas_power_on(ts, true);
	if (ret < 0)
		dev_err(&ts->client->dev, "[TOUCH] %s : touch resume failed\n", __func__);
}
#endif

static const struct i2c_device_id melfas_ts_id[] = {
	{ MELFAS_DRIVER_NAME, 0 },
	{ }
};

#ifdef CONFIG_OF
static struct of_device_id melfas_match_table[] = {
	{ .compatible = "melfas,mms136",},
	{},
};
#else
#define melfas_match_table NULL
#endif

static struct i2c_driver melfas_ts_driver = {
	.driver		= {
		.name	= MELFAS_DRIVER_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = melfas_match_table,
	},
	.id_table		= melfas_ts_id,
	.probe		= melfas_ts_probe,
	.remove		= __devexit_p (melfas_ts_remove),
#if (!defined(CONFIG_HAS_EARLYSUSPEND) && !defined(CONFIG_FB))
	.suspend		= melfas_ts_suspend,
	.resume		= melfas_ts_resume,
#endif
};

static int __devinit melfas_ts_init(void)
{
	return i2c_add_driver(&melfas_ts_driver);
}

static void __exit melfas_ts_exit(void)
{
	i2c_del_driver(&melfas_ts_driver);
}

MODULE_DESCRIPTION("Driver for Melfas MTSI Touchscreen Controller");
MODULE_AUTHOR("MinSang, Kim <kimms@melfas.com>");
MODULE_LICENSE("GPL");

module_init(melfas_ts_init);
module_exit(melfas_ts_exit);
