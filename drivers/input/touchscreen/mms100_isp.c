
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/i2c/melfas_ts.h>

#include "X5_GFF_V06_bin.c"

enum {
	ISP_MODE_FLASH_ERASE	= 0x59F3,
	ISP_MODE_FLASH_WRITE	= 0x62CD,
	ISP_MODE_FLASH_READ	= 0x6AC9,
};

/* each address addresses 4-byte words */
#define ISP_MAX_FW_SIZE		(0x1F00 * 4)
#define ISP_IC_INFO_ADDR	0x1F00

#define ISP_CAL_INFO_ADDR	7936
#define ISP_CAL_DATA_SIZE	256

struct mms_info {
	struct i2c_client		*client;
	struct melfas_tsi_platform_data	*pdata;
};

#define GPIO_TOUCH_INT 105
#define GPIO_TSP_SCL 93
#define GPIO_TSP_SDA 94

extern int melfas_poweron(char on);
extern int mms100_set_gpio_mode(char gpio);

int mms100_set_gpio_mode(char gpio)
{
	int ret;

	if(gpio == 1)
	{
		printk("<MELFAS> mms100_set_gpio_mode(GPIO) \n");

		ret = gpio_request(GPIO_TSP_SDA, "Melfas_I2C_SDA");
		if (ret) {
			printk(KERN_ERR "gpio_request(Melfas_I2C_SDA) failed \n");
		}
		ret = gpio_request(GPIO_TSP_SCL, "Melfas_I2C_SCL");
		if (ret) {
			printk(KERN_ERR "gpio_request(Melfas_I2C_SCL) failed \n");
		}
		ret = gpio_request(GPIO_TOUCH_INT, "Melfas_I2C_INT");
		if (ret) {
			printk(KERN_ERR "gpio_request(Melfas_I2C_INT) failed \n");
		}
	}
	else
	{
		printk("<MELFAS> mms100_set_gpio_mode(I2C) \n");
		gpio_free(GPIO_TSP_SDA);
		gpio_free(GPIO_TSP_SCL);
		gpio_free(GPIO_TOUCH_INT);
	}

	return ret;
}


static void hw_reboot(struct mms_info *info, bool bootloader)
{
	//gpio_direction_output(info->pdata->gpio_vdd_en, 0);
	melfas_poweron(0);
	gpio_direction_output(GPIO_TSP_SDA, bootloader ? 0 : 1);
	gpio_direction_output(GPIO_TSP_SCL, bootloader ? 0 : 1);
	gpio_direction_output(GPIO_TOUCH_INT, 0);
	msleep(30);
	//gpio_set_value(info->pdata->gpio_vdd_en, 1);
	//msleep(30);
	melfas_poweron(1);

	if (bootloader) {
		gpio_set_value(GPIO_TSP_SCL, 0);
		gpio_set_value(GPIO_TSP_SDA, 1);
	} else {
		gpio_set_value(GPIO_TOUCH_INT, 1);
		gpio_direction_input(GPIO_TOUCH_INT);
		gpio_direction_input(GPIO_TSP_SCL);
		gpio_direction_input(GPIO_TSP_SDA);
	}
	msleep(40);
}

static void isp_toggle_clk(struct mms_info *info, int start_lvl, int end_lvl,
			   int hold_us)
{
	gpio_set_value(GPIO_TSP_SCL, start_lvl);
	udelay(hold_us);
	gpio_set_value(GPIO_TSP_SCL, end_lvl);
	udelay(hold_us);
}

/* 1 <= cnt <= 32 bits to write */
static void isp_send_bits(struct mms_info *info, u32 data, int cnt)
{
	gpio_direction_output(GPIO_TOUCH_INT, 0);
	gpio_direction_output(GPIO_TSP_SCL, 0);
	gpio_direction_output(GPIO_TSP_SDA, 0);

	/* clock out the bits, msb first */
	while (cnt--) {
		gpio_set_value(GPIO_TSP_SDA, (data >> cnt) & 1);
		udelay(3);
		isp_toggle_clk(info, 1, 0, 3);
	}
}

/* 1 <= cnt <= 32 bits to read */
static u32 isp_recv_bits(struct mms_info *info, int cnt)
{
	u32 data = 0;

	gpio_direction_output(GPIO_TOUCH_INT, 0);
	gpio_direction_output(GPIO_TSP_SCL, 0);
	gpio_set_value(GPIO_TSP_SDA, 0);
	gpio_direction_input(GPIO_TSP_SDA);

	/* clock in the bits, msb first */
	while (cnt--) {
		isp_toggle_clk(info, 0, 1, 1);
		data = (data << 1) | (!!gpio_get_value(GPIO_TSP_SDA));
	}

	gpio_direction_output(GPIO_TSP_SDA, 0);
	return data;
}

static void isp_enter_mode(struct mms_info *info, u32 mode)
{
	int cnt;
	unsigned long flags;

	local_irq_save(flags);
	gpio_direction_output(GPIO_TOUCH_INT, 0);
	gpio_direction_output(GPIO_TSP_SCL, 0);
	gpio_direction_output(GPIO_TSP_SDA, 1);

	mode &= 0xffff;
	for (cnt = 15; cnt >= 0; cnt--) {
		gpio_set_value(GPIO_TOUCH_INT, (mode >> cnt) & 1);
		udelay(3);
		isp_toggle_clk(info, 1, 0, 3);
	}

	gpio_set_value(GPIO_TOUCH_INT, 0);
	local_irq_restore(flags);
}

static void isp_exit_mode(struct mms_info *info)
{
	int i;
	unsigned long flags;

	local_irq_save(flags);
	gpio_direction_output(GPIO_TOUCH_INT, 0);
	udelay(3);

	for (i = 0; i < 10; i++)
		isp_toggle_clk(info, 1, 0, 3);
	local_irq_restore(flags);
}

static void flash_set_address(struct mms_info *info, u16 addr)
{
	/* Only 13 bits of addr are valid.
	 * The addr is in bits 13:1 of cmd */
	isp_send_bits(info, (u32)(addr & 0x1fff) << 1, 18);
}

static void flash_erase(struct mms_info *info)
{
	isp_enter_mode(info, ISP_MODE_FLASH_ERASE);

	gpio_direction_output(GPIO_TOUCH_INT, 0);
	gpio_direction_output(GPIO_TSP_SCL, 0);
	gpio_direction_output(GPIO_TSP_SDA, 1);

	/* 4 clock cycles with different timings for the erase to
	 * get processed, clk is already 0 from above */
	udelay(7);
	isp_toggle_clk(info, 1, 0, 3);
	udelay(7);
	isp_toggle_clk(info, 1, 0, 3);
	usleep_range(25000, 35000);
	isp_toggle_clk(info, 1, 0, 3);
	usleep_range(150, 200);
	isp_toggle_clk(info, 1, 0, 3);

	gpio_set_value(GPIO_TSP_SDA, 0);

	isp_exit_mode(info);
}

static u32 flash_readl(struct mms_info *info, u16 addr)
{
	int i;
	u32 val;
	unsigned long flags;

	local_irq_save(flags);
	isp_enter_mode(info, ISP_MODE_FLASH_READ);
	flash_set_address(info, addr);

	gpio_direction_output(GPIO_TSP_SCL, 0);
	gpio_direction_output(GPIO_TSP_SDA, 0);
	udelay(40);

	/* data load cycle */
	for (i = 0; i < 6; i++)
		isp_toggle_clk(info, 1, 0, 10);

	val = isp_recv_bits(info, 32);
	isp_exit_mode(info);
	local_irq_restore(flags);

	return val;
}

static void flash_writel(struct mms_info *info, u16 addr, u32 val)
{
	unsigned long flags;

	local_irq_save(flags);
	isp_enter_mode(info, ISP_MODE_FLASH_WRITE);
	flash_set_address(info, addr);
	isp_send_bits(info, val, 32);

	gpio_direction_output(GPIO_TSP_SDA, 1);
	/* 6 clock cycles with different timings for the data to get written
	 * into flash */
	isp_toggle_clk(info, 0, 1, 3);
	isp_toggle_clk(info, 0, 1, 3);
	isp_toggle_clk(info, 0, 1, 6);
	isp_toggle_clk(info, 0, 1, 12);
	isp_toggle_clk(info, 0, 1, 3);
	isp_toggle_clk(info, 0, 1, 3);

	isp_toggle_clk(info, 1, 0, 1);

	gpio_direction_output(GPIO_TSP_SDA, 0);
	isp_exit_mode(info);
	local_irq_restore(flags);
	usleep_range(300, 400);
}

static int fw_write_image(struct mms_info *info, const u8 *data, size_t len)
{
	struct i2c_client *client = info->client;
	u16 addr = 0;
	int test = 0;
	int testsize = 0;

	for (addr = 0; addr < (len / 4); addr++, data += 4) {
		u32 val = get_unaligned_le32(data);
		u32 verify_val;
		int retries = 3;

		while (retries--) {
			flash_writel(info, addr, val);
			verify_val = flash_readl(info, addr);

			if(testsize <= 40 || testsize >= 31700) {
				if (val == verify_val)
					test = 1;
				else
					test = 0;
				dev_err(&client->dev, "Addr[%04X] Write[%08X] ReadBack[%08X] :%d \n", addr, val, verify_val, test);
			}
			testsize += 4;

			if (val == verify_val)
				break;
			dev_err(&client->dev,
				"mismatch @ addr 0x%x: 0x%x != 0x%x\n",
				addr, verify_val, val);
			hw_reboot(info, true);
			continue;
		}
		if (retries < 0)
			return -ENXIO;
	}

	return 0;
}

static int fw_download(struct mms_info *info, const u8 *data, size_t len)
{
	struct i2c_client *client = info->client;
	u32 val;
	int ret = 0;
	int i;
	int addr = ISP_CAL_INFO_ADDR;
	u32 *buf = kzalloc(ISP_CAL_DATA_SIZE * 4, GFP_KERNEL);
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	if (len % 4) {
		dev_err(&client->dev,
			"fw image size (%d) must be a multiple of 4 bytes\n",
			len);
		ret = -EINVAL;
		goto out;
	} else if (len > ISP_MAX_FW_SIZE) {
		dev_err(&client->dev,
			"fw image is too big, %d > %d\n", len, ISP_MAX_FW_SIZE);
		ret = -EINVAL;
		goto out;
	}

	dev_info(&client->dev, "fw download start\n");

	i2c_lock_adapter(adapter);
	//info->pdata->mux_fw_flash(true);

	//gpio_direction_output(info->pdata->gpio_vdd_en, 0);
	melfas_poweron(0);
	gpio_direction_output(GPIO_TSP_SDA, 0);
	gpio_direction_output(GPIO_TSP_SCL, 0);
	gpio_direction_output(GPIO_TOUCH_INT, 0);

	hw_reboot(info, true);

	dev_info(&client->dev, "calibration data backup\n");
	for (i = 0; i < ISP_CAL_DATA_SIZE; i++) {
		buf[i] = flash_readl(info, addr);
		//dev_info(&client->dev, "cal data : 0x%02x\n", buf[i]);
	}

	val = flash_readl(info, ISP_IC_INFO_ADDR);
	dev_info(&client->dev, "IC info: 0x%02x (%x)\n", val & 0xff, val);

	dev_info(&client->dev, "fw erase...\n");
	flash_erase(info);

	dev_info(&client->dev, "fw write...\n");
	/* XXX: what does this do?! */
	flash_writel(info, ISP_IC_INFO_ADDR, 0xffffff00 | (val & 0xff));
	usleep_range(1000, 1500);
	ret = fw_write_image(info, data, len);
	if (ret)
		goto out;

	dev_info(&client->dev, "fw download done...\n");

	dev_info(&client->dev, "restoring data\n");
	for (i = 0; i < ISP_CAL_DATA_SIZE; i++) {
		flash_writel(info, addr, buf[i]);
	}

	hw_reboot(info, false);
	ret = 0;

out:
	if (ret != 0)
		dev_err(&client->dev, "fw download failed...\n");

	hw_reboot(info, false);
	kfree(buf);

	//info->pdata->mux_fw_flash(false);
	i2c_unlock_adapter(adapter);

	return ret;
}

int mms_flash_fw(struct i2c_client *client, struct melfas_tsi_platform_data *pdata)
{
	int ret;
	const struct firmware *fw;
	const char *fw_name = "mms_ts.bin";
	struct mms_info *info = kzalloc(sizeof(struct mms_info), GFP_KERNEL);

	info->client = client;
	info->pdata = pdata;

	ret = request_firmware(&fw, fw_name, &client->dev);
	if (ret) {
		dev_err(&client->dev, "error requesting firmware\n");
		goto out;
	}

	ret = fw_download(info, fw->data, fw->size);

out:
	kfree(info);
	release_firmware(fw);

	return ret;
}


int mms_flash_fw_file(struct i2c_client *client, struct melfas_tsi_platform_data *pdata)
{
	int ret;
	struct mms_info *info = kzalloc(sizeof(struct mms_info), GFP_KERNEL);

	info->client = client;
	info->pdata = pdata;

	mms100_set_gpio_mode(1);

	dev_err(&client->dev, "< Melfas > mms_flash_fw_file \n");

	ret = fw_download(info, MELFAS_binary, MELFAS_binary_nLength);

	mms100_set_gpio_mode(0);

	kfree(info);

	return ret;
}
EXPORT_SYMBOL(mms_flash_fw_file);

