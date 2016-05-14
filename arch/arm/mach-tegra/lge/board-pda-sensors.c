/*
 * arch/arm/mach-tegra/lge/board-pda-sensors.c
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

#include <linux/i2c.h>
#if defined(CONFIG_INV_MPU_IIO)
#include <linux/mpu.h>
#endif
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#if defined(CONFIG_MACH_PDA)
#include <linux/interrupt.h>
#endif

#include <media/camera.h>
#include <media/imx091.h>
#include <media/imx132.h>
#include <media/ad5816.h>
#include <media/imx135.h>
#include <media/max77387.h>
#include <media/lm3565.h>
#include <linux/nct1008.h>
#include <linux/max17048_battery.h>
#include <mach/edp.h>
#include <generated/mach-types.h>
#include <linux/pid_thermal_gov.h>

#include "../cpu-tegra.h"
#include "../devices.h"
#include "../board-common.h"
#include "board-pda.h"
#include "board-atlantis-pda.h"
#include "../tegra-board-id.h"
#include "../board.h"
#include "../battery-ini-model-data.h"

#ifdef CONFIG_MACH_PDA
#include <media/imx111.h>
#include <media/imx179.h>
#include <media/imx119.h>
#include <media/dw9714.h>
#include <mach/board_lge.h>
#include <linux/lge_uei_irrc.h>
#endif

static struct board_info board_info;

static struct nvc_gpio_pdata imx091_gpio_pdata[] = {
	{IMX091_GPIO_RESET, CAM_RSTN, true, false},
};

static struct throttle_table tj_throttle_table[] = {
	/* CPU_THROT_LOW cannot be used by other than CPU */
	/*      CPU,  C2BUS,  C3BUS,   SCLK,    EMC   */
	{ { 1530000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1504500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1479000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1453500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1428000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1402500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1377000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1351500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1326000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1300500, 691200, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1275000, 691200, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1249500, 691200, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1224000, 691200, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1198500, 691200, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1173000, 691200, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1147500, 652800, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1122000, 652800, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1096500, 652800, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1071000, 652800, NO_CAP, 384000, NO_CAP } },
	{ { 1045500, 652800, 480000, 384000, NO_CAP } },
	{ { 1020000, 595200, 480000, 384000, NO_CAP } },
	{ {  994500, 595200, 480000, 384000, NO_CAP } },
	{ {  969000, 595200, 480000, 384000, NO_CAP } },
	{ {  943500, 595200, 480000, 384000, NO_CAP } },
	{ {  918000, 595200, 480000, 384000, NO_CAP } },
	{ {  892500, 595200, 441600, 384000, NO_CAP } },
	{ {  867000, 556800, 441600, 384000, NO_CAP } },
	{ {  841500, 556800, 441600, 384000, NO_CAP } },
	{ {  816000, 556800, 441600, 384000, NO_CAP } },
	{ {  790500, 556800, 441600, 384000, 408000 } },
	{ {  765000, 556800, 441600, 204000, 408000 } },
	{ {  739500, 556800, 364800, 204000, 408000 } },
	{ {  714000, 499200, 364800, 204000, 408000 } },
	{ {  688500, 499200, 364800, 204000, 408000 } },
	{ {  663000, 499200, 364800, 204000, 408000 } },
	{ {  637500, 499200, 364800, 102000, 408000 } },
	{ {  612000, 499200, 326400, 102000, 408000 } },
	{ {  586500, 422400, 326400, 102000, 408000 } },
	{ {  561000, 422400, 326400, 102000, 408000 } },
	{ {  535500, 422400, 326400, 102000, 408000 } },
	{ {  510000, 422400, 326400, 102000, 408000 } },
	{ {  484500, 422400, 249600, 102000, 408000 } },
	{ {  459000, 307200, 249600, 102000, 408000 } },
	{ {  433500, 307200, 249600, 102000, 408000 } },
	{ {  408000, 307200, 249600, 102000, 408000 } },
	{ {  382500, 307200, 249600, 102000, 408000 } },
	{ {  357000, 307200, 249600, 102000, 408000 } },
	{ {  331500, 307200, 249600, 102000, 408000 } },
	{ {  306000, 307200, 249600, 102000, 408000 } },
	};

static struct balanced_throttle tj_throttle = {
	.throt_tab_size = ARRAY_SIZE(tj_throttle_table),
	.throt_tab = tj_throttle_table,
};

static int __init pda_throttle_init(void)
{
	if (of_machine_is_compatible("nvidia,pda"))
		balanced_throttle_register(&tj_throttle, "tegra-balanced");
	return 0;
}
module_init(pda_throttle_init);

static struct nct1008_platform_data pda_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x06, /* 4Hz conversion rate */
	.shutdown_ext_limit = 95, /* C */
	.shutdown_local_limit = 100, /* C */

	.passive_delay = 2000,

	.num_trips = 1,
	.trips = {
		{
			.cdev_type = "suspend_soctherm",
			.trip_temp = 50000,
			.trip_type = THERMAL_TRIP_ACTIVE,
			.upper = 1,
			.lower = 1,
			.hysteresis = 5000,
		},
	},

	.num_local_trips = 1,
	.local_trips = {
		{
			.cdev_type = "therm_est_activ",
			.trip_temp = 40000,
			.trip_type = THERMAL_TRIP_ACTIVE,
			.upper = THERMAL_NO_LIMIT,
			.lower = THERMAL_NO_LIMIT,
			.hysteresis = 1000,
		},
	},
};

struct max17048_platform_data max17048_pdata = {
	.model_data = &ceres_yoku_2000_ssv_3_1_max17048_battery,
	.tz_name = "battery-temp",
#ifdef CONFIG_MACH_PDA
	.soc_error_max_value = 95,
#else
	.soc_error_max_value = 96,
#endif
};

static struct i2c_board_info __initdata max77660_fg_board_info[] = {
	{
		I2C_BOARD_INFO("max17048", 0x36),
		.platform_data  = &max17048_pdata,
	},
};

static struct i2c_board_info pda_i2c0_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct72", 0x4C),
		.platform_data = &pda_nct1008_pdata,
		.irq = -1,
	}
};

#define PDA_TEMP_ALERT_GPIO	TEGRA_GPIO_PO1
static int pda_nct1008_init(void)
{
	int ret = 0;

	tegra_add_cdev_trips(pda_nct1008_pdata.trips,
			     &pda_nct1008_pdata.num_trips);

	/* FIXME: enable irq when throttling is supported */
	pda_i2c0_nct1008_board_info[0].irq =
		gpio_to_irq(PDA_TEMP_ALERT_GPIO);

	ret = gpio_request(PDA_TEMP_ALERT_GPIO, "temp_alert");
	if (ret < 0) {
		pr_err("%s: gpio_request failed\n", __func__);
		return ret;
	}

	ret = gpio_direction_input(PDA_TEMP_ALERT_GPIO);
	if (ret < 0) {
		pr_err("%s: set gpio to input failed\n", __func__);
		gpio_free(PDA_TEMP_ALERT_GPIO);
	}

	return ret;
}

static int pd_a_dw9714_power_on(struct dw9714_power_rail *pw)
{
	int err;
	pr_info("%s", __func__);

	if (unlikely(WARN_ON(!pw || !pw->vdd || !pw->vdd_i2c)))
			return -EFAULT;

	err = regulator_enable(pw->vdd_i2c);

	if (unlikely(err))
		goto dw9714_vdd_i2c_fail;

	err = regulator_enable(pw->vdd);
	if(unlikely(err))
		goto dw9714_vdd_fail;

	return 0;

dw9714_vdd_fail:
	regulator_disable(pw->vdd_i2c);

dw9714_vdd_i2c_fail:
	pr_err("%s FAILED\n", __func__);
	return -ENODEV;

	return 0;
}

static int pd_a_dw9714_power_off(struct dw9714_power_rail *pw)
{
	if (unlikely(WARN_ON(!pw || !pw->vdd || !pw->vdd_i2c)))
		return -EFAULT;

	regulator_disable(pw->vdd);
	regulator_disable(pw->vdd_i2c);

	return 0;
}

static struct nvc_focus_cap dw9714_cap = {
    .settle_time = 28,
    .slew_rate = 0x080006,
    .focus_macro = 532,
    .focus_infinity = 140,
    .focus_hyper = 140,
};

static struct dw9714_platform_data pd_a_dw9714_pdata = {
	.cfg		= 0,
	.num		= 0,
	.sync		= 0,
	.dev_name	= "focuser",
	.cap                = &dw9714_cap,
	.power_on	= pd_a_dw9714_power_on,
	.power_off	= pd_a_dw9714_power_off,
};

static int pda_imx091_power_on(struct nvc_regulator *vreg)
{
	int err;

	if (unlikely(WARN_ON(!vreg)))
		return -EFAULT;

	gpio_set_value(CAM1_POWER_DWN_GPIO, 0);
	usleep_range(10, 20);

	err = regulator_enable(vreg[IMX091_VREG_DVDD].vreg);
	if (unlikely(err))
		goto imx091_dvdd_fail;

	err = regulator_enable(vreg[IMX091_VREG_AVDD].vreg);
	if (err)
		goto imx091_avdd_fail;

	err = regulator_enable(vreg[IMX091_VREG_IOVDD].vreg);
	if (err)
		goto imx091_iovdd_fail;

	usleep_range(1, 2);
	gpio_set_value(CAM1_POWER_DWN_GPIO, 1);

	usleep_range(300, 310);

	return 1;

imx091_iovdd_fail:
	regulator_disable(vreg[IMX091_VREG_DVDD].vreg);

imx091_avdd_fail:
	gpio_set_value(CAM1_POWER_DWN_GPIO, 0);

imx091_dvdd_fail:
	regulator_disable(vreg[IMX091_VREG_AVDD].vreg);

	pr_err("%s FAILED\n", __func__);
	return -ENODEV;
}

static int pda_imx091_power_off(struct nvc_regulator *vreg)
{
	if (unlikely(WARN_ON(!vreg)))
		return -EFAULT;

	usleep_range(1, 2);

	gpio_set_value(CAM1_POWER_DWN_GPIO, 0);
	usleep_range(1, 2);

	regulator_disable(vreg[IMX091_VREG_IOVDD].vreg);
	regulator_disable(vreg[IMX091_VREG_AVDD].vreg);
	regulator_disable(vreg[IMX091_VREG_DVDD].vreg);
	return 0;
}
static struct nvc_imager_cap imx091_cap = {
	.identifier		= "IMX091",
	.sensor_nvc_interface	= 3,
	.pixel_types[0]		= 0x101,
	.orientation		= 0,
	.direction		= 0,
	.initial_clock_rate_khz	= 6000,
	.clock_profiles[0] = {
		.external_clock_khz	= 24000,
		.clock_multiplier	= 850000, /* value / 1,000,000 */
	},
	.clock_profiles[1] = {
		.external_clock_khz	= 0,
		.clock_multiplier	= 0,
	},
	.h_sync_edge		= 0,
	.v_sync_edge		= 0,
	.mclk_on_vgp0		= 0,
	.csi_port		= 0,
	.data_lanes		= 4,
	.virtual_channel_id	= 0,
	.discontinuous_clk_mode	= 0,
	.cil_threshold_settle	= 0xd,
	.min_blank_time_width	= 16,
	.min_blank_time_height	= 16,
	.preferred_mode_index	= 0,
	.focuser_guid		= NVC_FOCUS_GUID(0),
	.torch_guid		= NVC_TORCH_GUID(0),
	.cap_version		= NVC_IMAGER_CAPABILITIES_VERSION2,
};

static unsigned imx091_estates[] = {600, 0};

static struct imx091_platform_data pda_imx091_data = {
	.num			= 0,
	.sync			= 0,
	.cfg			= 0,
	.dev_name		= "camera",
	.gpio_count		= ARRAY_SIZE(imx091_gpio_pdata),
	.gpio			= imx091_gpio_pdata,
	.flash_cap		= {
		.sdo_trigger_enabled = 1,
		.adjustable_flash_timing = 1,
	},
	.cap			= &imx091_cap,
	.edpc_config		= {
		.states = imx091_estates,
		.num_states = ARRAY_SIZE(imx091_estates),
		.e0_index = ARRAY_SIZE(imx091_estates) - 1,
		.priority = EDP_MAX_PRIO + 1,
	},
	.power_on		= pda_imx091_power_on,
	.power_off		= pda_imx091_power_off,
};

static int pd_a_imx119_power_on(struct imx119_power_rail *pw)
{
	int err;

	if (unlikely(WARN_ON(!pw || !pw->avdd || !pw->iovdd || !pw->dvdd)))
		return -EFAULT;

    pr_info("%s\n", __func__);

	gpio_set_value(VT_RESET_N, 0);
	usleep_range(10, 20);

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
	gpio_set_value(VT_RESET_N, 1);
	usleep_range(300, 310);

	return 1;

imx119_avdd_fail:
	gpio_set_value(VT_RESET_N, 0);

imx119_iovdd_fail:
	regulator_disable(pw->dvdd);

imx119_dvdd_fail:
	regulator_disable(pw->avdd);

	pr_err("%s FAILED\n", __func__);
	return -ENODEV;
}

static int pd_a_imx119_power_off(struct imx119_power_rail *pw)
{
	if (unlikely(WARN_ON(!pw || !pw->avdd || !pw->iovdd || !pw->dvdd)))
		return -EFAULT;

	usleep_range(1, 2);
	gpio_set_value(VT_RESET_N, 0);
	usleep_range(1, 2);

	regulator_disable(pw->avdd);
	regulator_disable(pw->iovdd);
	regulator_disable(pw->dvdd);

	return 0;
}

static unsigned imx119_estates[] = {200, 100, 2};

struct imx119_platform_data pd_a_imx119_data = {
	.edpc_config	= {
		.states = imx119_estates,
		.num_states = ARRAY_SIZE(imx119_estates),
		.e0_index = ARRAY_SIZE(imx119_estates) - 1,
		.priority = EDP_MAX_PRIO + 1,
		},
	.mclk_name = "vi_sensor",
	.power_on = pd_a_imx119_power_on,
	.power_off = pd_a_imx119_power_off,
};

static struct i2c_board_info __initdata pda_i2c_board_info_tcs3772[] = {
	{
		I2C_BOARD_INFO("tcs3772", 0x29),
	},
};

static struct i2c_board_info pd_a_i2c2_boardinfo_camera[] = {
#ifdef CONFIG_MACH_PDA
	{
		I2C_BOARD_INFO("imx091", 0x10),
		.platform_data = &pda_imx091_data,
	},
#else
	{
		I2C_BOARD_INFO("imx179", 0x10),
		.platform_data = &pd_a_imx179_data,
	},
#endif
	{
		I2C_BOARD_INFO("dw9714", 0x0c),
		.platform_data  = &pd_a_dw9714_pdata,
	},
	{
		I2C_BOARD_INFO("imx119", 0x37),
		.platform_data  = &pd_a_imx119_data,
	},
	{
		I2C_BOARD_INFO("gt24c32a", 0x53),
	},
};

static int pda_camera_init(void)
{
#if defined(CONFIG_MACH_PDA)
	i2c_register_board_info(2, pd_a_i2c2_boardinfo_camera,
			ARRAY_SIZE(pd_a_i2c2_boardinfo_camera));
#else
	if (board_info.board_id == BOARD_E1670)
		i2c_register_board_info(2, pda_i2c_board_info_e1697,
			ARRAY_SIZE(pda_i2c_board_info_e1697));
	else if (board_info.board_id == BOARD_E1740)
		i2c_register_board_info(2, pda_i2c_board_info_e1697,
			ARRAY_SIZE(pda_i2c_board_info_e1697));
	else if (board_info.board_id == BOARD_E1680)
		i2c_register_board_info(2, pda_i2c_board_info_e1707,
			ARRAY_SIZE(pda_i2c_board_info_e1707));
	else
		i2c_register_board_info(2, pda_i2c_board_info_e1690,
			ARRAY_SIZE(pda_i2c_board_info_e1690));
#endif
	return 0;
}

#ifndef CONFIG_MACH_PDA
/* MPU board file definition	*/
static struct mpu_platform_data mpu9150_gyro_data = {
	.int_config	= 0x10,
	.level_shifter	= 0,
	/* Located in board_[platformname].h */
	.orientation	= MPU_GYRO_ORIENTATION,
	.sec_slave_type	= SECONDARY_SLAVE_TYPE_NONE,
	.key		= {0x4E, 0xCC, 0x7E, 0xEB, 0xF6, 0x1E, 0x35, 0x22,
			   0x00, 0x34, 0x0D, 0x65, 0x32, 0xE9, 0x94, 0x89},
};

static struct mpu_platform_data mpu9150_gyro_data_e1680 = {
	.int_config	= 0x10,
	.level_shifter	= 0,
	/* Located in board_[platformname].h */
	.orientation	= MPU_GYRO_ORIENTATION_E1680,
	.sec_slave_type	= SECONDARY_SLAVE_TYPE_NONE,
	.key		= {0x4E, 0xCC, 0x7E, 0xEB, 0xF6, 0x1E, 0x35, 0x22,
			   0x00, 0x34, 0x0D, 0x65, 0x32, 0xE9, 0x94, 0x89},
};

static struct mpu_platform_data mpu9150_gyro_data_e1670 = {
	.int_config	= 0x10,
	.level_shifter	= 0,
	/* Located in board_[platformname].h */
	.orientation	= MPU_GYRO_ORIENTATION_E1670,
	.sec_slave_type	= SECONDARY_SLAVE_TYPE_NONE,
	.key		= {0x4E, 0xCC, 0x7E, 0xEB, 0xF6, 0x1E, 0x35, 0x22,
			   0x00, 0x34, 0x0D, 0x65, 0x32, 0xE9, 0x94, 0x89},
};

static struct mpu_platform_data mpu_compass_data = {
	.orientation	= MPU_COMPASS_ORIENTATION,
	.config		= NVI_CONFIG_BOOT_MPU,
};

static struct mpu_platform_data mpu_compass_data_e1680 = {
	.orientation	= MPU_COMPASS_ORIENTATION_E1680,
	.config		= NVI_CONFIG_BOOT_MPU,
};

static struct mpu_platform_data mpu_compass_data_e1670 = {
	.orientation	= MPU_COMPASS_ORIENTATION_E1670,
	.config		= NVI_CONFIG_BOOT_MPU,
};

static struct mpu_platform_data bmp180_pdata = {
	.config		= NVI_CONFIG_BOOT_MPU,
};

static struct i2c_board_info __initdata inv_mpu9150_i2c1_board_info[] = {
	{
		I2C_BOARD_INFO(MPU_GYRO_NAME, MPU_GYRO_ADDR),
		.platform_data = &mpu9150_gyro_data,
	},
	{
		/* The actual BMP180 address is 0x77 but because this conflicts
		 * with another device, this address is hacked so Linux will
		 * call the driver.  The conflict is technically okay since the
		 * BMP180 is behind the MPU.  Also, the BMP180 driver uses a
		 * hard-coded address of 0x77 since it can't be changed anyway.
		 */
		I2C_BOARD_INFO("bmp180", 0x78),
		.platform_data = &bmp180_pdata,
	},
	{
		I2C_BOARD_INFO(MPU_COMPASS_NAME, MPU_COMPASS_ADDR),
		.platform_data = &mpu_compass_data,
	},
};
#endif /* !CONFIG_MACH_PDA */

/* MPU6515 board file definition */
static struct mpu_platform_data mpu_gyro_data = {
	.int_config = 0x00,
	.level_shifter = 0,
	.orientation = MPU_GYRO_ORIENTATION,

	.sec_slave_type = SECONDARY_SLAVE_TYPE_COMPASS,
	.sec_slave_id = COMPASS_ID_AK8963,
	.secondary_i2c_addr = MPU_COMPASS_ADDR,
	.secondary_orientation = MPU_COMPASS_ORIENTATION,

	.key = {221, 22, 205, 7,   217, 186, 151, 55,
		206, 254, 35, 144, 225, 102,  47, 50},
};

static struct i2c_board_info __initdata inv_mpu_i2c0_board_info[] = {
	{
		I2C_BOARD_INFO("mpu6515", MPU_GYRO_ADDR),
		.irq = (INT_GPIO_BASE + MPU_GYRO_IRQ_GPIO),
		.platform_data = &mpu_gyro_data,
	},
};

static void mpuirq_init(void)
{
	int ret = 0;

	pr_info("*** MPU START *** mpuirq_init...\n");

	ret = gpio_request(MPU_GYRO_IRQ_GPIO, "mpu6515");
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(MPU_GYRO_IRQ_GPIO);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(MPU_GYRO_IRQ_GPIO);
		return;
	}

	pr_info("*** MPU END *** mpuirq_init...\n");

	i2c_register_board_info(MPU_GYRO_BUS_NUM, inv_mpu_i2c0_board_info,
		ARRAY_SIZE(inv_mpu_i2c0_board_info));
}

#if defined(CONFIG_INPUT_SENSOR)
struct bosch_sensor_specific {
	char *name;
	int place;	/* 0 to 7 */
	int irq;
};

static struct bosch_sensor_specific bmc_accel_data = {
	.name = BMC_ACCEL_NAME,
	.irq = BMC_ACCEL_IRQ_GPIO,
	.place = 5,
};

static struct bosch_sensor_specific bmc_compass_data = {
	.name = BMC_COMPASS_NAME,
	.place = 5,
};

static struct i2c_board_info __initdata bmc_i2c0_board_info[] = {
	{
		I2C_BOARD_INFO(BMC_ACCEL_NAME, BMC_ACCEL_ADDR),
		.irq = (INT_GPIO_BASE + BMC_ACCEL_IRQ_GPIO),
		.platform_data = &bmc_accel_data,
	},
	{
		I2C_BOARD_INFO(BMC_COMPASS_NAME, BMC_COMPASS_ADDR),
		.platform_data = &bmc_compass_data,
	},
};

static struct bosch_sensor_specific bmc_accel_data_revA = {
        .name = BMC_ACCEL_NAME,
        .irq = BMC_ACCEL_IRQ_GPIO,
        .place = 7,     /* S17 Rev_A */
};

static struct bosch_sensor_specific bmc_compass_data_revA = {
        .name = BMC_COMPASS_NAME,
        .place = 7,     /* S17 Rev_A */
};

static struct i2c_board_info __initdata bmc_i2c0_board_info_revA[] = {
        {
                I2C_BOARD_INFO(BMC_ACCEL_NAME, BMC_ACCEL_ADDR),
                .irq = (INT_GPIO_BASE + BMC_ACCEL_IRQ_GPIO),
                .platform_data = &bmc_accel_data_revA,
        },
        {
                I2C_BOARD_INFO(BMC_COMPASS_NAME, BMC_COMPASS_ADDR),
                .platform_data = &bmc_compass_data_revA,
        },
};

static struct bosch_sensor_specific bmc_accel_data_rev1 = {
        .name = BMC_ACCEL_NAME,
        .irq = BMC_ACCEL_IRQ_GPIO,
        .place = 6,     /* S17 Rev_1.0 */
};

static struct bosch_sensor_specific bmc_compass_data_rev1 = {
        .name = BMC_COMPASS_NAME,
        .place = 6,     /* S17 Rev_1.0 */
};

static struct i2c_board_info __initdata bmc_i2c0_board_info_rev1[] = {
        {
                I2C_BOARD_INFO(BMC_ACCEL_NAME, BMC_ACCEL_ADDR),
                .irq = (INT_GPIO_BASE + BMC_ACCEL_IRQ_GPIO),
                .platform_data = &bmc_accel_data_rev1,
        },
        {
                I2C_BOARD_INFO(BMC_COMPASS_NAME, BMC_COMPASS_ADDR),
                .platform_data = &bmc_compass_data_rev1,
        },
};

static void bmcirq_init(void)
{
	int ret = 0;

	pr_info("%s: BMC150 ACCEL_INT init\n", __func__);

	ret = gpio_request(BMC_ACCEL_IRQ_GPIO, "ACCEL_INT");
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(BMC_ACCEL_IRQ_GPIO);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(BMC_ACCEL_IRQ_GPIO);
		return;
	}

	pr_info("%s: BMC150 I2C register\n", __func__);

	if (lge_get_board_revno() >= HW_REV_D625_1_0) {
		i2c_register_board_info(BMC_ACCEL_BUS_NUM, bmc_i2c0_board_info_rev1,
			ARRAY_SIZE(bmc_i2c0_board_info_rev1));
	} else if ((lge_get_board_revno() < HW_REV_D625_1_0) && (lge_get_board_revno() >= HW_REV_D625_B)){
		i2c_register_board_info(BMC_ACCEL_BUS_NUM, bmc_i2c0_board_info,
			ARRAY_SIZE(bmc_i2c0_board_info));
	} else {
		i2c_register_board_info(BMC_ACCEL_BUS_NUM, bmc_i2c0_board_info_revA,
			ARRAY_SIZE(bmc_i2c0_board_info_revA));
	}
}
#endif

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static struct thermal_trip_info skin_trips[] = {
	{
		.cdev_type = "skin-balanced",
		.trip_temp = 42000,
		.trip_type = THERMAL_TRIP_PASSIVE,
		.upper = THERMAL_NO_LIMIT,
		.lower = THERMAL_NO_LIMIT,
		.hysteresis = 0,
	},
	{
		.cdev_type = "tegra-shutdown",
		.trip_temp = 57000,
		.trip_type = THERMAL_TRIP_CRITICAL,
		.upper = THERMAL_NO_LIMIT,
		.lower = THERMAL_NO_LIMIT,
		.hysteresis = 0,
	},
};

static struct therm_est_subdevice skin_devs[] = {
	{
		.dev_data = "Tdiode",
		.coeffs = {
		-6, 1, 1, -2,
		-2, -1, -3, -3,
		-1, -1, -1, -1,
		 1, 1, -1, -1,
		 1, 0, -4, -21
		},
	},
	{
		.dev_data = "Tboard",
		.coeffs = {
			29, 20, 10, 7,
			6, 8, 3, 2,
			4, 5, 5, 4,
			6, 9, 6, 7,
			8, 6, -6, -14
		},
	},
};

static struct therm_est_subdevice skin_devs_revb[] = {
	{
		.dev_data = "Tdiode",
		.coeffs = {
			-1, -1, -1, -1,
			-1, -1, -2, -1,
			-1, -2, -1, -1,
			-2, -1, -1, 0,
			 1, 1, -2, -18
		},
	},
	{
		.dev_data = "Tboard",
		.coeffs = {
			29, 18, 10, 7,
			4, 4, 4, 5,
			6, 5, 6, 5,
			4, 5, 4, 5,
			4, 1, 0, -5
		},
	},
};


static struct pid_thermal_gov_params skin_pid_params = {
	.max_err_temp = 4000,
	.max_err_gain = 1000,

	.gain_p = 1000,
	.gain_d = 0,

	.up_compensation = 15,
	.down_compensation = 15,
};

static struct thermal_zone_params skin_tzp = {
	.governor_name = "pid_thermal_gov",
	.governor_params = &skin_pid_params,
};

static struct therm_est_data skin_data = {
	.num_trips = ARRAY_SIZE(skin_trips),
	.trips = skin_trips,
	.polling_period = 1100,
	.passive_delay = 15000,
	.tc1 = 10,
	.tc2 = 1,
	.tzp = &skin_tzp,
	.use_activator = 1,
};

#if 0
static struct throttle_table skin_throttle_table[] = {
	/* CPU_THROT_LOW cannot be used by other than CPU */
	/*      CPU,  C2BUS,  C3BUS,   SCLK,    EMC   */
	{ { 1530000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1504500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1479000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1453500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1428000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1402500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1377000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1351500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1326000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1300500, 691200, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1275000, 691200, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1249500, 691200, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1224000, 691200, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1198500, 691200, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1173000, 691200, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1147500, 652800, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1122000, 652800, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1096500, 652800, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1071000, 652800, NO_CAP, 384000, NO_CAP } },
	{ { 1045500, 652800, 480000, 384000, NO_CAP } },
	{ { 1020000, 595200, 480000, 384000, NO_CAP } },
	{ {  994500, 595200, 480000, 384000, NO_CAP } },
	{ {  969000, 595200, 480000, 384000, NO_CAP } },
	{ {  943500, 595200, 480000, 384000, NO_CAP } },
	{ {  918000, 595200, 480000, 384000, NO_CAP } },
	{ {  892500, 595200, 441600, 384000, NO_CAP } },
	{ {  867000, 556800, 441600, 384000, NO_CAP } },
	{ {  841500, 556800, 441600, 384000, NO_CAP } },
	{ {  816000, 556800, 441600, 384000, NO_CAP } },
	{ {  790500, 556800, 441600, 384000, 408000 } },
	{ {  765000, 556800, 441600, 204000, 408000 } },
	{ {  739500, 556800, 364800, 204000, 408000 } },
	{ {  714000, 499200, 364800, 204000, 408000 } },
	{ {  688500, 499200, 364800, 204000, 408000 } },
	{ {  663000, 499200, 364800, 204000, 408000 } },
	{ {  637500, 499200, 364800, 102000, 408000 } },
	{ {  612000, 499200, 326400, 102000, 408000 } },
	{ {  586500, 422400, 326400, 102000, 408000 } },
	{ {  561000, 422400, 326400, 102000, 408000 } },
	{ {  535500, 422400, 326400, 102000, 408000 } },
	{ {  510000, 422400, 326400, 102000, 408000 } },
	{ {  484500, 422400, 249600, 102000, 408000 } },
	{ {  459000, 307200, 249600, 102000, 408000 } },
	{ {  433500, 307200, 249600, 102000, 408000 } },
	{ {  408000, 307200, 249600, 102000, 408000 } },
	{ {  382500, 307200, 249600, 102000, 408000 } },
	{ {  357000, 307200, 249600, 102000, 408000 } },
	{ {  331500, 307200, 249600, 102000, 408000 } },
	{ {  306000, 307200, 249600, 102000, 408000 } },
};

static struct balanced_throttle skin_throttle = {
	.throt_tab_size = ARRAY_SIZE(skin_throttle_table),
	.throt_tab = skin_throttle_table,
};
#endif

static int __init pda_skin_init(void)
{
	int i;

	if (of_machine_is_compatible("nvidia,pda")) {
		if (board_info.board_id == BOARD_E1690 &&
				board_info.fab < BOARD_FAB_B) {
			skin_data.ndevs = ARRAY_SIZE(skin_devs);
			skin_data.devs = skin_devs;
			skin_data.toffset = 1909;
		} else { /* >= BOARD_FAB_B */
			skin_data.ndevs = ARRAY_SIZE(skin_devs_revb);
			skin_data.devs = skin_devs_revb;
			skin_data.toffset = 364;
		}

		if (board_info.board_id == BOARD_E1690 &&
				board_info.fab <= BOARD_FAB_B) {
			/* we effectively disabled the shutdown trip point */
			for (i = 0; i < skin_data.num_trips; i++) {
				if (strcmp(skin_data.trips[i].cdev_type,
					  "tegra-shutdown") == 0) {
					break;
				}
			}

			if (i != skin_data.num_trips)
				skin_data.trips[i].trip_temp = 120000;
		}
#if 0
		balanced_throttle_register(&skin_throttle, "skin-balanced");
#endif
		tegra_skin_therm_est_device.dev.platform_data = &skin_data;
		platform_device_register(&tegra_skin_therm_est_device);
	}

	return 0;
}
late_initcall(pda_skin_init);
#endif

#if defined(CONFIG_MACH_PDA)
#include <linux/mfd/pm8xxx/cradle.h>

struct pm8xxx_cradle_platform_data pda_hall_ic_data = {
	.hallic_pouch_detect_pin = TEGRA_GPIO_PO6,
};

struct platform_device pda_hall_ic_device = {
	.name = HALL_IC_DEV_NAME,
	.dev = {
		.platform_data = &pda_hall_ic_data,
	},
};

static void pda_hall_ic_init(void)
{
	s32 ret = 0;
	pr_info("hall_ic: HALL_IC_INT GPIO(%d) IRQ(%d)\n",
			TEGRA_GPIO_PO6, gpio_to_irq(TEGRA_GPIO_PO6));

	ret = gpio_request(TEGRA_GPIO_PO6, "HALL_IC_INT");
	if (ret < 0)
	{
		pr_err("hall_ic: fail to gpio_request HALL_IC_INT\n");
		return;
	}

	ret = gpio_direction_input(TEGRA_GPIO_PO6);
	if (ret < 0)
	{
		pr_err("hall_ic: fail to gpio_direction_input HALL_IC_INT\n");
		gpio_free(TEGRA_GPIO_PO6);
		return;
	}

	platform_device_register(&pda_hall_ic_device);
}
#endif

#if defined(CONFIG_MACH_PDA)
#include <linux/i2c/apds9x3x.h>

/* APDS9130 */
struct apds9130_platform_data apds9130_prox_data = {
	.irq_gpio = APDS9130_INT,
};

static struct i2c_board_info __initdata apds9130_board_info[] = {
	{
		I2C_BOARD_INFO(APDS9130_NAME, APDS9130_ADDR),
		.platform_data = &apds9130_prox_data,
	},
};

/* APDS993X */
struct apds993x_platform_data apds993x_prox_data = {
	.irq_gpio = APDS993X_INT,
};

static struct i2c_board_info __initdata apds993x_board_info[] = {
	{
		I2C_BOARD_INFO(APDS993X_NAME, APDS993X_ADDR),
		.platform_data = &apds993x_prox_data,
		.irq = (INT_GPIO_BASE + APDS993X_INT),
	},
};

static void pda_proximity_init(void)
{
/*                                                                                            
                         
                         
                          
                                                                                             */
	if (lge_get_board_revno() < HW_REV_PDA_C) {
		s32 ret = 0;
		unsigned prox_int;
		prox_int = APDS993X_INT;	 /*APDS9930 proximity interrupt*/

		pr_info("proximity: PROXIMITY_INT GPIO(%d) IRQ(%d)\n",
			prox_int, gpio_to_irq(prox_int));

		ret = gpio_request(prox_int, "PROXIMITY_INT");
		if (ret < 0)
		{
			pr_err("proximity: fail to gpio_request PROXIMITY_INT\n");
			return;
		}

		ret = gpio_direction_input(prox_int);
		if (ret < 0)
		{
			pr_err("proximity: fail to gpio_direction_input PROXIMITY_INT\n");
			gpio_free(prox_int);
			return;
		}

		i2c_register_board_info(APDS993X_BUS_NUM,
			apds993x_board_info, ARRAY_SIZE(apds993x_board_info));
	} else {
		i2c_register_board_info(APDS9130_BUS_NUM,
			apds9130_board_info, ARRAY_SIZE(apds9130_board_info));
		printk("%s : apds9130 enable \n", __func__);
	}
}
#endif

#ifdef CONFIG_LGE_IRRC
static struct uei_irrc_pdata_type uei_irrc_pdata = {
	.reset_gpio = TEGRA_GPIO_PL1,
    .led_ldo_gpio = TEGRA_GPIO_PE4,
};

static struct platform_device uei_irrc_device = {
	.name = "uei_irrc",
	.id = -1,
	.dev = {
        .platform_data = &uei_irrc_pdata,
	},
};

static int pda_irrc_init(void)
{
	return platform_device_register(&uei_irrc_device);
}
#endif

int __init pda_sensors_init(void)
{
	int err;

	tegra_get_board_info(&board_info);

	pda_camera_init();

/*                                                                                            
                         
                         
                          
                                                                                             */
	if (lge_get_board_revno() < HW_REV_PDA_C)
		mpuirq_init();	/* MPU6515 accel+gyro, AK8963 compass */
	else
		bmcirq_init();	/* BMC150 accel+compass */

#if defined(CONFIG_MACH_PDA)
	pda_proximity_init();
#endif
/*                                                                                            
                         
                         
                          
                                                                                             */
	if(lge_get_board_revno() >= HW_REV_PDA_C){
		pda_hall_ic_init();
	}

#ifdef CONFIG_LGE_IRRC
	err = pda_irrc_init();
	if (err)
		pr_err("%s: uei IRRC MXQ616 init failed\n", __func__);
#endif
	err = pda_nct1008_init();
	if (err)
		pr_err("%s: nct1008 init failed\n", __func__);
	else
		i2c_register_board_info(0, pda_i2c0_nct1008_board_info,
				ARRAY_SIZE(pda_i2c0_nct1008_board_info));
	if ((board_info.board_id != BOARD_E1670) &&
		 (board_info.board_id != BOARD_E1740)) {
#ifndef CONFIG_MACH_PDA
		i2c_register_board_info(0, pda_i2c_board_info_max44005,
				ARRAY_SIZE(pda_i2c_board_info_max44005));
#endif
		if (get_power_supply_type() == POWER_SUPPLY_TYPE_BATTERY)
			i2c_register_board_info(0, max77660_fg_board_info, 1);
	} else {
		i2c_register_board_info(0, pda_i2c_board_info_tcs3772,
				ARRAY_SIZE(pda_i2c_board_info_tcs3772));
	}

	return 0;
}

#ifndef CONFIG_MACH_PDA
#define IMX091_ID	0x0091
static int pda_chk_imx091(struct device *dev, void *addrp)
{
	struct i2c_client *client = i2c_verify_client(dev);
	unsigned short addr = *(unsigned short *)addrp;

	if (!client)
		return 0;

	if (client->addr == addr) {
		struct i2c_adapter *adap = i2c_get_adapter(2);
		u16 *imx091_devid = (u16 *) i2c_get_clientdata(client);

		if (imx091_devid != NULL && *imx091_devid == IMX091_ID) {
			if (board_info.board_id == BOARD_E1670)
				i2c_new_device(adap,
					&pda_i2c_board_info_lm3565);
			else
				i2c_new_device(adap,
					&pda_i2c_board_info_max77387);
		} else {
			i2c_unregister_device(client);
			i2c_new_device(adap, &pda_i2c_board_info_imx135);
			i2c_new_device(adap, &pda_i2c_board_info_max77387);
		}
		return 1;
	}

	return 0;
}

int camera_auto_detect(void)
{
	struct i2c_adapter *adap = i2c_get_adapter(2);
	u16 imx091_addr = 0x10;

	device_for_each_child(&adap->dev,
			&imx091_addr,
			pda_chk_imx091);

	return 0;
}

int __init pda_camera_late_init(void)
{
	if ((board_info.board_id != BOARD_E1670) &&
		(board_info.board_id != BOARD_E1680)) {
		pr_err("%s: Ceres/Atlantis ERS not found!\n", __func__);
		return 0;
	}

	camera_auto_detect();

	return 0;
}

late_initcall(pda_camera_late_init);
#endif

