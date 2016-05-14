/*
 * max77660-charger-extcon.c -- MAXIM MAX77660 VBUS detection.
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Darbha Sriharsha <dsriharsha@nvidia.com>
 * Author: Syed Rafiuddin <srafiuddin@nvidia.com>
 * Author: Laxman Dewangan <ldewangan@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/iio/consumer.h>
#include <linux/pm.h>
#include <linux/extcon.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/max77660/max77660-core.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/power_supply.h>
#include <linux/power/battery-charger-gauge-comm.h>
#if defined(CONFIG_MACH_PD_A) || defined(CONFIG_MACH_PDA)
#include <linux/debugfs.h>
#include <mach/board_lge.h>
#endif

#define CHARGER_USB_EXTCON_REGISTRATION_DELAY	5000
#define CHARGER_TYPE_DETECTION_DEBOUNCE_TIME_MS	500
//#define MAX77660_RESTART_CHARGING_AFTER_DONE	(15 * 60)

#define BATT_TEMP_HOT				56
#define BATT_TEMP_WARM				45
#define BATT_TEMP_COOL				10
#define BATT_TEMP_COLD				3

#define NO_LIMIT				99999

#if defined(CONFIG_MACH_PD_A) || defined(CONFIG_MACH_PDA)
/* For 0xC revision */
#define	MAX77660_USBCHGCTRL_USB_HICURRENT	0x08
#define	MAX77660_AICLCNTL_DCMON_DIS		0x01
#define MAX77660_AICLCNTL_ENABLE		0x00 // enable flag
#endif /* CONFIG_MACH_PDA */

/*                                                                            */
#ifdef CONFIG_MACH_PDA
#define LGE_CHG_MON_TIME_NORMAL			(600*HZ)
#define LGE_CHG_MON_TIME_ABNORMAL		(30*HZ)
#define LGE_CHG_MON_TIME_NORMAL_PSEUDO			(60*HZ)
#define LGE_CHG_MON_TIME_ABNORMAL_PSEUDO		(10*HZ)

extern bool is_pseudo_batt(void);
extern void set_chg_done_flag(int enable);
#endif
/*                                                                          */
enum charging_states {
	ENABLED_HALF_IBAT = 1,
	ENABLED_FULL_IBAT,
#ifdef CONFIG_MACH_PDA
	ENABLED_LGE_OTP_450MA,
#endif
	DISABLED,
};

static int max77660_dccrnt_current_lookup[] = {
	1000, 1000, 1000, 2750, 3000, 3250, 3500, 3750,
	4000, 4250, 4500, 4750, 5000, 5250, 5500, 5750,
	6000, 6250, 6500, 6750, 7000, 7250, 7500, 7750,
	8000, 8250, 8500, 8750, 9000, 9250, 9500, 9750,
	10000, 10250, 10500, 10750, 11000, 11250, 11500, 11750,
	12000, 12250, 12500, 12750, 13000, 13250, 13500, 13750,
	14000, 14250, 14500, 14750, 15000, 15375, 15750, 16125,
	16500, 16875, 17250, 17625, 18000, 18375, 18750, NO_LIMIT,
};

static int max77660_maxcharge_voltage_lookup[] = {
	3550, 3700, 3750, 3800, 3850, 3900, 3950, 4000,
	4050, 4100, 4150, 4200, 4250, 4300, 4350, 4400,
};

static int max77660_chrg_wdt[] = {16, 32, 64, 128};

struct max77660_charger {
	struct max77660_bcharger_platform_data *bcharger_pdata;
	struct device *dev;
	int irq;
	int status;
	int in_current_lim;
	int wdt_timeout;
	int max_term_vol_mV;
#if defined(CONFIG_MACH_PD_A) || defined(CONFIG_MACH_PDA)
	int pmuotp;
	int fchg_current_limit; /*                                */
#endif
};

struct max77660_chg_extcon {
	struct device			*dev;
	struct device			*parent;
	struct extcon_dev		*edev;
	struct regulator_dev		*chg_rdev;

/*                                                                            */
#ifdef CONFIG_MACH_PDA
	struct delayed_work		chgmon_work;
	struct delayed_work		chg_irq_work;
#endif
/*                                                                          */
	struct regulator_dev		*vbus_rdev;
	struct regulator_desc		vbus_reg_desc;
	struct regulator_init_data	vbus_reg_init_data;

	struct max77660_charger		*charger;
	int				chg_irq;
	int				chg_wdt_irq;
	int				cable_connected;
	struct regulator_desc		chg_reg_desc;
	struct regulator_init_data	chg_reg_init_data;
	struct battery_charger_dev	*bc_dev;
	int				charging_state;
	int				cable_connected_state;
	int				battery_present;
	int				chg_restart_time_sec;
	int				last_charging_current;
};

struct max77660_chg_extcon *max77660_ext;

const char *max77660_excon_cable[] = {
	[0] = "USB",
	NULL,
};

/* IRQ definitions */
enum {
	MAX77660_CHG_BAT_I = 0,
	MAX77660_CHG_CHG_I,
	MAX77660_CHG_DC_UVP,
	MAX77660_CHG_DC_OVP,
	MAX77660_CHG_DC_I,
	MAX77660_CHG_DC_V,
	MAX77660_CHG_NR_IRQS,
};

static inline int fchg_current(int x)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(max77660_dccrnt_current_lookup); i++) {
		if (10*x < max77660_dccrnt_current_lookup[i])
			break;
	}
	ret = i;

	if (ret <= 3)
		return 0;
	else
		return ret;
}

static inline int convert_to_reg(int x)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(max77660_maxcharge_voltage_lookup); i++) {
		if (x < max77660_maxcharge_voltage_lookup[i])
			break;
	}

	return i > 0 ? i - 1 : -EINVAL;
}

static int max77660_battery_detect(struct max77660_chg_extcon *chip)
{
	int ret = 0;
	u8 status;

	ret = max77660_reg_read(chip->parent, MAX77660_CHG_SLAVE,
			MAX77660_CHARGER_DETAILS1, &status);
	if (ret < 0) {
		dev_err(chip->dev, "CHARGER_CHGINT read failed: %d\n", ret);
		return ret;
	}
	if ((status & MAX77660_BATDET_DTLS) == MAX77660_BATDET_DTLS_NO_BAT)
		return -EPERM;
	return 0;
}

static int max77660_charger_init(struct max77660_chg_extcon *chip, int enable)
{
	struct max77660_charger *charger = chip->charger;
	int ret;
	u8 read_val;

	if (!chip->battery_present)
		return 0;

	/* Clear CEN -juya.kim */
	ret = max77660_reg_clr_bits(chip->parent,
			MAX77660_CHG_SLAVE, MAX77660_CHARGER_CHGCTRL2,
			MAX77660_CEN_MASK);
	if (ret < 0)
		return ret;

	/* Enable USB suspend if 2mA is current configuration */
	if (charger->in_current_lim == 2)
		ret = max77660_reg_set_bits(chip->parent,
				MAX77660_CHG_SLAVE,
				MAX77660_CHARGER_USBCHGCTRL,
				MAX77660_USBCHGCTRL_USB_SUSPEND);
	else
		ret = max77660_reg_clr_bits(chip->parent,
				MAX77660_CHG_SLAVE,
				MAX77660_CHARGER_USBCHGCTRL,
				MAX77660_USBCHGCTRL_USB_SUSPEND);
	if (ret < 0)
		return ret;

	charger->in_current_lim = fchg_current(charger->in_current_lim);

/* unlock charger protection */
#ifdef CONFIG_MACH_PDA
	ret = max77660_reg_write(chip->parent, MAX77660_CHG_SLAVE,
		MAX77660_CHARGER_CHGCTRL1,
		0x17); /* 0x1F -> 0x17 THM_DIS disabled, JEITA_EN disabled, BUCK_EN enabled and CHGPROT unlocked */
#else
	ret = max77660_reg_write(chip->parent, MAX77660_CHG_SLAVE,
		MAX77660_CHARGER_CHGCTRL1,
		MAX77660_CHGCC_MASK);
#endif
	if (ret < 0)
		return ret;

#ifdef CONFIG_MACH_PDA
	ret = max77660_reg_read(chip->parent, MAX77660_CHG_SLAVE,
		MAX77660_CHARGER_CHGCTRL1, &read_val);
	if (ret < 0)
		return ret;
	pr_info("%s: CHGCTRL1:0x%02x\n", __func__, read_val);
	pr_info("%s: in_current_lim = %d enable( %d )\n", __func__,charger->in_current_lim, enable); //                       
#endif

	if (enable) {
#if defined(CONFIG_MACH_PD_A) || defined(CONFIG_MACH_PDA)
		if (charger->pmuotp == 0xC) {
			/* Set USB_HICURRENT (100mA > 500mA) */
			ret = max77660_reg_set_bits(max77660_ext->parent,
					MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_USBCHGCTRL,
					MAX77660_USBCHGCTRL_USB_HICURRENT);
			if (ret < 0) {
				pr_err("%s: set USB_HICURRENT fail\n", __func__);
				return ret;
			}
		}
		else if (charger->pmuotp == 0xB) {
			/*
                
                               
    */
#if 0
			/* Disable AICL : 0xC revision */
			ret = max77660_reg_set_bits(max77660_ext->parent,
					MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_AICLCNTL,
					MAX77660_AICLCNTL_DCMON_DIS);
			if (ret < 0) {
				pr_err("%s: disable AICL fail\n", __func__);
				return ret;
			}
			pr_info("%s: Set USB_HICURRENT and Disable AICL\n",
				       	__func__);
#else
			/* Enable AICL */
			ret = max77660_reg_set_bits(max77660_ext->parent,
					MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_AICLCNTL,
					MAX77660_AICLCNTL_ENABLE);
			if(ret < 0)
				pr_err("%s: enable AICL failed\n", __func__);
#endif //           
		}
#endif
		/* enable charger */
		/* Set DCILMT depends on charger */
		ret = max77660_reg_write(chip->parent,
				MAX77660_CHG_SLAVE,
				MAX77660_CHARGER_DCCRNT,
				charger->in_current_lim);
		if (ret < 0)
			return ret;
#if defined(CONFIG_MACH_PD_A) || defined(CONFIG_MACH_PDA)
		pr_info("%s: Set charging current (%dmA)\n",
			__func__,
			max77660_dccrnt_current_lookup[charger->in_current_lim]/10);

		/* Fast charge to 5 hours and fast charge current is set by DCCRNT */
		if (max77660_dccrnt_current_lookup[charger->in_current_lim]/10 >= 250)
			charger->fchg_current_limit = 0x00 + ((((max77660_dccrnt_current_lookup[charger->in_current_lim]/10) - 250)/50) + 1);
		else
			charger->fchg_current_limit = 0x01; //have default of 200mA

		if(!is_pseudo_batt()) charger->fchg_current_limit = charger->fchg_current_limit + 0xA0; // if pseudo_batt is not enabled, set timer fault register 8hours
		pr_info("%s: FAST Charging Current Limit 0x%02x)\n",__func__, charger->fchg_current_limit);

		if(charger->fchg_current_limit == 0x20) charger->fchg_current_limit = MAX77660_FCHG_CRNT_PSEUDO; /* pseudo_batt */
		else if(charger->fchg_current_limit == 0xC0) charger->fchg_current_limit = MAX77660_FCHG_CRNT; /* force to set 1.62A when 1835mA is inserted */
#endif


#ifdef CONFIG_MACH_PDA
		ret = max77660_reg_write(chip->parent,
				MAX77660_CHG_SLAVE, MAX77660_CHARGER_FCHGCRNT,
				charger->fchg_current_limit);
#else
		ret = max77660_reg_write(chip->parent,
				MAX77660_CHG_SLAVE, MAX77660_CHARGER_FCHGCRNT,
				MAX77660_FCHG_CRNT);
#endif
		if (ret < 0)
			return ret;

		ret = max77660_reg_read(chip->parent,
				MAX77660_CHG_SLAVE,
				MAX77660_CHARGER_FCHGCRNT, &read_val);
		if (ret < 0)
			return ret;

		/* Set TOPOFF to 0 min */
		ret = max77660_reg_write(chip->parent,
				MAX77660_CHG_SLAVE,
				MAX77660_CHARGER_TOPOFF,
				MAX77660_ITOPOFF_200MA |
				MAX77660_TOPOFFT_0MIN);
		if (ret < 0)
			return ret;
		/* MBATREG to 4.2V */
		/*                       */
		ret = max77660_reg_write(chip->parent,
				MAX77660_CHG_SLAVE,
				MAX77660_CHARGER_BATREGCTRL,
				MAX77660_MBATREG_4350MV);
		if (ret < 0)
			return ret;

		/* set MBATREGMAX voltage */
		ret = max77660_reg_write(chip->parent,
				MAX77660_CHG_SLAVE,
				MAX77660_CHARGER_MBATREGMAX,
				charger->max_term_vol_mV);
		if (ret < 0)
			return ret;


		/* DSILIM_EN = 1; CEN= 1; QBATEN = 0; VSYSREG = 3.6V */
		ret = max77660_reg_write(chip->parent,
				MAX77660_CHG_SLAVE,
				MAX77660_CHARGER_CHGCTRL2,
				MAX77660_VSYSREG_3600MV |
				MAX77660_CEN_MASK |
				MAX77660_PREQ_CURNT |
				MAX77660_DCILIM_EN_MASK);
		if (ret < 0)
			return ret;

		/* Enable register settings for charging*/
		ret = max77660_reg_write(chip->parent,
				MAX77660_CHG_SLAVE,
				MAX77660_CHARGER_CHGCCMAX,
				MAX77660_CHGCCMAX_CRNT);

		if (ret < 0)
			return ret;

		/* Enable top level charging */
		ret = max77660_reg_write(chip->parent, MAX77660_PWR_SLAVE,
				MAX77660_REG_GLOBAL_CFG1,
				MAX77660_GLBLCNFG1_MASK);
		if (ret < 0)
			return ret;
		ret = max77660_reg_write(chip->parent, MAX77660_PWR_SLAVE,
				MAX77660_REG_GLOBAL_CFG4,
				MAX77660_GLBLCNFG4_WDTC_SYS_CLR);
		if (ret < 0)
			return ret;

		ret = max77660_reg_write(chip->parent, MAX77660_PWR_SLAVE,
				MAX77660_REG_GLOBAL_CFG6,
				MAX77660_GLBLCNFG6_MASK);
		if (ret < 0)
			return ret;

	} else {
		ret = max77660_reg_write(chip->parent,
				MAX77660_CHG_SLAVE,
				MAX77660_CHARGER_DCCRNT,
				0);
		if (ret < 0)
			return ret;
		/* disable charge */
		/* Clear CEN */
		ret = max77660_reg_clr_bits(chip->parent,
			MAX77660_CHG_SLAVE, MAX77660_CHARGER_CHGCTRL2,
			MAX77660_CEN_MASK);
		if (ret < 0)
			return ret;
		/* Clear top level charge */
		ret = max77660_reg_clr_bits(chip->parent, MAX77660_PWR_SLAVE,
			MAX77660_REG_GLOBAL_CFG1, MAX77660_GLBLCNFG1_ENCHGTL);
		if (ret < 0)
			return ret;

	}

#ifdef CONFIG_MACH_PDA
	/* Lock CHGPROT */
	ret = max77660_reg_clr_bits(chip->parent,
			MAX77660_CHG_SLAVE,
			MAX77660_CHARGER_CHGCTRL1,
			MAX77660_CHARGER_CHGPROT_LOCK_BIT1|MAX77660_CHARGER_CHGPROT_LOCK_BIT0);
	if (ret < 0)
		return ret;
#if 0 // debug purpose
	ret = max77660_reg_read(chip->parent, MAX77660_CHG_SLAVE,
			MAX77660_CHARGER_CHGCTRL1, &read_val);
	if (ret < 0)
		return ret;
	pr_info("%s: CHGCTRL1:0x%02x\n", __func__, read_val);
#endif

#endif

	dev_info(chip->dev, "%s\n", (enable) ? "Enable charger" :
			"Disable charger");
	return 0;
}

int max77660_full_current_enable(struct max77660_chg_extcon *chip)
{
	int ret;

	pr_info("[POWER] max77660_full_current_enable!! \n");
	chip->charger->in_current_lim = chip->last_charging_current/1000;
	ret = max77660_charger_init(chip, true);
	if (ret < 0) {
		dev_err(chip->dev,
			"Failed to initialise full current charging\n");
		return ret;
	}

	chip->charging_state = ENABLED_FULL_IBAT;

	return 0;
}

int max77660_half_current_enable(struct max77660_chg_extcon *chip)
{
	int ret;
	pr_info("[POWER] max77660_half_current_enable!! \n");
	chip->charger->in_current_lim = chip->last_charging_current/2000;
	ret = max77660_charger_init(chip, true);
	if (ret < 0) {
		dev_err(chip->dev,
			"Failed to initialise full current charging\n");
		return ret;
	}
	chip->charging_state = ENABLED_HALF_IBAT;

	return 0;
}
#ifdef CONFIG_MACH_PDA
int max77660_450mA_current_enable(struct max77660_chg_extcon *chip)
{
	int ret;
	pr_info("[POWER] max77660_450mA_current_enable!! \n");
	//chip->charger->in_current_lim = chip->last_charging_current/2000;
	chip->charger->in_current_lim = 425;
	ret = max77660_charger_init(chip, true);
	if (ret < 0) {
		dev_err(chip->dev,
			"Failed to initialise full current charging\n");
		return ret;
	}
	chip->charging_state = ENABLED_LGE_OTP_450MA;

	return 0;
}
#endif

int max77660_charging_disable(struct max77660_chg_extcon *chip)
{
	int ret;
	pr_info("[POWER] max77660_charging_disable!! \n");
	ret = max77660_charger_init(chip, false);
	if (ret < 0) {
		dev_err(chip->dev,
			"Failed to disable charging\n");
		return ret;
	}
	chip->charging_state = DISABLED;

	return 0;
}

#ifdef CONFIG_MACH_PDA
int max77660_lge_charging_enable(void)
{
	int ret;

	pr_info("[POWER] max77660_lge_charging_enable!! \n");
	max77660_ext->charger->in_current_lim = 800; // set to 800mA
	ret = max77660_charger_init(max77660_ext, true);
	if (ret < 0) {
		dev_err(max77660_ext->dev,
			"Failed to initialise full current charging\n");
		return ret;
	}
	max77660_ext->charging_state = ENABLED_FULL_IBAT;
	battery_charging_status_update(max77660_ext->bc_dev,
						BATTERY_CHARGING);
	battery_charger_run_otp_scenario(max77660_ext->bc_dev);
	return 0;
}
EXPORT_SYMBOL(max77660_lge_charging_enable);

int max77660_lge_charging_disable(void)
{
	int ret;
	ret = max77660_charging_disable(max77660_ext);
	if (ret < 0) {
		dev_err(max77660_ext->dev,
			"Failed to disable charging\n");
		return ret;
	}
	battery_charging_status_update(max77660_ext->bc_dev,
						BATTERY_DISCHARGING);
	battery_charger_stop_otp_scenario(max77660_ext->bc_dev);
	return 0;
}
EXPORT_SYMBOL(max77660_lge_charging_disable);
#endif

static int max77660_set_charging_current(struct regulator_dev *rdev,
		int min_uA, int max_uA)
{
	struct max77660_chg_extcon *chip = rdev_get_drvdata(rdev);
	struct max77660_charger *charger = chip->charger;
	int ret;
	u8 status;
	ret = max77660_battery_detect(chip);
	if (ret < 0) {
		dev_err(chip->dev,
			"Battery detection failed\n");
		goto error;
	}

	chip->last_charging_current = max_uA;
	charger->in_current_lim = max_uA/1000;


	ret = max77660_reg_read(chip->parent, MAX77660_CHG_SLAVE,
			MAX77660_CHARGER_CHGSTAT, &status);
	if (ret < 0) {
		dev_err(chip->dev, "CHSTAT read failed: %d\n", ret);
		return ret;
	}
	if (charger->in_current_lim == 0 &&
			!(status & MAX77660_CHG_CHGINT_DC_UVP))
		return 0;

	if (charger->in_current_lim == 0) {
		chip->cable_connected = 0;
		ret = max77660_charging_disable(chip);
		if (ret < 0)
			goto error;
		charger->status = BATTERY_DISCHARGING;
		battery_charging_status_update(chip->bc_dev,
					BATTERY_DISCHARGING);
#ifdef CONFIG_MACH_PDA
		battery_charger_stop_otp_scenario(chip->bc_dev);
#else
		battery_charger_thermal_stop_monitoring(chip->bc_dev);
#endif

	} else {
		chip->cable_connected = 1;
		charger->status = BATTERY_CHARGING;
		ret = max77660_full_current_enable(chip);
		if (ret < 0)
			goto error;
		battery_charging_status_update(chip->bc_dev,
					BATTERY_CHARGING);
#ifdef CONFIG_MACH_PDA
		battery_charger_run_otp_scenario(chip->bc_dev);
		/*                                                                            */
		pr_info("[POWER-CHGMON] charging status work queue START!!! \n");
		schedule_delayed_work(&chip->chgmon_work,0);
		/*                                                                          */
#else
		battery_charger_thermal_start_monitoring(chip->bc_dev);
#endif

	}

	return 0;
error:
	return ret;
}

static struct regulator_ops max77660_charger_ops = {
	.set_current_limit = max77660_set_charging_current,
};

static int max77660_init_charger_regulator(struct max77660_chg_extcon *chip,
	struct max77660_bcharger_platform_data *bcharger_pdata)
{
	int ret = 0;

	if (!bcharger_pdata) {
		dev_err(chip->dev, "No charger platform data\n");
		return 0;
	}

	chip->chg_reg_desc.name  = "max77660-charger";
	chip->chg_reg_desc.ops   = &max77660_charger_ops;
	chip->chg_reg_desc.type  = REGULATOR_CURRENT;
	chip->chg_reg_desc.owner = THIS_MODULE;

	chip->chg_reg_init_data.supply_regulator     = NULL;
	chip->chg_reg_init_data.regulator_init	= NULL;
	chip->chg_reg_init_data.num_consumer_supplies =
				bcharger_pdata->num_consumer_supplies;
	chip->chg_reg_init_data.consumer_supplies    =
				bcharger_pdata->consumer_supplies;
	chip->chg_reg_init_data.driver_data	   = chip->charger;
	chip->chg_reg_init_data.constraints.name     = "max77660-charger";
	chip->chg_reg_init_data.constraints.min_uA   = 0;
	chip->chg_reg_init_data.constraints.max_uA   =
			bcharger_pdata->max_charge_current_mA * 1000;

	 chip->chg_reg_init_data.constraints.valid_modes_mask =
						REGULATOR_MODE_NORMAL |
						REGULATOR_MODE_STANDBY;

	chip->chg_reg_init_data.constraints.valid_ops_mask =
						REGULATOR_CHANGE_MODE |
						REGULATOR_CHANGE_STATUS |
						REGULATOR_CHANGE_CURRENT;

	chip->chg_rdev = regulator_register(&chip->chg_reg_desc,
				chip->dev, &chip->chg_reg_init_data,
				chip, NULL);
	if (IS_ERR(chip->chg_rdev)) {
		ret = PTR_ERR(chip->chg_rdev);
		dev_err(chip->dev,
			"vbus-charger regulator register failed %d\n", ret);
	}
	return ret;
}

static int max77660_chg_extcon_cable_update(
		struct max77660_chg_extcon *chg_extcon)
{
	int ret;
	u8 status;

	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
			MAX77660_CHARGER_CHGSTAT, &status);
	if (ret < 0) {
		dev_err(chg_extcon->dev, "CHSTAT read failed: %d\n", ret);
		return ret;
	}
	if ((status & MAX77660_CHG_CHGINT_DC_UVP) ||(status & MAX77660_CHG_CHGINT_DC_OVP))
		extcon_set_cable_state(chg_extcon->edev, "USB", false);
	else
		extcon_set_cable_state(chg_extcon->edev, "USB", true);

	dev_info(chg_extcon->dev, "VBUS UVP %s status: 0x%02x\n",
		(status & MAX77660_CHG_CHGINT_DC_UVP) ? "Invalid" : "Valid",
		status);

	dev_info(chg_extcon->dev, "VBUS OVP %s status: 0x%02x\n",
		(status & MAX77660_CHG_CHGINT_DC_OVP) ? "Invalid" : "Valid",
		status);

	return 0;
}

static int max77660_charger_detail_irq(int irq, void *data, u8 *val)
{
	struct max77660_chg_extcon *chip = (struct max77660_chg_extcon *)data;

	pr_info("[POWER] irq occur!! irq = %d \n", irq);
	switch (irq) {
	case MAX77660_CHG_BAT_I:
		switch ((val[2] & MAX77660_BAT_DTLS_MASK)
					>>MAX77660_BAT_DTLS_SHIFT) {
		case MAX77660_BAT_DTLS_BATDEAD:
			dev_info(chip->dev,
				"Battery Interrupt: Battery Dead\n");
			break;
		case MAX77660_BAT_DTLS_TIMER_FAULT:
			dev_info(chip->dev,
			"Battery Interrupt: Charger Watchdog Timer fault\n");
			break;
		case MAX77660_BAT_DTLS_BATOK:
			dev_info(chip->dev,
					"Battery Interrupt: Battery Ok\n");
			break;
		case MAX77660_BAT_DTLS_GTBATOVF:
			dev_info(chip->dev,
				"Battery Interrupt: GTBATOVF\n");
			break;
		default:
			dev_info(chip->dev,
				"Battery Interrupt: details-0x%x\n",
					(val[2] & MAX77660_BAT_DTLS_MASK));
			break;
		}
		break;

	case MAX77660_CHG_CHG_I:
		switch (val[2] & MAX77660_CHG_DTLS_MASK) {
		case MAX77660_CHG_DTLS_DEAD_BAT:
			dev_info(chip->dev,
			"Fast Charge current Interrupt: Dead Battery\n");

			break;
		case MAX77660_CHG_DTLS_PREQUAL:
			dev_info(chip->dev,
				"Fast Charge current Interrupt: PREQUAL\n");

			break;
		case MAX77660_CHG_DTLS_FAST_CHARGE_CC:
			dev_info(chip->dev,
			"Fast Charge current Interrupt: FAST_CHARGE_CC\n");

			break;
		case MAX77660_CHG_DTLS_FAST_CHARGE_CV:
			dev_info(chip->dev,
			"Fast Charge current Interrupt: FAST_CHARGE_CV\n");

			break;
		case MAX77660_CHG_DTLS_TOP_OFF:
			dev_info(chip->dev,
			"Fast Charge current Interrupt: TOP_OFF\n");

			break;
		case MAX77660_CHG_DTLS_DONE:
			dev_info(chip->dev,
			"Fast Charge current Interrupt: DONE\n");
			battery_charging_status_update(chip->bc_dev,
						BATTERY_CHARGING_DONE);
			//battery_charger_thermal_stop_monitoring(chip->bc_dev);
			if(is_pseudo_batt()) {
				battery_charging_restart(chip->bc_dev,
						chip->chg_restart_time_sec);
			}else { /* see if done irq is occured when soc is really high enough */
				battery_charging_restart(chip->bc_dev,
						10); /* 10 sec */
			}
			set_chg_done_flag(1); /*juya.kim -  set chg done flag */

			break;
		case MAX77660_CHG_DTLS_DONE_QBAT_ON:
			dev_info(chip->dev,
			"Fast Charge current Interrupt: DONE_QBAT_ON\n");

			break;
		case MAX77660_CHG_DTLS_TIMER_FAULT:
			dev_info(chip->dev,
			"Fast Charge current Interrupt: TIMER_FAULT\n");
#ifdef CONFIG_MACH_PDA
			if(is_pseudo_batt()) {
				schedule_delayed_work(&chip->chg_irq_work,0);
			}
#endif

			break;
		case MAX77660_CHG_DTLS_DC_INVALID:
			dev_info(chip->dev,
			"Fast Charge current Interrupt: DC_INVALID\n");

			break;
		case MAX77660_CHG_DTLS_THERMAL_LOOP_ACTIVE:
			dev_info(chip->dev,
			"Fast Charge current Interrupt:"
					"THERMAL_LOOP_ACTIVE\n");

			break;
		case MAX77660_CHG_DTLS_CHG_OFF:
			dev_info(chip->dev,
			"Fast Charge current Interrupt: CHG_OFF\n");

			break;
		default:
			dev_info(chip->dev,
			"Fast Charge current Interrupt: details-0x%x\n",
					(val[2] & MAX77660_CHG_DTLS_MASK));
			break;
		}
		break;

	case MAX77660_CHG_DC_UVP:
/*                                                                            */
#ifdef CONFIG_MACH_PDA
		dev_info(chip->dev,
			"DC Under voltage Interrupt: cable update!!\n");
#endif
/*                                                                          */
		max77660_chg_extcon_cable_update(chip);
		break;

	case MAX77660_CHG_DC_OVP:
		pr_info("[POWER] MAX77660_CHG_DC_OVP!!! \n");
		if ((val[1] & MAX77660_DC_OVP_MASK) == MAX77660_DC_OVP_MASK) {
			dev_info(chip->dev,
			"DC Over voltage Interrupt: VDC > VDC_OVLO\n");

			/*  VBUS is invalid. VDC > VDC_OVLO  */
		max77660_lge_charging_disable();
	} else if ((val[1] & MAX77660_DC_OVP_MASK) == 0) {
			dev_info(chip->dev,
			"DC Over voltage Interrupt: VDC < VDC_OVLO\n");

			/*  VBUS is valid. VDC < VDC_OVLO  */
			max77660_charger_init(chip, 1);
		}
		pr_info("[POWER] MAX77660_CHG_DC_OVP!!! Cable Update!!\n");
		max77660_chg_extcon_cable_update(chip);

		break;

	case MAX77660_CHG_DC_I:
		dev_info(chip->dev,
		"DC Input Current Limit Interrupt: details-0x%x\n",
				(val[1] & MAX77660_DC_I_MASK));
		break;

	case MAX77660_CHG_DC_V:
		dev_info(chip->dev,
			"DC Input Voltage Limit Interrupt: details-0x%x\n",
					(val[1] & MAX77660_DC_V_MASK));
		break;

	}
	return 0;
}


static irqreturn_t max77660_chg_extcon_irq(int irq, void *data)
{
	struct max77660_chg_extcon *chg_extcon = data;
	u8 irq_val = 0;
	u8 irq_mask = 0;
	u8 irq_name = 0;
/*                                                                            */
#ifdef CONFIG_MACH_PDA
	u8 val[11];
	int ret;
	val[0] = 0;
	val[1] = 0;
	val[2] = 0;
	val[3] = 0;
	val[4] = 0;
	val[5] = 0;
	val[6] = 0;
	val[7] = 0;
	val[8] = 0;
	val[9] = 0;
	val[10] = 0;
#else //original
	u8 val[3];
	int ret;

	val[0] = 0;
	val[1] = 0;
	val[2] = 0;

#endif

	pr_info("[POWER] irq occured!!, irq = %d !!! \n",irq);
/*                                                                          */
	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_CHGINT, &irq_val);
	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_CHGINTM, &irq_mask);

	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_CHGSTAT, &val[0]);
	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_DETAILS1, &val[1]);
	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_DETAILS2, &val[2]);

/*                                                                            */
#ifdef CONFIG_MACH_PDA
	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_AICLCNTL, &val[3]);
	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_CHGCTRL1, &val[4]);
	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_FCHG_CRNT, &val[5]);
	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_RBOOST, &val[6]);
	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_CHGCTRL2, &val[7]);
	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_USBCHGCTRL, &val[8]);
	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_FCHGCRNT, &val[9]);
	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_DCCRNT, &val[10]);
#endif
/*                                                                          */
#ifdef CONFIG_MACH_PDA
	dev_info(chg_extcon->dev,
			"%s: chgint:0x%02x, chgstat:0x%02x\n",
					__func__, (irq_val & ~irq_mask), val[0]);
	dev_info(chg_extcon->dev,
			"%s: detail1:0x%02x, detail2:0x%02x\n",
					__func__, val[1], val[2]);
#endif

	for (irq_name = MAX77660_CHG_BAT_I; irq_name < MAX77660_CHG_NR_IRQS;
								irq_name++) {
		if ((irq_val & (0x01<<(irq_name+2))) &&
				!(irq_mask & (0x01<<(irq_name+2))))
			max77660_charger_detail_irq(irq_name, data, val);
	}

	return IRQ_HANDLED;
}

static int max77660_charger_wdt(struct max77660_chg_extcon *chip)
{
	struct max77660_charger *charger = chip->charger;
	int i, ret = 0;

	for (i = 0; i < ARRAY_SIZE(max77660_chrg_wdt); ++i) {
		if (max77660_chrg_wdt[i] >= charger->wdt_timeout)
			break;
	}

	if (i == ARRAY_SIZE(max77660_chrg_wdt)) {
		dev_err(chip->dev, "Charger WDT %d sec is not supported\n",
			charger->wdt_timeout);
		return -EINVAL;
	}

	ret = max77660_reg_update(chip->parent, MAX77660_PWR_SLAVE,
			  MAX77660_REG_GLOBAL_CFG2,
			  MAX77660_GLBLCNFG2_TWD_CHG_MASK,
			  MAX77660_GLBLCNFG2_TWD_CHG(i));
	if (ret < 0) {
		dev_err(chip->dev,
			"GLOBAL_CFG2 update failed: %d\n", ret);
		return ret;
	}

	if (charger->wdt_timeout)
		ret = max77660_reg_set_bits(chip->parent, MAX77660_PWR_SLAVE,
			MAX77660_REG_GLOBAL_CFG1, MAX77660_GLBLCNFG1_ENCHGTL);
	else
		ret = max77660_reg_clr_bits(chip->parent, MAX77660_PWR_SLAVE,
			MAX77660_REG_GLOBAL_CFG1, MAX77660_GLBLCNFG1_ENCHGTL);
	if (ret < 0) {
		dev_err(chip->dev,
			"GLBLCNFG1_ENCHGTL update failed: %d\n", ret);
		 return ret;
	}
	return ret;
}

static irqreturn_t max77660_chg_wdt_irq(int irq, void *data)
{
	 struct max77660_chg_extcon *chip = (struct max77660_chg_extcon *)data;
	 int ret;

	ret = max77660_reg_write(chip->parent, MAX77660_PWR_SLAVE,
		MAX77660_REG_GLOBAL_CFG6,
		MAX77660_GLBLCNFG4_WDTC_SYS_CLR);
	if (ret < 0)
		dev_err(chip->dev, "GLOBAL_CFG4 update failed: %d\n", ret);

	 return IRQ_HANDLED;
}

static int max77660_vbus_enable_time(struct regulator_dev *rdev)
{
	 return 500;
}

static int max77660_vbus_is_enabled(struct regulator_dev *rdev)
{
	struct max77660_chg_extcon *chg_extcon =rdev_get_drvdata(rdev);
	int ret;
	u8 val;

	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_RBOOST, &val);
	if (ret < 0) {
		dev_err(chg_extcon->dev, "RBOOST read failed: %d\n", ret);
		return ret;
	}
	return !!(val & MAX77660_RBOOST_RBOOSTEN);
}

static int max77660_vbus_enable(struct regulator_dev *rdev)
{
	struct max77660_chg_extcon *chg_extcon =rdev_get_drvdata(rdev);
	int ret;

	ret = max77660_reg_update(chg_extcon->parent, MAX77660_CHG_SLAVE,
			MAX77660_CHARGER_RBOOST,
			MAX77660_RBOOST_RBOUT_VOUT(0x6),
			MAX77660_RBOOST_RBOUT_MASK);

	if (ret < 0) {
		dev_err(chg_extcon->dev, "RBOOST update failed: %d\n", ret);
		return ret;
	}

	ret = max77660_reg_set_bits(chg_extcon->parent, MAX77660_CHG_SLAVE,
			MAX77660_CHARGER_RBOOST, MAX77660_RBOOST_RBOOSTEN);
	if (ret < 0)
		dev_err(chg_extcon->dev, "RBOOST setbits failed: %d\n", ret);
	return ret;
}

static int max77660_vbus_disable(struct regulator_dev *rdev)
{
	struct max77660_chg_extcon *chg_extcon =rdev_get_drvdata(rdev);
	 int ret;

	ret = max77660_reg_clr_bits(chg_extcon->parent, MAX77660_CHG_SLAVE,
			MAX77660_CHARGER_RBOOST, MAX77660_RBOOST_RBOOSTEN);
	if (ret < 0)
		dev_err(chg_extcon->dev, "RBOOST clrbits failed: %d\n", ret);
	 return ret;
}

static struct regulator_ops max77660_vbus_ops = {
	.enable		= max77660_vbus_enable,
	.disable	= max77660_vbus_disable,
	.is_enabled	= max77660_vbus_is_enabled,
	.enable_time	= max77660_vbus_enable_time,
};

static int max77660_init_vbus_regulator(struct max77660_chg_extcon *chg_extcon,
		struct max77660_vbus_platform_data *vbus_pdata)
{
	int ret = 0;

	if (!vbus_pdata) {
		dev_err(chg_extcon->dev, "No vbus platform data\n");
		return 0;
	}

	chg_extcon->vbus_reg_desc.name = "max77660-vbus";
	chg_extcon->vbus_reg_desc.ops = &max77660_vbus_ops;
	chg_extcon->vbus_reg_desc.type = REGULATOR_VOLTAGE;
	chg_extcon->vbus_reg_desc.owner = THIS_MODULE;

	chg_extcon->vbus_reg_init_data.supply_regulator    = NULL;
	chg_extcon->vbus_reg_init_data.regulator_init      = NULL;
	chg_extcon->vbus_reg_init_data.num_consumer_supplies       =
					vbus_pdata->num_consumer_supplies;
	chg_extcon->vbus_reg_init_data.consumer_supplies   =
					vbus_pdata->consumer_supplies;
	chg_extcon->vbus_reg_init_data.driver_data	 = chg_extcon;

	chg_extcon->vbus_reg_init_data.constraints.name    = "max77660-vbus";
	chg_extcon->vbus_reg_init_data.constraints.min_uV  = 0;
	chg_extcon->vbus_reg_init_data.constraints.max_uV  = 5000000,
	chg_extcon->vbus_reg_init_data.constraints.valid_modes_mask =
					REGULATOR_MODE_NORMAL |
					REGULATOR_MODE_STANDBY;
	chg_extcon->vbus_reg_init_data.constraints.valid_ops_mask =
					REGULATOR_CHANGE_MODE |
					REGULATOR_CHANGE_STATUS |
					REGULATOR_CHANGE_VOLTAGE;

	chg_extcon->vbus_rdev = regulator_register(&chg_extcon->vbus_reg_desc,
				chg_extcon->dev,
				&chg_extcon->vbus_reg_init_data,
				chg_extcon, NULL);
	if (IS_ERR(chg_extcon->vbus_rdev)) {
		ret = PTR_ERR(chg_extcon->vbus_rdev);
		dev_err(chg_extcon->dev, "Failed to register VBUS regulator: %d\n",
					ret);
		return ret;
	}
	return ret;
}

static int max77660_charger_get_status(struct battery_charger_dev *bc_dev)
{
	struct max77660_chg_extcon *chip = battery_charger_get_drvdata(bc_dev);
	struct max77660_charger *charger = chip->charger;

	return charger->status;
}

static int max77660_charger_thermal_configure(
		struct battery_charger_dev *bc_dev,
		int temp, bool enable_charger, bool enable_charg_half_current,
		int battery_voltage)
{
	struct max77660_chg_extcon *chip = battery_charger_get_drvdata(bc_dev);
	int temperature;
	int battery_threshold_voltage;
	int ret;

	if (!chip->cable_connected)
		return 0;

	temperature = temp;
	dev_info(chip->dev, "Battery temp %d\n", temperature);
	if (enable_charger) {
		if (!enable_charg_half_current &&
			chip->charging_state != ENABLED_FULL_IBAT) {
			max77660_full_current_enable(chip);
			battery_charging_status_update(chip->bc_dev,
				BATTERY_CHARGING);
		} else if (enable_charg_half_current &&
			chip->charging_state != ENABLED_HALF_IBAT) {
			max77660_half_current_enable(chip);
			/*Set MBATREG voltage */
			battery_threshold_voltage =
					convert_to_reg(battery_voltage);
			ret = max77660_reg_write(chip->parent,
					MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_BATREGCTRL,
					(battery_threshold_voltage << 1));
			if (ret < 0)
				return ret;
			battery_charging_status_update(chip->bc_dev,
							BATTERY_CHARGING);
		}
	} else {
		if (chip->charging_state != DISABLED) {
			max77660_charging_disable(chip);
			battery_charging_status_update(chip->bc_dev,
						BATTERY_DISCHARGING);
		}
	}
	return 0;
}

#ifdef CONFIG_MACH_PDA
int max77660_is_chg_enabled(void)
{
	int ret;
	u8 val;

	printk("%s is called\n",__func__);
	ret = max77660_reg_read(max77660_ext->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_CHGSTAT, &val);

	if (ret < 0) {
		dev_err(max77660_ext->dev, "CHGSTAT read failed: %d\n", ret);
		return ret;
	}
	printk("%s : 0x%x\n",__func__,val);
	if (val & 0x10) return 0;
	return 1;
}
EXPORT_SYMBOL(max77660_is_chg_enabled);

static int max77660_charger_lge_otp_scenario(
		struct battery_charger_dev *bc_dev,
		int temp, bool is_chg_enabled, bool decrease_chg,
		bool is_powerOff)
{
	struct max77660_chg_extcon *chip = battery_charger_get_drvdata(bc_dev);
	int temperature;
	//int ret;

	if (!chip->cable_connected)
		return 0;

	temperature = temp;
	dev_dbg(chip->dev, "[OTP] Battery temp %d\n", temperature);

	if(is_chg_enabled) {
		if(!decrease_chg &&
			chip->charging_state != ENABLED_FULL_IBAT) {
			max77660_full_current_enable(chip);
			battery_charging_status_update(chip->bc_dev,
				BATTERY_CHARGING);
			dev_info(chip->dev, "[OTP] Full Charging Area\n");
		} else if (decrease_chg &&
			chip->charging_state != ENABLED_LGE_OTP_450MA) {
			max77660_450mA_current_enable(chip);
			battery_charging_status_update(chip->bc_dev,
				BATTERY_CHARGING);
			dev_info(chip->dev, "[OTP] 450mA Charging Area\n");
		}
	}else {
		if(chip->charging_state != DISABLED) {
			max77660_charging_disable(chip);
			battery_charging_status_update(chip->bc_dev,
						BATTERY_DISCHARGING);
			dev_info(chip->dev, "[OTP] Discharging Area\n");
		}
		if(is_powerOff == true) {
		// do power off
			dev_info(chip->dev, "[OTP] Power off temperture Over 60  \n");
		}
    }
	return 0;
}
#endif

static int max77660_charging_restart(struct battery_charger_dev *bc_dev)
{
	struct max77660_chg_extcon *chip = battery_charger_get_drvdata(bc_dev);
	int ret;

	if (!chip->cable_connected)
		return 0;

	dev_info(chip->dev, "Restarting the charging\n");
	ret = max77660_set_charging_current(chip->chg_rdev,
			chip->last_charging_current,
			chip->last_charging_current);
	if (ret < 0) {
		dev_err(chip->dev, "restarting of charging failed: %d\n", ret);
		battery_charging_restart(chip->bc_dev,
				chip->chg_restart_time_sec);
	}
	return ret;
}

static int max77660_init_oc_alert(struct max77660_chg_extcon *chip)
{
	int ret;
	u8 octh;

	octh = chip->charger->bcharger_pdata->oc_thresh;

	if (octh >= OC_THRESH_DIS) {
		ret = max77660_reg_clr_bits(chip->parent, MAX77660_CHG_SLAVE,
				MAX77660_CHARGER_BAT2SYS,
				MAX77660_CHARGER_BAT2SYS_OCEN);
		if (ret < 0)
			dev_err(chip->dev, "BAT2SYS update failed: %d\n", ret);
		return ret;
	}

	ret = max77660_reg_set_bits(chip->parent, MAX77660_PWR_SLAVE,
			MAX77660_REG_GLOBAL_CFG1,
			MAX77660_GLBLCNFG1_ENPGOC);
	if (ret < 0) {
		dev_err(chip->dev, "GLBLCNFG1 update failed: %d\n", ret);
		return ret;
	}

	ret = max77660_reg_update(chip->parent,
			MAX77660_CHG_SLAVE, MAX77660_CHARGER_BAT2SYS,
			MAX77660_CHARGER_BAT2SYS_OCEN |
			MAX77660_CHARGER_BAT2SYS_OC_MASK,
			MAX77660_CHARGER_BAT2SYS_OCEN | octh);
	if (ret < 0)
		dev_err(chip->dev, "BAT2SYS update failed: %d\n", ret);
	return ret;
}

static struct battery_charging_ops max77660_charger_bci_ops = {
	.get_charging_status = max77660_charger_get_status,
	.thermal_configure = max77660_charger_thermal_configure,
	.restart_charging = max77660_charging_restart,
#ifdef CONFIG_MACH_PDA
	.lge_otp_scenario = max77660_charger_lge_otp_scenario,
#endif
};

static struct battery_charger_info max77660_charger_bci = {
	.cell_id = 0,
	.bc_ops = &max77660_charger_bci_ops,
};

#ifdef CONFIG_DEBUG_FS
#if defined(CONFIG_MACH_PD_A) || defined(CONFIG_MACH_PDA)
static struct dentry *ext_debugfs_root;

static int extcon_stats_show(struct seq_file *s, void *data)
{
	int ret;
	u8 val=0;

	seq_printf(s, "----------------------------------------------\n");
	seq_printf(s, "extconn: chg_irq: %d\n",
		max77660_ext->chg_irq);
	seq_printf(s, "extconn: chg_wdt_irq: %d\n",
		max77660_ext->chg_wdt_irq);
	seq_printf(s, "extconn: cable_connected: %d\n",
		max77660_ext->cable_connected);
	seq_printf(s, "extconn: charging_state: %d\n",
		max77660_ext->charging_state);
	seq_printf(s, "extconn: cable_connected_state: %d\n",
		max77660_ext->cable_connected_state);
	seq_printf(s, "extconn: battery_present: %d\n",
		max77660_ext->battery_present);
	seq_printf(s, "extconn: chg_restart_time_sec: %d\n",
		max77660_ext->chg_restart_time_sec);
	seq_printf(s, "extconn: last_charging_current: %d\n",
		max77660_ext->last_charging_current);
	seq_printf(s, "----------------------------------------------\n");
	seq_printf(s, "extconn_chg: irq: %d\n",
		max77660_ext->charger->irq);
	seq_printf(s, "extconn_chg: status: %d\n",
		max77660_ext->charger->status);
	seq_printf(s, "extconn_chg: in_current_lim: %d\n",
		max77660_ext->charger->in_current_lim);
	seq_printf(s, "extconn_chg: wdt_timeout: %d\n",
		max77660_ext->charger->wdt_timeout);
	seq_printf(s, "extconn_chg: max_term_vol_mV: %d\n",
		max77660_ext->charger->max_term_vol_mV);
	seq_printf(s, "----------------------------------------------\n");
	seq_printf(s, "PMU OTP   : %02x\n", max77660_ext->charger->pmuotp);
	if (max77660_ext->charger->pmuotp == 0xC) {
		ret = max77660_reg_read(max77660_ext->parent, MAX77660_CHG_SLAVE,
				MAX77660_CHARGER_USBCHGCTRL, &val);
		seq_printf(s, "USBCHGCTRL: %02x %02x\n", MAX77660_CHARGER_USBCHGCTRL, val);
		ret = max77660_reg_read(max77660_ext->parent, MAX77660_CHG_SLAVE,
				MAX77660_CHARGER_AICLCNTL, &val);
		seq_printf(s, "AICLCNTL  : %02x %02x\n", MAX77660_CHARGER_AICLCNTL, val);
	}
	seq_printf(s, "----------------------------------------------\n");

	return 0;
}

static int extcon_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, extcon_stats_show, inode->i_private);
}

static const struct file_operations extcon_stats_fops = {
	.open 		= extcon_stats_open,
	.read 		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/* /d/max77660-extcon/status */
static void max77660_chg_extcon_init_debugfs(void)
{
	ext_debugfs_root = debugfs_create_dir(max77660_ext->edev->name, NULL);
	if (!ext_debugfs_root) {
		pr_err("%s: Failed to create debugfs directory\n", __func__);
		return;
	}

	debugfs_create_file("status", 0644, ext_debugfs_root, NULL, &extcon_stats_fops);
}
#endif /* CONFIG_MACH_PDA */
#endif /* CONFIG_DEBUG_FS */

/*                                                                            */
#ifdef CONFIG_MACH_PDA
static int dcilim_en_counter = 0;

static void lge_chgmon_work(struct work_struct *work)
{

	struct max77660_chg_extcon *chg_extcon;
	u8 val=0;
	int ret;

	chg_extcon = container_of(work, struct max77660_chg_extcon,
					chgmon_work.work);

	if(chg_extcon->charger->in_current_lim == 0){
		pr_info("[POWER-CHGMON] in_current_lim!! so lge_chgmon_work STOP!! \n");
		return;
	}

	//u8 val[10];

/*
	val[0] = 0;
	val[1] = 0;
	val[2] = 0;
	val[3] = 0;
	val[4] = 0;
	val[5] = 0;
	val[6] = 0;
	val[7] = 0;
	val[8] = 0;
	val[9] = 0;
*/

/*
	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_CHGSTAT, &val[0]);
	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_DETAILS1, &val[1]);
	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_DETAILS2, &val[2]);

	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_AICLCNTL, &val[3]);
	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_CHGCTRL1, &val[4]);

	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_RBOOST, &val[5]);
	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_CHGCTRL2, &val[6]);

	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_USBCHGCTRL, &val[7]);
	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_FCHGCRNT, &val[8]);

    ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_DCCRNT, &val[9]);

*/
	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_CHGCTRL2, &val);

	if(!(val&MAX77660_DCILIM_EN_MASK)){
		pr_info("[POWER-CHGMON] DCILIM_EN:0, counter is %d\n",dcilim_en_counter);

		dcilim_en_counter++;

		if(dcilim_en_counter>3){
			chg_extcon->charger->in_current_lim = chg_extcon->last_charging_current/1000;
			pr_info("[POWER-CHGMON] charging init AGAIN !!!, in_current_lim=%d \n",chg_extcon->charger->in_current_lim);

			ret = max77660_charger_init(chg_extcon,1);
			if(ret < 0)
				pr_err("[POWER-CHGMON] charging init AGAIN Failed.\n");

		dcilim_en_counter = 0;
		}
	}

	//pr_info("[POWER-CHGMON] cst:0x%02x,det1:0x%02x,det2:0x%02x,acl:0x%2x,fcrt:0x%2x, dcrt:0x%2x\n", val[0],val[1], val[2],val[3],val[8], val[9]);
	//pr_info("[POWER-CHGMON] uctl:0x%2x ctl1:0x%2x, ctl2:0x%2x, rbst:0x%2x \n",val[7], val[4], val[6], val[5]);
	  pr_info("[POWER-CHGMON] ctl2:0x%2x \n",val);
	if(dcilim_en_counter>0){
		if(is_pseudo_batt()) {
			schedule_delayed_work(&chg_extcon->chgmon_work,
				msecs_to_jiffies(LGE_CHG_MON_TIME_ABNORMAL_PSEUDO));
		} else {
			schedule_delayed_work(&chg_extcon->chgmon_work,
				msecs_to_jiffies(LGE_CHG_MON_TIME_ABNORMAL));
		}
	}else{
		if(is_pseudo_batt()) {
			schedule_delayed_work(&chg_extcon->chgmon_work,
				msecs_to_jiffies(LGE_CHG_MON_TIME_NORMAL_PSEUDO));
		} else {
			schedule_delayed_work(&chg_extcon->chgmon_work,
						msecs_to_jiffies(LGE_CHG_MON_TIME_NORMAL));
		}
	}
	return;
}

static void lge_irq_work(struct work_struct *work) {

	int ret = 0;
	u8 val[2] = {0,};
	struct max77660_chg_extcon *chg_extcon;

	chg_extcon = container_of(work, struct max77660_chg_extcon,
					chg_irq_work.work);

	if(!chg_extcon->cable_connected) return;

	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_CHGCTRL1, &val[0]);

	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_DETAILS2, &val[1]);

	printk("[PIPAS] tasklet run :: details2=0x%x, ctrl1=0x%x\n",val[1],val[0]);
	if(val[1] == 0x27 && val[0] == 0x0C) {
		printk("[PIPAS] Restarting the charging\n");
		ret = max77660_set_charging_current(chg_extcon->chg_rdev,
				chg_extcon->last_charging_current,
				chg_extcon->last_charging_current);
		if (ret < 0) {
			printk("[PIPAS] max77660_set_charging_current error\n");
			return;
		}
	}
	return;
}
#endif
/*                                                                          */
static int __devinit max77660_chg_extcon_probe(struct platform_device *pdev)
{
	struct max77660_chg_extcon *chg_extcon;
	struct max77660_platform_data *pdata;
	struct max77660_charger_platform_data *chg_pdata;
	struct max77660_bcharger_platform_data *bcharger_pdata;
	struct max77660_vbus_platform_data *vbus_pdata;
	struct extcon_dev *edev;
	struct max77660_charger *charger;
	int ret;
#if defined(CONFIG_MACH_PD_A) || defined(CONFIG_MACH_PDA)
	u8 val;
#endif

	pdata = dev_get_platdata(pdev->dev.parent);
	if (!pdata || !pdata->charger_pdata) {
		dev_err(&pdev->dev, "No platform data\n");
		return -ENODEV;
	}

	chg_pdata = pdata->charger_pdata;
	bcharger_pdata = chg_pdata->bcharger_pdata;
	vbus_pdata = chg_pdata->vbus_pdata;

	chg_extcon = devm_kzalloc(&pdev->dev, sizeof(*chg_extcon), GFP_KERNEL);
	if (!chg_extcon) {
		dev_err(&pdev->dev, "Memory allocation failed for chg_extcon\n");
		return -ENOMEM;
	}

	edev = devm_kzalloc(&pdev->dev, sizeof(*edev), GFP_KERNEL);
	if (!edev) {
		dev_err(&pdev->dev, "Memory allocation failed for edev\n");
		return -ENOMEM;
	}

	chg_extcon->charger = devm_kzalloc(&pdev->dev,
				sizeof(*(chg_extcon->charger)), GFP_KERNEL);
	if (!chg_extcon->charger) {
		dev_err(&pdev->dev, "Memory allocation failed for charger\n");
		return -ENOMEM;
	}

	charger = chg_extcon->charger;
	charger->status = BATTERY_DISCHARGING;
	charger->bcharger_pdata = bcharger_pdata;

	chg_extcon->edev = edev;
	chg_extcon->edev->name = (chg_pdata->ext_conn_name) ?
					chg_pdata->ext_conn_name :
					dev_name(&pdev->dev);
	chg_extcon->edev->supported_cable = max77660_excon_cable;


	chg_extcon->dev = &pdev->dev;
	chg_extcon->parent = pdev->dev.parent;
	dev_set_drvdata(&pdev->dev, chg_extcon);

	chg_extcon->chg_irq = platform_get_irq(pdev, 0);

	chg_extcon->chg_wdt_irq = platform_get_irq(pdev, 1);
	max77660_ext = chg_extcon;

	ret = extcon_dev_register(chg_extcon->edev, NULL);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register extcon device\n");
		return ret;
	}

	/* Set initial state */
	ret = max77660_chg_extcon_cable_update(chg_extcon);
	if (ret < 0) {
		dev_err(&pdev->dev, "Cable init failed: %d\n", ret);
		goto extcon_free;
	}

	ret = request_threaded_irq(chg_extcon->chg_irq, NULL,
		max77660_chg_extcon_irq,
		IRQF_ONESHOT | IRQF_EARLY_RESUME, "max77660-charger",
		chg_extcon);
	if (ret < 0) {
		dev_err(chg_extcon->dev,
			"request irq %d failed: %dn", chg_extcon->chg_irq, ret);
		goto extcon_free;
	}

	ret = max77660_init_vbus_regulator(chg_extcon, vbus_pdata);
	if (ret < 0) {
		dev_err(chg_extcon->dev, "Vbus regulator init failed %d\n", ret);
		goto chg_irq_free;
	}

	if (!bcharger_pdata) {
		dev_info(chg_extcon->dev,
			"Battery not connected, charging not supported\n");
		max77660_reg_clr_bits(chg_extcon->parent, MAX77660_CHG_SLAVE,
				MAX77660_CHARGER_CHGCTRL1,
				MAX77660_CHARGER_BUCK_EN_MASK);

		goto skip_bcharger_init;
	}

	ret = max77660_reg_clr_bits(chg_extcon->parent, MAX77660_CHG_SLAVE,
			MAX77660_CHARGER_CHGINTM, MAX77660_CHG_CHGINT_DC_UVP|MAX77660_DCOVP_MASK|
			MAX77660_CHG_CHGINT_CHG_M);
	if (ret < 0) {
		dev_err(chg_extcon->dev, "CHGINTM update failed: %d\n", ret);
		goto extcon_free;
	}

	if(bcharger_pdata->max_term_vol_mV)
		charger->max_term_vol_mV = convert_to_reg(bcharger_pdata->max_term_vol_mV);
	else
		charger->max_term_vol_mV = convert_to_reg(MAX77660_MBATREGMAX_4300MV);

	chg_extcon->battery_present = true;
	charger->wdt_timeout = bcharger_pdata->wdt_timeout;

	ret = request_threaded_irq(chg_extcon->chg_wdt_irq, NULL,
		max77660_chg_wdt_irq,
		IRQF_ONESHOT | IRQF_EARLY_RESUME, "max77660-charger-wdt",
		chg_extcon);
	if (ret < 0) {
		dev_err(chg_extcon->dev, "request irq %d failed: %d\n",
			chg_extcon->chg_wdt_irq, ret);
		goto vbus_reg_err;
	}

	ret = max77660_init_charger_regulator(chg_extcon, bcharger_pdata);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register charger regulator: %d\n",
					ret);
		goto wdt_irq_free;
	}

	ret = max77660_charger_wdt(chg_extcon);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Charger watchdog timer init failed %d\n", ret);
		goto chg_reg_err;
	}

	ret = max77660_init_oc_alert(chg_extcon);
	if (ret < 0) {
		dev_err(&pdev->dev, "OC init failed: %d\n", ret);
		goto chg_reg_err;
	}

#if defined(CONFIG_MACH_PD_A) || defined(CONFIG_MACH_PDA)
	ret = max77660_reg_read(max77660_ext->parent, MAX77660_PWR_SLAVE,
			MAX77660_REG_CID4, &val);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"PMU OTP read failed %d\n", ret);
		goto chg_reg_err;
	}
	charger->pmuotp = val;
#endif

	/* Charging auto restart time */
	chg_extcon->chg_restart_time_sec = MAX77660_RESTART_CHARGING_AFTER_DONE;
	max77660_charger_bci.polling_time_sec =
			bcharger_pdata->temperature_poll_period_secs;
	max77660_charger_bci.tz_name = bcharger_pdata->tz_name;
#ifdef CONFIG_MACH_PDA
    max77660_charger_bci.lge_otp_interval = bcharger_pdata->lge_otp_interval;
#endif
	chg_extcon->bc_dev = battery_charger_register(&pdev->dev,
					&max77660_charger_bci, chg_extcon);
	if (IS_ERR(chg_extcon->bc_dev)) {
		ret = PTR_ERR(chg_extcon->bc_dev);
		dev_err(&pdev->dev, "battery charger register failed: %d\n",
			ret);
		goto chg_reg_err;
	}
/*                                                                            */
#ifdef CONFIG_MACH_PDA
   INIT_DELAYED_WORK(&chg_extcon->chgmon_work,lge_chgmon_work);
   INIT_DELAYED_WORK(&chg_extcon->chg_irq_work,lge_irq_work);

#endif
/*                                                                          */

skip_bcharger_init:
	device_set_wakeup_capable(&pdev->dev, 1);
#ifdef CONFIG_DEBUG_FS
	max77660_chg_extcon_init_debugfs();
#endif
	return 0;

chg_reg_err:
	regulator_unregister(chg_extcon->chg_rdev);
wdt_irq_free:
	free_irq(chg_extcon->chg_wdt_irq, chg_extcon);
vbus_reg_err:
	regulator_unregister(chg_extcon->vbus_rdev);
chg_irq_free:
	free_irq(chg_extcon->chg_irq, chg_extcon);
extcon_free:
	extcon_dev_unregister(chg_extcon->edev);
	return ret;
}

static int __devexit max77660_chg_extcon_remove(struct platform_device *pdev)
{
	struct max77660_chg_extcon *chg_extcon = dev_get_drvdata(&pdev->dev);

	free_irq(chg_extcon->chg_irq, chg_extcon);
	regulator_unregister(chg_extcon->vbus_rdev);
	if (chg_extcon->battery_present) {
		battery_charger_unregister(chg_extcon->bc_dev);
		extcon_dev_unregister(chg_extcon->edev);
		free_irq(chg_extcon->chg_wdt_irq, chg_extcon);
	}
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77660_chg_extcon_suspend(struct device *dev)
{
	struct max77660_chg_extcon *chg_extcon = dev_get_drvdata(dev);
	int ret;

	if (device_may_wakeup(dev)) {
		enable_irq_wake(chg_extcon->chg_irq);
		if (chg_extcon->battery_present &&
				chg_extcon->charger->wdt_timeout)
			enable_irq_wake(chg_extcon->chg_wdt_irq);
	} else {
		if (chg_extcon->battery_present &&
				chg_extcon->charger->wdt_timeout) {
			ret = max77660_reg_clr_bits(chg_extcon->parent,
				MAX77660_PWR_SLAVE,
				MAX77660_REG_GLOBAL_CFG1,
				MAX77660_GLBLCNFG1_ENCHGTL);
			if (ret < 0)
				dev_err(chg_extcon->dev,
					"GLBLCNFG1_ENCHGTL update failed: %d\n",
					 ret);
		}
	}
	return 0;
}

static int max77660_chg_extcon_resume(struct device *dev)
{
	struct max77660_chg_extcon *chg_extcon = dev_get_drvdata(dev);
	int ret;

	if (device_may_wakeup(dev)) {
		disable_irq_wake(chg_extcon->chg_irq);
		if (chg_extcon->battery_present &&
			chg_extcon->charger->wdt_timeout)
			disable_irq_wake(chg_extcon->chg_wdt_irq);
	} else {
		if (chg_extcon->battery_present &&
			chg_extcon->charger->wdt_timeout) {
			ret = max77660_reg_set_bits(chg_extcon->parent,
				MAX77660_PWR_SLAVE,
				MAX77660_REG_GLOBAL_CFG1,
				MAX77660_GLBLCNFG1_ENCHGTL);
			if (ret < 0)
				dev_err(chg_extcon->dev,
					"GLBLCNFG1_ENCHGTL update failed: %d\n",
					 ret);
		}
	}
	return 0;
};
#endif

static const struct dev_pm_ops max77660_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(max77660_chg_extcon_suspend,
				max77660_chg_extcon_resume)
};

static struct platform_driver max77660_chg_extcon_driver = {
	.probe = max77660_chg_extcon_probe,
	.remove = __devexit_p(max77660_chg_extcon_remove),
	.driver = {
		.name = "max77660-charger-extcon",
		.owner = THIS_MODULE,
		.pm = &max77660_pm_ops,
	},
};

static int __init max77660_chg_extcon_driver_init(void)
{
	return platform_driver_register(&max77660_chg_extcon_driver);
}
subsys_initcall_sync(max77660_chg_extcon_driver_init);

static void __exit max77660_chg_extcon_driver_exit(void)
{
	platform_driver_unregister(&max77660_chg_extcon_driver);
}
module_exit(max77660_chg_extcon_driver_exit);

MODULE_DESCRIPTION("max77660 charger-extcon driver");
MODULE_AUTHOR("Darbha Sriharsha<dsriharsha@nvidia.com>");
MODULE_AUTHOR("Syed Rafiuddin<srafiuddin@nvidia.com>");
MODULE_AUTHOR("Laxman Dewangan<ldewangan@nvidia.com>");
MODULE_ALIAS("platform:max77660-charger-extcon");
MODULE_LICENSE("GPL v2");
