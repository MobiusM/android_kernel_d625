/*
 *  max17048_battery.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
 *  Chandler Zhang <chazhang@nvidia.com>
 *  Syed Rafiuddin <srafiuddin@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/unaligned.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/max17048_battery.h>
#include <linux/power/battery-charger-gauge-comm.h>
#include <linux/pm.h>
#include <linux/jiffies.h>
#include <mach/board_lge.h>

#define MAX17048_VCELL		0x02
#define MAX17048_SOC		0x04
#define MAX17048_VER		0x08
#define MAX17048_HIBRT		0x0A
#define MAX17048_CONFIG		0x0C
#define MAX17048_OCV		0x0E
#define MAX17048_VALRT		0x14
#define MAX17048_VRESET		0x18
#define MAX17048_STATUS		0x1A
#define MAX17048_UNLOCK		0x3E
#define MAX17048_TABLE		0x40
#define MAX17048_RCOMPSEG1	0x80
#define MAX17048_RCOMPSEG2	0x90
#define MAX17048_CMD		0xFF
#define MAX17048_UNLOCK_VALUE	0x4a57
#define MAX17048_RESET_VALUE	0x5400
#define MAX17048_DELAY		(30*HZ)
#define MAX17048_LGE_DELAY_SLOW	(25*HZ)
#define MAX17048_LGE_DELAY		(20*HZ)
#define MAX17048_LGE_DELAY_LOW_SOC	(15*HZ)
#define MAX17048_LGE_DELAY_CRITIAL	(10*HZ)
#define MAX17048_LGE_DELAY_LOW		(1*HZ)

#define MAX17048_BATTERY_FULL	100
#define MAX17048_BATTERY_LOW	15
#define MAX17048_VERSION_NO	0x11

struct max17048_chip {
	struct i2c_client		*client;
	struct delayed_work		work;
	struct power_supply		battery;
	struct max17048_platform_data *pdata;
	struct battery_gauge_dev	*bg_dev;

	/* battery voltage */
	int vcell;
	/* battery capacity */
	int soc;
	/* State Of Charge */
	int raw_soc;
	/* Raw soc value read from register */
	int status;
	/* battery health */
	int health;
	/* battery capacity */
	int capacity_level;

	int lasttime_soc;
	int lasttime_status;
	int shutdown_complete;
	int charge_complete;
	struct mutex mutex;
#ifdef CONFIG_MACH_PDA /* temperature compensation */
	u8 startingRcomp;
	int temp_co_hot;
	int temp_co_cold;
	int curr_temp;
	int prev_temp;
#endif
};
struct max17048_chip *max17048_data;

/*                                                                          */
#ifdef CONFIG_MACH_PDA
static struct pseudo_batt_info_type pseudo_batt_info = {
	.mode = 0,
};
/*                                                              */
static int cnt_soc = 0;
#define LGE_EMPTY_SOC 2
#endif
/*                                          */

/*                                                                           */
 #ifdef CONFIG_MACH_PDA
int is_battery_valid(void);
static int max17048_update_rcomp_by_temperature(struct max17048_chip *chip);
#endif
/*                                                                         */

/*                                                                           */
 #ifdef CONFIG_MACH_PDA
static int get_prop_batt_present(void);
extern int is_chg_done_irq(void);
extern void set_chg_done_flag(int enable);
#endif
/*                                                                         */

static int max17048_write_word(struct i2c_client *client, int reg, u16 value)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&chip->mutex);
	if (chip && chip->shutdown_complete) {
		mutex_unlock(&chip->mutex);
		return -ENODEV;
	}


	ret = i2c_smbus_write_word_data(client, reg, swab16(value));

	if (ret < 0)
		dev_err(&client->dev, "%s(): Failed in writing register"
					"0x%02x err %d\n", __func__, reg, ret);

	mutex_unlock(&chip->mutex);
	return ret;
}


static int max17048_write_block(const struct i2c_client *client,
		uint8_t command, uint8_t length, const uint8_t *values)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&chip->mutex);
	if (chip && chip->shutdown_complete) {
		mutex_unlock(&chip->mutex);
		return -ENODEV;
	}

	ret = i2c_smbus_write_i2c_block_data(client, command, length, values);
	if (ret < 0)
		dev_err(&client->dev, "%s(): Failed in writing block data to"
				"0x%02x err %d\n", __func__, command, ret);
	mutex_unlock(&chip->mutex);
	return ret;
}


static int max17048_read_word(struct i2c_client *client, int reg)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&chip->mutex);
	if (chip && chip->shutdown_complete) {
		mutex_unlock(&chip->mutex);
		return -ENODEV;
	}

	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in reading register"
					"0x%02x err %d\n", __func__, reg, ret);

		mutex_unlock(&chip->mutex);
		return ret;
	} else {
		ret = (int)swab16((uint16_t)(ret & 0x0000ffff));

		mutex_unlock(&chip->mutex);
		return ret;

	}
}

/* Return value in uV */
static int max17048_get_ocv(struct max17048_chip *chip)
{
	int r;
	int reg;
	int ocv;

	r = max17048_write_word(chip->client, MAX17048_UNLOCK,MAX17048_UNLOCK_VALUE);
	if (r)
		return r;

	reg = max17048_read_word(chip->client, MAX17048_OCV);
	ocv = (reg >> 4) * 1250;

	r = max17048_write_word(chip->client, MAX17048_UNLOCK, 0);
	WARN_ON(r);

	return ocv;
}

static int max17048_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct max17048_chip *chip = container_of(psy,
				struct max17048_chip, battery);
	int temp;
	int ret;
	switch (psp) {
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = chip->status;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
    /*                                                                          */
#ifdef CONFIG_MACH_PDA
	     if(pseudo_batt_info.mode == 1) {
		    //printk("[charger] Fake mode\n");
		    val->intval = pseudo_batt_info.volt * 1000;
	     }else{
		    val->intval = chip->vcell * 1000; // this is orignal.
	     }
#endif
    /*                                          */
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
    /*                                                                          */
#ifdef CONFIG_MACH_PDA
	     if(pseudo_batt_info.mode == 1) {
		    //printk("[charger] Fake mode\n");
		    val->intval = pseudo_batt_info.capacity;
	     }else{
	     /* prevent showing over 100 percentage */
            if(chip->soc > MAX17048_BATTERY_FULL) {
              val->intval = MAX17048_BATTERY_FULL;
            } else {
              val->intval = chip->soc;
            }
		 }
#endif
    /*                                          */
		if (chip->soc == 15)
			dev_warn(&chip->client->dev,
			"/nSystem Running low on battery - 15%\n");
		if (chip->soc == 10)
			dev_warn(&chip->client->dev,
			"/nSystem Running low on battery - 10%\n");
		if (chip->soc == 5)
			dev_warn(&chip->client->dev,
			"/nSystem Running low on battery - 5%\n");
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chip->health;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = chip->capacity_level;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		val->intval = max17048_get_ocv(chip);
		break;
	case POWER_SUPPLY_PROP_TEMP:
    /*                                                                          */
#ifdef CONFIG_MACH_PDA
		if(pseudo_batt_info.mode == 1)
			val->intval = pseudo_batt_info.temp * 10;
		else
			val->intval = chip->curr_temp * 10;
		break;
#endif
    /*                                          */
    /*                                                                          */
#ifdef CONFIG_MACH_PDA
	case POWER_SUPPLY_PROP_PSEUDO_BATT:
		val->intval = pseudo_batt_info.mode;
		break;
#endif
    /*                                          */

    /*                                                                           */
#ifdef CONFIG_MACH_PDA
	case POWER_SUPPLY_PROP_PRESENT:
		ret = get_prop_batt_present();
		if (ret >= 0) {
			val->intval = ret;
			ret = 0;
		}
		break;
#endif
     /*                                                                         */

	/*                                                                           */
#ifdef CONFIG_MACH_PDA
	case POWER_SUPPLY_PROP_BATTERY_ID_CHECK:
        ret = is_battery_valid();
		if (ret >= 0){
			val->intval = ret;
		    ret = 0;
		}
		break;
#endif
     /*                                                                         */
	default:
	return -EINVAL;
	}
	return 0;
}

static void max17048_get_vcell(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	int vcell;

	vcell = max17048_read_word(client, MAX17048_VCELL);
	if (vcell < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, vcell);
	else
		chip->vcell = (uint16_t)(((vcell >> 4) * 125) / 100);
}

#ifdef CONFIG_MACH_PDA
/*                                                              */
static int max17048_cal_rnd_avg(int *soc_value, int unit_num)
{
         int sum=0;
         int avr=0;
         int i;

         for(i = 0; i < unit_num; i++)
         {
                sum += soc_value[i];
         }

         avr = ((sum/unit_num)*10+5)/10;

         return avr;
}
#endif

static void max17048_get_soc(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	struct max17048_platform_data *pdata;
	int soc;
#ifdef CONFIG_MACH_PDA
/*                                                              */
	#define I_MAX 20 /* 30 -> 20 */
	static int soc_buf[I_MAX]={0,};
	static int idx_buf = 0;
	int idx_soc = 0;
	int avg_soc = 0;
	int soc_temp = 0;
#endif

	pdata = chip->pdata;
	soc = max17048_read_word(client, MAX17048_SOC);

	mutex_lock(&chip->mutex);

	if (soc < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, soc);
	else {
		chip->raw_soc = (uint16_t)soc >> 9;

		if (pdata->soc_error_max_value)
#ifdef CONFIG_MACH_PDA
            soc_temp =((((chip->raw_soc-LGE_EMPTY_SOC) * MAX17048_BATTERY_FULL)/(MAX17048_BATTERY_FULL-LGE_EMPTY_SOC)) * MAX17048_BATTERY_FULL)
			            / pdata->soc_error_max_value;
	/*                                                              */
	pr_debug("%s: raw_soc :: %d \n", __func__, chip->raw_soc);
	/* soc is lower than 0 && voltage is higher than 3200mV, set soc to 1 percent */
	if(soc_temp <= 0) soc_temp = 1;
	if(chip->raw_soc == 0) soc_temp = 0;
	chip->soc = soc_temp;

	pr_info("%s: calculated soc :: %d \n", __func__, chip->soc);

#else
			chip->soc =(chip->soc * MAX17048_BATTERY_FULL)
						/ pdata->soc_error_max_value;
#endif
	}

#ifdef CONFIG_MACH_PDA

    {
		if(cnt_soc == 0) {// firstime
			for(idx_soc = 0; idx_soc<I_MAX; idx_soc++) {
				soc_buf[idx_soc] = chip->soc;
			}
			avg_soc = chip->soc;
			cnt_soc = I_MAX;
			idx_buf = 1;
		}
		else {
			if (idx_buf >= I_MAX) {
				idx_buf = 0;
			}
			soc_buf[idx_buf] = chip->soc;
			idx_buf++;
			if(is_chg_done_irq() && chip->soc < 97 && is_pseudo_batt()){
				printk("[PIPAS] all soc table is set to 100\n");
				for(idx_soc = 0; idx_soc<I_MAX; idx_soc++) {
					soc_buf[idx_soc] = 100;
				}
				set_chg_done_flag(0);
			}

			avg_soc = max17048_cal_rnd_avg(soc_buf, I_MAX);

			if(avg_soc == 0 && chip->raw_soc == 1){
				printk("[PIPAS] all soc table is set to 1\n");
				for(idx_soc = 0; idx_soc<I_MAX; idx_soc++) {
					soc_buf[idx_soc] = 1;
				}
				avg_soc = 1;
			}

		}
		chip->soc = avg_soc;
		pr_debug("%s: avraged_soc :: %d \n", __func__, chip->soc);
	}
#endif
	pr_debug("%s: After Scaled SOC : %d \n", __func__, chip->soc);
#ifdef CONFIG_MACH_PDA
	/* if charger is not connected soc shoud not be increased */
	if ((chip->soc > chip->lasttime_soc) && (chip->status == POWER_SUPPLY_STATUS_DISCHARGING)) {
		chip->soc = chip->lasttime_soc;
	}
#endif
	if (chip->soc >= MAX17048_BATTERY_FULL && chip->charge_complete != 1) {
#ifdef CONFIG_MACH_PDA
/* if scaled SOC is over 101(unscaled SOC 97), user will be seen full charged */
        if(chip->soc >= MAX17048_BATTERY_FULL) {
			chip->soc = MAX17048_BATTERY_FULL;
			chip->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
			chip->health = POWER_SUPPLY_HEALTH_GOOD;
        }
#else //NV Original.
		chip->soc = MAX17048_BATTERY_FULL-1;

	}
#endif
    }
	//if (chip->status == POWER_SUPPLY_STATUS_FULL && chip->charge_complete) { /* DONE irg handled... */
	if(chip->charge_complete == 1) {
		if(chip->soc >= MAX17048_BATTERY_FULL) {
			chip->soc = MAX17048_BATTERY_FULL;
			chip->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
			chip->health = POWER_SUPPLY_HEALTH_GOOD;
			chip->status = POWER_SUPPLY_STATUS_FULL;
		}
	} else if (chip->soc < MAX17048_BATTERY_LOW) {
		chip->status = chip->lasttime_status;
		chip->health = POWER_SUPPLY_HEALTH_DEAD;
		chip->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	} else {
		chip->status = chip->lasttime_status;
		chip->health = POWER_SUPPLY_HEALTH_GOOD;
		chip->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	}
	printk("[PIPAS] raw_soc =%d disply_soc =%d vcell =%d\n ",chip->raw_soc,chip->soc,chip->vcell);
	mutex_unlock(&chip->mutex);
}

static int max17048_read_soc_raw_value(struct battery_gauge_dev *bg_dev)
{
	struct max17048_chip *chip = battery_gauge_get_drvdata(bg_dev);
	return chip->raw_soc;
}

#ifdef CONFIG_MACH_PDA
static int max17048_read_vcell_value(struct battery_gauge_dev *bg_dev)
{
	struct max17048_chip *chip = battery_gauge_get_drvdata(bg_dev);
	return chip->vcell;
}
#define CHG_DONE_VOLT 4300 /* 4350 to 4300 */
bool max17048_is_chg_done(void)
{
	if (!max17048_data){
		pr_info("%s: max17048 is not inittailized yet. \n", __func__);
		return false;
	}

	max17048_get_vcell(max17048_data->client);
	max17048_get_soc(max17048_data->client);
	if((max17048_data->raw_soc >= MAX17048_BATTERY_FULL) &&
		(max17048_data->vcell >= CHG_DONE_VOLT)){
		return true;
	}
	return false;
}
EXPORT_SYMBOL(max17048_is_chg_done);
#endif
static uint16_t max17048_get_version(struct i2c_client *client)
{
	return swab16(max17048_read_word(client, MAX17048_VER));
}

static void max17048_work(struct work_struct *work)
{
	struct max17048_chip *chip;

	chip = container_of(work, struct max17048_chip, work.work);

#ifdef CONFIG_MACH_PDA
	if(chip->soc > 1) {
        max17048_update_rcomp_by_temperature(chip);
	}
#endif

	max17048_get_vcell(chip->client);
	max17048_get_soc(chip->client);
#if 0
	/* TODO!!! */
	if (chip->soc != chip->lasttime_soc ||
		chip->status != chip->lasttime_status) {
		chip->lasttime_soc = chip->soc;
		power_supply_changed(&chip->battery);
	}
#endif
#ifdef CONFIG_MACH_PDA
	if (chip->soc != chip->lasttime_soc)
			chip->lasttime_soc = chip->soc;

	power_supply_changed(&chip->battery);
	if(chip->soc < 1) { /* preventing frequent i2c comm. */
        schedule_delayed_work(&chip->work, MAX17048_LGE_DELAY_LOW);
	}
	else if(chip->soc >= 1 && chip->soc <= 15) {
		schedule_delayed_work(&chip->work, MAX17048_LGE_DELAY_CRITIAL);
	}
	else if(chip->soc >= 16 && chip->soc <= 65) {
		schedule_delayed_work(&chip->work, MAX17048_LGE_DELAY_LOW_SOC);
	}
	else if(chip->soc >= 66 && chip->soc <= 98) {
		schedule_delayed_work(&chip->work, MAX17048_LGE_DELAY);
	}
	else {
		schedule_delayed_work(&chip->work, MAX17048_LGE_DELAY_SLOW);
	}
#else
	schedule_delayed_work(&chip->work, MAX17048_DELAY);
#endif
}

void max17048_battery_status(int status)
{
	if (!max17048_data)
		return;

	if (status == progress)
		max17048_data->status = POWER_SUPPLY_STATUS_CHARGING;
	else
		max17048_data->status = POWER_SUPPLY_STATUS_DISCHARGING;

	max17048_data->lasttime_status = max17048_data->status;
	power_supply_changed(&max17048_data->battery);
}
EXPORT_SYMBOL_GPL(max17048_battery_status);

static enum power_supply_property max17048_battery_props[] = {
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_TEMP,
	/*                                                                           */
#ifdef CONFIG_MACH_PDA
    POWER_SUPPLY_PROP_PRESENT,
#endif
    /*                                                                         */
	/*                                                                           */
#ifdef CONFIG_MACH_PDA
    POWER_SUPPLY_PROP_BATTERY_ID_CHECK,
#endif
    /*                                                                        */
    /*                                                                          */
#ifdef CONFIG_MACH_PDA
    POWER_SUPPLY_PROP_PSEUDO_BATT,
#endif
    /*                                          */
};

/*                                                                          */
#ifdef CONFIG_MACH_PDA
int pseudo_batt_set(struct pseudo_batt_info_type* info)
{
	if (!max17048_data)
		return;
	//int rc;

	pseudo_batt_info.mode = info->mode;
	pseudo_batt_info.id = info->id;
	pseudo_batt_info.therm = info->therm;
	pseudo_batt_info.temp = info->temp;
	pseudo_batt_info.volt = info->volt;
	pseudo_batt_info.capacity = info->capacity;
	pseudo_batt_info.charging = info->charging;


 /* this is QCT PMIC Logic, i will check more.
	if(pseudo_batt_info.mode)
	{
		rc = pm_chg_masked_write(chip, CHG_CNTRL_2,
		CHG_BAT_TEMP_DIS_BIT, CHG_BAT_TEMP_DIS_BIT);
		if (rc) {
			pr_err("Failed to disable temp control chg rc=%d\n", rc);
		}
	}else	{
		rc = pm_chg_masked_write(chip, CHG_CNTRL_2,
		CHG_BAT_TEMP_DIS_BIT, 0);
		if (rc) {
			pr_err("Failed to enable temp control chg rc=%d\n", rc);
		}
	}
*/

	power_supply_changed(&max17048_data->battery);
	return 0;
}
EXPORT_SYMBOL(pseudo_batt_set);

bool is_pseudo_batt(void)
{
	int result = 0;

	if(pseudo_batt_info.mode == 1){
		result = 1;
	}else{
		result = 0;
	}

	return result;
}EXPORT_SYMBOL(is_pseudo_batt);

#endif
/*                                          */



/*                                                                           */
#ifdef CONFIG_MACH_PDA
int get_prop_batt_present()
{
	if(pseudo_batt_info.mode == 1 || is_factory_cable())
		return 1;

/*                                          
                                                                                  
                             
                                                                               */
	if ((lge_battery_info == BATT_DS2704_N || lge_battery_info == BATT_DS2704_L ||
		lge_battery_info == BATT_ISL6296_N || lge_battery_info == BATT_ISL6296_L ||
		lge_battery_info == BATT_DS2704_C || lge_battery_info == BATT_ISL6296_C ||
	    lge_battery_info == BATT_UNKNOWN))
		return 1;
	else
		return 0;
}
#endif
/*                                                                         */

/*                                                                           */
#ifdef CONFIG_MACH_PDA
int is_battery_valid()
{
	if(pseudo_batt_info.mode == 1 || is_factory_cable())
		return 1;
//	else if (is_factory_cable())
//		return 1;

	if ((lge_battery_info == BATT_DS2704_N || lge_battery_info == BATT_DS2704_L ||
		lge_battery_info == BATT_ISL6296_N || lge_battery_info == BATT_ISL6296_L ||
		lge_battery_info == BATT_DS2704_C || lge_battery_info == BATT_ISL6296_C))
	{
		return 1;
	}else{
		return 0;
	}
}
#endif
/*                                                                         */

static int max17048_write_rcomp_seg(struct i2c_client *client,
						uint16_t rcomp_seg)
{
	uint8_t rs1, rs2;
	int ret;
	uint8_t rcomp_seg_table[16];

	rs1 = (rcomp_seg >> 8) & 0xff;
	rs2 = rcomp_seg & 0xff;

	rcomp_seg_table[0] = rcomp_seg_table[2] = rcomp_seg_table[4] =
		rcomp_seg_table[6] = rcomp_seg_table[8] = rcomp_seg_table[10] =
			rcomp_seg_table[12] = rcomp_seg_table[14] = rs1;

	rcomp_seg_table[1] = rcomp_seg_table[3] = rcomp_seg_table[5] =
		rcomp_seg_table[7] = rcomp_seg_table[9] = rcomp_seg_table[11] =
			rcomp_seg_table[13] = rcomp_seg_table[15] = rs2;

	ret = max17048_write_block(client, MAX17048_RCOMPSEG1,
				16, (uint8_t *)rcomp_seg_table);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	ret = max17048_write_block(client, MAX17048_RCOMPSEG2,
				16, (uint8_t *)rcomp_seg_table);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int max17048_load_model_data(struct max17048_chip *chip)
{
	struct i2c_client *client = chip->client;
	struct max17048_battery_model *mdata = chip->pdata->model_data;
	uint16_t soc_tst, ocv;
	int i, ret = 0;

	/* read OCV */
	ret = max17048_read_word(client, MAX17048_OCV);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}
	ocv = (uint16_t)ret;
	if (ocv == 0xffff) {
		dev_err(&client->dev, "%s: Failed in unlocking"
					"max17048 err: %d\n", __func__, ocv);
		return -1;
	}

	/* write custom model data */
	for (i = 0; i < 4; i += 1) {
		if (max17048_write_block(client,
			(MAX17048_TABLE+i*16), 16,
				&mdata->data_tbl[i*0x10]) < 0) {
			dev_err(&client->dev, "%s: error writing model data:\n",
								__func__);
			return -1;
		}
	}

	/* Write OCV Test value */
	ret = max17048_write_word(client, MAX17048_OCV, mdata->ocvtest);
	if (ret < 0)
		return ret;

	ret = max17048_write_rcomp_seg(client, mdata->rcomp_seg);
	if (ret < 0)
		return ret;

	/* Disable hibernate */
	ret = max17048_write_word(client, MAX17048_HIBRT, 0x0000);
	if (ret < 0)
		return ret;

	/* Lock model access */
	ret = max17048_write_word(client, MAX17048_UNLOCK, 0x0000);
	if (ret < 0)
		return ret;

	/* Delay between 150ms to 600ms */
	mdelay(200);

	/* Read SOC Register and compare to expected result */
	ret = max17048_read_word(client, MAX17048_SOC);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}
	soc_tst = (uint16_t)ret;
	if (!((soc_tst >> 8) >= mdata->soccheck_A &&
				(soc_tst >> 8) <=  mdata->soccheck_B)) {
		dev_err(&client->dev, "%s: soc comparison failed %d\n",
					__func__, ret);
		return ret;
	} else {
		dev_info(&client->dev, "MAX17048 Custom data"
						" loading successfull\n");
	}

	/* unlock model access */
	ret = max17048_write_word(client, MAX17048_UNLOCK,
					MAX17048_UNLOCK_VALUE);
	if (ret < 0)
		return ret;

	/* Restore OCV */
	ret = max17048_write_word(client, MAX17048_OCV, ocv);
	if (ret < 0)
		return ret;

	return ret;
}

static int max17048_initialize(struct max17048_chip *chip)
{
	uint8_t ret, config = 0;
	struct i2c_client *client = chip->client;
	struct max17048_battery_model *mdata = chip->pdata->model_data;

	/* unlock model access */
	ret = max17048_write_word(client, MAX17048_UNLOCK,
			MAX17048_UNLOCK_VALUE);
	if (ret < 0)
		return ret;

#ifndef CONFIG_MACH_PDA
/*                                                               */
	/* load model data */
	ret = max17048_load_model_data(chip);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}
#endif

	if (mdata->bits == 19)
		config = 32 - (mdata->alert_threshold * 2);
	else if (mdata->bits == 18)
		config = 32 - mdata->alert_threshold;

	config = mdata->one_percent_alerts | config;

	ret = max17048_write_word(client, MAX17048_CONFIG,
			((mdata->rcomp << 8) | config));
	if (ret < 0)
		return ret;

	/* Voltage Alert configuration */
	ret = max17048_write_word(client, MAX17048_VALRT, mdata->valert);
	if (ret < 0)
		return ret;

	ret = max17048_write_word(client, MAX17048_VRESET, mdata->vreset);
	if (ret < 0)
		return ret;

	/* Lock model access */
	ret = max17048_write_word(client, MAX17048_UNLOCK, 0x0000);
	if (ret < 0)
		return ret;

	/* Add delay */
	mdelay(200);
	return 0;
}

int max17048_check_battery()
{
	uint16_t version;

	if (!max17048_data)
		return -ENODEV;

	version = max17048_get_version(max17048_data->client);
	if (version != MAX17048_VERSION_NO)
		return -ENODEV;

	return 0;
}
EXPORT_SYMBOL_GPL(max17048_check_battery);

#ifdef CONFIG_OF
static struct max17048_platform_data *max17048_parse_dt(struct device *dev)
{
	struct max17048_platform_data *pdata;
	struct max17048_battery_model *model_data;
	struct device_node *np = dev->of_node;
	u32 val, val_array[MAX17048_DATA_SIZE];
	int i, ret;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	model_data = devm_kzalloc(dev, sizeof(*model_data), GFP_KERNEL);
	if (!model_data)
		return ERR_PTR(-ENOMEM);

	pdata->model_data = model_data;

	ret = of_property_read_u32(np, "bits", &val);
	if (ret < 0)
		return ERR_PTR(ret);

	if ((val == 18) || (val == 19))
		model_data->bits = val;

	ret = of_property_read_u32(np, "alert-threshold", &val);
	if (ret < 0)
		return ERR_PTR(ret);

	model_data->alert_threshold = val;
	if (model_data->bits == 19) /* LSB is 0.5%, if 19-bit model. */
		model_data->alert_threshold /= 2;

	ret = of_property_read_u32(np, "one-percent-alerts", &val);
	if (ret < 0)
		return ERR_PTR(ret);

	if (val)
		model_data->one_percent_alerts = 0x40;

	ret = of_property_read_u32(np, "valert-max", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->valert = ((val / 20) & 0xFF) << 8; /* LSB is 20mV. */

	ret = of_property_read_u32(np, "valert-min", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->valert |= (val / 20) & 0xFF; /* LSB is 20mV. */

	ret = of_property_read_u32(np, "vreset-threshold", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->vreset = ((val / 40) & 0xFE) << 8; /* LSB is 40mV. */

	ret = of_property_read_u32(np, "vreset-disable", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->vreset |= (val & 0x01) << 8;

	ret = of_property_read_u32(np, "hib-threshold", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->hibernate = (val & 0xFF) << 8;

	ret = of_property_read_u32(np, "hib-active-threshold", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->hibernate |= val & 0xFF;

	ret = of_property_read_u32(np, "rcomp", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->rcomp = val;

	ret = of_property_read_u32(np, "rcomp-seg", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->rcomp_seg = val;

	ret = of_property_read_u32(np, "soccheck-a", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->soccheck_A = val;

	ret = of_property_read_u32(np, "soccheck-b", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->soccheck_B = val;

	ret = of_property_read_u32(np, "ocvtest", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->ocvtest = val;

	ret = of_property_read_u32_array(np, "data-tbl", val_array,
					 MAX17048_DATA_SIZE);
	if (ret < 0)
		return ERR_PTR(ret);

	for (i = 0; i < MAX17048_DATA_SIZE; i++)
		model_data->data_tbl[i] = val_array[i];

	return pdata;
}
#else
static struct max17048_platform_data *max17048_parse_dt(struct device *dev)
{
	return NULL;
}
#endif /* CONFIG_OF */

static int max17048_update_battery_status(struct battery_gauge_dev *bg_dev,
		enum battery_charger_status status)
{
	struct max17048_chip *chip = battery_gauge_get_drvdata(bg_dev);

	if (status == BATTERY_CHARGING) {
		chip->charge_complete = 0;
		chip->status = POWER_SUPPLY_STATUS_CHARGING;
	}
	else if (status == BATTERY_CHARGING_DONE) { /* soc and status might unnecessary to update and power supply changed. */
		chip->charge_complete = 1;
		//chip->soc = MAX17048_BATTERY_FULL;
		//chip->status = POWER_SUPPLY_STATUS_FULL;
		//power_supply_changed(&chip->battery);
		return 0;
	} else {
		chip->status = POWER_SUPPLY_STATUS_DISCHARGING;
		chip->charge_complete = 0;
	}
	chip->lasttime_status = chip->status;
	power_supply_changed(&chip->battery);
	return 0;
}

static struct battery_gauge_ops max17048_bg_ops = {
	.update_battery_status = max17048_update_battery_status,
	.get_soc_value = max17048_read_soc_raw_value,
#ifdef CONFIG_MACH_PDA
	.get_vcell_value = max17048_read_vcell_value,
#endif
};

static struct battery_gauge_info max17048_bgi = {
	.cell_id = 0,
	.bg_ops = &max17048_bg_ops,
};

#ifdef CONFIG_MACH_PDA
static ssize_t max77660_fuelgauge_reset(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct max17048_chip *chip = dev_get_drvdata(dev);
	int ret;

	pr_info("%s: FUEL GAUGE RESET \n", __func__);
	ret = max17048_write_word(chip->client, 0x06, 0x4000);
	if (ret < 0) {
		dev_err(dev, "failed in resetting fuelgauge\n");
		sprintf(buf, "%d\n", 0);
		return strlen(buf);
	}
	cnt_soc = 0;
	cancel_delayed_work_sync(&chip->work);
	schedule_delayed_work(&chip->work, (3*HZ));

	sprintf(buf, "%d\n", 1);
	return strlen(buf);
}
DEVICE_ATTR(at_fuelrst, 0640, max77660_fuelgauge_reset, NULL);

extern int max77660_lge_charging_enable(void);
extern int max77660_lge_charging_disable(void);

static ssize_t max77660_at_charge_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct max17048_chip *chip = dev_get_drvdata(dev);

	if(chip->status == POWER_SUPPLY_STATUS_CHARGING) {
		sprintf(buf, "%d", 1);
	}else if(chip->status == POWER_SUPPLY_STATUS_NOT_CHARGING ||
				chip->status == POWER_SUPPLY_STATUS_DISCHARGING) {
		sprintf(buf, "%d", 0);
	}else { // do not happen
		sprintf(buf, "%d", 0);
	}
	strcat(buf, "\n");
	//charging 1 no charging 0
	return strlen(buf);
}

static ssize_t max77660_at_charge_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct max17048_chip *chip = dev_get_drvdata(dev);
	int mode;
	pr_info("%s: [AT_CHARGE] Enter AT_CHARGE!!! \n", __func__);
	if (sscanf(buf, "%d", &mode) != 1)
		return -EINVAL;
	pr_info("%s: [AT_CHARGE] mode : %d \n", __func__,mode);
	if (mode) {
		//Charging start
		pr_info("%s: [AT_CHARGE] Charging Start \n", __func__);
		max77660_lge_charging_enable();
	} else {
		//Charging stop
		pr_info("%s: [AT_CHARGE] Charging Stop \n", __func__);
		max77660_lge_charging_disable();
	}

	return count;
}
DEVICE_ATTR(at_charge, 0640, max77660_at_charge_show, max77660_at_charge_store);

static ssize_t max77660_at_usbid_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
    struct cable_info_table mCable_info;

	lge_pm_get_cable_info(&mCable_info);

	sprintf(buf, "%d,", mCable_info.adc_val);

	if(mCable_info.type == USB_CABLE_LT_56K)
		strcat(buf, "56");
	else if(mCable_info.type == USB_CABLE_LT_130K)
		strcat(buf, "130");
	else if(mCable_info.type == USB_CABLE_LT_910K)
		strcat(buf, "910");
	else if(mCable_info.type == USB_CABLE_NO_USB)
		strcat(buf, "OFF");
	else if(mCable_info.type == USB_CABLE_400MA
		    || mCable_info.type == USB_CABLE_DTC_500MA
		    || mCable_info.type == USB_CABLE_LT_910K)
		strcat(buf, "NORMAL");
	else {
		strcat(buf, "Undefined");
	}

	printk("USBID = %d:%d\n",mCable_info.adc_val,mCable_info.type);

	return strlen(buf);
}
DEVICE_ATTR(at_usbid, 0640, max77660_at_usbid_show, NULL);

static int max17048_set_rcomp(int rcomp)
{
	struct i2c_client *client = max17048_data->client;
	int config;
	if (!max17048_data)
		return -ENODEV;

	//read config
	config = max17048_read_word(client,MAX17048_CONFIG);
	pr_info("%s read config =0x%02X\n",__func__, config);

	//set rcomp
	rcomp &= 0xff;
	config = ((config & 0x00ff) | (rcomp << 8));
	pr_info("%s write config with new rcomp =0x%02X\n",__func__, config);

	max17048_write_word(client,MAX17048_CONFIG,config);
#if 0 /* read config Debug only */
	config = max17048_read_word(client,MAX17048_CONFIG);
	pr_info("%s read again config =0x%02X\n",__func__, config);
#endif
	return 0;
}

/* temperature compensation */
static int max17048_update_rcomp_by_temperature(struct max17048_chip *chip)
{
	struct i2c_client *client = chip->client;
	u8 startingRcomp = chip->startingRcomp;
	int PreviousRcomp = 0;
	int tempCoHot = chip->temp_co_hot;		/* -3375*/
	int tempCoCold = chip->temp_co_cold;	/* -54375 */

	int newRcomp;
	int temp;
	int ret;
	int read_cnt = 0;
	//Read temperature
	//WR retry if reading fails
#if 0
	if(battery_gauge_get_battery_temperature(chip->bg_dev,&temp)<0) {
        pr_info("%s run failure!!! temperature read fail. \n",__func__);
		return 0;
	}
#endif

	for(read_cnt =0;read_cnt <3; read_cnt++) {
		if(battery_gauge_get_battery_temperature(chip->bg_dev,&temp)<0) {
			pr_info("%s run failure!!! temperature read fail. retry :%d\n",__func__,read_cnt);
		}else {
			pr_debug("%s run temperature read success\n",__func__);
			break;
		}
	}
	chip->curr_temp = temp;

    if (chip->curr_temp != chip->prev_temp) {
        chip->prev_temp = chip->curr_temp;
    }else {
         printk("[PIPAS] temp has not been changed, do not update RCOMP\n");
         return 0;
    }

	PreviousRcomp = max17048_read_word(client, MAX17048_CONFIG);
	PreviousRcomp = (PreviousRcomp & 0xFF00) >> 8;
	if (PreviousRcomp < 0)
		return PreviousRcomp;

	pr_debug("%s check temp = %d, PreviousRcomp =0x%02X\n",__func__, temp, PreviousRcomp);

	if (temp > 20)
		newRcomp = startingRcomp + (int)((temp - 20)*tempCoHot/10000);
	else if (temp < 20){
		newRcomp = startingRcomp + (int)((temp - 20)*tempCoCold/10000);
		}
	else
		newRcomp = startingRcomp;

	if (newRcomp > 0xFF)
		newRcomp = 0xFF;
	else if (newRcomp < 0)
		newRcomp = 0;

	if (newRcomp != PreviousRcomp)
	{
		pr_info("%s is run!!! RCOMP: new rcomp is 0x%02X(0x%02X)\n",__func__, newRcomp, startingRcomp);
		max17048_set_rcomp(newRcomp);
	}
	return 0;
}

#endif
static int __devinit max17048_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct max17048_chip *chip;
	int ret;
	uint16_t version;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;

	if (client->dev.of_node) {
		chip->pdata = max17048_parse_dt(&client->dev);
		if (IS_ERR(chip->pdata))
			return PTR_ERR(chip->pdata);
	} else {
		chip->pdata = client->dev.platform_data;
		if (!chip->pdata)
			return -ENODATA;
	}

	max17048_data = chip;
	mutex_init(&chip->mutex);
	chip->shutdown_complete = 0;
	i2c_set_clientdata(client, chip);

	version = max17048_check_battery();
	if (version < 0) {
		ret = -ENODEV;
		goto error;
	}
	dev_info(&client->dev, "MAX17048 Fuel-Gauge Ver 0x%x\n", version);

	ret = max17048_initialize(chip);
	if (ret < 0) {
		dev_err(&client->dev, "Error: Initializing fuel-gauge\n");
		goto error;
	}

	chip->battery.name		= "battery";
	chip->battery.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->battery.get_property	= max17048_get_property;
	chip->battery.properties	= max17048_battery_props;
	chip->battery.num_properties	= ARRAY_SIZE(max17048_battery_props);
	chip->status			= POWER_SUPPLY_STATUS_DISCHARGING;
	chip->lasttime_status   	= POWER_SUPPLY_STATUS_DISCHARGING;
	chip->charge_complete   	= 0;
#ifdef CONFIG_MACH_PDA
	chip->lasttime_soc		= 200; /* Getting lasttime_soc for the first time */
	chip->startingRcomp = 0x32;
	chip->temp_co_cold = -3375;
	chip->temp_co_hot = -54375;
	chip->curr_temp = 25;
	chip->prev_temp = 25;
#endif
	ret = power_supply_register(&client->dev, &chip->battery);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		goto error;
	}
	max17048_bgi.tz_name = chip->pdata->tz_name;
	chip->bg_dev = battery_gauge_register(&client->dev, &max17048_bgi,
				chip);
	if (IS_ERR(chip->bg_dev)) {
		ret = PTR_ERR(chip->bg_dev);
		dev_err(&client->dev, "battery gauge register failed: %d\n",
			ret);
		goto bg_err;
	}
#ifdef CONFIG_MACH_PDA
	ret = device_create_file(&client->dev, &dev_attr_at_fuelrst);
	if(ret < 0) {
		dev_err(&client->dev,"AT_FUELRST File device creation failed: %d\n", ret);
		goto bg_err;
	}
	ret = device_create_file(&client->dev, &dev_attr_at_charge);
	if(ret < 0) {
		dev_err(&client->dev,"AT_CHARGE File device creation failed: %d\n", ret);
		goto bg_err;
	}
	ret = device_create_file(&client->dev, &dev_attr_at_usbid);
	if(ret < 0) {
		dev_err(&client->dev,"AT_USBIDADC File device creation failed: %d\n", ret);
		goto bg_err;
	}
#endif

	INIT_DELAYED_WORK_DEFERRABLE(&chip->work, max17048_work);
	schedule_delayed_work(&chip->work, 0);

	return 0;
bg_err:
	power_supply_unregister(&chip->battery);
#ifdef CONFIG_MACH_PDA
	device_remove_file(&client->dev, &dev_attr_at_fuelrst);
	device_remove_file(&client->dev, &dev_attr_at_charge);
	device_remove_file(&client->dev, &dev_attr_at_usbid);
#endif
error:
	mutex_destroy(&chip->mutex);

	return ret;
}

static int __devexit max17048_remove(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);

#ifdef CONFIG_MACH_PDA
	device_remove_file(&client->dev, &dev_attr_at_fuelrst);
	device_remove_file(&client->dev, &dev_attr_at_charge);
	device_remove_file(&client->dev, &dev_attr_at_usbid);
#endif
	battery_gauge_unregister(chip->bg_dev);
	power_supply_unregister(&chip->battery);
	cancel_delayed_work_sync(&chip->work);
	mutex_destroy(&chip->mutex);

	return 0;
}

static void max17048_shutdown(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&chip->work);
	mutex_lock(&chip->mutex);
	chip->shutdown_complete = 1;
	mutex_unlock(&chip->mutex);

}

#ifdef CONFIG_PM_SLEEP
static int max17048_suspend(struct device *dev)
{
	struct max17048_chip *chip = dev_get_drvdata(dev);
	/*
	int ret;

	ret = max17048_write_word(chip->client, MAX17048_HIBRT, 0xffff);
	if (ret < 0) {
		dev_err(dev, "failed in entering hibernate mode\n");
		return ret;
	}
	*/
	cancel_delayed_work_sync(&chip->work);
	return 0;
}

static int max17048_resume(struct device *dev)
{
	struct max17048_chip *chip = dev_get_drvdata(dev);

	/*
	int ret;
	struct max17048_battery_model *mdata = chip->pdata->model_data;

	ret = max17048_write_word(chip->client, MAX17048_HIBRT, mdata->hibernate);
	if (ret < 0) {
		dev_err(dev, "failed in exiting hibernate mode\n");
		return ret;
	}
	// lasttime_soc and soc is different more than 5% then cnt_soc is set to 0
	//if(cnt_soc!=0)cnt_soc=0;
	*/
#ifdef CONFIG_MACH_PDA
	schedule_delayed_work(&chip->work, 0);
#else
	schedule_delayed_work(&chip->work, MAX17048_DELAY);
#endif

	return 0;
}
#endif /* CONFIG_PM */

static SIMPLE_DEV_PM_OPS(max17048_pm_ops, max17048_suspend, max17048_resume);

#ifdef CONFIG_OF
static const struct of_device_id max17048_dt_match[] = {
	{ .compatible = "maxim,max17048" },
	{ },
};
MODULE_DEVICE_TABLE(of, max17048_dt_match);
#endif

static const struct i2c_device_id max17048_id[] = {
	{ "max17048", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17048_id);

static struct i2c_driver max17048_i2c_driver = {
	.driver	= {
		.name	= "max17048",
		.of_match_table = of_match_ptr(max17048_dt_match),
		.pm = &max17048_pm_ops,
	},
	.probe		= max17048_probe,
	.remove		= __devexit_p(max17048_remove),
	.id_table	= max17048_id,
	.shutdown	= max17048_shutdown,
};

static int __init max17048_init(void)
{
	return i2c_add_driver(&max17048_i2c_driver);
}
subsys_initcall(max17048_init);

static void __exit max17048_exit(void)
{
	i2c_del_driver(&max17048_i2c_driver);
}
module_exit(max17048_exit);

MODULE_AUTHOR("Chandler Zhang <chazhang@nvidia.com>");
MODULE_DESCRIPTION("MAX17048 Fuel Gauge");
MODULE_LICENSE("GPL");
