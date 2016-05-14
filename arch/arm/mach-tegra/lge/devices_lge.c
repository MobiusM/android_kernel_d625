#ifndef __ASM_ARCH_TEGRA_BOARD_LGE_H
#define __ASM_ARCH_TEGRA_BOARD_LGE_H

#include <linux/kernel.h>
#include <linux/string.h>

#include <mach/board_lge.h>

#include <linux/platform_device.h>
#include <asm/setup.h>
#include <asm/system_info.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/vmalloc.h>
#include <linux/memblock.h>
#ifdef CONFIG_MACH_PDA
#include <linux/mfd/max77660/max77660-core.h>
#endif

struct cable_info_table lge_cable_info;

static int factory_cable_inserted = 1;
static int disconnected_fcable = 1;
static int is_chg_done = 0;

usb_cable_adc_type cables[CABLE_MAX] = {
/*            adc_min,    adc_max,  cable_type   */
/* _56K  */  {250,        410,      USB_CABLE_LT_56K}, /* 360 --> 250 */
/* _130K */  {600,       700,      USB_CABLE_LT_130K}, /* 640 --> 600 */
/* _910K */  {1320,      1420,      USB_CABLE_LT_910K},/* 1340 --> 1320 */
/* _NO_USB */{1550,      1760,      USB_CABLE_NO_USB}, /* 1590 --> 1550, 1610 --> 1760 */
/* _USER */  {890,      920,      USB_CABLE_400MA}
};

int get_firstboot_factory_flag(void)
{
	return factory_cable_inserted;
}

void clear_firstboot_factory_flag(void)
{
	factory_cable_inserted = 0;
}

/*
  * Return  1 : when factory cable disconnected as the device boots
  *            0 : when factory cable is NEVER disconneted as the device boots.
  */
int get_fcable_disconnect_check_flag(void)
{
	return disconnected_fcable;
}

void clear_fcable_disconnect_check_flag(void)
{
	disconnected_fcable = 0;
}


/* Belows are for using in interrupt context */
usb_cable_type lge_pm_get_cable_type(void)
{
	return lge_cable_info.type;
}

/* This must be invoked in process context */
char *lge_get_cabletype_to_str (struct cable_info_table *info) {
	switch(info->type)
	{
	case USB_CABLE_LT_56K:
		return "56K";
	case USB_CABLE_LT_130K:
		return "130K";
	case USB_CABLE_LT_910K:
		return "910K";
	case USB_CABLE_400MA:
	case USB_CABLE_DTC_500MA:
	case USB_CABLE_ABNORMAL_400MA:
		return "USER";
	case USB_CABLE_NO_USB:
		return "NO USB";
	default:
		return "NOT DEFINED";
	}
	return "fail";
}

void lge_pm_get_cable_info(struct cable_info_table *info)
{
	int adc_val;
	int cable_enum;

	adc_val = max77660_lge_get_adc(MAX77660_ADC_CH_ADC3);
	info->adc_val = adc_val;

	for (cable_enum = 0; cable_enum < CABLE_MAX; cable_enum++)
	{
		if (cables[cable_enum].adc_min <= adc_val &&
		    adc_val <= cables[cable_enum].adc_max )
		    break;
	}

	if (CABLE_MAX == cable_enum)
		info->type = USB_CABLE_NOT_DEFINED;
	else
		info->type = cables[cable_enum].cable_type;

	pr_info("\n\n[PM] Cable detected: %d(%s)\n\n",
		adc_val,
		lge_get_cabletype_to_str(info));
}

void lge_pm_read_cable_info(void)
{
	lge_pm_get_cable_info(&lge_cable_info);
}

/* select androidboot.mode */
static enum lge_boot_mode_type lge_boot_mode = LGE_BOOT_MODE_NORMAL;
int __init lge_boot_mode_init(char *s)
{
	if (!strcmp(s, "charger"))
		lge_boot_mode = LGE_BOOT_MODE_CHARGER;
	else if (!strcmp(s, "qem_charger"))
		lge_boot_mode = LGE_BOOT_MODE_CHARGER;
	else if (!strcmp(s, "chargerlogo"))
		lge_boot_mode = LGE_BOOT_MODE_CHARGERLOGO;
	else if (!strcmp(s, "factory"))
		lge_boot_mode = LGE_BOOT_MODE_FACTORY;
	else if (!strcmp(s, "factory2"))
		lge_boot_mode = LGE_BOOT_MODE_FACTORY2;
	else if (!strcmp(s, "pifboot"))
		lge_boot_mode = LGE_BOOT_MODE_PIFBOOT;
	else if (!strcmp(s, "pifboot2"))
		lge_boot_mode = LGE_BOOT_MODE_PIFBOOT2;	
	/*                            */
	else if (!strcmp(s, "miniOS"))
		lge_boot_mode = LGE_BOOT_MODE_MINIOS;
	else if (!strcmp(s, "qem_56k"))
		lge_boot_mode = LGE_BOOT_MODE_QEM_56K;
	else if (!strcmp(s, "qem_130k"))
		lge_boot_mode = LGE_BOOT_MODE_QEM_130K;
	else if (!strcmp(s, "qem_nousb"))
		lge_boot_mode = LGE_BOOT_MODE_QEM_NO_USB;
	/*                            */
	else if (!strcmp(s, "qem_normal"))
		lge_boot_mode = LGE_BOOT_MODE_QEM_NORMAL;
	return 1;
}
__setup("androidboot.mode=", lge_boot_mode_init);

enum lge_boot_mode_type lge_get_boot_mode(void)
{
	return lge_boot_mode;
}

int lge_get_normal_boot(void)
{
	int res;

	switch (lge_boot_mode) {
	case LGE_BOOT_MODE_NORMAL  :
	    res = 1;
	    break;
	default:
	    res = 0;
	    break;
	}
	return res;
}

int lge_get_factory_boot(void)
{
	int res;

	switch (lge_boot_mode) {
	case LGE_BOOT_MODE_FACTORY  :
	case LGE_BOOT_MODE_FACTORY2 :
	case LGE_BOOT_MODE_PIFBOOT  :
	case LGE_BOOT_MODE_PIFBOOT2 :
	case LGE_BOOT_MODE_MINIOS   :
	case LGE_BOOT_MODE_QEM_56K  :
	case LGE_BOOT_MODE_QEM_130K :
	    res = 1;
	    break;
	default:
	    res = 0;
	    break;
	}
	return res;
}

/* LAF mode */
static enum lge_laf_mode_type lge_laf_mode = LGE_LAF_MODE_NORMAL;
int __init lge_laf_mode_init(char *s)
{
	if (!strcmp(s, "56K"))
		lge_laf_mode = LGE_LAF_MODE_LAF;
	else if (!strcmp(s, "910K"))
		lge_laf_mode = LGE_LAF_MODE_LAF;
	else if (!strcmp(s, "USER"))
		lge_laf_mode = LGE_LAF_MODE_NORMAL;
	else
		lge_laf_mode = LGE_LAF_MODE_NORMAL;

	return 1;
}
__setup("androidboot.laf=", lge_laf_mode_init);

enum lge_laf_mode_type lge_get_laf_mode(void)
{
	return lge_laf_mode;
}

/*                                                                                            
                         
                         
                          
                                                                                             */
static hw_rev_type lge_bd_rev = HW_REV_PDA_B;


char *rev_str[] = {
	"rev_pda_a",
	"rev_pda_b",
	"rev_pda_c",
	"rev_d625_a",
	"rev_d625_a_1",
	"rev_d625_b",
	"rev_d625_c",
	"rev_d625_d",
	"rev_d625_e",
	"rev_d625_1_0",
	"rev_d625_1_1",
	"rev_d625_1_2",
	"rev_d625_1_3",
	"rev_d625_1_4",
	"reserved",
};

static int __init board_revno_setup(char *rev_info)
{
	int i;

	for (i = 0; i < HW_REV_MAX; i++) {
		if (!strncmp(rev_info, rev_str[i], 12)) {
			lge_bd_rev = (hw_rev_type) i;
			system_rev = lge_bd_rev;
			break;
		}
	}

	printk(KERN_INFO "BOARD : LGE %s \n", rev_str[lge_bd_rev]);
	return 1;
}
__setup("lge.rev=", board_revno_setup);

hw_rev_type lge_get_board_revno(void)
{
    return lge_bd_rev;
}

/*                                                                           */
#ifdef CONFIG_MACH_PDA
int lge_battery_info = BATT_UNKNOWN;

static int __init battery_information_setup(char *batt_info)
{
	if(!strcmp(batt_info, "17"))
		lge_battery_info = BATT_DS2704_N;
	else if(!strcmp(batt_info, "32"))
		lge_battery_info = BATT_DS2704_L;
	else if(!strcmp(batt_info, "73"))
		lge_battery_info = BATT_ISL6296_N;
	else if(!strcmp(batt_info, "94"))
		lge_battery_info = BATT_ISL6296_L;
	else if(!strcmp(batt_info, "48"))
		lge_battery_info = BATT_DS2704_C;
	else if(!strcmp(batt_info, "105"))
		lge_battery_info = BATT_ISL6296_C;
	else
		lge_battery_info = BATT_UNKNOWN;

	printk(KERN_INFO "[POWER] Battery : %s %d\n", batt_info, lge_battery_info);

	return 1;
}
__setup("lge.battid=", battery_information_setup);
#endif
/*                                                                         */

/*                                                                          */
#ifdef CONFIG_MACH_PDA

static int poweron_reason = PWR_ON_EVENT_KEYPAD;

static int __init power_on_status_info_get(char *pwr_reason)
{
	if(!strcmp(pwr_reason, "0"))
		poweron_reason = PWR_ON_EVENT_KEYPAD;
	else if(!strcmp(pwr_reason, "1"))
		poweron_reason = PWR_ON_EVENT_RTC;
	else if(!strcmp(pwr_reason, "2"))
		poweron_reason = PWR_ON_EVENT_CABLE;
	else if(!strcmp(pwr_reason, "3"))
		poweron_reason = PWR_ON_EVENT_SMPL;
	else if(!strcmp(pwr_reason, "4"))
		poweron_reason = PWR_ON_EVENT_WDOG;
	else if(!strcmp(pwr_reason, "5"))
		poweron_reason = PWR_ON_EVENT_USB_CHG;
	else if(!strcmp(pwr_reason, "6"))
		poweron_reason = PWR_ON_EVENT_WALL_CHG;
	else
		poweron_reason = PWR_ON_EVENT_HARD_RESET;

	printk(KERN_INFO "[POWER] PowerONReason : %s %d\n", pwr_reason, poweron_reason);

	return 1;
}
__setup("lge.pwr_on_rsn=", power_on_status_info_get);

int lge_get_poweron_reason(void)
{
   return poweron_reason;
}

bool is_56k_cable(void)
{
	int result = 0;
	static usb_cable_type old_cable_type = USB_CABLE_NO_USB;
	usb_cable_type cable_type = lge_pm_get_cable_type();

	if (cable_type == USB_CABLE_LT_56K) {
		result = 1;
	}
	else {
		result = 0;
	}

	old_cable_type = cable_type;
	return result;

}

bool is_factory_cable(void)
{
	int result = 0;
	static usb_cable_type old_cable_type = USB_CABLE_NO_USB;
	usb_cable_type cable_type = lge_pm_get_cable_type();


	if ((cable_type == USB_CABLE_LT_56K ||
		cable_type == USB_CABLE_LT_130K ||
		cable_type == USB_CABLE_LT_910K)) {
		result = 1;
	}
	else {
		result = 0;
	}
	/* This is working code */
	if(lge_get_boot_mode() >= LGE_BOOT_MODE_PIFBOOT2
		&& get_firstboot_factory_flag() == 1
		&& get_fcable_disconnect_check_flag() == 0)
	{
		result = 1;
	}
	if (cable_type != old_cable_type) {
		char buf[32];
		switch (cable_type) {
			case USB_CABLE_NO_USB:
				strcpy(buf, "USB_CABLE_NO_USB");
				break;

			case USB_CABLE_LT_56K:
				strcpy(buf, "CABLE_56K");
				break;

			case USB_CABLE_LT_130K:
				strcpy(buf, "CABLE_130K");
				break;

			case USB_CABLE_LT_910K:
				strcpy(buf, "CABLE_910K");
				break;

			case USB_CABLE_ABNORMAL_400MA:
				strcpy(buf, "USB_CABLE_ABNORMAL_400MA");
				break;

			default:
				sprintf(buf, "return %d", cable_type);
				break;
		}

		printk(KERN_INFO "[POWER] %s(): %s\n", __func__, buf);
	}

	old_cable_type = cable_type;
	return result;
}

int is_chg_done_irq(void)
{
	return is_chg_done;
}
void set_chg_done_flag(int enable)
{
	if(enable == 1){
		is_chg_done = 1;
		printk("[PIPAS] set chg done irq flag!\n");
	}
	else if (enable == 0) is_chg_done = 0;
	else { printk("[PIPAS] wrong value!\n");}
}
#endif
/*                                                                        */

#endif /*                              */
