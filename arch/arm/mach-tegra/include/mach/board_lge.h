#ifndef __ASM_ARCH_MSM_BOARD_LGE_H
#define __ASM_ARCH_MSM_BOARD_LGE_H

/* detect cable type */
#define USB_CABLE_NOT_DEFINED 0

//                                                                        
typedef enum {
	HW_REV_PDA_A,
	HW_REV_PDA_B,
	HW_REV_PDA_C,
	HW_REV_D625_A,
	HW_REV_D625_A_1,
	HW_REV_D625_B,
	HW_REV_D625_C,
	HW_REV_D625_D,
	HW_REV_D625_E,  // reserved...
	HW_REV_D625_1_0,
	HW_REV_D625_1_1,
	HW_REV_D625_1_2,
	HW_REV_D625_1_3,
	HW_REV_D625_1_4,
	HW_REV_MAX
} hw_rev_type;

typedef enum {
    USB_CABLE_LT_56K = 6,
    USB_CABLE_LT_130K,
    USB_CABLE_400MA,
    USB_CABLE_DTC_500MA,
    USB_CABLE_ABNORMAL_400MA,
    USB_CABLE_LT_910K,
    USB_CABLE_NO_USB
} usb_cable_type;

typedef enum {
    _56K = 0,
    _130K,
    _910K,
    _NO_USB,
    _USER,
    CABLE_MAX
} usb_cable_enum_type;

struct cable_info_table {
	usb_cable_type type;
	int adc_val;
};

typedef struct {
    unsigned int adc_min;
    unsigned int adc_max;
    unsigned int cable_type;
} usb_cable_adc_type;

char *lge_get_cabletype_to_str (struct cable_info_table *info);
void lge_pm_get_cable_info(struct cable_info_table *info);
void lge_pm_read_cable_info(void);
hw_rev_type lge_get_board_revno(void);

/* select androidboot.mode */
enum lge_boot_mode_type {
	LGE_BOOT_MODE_NORMAL = 0,
	LGE_BOOT_MODE_CHARGER,
	LGE_BOOT_MODE_CHARGERLOGO,
	LGE_BOOT_MODE_QEM_NO_USB,
	LGE_BOOT_MODE_FACTORY,
	LGE_BOOT_MODE_FACTORY2,
	LGE_BOOT_MODE_PIFBOOT,
	LGE_BOOT_MODE_PIFBOOT2,	
	LGE_BOOT_MODE_MINIOS,    /*                          */
	LGE_BOOT_MODE_QEM_56K,   /*                    */
	LGE_BOOT_MODE_QEM_130K,
	LGE_BOOT_MODE_QEM_NORMAL,
};

int lge_get_factory_boot(void);
int lge_get_normal_boot(void);


enum lge_boot_mode_type lge_get_boot_mode(void);

enum lge_laf_mode_type {
	LGE_LAF_MODE_NORMAL = 0,
	LGE_LAF_MODE_LAF
};

enum lge_laf_mode_type lge_get_laf_mode(void);

/*                                                                           */
#ifdef CONFIG_MACH_PDA
enum {
	BATT_UNKNOWN = 0,
	BATT_DS2704_N = 17,
	BATT_DS2704_L = 32,
	BATT_ISL6296_N = 73,
	BATT_ISL6296_L = 94,
	BATT_DS2704_C = 48,
	BATT_ISL6296_C = 105,
};
extern int lge_battery_info;
#endif
/*                                                                         */

/*                                                                          */
#ifdef CONFIG_MACH_PDA
enum {
    PWR_ON_EVENT_KEYPAD = 0,
    PWR_ON_EVENT_RTC = 1,
    PWR_ON_EVENT_CABLE = 2,
    PWR_ON_EVENT_SMPL = 3,
    PWR_ON_EVENT_WDOG = 4,
    PWR_ON_EVENT_USB_CHG = 5,
    PWR_ON_EVENT_WALL_CHG = 6,
    PWR_ON_EVENT_HARD_RESET = 7,
};
//extern int poweron_reason;

bool is_factory_cable(void); /* This func. is designed for getting cable types like 56K, 130K and 910K */

bool is_56k_cable(void); /* This func. is designed for getting 56K or NOT */
int get_firstboot_factory_flag(void);

void clear_firstboot_factory_flag(void);

int get_fcable_disconnect_check_flag(void);

void clear_fcable_disconnect_check_flag(void);


#endif
/*                                                                        */
#endif
