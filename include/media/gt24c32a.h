#ifndef __GT24C32A_MAIN_H__
#define __GT24C32A_MAIN_H__

#include <linux/ioctl.h> /* For IOCTL macros */

#define GT24C32A_IOCTL_GET_E2PROM_DATA		_IOW('o', 1, struct gt24c32a_register_info)
#define GT24C32A_IOCTL_PUT_E2PROM_DATA		_IOW('o', 2, struct gt24c32a_register_info)

struct gt24c32a_register_info {
	u8 e2prom_data[4096];
	u16 reg_addr;
	u16 length;
} __attribute__ ((packed));

struct eeprom_power_rail {
	struct regulator *dvdd;
	struct regulator *avdd;
	struct regulator *iovdd;
};

struct gt24c32a_info {
	struct i2c_client *i2c_client;
	struct gt24c32a_register_info reg_info;
	struct eeprom_power_rail power;
} __attribute__ ((packed));

struct gt24c32a_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif

