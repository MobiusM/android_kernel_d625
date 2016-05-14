/*
 *  include/linux/rt8542-fled.h
 *  Include header file for Richtek RT8542 FLED file
 *
 *  Copyright (C) 2013 Richtek Electronics
 *  cy_huang <cy_huang@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_RT8542_FLED_H
#define __LINUX_RT8542_FLED_H

#define RT8542_FLED_EN_MASK	0x02
#define RT8542_FLED_MODE_MASK	0x04
#define RT8542_FLED_CTL2_MASK	0x3F
#define RT8542_FLED_STROBE_MASK	0x10
#define RT8542_MAX_BRIGHTNESS	255

enum fled_mode {
	FLED_MODE_TORCH,
	FLED_MODE_STROBE,
};

int rt8542_internal_set_fl_enable(int);
int rt8542_internal_set_fl_mode(enum fled_mode);

#endif /* __LINUX_RT8542_FLED_H */
