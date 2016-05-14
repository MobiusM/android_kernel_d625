/*
 *  include/linux/rt8542-bled.h
 *  Include header file for Richtek RT8542 BLED file
 *
 *  Copyright (C) 2013 Richtek Electronics
 *  cy_huang <cy_huang@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_RT8542_BLED_H
#define __LINUX_RT8542_BLED_H

#define RT8542_BL_LEVEL_MASK	0x7F
#define RT8542_BLED_PWM_MASK	0x40
#define RT8542_BLED_EN_MASK	0x01
#define RT8542_MAX_BRIGHTNESS	127
#if defined(CONFIG_MACH_PDA)
#define ANDROID_MAX_BRIGHTNESS 255
#endif

int rt8542_internal_set_bl_enable(int);
int rt8542_internal_set_bl_brightness(int);
int rt8542_bl_enable(bool enable);

#endif /* __LINUX_RT8542_BLED_H */
