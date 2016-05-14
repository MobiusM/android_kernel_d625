/*
 * include/linux/thermal_backlight.h
 *
 * Generic thermal backlight dimming driver
 *
 * Copyright (c) 2014, NVIDIA Corporation. All rights reserved.
 *
 * Author: Jinyoung Park <jinyoungp@nvidia.com>
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

#ifndef __THERMAL_BACLKLIGHT_H__
#define __THERMAL_BACLKLIGHT_H__

#define THERMAL_BACKLIGHT_BRIGHTNESS_SIZE_MAX	10

struct thermal_backlight_dimming {
	int brightness[THERMAL_BACKLIGHT_BRIGHTNESS_SIZE_MAX];
	int brightness_size;
};

#ifdef CONFIG_THERMAL_BACKLIGHT
struct thermal_cooling_device *thermal_backlight_register(
		struct backlight_device *bl_dev,
		char *cdev_type,
		struct thermal_backlight_dimming *tbl_dimming);
void thermal_backlight_unregister(struct thermal_cooling_device *cdev);
#else
static inline struct thermal_backlight_device *thermal_backlight_register(
		struct backlight_device *bl_dev,
		char *cdev_type,
		struct thermal_backlight_dimming *tbl_dimming)
{
	return ERR_PTR(-ENODEV);
}

static inline void thermal_backlight_unregister(
		struct thermal_cooling_device *cdev)
{
	return;
}
#endif	/* CONFIG_THERMAL_BACKLIGHT */

#endif /* __THERMAL_BACLKLIGHT_H__ */
