/*
 * drivers/thermal/thermal_backlight.c
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/thermal.h>
#include <linux/backlight.h>
#include <linux/thermal_backlight.h>

struct thermal_backlight_device {
	struct thermal_cooling_device *cdev;
	struct backlight_device *bl_dev;

	struct thermal_backlight_dimming *tbl_dimming;
	int cur_state;

	struct dentry *dentry;
};

static DEFINE_MUTEX(tbl_lock);
static struct dentry *tbl_dentry_root;
static int tbl_count;

#ifdef CONFIG_DEBUG_FS
static int tbl_bl_dev_show(struct seq_file *s, void *p)
{
	struct thermal_backlight_device *tbl_dev = s->private;

	seq_printf(s, "%s\n", dev_name(&tbl_dev->bl_dev->dev));
	return 0;
}

static int tbl_bl_dev_open(struct inode *inode, struct file *file)
{
	return single_open(file, tbl_bl_dev_show, inode->i_private);
}

static const struct file_operations tbl_bl_dev_fops = {
	.open		= tbl_bl_dev_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static ssize_t tbl_dimming_read(struct file *file, char __user *user_buf,
				size_t count, loff_t *ppos)
{
	struct thermal_backlight_device *tbl_dev = file->private_data;
	struct thermal_backlight_dimming *tbl_dimming = tbl_dev->tbl_dimming;
	char buf[255];
	ssize_t size = 0;
	int i;

	for (i = 0; i < tbl_dimming->brightness_size; i++) {
		size += sprintf(buf + size, "%d ", tbl_dimming->brightness[i]);
		if (size >= (255 - 1))
			break;
		if ((i == (tbl_dimming->brightness_size - 1)) &&
				(size < (255 - 2))) {
			size += sprintf(buf + size, "\n");
			break;
		}
	}

	/* Truncate size */
	if (size >= 255)
		size = 255 - 1;

	return simple_read_from_buffer(user_buf, count, ppos, buf, size);
}

static ssize_t tbl_dimming_write(struct file *file, const char __user *user_buf,
				 size_t count, loff_t *ppos)
{
	struct thermal_backlight_device *tbl_dev = file->private_data;
	struct thermal_backlight_dimming tbl_dimming;
	char buf[255];
	ssize_t size;
	char *sp = buf, *ep;
	int i = 0;

	size = min(count, (sizeof(buf) - 1));
	if (copy_from_user(buf, user_buf, size))
		return -EFAULT;
	buf[size] = 0;

	while (*sp && (i < THERMAL_BACKLIGHT_BRIGHTNESS_SIZE_MAX)) {
		while (*sp == ' ')
			sp++;
		if (*sp == '\n')
			break;
		/* Only allow number */
		if (*sp < '0' || '9' < *sp)
			return -EINVAL;
		tbl_dimming.brightness[i++] = simple_strtol(sp, &ep, 10);
		sp = ep;
	}
	tbl_dimming.brightness_size = i;

	mutex_lock(&tbl_lock);
	memcpy(tbl_dev->tbl_dimming, &tbl_dimming, sizeof(tbl_dimming));
	mutex_unlock(&tbl_lock);

	return size;
}

static const struct file_operations tbl_dimming_fops = {
	.open = simple_open,
	.read = tbl_dimming_read,
	.write = tbl_dimming_write,
};

static int thermal_backlight_debugfs_init(
				struct thermal_backlight_device *tbl_dev)
{
	struct dentry *d_file;

	if (!tbl_dentry_root) {
		tbl_dentry_root = debugfs_create_dir("thermal-backlight", NULL);
		if (!tbl_dentry_root)
			return -ENOMEM;
	}

	tbl_dev->dentry = debugfs_create_dir(tbl_dev->cdev->type,
					     tbl_dentry_root);
	if (!tbl_dev->dentry)
		return -ENOMEM;

	d_file = debugfs_create_file("bl_dev", 0444, tbl_dev->dentry,
				     tbl_dev, &tbl_bl_dev_fops);
	if (!d_file)
		goto err;

	d_file = debugfs_create_file("dimming", 0644, tbl_dev->dentry,
				     tbl_dev, &tbl_dimming_fops);
	if (!d_file)
		goto err;

	return 0;

err:
	debugfs_remove_recursive(tbl_dev->dentry);
	return -ENOMEM;
}
#else
static int thermal_backlight_debugfs_init(
				struct thermal_backlight_device *tbl_dev)
(
	return 0;
)
#endif /*  CONFIG_DEBUG_FS */

static int thermal_backlight_get_max_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	struct thermal_backlight_device *tbl_dev = cdev->devdata;

	*state = tbl_dev->tbl_dimming->brightness_size;
	return 0;
}

static int thermal_backlight_get_cur_state(struct thermal_cooling_device *cdev,
					   unsigned long *state)
{
	struct thermal_backlight_device *tbl_dev = cdev->devdata;

	*state = tbl_dev->cur_state;
	return 0;
}

static int thermal_backlight_set_cur_state(struct thermal_cooling_device *cdev,
					   unsigned long state)
{
	struct thermal_backlight_device *tbl_dev = cdev->devdata;
	struct backlight_device *bl_dev = tbl_dev->bl_dev;
	struct thermal_backlight_dimming *tbl_dimming = tbl_dev->tbl_dimming;
	int throttle_brightness;

	if (tbl_dev->cur_state == state)
		return 0;

	if (tbl_dimming->brightness_size < state)
		return -EINVAL;

	mutex_lock(&tbl_lock);
	if (state > 0) {
		throttle_brightness = tbl_dimming->brightness[state - 1];
		if (throttle_brightness > bl_dev->props.max_brightness)
			throttle_brightness = bl_dev->props.max_brightness;
	} else
		throttle_brightness = bl_dev->props.max_brightness;

	backlight_update_throttle_brightness(bl_dev, throttle_brightness);
	tbl_dev->cur_state = state;
	mutex_unlock(&tbl_lock);

	return 0;
}

static struct thermal_cooling_device_ops const thermal_backlight_ops = {
	.get_max_state = thermal_backlight_get_max_state,
	.get_cur_state = thermal_backlight_get_cur_state,
	.set_cur_state = thermal_backlight_set_cur_state,
};

struct thermal_cooling_device *thermal_backlight_register(
		struct backlight_device *bl_dev,
		char *cdev_type,
		struct thermal_backlight_dimming *tbl_dimming)
{
	struct thermal_cooling_device *cdev;
	struct thermal_backlight_device *tbl_dev;
	int ret;

	if (!bl_dev || !cdev_type || !tbl_dimming)
		return ERR_PTR(-EINVAL);

	tbl_dev = kzalloc(sizeof(struct thermal_backlight_device), GFP_KERNEL);
	if (!tbl_dev)
		return ERR_PTR(-ENOMEM);

	tbl_dev->bl_dev = bl_dev;
	tbl_dev->tbl_dimming = tbl_dimming;
	tbl_dev->cur_state = 0;

	cdev = thermal_cooling_device_register(cdev_type, tbl_dev,
					       &thermal_backlight_ops);
	if (IS_ERR(cdev)) {
		ret = -EINVAL;
		goto err_free;
	}

	tbl_dev->cdev = cdev;
	thermal_backlight_debugfs_init(tbl_dev);
	mutex_lock(&tbl_lock);
	tbl_count++;
	mutex_unlock(&tbl_lock);

	return cdev;

err_free:
	kfree(tbl_dev);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL(thermal_backlight_register);

void thermal_backlight_unregister(struct thermal_cooling_device *cdev)
{
	struct thermal_backlight_device *tbl_dev = cdev->devdata;

	mutex_lock(&tbl_lock);
	if (tbl_dev->dentry)
		debugfs_remove_recursive(tbl_dev->dentry);
	thermal_cooling_device_unregister(cdev);
	kfree(tbl_dev);

	if (--tbl_count == 0 && tbl_dentry_root) {
		debugfs_remove_recursive(tbl_dentry_root);
		tbl_dentry_root = NULL;
	}
	mutex_unlock(&tbl_lock);
}
EXPORT_SYMBOL(thermal_backlight_unregister);
