/*
 * arch/arm/mach-tegra/nvdumper.c
 *
 * Copyright (c) 2011-2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/debugfs.h>
#include <linux/slab.h>

#include "board.h"
#include <mach/nvdumper.h>

#ifdef CONFIG_TEGRA_USE_NCT
#include <mach/nct.h>
#endif

#define NVDUMPER_CLEAN 0xf000caf3U
#define NVDUMPER_DIRTY 0xdeadbeefU

#define RW_MODE (S_IWUSR | S_IRUGO)

static uint32_t *nvdumper_ptr;

static int get_dirty_state(void)
{
	uint32_t val;

	val = ioread32(nvdumper_ptr);
	if (val == NVDUMPER_DIRTY)
		return 1;
	else if (val == NVDUMPER_CLEAN)
		return 0;
	else
		return -1;
}

static void set_dirty_state(int dirty)
{
	if (dirty)
		iowrite32(NVDUMPER_DIRTY, nvdumper_ptr);
	else
		iowrite32(NVDUMPER_CLEAN, nvdumper_ptr);
}

static int nvdumper_reboot_cb(struct notifier_block *nb,
		unsigned long event, void *unused)
{
	printk(KERN_INFO "nvdumper: rebooting cleanly.\n");
	set_dirty_state(0);
	return NOTIFY_DONE;
}

struct notifier_block nvdumper_reboot_notifier = {
	.notifier_call = nvdumper_reboot_cb,
};

#ifdef CONFIG_DEBUG_FS
static struct dentry *nvdumper_debugfs_root;

/* bug test code */
static int bug_get(void *data, u64 *val)
{
	BUG();
	return 0;
}
static int bug_set(void *data, u64 val)
{
	BUG();
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(bug_fops, bug_get, bug_set, "%llu\n");

/* panic test code */
static int panic_get(void *data, u64 *val)
{
	panic("NVDUMPER TEST");
	return 0;
}
static int panic_set(void *data, u64 val)
{
	panic("NVDUMPER TEST");
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(panic_fops, panic_get, panic_set, "%llu\n");

static int __init nvdumper_debugfs_init(void)
{
	int ret = -ENOMEM;
	struct dentry *dir;
	struct dentry *d;

	dir = debugfs_create_dir("nvdumper", NULL);
	if (!dir)
		return ret;
	nvdumper_debugfs_root = dir;

	d = debugfs_create_file("test_bug", RW_MODE, dir, NULL, &bug_fops);
	if (!d)
		goto err_out;
	d = debugfs_create_file("test_panic", RW_MODE, dir, NULL, &panic_fops);
	if (!d)
		goto err_out;

	return 0;

err_out:
	debugfs_remove_recursive(nvdumper_debugfs_root);
	return ret;
}
#endif /* CONFIG_DEBUG_FS */

static int __init nvdumper_init(void)
{
	int ret, dirty;
#ifdef CONFIG_TEGRA_USE_NCT
	union nct_item_type *item;
#endif
	if (!tegra_nvdumper_start) {
		printk(KERN_INFO "nvdumper: not configured\n");
		return -ENOTSUPP;
	}
	nvdumper_ptr = ioremap_nocache(tegra_nvdumper_start,
			tegra_nvdumper_size);
	if (!nvdumper_ptr) {
		printk(KERN_INFO "nvdumper: failed to ioremap memory " \
			"at %08lu@0x%08lx\n", tegra_nvdumper_size,
			tegra_nvdumper_start);
		return -EIO;
	}

	ret = register_reboot_notifier(&nvdumper_reboot_notifier);
	if (ret)
		goto err_out1;

	ret = nvdumper_regdump_init();
	if (ret)
		goto err_out2;

	dirty = get_dirty_state();
	switch (dirty) {
	case 0:
		printk(KERN_INFO "nvdumper: last reboot was clean\n");
		break;
	case 1:
		printk(KERN_INFO "nvdumper: last reboot was dirty\n");
		break;
	default:
		printk(KERN_INFO "nvdumper: last reboot was unknown\n");
		break;
	}

#ifdef CONFIG_DEBUG_FS
	ret = nvdumper_debugfs_init();
	if (ret)
		pr_err("nvdumper_debugfs_init failed\n");
#endif

	set_dirty_state(1); /* TODO: remove this */

#ifdef CONFIG_TEGRA_USE_NCT
	item = kzalloc(sizeof(*item), GFP_KERNEL);
	if (!item) {
		pr_err("failed to allocate memory\n");
		goto err_out3;
	}

	ret = tegra_nct_read_item(NCT_ID_RAMDUMP, item);
	if (ret < 0) {
		pr_err("%s: NCT read failure\n", __func__);
		kfree(item);
		goto err_out3;
	}

	pr_info("%s: RAMDUMP flag(%d) from NCT\n",
			__func__, item->ramdump.flag);
	if (item->ramdump.flag == 1)
		set_dirty_state(1);
	else
		set_dirty_state(0);

	kfree(item);

	return 0;

err_out3:

#else
	set_dirty_state(1);
	return 0;
#endif

err_out2:
	unregister_reboot_notifier(&nvdumper_reboot_notifier);
err_out1:
	iounmap(nvdumper_ptr);

	return ret;
}

static void __exit nvdumper_exit(void)
{
	set_dirty_state(0);
	nvdumper_regdump_exit();
	unregister_reboot_notifier(&nvdumper_reboot_notifier);
	iounmap(nvdumper_ptr);
}

arch_initcall(nvdumper_init);
module_exit(nvdumper_exit);

MODULE_LICENSE("GPL");
