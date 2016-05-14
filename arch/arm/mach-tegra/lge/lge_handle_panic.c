/*
 * arch/arm/mach-tegra/lge/lge_handle_panic.c
 *
 * Copyright (C) 2012 LGE, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/io.h>
#include <asm/setup.h>
#include <linux/init.h>
#include "../board.h"
#include <lge_handle_panic.h>
#include <asm/cacheflush.h>
#include <mach/nvdumper.h>

#define PANIC_HANDLER_NAME        "panic-handler"

static uint32_t *restart_reason;
#if defined(CONFIG_CPU_CP15_MMU) && defined(CONFIG_LGE_HANDLE_PANIC_CP15)
unsigned long *cpu_crash_ctx = NULL;
#endif
static int dummy_arg;
int in_panic;

#ifdef CONFIG_LGE_MODEM_CRASH_HANDLER
unsigned char *modem_ctx;
static int modem_crash_flag = 0;
unsigned char modem_buf[255];
static int modem_crash(const char *val, struct kernel_param *kp)
{
	int i,buf_size;
       modem_crash_flag = 1;
	buf_size = strlen(val);
	printk("modem asserted by %s\n", val);
	for(i=0;i<buf_size;i++)
		iowrite8(val[i], &modem_ctx[i]);
       panic("modem_crash : Resetting the SoC");

       return 0;
}
module_param_call(modem_crash, modem_crash, param_get_bool, &modem_buf,\
               S_IWUSR | S_IRUGO);
#endif

#if defined(CONFIG_CPU_CP15_MMU) && defined(CONFIG_LGE_HANDLE_PANIC_CP15)
void lge_save_ctx(struct pt_regs* regs, unsigned int ctrl, unsigned int transbase, unsigned int dac)
{
	/* save cpu register for simulation */
	cpu_crash_ctx[0] = regs->ARM_r0;
	cpu_crash_ctx[1] = regs->ARM_r1;
	cpu_crash_ctx[2] = regs->ARM_r2;
	cpu_crash_ctx[3] = regs->ARM_r3;
	cpu_crash_ctx[4] = regs->ARM_r4;
	cpu_crash_ctx[5] = regs->ARM_r5;
	cpu_crash_ctx[6] = regs->ARM_r6;
	cpu_crash_ctx[7] = regs->ARM_r7;
	cpu_crash_ctx[8] = regs->ARM_r8;
	cpu_crash_ctx[9] = regs->ARM_r9;
	cpu_crash_ctx[10] = regs->ARM_r10;
	cpu_crash_ctx[11] = regs->ARM_fp;
	cpu_crash_ctx[12] = regs->ARM_ip;
	cpu_crash_ctx[13] = regs->ARM_sp;
	cpu_crash_ctx[14] = regs->ARM_lr;
	cpu_crash_ctx[15] = regs->ARM_pc;
	cpu_crash_ctx[16] = regs->ARM_cpsr;
	/* save mmu register for simulation */
	cpu_crash_ctx[17] = ctrl;
	cpu_crash_ctx[18] = transbase;
	cpu_crash_ctx[19] = dac;

}
#endif

#ifdef CONFIG_LGE_HIDDEN_RESET
static int hreset_mode = 0;
static int hreset_mode_set(const char *val, struct kernel_param *kp)
{
	int ret;
	int old_val = hreset_mode;

	ret = param_set_int(val, kp);

	if (ret)
		return ret;

	/* If download_mode is not zero or one, ignore. */
	if (hreset_mode >> 1) {
		hreset_mode = old_val;
		return -EINVAL;
	}

	return 0;
}
module_param_call(hreset_mode, hreset_mode_set, param_get_int,
			&hreset_mode, 0644);
#endif

void lge_set_restart_reason(unsigned int reason)
{
	iowrite32(reason, restart_reason);
	flush_cache_all();
	outer_flush_all();
}
EXPORT_SYMBOL(lge_set_restart_reason);

void lge_set_panic_reason(void)
{
#ifdef CONFIG_LGE_HIDDEN_RESET
	if (hreset_mode)
		lge_set_restart_reason(LGE_RB_MAGIC | LGE_HIDDEN_RESET);
	else
#endif
#ifdef CONFIG_LGE_MODEM_CRASH_HANDLER
	if (modem_crash_flag)
		lge_set_restart_reason(LGE_RB_MAGIC | LGE_MODEM_CRASH);
	else
#endif
		lge_set_restart_reason(LGE_RB_MAGIC | LGE_ERR_KERN);
}
EXPORT_SYMBOL(lge_set_panic_reason);

static int gen_bug(const char *val, struct kernel_param *kp)
{
	BUG();
	return 0;
}
module_param_call(gen_bug, gen_bug, param_get_bool, &dummy_arg,
		S_IWUSR | S_IRUGO);

static int gen_panic(const char *val, struct kernel_param *kp)
{
	panic("generate test-panic");
	return 0;
}
module_param_call(gen_panic, gen_panic, param_get_bool, &dummy_arg,\
		S_IWUSR | S_IRUGO);

static int panic_prep_restart(struct notifier_block *this,
			      unsigned long event, void *ptr)
{
	in_panic = 1;
	return NOTIFY_DONE;
}

static struct notifier_block panic_blk = {
	.notifier_call	= panic_prep_restart,
};

struct platform_device lge_handle_panic = {
	.name		= PANIC_HANDLER_NAME,
	.id		= -1,
};

static int __init lge_panic_handler_probe(struct platform_device *pdev)
{
	int ret;
#if defined(CONFIG_CPU_CP15_MMU) && defined(CONFIG_LGE_HANDLE_PANIC_CP15)
	void *ctx_buf;
#endif

	if (!nvdumper_reserved) {
		printk(KERN_INFO "nvdumper: not configured\n");
		return -ENOTSUPP;
	}
	restart_reason = ioremap_nocache(nvdumper_reserved,
			NVDUMPER_RESERVED_SIZE);
	if (!restart_reason) {
		printk(KERN_INFO "nvdumper: failed to ioremap memory "
			"at 0x%08lx\n", nvdumper_reserved);
		return -EIO;
	}
	printk ("reboot_reason : 0x%x \n", ioread32(restart_reason));

#ifdef CONFIG_LGE_HIDDEN_RESET
        if (hreset_mode)
	{
		/*                                            */
		/* when H/W hang ocurrs, tegra_wdt make the device reset as hidden reset */
		lge_set_restart_reason(LGE_RB_MAGIC | LGE_HIDDEN_RESET);
	}
#endif

	if (!modem_info_ctx_reserved) {
		printk(KERN_INFO "modem_info_ctx: not configured\n");
		return -ENOTSUPP;
	}

	modem_ctx = ioremap_nocache(modem_info_ctx_reserved,
		MODEM_CTX_RESERVED_SIZE);
	if (!modem_ctx) {
		printk(KERN_INFO "modem_ctx: failed to ioremap memory "
			"at 0x%08lx\n", modem_info_ctx_reserved);
		return -EIO;
	}

#if defined(CONFIG_CPU_CP15_MMU) && defined(CONFIG_LGE_HANDLE_PANIC_CP15)
	if (!cpu_crash_ctx_reserved) {
		printk(KERN_INFO "cpu_crash_ctx: not configured\n");
		return -ENOTSUPP;
	}
	ctx_buf = ioremap_nocache(cpu_crash_ctx_reserved,
			CPU_CRASH_CTX_SIZE);
	if (!ctx_buf) {
		printk(KERN_INFO "cpu_crash_ctx: failed to ioremap memory "
			"at 0x%08lx\n", cpu_crash_ctx_reserved);
		return -EIO;
	}
	cpu_crash_ctx = (unsigned long *)ctx_buf;
#endif

	ret = atomic_notifier_chain_register(&panic_notifier_list, &panic_blk);
	if (ret)
		return ret;

	ret = nvdumper_regdump_init();
	if (ret)
		return ret;

	return 0;
}

static int __devexit lge_panic_handler_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver panic_handler_driver = {
	.probe = lge_panic_handler_probe,
	.remove = __devexit_p(lge_panic_handler_remove),
	.driver = {
		.name = PANIC_HANDLER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init lge_panic_handler_init(void)
{
	platform_device_register(&lge_handle_panic);
	return platform_driver_register(&panic_handler_driver);
}

static void __exit lge_panic_handler_exit(void)
{
	platform_driver_unregister(&panic_handler_driver);
}

module_init(lge_panic_handler_init);
module_exit(lge_panic_handler_exit);

MODULE_DESCRIPTION("LGE panic handler driver");
MODULE_AUTHOR("SungEun Kim <cleaneye.kim@lge.com>");
MODULE_LICENSE("GPL");

