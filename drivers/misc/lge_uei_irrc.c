/*
UEI_IRRC_DRIVER_FOR_MSM9860
*/

#include <linux/err.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/types.h>
#include <linux/time.h>
#include <linux/version.h>
#include <mach/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/lge_uei_irrc.h>
#include <linux/module.h>
#include <mach/board_lge.h>

struct uei_irrc_pdata_type *irrc_pdata;

static int uei_irrc_probe(struct platform_device *pdev)
{
	int rc = 0;

	irrc_pdata = pdev->dev.platform_data;
	pr_info("%s\n", __func__);

	if (!irrc_pdata)
		return -EINVAL;

	rc = gpio_request(irrc_pdata->reset_gpio,"irrc_reset_n");

	if (rc) {
		printk(KERN_ERR "%s: irrc_reset_n %d request failed\n",
			__func__, irrc_pdata->reset_gpio);
		return rc;
	}

	rc = gpio_request(irrc_pdata->led_ldo_gpio,"irrc_led_ldo");

	if (rc) {
		printk(KERN_ERR "%s: irrc_led_ldo %d request failed\n",
			__func__, irrc_pdata->led_ldo_gpio);
		return rc;
	}

	rc = gpio_direction_output(irrc_pdata->reset_gpio, 0);

	if (rc) {
		printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=fg%d\n",
			__func__, irrc_pdata->reset_gpio, rc);
		return rc;
	}

	rc = gpio_direction_output(irrc_pdata->led_ldo_gpio, 1);

	if (rc) {
		printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=fg%d\n",
			__func__, irrc_pdata->led_ldo_gpio, rc);
		return rc;
	}

	gpio_set_value(irrc_pdata->reset_gpio, 1);
	gpio_set_value(irrc_pdata->led_ldo_gpio, 1);

	return rc;
}

static int uei_irrc_remove(struct platform_device *pdev)
{
	struct uei_irrc_pdata_type *pdata = platform_get_drvdata(pdev);
	printk(KERN_ERR "[IRRC] remove (err:%d)\n", 104);
	pdata = NULL;

	return 0;
}

static void uei_irrc_shutdown(struct platform_device *pdev)
{
	printk(KERN_INFO "%s\n", __func__);
}

static int uei_irrc_suspend(struct platform_device *pdev, pm_message_t state)
{
	gpio_set_value(irrc_pdata->led_ldo_gpio, 0);
	return 0;
}

static int uei_irrc_resume(struct platform_device *pdev)
{
	gpio_set_value(irrc_pdata->led_ldo_gpio, 1);
	return 0;
}

static struct platform_driver uei_irrc_driver = {
	.probe = uei_irrc_probe,
	.remove = uei_irrc_remove,
	.shutdown = uei_irrc_shutdown,
	.suspend = uei_irrc_suspend,
	.resume = uei_irrc_resume,
	.driver = {
		.name = UEI_IRRC_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init uei_irrc_init(void)
{
	int ret = 0;
	printk(KERN_INFO "%s\n", __func__);
	ret = platform_driver_register(&uei_irrc_driver);

	if (ret)
		printk(KERN_INFO "%s: init fail\n", __func__);

	return ret;
}

static void __exit uei_irrc_exit(void)
{
	printk(KERN_INFO "%s\n", __func__);
	platform_driver_unregister(&uei_irrc_driver);
}

module_init(uei_irrc_init);
module_exit(uei_irrc_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("UEI IrRC Driver");
MODULE_LICENSE("GPL");
