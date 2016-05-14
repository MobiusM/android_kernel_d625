/*
 * Gadget Driver for Android
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *         Benoit Goby <benoit@android.com>
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/platform_device.h>

#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>

#include "gadget_chips.h"

/*
 * Kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */

#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"
#include "composite.c"

#include "f_nvusb.c"
#include "f_fs.c"
#include "f_audio_source.c"
#include "f_mass_storage.c"
#ifdef CONFIG_USB_G_LGE_ANDROID
#include "u_lgeusb.h"
#include "f_serial.c"
#define CONFIG_LAF_G_DRIVER 1
#endif

#include "u_serial.c"
#include "f_acm.c"
#include "f_acm_nvidia.c"
#include "f_adb.c"
#include "f_mtp.c"
#include "f_accessory.c"

#ifdef CONFIG_USB_ANDROID_CDC_ECM
#include "f_ecm.c"
#include "u_ether.c"
#else
#define USB_ETH_RNDIS y
#include "f_rndis.c"
#include "rndis.c"
#include "u_ether.c"
#endif

#ifdef CONFIG_LAF_G_DRIVER
#include "f_laf.c"
#endif

#ifdef CONFIG_USB_G_LGE_ANDROID_AUTORUN
#include "f_charge_only.c"
#endif
#ifdef CONFIG_USB_G_LGE_ANDROID_NCM
#include "f_ncm.c"
#endif

#include <mach/board_lge.h>
#include <linux/platform_data/lge_android_usb.h>

MODULE_AUTHOR("Mike Lockwood");
MODULE_DESCRIPTION("Android Composite USB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

static const char longname[] = "Gadget Android";

/* Default vendor and product IDs, overridden by userspace */
#ifdef CONFIG_USB_G_LGE_ANDROID
#define VENDOR_ID		0x1004
#define PRODUCT_ID		0x61F1
#define BCD_DEVICE		0x0229

#define ACM_ENABLED		0x01
#define SERIAL_ENABLED	0x02
#else
#define VENDOR_ID		0x18D1
#define PRODUCT_ID		0x0001
#endif

struct android_usb_function {
	char *name;
	void *config;

	struct device *dev;
	char *dev_name;
	struct device_attribute **attributes;

	/* for android_dev.enabled_functions */
	struct list_head enabled_list;

	/* Optional: initialization during gadget bind */
	int (*init)(struct android_usb_function *, struct usb_composite_dev *);
	/* Optional: cleanup during gadget unbind */
	void (*cleanup)(struct android_usb_function *);
	/* Optional: called when the function is added the list of
	 *		enabled functions */
	void (*enable)(struct android_usb_function *);
	/* Optional: called when it is removed */
	void (*disable)(struct android_usb_function *);

	int (*bind_config)(struct android_usb_function *,
			   struct usb_configuration *);

	/* Optional: called when the configuration is removed */
	void (*unbind_config)(struct android_usb_function *,
			      struct usb_configuration *);
	/* Optional: handle ctrl requests before the device is configured */
	int (*ctrlrequest)(struct android_usb_function *,
					struct usb_composite_dev *,
					const struct usb_ctrlrequest *);
};

struct android_dev {
	struct android_usb_function **functions;
	struct list_head enabled_functions;
	struct usb_composite_dev *cdev;
	struct device *dev;
	struct android_usb_platform_data *pdata;

	bool enabled;
	int disable_depth;
	struct mutex mutex;
	bool connected;
	bool sw_connected;
	struct work_struct work;
#ifdef CONFIG_USB_G_LGE_ANDROID
	int serial_flags;
#endif
	char ffs_aliases[256];
#ifdef CONFIG_USB_G_LGE_ANDROID_AUTORUN
	bool check_charge_only;
#endif
#ifdef CONFIG_MACH_PDA
	bool check_pif;
#endif
};

static struct class *android_class;
static struct android_dev *_android_dev;
static int android_bind_config(struct usb_configuration *c);
static void android_unbind_config(struct usb_configuration *c);

/* string IDs are assigned dynamically */
#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2

#ifdef CONFIG_USB_G_LGE_ANDROID
#define LGE_PIF_VID 0x1004
#define LGE_PIF_PID 0x6000
#define LGE_PIF_SN 0
#endif

static char manufacturer_string[256];
static char product_string[256];
static char serial_string[256];

#ifdef CONFIG_USB_G_LGE_ANDROID_AUTORUN
#define CHARGE_ONLY_STRING_IDX		3
static char charge_only_string[256];
#endif
/* String Table */
static struct usb_string strings_dev[] = {
	[STRING_MANUFACTURER_IDX].s = manufacturer_string,
	[STRING_PRODUCT_IDX].s = product_string,
	[STRING_SERIAL_IDX].s = serial_string,
#ifdef CONFIG_USB_G_LGE_ANDROID_AUTORUN
	[CHARGE_ONLY_STRING_IDX].s = charge_only_string,
#endif
	{  }			/* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength              = sizeof(device_desc),
	.bDescriptorType      = USB_DT_DEVICE,
	.bcdUSB               = __constant_cpu_to_le16(0x0200),
#ifdef CONFIG_USB_G_LGE_ANDROID
	.bDeviceClass         = USB_CLASS_MISC,
	.bDeviceSubClass      = 0x02,
	.bDeviceProtocol      = 0x01,
#else
	.bDeviceClass         = USB_CLASS_PER_INTERFACE,
#endif
	.idVendor             = __constant_cpu_to_le16(VENDOR_ID),
	.idProduct            = __constant_cpu_to_le16(PRODUCT_ID),
	.bcdDevice            = __constant_cpu_to_le16(0xffff),
	.bNumConfigurations   = 1,
};

static struct usb_configuration android_config_driver = {
	.label		= "android",
	.unbind		= android_unbind_config,
	.bConfigurationValue = 1,
};
enum android_device_state {
	USB_DISCONNECTED,
	USB_CONNECTED,
	USB_CONFIGURED,
};


static void android_work(struct work_struct *data)
{
	struct android_dev *dev = container_of(data, struct android_dev, work);
	struct usb_composite_dev *cdev = dev->cdev;
	char *disconnected[2] = { "USB_STATE=DISCONNECTED", NULL };
	char *connected[2]    = { "USB_STATE=CONNECTED", NULL };
	char *configured[2]   = { "USB_STATE=CONFIGURED", NULL };
	char **uevent_envp = NULL;
	static enum android_device_state last_uevent, next_state;
	unsigned long flags;

	spin_lock_irqsave(&cdev->lock, flags);
	if (cdev->config) {
		uevent_envp = configured;
		next_state = USB_CONFIGURED;
	} else if (dev->connected != dev->sw_connected) {
		uevent_envp = dev->connected ? connected : disconnected;
		next_state = dev->connected ? USB_CONNECTED : USB_DISCONNECTED;
	}
	dev->sw_connected = dev->connected;
	spin_unlock_irqrestore(&cdev->lock, flags);

	if (uevent_envp) {
		/*
		 * Some userspace modules, e.g. MTP, work correctly only if
		 * CONFIGURED uevent is preceded by DISCONNECT uevent.
		 * Check if we missed sending out a DISCONNECT uevent. This can
		 * happen if host PC resets and configures device really quick.
		 */
		if (((uevent_envp == connected) &&
		      (last_uevent != USB_DISCONNECTED)) ||
		    ((uevent_envp == configured) &&
		      (last_uevent == USB_CONFIGURED))) {
			pr_info("%s: sent missed DISCONNECT event\n", __func__);
			kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE,
								disconnected);
			msleep(20);
		}
		/*
		 * Before sending out CONFIGURED uevent give function drivers
		 * a chance to wakeup userspace threads and notify disconnect
		 */
		if (uevent_envp == configured)
			msleep(50);

		kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE, uevent_envp);
		last_uevent = next_state;
		pr_info("%s: sent uevent %s\n", __func__, uevent_envp[0]);
	} else {
		pr_info("%s: did not send uevent (%d %d %p)\n", __func__,
			 dev->connected, dev->sw_connected, cdev->config);
	}
}

static void android_enable(struct android_dev *dev)
{
	struct usb_composite_dev *cdev = dev->cdev;

	if (WARN_ON(!dev->disable_depth))
		return;

	if (--dev->disable_depth == 0) {
		usb_add_config(cdev, &android_config_driver,
					android_bind_config);
		usb_gadget_connect(cdev->gadget);
	}
}

static void android_disable(struct android_dev *dev)
{
	struct usb_composite_dev *cdev = dev->cdev;

	if (dev->disable_depth++ == 0) {
		usb_gadget_disconnect(cdev->gadget);
		/* Cancel pending control requests */
		usb_ep_dequeue(cdev->gadget->ep0, cdev->req);
		usb_remove_config(cdev, &android_config_driver);
	}
}

/*-------------------------------------------------------------------------*/
/* Supported functions initialization */

struct functionfs_config {
	bool opened;
	bool enabled;
	struct ffs_data *data;
};

static int ffs_function_init(struct android_usb_function *f,
			     struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct functionfs_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	return functionfs_init();
}

static void ffs_function_cleanup(struct android_usb_function *f)
{
	functionfs_cleanup();
	kfree(f->config);
}

static void ffs_function_enable(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;
	struct functionfs_config *config = f->config;

	config->enabled = true;

	/* Disable the gadget until the function is ready */
	if (!config->opened)
		android_disable(dev);
}

static void ffs_function_disable(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;
	struct functionfs_config *config = f->config;

	config->enabled = false;

	/* Balance the disable that was called in closed_callback */
	if (!config->opened)
		android_enable(dev);
}

static int ffs_function_bind_config(struct android_usb_function *f,
				    struct usb_configuration *c)
{
	struct functionfs_config *config = f->config;
	return functionfs_bind_config(c->cdev, c, config->data);
}

static ssize_t
ffs_aliases_show(struct device *pdev, struct device_attribute *attr, char *buf)
{
	struct android_dev *dev = _android_dev;
	int ret;

	mutex_lock(&dev->mutex);
	ret = sprintf(buf, "%s\n", dev->ffs_aliases);
	mutex_unlock(&dev->mutex);

	return ret;
}

static ssize_t
ffs_aliases_store(struct device *pdev, struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct android_dev *dev = _android_dev;
	char buff[256];

	mutex_lock(&dev->mutex);

	if (dev->enabled) {
		mutex_unlock(&dev->mutex);
		return -EBUSY;
	}

	strlcpy(buff, buf, sizeof(buff));
	strlcpy(dev->ffs_aliases, strim(buff), sizeof(dev->ffs_aliases));

	mutex_unlock(&dev->mutex);

	return size;
}

static DEVICE_ATTR(aliases, S_IRUGO | S_IWUSR, ffs_aliases_show,
					       ffs_aliases_store);
static struct device_attribute *ffs_function_attributes[] = {
	&dev_attr_aliases,
	NULL
};

static struct android_usb_function ffs_function = {
	.name		= "ffs",
	.init		= ffs_function_init,
	.enable		= ffs_function_enable,
	.disable	= ffs_function_disable,
	.cleanup	= ffs_function_cleanup,
	.bind_config	= ffs_function_bind_config,
	.attributes	= ffs_function_attributes,
};

static int functionfs_ready_callback(struct ffs_data *ffs)
{
	struct android_dev *dev = _android_dev;
	struct functionfs_config *config = ffs_function.config;
	int ret = 0;

	mutex_lock(&dev->mutex);

	ret = functionfs_bind(ffs, dev->cdev);
	if (ret)
		goto err;

	config->data = ffs;
	config->opened = true;

	if (config->enabled)
		android_enable(dev);

err:
	mutex_unlock(&dev->mutex);
	return ret;
}

static void functionfs_closed_callback(struct ffs_data *ffs)
{
	struct android_dev *dev = _android_dev;
	struct functionfs_config *config = ffs_function.config;

	mutex_lock(&dev->mutex);

	if (config->enabled)
		android_disable(dev);

	config->opened = false;
	config->data = NULL;

	functionfs_unbind(ffs);

	mutex_unlock(&dev->mutex);
}

static int functionfs_check_dev_callback(const char *dev_name)
{
	return 0;
}

struct adb_data {
	bool opened;
	bool enabled;
};

static int
adb_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct adb_data), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	return adb_setup();
}

static void adb_function_cleanup(struct android_usb_function *f)
{
	adb_cleanup();
	kfree(f->config);
}

static int
adb_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	return adb_bind_config(c);
}

static void adb_android_function_enable(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;
	struct adb_data *data = f->config;

	data->enabled = true;

	/* Disable the gadget until adbd is ready */
	if (!data->opened)
		android_disable(dev);
}

static void adb_android_function_disable(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;
	struct adb_data *data = f->config;

	data->enabled = false;

	/* Balance the disable that was called in closed_callback */
	if (!data->opened)
		android_enable(dev);
}

static struct android_usb_function adb_function = {
	.name		= "adb",
	.enable		= adb_android_function_enable,
	.disable	= adb_android_function_disable,
	.init		= adb_function_init,
	.cleanup	= adb_function_cleanup,
	.bind_config	= adb_function_bind_config,
};

#ifdef CONFIG_USB_G_LGE_ANDROID
/* Serial ports*/
#define MAX_LG_SERIAL_INSTANCES  1
/* Restrict Max acm instances to 1 for consistent LG serial port behavior */
#define MAX_ACM_INSTANCES 1
#endif
static void adb_ready_callback(void)
{
	struct android_dev *dev = _android_dev;
	struct adb_data *data = adb_function.config;

	mutex_lock(&dev->mutex);

	data->opened = true;

	if (data->enabled)
		android_enable(dev);

	mutex_unlock(&dev->mutex);
}

static void adb_closed_callback(void)
{
	struct android_dev *dev = _android_dev;
	struct adb_data *data = adb_function.config;

	mutex_lock(&dev->mutex);
	data->opened = false;

	if (data->enabled)
		android_disable(dev);

	mutex_unlock(&dev->mutex);
}


#ifdef CONFIG_LAF_G_DRIVER
/*-------------------------------------------------------------------------*/
/* Supported functions initialization */

/* laf */
struct laf_data {
	bool opened;
	bool enabled;
};

static int
laf_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct laf_data), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	return laf_setup();
}

static void laf_function_cleanup(struct android_usb_function *f)
{
	laf_cleanup();
	kfree(f->config);
}

static int
laf_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	return laf_bind_config(c);
}

static void laf_android_function_enable(struct android_usb_function *f)
{
	struct laf_data *data = f->config;

	data->enabled = true;

	pr_err("laf_android_function_enable");
}

static void laf_android_function_disable(struct android_usb_function *f)
{
	struct laf_data *data = f->config;

	data->enabled = false;
}

static struct android_usb_function laf_function = {
	.name		= "laf",
	.enable		= laf_android_function_enable,
	.disable	= laf_android_function_disable,
	.init		= laf_function_init,
	.cleanup	= laf_function_cleanup,
	.bind_config	= laf_function_bind_config,
};

static void laf_ready_callback(void)
{
	struct laf_data *data = laf_function.config;

	data->opened = true;
}

static void laf_closed_callback(void)
{
	struct laf_data *data = laf_function.config;

	data->opened = false;
}
#endif

struct acm_function_config {
	int instances;
};

static int
acm_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
#ifdef CONFIG_USB_G_LGE_ANDROID
	int ret = 0;
	struct android_dev *dev = _android_dev;
#endif

	f->config = kzalloc(sizeof(struct acm_function_config), GFP_KERNEL);

	if (!f->config)
		return -ENOMEM;
#ifdef CONFIG_USB_G_LGE_ANDROID
	if(!dev->serial_flags)
	{
		ret = gserial_setup(cdev->gadget, MAX_ACM_INSTANCES+MAX_LG_SERIAL_INSTANCES);
	}

	if(ret == 0)
		dev->serial_flags |= ACM_ENABLED; /* ACM_ENABLED 0x01 */

	/* default ACM instances to 1 for LG configuration*/
	((struct acm_function_config *)f->config)->instances = MAX_ACM_INSTANCES;

	return ret;
#else
	return gserial_setup(cdev->gadget, MAX_ACM_INSTANCES);
#endif
}

static void acm_function_cleanup(struct android_usb_function *f)
{
#ifdef CONFIG_USB_G_LGE_ANDROID
	struct android_dev *dev = _android_dev;
	dev->serial_flags &= ~ACM_ENABLED; /* ACM_ENABLED 0x01 */

	if(!dev->serial_flags)
		gserial_cleanup();
#else
	gserial_cleanup();
#endif
	kfree(f->config);
	f->config = NULL;
}

static int
acm_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	int i;
	int ret = 0;
	struct acm_function_config *config = f->config;

	for (i = 0; i < config->instances; i++) {
		ret = acm_bind_config(c, i);
		if (ret) {
			pr_err("Could not bind acm%u config\n", i);
			break;
		}
	}

	return ret;
}

static ssize_t acm_instances_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct acm_function_config *config = f->config;
	return sprintf(buf, "%d\n", config->instances);
}

static ssize_t acm_instances_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct acm_function_config *config = f->config;
	int value;

	sscanf(buf, "%d", &value);
	if (value > MAX_ACM_INSTANCES)
		value = MAX_ACM_INSTANCES;
	config->instances = value;
	return size;
}

static DEVICE_ATTR(instances, S_IRUGO | S_IWUSR, acm_instances_show,
						 acm_instances_store);
static struct device_attribute *acm_function_attributes[] = {
	&dev_attr_instances,
	NULL
};

static struct android_usb_function acm_function = {
	.name		= "acm",
	.init		= acm_function_init,
	.cleanup	= acm_function_cleanup,
	.bind_config	= acm_function_bind_config,
	.attributes	= acm_function_attributes,
};

#ifdef CONFIG_USB_G_LGE_ANDROID
static int serial_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
    int ret = 0;
    struct android_dev *dev = _android_dev;

    if(!dev->serial_flags)
        ret = gserial_setup(cdev->gadget, MAX_ACM_INSTANCES+MAX_LG_SERIAL_INSTANCES);

	if(ret == 0)
        dev->serial_flags |= SERIAL_ENABLED; /* 0x02 */

    return ret;
}
static int serial_function_bind_config(struct android_usb_function *f,struct usb_configuration *c)
{
	return gser_bind_config(c, MAX_LG_SERIAL_INSTANCES);
}

static void serial_function_cleanup(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;

	dev->serial_flags &= ~SERIAL_ENABLED; /* 0x02 */
	if(!dev->serial_flags)
        gserial_cleanup();
}

static struct android_usb_function serial_function = {
	.name = "serial",
	.init = serial_function_init,
	.cleanup = serial_function_cleanup,
	.bind_config = serial_function_bind_config,
};
#endif

#define MAX_ACM_INSTANCES_NVIDIA 2

struct acm_function_config_nv {
	int instances;
};

static int
acm_function_init_nv(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct acm_function_config_nv), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	return gserial_setup(cdev->gadget, MAX_ACM_INSTANCES_NVIDIA);
}

static void acm_function_cleanup_nv(struct android_usb_function *f)
{
	gserial_cleanup();
	kfree(f->config);
	f->config = NULL;
}

static int
acm_function_bind_config_nv(struct android_usb_function *f,
		struct usb_configuration *c)
{
	int i;
	int ret = 0;
	struct acm_function_config_nv *config = f->config;

	for (i = 0; i < config->instances; i++) {
		ret = acm_bind_config_nv(c, i);
		if (ret) {
			pr_err("Could not bind acm%u config\n", i);
			break;
		}
	}

	return ret;
}

static ssize_t acm_nvinstances_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct acm_function_config_nv *config = f->config;
	return sprintf(buf, "%d\n", config->instances);
}

static ssize_t acm_nvinstances_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct acm_function_config_nv *config = f->config;
	int value;

	sscanf(buf, "%d", &value);
	if (value > MAX_ACM_INSTANCES_NVIDIA)
		value = MAX_ACM_INSTANCES_NVIDIA;
	config->instances = value;
	return size;
}

static DEVICE_ATTR(nvinstances, S_IRUGO | S_IWUSR, acm_nvinstances_show,
						 acm_nvinstances_store);
static struct device_attribute *acm_function_attributes_nv[] = {
	&dev_attr_instances,
	NULL
};

static struct android_usb_function acm_nv_function = {
	.name		= "acm_nv",
	.init		= acm_function_init_nv,
	.cleanup	= acm_function_cleanup_nv,
	.bind_config	= acm_function_bind_config_nv,
	.attributes	= acm_function_attributes_nv,
};


static int
mtp_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	return mtp_setup();
}

static void mtp_function_cleanup(struct android_usb_function *f)
{
	mtp_cleanup();
}

static int
mtp_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	return mtp_bind_config(c, false);
}

static int
ptp_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	/* nothing to do - initialization is handled by mtp_function_init */
	return 0;
}

static void ptp_function_cleanup(struct android_usb_function *f)
{
	/* nothing to do - cleanup is handled by mtp_function_cleanup */
}

static int
ptp_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	return mtp_bind_config(c, true);
}

static int mtp_function_ctrlrequest(struct android_usb_function *f,
					struct usb_composite_dev *cdev,
					const struct usb_ctrlrequest *c)
{
	return mtp_ctrlrequest(cdev, c);
}

static int ptp_function_ctrlrequest(struct android_usb_function *f,
						struct usb_composite_dev *cdev,
						const struct usb_ctrlrequest *c)
{
	return mtp_ctrlrequest(cdev, c);
}

static struct android_usb_function mtp_function = {
	.name		= "mtp",
	.init		= mtp_function_init,
	.cleanup	= mtp_function_cleanup,
	.bind_config	= mtp_function_bind_config,
	.ctrlrequest	= mtp_function_ctrlrequest,
};

/* PTP function is same as MTP with slightly different interface descriptor */
static struct android_usb_function ptp_function = {
	.name		= "ptp",
	.init		= ptp_function_init,
	.cleanup	= ptp_function_cleanup,
	.bind_config	= ptp_function_bind_config,
	.ctrlrequest	= ptp_function_ctrlrequest,
};

#ifndef CONFIG_USB_ANDROID_CDC_ECM
struct rndis_function_config {
	u8      ethaddr[ETH_ALEN];
	u32     vendorID;
	char	manufacturer[256];
	/* "Wireless" RNDIS; auto-detected by Windows */
	bool	wceis;
};

static int
rndis_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct rndis_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;
	return 0;
}

static void rndis_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int
rndis_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	int ret;
	struct rndis_function_config *rndis = f->config;

	if (!rndis) {
		pr_err("%s: rndis_pdata\n", __func__);
		return -1;
	}

	pr_info("%s MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		rndis->ethaddr[0], rndis->ethaddr[1], rndis->ethaddr[2],
		rndis->ethaddr[3], rndis->ethaddr[4], rndis->ethaddr[5]);

	ret = gether_setup_name(c->cdev->gadget, rndis->ethaddr, "rndis");
	if (ret) {
		pr_err("%s: gether_setup failed\n", __func__);
		return ret;
	}

	if (rndis->wceis) {
		/* "Wireless" RNDIS; auto-detected by Windows */
		rndis_iad_descriptor.bFunctionClass =
						USB_CLASS_WIRELESS_CONTROLLER;
		rndis_iad_descriptor.bFunctionSubClass = 0x01;
		rndis_iad_descriptor.bFunctionProtocol = 0x03;
		rndis_control_intf.bInterfaceClass =
						USB_CLASS_WIRELESS_CONTROLLER;
		rndis_control_intf.bInterfaceSubClass =	 0x01;
		rndis_control_intf.bInterfaceProtocol =	 0x03;
	}

	return rndis_bind_config_vendor(c, rndis->ethaddr, rndis->vendorID,
					   rndis->manufacturer);
}

static void rndis_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	gether_cleanup();
}

static ssize_t rndis_manufacturer_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return sprintf(buf, "%s\n", config->manufacturer);
}

static ssize_t rndis_manufacturer_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;

	if (size >= sizeof(config->manufacturer))
		return -EINVAL;
	if (sscanf(buf, "%s", config->manufacturer) == 1)
		return size;
	return -1;
}

static DEVICE_ATTR(manufacturer, S_IRUGO | S_IWUSR, rndis_manufacturer_show,
						    rndis_manufacturer_store);

static ssize_t rndis_wceis_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return sprintf(buf, "%d\n", config->wceis);
}

static ssize_t rndis_wceis_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	int value;

	if (sscanf(buf, "%d", &value) == 1) {
		config->wceis = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(wceis, S_IRUGO | S_IWUSR, rndis_wceis_show,
					     rndis_wceis_store);

static ssize_t rndis_ethaddr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *rndis = f->config;
	return sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		rndis->ethaddr[0], rndis->ethaddr[1], rndis->ethaddr[2],
		rndis->ethaddr[3], rndis->ethaddr[4], rndis->ethaddr[5]);
}

static ssize_t rndis_ethaddr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *rndis = f->config;

	if (sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		    (int *)&rndis->ethaddr[0], (int *)&rndis->ethaddr[1],
		    (int *)&rndis->ethaddr[2], (int *)&rndis->ethaddr[3],
		    (int *)&rndis->ethaddr[4], (int *)&rndis->ethaddr[5]) == 6)
		return size;
	return -EINVAL;
}

static DEVICE_ATTR(ethaddr, S_IRUGO | S_IWUSR, rndis_ethaddr_show,
					       rndis_ethaddr_store);

static ssize_t rndis_vendorID_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return sprintf(buf, "%04x\n", config->vendorID);
}

static ssize_t rndis_vendorID_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	int value;

	if (sscanf(buf, "%04x", &value) == 1) {
		config->vendorID = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(vendorID, S_IRUGO | S_IWUSR, rndis_vendorID_show,
						rndis_vendorID_store);

static struct device_attribute *rndis_function_attributes[] = {
	&dev_attr_manufacturer,
	&dev_attr_wceis,
	&dev_attr_ethaddr,
	&dev_attr_vendorID,
	NULL
};

static struct android_usb_function rndis_function = {
	.name		= "rndis",
	.init		= rndis_function_init,
	.cleanup	= rndis_function_cleanup,
	.bind_config	= rndis_function_bind_config,
	.unbind_config	= rndis_function_unbind_config,
	.attributes	= rndis_function_attributes,
};
#else
static struct ecm_function_config {
	u8      ethaddr[ETH_ALEN];
	u32     vendorID;
	char	manufacturer[256];
	bool	wceis;
};

static int ecm_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
    f->config = kzalloc(sizeof(struct ecm_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	return 0;
}

static void ecm_function_cleanup(struct android_usb_function *f)
{
	gether_cleanup();
	kfree(f->config);
	f->config = NULL;
}

static int ecm_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	int ret;
	struct ecm_function_config *ecm = f->config;

	if (!ecm) {
		pr_err("%s: ecm_function_config\n", __func__);
		return -1;
	}

	pr_info("%s MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		ecm->ethaddr[0], ecm->ethaddr[1], ecm->ethaddr[2],
		ecm->ethaddr[3], ecm->ethaddr[4], ecm->ethaddr[5]);

#ifdef CONFIG_USB_G_LGE_ANDROID
	ret = gether_setup_name(c->cdev->gadget, ecm->ethaddr, "usb");
#else
	ret = gether_setup_name(c->cdev->gadget, ecm->ethaddr, "ecm");
#endif
	if (ret)
	{
		pr_err("%s: gether_setup failed\n", __func__);
		return ret;
	}

	return ecm_bind_config(c, ecm->ethaddr);
}

static void ecm_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	gether_cleanup();
}

static ssize_t ecm_manufacturer_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct ecm_function_config *config = f->config;
	return sprintf(buf, "%s\n", config->manufacturer);
}

static ssize_t ecm_manufacturer_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct ecm_function_config *config = f->config;

	if (size >= sizeof(config->manufacturer))
		return -EINVAL;
	if (sscanf(buf, "%s", config->manufacturer) == 1)
		return size;
	return -1;
}

static DEVICE_ATTR(manufacturer, S_IRUGO | S_IWUSR, ecm_manufacturer_show,
						    ecm_manufacturer_store);

static ssize_t ecm_ethaddr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct ecm_function_config *ecm = f->config;
	return sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		ecm->ethaddr[0], ecm->ethaddr[1], ecm->ethaddr[2],
		ecm->ethaddr[3], ecm->ethaddr[4], ecm->ethaddr[5]);
}

static ssize_t ecm_ethaddr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct ecm_function_config *ecm = f->config;

	if (sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		    (int *)&ecm->ethaddr[0], (int *)&ecm->ethaddr[1],
		    (int *)&ecm->ethaddr[2], (int *)&ecm->ethaddr[3],
		    (int *)&ecm->ethaddr[4], (int *)&ecm->ethaddr[5]) == 6)
		return size;
	return -EINVAL;
}

static DEVICE_ATTR(ethaddr, S_IRUGO | S_IWUSR, ecm_ethaddr_show,
					       ecm_ethaddr_store);

static ssize_t ecm_vendorID_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct ecm_function_config *config = f->config;
	return sprintf(buf, "%04x\n", config->vendorID);
}

static ssize_t ecm_vendorID_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct ecm_function_config *config = f->config;
	int value;

	if (sscanf(buf, "%04x", &value) == 1) {
		config->vendorID = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(vendorID, S_IRUGO | S_IWUSR, ecm_vendorID_show,
						ecm_vendorID_store);

static struct device_attribute *ecm_function_attributes[] = {
	&dev_attr_manufacturer,
	&dev_attr_ethaddr,
	&dev_attr_vendorID,
	NULL
};

static struct android_usb_function ecm_function = {
	.name		= "ecm",
	.init		= ecm_function_init,
	.cleanup	= ecm_function_cleanup,
	.bind_config	= ecm_function_bind_config,
	.unbind_config	= ecm_function_unbind_config,
	.attributes	= ecm_function_attributes,
};

#endif /* ! CONFIG_USB_ANDROID_CDC_ECM */

#ifdef CONFIG_USB_G_LGE_ANDROID_NCM
/*-------------------------------------------------------------------------*/
/* NCM */
/*-------------------------------------------------------------------------*/
struct ncm_function_config {
	u8      ethaddr[ETH_ALEN];
};
static int
ncm_function_init(struct android_usb_function *f, struct usb_composite_dev *c)
{
	f->config = kzalloc(sizeof(struct ncm_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	return ncm_init();
}

static void ncm_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
	ncm_cleanup();
}

static int
ncm_function_bind_config(struct android_usb_function *f,
				struct usb_configuration *c)
{
	struct ncm_function_config *ncm = f->config;
	int ret;

	if (!ncm) {
		pr_err("%s: ncm config is null\n", __func__);
		return -EINVAL;
	}

	pr_info("%s MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		ncm->ethaddr[0], ncm->ethaddr[1], ncm->ethaddr[2],
		ncm->ethaddr[3], ncm->ethaddr[4], ncm->ethaddr[5]);

/*                           
                                 

                                                                 
 */
	ret = gether_setup_name(c->cdev->gadget, ncm->ethaddr, "usb");
	if (ret) {
		pr_err("%s: gether setup failed err:%d\n", __func__, ret);
		return ret;
	}

	ret = ncm_bind_config(c, ncm->ethaddr);
	if (ret) {
		pr_err("%s: ncm bind config failed err:%d", __func__, ret);
		gether_cleanup();
		return ret;
	}

	return ret;
}

static void ncm_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	gether_cleanup();
}

static ssize_t ncm_ethaddr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct ncm_function_config *ncm = f->config;
	return snprintf(buf, PAGE_SIZE, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		ncm->ethaddr[0], ncm->ethaddr[1], ncm->ethaddr[2],
		ncm->ethaddr[3], ncm->ethaddr[4], ncm->ethaddr[5]);
}

static ssize_t ncm_ethaddr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct ncm_function_config *ncm = f->config;

	if (sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		    (int *)&ncm->ethaddr[0], (int *)&ncm->ethaddr[1],
		    (int *)&ncm->ethaddr[2], (int *)&ncm->ethaddr[3],
		    (int *)&ncm->ethaddr[4], (int *)&ncm->ethaddr[5]) == 6)
		return size;
	return -EINVAL;
}

static DEVICE_ATTR(ncm_ethaddr, S_IRUGO | S_IWUSR, ncm_ethaddr_show,
					       ncm_ethaddr_store);
static struct device_attribute *ncm_function_attributes[] = {
	&dev_attr_ncm_ethaddr,
	NULL
};

static struct android_usb_function ncm_function = {
	.name		= "ncm",
	.init		= ncm_function_init,
	.cleanup	= ncm_function_cleanup,
	.bind_config	= ncm_function_bind_config,
	.unbind_config	= ncm_function_unbind_config,
	.attributes	= ncm_function_attributes,
};
#endif

#ifdef CONFIG_USB_G_LGE_ANDROID
static const char lge_vendor_name[] = "LGE";
static const char lge_product_name[] = CONFIG_USB_G_LGE_ANDROID_STORAGE_NAME;
#endif

struct mass_storage_function_config {
	struct fsg_config fsg;
	struct fsg_common *common;
};

static int mass_storage_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	struct mass_storage_function_config *config;
	struct fsg_common *common;
	int err;

	config = kzalloc(sizeof(struct mass_storage_function_config),
								GFP_KERNEL);
	if (!config)
		return -ENOMEM;

#ifdef CONFIG_USB_G_LGE_ANDROID
	config->fsg.vendor_name = lge_vendor_name;
	config->fsg.product_name = lge_product_name;
#endif
	config->fsg.nluns = 1;
	config->fsg.luns[0].removable = 1;

	common = fsg_common_init(NULL, cdev, &config->fsg);
	if (IS_ERR(common)) {
		kfree(config);
		return PTR_ERR(common);
	}

	err = sysfs_create_link(&f->dev->kobj,
				&common->luns[0].dev.kobj,
				"lun");
	if (err) {
		kfree(config);
		return err;
	}

	config->common = common;
	f->config = config;
	return 0;
}

static void mass_storage_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int mass_storage_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct mass_storage_function_config *config = f->config;
	return fsg_bind_config(c->cdev, c, config->common);
}

static ssize_t mass_storage_inquiry_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
	return sprintf(buf, "%s\n", config->common->inquiry_string);
}

static ssize_t mass_storage_inquiry_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
	if (size >= sizeof(config->common->inquiry_string))
		return -EINVAL;
	if (sscanf(buf, "%s", config->common->inquiry_string) != 1)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR(inquiry_string, S_IRUGO | S_IWUSR,
					mass_storage_inquiry_show,
					mass_storage_inquiry_store);

static struct device_attribute *mass_storage_function_attributes[] = {
	&dev_attr_inquiry_string,
	NULL
};

static struct android_usb_function mass_storage_function = {
	.name		= "mass_storage",
	.init		= mass_storage_function_init,
	.cleanup	= mass_storage_function_cleanup,
	.bind_config	= mass_storage_function_bind_config,
	.attributes	= mass_storage_function_attributes,
};

#ifdef CONFIG_USB_G_LGE_ANDROID_AUTORUN
struct cdrom_storage_function_config {
	struct fsg_config fsg;
	struct fsg_common *common;
};

static const char cdrom_storage_kthread_name[] = "kcdrom-storaged";
static const char cdrom_lun_format[] = "clun%d";

static int cdrom_storage_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	struct cdrom_storage_function_config *config;
	struct fsg_common *common;
	int err;

	config = kzalloc(sizeof(struct cdrom_storage_function_config),
			GFP_KERNEL);
	if (!config)
		return -ENOMEM;

	config->fsg.nluns = 1;
	config->fsg.luns[0].removable = 1;
	config->fsg.luns[0].cdrom = 1; /* cdrom(read only) flag */
	config->fsg.thread_name = cdrom_storage_kthread_name;
	config->fsg.lun_name_format = cdrom_lun_format;
	config->fsg.vendor_name = lge_vendor_name;
	config->fsg.product_name = lge_product_name;

	common = fsg_common_init(NULL, cdev, &config->fsg);
	if (IS_ERR(common)) {
		kfree(config);
		return PTR_ERR(common);
	}

	err = sysfs_create_link(&f->dev->kobj,
			&common->luns[0].dev.kobj,
			"lun");
	if (err) {
		fsg_common_release(&common->ref);
		kfree(config);
		return err;
	}

	config->common = common;
	f->config = config;
	return 0;
}

static int cdrom_storage_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	struct cdrom_storage_function_config *config = f->config;
	return csg_bind_config(c->cdev, c, config->common);
}

static void cdrom_storage_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static ssize_t cdrom_storage_inquiry_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct cdrom_storage_function_config *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%s\n", config->common->inquiry_string);
}

static ssize_t cdrom_storage_inquiry_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct cdrom_storage_function_config *config = f->config;
	if (size >= sizeof(config->common->inquiry_string))
		return -EINVAL;
	if (strlcpy(config->common->inquiry_string, buf,
				sizeof(config->common->inquiry_string)) == 0)
		return -EINVAL;

	return size;
}

static DEVICE_ATTR(cdrom_inquiry_string, S_IRUGO | S_IWUSR,
		cdrom_storage_inquiry_show,
		cdrom_storage_inquiry_store);

static struct device_attribute *cdrom_storage_function_attributes[] = {
	&dev_attr_cdrom_inquiry_string,
	NULL
};

static struct android_usb_function cdrom_storage_function = {
	.name		= "cdrom_storage",
	.init		= cdrom_storage_function_init,
	.cleanup	= cdrom_storage_function_cleanup,
	.bind_config	= cdrom_storage_function_bind_config,
	.attributes	= cdrom_storage_function_attributes,
};

/* charge only mode */
static int charge_only_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	return charge_only_setup();
}

static void charge_only_function_cleanup(struct android_usb_function *f)
{
	charge_only_cleanup();
}

static int charge_only_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	return charge_only_bind_config(c);
}

static struct android_usb_function charge_only_function = {
	.name		= "charge_only",
	.init		= charge_only_function_init,
	.cleanup	= charge_only_function_cleanup,
	.bind_config	= charge_only_function_bind_config,
};
#endif /*                                  */

static int accessory_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	return acc_setup();
}

static void accessory_function_cleanup(struct android_usb_function *f)
{
	acc_cleanup();
}

static int accessory_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	return acc_bind_config(c);
}

static int accessory_function_ctrlrequest(struct android_usb_function *f,
						struct usb_composite_dev *cdev,
						const struct usb_ctrlrequest *c)
{
	return acc_ctrlrequest(cdev, c);
}

static struct android_usb_function accessory_function = {
	.name		= "accessory",
	.init		= accessory_function_init,
	.cleanup	= accessory_function_cleanup,
	.bind_config	= accessory_function_bind_config,
	.ctrlrequest	= accessory_function_ctrlrequest,
};

static int audio_source_function_init(struct android_usb_function *f,
			struct usb_composite_dev *cdev)
{
	struct audio_source_config *config;

	config = kzalloc(sizeof(struct audio_source_config), GFP_KERNEL);
	if (!config)
		return -ENOMEM;
	config->card = -1;
	config->device = -1;
	f->config = config;
	return 0;
}

static void audio_source_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
}

static int audio_source_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct audio_source_config *config = f->config;

	return audio_source_bind_config(c, config);
}

static void audio_source_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct audio_source_config *config = f->config;

	config->card = -1;
	config->device = -1;
}

static ssize_t audio_source_pcm_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct audio_source_config *config = f->config;

	/* print PCM card and device numbers */
	return sprintf(buf, "%d %d\n", config->card, config->device);
}

static DEVICE_ATTR(pcm, S_IRUGO | S_IWUSR, audio_source_pcm_show, NULL);

static struct device_attribute *audio_source_function_attributes[] = {
	&dev_attr_pcm,
	NULL
};

static struct android_usb_function audio_source_function = {
	.name		= "audio_source",
	.init		= audio_source_function_init,
	.cleanup	= audio_source_function_cleanup,
	.bind_config	= audio_source_function_bind_config,
	.unbind_config	= audio_source_function_unbind_config,
	.attributes	= audio_source_function_attributes,
};

static int
nvusb_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	return 0;
}

static void nvusb_function_cleanup(struct android_usb_function *f)
{
}

static int
nvusb_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	return nvusb_bind_config(c);
}

static struct android_usb_function nvusb_function = {
	.name		= "nvusb",
	.init		= nvusb_function_init,
	.cleanup	= nvusb_function_cleanup,
	.bind_config	= nvusb_function_bind_config,
};

static struct android_usb_function *supported_functions[] = {
	&ffs_function,
	&adb_function,
#ifdef CONFIG_LAF_G_DRIVER
	&laf_function,
#endif
	&acm_function,
#ifdef CONFIG_USB_G_LGE_ANDROID
	&serial_function,
#endif
	// &acm_nv_function,
	&mtp_function,
	&ptp_function,
#ifdef CONFIG_USB_ANDROID_CDC_ECM
	&ecm_function,
#else
	&rndis_function,
#endif
#ifdef CONFIG_USB_G_LGE_ANDROID_NCM
	&ncm_function,
#endif
	&mass_storage_function,
#ifdef CONFIG_USB_G_LGE_ANDROID_AUTORUN
	&cdrom_storage_function,
	&charge_only_function,
#endif
	&accessory_function,
	&audio_source_function,
	&nvusb_function,
	NULL
};


static int android_init_functions(struct android_usb_function **functions,
				  struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_function *f;
	struct device_attribute **attrs;
	struct device_attribute *attr;
	int err = 0;
	int index = 0;

	for (; (f = *functions++); index++) {
		f->dev_name = kasprintf(GFP_KERNEL, "f_%s", f->name);
		f->dev = device_create(android_class, dev->dev,
				MKDEV(0, index), f, f->dev_name);
		if (IS_ERR(f->dev)) {
			pr_err("%s: Failed to create dev %s", __func__,
							f->dev_name);
			err = PTR_ERR(f->dev);
			goto err_create;
		}

		if (f->init) {
			err = f->init(f, cdev);
			if (err) {
				pr_err("%s: Failed to init %s", __func__,
								f->name);
				goto err_out;
			}
		}

		attrs = f->attributes;
		if (attrs) {
			while ((attr = *attrs++) && !err)
				err = device_create_file(f->dev, attr);
		}
		if (err) {
			pr_err("%s: Failed to create function %s attributes",
					__func__, f->name);
			goto err_out;
		}
	}
	return 0;

err_out:
	device_destroy(android_class, f->dev->devt);
err_create:
	kfree(f->dev_name);
	return err;
}

static void android_cleanup_functions(struct android_usb_function **functions)
{
	struct android_usb_function *f;

	while (*functions) {
		f = *functions++;

		if (f->dev) {
			device_destroy(android_class, f->dev->devt);
			kfree(f->dev_name);
		}

		if (f->cleanup)
			f->cleanup(f);
	}
}

static int
android_bind_enabled_functions(struct android_dev *dev,
			       struct usb_configuration *c)
{
	struct android_usb_function *f;
	int ret;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		ret = f->bind_config(f, c);
		if (ret) {
			pr_err("%s: %s failed", __func__, f->name);
			return ret;
		}
	}
	return 0;
}

static void
android_unbind_enabled_functions(struct android_dev *dev,
			       struct usb_configuration *c)
{
	struct android_usb_function *f;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		if (f->unbind_config)
			f->unbind_config(f, c);
	}
}

static int android_enable_function(struct android_dev *dev, char *name)
{
	struct android_usb_function **functions = dev->functions;
	struct android_usb_function *f;
	while ((f = *functions++)) {
		if (!strcmp(name, f->name)) {
			list_add_tail(&f->enabled_list,
						&dev->enabled_functions);
			return 0;
		}
	}
	return -EINVAL;
}

/*-------------------------------------------------------------------------*/
/* /sys/class/android_usb/android%d/ interface */

static ssize_t
functions_show(struct device *pdev, struct device_attribute *attr, char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct android_usb_function *f;
	char *buff = buf;

	mutex_lock(&dev->mutex);

	list_for_each_entry(f, &dev->enabled_functions, enabled_list)
		buff += sprintf(buff, "%s,", f->name);

	mutex_unlock(&dev->mutex);

	if (buff != buf)
		*(buff-1) = '\n';
	return buff - buf;
}

static ssize_t
functions_store(struct device *pdev, struct device_attribute *attr,
			       const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	char *name;
	char buf[256], *b;
	char aliases[256], *a;
	int err;
	int is_ffs;
	int ffs_enabled = 0;

	mutex_lock(&dev->mutex);

	if (dev->enabled) {
		mutex_unlock(&dev->mutex);
		return -EBUSY;
	}

#ifdef CONFIG_USB_G_LGE_ANDROID
	dev_info(dev->dev, "request function list: %s\n", buff);
	if (dev->check_pif) {
		dev_info(dev->dev, "pif cable is plugged, not permitted\n");
		mutex_unlock(&dev->mutex);
		return -EPERM;
	}
#endif

	INIT_LIST_HEAD(&dev->enabled_functions);

	strlcpy(buf, buff, sizeof(buf));
	b = strim(buf);
#ifdef CONFIG_USB_G_LGE_ANDROID_AUTORUN
	dev->check_charge_only = false;
	if (!strcmp(b, "charge_only"))
		dev->check_charge_only = true;
#endif

	while (b) {
		name = strsep(&b, ",");
		if (!name)
			continue;

		is_ffs = 0;
		strlcpy(aliases, dev->ffs_aliases, sizeof(aliases));
		a = aliases;

		while (a) {
			char *alias = strsep(&a, ",");
			if (alias && !strcmp(name, alias)) {
				is_ffs = 1;
				break;
			}
		}

		if (is_ffs) {
			if (ffs_enabled)
				continue;
			err = android_enable_function(dev, "ffs");
			if (err)
				pr_err("android_usb: Cannot enable ffs (%d)",
									err);
			else
				ffs_enabled = 1;
			continue;
		}

		err = android_enable_function(dev, name);
		if (err)
			pr_err("android_usb: Cannot enable '%s' (%d)",
							   name, err);
	}

	mutex_unlock(&dev->mutex);

	return size;
}

static ssize_t enable_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	return sprintf(buf, "%d\n", dev->enabled);
}

static ssize_t enable_store(struct device *pdev, struct device_attribute *attr,
			    const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	struct android_usb_function *f;
	int enabled = 0;


	if (!cdev)
		return -ENODEV;

#ifdef CONFIG_USB_G_LGE_ANDROID
	dev_info(dev->dev, "gadget enable(%s)\n", buff);

	if (dev->check_pif) {
		dev_info(dev->dev, "pif cable is plugged, not permitted\n");
		return -EPERM;
	}
#endif

	mutex_lock(&dev->mutex);

	sscanf(buff, "%d", &enabled);
	if (enabled && !dev->enabled) {
		/*
		 * Update values in composite driver's copy of
		 * device descriptor.
		 */
		cdev->desc.idVendor = device_desc.idVendor;
		cdev->desc.idProduct = device_desc.idProduct;
		cdev->desc.bcdDevice = device_desc.bcdDevice;
		cdev->desc.bDeviceClass = device_desc.bDeviceClass;
		cdev->desc.bDeviceSubClass = device_desc.bDeviceSubClass;
		cdev->desc.bDeviceProtocol = device_desc.bDeviceProtocol;
		pr_info("LG-android_usb: idProduct(%04X)\n",
				cdev->desc.idProduct);
#ifdef CONFIG_USB_G_LGE_ANDROID_AUTORUN
		if (lge_get_laf_mode()) {
			cdev->desc.iSerialNumber = 0;
			cdev->desc.iProduct = strings_dev[STRING_PRODUCT_IDX].id;
		} else if (dev->check_charge_only) {
			cdev->desc.iSerialNumber = 0;
			cdev->desc.iProduct = strings_dev[CHARGE_ONLY_STRING_IDX].id;
		} else {
			cdev->desc.iSerialNumber = strings_dev[STRING_SERIAL_IDX].id;
			cdev->desc.iProduct = strings_dev[STRING_PRODUCT_IDX].id;
		}
#endif
		list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
			if (f->enable)
				f->enable(f);
		}
		android_enable(dev);
		dev->enabled = true;
	} else if (!enabled && dev->enabled) {
		android_disable(dev);
		list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
			if (f->disable)
				f->disable(f);
		}
		dev->enabled = false;
	} else {
		pr_err("android_usb: already %s\n",
				dev->enabled ? "enabled" : "disabled");
	}

	mutex_unlock(&dev->mutex);
	return size;
}

static ssize_t state_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	char *state = "DISCONNECTED";
	unsigned long flags;

	if (!cdev)
		goto out;

	spin_lock_irqsave(&cdev->lock, flags);
	if (cdev->config)
		state = "CONFIGURED";
	else if (dev->connected)
		state = "CONNECTED";
	spin_unlock_irqrestore(&cdev->lock, flags);
out:
	return sprintf(buf, "%s\n", state);
}

#ifdef CONFIG_USB_G_LGE_ANDROID
static ssize_t fmode_show(struct device *pdev, struct device_attribute *attr,
		char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);

	return snprintf(buf, PAGE_SIZE, "%d\n", dev->check_pif);
}

static ssize_t fmode_store(struct device *pdev, struct device_attribute *attr,
		const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	int enabled = 0;

	mutex_lock(&dev->mutex);
	sscanf(buff, "%d", &enabled);
	if (enabled)
		dev->check_pif = true;
	else
		dev->check_pif = false;
	mutex_unlock(&dev->mutex);

	return size;
}

/* if pif cable is plugged, not allow request from user space */
#define DESCRIPTOR_ATTR(field, format_string)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return snprintf(buf, PAGE_SIZE,					\
			format_string, device_desc.field);		\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)		       		\
{									\
	int value;					       		\
	struct android_dev *adev = dev_get_drvdata(dev); \
	if (adev->check_pif) \
		return -EPERM; \
	if (sscanf(buf, format_string, &value) == 1) {			\
		device_desc.field = value;				\
		return size;						\
	}								\
	return -1;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);

#define DESCRIPTOR_STRING_ATTR(field, buffer)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return snprintf(buf, PAGE_SIZE, "%s", buffer);			\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)		       		\
{									\
	struct android_dev *adev = dev_get_drvdata(dev); \
	if (adev->check_pif) \
		return -EPERM; \
	if (size >= sizeof(buffer)) \
		return -EINVAL;			\
	if (sscanf(buf, "%255s", buffer) == 1) {			\
		return size;						\
	}								\
	return -1;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);
#else /* google original */
#define DESCRIPTOR_ATTR(field, format_string)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return sprintf(buf, format_string, device_desc.field);		\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)				\
{									\
	int value;							\
	if (sscanf(buf, format_string, &value) == 1) {			\
		device_desc.field = value;				\
		return size;						\
	}								\
	return -1;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);

#define DESCRIPTOR_STRING_ATTR(field, buffer)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return sprintf(buf, "%s", buffer);				\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)				\
{									\
	if (size >= sizeof(buffer))					\
		return -EINVAL;						\
	return strlcpy(buffer, buf, sizeof(buffer));			\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);
#endif /*                          */


DESCRIPTOR_ATTR(idVendor, "%04x\n")
DESCRIPTOR_ATTR(idProduct, "%04x\n")
DESCRIPTOR_ATTR(bcdDevice, "%04x\n")
DESCRIPTOR_ATTR(bDeviceClass, "%d\n")
DESCRIPTOR_ATTR(bDeviceSubClass, "%d\n")
DESCRIPTOR_ATTR(bDeviceProtocol, "%d\n")
DESCRIPTOR_STRING_ATTR(iManufacturer, manufacturer_string)
DESCRIPTOR_STRING_ATTR(iProduct, product_string)
DESCRIPTOR_STRING_ATTR(iSerial, serial_string)
#ifdef CONFIG_USB_G_LGE_ANDROID
DESCRIPTOR_ATTR(iSerialNumber, "%d\n")
#endif

static DEVICE_ATTR(functions, S_IRUGO | S_IWUSR, functions_show,
						 functions_store);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, enable_show, enable_store);
static DEVICE_ATTR(state, S_IRUGO, state_show, NULL);
#ifdef CONFIG_USB_G_LGE_ANDROID
static DEVICE_ATTR(fmode, S_IRUGO | S_IWUSR, fmode_show, fmode_store);
#endif

static struct device_attribute *android_usb_attributes[] = {
	&dev_attr_idVendor,
	&dev_attr_idProduct,
	&dev_attr_bcdDevice,
	&dev_attr_bDeviceClass,
	&dev_attr_bDeviceSubClass,
	&dev_attr_bDeviceProtocol,
	&dev_attr_iManufacturer,
	&dev_attr_iProduct,
	&dev_attr_iSerial,
#ifdef CONFIG_USB_G_LGE_ANDROID
	&dev_attr_iSerialNumber,
#endif
	&dev_attr_functions,
	&dev_attr_enable,
	&dev_attr_state,
#ifdef CONFIG_USB_G_LGE_ANDROID
	&dev_attr_fmode,
#endif
	NULL
};

/*-------------------------------------------------------------------------*/
/* Composite driver */

#ifdef CONFIG_USB_G_LGE_ANDROID
static void android_lge_factory_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	char lge_factory_composition[256];
	char *name, *b;
	int err;

	/* update values in composite driver's copy of device descriptor */
	device_desc.idVendor = LGE_PIF_VID;
	device_desc.idProduct = LGE_PIF_PID;
	device_desc.iSerialNumber = LGE_PIF_SN;

	/*XXX: should we create sysfs with below 3 field? */
	device_desc.bDeviceClass = 2;
	device_desc.bDeviceSubClass = 0;
	device_desc.bDeviceProtocol = 0;

	cdev->desc.idVendor = device_desc.idVendor;
	cdev->desc.idProduct = device_desc.idProduct;
	cdev->desc.bcdDevice = device_desc.bcdDevice;
	cdev->desc.bDeviceClass = device_desc.bDeviceClass;
	cdev->desc.bDeviceSubClass = device_desc.bDeviceSubClass;
	cdev->desc.bDeviceProtocol = device_desc.bDeviceProtocol;

	INIT_LIST_HEAD(&dev->enabled_functions);

	strlcpy(lge_factory_composition, "acm,serial", sizeof(lge_factory_composition) - 1);

	b = strim(lge_factory_composition);
	while (b) {
		name = strsep(&b, ",");
		if (name) {
			err = android_enable_function(dev, name);
			if (err)
				 pr_err("android_usb: Cannot enable '%s'", name);
		}
	}

	if (usb_add_config(cdev, &android_config_driver, android_bind_config))
		return;

	usb_gadget_connect(cdev->gadget);
	dev->enabled = true;
}
#endif

static int android_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;
	int ret = 0;

	ret = android_bind_enabled_functions(dev, c);
	if (ret)
		return ret;

	return 0;
}

static void android_unbind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;

	android_unbind_enabled_functions(dev, c);
}

static int android_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct usb_gadget	*gadget = cdev->gadget;
	int			gcnum, id, ret;

	/*
	 * Start disconnected. Userspace will connect the gadget once
	 * it is done configuring the functions.
	 */
	usb_gadget_disconnect(gadget);

	ret = android_init_functions(dev->functions, cdev);
	if (ret)
		return ret;

	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

#ifdef CONFIG_USB_LGE_ANDROID
	/*                                */
	ret = lgeusb_get_manufacturer_name(lge_manufacturer);
	if (!ret)
		strlcpy(manufacturer_string, lge_manufacturer,
				sizeof(manufacturer_string) - 1);
	else
		strlcpy(manufacturer_string, "LG Electronics Inc.",
				sizeof(manufacturer_string) - 1);

	ret = lgeusb_get_product_name(lge_product);
	if (!ret)
		strlcpy(product_string, lge_product, sizeof(product_string) - 1);
	else
		strlcpy(product_string, "LGE Android Phone",
				sizeof(product_string) - 1);
#else
	/* Default strings - should be updated by userspace */
	strncpy(manufacturer_string, "Android", sizeof(manufacturer_string)-1);
	strncpy(product_string, "Android", sizeof(product_string) - 1);
#endif
	strncpy(serial_string, "0123456789ABCDEF", sizeof(serial_string) - 1);

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
	device_desc.iSerialNumber = id;
#ifdef CONFIG_USB_G_LGE_ANDROID_AUTORUN
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[CHARGE_ONLY_STRING_IDX].id = id;
	sprintf(charge_only_string, "USB Charge Only Interface");
#endif

	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	else {
		pr_warning("%s: controller '%s' not recognized\n",
			longname, gadget->name);
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	}

	dev->cdev = cdev;

#ifdef CONFIG_USB_G_LGE_ANDROID
#ifdef CONFIG_LAF_G_DRIVER
	if (!lge_get_laf_mode() && lgeusb_is_factory_mode()) {
#else
	if (lgeusb_is_factory_mode()) {
#endif
		dev_info(dev->dev, "pif cable is plugged, bind factory composition\n");
		dev->check_pif = true;
		android_lge_factory_bind(cdev);
	}
#endif
	return 0;
}

static int android_usb_unbind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;

	manufacturer_string[0] = '\0';
	product_string[0] = '\0';
	serial_string[0] = '0';
	cancel_work_sync(&dev->work);
	android_cleanup_functions(dev->functions);
	return 0;
}

static struct usb_composite_driver android_usb_driver = {
	.name		= "android_usb",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.unbind		= android_usb_unbind,
	.max_speed	= USB_SPEED_HIGH,
};

static int
android_setup(struct usb_gadget *gadget, const struct usb_ctrlrequest *c)
{
	struct android_dev		*dev = _android_dev;
	struct usb_composite_dev	*cdev = get_gadget_data(gadget);
	struct usb_request		*req = cdev->req;
	struct android_usb_function	*f;
	int value = -EOPNOTSUPP;
	unsigned long flags;

	req->zero = 0;
	req->complete = composite_setup_complete;
	req->length = 0;
	gadget->ep0->driver_data = cdev;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		if (f->ctrlrequest) {
			value = f->ctrlrequest(f, cdev, c);
			if (value >= 0)
				break;
		}
	}

	/* Special case the accessory function.
	 * It needs to handle control requests before it is enabled.
	 */
	if (value < 0)
		value = acc_ctrlrequest(cdev, c);

	if (value < 0)
		value = composite_setup(gadget, c);

	spin_lock_irqsave(&cdev->lock, flags);
	if (!dev->connected) {
		dev->connected = 1;
		schedule_work(&dev->work);
	} else if (c->bRequest == USB_REQ_SET_CONFIGURATION &&
						cdev->config) {
		schedule_work(&dev->work);
	}
	spin_unlock_irqrestore(&cdev->lock, flags);

	return value;
}

static void android_disconnect(struct usb_gadget *gadget)
{
	struct android_dev *dev = _android_dev;
	struct usb_composite_dev *cdev = get_gadget_data(gadget);
	unsigned long flags;

	composite_disconnect(gadget);
	/* accessory HID support can be active while the
	   accessory function is not actually enabled,
	   so we need to inform it when we are disconnected.
	 */
	acc_disconnect();

	spin_lock_irqsave(&cdev->lock, flags);
	dev->connected = 0;
	schedule_work(&dev->work);
	spin_unlock_irqrestore(&cdev->lock, flags);
}

static int android_create_device(struct android_dev *dev)
{
	struct device_attribute **attrs = android_usb_attributes;
	struct device_attribute *attr;
	int err;

	dev->dev = device_create(android_class, NULL,
					MKDEV(0, 0), NULL, "android0");
	if (IS_ERR(dev->dev))
		return PTR_ERR(dev->dev);

	dev_set_drvdata(dev->dev, dev);

	while ((attr = *attrs++)) {
		err = device_create_file(dev->dev, attr);
		if (err) {
			device_destroy(android_class, dev->dev->devt);
			return err;
		}
	}
	return 0;
}

#ifdef CONFIG_USB_G_LGE_ANDROID
static int __devinit android_probe(struct platform_device *pdev)
{
	struct android_usb_platform_data *pdata = pdev->dev.platform_data;
	struct android_dev *dev = _android_dev;

	dev->pdata = pdata;
	dev->check_pif = false;
	return 0;
}

static struct platform_driver android_platform_driver = {
	.driver = { .name = "android_usb"},
};
#endif

static int __init init(void)
{
	struct android_dev *dev;
	int err;

	android_class = class_create(THIS_MODULE, "android_usb");
	if (IS_ERR(android_class))
		return PTR_ERR(android_class);

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

#ifdef CONFIG_USB_G_LGE_ANDROID
	dev->serial_flags = 0x00;
#endif
	dev->disable_depth = 1;
	dev->functions = supported_functions;
	INIT_LIST_HEAD(&dev->enabled_functions);
	INIT_WORK(&dev->work, android_work);
	mutex_init(&dev->mutex);

	err = android_create_device(dev);
	if (err) {
		class_destroy(android_class);
		kfree(dev);
		return err;
	}

	_android_dev = dev;

	/* Override composite driver functions */
	composite_driver.setup = android_setup;
	composite_driver.disconnect = android_disconnect;
#ifdef CONFIG_USB_G_LGE_ANDROID
	platform_driver_probe(&android_platform_driver, android_probe);
#endif

	return usb_composite_probe(&android_usb_driver, android_bind);
}
module_init(init);

static void __exit cleanup(void)
{
	usb_composite_unregister(&android_usb_driver);
	class_destroy(android_class);
	kfree(_android_dev);
	_android_dev = NULL;
}
module_exit(cleanup);
