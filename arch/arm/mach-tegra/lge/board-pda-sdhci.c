/*
 * arch/arm/mach-tegra/lge/board-pda-sdhci.c
 *
 * Copyright (c) 2012-2014, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/wlan_plat.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mmc/host.h>
#include <linux/wl12xx.h>
#include <linux/mfd/max77660/max77660-core.h>
#include <linux/mfd/palmas.h>
#ifdef CONFIG_MACH_PDA
#include <linux/random.h>
#endif
#include <linux/skbuff.h>

#include <asm/mach-types.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/sdhci.h>
#include <mach/gpio-tegra.h>
#include <mach/board_lge.h> /*HW revision get*/

#include "../gpio-names.h"
#include "../board.h"
#include "board-pda.h"
#include "board-atlantis-pda.h"
#include "../tegra-board-id.h"
#include "../dvfs.h"
#include "../fuse.h"

#define FUSE_CORE_SPEEDO_0	0x134

#define PDA_WLAN_PWR  TEGRA_GPIO_PL7
#define PDA_WLAN_WOW  TEGRA_GPIO_PO0/*TEGRA_GPIO_PO2 --> TEGRA_GPIO_PO0  at Rev.C*/

//#define CONFIG_BROADCOM_WIFI_RESERVED_MEM/*SUPPORT WIFI STATIC BUFF*/
#define CONFIG_BROADCOM_WIFI_COUNTRYCODE/*SUPPORT WIFI COUNTRYCODE for BCM4334*/
#define CONFIG_BROADCOM_WIFI_SWOOB

#define MAXIM77660_SD_CD	(MAX77660_GPIO_BASE + MAX77660_GPIO9)
#define PALMAS_SD_CD	(PALMAS_TEGRA_GPIO_BASE + PALMAS_GPIO10)
#if defined(CONFIG_BCMDHD_EDP_SUPPORT)
/* Wifi power levels */
#define ON  1080 /* 1080 mW */
#define OFF 0
static unsigned int wifi_states[] = {ON, OFF};
#endif

extern hw_rev_type lge_get_board_revno(void);/*HW revision get*/

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;
static int pda_wifi_status_register(void (*callback)(int , void *), void *);

static int pda_wifi_reset(int on);
static int pda_wifi_power(int on);
static int pda_wifi_set_carddetect(int val);
static int pda_wifi_get_mac_addr(unsigned char *buf);

/* For broadcom */
#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM

#define WLAN_STATIC_SCAN_BUF0		5
#define WLAN_STATIC_SCAN_BUF1		6
#define PREALLOC_WLAN_SEC_NUM		4
#define PREALLOC_WLAN_BUF_NUM		160
#define PREALLOC_WLAN_SECTION_HEADER		24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_BUF_NUM * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_BUF_NUM * 1024)

#define DHD_SKB_HDRSIZE			336
#define DHD_SKB_1PAGE_BUFSIZE	((PAGE_SIZE*1)-DHD_SKB_HDRSIZE)
#define DHD_SKB_2PAGE_BUFSIZE	((PAGE_SIZE*2)-DHD_SKB_HDRSIZE)
#define DHD_SKB_4PAGE_BUFSIZE	((PAGE_SIZE*4)-DHD_SKB_HDRSIZE)

#define WLAN_SKB_BUF_NUM	17


static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

struct wlan_mem_prealloc {
	void *mem_ptr;
	unsigned long size;
};

static struct wlan_mem_prealloc wlan_mem_array[PREALLOC_WLAN_SEC_NUM] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER) }
};

void *wlan_static_scan_buf0;
void *wlan_static_scan_buf1;
static void *brcm_wlan_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_SEC_NUM)
		return wlan_static_skb;
	if (section == WLAN_STATIC_SCAN_BUF0)
		return wlan_static_scan_buf0;
	if (section == WLAN_STATIC_SCAN_BUF1)
		return wlan_static_scan_buf1;
	if ((section < 0) || (section > PREALLOC_WLAN_SEC_NUM))
		return NULL;

	if (wlan_mem_array[section].size < size)
		return NULL;

	return wlan_mem_array[section].mem_ptr;
}

static int brcm_init_wlan_mem(void)
{
	int i;
	int j;

	for (i = 0; i < 8; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_1PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	for (; i < 16; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_2PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_4PAGE_BUFSIZE);
	if (!wlan_static_skb[i])
		goto err_skb_alloc;

	for (i = 0; i < PREALLOC_WLAN_SEC_NUM; i++) {
		wlan_mem_array[i].mem_ptr =
				kmalloc(wlan_mem_array[i].size, GFP_KERNEL);

		if (!wlan_mem_array[i].mem_ptr)
			goto err_mem_alloc;
	}
	wlan_static_scan_buf0 = kmalloc(65536, GFP_KERNEL);
	if (!wlan_static_scan_buf0)
		goto err_mem_alloc;
	wlan_static_scan_buf1 = kmalloc(65536, GFP_KERNEL);
	if (!wlan_static_scan_buf1)
		goto err_mem_alloc;

	printk(KERN_INFO "%s: WIFI MEM Allocated\n", __func__);
	return 0;

 err_mem_alloc:
	pr_err("Failed to mem_alloc for WLAN\n");
	for (j = 0; j < i; j++)
		kfree(wlan_mem_array[j].mem_ptr);

	i = WLAN_SKB_BUF_NUM;

 err_skb_alloc:
	pr_err("Failed to skb_alloc for WLAN\n");
	for (j = 0; j < i; j++)
		dev_kfree_skb(wlan_static_skb[j]);

	return -ENOMEM;
}
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */

#ifdef CONFIG_BROADCOM_WIFI_COUNTRYCODE
#define COUNTRY_BUF_SZ	4
struct cntry_locales_custom {
	char iso_abbrev[COUNTRY_BUF_SZ];
	char custom_locale[COUNTRY_BUF_SZ];
	int custom_locale_rev;
};

/* Customized Locale table */
const struct cntry_locales_custom bcm_wifi_translate_custom_table[] = {
/* Table should be filled out based on custom platform regulatory requirement */
	{"",	"XY",	4},
	{"AD",	"GB",	0},
	{"AE",	"KR",	24},
	{"AF",	"AF",	0},
	{"AG",	"US",	100},
	{"AI",	"US",	100},
	{"AL",	"GB",	0},
	{"AM",	"IL",	10},
	{"AN",	"BR",	0},
	{"AO",	"IL",	10},
	{"AR",	"BR",	0},
	{"AS",	"US",	100},
	{"AT",	"GB",	0},
	{"AU",	"AU",	2},
	{"AW",	"KR",	24},
	{"AZ",	"BR",	0},
	{"BA",	"GB",	0},
	{"BB",	"RU",	1},
	{"BD",	"CN",	0},
	{"BE",	"GB",	0},
	{"BF",	"CN",	0},
	{"BG",	"GB",	0},
	{"BH",	"RU",	1},
	{"BI",	"IL",	10},
	{"BJ",	"IL",	10},
	{"BM",	"US",	100},
	{"BN",	"RU",	1},
	{"BO",	"IL",	10},
	{"BR",	"BR",	0},
	{"BS",	"RU",	1},
	{"BT",	"IL",	10},
	{"BW",	"GB",	0},
	{"BY",	"GB",	0},
	{"BZ",	"IL",	10},
	{"CA",	"US",	100},
	{"CD",	"IL",	10},
	{"CF",	"IL",	10},
	{"CG",	"IL",	10},
	{"CH",	"GB",	0},
	{"CI",	"IL",	10},
	{"CK",	"BR",	0},
	{"CL",	"RU",	1},
	{"CM",	"IL",	10},
	{"CN",	"CN",	0},
	{"CO",	"BR",	0},
	{"CR",	"BR",	0},
	{"CU",	"BR",	0},
	{"CV",	"GB",	0},
	{"CX",	"AU",	2},
	{"CY",	"GB",	0},
	{"CZ",	"GB",	0},
	{"DE",	"GB",	0},
	{"DJ",	"IL",	10},
	{"DK",	"GB",	0},
	{"DM",	"BR",	0},
	{"DO",	"BR",	0},
	{"DZ",	"KW",	1},
	{"EC",	"BR",	0},
	{"EE",	"GB",	0},
	{"EG",	"RU",	1},
	{"ER",	"IL",	10},
	{"ES",	"GB",	0},
	{"ET",	"GB",	0},
	{"FI",	"GB",	0},
	{"FJ",	"IL",	10},
	{"FM",	"US",	100},
	{"FO",	"GB",	0},
	{"FR",	"GB",	0},
	{"GA",	"IL",	10},
	{"GB",	"GB",	0},
	{"GD",	"BR",	0},
	{"GE",	"GB",	0},
	{"GF",	"GB",	0},
	{"GH",	"BR",	0},
	{"GI",	"GB",	0},
	{"GM",	"IL",	10},
	{"GN",	"IL",	10},
	{"GP",	"GB",	0},
	{"GQ",	"IL",	10},
	{"GR",	"GB",	0},
	{"GT",	"RU",	1},
	{"GU",	"US",	100},
	{"GW",	"IL",	10},
	{"GY",	"QA",	0},
	{"HK",	"BR",	0},
	{"HN",	"CN",	0},
	{"HR",	"GB",	0},
	{"HT",	"RU",	1},
	{"HU",	"GB",	0},
	{"ID",	"QA",	0},
	{"IE",	"GB",	0},
	{"IL",	"IL",	10},
	{"IM",	"GB",	0},
	{"IN",	"RU",	1},
	{"IQ",	"IL",	10},
	{"IR",	"IL",	10},
	{"IS",	"GB",	0},
	{"IT",	"GB",	0},
	{"JE",	"GB",	0},
	{"JM",	"GB",	0},
	{"JO",	"XY",	3},
	{"JP",	"JP",	5},
	{"KE",	"GB",	0},
	{"KG",	"IL",	10},
	{"KH",	"BR",	0},
	{"KI",	"AU",	2},
	{"KM",	"IL",	10},
	{"KP",	"IL",	10},
	{"KR",	"KR",	24},
	{"KW",	"KW",	1},
	{"KY",	"US",	100},
	{"KZ",	"BR",	0},
	{"LA",	"KR",	24},
	{"LB",	"BR",	0},
	{"LC",	"BR",	0},
	{"LI",	"GB",	0},
	{"LK",	"BR",	0},
	{"LR",	"BR",	0},
	{"LS",	"GB",	0},
	{"LT",	"GB",	0},
	{"LU",	"GB",	0},
	{"LV",	"GB",	0},
	{"LY",	"IL",	10},
	{"MA",	"KW",	1},
	{"MC",	"GB",	0},
	{"MD",	"GB",	0},
	{"ME",	"GB",	0},
	{"MF",	"GB",	0},
	{"MG",	"IL",	10},
	{"MH",	"BR",	0},
	{"MK",	"GB",	0},
	{"ML",	"IL",	10},
	{"MM",	"IL",	10},
	{"MN",	"IL",	10},
	{"MO",	"CN",	0},
	{"MP",	"US",	100},
	{"MQ",	"GB",	0},
	{"MR",	"GB",	0},
	{"MS",	"GB",	0},
	{"MT",	"GB",	0},
	{"MU",	"GB",	0},
	{"MD",	"GB",	0},
	{"ME",	"GB",	0},
	{"MF",	"GB",	0},
	{"MG",	"IL",	10},
	{"MH",	"BR",	0},
	{"MK",	"GB",	0},
	{"ML",	"IL",	10},
	{"MM",	"IL",	10},
	{"MN",	"IL",	10},
	{"MO",	"CN",	0},
	{"MP",	"US",	100},
	{"MQ",	"GB",	0},
	{"MR",	"GB",	0},
	{"MS",	"GB",	0},
	{"MT",	"GB",	0},
	{"MU",	"GB",	0},
	{"MV",	"RU",	1},
	{"MW",	"CN",	0},
	{"MX",	"RU",	1},
	{"MY",	"RU",	1},
	{"MZ",	"BR",	0},
	{"NA",	"BR",	0},
	{"NC",	"IL",	10},
	{"NE",	"BR",	0},
	{"NF",	"BR",	0},
	{"NG",	"NG",	0},
	{"NI",	"BR",	0},
	{"NL",	"GB",	0},
	{"NO",	"GB",	0},
	{"NP",	"SA",	0},
	{"NR",	"IL",	10},
	{"NU",	"BR",	0},
	{"NZ",	"BR",	0},
	{"OM",	"GB",	0},
	{"PA",	"RU",	1},
	{"PE",	"BR",	0},
	{"PF",	"GB",	0},
	{"PG",	"XY",	3},
	{"PH",	"BR",	0},
	{"PK",	"CN",	0},
	{"PL",	"GB",	0},
	{"PM",	"GB",	0},
	{"PN",	"GB",	0},
	{"PR",	"US",	100},
	{"PS",	"BR",	0},
	{"PT",	"GB",	0},
	{"PW",	"BR",	0},
	{"PY",	"BR",	0},
	{"QA",	"CN",	0},
	{"RE",	"GB",	0},
	{"RKS",	 "IL",	10},
	{"RO",	"GB",	0},
	{"RS",	"GB",	0},
	{"RU",	"RU",	10},
	{"RW",	"CN",	0},
	{"SA",	"SA",	0},
	{"SB",	"IL",	10},
	{"SC",	"IL",	10},
	{"SD",	"GB",	0},
	{"SE",	"GB",	0},
	{"SG",	"BR",	0},
	{"SI",	"GB",	0},
	{"SK",	"GB",	0},
	{"SKN",	 "CN",	0},
	{"SL",	"IL",	10},
	{"SM",	"GB",	0},
	{"SN",	"GB",	0},
	{"SO",	"IL",	10},
	{"SR",	"IL",	10},
	{"SS",	"GB",	0},
	{"ST",	"IL",	10},
	{"SV",	"RU",	1},
	{"SY",	"BR",	0},
	{"SZ",	"IL",	10},
	{"TC",	"GB",	0},
	{"TD",	"IL",	10},
	{"TF",	"GB",	0},
	{"TG",	"IL",	10},
	{"TH",	"BR",	0},
	{"TJ",	"IL",	10},
	{"TL",	"BR",	0},
	{"TM",	"IL",	10},
	{"TN",	"KW",	1},
	{"TO",	"IL",	10},
	{"TR",	"GB",	0},
	{"TT",	"BR",	0},
	{"TV",	"IL",	10},
	{"TW",	"TW",	2},
	{"TZ",	"CN",	0},
	{"UA",	"RU",	1},
	{"UG",	"BR",	0},
	{"US",	"US",	100},
	{"UY",	"BR",	0},
	{"UZ",	"IL",	10},
	{"VA",	"GB",	0},
	{"VC",	"BR",	0},
	{"VE",	"RU",	1},
	{"VG",	"GB",	0},
	{"VI",	"US",	100},
	{"VN",	"BR",	0},
	{"VU",	"IL",	10},
	{"WS",	"SA",	0},
	{"YE",	"IL",	10},
	{"YT",	"GB",	0},
	{"ZA",	"GB",	0},
	{"ZM",	"RU",	1},
	{"ZW",	"BR",	0},

};

static void *bcm_wifi_get_country_code(char *ccode)
{
	int size, i;
	static struct cntry_locales_custom country_code;

	size = ARRAY_SIZE(bcm_wifi_translate_custom_table);

	if ((size == 0) || (ccode == NULL))
		return NULL;

	for (i = 0; i < size; i++) {
		if (strcmp(ccode, bcm_wifi_translate_custom_table[i].iso_abbrev) == 0)
			return (void *)&bcm_wifi_translate_custom_table[i];
	}

	memset(&country_code, 0, sizeof(struct cntry_locales_custom));
	strlcpy(country_code.custom_locale, ccode, COUNTRY_BUF_SZ);

	return (void *)&country_code;
}
#endif //CONFIG_BROADCOM_WIFI_COUNTRYCODE
static struct wifi_platform_data pda_wifi_control = {
#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	.mem_prealloc	= brcm_wlan_mem_prealloc,
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */
	.set_power	= pda_wifi_power,
	.set_reset	= pda_wifi_reset,
	.set_carddetect	= pda_wifi_set_carddetect,
	.get_mac_addr	= pda_wifi_get_mac_addr,
#ifdef CONFIG_BROADCOM_WIFI_COUNTRYCODE
	.get_country_code = bcm_wifi_get_country_code,
#endif /* CONFIG_BROADCOM_WIFI_COUNTRYCODE */
#if defined(CONFIG_BCMDHD_EDP_SUPPORT)
	/* set the wifi edp client information here */
	.client_info    = {
		.name       = "wifi_edp_client",
		.states     = wifi_states,
		.num_states = ARRAY_SIZE(wifi_states),
		.e0_index   = 0,
		.priority   = EDP_MAX_PRIO,
	},
#endif
};

static struct resource wifi_resource[] = {
	[0] = {
		.name	= "bcmdhd_wlan_irq",
		/*HW OOB*/
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL
				| IORESOURCE_IRQ_SHAREABLE,
	},
};

static struct platform_device pda_wifi_device = {
	.name		= "bcmdhd_wlan",
	.id		= 1,
	.num_resources	= 1,
	.resource	= wifi_resource,
	.dev		= {
		.platform_data = &pda_wifi_control,
	},
};

#ifdef CONFIG_BROADCOM_WIFI_SWOOB /*SW OOB*/
static struct resource wifi_resource_swoob[] = {
	[0] = {
		.name	= "bcmdhd_wlan_irq",
		/*SW OOB*/
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE | IORESOURCE_IRQ_LOWEDGE
				| IORESOURCE_IRQ_SHAREABLE,
	},
};

static struct platform_device pda_wifi_device_swoob = {
	.name		= "bcmdhd_wlan",
	.id		= 1,
	.num_resources	= 1,
	.resource	= wifi_resource_swoob,
	.dev		= {
		.platform_data = &pda_wifi_control,
	},
};
#endif //#ifdef CONFIG_BROADCOM_WIFI_SWOOB /*SW OOB*/

static struct resource sdhci_resource0[] = {
	[0] = {
		.start  = INT_SDMMC1,
		.end    = INT_SDMMC1,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC1_BASE,
		.end	= TEGRA_SDMMC1_BASE + TEGRA_SDMMC1_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource sdhci_resource2[] = {
	[0] = {
		.start  = INT_SDMMC3,
		.end    = INT_SDMMC3,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC3_BASE,
		.end	= TEGRA_SDMMC3_BASE + TEGRA_SDMMC3_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource sdhci_resource3[] = {
	[0] = {
		.start  = INT_SDMMC4,
		.end    = INT_SDMMC4,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC4_BASE,
		.end	= TEGRA_SDMMC4_BASE + TEGRA_SDMMC4_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

#ifdef CONFIG_MMC_EMBEDDED_SDIO
static struct embedded_sdio_data embedded_sdio_data0 = {
	.cccr   = {
		.sdio_vsn       = 2,
		.multi_block    = 1,
		.low_speed      = 0,
		.wide_bus       = 0,
		.high_power     = 1,
		.high_speed     = 1,
	},
	.cis  = {
		.vendor	 = 0x02d0,
		.device	 = 0x4329,
	},
};
#endif

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data0 = {
	.mmc_data = {
		.register_status_notify	= pda_wifi_status_register,
#ifdef CONFIG_MMC_EMBEDDED_SDIO
		.embedded_sdio = &embedded_sdio_data0,
#endif
		.built_in = 0,
		.ocr_mask = MMC_OCR_1V8_MASK,
	},
#ifndef CONFIG_MMC_EMBEDDED_SDIO
	.pm_flags = MMC_PM_KEEP_POWER,
#endif
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.tap_delay = 0x3,
	.trim_delay = 0xA,
	.ddr_clk_limit = 41000000,
	.max_clk_limit = 136000000,
	.edp_support = false,
	.en_clock_gating = true,
	/* BCM4334 supports SDIO v3.0 mode up to 50MHz clock */
	.uhs_mask = MMC_UHS_MASK_DDR50 | MMC_UHS_MASK_SDR50 | MMC_UHS_MASK_SDR104,
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data2 = {
	.cd_gpio = MAXIM77660_SD_CD,
	.wp_gpio = -1,
	.power_gpio = -1,
	.tap_delay = 0x3,
	.trim_delay = 0xA,
	.ddr_clk_limit = 41000000,
	.max_clk_limit = 136000000,
	.edp_support = true,
	.edp_states = {1283, 0},
	.en_clock_gating = true,
	.uhs_mask = MMC_UHS_MASK_DDR50,
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data3 = {
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.is_8bit = 1,
	.tap_delay = 0x3,
	.trim_delay = 0xA,
	.max_clk_limit = 136000000,
	.ddr_trim_delay = 0,
	.mmc_data = {
		.built_in = 1,
		.ocr_mask = MMC_OCR_1V8_MASK,
	},
	.edp_support = true,
	.edp_states = {855, 0},
	.en_freq_scaling = true,
	.en_clock_gating = true,
	.en_pwroff_notify = false,
};

static u64 tegra_sdhci_dmamask0 = DMA_BIT_MASK(32);
static struct platform_device tegra_sdhci_device0 = {
	.name		= "sdhci-tegra",
	.id		= 0,
	.resource	= sdhci_resource0,
	.num_resources	= ARRAY_SIZE(sdhci_resource0),
	.dev = {
		.dma_mask = &tegra_sdhci_dmamask0,
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &tegra_sdhci_platform_data0,
	},
};

static u64 tegra_sdhci_dmamask2 = DMA_BIT_MASK(32);
static struct platform_device tegra_sdhci_device2 = {
	.name		= "sdhci-tegra",
	.id		= 2,
	.resource	= sdhci_resource2,
	.num_resources	= ARRAY_SIZE(sdhci_resource2),
	.dev = {
		.dma_mask = &tegra_sdhci_dmamask2,
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &tegra_sdhci_platform_data2,
	},
};

static u64 tegra_sdhci_dmamask3 = DMA_BIT_MASK(32);
static struct platform_device tegra_sdhci_device3 = {
	.name		= "sdhci-tegra",
	.id		= 3,
	.resource	= sdhci_resource3,
	.num_resources	= ARRAY_SIZE(sdhci_resource3),
	.dev = {
		.dma_mask = &tegra_sdhci_dmamask3,
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &tegra_sdhci_platform_data3,
	},
};

static int pda_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static int pda_wifi_set_carddetect(int val)
{
	pr_debug("%s: %d\n", __func__, val);
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
}

static int pda_wifi_power(int on)
{
	pr_err("%s: %d\n", __func__, on);

	gpio_set_value(PDA_WLAN_PWR, on);
	mdelay(100);

	return 0;
}

static int pda_wifi_reset(int on)
{
	pr_debug("%s: do nothing\n", __func__);
	return 0;
}

static int pda_wifi_get_mac_addr(unsigned char *buf)
{
	char mac_addr_str[] = "ff:ff:ff:ff:ff:ff";
#ifdef CONFIG_MACH_PDA
	/*                                                             */
	char wifi_mac_addr[6] = {0x00, 0x90, 0x4C,};

	get_random_bytes(&wifi_mac_addr[3], 3);
#endif

	memcpy(buf, wifi_mac_addr, 6);
	sprintf(mac_addr_str, "%02x:%02x:%02x:%02x:%02x:%02x",
		buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

	if ((strcmp("ff:ff:ff:ff:ff:ff", mac_addr_str) == 0) ||
			(strcmp("00:00:00:00:00:00", mac_addr_str) == 0)) {
		pr_err("WiFi MAC addr is not available in EEPROM\n");
		return -1;
	}

	pr_err("%s: WiFi MAC addr: %s\n", __func__, mac_addr_str);
	return 0;
}

static int __init pda_wifi_init(void)
{
	int rc;
	int pda_wlan_wow = PDA_WLAN_WOW;
	
	/*GPIO configuration changed*/
/*                                                                                            
                         
                         
                          
                                                                                             */
	if(lge_get_board_revno() < HW_REV_PDA_C) {
		pda_wlan_wow = TEGRA_GPIO_PO2;
	}

	rc = gpio_request(PDA_WLAN_PWR, "wlan_power");
	if (rc)
		pr_err("WLAN_PWR gpio request failed:%d\n", rc);
	rc = gpio_request(pda_wlan_wow, "bcmsdh_sdmmc");
	if (rc)
		pr_err("WLAN_WOW gpio request failed:%d\n", rc);

	rc = gpio_direction_output(PDA_WLAN_PWR, 0);
	if (rc)
		pr_err("WLAN_PWR gpio direction configuration failed:%d\n", rc);
	rc = gpio_direction_input(pda_wlan_wow);
	if (rc)
		pr_err("WLAN_WOW gpio direction configuration failed:%d\n", rc);

	wifi_resource[0].start = wifi_resource[0].end =
		gpio_to_irq(pda_wlan_wow);

	wifi_resource_swoob[0].start = wifi_resource_swoob[0].end =
		gpio_to_irq(pda_wlan_wow);

#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	brcm_init_wlan_mem();
#endif//CONFIG_BROADCOM_WIFI_RESERVED_MEM

	/*HW OOB support*/
/*                                                                                            
                         
                         
                          
                                                                                             */
	if(lge_get_board_revno() < HW_REV_D625_A) {
		platform_device_register(&pda_wifi_device);
		pr_err("pda_wifi_init pda_wifi_device registered\n");
	} else {
		/*from Rev.D or New Rev.A use swoob for bcm4334*/
		pr_err("pda_wifi_init pda_wifi_device_swoob registered\n");
		platform_device_register(&pda_wifi_device_swoob);
	}

	return 0;
}

#ifdef CONFIG_TEGRA_PREPOWER_WIFI
static int __init pda_wifi_prepower(void)
{
	if (!of_machine_is_compatible("nvidia,pda"))
		return 0;
	pda_wifi_power(1);

	return 0;
}

subsys_initcall_sync(pda_wifi_prepower);
#endif

int __init pda_sdhci_init(void)
{
	struct board_info board_info;
	int nominal_core_mv;
	int min_vcore_override_mv;
	int boot_vcore_mv;
	int speedo;

	nominal_core_mv =
		tegra_dvfs_rail_get_nominal_millivolts(tegra_core_rail);
	if (nominal_core_mv) {
		tegra_sdhci_platform_data0.nominal_vcore_mv = nominal_core_mv;
		tegra_sdhci_platform_data2.nominal_vcore_mv = nominal_core_mv;
		tegra_sdhci_platform_data3.nominal_vcore_mv = nominal_core_mv;
	}
	min_vcore_override_mv =
		tegra_dvfs_rail_get_override_floor(tegra_core_rail);
	if (min_vcore_override_mv) {
		tegra_sdhci_platform_data0.min_vcore_override_mv =
			min_vcore_override_mv;
		tegra_sdhci_platform_data2.min_vcore_override_mv =
			min_vcore_override_mv;
		tegra_sdhci_platform_data3.min_vcore_override_mv =
			min_vcore_override_mv;
	}
	boot_vcore_mv = tegra_dvfs_rail_get_boot_level(tegra_core_rail);
	if (boot_vcore_mv) {
		tegra_sdhci_platform_data0.boot_vcore_mv = boot_vcore_mv;
		tegra_sdhci_platform_data2.boot_vcore_mv = boot_vcore_mv;
		tegra_sdhci_platform_data3.boot_vcore_mv = boot_vcore_mv;
	}

	tegra_get_board_info(&board_info);
	if ((board_info.board_id == BOARD_E1670) ||
		 (board_info.board_id == BOARD_E1740))
		tegra_sdhci_platform_data2.cd_gpio = PALMAS_SD_CD;
	else {
	
		tegra_sdhci_platform_data2.max_clk_limit = 204000000;
		tegra_sdhci_platform_data2.en_freq_scaling = true;
//		tegra_sdhci_platform_data0.max_clk_limit = 136000000;
//		tegra_sdhci_platform_data3.max_clk_limit = 192000000;
	}

	speedo = tegra_fuse_readl(FUSE_CORE_SPEEDO_0);
	tegra_sdhci_platform_data0.cpu_speedo = speedo;
	tegra_sdhci_platform_data2.cpu_speedo = speedo;
	tegra_sdhci_platform_data3.cpu_speedo = speedo;

	platform_device_register(&tegra_sdhci_device3);
	platform_device_register(&tegra_sdhci_device2);
	platform_device_register(&tegra_sdhci_device0);
	pda_wifi_init();
	return 0;
}
