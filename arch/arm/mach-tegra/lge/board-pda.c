/*
 * arch/arm/mach-tegra/lge/board-pda.c
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/platform_data/tegra_usb_modem_power.h>
#include <linux/memblock.h>
#include <linux/of_platform.h>
#include <linux/bluedroid_pm.h>
#include <linux/serial_8250.h>
#include <linux/tegra_uart.h>
#include <linux/i2c.h>
#include <linux/i2c-tegra.h>
#include <linux/i2c/at24.h>
#include <linux/spi/spi.h>
#include <linux/nfc/pn544_lge.h>
#include <linux/spi/rm31080a_ts.h>
#include <linux/spi-tegra.h>
#include <sound/max98090.h>
#ifndef CONFIG_MACH_PD_A
#include <sound/max97236.h>
#endif
#include <asm/hardware/gic.h>

#include <linux/mfd/tlv320aic3256-registers.h>
#include <linux/mfd/tlv320aic325x-registers.h>
#include <linux/mfd/tlv320aic325x-core.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux-t14.h>
#include <mach/isomgr.h>
#include <mach/tegra_bb.h>
#include <mach/tegra_bbc_proxy.h>
#include <mach/hardware.h>
#include <mach/tegra_wakeup_monitor.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/tegra_fiq_debugger.h>

#include <mach/tegra_asoc_pdata.h>
#include <linux/platform_data/hds_max1462x.h>
#include <linux/platform_data/hds_max14688.h>

#include "../board.h"
#include "../tegra-board-id.h"
#include "board-pda.h"
#include "board-atlantis-pda.h"
#include "../board-common.h"
#include "../board-touch-raydium.h"
#include "../clock.h"
#include "../devices.h"
#include "../common.h"
#include "../board-touch.h"
#include "../fuse.h"
#include <mach/board_lge.h>

/* mms136 touch */
#include <linux/i2c/melfas_ts.h>
#include <linux/mfd/max77660/max77660-core.h>
//                                                                   
#if defined (CONFIG_TOUCHSCREEN_ATMEL_S336)
#include "../../../../drivers/input/touchscreen/atmel_s336.h"
#endif// CONFIG_TOUCHSCREEN_ATMEL_S336
//                                               

#define FUSE_SPARE_BIT_12_0     0x2d0

#define PDA_BT_EN		TEGRA_GPIO_PM3
#define PDA_BT_HOST_WAKE	TEGRA_GPIO_PM2
#define PDA_BT_EXT_WAKE	TEGRA_GPIO_PM1
// eric0.kim[2013.12.17]
//#define PDA_NFC_EN		TEGRA_GPIO_PI0
//#define PDA_NFC_WAKE		TEGRA_GPIO_PM0
#define PDA_NFC_IRQ			TEGRA_GPIO_PM4
#define PDA_NFC_EN			TEGRA_GPIO_PM5
#define PDA_NFC_WAKE		TEGRA_GPIO_PM6

//                                                                   
#if defined (CONFIG_TOUCHSCREEN_ATMEL_S336)
#define ATMEL_TS_ADDR		0x4a
#endif// CONFIG_TOUCHSCREEN_ATMEL_S336
//                                               

static struct board_info board_info;
#ifndef CONFIG_MACH_PDA
char wifi_mac_addr[6];
#endif

#ifdef CONFIG_BT_BLUESLEEP
static struct rfkill_gpio_platform_data pda_bt_rfkill_pdata = {
	.name           = "bt_rfkill",
	.reset_gpio	= PDA_BT_EN,
	.type           = RFKILL_TYPE_BLUETOOTH,
};

static struct platform_device pda_bt_rfkill_device = {
	.name = "rfkill_gpio",
	.id             = -1,
	.dev = {
		.platform_data = &pda_bt_rfkill_pdata,
	},
};

static noinline void __init pda_setup_bt_rfkill(void)
{
	platform_device_register(&pda_bt_rfkill_device);
}

static struct resource pda_bluesleep_resources[] = {
	[0] = {
		.name = "gpio_host_wake",
			.start  = PDA_BT_HOST_WAKE,
			.end    = PDA_BT_HOST_WAKE,
			.flags  = IORESOURCE_IO,
	},
	[1] = {
		.name = "gpio_ext_wake",
			.start  = PDA_BT_EXT_WAKE,
			.end    = PDA_BT_EXT_WAKE,
			.flags  = IORESOURCE_IO,
	},
	[2] = {
		.name = "host_wake",
			.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
};

static struct platform_device pda_bluesleep_device = {
	.name           = "bluesleep",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(pda_bluesleep_resources),
	.resource       = pda_bluesleep_resources,
};

static noinline void __init pda_setup_bluesleep(void)
{
	pda_bluesleep_resources[2].start =
		pda_bluesleep_resources[2].end =
			gpio_to_irq(PDA_BT_HOST_WAKE);
	platform_device_register(&pda_bluesleep_device);
	return;
}
#elif defined CONFIG_BLUEDROID_PM
static struct resource bluetooth_pm_resources[] = {
	[0] = {
		.name   = "gpio_bt_reset",
		.start  = PDA_BT_EN,
		.end    = PDA_BT_EN,
		.flags  = IORESOURCE_IO,
	},

	[1] = {
		.name = "host_wake",
		.flags  = IORESOURCE_IRQ,
	},
	[2] = {
		.name = "gpio_ext_wake",
		.start  = PDA_BT_EXT_WAKE,
		.end    = PDA_BT_EXT_WAKE,
		.flags  = IORESOURCE_IO,
	},
	[3] = {
		.name = "gpio_host_wake",
		.start  = PDA_BT_HOST_WAKE,
		.end    = PDA_BT_HOST_WAKE,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device bluetooth_pm_device = {
	.name = "bluetooth_pm",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(bluetooth_pm_resources),
	.resource       = bluetooth_pm_resources,
};

static noinline void __init register_bluetooth_pm(void)
{
	bluetooth_pm_resources[1].start =
		bluetooth_pm_resources[1].end =
					gpio_to_irq(PDA_BT_HOST_WAKE);
	platform_device_register(&bluetooth_pm_device);
}
#endif

/*
static struct bcm2079x_platform_data nfc_pdata = {
	.irq_gpio = PDA_NFC_IRQ,
	.en_gpio = PDA_NFC_EN,
	.wake_gpio = PDA_NFC_WAKE,
	};

static struct i2c_board_info __initdata pda_i2c_bus3_board_info[] = {
	{
		//I2C_BOARD_INFO("bcm2079x", 0x77), //for BCM20793
		I2C_BOARD_INFO("bcm2079x", 0x76), //for BCM20795
		.platform_data = &nfc_pdata,
	},
};
*/

static struct pn544_i2c_platform_data nfc_pdata = {
	.irq_gpio = PDA_NFC_IRQ,
	.ven_gpio = PDA_NFC_EN,
	.firm_gpio = PDA_NFC_WAKE,
};

static struct i2c_board_info __initdata pda_i2c_bus3_board_info[] = {
	{
		I2C_BOARD_INFO("pn547", 0x28),
		.platform_data = &nfc_pdata,
	},
};

#ifndef CONFIG_MACH_PDA
void pda_get_mac_addr(struct memory_accessor *mem_acc, void *context)
{
	int ret = 0;
	char *mac_addr;
	off_t offset = (off_t)context;
	mac_addr = kzalloc(sizeof(char)*32, GFP_ATOMIC);
	if (!mac_addr) {
		pr_err("no memory to allocate");
		return;
	}

	/* Read MAC addr from EEPROM */
	ret = mem_acc->read(mem_acc, mac_addr, offset, 32);
	if (ret == 32) {
		wifi_mac_addr[0] = *(mac_addr + 20);
		wifi_mac_addr[1] = *(mac_addr + 21);
		wifi_mac_addr[2] = *(mac_addr + 22);
		wifi_mac_addr[3] = *(mac_addr + 23);
		wifi_mac_addr[4] = *(mac_addr + 24);
		wifi_mac_addr[5] = *(mac_addr + 25);
	} else {
		pr_err("Error reading MAC addr from EEPROM\n");
	}
}

static struct at24_platform_data pda_eeprom_info = {
	.byte_len	= (256*1024)/8,
	.page_size	= 64,
	.flags		= AT24_FLAG_ADDR16,
	.setup		= pda_get_mac_addr,
};

static struct i2c_board_info pda_eeprom_mac_add = {
	I2C_BOARD_INFO("at24", 0x56),
	.platform_data = &pda_eeprom_info,
};
#endif

static struct max98090_eq_cfg max98090_eq_cfg[] = {
};

static struct max98090_biquad_cfg max98090_bq_cfg[] = {
};

static struct max98090_pdata pda_max98090_pdata = {
	/* Equalizer Configuration */
	.eq_cfg = max98090_eq_cfg,
	.eq_cfgcnt = ARRAY_SIZE(max98090_eq_cfg),

    /* bi-quad filters */
    .bq_cfg = max98090_bq_cfg,
    .bq_cfgcnt = ARRAY_SIZE(max98090_bq_cfg),

	/* Microphone Configuration */
	.digmic_left_mode = 1,
	.digmic_right_mode = 1,
};

#ifndef CONFIG_MACH_PDA
static struct aic325x_gpio_setup aic3256_gpio[] = {
	/* MISO: Clock o/p for DMIC */
	{
		.reg    = AIC3XXX_MAKE_REG(0, 0, 55),
		.value  = 0x0E,
	},
	/* SCLK: DMIC DATA*/
	{
		.reg    = AIC3XXX_MAKE_REG(0, 0, 56),
		.value  = 0x02,
	},
	{
		.reg    = AIC3XXX_MAKE_REG(0, 0, 81),
		.value  = 0x10,
	}
};

static struct aic325x_pdata aic3256_codec_pdata = {
	.gpio_irq	= 0,
	.num_gpios	= ARRAY_SIZE(aic3256_gpio),
	.gpio_defaults  = aic3256_gpio,
	.naudint_irq    = 0,
};
#endif

static struct i2c_board_info __initdata max98090_board_info = {
	I2C_BOARD_INFO("max98090", 0x10),
	.platform_data = &pda_max98090_pdata,
#ifdef CONFIG_MACH_PDA
	.irq		= AUD_INT,
#else
	.irq		= TEGRA_GPIO_CDC_IRQ,
#endif
};

#ifndef CONFIG_MACH_PDA
static struct i2c_board_info __initdata pda_codec_aic325x_info = {
	I2C_BOARD_INFO("tlv320aic325x", 0x18),
	.platform_data = &aic3256_codec_pdata,
};
#endif

struct ahub_bbc1_config ahub_bbc1_pdata = {
	.port_id	= 0,
	.sample_size	= 16,
	.rate		= 16000,
	.channels	= 2,
	.bit_clk	= 1536000,
};

static struct tegra_asoc_platform_data pda_audio_max98090_pdata = {
	.gpio_spkr_en		= TEGRA_GPIO_SPKR_EN,
#ifndef CONFIG_MACH_PDA
	.gpio_hp_det		= TEGRA_GPIO_HP_DET,
#else
	.gpio_hp_det		= -1,
#endif
	.gpio_hp_mute		= -1,
	.gpio_int_mic_en	= TEGRA_GPIO_INT_MIC_EN,
	.gpio_ext_mic_en	= TEGRA_GPIO_EXT_MIC_EN,
#ifndef CONFIG_MACH_PDA
	.gpio_ldo1_en		= TEGRA_GPIO_LDO1_EN,
#endif
	.edp_support		= true,
	.edp_states		= {1080, 842, 0},
	.i2s_param[HIFI_CODEC]	= {
		.audio_port_id	= 0,
		.is_i2s_master	= 1,
//                                           
//		.i2s_mode	= TEGRA_DAIFMT_DSP_A,
        .i2s_mode	= TEGRA_DAIFMT_I2S,
//                    
		.sample_size	= 16,
		.channels       = 2,
		.bit_clk	= 1536000,
		.boost_cpu_freq = 204000000, /* 204MHz */
		.boost_emc_freq = 102000000, /* 102MHz */
		.boost_sclk_freq = 20400000, /* 20.4MHz */
	},
	.i2s_param[BT_SCO]	= {
		.audio_port_id	= 1,
		.is_i2s_master	= 1,
		.i2s_mode	= TEGRA_DAIFMT_DSP_A,
		.sample_size	= 16,
		.channels	= 1,
//                                            
//		.bit_clk	= 512000,
		.bit_clk	= 2048000,
//                    
	},
#ifdef CONFIG_FM_RADIO
	.i2s_param[FM_RADIO]	= {
		.audio_port_id = 4,
		.is_i2s_master = 1,
		.i2s_mode = TEGRA_DAIFMT_I2S,
		.sample_size = 16,
		.channels = 2,
		.bit_clk = 1536000,
		.rate		= 48000
	},
#endif
	.ahub_bbc1_param	= &ahub_bbc1_pdata,
};

static struct platform_device pda_audio_max98090_device = {
	.name	= "tegra-snd-max98090",
	.id	= 0,
	.dev	= {
		.platform_data = &pda_audio_max98090_pdata,
	},
};

#ifndef CONFIG_MACH_PDA
static struct max97236_pdata pda_audio_max97236_pdata;

static struct platform_device pda_audio_max97236_device = {
	.name	= "tegra-snd-max97236",
	.id		= -1,
	.dev = {
		.platform_data = &pda_audio_max97236_pdata,
	},
};

static struct tegra_asoc_platform_data pda_audio_aic325x_pdata = {
	.gpio_ldo1_en = TEGRA_GPIO_LDO1_EN,
	.gpio_spkr_en = -1,
	.gpio_hp_det = (PALMAS_TEGRA_GPIO_BASE + PALMAS_GPIO4),
	.gpio_hp_mute = -1,
	.gpio_int_mic_en = -1,
	.gpio_ext_mic_en = TEGRA_GPIO_HP_MIC_DET,
	.debounce_time_hp = 200,
	.edp_support		= true,
	.edp_states		= {1080, 842, 0},
	.i2s_param[HIFI_CODEC]  = {
		.audio_port_id	= 0,
		.is_i2s_master	= 1,
		.i2s_mode	= TEGRA_DAIFMT_I2S,
		.sample_size	= 16,
		.bit_clk	= 1536000,
	},
	.i2s_param[BT_SCO]	= {
		.audio_port_id	= 1,
		.is_i2s_master	= 1,
		.i2s_mode	= TEGRA_DAIFMT_DSP_A,
		.sample_size	= 16,
		.channels	= 1,
		.bit_clk	= 512000,
	},
	.ahub_bbc1_param	= &ahub_bbc1_pdata,
};

static struct platform_device pda_audio_aic325x_device = {
	.name	= "tegra-snd-aic325x",
	.id	= 0,
	.dev	= {
		.platform_data  = &pda_audio_aic325x_pdata,
	},
};
#endif	/* !CONFIG_MACH_PDA */

#ifdef CONFIG_SWITCH_MAX1462X
static struct max1462x_platform_data lge_hs_pdata = {
	.switch_name = "h2w",
	.keypad_name = "hs_detect",

#if defined(CONFIG_SWITCH_3BUTTON_EARJACK)
	.key_code = 0,	/* KEY_MEDIA, KEY_VOLUMEUP or KEY_VOLUMEDOWN */
#else
	.key_code = KEY_MEDIA,	/* support only HOOK key */
#endif

	.gpio_mode	= TEGRA_GPIO_PH2, /* EAR_MIC_EN_35D, */
	.gpio_det	= EAR_SENSE_N_35D,
	.gpio_swd	= TEGRA_GPIO_PO6, /* EAR_KEY_INT_35D, */
	.external_ldo_mic_bias	= 0,
	.set_headset_mic_bias = 0,
	/* If gpio connect the PMIC, function use cansleep */
	/* If gpio connect the AP, function don't use cansleep */
	.gpio_set_value_func = gpio_set_value_cansleep,
	.gpio_get_value_func = gpio_get_value_cansleep,
	.latency_for_detection = 75,
};

static struct platform_device lge_hsd_device = {
	.name	= "max1462x",
	.id		= -1,
	.dev = {
		.platform_data = &lge_hs_pdata,
	},
};
#endif /* CONFIG_SWITCH_MAX1462X */

static struct tegra_asoc_platform_data dmic_platform_data = {
	.codec_name = "dmic-codec",
	.codec_dai_name = "dmic-hifi",
};

static struct platform_device pda_dmic_device = {
	.name	= "tegra-snd-dmic_rev1.0",
	.id	= 0,
	.dev = {
		.platform_data = &dmic_platform_data,
	},
};

static struct platform_device dmic_codec_device = {
	.name	= "dmic-codec",
	.id	= 0,
	.dev = {
		.platform_data = &dmic_platform_data,
	},
};

#if defined(CONFIG_TEGRA_BASEBAND)
static struct tegra_bb_platform_data pda_tegra_bb_data;

static struct platform_device pda_tegra_bb_device = {
	.name = "tegra_bb",
	.id = 0,
	.dev = {
		.platform_data = &pda_tegra_bb_data,
	},
};
#endif

static struct platform_device *pda_spi_devices[] __initdata = {
	&tegra11_spi_device2,
	&tegra11_spi_device3,
};

struct spi_clk_parent spi_parent_clk_pda[] = {
	[0] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
#else
	[1] = {.name = "clk_m"},
#endif
};

static struct tegra_spi_platform_data pda_spi_pdata = {
	.is_dma_based           = true,
	.max_dma_buffer         = 16 * 1024,
	.is_clkon_always        = false,
	.max_rate               = 25000000,
};

static void __init pda_spi_init(void)
{
	int i;
	struct clk *c;
	struct board_info display_board_info;

	tegra_get_display_board_info(&display_board_info);

	for (i = 0; i < ARRAY_SIZE(spi_parent_clk_pda); ++i) {
		c = tegra_get_clock_by_name(spi_parent_clk_pda[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
			       spi_parent_clk_pda[i].name);
			continue;
		}
		spi_parent_clk_pda[i].parent_clk = c;
		spi_parent_clk_pda[i].fixed_clk_rate = clk_get_rate(c);
	}
	pda_spi_pdata.parent_clk_list = spi_parent_clk_pda;
	pda_spi_pdata.parent_clk_count = ARRAY_SIZE(spi_parent_clk_pda);

	tegra11_spi_device2.dev.platform_data = &pda_spi_pdata;
	tegra11_spi_device3.dev.platform_data = &pda_spi_pdata;
	platform_add_devices(pda_spi_devices, ARRAY_SIZE(pda_spi_devices));
}

#define BBC_BOOT_EDP_MAX 0
static unsigned int bbc_boot_edp_states[] = { 1900, 0 };
static struct edp_client bbc_boot_edp_client = {
	.name = "bbc_boot",
	.states = bbc_boot_edp_states,
	.num_states = ARRAY_SIZE(bbc_boot_edp_states),
	.e0_index = BBC_BOOT_EDP_MAX,
	.priority = EDP_MAX_PRIO,
};

static struct tegra_bbc_proxy_platform_data bbc_proxy_pdata = {
	.modem_boot_edp_client = &bbc_boot_edp_client,
	.edp_manager_name = "battery",
	.ap_name = "core",
};

static struct platform_device tegra_bbc_proxy_device = {
	.name = "tegra_bbc_proxy",
	.id = -1,
	.dev = {
		.platform_data = &bbc_proxy_pdata,
	},
};

#if defined(CONFIG_TEGRA_WAKEUP_MONITOR)
static struct tegra_wakeup_monitor_platform_data
			pda_tegra_wakeup_monitor_pdata = {
	.wifi_wakeup_source	= 9,
	.rtc_wakeup_source	= 18,
};

static struct platform_device pda_tegra_wakeup_monitor_device = {
	.name = "tegra_wakeup_monitor",
	.id   = -1,
	.dev  = {
		.platform_data = &pda_tegra_wakeup_monitor_pdata,
	},
};
#endif

static struct platform_device *pda_only_audio_devices[] __initdata = {
	&pda_audio_max98090_device,
#ifndef CONFIG_MACH_PDA
	&pda_audio_max97236_device,
#endif
#ifdef CONFIG_SWITCH_MAX1462X
	&lge_hsd_device,
#endif
};

#ifndef CONFIG_MACH_PDA
static struct platform_device *atlantis_only_audio_devices[] __initdata = {
	&pda_audio_aic325x_device,
};
#endif

static struct platform_device *pda_common_audio_devices[] __initdata = {
	&tegra_ahub_device,
	&tegra_pcm_device,
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device0,
	&tegra_i2s_device1,
	&tegra_i2s_device2,
	&tegra_i2s_device3,
	&tegra_i2s_device4,
#ifdef CONFIG_ARCH_TEGRA_14x_SOC
	&tegra_dmic_device0,
	&tegra_dmic_device1,
#endif
	&spdif_dit_device,
	&tegra_spdif_device,
	&bluetooth_dit_device,
	&baseband_dit_device,
#if defined(CONFIG_SND_HDA_PLATFORM_NVIDIA_TEGRA)
	&tegra_hda_device,
#endif
	&dmic_codec_device,
	&pda_dmic_device,
};

static struct platform_device *pda_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_udc_device,
#if defined(CONFIG_TEGRA_WATCHDOG)
	&tegra_wdt0_device,
#endif
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	&tegra11_se_device,
#endif
#ifdef CONFIG_ARCH_TEGRA_14x_SOC
	&tegra_mipi_bif_device,
#endif
#if defined(CONFIG_TEGRA_WAKEUP_MONITOR)
	&pda_tegra_wakeup_monitor_device,
#endif
};

#ifndef CONFIG_MACH_PDA
static struct i2c_board_info __initdata max97236_board_info = {
	I2C_BOARD_INFO("max97236", 0x40),
	.irq = TEGRA_GPIO_HP_DET,
};
#endif

static void pda_audio_init(void)
{
#ifndef CONFIG_MACH_PDA
	if ((board_info.board_id == BOARD_E1670) ||
		(board_info.board_id == BOARD_E1671) ||
		(board_info.board_id == BOARD_E1740)) {
		pda_codec_aic325x_info.irq = gpio_to_irq(TEGRA_GPIO_CDC_IRQ);
		i2c_register_board_info(5, &pda_codec_aic325x_info, 1);
		platform_add_devices(atlantis_only_audio_devices,
			ARRAY_SIZE(atlantis_only_audio_devices));
	}
#endif

	if ((board_info.board_id == BOARD_E1680) ||
		(board_info.board_id == BOARD_E1681) ||
		(board_info.board_id == BOARD_E1683) ||
		(board_info.board_id == BOARD_E1690)) {
#ifndef CONFIG_MACH_PDA
		i2c_register_board_info(5, &max97236_board_info, 1);
#endif
		i2c_register_board_info(5, &max98090_board_info, 1);

/*                                                                                            
                         
                         
                          
                                                                                             */
		if(lge_get_board_revno() >= HW_REV_PDA_C) {
			platform_add_devices(pda_only_audio_devices,
				ARRAY_SIZE(pda_only_audio_devices)-1);
		} else {
			platform_add_devices(pda_only_audio_devices,
				ARRAY_SIZE(pda_only_audio_devices));
		}
	}

	platform_add_devices(pda_common_audio_devices,
			ARRAY_SIZE(pda_common_audio_devices));
}

#ifdef CONFIG_USB_SUPPORT
static struct tegra_usb_platform_data tegra_udc_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.support_pmu_vbus = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.unaligned_dma_buf_supported = false,
	.op_mode = TEGRA_USB_OPMODE_DEVICE,
	.u_data.dev = {
		.vbus_pmu_irq = 0,
		.vbus_gpio = -1,
		.charging_supported = false,
		.remote_wakeup_supported = false,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static struct tegra_usb_platform_data tegra_ehci1_utmi_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = false,
	.support_pmu_vbus = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 15,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static struct tegra_usb_otg_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci1_utmi_pdata,
};

#ifndef CONFIG_MACH_PDA
static struct tegra_usb_platform_data tegra_ehci2_hsic_smsc_hub_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = false,
	.phy_intf = TEGRA_USB_PHY_INTF_HSIC,
	.op_mode	= TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
};

static struct gpio modem2_gpios[] = {
	{MDM2_EN, GPIOF_OUT_INIT_LOW, "MDM2_EN"},
	{MDM2_RST, GPIOF_OUT_INIT_LOW, "MDM2_RST"},
};

static void baseband2_start(void)
{
	pr_info("%s\n", __func__);
	gpio_set_value(MDM2_EN, 1);
	gpio_set_value(MDM2_RST, 1);
}

static void baseband2_reset(void)
{
	/* Initiate power cycle on baseband sub system */
	pr_info("%s\n", __func__);
	gpio_set_value(MDM2_RST, 0);
	mdelay(200);
	gpio_set_value(MDM2_RST, 1);
}

static int baseband2_init(void)
{
	int ret;

	ret = gpio_request_array(modem2_gpios, ARRAY_SIZE(modem2_gpios));
	if (ret)
		return ret;

	/* enable pull-down for MDM2_COLD_BOOT */
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_DAP5_DIN,
				    TEGRA_PUPD_PULL_DOWN);

	/* export GPIO for user space access through sysfs */
	gpio_export(MDM2_RST, false);

	return 0;
}

static const struct tegra_modem_operations baseband2_operations = {
	.init = baseband2_init,
	.start = baseband2_start,
	.reset = baseband2_reset,
};

static struct tegra_usb_modem_power_platform_data baseband2_pdata = {
	.ops = &baseband2_operations,
	.wake_gpio = -1,
	.boot_gpio = MDM2_COLDBOOT,
	.boot_irq_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
	.autosuspend_delay = 2000,
	.short_autosuspend_delay = 50,
	.tegra_ehci_device = &tegra_ehci2_device,
	.tegra_ehci_pdata = &tegra_ehci2_hsic_smsc_hub_pdata,
};

static struct platform_device icera_baseband2_device = {
	.name = "tegra_usb_modem_power",
	.id = -1,
	.dev = {
		.platform_data = &baseband2_pdata,
	},
};
#endif /* !CONFIG_MACH_PDA */

static void pda_usb_init(void)
{
	// pr_info("%s board_id : %d\n", __func__, board_info.board_id);

	switch (board_info.board_id) {
	case BOARD_E1680:
	case BOARD_E1681:
	case BOARD_E1683:
		/* Device cable is detected through PMU Interrupt */
		tegra_otg_pdata.vbus_extcon_dev_name = "max77660-extcon";

		/* Host cable is detected through PMU GPIO Interrupt */
		if ((lge_get_board_revno() >= HW_REV_D625_B) && lge_get_normal_boot()) {
			pr_info("%s board_id : rev_b and the upper versions  \n", __func__);
			tegra_udc_pdata.id_det_type = TEGRA_USB_GPIO_ID;
			tegra_ehci1_utmi_pdata.id_det_type = TEGRA_USB_GPIO_ID;
			tegra_otg_pdata.id_det_gpio =
					MAX77660_GPIO_BASE + MAX77660_GPIO8;
		} else {
			pr_info("%s board_id : rev_a and the lower versions  \n", __func__);
			tegra_udc_pdata.id_det_type = TEGRA_USB_VIRTUAL_ID;
			tegra_ehci1_utmi_pdata.id_det_type = TEGRA_USB_VIRTUAL_ID;
		}
		break;
	case BOARD_E1670:
	case BOARD_E1671:
	case BOARD_E1740:
		/* Device cable is detected through PMU Interrupt */
		tegra_otg_pdata.vbus_extcon_dev_name = "palmas-extcon";

		/* Host cable is detected through PMU Interrupt */
		tegra_udc_pdata.id_det_type = TEGRA_USB_PMU_ID;
		tegra_ehci1_utmi_pdata.id_det_type = TEGRA_USB_PMU_ID;
		tegra_otg_pdata.id_extcon_dev_name = "palmas-extcon";
		break;
	case BOARD_E1690:
		/* Device cable is detected through PMU Interrupt */
		tegra_otg_pdata.vbus_extcon_dev_name = "max77660-extcon";

		if (board_info.fab < BOARD_FAB_A02) {
			tegra_udc_pdata.id_det_type = TEGRA_USB_VIRTUAL_ID;
			tegra_ehci1_utmi_pdata.id_det_type =
					TEGRA_USB_VIRTUAL_ID;
		} else {
			/* Host cable is detected through PMU GPIO Interrupt */
			tegra_udc_pdata.id_det_type = TEGRA_USB_GPIO_ID;
			tegra_ehci1_utmi_pdata.id_det_type = TEGRA_USB_GPIO_ID;
			tegra_otg_pdata.id_det_gpio =
					MAX77660_GPIO_BASE + MAX77660_GPIO8;
		}
		break;
	default:
		pr_err("%s: board_id=%#x unknown tegra14x board.\n", __func__,
					board_info.board_id);
	}

	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);

	/* Setup the udc platform data */
	tegra_udc_device.dev.platform_data = &tegra_udc_pdata;
}

#else
static void pda_usb_init(void) { }

/* Secondary modem init according to modem_id */
static void pda_modem_init(void)
{
#ifndef CONFIG_MACH_PDA
	int modem_id = tegra_get_modem_id();

	pr_info("%s: modem_id = %d\n", __func__, modem_id);

	switch (modem_id) {
	case TEGRA_BB_I500SWD: /* i500 SWD/Nemo */
		platform_device_register(&icera_baseband2_device);
		break;
	case TEGRA_BB_HSIC_HUB: /* HSIC hub */
		tegra_ehci2_device.dev.platform_data =
			&tegra_ehci2_hsic_smsc_hub_pdata;
		platform_device_register(&tegra_ehci2_device);
		break;
	default:
		return;
	}
#endif /* !CONFIG_MACH_PDA */
}
#endif

static __initdata struct tegra_clk_init_table pda_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",	NULL,		0,		false},
	{ "vi_sensor",	"pll_p",	150000000,	false},
	{ "vi_sensor2",	"pll_p",	150000000,	false},
	{ "vi",		"pll_p",	100000000,	false},
	{ "isp",	"pll_p",	150000000,	false},
	{ "isp_sapor",	"pll_p",	150000000,	false},
	{ "cilab",	"pll_p",	102000000,	false},
	{ "cile",	"pll_p",	102000000,	false},
	{ "i2c1",	"pll_p",	3200000,	false},
	{ "i2c2",	"pll_p",	3200000,	false},
	{ "i2c3",	"pll_p",	3200000,	false},
	{ "i2c4",	"pll_p",	3200000,	false},
	{ "i2c5",	"pll_p",	3200000,	false},
	{ "i2c6",	"pll_p",	3200000,	false},
	{ "hda",	"pll_p",	108000000,	false},
	{ "hda2codec_2x", "pll_p",	48000000,	false},
	{ "i2s0",	"pll_a_out0",	0,		false},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s2",	"pll_a_out0",	0,		false},
	{ "i2s3",	"pll_a_out0",	0,		false},
	{ "i2s4",	"pll_a_out0",	0,		false},
	{ "dmic0",	"pll_a_out0",	0,		false},
	{ "dmic1",	"pll_a_out0",	0,		false},
	{ "d_audio",	"clk_m",	12000000,	false},
	{ "dam0",	"clk_m",	12000000,	false},
	{ "dam1",	"clk_m",	12000000,	false},
	{ "dam2",	"clk_m",	12000000,	false},
	{ "audio0",	"i2s0_sync",	0,		false},
	{ "audio1",	"i2s1_sync",	0,		false},
	{ "audio2",	"i2s2_sync",	0,		false},
	{ "audio3",	"i2s3_sync",	0,		false},
	{ NULL,		NULL,		0,		0},
};

static struct platform_device *pda_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
};
static struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "clk_m"},
	[1] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[2] = {.name = "pll_m"},
#endif
};

static struct tegra_uart_platform_data pda_uart_pdata;
static struct tegra_uart_platform_data pda_bt_uart_pdata;
static struct tegra_uart_platform_data pda_loopback_uart_pdata;

static void __init uart_debug_init(void)
{
	int debug_port_id;

#ifdef CONFIG_MACH_PDA
	debug_port_id = uart_console_debug_init(0);
#else
	debug_port_id = uart_console_debug_init(3);
#endif
	if (debug_port_id < 0)
		return;

	pda_uart_devices[debug_port_id] = uart_console_debug_device;
}

static void __init pda_uart_init(void)
{
	struct clk *c;
	int i;

	for (i = 0; i < ARRAY_SIZE(uart_parent_clk); ++i) {
		c = tegra_get_clock_by_name(uart_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						uart_parent_clk[i].name);
			continue;
		}
		uart_parent_clk[i].parent_clk = c;
		uart_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	pda_uart_pdata.parent_clk_list = uart_parent_clk;
	pda_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	pda_bt_uart_pdata.parent_clk_list = uart_parent_clk;
	pda_bt_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	pda_bt_uart_pdata.boost_emc_freq = 204000000; /* Hz */
	pda_bt_uart_pdata.boost_sclk_freq = 102000000; /* Hz */
	pda_loopback_uart_pdata.parent_clk_list = uart_parent_clk;
	pda_loopback_uart_pdata.parent_clk_count =
						ARRAY_SIZE(uart_parent_clk);
	pda_loopback_uart_pdata.is_loopback = true;
	tegra_uarta_device.dev.platform_data = &pda_uart_pdata;
	tegra_uartb_device.dev.platform_data = &pda_uart_pdata;
	tegra_uartc_device.dev.platform_data = &pda_bt_uart_pdata;
	tegra_uartd_device.dev.platform_data = &pda_uart_pdata;

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs())
		uart_debug_init();

	platform_add_devices(pda_uart_devices,
				ARRAY_SIZE(pda_uart_devices));
}

static struct tegra_i2c_platform_data pda_i2c1_platform_data = {
	.bus_clk_rate	= 400000,
	.scl_gpio	= TEGRA_GPIO_I2C1_SCL,
	.sda_gpio	= TEGRA_GPIO_I2C1_SDA,
	.is_clkon_always = true,
};

static struct tegra_i2c_platform_data pda_i2c2_platform_data = {
	.bus_clk_rate	= 400000,
	.scl_gpio	= -1,
	.sda_gpio	= -1,
};

static struct tegra_i2c_platform_data pda_i2c3_platform_data = {
	.bus_clk_rate	= 400000,
	.scl_gpio	= -1,
	.sda_gpio	= -1,
};

static struct tegra_i2c_platform_data pda_i2c4_platform_data = {
	.bus_clk_rate	= 100000,
	.scl_gpio	= -1,
	.sda_gpio	= -1,
};

static struct tegra_i2c_platform_data pda_i2c5_platform_data = {
	.bus_clk_rate	= 400000,
	.scl_gpio	=  TEGRA_GPIO_PJ7,
	.sda_gpio	= TEGRA_GPIO_PP0,
	.needs_cl_dvfs_clock = true,
};

static __maybe_unused struct tegra_i2c_platform_data pda_i2c6_platform_data = {
	.bus_clk_rate	= 400000,
	.scl_gpio	= TEGRA_GPIO_I2C5_SCL,
	.sda_gpio	= TEGRA_GPIO_I2C5_SDA,
};


#if defined(CONFIG_SWITCH_MAX14688)
/* For initial working, IRQ set falling and rising */
#define MAX14688_IRQ_TRIGGER        IRQF_TRIGGER_FALLING
#define MAX14688_IRQ_GPIO           0
#define MAX14688_I2C_BUS            5

#define MAX14688_ADVANCED_JACK_DET  1

/* ADC operation should be put into here */
#if MAX14688_ADVANCED_JACK_DET
static int platform_read_mic_impedence (struct device *dev)
{
	int acc_read_value = 0, i = 0;
	int count = 3;

	for (i = 0; i < count; i++) {
		acc_read_value = max77660_lge_get_adc(MAX77660_ADC_CH_ADC1);
	}

	pr_debug("%s: acc_read_value - %d\n", __func__, acc_read_value);

	return acc_read_value;
}
#else
#define platform_read_mic_impedence NULL
#endif

#undef  Z
#define Z(_minZ, _maxZ) \
{ .min_Z = _minZ, .max_Z = _maxZ }

#define JACK_MATCH(_name, _mic, _left, _has_btn, _mode0, _mode1, _switch_state, _evt_type, _evt_code1, _evt_code2) \
{\
	.name           = _name,\
	.mic            = _mic,\
	.left           = _left,\
	.switch_state   = _switch_state,\
	.evt_type       = _evt_type,\
	.evt_code1      = _evt_code1,\
	.evt_code2      = _evt_code2,\
	.has_button     = _has_btn,\
	.mode0          = MAX14688_MODE_##_mode0,\
	.mode1          = MAX14688_MODE_##_mode1,\
}

#define BUTTON_MATCH(_name, _mic, _left, _evt_type, _evt_code) \
{\
	.name     = _name,\
	.mic      = _mic,\
	.left     = _left,\
	.evt_type = _evt_type,\
	.evt_code = _evt_code,\
}

#undef  DONTCARE
#define DONTCARE  Z(INT_MAX, INT_MIN)
#undef  ANY
#define ANY       Z(      1, INT_MAX)
#undef  GROUNDED
#define GROUNDED  Z(INT_MIN,       0)
#define NONE 0

enum {
	NO_DEVICE   			= 0,
	LGE_HEADSET 			= (1 << 0),
	LGE_HEADSET_NO_MIC 		= (1 << 1),
	LGE_ADVANCED_HEADSET 		= (1 << 5),
	LGE_ADVANCED_HEADSET_NO_MIC	= (1 << 6),
	LGE_AUX_ACCESSORY 		= (1 << 7),
};

/* mic impedence should be fixed after applying ADC operation */
static struct max14688_jack_match max14688_jack_matches[] = {
	//         name      mic                left         	has     mode0  mode1  switch                       event  event
	//                   impedence          impedence    	button                state                        type   code
//                                                                                                                                       
//                                                                                                                                                        
	JACK_MATCH("3P",     Z(0, 1000),  		DONTCARE,   	false,  LOW,   LOW,   LGE_HEADSET_NO_MIC,          EV_SW, SW_HEADPHONE_INSERT, NONE),
	JACK_MATCH("4P",     Z(1000, 2600), 	DONTCARE,   	true,   HIGH,  LOW,   LGE_HEADSET,                 EV_SW, SW_HEADPHONE_INSERT, SW_MICROPHONE_INSERT),
#if MAX14688_ADVANCED_JACK_DET
	JACK_MATCH("3P/ADV", Z(0, 1750),  		Z(100, 199),   	false,  LOW,   LOW,   LGE_ADVANCED_HEADSET_NO_MIC, EV_SW, SW_ADVANCED_HEADPHONE_INSERT, NONE),
	JACK_MATCH("4P/ADV", Z(1750, 2600),  		Z(200, 299),   	true,   HIGH,  LOW,   LGE_ADVANCED_HEADSET,        EV_SW, SW_ADVANCED_HEADPHONE_INSERT, SW_MICROPHONE_INSERT),
	JACK_MATCH("ACC/AUX",Z(1750, 2600),             Z(300, 399),   	false,  LOW,  HIGH,   LGE_AUX_ACCESSORY,           EV_SW, SW_AUX_ACCESSORY_INSERT, NONE),
#endif
};

static struct max14688_button_match max14688_button_matches[] = {
	//           name      mic          left         event   event
	//                     impedence    impedence    type    code
	BUTTON_MATCH("MEDIA",  Z(0, 150), DONTCARE,    EV_KEY, KEY_MEDIA),
	BUTTON_MATCH("VOLUP",  Z(150, 400), DONTCARE,    EV_KEY, KEY_VOLUMEUP),
	BUTTON_MATCH("VOLDN",  Z(400, 600), DONTCARE,    EV_KEY, KEY_VOLUMEDOWN),
};
#endif /* CONFIG_SWITCH_MAX14688 */

#if defined(CONFIG_SWITCH_MAX14688)
static struct max14688_platform_data lge_hs14688_pdata = {
	.switch_name = "h2w",
	.gpio_detect	= EAR_SENSE_N_35D,
	.gpio_int	= TEGRA_GPIO_PL0, /* EAR_KEY_INT_35D, */

	.irq_trigger           = MAX14688_IRQ_TRIGGER,
	.jack_matches          = max14688_jack_matches,
	.num_of_jack_matches   = ARRAY_SIZE(max14688_jack_matches),
	.button_matches        = max14688_button_matches,
	.num_of_button_matches = ARRAY_SIZE(max14688_button_matches),
	.detect_jack           = NULL, /* set NULL to driver default */
	.read_mic_impedence    = platform_read_mic_impedence, /* set NULL to driver default */
	.read_left_impedence   = NULL, /* set NULL to driver default */
	.report_jack           = NULL, /* set NULL to driver default */
	.report_button         = NULL, /* set NULL to driver default */
};

static struct i2c_board_info __initdata hds_max14688_dev_info = {
	I2C_BOARD_INFO(MAX14688_NAME, MAX14688_I2C_ADDR),
	.flags		= I2C_CLIENT_WAKE, /* max14688 could wake up system */
	.platform_data 	= &lge_hs14688_pdata,
};
#endif

#if defined(CONFIG_MACH_PDA)
static struct i2c_board_info __initdata tps_device_info = {
		I2C_BOARD_INFO("tps65132", 0x3E),
};
#endif
static void pda_i2c_init(void)
{
	tegra14_i2c_device1.dev.platform_data = &pda_i2c1_platform_data;
	tegra14_i2c_device2.dev.platform_data = &pda_i2c2_platform_data;
	tegra14_i2c_device3.dev.platform_data = &pda_i2c3_platform_data;
	tegra14_i2c_device4.dev.platform_data = &pda_i2c4_platform_data;
	tegra14_i2c_device5.dev.platform_data = &pda_i2c5_platform_data;
	tegra14_i2c_device6.dev.platform_data = &pda_i2c6_platform_data;

	if (((board_info.board_id == BOARD_E1680) &&
	     (board_info.fab >= BOARD_FAB_A04)) ||
	     (board_info.board_id == BOARD_E1683))
		pda_i2c5_platform_data.bit_banging_xfer_after_shutdown = true;

	platform_device_register(&tegra14_i2c_device6);
	platform_device_register(&tegra14_i2c_device5);
	platform_device_register(&tegra14_i2c_device4);
	platform_device_register(&tegra14_i2c_device3);
	platform_device_register(&tegra14_i2c_device2);
	platform_device_register(&tegra14_i2c_device1);

	//if (board_info.board_id == BOARD_E1690)
	//	nfc_pdata.en_gpio = TEGRA_GPIO_PM5;

	pda_i2c_bus3_board_info[0].irq = gpio_to_irq(PDA_NFC_IRQ);
	i2c_register_board_info(5, pda_i2c_bus3_board_info, 1);

#ifndef CONFIG_MACH_PDA
	i2c_register_board_info(0, &pda_eeprom_mac_add, 1);
#else
/*                                                                                            
                         
                         
                          
                                                                                             */
	if(lge_get_board_revno() >= HW_REV_PDA_C) {
		i2c_register_board_info(5, &tps_device_info, 1);
		i2c_register_board_info(5, &hds_max14688_dev_info, 1);
	}
#endif
}

#ifndef CONFIG_MACH_PDA
static __initdata struct tegra_clk_init_table raydium_touch_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "vi_sensor",	"pll_p",	41000000,	true},
	{ "csus",	NULL,		00000000,	true},
	{ NULL,		NULL,		0,		0},
};
#endif

struct rm_spi_ts_platform_data rm31080ts_pda_data = {
	.gpio_reset = 0,
	.config = 0,
	.platform_id = RM_PLATFORM_P005,
	/* Clocks are defined in tegraXX_clocks.c:  CLK_DUPLICATE(...)» */
	.name_of_clock = "touch_clk",
	.name_of_clock_con = "e1680_ts_clk_con",
};

static struct tegra_spi_device_controller_data dev_cdata = {
	.rx_clk_tap_delay = 0,
	.tx_clk_tap_delay = 0,
};

struct spi_board_info rm31080a_pda_spi_board[1] = {
	{
		.modalias = "rm_ts_spidev",
		.bus_num = 2,
		.chip_select = 0,
		.max_speed_hz = 9 * 1000 * 1000,
		.mode = SPI_MODE_0,
		.controller_data = &dev_cdata,
		.platform_data = &rm31080ts_pda_data,
	},
};

int synaptics_touch_enable(const void *pm_data)
{
	if (board_info.board_id == BOARD_E1670)
		gpio_free(TEGRA_GPIO_PL3);

	return 0;
}

int synaptics_touch_disable(const void *pm_data)
{
	int ret = 0;

	if (board_info.board_id == BOARD_E1670) {
		ret = gpio_request(TEGRA_GPIO_PL3, "touch_spi_cs");
		if (ret < 0) {
			pr_err("%s: gpio_request failed\n", __func__);
			return ret;
		}

		ret = gpio_direction_output(TEGRA_GPIO_PL3, 0);
		if (ret < 0) {
			pr_err("%s: set gpio to output failed\n", __func__);
			gpio_free(TEGRA_GPIO_PL3);
		}
	}

	return ret;
}

#ifndef CONFIG_MACH_PDA
static struct synaptics_gpio_data synaptics_gpio_pda_data = {
	.attn_gpio = SYNAPTICS_ATTN_GPIO,
	.attn_polarity = RMI_ATTN_ACTIVE_LOW,
	.reset_gpio = SYNAPTICS_RESET_GPIO,
};

static struct rmi_device_platform_data synaptics_pda_platformdata = {
	.sensor_name   = "TM9999",
	.attn_gpio     = SYNAPTICS_ATTN_GPIO,
	.attn_polarity = RMI_ATTN_ACTIVE_LOW,
	.gpio_data     = &synaptics_gpio_pda_data,
	.gpio_config   = synaptics_touchpad_gpio_setup,
	.spi_data = {
		.block_delay_us = 15,
		.read_delay_us = 15,
		.write_delay_us = 2,
	},
	.power_management = {
		.nosleep = RMI_F01_NOSLEEP_OFF,
	},
	.f19_button_map = &synaptics_button_map,
	.f54_direct_touch_report_size = 944,

	.pre_resume = synaptics_touch_enable,
	.post_suspend = synaptics_touch_disable,
};

static struct spi_board_info synaptics_9999_spi_board_pda[] = {
	{
		.modalias = "rmi_spi",
		.bus_num = 2,
		.chip_select = 0,
		.max_speed_hz = 8*1000*1000,
		.mode = SPI_MODE_3,
		.platform_data = &synaptics_pda_platformdata,
	},
};
#endif /* !CONFIG_MACH_PDA */

/* mms136 */
/* touch screen device */
static struct melfas_tsi_platform_data melfas_ts_pdata = {
	.x_max					= TS_X_MAX,
	.y_max					= TS_Y_MAX,
	.i2c_int_gpio				= TS_GPIO_IRQ,
	.gpio_scl				= TS_I2C_SCL,
	.gpio_sda				= TS_I2C_SDA,
	.touch_id_gpio  			= TS_MAKER_ID,
	.num_of_finger				= 10,
	.num_of_button				= 4,
	.button[0]				= KEY_MENU,
	.button[1]				= KEY_HOMEPAGE,
	.button[2]				= KEY_BACK,
	.button[3]				= KEY_SEARCH,
};

static struct i2c_board_info melfas_touch_panel_i2c_bdinfo = {
	I2C_BOARD_INFO(MELFAS_DRIVER_NAME, MELFAS_TS_I2C_ADDR),
	.platform_data = &melfas_ts_pdata,
};

void touch_init_mms136(void)
{
	pr_info("##@%s\n", __func__);

#if defined(CONFIG_MACH_PDA)
/*                                                                                            
                         
                         
                          
                                                                                             */
	if(lge_get_board_revno() >= HW_REV_PDA_C){
		melfas_ts_pdata.touch_en_gpio = MAX77660_GPIO_BASE + MAX77660_GPIO4;
	}
#endif

	melfas_touch_panel_i2c_bdinfo.irq = gpio_to_irq(TS_GPIO_IRQ);
	i2c_register_board_info(1, &melfas_touch_panel_i2c_bdinfo, 1);
}

//                                                                   
/* ATMEL_S336 */
#if defined (CONFIG_TOUCHSCREEN_ATMEL_S336)
static struct mxt_platform_data atmel_ts_pdata = {
	.numtouch				= 8,
	.max_x					= TS_X_MAX,
	.max_y					= TS_Y_MAX,
	.lcd_x					= TS_X_MAX,
	.lcd_y					= TS_Y_MAX,
//	.i2c_pull_up				= TS_GPIO_IRQ,
//	.irqflags				= 2,
	.gpio_reset				= TEGRA_GPIO_PN2,
	.gpio_int				= TEGRA_GPIO_PN1,
	.panel_check				= 0,//01,
//	.panel_on				= TS_I2C_SCL,
	.fw_name				= "atmel/S0A47P1_2_07.fw",
//	.extra_fw_name				= "atmel/S0A47P1_2_02.fw",
	.ref_reg_weight_val			= 16,
	.auto_fw_update				= 1,
	.knock_on_type				= 1,
	.t15_num_keys				= 0,
	.t15_keystate				= {0},
//	.t15_keystate[1]				= 2,
//	.t15_keystate[2]				= 4,
//	.t15_keystate[3]				= 8,	
	.t15_keymap					= {0},
//	.t15_keymap[1]				= KEY_HOMEPAGE,
//	.t15_keymap[2]				= KEY_MENU,
//	.t15_keymap[3]				= KEY_WIMAX,
	.global_access_pixel		= 5,
	.ghost_detection_enable		= 1,
	.ghost_detection_value 		= {10, 10, 100, 25, 250, 10, 0},
	.butt_check_enable			= 0,
	.diff_scaling				= 16,
	.error_check_count 			= {1, 3, 3, 5, 10},
	.time_reset_threshold		= 5,
	.time_reset_error_node_chk	= 5,
};

static struct mxt_platform_data atmel_ts_pdata_factory = {
	.numtouch				= 8,
	.max_x					= TS_X_MAX,
	.max_y					= TS_Y_MAX,
	.lcd_x					= TS_X_MAX,
	.lcd_y					= TS_Y_MAX,
	.gpio_reset				= TEGRA_GPIO_PN2,
	.gpio_int				= TEGRA_GPIO_PN1,
	.panel_check				= 0,//01,
	.fw_name				= "atmel/S0A47P1_2_07.fw",
	.ref_reg_weight_val			= 16,
	.auto_fw_update				= 1,
	.knock_on_type				= 1,
	.t15_num_keys				= 0,
	.t15_keystate				= {0},
	.t15_keymap					= {0},
	.global_access_pixel		= 5,
	.ghost_detection_enable		= 1,
	.ghost_detection_value 		= {10, 3, 100, 15, 250, 10, 0},
	.butt_check_enable			= 0,
	.diff_scaling				= 16,
	.error_check_count 			= {1, 3, 3, 5, 10},
	.time_reset_threshold		= 5,
	.time_reset_error_node_chk	= 5,
};

static struct i2c_board_info atmel_touch_panel_i2c_bdinfo = {
	I2C_BOARD_INFO("atmel_s336", ATMEL_TS_ADDR),
	.platform_data = &atmel_ts_pdata,
};

static struct i2c_board_info atmel_touch_panel_i2c_bdinfo_factory = {
	I2C_BOARD_INFO("atmel_s336", ATMEL_TS_ADDR),
	.platform_data = &atmel_ts_pdata_factory,
};


void touch_init_atmel_336s(void)
{
	int rc;

	pr_info("##@%s\n", __func__);

	/* gpio init : maker_id */
	rc = gpio_request(TS_MAKER_ID, "TOUCH_PANEL_MAKERID");
	if (unlikely(rc < 0))
		pr_err("%s not able to get gpio\n", __func__);
	gpio_direction_input(TS_MAKER_ID);

	if(lge_get_boot_mode() >= LGE_BOOT_MODE_QEM_NO_USB)
	{
		pr_info("[Touch] In touch_init_atmel_336s ## lge_get_boot_mode = %d, Here is factory!!\n",lge_get_boot_mode());
		atmel_touch_panel_i2c_bdinfo_factory.irq = gpio_to_irq(TEGRA_GPIO_PN1);
		i2c_register_board_info(1, &atmel_touch_panel_i2c_bdinfo_factory, 1);
	}
	else
	{
		atmel_touch_panel_i2c_bdinfo.irq = gpio_to_irq(TEGRA_GPIO_PN1);
		i2c_register_board_info(1, &atmel_touch_panel_i2c_bdinfo, 1);
	}
}
#endif // CONFIG_TOUCHSCREEN_ATMEL_S336
//                                               
static int __init pda_touch_init(void)
{
//                                                                   
	int touch_ic_id = -1;
    int rc = 0;

    rc = gpio_request(TEGRA_GPIO_PH5, "TOUCH_IC");

    if (unlikely(rc < 0))
            pr_err("%s not able to get gpio\n", __func__);
    
    gpio_direction_input(TEGRA_GPIO_PH5);
    touch_ic_id = gpio_get_value(TEGRA_GPIO_PH5);

	pr_info("##@ touch_ic_id = [%d]\n", touch_ic_id);
	if (lge_get_board_revno() <= HW_REV_D625_A_1) {
		pr_info("##@XXXXXX in pda_touch_init lge_get_board_revno = %d, HW rev is lower than HW_REV_D625_B !! I do not want to change device any more!!\n",lge_get_board_revno());
		if(touch_ic_id)
			touch_init_mms136();
		else
			touch_init_atmel_336s();
		}
	else
	{
		pr_info("##@ in pda_touch_init, HW rev is higher than HW_REV_D625_B !! I do not want to change device any more!!\n");
		touch_init_atmel_336s();
	}

//                                               
#ifndef CONFIG_MACH_PDA
	if (tegra_get_touch_vendor_id() == RAYDIUM_TOUCH) {
		pr_info("%s: initializing raydium\n", __func__);
		tegra_clk_init_from_table(raydium_touch_clk_init_table);
		rm31080a_pda_spi_board[0].irq =
			gpio_to_irq(TOUCH_GPIO_IRQ_RAYDIUM_SPI);
		touch_init_raydium(TOUCH_GPIO_IRQ_RAYDIUM_SPI,
					TOUCH_GPIO_RST_RAYDIUM_SPI,
					&rm31080ts_pda_data,
					&rm31080a_pda_spi_board[0],
					ARRAY_SIZE(rm31080a_pda_spi_board));
	} else {
		pr_info("%s: initializing synaptics\n", __func__);
		touch_init_synaptics(synaptics_9999_spi_board_pda,
				ARRAY_SIZE(synaptics_9999_spi_board_pda));
	}
#endif
	return 0;
}

#if defined(CONFIG_TEGRA_BASEBAND)
/* main (integrated) modem init */
static void pda_tegra_bb_init(void)
{
	int modem_id = tegra_get_modem_id();

	if (modem_id == TEGRA_BB_INTEGRATED_DISABLED)
		return;

	pr_info("%s: registering tegra bb\n", __func__);
	pda_tegra_bb_data.bb_irq = INT_BB2AP_INT0;
	pda_tegra_bb_data.mem_req_soon = INT_BB2AP_MEM_REQ_SOON_INT;

	if ((tegra_revision == TEGRA_REVISION_A01) &&
		(!tegra_fuse_readl(FUSE_SPARE_BIT_12_0)))
		pda_tegra_bb_data.pll_voltage = 1100000;
	else
		pda_tegra_bb_data.pll_voltage = 900000;

	platform_device_register(&tegra_bbc_proxy_device);
	platform_device_register(&pda_tegra_bb_device);
}
#endif

static void __init pda_dtv_init(void)
{
	platform_device_register(&tegra_dtv_device);
}

#if defined(CONFIG_LGE_BOOTLOADER_LOG_BUFFER)
static void __init lge_bootlog_init(void);
#endif

static void __init tegra_pda_early_init(void)
{
	pda_sysedp_init();
	tegra_clk_init_from_table(pda_clk_init_table);
	tegra_clk_verify_parents();
}

static void __init sysedp_psydepl_init(void)
{
	switch (board_info.board_id) {
	case BOARD_E1680:
	case BOARD_E1681:
	case BOARD_E1683:
		pda_sysedp_psydepl_init();
		break;
	case BOARD_E1670:
		atlantis_sysedp_psydepl_init();
		break;
	}
}

/*
                            
                                                   
                                                       
  
                                     
                       
 */
#if defined(CONFIG_LGE_BOOTLOADER_LOG_BUFFER)
struct log_buffer {
        uint32_t    start;
        uint32_t    size;
        uint32_t    tot_size;
        uint8_t     data[0];
};

static uint32_t boot_logbuf_phy = 0;
struct log_buffer *boot_logbuf_virt = NULL;

static void __init lge_bootlog_init(void)
{
        char *buffer;
        char *token;
        char *ct = "\n";

	/*
	 * This address should be same with declared address in bootloader
	 */
	boot_logbuf_phy = 0xBB800000;	// BBC_IPC

        boot_logbuf_virt =
                (struct log_buffer *) ioremap(boot_logbuf_phy, 129 * 1024);
        if (boot_logbuf_virt == NULL) {
                printk(KERN_INFO "%s: failed to map memory\n", __func__);
                return;
        }

        printk(KERN_INFO "=================================================\n");
        printk(KERN_INFO "%s: start %d\n", __func__, boot_logbuf_virt->start);
        printk(KERN_INFO "%s: size %d\n", __func__, boot_logbuf_virt->size);
        printk(KERN_INFO "Below logs are got from bootloader \n");
        printk(KERN_INFO "=================================================\n");
        printk(KERN_INFO "\n");

        buffer = (char *) boot_logbuf_virt->data;
        buffer[128 * 1024 - sizeof(struct log_buffer)] = '\0';

        while (1) {
                token = strsep(&buffer, ct);
                if (!token) {
                        printk(KERN_INFO "%s: token %p\n", __func__, token);
                        break;
                }
                printk(KERN_INFO"%s\n", token);
        }

        printk(KERN_INFO "=================================================\n");

        iounmap(boot_logbuf_virt);

        return;
}
#endif

static void __init tegra_pda_late_init(void)
{
	platform_device_register(&tegra_pinmux_device);
	pda_pinmux_init();
#if defined(CONFIG_TEGRA_BASEBAND)
	/* make bb early in list of devices, this helps it to be the
	 * last to suspend.
	 */
	pda_tegra_bb_init();
#endif
	pda_i2c_init();
	pda_spi_init();
	pda_uart_init();
	pda_usb_init();
	tegra_soc_device_init("pda");
	pda_keys_init();
	pda_regulator_init();
	pda_dtv_init();
	pda_suspend_init();
	pda_touch_init();
	pda_sdhci_init();
	platform_add_devices(pda_devices, ARRAY_SIZE(pda_devices));
	tegra_ram_console_debug_init();
#ifdef CONFIG_TEGRA_FIQ_DEBUGGER
	tegra_serial_debug_init(TEGRA_UARTA_BASE, INT_WDT_AVP, NULL, -1, -1);
#endif
	pda_emc_init();
	pda_edp_init();
	isomgr_init();
	pda_panel_init();
	pda_sensors_init();
#ifndef CONFIG_MACH_PDA
	pda_modem_init();
#endif
	tegra_register_fuse();
	pda_soctherm_init();
#ifdef CONFIG_BT_BLUESLEEP
	pda_setup_bluesleep();
	pda_setup_bt_rfkill();
#elif defined CONFIG_BLUEDROID_PM
	register_bluetooth_pm();
#endif
	pda_audio_init();
#ifndef CONFIG_MACH_PDA
	pda_pmon_init();
#endif
	pda_sysedp_core_init();
	sysedp_psydepl_init();
}

static void __init tegra_pda_dt_init(void)
{
	tegra_get_board_info(&board_info);

#if defined(CONFIG_LGE_BOOTLOADER_LOG_BUFFER)
	lge_bootlog_init();
#endif

	tegra_pda_early_init();

	of_platform_populate(NULL,
		of_default_bus_match_table, NULL, &platform_bus);

	tegra_pda_late_init();
}

static void __init tegra_pda_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	/* for PANEL_5_SHARP_1080p: 1920*1080*4*2 = 16588800 bytes */
#if defined(CONFIG_MACH_PDA)
	tegra_reserve(0, SZ_4M, 0);
#else
	tegra_reserve(0, SZ_16M, SZ_8M);
#endif
#else
	tegra_reserve(SZ_128M, SZ_16M, SZ_16M);
#endif
}

static const char * const pda_dt_board_compat[] = {
	"nvidia,pda",
	NULL
};

DT_MACHINE_START(PDA, "Pda")
	.atag_offset	= 0x100,
	.soc			= &tegra_soc_desc,
	.map_io			= tegra_map_common_io,
	.reserve		= tegra_pda_reserve,
	.init_early		= tegra14x_init_early,
	.init_irq		= tegra_dt_init_irq,
	.handle_irq		= gic_handle_irq,
	.timer			= &tegra_timer,
	.init_machine	= tegra_pda_dt_init,
	.restart		= tegra_assert_system_reset,
	.dt_compat		= pda_dt_board_compat,
MACHINE_END
