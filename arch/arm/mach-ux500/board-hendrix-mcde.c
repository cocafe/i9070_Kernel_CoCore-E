/*
 * Copyright (C) Samsung 2012
 *
 * License Terms: GNU General Public License v2
 * Authors: Robert Teather <robert.teather@samsung.com> for Samsung Electronics
 *
 * Board specific file for display initialization
 *
 */

#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/dispdev.h>
#include <linux/compdev.h>
#include <linux/clk.h>
#include <mach/devices.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <mach/board-sec-ux500.h>
#include <video/mcde_display.h>
#include <video/mcde_display-sec-dsi.h>
#include <video/mcde_fb.h>
#include <video/mcde_dss.h>
#include <plat/gpio-nomadik.h>
#include <plat/pincfg.h>
#include "pins-db8500.h"
#include "pins.h"
#include <mach/db8500-regs.h>
#include <linux/mfd/dbx500-prcmu.h>

#if defined (CONFIG_SAMSUNG_USE_GETLOG)
#include <mach/sec_getlog.h>
#endif

#define M1316A0_DRIVER_NAME		"mcde_disp_m1316a0_dsi"

#define LCD_PANEL_TYPE_M1316A0	13

#define DSI_PLL_FREQ_HZ_M1316A0		698880000
/* Based on PLL DDR Freq at 798,72 MHz */
#define HDMI_FREQ_HZ			33280000
#define TV_FREQ_HZ			38400000
#define DSI_HS_FREQ_HZ_M1316A0		349440000
#define DSI_LP_FREQ_HZ_M1316A0		9600000
#define DSI_LP_FREQ_HZ			19200000


enum {
	PRIMARY_DISPLAY_ID,
	MCDE_NR_OF_DISPLAYS
};

static int display_initialized_during_boot = (int)false;
static struct fb_info *primary_fbi;

static void hendrix_lcd_pwr_setup(struct device *dev)
{
	int ret;

	ret = gpio_request(LCD_PWR_EN_HENDRIX_BRINGUP, "LCD PWR EN");
	if (ret < 0)
		printk(KERN_ERR "Failed to get LCD PWR EN gpio (%d)\n",ret);
	}

static void hendrix_lcd_pwr_onoff(bool on)
{
	if (on)
		gpio_direction_output(LCD_PWR_EN_HENDRIX_BRINGUP, 1);
	else
		gpio_direction_output(LCD_PWR_EN_HENDRIX_BRINGUP, 0);
}

extern void hendrix_backlight_on_off(bool on);

struct sec_dsi_platform_data hendrix_dsi_pri_display_info = {
	.reset_gpio = LCD_RESET_N_HENDRIX_BRINGUP,
	.bl_ctrl = false,
	.lcd_pwr_setup = hendrix_lcd_pwr_setup,
	.lcd_pwr_onoff = hendrix_lcd_pwr_onoff,
	.min_ddr_opp = 50,
	.bl_on_off = hendrix_backlight_on_off,
};


static int __init startup_graphics_setup(char *str)
{
	if (get_option(&str, &display_initialized_during_boot) != 1)
		display_initialized_during_boot = false;

	if (display_initialized_during_boot)
		pr_info("Startup graphics support\n");
	else
		pr_info("No startup graphics supported\n");

	return 1;
}
__setup("startup_graphics=", startup_graphics_setup);

unsigned int lcd_type;
static int __init lcdtype_setup(char *str)
{
	get_option(&str, &lcd_type);

	return 1;
}
__setup("lcdtype=", lcdtype_setup);

static int __init lcdid_setup(char *str)
{
	int lcd_id[4];

	get_options(str, 4, lcd_id);
	if (lcd_id[0] == 3) {
		hendrix_dsi_pri_display_info.lcdId[0] = lcd_id[1];
		hendrix_dsi_pri_display_info.lcdId[1] = lcd_id[2];
		hendrix_dsi_pri_display_info.lcdId[2] = lcd_id[3];
	}
	return 1;
}
__setup("lcd.id=", lcdid_setup);

static int __init lcdmtpdata_setup(char *str)
{
	int i;
	int mtpData[SEC_DSI_MTP_DATA_LEN+1];

	get_options(str, SEC_DSI_MTP_DATA_LEN+1, mtpData);
	if (mtpData[0] == SEC_DSI_MTP_DATA_LEN) {
		hendrix_dsi_pri_display_info.mtpAvail = true;
		for (i = 0; i < SEC_DSI_MTP_DATA_LEN; i++)
			hendrix_dsi_pri_display_info.mtpData[i] = mtpData[i+1];
	}
	return 1;
}
__setup("lcd.mtp=", lcdmtpdata_setup);


static struct mcde_port m1316a0_port0 = {
	.link = 0,
	.type = MCDE_PORTTYPE_DSI,
	.mode = MCDE_PORTMODE_VID,
	.pixel_format = MCDE_PORTPIXFMT_DSI_24BPP,
	.sync_src = MCDE_SYNCSRC_OFF,
	.frame_trig = MCDE_TRIG_HW,
	.update_auto_trig = true,
	.phy.dsi = {
		.vid_mode = BURST_MODE_WITH_SYNC_EVENT,
		.vid_wakeup_time = 88,
		.num_data_lanes = 2,
		.host_eot_gen = true,
		.clk_cont = true,
		.hs_freq = DSI_HS_FREQ_HZ_M1316A0,
		.lp_freq = DSI_LP_FREQ_HZ_M1316A0,
	},
};

static struct mcde_display_device m1316a0_display0 = {
	.name = M1316A0_DRIVER_NAME,
	.id = PRIMARY_DISPLAY_ID,
	.port = &m1316a0_port0,
	.chnl_id = MCDE_CHNL_A,
	.fifo = MCDE_FIFO_A,
	.orientation = MCDE_DISPLAY_ROT_0,
	.default_pixel_format = MCDE_OVLYPIXFMT_RGBA8888,
	.dev = {
		.platform_data = &hendrix_dsi_pri_display_info,
	},
};


static int display_postregistered_callback(struct notifier_block *nb,
	unsigned long event, void *dev)
{
	struct mcde_display_device *ddev = dev;
	u16 width, height;
	u16 virtual_height;
	struct fb_info *fbi;
#if defined(CONFIG_COMPDEV)
	struct mcde_fb *mfb;
#endif

	if (event != MCDE_DSS_EVENT_DISPLAY_REGISTERED)
		return 0;

	if (ddev->id < PRIMARY_DISPLAY_ID || ddev->id >= MCDE_NR_OF_DISPLAYS)
		return 0;

	mcde_dss_get_native_resolution(ddev, &width, &height);
	virtual_height = height * 3;


	/* Create frame buffer */
	fbi = mcde_fb_create(ddev, width, height, width, virtual_height,
				ddev->default_pixel_format, FB_ROTATE_UR);

	if (IS_ERR(fbi)) {
		dev_warn(&ddev->dev,
			"Failed to create fb for display %s\n", ddev->name);
		goto display_postregistered_callback_err;
	} else {
		dev_info(&ddev->dev, "Framebuffer created (%s)\n", ddev->name);

			if (ddev->id == PRIMARY_DISPLAY_ID)
			{
				primary_fbi = fbi;
#if defined (CONFIG_SAMSUNG_USE_GETLOG)
				sec_getlog_supply_fbinfo(primary_fbi);
#endif
			}
		}

#ifdef CONFIG_COMPDEV
	mfb = to_mcde_fb(fbi);
	/* Create a compdev overlay for this display */
	if (compdev_create(ddev, mfb->ovlys[0], true, NULL) < 0) {
		dev_warn(&ddev->dev,
			"Failed to create compdev for display %s\n",
					ddev->name);
		goto display_postregistered_callback_err;
	} else {
		dev_info(&ddev->dev, "compdev created for (%s)\n",
					ddev->name);
	}
#endif

	return 0;

display_postregistered_callback_err:
	return -1;
}

static struct notifier_block display_nb = {
	.notifier_call = display_postregistered_callback,
};

int __init init_hendrix_display_devices(void)
{
	int ret;
	struct clk *clk_dsi_pll;
	struct clk *clk_hdmi;
	struct clk *clk_tv;
	struct mcde_platform_data *pdata = ux500_mcde_device.dev.platform_data;
	unsigned long dsi_pll_freq;


	ret = mcde_dss_register_notifier(&display_nb);
	if (ret)
		printk(KERN_ERR "Failed to register dss notifier\n");

	if (display_initialized_during_boot) {
		m1316a0_display0.power_mode = MCDE_DISPLAY_PM_STANDBY;
	} else {
		m1316a0_display0.power_mode = MCDE_DISPLAY_PM_OFF;
	}

	/* we need to initialize all the clocks used */
	/*
	 * The TV CLK is used as parent for the
	 * DSI LP clock.
	 */
	clk_tv = clk_get(&ux500_mcde_device.dev, "tv");
	if (TV_FREQ_HZ != clk_round_rate(clk_tv, TV_FREQ_HZ))
		printk(KERN_ERR "%s: TV_CLK freq differs %ld\n", __func__,
				clk_round_rate(clk_tv, TV_FREQ_HZ));
	clk_set_rate(clk_tv, TV_FREQ_HZ);
	clk_put(clk_tv);

	/*
	 * The HDMI CLK is used as parent for the
	 * DSI HS clock.
	 */
	clk_hdmi = clk_get(&ux500_mcde_device.dev, "hdmi");
	if (HDMI_FREQ_HZ != clk_round_rate(clk_hdmi, HDMI_FREQ_HZ))
		printk(KERN_ERR "%s: HDMI freq differs %ld\n", __func__,
				clk_round_rate(clk_hdmi, HDMI_FREQ_HZ));
	clk_set_rate(clk_hdmi, HDMI_FREQ_HZ);
	clk_put(clk_hdmi);

	/*
	 * The DSI PLL CLK is used as DSI PLL for direct freq for
	 * link 2. Link 0/1 is then divided with 1/2/4 from this freq.
	 */

//	if (lcd_type == LCD_PANEL_TYPE_M1316A0)
		dsi_pll_freq = DSI_PLL_FREQ_HZ_M1316A0;
//	else {
//		printk(KERN_ERR "Display device type %d not recognised\n", lcd_type);
//		ret = -ENODEV;
//		goto error;
//	}

	clk_dsi_pll = clk_get(&ux500_mcde_device.dev, "dsipll");
	if (dsi_pll_freq != clk_round_rate(clk_dsi_pll,	dsi_pll_freq))
		printk(KERN_ERR "%s: DSI_PLL freq differs %ld\n", __func__,
			clk_round_rate(clk_dsi_pll, dsi_pll_freq));
	clk_set_rate(clk_dsi_pll, dsi_pll_freq);
	clk_put(clk_dsi_pll);

	/* MCDE pixelfetchwtrmrk levels per overlay */
	{
		#define BITS_PER_WORD (4 * 64)
#if 1 /* 16 bit overlay */
		u32 fifo = (1024*8 - 8 * BITS_PER_WORD) / 3;
		fifo &= ~(BITS_PER_WORD - 1);
		pdata->pixelfetchwtrmrk[0] = fifo * 2 / 32;	/* LCD 32bpp */
		pdata->pixelfetchwtrmrk[1] = fifo * 1 / 16;	/* LCD 16bpp */
#else /* 24 bit overlay */
		u32 fifo = (1024*8 - 8 * BITS_PER_WORD) / 7;
		fifo &= ~(BITS_PER_WORD - 1);
		pdata->pixelfetchwtrmrk[0] = fifo * 4 / 32;	/* LCD 32bpp */
		pdata->pixelfetchwtrmrk[1] = fifo * 3 / 24;	/* LCD 24bpp */
#endif
	}

//	if (lcd_type == LCD_PANEL_TYPE_M1316A0)
		ret = mcde_display_device_register(&m1316a0_display0);

	if (ret)
		printk(KERN_ERR "Failed to register display device\n");

error:
	return ret;
}


struct fb_info *get_primary_display_fb_info(void)
{
	return primary_fbi;
}

module_init(init_hendrix_display_devices);

