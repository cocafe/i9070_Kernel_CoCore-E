/*
 * S6E63M0 AMOLED LCD panel driver.
 *
 * Author: InKi Dae  <inki.dae@samsung.com>
 * Modified by Anirban Sarkar <anirban.sarkar@samsung.com>
 * to add MCDE support
 *
 * Derived from drivers/video/omap/lcd-apollon.c
 *
 * Modified: Huang Ji (cocafe@xda-developers.com)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/wait.h>
#include <linux/fb.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/lcd.h>
#include <linux/backlight.h>
#include <linux/mutex.h>
#include <linux/kobject.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/moduleparam.h>

#include <linux/mfd/dbx500-prcmu.h>
#include <video/mcde_display.h>
#include <video/mcde_display-dpi.h>
#include <video/mcde_display_ssg_dpi.h>
#include "display-s6e63m0_gamma.h"

#include <plat/pincfg.h>
#include <plat/gpio-nomadik.h>
#include <mach/board-sec-u8500.h>
#include <mach/../../pins-db8500.h>
#include <mach/../../pins.h>

#define SMART_DIMMING
#define SPI_3WIRE_IF
#define DYNAMIC_ELVSS

#ifdef SMART_DIMMING
#include "smart_mtp_s6e63m0.h"
#endif

#define SLEEPMSEC		0x1000
#define ENDDEF			0x2000
#define DEFMASK			0xFF00
#define COMMAND_ONLY	0xFE
#define DATA_ONLY		0xFF

#define MIN_SUPP_BRIGHTNESS	0
#define MAX_SUPP_BRIGHTNESS	10
#define MAX_REQ_BRIGHTNESS	255
#define LCD_POWER_UP		1
#define LCD_POWER_DOWN		0
#define LDI_STATE_ON		1
#define LDI_STATE_OFF		0
/* Taken from the programmed value of the LCD clock in PRCMU */
#define PIX_CLK_FREQ		25000000
#define VMODE_XRES		480
#define VMODE_YRES		800
#define POWER_IS_ON(pwr)	((pwr) <= FB_BLANK_NORMAL)

#define DIM_BL 20
#define MIN_BL 30
#define MAX_BL 255
#define MAX_GAMMA_VALUE 25

#define DPI_DISP_TRACE	dev_dbg(&ddev->dev, "%s\n", __func__)

/* to be removed when display works */
/* #define dev_dbg	dev_info */

extern unsigned int system_rev;

struct s6e63m0 {
	struct device			*dev;
	struct spi_device		*spi;
	struct mutex			lock;
	struct mutex			lcd_lock;
	struct mutex			pwr_lock;
	unsigned int			power;
	unsigned int			current_gamma_mode;
	unsigned int			current_brightness;
	unsigned int			gamma_mode;
	unsigned int			gamma_table_count;
	unsigned int                    bl;
	unsigned int			ldi_state;
	unsigned int			acl_enable;
	unsigned int			cur_acl;
	unsigned char			panel_id;	
	unsigned int			auto_brightness;
	bool				justStarted;
	enum mcde_display_rotation	rotation;	
	struct mcde_display_device	*ddev;
	struct lcd_device		*ld;
	struct backlight_device		*bd;
	struct ssg_dpi_display_platform_data	*pd;
	struct spi_driver		spi_drv;
	unsigned int			elvss_ref;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend		earlysuspend;
#endif

#ifdef SMART_DIMMING
	struct SMART_DIM smart;
#endif

};

static struct s6e63m0 *plcd;

#ifdef CONFIG_HAS_EARLYSUSPEND
struct ux500_pins *dpi_pins;
#endif

#define ELVSS_MAX    0x28
const int ELVSS_OFFSET[] = {0x0, 0x07, 0x09, 0x0D};
#define SMART_MTP_PANEL_ID 0xa4

static int get_gamma_value_from_bl(int bl);
static int s6e63m0_set_brightness(struct backlight_device *bd);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void s6e63m0_mcde_panel_early_suspend(struct early_suspend
								*earlysuspend);
static void s6e63m0_mcde_panel_late_resume(struct early_suspend 
								*earlysuspend);
#endif
static int s6e63m0_power_on(struct s6e63m0 *lcd);
static int s6e63m0_power_off(struct s6e63m0 *lcd);
static int s6e63m0_ldi_disable(struct s6e63m0 *lcd);
static int s6e63m0_ldi_enable(struct s6e63m0 *lcd);
static int s6e63m0_set_power_mode(struct mcde_display_device *ddev,
	enum mcde_display_power_mode power_mode);
static int update_brightness(struct s6e63m0 *lcd, u8 force);

/* cocafe: S6E63M0 Color Control */
#define R_OFFSET			3
#define G_OFFSET			5
#define B_OFFSET			7

#define R_DEFVAL			0x18
#define G_DEFVAL			0x08
#define B_DEFVAL			0x24

static bool R_req = false;
static bool G_req = false;
static bool B_req = false;

static int R_val;
static int G_val;
static int B_val;

/* cocafe: S6E63M0 Gamma Tuner */
#define GAMMA_TABLE_START		0
#define GAMMA_TABLE_END			47

#define GAMMA_VAL_MIN			0
#define GAMMA_VAL_MAX			255

static bool gamma_table_req = false;

/* cocafe: S6E63M0 Illumination Tuner */
/* FIXME: illumination 0 and 3 have color issues! */
#define ILLUMINATION_MIN		1
#define ILLUMINATION_MAX		300

static bool illumination_req = false;
static unsigned int illumination_val = ILLUMINATION_MIN;

/* cocafe: Night Mode and Sunlight Mode */
static bool night_mode = false;
static bool sunlight_mode = false;

/* cocafe: S6E63M0 PRCMU LCDCLK */
/* 60+++ 	79872000	unsafe
 * 60++		66560000	unsafe
 * 60+		57051428	unsafe
 * 60		49920000
 * 50		39936000
 * 45		36305454
 * 40		33280000
 */
#include <linux/mfd/dbx500-prcmu.h>
#include <linux/mfd/db8500-prcmu.h>

#define LCDCLK_SET(clk)		prcmu_set_clock_rate(PRCMU_LCDCLK, (unsigned long) clk);

struct lcdclk_prop
{
	char *name;
	unsigned int clk;
};

static struct lcdclk_prop lcdclk_prop[] = {
	[0] = {
		.name = "60Hz",
		.clk = 49920000,
	},
	[1] = {
		.name = "50Hz",
		.clk = 39936000,
	},
	[2] = {
		.name = "45Hz",
		.clk = 36305454,
	},
	[3] = {
		.name = "40Hz",
		.clk = 33280000,
	},
};

static unsigned int lcdclk_usr = 0;	/* 60fps */

static void s6e63m0_lcdclk_thread(struct work_struct *s6e6m0_lcdclk_work)
{
	msleep(200);

	pr_err("[S6E63M0] LCDCLK %dHz\n", lcdclk_prop[lcdclk_usr].clk);

	LCDCLK_SET(lcdclk_prop[lcdclk_usr].clk);
}
static DECLARE_WORK(s6e63m0_lcdclk_work, s6e63m0_lcdclk_thread);

#ifdef SMART_DIMMING
#define LDI_MTP_LENGTH 21
#define LDI_MTP_ADDR	0xd3

/* mtp_data_from_boot used console memory */
extern char mtp_data_from_boot[];

static unsigned int LCD_DB[] = {70, 71, 72, 73, 74, 75, 76, 77};

static int is_load_mtp_offset;
module_param(is_load_mtp_offset, bool, 0644);

/* READ CLK */
#define LCD_MPU80_RDX 169
/* DATA or COMMAND */
#define LCD_MPU80_DCX 171
/* WRITE CLK */
#define LCD_MPU80_WRX 220
/* CHIP SELECT */
#define LCD_MPU80_CSX 223

static pin_cfg_t  janice_mpu80_pins_enable[] = {
	GPIO169_GPIO | PIN_OUTPUT_HIGH, /* LCD_MPU80_RDX */
	GPIO171_GPIO | PIN_OUTPUT_HIGH, /* LCD_MPU80_DCX */
	GPIO220_GPIO | PIN_OUTPUT_HIGH, /* LCD_MPU80_WRX */
	GPIO223_GPIO | PIN_OUTPUT_HIGH, /* LCD_MPU80_CSX */

	/*LCD_MPU80_D0 ~ LCD_MPU80_D7 */
	GPIO70_GPIO | PIN_OUTPUT_HIGH,
	GPIO71_GPIO | PIN_OUTPUT_HIGH,
	GPIO72_GPIO | PIN_OUTPUT_HIGH,
	GPIO73_GPIO | PIN_OUTPUT_HIGH,
	GPIO74_GPIO | PIN_OUTPUT_HIGH,
	GPIO75_GPIO | PIN_OUTPUT_HIGH,
	GPIO76_GPIO | PIN_OUTPUT_HIGH,
	GPIO77_GPIO | PIN_OUTPUT_HIGH,
};

static pin_cfg_t janice_mpu80_pins_disable[] = {
	GPIO169_LCDA_DE,
	GPIO171_LCDA_HSO,
	GPIO220_GPIO | PIN_OUTPUT_HIGH, /* LCD_MPU80_WRX */
	GPIO223_GPIO | PIN_OUTPUT_HIGH, /* LCD_MPU80_CSX */

	/*LCD_MPU80_D0 ~ LCD_MPU80_D7 */
	GPIO70_LCD_D0,
	GPIO71_LCD_D1,
	GPIO72_LCD_D2,
	GPIO73_LCD_D3,
	GPIO74_LCD_D4,
	GPIO75_LCD_D5,
	GPIO76_LCD_D6,
	GPIO77_LCD_D7,
};

static pin_cfg_t janice_mpu80_data_line_input[] = {
	/*LCD_MPU80_D0 ~ LCD_MPU80_D7 */
	GPIO70_GPIO | PIN_INPUT_NOPULL,
	GPIO71_GPIO | PIN_INPUT_NOPULL,
	GPIO72_GPIO | PIN_INPUT_NOPULL,
	GPIO73_GPIO | PIN_INPUT_NOPULL,
	GPIO74_GPIO | PIN_INPUT_NOPULL,
	GPIO75_GPIO | PIN_INPUT_NOPULL,
	GPIO76_GPIO | PIN_INPUT_NOPULL,
	GPIO77_GPIO | PIN_INPUT_NOPULL,
};

static pin_cfg_t  janice_mpu80_data_line_output[] = {
	/*LCD_MPU80_D0 ~ LCD_MPU80_D7 */
	GPIO70_GPIO | PIN_OUTPUT_HIGH,
	GPIO71_GPIO | PIN_OUTPUT_HIGH,
	GPIO72_GPIO | PIN_OUTPUT_HIGH,
	GPIO73_GPIO | PIN_OUTPUT_HIGH,
	GPIO74_GPIO | PIN_OUTPUT_HIGH,
	GPIO75_GPIO | PIN_OUTPUT_HIGH,
	GPIO76_GPIO | PIN_OUTPUT_HIGH,
	GPIO77_GPIO | PIN_OUTPUT_HIGH,
};


const unsigned short  prepare_mtp_read[] = {
	/* LV2, LV3, MTP lock release code */
	0xf0, 0x5a,
	DATA_ONLY, 0x5a,
	0xf1, 0x5a,
	DATA_ONLY, 0x5a,
	0xfc, 0x5a,
	DATA_ONLY, 0x5a,
	/* MTP cell enable */
	0xd1,0x80,

	ENDDEF, 0x0000,
};

const unsigned short  start_mtp_read[] = {
	/* MPU  8bit read mode start */
	0xfc, 0x0c,
	DATA_ONLY, 0x00,
	ENDDEF, 0x0000,
};
#endif

static const unsigned short SEQ_PANEL_CONDITION_SET[] = {
	0xf8, 0x01,
	DATA_ONLY, 0x27,
	DATA_ONLY, 0x27,
	DATA_ONLY, 0x07,
	DATA_ONLY, 0x07,
	DATA_ONLY, 0x54,
	DATA_ONLY, 0x9f,
	DATA_ONLY, 0x63,
	DATA_ONLY, 0x8f,
//	DATA_ONLY, 0x86,
	DATA_ONLY, 0x1a,
	DATA_ONLY, 0x33,
	DATA_ONLY, 0x0d,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,

	ENDDEF, 0x0000
};

static const unsigned short SEQ_DISPLAY_CONDITION_SET[] = {
	0xf2, 0x02,
	DATA_ONLY, 0x03,
	DATA_ONLY, 0x1c,
	DATA_ONLY, 0x10,
	DATA_ONLY, 0x10,

	0xf7, 0x03,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,

	ENDDEF, 0x0000
};

static const unsigned short SEQ_GAMMA_SETTING_160[] = {
	0xfa, 0x02,

	DATA_ONLY, 0x18,
	DATA_ONLY, 0x08,
	DATA_ONLY, 0x24,
	DATA_ONLY, 0x7F,
	DATA_ONLY, 0x6E,
	DATA_ONLY, 0x5F,
	DATA_ONLY, 0xC0,
	DATA_ONLY, 0xC6,
	DATA_ONLY, 0xB5,
	DATA_ONLY, 0xBA,
	DATA_ONLY, 0xBF,
	DATA_ONLY, 0xAD,
	DATA_ONLY, 0xCB,
	DATA_ONLY, 0xCF,
	DATA_ONLY, 0xC0,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x94,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x91,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0xC8,

	0xfa, 0x03,

	ENDDEF, 0x0000
};

static const unsigned short SEQ_ETC_CONDITION_SET[] = {
	0xf6, 0x00,
	DATA_ONLY, 0x8E,
	DATA_ONLY, 0x07,

	0xb3, 0x6C,

	0xb5, 0x2c,
	DATA_ONLY, 0x12,
	DATA_ONLY, 0x0c,
	DATA_ONLY, 0x0a,
	DATA_ONLY, 0x10,
	DATA_ONLY, 0x0e,
	DATA_ONLY, 0x17,
	DATA_ONLY, 0x13,
	DATA_ONLY, 0x1f,
	DATA_ONLY, 0x1a,
	DATA_ONLY, 0x2a,
	DATA_ONLY, 0x24,
	DATA_ONLY, 0x1f,
	DATA_ONLY, 0x1b,
	DATA_ONLY, 0x1a,
	DATA_ONLY, 0x17,

	DATA_ONLY, 0x2b,
	DATA_ONLY, 0x26,
	DATA_ONLY, 0x22,
	DATA_ONLY, 0x20,
	DATA_ONLY, 0x3a,
	DATA_ONLY, 0x34,
	DATA_ONLY, 0x30,
	DATA_ONLY, 0x2c,
	DATA_ONLY, 0x29,
	DATA_ONLY, 0x26,
	DATA_ONLY, 0x25,
	DATA_ONLY, 0x23,
	DATA_ONLY, 0x21,
	DATA_ONLY, 0x20,
	DATA_ONLY, 0x1e,
	DATA_ONLY, 0x1e,

	0xb6, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x11,
	DATA_ONLY, 0x22,
	DATA_ONLY, 0x33,
	DATA_ONLY, 0x44,
	DATA_ONLY, 0x44,
	DATA_ONLY, 0x44,

	DATA_ONLY, 0x55,
	DATA_ONLY, 0x55,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,

	0xb7, 0x2c,
	DATA_ONLY, 0x12,
	DATA_ONLY, 0x0c,
	DATA_ONLY, 0x0a,
	DATA_ONLY, 0x10,
	DATA_ONLY, 0x0e,
	DATA_ONLY, 0x17,
	DATA_ONLY, 0x13,
	DATA_ONLY, 0x1f,
	DATA_ONLY, 0x1a,
	DATA_ONLY, 0x2a,
	DATA_ONLY, 0x24,
	DATA_ONLY, 0x1f,
	DATA_ONLY, 0x1b,
	DATA_ONLY, 0x1a,
	DATA_ONLY, 0x17,

	DATA_ONLY, 0x2b,
	DATA_ONLY, 0x26,
	DATA_ONLY, 0x22,
	DATA_ONLY, 0x20,
	DATA_ONLY, 0x3a,
	DATA_ONLY, 0x34,
	DATA_ONLY, 0x30,
	DATA_ONLY, 0x2c,
	DATA_ONLY, 0x29,
	DATA_ONLY, 0x26,
	DATA_ONLY, 0x25,
	DATA_ONLY, 0x23,
	DATA_ONLY, 0x21,
	DATA_ONLY, 0x20,
	DATA_ONLY, 0x1e,
	DATA_ONLY, 0x1e,

	0xb8, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x11,
	DATA_ONLY, 0x22,
	DATA_ONLY, 0x33,
	DATA_ONLY, 0x44,
	DATA_ONLY, 0x44,
	DATA_ONLY, 0x44,

	DATA_ONLY, 0x55,
	DATA_ONLY, 0x55,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,

	0xb9, 0x2c,
	DATA_ONLY, 0x12,
	DATA_ONLY, 0x0c,
	DATA_ONLY, 0x0a,
	DATA_ONLY, 0x10,
	DATA_ONLY, 0x0e,
	DATA_ONLY, 0x17,
	DATA_ONLY, 0x13,
	DATA_ONLY, 0x1f,
	DATA_ONLY, 0x1a,
	DATA_ONLY, 0x2a,
	DATA_ONLY, 0x24,
	DATA_ONLY, 0x1f,
	DATA_ONLY, 0x1b,
	DATA_ONLY, 0x1a,
	DATA_ONLY, 0x17,

	DATA_ONLY, 0x2b,
	DATA_ONLY, 0x26,
	DATA_ONLY, 0x22,
	DATA_ONLY, 0x20,
	DATA_ONLY, 0x3a,
	DATA_ONLY, 0x34,
	DATA_ONLY, 0x30,
	DATA_ONLY, 0x2c,
	DATA_ONLY, 0x29,
	DATA_ONLY, 0x26,
	DATA_ONLY, 0x25,
	DATA_ONLY, 0x23,
	DATA_ONLY, 0x21,
	DATA_ONLY, 0x20,
	DATA_ONLY, 0x1e,
	DATA_ONLY, 0x1e,

	0xba, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x11,
	DATA_ONLY, 0x22,
	DATA_ONLY, 0x33,
	DATA_ONLY, 0x44,
	DATA_ONLY, 0x44,
	DATA_ONLY, 0x44,

	DATA_ONLY, 0x55,
	DATA_ONLY, 0x55,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,

	ENDDEF, 0x0000
};

static const unsigned short SEQ_ACL_SETTING_40[] = {
	0xc1, 0x4D,
	DATA_ONLY, 0x96,
	DATA_ONLY, 0x1D,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x01,
	DATA_ONLY, 0xDF,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x03,
	DATA_ONLY, 0x1F,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x01,
	DATA_ONLY, 0x06,
	DATA_ONLY, 0x0C,
	DATA_ONLY, 0x11,
	DATA_ONLY, 0x16,
	DATA_ONLY, 0x1C,
	DATA_ONLY, 0x21,
	DATA_ONLY, 0x26,
	DATA_ONLY, 0x2B,
	DATA_ONLY, 0x31,
	DATA_ONLY, 0x36,

	ENDDEF, 0x0000
};

static const unsigned short SEQ_ELVSS_SETTING[] = {
	0xb2, 0x17,
	DATA_ONLY, 0x17,
	DATA_ONLY, 0x17,
	DATA_ONLY, 0x17,

	ENDDEF, 0x0000
};

static const unsigned short SEQ_ELVSS_ON[] = {
	/* ELVSS on */
	0xb1, 0x0b,

	ENDDEF, 0x0000
};

static const unsigned short SEQ_ELVSS_OFF[] = {
	/* ELVSS off */
	0xb1, 0x0a,

	ENDDEF, 0x0000
};

static const unsigned short SEQ_STAND_BY_OFF[] = {
	0x11, COMMAND_ONLY,
	SLEEPMSEC, 120,

	ENDDEF, 0x0000
};

static const unsigned short SEQ_STAND_BY_ON[] = {
	0x10, COMMAND_ONLY,
	SLEEPMSEC, 120,

	ENDDEF, 0x0000
};

static const unsigned short SEQ_DISPLAY_ON[] = {
	0x29, COMMAND_ONLY,

	ENDDEF, 0x0000
};

/* Horizontal flip */
static const unsigned short DCS_CMD_SEQ_ORIENTATION_180[] = {
/*	Length	Command 			Parameters */
	0xF7,	0x00,

	ENDDEF, 0x0000
};

/* Default Orientation */
static const unsigned short DCS_CMD_SEQ_ORIENTATION_DEFAULT[] = {
/*	Length	Command 			Parameters */
	0xF7,	0x03,
	
	ENDDEF, 0x0000
};
static void print_vmode(struct device *dev, struct mcde_video_mode *vmode)
{
/*
	dev_dbg(dev, "resolution: %dx%d\n", vmode->xres, vmode->yres);
	dev_dbg(dev, "  pixclock: %d\n",    vmode->pixclock);
	dev_dbg(dev, "       hbp: %d\n",    vmode->hbp);
	dev_dbg(dev, "       hfp: %d\n",    vmode->hfp);
	dev_dbg(dev, "       hsw: %d\n",    vmode->hsw);
	dev_dbg(dev, "       vbp: %d\n",    vmode->vbp);
	dev_dbg(dev, "       vfp: %d\n",    vmode->vfp);
	dev_dbg(dev, "       vsw: %d\n",    vmode->vsw);
	dev_dbg(dev, "interlaced: %s\n",	vmode->interlaced ? "true" : "false");
*/
}

static int try_video_mode(
	struct mcde_display_device *ddev, struct mcde_video_mode *video_mode)
{
	int res = -EINVAL;

	if (ddev == NULL || video_mode == NULL) {
		dev_warn(&ddev->dev, "%s:ddev = NULL or video_mode = NULL\n",
			__func__);
		return res;
	}

	if ((video_mode->xres == VMODE_XRES && video_mode->yres == VMODE_YRES) ||
	    (video_mode->xres == VMODE_YRES && video_mode->yres == VMODE_XRES)) {

		video_mode->hsw = 2;
		video_mode->hbp = 16; /* from end of hsync */
		video_mode->hfp = 16;
		video_mode->vsw = 2;
                video_mode->vbp = 1;//3  /* from end of vsync */
                video_mode->vfp = 28;//26;
		video_mode->interlaced = false;
		/* +445681 display padding */
		video_mode->xres_padding = ddev->x_res_padding;
		video_mode->yres_padding = ddev->y_res_padding;
		/* -445681 display padding */
		
		/*
		 * The pixclock setting is not used within MCDE. The clock is
		 * setup elsewhere. But the pixclock value is visible in user
		 * space.
		 */
		video_mode->pixclock =	(int) (1e+12 * (1.0 / PIX_CLK_FREQ));

		res = 0;
	}

	if (res == 0)
		print_vmode(&ddev->dev, video_mode);
	else
		dev_warn(&ddev->dev,
			"%s:Failed to find video mode x=%d, y=%d\n",
			__func__, video_mode->xres, video_mode->yres);

	return res;

}

static int set_video_mode(
	struct mcde_display_device *ddev, struct mcde_video_mode *video_mode)
{
	int res = -EINVAL;
	struct mcde_video_mode channel_video_mode;
	static int video_mode_apply_during_boot = 1;
	struct s6e63m0 *lcd = dev_get_drvdata(&ddev->dev);

	if (ddev == NULL || video_mode == NULL) {
		dev_warn(&ddev->dev, "%s:ddev = NULL or video_mode = NULL\n",
			__func__);
		goto out;
	}
	ddev->video_mode = *video_mode;
	print_vmode(&ddev->dev, video_mode);
	if ((video_mode->xres == VMODE_XRES && video_mode->yres == VMODE_YRES) ||
	    (video_mode->xres == VMODE_YRES && video_mode->yres == VMODE_XRES)) {
		res = 0;
	}
	if (res < 0) {
		dev_warn(&ddev->dev, "%s:Failed to set video mode x=%d, y=%d\n",
			__func__, video_mode->xres, video_mode->yres);
		goto error;
	}

	channel_video_mode = ddev->video_mode;
	/* Dependant on if display should rotate or MCDE should rotate */
	if (ddev->rotation == MCDE_DISPLAY_ROT_90_CCW ||
		ddev->rotation == MCDE_DISPLAY_ROT_90_CW) {
		channel_video_mode.xres = ddev->native_x_res;
		channel_video_mode.yres = ddev->native_y_res;
	}
		channel_video_mode.xres_padding= 0;
		channel_video_mode.yres_padding= 0;
	
	res = mcde_chnl_set_video_mode(ddev->chnl_state, &channel_video_mode);
	if (res < 0) {
		dev_warn(&ddev->dev, "%s:Failed to set video mode on channel\n",
			__func__);

		goto error;
	}
	/* notify mcde display driver about updated video mode, excepted for
	 * the first update to preserve the splash screen and avoid a
	 * stop_flow() */
	if (video_mode_apply_during_boot && lcd->pd->platform_enabled) {
		ddev->update_flags |= UPDATE_FLAG_PIXEL_FORMAT;
		video_mode_apply_during_boot = 0;
	} else
		ddev->update_flags |= UPDATE_FLAG_VIDEO_MODE;
	return res;
out:
error:
	return res;
}

/* Reverse order of power on and channel update as compared with MCDE default display update */
static int s6e63m0_display_update(struct mcde_display_device *ddev,
							bool tripple_buffer)
{
	int ret = 0;

	if (ddev->power_mode != MCDE_DISPLAY_PM_ON && ddev->set_power_mode) {
		ret = ddev->set_power_mode(ddev, MCDE_DISPLAY_PM_ON);
		if (ret < 0) {
			dev_warn(&ddev->dev,
				"%s:Failed to set power mode to on\n",
				__func__);
			return ret;
		}
	}

	ret = mcde_chnl_update(ddev->chnl_state, tripple_buffer);
	if (ret < 0) {
		dev_warn(&ddev->dev, "%s:Failed to update channel\n", __func__);
		return ret;
	}
	return 0;
}

static int s6e63m0_apply_config(struct mcde_display_device *ddev)
{
	int ret;

	if (!ddev->update_flags)
		return 0;

	if (ddev->update_flags & (UPDATE_FLAG_VIDEO_MODE |
			UPDATE_FLAG_ROTATION))
		mcde_chnl_stop_flow(ddev->chnl_state);

	ret = mcde_chnl_apply(ddev->chnl_state);
	if (ret < 0) {
		dev_warn(&ddev->dev, "%s:Failed to apply to channel\n",
							__func__);
		return ret;
	}

	ddev->update_flags = 0;
	ddev->first_update = true;

	return 0;
}

static int s6e63m0_spi_write_byte(struct s6e63m0 *lcd, int addr, int data)
{
	u16 buf[1];
	struct spi_message msg;

	struct spi_transfer xfer = {
		.len		= 2,
		.tx_buf		= buf,
	};

	buf[0] = (addr << 8) | data;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

		return spi_sync(lcd->spi, &msg);
	}

static int s6e63m0_spi_read_byte(struct s6e63m0 *lcd, int addr, u8 *data)
{
	u16 buf[2];
	u16 rbuf[2];
	int ret;
	struct spi_message msg;
	struct spi_transfer xfer = {
		.len		= 4,
		.tx_buf		= buf,
		.rx_buf		= rbuf,
	};

	buf[0] = addr;
	buf[1] = 0x100;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	ret = spi_sync(lcd->spi, &msg);
	if (ret)
		return ret;

	*data = (rbuf[1] & 0x1FF) >> 1;

	return ret;
}

static int s6e63m0_spi_write(struct s6e63m0 *lcd, unsigned char address,
	unsigned char command)
{
	int ret = 0;

	if (address != DATA_ONLY)
		ret = s6e63m0_spi_write_byte(lcd, 0x0, address);
	if (command != COMMAND_ONLY)
		ret = s6e63m0_spi_write_byte(lcd, 0x1, command);

	return ret;
}

static int s6e63m0_spi_write_words(struct s6e63m0 *lcd,
	const u16 *buf, int len)
{
	struct spi_message msg;
	struct spi_transfer xfer = {
		.len		= 2 * len,
		.tx_buf		= buf,
	};

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	if(lcd->spi){
		return spi_sync(lcd->spi, &msg);
	}else{
		return -1;
	}
}


int s6e63m0_panel_send_sequence(struct s6e63m0 *lcd,
	const unsigned short *buf)
{
	int ret = 0, i = 0;

	const unsigned short *wbuf;

	mutex_lock(&lcd->lock);

	wbuf = buf;

	while ((wbuf[i] & DEFMASK) != ENDDEF) {
		if ((wbuf[i] & DEFMASK) != SLEEPMSEC) {
			ret = s6e63m0_spi_write(lcd, wbuf[i], wbuf[i + 1]);
			if (ret)
				break;
		} else
			udelay(wbuf[i + 1] * 1000);
		i += 2;
	}

	mutex_unlock(&lcd->lock);

	return ret;
}

#ifdef SPI_3WIRE_IF
static pin_cfg_t janice_spi_3wire_pins_enable[] = {
	GPIO220_GPIO | PIN_OUTPUT_HIGH, /* GPIO220_SPI0_CLK */
	GPIO223_GPIO | PIN_OUTPUT_HIGH, /* GPIO223_SPI0_CS */
	GPIO224_GPIO | PIN_OUTPUT_HIGH, /* GPIO224_SPI0_TXD */
};

static pin_cfg_t janice_spi_3wire_SDA = GPIO224_GPIO|PIN_OUTPUT_HIGH;
static pin_cfg_t janice_spi_3wire_SDI = GPIO224_GPIO|PIN_INPUT_NOPULL;

#define DEFAULT_SLEEP 1
#define SPI_3WIRE_CLK_HIGH do { \
	gpio_direction_output(LCD_CLK_JANICE_R0_0, 1); \
} while (0);

#define SPI_3WIRE_CLK_LOW do { \
	gpio_direction_output(LCD_CLK_JANICE_R0_0, 0); \
} while (0);

#define SPI_3WIRE_CS_HIGH do { \
	gpio_direction_output(LCD_CSX_JANICE_R0_0, 1); \
} while (0);

#define SPI_3WIRE_CS_LOW do { \
	gpio_direction_output(LCD_CSX_JANICE_R0_0, 0); \
} while (0);

#define SPI_3WIRE_SDA_HIGH do { \
	gpio_direction_output(LCD_SDI_JANICE_R0_0, 1); \
} while (0);

#define SPI_3WIRE_SDA_LOW do { \
	gpio_direction_output(LCD_SDI_JANICE_R0_0, 0); \
} while (0);

void spi_3wire_gpio_enable(unsigned char enable)
{
	if (enable) {
		nmk_config_pins(janice_spi_3wire_pins_enable,
				ARRAY_SIZE(janice_spi_3wire_pins_enable));
	}
}

void spi_3wire_write_byte(u8 cmd, u8 addr)
{
	int bit;
	int bnum;

	SPI_3WIRE_CLK_LOW

	if (cmd == COMMAND_ONLY)
		SPI_3WIRE_SDA_LOW
	else
		SPI_3WIRE_SDA_HIGH

	udelay(DEFAULT_SLEEP);
	SPI_3WIRE_CLK_HIGH
	udelay(DEFAULT_SLEEP);

	bnum = 8;
	bit = 0x80;
	while (bnum--) {
		SPI_3WIRE_CLK_LOW
		if (addr & bit)
			SPI_3WIRE_SDA_HIGH
		else
			SPI_3WIRE_SDA_LOW
		udelay(1);
		SPI_3WIRE_CLK_HIGH
		udelay(1);
		bit >>= 1;
	}
}

void spi_3wire_read_byte(u8 addr, u8 *data)
{
	int bit;
	spi_3wire_gpio_enable(1);

	SPI_3WIRE_CS_LOW
	udelay(DEFAULT_SLEEP);

	spi_3wire_write_byte(COMMAND_ONLY, addr);

	SPI_3WIRE_SDA_LOW

	nmk_config_pin(janice_spi_3wire_SDI, 0);

	bit = 8;
	*data = 0;
	while (bit) {
		SPI_3WIRE_CLK_LOW
		udelay(DEFAULT_SLEEP);
		*data <<= 1;
		*data |= gpio_get_value(LCD_SDI_JANICE_R0_0) ? 1 : 0;
		SPI_3WIRE_CLK_HIGH
		udelay(DEFAULT_SLEEP);
		--bit;
	}
	nmk_config_pin(janice_spi_3wire_SDA, 0);

	SPI_3WIRE_CS_HIGH
	SPI_3WIRE_CLK_HIGH
	SPI_3WIRE_SDA_HIGH
}

#endif

static int s6e63m0_read_panel_id(struct s6e63m0 *lcd, u8 *idbuf)
{
#ifdef SPI_3WIRE_IF
	static int pre_id_read;
	static u8 lcd_panel_id[3];

	if (pre_id_read) {
		memcpy(idbuf, lcd_panel_id, 3);
	} else {
		spi_3wire_read_byte(0xDA, &idbuf[0]);
		spi_3wire_read_byte(0xDB, &idbuf[1]);
		spi_3wire_read_byte(0xDC, &idbuf[2]);
		memcpy(lcd_panel_id, idbuf, 3);
		lcd->panel_id = idbuf[1];
		lcd->elvss_ref = idbuf[2];
		pre_id_read = 1;
	}
	return 0;
#else
	int ret;

	ret = s6e63m0_spi_write(lcd,0xB0,0x00);
	ret |= s6e63m0_spi_write(lcd, 0xDE, COMMAND_ONLY);
	ret |= s6e63m0_spi_read_byte(lcd, 0xDA, &idbuf[0]);
	ret |= s6e63m0_spi_read_byte(lcd, 0xDB, &idbuf[1]);
	ret |= s6e63m0_spi_read_byte(lcd, 0xDC, &idbuf[2]);
	ret |= s6e63m0_spi_write(lcd, 0xDF, COMMAND_ONLY);

	return ret;
#endif
}


#ifdef SMART_DIMMING
#define gen_table_max 21
#if 0
int illumination_table[] =
{
30, 40, 50, 60, 70, 80, 90,
100, 105, 110, 120, 130, 140,
150, 160, 170, 180, 190, 200,
205, 210, 220, 230, 240, 250,
300,
};
#else
int illumination_table[] = {
	25, 
	40, 
	50, 
	60, 
	70, 
	80, 
	90,
	100, 
	105, 
	110, 
	120, 
	130, 
	140,
	150, 
	160, 
	170, 
	173, 
	180, 
	193,
	198, 
	203, 
	213, 
	223, 
	233, 
	243,
	293,
};

#endif
unsigned short s6e63m0_22_gamma_table[] = {
	0xFA, 0x02,

	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,

	0xFA, 0x03,

	ENDDEF, 0x0000
};

unsigned short s6e63m0_22_gamma_table_custom[] = {
	0xFA, 0x02,

	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,

	0xFA, 0x03,

	ENDDEF, 0x0000
};

static unsigned short s6e63m0_gamma_table_night[] = {
	0xFA, 0x02,

	DATA_ONLY, 0x18,
	DATA_ONLY, 0x08,
	DATA_ONLY, 0x24,
	DATA_ONLY, 0xDC,
	DATA_ONLY, 0xDD,
	DATA_ONLY, 0xD0,
	DATA_ONLY, 0xB5,
	DATA_ONLY, 0xB5,
	DATA_ONLY, 0xB6,
	DATA_ONLY, 0x7F,
	DATA_ONLY, 0x8D,
	DATA_ONLY, 0x63,
	DATA_ONLY, 0x85,	// Reduced brightness
	DATA_ONLY, 0x88,	// Reduced brightness
	DATA_ONLY, 0x7A,	// Reduced brightness
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,	// Reduced brightness
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,	// Reduced brightness
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,	// Reduced brightness

	0xFA, 0x03,

	ENDDEF, 0x0000
};

static unsigned short s6e63m0_gamma_table_sunlight[] = {
	0xFA, 0x02,

	DATA_ONLY, 0x18,
	DATA_ONLY, 0x08,
	DATA_ONLY, 0x24,
	DATA_ONLY, 0x44,
	DATA_ONLY, 0x4D,
	DATA_ONLY, 0x27,
	DATA_ONLY, 0x9B,
	DATA_ONLY, 0xA2,
	DATA_ONLY, 0x8D,
	DATA_ONLY, 0xC2,
	DATA_ONLY, 0xC9,
	DATA_ONLY, 0xB7,
	DATA_ONLY, 0xD0,
	DATA_ONLY, 0xD4,
	DATA_ONLY, 0xC8,
	DATA_ONLY, 0x01,
	DATA_ONLY, 0x40,
	DATA_ONLY, 0x01,
	DATA_ONLY, 0x3E,
	DATA_ONLY, 0x01,
	DATA_ONLY, 0x59,

	0xFA, 0x03,

	ENDDEF, 0x0000
};

unsigned short *Gen_gamma_table(struct s6e63m0 *lcd)
{
	int i;
	int j = 0;
	char gen_gamma[gen_table_max] ={0,};

	/* TODO: Send illumination(brightness) to smart dimming */
	if (illumination_req) {
		lcd->smart.brightness_level = illumination_val;
	} else {
		lcd->smart.brightness_level = illumination_table[lcd->bl];
	}

	generate_gamma(&(lcd->smart), gen_gamma, gen_table_max);

	for(i=3;i<((gen_table_max*2)+3);i+=2) {
		s6e63m0_22_gamma_table[i] = gen_gamma[j++];
	}

	/* for debug */
	#if 0
		printk("%s lcd->bl : %d ",__func__,lcd->bl);
		for(i=3;i<((gen_table_max*2)+3);i+=2) {
			printk("0x%x ",s6e63m0_22_gamma_table[i]);
		}
		printk("\n");
	#endif

	if (R_req) {
		s6e63m0_22_gamma_table[R_OFFSET] = R_val;
	}

	if (G_req) {
		s6e63m0_22_gamma_table[G_OFFSET] = G_val;
	}

	if (B_req) {
		s6e63m0_22_gamma_table[B_OFFSET] = B_val;
	}

	return s6e63m0_22_gamma_table;
}

#endif

static int s6e63m0_gamma_ctl(struct s6e63m0 *lcd)
{
	int ret = 0;
	const unsigned short *gamma;


	if (lcd->gamma_mode) {
		gamma = gamma_table.gamma_19_table[lcd->bl];
	} else {
		#ifdef SMART_DIMMING
		if(is_load_mtp_offset && !gamma_table_req) {
			gamma = Gen_gamma_table(lcd);
		} else if (gamma_table_req) {
			gamma = s6e63m0_22_gamma_table_custom;
		} else {
			gamma = gamma_table.gamma_22_table[lcd->bl];
		}
		#else
		gamma = gamma_table.gamma_22_table[lcd->bl];
		#endif
	}

	ret = s6e63m0_panel_send_sequence(lcd, gamma);
	if (ret) {
		dev_err(lcd->dev, "failed to disable gamma table updating.\n");
		goto gamma_err;
	}

	lcd->current_brightness = lcd->bl;
	lcd->current_gamma_mode = lcd->gamma_mode;
gamma_err:
	return ret;
}


unsigned short SEQ_DYNAMIC_ELVSS[] = {
	0xB2, 0x0,
	DATA_ONLY, 0x0,
	DATA_ONLY, 0x0,
	DATA_ONLY, 0x0,

	0xB1, 0x0B,

	ENDDEF, 0x00
};

int dynamic_elvss_cal(struct s6e63m0 *lcd, int step)
{
	int data, cnt;

	data = lcd->elvss_ref + ELVSS_OFFSET[step];

	if (data > ELVSS_MAX)
		data = ELVSS_MAX;

	for (cnt = 1; cnt <= 7; cnt += 2)
		SEQ_DYNAMIC_ELVSS[cnt] = data;

	return s6e63m0_panel_send_sequence(lcd, SEQ_DYNAMIC_ELVSS);
}

static int s6e63m0_set_elvss(struct s6e63m0 *lcd)
{
	int ret = 0;

	if ((lcd->elvss_ref) && (lcd->panel_id >= SMART_MTP_PANEL_ID)) {
		switch (lcd->bl) {
		case 0 ... 7: /* 30cd ~ 100cd */
			ret = dynamic_elvss_cal(lcd, 3);
			break;
		case 8 ... 14: /* 110cd ~ 160cd */
			ret = dynamic_elvss_cal(lcd, 2);
			break;
		case 15 ... 18: /* 170cd ~ 200cd */
			ret = dynamic_elvss_cal(lcd, 1);
			break;
		case 19 ... 27: /* 210cd ~ 300cd */
			ret = dynamic_elvss_cal(lcd, 0);
			break;
		default:
			break;
		}
	} else {
		switch (lcd->bl) {
		case 0 ... 4: /* 30cd ~ 100cd */
		ret = s6e63m0_panel_send_sequence(lcd, SEQ_ELVSS_SET[3]);
		break;
		case 5 ... 10: /* 110cd ~ 160cd */
		ret = s6e63m0_panel_send_sequence(lcd, SEQ_ELVSS_SET[2]);
		break;
		case 11 ... 14: /* 170cd ~ 200cd */
		ret = s6e63m0_panel_send_sequence(lcd, SEQ_ELVSS_SET[1]);
		break;
		case 15 ... 27: /* 210cd ~ 300cd */
		ret = s6e63m0_panel_send_sequence(lcd, SEQ_ELVSS_SET[0]);
		break;
		default:
		break;
		}
	}

	dev_dbg(lcd->dev, "level  = %d\n", lcd->bl);

	if (ret) {
		dev_err(lcd->dev, "failed to initialize ldi.\n");
		return -EIO;
	}

	return ret;
}

static int s6e63m0_set_acl(struct s6e63m0 *lcd)
{
	int ret = 0;

	if (lcd->acl_enable) {
		switch (lcd->bl) {
		case 0 ... 3: /* 30cd ~ 60cd */
			if (lcd->cur_acl != 0) {
				ret = s6e63m0_panel_send_sequence(lcd, ACL_cutoff_set[8]);
				dev_dbg(lcd->dev, "ACL_cutoff_set Percentage : off!!\n");
				lcd->cur_acl = 0;
			}
			break;
		case 4 ... 24: /* 70cd ~ 250 */
			if (lcd->cur_acl != 40) {
				ret |= s6e63m0_panel_send_sequence(lcd, ACL_cutoff_set[1]);
				dev_dbg(lcd->dev, "ACL_cutoff_set Percentage : 40!!\n");
				lcd->cur_acl = 40;
			}
			break;
		default: /* 300 */
			if (lcd->cur_acl != 50) {
				ret |= s6e63m0_panel_send_sequence(lcd, ACL_cutoff_set[6]);
				dev_dbg(lcd->dev, "ACL_cutoff_set Percentage : 50!!\n");
				lcd->cur_acl = 50;
			}
			break;
		}
	} else {
			ret = s6e63m0_panel_send_sequence(lcd, ACL_cutoff_set[8]);
			lcd->cur_acl = 0;
			dev_dbg(lcd->dev, "ACL_cutoff_set Percentage : off!!\n");
	}

	if (ret) {
		dev_err(lcd->dev, "failed to initialize ldi.\n");
		return -EIO;
	}

	return ret;
}

/* s6e63m0_set_rotation */
static int s6e63m0_set_rotation(struct mcde_display_device *ddev,
	enum mcde_display_rotation rotation)
{
	static int notFirstTime;
	int ret = 0;
	enum mcde_display_rotation final;
	struct s6e63m0 *lcd = dev_get_drvdata(&ddev->dev);
	enum mcde_hw_rotation final_hw_rot;

	final = (360 + rotation - ddev->orientation) % 360;

	switch (final) {
	case MCDE_DISPLAY_ROT_180:	/* handled by LDI */
	case MCDE_DISPLAY_ROT_0:
		final_hw_rot = MCDE_HW_ROT_0;
		break;
	case MCDE_DISPLAY_ROT_90_CW:	/* handled by MCDE */
		final_hw_rot = MCDE_HW_ROT_90_CW;
		break;
	case MCDE_DISPLAY_ROT_90_CCW:	/* handled by MCDE */
		final_hw_rot = MCDE_HW_ROT_90_CCW;
		break;
	default:
		return -EINVAL;
	}

	if (rotation != ddev->rotation) {
		
		if (final == MCDE_DISPLAY_ROT_180) {
			if (final != lcd->rotation) {
				ret = s6e63m0_panel_send_sequence(lcd,
						DCS_CMD_SEQ_ORIENTATION_180);
				lcd->rotation = final;
			}
		} else if (final == MCDE_DISPLAY_ROT_0) {
			if (final != lcd->rotation) {
				ret = s6e63m0_panel_send_sequence(lcd,
						DCS_CMD_SEQ_ORIENTATION_DEFAULT);
				lcd->rotation = final;
			}
			(void)mcde_chnl_set_rotation(ddev->chnl_state, final_hw_rot);
		} else {
			ret = mcde_chnl_set_rotation(ddev->chnl_state, final_hw_rot);
		}

		if (ret)
			return ret;
		dev_dbg(lcd->dev, "Display rotated %d\n", final);
	}
	ddev->rotation = rotation;
	/* avoid disrupting splash screen by changing update_flags */
	if (notFirstTime || (final != MCDE_DISPLAY_ROT_0)) {
		notFirstTime = 1;
		ddev->update_flags |= UPDATE_FLAG_ROTATION;
	}
	return 0;
}

static int s6e63m0_ldi_init(struct s6e63m0 *lcd)
{
	int ret = 0, i;
	const unsigned short *init_seq[] = {
		SEQ_PANEL_CONDITION_SET,
		SEQ_DISPLAY_CONDITION_SET,
		SEQ_GAMMA_SETTING_160,
		SEQ_ETC_CONDITION_SET,
		SEQ_ACL_SETTING_40,
		SEQ_ACL_ON,
		SEQ_ELVSS_SETTING,
		SEQ_ELVSS_ON,
	};

	for (i = 0; i < ARRAY_SIZE(init_seq); i++) {
		ret = s6e63m0_panel_send_sequence(lcd, init_seq[i]);
		if (ret)
			break;
	}

	return ret;
}

static int s6e63m0_ldi_enable(struct s6e63m0 *lcd)
{
	int ret = 0, i;
	const unsigned short *enable_seq[] = {
		SEQ_STAND_BY_OFF,
		SEQ_DISPLAY_ON,
	};

	dev_dbg(lcd->dev, "s6e63m0_ldi_enable\n");

	for (i = 0; i < ARRAY_SIZE(enable_seq); i++) {
		ret = s6e63m0_panel_send_sequence(lcd, enable_seq[i]);
		if (ret)
			break;
	}
	lcd->ldi_state = LDI_STATE_ON;

	update_brightness(lcd,1);

	return ret;
}

static int s6e63m0_ldi_disable(struct s6e63m0 *lcd)
{
	int ret;

	dev_dbg(lcd->dev, "s6e63m0_ldi_disable\n");
	ret = s6e63m0_panel_send_sequence(lcd, SEQ_STAND_BY_ON);
	lcd->ldi_state = LDI_STATE_OFF;

	return ret;
}

#ifdef SMART_DIMMING
void s6e63m0_parallel_read(struct s6e63m0 *lcd,u8 cmd, u8 *data, size_t len)
{
	int delay = 1;
	int i;

	gpio_direction_output(LCD_MPU80_DCX, 0);
	udelay(delay);
	gpio_direction_output(LCD_MPU80_WRX, 0);
	nmk_config_pins(janice_mpu80_data_line_output,
			ARRAY_SIZE(janice_mpu80_data_line_output));

	for (i = 0; i < 8; i++) {
		gpio_direction_output(LCD_DB[i], (cmd >> i) & 1);
	}
	udelay(delay);
	gpio_direction_output(LCD_MPU80_WRX, 1);
	udelay(delay);
	gpio_direction_output(LCD_MPU80_DCX, 1);

	for (i = 0; i < 8; i++) {
		gpio_direction_output(LCD_DB[i], 0);
	}

	nmk_config_pins(janice_mpu80_data_line_input,
			ARRAY_SIZE(janice_mpu80_data_line_input));
	/*1 byte dummy */
	udelay(delay);
	gpio_direction_output(LCD_MPU80_RDX, 0);
	udelay(delay);
	gpio_direction_output(LCD_MPU80_RDX, 1);

	while (len--) {
		char d = 0;
		gpio_direction_output(LCD_MPU80_RDX, 0);
		udelay(delay);
		for (i = 0; i < 8; i++)
			d |= gpio_get_value(LCD_DB[i]) << i;
		*data++ = d;
		gpio_direction_output(LCD_MPU80_RDX, 1);
		udelay(delay);
	}
	gpio_direction_output(LCD_MPU80_RDX, 1);

}

static void configure_mtp_gpios(bool enable)
{
	if (enable) {
		nmk_config_pins(janice_mpu80_pins_enable,
				ARRAY_SIZE(janice_mpu80_pins_enable));

		gpio_request(169, "LCD_MPU80_RDX");
		gpio_request(171, "LCD_MPU80_DCX");
		gpio_request(70, "LCD_MPU80_D0");
		gpio_request(71, "LCD_MPU80_D1");
		gpio_request(72, "LCD_MPU80_D2");
		gpio_request(73, "LCD_MPU80_D3");
		gpio_request(74, "LCD_MPU80_D4");
		gpio_request(75, "LCD_MPU80_D5");
		gpio_request(76, "LCD_MPU80_D6");
		gpio_request(77, "LCD_MPU80_D7");
	} else {
		nmk_config_pins(janice_mpu80_pins_disable,
				ARRAY_SIZE(janice_mpu80_pins_disable));

		gpio_free(169);
		gpio_free(171);
		gpio_free(70);
		gpio_free(71);
		gpio_free(72);
		gpio_free(73);
		gpio_free(74);
		gpio_free(75);
		gpio_free(76);
		gpio_free(77);
	}
}


static void s6e63m0_parallel_setup_gpios(bool init)
{
	if (init) {
		configure_mtp_gpios(true);
		gpio_set_value(LCD_MPU80_CSX, 0);
		gpio_set_value(LCD_MPU80_WRX, 1);
		gpio_set_value(LCD_MPU80_RDX, 1);
		gpio_set_value(LCD_MPU80_DCX, 0);

	} else {
		configure_mtp_gpios(false);
		gpio_set_value(LCD_MPU80_CSX, 1);
	}
}

static void s6e63mo_read_mtp_info(struct s6e63m0 *lcd)
{
	int i=0;

	s6e63m0_panel_send_sequence(lcd,prepare_mtp_read);
	s6e63m0_panel_send_sequence(lcd,start_mtp_read);

	s6e63m0_parallel_setup_gpios(true);

	s6e63m0_parallel_read(lcd, LDI_MTP_ADDR,
				(u8 *)(&(lcd->smart.MTP)), LDI_MTP_LENGTH);

	for(i=0;i<LDI_MTP_LENGTH;i++) {
		printk("[S6E63M0] MainMTPData [%d] : %02x\n", i,
				((char*)&(lcd->smart.MTP))[i]);
	}

	Smart_dimming_init(&(lcd->smart));

	s6e63m0_parallel_setup_gpios(false);
}

static void s6e63mo_mtp_from_boot(struct s6e63m0 *lcd, char *mtp)
{
	int i;
	memcpy(&(lcd->smart.MTP), mtp, LDI_MTP_LENGTH);

	for (i = 0; i < LDI_MTP_LENGTH; i++) {
		printk("[S6E63M0] MainMTPData [%d] %02x\n", i,
				((char *)&(lcd->smart.MTP))[i]);
	}
	Smart_dimming_init(&(lcd->smart));
}
#endif

static int update_brightness(struct s6e63m0 *lcd, u8 force)
{
	int ret = 0;
	int bl = 0;

	mutex_lock(&lcd->lcd_lock);

	bl = lcd->bd->props.brightness;

	/* FIXME: Allow maximum gamma level(25) in manual mode */
	#if 0
	if (unlikely(!lcd->auto_brightness && bl > 241))
		bl = 241;
	#endif

	lcd->bl = get_gamma_value_from_bl(bl);

	if ((force) || ((lcd->ldi_state) &&
				(lcd->current_brightness != lcd->bl))) {

	printk("[S6E63M0] Brightness: %d BL: %d\n", bl, lcd->bl);

		ret = s6e63m0_set_elvss(lcd);
		if (ret) {
			dev_err(lcd->dev, "lcd brightness setting failed.\n");
			goto err;
		}

		ret = s6e63m0_set_acl(lcd);
		if (ret) {
			dev_err(lcd->dev, "lcd brightness setting failed.\n");
			goto err;
		}

		ret = s6e63m0_gamma_ctl(lcd);
		if (ret) {
			dev_err(lcd->dev, "lcd brightness setting failed.\n");
			goto err;
		}
	}
err:
	mutex_unlock(&lcd->lcd_lock);
	return 0;
}


static int s6e63m0_power_on(struct s6e63m0 *lcd)
{
	int ret = 0;
	struct ssg_dpi_display_platform_data *dpd = NULL;
	struct backlight_device *bd = NULL;

	dpd = lcd->pd;
	if (!dpd) {
		dev_err(lcd->dev, "s6e63m0 platform data is NULL.\n");
		return -EFAULT;
	}

	bd = lcd->bd;
	if (!bd) {
		dev_err(lcd->dev, "backlight device is NULL.\n");
		return -EFAULT;
	}

	dpd->power_on(dpd, LCD_POWER_UP);
	msleep(dpd->power_on_delay);

	if (!dpd->gpio_cfg_lateresume) {
		dev_err(lcd->dev, "gpio_cfg_lateresume is NULL.\n");
		return -EFAULT;
	} else
		dpd->gpio_cfg_lateresume();

	dpd->reset(dpd);
	msleep(dpd->reset_delay);

	/* force acl on */
	s6e63m0_panel_send_sequence(lcd, ACL_cutoff_set[7]);

	return ret;
}

static int s6e63m0_power_off(struct s6e63m0 *lcd)
{
	int ret = 0;
	struct ssg_dpi_display_platform_data *dpd = NULL;

	dev_dbg(lcd->dev, "s6e63m0_power_off\n");

	dpd = lcd->pd;
	if (!dpd) {
		dev_err(lcd->dev, "platform data is NULL.\n");
		return -EFAULT;
	}

	msleep(dpd->display_off_delay);

	if (!dpd->gpio_cfg_earlysuspend) {
		dev_err(lcd->dev, "gpio_cfg_earlysuspend is NULL.\n");
		return -EFAULT;
	} else
		dpd->gpio_cfg_earlysuspend();

	if (!dpd->power_on) {
		dev_err(lcd->dev, "power_on is NULL.\n");
		return -EFAULT;
	} else
		dpd->power_on(dpd, LCD_POWER_DOWN);

	return ret;
}

static int s6e63m0_power(struct s6e63m0 *lcd, int power)
{
	int ret = 0;

	switch (power) {
	case FB_BLANK_POWERDOWN:
		dev_dbg(lcd->dev, "%s(): Powering Off, was %s\n",__func__,
			(lcd->ddev->power_mode != MCDE_DISPLAY_PM_OFF) ? "ON" : "OFF");
		ret = s6e63m0_set_power_mode(lcd->ddev, MCDE_DISPLAY_PM_OFF);
		break;
	case FB_BLANK_NORMAL:
		dev_dbg(lcd->dev, "%s(): Into Sleep, was %s\n",__func__,
			(lcd->ddev->power_mode == MCDE_DISPLAY_PM_ON) ? "ON" : "SLEEP/OFF");
		ret = s6e63m0_set_power_mode(lcd->ddev, MCDE_DISPLAY_PM_STANDBY);
		break;
	case FB_BLANK_UNBLANK:
		dev_dbg(lcd->dev, "%s(): Exit Sleep, was %s\n",__func__,
			(lcd->ddev->power_mode == MCDE_DISPLAY_PM_STANDBY) ? "SLEEP" : "ON/OFF");
		ret = s6e63m0_set_power_mode(lcd->ddev, MCDE_DISPLAY_PM_ON);
		break;
	default:
		ret = -EINVAL;
		dev_info(lcd->dev, "Invalid power change request (%d)\n", power);
		break;
	}

	return ret;
}

static int s6e63m0_set_power(struct lcd_device *ld, int power)
{
	struct s6e63m0 *lcd = lcd_get_data(ld);

	if (power != FB_BLANK_UNBLANK && power != FB_BLANK_POWERDOWN &&
		power != FB_BLANK_NORMAL) {
		dev_err(lcd->dev, "power value should be 0, 1 or 4.\n");
		return -EINVAL;
	}

	return s6e63m0_power(lcd, power);
}

static int s6e63m0_get_power(struct lcd_device *ld)
{
	struct s6e63m0 *lcd = lcd_get_data(ld);

	int power;

	switch (lcd->ddev->power_mode) {
	case MCDE_DISPLAY_PM_OFF:
		power = FB_BLANK_POWERDOWN;
		break;
	case MCDE_DISPLAY_PM_STANDBY:
		power = FB_BLANK_NORMAL;
		break;
	case MCDE_DISPLAY_PM_ON:
		power = FB_BLANK_UNBLANK;
		break;
	default:
		power = -1;
		break;
	}
	return power;

}

static struct lcd_ops s6e63m0_lcd_ops = {
	.set_power = s6e63m0_set_power,
	.get_power = s6e63m0_get_power,
};


/* This structure defines all the properties of a backlight */
struct backlight_properties s6e63m0_backlight_props = {
	.brightness = MAX_REQ_BRIGHTNESS,
	.max_brightness = MAX_REQ_BRIGHTNESS,
	.type = BACKLIGHT_RAW,
};

static int s6e63m0_get_brightness(struct backlight_device *bd)
{
	dev_dbg(&bd->dev, "lcd get brightness returns %d\n", bd->props.brightness);
	return bd->props.brightness;
}

static int get_gamma_value_from_bl(int bl)
{
        int gamma_value =0;
        int gamma_val_x10 =0;

        if(bl >= MIN_BL){
                gamma_val_x10 = 10 *(MAX_GAMMA_VALUE-1)*bl/(MAX_BL-MIN_BL) + (10 - 10*(MAX_GAMMA_VALUE-1)*(MIN_BL)/(MAX_BL-MIN_BL));
                gamma_value=(gamma_val_x10 +5)/10;
        }else{
                gamma_value =0;
        }

        return gamma_value;
}

static int s6e63m0_set_brightness(struct backlight_device *bd)
{
	int ret = 0, bl = bd->props.brightness;
	struct s6e63m0 *lcd = bl_get_data(bd);

	if (bl < MIN_SUPP_BRIGHTNESS ||
		bl > bd->props.max_brightness) {
		dev_err(&bd->dev, "lcd brightness should be %d to %d.\n",
			MIN_SUPP_BRIGHTNESS, bd->props.max_brightness);
		return -EINVAL;
	}

	if (lcd->ldi_state) {
		ret = update_brightness(lcd,0);
		if (ret < 0)
			dev_err(&bd->dev, "update brightness failed.\n");
	}

	return ret;
}

static struct backlight_ops s6e63m0_backlight_ops  = {
	.get_brightness = s6e63m0_get_brightness,
	.update_status = s6e63m0_set_brightness,
};

static ssize_t s6e63m0_sysfs_store_lcd_power(struct device *dev,
                                       struct device_attribute *attr,
                                       const char *buf, size_t len)
{
        int rc;
        int lcd_enable;
	struct s6e63m0 *lcd = dev_get_drvdata(dev);

	dev_info(lcd->dev,"s6e63m0 lcd_sysfs_store_ldi_power\n");

        rc = strict_strtoul(buf, 0, (unsigned long *)&lcd_enable);
        if (rc < 0)
                return rc;

        if(lcd_enable) {
		s6e63m0_power(lcd, FB_BLANK_UNBLANK);
        }
        else {
		s6e63m0_power(lcd, FB_BLANK_POWERDOWN);
        }

        return len;
}

static DEVICE_ATTR(ldi_power, 0644,
                NULL, s6e63m0_sysfs_store_lcd_power);

static ssize_t lcd_type_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	char temp[20];
	sprintf(temp, "SMD_AMS397GEXX\n");
	strcat(buf, temp);
	return strlen(buf);
}
static DEVICE_ATTR(lcd_type, 0444, lcd_type_show, NULL);

static ssize_t power_reduce_show(struct device *dev, struct
device_attribute *attr, char *buf)
{
	struct s6e63m0 *lcd = dev_get_drvdata(dev);
	char temp[3];

	sprintf(temp, "%d\n", lcd->acl_enable);
	strcpy(buf, temp);

	return strlen(buf);
}
static ssize_t power_reduce_store(struct device *dev, struct
device_attribute *attr, const char *buf, size_t size)
{
	struct s6e63m0 *lcd = dev_get_drvdata(dev);
	int value;
	int rc;

	/*Protection code for  power on /off test */
	if(lcd->ddev <= 0)
		return size;

	rc = strict_strtoul(buf, (unsigned int) 0, (unsigned long *)&value);
	if (rc < 0)
		return rc;
	else{
		pr_info("[S6E63M0] Power reduce %d ==> %d\n", lcd->acl_enable, value);
		if (lcd->acl_enable != value) {
			lcd->acl_enable = value;
			if (lcd->ldi_state)
				s6e63m0_set_acl(lcd);
		}
		return size;
	}
}

static DEVICE_ATTR(power_reduce, 0664,
		power_reduce_show, power_reduce_store);

static ssize_t auto_brightness_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct s6e63m0 *lcd = dev_get_drvdata(dev);
	char temp[3];

	sprintf(temp, "%d\n", lcd->auto_brightness);
	strcpy(buf, temp);

	return strlen(buf);
}

static ssize_t auto_brightness_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct s6e63m0 *lcd = dev_get_drvdata(dev);
	int value;
	int rc;

	rc = strict_strtoul(buf, (unsigned int)0, (unsigned long *)&value);
	if (rc < 0)
		return rc;
	else {
		if (lcd->auto_brightness != value) {
			pr_err("[S6E63M0] Auto Brightness %d ==> %d\n", lcd->auto_brightness, value);
			mutex_lock(&lcd->lcd_lock);
			lcd->auto_brightness = value;
			mutex_unlock(&lcd->lcd_lock);
			if (lcd->ldi_state)
				update_brightness(lcd, 0);
		}
	}
	return size;
}

static DEVICE_ATTR(auto_brightness, 0644, auto_brightness_show, auto_brightness_store);

unsigned short SEQ_CUSTOM_ELVSS[] = {
	0xB2, 0x0,
	DATA_ONLY, 0x0,
	DATA_ONLY, 0x0,
	DATA_ONLY, 0x0,

	0xB1, 0x0B,

	ENDDEF, 0x00
};

int custom_elvss_cal(struct s6e63m0 *lcd, int val)
{
	int data, cnt;

	data = val;

	for (cnt = 1; cnt <= 7; cnt += 2)
		SEQ_CUSTOM_ELVSS[cnt] = data;

	return s6e63m0_panel_send_sequence(lcd, SEQ_CUSTOM_ELVSS);
}

static ssize_t s6e63m0_sysfs_show_gamma_mode(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct s6e63m0 *lcd = dev_get_drvdata(dev);

	sprintf(buf, "Current: %s\n\n", lcd->gamma_mode ? "1.9" : "2.2");
	sprintf(buf, "%s[0][%s] 2.2 mode\n", buf, lcd->gamma_mode ? " " : "*");
	sprintf(buf, "%s[1][%s] 1.9 mode\n", buf, lcd->gamma_mode ? "*" : " ");

	return strlen(buf);
}

static ssize_t s6e63m0_sysfs_store_gamma_mode(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	struct s6e63m0 *lcd = dev_get_drvdata(dev);
	int rc;

	rc = strict_strtoul(buf, 0, (unsigned long *)&lcd->gamma_mode);
	if (rc < 0)
		return rc;

	if (lcd->gamma_mode > 1)
	{
		lcd->gamma_mode = 0;
		dev_err(dev, "there are only 2 types of gamma mode(0:2.2, 1:1.9)\n");
	}
	else
		pr_info("[S6E63M0] Gamma mode ==> %d\n", lcd->gamma_mode);

	if (lcd->ldi_state)
	{
		if((lcd->current_brightness == lcd->bl) && (lcd->current_gamma_mode == lcd->gamma_mode))
			printk("there is no gamma_mode & brightness changed\n");
		else
			s6e63m0_gamma_ctl(lcd);
	}
	return len;
}

static DEVICE_ATTR(gamma_mode, 0644,
		s6e63m0_sysfs_show_gamma_mode, s6e63m0_sysfs_store_gamma_mode);

static ssize_t s6e63m0_sysfs_show_update_brightness(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct s6e63m0 *lcd = dev_get_drvdata(dev);
	
	update_brightness(lcd, 1);

	return sprintf(buf, "Updating brightness...\n");
}

static ssize_t s6e63m0_sysfs_store_update_brightness(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	struct s6e63m0 *lcd = dev_get_drvdata(dev);

	update_brightness(lcd, 1);

	return len;
}
static DEVICE_ATTR(brightness_update, 0644,
		s6e63m0_sysfs_show_update_brightness, s6e63m0_sysfs_store_update_brightness);

static int s6e63m0_set_power_mode(struct mcde_display_device *ddev,
	enum mcde_display_power_mode power_mode)
{
	struct s6e63m0 *lcd = dev_get_drvdata(&ddev->dev);
	struct ssg_dpi_display_platform_data *dpd = NULL;
	enum mcde_display_power_mode orig_mode = ddev->power_mode;

	int ret = 0;

	mutex_lock(&lcd->pwr_lock);

	dpd = lcd->pd;
	if (!dpd) {
		dev_err(lcd->dev, "s6e63m0 platform data is NULL.\n");
		return -EFAULT;
	}

	dev_dbg(&ddev->dev, "s6e63m0_power_mode = [%d]-->[%d]\n",ddev->power_mode, power_mode);

	/* OFF -> STANDBY or OFF -> ON */
	if (ddev->power_mode == MCDE_DISPLAY_PM_OFF &&
					power_mode != MCDE_DISPLAY_PM_OFF) {
					
		ret |= s6e63m0_power_on(lcd);

		ret |= s6e63m0_ldi_init(lcd);

		if (ret)
			goto err;

		ddev->power_mode = MCDE_DISPLAY_PM_STANDBY;
	}

	/* STANDBY -> ON */
	if (ddev->power_mode == MCDE_DISPLAY_PM_STANDBY &&
					power_mode == MCDE_DISPLAY_PM_ON) {

		if (lcd->justStarted) {
			lcd->justStarted = false;
			mcde_chnl_disable(ddev->chnl_state);
			if (lcd->pd->reset_gpio) {
				dpd->reset(dpd);
				msleep(dpd->reset_delay);
			}
			ret = s6e63m0_ldi_init(lcd);
			mcde_formatter_enable(ddev->chnl_state);
		}

		ret = s6e63m0_ldi_enable(lcd);
		if (ret)
			goto err;

		ddev->power_mode = MCDE_DISPLAY_PM_ON;
		
	}
	/* ON -> STANDBY */
	else if (ddev->power_mode == MCDE_DISPLAY_PM_ON &&
					power_mode <= MCDE_DISPLAY_PM_STANDBY) {

		ret = s6e63m0_ldi_disable(lcd);
		if (ret && (power_mode != MCDE_DISPLAY_PM_OFF))
			goto err;
		ddev->power_mode = MCDE_DISPLAY_PM_STANDBY;
	}

	/* STANDBY -> OFF */
	if (ddev->power_mode == MCDE_DISPLAY_PM_STANDBY &&
					power_mode == MCDE_DISPLAY_PM_OFF) {
					
		ret = s6e63m0_power_off(lcd);
		if (ret)
			goto err;

		ddev->power_mode = MCDE_DISPLAY_PM_OFF;
	}

	if (orig_mode != ddev->power_mode)
		pr_err("[S6E63M0] Power from mode %d to %d\n",
			orig_mode, ddev->power_mode);

	ret =  mcde_chnl_set_power_mode(ddev->chnl_state, ddev->power_mode);

err:
	mutex_unlock(&lcd->pwr_lock);
	return ret;
	
}

static int __devinit s6e63m0_spi_probe(struct spi_device *spi)
{
	int ret = 0;
	struct s6e63m0 *lcd = container_of(spi->dev.driver, struct s6e63m0, spi_drv.driver);
	#ifdef SMART_DIMMING
	u8 lcd_id[3];
	#endif

	dev_dbg(&spi->dev, "panel s6e63m0 spi being probed\n");

	dev_set_drvdata(&spi->dev, lcd);

	/* s6e63m0 lcd panel uses 3-wire 9bits SPI Mode. */
	spi->bits_per_word = 9;

	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(&spi->dev, "spi setup failed.\n");
		goto out;
	}

	lcd->spi = spi;

	/*
	 * if lcd panel was on from bootloader like u-boot then
	 * do not lcd on.
	 */
	if (!lcd->pd->platform_enabled) {
		/*
		 * if lcd panel was off from bootloader then
		 * current lcd status is powerdown and then
		 * it enables lcd panel.
		 */

		s6e63m0_power(lcd,FB_BLANK_UNBLANK);

	} else {
		lcd->ldi_state = LDI_STATE_ON;
	}

	/* force acl on */
	s6e63m0_panel_send_sequence(lcd, ACL_cutoff_set[7]);
	dev_dbg(&spi->dev, "s6e63m0 spi has been probed.\n");

	#ifdef SMART_DIMMING
	s6e63m0_read_panel_id(lcd, lcd_id);
	pr_info("[S6E63M0] LCD ID [%#04X]\n", lcd_id[1]);
//	pr_info("[S6E63M0] smart dimming lcd id: %#04X\n", SMART_MTP_PANEL_ID);
//	if (lcd_id[1] >= SMART_MTP_PANEL_ID) {
		if (!is_load_mtp_offset) {
		#if 0
			s6e63mo_read_mtp_info(lcd);
		#else
			s6e63mo_mtp_from_boot(lcd, mtp_data_from_boot);
		#endif
			is_load_mtp_offset =  1;
		}
//	}
	#endif

out:
	return ret;
}

#define ATTR_RO(_name)	\
	static struct kobj_attribute _name##_interface = __ATTR(_name, 0444, _name##_show, NULL);

#define ATTR_WO(_name)	\
	static struct kobj_attribute _name##_interface = __ATTR(_name, 0220, NULL, _name##_store);

#define ATTR_RW(_name)	\
	static struct kobj_attribute _name##_interface = __ATTR(_name, 0644, _name##_show, _name##_store);

static ssize_t lcd_id_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct s6e63m0 *lcd = plcd;
	u8 lcd_id[3];

	s6e63m0_read_panel_id(lcd, lcd_id);

	return sprintf(buf, "%#04X\n", lcd_id[1]);
}

ATTR_RO(lcd_id);

static ssize_t elvss_table_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int i;

	sprintf(buf, "Custom ELVSS table:\n\n");

	for (i = 1; i <= 7; i += 2) {
		sprintf(buf, "%s[%02d]\t%#04X\n", buf, i, SEQ_CUSTOM_ELVSS[i]);
	}

	sprintf(buf, "%s\nDynamic ELVSS table:\n\n", buf);

	for (i = 1; i <= 7; i += 2) {
		sprintf(buf, "%s[%02d]\t%#04X\n", buf, i, SEQ_DYNAMIC_ELVSS[i]);
	}
	
	return strlen(buf);
}

static ssize_t elvss_table_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct s6e63m0 *lcd = plcd;
	u32 val;

	if (sscanf(buf, "%x", &val))
		custom_elvss_cal(lcd, val);

	return count;
}

ATTR_RW(elvss_table);

static ssize_t mcde_chnl_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct s6e63m0 *lcd = plcd;

	sprintf(buf,   "[S6E63M0 MCDE Channel]\n");
	sprintf(buf, "%spixclock: %d\n", buf, lcd->ddev->video_mode.pixclock);
	sprintf(buf, "%shbp: %d\n", buf, lcd->ddev->video_mode.hbp);
	sprintf(buf, "%shfp: %d\n", buf, lcd->ddev->video_mode.hfp);
	sprintf(buf, "%shsw: %d\n", buf, lcd->ddev->video_mode.hsw);
	sprintf(buf, "%svbp: %d\n", buf, lcd->ddev->video_mode.vbp);
	sprintf(buf, "%svfp: %d\n", buf, lcd->ddev->video_mode.vfp);
	sprintf(buf, "%svsw: %d\n", buf, lcd->ddev->video_mode.vsw);
	sprintf(buf, "%sinterlaced: %d\n", buf, lcd->ddev->video_mode.interlaced);

	return strlen(buf);
}

static ssize_t mcde_chnl_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct s6e63m0 *lcd = plcd;
	int ret;
	u32 pclk;	/* pixel clock in ps (pico seconds) */
	u32 hbp;	/* horizontal back porch: left margin (excl. hsync) */
	u32 hfp;	/* horizontal front porch: right margin (excl. hsync) */
	u32 hsw;	/* horizontal sync width */
	u32 vbp;	/* vertical back porch: upper margin (excl. vsync) */
	u32 vfp;	/* vertical front porch: lower margin (excl. vsync) */
	u32 vsw;
	u32 interlaced;
	u32 enable;

	if (!strncmp(buf, "set_vmode", 8))
	{
		pr_err("[S6E63M0] Save chnl params\n");
		mcde_chnl_set_video_mode(lcd->ddev->chnl_state, &lcd->ddev->video_mode);

		return count;
	}

	if (!strncmp(buf, "apply_config", 8)) 
	{
		pr_err("[S6E63M0] Apply chnl config!\n");
		mcde_chnl_apply(lcd->ddev->chnl_state);
		
		return count;
	}

	if (!strncmp(buf, "stop_flow", 8)) 
	{
		pr_err("[S6E63M0] MCDE chnl stop flow!\n");
		mcde_chnl_stop_flow(lcd->ddev->chnl_state);
		
		return count;
	}

	if (!strncmp(buf, "update", 6)) 
	{
		pr_err("[S6E63M0] Update MCDE chnl!\n");
		mcde_chnl_set_video_mode(lcd->ddev->chnl_state, &lcd->ddev->video_mode);
		mcde_chnl_apply(lcd->ddev->chnl_state);
		mcde_chnl_stop_flow(lcd->ddev->chnl_state);
		
		return count;
	}

	if (!strncmp(&buf[0], "enable=", 7))
	{
		sscanf(&buf[7], "%d", &enable);
		pr_err("[S6E63M0] %s chnl\n", enable ? "Enable" : "Disable");

		if (!enable)
			mcde_chnl_disable(lcd->ddev->chnl_state);
		else
			mcde_chnl_enable(lcd->ddev->chnl_state);
		
		return count;
	}
	
	if (!strncmp(&buf[0], "pclk=", 5))
	{
		ret = sscanf(&buf[5], "%d", &pclk);
		if (!ret) {
			pr_err("[S6E63M0] Invaild param\n");
	
			return -EINVAL;
		}

		pr_err("[S6E63M0] pclk: %d\n", pclk);
		lcd->ddev->video_mode.pixclock = pclk;

		return count;
	}

	if (!strncmp(&buf[0], "hbp=", 4))
	{
		ret = sscanf(&buf[4], "%d", &hbp);
		if (!ret) {
			pr_err("[S6E63M0] Invaild param\n");
	
			return -EINVAL;
		}

		pr_err("[S6E63M0] hbp: %d\n", hbp);
		lcd->ddev->video_mode.hbp = hbp;

		return count;
	}

	if (!strncmp(&buf[0], "hfp=", 4))
	{
		ret = sscanf(&buf[4], "%d", &hfp);
		if (!ret) {
			pr_err("[S6E63M0] Invaild param\n");
	
			return -EINVAL;
		}

		pr_err("[S6E63M0] hfp: %d\n", hfp);
		lcd->ddev->video_mode.hfp = hfp;

		return count;
	}

	if (!strncmp(&buf[0], "hsw=", 4))
	{
		ret = sscanf(&buf[4], "%d", &hsw);
		if (!ret) {
			pr_err("[S6E63M0] Invaild param\n");
	
			return -EINVAL;
		}

		pr_err("[S6E63M0] hsw: %d\n", hsw);
		lcd->ddev->video_mode.hsw = hsw;

		return count;
	}

	if (!strncmp(&buf[0], "vbp=", 4))
	{
		ret = sscanf(&buf[4], "%d", &vbp);
		if (!ret) {
			pr_err("[S6E63M0] Invaild param\n");
	
			return -EINVAL;
		}

		pr_err("[S6E63M0] vbp: %d\n", vbp);
		lcd->ddev->video_mode.vbp = vbp;

		return count;
	}

	if (!strncmp(&buf[0], "vfp=", 4))
	{
		ret = sscanf(&buf[4], "%d", &vfp);
		if (!ret) {
			pr_err("[S6E63M0] Invaild param\n");
	
			return -EINVAL;
		}

		pr_err("[S6E63M0] vfp: %d\n", vfp);
		lcd->ddev->video_mode.vfp = vfp;

		return count;
	}

	if (!strncmp(&buf[0], "vsw=", 4))
	{
		ret = sscanf(&buf[4], "%d", &vsw);
		if (!ret) {
			pr_err("[S6E63M0] Invaild param\n");
	
			return -EINVAL;
		}

		pr_err("[S6E63M0] vsw: %d\n", vsw);
		lcd->ddev->video_mode.vsw = vsw;

		return count;
	}

	if (!strncmp(&buf[0], "interlaced=", 11))
	{
		ret = sscanf(&buf[11], "%d", &interlaced);
		if (!ret) {
			pr_err("[S6E63M0] Invaild param\n");
	
			return -EINVAL;
		}

		pr_err("[S6E63M0] interlaced: %d\n", interlaced);
		lcd->ddev->video_mode.interlaced = interlaced;

		return count;
	}

	pr_err("[S6E63M0] Invaild cmd\n");

	return count;
}

ATTR_RW(mcde_chnl);

static ssize_t lcd_clk_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int i;
	bool matched;

	sprintf(buf, "Current: %s\n\n", lcdclk_prop[lcdclk_usr].name);

	for (i = 0; i <= 3; i++) {
		if (i == lcdclk_usr)
			matched = true;
		else
			matched = false;

		sprintf(buf, "%s[%d][%s] %s\n", buf, i, matched ? "*" : " ", lcdclk_prop[i].name);
	}

	return strlen(buf);
}

static ssize_t lcd_clk_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret, tmp;

	ret = sscanf(buf, "%d", &tmp);
	if (!ret || (tmp < 0) || (tmp > 3)) {
		pr_err("[S6E63M0] Bad cmd\n");
		return -EINVAL;
	}

	lcdclk_usr = tmp;

	schedule_work(&s6e63m0_lcdclk_work);

	return count;
}

ATTR_RW(lcd_clk);

static ssize_t illumination_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	sprintf(buf, "Illumination Control [%s]\n", illumination_req ? "*" : " ");
	sprintf(buf, "%sIllumination: %d\n", buf, illumination_val);

	return strlen(buf);
}

static ssize_t illumination_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct s6e63m0 *lcd = plcd;
	int buf_val;

	if (sysfs_streq(buf, "reset") || sysfs_streq(buf, "off")) {
		illumination_req = false;
		update_brightness(lcd, 1);

		return count;
	}

	if (!sscanf(buf, "%d", &buf_val)) {
		pr_err("[S6E63M0] invalid inputs!\n");
		return -EINVAL;
	}

	illumination_req = true;
	illumination_val = buf_val;

	pr_info("[S6E63M0] Illumination [%d]\n", illumination_val);

	update_brightness(lcd, 1);

	return count;
}

ATTR_RW(illumination);

static ssize_t illumination_table_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct s6e63m0 *lcd = plcd;
	int i;
	
	sprintf(buf, "Illumination table:\n");
	sprintf(buf, "%sCurrent: [%02d] %03d\n\n", buf, lcd->bl, illumination_table[lcd->bl]);
	for (i = 0; i <= 25; i++) {
		sprintf(buf, "%s[%02d]\t\t%03d\n", buf, i, illumination_table[i]);
	}
	
	return strlen(buf);
}

static ssize_t illumination_table_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct s6e63m0 *lcd = plcd;
	int num, val;

	if (sscanf(buf, "%d %d", &num, &val) != 2) {
		pr_err("[S6E63M0] invalid inputs!\n");
		return -EINVAL;
	}

	if (num < 0 || num > 25) {
		pr_err("[S6E63M0] invalid range!\n");
		return -EINVAL;
	}

	if (num != 0 && (illumination_table[num - 1] > val)) {
		pr_err("[S6E63M0] the value inputed should be larger than the prev\n");
		return -EINVAL;
	}

	if (num != 25 && (illumination_table[num + 1] < val)) {
		pr_err("[S6E63M0] the value inputed should be smaller than the next\n");
		return -EINVAL;
	}

	pr_info("[S6E63M0] illumination table [%02d] %02d -> %02d\n", num, illumination_table[num], val);
	illumination_table[num] = val;

	update_brightness(lcd, 1);

	return count;
}

ATTR_RW(illumination_table);

static ssize_t gamma_mode_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct s6e63m0 *lcd = plcd;

	sprintf(buf, "Current mode: %s\n\n", lcd->gamma_mode ? "1.9" : "2.2");
	sprintf(buf, "%s[0][%s] 2.2 mode\n", buf, lcd->gamma_mode ? " " : "*");
	sprintf(buf, "%s[1][%s] 1.9 mode\n", buf, lcd->gamma_mode ? "*" : " ");

	return strlen(buf);
}

static ssize_t gamma_mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct s6e63m0 *lcd = plcd;
	int rc;

	rc = strict_strtoul(buf, 0, (unsigned long *)&lcd->gamma_mode);
	if (rc < 0)
		return rc;

	if (lcd->gamma_mode > 1)
	{
		lcd->gamma_mode = 0;
	}
	
	pr_info("[S6E63M0] Gamma mode ==> %d\n", lcd->gamma_mode);

	if (lcd->ldi_state)
	{
		if((lcd->current_brightness == lcd->bl) && (lcd->current_gamma_mode == lcd->gamma_mode))
			pr_info("[S6E63M0] Gamma mode no changed\n");
		else
			s6e63m0_gamma_ctl(lcd);
	}

	return count;
}

ATTR_RW(gamma_mode);

static ssize_t gamma_table_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct s6e63m0 *lcd = plcd;

	int i;

	if (!lcd->gamma_mode) {					/* 2.2 mode */
		if (is_load_mtp_offset && !gamma_table_req) {	/* smart dimming */
			sprintf(buf, "Gamma mode: 2.2 (Smart Dimming)\n");
			sprintf(buf, "%sGamma level: %02d\n\n", buf, lcd->bl);
			for (i = 3; i < ((gen_table_max * 2) + 3); i += 2) {
				sprintf(buf, "%s[%02d]\t\t%#04X\n", buf, i, s6e63m0_22_gamma_table[i]);
			}
		} else if (!gamma_table_req) {
			sprintf(buf, "Gamma mode: 2.2\n");
			sprintf(buf, "%sGamma level: %02d\n\n", buf, lcd->bl);
			for (i = 3; i < ((gen_table_max * 2) + 3); i += 2) {
				sprintf(buf, "%s[%02d]\t\t%#04X\n", buf, i, (unsigned int)gamma_table.gamma_22_table[lcd->bl][i]);
			}
		}
	} else {						/* 1.9 mode */
		sprintf(buf, "Gamma mode 1.9\n");
		sprintf(buf, "%sGamma level: %02d\n\n", buf, lcd->bl);
		for (i = 3; i < ((gen_table_max * 2) + 3); i += 2) {
			sprintf(buf, "%s[%02d]\t\t%#04X\n", buf, i, (unsigned int)gamma_table.gamma_19_table[lcd->bl][i]);
		}
	}

	sprintf(buf, "%s\nCustom Gamma Table:\n\n", buf);
	for (i = 3; i < ((gen_table_max * 2) + 3); i += 2) {
		sprintf(buf, "%s[%02d]\t\t%#04X\n", buf, i, (unsigned int)s6e63m0_22_gamma_table_custom[i]);
	}

	return strlen(buf);
}

static ssize_t gamma_table_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct s6e63m0 *lcd = plcd;

	int idx, val, i;

	if (sysfs_streq(buf, "on")) {
		gamma_table_req = true;
		s6e63m0_gamma_ctl(lcd);

		return count;
	}

	if (sysfs_streq(buf, "off")) {
		gamma_table_req = false;
		s6e63m0_gamma_ctl(lcd);

		return count;
	}

	if (sysfs_streq(buf, "copy")) {
		for(i=3; i < ((gen_table_max * 2) + 3); i+=2) {
			s6e63m0_22_gamma_table_custom[i] = s6e63m0_22_gamma_table[i];
		}

		return count;
	}

	if (sscanf(buf, "%d %x", &idx, &val) == 2) {
		s6e63m0_22_gamma_table_custom[idx] = val;

		return count;
	}

	return count;
}
ATTR_RW(gamma_table);

static ssize_t main_R_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{	
	return sprintf(buf, "%d\n", s6e63m0_22_gamma_table[R_OFFSET]);
}
static ssize_t main_R_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct s6e63m0 *lcd = plcd;

	int buf_val;
	int buf_R;

	int ret;

	if (lcd->gamma_mode || !is_load_mtp_offset) {
		pr_info("[S6E63M0] not proper gamma mode\n");
		return -EINVAL;
	}

	if (!strncmp(buf, "reset", 5)) {
		pr_info("[S6E63M0] reset R filter\n");

		R_req = false;
		s6e63m0_gamma_ctl(lcd);

		return count;
	}

	ret = sscanf(buf, "%d", &buf_val);

	if (!ret) {
		pr_info("[S6E63M0] invalid input\n");
	}

	if (buf_val >= GAMMA_VAL_MIN && buf_val <= GAMMA_VAL_MAX) {
		buf_R = s6e63m0_22_gamma_table[R_OFFSET];

		R_val = buf_val;
		R_req = true;

		s6e63m0_gamma_ctl(lcd);

		pr_info("[S6E63M0] [R] %#04X -> %#04X\n", buf_R, s6e63m0_22_gamma_table[R_OFFSET]);
	} else {
		pr_info("[S6E63M0] invalid input\n");
	}

	return count;
}

ATTR_RW(main_R);

static ssize_t main_G_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{	
	return sprintf(buf, "%d\n", s6e63m0_22_gamma_table[G_OFFSET]);
}
static ssize_t main_G_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct s6e63m0 *lcd = plcd;

	int buf_val;
	int buf_G;

	int ret;

	if (lcd->gamma_mode || !is_load_mtp_offset) {
		pr_info("[S6E63M0] not proper gamma mode\n");
		return -EINVAL;
	}

	if (!strncmp(buf, "reset", 5)) {
		pr_info("[S6E63M0] reset G filter\n");

		G_req = false;
		s6e63m0_gamma_ctl(lcd);

		return count;
	}

	ret = sscanf(buf, "%d", &buf_val);

	if (!ret) {
		pr_info("[S6E63M0] invalid input\n");
	}

	if (buf_val >= GAMMA_VAL_MIN && buf_val <= GAMMA_VAL_MAX) {
		buf_G = s6e63m0_22_gamma_table[G_OFFSET];

		G_val = buf_val;
		G_req = true;

		s6e63m0_gamma_ctl(lcd);

		pr_info("[S6E63M0] [G] %#04X -> %#04X\n", buf_G, s6e63m0_22_gamma_table[G_OFFSET]);
	} else {
		pr_info("[S6E63M0] invalid input\n");
	}

	return count;
}

ATTR_RW(main_G);

static ssize_t main_B_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{	
	return sprintf(buf, "%d\n", s6e63m0_22_gamma_table[B_OFFSET]);
}
static ssize_t main_B_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct s6e63m0 *lcd = plcd;

	int buf_val;
	int buf_B;

	int ret;

	if (lcd->gamma_mode || !is_load_mtp_offset) {
		pr_info("[S6E63M0] not proper gamma mode\n");
		return -EINVAL;
	}

	if (!strncmp(buf, "reset", 5)) {
		pr_info("[S6E63M0] reset B filter\n");

		B_req = false;
		s6e63m0_gamma_ctl(lcd);

		return count;
	}

	ret = sscanf(buf, "%d", &buf_val);

	if (!ret) {
		pr_info("[S6E63M0] invalid input\n");
	}

	if (buf_val >= GAMMA_VAL_MIN && buf_val <= GAMMA_VAL_MAX) {
		buf_B = s6e63m0_22_gamma_table[B_OFFSET];

		B_val = buf_val;
		B_req = true;

		s6e63m0_gamma_ctl(lcd);

		pr_info("[S6E63M0] [B] %#04X -> %#04X\n", buf_B, s6e63m0_22_gamma_table[B_OFFSET]);
	} else {
		pr_info("[S6E63M0] invalid input\n");
	}

	return count;
}
ATTR_RW(main_B);

static ssize_t night_mode_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", night_mode ? "on" : "off");
}

static ssize_t night_mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct s6e63m0 *lcd = plcd;
	int i;

	if (sysfs_streq(buf, "on")) {
		if (sunlight_mode)
			sunlight_mode = false;

		for(i = 3; i < ((gen_table_max * 2) + 3); i += 2) {
			s6e63m0_22_gamma_table_custom[i] = s6e63m0_gamma_table_night[i];
		}

		night_mode = true;
		gamma_table_req = true;
		s6e63m0_gamma_ctl(lcd);

		return count;
	}

	if (sysfs_streq(buf, "off")) {
		if (!night_mode)
			return count;

		night_mode = false;
		gamma_table_req = false;
		s6e63m0_gamma_ctl(lcd);

		return count;
	}

	return count;
}

ATTR_RW(night_mode);

static ssize_t sunlight_mode_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", sunlight_mode ? "on" : "off");
}

static ssize_t sunlight_mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct s6e63m0 *lcd = plcd;
	int i;

	if (sysfs_streq(buf, "on")) {
		if (night_mode)
			night_mode = false;

		for(i = 3; i < ((gen_table_max * 2) + 3); i += 2) {
			s6e63m0_22_gamma_table_custom[i] = s6e63m0_gamma_table_sunlight[i];
		}

		sunlight_mode = true;
		gamma_table_req = true;
		s6e63m0_gamma_ctl(lcd);

		return count;
	}

	if (sysfs_streq(buf, "off")) {
		if (!sunlight_mode)
			return count;

		sunlight_mode = false;
		gamma_table_req = false;
		s6e63m0_gamma_ctl(lcd);

		return count;
	}

	return count;
}

ATTR_RW(sunlight_mode);

static struct attribute *s6e63m0_panel_attrs[] = {
	&lcd_id_interface.attr, 
	&lcd_clk_interface.attr, 
	&elvss_table_interface.attr, 
	&mcde_chnl_interface.attr, 
	&illumination_interface.attr, 
	&illumination_table_interface.attr, 
	NULL, 
};


static struct attribute *s6e63m0_gamma_attrs[] = {
	&gamma_mode_interface.attr, 
	&gamma_table_interface.attr, 
	NULL, 
};


static struct attribute *s6e63m0_color_attrs[] = {
	&main_R_interface.attr, 
	&main_G_interface.attr, 
	&main_B_interface.attr, 
	NULL, 
};


static struct attribute *s6e63m0_attrs[] = {
	&night_mode_interface.attr, 
	&sunlight_mode_interface.attr, 
	NULL, 
};

static struct attribute_group s6e63m0_panel_interface_group = {
	.attrs = s6e63m0_panel_attrs, 
	.name  = "panel" 
};

static struct attribute_group s6e63m0_gamma_interface_group = {
	.attrs = s6e63m0_gamma_attrs, 
	.name  = "gamma" 
};


static struct attribute_group s6e63m0_color_interface_group = {
	.attrs = s6e63m0_color_attrs, 
	.name  = "color" 
};

static struct attribute_group s6e63m0_interface_group = {
	.attrs = s6e63m0_attrs,
};

static struct kobject *s6e63m0_kobject;

static int __devinit s6e63m0_mcde_panel_probe(struct mcde_display_device *ddev)
{
	int ret = 0;
	struct s6e63m0 *lcd = NULL;
	struct backlight_device *bd = NULL;
	struct ssg_dpi_display_platform_data *pdata = ddev->dev.platform_data;

	dev_dbg(&ddev->dev, "Invoked %s\n", __func__);

	if (pdata == NULL) {
		dev_err(&ddev->dev, "%s:Platform data missing\n", __func__);
		ret = -EINVAL;
		goto no_pdata;
	}

	if (ddev->port->type != MCDE_PORTTYPE_DPI) {
		dev_err(&ddev->dev,
			"%s:Invalid port type %d\n",
			__func__, ddev->port->type);
		ret = -EINVAL;
		goto invalid_port_type;
	}

	ddev->set_power_mode = s6e63m0_set_power_mode;
	ddev->try_video_mode = try_video_mode;
	ddev->set_video_mode = set_video_mode;
	ddev->set_rotation = s6e63m0_set_rotation;
	ddev->update = s6e63m0_display_update;
	ddev->apply_config = s6e63m0_apply_config;

	lcd = kzalloc(sizeof(struct s6e63m0), GFP_KERNEL);
	if (!lcd)
		return -ENOMEM;

	dev_set_drvdata(&ddev->dev, lcd);
	lcd->ddev = ddev;
	lcd->dev = &ddev->dev;
	lcd->pd = pdata;
	lcd->auto_brightness = 0;
	lcd->justStarted = true;

#ifdef CONFIG_LCD_CLASS_DEVICE
	lcd->ld = lcd_device_register("panel", &ddev->dev,
					lcd, &s6e63m0_lcd_ops);
	if (IS_ERR(lcd->ld)) {
		ret = PTR_ERR(lcd->ld);
		goto out_free_lcd;
	}else {
		if(device_create_file(&(lcd->ld->dev), &dev_attr_lcd_type) < 0) {
			dev_err(&(lcd->ld->dev), "failed to add panel_type sysfs entries\n");
		}
	}
#endif


	mutex_init(&lcd->lock);
	mutex_init(&lcd->lcd_lock);
	mutex_init(&lcd->pwr_lock);

	bd = backlight_device_register("panel",
					&ddev->dev,
					lcd,
					&s6e63m0_backlight_ops,
					&s6e63m0_backlight_props);
	if (IS_ERR(bd)) {
		ret =  PTR_ERR(bd);
		goto out_backlight_unregister;
	}
	lcd->bd = bd;
	lcd->bd->props.max_brightness = MAX_BRIGHTNESS;
	lcd->bd->props.brightness = DEFAULT_BRIGHTNESS;
	lcd->bl = DEFAULT_GAMMA_LEVEL;
	lcd->current_brightness = DEFAULT_GAMMA_LEVEL;
	lcd->rotation = MCDE_DISPLAY_ROT_0;	
	lcd->acl_enable = 0;
	lcd->cur_acl = 0;
	lcd->panel_id = 0;
	lcd->elvss_ref = 0;

	/*
	 * it gets gamma table count available so it lets user
	 * know that.
	 */
	lcd->gamma_table_count = sizeof(gamma_table) / (MAX_GAMMA_LEVEL * sizeof(int));

        ret = device_create_file(&(lcd->ld->dev), &dev_attr_ldi_power);
        if (ret < 0)
                dev_err(&(ddev->dev), "failed to add ldi_power sysfs entries\n");

	ret = device_create_file(&(lcd->ld->dev), &dev_attr_power_reduce);
        if (ret < 0)
                dev_err(&(ddev->dev), "failed to add acl_set sysfs entries\n");
	
	ret = device_create_file(&lcd->bd->dev, &dev_attr_auto_brightness);
	if (ret < 0)
		dev_err(&lcd->ld->dev, "failed to add sysfs entries\n");

	ret = device_create_file(&lcd->bd->dev, &dev_attr_brightness_update);
	if (ret < 0)
		dev_err(&lcd->ld->dev, "failed to add sysfs entries\n");

	ret = device_create_file(&(ddev->dev), &dev_attr_gamma_mode);
	if (ret < 0)
		dev_err(&(ddev->dev), "failed to add sysfs entries\n");

	lcd->spi_drv.driver.name	= "pri_lcd_spi";
	lcd->spi_drv.driver.bus		= &spi_bus_type;
	lcd->spi_drv.driver.owner	= THIS_MODULE;
	lcd->spi_drv.probe		= s6e63m0_spi_probe;
	ret = spi_register_driver(&lcd->spi_drv);
	if (ret < 0) {
		dev_err(&(ddev->dev), "Failed to register SPI driver");
		goto out_backlight_unregister;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	lcd->earlysuspend.level   = EARLY_SUSPEND_LEVEL_DISABLE_FB - 2;
	lcd->earlysuspend.suspend = s6e63m0_mcde_panel_early_suspend;
	lcd->earlysuspend.resume  = s6e63m0_mcde_panel_late_resume;
	register_early_suspend(&lcd->earlysuspend);
#endif
	plcd = lcd;

	s6e63m0_kobject = kobject_create_and_add("s6e63m0", kernel_kobj);
	if (!s6e63m0_kobject) {
		pr_err("[S6E63M0] Failed to create kobject interface\n");
	}
	ret = sysfs_create_group(s6e63m0_kobject, &s6e63m0_interface_group);
	ret = sysfs_create_group(s6e63m0_kobject, &s6e63m0_color_interface_group);
	ret = sysfs_create_group(s6e63m0_kobject, &s6e63m0_gamma_interface_group);
	ret = sysfs_create_group(s6e63m0_kobject, &s6e63m0_panel_interface_group);
	if (ret) {
		kobject_put(s6e63m0_kobject);
	}

	//when screen is on, APE_OPP 25 sometimes messes it up
	//TODO change these to add/update/remove
	if (prcmu_qos_add_requirement(PRCMU_QOS_APE_OPP,
			"janice_lcd_dpi", 50)) {
		pr_info("pcrm_qos_add APE failed\n");
	}

	if (prcmu_qos_add_requirement(PRCMU_QOS_DDR_OPP,
			"janice_lcd_dpi", 50)) {
		pr_info("pcrm_qos_add DDR failed\n");
	}

	dev_dbg(&ddev->dev, "DPI display probed\n");

	goto out;

out_backlight_unregister:
	backlight_device_unregister(bd);
out_free_lcd:
	mutex_destroy(&lcd->lock);
	kfree(lcd);
invalid_port_type:
no_pdata:
out:
	return ret;
}

static int __devexit s6e63m0_mcde_panel_remove(struct mcde_display_device *ddev)
{
	int ret;
	struct s6e63m0 *lcd = dev_get_drvdata(&ddev->dev);

	dev_dbg(&ddev->dev, "Invoked %s\n", __func__);
	ret = ddev->set_power_mode(ddev, MCDE_DISPLAY_PM_OFF);
	if (ret < 0)
		dev_warn(&ddev->dev, "%s:Failed to resume display\n"
			, __func__);
	backlight_device_unregister(lcd->bd);
	spi_unregister_driver(&lcd->spi_drv);
	kfree(lcd);

	return 0;
}

static int s6e63m0_mcde_panel_shutdown(struct mcde_display_device *ddev)
{
	int ret;
	struct s6e63m0 *lcd = dev_get_drvdata(&ddev->dev);

	dev_dbg(&ddev->dev, "Invoked %s\n", __func__);
	ret = ddev->set_power_mode(ddev, MCDE_DISPLAY_PM_OFF);
	if (ret < 0)
		dev_warn(&ddev->dev, "%s:Failed to resume display\n"
			, __func__);
	backlight_device_unregister(lcd->bd);
	spi_unregister_driver(&lcd->spi_drv);

	#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&lcd->earlysuspend);
	#endif

	kfree(lcd);
	return 0;	
}

static int s6e63m0_mcde_panel_resume(struct mcde_display_device *ddev)
{
	int ret;
//	struct s6e63m0 *lcd = dev_get_drvdata(&ddev->dev);
	DPI_DISP_TRACE;
	pr_err("[S6E63M0] MCDE panel resumed\n");

	/* set_power_mode will handle call platform_enable */
	ret = ddev->set_power_mode(ddev, MCDE_DISPLAY_PM_STANDBY);
	if (ret < 0)
		dev_warn(&ddev->dev, "%s:Failed to resume display\n"
			, __func__);

	return ret;
}

static int s6e63m0_mcde_panel_suspend(struct mcde_display_device *ddev, pm_message_t state)
{
	int ret = 0;
//	struct s6e63m0 *lcd = dev_get_drvdata(&ddev->dev);

	pr_err("[S6E63M0] MCDE panel suspended\n");

	/*
	 * when lcd panel is suspend, lcd panel becomes off
	 * regardless of status.
	 */

	/* set_power_mode will handle call platform_disable */
	ret = ddev->set_power_mode(ddev, MCDE_DISPLAY_PM_OFF);
	if (ret < 0)
		dev_warn(&ddev->dev, "%s:Failed to suspend display\n"
			, __func__);

	dev_dbg(&ddev->dev, "end %s\n", __func__);
	return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND

static pin_cfg_t janice_sleep_pins[] = {
	GPIO169_GPIO | PIN_INPUT_PULLDOWN,
	GPIO171_GPIO | PIN_INPUT_PULLDOWN,
};

static pin_cfg_t janice_resume_pins[] = {
	GPIO169_LCDA_DE,
	GPIO171_LCDA_HSO,
};


static int dpi_display_platform_enable(struct s6e63m0 *lcd)
{
	int res = 0;
	pr_err("[S6E63M0] Display enabled\n");
	nmk_config_pins(janice_resume_pins, ARRAY_SIZE(janice_resume_pins));
	res = ux500_pins_enable(dpi_pins);
	if (res)
		dev_warn(lcd->dev, "Failure during %s\n", __func__);
	return res;
}

static int dpi_display_platform_disable(struct s6e63m0 *lcd)
{
	int res = 0;
	pr_err("[S6E63M0] Display disabled\n");
	nmk_config_pins(janice_sleep_pins, ARRAY_SIZE(janice_sleep_pins));

	/* pins disabled to save power */
	res = ux500_pins_disable(dpi_pins);
	if (res)
		dev_warn(lcd->dev, "Failure during %s\n", __func__);
	return res;
}

static void s6e63m0_mcde_panel_early_suspend(struct early_suspend *earlysuspend)
{
	struct s6e63m0 *lcd = container_of(earlysuspend, struct s6e63m0, earlysuspend);
	pm_message_t dummy;

	s6e63m0_mcde_panel_suspend(lcd->ddev, dummy);
	dpi_display_platform_disable(lcd);

	prcmu_qos_remove_requirement(PRCMU_QOS_APE_OPP,
				"janice_lcd_dpi");
	prcmu_qos_remove_requirement(PRCMU_QOS_DDR_OPP,
				"janice_lcd_dpi");
}

static void s6e63m0_mcde_panel_late_resume(struct early_suspend *earlysuspend)
{
	struct s6e63m0 *lcd = container_of(earlysuspend, struct s6e63m0, earlysuspend);

	if (prcmu_qos_add_requirement(PRCMU_QOS_APE_OPP,
			"janice_lcd_dpi", 50)) {
		pr_info("pcrm_qos_add APE failed\n");
	}

	if (prcmu_qos_add_requirement(PRCMU_QOS_DDR_OPP,
			"janice_lcd_dpi", 50)) {
		pr_info("pcrm_qos_add DDR failed\n");
	}

	dpi_display_platform_enable(lcd);
	s6e63m0_mcde_panel_resume(lcd->ddev);

	if (lcdclk_usr != 0) {
		pr_err("[S6E63M0] Rebasing LCDCLK...\n");
		schedule_work(&s6e63m0_lcdclk_work);
	}
}
#endif

static struct mcde_display_driver s6e63m0_mcde __refdata = {
	.probe          = s6e63m0_mcde_panel_probe,
	.remove         = s6e63m0_mcde_panel_remove,
	#if 0
	.shutdown	= s6e63m0_mcde_panel_shutdown,
	#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	.suspend        = NULL,
	.resume         = NULL,
#else
	.suspend        = s6e63m0_mcde_panel_suspend,
	.resume         = s6e63m0_mcde_panel_resume,
#endif
	.driver		= {
		.name	= LCD_DRIVER_NAME_S6E63M0,
		.owner	= THIS_MODULE,
	},
};


static int __init s6e63m0_init(void)
{
	int ret = 0;
	ret =  mcde_display_driver_register(&s6e63m0_mcde);

	#ifdef CONFIG_HAS_EARLYSUSPEND
	dpi_pins = ux500_pins_get("mcde-dpi");
	if (!dpi_pins)
		return -EINVAL;

	ret = ux500_pins_enable(dpi_pins);
	if (ret)
		pr_err("[S6E63M0] failed to enable mcde-dpi pins during init\n");
	#endif

        return ret;
}

static void __exit s6e63m0_exit(void)
{
	mcde_display_driver_unregister(&s6e63m0_mcde);
}

module_init(s6e63m0_init);
module_exit(s6e63m0_exit);

MODULE_AUTHOR("InKi Dae <inki.dae@samsung.com>");
MODULE_DESCRIPTION("S6E63M0 LCD Driver");
MODULE_LICENSE("GPL");
