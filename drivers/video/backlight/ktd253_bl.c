/*
 * Kinetic KTD253 LED driver.
 *
 * Copyright (c) 2011 Samsung Electronics (UK) Ltd.
 *
 * Author: Gareth Phillips  <gareth.phillips@samsung.com>
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/earlysuspend.h>
#include <video/ktd253x_bl.h>

/* to be removed when driver works */
//#define dev_dbg dev_info


#define KTD253_BACKLIGHT_OFF		0
#define KTD253_MIN_CURRENT_RATIO	1	/* 1/32 full current */
#define KTD253_MAX_CURRENT_RATIO	32	/* 32/32 full current */


/* WARNING:
    If incrementing T_LOW_NS or T_HIGH_NS see WARNING comment in function ktd253_set_brightness().
*/
#define T_LOW_NS       (200 + 10) /* Additional 10 as safety factor */
#define T_HIGH_NS      (200 + 10) /* Additional 10 as safety factor */

#define T_OFF_MS       3

struct ktd253 {
	unsigned int currentRatio;
	unsigned int brightness;
	const struct ktd253x_bl_platform_data *pd;
	bool backlight_disabled;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend	earlysuspend;
#endif
};

static int ktd253_set_brightness(struct backlight_device *bd)
{
	struct ktd253 *pKtd253Data = bl_get_data(bd);
	struct ktd253x_bl_platform_data *pd = pKtd253Data->pd;
	int reqBrightness = bd->props.brightness;
	int currentRatio  = pKtd253Data->currentRatio;
	int newCurrentRatio;
	int step_count = 0;
	unsigned long irqFlags;

	dev_dbg(&bd->dev, "%s function enter (%d->%d)\n", __func__,
		pKtd253Data->brightness, reqBrightness);

	/* Don't update backlight controller if the backlight is disabled (e.g. phone is suspended) */
	if (pKtd253Data->backlight_disabled) {
		dev_dbg(&bd->dev, "%s: backlight disabled\n", __func__);
		goto exit_backlight_disabled;
	}

	if ((bd->props.power != FB_BLANK_UNBLANK) ||
		(bd->props.fb_blank != FB_BLANK_UNBLANK)) {
		reqBrightness = KTD253_BACKLIGHT_OFF;
	} else if (reqBrightness < KTD253_BACKLIGHT_OFF) {
		reqBrightness = KTD253_BACKLIGHT_OFF;
	} else if (reqBrightness > pd->max_brightness) {
		reqBrightness = pd->max_brightness;
	}

	for (newCurrentRatio = KTD253_MAX_CURRENT_RATIO; newCurrentRatio > KTD253_BACKLIGHT_OFF; newCurrentRatio--) {
		if (reqBrightness > pd->brightness_to_current_ratio[newCurrentRatio - 1])
			break;
	}

	if (newCurrentRatio > KTD253_MAX_CURRENT_RATIO) {
		dev_warn(&bd->dev, "%s: new current ratio (%d) exceeds max (%d)\n",
			__func__, newCurrentRatio, KTD253_MAX_CURRENT_RATIO);
		newCurrentRatio = KTD253_MAX_CURRENT_RATIO;
	}

	if (newCurrentRatio != currentRatio) {
		if (newCurrentRatio == KTD253_BACKLIGHT_OFF) {
			/* Switch off backlight.
			*/
			dev_dbg(&bd->dev, "%s: switching backlight off\n", __func__);
			gpio_set_value(pd->ctrl_gpio, pd->ctrl_low);
			msleep(T_OFF_MS);
		} else {
			if (currentRatio == KTD253_BACKLIGHT_OFF) {
				/* Switch on backlight. */
				dev_dbg(&bd->dev, "%s: switching backlight on\n", __func__);
				gpio_set_value(pd->ctrl_gpio, pd->ctrl_high);
				ndelay(T_HIGH_NS);

				/* Backlight is always at full intensity when switched on. */
				currentRatio = KTD253_MAX_CURRENT_RATIO;
			}

			/* WARNING:
			The loop to set the correct current level is performed
			with interrupts disabled as it is timing critical.
			The maximum number of cycles of the loop is 32,
			so the time taken will be (T_LOW_NS + T_HIGH_NS + loop_time) * 32,
			where loop_time equals the time taken in executing the
			loop (excluding delays). If T_LOW_NS or T_HIGH_NS are
			increased care must be taken to ensure that the time
			during which interrupts are disabled will not result
			in interrupts being lost.
			*/
			local_irq_save(irqFlags);

			while (currentRatio != newCurrentRatio) {
				gpio_set_value(pd->ctrl_gpio, pd->ctrl_low);
				ndelay(T_LOW_NS);
				gpio_set_value(pd->ctrl_gpio, pd->ctrl_high);
				ndelay(T_HIGH_NS);

				/* Each time CTRL is toggled, current level drops by one step.
				     ...but loops from minimum (1/32) to maximum (32/32).
				*/
				if (currentRatio == KTD253_MIN_CURRENT_RATIO) {
					currentRatio = KTD253_MAX_CURRENT_RATIO;
				} else {
					currentRatio--;
				}
				step_count++;
			}

			local_irq_restore(irqFlags);

			dev_dbg(&bd->dev, "%s: new current ratio = %d; stepped by %d\n", __func__,
				newCurrentRatio, step_count);

		}

		pKtd253Data->currentRatio = newCurrentRatio;
	}

exit_backlight_disabled:
	
	pKtd253Data->brightness   = reqBrightness;

	return 0;
}


static int ktd253_get_brightness(struct backlight_device *bd)
{
	struct ktd253 *pKtd253Data = bl_get_data(bd);

	dev_dbg(&bd->dev, "%s returning %d\n", __func__, pKtd253Data->brightness);
	return pKtd253Data->brightness;
}


static const struct backlight_ops ktd253_ops = {
	.get_brightness = ktd253_get_brightness,
	.update_status  = ktd253_set_brightness,
};

/* Control function to switch the backlight on/off. May be called by suspend/resume or externally */
static void ktd253_backlight_on_off(struct backlight_device *bd, bool on)
{
	struct ktd253 *pKtd253Data = bl_get_data(bd);

	dev_dbg(&bd->dev, "%s function enter\n", __func__);

	if (on){
		pKtd253Data->backlight_disabled = false;
		bd->props.brightness = pKtd253Data->brightness;
		ktd253_set_brightness(bd);
	} else {
		pKtd253Data->backlight_disabled = true;
		gpio_set_value(pKtd253Data->pd->ctrl_gpio, pKtd253Data->pd->ctrl_low);
		pKtd253Data->currentRatio = KTD253_BACKLIGHT_OFF;
	}
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ktd253_early_suspend(struct early_suspend *earlysuspend)
{
	struct ktd253 *pKtd253Data = container_of(earlysuspend,
						struct ktd253,
						earlysuspend);
	printk("%s function enter\n", __func__);

	/* Ignore suspend if external backlight control is used */
	if (pKtd253Data->pd->external_bl_control == false)
		ktd253_backlight_on_off(pKtd253Data->pd->bd, false);

// 2012.11.01. When LCD OFF, there is blink issue. So comment this line. ( P121025-6088 )
#if 0
	/* Ensures backlight is turned off */
	gpio_set_value(pKtd253Data->pd->ctrl_gpio, pKtd253Data->pd->ctrl_low);
#endif
}
static void ktd253_late_resume(struct early_suspend *earlysuspend)
{
	struct ktd253 *pKtd253Data = container_of(earlysuspend,
						struct ktd253,
						earlysuspend);
	printk("%s function enter\n", __func__);

	/* Ignore resume if external backlight control is used */
	if (pKtd253Data->pd->external_bl_control == false)
		ktd253_backlight_on_off(pKtd253Data->pd->bd, true);
}
#endif

static int ktd253_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct backlight_device *ktd253_backlight_device;
	struct ktd253 *pKtd253Data;
	struct ktd253x_bl_platform_data *pd;

	int ret = 0;

	dev_dbg(&pdev->dev, "%s function enter\n", __func__);

	pKtd253Data = kmalloc(sizeof(struct ktd253), GFP_KERNEL);
	memset(pKtd253Data, 0, sizeof(struct ktd253));
#if 0 // HW request. Down the backlight because of current consumption.
	pKtd253Data->currentRatio = KTD253_BACKLIGHT_OFF;
	pKtd253Data->brightness = KTD253_BACKLIGHT_OFF;
#else
	pKtd253Data->currentRatio = 12;
	pKtd253Data->brightness = 160;
#endif
	pKtd253Data->pd = pdev->dev.platform_data;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = pKtd253Data->pd->max_brightness;
	props.type = BACKLIGHT_RAW;

	ktd253_backlight_device = backlight_device_register(	"panel",
								&pdev->dev,
								pKtd253Data,
							     	&ktd253_ops,
							     	&props);

	if (IS_ERR(ktd253_backlight_device)) {
		dev_info(&pdev->dev, "backlight_device_register() failed\n");

		ret = PTR_ERR(ktd253_backlight_device);
	} else {
		ktd253_backlight_device->props.power      = FB_BLANK_UNBLANK;
#if 0 // HW request. Down the backlight because of current consumption.
		ktd253_backlight_device->props.brightness = ktd253_backlight_device->props.max_brightness;
		ktd253_set_brightness(ktd253_backlight_device);
#endif
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	pKtd253Data->earlysuspend.level   = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1;
	pKtd253Data->earlysuspend.suspend = ktd253_early_suspend;
	pKtd253Data->earlysuspend.resume  = ktd253_late_resume;
	register_early_suspend(&pKtd253Data->earlysuspend);
	pKtd253Data->backlight_disabled = false;
#endif

	pd = pKtd253Data->pd;
	pd->bd = ktd253_backlight_device;

	/* If external control of the backlight has been requested, then provide interface function
	    in backlight platform data */
	if (pd->external_bl_control)
		pd->bl_on_off = ktd253_backlight_on_off;

	dev_dbg(&pdev->dev, "%s function exit\n", __func__);

	return ret;
}


static int ktd253_remove(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);
	struct ktd253 *pKtd253Data  = bl_get_data(bd);

	dev_dbg(&pdev->dev, "%s: unregistering backlight device\n", __func__);

	backlight_device_unregister(bd);
	kfree(pKtd253Data);

	return 0;
}

static struct platform_driver ktd253_driver = {
	.probe = ktd253_probe,
	.remove = ktd253_remove,
	.driver = {
		.name = BL_DRIVER_NAME_KTD253,
	},
};


static int __init ktd253_init(void)
{
	return platform_driver_probe(&ktd253_driver, ktd253_probe);
}

static void __exit ktd253_exit(void)
{
	platform_driver_unregister(&ktd253_driver);
}

module_init(ktd253_init);
module_exit(ktd253_exit);

MODULE_AUTHOR("Gareth Phillips <gareth.phillips@samsung.com>");
MODULE_DESCRIPTION("KTD253 Backlight Driver");
MODULE_LICENSE("GPL");

