/*
 * Copyright (C) ST-Ericsson SA 2010
 * Author: Mattias Nilsson <mattias.i.nilsson@stericsson.com> for ST Ericsson.
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/reboot.h>
#include <linux/signal.h>
#include <linux/power_supply.h>
#include <linux/mfd/ab8500.h>
#include <linux/mfd/abx500.h>
#include <linux/mfd/abx500/ab8500-bm.h>
#include <linux/mfd/abx500/ux500_sysctrl.h>
#include <linux/time.h>
#include <linux/hwmon.h>

/* RtcCtrl bits */
#define AB8500_ALARM_MIN_LOW  0x08
#define AB8500_ALARM_MIN_MID 0x09
#define RTC_CTRL 0x0B
#define RTC_ALARM_ENABLE 0x4

static struct device *sysctrl_dev;

void ab8500_power_off(void)
{
	struct ab8500_platform_data *plat;
	struct timespec ts;
	sigset_t old;
	sigset_t all;
#ifdef CONFIG_BATTERY_SAMSUNG
	static char *pss[] = {"battery"};
#else
	static char *pss[] = {"ab8500_ac", "ab8500_usb"};
#endif
	int i;
	bool charger_present = false;
	union power_supply_propval val;
	struct power_supply *psy;
	int ret;
	u8 data;

	if (sysctrl_dev == NULL) {
		pr_err("%s: sysctrl not initialized\n", __func__);
		return;
	}

	/*
	 * If we have a charger connected and we're powering off,
	 * reboot into charge-only mode.
	 */

	for (i = 0; i < ARRAY_SIZE(pss); i++) {
		psy = power_supply_get_by_name(pss[i]);
		if (!psy)
			continue;

		ret = psy->get_property(psy, POWER_SUPPLY_PROP_ONLINE, &val);

#ifdef CONFIG_BATTERY_SAMSUNG
		if (!ret && (val.intval != POWER_SUPPLY_TYPE_BATTERY)) {
#else
		if (!ret && val.intval) {
#endif
			charger_present = true;
			break;
		}
	}

        abx500_get_register_interruptible(sysctrl_dev, AB8500_CHARGER,
                                          AB8500_CH_STATUS1_REG, &data);

	if (!charger_present && !(data & 0x01))
		goto shutdown;


#ifdef CONFIG_BATTERY_SAMSUNG
	machine_restart("ta");
#else
	/* Check if battery is known */
	psy = power_supply_get_by_name("ab8500_btemp");
	if (psy) {
		ret = psy->get_property(psy, POWER_SUPPLY_PROP_TECHNOLOGY,
					&val);
		if (!ret && val.intval != POWER_SUPPLY_TECHNOLOGY_UNKNOWN) {
			printk(KERN_INFO
			       "Charger \"%s\" is connected with known battery."
			       " Rebooting.\n",
			       pss[i]);
			machine_restart("ta");
		}
	}
#endif

shutdown:
	sigfillset(&all);

	plat = dev_get_platdata(sysctrl_dev->parent);
		getnstimeofday(&ts);
	if (!sigprocmask(SIG_BLOCK, &all, &old)) {
		if (ts.tv_sec == 0 ||
			(ts.tv_sec - plat->thermal_set_time_sec >
				plat->thermal_time_out))
			plat->thermal_power_off_pending = false;
		if (!plat->thermal_power_off_pending) {
			(void)ab8500_sysctrl_set(AB8500_STW4500CTRL1,
				AB8500_STW4500CTRL1_SWOFF |
				AB8500_STW4500CTRL1_SWRESET4500N);
			(void)sigprocmask(SIG_SETMASK, &old, NULL);
		} else {
			(void)ab8500_sysctrl_set(AB8500_STW4500CTRL1,
				AB8500_STW4500CTRL1_THDB8500SWOFF |
				AB8500_STW4500CTRL1_SWRESET4500N);
			(void)sigprocmask(SIG_SETMASK, &old, NULL);
		}
	}
}

/*
 * Use the AB WD to reset the platform. It will perform a hard
 * reset instead of a soft reset. Write the reset reason to
 * the AB before reset, which can be read upon restart.
 */
void ab8500_restart(u16 reset_code)
{
	struct ab8500_platform_data *plat;
	struct ab8500_sysctrl_platform_data *pdata;
	u16 reason = 0;
	u8 val, val_s;
	int trial = 10;

	if (sysctrl_dev == NULL) {
		pr_err("%s: sysctrl not initialized\n", __func__);
		return;
	}

#if 0
	plat = dev_get_platdata(sysctrl_dev->parent);
	pdata = plat->sysctrl;
	if (pdata->reboot_reason_code)
		reason = pdata->reboot_reason_code(cmd);
	else
		pr_warn("[%s] No reboot reason set. Default reason %d\n",
			__func__, reason);
#else
	reason = reset_code;
#endif
	/*
	 * Disable RTC alarm, just a precaution so that no alarm
	 * is running when WD reset is executed.
	 */
	abx500_get_register_interruptible(sysctrl_dev, AB8500_RTC,
		RTC_CTRL , &val);
	abx500_set_register_interruptible(sysctrl_dev, AB8500_RTC,
		RTC_CTRL , (val & ~RTC_ALARM_ENABLE));

	/* SMPL disabled for AB WatchDog */
	while (trial) {
		abx500_set_register_interruptible(sysctrl_dev, AB8500_RTC,
			0x12 , 0);
		abx500_get_register_interruptible(sysctrl_dev, AB8500_RTC,
		0x12 , &val_s);
		if(!val_s)
			break;
		else
			trial--;
	}

	/*
	 * Android is not using the RTC alarm registers during reboot
	 * so we borrow them for writing the reason of reset
	 */

	/* reason[8 LSB] */
	val = reason & 0xFF;
	abx500_set_register_interruptible(sysctrl_dev, AB8500_RTC,
		AB8500_ALARM_MIN_LOW , val);

	/* reason[8 MSB] */
	val = (reason>>8) & 0xFF;
	abx500_set_register_interruptible(sysctrl_dev, AB8500_RTC,
		AB8500_ALARM_MIN_MID , val);

	/* Setting WD timeout to 0 */
	ab8500_sysctrl_write(AB8500_MAINWDOGTIMER, 0xFF, 0x0);

	/* Setting the parameters to AB8500 WD*/
	ab8500_sysctrl_write(AB8500_MAINWDOGCTRL, 0xFF, (AB8500_ENABLE_WD |
		AB8500_WD_RESTART_ON_EXPIRE | AB8500_KICK_WD));
}

static int ab8500_notifier_call(struct notifier_block *this,
				unsigned long val, void *data)
{
	struct ab8500_platform_data *plat;
	static struct timespec ts;
	if (sysctrl_dev == NULL)
		return -EAGAIN;

	plat = dev_get_platdata(sysctrl_dev->parent);
	if (val) {
		getnstimeofday(&ts);
		plat->thermal_set_time_sec = ts.tv_sec;
		plat->thermal_power_off_pending = true;
	} else {
		plat->thermal_set_time_sec = 0;
		plat->thermal_power_off_pending = false;
	}
	return 0;
}

static struct notifier_block ab8500_notifier = {
	.notifier_call = ab8500_notifier_call,
};

static inline bool valid_bank(u8 bank)
{
	return ((bank == AB8500_SYS_CTRL1_BLOCK) ||
		(bank == AB8500_SYS_CTRL2_BLOCK) ||
		(bank == AB8500_RTC));
}

int ab8500_sysctrl_read(u16 reg, u8 *value)
{
	u8 bank;

	if (sysctrl_dev == NULL)
		return -EINVAL;

	bank = (reg >> 8);
	if (!valid_bank(bank))
		return -EINVAL;

	return abx500_get_register_interruptible(sysctrl_dev, bank,
		(u8)(reg & 0xFF), value);
}
EXPORT_SYMBOL(ab8500_sysctrl_read);

int ab8500_sysctrl_write(u16 reg, u8 mask, u8 value)
{
	u8 bank;

	if (sysctrl_dev == NULL)
		return -EINVAL;

	bank = (reg >> 8);
	if (!valid_bank(bank))
		return -EINVAL;

	return abx500_mask_and_set_register_interruptible(sysctrl_dev, bank,
		(u8)(reg & 0xFF), mask, value);
}
EXPORT_SYMBOL(ab8500_sysctrl_write);

static int __devinit ab8500_sysctrl_probe(struct platform_device *pdev)
{
	struct ab8500 *ab8500 = dev_get_drvdata(pdev->dev.parent);
	struct ab8500_platform_data *plat;
	struct ab8500_sysctrl_platform_data *pdata;
	int ret, i, j;

	plat = dev_get_platdata(pdev->dev.parent);

	if (!(plat && plat->sysctrl))
		return -EINVAL;

	if (plat->pm_power_off)
		pm_power_off = ab8500_power_off;
	hwmon_notifier_register(&ab8500_notifier);

	pdata = plat->sysctrl;

	if (pdata) {
		int last;

		if (is_ab8505(ab8500))
			last = AB8500_SYSCLKREQ4RFCLKBUF;
		else
			last = AB8500_SYSCLKREQ8RFCLKBUF;

		for (i = AB8500_SYSCLKREQ1RFCLKBUF; i <= last; i++) {
			j = i - AB8500_SYSCLKREQ1RFCLKBUF;
			ret = ab8500_sysctrl_write(i, 0xff,
					pdata->initial_req_buf_config[j]);
			dev_dbg(&pdev->dev,
					"Setting SysClkReq%dRfClkBuf 0x%X\n",
					j + 1,
					pdata->initial_req_buf_config[j]);
			if (ret < 0) {
				dev_err(&pdev->dev,
					"unable to set sysClkReq%dRfClkBuf: "
					"%d\n", j + 1, ret);
			}
		}
	}

	sysctrl_dev = &pdev->dev;
	return 0;
}

static int __devexit ab8500_sysctrl_remove(struct platform_device *pdev)
{
	sysctrl_dev = NULL;
	return 0;
}

static struct platform_driver ab8500_sysctrl_driver = {
	.driver = {
		.name = "ab8500-sysctrl",
		.owner = THIS_MODULE,
	},
	.probe = ab8500_sysctrl_probe,
	.remove = __devexit_p(ab8500_sysctrl_remove),
};

static int __init ab8500_sysctrl_init(void)
{
	return platform_driver_register(&ab8500_sysctrl_driver);
}
subsys_initcall(ab8500_sysctrl_init);

MODULE_AUTHOR("Mattias Nilsson <mattias.i.nilsson@stericsson.com");
MODULE_DESCRIPTION("AB8500 system control driver");
MODULE_LICENSE("GPL v2");
