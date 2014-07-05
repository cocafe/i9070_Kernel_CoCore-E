/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License terms: GNU General Public License (GPL) version 2
 * Author: Virupax Sadashivpetimath <virupax.sadashivpetimath@stericsson.com>
 *
 * RTC clock driver for the RTC part of the AB8500 Power management chip.
 * Based on RTC clock driver for the AB3100 Analog Baseband Chip by
 * Linus Walleij <linus.walleij@stericsson.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/mfd/abx500.h>
#include <linux/mfd/ab8500.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>

#if defined(CONFIG_MACH_SEC_GOLDEN_CHN)
#include <mach/sec_param.h>
#include <mach/sec_common.h>
#include <linux/reboot.h>
#endif

#define AB8500_RTC_SOFF_STAT_REG	0x00
#define AB8500_RTC_CC_CONF_REG		0x01
#define AB8500_RTC_READ_REQ_REG		0x02
#define AB8500_RTC_WATCH_TSECMID_REG	0x03
#define AB8500_RTC_WATCH_TSECHI_REG	0x04
#define AB8500_RTC_WATCH_TMIN_LOW_REG	0x05
#define AB8500_RTC_WATCH_TMIN_MID_REG	0x06
#define AB8500_RTC_WATCH_TMIN_HI_REG	0x07
#define AB8500_RTC_ALRM_MIN_LOW_REG	0x08
#define AB8500_RTC_ALRM_MIN_MID_REG	0x09
#define AB8500_RTC_ALRM_MIN_HI_REG	0x0A
#define AB8500_RTC_STAT_REG		0x0B
#define AB8500_RTC_BKUP_CHG_REG		0x0C
#define AB8500_RTC_FORCE_BKUP_REG	0x0D
#define AB8500_RTC_CALIB_REG		0x0E
#define AB8500_RTC_SWITCH_STAT_REG	0x0F

/* RtcReadRequest bits */
#define RTC_READ_REQUEST		0x01
#define RTC_WRITE_REQUEST		0x02

/* RtcCtrl bits */
#define RTC_ALARM_ENA			0x04
#define RTC_STATUS_DATA			0x01

#define COUNTS_PER_SEC			(0xF000 / 60)
#define AB8500_RTC_EPOCH		1999
#define AB8500_ALARM_DEBUG		0
static struct rtc_device *_rtc;
static int rtc_60s_irq;

#ifdef CONFIG_RTC_INTF_ALARM
extern struct mutex alarm_setrtc_mutex;
#else
static DEFINE_MUTEX(alarm_setrtc_mutex);
#endif

static const u8 ab8500_rtc_time_regs[] = {
	AB8500_RTC_WATCH_TMIN_HI_REG, AB8500_RTC_WATCH_TMIN_MID_REG,
	AB8500_RTC_WATCH_TMIN_LOW_REG, AB8500_RTC_WATCH_TSECHI_REG,
	AB8500_RTC_WATCH_TSECMID_REG
};

static const u8 ab8500_rtc_alarm_regs[] = {
	AB8500_RTC_ALRM_MIN_HI_REG, AB8500_RTC_ALRM_MIN_MID_REG,
	AB8500_RTC_ALRM_MIN_LOW_REG
};

/* Calculate the seconds from 1970 to 01-01-2000 00:00:00 */
static unsigned long get_elapsed_seconds(int year)
{
	unsigned long secs;
	struct rtc_time tm = {
		.tm_year = year - 1900,
		.tm_mday = 1,
	};

	/*
	 * This function calculates secs from 1970 and not from
	 * 1900, even if we supply the offset from year 1900.
	 */
	rtc_tm_to_time(&tm, &secs);
	return secs;
}

static int ab8500_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long timeout = jiffies + HZ;
	int retval, i;
	unsigned long mins, secs;
	unsigned char buf[ARRAY_SIZE(ab8500_rtc_time_regs)];
	u8 value;

	/* Request a data read */
	retval = abx500_set_register_interruptible(dev,
		AB8500_RTC, AB8500_RTC_READ_REQ_REG, RTC_READ_REQUEST);
	if (retval < 0)
		return retval;

	/* Wait for some cycles after enabling the rtc read in ab8500 */
	while (time_before(jiffies, timeout)) {
		retval = abx500_get_register_interruptible(dev,
			AB8500_RTC, AB8500_RTC_READ_REQ_REG, &value);
		if (retval < 0)
			return retval;

		if (!(value & RTC_READ_REQUEST))
			break;

		mdelay(1);
	}

	/* Read the Watchtime registers */
	for (i = 0; i < ARRAY_SIZE(ab8500_rtc_time_regs); i++) {
		retval = abx500_get_register_interruptible(dev,
			AB8500_RTC, ab8500_rtc_time_regs[i], &value);
		if (retval < 0)
			return retval;
		buf[i] = value;
	}

	mins = (buf[0] << 16) | (buf[1] << 8) | buf[2];

	secs =	(buf[3] << 8) | buf[4];
	secs =	secs / COUNTS_PER_SEC;
	secs =	secs + (mins * 60);

	/* Add back the initially subtracted number of seconds */
	secs += get_elapsed_seconds(AB8500_RTC_EPOCH);

	rtc_time_to_tm(secs, tm);
	return rtc_valid_tm(tm);
}

static int ab8500_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	int i;
	unsigned char buf[ARRAY_SIZE(ab8500_rtc_time_regs)];
	unsigned long no_secs, no_mins, secs = 0;
	int ret;

	disable_irq(rtc_60s_irq);

	if (tm->tm_year < (AB8500_RTC_EPOCH - 1900)) {
		dev_dbg(dev, "year should be equal to or greater than %d\n",
				AB8500_RTC_EPOCH);
		ret = -EINVAL;
		goto fail;
	}

	/* Get the number of seconds since 1970 */
	rtc_tm_to_time(tm, &secs);

	/*
	 * Convert it to the number of seconds since 01-01-2000 00:00:00, since
	 * we only have a small counter in the RTC.
	 */
	secs -= get_elapsed_seconds(AB8500_RTC_EPOCH);

	no_mins = secs / 60;

	no_secs = secs % 60;
	/* Make the seconds count as per the RTC resolution */
	no_secs = no_secs * COUNTS_PER_SEC;

	buf[4] = no_secs & 0xFF;
	buf[3] = (no_secs >> 8) & 0xFF;

	buf[2] = no_mins & 0xFF;
	buf[1] = (no_mins >> 8) & 0xFF;
	buf[0] = (no_mins >> 16) & 0xFF;

	for (i = 0; i < ARRAY_SIZE(ab8500_rtc_time_regs); i++) {
		ret = abx500_set_register_interruptible(dev, AB8500_RTC,
			ab8500_rtc_time_regs[i], buf[i]);
		if (ret < 0)
			goto fail;

	}

	/* Request a data write */
	ret = abx500_set_register_interruptible(dev, AB8500_RTC,
		AB8500_RTC_READ_REQ_REG, RTC_WRITE_REQUEST);

fail:
	enable_irq(rtc_60s_irq);
	return ret;
}

static int ab8500_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	int retval, i;
	u8 rtc_ctrl, value;
	unsigned char buf[ARRAY_SIZE(ab8500_rtc_alarm_regs)];
	unsigned long secs, mins;

	/* Check if the alarm is enabled or not */
	retval = abx500_get_register_interruptible(dev, AB8500_RTC,
		AB8500_RTC_STAT_REG, &rtc_ctrl);
	if (retval < 0)
		return retval;

	if (rtc_ctrl & RTC_ALARM_ENA)
		alarm->enabled = 1;
	else
		alarm->enabled = 0;

	alarm->pending = 0;

	for (i = 0; i < ARRAY_SIZE(ab8500_rtc_alarm_regs); i++) {
		retval = abx500_get_register_interruptible(dev, AB8500_RTC,
			ab8500_rtc_alarm_regs[i], &value);
		if (retval < 0)
			return retval;
		buf[i] = value;
	}

	mins = (buf[0] << 16) | (buf[1] << 8) | (buf[2]);
	secs = mins * 60;

	/* Add back the initially subtracted number of seconds */
	secs += get_elapsed_seconds(AB8500_RTC_EPOCH);

	rtc_time_to_tm(secs, &alarm->time);

	return rtc_valid_tm(&alarm->time);
}

static int ab8500_rtc_irq_enable(struct device *dev, unsigned int enabled)
{
	return abx500_mask_and_set_register_interruptible(dev, AB8500_RTC,
		AB8500_RTC_STAT_REG, RTC_ALARM_ENA,
		enabled ? RTC_ALARM_ENA : 0);
}

static int ab8500_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	int retval, i;
	unsigned char buf[ARRAY_SIZE(ab8500_rtc_alarm_regs)];
	unsigned long mins, secs = 0;
	struct rtc_time tm;
	unsigned long secs_now = 0;

	if (alarm->time.tm_year < (AB8500_RTC_EPOCH - 1900)) {
		dev_dbg(dev, "year should be equal to or greater than %d\n",
				AB8500_RTC_EPOCH);
		return -EINVAL;
	}

	/* Get the number of seconds since 1970 */
	rtc_tm_to_time(&alarm->time, &secs);

#if AB8500_ALARM_DEBUG
	pr_info("ab8500_rtc - alarm time : %ld\n", secs);
#endif

	retval = ab8500_rtc_read_time(dev, &tm);
	if (retval < 0)
		return retval;

	rtc_tm_to_time(&tm, &secs_now);

	/*
	 * Convert it to the number of seconds since 01-01-2000 00:00:00, since
	 * we only have a small counter in the RTC.
	 */
	secs -= get_elapsed_seconds(AB8500_RTC_EPOCH);

#ifndef CONFIG_ANDROID
	secs += 30; /* Round to nearest minute */
#endif

	mins = secs / 60;

#ifdef CONFIG_ANDROID
	/*
	 * Needed due to Android believes all hw have a wake-up resolution
	 * in seconds.
	 */
	if (secs%60)
		mins++;
#endif
	buf[2] = mins & 0xFF;
	buf[1] = (mins >> 8) & 0xFF;
	buf[0] = (mins >> 16) & 0xFF;

	/* Set the alarm time */
	for (i = 0; i < ARRAY_SIZE(ab8500_rtc_alarm_regs); i++) {
		retval = abx500_set_register_interruptible(dev, AB8500_RTC,
			ab8500_rtc_alarm_regs[i], buf[i]);
		if (retval < 0)
			return retval;
	}

	return ab8500_rtc_irq_enable(dev, alarm->enabled);
}

#if defined(CONFIG_MACH_SEC_GOLDEN_CHN)
extern unsigned int battpwroff_charging;
void check_alarm_boot_lpm(void)
{
	pr_info("[AB8500 rtc] %s\n", __func__);
	if (battpwroff_charging == 1) {
		u8 check_param = 0;

		sec_get_param_value(__PARAM_INT_14, &check_param);

		pr_info("battpwroff_charging:%d, check_param:%d, alarm_en_exit:%d\n",
			battpwroff_charging, check_param, alarm_en_exit);

		if (check_param == 1)
			machine_restart(NULL);
	}
}
#endif
static int ab8500_rtc_set_calibration(struct device *dev, int calibration)
{
	int retval;
	u8  rtccal = 0;

	/*
	 * Check that the calibration value (which is in units of 0.5 parts-per-million)
	 * is in the AB8500's range for RtcCalibration register.
	 */
	if ((calibration < -127) || (calibration > 127)) {
		dev_err(dev, "RtcCalibration value outside permitted range\n");
		return -EINVAL;
	}

	/*
	 * The AB8500 uses sign (in bit7) and magnitude (in bits0-7)
	 * so need to convert to this sort of representation before writing
	 * into RtcCalibration register...
	 */
	if (calibration >= 0)
		rtccal = 0x7F & calibration;
	else
		rtccal = ~(calibration -1) | 0x80;

	retval = abx500_set_register_interruptible(dev, AB8500_RTC,
			AB8500_RTC_CALIB_REG, rtccal);

	return retval;
}

static int ab8500_rtc_get_calibration(struct device *dev, int *calibration)
{
	int retval;
	u8  rtccal = 0;

	retval =  abx500_get_register_interruptible(dev, AB8500_RTC,
			AB8500_RTC_CALIB_REG, &rtccal);
	if (retval >= 0) {
		/*
		 * The AB8500 uses sign (in bit7) and magnitude (in bits0-7)
		 * so need to convert value from RtcCalibration register into
		 * a two's complement signed value...
		 */
		if (rtccal & 0x80)
			*calibration = 0 - (rtccal & 0x7F);
		else
			*calibration = 0x7F & rtccal;
	}

	return retval;
}

static ssize_t ab8500_sysfs_store_rtc_calibration(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int retval;
	int calibration = 0;

	if (sscanf(buf, " %i ", &calibration) != 1) {
		dev_err(dev, "Failed to store RTC calibration attribute\n");
		return -EINVAL;
	}

	retval = ab8500_rtc_set_calibration(dev, calibration);

	return retval ? retval : count;
}

static ssize_t ab8500_sysfs_show_rtc_calibration(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int  retval = 0;
	int  calibration = 0;

	retval = ab8500_rtc_get_calibration(dev, &calibration);
	if (retval < 0) {
		dev_err(dev, "Failed to read RTC calibration attribute\n");
		sprintf(buf, "0\n");
		return retval;
	}

	return sprintf(buf, "%d\n", calibration);
}

static DEVICE_ATTR(rtc_calibration, S_IRUGO | S_IWUSR,
		   ab8500_sysfs_show_rtc_calibration,
		   ab8500_sysfs_store_rtc_calibration);

static int ab8500_sysfs_rtc_register(struct device *dev)
{
	return device_create_file(dev, &dev_attr_rtc_calibration);
}

static void ab8500_sysfs_rtc_unregister(struct device *dev)
{
	device_remove_file(dev, &dev_attr_rtc_calibration);
}

static irqreturn_t rtc_60s_handler(int irq, void* data)
{
	struct timespec ts;
	struct rtc_time rtc_tm;
	ktime_t sys_now, rtc_now;

	if (!mutex_trylock(&alarm_setrtc_mutex))
		return IRQ_HANDLED;

	if (ab8500_rtc_read_time(_rtc->dev.parent, &rtc_tm))
		goto fail;

	rtc_now = rtc_tm_to_ktime(rtc_tm);

#if AB8500_ALARM_DEBUG
	getnstimeofday(&ts);
	sys_now = timespec_to_ktime(ts);
	pr_info("ab8500_rtc : sys_now : %.18lld\n", sys_now.tv64);
#endif

	/* approx. 58ms drift / 1 min */
	sys_now = ktime_add_us(rtc_now, 60000);
	ts = ktime_to_timespec(sys_now);
	do_settimeofday(&ts);
fail:
	mutex_unlock(&alarm_setrtc_mutex);
	return IRQ_HANDLED;
}

static irqreturn_t rtc_alarm_handler(int irq, void *data)
{
	struct rtc_device *rtc = data;
	unsigned long events = RTC_IRQF | RTC_AF;

	dev_dbg(&rtc->dev, "%s\n", __func__);
	rtc_update_irq(rtc, 1, events);

#if defined(CONFIG_MACH_SEC_GOLDEN_CHN)
	if (battpwroff_charging)
		check_alarm_boot_lpm();
#endif
	return IRQ_HANDLED;
}

static const struct rtc_class_ops ab8500_rtc_ops = {
	.read_time		= ab8500_rtc_read_time,
	.set_time		= ab8500_rtc_set_time,
	.read_alarm		= ab8500_rtc_read_alarm,
	.set_alarm		= ab8500_rtc_set_alarm,
	.alarm_irq_enable	= ab8500_rtc_irq_enable,
};

static int __devinit ab8500_rtc_probe(struct platform_device *pdev)
{
	int err;
	struct rtc_device *rtc;
	u8 rtc_ctrl;
	int irq;

	irq = platform_get_irq_byname(pdev, "ALARM");
	if (irq < 0)
		return irq;

	rtc_60s_irq = platform_get_irq_byname(pdev, "60S");
	if (rtc_60s_irq < 0)
		return rtc_60s_irq;

	/* For RTC supply test */
	err = abx500_mask_and_set_register_interruptible(&pdev->dev, AB8500_RTC,
		AB8500_RTC_STAT_REG, RTC_STATUS_DATA, RTC_STATUS_DATA);
	if (err < 0)
		return err;

	/* Wait for reset by the PorRtc */
	mdelay(1);

	err = abx500_get_register_interruptible(&pdev->dev, AB8500_RTC,
		AB8500_RTC_STAT_REG, &rtc_ctrl);
	if (err < 0)
		return err;

	/* Check if the RTC Supply fails */
	if (!(rtc_ctrl & RTC_STATUS_DATA)) {
		dev_err(&pdev->dev, "RTC supply failure\n");
		return -ENODEV;
	}

	device_init_wakeup(&pdev->dev, true);

	rtc = rtc_device_register("ab8500-rtc", &pdev->dev, &ab8500_rtc_ops,
			THIS_MODULE);
	if (IS_ERR(rtc)) {
		dev_err(&pdev->dev, "Registration failed\n");
		err = PTR_ERR(rtc);
		return err;
	}

	err = request_threaded_irq(irq, NULL, rtc_alarm_handler,
		IRQF_NO_SUSPEND, "ab8500-rtc", rtc);
	if (err < 0) {
		rtc_device_unregister(rtc);
		return err;
	}

	err = request_threaded_irq(rtc_60s_irq, NULL, rtc_60s_handler,
		IRQF_SHARED|IRQF_NO_SUSPEND, "ab8500-rtc-60s", rtc);
	if (err < 0) {
		rtc_device_unregister(rtc);
		return err;
	}

	_rtc = rtc;

#if defined(CONFIG_MACH_SEC_GOLDEN_CHN)
	{
		u8 rtc_status = 0;
		struct rtc_wkalrm alarm_time;

		/* check rtc status  */
		abx500_get_register_interruptible(&pdev->dev, AB8500_RTC,
			AB8500_RTC_STAT_REG, &rtc_status);
		pr_info("[AB8500 rtc] rtc status : 0x%x\n", rtc_status);

		/* check interrupt status */
		abx500_get_register_interruptible(&pdev->dev,
			AB8500_SYS_CTRL1_BLOCK, 0x0, &rtc_status);
		pr_info("[AB8500 rtc] turn on status : 0x%x\n", rtc_status);
		pr_info("[AB8500 rtc] lpm mode : %d\n", battpwroff_charging);

		ab8500_rtc_read_alarm(&pdev->dev, &alarm_time);
		pr_info("%s, [%d] %d/%d/%d %d:%d:%d\n", __func__,
			alarm_time.enabled, alarm_time.time.tm_year-100,
			alarm_time.time.tm_mon+1, alarm_time.time.tm_mday,
			alarm_time.time.tm_hour, alarm_time.time.tm_min,
			alarm_time.time.tm_sec);
	}
#endif
	platform_set_drvdata(pdev, rtc);


	err = ab8500_sysfs_rtc_register(&pdev->dev);
	if (err) {
		dev_err(&pdev->dev, "sysfs RTC failed to register\n");
		return err;
	}

	return 0;
}

static int __devexit ab8500_rtc_remove(struct platform_device *pdev)
{
	struct rtc_device *rtc = platform_get_drvdata(pdev);
	int irq = platform_get_irq_byname(pdev, "ALARM");
	int irq_60s = platform_get_irq_byname(pdev, "60S");

#if defined(CONFIG_MACH_SEC_GOLDEN_CHN)
	int temp_param;
	if (alarm_en_exit == 1) {
		
		autoboot_alm.time.tm_min--;
		ab8500_rtc_set_alarm(&pdev->dev, &autoboot_alm);
	}

	sec_get_param_value(__PARAM_INT_14, &temp_param);
	pr_info("[AB8500 rtc] __PARAM_INT_14 = %d, %d\n",
		temp_param, alarm_en_exit);
	if ((temp_param == 0) && (alarm_en_exit == 1)) {
		temp_param = alarm_en_exit;
		sec_set_param_value(__PARAM_INT_14, &temp_param);
	}
#endif

	ab8500_sysfs_rtc_unregister(&pdev->dev);

	free_irq(irq_60s, rtc);
	free_irq(irq, rtc);
	rtc_device_unregister(rtc);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static int ab8500_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* disable minute interrupt during suspend */
	disable_irq(rtc_60s_irq);
	return 0;
}

static int ab8500_rtc_resume(struct platform_device *pdev)
{
	/* enable minute interrupt */
	enable_irq(rtc_60s_irq);
	return 0;
}

static struct platform_driver ab8500_rtc_driver = {
	.driver = {
		.name = "ab8500-rtc",
		.owner = THIS_MODULE,
	},
	.probe	= ab8500_rtc_probe,
	.remove = __devexit_p(ab8500_rtc_remove),
	.suspend = ab8500_rtc_suspend,
	.resume = ab8500_rtc_resume,
};

static int __init ab8500_rtc_init(void)
{
	return platform_driver_register(&ab8500_rtc_driver);
}

static void __exit ab8500_rtc_exit(void)
{
	platform_driver_unregister(&ab8500_rtc_driver);
}

module_init(ab8500_rtc_init);
module_exit(ab8500_rtc_exit);
MODULE_AUTHOR("Virupax Sadashivpetimath <virupax.sadashivpetimath@stericsson.com>");
MODULE_DESCRIPTION("AB8500 RTC Driver");
MODULE_LICENSE("GPL v2");
