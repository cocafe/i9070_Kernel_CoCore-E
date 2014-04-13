/*
 * Copyright (C) ST-Ericsson SA 2011
 *
 * Mali related Ux500 platform initialization
 *
 * Author: Marta Lofstedt <marta.lofstedt@stericsson.com> for ST-Ericsson.
 * Author: Huang Ji (cocafe@xda) <cocafehj@gmail.com>
 * 
 * License terms: GNU General Public License (GPL) version 2.
 */

/**
 * @file mali_platform.c
 * Platform specific Mali driver functions for ST-Ericsson's Ux500 platforms
 */
#include "mali_kernel_common.h"
#include "mali_osk.h"
#include "mali_platform.h"

#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/workqueue.h>
#include <linux/version.h>
#include <linux/moduleparam.h>
#include <linux/kobject.h>
#include <linux/jiffies.h>

#if CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 0, 0)
#include <mach/prcmu.h>
#else
#include <linux/mfd/dbx500-prcmu.h>
#endif

#define MALI_HIGH_TO_LOW_LEVEL_UTILIZATION_LIMIT 64
#define MALI_LOW_TO_HIGH_LEVEL_UTILIZATION_LIMIT 192

#define MALI_UX500_VERSION		"1.0.1"

#define MALI_MAX_UTILIZATION		256

#define PRCMU_SGACLK			0x0014
#define PRCMU_PLLSOC0			0x0080

#define PRCMU_SGACLK_INIT		0x00000021
#define PRCMU_PLLSOC0_INIT		0x01050168

#define AB8500_VAPE_SEL1 		0x0E
#define AB8500_VAPE_SEL2	 	0x0F
#define AB8500_VAPE_STEP_UV		12500
#define AB8500_VAPE_MIN_UV		700000
#define AB8500_VAPE_MAX_UV		1362500

#define MALI_CLOCK_DEFLO		399360
#define MALI_CLOCK_DEFHI		480000

struct mali_dvfs_data
{
	u32 	freq;
	u32 	clkpll;
	u8 	vape_raw;
};

static struct mali_dvfs_data mali_dvfs[] = {
	{192000, 0x0101010A, 0x26},
	{256000, 0x01030128, 0x26},
	{299520, 0x0105014E, 0x26},
	{320000, 0x01030132, 0x26},
	{360000, 0x0105015E, 0x26},
	{399360, 0x01050168, 0x26},
	{422400, 0x01010116, 0x26},
	{441600, 0x0102012E, 0x26},
	{460800, 0x01010118, 0x29},
	{480000, 0x01020132, 0x2A},
	{499200, 0x0101011A, 0x2B},
	{518400, 0x01020136, 0x2C},
	{537600, 0x0101011C, 0x2D},
	{560640, 0x01050192, 0x2F},
	{579840, 0x01050197, 0x30},
	{600000, 0x0104017D, 0x32},
	{619200, 0x01040181, 0x33},
	{640000, 0x01030164, 0x34},
	{660480, 0x010501AC, 0x3F},
	{679680, 0x010501B1, 0x3F},
	{700800, 0x01040192, 0x3F},
	{710400, 0x01010125, 0x3F},
	{720000, 0x01040196, 0x3F},
	{729600, 0x01010126, 0x3F},
};

int mali_utilization_high_to_low = MALI_HIGH_TO_LOW_LEVEL_UTILIZATION_LIMIT;
int mali_utilization_low_to_high = MALI_LOW_TO_HIGH_LEVEL_UTILIZATION_LIMIT;

static bool is_running;
static bool is_initialized;

static u32 mali_last_utilization;
module_param(mali_last_utilization, uint, 0444);

static struct regulator *regulator;
static struct clk *clk_sga;
static struct work_struct mali_utilization_work;
static struct workqueue_struct *mali_utilization_workqueue;

#if CONFIG_HAS_WAKELOCK
static struct wake_lock wakelock;
#endif

static u32 boost_enable 	= 1;
static u32 boost_working 	= 0;
static u32 boost_required 	= 0;
static u32 boost_delay 		= 500;
static u32 boost_low 		= 0;
static u32 boost_high 		= 0;
static u32 boost_utilization 	= 0;
static u32 boost_upthreshold 	= 233;
static u32 boost_downthreshold 	= 64;

static struct delayed_work mali_boost_delayedwork;

static int vape_voltage(u8 raw)
{
	if (raw <= 0x35) {
		return (AB8500_VAPE_MIN_UV + (raw * AB8500_VAPE_STEP_UV));
	} else {
		return AB8500_VAPE_MAX_UV;
	}
}

static int pllsoc0_freq(u32 raw)
{
	int multiple = raw & 0x000000FF;
	int divider = (raw & 0x00FF0000) >> 16;
	int half = (raw & 0x01000000) >> 24;
	int pll;

	pll = (multiple * 38400);
	pll /= divider;

	if (half) {
		pll /= 2;
	}

	return pll;
}

static int sgaclk_freq(void)
{
	u32 soc0pll = prcmu_read(PRCMU_PLLSOC0);
	u32 sgaclk = prcmu_read(PRCMU_SGACLK);

	if (!(sgaclk & BIT(5)))
		return 0;

	if (!(sgaclk & 0xf))
		return 0;
	
	return (pllsoc0_freq(soc0pll) / (sgaclk & 0xf));
}

static void mali_boost_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mali_dvfs); i++) {
		if (mali_dvfs[i].freq == MALI_CLOCK_DEFLO) {
			boost_low = i;
			break;
		}
	}

	for (i = 0; i < ARRAY_SIZE(mali_dvfs); i++) {
		if (mali_dvfs[i].freq == MALI_CLOCK_DEFHI) {
			boost_high = i;
			break;
		}
	}

	pr_info("[Mali] Booster: %u kHz - %u kHz\n", 
			mali_dvfs[boost_low].freq, 
			mali_dvfs[boost_high].freq);
}

static void mali_boost_update(void)
{
	u8 vape;
	u32 pll;

	if (!boost_required) {
		if (boost_utilization >= boost_upthreshold) {
			vape = mali_dvfs[boost_high].vape_raw;
			pll = mali_dvfs[boost_high].clkpll;

			pr_err("[Mali] @%u kHz - Boost\n", mali_dvfs[boost_high].freq);

			prcmu_abb_write(AB8500_REGU_CTRL2, 
				AB8500_VAPE_SEL1, 
				&vape, 
				1);
			prcmu_write(PRCMU_PLLSOC0, pll);

			boost_required = 1;
		}
	} else {
		if (boost_utilization <= boost_downthreshold) {
			vape = mali_dvfs[boost_low].vape_raw;
			pll = mali_dvfs[boost_low].clkpll;

			pr_err("[Mali] @%u kHz - Deboost\n", mali_dvfs[boost_low].freq);

			prcmu_write(PRCMU_PLLSOC0, pll);
			prcmu_abb_write(AB8500_REGU_CTRL2, 
				AB8500_VAPE_SEL1, 
				&vape, 
				1);

			boost_required = 0;
		}
	}
}

static void mali_clock_apply(u32 idx)
{
	u8 vape;
	u32 pll;

	vape = mali_dvfs[idx].vape_raw;
	pll = mali_dvfs[idx].clkpll;

	prcmu_abb_write(AB8500_REGU_CTRL2, 
		AB8500_VAPE_SEL1, 
		&vape, 
		1);
	prcmu_write(PRCMU_PLLSOC0, pll);
}

static void mali_boost_work(struct work_struct *work)
{
	mali_boost_update();
	boost_working = false;
}

static _mali_osk_errcode_t mali_platform_powerdown(void)
{
	if (is_running) {

#if CONFIG_HAS_WAKELOCK
		wake_unlock(&wakelock);
#endif
		clk_disable(clk_sga);
		if (regulator) {
			int ret = regulator_disable(regulator);
			if (ret < 0) {
				MALI_DEBUG_PRINT(2, ("%s: Failed to disable regulator %s\n", __func__, "v-mali"));
				is_running = false;
				MALI_ERROR(_MALI_OSK_ERR_FAULT);
			}
		}
		is_running = false;
	}
	MALI_DEBUG_PRINT(4, ("mali_platform_powerdown is_running: %u\n", is_running));
	MALI_SUCCESS;
}

static _mali_osk_errcode_t mali_platform_powerup(void)
{
	if (!is_running) {
		int ret = regulator_enable(regulator);
		if (ret < 0) {
			MALI_DEBUG_PRINT(2, ("%s: Failed to enable regulator %s\n", __func__, "v-mali"));
			goto error;
		}

		ret = clk_enable(clk_sga);
		if (ret < 0) {
			regulator_disable(regulator);
			MALI_DEBUG_PRINT(2, ("%s: Failed to enable clock %s\n", __func__, "mali"));
			goto error;
		}

#if CONFIG_HAS_WAKELOCK
		wake_lock(&wakelock);
#endif
		is_running = true;
	}
	MALI_DEBUG_PRINT(4, ("mali_platform_powerup is_running:%u\n", is_running));
	MALI_SUCCESS;
error:
	MALI_DEBUG_PRINT(1, ("Failed to power up.\n"));
	MALI_ERROR(_MALI_OSK_ERR_FAULT);
}

/* Rationale behind the values for:
* MALI_HIGH_LEVEL_UTILIZATION_LIMIT and MALI_LOW_LEVEL_UTILIZATION_LIMIT
* When operating at half clock frequency a faster clock is requested when
* reaching 75% utilization. When operating at full clock frequency a slower
* clock is requested when reaching 25% utilization. There is a margin of 25%
* at the high range of the slow clock to avoid complete saturation of the
* hardware and there is some overlap to avoid an oscillating situation where
* the clock goes back and forth from high to low.
*
* Utilization on full speed clock
* 0               64             128             192              255
* |---------------|---------------|---------------|---------------|
*                 XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
*                 |       ^
*                 V       |
* XXXXXXXXXXXXXXXXXXXXXXXXX
* 0       64     128     192      255
* |-------|-------|-------|-------|
* Utilization on half speed clock
*/
void mali_utilization_function(struct work_struct *ptr)
{
	/*By default, platform start with 50% APE OPP and 25% DDR OPP*/
	static u32 has_requested_low = 1;

	if (mali_last_utilization > mali_utilization_low_to_high) {
		if (has_requested_low) {
			MALI_DEBUG_PRINT(5, ("MALI GPU utilization: %u SIGNAL_HIGH\n", mali_last_utilization));
			/*Request 100% APE_OPP.*/
			if (prcmu_qos_add_requirement(PRCMU_QOS_APE_OPP, "mali", 100)) {
				MALI_DEBUG_PRINT(2, ("MALI 100% APE_OPP failed\n"));
				return;
			}
			/*
			* Since the utilization values will be reported higher
			* if DDR_OPP is lowered, we also request 100% DDR_OPP.
			*/
			if (prcmu_qos_add_requirement(PRCMU_QOS_DDR_OPP, "mali", 100)) {
				MALI_DEBUG_PRINT(2, ("MALI 100% DDR_OPP failed\n"));
				return;
			}
			has_requested_low = 0;
		}
	} else {
		if (mali_last_utilization < mali_utilization_high_to_low) {
			if (!has_requested_low) {
				/*Remove APE_OPP and DDR_OPP requests*/
				prcmu_qos_remove_requirement(PRCMU_QOS_APE_OPP, "mali");
				prcmu_qos_remove_requirement(PRCMU_QOS_DDR_OPP, "mali");
				MALI_DEBUG_PRINT(5, ("MALI GPU utilization: %u SIGNAL_LOW\n", mali_last_utilization));
				has_requested_low = 1;
			}
		}
	}

	if (boost_enable) {
		if (!boost_working) {
			boost_utilization = mali_last_utilization;
			boost_working = true;

			schedule_delayed_work(&mali_boost_delayedwork, 
				msecs_to_jiffies(boost_delay));
		}
	}

	MALI_DEBUG_PRINT(5, ("MALI GPU utilization: %u\n", mali_last_utilization));
}

#define ATTR_RO(_name)	\
	static struct kobj_attribute _name##_interface = __ATTR(_name, 0444, _name##_show, NULL);

#define ATTR_WO(_name)	\
	static struct kobj_attribute _name##_interface = __ATTR(_name, 0220, NULL, _name##_store);

#define ATTR_RW(_name)	\
	static struct kobj_attribute _name##_interface = __ATTR(_name, 0644, _name##_show, _name##_store);

static ssize_t version_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "DB8500 GPU OC Driver (%s), cocafe\n", MALI_UX500_VERSION);
}
ATTR_RO(version);

static ssize_t mali_gpu_clock_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d kHz\n", sgaclk_freq());
}

static ssize_t mali_gpu_clock_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val;
	int i;

	if (sscanf(buf, "idx=%u", &val)) {
		if (val >= ARRAY_SIZE(mali_dvfs))
			return -EINVAL;

		mali_clock_apply(val);

		return count;
	}

	if (sscanf(buf, "%u", &val)) {
		for (i = 0; i < ARRAY_SIZE(mali_dvfs); i++) {
			if (mali_dvfs[i].freq == val) {
				mali_clock_apply(i);

				break;
			}
		}

		return count;
	}

	return -EINVAL;
}

ATTR_RW(mali_gpu_clock);

static ssize_t mali_gpu_fullspeed_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	/*
	 * Check APE OPP status, on OPP50, clock is half.
	 */
	return sprintf(buf, "%s\n", (prcmu_get_ape_opp() == APE_100_OPP) ? "1" : "0");
}

static ssize_t mali_gpu_fullspeed_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val;

	if (sscanf(buf, "%d", &val)) {
		if (val)
			prcmu_set_ape_opp(APE_100_OPP);
		else
			prcmu_set_ape_opp(APE_50_OPP);

		return count;
	}

	return -EINVAL;
}

ATTR_RW(mali_gpu_fullspeed);

static ssize_t mali_gpu_load_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d (%d%%)\n", mali_last_utilization, mali_last_utilization * 100 / 256);
}
ATTR_RO(mali_gpu_load);

static ssize_t mali_gpu_vape_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	u8 value;

	/*
	 * cocafe:
	 * Display Vape Seletion 1 only, 
	 * In APE 50OPP, Vape uses SEL2. 
	 * And the clocks are half.
	 */
	prcmu_abb_read(AB8500_REGU_CTRL2, 
			AB8500_VAPE_SEL1, 
			&value, 
			1);

	return sprintf(buf, "%u uV\n", vape_voltage(value));
}
ATTR_RO(mali_gpu_vape);

static ssize_t mali_auto_boost_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", boost_enable);
}

static ssize_t mali_auto_boost_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	if(sysfs_streq(buf, "0")) {
		boost_enable = 0;

		if (boost_working) {
			cancel_delayed_work(&mali_boost_delayedwork);
			mali_clock_apply(boost_low);
			boost_required = 0;
			boost_working = 0;
		}
	} else {
		boost_enable = 1;
		mali_clock_apply(boost_low);
	}

	return count;
}
ATTR_RW(mali_auto_boost);

static ssize_t mali_boost_delay_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", boost_delay);
}

static ssize_t mali_boost_delay_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val;

	if (sscanf(buf, "%u", &val)) {
		boost_delay = val;
	}

	return count;
}
ATTR_RW(mali_boost_delay);

static ssize_t mali_boost_low_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	sprintf(buf, "%sDOWNthreshold: %u\n", buf, boost_downthreshold);
	sprintf(buf, "%sDVFS idx: %u\n", buf, boost_low);
	sprintf(buf, "%sfrequency: %u kHz\n", buf, mali_dvfs[boost_low].freq);
	sprintf(buf, "%sVape: %u uV\n", buf, vape_voltage(mali_dvfs[boost_low].vape_raw));

	return strlen(buf);
}

static ssize_t mali_boost_low_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val;
	int i;

	if (sscanf(buf, "idx=%u", &val)) {
		if (val >= ARRAY_SIZE(mali_dvfs))
			return -EINVAL;

		boost_low = val;
		if (boost_enable)
			mali_clock_apply(boost_low);

		return count;
	}

	if (sscanf(buf, "threshold=%u", &val)) {
		boost_downthreshold = val;

		return count;
	}

	if (sscanf(buf, "%u", &val)) {
		for (i = 0; i < ARRAY_SIZE(mali_dvfs); i++) {
			if (mali_dvfs[i].freq == val) {
				boost_low = i;
				if (boost_enable)
					mali_clock_apply(boost_low);

				break;
			}
		}

		return count;
	}

	return -EINVAL;
}
ATTR_RW(mali_boost_low);

static ssize_t mali_boost_high_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	sprintf(buf, "%sUPthreshold: %u\n", buf, boost_upthreshold);
	sprintf(buf, "%sDVFS idx: %u\n", buf, boost_high);
	sprintf(buf, "%sfrequency: %u kHz\n", buf, mali_dvfs[boost_high].freq);
	sprintf(buf, "%sVape: %u uV\n", buf, vape_voltage(mali_dvfs[boost_high].vape_raw));

	return strlen(buf);
}

static ssize_t mali_boost_high_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val;
	int i;

	if (sscanf(buf, "idx=%u", &val)) {
		if (val >= ARRAY_SIZE(mali_dvfs))
			return -EINVAL;

		boost_high = val;

		return count;
	}

	if (sscanf(buf, "threshold=%u", &val)) {
		boost_upthreshold = val;

		return count;
	}

	if (sscanf(buf, "%u", &val)) {
		for (i = 0; i < ARRAY_SIZE(mali_dvfs); i++) {
			if (mali_dvfs[i].freq == val) {
				boost_high = i;

				break;
			}
		}

		return count;
	}

	return -EINVAL;
}
ATTR_RW(mali_boost_high);

static ssize_t mali_dvfs_config_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int i;

	sprintf(buf, "idx freq   rawfreq clkpll     Vape\n");

	for (i = 0; i < ARRAY_SIZE(mali_dvfs); i++) {
		sprintf(buf, "%s%3u%7u%7u  %#010x%8u %#04x\n", 
			buf, 
			i, 
			mali_dvfs[i].freq, 
			pllsoc0_freq(mali_dvfs[i].clkpll), 
			mali_dvfs[i].clkpll, 
			vape_voltage(mali_dvfs[i].vape_raw), 
			mali_dvfs[i].vape_raw);
	}

	return strlen(buf);
}

static ssize_t mali_dvfs_config_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int idx, val;

	if (sscanf(buf, "%u pll=%x", &idx, &val) == 2) {
		mali_dvfs[idx].clkpll = val;

		return count;
	}

	if (sscanf(buf, "%u vape=%x", &idx, &val) == 2) {
		mali_dvfs[idx].vape_raw = val;

		return count;
	}

	return -EINVAL;
}
ATTR_RW(mali_dvfs_config);

static ssize_t mali_available_frequencies_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mali_dvfs); i++) {
		sprintf(buf, "%s%u\n", buf, mali_dvfs[i].freq);
	}

	return strlen(buf);
}
ATTR_RO(mali_available_frequencies);

static struct attribute *mali_attrs[] = {
	&version_interface.attr, 
	&mali_gpu_clock_interface.attr, 
	&mali_gpu_fullspeed_interface.attr, 
	&mali_gpu_load_interface.attr, 
	&mali_gpu_vape_interface.attr, 
	&mali_auto_boost_interface.attr, 
	&mali_boost_delay_interface.attr, 
	&mali_boost_low_interface.attr, 
	&mali_boost_high_interface.attr, 
	&mali_dvfs_config_interface.attr, 
	&mali_available_frequencies_interface.attr, 
	NULL,
};

static struct attribute_group mali_interface_group = {
	 /* .name  = "governor", */ /* Not using subfolder now */
	.attrs = mali_attrs,
};

static struct kobject *mali_kobject;

_mali_osk_errcode_t mali_platform_init()
{
	int ret;

	is_running = false;
	mali_last_utilization = 0;

	if (!is_initialized) {

		prcmu_write(PRCMU_PLLSOC0, PRCMU_PLLSOC0_INIT);
		prcmu_write(PRCMU_SGACLK,  PRCMU_SGACLK_INIT);
		mali_boost_init();

		mali_utilization_workqueue = create_singlethread_workqueue("mali_utilization_workqueue");
		if (NULL == mali_utilization_workqueue) {
			MALI_DEBUG_PRINT(2, ("%s: Failed to setup workqueue %s\n", __func__, "mali_utilization_workqueue"));
			goto error;
		}

		INIT_WORK(&mali_utilization_work, mali_utilization_function);
		INIT_DELAYED_WORK(&mali_boost_delayedwork, mali_boost_work);

		regulator = regulator_get(NULL, "v-mali");
		if (IS_ERR(regulator)) {
			MALI_DEBUG_PRINT(2, ("%s: Failed to get regulator %s\n", __func__, "v-mali"));
			goto error;
		}

		clk_sga = clk_get_sys("mali", NULL);
		if (IS_ERR(clk_sga)) {
			regulator_put(regulator);
			MALI_DEBUG_PRINT(2, ("%s: Failed to get clock %s\n", __func__, "mali"));
			goto error;
		}

#if CONFIG_HAS_WAKELOCK
		wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND, "mali_wakelock");
#endif

		mali_kobject = kobject_create_and_add("mali", kernel_kobj);
		if (!mali_kobject) {
			pr_err("[Mali] Failed to create kobject interface\n");
		}

		ret = sysfs_create_group(mali_kobject, &mali_interface_group);
		if (ret) {
			kobject_put(mali_kobject);
		}

		pr_info("[Mali] DB8500 GPU OC Initialized (%s)\n", MALI_UX500_VERSION);

		is_initialized = true;
	}

	MALI_SUCCESS;
error:
	MALI_DEBUG_PRINT(1, ("SGA initialization failed.\n"));
	MALI_ERROR(_MALI_OSK_ERR_FAULT);
}

_mali_osk_errcode_t mali_platform_deinit()
{
	destroy_workqueue(mali_utilization_workqueue);
	regulator_put(regulator);
	clk_put(clk_sga);

#if CONFIG_HAS_WAKELOCK
	wake_lock_destroy(&wakelock);
#endif
	kobject_put(mali_kobject);
	is_running = false;
	mali_last_utilization = 0;
	is_initialized = false;
	MALI_DEBUG_PRINT(2, ("SGA terminated.\n"));
	MALI_SUCCESS;
}

_mali_osk_errcode_t mali_platform_power_mode_change(mali_power_mode power_mode)
{
	if (MALI_POWER_MODE_ON == power_mode)
		return mali_platform_powerup();

	/*We currently don't make any distinction between MALI_POWER_MODE_LIGHT_SLEEP and MALI_POWER_MODE_DEEP_SLEEP*/
	return mali_platform_powerdown();
}

void mali_gpu_utilization_handler(u32 utilization)
{
	mali_last_utilization = utilization;
	/*
	* We should not cancel the potentially not yet run old work
	* in favor of a new work.
	* Since the utilization value will change,
	* the mali_utilization_function will evaluate based on
	* what is the utilization now and not on what it was
	* when it was scheduled.
	*/
	queue_work(mali_utilization_workqueue, &mali_utilization_work);
}

void set_mali_parent_power_domain(void *dev)
{
	MALI_DEBUG_PRINT(2, ("This function should not be called since we are not using run time pm\n"));
}
