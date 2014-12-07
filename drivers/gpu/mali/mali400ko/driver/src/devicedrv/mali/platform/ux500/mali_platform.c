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

#define MALI_UX500_VERSION		"1.0.2"

#define MALI_MAX_UTILIZATION		256

#define PRCMU_SGACLK			0x0014
#define PRCMU_PLLSOC0			0x0080

#define PRCMU_SGACLK_INIT		0x00000021
#define PRCMU_PLLSOC0_INIT		0x00050134

#define AB8500_VAPE_SEL1 		0x0E
#define AB8500_VAPE_SEL2	 	0x0F
#define AB8500_VAPE_STEP_UV		12500
#define AB8500_VAPE_MIN_UV		700000
#define AB8500_VAPE_MAX_UV		1362500

#define MALI_CLOCK_DEFLO		399360
#define MALI_CLOCK_DEFHI		499200

struct mali_dvfs_data
{
	u32 	freq;
	u32 	clkpll;
	u8 	vape_raw;
};

static struct mali_dvfs_data mali_dvfs[] = {
	{ 99840, 0x0005010d, 0x26},
	{107520, 0x0005010e, 0x26},
	{115200, 0x0005010f, 0x26},
	{122880, 0x00050110, 0x26},
	{130560, 0x00050111, 0x26},
	{138240, 0x00050112, 0x26},
	{145920, 0x00050113, 0x26},
	{153600, 0x00050114, 0x26},
	{161280, 0x00050115, 0x26},
	{168960, 0x00050116, 0x26},
	{176640, 0x00050117, 0x26},
	{184320, 0x00050118, 0x26},
	{192000, 0x00050119, 0x26},
	{199680, 0x0005011a, 0x26},
	{207360, 0x0005011b, 0x26},
	{215040, 0x0005011c, 0x26},
	{222720, 0x0005011d, 0x26},
	{230400, 0x0005011e, 0x26},
	{238080, 0x0005011f, 0x26},
	{245760, 0x00050120, 0x26},
	{253440, 0x00050121, 0x26},
	{261120, 0x00050122, 0x26},
	{268800, 0x00050123, 0x26},
	{276480, 0x00050124, 0x26},
	{284160, 0x00050125, 0x26},
	{291840, 0x00050126, 0x26},
	{299520, 0x00050127, 0x26},
	{307200, 0x00050128, 0x26},
	{314880, 0x00050129, 0x26},
	{322560, 0x0005012a, 0x26},
	{330240, 0x0005012b, 0x26},
	{337920, 0x0005012c, 0x26},
	{345600, 0x0005012d, 0x26},
	{353280, 0x0005012e, 0x26},
	{360960, 0x0005012f, 0x26},
	{368640, 0x00050130, 0x26},
	{376320, 0x00050131, 0x26},
	{384000, 0x00050132, 0x26},
	{391680, 0x00050133, 0x26},
	{399360, 0x00050134, 0x26},
	{407040, 0x00050135, 0x26},
	{414720, 0x00050136, 0x26},
	{422400, 0x00050137, 0x29},
	{430080, 0x00050138, 0x29},
	{437760, 0x00050139, 0x2B},
	{445440, 0x0005013a, 0x2B},
	{453120, 0x0005013b, 0x2C},
	{460800, 0x0005013c, 0x2C},
	{468480, 0x0005013d, 0x2D},
	{476160, 0x0005013e, 0x2D},
	{483840, 0x0005013f, 0x2E},
	{491520, 0x00050140, 0x2E},
	{499200, 0x00050141, 0x2F},
	{506880, 0x00050142, 0x2F},
	{514560, 0x00050143, 0x2F},
	{522240, 0x00050144, 0x30},
	{529920, 0x00050145, 0x30},
	{537600, 0x00050146, 0x31},
	{545280, 0x00050147, 0x31},
	{552960, 0x00050148, 0x31},
	{560640, 0x00050149, 0x32},
	{568320, 0x0005014a, 0x32},
	{576000, 0x0005014b, 0x32},
	{583680, 0x0005014c, 0x33},
	{591360, 0x0005014d, 0x33},
	{599040, 0x0005014e, 0x33},
	{606720, 0x0005014f, 0x34},
	{614400, 0x00050150, 0x34},
	{622080, 0x00050151, 0x34},
	{629760, 0x00050152, 0x34},
	{637440, 0x00050153, 0x34},
	{645120, 0x00050154, 0x34},
	{652800, 0x00050155, 0x34},
	{660480, 0x00050156, 0x3F},
	{668160, 0x00050157, 0x3F},
	{675840, 0x00050158, 0x3F},
	{683520, 0x00050159, 0x3F},
	{691200, 0x0005015a, 0x3F},
	{698880, 0x0005015b, 0x3F},
	{706560, 0x0005015c, 0x3F},
	{714240, 0x0005015d, 0x3F},
	{721920, 0x0005015e, 0x3F},
	{729600, 0x0005015f, 0x3F},
	{737280, 0x00050160, 0x3F},
	{744960, 0x00050161, 0x3F},
	{752640, 0x00050162, 0x3F},
	{760320, 0x00050163, 0x3F},
	{768000, 0x00050164, 0x3F},
	{775680, 0x00050165, 0x3F},
	{783360, 0x00050166, 0x3F},
	{791040, 0x00050167, 0x3F},
	{798720, 0x00050168, 0x3F},
	{806400, 0x00050169, 0x3F},
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
static u32 boost_scheduled 	= 0;
static u32 boost_required 	= 0;
static u32 boost_delay 		= 200;
static u32 boost_low 		= 0;
static u32 boost_high 		= 0;
static u32 boost_upthreshold 	= 233;
static u32 boost_downthreshold 	= 64;
//mutex to protect above variables
static DEFINE_MUTEX(mali_boost_lock);

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

static void mali_boost_update(void)
{
	u8 vape;
	u32 pll;

	if (boost_required) {
		vape = mali_dvfs[boost_high].vape_raw;
		pll = mali_dvfs[boost_high].clkpll;

		pr_err("[Mali] @%u kHz - Boost\n", mali_dvfs[boost_high].freq);

		prcmu_abb_write(AB8500_REGU_CTRL2,
			AB8500_VAPE_SEL1,
			&vape,
			1);
		prcmu_write(PRCMU_PLLSOC0, pll);
		boost_working = true;
	} else {
		vape = mali_dvfs[boost_low].vape_raw;
		pll = mali_dvfs[boost_low].clkpll;

		pr_err("[Mali] @%u kHz - Deboost\n", mali_dvfs[boost_low].freq);

		prcmu_write(PRCMU_PLLSOC0, pll);
		prcmu_abb_write(AB8500_REGU_CTRL2,
			AB8500_VAPE_SEL1,
			&vape,
			1);

		boost_working = false;
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
	mutex_lock(&mali_boost_lock);
	mali_boost_update();
	boost_scheduled = false;
	mutex_unlock(&mali_boost_lock);
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

	if (sgaclk_freq() != mali_dvfs[boost_low].freq) {
		mali_clock_apply(boost_low);
	}

	pr_info("[Mali] Booster: %u kHz - %u kHz\n", 
			mali_dvfs[boost_low].freq, 
			mali_dvfs[boost_high].freq);
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

/* Rationale behind the values for: (switching between APE_50_OPP and APE_100_OPP)
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

/* boost switching logic:
 * - boost_working means that freq is already set to high value
 * - boost_scheduled means that job is scheduled to turn boost either on or off
 * - boost_required is a flag for scheduled job telling it what to do with boost
 *
 * if we are in APE_50_OPP, skip boost
 * if we are in APE_100_OPP and util>boost_up_thresh, shedule boost if its not on or - if its on and scheduled to be turned off -  cancel that schedule
 * if boost is scheduled and not yet working and util < util_high_to_low, then cancel scheduled boost
 * if boost is on and util < boost_down_thresh, schedule boost to be turned off
 */
void mali_utilization_function(struct work_struct *ptr)
{
	/*By default, platform start with 50% APE OPP and 25% DDR OPP*/
	static u32 has_requested_low = 1;

	MALI_DEBUG_PRINT(5, ("MALI GPU utilization: %u\n", mali_last_utilization));

	mutex_lock(&mali_boost_lock);
	if ((!boost_required && !boost_working && !boost_scheduled) || !boost_enable) {
		// consider power saving mode (APE_50_OPP) only if we're not on boost
		if (mali_last_utilization > mali_utilization_low_to_high) {
			if (has_requested_low) {
				MALI_DEBUG_PRINT(5, ("MALI GPU utilization: %u SIGNAL_HIGH\n", mali_last_utilization));
				/*Request 100% APE_OPP.*/
				prcmu_qos_update_requirement(PRCMU_QOS_APE_OPP, "mali", PRCMU_QOS_MAX_VALUE);
				/*
				* Since the utilization values will be reported higher
				* if DDR_OPP is lowered, we also request 100% DDR_OPP.
				*/
				prcmu_qos_update_requirement(PRCMU_QOS_DDR_OPP, "mali", PRCMU_QOS_MAX_VALUE);
				has_requested_low = 0;
				mutex_unlock(&mali_boost_lock);
				return;		//After we switch to APE_100_OPP we want to measure utilization once again before entering boost logic
			}
		} else {
			if (mali_last_utilization < mali_utilization_high_to_low) {
				if (!has_requested_low) {
					/*Remove APE_OPP and DDR_OPP requests*/
					prcmu_qos_update_requirement(PRCMU_QOS_DDR_OPP, "mali", PRCMU_QOS_DEFAULT_VALUE);
					prcmu_qos_update_requirement(PRCMU_QOS_APE_OPP, "mali", PRCMU_QOS_DEFAULT_VALUE);
					MALI_DEBUG_PRINT(5, ("MALI GPU utilization: %u SIGNAL_LOW\n", mali_last_utilization));
					has_requested_low = 1;
				}
			}
		}
	}

	if (!has_requested_low && boost_enable) {
		// consider boost only if we are in APE_100_OPP mode
		if (!boost_required && mali_last_utilization > boost_upthreshold) {
			boost_required = true;
			if (!boost_scheduled) {
				//schedule job to turn boost on
				boost_scheduled = true;
				schedule_delayed_work(&mali_boost_delayedwork, msecs_to_jiffies(boost_delay));
			} else {
				//cancel job meant to turn boost off
				boost_scheduled = false;
				cancel_delayed_work(&mali_boost_delayedwork);
			}
		} else if (boost_required && !boost_working && mali_last_utilization < boost_downthreshold) {
			boost_required = false;
			if (boost_scheduled) {
				//if it's not working yet, but is scheduled to be turned on, than cancel scheduled job
				cancel_delayed_work(&mali_boost_delayedwork);
				boost_scheduled = false;
			}
		} else if (boost_working && mali_last_utilization < boost_downthreshold) {
			boost_required = false;
			if (!boost_scheduled) {
				// if boost is on and isn't yet scheduled to be turned off then schedule it
				boost_scheduled = true;
				schedule_delayed_work(&mali_boost_delayedwork, msecs_to_jiffies(boost_delay));
			}
		}
	}
	mutex_unlock(&mali_boost_lock);

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

static ssize_t mali_gpu_vape_50_opp_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	u8 value;

	prcmu_abb_read(AB8500_REGU_CTRL2,
			AB8500_VAPE_SEL2,
			&value,
			1);

	return sprintf(buf, "%u uV - 0x%x\n", vape_voltage(value), value);
}

static ssize_t mali_gpu_vape_50_opp_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val;
	u8 vape;

	if (sscanf(buf, "%x", &val)) {
		vape = val;
		prcmu_abb_write(AB8500_REGU_CTRL2,
			AB8500_VAPE_SEL2,
			&vape,
			1);
		return count;
	}

	return -EINVAL;
}

ATTR_RW(mali_gpu_vape_50_opp);

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
	bool opp50;

	/*
	 * cocafe:
	 * Display Vape Seletion 1 only, 
	 * In APE 50OPP, Vape uses SEL2. 
	 * And the clocks are half.
	 */
	opp50 = (prcmu_get_ape_opp() != APE_100_OPP);
	prcmu_abb_read(AB8500_REGU_CTRL2, 
			opp50 ? AB8500_VAPE_SEL2 : AB8500_VAPE_SEL1,
			&value, 
			1);

	return sprintf(buf, "%u uV - 0x%x (OPP:%d)\n", vape_voltage(value), value, opp50 ? 50 : 100);
}
ATTR_RO(mali_gpu_vape);

static ssize_t mali_auto_boost_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "enabled: %d, required: %d, scheduled: %d, working: %d\n", boost_enable, boost_required, boost_scheduled, boost_working);
}

static ssize_t mali_auto_boost_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	if(sysfs_streq(buf, "0")) {
		boost_enable = 0;

		if (boost_scheduled) {
			cancel_delayed_work(&mali_boost_delayedwork);
			boost_scheduled = false;
		}
		if (boost_working) {
			boost_working = false;
			mali_clock_apply(boost_low);
		}
		boost_required = 0;
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
	&mali_gpu_vape_50_opp_interface.attr,
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

		prcmu_qos_add_requirement(PRCMU_QOS_APE_OPP, "mali", PRCMU_QOS_DEFAULT_VALUE);
		prcmu_qos_add_requirement(PRCMU_QOS_DDR_OPP, "mali", PRCMU_QOS_DEFAULT_VALUE);

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
	prcmu_qos_remove_requirement(PRCMU_QOS_APE_OPP, "mali");
	prcmu_qos_remove_requirement(PRCMU_QOS_DDR_OPP, "mali");
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
