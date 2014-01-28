/*
 * License terms: GNU General Public License (GPL) version 2
 * 
 * Author: Huang Ji (cocafe) <cocafehj@gmail.com>
 * 
 */

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/mfd/dbx500-prcmu.h>

#define QOS_NAME		"dvfs-dbg"

enum janice_dvfs_list
{
	DVFS_APE = 0x00,
	DVFS_DDR,
	DVFS_ARM,
};

enum janice_dvfs_state
{
	DEFAULT = 0x00,
	REQUEST, 
};

struct janice_dvfs_info
{
	int state;
	int freq;
};

static struct janice_dvfs_info dvfs_info[] = {
	[DVFS_APE] = {
		.state = DEFAULT,
	},
	[DVFS_DDR] = {
		.state = DEFAULT,
	},
	[DVFS_ARM] = {
		.state = DEFAULT,
		.freq =	0,
	},
};

static ssize_t janice_dvfs_ape_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	sprintf(buf, "[%s] default\n", dvfs_info[DVFS_APE].state ? " " : "*");
	sprintf(buf, "%s[%s] max\n", buf, dvfs_info[DVFS_APE].state ? "*" : " ");

	return strlen(buf);
}

static ssize_t janice_dvfs_ape_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{

	if (!strncmp(buf, "default", 7)) {
		pr_err("[DVFS-DBG]: APE OPP -> default\n");
		dvfs_info[DVFS_APE].state = DEFAULT;
		prcmu_qos_update_requirement(PRCMU_QOS_APE_OPP, QOS_NAME, PRCMU_QOS_DEFAULT_VALUE);

		return count;
	}

	if (!strncmp(buf, "max", 3)) {
		pr_err("[DVFS-DBG]: APE OPP -> MAX\n");
		dvfs_info[DVFS_APE].state = REQUEST;
		prcmu_qos_update_requirement(PRCMU_QOS_APE_OPP, QOS_NAME, PRCMU_QOS_APE_OPP_MAX);

		return count;
	}
	
	return count;
}

static struct kobj_attribute janice_dvfs_ape_interface = __ATTR(ape_opp, 0644, janice_dvfs_ape_show, janice_dvfs_ape_store);

static ssize_t janice_dvfs_ddr_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	sprintf(buf, "[%s] default\n", dvfs_info[DVFS_DDR].state ? " " : "*");
	sprintf(buf, "%s[%s] max\n", buf, dvfs_info[DVFS_DDR].state ? "*" : " ");

	return strlen(buf);
}

static ssize_t janice_dvfs_ddr_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{

	if (!strncmp(buf, "default", 7)) {
		pr_err("[DVFS-DBG]: DDR OPP -> default\n");
		dvfs_info[DVFS_DDR].state = DEFAULT;
		prcmu_qos_update_requirement(PRCMU_QOS_DDR_OPP, QOS_NAME, PRCMU_QOS_DEFAULT_VALUE);

		return count;
	}

	if (!strncmp(buf, "max", 3)) {
		pr_err("[DVFS-DBG]: DDR OPP -> MAX\n");
		dvfs_info[DVFS_DDR].state = REQUEST;
		prcmu_qos_update_requirement(PRCMU_QOS_DDR_OPP, QOS_NAME, PRCMU_QOS_DDR_OPP_MAX);

		return count;
	}
	
	return count;
}

static struct kobj_attribute janice_dvfs_ddr_interface = __ATTR(ddr_opp, 0644, janice_dvfs_ddr_show, janice_dvfs_ddr_store);

static ssize_t janice_dvfs_arm_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	sprintf(buf, "[%s] default\n", dvfs_info[DVFS_ARM].state ? " " : "*");
	sprintf(buf, "%s[%s] freq: %d\n", buf, dvfs_info[DVFS_ARM].state ? "*" : " ", dvfs_info[DVFS_ARM].freq);

	return strlen(buf);
}

static ssize_t janice_dvfs_arm_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int freq;

	if (!strncmp(buf, "default", 7)) {
		pr_err("[DVFS-DBG]: ARM KHz -> default\n");
		dvfs_info[DVFS_ARM].state = DEFAULT;
		prcmu_qos_update_requirement(PRCMU_QOS_ARM_KHZ, QOS_NAME, PRCMU_QOS_DEFAULT_VALUE);

		return count;
	}

	ret = sscanf(buf, "%d", &freq);

	if (ret > 0) {
		pr_err("[DVFS-DBG]: ARM KHz -> %d\n", freq);
		dvfs_info[DVFS_ARM].freq = freq;
		dvfs_info[DVFS_ARM].state = REQUEST;
		prcmu_qos_update_requirement(PRCMU_QOS_ARM_KHZ, QOS_NAME, freq);
	}
	
	return count;
}

static struct kobj_attribute janice_dvfs_arm_interface = __ATTR(arm_khz, 0644, janice_dvfs_arm_show, janice_dvfs_arm_store);

static struct attribute *janice_dvfs_debug_attrs[] = {
	&janice_dvfs_ape_interface.attr, 
	&janice_dvfs_ddr_interface.attr, 
	&janice_dvfs_arm_interface.attr, 
	NULL,
};

static struct attribute_group janice_dvfs_debug_interface_group = {
	.attrs = janice_dvfs_debug_attrs,
};

static struct kobject *janice_dvfs_debug_kobject;

static int __init janice_dvfs_debug_init(void)
{
	int ret;

	prcmu_qos_add_requirement(PRCMU_QOS_APE_OPP, QOS_NAME,
				  PRCMU_QOS_DEFAULT_VALUE);
	prcmu_qos_add_requirement(PRCMU_QOS_DDR_OPP, QOS_NAME,
				  PRCMU_QOS_DEFAULT_VALUE);
	prcmu_qos_add_requirement(PRCMU_QOS_ARM_KHZ, QOS_NAME,
				  PRCMU_QOS_DEFAULT_VALUE);

	janice_dvfs_debug_kobject = kobject_create_and_add("dvfs", kernel_kobj);
	if (!janice_dvfs_debug_kobject) {
		return -ENOMEM;
	}

	ret = sysfs_create_group(janice_dvfs_debug_kobject, &janice_dvfs_debug_interface_group);

	if (ret) {
		kobject_put(janice_dvfs_debug_kobject);
	}

	return ret;
}
module_init(janice_dvfs_debug_init);
