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

static ssize_t janice_dvfs_ape_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{

	if (!strncmp(buf, "default", 7)) {
		pr_err("dvfs-dbg: APE OPP -> default\n");
		prcmu_qos_update_requirement(PRCMU_QOS_APE_OPP, QOS_NAME, PRCMU_QOS_DEFAULT_VALUE);

		return count;
	}

	if (!strncmp(buf, "max", 3)) {
		pr_err("dvfs-dbg: APE OPP -> MAX\n");
		prcmu_qos_update_requirement(PRCMU_QOS_APE_OPP, QOS_NAME, PRCMU_QOS_APE_OPP_MAX);

		return count;
	}
	
	return count;
}

static struct kobj_attribute janice_dvfs_ape_interface = __ATTR(ape_opp, 0200, NULL, janice_dvfs_ape_store);

static ssize_t janice_dvfs_ddr_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{

	if (!strncmp(buf, "default", 7)) {
		pr_err("dvfs-dbg: DDR OPP -> default\n");
		prcmu_qos_update_requirement(PRCMU_QOS_DDR_OPP, QOS_NAME, PRCMU_QOS_DEFAULT_VALUE);

		return count;
	}

	if (!strncmp(buf, "max", 3)) {
		pr_err("dvfs-dbg: DDR OPP -> MAX\n");
		prcmu_qos_update_requirement(PRCMU_QOS_DDR_OPP, QOS_NAME, PRCMU_QOS_DDR_OPP_MAX);

		return count;
	}
	
	return count;
}

static struct kobj_attribute janice_dvfs_ddr_interface = __ATTR(ddr_opp, 0200, NULL, janice_dvfs_ddr_store);

static ssize_t janice_dvfs_arm_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int freq;

	if (!strncmp(buf, "default", 7)) {
		pr_err("dvfs-dbg: ARM KHz -> default\n");
		prcmu_qos_update_requirement(PRCMU_QOS_ARM_KHZ, QOS_NAME, PRCMU_QOS_DEFAULT_VALUE);

		return count;
	}

	ret = sscanf(buf, "%d", &freq);

	if (ret > 0) {
		if (freq != 200000 && freq != 400000 && freq != 800000 && freq != 1000000)
			return count;

		pr_err("dvfs-dbg: ARM KHz -> %d\n", freq);
		prcmu_qos_update_requirement(PRCMU_QOS_ARM_KHZ, QOS_NAME, freq);
	}
	
	return count;
}

static struct kobj_attribute janice_dvfs_arm_interface = __ATTR(arm_khz, 0200, NULL, janice_dvfs_arm_store);

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
