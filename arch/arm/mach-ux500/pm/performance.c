/*
 * Copyright (C) ST-Ericsson SA 2011
 *
 * License Terms: GNU General Public License v2
 *
 * Author: Johan Rudholm <johan.rudholm@stericsson.com>
 * Author: Jonas Aaberg <jonas.aberg@stericsson.com>
 */

#include <linux/kernel.h>
#include <linux/genhd.h>
#include <linux/major.h>
#include <linux/cdev.h>
#include <linux/kernel_stat.h>
#include <linux/workqueue.h>
#include <linux/kernel.h>
#include <linux/mfd/dbx500-prcmu.h>
#include <linux/cpu.h>
#include <linux/pm_qos_params.h>

#include <mach/irqs.h>

#define WLAN_PROBE_DELAY_DEFAULT 3000 /* 3 seconds */
#define WLAN_LIMIT_DEFAULT (3000/3) /* If we have more than 1000 irqs per second */

int wlan_probe_delay = WLAN_PROBE_DELAY_DEFAULT;
int wlan_limit = WLAN_LIMIT_DEFAULT;
int wlan_arm_khz = 400000;

/*
 * MMC TODO:
 * o Develop a more power-aware algorithm
 * o Make the parameters visible through debugfs
 * o Get the value of CONFIG_MMC_BLOCK_MINORS in runtime instead, since
 *   it may be altered by drivers/mmc/card/block.c
 */

#define ATTR_RO(_name)	\
	static struct kobj_attribute _name##_interface = __ATTR(_name, 0444, _name##_show, NULL);

#define ATTR_WO(_name)	\
	static struct kobj_attribute _name##_interface = __ATTR(_name, 0220, NULL, _name##_store);

#define ATTR_RW(_name)	\
	static struct kobj_attribute _name##_interface = __ATTR(_name, 0644, _name##_show, _name##_store);

#define INT_PARAM_RW(_name, _variable)  \
static ssize_t _name##_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)	\
{	\
	return sprintf(buf, "%d\n", _variable); \
}	\
	\
static ssize_t _name##_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)	\
{	\
	if (sscanf(buf, "%d", &_variable)) \
		return count; \
	else \
		return -EINVAL; \
}	\
ATTR_RW(_name);

INT_PARAM_RW(wlan_probe_delay, wlan_probe_delay);
INT_PARAM_RW(wlan_limit, wlan_limit);
INT_PARAM_RW(wlan_arm_khz, wlan_arm_khz);

/* Sample reads and writes every n ms */
#define PERF_MMC_PROBE_DELAY_DEFAULT 1000
/* Read threshold, sectors/second */
#define PERF_MMC_LIMIT_READ_DEFAULT 8192
/* Write threshold, sectors/second */
#define PERF_MMC_LIMIT_WRITE_DEFAULT 4096
/* Nr of MMC devices */
#define PERF_MMC_HOSTS 8

int perf_mmc_probe_delay = PERF_MMC_PROBE_DELAY_DEFAULT;
int perf_mmc_limit_read = PERF_MMC_LIMIT_READ_DEFAULT;
int perf_mmc_limit_write = PERF_MMC_LIMIT_WRITE_DEFAULT;
int perf_mmc_limit_combined = 8192;
int perf_mmc_arm_khz = 400000;

INT_PARAM_RW(mmc_probe_delay, perf_mmc_probe_delay);
INT_PARAM_RW(mmc_limit_read, perf_mmc_limit_read);
INT_PARAM_RW(mmc_limit_write, perf_mmc_limit_write);
INT_PARAM_RW(mmc_limit_combined, perf_mmc_limit_combined);
INT_PARAM_RW(mmc_arm_khz, perf_mmc_arm_khz);

/*
 * Rescan for new MMC devices every
 * perf_mmc_probe_delay * perf_mmc_rescan_cycles ms
 */
#define PERF_MMC_RESCAN_CYCLES_DEFAULT 10

int perf_mmc_rescan_cycles = PERF_MMC_RESCAN_CYCLES_DEFAULT;

INT_PARAM_RW(mmc_rescan_cycles, perf_mmc_rescan_cycles);

static struct attribute *performance_attrs[] = {
	&wlan_probe_delay_interface.attr,
	&wlan_limit_interface.attr,
	&wlan_arm_khz_interface.attr,
	&mmc_probe_delay_interface.attr,
	&mmc_limit_read_interface.attr,
	&mmc_limit_write_interface.attr,
	&mmc_limit_combined_interface.attr,
	&mmc_rescan_cycles_interface.attr,
	&mmc_arm_khz_interface.attr,
	NULL,
};

static struct attribute_group performance_interface_group = {
	.attrs = performance_attrs,
};

static struct kobject *performance_kobject;

#ifdef CONFIG_MMC_BLOCK
static struct delayed_work work_mmc;
#endif

static struct delayed_work work_wlan_workaround;
static struct pm_qos_request_list wlan_pm_qos_latency;
static bool wlan_pm_qos_is_latency_0;

static void wlan_load(struct work_struct *work)
{
	int cpu;
	unsigned int num_irqs = 0;
	static unsigned int old_num_irqs = UINT_MAX;

	for_each_online_cpu(cpu)
		num_irqs += kstat_irqs_cpu(IRQ_DB8500_SDMMC1, cpu);

	if ((num_irqs > old_num_irqs) &&
	    (num_irqs - old_num_irqs) > wlan_limit) {
		if (wlan_arm_khz)
		prcmu_qos_update_requirement(PRCMU_QOS_ARM_KHZ,
					     "wlan",
					     wlan_arm_khz);
		prcmu_qos_update_requirement(PRCMU_QOS_APE_OPP,
					     "wlan",
					     PRCMU_QOS_MAX_VALUE);
		prcmu_qos_update_requirement(PRCMU_QOS_DDR_OPP,
					     "wlan",
					     PRCMU_QOS_MAX_VALUE);
		if (!wlan_pm_qos_is_latency_0) {
			/*
			 * The wake up latency is set to 0 to prevent
			 * the system from going to sleep. This improves
			 * the wlan throughput in DMA mode.
			 * The wake up latency from sleep adds ~5% overhead
			 * for TX in some cases.
			 * This change doesn't increase performance for wlan
			 * PIO since the CPU usage prevents sleep in this mode.
			 */
			pm_qos_add_request(&wlan_pm_qos_latency,
					   PM_QOS_CPU_DMA_LATENCY, 0);
			wlan_pm_qos_is_latency_0 = true;
		}
	} else {
		prcmu_qos_update_requirement(PRCMU_QOS_ARM_KHZ,
					     "wlan",
					     PRCMU_QOS_DEFAULT_VALUE);
		prcmu_qos_update_requirement(PRCMU_QOS_APE_OPP,
					     "wlan",
					     PRCMU_QOS_DEFAULT_VALUE);
		prcmu_qos_update_requirement(PRCMU_QOS_DDR_OPP,
					     "wlan",
					     PRCMU_QOS_DEFAULT_VALUE);
		if (wlan_pm_qos_is_latency_0) {
			pm_qos_remove_request(&wlan_pm_qos_latency);
			wlan_pm_qos_is_latency_0 = false;
		}
	}

	old_num_irqs = num_irqs;

	schedule_delayed_work_on(0,
				 &work_wlan_workaround,
				 msecs_to_jiffies(wlan_probe_delay));
}

#ifdef CONFIG_MMC_BLOCK
/*
 * Loop through every CONFIG_MMC_BLOCK_MINORS'th minor device for
 * MMC_BLOCK_MAJOR, get the struct gendisk for each device. Returns
 * nr of found disks. Populate mmc_disks.
 */
static int scan_mmc_devices(struct gendisk *mmc_disks[])
{
	dev_t devnr;
	int i, j = 0, part;
	struct gendisk *mmc_devices[256 / CONFIG_MMC_BLOCK_MINORS];

	memset(&mmc_devices, 0, sizeof(mmc_devices));

	for (i = 0; i * CONFIG_MMC_BLOCK_MINORS < 256; i++) {
		devnr = MKDEV(MMC_BLOCK_MAJOR, i * CONFIG_MMC_BLOCK_MINORS);
		mmc_devices[i] = get_gendisk(devnr, &part);

		/* Invalid capacity of device, do not add to list */
		if (!mmc_devices[i] || !get_capacity(mmc_devices[i]))
			continue;

		mmc_disks[j] = mmc_devices[i];
		j++;

		if (j == PERF_MMC_HOSTS)
			break;
	}

	return j;
}

/*
 * Sample sectors read and written to any MMC devices, update PRCMU
 * qos requirement
 */
static void mmc_load(struct work_struct *work)
{
	static unsigned long long old_sectors_read[PERF_MMC_HOSTS];
	static unsigned long long old_sectors_written[PERF_MMC_HOSTS];
	static struct gendisk *mmc_disks[PERF_MMC_HOSTS];
	static int cycle, nrdisk;
	static bool old_mode;
	unsigned long long sectors;
	bool new_mode = false;
	int i;
	long dr, dw;

	if (!cycle) {
		memset(&mmc_disks, 0, sizeof(mmc_disks));
		nrdisk = scan_mmc_devices(mmc_disks);
		cycle = perf_mmc_rescan_cycles;
	}
	cycle--;

	for (i = 0; i < nrdisk; i++) {
		sectors = part_stat_read(&(mmc_disks[i]->part0),
						sectors[READ]);

		dr = sectors - old_sectors_read[i];
		if (dr < 0) dr = 0;
		if (old_sectors_read[i] &&
			dr > perf_mmc_limit_read*perf_mmc_probe_delay/1000)
			new_mode = true;

		old_sectors_read[i] = sectors;
		sectors = part_stat_read(&(mmc_disks[i]->part0),
						sectors[WRITE]);

		dw = sectors - old_sectors_read[i];
		if (dw < 0) dw = 0;
		if (old_sectors_written[i] &&
			dw > perf_mmc_limit_write*perf_mmc_probe_delay/1000)
			new_mode = true;

		old_sectors_written[i] = sectors;

		if (dr + dw > perf_mmc_limit_combined*perf_mmc_probe_delay/1000)
		       new_mode = true;
	}

	if (!old_mode && new_mode) {
		if (perf_mmc_arm_khz)
		prcmu_qos_update_requirement(PRCMU_QOS_ARM_KHZ,
					     "mmc",
					     perf_mmc_arm_khz);
		prcmu_qos_update_requirement(PRCMU_QOS_APE_OPP,
					     "mmc",
					     PRCMU_QOS_MAX_VALUE);
		prcmu_qos_update_requirement(PRCMU_QOS_DDR_OPP,
					     "mmc",
					     PRCMU_QOS_MAX_VALUE);
	}

	if (old_mode && !new_mode) {
		prcmu_qos_update_requirement(PRCMU_QOS_ARM_KHZ,
					     "mmc",
					     PRCMU_QOS_DEFAULT_VALUE);
		prcmu_qos_update_requirement(PRCMU_QOS_APE_OPP,
					     "mmc",
					     PRCMU_QOS_DEFAULT_VALUE);
		prcmu_qos_update_requirement(PRCMU_QOS_DDR_OPP,
					     "mmc",
					     PRCMU_QOS_DEFAULT_VALUE);
	}

	old_mode = new_mode;

	schedule_delayed_work(&work_mmc,
				 msecs_to_jiffies(perf_mmc_probe_delay));

}
#endif /* CONFIG_MMC_BLOCK */

static int __init performance_register(void)
{
	int ret_mmc;
	int ret_wlan;
	int ret;

#ifdef CONFIG_MMC_BLOCK
	ret_mmc = prcmu_qos_add_requirement(PRCMU_QOS_ARM_KHZ, "mmc",
					    PRCMU_QOS_DEFAULT_VALUE);
	if (!ret_mmc)
		ret_mmc = prcmu_qos_add_requirement(PRCMU_QOS_APE_OPP, "mmc",
						    PRCMU_QOS_DEFAULT_VALUE);
	if (!ret_mmc)
		ret_mmc = prcmu_qos_add_requirement(PRCMU_QOS_DDR_OPP, "mmc",
						    PRCMU_QOS_DEFAULT_VALUE);
	if (ret_mmc) {
		pr_err("%s: Failed to add PRCMU QoS req for mmc\n", __func__);
		prcmu_qos_remove_requirement(PRCMU_QOS_ARM_KHZ, "mmc");
		prcmu_qos_remove_requirement(PRCMU_QOS_APE_OPP, "mmc");
		prcmu_qos_remove_requirement(PRCMU_QOS_DDR_OPP, "mmc");
	} else {
		INIT_DELAYED_WORK_DEFERRABLE(&work_mmc, mmc_load);
		schedule_delayed_work(&work_mmc,
				      msecs_to_jiffies(perf_mmc_probe_delay));
	}
#endif

	ret_wlan = prcmu_qos_add_requirement(PRCMU_QOS_ARM_KHZ, "wlan",
					     PRCMU_QOS_DEFAULT_VALUE);
	if (!ret_wlan)
		ret_wlan = prcmu_qos_add_requirement(PRCMU_QOS_APE_OPP, "wlan",
						     PRCMU_QOS_DEFAULT_VALUE);
	if (!ret_wlan)
		ret_wlan = prcmu_qos_add_requirement(PRCMU_QOS_DDR_OPP, "wlan",
						     PRCMU_QOS_DEFAULT_VALUE);
	if (ret_wlan) {
		pr_err("%s: Failed to add PRCMU QoS req for wlan\n", __func__);
		prcmu_qos_remove_requirement(PRCMU_QOS_ARM_KHZ, "wlan");
		prcmu_qos_remove_requirement(PRCMU_QOS_APE_OPP, "wlan");
		prcmu_qos_remove_requirement(PRCMU_QOS_DDR_OPP, "wlan");
	} else {
		INIT_DELAYED_WORK_DEFERRABLE(&work_wlan_workaround, wlan_load);
		schedule_delayed_work_on(0, &work_wlan_workaround,
					 msecs_to_jiffies(wlan_probe_delay));
	}

	performance_kobject = kobject_create_and_add("performance", kernel_kobj);
	if (!performance_kobject) {
		pr_err("[Performance] Failed to create kobject interface\n");
	}
	ret = sysfs_create_group(performance_kobject, &performance_interface_group);
	if (ret) {
		kobject_put(performance_kobject);
	}

	/* Only return one error code */
	return ret_mmc ? ret_mmc : (ret_wlan ? ret_wlan : ret);
}
late_initcall(performance_register);
