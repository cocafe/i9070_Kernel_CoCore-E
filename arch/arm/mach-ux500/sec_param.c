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
#include <mach/sec_common.h>
#include <mach/sec_param.h>

static ssize_t sec_param_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	if (sec_get_param_value) {
		signed int value = 0;
		signed char str_val[512];

		sprintf(buf, "  <SecBootloader Parameters>:  \n\n");

		sprintf(buf, "%s  PARAM_MAGIC  : 0x72726624\n", buf);
		sprintf(buf, "%s  PARAM_VERSION  : 0x13 Rev 1.3\n", buf);

		sec_get_param_value(__SERIAL_SPEED, &value);
		sprintf(buf, "%s  - 00. SERIAL_SPEED  : %d\n", buf, value);
		sec_get_param_value(__LOAD_TESTKERNEL, &value);
		sprintf(buf, "%s  - 01. LOAD_TESTKERNEL  : %d\n", buf, value);
		sec_get_param_value(__BOOT_DELAY, &value);
		sprintf(buf, "%s  - 02. BOOT_DELAY  : %d\n", buf, value);
		sec_get_param_value(__LCD_LEVEL, &value);
		sprintf(buf, "%s  - 03. LCD_LEVEL  : %d\n", buf, value);
		sec_get_param_value(__SWITCH_SEL, &value);
		sprintf(buf, "%s  - 04. SWITCH_SEL  : %d\n", buf, value);
		sec_get_param_value(__PHONE_DEBUG_ON, &value);
		sprintf(buf, "%s  - 05. PHONE_DEBUG_ON  : %d\n", buf, value);
		sec_get_param_value(__LCD_DIM_LEVEL, &value);
		sprintf(buf, "%s  - 06. LCD_DIM_LEVEL  : %d\n", buf, value);
		sec_get_param_value(__LCD_DIM_TIME, &value);
		sprintf(buf, "%s  - 07. LCD_DIM_TIME  : %d\n", buf, value);
		sec_get_param_value(__FORCE_PRERECOVERY, &value);
		sprintf(buf, "%s  - 08. FORCE_PRERECOVERY  : %d\n", buf, value);
		sec_get_param_value(__REBOOT_MODE, &value);
		sprintf(buf, "%s  - 09. REBOOT_MODE  : %d\n", buf, value);
		sec_get_param_value(__NATION_SEL, &value);
		sprintf(buf, "%s  - 10. NATION_SEL  : %d\n", buf, value);
		sec_get_param_value(__DEBUG_LEVEL, &value);
		sprintf(buf, "%s  - 11. DEBUG_LEVEL  : %d\n", buf, value);
		sec_get_param_value(__SET_DEFAULT_PARAM, &value);
		sprintf(buf, "%s  - 12. SET_DEFAULT_PARAM  : %d\n", buf, value);
		sec_get_param_value(__BATT_CAPACITY, &value);
		sprintf(buf, "%s  - 13. BATTERY_CAPACITY  : %d\n", buf, value);
		sec_get_param_value(__LOAD_KERNEL2, &value);
		sprintf(buf, "%s  - 14. LOAD_KERNEL2  : %d\n", buf, value);
		sec_get_param_value(__FLASH_LOCK_STATUS, &value);
		sprintf(buf, "%s  - 15. FLASH_LOCK_STATUS  : %d\n", buf, value);
		sec_get_param_value(__PARAM_INT_14, &value);
		sprintf(buf, "%s  - 16. PARAM_INT_14  : %d\n", buf, value);
		sec_get_param_value(__VERSION, &str_val);
		sprintf(buf, "%s  - 17. VERSION(STR)  : %s\n", buf, str_val);
		sec_get_param_value(__CMDLINE, &str_val);
		sprintf(buf, "%s  - 18. CMDLINE(STR)  : %s\n", buf, str_val);
		sec_get_param_value(__DELTA_LOCATION, &str_val);
		sprintf(buf, "%s  - 19. DELTA_LOCATION(STR)  : %s\n", buf, str_val);
		sec_get_param_value(__CMDLINE_MODE, &str_val);
		sprintf(buf, "%s  - 20. CMDLINE_MODE(STR)  : %s\n", buf, str_val);
		sec_get_param_value(__PARAM_STR_4, &str_val);
		sprintf(buf, "%s  - 21. PARAM_STR_4(STR)  : %s\n", buf, str_val);
	}

	return strlen(buf);
}

static ssize_t sec_param_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int idx, val, ret;

	ret = sscanf(buf, "%d %d", &idx, &val);

	if ((ret < 0) || (idx < 0)) {
		pr_err("sec-param: invalid inputs!\n");
		return -EINVAL;
	}

	if (sec_set_param_value) {
		pr_err("sec-param: set idx[%02d] %#04x\n", idx, val);
		sec_set_param_value(idx, &val);
	} else {
		pr_err("sec-param: param module hasnt been loaded\n");
	}
	
	return count;
}

static struct kobj_attribute sec_param_interface = __ATTR(param, 0600, sec_param_show, sec_param_store);

static struct attribute *sec_param_attrs[] = {
	&sec_param_interface.attr, 
	NULL,
};

static struct attribute_group sec_param_interface_group = {
	.attrs = sec_param_attrs,
};

static struct kobject *sec_param_kobject;

static int __init sec_param_init(void)
{
	int ret;

	sec_param_kobject = kobject_create_and_add("sec-param", kernel_kobj);

	if (!sec_param_kobject) {
		pr_err("sec-param: failed to allocate /sys/kernel/sec-param\n");
		return -ENOMEM;
	}

	ret = sysfs_create_group(sec_param_kobject, &sec_param_interface_group);

	if (ret) {
		pr_err("sec-param: failed to create /sys/kernel/sec-param\n");
		kobject_put(sec_param_kobject);
	}

	return ret;
}

module_init(sec_param_init);
