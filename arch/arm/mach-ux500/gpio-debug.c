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
#include <asm/gpio.h>

static ssize_t janice_gpio_onoff_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "[pin] [0/1]\n");
}

static ssize_t janice_gpio_onoff_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int pin, onoff, ret;

	ret = sscanf(buf, "%d %d", &pin, &onoff);

	if (!ret) {
		pr_err("janice-gpio: invalid inputs\n");
		return -EINVAL;
	}

	if ((onoff != 0 ) && (onoff != 1)) {
		pr_err("janice-gpio: invalid inputs\n");
		return -EINVAL;
	}

	pr_info("janice-gpio: PIN[%03d] [%d]\n", pin, onoff);

	gpio_set_value(pin, onoff);
	
	return count;
}

static struct kobj_attribute janice_gpio_onoff_interface = __ATTR(gpio, 0600, janice_gpio_onoff_show, janice_gpio_onoff_store);

static struct attribute *janice_gpio_debug_attrs[] = {
	&janice_gpio_onoff_interface.attr, 
	NULL,
};

static struct attribute_group janice_gpio_debug_interface_group = {
	.attrs = janice_gpio_debug_attrs,
};

static struct kobject *janice_gpio_debug_kobject;

static int __init janice_gpio_debug_init(void)
{
	int ret;

	janice_gpio_debug_kobject = kobject_create_and_add("janice-gpio", kernel_kobj);
	if (!janice_gpio_debug_kobject) {
		return -ENOMEM;
	}

	ret = sysfs_create_group(janice_gpio_debug_kobject, &janice_gpio_debug_interface_group);
	if (ret) {
		kobject_put(janice_gpio_debug_kobject);
	}

	return ret;
}
module_init(janice_gpio_debug_init);
