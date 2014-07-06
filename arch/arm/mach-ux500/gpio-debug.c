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
#include <plat/gpio-nomadik.h>

static ssize_t janice_gpio_set_mode_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "[gpio] [0/1]\n");
}

static ssize_t janice_gpio_set_mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int gpio, onoff, ret;

	ret = sscanf(buf, "%d %d", &gpio, &onoff);

	if (!ret) {
		pr_err("janice-gpio: invalid inputs\n");
		return -EINVAL;
	}

	if ((onoff != 0 ) && (onoff != 1)) {
		pr_err("janice-gpio: invalid inputs\n");
		return -EINVAL;
	}

	pr_info("janice-gpio: GPIO[%03d] [%d]\n", gpio, onoff);

	gpio_set_value(gpio, onoff);
	
	return count;
}

static struct kobj_attribute janice_gpio_set_mode_interface = __ATTR(set_mode, 0600, janice_gpio_set_mode_show, janice_gpio_set_mode_store);

static ssize_t janice_gpio_set_pull_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	sprintf(buf, "NMK_GPIO_PULL_NONE = 0\nNMK_GPIO_PULL_UP = 1\nNMK_GPIO_PULL_DOWN = 2\n");
	return strlen(buf);
}

static ssize_t janice_gpio_set_pull_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int gpio, pull, ret;

	ret = sscanf(buf, "%d %d", &gpio, &pull);

	if (!ret) {
		pr_err("janice-gpio: invalid inputs\n");
		return -EINVAL;
	}

	if (pull < 0 || pull > 2) {
		pr_err("janice-gpio: invalid inputs\n");
		return -EINVAL;
	}

	nmk_gpio_set_pull(gpio, pull);
	
	return count;
}

static struct kobj_attribute janice_gpio_set_pull_interface = __ATTR(set_pull, 0600, janice_gpio_set_pull_show, janice_gpio_set_pull_store);

static ssize_t janice_gpio_set_slpm_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	sprintf(buf, "NMK_GPIO_SLPM_INPUT(WAKEUP_ENABLE) = 0\nNMK_GPIO_SLPM_NOCHANGE(WAKEUP_DISABLE) = 1\n");
	return strlen(buf);
}

static ssize_t janice_gpio_set_slpm_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int gpio, slpm, ret;

	ret = sscanf(buf, "%d %d", &gpio, &slpm);

	if (!ret) {
		pr_err("janice-gpio: invalid inputs\n");
		return -EINVAL;
	}

	if (slpm < 0 || slpm > 1) {
		pr_err("janice-gpio: invalid inputs\n");
		return -EINVAL;
	}

	nmk_gpio_set_slpm(gpio, slpm);
	
	return count;
}

static struct kobj_attribute janice_gpio_set_slpm_interface = __ATTR(set_slpm, 0600, janice_gpio_set_slpm_show, janice_gpio_set_slpm_store);

static struct attribute *janice_gpio_debug_attrs[] = {
	&janice_gpio_set_mode_interface.attr, 
	&janice_gpio_set_pull_interface.attr, 
	&janice_gpio_set_slpm_interface.attr, 
	NULL,
};

static struct attribute_group janice_gpio_debug_interface_group = {
	.attrs = janice_gpio_debug_attrs,
};

static struct kobject *janice_gpio_debug_kobject;

static int __init janice_gpio_debug_init(void)
{
	int ret;

	janice_gpio_debug_kobject = kobject_create_and_add("gpio", kernel_kobj);
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
