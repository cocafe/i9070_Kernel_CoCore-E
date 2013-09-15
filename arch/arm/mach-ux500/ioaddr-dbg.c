/*
 * License terms: GNU General Public License (GPL) version 2
 * 
 * Author: Huang Ji (cocafe) <cocafehj@gmail.com>
 * 
 * IOMEM debugger for development.
 * 
 */

#include <linux/io.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>

#include <asm/io.h>
#include <mach/hardware.h>

#define BASE_TO_IO(v) 		__io_address(v)

static unsigned int last_reg;
static unsigned int last_base;
static unsigned int last_read;

static ssize_t ioaddr_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	sprintf(buf,   "<Usage>:\n");
	sprintf(buf, "%s[writel] [db8500 base] [reg] [value]\n", buf);
//	sprintf(buf, "%s[readb/readw/readl] [db8500 base] [reg]\n\n", buf);

//	sprintf(buf, "%s<Last read>:\n", buf);
//	sprintf(buf, "%s@[%#08x] + [%#04x] : [%#08x]\n", buf, last_base, last_reg, last_read);

	return strlen(buf);
}

static ssize_t ioaddr_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	void __iomem *iomem_base;

	unsigned int tmp_base, tmp_reg, tmp_value;
	int ret;

	if (!strncmp(&buf[0], "writel", 6)) {
		ret = sscanf(&buf[6], "%x %x %x", &tmp_base, &tmp_reg, &tmp_value);
		
		if (!ret) {
			pr_err("[IOADDR] Invalid cmd\n");
			return -EINVAL;
		}

		iomem_base = BASE_TO_IO(tmp_base);

		pr_err("[IOADDR] Writel %#08x @ %#08x(base) + %#04x\n", tmp_value, tmp_base, tmp_reg);

		writel(tmp_value, iomem_base + tmp_reg);

		return count;
	}

	pr_err("[IOADDR] Invalid cmd\n");
	
	return count;
}

static struct kobj_attribute ioaddr_dbg_interface = __ATTR(ioaddr, 0644, ioaddr_show, ioaddr_store);

static struct attribute *ioaddr_dbg_attrs[] = {
	&ioaddr_dbg_interface.attr, 
	NULL,
};

static struct attribute_group ioaddr_dbg_interface_group = {
	.attrs = ioaddr_dbg_attrs,
};

static struct kobject *ioaddr_dbg_kobject;

static int __init ioaddr_dbg_init(void)
{
	int ret;

	ioaddr_dbg_kobject = kobject_create_and_add("iomem", kernel_kobj);
	if (!ioaddr_dbg_kobject) {
		return -ENOMEM;
	}

	ret = sysfs_create_group(ioaddr_dbg_kobject, &ioaddr_dbg_interface_group);

	if (ret) {
		kobject_put(ioaddr_dbg_kobject);
	}

	return ret;
}
module_init(ioaddr_dbg_init);
