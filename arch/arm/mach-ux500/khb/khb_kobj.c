#include <linux/init.h>
#include <linux/kobject.h>
#include "khb_main.h"
#include "khb_lcd.h"

static u32 font_color    = COLOR_GREEN;
static u32 bg_color      = COLOR_BLACK;
static u8  font_size     = DEFAULT_SIZE;
static u8  msg_retain    = DISCARD_MSG;

static ssize_t font_color_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%#08x\n", font_color);
}

static ssize_t font_color_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	u32 val;

	if (sscanf(buf, "%x", &val)) {
		font_color = val;
		return count;
	}

	return -EINVAL;
}
static struct kobj_attribute font_color_interface = __ATTR(font_color, 0644, font_color_show, font_color_store);

static ssize_t bg_color_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%#08x\n", bg_color);
}

static ssize_t bg_color_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	u32 val;

	if (sscanf(buf, "%x", &val)) {
		bg_color = val;
		return count;
	}

	return -EINVAL;
}
static struct kobj_attribute bg_color_interface = __ATTR(bg_color, 0644, bg_color_show, bg_color_store);

static ssize_t font_size_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", (u32)font_size);
}

static ssize_t font_size_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	u32 val;

	if (sscanf(buf, "%d", &val)) {
		font_size = val;
		return count;
	}

	return -EINVAL;
}
static struct kobj_attribute font_size_interface = __ATTR(font_size, 0644, font_size_show, font_size_store);

static ssize_t msg_retain_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", (u32)msg_retain);
}

static ssize_t msg_retain_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	u32 val;

	if (sscanf(buf, "%d", &val)) {
		msg_retain = val;
		return count;
	}

	return -EINVAL;
}
static struct kobj_attribute msg_retain_interface = __ATTR(msg_retain, 0644, msg_retain_show, msg_retain_store);

static ssize_t msg_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t msg_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	char txt[60];

	sprintf(txt, "%s", buf);
	display_msg_colored(txt, font_color, bg_color, font_size, msg_retain);

	return count;
}
static struct kobj_attribute msg_interface = __ATTR(msg, 0644, msg_show, msg_store);

static struct attribute *khb_kobj_attrs[] = {
	&font_color_interface.attr,
	&bg_color_interface.attr,
	&font_size_interface.attr,
	&msg_retain_interface.attr,
	&msg_interface.attr,
	NULL,
};

static struct attribute_group khb_kobj_interface_group = {
	.attrs = khb_kobj_attrs,
};

static struct kobject *khb_kobj_kobject;

static int __init khb_kobj_sysfs_init(void)
{
	int ret;

	khb_kobj_kobject = kobject_create_and_add("khb_lcd", kernel_kobj);
	if (!khb_kobj_kobject) {
		pr_err("khb_kobj: Failed to create kobject interface\n");
	}
	ret = sysfs_create_group(khb_kobj_kobject, &khb_kobj_interface_group);
	if (ret) {
		kobject_put(khb_kobj_kobject);
	}

        return 0;
}
late_initcall(khb_kobj_sysfs_init);
