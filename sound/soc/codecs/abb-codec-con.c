/* 
 * abb-codec-con.c
 * AB8500 Codec Sysfs Interface Module
 * 
 * Author: 	Aditya Patange (Adi_Pat) <adithemagnificent@gmail.com>
 * 		Huang Ji (cocafe) (cocafe@xda-developers.com)
 * 
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#include <linux/init.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>

#include <sound/soc.h>
#include "ab8500_audio.h"
#include "abb-codec-con.h"

/* To convert register definition shifts to masks */
#define BMASK(bsft)		(1 << (bsft))

/* Codec */
static struct snd_soc_codec *abb_codec;

/* Register codec */
void abb_codec_register(struct snd_soc_codec *codec)
{
	pr_info("abb-codec: registering codec\n");
	abb_codec = codec;
	
	if (abb_codec == NULL) {
		pr_info("abb-codec: abb codec hasn't probed yet\n");
	}
}

/* Sysfs fops */
static ssize_t abb_codec_update_bit_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	char *texts = "<Usage>:\necho REG BitWidth Bit IS_locked > update_bit\n";
	return sprintf(buf, "%s", texts);
}

static ssize_t abb_codec_update_bit_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int reg;
	unsigned int reg_a;
	unsigned int reg_b;
	unsigned int bit_wid;	/* Bit offset */
	unsigned char bit_mask;
	unsigned int bit;
	unsigned int is_locked;

	sscanf(buf, "%x %d %d %d", &reg, &bit_wid, &bit, &is_locked);

	if (reg < 0 || bit_wid < 0 || bit < 0 || is_locked < 0 || is_locked > 1) {
		pr_info("abb-codec: invalid inputs \n");
		return -EINVAL;
	}

	bit_mask = BMASK(bit_wid);
	
	pr_info("abb-codec: Reg[%#04x] BMask[%#04x] Bit[%#08x] IS_locked[%d]\n", reg, bit_mask, bit, is_locked);

	reg_a = snd_soc_read(abb_codec, reg);

	if (is_locked) {
		ret = snd_soc_update_bits_locked(abb_codec, reg, bit_mask, bit);
	} else {
		ret = snd_soc_update_bits(abb_codec, reg, bit_mask, bit);
	}

	reg_b = snd_soc_read(abb_codec, reg);

	pr_info("abb-codec: REG[%#04x] %#04x -> %#04x\n", reg_a, reg_b);
	
	if (!ret) {
		pr_info("abb-codec: return errors or no change \n");
	}

	return count;
}

/* Sysfs register */
static struct kobj_attribute abb_codec_update_bit_interface = __ATTR(update_bit, 0644, abb_codec_update_bit_show, abb_codec_update_bit_store);

static struct attribute *abb_codec_attrs[] = {
	&abb_codec_update_bit_interface.attr, 
	NULL,
};

static struct attribute_group abb_codec_interface_group = {
	.attrs = abb_codec_attrs,
};

static struct kobject *abb_codec_kobject;

/* Module probe */
static int __init abb_codec_init(void)
{
	int ret;

	pr_info("abb-codec: registering sysfs\n");
	abb_codec_kobject = kobject_create_and_add("abb-codec", kernel_kobj);

	if (!abb_codec_kobject) {
		pr_info("abb-codec: faile to register sysfs\n");
		return -ENOMEM;
	}

	ret = sysfs_create_group(abb_codec_kobject, &abb_codec_interface_group);

	if (ret) {
		pr_info("abb-codec: faile to register sysfs\n");
		kobject_put(abb_codec_kobject);
	}

	return ret;
}

static void __exit abb_codec_exit(void)
{
	kobject_put(abb_codec_kobject);
}

module_init(abb_codec_init);
module_exit(abb_codec_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aditya Patange(Adi_Pat), Huang Ji(cocafe)");
