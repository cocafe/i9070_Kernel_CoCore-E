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

static const u8 abb_reg_cache[] = {
	0x00, /* REG_POWERUP		(0x00) */
	0x00, /* REG_AUDSWRESET		(0x01) */
	0x00, /* REG_ADPATHENA		(0x02) */
	0x00, /* REG_DAPATHENA		(0x03) */
	0x00, /* REG_ANACONF1		(0x04) */
	0x0F, /* REG_ANACONF2		(0x05) */
	0x00, /* REG_DIGMICCONF		(0x06) */
	0x00, /* REG_ANACONF3		(0x07) */
	0x00, /* REG_ANACONF4		(0x08) */
	0x00, /* REG_DAPATHCONF		(0x09) */
	0x40, /* REG_MUTECONF		(0x0A) */
	0x01, /* REG_SHORTCIRCONF	(0x0B) */
	0x01, /* REG_ANACONF5		(0x0C) */
	0x00, /* REG_ENVCPCONF		(0x0D) */
	0x00, /* REG_SIGENVCONF		(0x0E) */
	0x3F, /* REG_PWMGENCONF1	(0x0F) */
	0x32, /* REG_PWMGENCONF2	(0x10) */
	0x32, /* REG_PWMGENCONF3	(0x11) */
	0x32, /* REG_PWMGENCONF4	(0x12) */
	0x32, /* REG_PWMGENCONF5	(0x13) */
	0x0F, /* REG_ANAGAIN1		(0x14) */
	0x0F, /* REG_ANAGAIN2		(0x15) */
	0x22, /* REG_ANAGAIN3		(0x16) */
	0x55, /* REG_ANAGAIN4		(0x17) */
	0x13, /* REG_DIGLINHSLGAIN	(0x18) */
	0x13, /* REG_DIGLINHSRGAIN	(0x19) */
	0x00, /* REG_ADFILTCONF		(0x1A) */
	0x00, /* REG_DIGIFCONF1		(0x1B) */
	0x02, /* REG_DIGIFCONF2		(0x1C) */
	0x00, /* REG_DIGIFCONF3		(0x1D) */
	0x02, /* REG_DIGIFCONF4		(0x1E) */
	0xCC, /* REG_ADSLOTSEL1		(0xCC) */
	0xCC, /* REG_ADSLOTSEL2		(0xCC) */
	0xCC, /* REG_ADSLOTSEL3		(0xCC) */
	0xCC, /* REG_ADSLOTSEL4		(0xCC) */
	0xCC, /* REG_ADSLOTSEL5		(0xCC) */
	0xCC, /* REG_ADSLOTSEL6		(0xCC) */
	0xCC, /* REG_ADSLOTSEL7		(0xCC) */
	0xCC, /* REG_ADSLOTSEL8		(0xCC) */
	0xCC, /* REG_ADSLOTSEL9		(0xCC) */
	0xCC, /* REG_ADSLOTSEL10	(0xCC) */
	0xCC, /* REG_ADSLOTSEL11	(0xCC) */
	0xCC, /* REG_ADSLOTSEL12	(0xCC) */
	0xCC, /* REG_ADSLOTSEL13	(0xCC) */
	0xCC, /* REG_ADSLOTSEL14	(0xCC) */
	0xCC, /* REG_ADSLOTSEL15	(0xCC) */
	0xCC, /* REG_ADSLOTSEL16	(0xCC) */
	0x00, /* REG_ADSLOTHIZCTRL1	(0x2F) */
	0x00, /* REG_ADSLOTHIZCTRL2	(0x30) */
	0x00, /* REG_ADSLOTHIZCTRL3	(0x31) */
	0x00, /* REG_ADSLOTHIZCTRL4	(0x32) */
	0x08, /* REG_DASLOTCONF1	(0x33) */
	0x08, /* REG_DASLOTCONF2	(0x34) */
	0x08, /* REG_DASLOTCONF3	(0x35) */
	0x08, /* REG_DASLOTCONF4	(0x36) */
	0x08, /* REG_DASLOTCONF5	(0x37) */
	0x08, /* REG_DASLOTCONF6	(0x38) */
	0x08, /* REG_DASLOTCONF7	(0x39) */
	0x08, /* REG_DASLOTCONF8	(0x3A) */
	0x00, /* REG_CLASSDCONF1	(0x3B) */
	0x00, /* REG_CLASSDCONF2	(0x3C) */
	0x84, /* REG_CLASSDCONF3	(0x3D) */
	0x00, /* REG_DMICFILTCONF	(0x3E) */
	0xFE, /* REG_DIGMULTCONF1	(0x3F) */
	0xC0, /* REG_DIGMULTCONF2	(0x40) */
	0x3F, /* REG_ADDIGGAIN1		(0x41) */
	0x3F, /* REG_ADDIGGAIN2		(0x42) */
	0x1F, /* REG_ADDIGGAIN3		(0x43) */
	0x1F, /* REG_ADDIGGAIN4		(0x44) */
	0x3F, /* REG_ADDIGGAIN5		(0x45) */
	0x3F, /* REG_ADDIGGAIN6		(0x46) */
	0x1F, /* REG_DADIGGAIN1		(0x47) */
	0x1F, /* REG_DADIGGAIN2		(0x48) */
	0x3F, /* REG_DADIGGAIN3		(0x49) */
	0x3F, /* REG_DADIGGAIN4		(0x4A) */
	0x3F, /* REG_DADIGGAIN5		(0x4B) */
	0x3F, /* REG_DADIGGAIN6		(0x4C) */
	0x3F, /* REG_ADDIGLOOPGAIN1	(0x4D) */
	0x3F, /* REG_ADDIGLOOPGAIN2	(0x4E) */
	0x00, /* REG_HSLEARDIGGAIN	(0x4F) */
	0x00, /* REG_HSRDIGGAIN		(0x50) */
	0x1F, /* REG_SIDFIRGAIN1	(0x51) */
	0x1F, /* REG_SIDFIRGAIN2	(0x52) */
	0x00, /* REG_ANCCONF1		(0x53) */
	0x00, /* REG_ANCCONF2		(0x54) */
	0x00, /* REG_ANCCONF3		(0x55) */
	0x00, /* REG_ANCCONF4		(0x56) */
	0x00, /* REG_ANCCONF5		(0x57) */
	0x00, /* REG_ANCCONF6		(0x58) */
	0x00, /* REG_ANCCONF7		(0x59) */
	0x00, /* REG_ANCCONF8		(0x5A) */
	0x00, /* REG_ANCCONF9		(0x5B) */
	0x00, /* REG_ANCCONF10		(0x5C) */
	0x00, /* REG_ANCCONF11		(0x5D) - read only */
	0x00, /* REG_ANCCONF12		(0x5E) - read only */
	0x00, /* REG_ANCCONF13		(0x5F) - read only */
	0x00, /* REG_ANCCONF14		(0x60) - read only */
	0x00, /* REG_SIDFIRADR		(0x61) */
	0x00, /* REG_SIDFIRCOEF1	(0x62) */
	0x00, /* REG_SIDFIRCOEF2	(0x63) */
	0x00, /* REG_SIDFIRCONF		(0x64) */
	0x00, /* REG_AUDINTMASK1	(0x65) */
	0x00, /* REG_AUDINTSOURCE1	(0x66) - read only */
	0x00, /* REG_AUDINTMASK2	(0x67) */
	0x00, /* REG_AUDINTSOURCE2	(0x68) - read only */
	0x00, /* REG_FIFOCONF1		(0x69) */
	0x00, /* REG_FIFOCONF2		(0x6A) */
	0x00, /* REG_FIFOCONF3		(0x6B) */
	0x00, /* REG_FIFOCONF4		(0x6C) */
	0x00, /* REG_FIFOCONF5		(0x6D) */
	0x00, /* REG_FIFOCONF6		(0x6E) */
	0x02, /* REG_AUDREV		(0x6F) - read only */
};

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

static ssize_t abb_codec_reset_reg_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	char *texts = "<Usage>:\necho REG > reset_reg\n";
	return sprintf(buf, "%s", texts);
}

static ssize_t abb_codec_reset_reg_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int reg;
	unsigned int reg_a;
	unsigned int reg_b;

	sscanf(buf, "%x", &reg);

	if (reg < 0) {
		pr_info("abb-codec: invalid inputs \n");
		return -EINVAL;
	}
	
	pr_info("abb-codec: reset REG[%#04x]\n", reg);

	reg_a = snd_soc_read(abb_codec, reg);

	ret = snd_soc_write(abb_codec, reg, abb_reg_cache[reg]);

	reg_b = snd_soc_read(abb_codec, reg);

	pr_info("abb-codec: REG[%#04x] %#04x -> %#04x\n", reg_a, reg_b);
	
	if (!ret) {
		pr_info("abb-codec: return errors or no change \n");
	}

	return count;
}

/* Sysfs register */
static struct kobj_attribute abb_codec_update_bit_interface = __ATTR(update_bit, 0644, abb_codec_update_bit_show, abb_codec_update_bit_store);
static struct kobj_attribute abb_codec_reset_reg_interface = __ATTR(reset_reg, 0644, abb_codec_reset_reg_show, abb_codec_reset_reg_store);

static struct attribute *abb_codec_attrs[] = {
	&abb_codec_update_bit_interface.attr, 
	&abb_codec_reset_reg_interface.attr, 
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
