/* 
 * clock-con.c
 * DB8500 PRCMU Clock Sysfs Interface Module
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
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>

#include <linux/clk.h>
#include <linux/mfd/dbx500-prcmu.h>
#include <linux/mfd/db8500-prcmu.h>

/* PRCMU clock registers:
 * 
 * PRCMU_SGACLK				0x00
 * PRCMU_UARTCLK			0x01
 * PRCMU_MSP02CLK			0x02
 * PRCMU_MSP1CLK			0x03
 * PRCMU_I2CCLK				0x04
 * PRCMU_SDMMCCLK			0x05
 * PRCMU_SPARE1CLK			0x06
 * PRCMU_SLIMCLK			0x07
 * PRCMU_PER1CLK			0x08
 * PRCMU_PER2CLK			0x09
 * PRCMU_PER3CLK			0x0A
 * PRCMU_PER5CLK			0x0B
 * PRCMU_PER6CLK			0x0C
 * PRCMU_PER7CLK			0x0D
 * PRCMU_LCDCLK				0x0E
 * PRCMU_BMLCLK				0x0F
 * PRCMU_HSITXCLK			0x10
 * PRCMU_HSIRXCLK			0x11
 * PRCMU_HDMICLK			0x12
 * PRCMU_APEATCLK			0x13
 * PRCMU_APETRACECLK			0x14
 * PRCMU_MCDECLK			0x15
 * PRCMU_IPI2CCLK			0x16
 * PRCMU_DSIALTCLK			0x17
 * PRCMU_DMACLK				0x18
 * PRCMU_B2R2CLK			0x19
 * PRCMU_TVCLK				0x1A
 * PRCMU_SSPCLK				0x1B
 * PRCMU_RNGCLK				0x1C
 * PRCMU_UICCCLK			0x1D
 * PRCMU_PWMCLK				0x1E
 * PRCMU_IRDACLK			0x1F
 * PRCMU_IRRCCLK			0x20
 * PRCMU_SIACLK				0x21
 * PRCMU_SVACLK				0x22
 * PRCMU_ACLK				0x23
 * PRCMU_HVACLK				0x24
 * PRCMU_G1CLK				0x25
 * PRCMU_NUM_REG_CLOCKS			0x26
 * PRCMU_SYSCLK				0x26
 * PRCMU_CDCLK				0x27
 * PRCMU_TIMCLK				0x28
 * PRCMU_PLLSOC0			0x29
 * PRCMU_PLLSOC1			0x2A
 * PRCMU_ARMSS				0x2B
 * PRCMU_ARMCLK				0x2C
 * PRCMU_PLLDDR				0x2D
 * PRCMU_PLLDSI				0x2E
 * PRCMU_DSI0CLK			0x2F
 * PRCMU_DSI1CLK			0x30
 * PRCMU_DSI0ESCCLK			0x31
 * PRCMU_DSI1ESCCLK			0x32
 * PRCMU_DSI2ESCCLK			0x33
 * PRCMU_PLLDSI_LCD			0x34
 * PRCMU_DSI0CLK_LCD			0x35
 * PRCMU_DSI1CLK_LCD			0x36
 * PRCMU_DSI0ESCCLK_LCD			0x37
 * PRCMU_DSI1ESCCLK_LCD			0x38
 * PRCMU_DSI2ESCCLK_LCD			0x39
*/
static enum prcmu_clock clk_regs;

/* Sysfs fops */
static ssize_t clk_prcmu_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	char* helps = "PRCMU Clocks Registers:\n\nPRCMU_SGACLK\t\t\t 0x00\nPRCMU_UARTCLK\t\t\t 0x01\nPRCMU_MSP02CLK\t\t\t 0x02\nPRCMU_MSP1CLK\t\t\t 0x03\nPRCMU_I2CCLK\t\t\t 0x04\nPRCMU_SDMMCCLK\t\t\t 0x05\nPRCMU_SPARE1CLK\t\t\t 0x06\nPRCMU_SLIMCLK\t\t\t 0x07\nPRCMU_PER1CLK\t\t\t 0x08\nPRCMU_PER2CLK\t\t\t 0x09\nPRCMU_PER3CLK\t\t\t 0x0A\nPRCMU_PER5CLK\t\t\t 0x0B\nPRCMU_PER6CLK\t\t\t 0x0C\nPRCMU_PER7CLK\t\t\t 0x0D\nPRCMU_LCDCLK\t\t\t 0x0E\nPRCMU_BMLCLK\t\t\t 0x0F\nPRCMU_HSITXCLK\t\t\t 0x10\nPRCMU_HSIRXCLK\t\t\t 0x11\nPRCMU_HDMICLK\t\t\t 0x12\nPRCMU_APEATCLK\t\t\t 0x13\nPRCMU_APETRACECLK\t\t 0x14\nPRCMU_MCDECLK\t\t\t 0x15\nPRCMU_IPI2CCLK\t\t\t 0x16\nPRCMU_DSIALTCLK\t\t\t 0x17\nPRCMU_DMACLK\t\t\t 0x18\nPRCMU_B2R2CLK\t\t\t 0x19\nPRCMU_TVCLK\t\t\t 0x1A\nPRCMU_SSPCLK\t\t\t 0x1B\nPRCMU_RNGCLK\t\t\t 0x1C\nPRCMU_UICCCLK\t\t\t 0x1D\nPRCMU_PWMCLK\t\t\t 0x1E\nPRCMU_IRDACLK\t\t\t 0x1F\nPRCMU_IRRCCLK\t\t\t 0x20\nPRCMU_SIACLK\t\t\t 0x21\nPRCMU_SVACLK\t\t\t 0x22\nPRCMU_ACLK\t\t\t 0x23\nPRCMU_HVACLK\t\t\t 0x24\nPRCMU_G1CLK\t\t\t 0x25\nPRCMU_SYSCLK\t\t\t 0x26\nPRCMU_CDCLK\t\t\t 0x27\nPRCMU_TIMCLK\t\t\t 0x28\nPRCMU_PLLSOC0\t\t\t 0x29\nPRCMU_PLLSOC1\t\t\t 0x2A\nPRCMU_ARMSS\t\t\t 0x2B\nPRCMU_ARMCLK\t\t\t 0x2C\nPRCMU_PLLDDR\t\t\t 0x2D\nPRCMU_PLLDSI\t\t\t 0x2E\nPRCMU_DSI0CLK\t\t\t 0x2F\nPRCMU_DSI1CLK\t\t\t 0x30\nPRCMU_DSI0ESCCLK\t\t 0x31\nPRCMU_DSI1ESCCLK\t\t 0x32\nPRCMU_DSI2ESCCLK\t\t 0x33\nPRCMU_PLLDSI_LCD\t\t 0x34\nPRCMU_DSI0CLK_LCD\t\t 0x35\nPRCMU_DSI1CLK_LCD\t\t 0x36\nPRCMU_DSI0ESCCLK_LCD\t\t 0x37\nPRCMU_DSI1ESCCLK_LCD\t\t 0x38\nPRCMU_DSI2ESCCLK_LCD\t\t 0x39\n\necho 0x19 399360000 > prcmu_clk\n";
	return sprintf(buf, "%s", helps);
}

static ssize_t clk_prcmu_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned long clk_tmpbuf;
	unsigned char clkreg_buf;

	sscanf(buf, "%x %ld", &clk_regs, &clk_tmpbuf);

	clkreg_buf = clk_regs;

	pr_info("clock-con: 0x%x %ldHz\n", clkreg_buf, clk_tmpbuf);

	prcmu_set_clock_rate(clkreg_buf, (unsigned long) clk_tmpbuf);
	
	return count;
}

/* Sysfs register */
static struct kobj_attribute clk_prcmu_interface = __ATTR(prcmu_clk, 0644, clk_prcmu_show, clk_prcmu_store);

static struct attribute *clock_con_attrs[] = {
	&clk_prcmu_interface.attr, 
	NULL,
};

static struct attribute_group clock_interface_group = {
	.attrs = clock_con_attrs,
};

static struct kobject *clock_con_kobject;

static int __init clock_con_init(void) 
{
	int ret;

	pr_info("clock-con: registering sysfs\n");
	clock_con_kobject = kobject_create_and_add("clock", kernel_kobj);
	if (!clock_con_kobject) {
		pr_info("clock-con: no available memory\n");
		return -ENOMEM;
	}

	ret = sysfs_create_group(clock_con_kobject, &clock_interface_group);
	if (ret) {
		kobject_put(clock_con_kobject);
		pr_info("clock-con: failed to register sysfs\n");
	}

	return ret;
}

static void __exit clock_con_exit(void)
{
	kobject_put(clock_con_kobject);
}

module_init(clock_con_init);
module_exit(clock_con_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aditya Patange(Adi_Pat), Huang Ji(cocafe)");
