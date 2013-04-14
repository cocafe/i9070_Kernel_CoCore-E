/* 
 * clock-con.c
 * DB8500 PRCMU Clock Debugfs Module
 * 
 * Author: Huang Ji (cocafe@xda-developers.com)
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
 */

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>

#include <linux/clk.h>
#include <linux/mfd/dbx500-prcmu.h>
#include <linux/mfd/db8500-prcmu.h>

static struct dentry *clk_con_dir;
static struct dentry *clk_b2r2_con;

static struct clk *clk_b2r2;

unsigned long clk_b2r2_rate;

static ssize_t clk_b2r2_read(struct file *filp, char __user *buf,
				   size_t count, loff_t *f_pos)
{
	clk_b2r2_rate = clk_get_rate(clk_b2r2);
	
	return sprintf(buf, "%ld\n", clk_b2r2_rate);
}

static ssize_t clk_b2r2_write(struct file *filp, const char __user *buf,
				    size_t count, loff_t *f_pos)
{
	unsigned long tmprate;

	sscanf(buf, "%ld", &tmprate);
	
	pr_info("clock-con: write b2r2 %ld", tmprate);

	prcmu_set_clock_rate(PRCMU_B2R2CLK, (unsigned long) tmprate);

	return 0;
}

static const struct file_operations clk_b2r2_fops = {
	.owner = THIS_MODULE,
	.read  = clk_b2r2_read,
	.write = clk_b2r2_write,
};

static int __init clock_con_init(void) {
	pr_info("clock-con: module init.\n");

	pr_info("clock-con: register debugfs");
	clk_con_dir = debugfs_create_dir("clk_con", NULL);
	clk_b2r2_con = debugfs_create_file("clk_b2r2", 
					(S_IRUGO | S_IWUSR | S_IWGRP), 
					clk_con_dir, 
					(void *)0, 
					&clk_b2r2_fops);

	pr_info("clock-con: register clocks");
	clk_b2r2 = clk_get_sys("b2r2_core", NULL);

	return 0;
}
module_init(clock_con_init);


