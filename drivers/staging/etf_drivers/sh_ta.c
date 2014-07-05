/*
* ST-Ericsson ETF driver
*
* Copyright (c) ST-Ericsson SA 2011
*
* License terms: Redistribution and use in source and binary forms, with or
* without modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*   - Neither the name of ST-Ericsson S.A. nor the names of its contributors
*     may be used to endorse or promote products derived from this software
*     without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
/**********************************************************************
* sh_ta.c
*
* This module interfaces with the Linux Kernel MMC/SDIO stack.
*
***********************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

#include "dev_ioctl.h"
#include "hi_api.h"
#include <linux/device.h>

#include "st90tds_if.h"
#include "driver.h"
#include "debug.h"

#define SUCCESS 0

#define NTSTATUS int
#define OUT_SIZE 200

Dev_t *device=NULL;
uint32 g_fwsze=0;
uint32 g_msgsze=0;
uint32 started=0;
struct class *my_class=NULL;

NTSTATUS STLC_ReadAdapter(Dev_t *device, HI_MSG_HDR *msg, uint32 out_size);
NTSTATUS STLC_WriteAdapter(Dev_t *device, HI_MSG_HDR *msg, uint32 size);
NTSTATUS STLC_Request(Dev_t *device, st90tds_msg_t *msg,
	uint32 inp_size, uint32 out_size);
int STLC_InitializeAdapter_ChipDetect(void);

static int device_open(struct inode *inode, struct file *file)
{
	DEBUG(DBG_FUNC, "\thit %s\n", __func__);
	return SUCCESS;
}

static int device_release(struct inode *inode, struct file *file)
{
	DEBUG(DBG_FUNC, "\thit %s: Reboot to restart tests.\n", __func__);
	return 0;
}

static ssize_t device_read(struct file *file, char *buffer,
		size_t length, loff_t *offset) {
	DEBUG(DBG_FUNC, "\thit %s\n", __func__);
	return 0;
}

static ssize_t device_write(struct file *file, const char *buffer,
			size_t length, loff_t *offset) {
	DEBUG(DBG_FUNC, "\thit %s\n", __func__);
	return 0;
}
st90tds_msg_t *msgbuf = NULL;


static int pr_dmp_msg(char *buf, uint32 len)
{
	int i;
	DEBUG(DBG_MESSAGE, "dump starts:\n");
	for (i = 1; i <= len; i++) {
		DEBUG(DBG_MESSAGE, "0x%2x ", buf[i-1]);
		if ((i % 8) == 0)
			DEBUG(DBG_MESSAGE, "\n");
	}
	DEBUG(DBG_MESSAGE, "dump ends\n");
	return 0;
}


#ifdef HAVE_UNLOCKED_IOCTL
long device_ioctl(struct file *file, uint32 ioctl_num,
	unsigned long ioctl_param) {
#else
int device_ioctl(struct inode *inode, struct file *file,
	uint32 ioctl_num, unsigned long ioctl_param) {
#endif
	NTSTATUS status = STATUS_SUCCESS;
	char *msg;
	unsigned out_buf_size;
	out_buf_size = OUT_SIZE;

	DEBUG(DBG_FUNC, "%s: hit ioctl:%d\n", __func__, ioctl_num);

	switch (ioctl_num) {
	case IOCTL_ST90TDS_ADAP_READ:
		msg = (char *)ioctl_param;
		/* Re-entrance is an error in the application level.*/
		status = STLC_ReadAdapter(device, (HI_MSG_HDR *)msg,
				out_buf_size);
		break;

	case IOCTL_ST90TDS_ADAP_WRITE:
		msg = kmalloc(g_msgsze, GFP_KERNEL);
		DEBUG(DBG_FUNC, "IOCTL adap write:"
				"allocated buf of size %d, @ %p\n",
				g_msgsze, msg);

		pr_dmp_msg(msg, g_msgsze);

		if (copy_from_user(msg, (char *)ioctl_param, g_msgsze))
			DEBUG(DBG_ERROR, "%s: copy_from_user failed\n",
				__func__);
		status = STLC_WriteAdapter(device,
				(HI_MSG_HDR *)msg, g_msgsze);
		DEBUG(DBG_FUNC, "freeing msg @%p\n", msg);
		kfree(msg);
		break;

	case IOCTL_ST90TDS_REQ_CNF_SIZE:
		g_fwsze = ioctl_param;
		DEBUG(DBG_FUNC, "IOCTL size of request = %d\n",
			g_fwsze);
		break;

	case IOCTL_ST90TDS_WRITE_CNF_SIZE:
		g_msgsze = ioctl_param;
		DEBUG(DBG_FUNC, "IOCTL size of message = %d\n",
			g_msgsze);
		break;

	case IOCTL_ST90TDS_REQ_CNF:
		msg = kmalloc(g_fwsze, GFP_KERNEL);
		DEBUG(DBG_FUNC, "IOCTL request: allocated"
				"buf of size %d, @ %p\n",
				g_fwsze, msg);
		pr_dmp_msg(msg, g_msgsze);
		if (copy_from_user(msg, (void *)ioctl_param, g_fwsze))
			DEBUG(DBG_ERROR, "%s: copy_from_user failed\n",
				__func__);

		/*To keep track of userspace buf address for reply */
		msgbuf = (st90tds_msg_t *)ioctl_param;
		status = STLC_Request(device, (st90tds_msg_t *)msg,
				g_fwsze, out_buf_size);
		DEBUG(DBG_FUNC, "freeing msg @%p\n", msg);
		started = 1; /*unused*/
		kfree(msg);
		break;

	case IOCTL_ST90TDS_CHIP_VERSION:
		/* check if chip version has been detected or not */
		if (chip_version_global_int == FALSE) {
			status = STLC_InitializeAdapter_ChipDetect();
			if (status != 0) {
				DEBUG(DBG_ERROR,
					"STLC_InitializeAdapter_ChipDetect"
					" returned error\n");
				break;
			}

			DEBUG(DBG_MESSAGE, "%s: chip version: %d\n", __func__,
				chip_version_global_int);
		}

		msgbuf = (st90tds_msg_t *)ioctl_param;

		DEBUG(DBG_MESSAGE, "%s: chip version: %d, sizeof(chip_version_global_int)=%d\n",
			__func__, chip_version_global_int, sizeof(chip_version_global_int));
		status = copy_to_user((void *)ioctl_param, &chip_version_global_int,
				sizeof(chip_version_global_int));
                if (status) {
                        DEBUG(DBG_ERROR, "copy_to_user fails: %d\n", status);
                }
		else {
			DEBUG(DBG_MESSAGE, "copy_to_user done copying"
				" chip_version_global_int=%d\n",
				chip_version_global_int);
			status = STATUS_SUCCESS;
		}

		break;

	default:
		DEBUG(DBG_ERROR, "%s:ERROR:Wrong Ioctl(%d) sent from host!\n",
				__func__, ioctl_num);
	}
	return status;
}

struct file_operations Fops = {
	.read = device_read,
	.write = device_write,
#ifdef HAVE_UNLOCKED_IOCTL
	.unlocked_ioctl = device_ioctl,
#else
	.ioctl = device_ioctl,
#endif
	.open = device_open,
	.release = device_release,
};

int drv_etf_ioctl_enter(void)
{
	int ret_val;

	if (debug_level > 15)
		debug_level = 15;
	else if (debug_level < 8)
		debug_level += DBG_ALWAYS;

	ret_val = register_chrdev(MAJOR_NUM, DEVICE_NAME, &Fops);
	if (ret_val < 0) {
		DEBUG(DBG_ERROR, "register chrdev failed with %d\n", ret_val);
		return ret_val;
	}
	DEBUG(DBG_ALWAYS, "The major device device number is %d\n", MAJOR_NUM);

	my_class = class_create(THIS_MODULE, DEVICE_NAME);
	DEBUG(DBG_ALWAYS, "class_create %s\n", DEVICE_NAME);
	device_create(my_class,NULL,MKDEV(MAJOR_NUM, 0), NULL, DEVICE_NAME);
	DEBUG(DBG_ALWAYS, "Created %s in dev\n",DEVICE_NAME);

	return 0;
}

void drv_etf_ioctl_release(void)
{
	st90tds_msg_t *msg;
	msg = kmalloc(sizeof(st90tds_msg_t), GFP_KERNEL);
	msg->msg_id = ST90TDS_EXIT_ADAPTER;
	STLC_Request(device, msg, 4, 200);
	DEBUG(DBG_ALWAYS, "Exiting character driver\n");
	device_destroy(my_class,MKDEV(MAJOR_NUM,0));
	class_unregister(my_class);
	class_destroy(my_class);
	unregister_chrdev(MAJOR_NUM, DEVICE_NAME);
}
