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
/*****************************************************************************
* cw1200_common.h
*
* Contains common declarations used by all the driver sub-blocks.
*
*****************************************************************************/

#ifndef _CW1200_COMMON_H
#define _CW1200_COMMON_H

#include <asm/atomic.h>
#include <linux/netdevice.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>

#define ETF_NOTSTARTED 0
#define ETF_STARTED 1

#define MACADDR_LEN 6

#define SDD_REFERENCE_FREQUENCY_ELT_ID 0xC5
#define FIELD_OFFSET(type, field) ( (uint8_t *)&((type*)0)->field - \
				(uint8_t *)0 )

extern int etf_flavour;

typedef struct sdio_func CW1200_bus_device_t;

extern const struct cw1200_platform_data *wpd;

int pr_dmp_msg(char *buf, unsigned int len);


struct stlc_firmware
{
	char * firmware;
	int length;
};

/* 1260_changes */
struct bootloader_1260
{
	char * bootloader;
	int length;
};
/* 1260_changes end */

typedef enum CW1200_STATUS
{
	SUCCESS=0,
	ERROR,
	ERROR_SDIO,
	ERROR_INVALID_PARAMETERS,
	ERROR_BUF_UNALIGNED,
	WAIT_INIT_VAL,
} CW1200_STATUS_E;

typedef struct cw1200_sdd
{
	uint8_t id;
	uint8_t length;
	uint8_t data[];
} CW1200_SDD;

struct CW1200_priv
{
	void* umac_handle;
	void* lower_handle;
	struct workqueue_struct * umac_WQ;
	struct work_struct umac_work;

	/*SBUS related declaraions*/
	atomic_t Interrupt_From_Device;
	atomic_t Umac_Tx;
	struct workqueue_struct * sbus_WQ;
	int32_t sdio_wr_buf_num_qmode;
	int32_t sdio_rd_buf_num_qmode;
	CW1200_bus_device_t *func;
	struct work_struct sbus_work;
	uint32_t max_size_supp;
	uint32_t max_num_buffs_supp;
	uint32_t num_unprocessed_buffs_in_device;
	uint16_t in_seq_num;
	uint16_t out_seq_num;
	uint32_t hw_type;
	uint32_t cw1200_cut_no;
	uint32_t device_sleep_status;
	struct completion comp;
	struct delayed_work sbus_sleep_work;
	struct list_head rx_list;
	struct mutex rx_list_lock;
	struct device *pdev;
};

struct cw1200_rx_buf {
	struct list_head list;
	struct sk_buff *skb;
	uint8_t *data;
};

#endif /* _CW1200_COMMON_H */
