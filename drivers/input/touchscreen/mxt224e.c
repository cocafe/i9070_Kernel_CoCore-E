/*
 *  Copyright (C) 2010, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 *  Modified: Huang Ji (cocafe@xda-developers.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#define TOUCH_BOOSTER
/* #define TOUCH_DEBUGGER */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/input/mxt224e.h>
#include <asm/unaligned.h>
#include <linux/firmware.h>
#include <linux/input/mt.h>
#include <linux/ab8500-ponkey.h>

#include <plat/pincfg.h>
#include <plat/gpio-nomadik.h>

#include <mach/board-sec-u8500.h>
#include <mach/../../pins-db8500.h>
#include <mach/../../pins.h>

#if defined(TOUCH_BOOSTER)
#include <linux/mfd/dbx500-prcmu.h>
#endif

#define OBJECT_TABLE_START_ADDRESS	7
#define OBJECT_TABLE_ELEMENT_SIZE	6

#define CMD_RESET_OFFSET		0
#define CMD_BACKUP_OFFSET		1
#define CMD_CALIBRATE_OFFSET    	2
#define CMD_REPORTATLL_OFFSET   	3
#define CMD_DEBUG_CTRL_OFFSET   	4
#define CMD_DIAGNOSTIC_OFFSET   	5


#define DETECT_MSG_MASK			0x80
#define PRESS_MSG_MASK			0x40
#define RELEASE_MSG_MASK		0x20
#define MOVE_MSG_MASK			0x10
#define SUPPRESS_MSG_MASK		0x02

/* Version */
#define MXT224_VER_20			20
#define MXT224_VER_21			21
#define MXT224_VER_22			22

/* Slave addresses */
#define MXT224_APP_LOW			0x4a
#define MXT224_APP_HIGH			0x4b
#define MXT224_BOOT_LOW			0x24
#define MXT224_BOOT_HIGH		0x25

/* FIRMWARE NAME */
#define MXT224_ECHO_FW_NAME		"mXT224E.fw"
#define MXT224_FW_NAME			"mXT224.fw"

#define MXT224_FWRESET_TIME		175	/* msec */
#define MXT224_RESET_TIME		80	/* msec */

#define MXT224_BOOT_VALUE		0xa5
#define MXT224_BACKUP_VALUE		0x55

/* Bootloader mode status */
#define MXT224_WAITING_BOOTLOAD_CMD	0xc0	/* valid 7 6 bit only */
#define MXT224_WAITING_FRAME_DATA	0x80	/* valid 7 6 bit only */
#define MXT224_FRAME_CRC_CHECK	0x02
#define MXT224_FRAME_CRC_FAIL		0x03
#define MXT224_FRAME_CRC_PASS		0x04
#define MXT224_APP_CRC_FAIL		0x40	/* valid 7 8 bit only */
#define MXT224_BOOT_STATUS_MASK	0x3f

/* Command to unlock bootloader */
#define MXT224_UNLOCK_CMD_MSB		0xaa
#define MXT224_UNLOCK_CMD_LSB		0xdc

#define ID_BLOCK_SIZE			7

#define DRIVER_FILTER

#define MXT224_STATE_INACTIVE		-1
#define MXT224_STATE_RELEASE		0
#define MXT224_STATE_PRESS		1
#define MXT224_STATE_MOVE		2

#define MAX_USING_FINGER_NUM 10

#define T7_MAXADDR			2
#define T8_MAXADDR			9
#define T9_MAXADDR			34
#define T19_MAXADDR			15
#define T38_MAXADDR			7
#define T46_MAXADDR			7
#define T48_MAXADDR			53

static bool movefilter_t48_req = false;
static u8 movefilter_t48_val = 81;

static bool numtouch_t48_req = false;
static u8 numtouch_t48_val = 10;

static bool threshold_t48_req = false;
static u8 threshold_t48_val = 17;

static bool parameter1_t48_req = false;
static u32 parameter1_t48_addr;
static u8 parameter1_t48_val;

static bool parameter2_t48_req = false;
static u32 parameter2_t48_addr;
static u8 parameter2_t48_val;

static bool parameter3_t48_req = false;
static u32 parameter3_t48_addr;
static u8 parameter3_t48_val;

/* cocafe: Touch Booster Control */
#define TOUCHBOOST_FREQ_DEF		400000

static bool touchboost = false;
static bool touchboost_ape = true;
static bool touchboost_ddr = true;
static unsigned int touchboost_freq = TOUCHBOOST_FREQ_DEF;

/* cocafe: SweepToWake with wakelock implementation */
#define ABS_THRESHOLD_X			120
#define ABS_THRESHOLD_Y			240

#if CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
static struct wake_lock s2w_wakelock;
#endif

static int x_press, x_release;
static int y_press, y_release;

static int x_threshold = ABS_THRESHOLD_X;
static int y_threshold = ABS_THRESHOLD_Y;

static bool is_suspend = false;
static bool waking_up = false;

static bool sweep2wake = false;

/* control for pinch2wake */
static bool pinch2wake = false;
module_param(pinch2wake, bool, 0644);

static void mxt224e_ponkey_thread(struct work_struct *mxt224e_ponkey_work)
{
	waking_up = true;

	pr_err("[TSP] %s fn\n", __func__);

	ab8500_ponkey_emulator(1);	/* press */

	msleep(100);

	ab8500_ponkey_emulator(0);	/* release */
	
	waking_up = false;
}
static DECLARE_WORK(mxt224e_ponkey_work, mxt224e_ponkey_thread);

static void mxt224e_tsp_off(void);
static void mxt224e_tsp_on(void);

/*
static pin_cfg_t janice_mxt224e_pins_wakeup[] = 
{
	GPIO94_GPIO | PIN_OUTPUT_HIGH | PIN_SLPM_OUTPUT_HIGH |
		PIN_SLPM_WAKEUP_ENABLE | PIN_SLPM_PDIS_DISABLED, 
};
*/

/* cocafe: Debugging Prints */
static bool debug_mask = false;
module_param(debug_mask, bool, 0644);

struct object_t {
	u8 object_type;
	u16 i2c_address;
	u8 size;
	u8 instances;
	u8 num_report_ids;
} __packed;

struct finger_info {
	s16 x;
	s16 y;
	s16 z;
	u16 w;
	s8 state;
	int16_t component;
};

struct mxt224_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct early_suspend early_suspend;
	u8 family_id;
	int finger_mask;
	int gpio_read_done;
	struct object_t *objects;
	struct mutex lock;
	bool enabled;
	u8 objects_len;
	u8 tsp_version;
	u8 tsp_build;
	const u8 *power_cfg;
	u8 finger_type;
	u16 msg_proc;
	u16 cmd_proc;
	u16 msg_object_size;
	u32 x_dropbits:2;
	u32 y_dropbits:2;
	u8 atchcalst;
	u8 atchcalsthr;
	u8 tchthr_batt;
	u8 tchthr_batt_init;
	u8 tchthr_charging;
	u8 noisethr_batt;
	u8 noisethr_charging;
	u8 movfilter_batt;
	u8 movfilter_charging;
	u8 calcfg_batt_e;
	u8 calcfg_charging_e;
	u8 atchfrccalthr_e;
	u8 atchfrccalratio_e;
	const u8 *t48_config_batt_e;
	const u8 *t48_config_chrg_e;
	void (*power_con)(bool);
	void (*register_cb)(void *);
	void (*read_ta_status)(void *);
	const u8 *config_fw_version;
	int num_fingers;
#if defined(TOUCH_BOOSTER)
	u8 finger_cnt;
#endif
	struct finger_info fingers[];
};

struct mxt224_data *copy_data;
int touch_is_pressed;
EXPORT_SYMBOL(touch_is_pressed);
static int mxt224_enabled;
static bool g_debug_switch;
static int firm_status_data;
bool tsp_deepsleep;
EXPORT_SYMBOL(tsp_deepsleep);
static bool auto_cal_flag;
static int not_yet_count;

extern u8 vbus_state;

static u8 valid_touch;

#define CLEAR_MEDIAN_FILTER_ERROR

#ifdef CLEAR_MEDIAN_FILTER_ERROR
enum {
	ERR_RTN_CONDITION_T9,
	ERR_RTN_CONDITION_T48,
	ERR_RTN_CONDITION_IDLE,
	ERR_RTN_CONDITION_MAX
};

static int gErrCondition = ERR_RTN_CONDITION_IDLE;
#endif

struct t48_median_config_t {
	 bool median_on_flag;
	 bool mferr_setting;
	 uint8_t mferr_count;
	 uint8_t t46_actvsyncsperx_for_mferr;
	 uint8_t t48_mfinvlddiffthr_for_mferr;
	 uint8_t t48_mferrorthr_for_mferr;
	 uint8_t t48_thr_for_mferr;
	 uint8_t t48_movfilter_for_mferr;
} __packed;
static struct t48_median_config_t noise_median; /* 110927 gumi noise */


static int threshold = 55;
static int threshold_e = 50;

static int read_mem(struct mxt224_data *data, u16 reg, u8 len, u8 *buf)
{
	int ret;
	u16 le_reg = cpu_to_le16(reg);
	struct i2c_msg msg[2] = {
		{
			.addr = data->client->addr,
			.flags = 0,
			.len = 2,
			.buf = (u8 *)&le_reg,
		},
		{
			.addr = data->client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	ret = i2c_transfer(data->client->adapter, msg, 2);
	if (ret < 0)
		return ret;

	return ret == 2 ? 0 : -EIO;
}

static int write_mem(struct mxt224_data *data, u16 reg, u8 len, const u8 *buf)
{
	int ret;
	u8 tmp[len + 2];

	put_unaligned_le16(cpu_to_le16(reg), tmp);
	memcpy(tmp + 2, buf, len);

	ret = i2c_master_send(data->client, tmp, sizeof(tmp));

	if (ret < 0)
		return ret;

	return ret == sizeof(tmp) ? 0 : -EIO;
}

static int __devinit mxt224_reset(struct mxt224_data *data)
{
	u8 buf = 1u;
	return write_mem(data, data->cmd_proc + CMD_RESET_OFFSET, 1, &buf);
}

static int __devinit mxt224_backup(struct mxt224_data *data)
{
	u8 buf = 0x55u;
	return write_mem(data, data->cmd_proc + CMD_BACKUP_OFFSET, 1, &buf);
}

static int get_object_info(struct mxt224_data *data,
		u8 object_type, u16 *size, u16 *address)
{
	int i;

	for (i = 0; i < data->objects_len; i++) {
		if (data->objects[i].object_type == object_type) {
			*size = data->objects[i].size + 1;
			*address = data->objects[i].i2c_address;
			return 0;
		}
	}

	return -ENODEV;
}

static int write_config(struct mxt224_data *data, u8 type, const u8 *cfg)
{
	int ret;
	u16 address = 0;
	u16 size = 0;

	ret = get_object_info(data, type, &size, &address);

	if (size == 0 && address == 0)
		ret = 0;
	else
		ret = write_mem(data, address, size, cfg);

	return ret;
}


static u32 __devinit crc24(u32 crc, u8 byte1, u8 byte2)
{
	static const u32 crcpoly = 0x80001B;
	u32 res;
	u16 data_word;

	data_word = (((u16)byte2) << 8) | byte1;
	res = (crc << 1) ^ (u32)data_word;

	if (res & 0x1000000)
		res ^= crcpoly;

	return res;
}

static int __devinit calculate_infoblock_crc(struct mxt224_data *data,
							u32 *crc_pointer)
{
	u32 crc = 0;
	u8 mem[7 + data->objects_len * 6];
	int status;
	int i;

	status = read_mem(data, 0, sizeof(mem), mem);

	if (status)
		return status;

	for (i = 0; i < sizeof(mem) - 1; i += 2)
		crc = crc24(crc, mem[i], mem[i + 1]);

	*crc_pointer = crc24(crc, mem[i], 0) & 0x00FFFFFF;

	return 0;
}

/*
static void set_autocal(u8 val)
{
	int error;
	u16 size;
	u16 obj_address = 0;
	struct mxt224_data *data = copy_data;

	get_object_info(data,
		GEN_ACQUISITIONCONFIG_T8, &size, &obj_address);
	error = write_mem(data, obj_address+4, 1, &val);
	if (error < 0)
		printk(KERN_ERR "[TSP] %s, %d Error!!\n", __func__, __LINE__);

	if (val > 0) {
		auto_cal_flag = 1;
		printk(KERN_DEBUG"[TSP] auto calibration enabled : %d\n", val);
	} else {
		auto_cal_flag = 0;
		printk(KERN_DEBUG"[TSP] auto calibration disabled\n");
	}
}
*/

static unsigned int mxt_time_point;
static unsigned int mxt_time_diff;
static unsigned int mxt_timer_state;
static unsigned int good_check_flag;
static u8 cal_check_flag;
//static u8 atchfrccalthr = 40;
//static u8 atchfrccalratio = 55;

static u8 Doing_calibration_falg;

uint8_t calibrate_chip(void)
{
	u8 cal_data = 1;
	int ret = 0;
	u8 atchcalst_tmp, atchcalsthr_tmp;
	u16 obj_address = 0;
	u16 size = 1;
	int ret1 = 0;

	not_yet_count = 0;

	if (cal_check_flag == 0) {

		ret = get_object_info(copy_data,
				GEN_ACQUISITIONCONFIG_T8, &size, &obj_address);
		size = 1;

		/* change calibration suspend settings to zero
			until calibration confirmed good */
		/* resume calibration must be performed with zero settings */
		atchcalst_tmp = 0;
		atchcalsthr_tmp = 0;
		auto_cal_flag = 0;
		ret = write_mem(copy_data, obj_address+6, size, &atchcalst_tmp);
		ret1 = write_mem(copy_data, obj_address+7, size, &atchcalsthr_tmp);
		if (copy_data->family_id == 0x81) { /* mxT224E */
			ret = write_mem(copy_data,
				obj_address+8,
				1, &atchcalst_tmp); /* forced cal thr  */
			ret1 = write_mem(copy_data,
				obj_address+9,
				1, &atchcalsthr_tmp); /* forced cal thr ratio */
		}
	}

	/* send calibration command to the chip */
	if (!ret && !ret1 && !Doing_calibration_falg) {
		/* change calibration suspend settings to zero until calibration confirmed good */
		ret = write_mem(copy_data,
			copy_data->cmd_proc + CMD_CALIBRATE_OFFSET,
			1, &cal_data);

		/* set flag for calibration lockup recovery
			if cal command was successful */
		if (!ret) {
			/* set flag to show we must still confirm
				if calibration was good or bad */
			cal_check_flag = 1u;
			Doing_calibration_falg = 1;
			printk(KERN_ERR "[TSP] Calibration succeeded!!!\n");
		}
	}
	return ret;
}

static int check_abs_time(void)
{
	if (!mxt_time_point)
		return 0;

	mxt_time_diff = jiffies_to_msecs(jiffies) - mxt_time_point;

	if (mxt_time_diff > 0)
		return 1;
	else
		return 0;
}

static void mxt224_ta_probe(int __vbus_state)
{
	struct mxt224_data *data = copy_data;
	u16 obj_address = 0;
	u16 size_one;
	int ret;
	u8 value;
//	u8 val = 0;
//	unsigned int register_address = 7;
	u8 noise_threshold;
	u8 movfilter;
	u8 blen;
	u16 size;
	u8 active_depth;
	u8 charge_time;

	if (!data) {
		printk(KERN_ERR "mxt224e: %s: no platform data\n", __func__);
		return;
	}

	if (!data->enabled) {
		printk(KERN_ERR "[TSP] mxt224_enabled is 0\n");
		return;
	}

	if (__vbus_state) {
		threshold = copy_data->tchthr_charging;
		threshold_e = 45;

		/* noise_threshold = copy_data->noisethr_charging; */
		movfilter = 47; /* copy_data->movfilter_charging; */
		blen = 16;
		active_depth = 34; /* 0x22; 34 */
		charge_time = 22;
	#ifdef CLEAR_MEDIAN_FILTER_ERROR
		gErrCondition = ERR_RTN_CONDITION_MAX;
		noise_median.mferr_setting = false;
	#endif
	} else {
		threshold = copy_data->tchthr_batt;
		threshold_e = 40;

		/* noise_threshold = copy_data->noisethr_batt; */
		movfilter = 46; /* copy_data->movfilter_batt; */
		blen = 32;
		active_depth = 30;
		charge_time = 22;
	#ifdef CLEAR_MEDIAN_FILTER_ERROR
		gErrCondition =  ERR_RTN_CONDITION_IDLE;
		noise_median.mferr_count = 0;
		noise_median.mferr_setting = false;
		noise_median.median_on_flag = false;
	#endif
	}

	if (copy_data->family_id == 0x81) { 	/* MXT224E */

#ifdef CLEAR_MEDIAN_FILTER_ERROR
		if (!__vbus_state) {
//			printk(KERN_INFO "[TSP] Clean median filter errors\n");
			ret = get_object_info(copy_data,
				TOUCH_MULTITOUCHSCREEN_T9,
				&size_one, &obj_address);
			/* blen */
			value = 32;
			write_mem(copy_data, obj_address+6, 1, &value);
			/* threshold */
			value = 40;
			write_mem(copy_data, obj_address+7, 1, &value);
			/* move Filter */
			value = 46;
			write_mem(copy_data, obj_address+13, 1, &value);
			/* next di */
		}
#endif

		value = active_depth;
		ret = get_object_info(copy_data,
			SPT_CTECONFIG_T46, &size_one, &obj_address);
		write_mem(copy_data, obj_address+3, 1, &value);

		/* GEN_ACQUISITIONCONFIG_T8 */
		value = charge_time;
		ret = get_object_info(copy_data,
			GEN_ACQUISITIONCONFIG_T8, &size_one, &obj_address);
		write_mem(copy_data, obj_address+0, 1, &value);

		/* PROCG_NOISESUPPRESSION_T48 */
		ret = get_object_info(copy_data,
			PROCG_NOISESUPPRESSION_T48, &size_one, &obj_address);

		if (__vbus_state) {
			write_config(copy_data,
				copy_data->t48_config_chrg_e[0],
				copy_data->t48_config_chrg_e + 1);
		} else {
			write_config(copy_data,
				copy_data->t48_config_batt_e[0],
				copy_data->t48_config_batt_e + 1);
		}
//		printk(KERN_INFO "[TSP] TA_probe MXT224E T%d Byte%d is %d\n", 48, register_address, val);
	} else if (copy_data->family_id == 0x80) { 	/* MXT224 */
		get_object_info(copy_data,
			TOUCH_MULTITOUCHSCREEN_T9, &size, &obj_address);
		value = (u8)threshold;
		write_mem(copy_data, obj_address+7, 1, &value);

		write_mem(copy_data, obj_address+13, 1, &movfilter);

		get_object_info(copy_data,
			PROCG_NOISESUPPRESSION_T22, &size, &obj_address);
		write_mem(copy_data,
			obj_address+8, 1, &noise_threshold);
	}

	/* TODO: Write registers after TA probe */
	if (!is_suspend) {
		if (movefilter_t48_req) {
			ret = get_object_info(copy_data, PROCG_NOISESUPPRESSION_T48, &size_one, &obj_address);
			write_mem(copy_data, obj_address + 39, 1, &movefilter_t48_val);
			pr_err("[TSP] [T48] [movefilter]\n");
		}
		if (numtouch_t48_req) {
			ret = get_object_info(copy_data, PROCG_NOISESUPPRESSION_T48, &size_one, &obj_address);
			write_mem(copy_data, obj_address + 40, 1, &numtouch_t48_val);
			pr_err("[TSP] [T48] [numtouch]\n");
		}
		if (threshold_t48_req) {
			ret = get_object_info(copy_data, PROCG_NOISESUPPRESSION_T48, &size_one, &obj_address);
			write_mem(copy_data, obj_address + 35, 1, &threshold_t48_val);
			pr_err("[TSP] [T48] [threshold]\n");
		}
		if (parameter1_t48_req) {
			ret = get_object_info(copy_data, PROCG_NOISESUPPRESSION_T48, &size_one, &obj_address);
			write_mem(copy_data, obj_address + parameter1_t48_addr, 1, &parameter1_t48_val);
			pr_err("[TSP] [T48] [parameter1]\n");
		}
		if (parameter2_t48_req) {
			ret = get_object_info(copy_data, PROCG_NOISESUPPRESSION_T48, &size_one, &obj_address);
			write_mem(copy_data, obj_address + parameter2_t48_addr, 1, &parameter2_t48_val);
			pr_err("[TSP] [T48] [parameter2]\n");
		}
		if (parameter3_t48_req) {
			ret = get_object_info(copy_data, PROCG_NOISESUPPRESSION_T48, &size_one, &obj_address);
			write_mem(copy_data, obj_address + parameter3_t48_addr, 1, &parameter3_t48_val);
			pr_err("[TSP] [T48] [parameter3]\n");
		}
	}
}

void mxt224e_ts_change_vbus_state(bool vbus_status) {
	printk(KERN_INFO "[TSP] mxt224e vbus state changed (%d)\n", vbus_status);
	mxt224_ta_probe((int)vbus_status);
}
EXPORT_SYMBOL(mxt224e_ts_change_vbus_state);

void check_chip_calibration(struct mxt224_data *data)
{
	u8 data_buffer[100] = { 0 };
	u8 try_ctr = 0;
	u8 data_byte = 0xF3; /* dianostic command to get touch flags */
	u8 tch_ch = 0, atch_ch = 0;
	/* u8 atchcalst, atchcalsthr; */
	u8 check_mask;
	u8 i, j = 0;
	u8 x_line_limit;
	int ret;
	u16 size;
	u16 object_address = 0;

	/* we have had the first touchscreen or face suppression message
	* after a calibration - check the sensor state and try to confirm if
	* cal was good or bad */

	/* get touch flags from the chip using the diagnostic object */
	/* write command to command processor to get touch flags - 0xF3 Command required to do this */
	write_mem(copy_data, copy_data->cmd_proc + CMD_DIAGNOSTIC_OFFSET, 1, &data_byte);


	/* get the address of the diagnostic object so we can get the data we need */
	ret = get_object_info(copy_data,
		DEBUG_DIAGNOSTIC_T37, &size, &object_address);

	usleep_range(10000, 10000);

	/* read touch flags from the diagnostic object - clear buffer so the while loop can run first time */
	memset(data_buffer , 0xFF, sizeof(data_buffer));

	/* wait for diagnostic object to update */
	while (!((data_buffer[0] == 0xF3) && (data_buffer[1] == 0x00))) {
		/* wait for data to be valid  */
		if (try_ctr > 10) {

			/* Failed! */
			printk(KERN_ERR "[TSP] Diagnostic data didn't update!!\n");
			mxt_timer_state = 0;
			break;
		}

		usleep_range(2000, 3000);
		try_ctr++; /* timeout counter */
		read_mem(copy_data, object_address, 2, data_buffer);
	}


	/* data is ready - read the detection flags */
	read_mem(copy_data, object_address, 82, data_buffer);

	/* data array is 20 x 16 bits for each set of flags, 2 byte header, 40 bytes for touch flags 40 bytes for antitouch flags*/

	/* count up the channels/bits if we recived the data properly */
	if ((data_buffer[0] == 0xF3) && (data_buffer[1] == 0x00)) {

		/* mode 0 : 16 x line, mode 1 : 17 etc etc upto mode 4.*/
		/* x_line_limit = 16 + cte_config.mode; */
		x_line_limit = 16 + 3;

		if (x_line_limit > 20) {
			/* hard limit at 20 so we don't over-index the array */
			x_line_limit = 20;
		}

		/* double the limit as the array is in bytes not words */
		x_line_limit = x_line_limit << 1;

		/* count the channels and print the flags to the log */
		for (i = 0; i < x_line_limit; i += 2) { /* check X lines - data is in words so increment 2 at a time */

			/* print the flags to the log - only really needed for debugging */

			/* count how many bits set for this row */
			for (j = 0; j < 8; j++) {
				/* create a bit mask to check against */
				check_mask = 1 << j;

				/* check detect flags */
				if (data_buffer[2+i] & check_mask)
					tch_ch++;

				if (data_buffer[3+i] & check_mask)
					tch_ch++;

				/* check anti-detect flags */
				if (data_buffer[42+i] & check_mask)
					atch_ch++;

				if (data_buffer[43+i] & check_mask)
					atch_ch++;

			}
		}

		#ifdef TOUCH_DEBUGGER
		if (debug_mask)
			printk(KERN_INFO "[TSP] t: %d  a: %d\n", tch_ch, atch_ch);
		#endif

		/* send page up command so we can detect
		   when data updates next time,
		   page byte will sit at 1 until we next send F3 command */
		data_byte = 0x01;

		write_mem(copy_data, copy_data->cmd_proc + CMD_DIAGNOSTIC_OFFSET, 1, &data_byte);

		/* process counters and decide if we must re-calibrate or if cal was good */
		if (tch_ch+atch_ch >= 25) {
			/* cal was bad - must recalibrate
			   and check afterwards */
			printk(KERN_ERR "[TSP] tch_ch+atch_ch Calibration was bad\n");
			calibrate_chip();
			mxt_timer_state = 0;
			mxt_time_point = jiffies_to_msecs(jiffies);
		} else if ((tch_ch > 0) && (atch_ch == 0)) {
			/* cal was good - don't need to check any more */
			if (!check_abs_time())
				mxt_time_diff = 301;

			if (mxt_timer_state == 1) {
				if (mxt_time_diff > 300) {
					printk(KERN_INFO "[TSP] Calibration was good\n");
					cal_check_flag = 0;
					good_check_flag = 0;
					mxt_timer_state = 0;
					data_byte = 0;
					not_yet_count = 0;
					auto_cal_flag = 0;
					mxt_time_point = jiffies_to_msecs(jiffies);

					ret = get_object_info(copy_data,
						GEN_ACQUISITIONCONFIG_T8,
						&size, &object_address);

					/* change calibration suspend settings to zero until calibration confirmed good */
					/* store normal settings */
					write_mem(copy_data, object_address+6, 1, &copy_data->atchcalst);
					write_mem(copy_data, object_address+7, 1, &copy_data->atchcalsthr);
					if (copy_data->family_id == 0x81) {	/*  : MXT-224E */
						write_mem(copy_data, object_address+8, 1, &copy_data->atchfrccalthr_e);
						write_mem(copy_data, object_address+9, 1, &copy_data->atchfrccalratio_e);
					} else {
						printk(KERN_INFO "[TSP] vbus_state: %d (%s)\n", (int)vbus_state, 
										vbus_state ? "TA" : "Battery");
						if (vbus_state == 0)
							mxt224_ta_probe(vbus_state);
					}
				} else  {
					cal_check_flag = 1;
				}
			} else {
				mxt_timer_state = 1;
				mxt_time_point = jiffies_to_msecs(jiffies);
				cal_check_flag = 1;
			}
		} else if (atch_ch >= 5) {
			/* cal was bad - must recalibrate
			   and check afterwards */
			printk(KERN_ERR "[TSP] Calibration was bad\n");
			calibrate_chip();
			mxt_timer_state = 0;
			mxt_time_point = jiffies_to_msecs(jiffies);
		} else {
			if (atch_ch >= 1)
				not_yet_count++;
			if (not_yet_count > 5) {
				printk(KERN_ERR "[TSP] not_yet_count calibration was bad\n");
				calibrate_chip();
				mxt_timer_state = 0;
				mxt_time_point = jiffies_to_msecs(jiffies);
			} else {
				/* we cannot confirm if good or bad -
				   we must wait for next touch message
				   to confirm */
				printk(KERN_ERR "[TSP] Calibration is waiting\n");
				cal_check_flag = 1u;
				mxt_timer_state = 0;
				mxt_time_point = jiffies_to_msecs(jiffies);
			}
		}
	}
}
 
#if defined(DRIVER_FILTER)
static void equalize_coordinate(bool detect, u8 id, u16 *px, u16 *py)
{
	static int tcount[MAX_USING_FINGER_NUM] = { 0, };
	static u16 pre_x[MAX_USING_FINGER_NUM][4] = {{0}, };
	static u16 pre_y[MAX_USING_FINGER_NUM][4] = {{0}, };
	int coff[4] = {0,};
	int distance = 0;

	if (detect)
		tcount[id] = 0;

	pre_x[id][tcount[id]%4] = *px;
	pre_y[id][tcount[id]%4] = *py;

	if (tcount[id] > 3) {
		{
			distance = abs(pre_x[id][(tcount[id]-1)%4] - *px)
				+ abs(pre_y[id][(tcount[id]-1)%4] - *py);

			coff[0] = (u8)(2 + distance/5);
			if (coff[0] < 8) {
				coff[0] = max(2, coff[0]);
				coff[1] = min((8 - coff[0]), (coff[0]>>1)+1);
				coff[2] = min((8 - coff[0] - coff[1]),
							(coff[1]>>1)+1);
				coff[3] = 8 - coff[0] - coff[1] - coff[2];

				/* printk(KERN_DEBUG
					"[TSP] %d, %d, %d, %d",
					coff[0], coff[1], coff[2], coff[3]); */

				*px = (u16)((*px*(coff[0]) 
				 + pre_x[id][(tcount[id]-1)%4]*(coff[1])
				 + pre_x[id][(tcount[id]-2)%4]*(coff[2])
				 + pre_x[id][(tcount[id]-3)%4]*(coff[3]))/8);
				*py = (u16)((*py*(coff[0])
				 + pre_y[id][(tcount[id]-1)%4]*(coff[1])
				 + pre_y[id][(tcount[id]-2)%4]*(coff[2])
				 + pre_y[id][(tcount[id]-3)%4]*(coff[3]))/8);
			} else {
				*px = (u16)
				 ((*px*4 + pre_x[id][(tcount[id]-1)%4])/5);
				*py = (u16)
				 ((*py*4 + pre_y[id][(tcount[id]-1)%4])/5);
			}
		}
	}
	tcount[id]++;
}
#endif  /* DRIVER_FILTER */

static int __devinit mxt224_init_touch_driver(struct mxt224_data *data)
{
	struct object_t *object_table;
	u32 read_crc = 0;
	u32 calc_crc;
	u16 crc_address;
	u16 dummy;
	int i;
	u8 id[ID_BLOCK_SIZE];
	int ret;
	u8 type_count = 0;
	u8 tmp;

	ret = read_mem(data, 0, sizeof(id), id);
	if (ret)
		return ret;

	dev_info(&data->client->dev, "family = %#02x, variant = %#02x, version "
			"= %#02x, build = %d\n", id[0], id[1], id[2], id[3]);
	printk(KERN_ERR "family = %#02x, variant = %#02x, version "
			"= %#02x, build = %d\n", id[0], id[1], id[2], id[3]);
	dev_dbg(&data->client->dev, "matrix X size = %d\n", id[4]);
	dev_dbg(&data->client->dev, "matrix Y size = %d\n", id[5]);

	data->family_id = id[0];
	data->tsp_version = id[2];
	data->tsp_build = id[3];
	data->objects_len = id[6];

	object_table = kmalloc(data->objects_len * sizeof(*object_table),
				GFP_KERNEL);
	if (!object_table)
		return -ENOMEM;

	ret = read_mem(data, OBJECT_TABLE_START_ADDRESS,
			data->objects_len * sizeof(*object_table),
			(u8 *)object_table);
	if (ret)
		goto err;

	for (i = 0; i < data->objects_len; i++) {
		object_table[i].i2c_address =
				le16_to_cpu(object_table[i].i2c_address);
		tmp = 0;
		if (object_table[i].num_report_ids) {
			tmp = type_count + 1;
			type_count += object_table[i].num_report_ids *
						(object_table[i].instances + 1);
		}
		switch (object_table[i].object_type) {
		case TOUCH_MULTITOUCHSCREEN_T9:
			data->finger_type = tmp;
			dev_dbg(&data->client->dev, "Finger type = %d\n",
						data->finger_type);
			break;
		case GEN_MESSAGEPROCESSOR_T5:
			data->msg_object_size = object_table[i].size + 1;
			dev_dbg(&data->client->dev, "Message object size = "
						"%d\n", data->msg_object_size);
			break;
		}
	}

	data->objects = object_table;

	/* Verify CRC */
	crc_address = OBJECT_TABLE_START_ADDRESS +
			data->objects_len * OBJECT_TABLE_ELEMENT_SIZE;

#ifdef __BIG_ENDIAN
#error The following code will likely break on a big endian machine
#endif
	ret = read_mem(data, crc_address, 3, (u8 *)&read_crc);
	if (ret)
		goto err;

	read_crc = le32_to_cpu(read_crc);

	ret = calculate_infoblock_crc(data, &calc_crc);
	if (ret)
		goto err;

	if (read_crc != calc_crc) {
		dev_err(&data->client->dev, "CRC error\n");
		ret = -EFAULT;
		goto err;
	}

	ret = get_object_info(data, GEN_MESSAGEPROCESSOR_T5, &dummy,
					&data->msg_proc);
	if (ret)
		goto err;

	ret = get_object_info(data, GEN_COMMANDPROCESSOR_T6, &dummy,
					&data->cmd_proc);
	if (ret)
		goto err;

	return 0;

err:
	kfree(object_table);
	return ret;
}

static void report_input_data(struct mxt224_data *data)
{
	int id;
	int count = 0;
	int y_up;			// stores value of Y-axis when touch event is completed
	int y_down;			// stores value of Y-axis when touch event starts

	if (!valid_touch)
		return;

	id = data->finger_mask;

	if (data->fingers[id].state == MXT224_STATE_INACTIVE)
		goto out;

	#if defined(TOUCH_BOOSTER)
	if (touchboost) {
		if (!is_suspend) {
			if (data->fingers[id].state == MXT224_STATE_PRESS) {
				if (data->finger_cnt == 0) {
					if (touchboost_ape) {
						prcmu_qos_update_requirement(
							PRCMU_QOS_APE_OPP,
							(char *)data->client->name,
							PRCMU_QOS_APE_OPP_MAX);
					}
					if (touchboost_ddr) {
						prcmu_qos_update_requirement(
							PRCMU_QOS_DDR_OPP,
							(char *)data->client->name,
							PRCMU_QOS_DDR_OPP_MAX);
					}
					/* Allow to disable cpufreq requirement */
					if (touchboost_freq != 0) {
						prcmu_qos_update_requirement(
							PRCMU_QOS_ARM_KHZ,
							(char *)data->client->name,
							touchboost_freq);
					}
				}

				data->finger_cnt++;

			} else if (data->fingers[id].state == MXT224_STATE_RELEASE) {
				if (data->finger_cnt > 0)
					data->finger_cnt--;
	
				if (data->finger_cnt == 0) {
					prcmu_qos_update_requirement(
						PRCMU_QOS_APE_OPP,(
						char *)data->client->name,
						PRCMU_QOS_DEFAULT_VALUE);
					prcmu_qos_update_requirement(
						PRCMU_QOS_DDR_OPP,
						(char *)data->client->name,
						PRCMU_QOS_DEFAULT_VALUE);
					prcmu_qos_update_requirement(
						PRCMU_QOS_ARM_KHZ,
						(char *)data->client->name,
						PRCMU_QOS_DEFAULT_VALUE);
				}
			}
		}
	}
	#endif /* TOUCH_BOOSTER */

	input_mt_slot(data->input_dev, id);
	input_mt_report_slot_state(data->input_dev,
				MT_TOOL_FINGER, !!data->fingers[id].z);

	if (data->fingers[id].z != 0) {
		input_report_abs(data->input_dev,
				ABS_MT_POSITION_X, data->fingers[id].x);
		input_report_abs(data->input_dev,
				ABS_MT_POSITION_Y, data->fingers[id].y);
		input_report_abs(data->input_dev,
				ABS_MT_PRESSURE, data->fingers[id].z);
		input_report_abs(data->input_dev,
				ABS_MT_TOUCH_MAJOR, data->fingers[id].w);
	}

	#ifdef TOUCH_DEBUGGER
	if (debug_mask) {	/* This is noisy */	/* But helpful in debugging */
		if (data->fingers[id].state == MXT224_STATE_PRESS || data->fingers[id].state == MXT224_STATE_RELEASE) {
			printk("[TSP] id[%d] x=%d y=%d z=%d w=%d\n",
				id , data->fingers[id].x, data->fingers[id].y,
				data->fingers[id].z, data->fingers[id].w);
		}
	}
	#endif

	if (is_suspend) {
		if (sweep2wake) {
			if (data->fingers[0].state == MXT224_STATE_PRESS) {
				x_press = data->fingers[0].x;
				y_press = data->fingers[0].y;
			} else if (data->fingers[0].state == MXT224_STATE_RELEASE) {
				x_release = data->fingers[0].x;
				y_release = data->fingers[0].y;
				if ((abs(x_release - x_press) >= x_threshold) ||
					(abs(y_release - y_press) >= y_threshold)) {
						if (!waking_up)
							schedule_work(&mxt224e_ponkey_work);
				}
			}
		}
	}
	
	/* pinch2wake implementation */
	if (pinch2wake) {
		if (data->fingers[2].state == MXT224_STATE_PRESS) {	// We are using 3 fingers
			y_down = data->fingers[2].y;
		}
		if (data->fingers[2].state == MXT224_STATE_RELEASE) {
			y_up = data->fingers[2].y;
		}
		if ((y_up - y_down) >= 700) {				// Threshold = 700 pixels
			ab8500_ponkey_emulator(1);
			msleep(250);
			ab8500_ponkey_emulator(0);
		}
	}

	if (data->fingers[id].state == MXT224_STATE_RELEASE)
		data->fingers[id].state = MXT224_STATE_INACTIVE;
	else {
		data->fingers[id].state = MXT224_STATE_MOVE;
		count++;
	}

	input_sync(data->input_dev);

out:
	touch_is_pressed = count ? 1 : 0;
	data->finger_mask = -1;
}

#ifdef CLEAR_MEDIAN_FILTER_ERROR
int Check_Err_Condition(void)
{
	int rtn;

	switch (gErrCondition) {
	case ERR_RTN_CONDITION_IDLE:
		rtn = ERR_RTN_CONDITION_T9;
		break;
	}
	return rtn;
}


static int median_err_setting(struct mxt224_data *data)
{
	u16 obj_address;
	u16 size_one;
	u8 value, state;
	int ret = 0;

	if (!vbus_state) {
		gErrCondition = Check_Err_Condition();

		switch (gErrCondition) {
		case ERR_RTN_CONDITION_T9:
		{
/*			ret = get_object_info(copy_data,
				TOUCH_MULTITOUCHSCREEN_T9,
				&size_one, &obj_address);
			value = 16;
			write_mem(copy_data, obj_address+6, 1, &value);
			value = 40;
			write_mem(copy_data, obj_address+7, 1, &value);
			value = 80;
			write_mem(copy_data, obj_address+13, 1, &value);
*/
			ret = get_object_info(copy_data,
				SPT_CTECONFIG_T46, &size_one, &obj_address);
			value = 32;
			write_mem(copy_data,
				obj_address+3, 1, &value);
			ret = get_object_info(copy_data,
				PROCG_NOISESUPPRESSION_T48,
				&size_one, &obj_address);
			value = 24;
			write_mem(copy_data, obj_address+3, 1, &value);
			value = 1;
			write_mem(copy_data, obj_address+8, 1, &value);
			value = 2;
			write_mem(copy_data, obj_address+9, 1, &value);
			value = 100;
			write_mem(copy_data, obj_address+17, 1, &value);
			value = 64;
			write_mem(copy_data, obj_address+19, 1, &value);
			value = 100;
			write_mem(copy_data, obj_address+22, 1, &value);
			value = 100;
			write_mem(copy_data, obj_address+25, 1, &value);
			value = 0;
			write_mem(copy_data, obj_address+34, 1, &value);
			value = 27;
			write_mem(copy_data, obj_address+35, 1, &value);
			value = 81;
			write_mem(copy_data, obj_address+39, 1, &value);
		}
				break;
		}
	} else {
/*
	get_object_info(copy_data,
		SPT_USERDATA_T38, &size_one, &obj_address);
	read_mem(copy_data, obj_address+1, 1, &value);
	printk(KERN_ERR"[TSP]info value is %d\n", value);
*/
		value = 1;
		if (noise_median.mferr_count < 3)
			noise_median.mferr_count++;
		if (!(noise_median.mferr_count%value)
			&& (noise_median.mferr_count < 3)) {
			printk(KERN_DEBUG
				"[TSP] median thr noise level too high. %d\n",
				noise_median.mferr_count/value);
		state = noise_median.mferr_count/value;
/*
	get_object_info(copy_data,
		SPT_USERDATA_T38, &size_one, &obj_address);
	read_mem(copy_data,
		obj_address+2, 1, & noise_median.t48_mfinvlddiffthr_for_mferr);
	printk(KERN_ERR"[TSP]mfinvlddiffthr value is %d\n",
		noise_median.t48_mfinvlddiffthr_for_mferr);
	read_mem(copy_data,
		obj_address+3, 1, & noise_median.t48_mferrorthr_for_mferr);
	printk(KERN_ERR"[TSP]mferrorthr value is %d\n",
		noise_median.t48_mferrorthr_for_mferr);
	read_mem(copy_data,
		obj_address+4, 1, &noise_median.t46_actvsyncsperx_for_mferr);
	printk(KERN_ERR"[TSP]actvsyncsperx value is %d\n",
		noise_median.t46_actvsyncsperx_for_mferr);
	read_mem(copy_data,
		obj_address+5, 1, &noise_median.t48_thr_for_mferr);
	printk(KERN_ERR"[TSP]t48_thr_for_mferr value is %d\n",
		noise_median.t48_thr_for_mferr);
	read_mem(copy_data,
		obj_address+6, 1, &noise_median.t48_movfilter_for_mferr);
	printk(KERN_ERR"[TSP]t48_movfilter_for_mferr value is %d\n",
	noise_median.t48_movfilter_for_mferr);
*/
	   get_object_info(copy_data,
			PROCG_NOISESUPPRESSION_T48, &size_one, &obj_address);

		if (state == 1) {
			value = noise_median.t48_mfinvlddiffthr_for_mferr;
			write_mem(copy_data, obj_address+22, 1, &value);
			value = noise_median.t48_mferrorthr_for_mferr;
			write_mem(copy_data, obj_address+25, 1, &value);
			value = noise_median.t48_thr_for_mferr;
			write_mem(copy_data, obj_address+35, 1, &value);
			value = noise_median.t48_movfilter_for_mferr;
			write_mem(copy_data, obj_address+39, 1, &value);
			get_object_info(copy_data,
				SPT_CTECONFIG_T46, &size_one, &obj_address);
			value = noise_median.t46_actvsyncsperx_for_mferr;
			write_mem(copy_data, obj_address+3, 1, &value);
		} else if (state >= 2) {
			value = 10;
			write_mem(copy_data, obj_address+3, 1, &value);
			value = 1;
			write_mem(copy_data, obj_address+8, 1, &value);
			value = 2;
			write_mem(copy_data, obj_address+9, 1, &value);
			value = 100;
			write_mem(copy_data, obj_address+17, 1, &value);
			value = 20;
			write_mem(copy_data, obj_address+22, 1, &value);
			value = 38;
			write_mem(copy_data, obj_address+25, 1, &value);
			value = 40;
			write_mem(copy_data, obj_address+35, 1, &value);
			value = 81;
			write_mem(copy_data, obj_address+39, 1, &value);
			get_object_info(copy_data,
				SPT_CTECONFIG_T46, &size_one, &obj_address);
			value = 63;
			write_mem(copy_data, obj_address+3, 1, &value);
		}
		noise_median.mferr_setting = true;
		}
	}
	return 0;
}
#endif
static irqreturn_t mxt224_irq_thread(int irq, void *ptr)
{
	struct mxt224_data *data = ptr;
	struct mxt224_data *data_backup = data;
	int id;
	u8 msg[data->msg_object_size];
	u8 touch_message_flag = 0;
	int reset;

	data->finger_mask = -1;
	do {
		touch_message_flag = 0;
		if (read_mem(data, data->msg_proc, sizeof(msg), msg))
			return IRQ_HANDLED;

		if (msg[0] == 0x1) {
			if ((msg[1]&0x10) == 0x00) {/* normal mode */
				Doing_calibration_falg = 0;
				printk(KERN_ERR"[TSP] Calibration End!!!\n");
				valid_touch = 1;
				if (cal_check_flag == 1) {
					mxt_timer_state = 0;
					mxt_time_point =
						jiffies_to_msecs(jiffies);
				}
			}
			if ((msg[1]&0x04) == 0x04) /* I2C checksum error */
				printk(KERN_ERR"[TSP] i2c checksum error\n");
			if ((msg[1]&0x08) == 0x08) {/* config error */
				printk(KERN_ERR"[TSP] config error\n");
				reset = 1;
			}
			if ((msg[1]&0x10) == 0x10) /* calibration */
				printk(KERN_ERR"[TSP] Calibration!!!\n");
			if ((msg[1]&0x20) == 0x20) { /* signal error */
				printk(KERN_ERR"[TSP] signal error\n");
				reset = 1;
			}
			if ((msg[1]&0x40) == 0x40) { /* overflow */
				printk(KERN_ERR"[TSP] overflow detected\n");
				reset = 1;
			}
			if ((msg[1]&0x80) == 0x80) /* reset */
				printk(KERN_ERR"[TSP] reset is ongoing\n");
		}

		if (msg[0] == 14) {
			if ((msg[1] & 0x01) == 0x00) { /* Palm release */
				printk(KERN_ERR"[TSP] palm touch released\n");
				touch_is_pressed = 0;
			} else if ((msg[1] & 0x01) == 0x01) { /* Palm Press */
				printk(KERN_ERR"[TSP] palm touch detected\n");
				touch_is_pressed = 1;
				touch_message_flag = 1;
			}
		}

		/* freq error release */
		if ((msg[0] == 0xf) && (data->family_id == 0x80)) {
			printk(KERN_ERR"[TSP] Starting irq with 0x%2x, 0x%2x, 0x%2x, 0x%2x",
				msg[0], msg[1], msg[2], msg[3]);
			printk(KERN_ERR"[TSP] 0x%2x, 0x%2x, 0x%2x, 0x%2x\n",
				msg[4], msg[5], msg[6], msg[7]);
			if ((msg[1]&0x08) == 0x08)
				calibrate_chip();
		}

		if ((msg[0] == 18) && (data->family_id == 0x81)) {
			/*
			dev_info(&data->client->dev, "Starting irq with 0x%2x, 0x%2x, 0x%2x",
				msg[0], msg[1], msg[2], msg[3]);
			dev_info(&data->client->dev, "0x%2x, 0x%2x, 0x%2x, 0x%2x\n",
				msg[4], msg[5], msg[6], msg[7]);
			*/
			if ((msg[4]&0x5) == 0x5) {
				printk(KERN_ERR"[TSP] median filter state error!!!\n");
				median_err_setting(data);
			} else if ((msg[4]&0x4) == 0x4) {
				if ((!vbus_state)
				  && (noise_median.mferr_setting == false)
				  && (noise_median.median_on_flag == false)
					) {
					printk(KERN_ERR "[TSP] median filter ON!!!\n");
					noise_median.median_on_flag = true;
				}
			}
		}

		if (msg[0] > 1 && msg[0] < 12) {

			id = msg[0] - data->finger_type;

			if (data->family_id == 0x80) {	/*  : MXT-224 */
				if ((data->fingers[id].state >= MXT224_STATE_PRESS) && msg[1] & PRESS_MSG_MASK) {
					printk(KERN_ERR"[TSP] calibrate on ghost touch\n");
					calibrate_chip();
				}
			}

			/* If not a touch event, then keep going */
			if (id < 0 || id >= data->num_fingers)
					continue;


		  /* if (data->finger_mask & (1U << id))
				report_input_data(data); */

			if (msg[1] & RELEASE_MSG_MASK) {
				data->fingers[id].z = 0;
				data->fingers[id].w = msg[5];
				data->finger_mask = id;
				data->fingers[id].state = MXT224_STATE_RELEASE;
			} else if ((msg[1] & DETECT_MSG_MASK) && (msg[1] &
					(PRESS_MSG_MASK | MOVE_MSG_MASK))) {
				touch_message_flag = 1;
				data->fingers[id].z = msg[6];
				data->fingers[id].w = msg[5];
				data->fingers[id].x = ((msg[2] << 4) | (msg[4] >> 4)) >>
								data->x_dropbits;
				data->fingers[id].y = ((msg[3] << 4) |
						(msg[4] & 0xF)) >> data->y_dropbits;
				data->finger_mask = id;
#if defined(DRIVER_FILTER)
				if (msg[1] & PRESS_MSG_MASK) {
					equalize_coordinate(1, id, &data->fingers[id].x, &data->fingers[id].y);
					data->fingers[id].state = MXT224_STATE_PRESS;
				} else if (msg[1] & MOVE_MSG_MASK)
					equalize_coordinate(0, id, &data->fingers[id].x, &data->fingers[id].y);

#endif
#if defined(CONFIG_SHAPE_TOUCH)
				data->fingers[id].component = msg[7];
#endif
			} else if ((msg[1] & SUPPRESS_MSG_MASK) && (data->fingers[id].state != MXT224_STATE_INACTIVE)) {
				data->fingers[id].z = 0;
				data->fingers[id].w = msg[5];
				data->fingers[id].state = MXT224_STATE_RELEASE;
				data->finger_mask = id;
			} 
//			else {
//				printk(KERN_DEBUG "[TSP] Unknown state %#02x %#02x\n", msg[0], msg[1]);
				/* continue; */
//			}
		}
		if (data != data_backup)
			data = data_backup;

		if (data->finger_mask >= 0)
			report_input_data(data);

		if (touch_message_flag && (cal_check_flag))
			check_chip_calibration(data);

	} while (!gpio_get_value(data->gpio_read_done));

	return IRQ_HANDLED;
}

static void mxt224_deepsleep(struct mxt224_data *data)
{
	u8 power_cfg[3] = {0, };
	write_config(data, GEN_POWERCONFIG_T7, power_cfg);
	tsp_deepsleep = 1;
}

static void mxt224_wakeup(struct mxt224_data *data)
{
	write_config(data, GEN_POWERCONFIG_T7, data->power_cfg);
}

static int mxt224_internal_suspend(struct mxt224_data *data)
{
	int i;

	for (i = 0; i < data->num_fingers; i++) {
		if (data->fingers[i].state == MXT224_STATE_INACTIVE)
			continue;
		data->fingers[i].z = 0;
		data->fingers[i].state = MXT224_STATE_RELEASE;
		data->finger_mask = i;
		report_input_data(data);
	}

	#if defined(TOUCH_BOOSTER)
	if (touchboost) {
		if (data->finger_cnt > 0) {
			prcmu_qos_update_requirement(
				PRCMU_QOS_APE_OPP,(
				char *)data->client->name,
				PRCMU_QOS_DEFAULT_VALUE);
			prcmu_qos_update_requirement(
				PRCMU_QOS_DDR_OPP,
				(char *)data->client->name,
				PRCMU_QOS_DEFAULT_VALUE);
			prcmu_qos_update_requirement(
				PRCMU_QOS_ARM_KHZ,
				(char *)data->client->name,
				PRCMU_QOS_DEFAULT_VALUE);
	
			data->finger_cnt = 0;
		}
	}
	#endif

	if (!tsp_deepsleep)
		data->power_con(false);

	return 0;
}

static int mxt224_internal_resume(struct mxt224_data *data)
{
	if (!tsp_deepsleep)
		data->power_con(true);
	else
		mxt224_wakeup(data);

	return 0;
}

/* Turn off mxT224E touchscreen */
static void mxt224e_tsp_off(void)
{
	struct mxt224_data *data = copy_data;

	mutex_lock(&data->lock);
	if (data->enabled)
		data->enabled = 0;
	mutex_unlock(&data->lock);

	touch_is_pressed = 0;

	disable_irq(data->client->irq);
	mxt224_internal_suspend(data);
}

/* Turn on mxT224E touchscreen */
static void mxt224e_tsp_on(void)
{
	struct mxt224_data *data = copy_data;

	mutex_lock(&data->lock);

	mxt224_internal_resume(data);

	if (!data->enabled)
		data->enabled = 1;

	pr_info("[TSP] vbus_state = %d\n", (int)vbus_state);
	mxt224_ta_probe(vbus_state);

	calibrate_chip();

	enable_irq(data->client->irq);

	mutex_unlock(&data->lock);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
#define mxt224_suspend	NULL
#define mxt224_resume	NULL

static void mxt224_early_suspend(struct early_suspend *h)
{
	struct mxt224_data *data =
			container_of(h, struct mxt224_data, early_suspend);

	is_suspend = true;

	mutex_lock(&data->lock);
	if (!data->enabled)
		goto out;

	if (sweep2wake) {
		if (data->finger_cnt > 0) {
			prcmu_qos_update_requirement(
				PRCMU_QOS_APE_OPP,
				(char *)data->client->name,
				PRCMU_QOS_DEFAULT_VALUE);
			prcmu_qos_update_requirement(
				PRCMU_QOS_DDR_OPP,
				(char *)data->client->name,
				PRCMU_QOS_DEFAULT_VALUE);
			prcmu_qos_update_requirement(
				PRCMU_QOS_ARM_KHZ,
				(char *)data->client->name,
				PRCMU_QOS_DEFAULT_VALUE);
	
			data->finger_cnt = 0;
		}
	
		/* nmk_config_pins(janice_mxt224e_pins_wakeup, ARRAY_SIZE(janice_mxt224e_pins_wakeup)); */

		goto out;
	}

	disable_irq(data->client->irq);
	data->enabled = 0;
	touch_is_pressed = 0;

	if (vbus_state)
		mxt224_deepsleep(data);

#ifdef CLEAR_MEDIAN_FILTER_ERROR
	noise_median.mferr_count = 0;
	noise_median.mferr_setting = false;
	noise_median.median_on_flag = false;
#endif
	mxt224_internal_suspend(data);

out:
	mutex_unlock(&data->lock);
}

static void mxt224_late_resume(struct early_suspend *h)
{
	struct mxt224_data *data =
			container_of(h, struct mxt224_data, early_suspend);

	is_suspend = false;

	valid_touch = 0;

	mutex_lock(&data->lock);
	if (data->enabled && !sweep2wake)
		goto out;

	data->enabled = 1;

	mxt224_internal_resume(data);

	pr_info("[TSP] vbus state: %d (%s)\n", (int)vbus_state, vbus_state ? "TA" : "Batt");

	if (!(tsp_deepsleep && vbus_state))
		mxt224_ta_probe(vbus_state);

	if (tsp_deepsleep)
		tsp_deepsleep = 0;

	calibrate_chip();

	if (!sweep2wake)
		enable_irq(data->client->irq);

out:
	mutex_unlock(&data->lock);
}
#else
static int mxt224_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt224_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->lock);

	if (!data->enabled)
		goto out;
	data->enabled = 0;

	ret = mxt224_internal_suspend(data);

	touch_is_pressed = 0;
	disable_irq(data->client->irq);

out:
	mutex_unlock(&data->lock);
	return ret;
}

static int mxt224_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt224_data *data = i2c_get_clientdata(client);
	int ret = 0;

	mutex_lock(&data->lock);

	if (data->enabled)
		goto out;

	data->enabled = 1;

	ret = mxt224_internal_resume(data);

	dev_info(&data->client->dev, "vbus_state = %d\n", (int)vbus_state);
	mxt224_ta_probe(vbus_state);

	enable_irq(data->client->irq);

out:
	mutex_unlock(&data->lock);
	return ret;
}
#endif

void Mxt224_force_released(void)
{
	struct mxt224_data *data = copy_data;
	int i;

	if (!mxt224_enabled) {
		printk(KERN_ERR"[TSP] mxt224_enabled is 0\n");
		return;
	}

	for (i = 0; i < data->num_fingers; i++) {
		if (data->fingers[i].state == MXT224_STATE_INACTIVE)
			continue;
		data->fingers[i].z = 0;
		data->fingers[i].state = MXT224_STATE_RELEASE;
		data->finger_mask = i;
		report_input_data(data);
	}

	calibrate_chip();
}
EXPORT_SYMBOL(Mxt224_force_released);

static ssize_t mxt224_debug_setting(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	g_debug_switch = !g_debug_switch;
	return 0;
}

static ssize_t mxt224_object_setting(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt224_data *data = dev_get_drvdata(dev);
	unsigned int object_type;
	unsigned int object_register;
	unsigned int register_value;
	u8 value;
	u8 val;
	int ret;
	u16 address;
	u16 size;

	sscanf(buf, "%u%u%u", &object_type, &object_register, &register_value);
	dev_info(&data->client->dev,
		"set value %u on object type T%d (offset:%u)\n",
		register_value, object_type, object_register);

	ret = get_object_info(data, (u8)object_type, &size, &address);
	if (ret) {
		dev_err(&data->client->dev, "fail to get object_info\n");
		return count;
	}

	size = 1;
	value = (u8)register_value;
	write_mem(data, address+(u16)object_register, size, &value);
	read_mem(data, address+(u16)object_register, (u8)size, &val);

	dev_err(&data->client->dev, "T%d Byte%d is %d\n", object_type, object_register, val);
	return count;
}

static ssize_t mxt224_object_show(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt224_data *data = dev_get_drvdata(dev);
	unsigned int object_type;
	u8 val;
	int ret;
	u16 address;
	u16 size;
	u16 i;

	sscanf(buf, "%u", &object_type);
	dev_info(&data->client->dev, "show object type T%d\n", object_type);

	ret = get_object_info(data, (u8)object_type, &size, &address);
	if (ret) {
		dev_err(&data->client->dev, "fail to get object_info\n");
		return count;
	}

	for (i = 0; i < size; i++) {
		read_mem(data, address+i, 1, &val);
		dev_info(&data->client->dev, "Byte %u --> %u\n", i, val);
	}

	return count;
}

struct device *sec_touchscreen;
static u8 firmware_latest[] = {0x16, 0x10}; /* mxt224 : 0x16, mxt224E : 0x10 */
static u8 build_latest[] = {0xAB, 0xAA};

struct device *mxt224_noise_test;
/*
	botton_right, botton_left, center, top_right, top_left
*/
unsigned char test_node[5] = {12, 20, 104, 188, 196};
uint16_t mxt_reference_node[209] = { 0 };
uint16_t mxt_delta_node[209] = { 0 };
uint16_t max_ref, min_ref;

void diagnostic_chip(struct mxt224_data *data, u8 mode)
{
	int error;
	u16 t6_address = 0;
	u16 size_one;
	int ret;
	u8 value;
	u16 t37_address = 0;

	ret = get_object_info(copy_data,
		GEN_COMMANDPROCESSOR_T6, &size_one, &t6_address);

	size_one = 1;
	error = write_mem(copy_data, t6_address+5, (u8)size_one, &mode);
	/* qt602240_write_object(p_qt602240_data, */
	/* QT602240_GEN_COMMAND, QT602240_COMMAND_DIAGNOSTIC, mode); */
	if (error < 0) {
		printk(KERN_ERR"[TSP] error %s: write_object\n", __func__);
	} else {
		get_object_info(copy_data,
			DEBUG_DIAGNOSTIC_T37, &size_one, &t37_address);
		size_one = 1;
		/* printk(KERN_ERR"diagnostic_chip setting success\n"); */
		read_mem(copy_data, t37_address, (u8)size_one, &value);
		/* printk(KERN_ERR"dianostic_chip mode is %d\n",value); */
	}
}

uint8_t read_uint16_t(struct mxt224_data *data,
				uint16_t address, uint16_t *buf)
{
	uint8_t status;
	uint8_t temp[2];

	status = read_mem(data, address, 2, temp);
	*buf = ((uint16_t)temp[1]<<8) + (uint16_t)temp[0];

	return status;
}

void read_dbg_data(struct mxt224_data *data, uint8_t dbg_mode , uint8_t node, uint16_t *dbg_data)
{
	u8 read_page, read_point;
	uint8_t mode, page;
	u16 size;
	u16 diagnostic_addr = 0;

	get_object_info(copy_data,
		DEBUG_DIAGNOSTIC_T37, &size, &diagnostic_addr);

	read_page = node / 64;
	node %= 64;
	read_point = (node * 2) + 2;

	/* Page Num Clear */
	diagnostic_chip(data, MXT_CTE_MODE);
	msleep(10);

	do {
		if (read_mem(copy_data, diagnostic_addr, 1, &mode)) {
			printk(KERN_ERR"[TSP] READ_MEM_FAILED\n");
			return;
		}
	} while (mode != MXT_CTE_MODE);

	diagnostic_chip(data, dbg_mode);
	msleep(20);

	do {
		if (read_mem(copy_data, diagnostic_addr, 1, &mode)) {
			printk(KERN_ERR"[TSP] READ_MEM_FAILED\n");
			return;
		}
	} while (mode != dbg_mode);

	for (page = 1; page <= read_page; page++) {
		diagnostic_chip(data, MXT_PAGE_UP);
		msleep(20);
		do {
			if (read_mem(copy_data, diagnostic_addr + 1, 1, &mode)) {
				dev_err(&data->client->dev, "READ_MEM_FAILED\n");
				return;
			}
		} while (mode != page);
	}

	if (read_uint16_t(copy_data, diagnostic_addr + read_point, dbg_data)) {
		printk(KERN_ERR"[TSP] READ_MEM_FAILED\n");
		return;
	}
}

#define MAX_VALUE 3680
#define MIN_VALUE 13280

int read_all_data(struct mxt224_data *data, uint16_t dbg_mode)
{
	u8 read_page, read_point;
	u16 max_value = MAX_VALUE, min_value = MIN_VALUE;
	u16 object_address = 0;
	u8 data_buffer[2] = { 0 };
	u8 node = 0;
	int state = 0;
	int num = 0;
	int ret;
	u16 size;

	/* Page Num Clear */
	diagnostic_chip(data, MXT_CTE_MODE);
	msleep(30);/* msleep(20);  */

	diagnostic_chip(data, dbg_mode);
	msleep(30);/* msleep(20);  */

	ret = get_object_info(copy_data,
		DEBUG_DIAGNOSTIC_T37, &size, &object_address);
/*jerry no need to leave it */
	msleep(50); /* msleep(20);  */

	if (copy_data->family_id == 0x81) {
		max_value = max_value + 16384;
		min_value = min_value + 16384;
	}

	for (read_page = 0 ; read_page < 4; read_page++) {
		for (node = 0; node < 64; node++) {
			read_point = (node * 2) + 2;
			read_mem(copy_data, object_address+(u16)read_point, 2, data_buffer);
			mxt_reference_node[num] = ((uint16_t)data_buffer[1]<<8) + (uint16_t)data_buffer[0];

			/* MXT224E use dual tx channel when TA is connected.
			 * But the last tx channel still use single tx. so,
			 * The raw data values must be doubled.
			 */
			if (vbus_state == 1 && (num >= 198 &&  num <= 208))
				mxt_reference_node[num] =
					(((uint16_t)data_buffer[1] << 8) +
					(uint16_t)data_buffer[0]) << 1;

			if (copy_data->family_id == 0x81) {
				if ((mxt_reference_node[num] > MIN_VALUE + 16384) || (mxt_reference_node[num] < MAX_VALUE + 16384)) {
					state = 1;
					dev_err(&data->client->dev, "Mxt224-E mxt_reference_node[%3d] = %5d\n",
							num, mxt_reference_node[num]);
				/*	break; */
				}
			} else {
				if ((mxt_reference_node[num] > MIN_VALUE) || (mxt_reference_node[num] < MAX_VALUE)) {
					state = 1;
					dev_err(&data->client->dev, "Mxt224 mxt_reference_node[%3d] = %5d \n",
							num, mxt_reference_node[num]);
				/*	break; */
				}
			}

			if (data_buffer[0] != 0) {
				if (mxt_reference_node[num] > max_value)
					max_value = mxt_reference_node[num];
				if (mxt_reference_node[num] < min_value)
					min_value = mxt_reference_node[num];
			}

			num++;

			/* all node => 19 * 11 = 209 => (3page * 64) + 17 */
			if ((read_page == 3) && (node == 16))
				break;

		}
		diagnostic_chip(data, MXT_PAGE_UP);
		msleep(10);
	}

	if ((max_value - min_value) > 4000) {
		dev_err(&data->client->dev, "diff = %d, max_value = %d, min_value = %d\n",
				(max_value - min_value), max_value, min_value);
		state = 1;
	}

	max_ref = max_value;
	min_ref = min_value;

	return state;
}

int read_all_delta_data(struct mxt224_data *data, uint16_t dbg_mode)
{
	u8 read_page, read_point;
	u16 object_address = 0;
	u8 data_buffer[2] = { 0 };
	u8 node = 0;
	int state = 0;
	int num = 0;
	int ret;
	u16 size;

	/* Page Num Clear */
	diagnostic_chip(data, MXT_CTE_MODE);
	msleep(30);/* msleep(20);  */

	diagnostic_chip(data, dbg_mode);
	msleep(30);/* msleep(20);  */

	ret = get_object_info(copy_data,
		DEBUG_DIAGNOSTIC_T37, &size, &object_address);
	/*jerry no need to leave it */
	msleep(50); /* msleep(20);  */

	for (read_page = 0 ; read_page < 4; read_page++) {
		for (node = 0; node < 64; node++) {
			read_point = (node * 2) + 2;
			read_mem(copy_data, object_address+(u16)read_point, 2, data_buffer);
				mxt_delta_node[num] = ((uint16_t)data_buffer[1]<<8) + (uint16_t)data_buffer[0];

			num = num+1;

			/* all node => 19 * 11 = 209 => (3page * 64) + 17 */
			if ((read_page == 3) && (node == 16))
				break;

		}
		diagnostic_chip(data, MXT_PAGE_UP);
		msleep(10);
	}

	return state;
}

static int mxt224_check_bootloader(struct i2c_client *client,
					unsigned int state)
{
	u8 val;

recheck:
	if (i2c_master_recv(client, &val, 1) != 1) {
		dev_err(&client->dev, "%s: i2c recv failed\n", __func__);
		return -EIO;
	}

	switch (state) {
	case MXT224_WAITING_BOOTLOAD_CMD:
	case MXT224_WAITING_FRAME_DATA:
		val &= ~MXT224_BOOT_STATUS_MASK;
		break;
	case MXT224_FRAME_CRC_PASS:
		if (val == MXT224_FRAME_CRC_CHECK)
			goto recheck;
		break;
	default:
		return -EINVAL;
	}

	if (val != state) {
		dev_err(&client->dev, "Unvalid bootloader mode state\n");
		printk(KERN_ERR"[TSP] Unvalid bootloader mode state\n");
		return -EINVAL;
	}

	return 0;
}

static int mxt224_unlock_bootloader(struct i2c_client *client)
{
	u8 buf[2];

	buf[0] = MXT224_UNLOCK_CMD_LSB;
	buf[1] = MXT224_UNLOCK_CMD_MSB;

	if (i2c_master_send(client, buf, 2) != 2) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int mxt224_fw_write(struct i2c_client *client,
				const u8 *data, unsigned int frame_size)
{
	if (i2c_master_send(client, data, frame_size) != frame_size) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int mxt224_load_fw(struct device *dev, const char *fn)
{
	struct mxt224_data *data = copy_data;
	struct i2c_client *client = copy_data->client;
	const struct firmware *fw = NULL;
	unsigned int frame_size;
	unsigned int pos = 0;
	int ret;
	u16 obj_address = 0;
	u16 size_one;
	u8 value;
	unsigned int object_register;

	dev_info(&data->client->dev, "mxt224_load_fw start!!!\n");

	ret = request_firmware(&fw, fn, &client->dev);
	if (ret) {
		dev_err(&data->client->dev, "Unable to open firmware %s\n", fn);
		return ret;
	}

	/* Change to the bootloader mode */
	/* mxt224_write_object(data, MXT224_GEN_COMMAND, MXT224_COMMAND_RESET, MXT224_BOOT_VALUE); */
	object_register = 0;
	value = (u8)MXT224_BOOT_VALUE;

	ret = get_object_info(data,
		GEN_COMMANDPROCESSOR_T6, &size_one, &obj_address);
	if (ret) {
		dev_err(&data->client->dev, "[TSP] fail to get object_info\n");
		return ret;
	}
	size_one = 1;
	write_mem(data, obj_address+(u16)object_register, (u8)size_one, &value);
	msleep(MXT224_RESET_TIME);

	/* Change to slave address of bootloader */
	if (client->addr == MXT224_APP_LOW)
		client->addr = MXT224_BOOT_LOW;
	else
		client->addr = MXT224_BOOT_HIGH;

	ret = mxt224_check_bootloader(client, MXT224_WAITING_BOOTLOAD_CMD);
	if (ret)
		goto out;

	/* Unlock bootloader */
	mxt224_unlock_bootloader(client);

	while (pos < fw->size) {
		ret = mxt224_check_bootloader(client,
						MXT224_WAITING_FRAME_DATA);
		if (ret)
			goto out;

		frame_size = ((*(fw->data + pos) << 8) | *(fw->data + pos + 1));

		/* We should add 2 at frame size as the the firmware data is not
		* included the CRC bytes.
		*/
		frame_size += 2;

		/* Write one frame to device */
		/* mxt224_fw_write(client, fw->data + pos, frame_size); */
		mxt224_fw_write(client, fw->data + pos, frame_size);

		ret = mxt224_check_bootloader(client,
						MXT224_FRAME_CRC_PASS);
		if (ret)
			goto out;

		pos += frame_size;

		dev_info(&data->client->dev,
			"[TSP] Updated %d bytes / %zd bytes\n", pos, fw->size);
	}

out:
	release_firmware(fw);

	/* Change to slave address of application */
	if (client->addr == MXT224_BOOT_LOW)
		client->addr = MXT224_APP_LOW;
	else
		client->addr = MXT224_APP_HIGH;

	return ret;
}

static ssize_t set_refer0_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt224_data *data = copy_data;
	uint16_t mxt_reference = 0;

	read_dbg_data(data, MXT_REFERENCE_MODE, test_node[0], &mxt_reference);
	if (data->family_id == 0x81)
		mxt_reference -= 16384;

	return sprintf(buf, "%u\n", mxt_reference);
}

static ssize_t set_refer1_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt224_data *data = copy_data;
	uint16_t mxt_reference = 0;

	read_dbg_data(data, MXT_REFERENCE_MODE, test_node[1], &mxt_reference);
	if (data->family_id == 0x81)
		mxt_reference -= 16384;

	return sprintf(buf, "%u\n", mxt_reference);
}

static ssize_t set_refer2_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct mxt224_data *data = copy_data;
	uint16_t mxt_reference = 0;

	read_dbg_data(data, MXT_REFERENCE_MODE, test_node[2], &mxt_reference);
	if (data->family_id == 0x81)
		mxt_reference -= 16384;

	return sprintf(buf, "%u\n", mxt_reference);
}

static ssize_t set_refer3_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt224_data *data = copy_data;
	uint16_t mxt_reference = 0;

	read_dbg_data(data, MXT_REFERENCE_MODE, test_node[3], &mxt_reference);
	if (data->family_id == 0x81)
		mxt_reference -= 16384;

	return sprintf(buf, "%u\n", mxt_reference);
}

static ssize_t set_refer4_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt224_data *data = copy_data;
	uint16_t mxt_reference = 0;

	read_dbg_data(data, MXT_REFERENCE_MODE, test_node[4], &mxt_reference);
	if (data->family_id == 0x81)
		mxt_reference -= 16384;
	return sprintf(buf, "%u\n", mxt_reference);
}

static ssize_t set_delta0_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt224_data *data = dev_get_drvdata(dev);
	uint16_t mxt_delta = 0;

	read_dbg_data(data, MXT_DELTA_MODE, test_node[0], &mxt_delta);
	if (mxt_delta < 32767)
		return sprintf(buf, "%u\n", mxt_delta);
	else
		mxt_delta = 65535 - mxt_delta;

	if (mxt_delta)
		return sprintf(buf, "-%u\n", mxt_delta);
	else
		return sprintf(buf, "%u\n", mxt_delta);
}

static ssize_t set_delta1_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt224_data *data = dev_get_drvdata(dev);
	uint16_t mxt_delta = 0;

	read_dbg_data(data, MXT_DELTA_MODE, test_node[1], &mxt_delta);
	if (mxt_delta < 32767)
		return sprintf(buf, "%u\n", mxt_delta);
	else
		mxt_delta = 65535 - mxt_delta;

	if (mxt_delta)
		return sprintf(buf, "-%u\n", mxt_delta);
	else
		return sprintf(buf, "%u\n", mxt_delta);
}

static ssize_t set_delta2_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt224_data *data = dev_get_drvdata(dev);
	uint16_t mxt_delta = 0;

	read_dbg_data(data, MXT_DELTA_MODE, test_node[2], &mxt_delta);
	if (mxt_delta < 32767)
		return sprintf(buf, "%u\n", mxt_delta);
	else
		mxt_delta = 65535 - mxt_delta;

	if (mxt_delta)
		return sprintf(buf, "-%u\n", mxt_delta);
	else
		return sprintf(buf, "%u\n", mxt_delta);
}

static ssize_t set_delta3_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt224_data *data = dev_get_drvdata(dev);
	uint16_t mxt_delta = 0;

	read_dbg_data(data, MXT_DELTA_MODE, test_node[3], &mxt_delta);
	if (mxt_delta < 32767)
		return sprintf(buf, "%u\n", mxt_delta);
	else
		mxt_delta = 65535 - mxt_delta;

	if (mxt_delta)
		return sprintf(buf, "-%u\n", mxt_delta);
	else
		return sprintf(buf, "%u\n", mxt_delta);
}

static ssize_t set_delta4_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt224_data *data = dev_get_drvdata(dev);
	uint16_t mxt_delta = 0;

	read_dbg_data(data, MXT_DELTA_MODE, test_node[4], &mxt_delta);
	if (mxt_delta < 32767)
		return sprintf(buf, "%u\n", mxt_delta);
	else
		mxt_delta = 65535 - mxt_delta;

	if (mxt_delta)
		return sprintf(buf, "-%u\n", mxt_delta);
	else
		return sprintf(buf, "%u\n", mxt_delta);
}

static ssize_t set_all_refer_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt224_data *data = dev_get_drvdata(dev);
	int status = 0;

	status = read_all_data(data, MXT_REFERENCE_MODE);

	return sprintf(buf, "%u, %u, %u\n", status, max_ref, min_ref);
}

static int index_reference;

static int atoi(char *str)
{
	int result = 0;
	int count = 0;

	if (str == NULL)
		return -1;

	while (str[count] != 0 && str[count] >= '0' && str[count] <= '9') {
		result = result * 10 + str[count] - '0';
		++count;
	}

	return result;
}

ssize_t disp_all_refdata_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n",  mxt_reference_node[index_reference]);
}

ssize_t disp_all_refdata_store(struct device *dev, struct device_attribute *attr,
								   const char *buf, size_t size)
{
	index_reference = atoi((char *)buf);
	return size;
}

static ssize_t set_all_delta_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt224_data *data = dev_get_drvdata(dev);
	int status = 0;

	status = read_all_delta_data(data, MXT_DELTA_MODE);

	return sprintf(buf, "%u\n", status);
}

static int index_delta;

ssize_t disp_all_deltadata_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (mxt_delta_node[index_delta] < 32767)
		return sprintf(buf, "%u\n", mxt_delta_node[index_delta]);
	else
		mxt_delta_node[index_delta] = 65535 - mxt_delta_node[index_delta];

	return sprintf(buf, "-%u\n", mxt_delta_node[index_delta]);
}

ssize_t disp_all_deltadata_store(struct device *dev, struct device_attribute *attr,
								   const char *buf, size_t size)
{
	index_delta = atoi((char *)buf);
	return size;
}

static ssize_t set_tsp_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "ATMEL mxT224E\n");
}

static ssize_t set_tsp_channel_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "19,11\n");
}

static ssize_t set_module_off_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt224_data *data = copy_data;
	int count;

	mutex_lock(&data->lock);
	if (data->enabled)
		data->enabled = 0;
	mutex_unlock(&data->lock);

	touch_is_pressed = 0;

	disable_irq(data->client->irq);
	mxt224_internal_suspend(data);

	count = sprintf(buf, "tspoff\n");

	return count;
}

static ssize_t set_module_on_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt224_data *data = copy_data;
	int count;

	mxt224_internal_resume(data);
	enable_irq(data->client->irq);

	mutex_lock(&data->lock);
	if (!data->enabled)
		data->enabled = 1;
	mutex_unlock(&data->lock);

	dev_info(&data->client->dev, "vbus_state = %d\n", (int)vbus_state);
	mxt224_ta_probe(vbus_state);

	calibrate_chip();

	count = sprintf(buf, "tspon\n");

	return count;
}

static ssize_t set_threshold_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", threshold);
}

static ssize_t set_mxt_firm_update_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct mxt224_data *data = dev_get_drvdata(dev);
	int error = 0;

	dev_info(&data->client->dev, "set_mxt_update_show start!!\n");
	if (*buf != 'S' && *buf != 'F') {
		dev_err(&data->client->dev, "Invalid values\n");
		return -EINVAL;
	}

	disable_irq(data->client->irq);
	firm_status_data = 1;
	if (data->family_id == 0x80) {	/*  : MXT-224 */
		if (*buf != 'F' && data->tsp_version >= firmware_latest[0] && data->tsp_build >= build_latest[0]) {
			dev_info(&data->client->dev, "mxt224 has latest firmware\n");
			firm_status_data = 2;
			enable_irq(data->client->irq);
			return size;
		}
		dev_info(&data->client->dev, "mxt224_fm_update\n");
		error = mxt224_load_fw(dev, MXT224_FW_NAME);
	} else if (data->family_id == 0x81)  {	/* tsp_family_id - 0x81 : MXT-224E */
		if (*buf != 'F' && data->tsp_version >= firmware_latest[1] && data->tsp_build >= build_latest[1]) {
			dev_info(&data->client->dev, "mxt224E has latest firmware\n");
			firm_status_data = 2;
			enable_irq(data->client->irq);
			return size;
		}
		dev_info(&data->client->dev, "mxt224E_fm_update\n");
		error = mxt224_load_fw(dev, MXT224_ECHO_FW_NAME);
	}

	if (error) {
		dev_err(&data->client->dev,
			"The firmware update failed(%d)\n", error);
		firm_status_data = 3;
		return error;
	} else {
		dev_info(dev, "The firmware update succeeded\n");
		firm_status_data = 2;

		/* Wait for reset */
		msleep(MXT224_FWRESET_TIME);

		mxt224_init_touch_driver(data);
		/* mxt224_initialize(data); */
	}

	enable_irq(data->client->irq);
	error = mxt224_backup(data);
	if (error) {
		dev_err(&data->client->dev, "mxt224_backup fail!!!\n");
		return error;
	}

	/* reset the touch IC. */
	error = mxt224_reset(data);
	if (error) {
		dev_err(&data->client->dev, "mxt224_reset fail!!!\n");
		return error;
	}

	msleep(MXT224_RESET_TIME);
	return size;
}

static ssize_t set_mxt_firm_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt224_data *data = dev_get_drvdata(dev);
	int count;

	dev_info(&data->client->dev,
		"Enter firmware_status_show by Factory command\n");

	if (firm_status_data == 1)
		count = sprintf(buf, "DOWNLOADING\n");
	else if (firm_status_data == 2)
		count = sprintf(buf, "PASS\n");
	else if (firm_status_data == 3)
		count = sprintf(buf, "FAIL\n");
	else
		count = sprintf(buf, "PASS\n");

	return count;
}

static ssize_t key_threshold_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", threshold);
}

static ssize_t key_threshold_store(struct device *dev, struct device_attribute *attr,
								   const char *buf, size_t size)
{
	struct mxt224_data *data = dev_get_drvdata(dev);
	/*TO DO IT*/
	unsigned int object_register = 7;
	u8 value;
	u8 val;
	int ret;
	u16 address = 0;
	u16 size_one;
	int num;

	if (sscanf(buf, "%d", &num) == 1) {
		threshold = num;
		dev_info(&data->client->dev, "threshold value %d\n", threshold);
		ret = get_object_info(copy_data,
			TOUCH_MULTITOUCHSCREEN_T9, &size_one, &address);
		size_one = 1;
		value = (u8)threshold;
		write_mem(copy_data, address+(u16)object_register, size_one, &value);
		read_mem(copy_data, address+(u16)object_register, (u8)size_one, &val);
		dev_info(&data->client->dev, "T9 Byte%d is %d\n", object_register, val);
	}
	return size;
}

static ssize_t set_mxt_firm_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt224_data *data = dev_get_drvdata(dev);
	u8 fw_latest_version = 0;

	if (data->family_id == 0x80)
		fw_latest_version = firmware_latest[0];
	else if (data->family_id == 0x81)
		fw_latest_version = firmware_latest[1];

	pr_info("Atmel Last firmware version is %d\n", fw_latest_version);

	return sprintf(buf, "%#02x\n", fw_latest_version);
}

static ssize_t set_mxt_firm_version_read_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt224_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%#02x\n", data->tsp_version);
}

static ssize_t tsp_config_version_show(struct device *dev, struct device_attribute *attr,
				       char *buf)
{
	struct mxt224_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", data->config_fw_version);
}

static DEVICE_ATTR(set_refer0, S_IRUGO, set_refer0_mode_show, NULL);
static DEVICE_ATTR(set_delta0, S_IRUGO, set_delta0_mode_show, NULL);
static DEVICE_ATTR(set_refer1, S_IRUGO, set_refer1_mode_show, NULL);
static DEVICE_ATTR(set_delta1, S_IRUGO, set_delta1_mode_show, NULL);
static DEVICE_ATTR(set_refer2, S_IRUGO, set_refer2_mode_show, NULL);
static DEVICE_ATTR(set_delta2, S_IRUGO, set_delta2_mode_show, NULL);
static DEVICE_ATTR(set_refer3, S_IRUGO, set_refer3_mode_show, NULL);
static DEVICE_ATTR(set_delta3, S_IRUGO, set_delta3_mode_show, NULL);
static DEVICE_ATTR(set_refer4, S_IRUGO, set_refer4_mode_show, NULL);
static DEVICE_ATTR(set_delta4, S_IRUGO, set_delta4_mode_show, NULL);
static DEVICE_ATTR(set_all_refer, S_IRUGO | S_IWUSR | S_IWGRP, set_all_refer_mode_show, NULL);
static DEVICE_ATTR(disp_all_refdata, S_IRUGO | S_IWUSR | S_IWGRP,
			disp_all_refdata_show, disp_all_refdata_store);
static DEVICE_ATTR(set_all_delta, S_IRUGO | S_IWUSR | S_IWGRP, set_all_delta_mode_show, NULL);
static DEVICE_ATTR(disp_all_deltadata, S_IRUGO | S_IWUSR | S_IWGRP,
			disp_all_deltadata_show, disp_all_deltadata_store);
static DEVICE_ATTR(set_threshold, S_IRUGO | S_IWUSR | S_IWGRP, set_threshold_mode_show, NULL);
static DEVICE_ATTR(set_tsp_name, S_IRUGO , set_tsp_name_show, NULL);
static DEVICE_ATTR(set_tsp_channel, S_IRUGO , set_tsp_channel_show, NULL);
static DEVICE_ATTR(set_module_off, S_IRUGO | S_IWUSR | S_IWGRP, set_module_off_show, NULL);
static DEVICE_ATTR(set_module_on, S_IRUGO | S_IWUSR | S_IWGRP, set_module_on_show, NULL);

static DEVICE_ATTR(tsp_firm_update, S_IWUSR | S_IWGRP, NULL, set_mxt_firm_update_store);		/* firmware update */
static DEVICE_ATTR(tsp_firm_update_status, S_IRUGO, set_mxt_firm_status_show, NULL);	/* firmware update status return */
static DEVICE_ATTR(tsp_threshold, S_IRUGO | S_IWUSR | S_IWGRP, key_threshold_show, key_threshold_store);	/* touch threshold return, store */
static DEVICE_ATTR(tsp_firm_version_phone, S_IRUGO, set_mxt_firm_version_show, NULL);/* PHONE*/	/* firmware version resturn in phone driver version */
static DEVICE_ATTR(tsp_firm_version_panel, S_IRUGO, set_mxt_firm_version_read_show, NULL);/*PART*/	/* firmware version resturn in TSP panel version */
static DEVICE_ATTR(tsp_config_version, S_IRUGO, tsp_config_version_show, NULL);
static DEVICE_ATTR(object_show, S_IWUSR | S_IWGRP, NULL, mxt224_object_show);
static DEVICE_ATTR(object_write, S_IWUSR | S_IWGRP, NULL, mxt224_object_setting);
static DEVICE_ATTR(dbg_switch, S_IWUSR | S_IWGRP, NULL, mxt224_debug_setting);


static struct attribute *mxt224_attrs[] = {
	&dev_attr_object_show.attr,
	&dev_attr_object_write.attr,
	&dev_attr_dbg_switch.attr,
	NULL
};

static const struct attribute_group mxt224_attr_group = {
	.attrs = mxt224_attrs,
};

/* mxT224E kobjects */
static ssize_t mxt224e_sweep2wake_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	sprintf(buf, "status: %s\n", sweep2wake ? "on" : "off");
	sprintf(buf, "%sthreshold_x: %d\n", buf, x_threshold);
	sprintf(buf, "%sthreshold_y: %d\n", buf, y_threshold);
	#if CONFIG_HAS_WAKELOCK
	sprintf(buf, "%swakelock_ena: %d\n", buf, wake_lock_active(&s2w_wakelock));
	#endif

	return strlen(buf);
}

static ssize_t mxt224e_sweep2wake_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int threshold_tmp;

	if (!strncmp(buf, "on", 2)) {
		/* In case that we receive cmd in sleep */
		if (is_suspend && !sweep2wake) {
			mxt224e_tsp_on();
		}
		sweep2wake = true;

		#if CONFIG_HAS_WAKELOCK
		wake_lock(&s2w_wakelock);
		#endif

		pr_err("[TSP] Sweep2Wake On\n");

		return count;
	}

	if (!strncmp(buf, "off", 3)) {
		if (is_suspend && !sweep2wake) {
			mxt224e_tsp_off();
		}
		sweep2wake = false;

		#if CONFIG_HAS_WAKELOCK
		wake_unlock(&s2w_wakelock);
		#endif

		pr_err("[TSP] Sweep2Wake Off\n");

		return count;
	}

	if (!strncmp(&buf[0], "threshold_x=", 12)) {
		ret = sscanf(&buf[12], "%d", &threshold_tmp);

		if ((!ret) || (threshold_tmp > 480)) {
			pr_err("[TSP] invalid input\n");
			return -EINVAL;
		}

		x_threshold = threshold_tmp;
		
		return count;
	}

	if (!strncmp(&buf[0], "threshold_y=", 12)) {
		ret = sscanf(&buf[12], "%d", &threshold_tmp);

		if ((!ret) || (threshold_tmp > 800)) {
			pr_err("[TSP] invalid input\n");
			return -EINVAL;
		}

		y_threshold = threshold_tmp;
		
		return count;
	}
		
	return count;
}

static struct kobj_attribute mxt224e_sweep2wake_interface = __ATTR(sweep2wake, 0644, mxt224e_sweep2wake_show, mxt224e_sweep2wake_store);

static ssize_t mxt224e_config_t48_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	u8 mbuf;
	u16 addr = 0;
	u16 size;
	u32 i;

	get_object_info(copy_data, PROCG_NOISESUPPRESSION_T48, &size, &addr);

	sprintf(buf, "[PROCG_NOISESUPPRESSION_T48]\n\n");
	sprintf(buf, "%s+------------------+--------+\n", buf);
	sprintf(buf, "%s|Addr              |Value   |\n", buf);

	for (i = 0; i <= T48_MAXADDR; i++) {
		read_mem(copy_data, addr + i, 1, &mbuf);

		sprintf(buf, "%s+------------------+--------+\n", buf);
		sprintf(buf, "%s|%-18d|%-8d|\n", buf, i, mbuf);
	}
	sprintf(buf, "%s+------------------+--------+\n", buf);
	
	return strlen(buf);
}

static ssize_t mxt224e_config_t48_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	u8 val;
	u16 addr = 0;
	u16 addr_u;
	u16 size;

	get_object_info(copy_data, PROCG_NOISESUPPRESSION_T48, &size, &addr);

	ret = sscanf(buf, "%d %d", (int*)&addr_u, (int*)&val);

	if (!ret) {
		pr_err("[TSP] invalid inputs\n");
		return -EINVAL;
	}

	write_mem(copy_data, addr + addr_u, 1, &val);

	pr_err("[TSP] T48 [%2d] [%d]\n", addr_u, val);
		
	return count;
}

static struct kobj_attribute mxt224e_config_t48_interface = __ATTR(config_t48, 0644, mxt224e_config_t48_show, mxt224e_config_t48_store);

static ssize_t mxt224e_config_t46_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	u8 mbuf;
	u16 addr = 0;
	u16 size;
	u32 i;

	get_object_info(copy_data, SPT_CTECONFIG_T46, &size, &addr);

	sprintf(buf, "[SPT_CTECONFIG_T46]\n\n");
	sprintf(buf, "%s+------------------+--------+\n", buf);
	sprintf(buf, "%s|Addr              |Value   |\n", buf);

	for (i = 0; i <= T46_MAXADDR; i++) {
		read_mem(copy_data, addr + i, 1, &mbuf);

		sprintf(buf, "%s+------------------+--------+\n", buf);
		sprintf(buf, "%s|%-18d|%-8d|\n", buf, i, mbuf);
	}
	sprintf(buf, "%s+------------------+--------+\n", buf);
	
	return strlen(buf);
}

static ssize_t mxt224e_config_t46_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	u8 val;
	u16 addr = 0;
	u16 addr_u;
	u16 size;

	get_object_info(copy_data, SPT_CTECONFIG_T46, &size, &addr);

	ret = sscanf(buf, "%d %d", (int*)&addr_u, (int*)&val);

	if (!ret) {
		pr_err("[TSP] invalid inputs\n");
		return -EINVAL;
	}

	write_mem(copy_data, addr + addr_u, 1, &val);

	pr_err("[TSP] T46 [%2d] [%d]\n", addr_u, val);
		
	return count;
}

static struct kobj_attribute mxt224e_config_t46_interface = __ATTR(config_t46, 0644, mxt224e_config_t46_show, mxt224e_config_t46_store);

static ssize_t mxt224e_config_t38_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	u8 mbuf;
	u16 addr = 0;
	u16 size;
	u32 i;

	get_object_info(copy_data, SPT_USERDATA_T38, &size, &addr);

	sprintf(buf, "[SPT_USERDATA_T38]\n\n");
	sprintf(buf, "%s+------------------+--------+\n", buf);
	sprintf(buf, "%s|Addr              |Value   |\n", buf);

	for (i = 0; i <= T38_MAXADDR; i++) {
		read_mem(copy_data, addr + i, 1, &mbuf);

		sprintf(buf, "%s+------------------+--------+\n", buf);
		sprintf(buf, "%s|%-18d|%-8d|\n", buf, i, mbuf);
	}
	sprintf(buf, "%s+------------------+--------+\n", buf);
	
	return strlen(buf);
}

static ssize_t mxt224e_config_t38_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	u8 val;
	u16 addr = 0;
	u16 addr_u;
	u16 size;

	get_object_info(copy_data, SPT_USERDATA_T38, &size, &addr);

	ret = sscanf(buf, "%d %d", (int*)&addr_u, (int*)&val);

	if (!ret) {
		pr_err("[TSP] invalid inputs\n");
		return -EINVAL;
	}

	write_mem(copy_data, addr + addr_u, 1, &val);

	pr_err("[TSP] T38 [%2d] [%d]\n", addr_u, val);
		
	return count;
}

static struct kobj_attribute mxt224e_config_t38_interface = __ATTR(config_t38, 0644, mxt224e_config_t38_show, mxt224e_config_t38_store);

static ssize_t mxt224e_config_t19_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	u8 mbuf;
	u16 addr = 0;
	u16 size;
	u32 i;

	get_object_info(copy_data, SPT_GPIOPWM_T19, &size, &addr);

	sprintf(buf, "[SPT_GPIOPWM_T19]\n\n");
	sprintf(buf, "%s+------------------+--------+\n", buf);
	sprintf(buf, "%s|Addr              |Value   |\n", buf);

	for (i = 0; i <= T19_MAXADDR; i++) {
		read_mem(copy_data, addr + i, 1, &mbuf);

		sprintf(buf, "%s+------------------+--------+\n", buf);
		sprintf(buf, "%s|%-18d|%-8d|\n", buf, i, mbuf);
	}
	sprintf(buf, "%s+------------------+--------+\n", buf);
	
	return strlen(buf);
}

static ssize_t mxt224e_config_t19_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	u8 val;
	u16 addr = 0;
	u16 addr_u;
	u16 size;

	get_object_info(copy_data, SPT_GPIOPWM_T19, &size, &addr);

	ret = sscanf(buf, "%d %d", (int*)&addr_u, (int*)&val);

	if (!ret) {
		pr_err("[TSP] invalid inputs\n");
		return -EINVAL;
	}

	write_mem(copy_data, addr + addr_u, 1, &val);

	pr_err("[TSP] T19 [%2d] [%d]\n", addr_u, val);
		
	return count;
}

static struct kobj_attribute mxt224e_config_t19_interface = __ATTR(config_t19, 0644, mxt224e_config_t19_show, mxt224e_config_t19_store);

static ssize_t mxt224e_config_t9_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	u8 mbuf;
	u16 addr = 0;
	u16 size;
	u32 i;

	get_object_info(copy_data, TOUCH_MULTITOUCHSCREEN_T9, &size, &addr);

	sprintf(buf, "[TOUCH_MULTITOUCHSCREEN_T9]\n\n");
	sprintf(buf, "%s+------------------+--------+\n", buf);
	sprintf(buf, "%s|Addr              |Value   |\n", buf);

	for (i = 0; i <= T9_MAXADDR; i++) {
		read_mem(copy_data, addr + i, 1, &mbuf);

		sprintf(buf, "%s+------------------+--------+\n", buf);
		sprintf(buf, "%s|%-18d|%-8d|\n", buf, i, mbuf);
	}
	sprintf(buf, "%s+------------------+--------+\n", buf);
	
	return strlen(buf);
}

static ssize_t mxt224e_config_t9_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	u8 val;
	u16 addr = 0;
	u16 addr_u;
	u16 size;

	get_object_info(copy_data, TOUCH_MULTITOUCHSCREEN_T9, &size, &addr);

	ret = sscanf(buf, "%d %d", (int*)&addr_u, (int*)&val);

	if (!ret) {
		pr_err("[TSP] invalid inputs\n");
		return -EINVAL;
	}

	write_mem(copy_data, addr + addr_u, 1, &val);

	pr_err("[TSP] T9 [%2d] [%d]\n", addr_u, val);
		
	return count;
}

static struct kobj_attribute mxt224e_config_t9_interface = __ATTR(config_t9, 0644, mxt224e_config_t9_show, mxt224e_config_t9_store);

static ssize_t mxt224e_config_t8_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	u8 mbuf;
	u16 addr = 0;
	u16 size;
	u32 i;

	get_object_info(copy_data, GEN_ACQUISITIONCONFIG_T8, &size, &addr);

	sprintf(buf, "[GEN_ACQUISITIONCONFIG_T8]\n\n");
	sprintf(buf, "%s+------------------+--------+\n", buf);
	sprintf(buf, "%s|Addr              |Value   |\n", buf);

	for (i = 0; i <= T8_MAXADDR; i++) {
		read_mem(copy_data, addr + i, 1, &mbuf);

		sprintf(buf, "%s+------------------+--------+\n", buf);
		sprintf(buf, "%s|%-18d|%-8d|\n", buf, i, mbuf);
	}
	sprintf(buf, "%s+------------------+--------+\n", buf);
	
	return strlen(buf);
}

static ssize_t mxt224e_config_t8_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	u8 val;
	u16 addr = 0;
	u16 addr_u;
	u16 size;

	get_object_info(copy_data, GEN_ACQUISITIONCONFIG_T8, &size, &addr);

	ret = sscanf(buf, "%d %d", (int*)&addr_u, (int*)&val);

	if (!ret) {
		pr_err("[TSP] invalid inputs\n");
		return -EINVAL;
	}

	write_mem(copy_data, addr + addr_u, 1, &val);

	pr_err("[TSP] T8 [%2d] [%d]\n", addr_u, val);
		
	return count;
}

static struct kobj_attribute mxt224e_config_t8_interface = __ATTR(config_t8, 0644, mxt224e_config_t8_show, mxt224e_config_t8_store);

static ssize_t mxt224e_config_t7_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	u8 mbuf;
	u16 addr = 0;
	u16 size;
	u32 i;

	get_object_info(copy_data, GEN_POWERCONFIG_T7, &size, &addr);

	sprintf(buf, "[GEN_POWERCONFIG_T7]\n\n");
	sprintf(buf, "%s+------------------+--------+\n", buf);
	sprintf(buf, "%s|Addr              |Value   |\n", buf);

	for (i = 0; i <= T7_MAXADDR; i++) {
		read_mem(copy_data, addr + i, 1, &mbuf);

		sprintf(buf, "%s+------------------+--------+\n", buf);
		sprintf(buf, "%s|%-18d|%-8d|\n", buf, i, mbuf);
	}
	sprintf(buf, "%s+------------------+--------+\n", buf);
	
	return strlen(buf);
}

static ssize_t mxt224e_config_t7_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	u8 val;
	u16 addr = 0;
	u16 addr_u;
	u16 size;

	get_object_info(copy_data, GEN_POWERCONFIG_T7, &size, &addr);

	ret = sscanf(buf, "%d %d", (int*)&addr_u, (int*)&val);

	if (!ret) {
		pr_err("[TSP] invalid inputs\n");
		return -EINVAL;
	}

	write_mem(copy_data, addr + addr_u, 1, &val);

	pr_err("[TSP] T7 [%2d] [%d]\n", addr_u, val);
		
	return count;
}

static struct kobj_attribute mxt224e_config_t7_interface = __ATTR(config_t7, 0644, mxt224e_config_t7_show, mxt224e_config_t7_store);

#ifdef TOUCH_BOOSTER
static void mxt224e_touchboost_clear(void)
{
	prcmu_qos_update_requirement(PRCMU_QOS_APE_OPP, (char *)copy_data->client->name, PRCMU_QOS_DEFAULT_VALUE);
	prcmu_qos_update_requirement(PRCMU_QOS_DDR_OPP, (char *)copy_data->client->name, PRCMU_QOS_DEFAULT_VALUE);
	prcmu_qos_update_requirement(PRCMU_QOS_ARM_KHZ, (char *)copy_data->client->name, PRCMU_QOS_DEFAULT_VALUE);
}

static ssize_t mxt224e_touchboost_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", touchboost ? "on" : "off");
}

static ssize_t mxt224e_touchboost_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int tbuf;

	if (!strncmp(buf, "on", 2)) {
		touchboost = true;
	}

	if (!strncmp(buf, "off", 3)) {
		touchboost = false;
	}

	/* In case that user pass 1/0 */
	ret = sscanf(buf, "%d", &tbuf);

	if (ret) {
		if ((tbuf == 0) || (tbuf == 1))
			touchboost = tbuf;
	}

	if (!touchboost)
		mxt224e_touchboost_clear();

	pr_err("[TSP] TouchBoost %s\n", touchboost ? "enable" : "disable");
		
	return count;
}

static struct kobj_attribute mxt224e_touchboost_interface = __ATTR(touchboost, 0644, mxt224e_touchboost_show, mxt224e_touchboost_store);

static ssize_t mxt224e_touchboost_freq_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", touchboost_freq);
}

static ssize_t mxt224e_touchboost_freq_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int tbuf;

	ret = sscanf(buf, "%d", &tbuf);

	if (!ret)
		return -EINVAL;

	if (tbuf != 0 && 
	    tbuf != 200000 && 
	    tbuf != 400000 && 
	    tbuf != 800000 && 
	    tbuf != 1000000) 
	{
		pr_err("[TSP] passed an invalid cpufreq\n");
		return -EINVAL;
	}

	touchboost_freq = tbuf;

	mxt224e_touchboost_clear();

	pr_err("[TSP] TouchBoost cpufreq: %d\n", touchboost_freq);
		
	return count;
}

static struct kobj_attribute mxt224e_touchboost_freq_interface = __ATTR(touchboost_freq, 0644, mxt224e_touchboost_freq_show, mxt224e_touchboost_freq_store);

static ssize_t mxt224e_touchboost_ape_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", touchboost_ape);
}

static ssize_t mxt224e_touchboost_ape_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int tbuf;

	ret = sscanf(buf, "%d", &tbuf);

	if (!ret)
		return -EINVAL;

	if (tbuf != 0 && tbuf != 1) 
	{
		pr_err("[TSP] passed an invalid cpufreq\n");
		return -EINVAL;
	}

	touchboost_ape = tbuf;

	mxt224e_touchboost_clear();
		
	return count;
}

static struct kobj_attribute mxt224e_touchboost_ape_interface = __ATTR(touchboost_ape, 0644, mxt224e_touchboost_ape_show, mxt224e_touchboost_ape_store);

static ssize_t mxt224e_touchboost_ddr_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", touchboost_ddr);
}

static ssize_t mxt224e_touchboost_ddr_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int tbuf;

	ret = sscanf(buf, "%d", &tbuf);

	if (!ret)
		return -EINVAL;

	if (tbuf != 0 && tbuf != 1) 
	{
		pr_err("[TSP] passed an invalid cpufreq\n");
		return -EINVAL;
	}

	touchboost_ddr = tbuf;

	mxt224e_touchboost_clear();
		
	return count;
}

static struct kobj_attribute mxt224e_touchboost_ddr_interface = __ATTR(touchboost_ddr, 0644, mxt224e_touchboost_ddr_show, mxt224e_touchboost_ddr_store);
#endif

static ssize_t mxt224e_tsp_calibrate_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{

	if (is_suspend)
		return count;

	mxt224_ta_probe(vbus_state);

	calibrate_chip();

	pr_err("[TSP] Calibrate now!\n");
		
	return count;
}

static struct kobj_attribute mxt224e_tsp_calibrate_interface = __ATTR(calibrate_tsp, 0644, NULL, mxt224e_tsp_calibrate_store);

static ssize_t mxt224e_movefilter_t48_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	u8 mbuf;
	u16 addr = 0;
	u16 size;

	get_object_info(copy_data, PROCG_NOISESUPPRESSION_T48, &size, &addr);

	read_mem(copy_data, addr + 39, 1, &mbuf);

	sprintf(buf,   "status: %s\n", movefilter_t48_req ? "on" : "off");
	sprintf(buf, "%smem: %d\n", buf, mbuf);
	sprintf(buf, "%sval: %d\n", buf, movefilter_t48_val);

	return strlen(buf);
}

static ssize_t mxt224e_movefilter_t48_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	u8  val;
	u16 addr = 0;
	u16 size;

	/* Fetch config data */
	get_object_info(copy_data, PROCG_NOISESUPPRESSION_T48, &size, &addr);

	if (!strncmp(buf, "on", 2)) {
		movefilter_t48_req = true;
		
		pr_info("[TSP] movefilter_t48 %s\n", movefilter_t48_req ? "on" : "off");

		if (!is_suspend) {
			write_mem(copy_data, addr + 39, 1, &movefilter_t48_val);
		}

		return count;
	}

	if (!strncmp(buf, "off", 3)) {
		movefilter_t48_req = false;

		pr_info("[TSP] movefilter_t48 %s\n", movefilter_t48_req ? "on" : "off");

		if (!is_suspend) {
			val = 81;
			write_mem(copy_data, addr + 39, 1, &val);
		}

		return count;
	}

	if (!strncmp(&buf[0], "val=", 4)) {
		ret = sscanf(&buf[4], "%d", (int *)&val);

		if (!ret) {
			pr_err("[TSP] invalid inputs\n");

			return -EINVAL;
		}

		movefilter_t48_val = val;

		if (movefilter_t48_req) {
			write_mem(copy_data, addr + 39, 1, &movefilter_t48_val);
		}

		return count;
	}

	pr_err("[TSP] unknown command\n");
		
	return count;
}

static struct kobj_attribute mxt224e_movefilter_t48_interface = __ATTR(movefilter_t48, 0644, mxt224e_movefilter_t48_show, mxt224e_movefilter_t48_store);

static ssize_t mxt224e_numtouch_t48_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	u8 mbuf;
	u16 addr = 0;
	u16 size;

	get_object_info(copy_data, PROCG_NOISESUPPRESSION_T48, &size, &addr);

	read_mem(copy_data, addr + 40, 1, &mbuf);

	sprintf(buf,   "status: %s\n", numtouch_t48_req ? "on" : "off");
	sprintf(buf, "%smem: %d\n", buf, mbuf);
	sprintf(buf, "%sval: %d\n", buf, numtouch_t48_val);

	return strlen(buf);
}

static ssize_t mxt224e_numtouch_t48_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	u8  val;
	u16 addr = 0;
	u16 size;

	/* Fetch config data */
	get_object_info(copy_data, PROCG_NOISESUPPRESSION_T48, &size, &addr);

	if (!strncmp(buf, "on", 2)) {
		numtouch_t48_req = true;
		
		pr_info("[TSP] numtouch_t48 %s\n", numtouch_t48_req ? "on" : "off");

		if (!is_suspend) {
			write_mem(copy_data, addr + 40, 1, &numtouch_t48_val);
		}

		return count;
	}

	if (!strncmp(buf, "off", 3)) {
		numtouch_t48_req = false;

		pr_info("[TSP] numtouch_t48 %s\n", numtouch_t48_req ? "on" : "off");

		if (!is_suspend) {
			val = 10;
			write_mem(copy_data, addr + 40, 1, &val);
		}

		return count;
	}

	if (!strncmp(&buf[0], "val=", 4)) {
		ret = sscanf(&buf[4], "%d", (int *)&val);

		if (!ret) {
			pr_err("[TSP] invalid inputs\n");

			return -EINVAL;
		}

		numtouch_t48_val = val;

		if (numtouch_t48_req) {
			write_mem(copy_data, addr + 40, 1, &numtouch_t48_val);
		}

		return count;
	}

	pr_err("[TSP] unknown command\n");
		
	return count;
}

static struct kobj_attribute mxt224e_numtouch_t48_interface = __ATTR(numtouch_t48, 0644, mxt224e_numtouch_t48_show, mxt224e_numtouch_t48_store);

static ssize_t mxt224e_threshold_t48_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	u8 mbuf;
	u16 addr = 0;
	u16 size;

	get_object_info(copy_data, PROCG_NOISESUPPRESSION_T48, &size, &addr);

	read_mem(copy_data, addr + 35, 1, &mbuf);

	sprintf(buf,   "status: %s\n", threshold_t48_req ? "on" : "off");
	sprintf(buf, "%smem: %d\n", buf, mbuf);
	sprintf(buf, "%sval: %d\n", buf, threshold_t48_val);

	return strlen(buf);
}

static ssize_t mxt224e_threshold_t48_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	u8  val;
	u16 addr = 0;
	u16 size;

	/* Fetch config data */
	get_object_info(copy_data, PROCG_NOISESUPPRESSION_T48, &size, &addr);

	if (!strncmp(buf, "on", 2)) {
		threshold_t48_req = true;
		
		pr_info("[TSP] threshold_t48 %s\n", threshold_t48_req ? "on" : "off");

		if (!is_suspend) {
			write_mem(copy_data, addr + 35, 1, &threshold_t48_val);
		}

		return count;
	}

	if (!strncmp(buf, "off", 3)) {
		threshold_t48_req = false;

		pr_info("[TSP] threshold_t48 %s\n", threshold_t48_req ? "on" : "off");

		if (!is_suspend) {
			val = 17;

			write_mem(copy_data, addr + 35, 1, &val);
		}

		return count;
	}

	if (!strncmp(&buf[0], "val=", 4)) {
		ret = sscanf(&buf[4], "%d", (int *)&val);

		if (!ret) {
			pr_err("[TSP] invalid inputs\n");

			return -EINVAL;
		}

		threshold_t48_val = val;

		if (threshold_t48_req) {
			write_mem(copy_data, addr + 35, 1, &threshold_t48_val);
		}

		return count;
	}

	pr_err("[TSP] unknown command\n");
		
	return count;
}

static struct kobj_attribute mxt224e_threshold_t48_interface = __ATTR(threshold_t48, 0644, mxt224e_threshold_t48_show, mxt224e_threshold_t48_store);

static ssize_t mxt224e_parameter1_t48_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	sprintf(buf,   "status: %s\n", parameter1_t48_req ? "on" : "off");
	sprintf(buf, "%saddr: %d\n", buf, parameter1_t48_addr);
	sprintf(buf, "%sval: %d\n", buf, parameter1_t48_val);

	return strlen(buf);
}

static ssize_t mxt224e_parameter1_t48_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	u8  val;
	u16 addr = 0;
	u16 size;

	/* Fetch config data */
	get_object_info(copy_data, PROCG_NOISESUPPRESSION_T48, &size, &addr);

	if (!strncmp(buf, "on", 2)) {
		parameter1_t48_req = true;
		
		pr_info("[TSP] parameter1_t48 %s\n", parameter1_t48_req ? "on" : "off");

		if (!is_suspend) {
			write_mem(copy_data, addr + parameter1_t48_addr, 1, &parameter1_t48_val);
		}

		return count;
	}

	if (!strncmp(buf, "off", 3)) {
		parameter1_t48_req = false;

		pr_info("[TSP] parameter1_t48 %s\n", parameter1_t48_req ? "on" : "off");

		return count;
	}

	if (!strncmp(&buf[0], "val=", 4)) {
		ret = sscanf(&buf[4], "%d", (int *)&val);

		if (!ret) {
			pr_err("[TSP] invalid inputs\n");

			return -EINVAL;
		}

		parameter1_t48_val = val;

		if (parameter1_t48_req) {
			write_mem(copy_data, addr + parameter1_t48_addr, 1, &parameter1_t48_val);
		}

		return count;
	}

	if (!strncmp(&buf[0], "addr=", 5)) {
		ret = sscanf(&buf[5], "%d", (int *)&val);

		if (!ret) {
			pr_err("[TSP] invalid inputs\n");

			return -EINVAL;
		}

		parameter1_t48_addr = val;

		return count;
	}

	pr_err("[TSP] unknown command\n");
		
	return count;
}

static struct kobj_attribute mxt224e_parameter1_t48_interface = __ATTR(parameter1_t48, 0644, mxt224e_parameter1_t48_show, mxt224e_parameter1_t48_store);

static ssize_t mxt224e_parameter2_t48_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	sprintf(buf,   "status: %s\n", parameter2_t48_req ? "on" : "off");
	sprintf(buf, "%saddr: %d\n", buf, parameter2_t48_addr);
	sprintf(buf, "%sval: %d\n", buf, parameter2_t48_val);

	return strlen(buf);
}

static ssize_t mxt224e_parameter2_t48_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	u8  val;
	u16 addr = 0;
	u16 size;

	/* Fetch config data */
	get_object_info(copy_data, PROCG_NOISESUPPRESSION_T48, &size, &addr);

	if (!strncmp(buf, "on", 2)) {
		parameter2_t48_req = true;
		
		pr_info("[TSP] parameter2_t48 %s\n", parameter2_t48_req ? "on" : "off");

		if (!is_suspend) {
			write_mem(copy_data, addr + parameter2_t48_addr, 1, &parameter2_t48_val);
		}

		return count;
	}

	if (!strncmp(buf, "off", 3)) {
		parameter2_t48_req = false;

		pr_info("[TSP] parameter2_t48 %s\n", parameter2_t48_req ? "on" : "off");

		return count;
	}

	if (!strncmp(&buf[0], "val=", 4)) {
		ret = sscanf(&buf[4], "%d", (int *)&val);

		if (!ret) {
			pr_err("[TSP] invalid inputs\n");

			return -EINVAL;
		}

		parameter2_t48_val = val;

		if (parameter2_t48_req) {
			write_mem(copy_data, addr + parameter2_t48_addr, 1, &parameter2_t48_val);
		}

		return count;
	}

	if (!strncmp(&buf[0], "addr=", 5)) {
		ret = sscanf(&buf[5], "%d", (int *)&val);

		if (!ret) {
			pr_err("[TSP] invalid inputs\n");

			return -EINVAL;
		}

		parameter2_t48_addr = val;

		return count;
	}

	pr_err("[TSP] unknown command\n");
		
	return count;
}

static struct kobj_attribute mxt224e_parameter2_t48_interface = __ATTR(parameter2_t48, 0644, mxt224e_parameter2_t48_show, mxt224e_parameter2_t48_store);

static ssize_t mxt224e_parameter3_t48_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	sprintf(buf,   "status: %s\n", parameter3_t48_req ? "on" : "off");
	sprintf(buf, "%saddr: %d\n", buf, parameter3_t48_addr);
	sprintf(buf, "%sval: %d\n", buf, parameter3_t48_val);

	return strlen(buf);
}

static ssize_t mxt224e_parameter3_t48_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	u8  val;
	u16 addr = 0;
	u16 size;

	/* Fetch config data */
	get_object_info(copy_data, PROCG_NOISESUPPRESSION_T48, &size, &addr);

	if (!strncmp(buf, "on", 2)) {
		parameter3_t48_req = true;
		
		pr_info("[TSP] parameter3_t48 %s\n", parameter3_t48_req ? "on" : "off");

		if (!is_suspend) {
			write_mem(copy_data, addr + parameter3_t48_addr, 1, &parameter3_t48_val);
		}

		return count;
	}

	if (!strncmp(buf, "off", 3)) {
		parameter3_t48_req = false;

		pr_info("[TSP] parameter3_t48 %s\n", parameter3_t48_req ? "on" : "off");

		return count;
	}

	if (!strncmp(&buf[0], "val=", 4)) {
		ret = sscanf(&buf[4], "%d", (int *)&val);

		if (!ret) {
			pr_err("[TSP] invalid inputs\n");

			return -EINVAL;
		}

		parameter3_t48_val = val;

		if (parameter3_t48_req) {
			write_mem(copy_data, addr + parameter3_t48_addr, 1, &parameter3_t48_val);
		}

		return count;
	}

	if (!strncmp(&buf[0], "addr=", 5)) {
		ret = sscanf(&buf[5], "%d", (int *)&val);

		if (!ret) {
			pr_err("[TSP] invalid inputs\n");

			return -EINVAL;
		}

		parameter3_t48_addr = val;

		return count;
	}

	pr_err("[TSP] unknown command\n");
		
	return count;
}

static struct kobj_attribute mxt224e_parameter3_t48_interface = __ATTR(parameter3_t48, 0644, mxt224e_parameter3_t48_show, mxt224e_parameter3_t48_store);

static struct attribute *mxt224e_attrs[] = {
	&mxt224e_sweep2wake_interface.attr, 
	&mxt224e_config_t7_interface.attr, 
	&mxt224e_config_t8_interface.attr, 
	&mxt224e_config_t9_interface.attr, 
	&mxt224e_config_t19_interface.attr, 
	&mxt224e_config_t38_interface.attr, 
	&mxt224e_config_t46_interface.attr, 
	&mxt224e_config_t48_interface.attr, 
#ifdef TOUCH_BOOSTER
	&mxt224e_touchboost_interface.attr, 
	&mxt224e_touchboost_freq_interface.attr, 
	&mxt224e_touchboost_ape_interface.attr, 
	&mxt224e_touchboost_ddr_interface.attr, 
#endif
	&mxt224e_tsp_calibrate_interface.attr, 
	&mxt224e_numtouch_t48_interface.attr, 
	&mxt224e_movefilter_t48_interface.attr, 
	&mxt224e_threshold_t48_interface.attr, 
	&mxt224e_parameter1_t48_interface.attr, 
	&mxt224e_parameter2_t48_interface.attr, 
	&mxt224e_parameter3_t48_interface.attr, 
	NULL,
};

static struct attribute_group mxt224e_interface_group = {
	.attrs = mxt224e_attrs,
};

static struct kobject *mxt224e_kobject;

static int __devinit mxt224_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	struct mxt224_platform_data *pdata = client->dev.platform_data;
	struct mxt224_data *data;
	struct input_dev *input_dev;
	int ret;
	int i;
	u8 **tsp_config;
//	u8 val;
	u16 size;
	u16 object_address = 0;
	char msg[8];

	touch_is_pressed = 0;

	if (!pdata) {
		dev_err(&client->dev, "missing platform data\n");
		return -ENODEV;
	}

	if (pdata->max_finger_touches <= 0)
		return -EINVAL;

	data = kzalloc(sizeof(*data) + pdata->max_finger_touches *
					sizeof(*data->fingers), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->num_fingers = pdata->max_finger_touches;
	data->power_con = pdata->power_con;
//	data->register_cb = pdata->register_cb;
	data->read_ta_status = pdata->read_ta_status;
	data->config_fw_version = pdata->config_fw_version;

	data->client = client;
	i2c_set_clientdata(client, data);

	input_dev = input_allocate_device();
	if (!input_dev) {
		ret = -ENOMEM;
		dev_err(&client->dev, "input device allocation failed\n");
		goto err_alloc_dev;
	}
	data->input_dev = input_dev;
	input_set_drvdata(input_dev, data);
	input_dev->name = "sec_touchscreen";

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_mt_init_slots(input_dev, data->num_fingers);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->min_x,
			pdata->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->min_y,
			pdata->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, pdata->min_z,
			pdata->max_z, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, pdata->min_w,
			pdata->max_w, 0, 0);

#if defined(CONFIG_SHAPE_TOUCH)
	input_set_abs_params(input_dev, ABS_MT_COMPONENT, 0, 255, 0, 0);
#endif
	ret = input_register_device(input_dev);
	if (ret) {
		input_free_device(input_dev);
		goto err_reg_dev;
	}

	data->gpio_read_done = pdata->gpio_read_done;

	data->power_con(true);

	ret = mxt224_init_touch_driver(data);
	if (ret < 0)
		goto err_gpio_req;

	copy_data = data;

//	data->register_cb(mxt224_ta_probe);

	if (ret) {
		dev_err(&client->dev, "chip initialization failed\n");
		goto err_init_drv;
	}

	if (data->family_id == 0x80) {
		/*  : MXT-224 */
		tsp_config = (u8 **)pdata->config;
		data->atchcalst = pdata->atchcalst;
		data->atchcalsthr = pdata->atchcalsthr;
		data->tchthr_batt = pdata->tchthr_batt;
		data->tchthr_batt_init = pdata->tchthr_batt_init;
		data->tchthr_charging = pdata->tchthr_charging;
		data->noisethr_batt = pdata->noisethr_batt;
		data->noisethr_charging = pdata->noisethr_charging;
		data->movfilter_batt = pdata->movfilter_batt;
		data->movfilter_charging = pdata->movfilter_charging;
		printk(KERN_INFO"[TSP] TSP chip is MXT224\n");
	} else if (data->family_id == 0x81) {
		/* tsp_family_id - 0x81 : MXT-224E */
		tsp_config = (u8 **)pdata->config_e;
		data->t48_config_batt_e = pdata->t48_config_batt_e;
		data->t48_config_chrg_e = pdata->t48_config_chrg_e;
		data->tchthr_batt = pdata->tchthr_batt_e;
		data->tchthr_charging = pdata->tchthr_charging_e;
		data->calcfg_batt_e = pdata->calcfg_batt_e;
		data->calcfg_charging_e = pdata->calcfg_charging_e;
		data->atchfrccalthr_e = pdata->atchfrccalthr_e;
		data->atchfrccalratio_e = pdata->atchfrccalratio_e;
		data->atchcalst = pdata->atchcalst;
		data->atchcalsthr = pdata->atchcalsthr;
		printk(KERN_INFO"[TSP] TSP chip is MXT224-E\n");
		if (!(data->tsp_version >= firmware_latest[1]
			&& data->tsp_build >= build_latest[1])) {
			printk(KERN_INFO"[TSP] mxt224E force firmware update\n");
			if (mxt224_load_fw(NULL, MXT224_ECHO_FW_NAME))
				goto err_config;
			else {
				msleep(MXT224_FWRESET_TIME);
				mxt224_init_touch_driver(data);
			}
		}
	} else  {
		printk(KERN_ERR"ERROR : There is no valid TSP ID\n");
		goto err_config;
	}

	for (i = 0; tsp_config[i][0] != RESERVED_T255; i++) {
		ret = write_config(data, tsp_config[i][0],
							tsp_config[i] + 1);
		if (ret)
			goto err_config;

		if (tsp_config[i][0] == GEN_POWERCONFIG_T7)
			data->power_cfg = tsp_config[i] + 1;

		if (tsp_config[i][0] == TOUCH_MULTITOUCHSCREEN_T9) {
			/* Are x and y inverted? */
			if (tsp_config[i][10] & 0x1) {
				data->x_dropbits =
					(!(tsp_config[i][22] & 0xC)) << 1;
				data->y_dropbits =
					(!(tsp_config[i][20] & 0xC)) << 1;
			} else {
				data->x_dropbits =
					(!(tsp_config[i][20] & 0xC)) << 1;
				data->y_dropbits =
					(!(tsp_config[i][22] & 0xC)) << 1;
			}
		}
	}

	ret = mxt224_backup(data);
	if (ret)
		goto err_backup;

	/* reset the touch IC. */
	ret = mxt224_reset(data);
	if (ret)
		goto err_reset;

	msleep(MXT224_RESET_TIME);
	calibrate_chip();

	for (i = 0; i < data->num_fingers; i++)
		data->fingers[i].state = MXT224_STATE_INACTIVE;


#if 1 /*  janice only value */
	noise_median.t46_actvsyncsperx_for_mferr = 40;
	noise_median.t48_mfinvlddiffthr_for_mferr = 12;
	noise_median.t48_mferrorthr_for_mferr = 19;
	noise_median.t48_thr_for_mferr = 35;
	noise_median.t48_movfilter_for_mferr = 0;
#else
	noise_median.t46_actvsyncsperx_for_mferr = 38;
	noise_median.t48_mfinvlddiffthr_for_mferr = 12;
	noise_median.t48_mferrorthr_for_mferr = 19;
	noise_median.t48_thr_for_mferr = 40;
	noise_median.t48_movfilter_for_mferr = 0;
#endif

	mutex_init(&data->lock);
	data->enabled = 1;

#if defined(TOUCH_BOOSTER)
	prcmu_qos_add_requirement(PRCMU_QOS_APE_OPP, (char *)client->name,
				  PRCMU_QOS_DEFAULT_VALUE);
	prcmu_qos_add_requirement(PRCMU_QOS_DDR_OPP, (char *)client->name,
				  PRCMU_QOS_DEFAULT_VALUE);
	prcmu_qos_add_requirement(PRCMU_QOS_ARM_KHZ, (char *)client->name,
				  PRCMU_QOS_DEFAULT_VALUE);
	dev_info(&client->dev, "add_prcmu_qos is added\n");
#endif

	valid_touch = 0;
	ret = request_threaded_irq(client->irq, NULL, mxt224_irq_thread,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "mxt224_ts", data);

	if (ret < 0)
		goto err_irq;

	ret = sysfs_create_group(&client->dev.kobj, &mxt224_attr_group);
	if (ret)
		dev_err(&data->client->dev, "sysfs_create_group()is falled\n");

	sec_touchscreen = device_create(sec_class, NULL, 0, NULL, "sec_touchscreen");
	dev_set_drvdata(sec_touchscreen, data);
	if (IS_ERR(sec_touchscreen))
		dev_err(&data->client->dev,
			"Failed to create device(sec_touchscreen)!\n");

	if (device_create_file(sec_touchscreen, &dev_attr_tsp_firm_update) < 0)
		dev_err(&data->client->dev, "Failed to create device file(%s)!\n",
				dev_attr_tsp_firm_update.attr.name);

	if (device_create_file(sec_touchscreen, &dev_attr_tsp_firm_update_status) < 0)
		dev_err(&data->client->dev, "[TSP] Failed to create device file(%s)!\n",
				dev_attr_tsp_firm_update_status.attr.name);

	if (device_create_file(sec_touchscreen, &dev_attr_tsp_threshold) < 0)
		dev_err(&data->client->dev, "Failed to create device file(%s)!\n",
				dev_attr_tsp_threshold.attr.name);

	if (device_create_file(sec_touchscreen, &dev_attr_tsp_firm_version_phone) < 0)
		dev_err(&data->client->dev, "[TSP] Failed to create device file(%s)!\n",
				dev_attr_tsp_firm_version_phone.attr.name);

	if (device_create_file(sec_touchscreen, &dev_attr_tsp_firm_version_panel) < 0)
		dev_err(&data->client->dev, "Failed to create device file(%s)!\n",
				dev_attr_tsp_firm_version_panel.attr.name);


	if (device_create_file(sec_touchscreen, &dev_attr_tsp_config_version) < 0)
		dev_err(&data->client->dev, "Failed to create device file(%s)!\n",
				dev_attr_tsp_config_version.attr.name);

	mxt224_noise_test = device_create(sec_class, NULL, 0, NULL, "tsp_noise_test");
	dev_set_drvdata(mxt224_noise_test, data);
	if (IS_ERR(mxt224_noise_test))
		dev_err(&data->client->dev,
			"Failed to create device(mxt224_noise_test)!\n");

	if (device_create_file(mxt224_noise_test, &dev_attr_set_refer0) < 0)
		dev_err(&data->client->dev, "Failed to create device file(%s)!\n",
				dev_attr_set_refer0.attr.name);

	if (device_create_file(mxt224_noise_test, &dev_attr_set_delta0) < 0)
		dev_err(&data->client->dev, "Failed to create device file(%s)!\n",
				dev_attr_set_delta0.attr.name);

	if (device_create_file(mxt224_noise_test, &dev_attr_set_refer1) < 0)
		dev_err(&data->client->dev, "Failed to create device file(%s)!\n",
				dev_attr_set_refer1.attr.name);

	if (device_create_file(mxt224_noise_test, &dev_attr_set_delta1) < 0)
		dev_err(&data->client->dev, "Failed to create device file(%s)!\n",
				dev_attr_set_delta1.attr.name);

	if (device_create_file(mxt224_noise_test, &dev_attr_set_refer2) < 0)
		dev_err(&data->client->dev, "Failed to create device file(%s)!\n",
				dev_attr_set_refer2.attr.name);

	if (device_create_file(mxt224_noise_test, &dev_attr_set_delta2) < 0)
		dev_err(&data->client->dev, "Failed to create device file(%s)!\n",
				dev_attr_set_delta2.attr.name);

	if (device_create_file(mxt224_noise_test, &dev_attr_set_refer3) < 0)
		dev_err(&data->client->dev, "Failed to create device file(%s)!\n",
				dev_attr_set_refer3.attr.name);

	if (device_create_file(mxt224_noise_test, &dev_attr_set_delta3) < 0)
		dev_err(&data->client->dev, "Failed to create device file(%s)!\n",
				dev_attr_set_delta3.attr.name);

	if (device_create_file(mxt224_noise_test, &dev_attr_set_refer4) < 0)
		dev_err(&data->client->dev, "Failed to create device file(%s)!\n",
				dev_attr_set_refer4.attr.name);

	if (device_create_file(mxt224_noise_test, &dev_attr_set_delta4) < 0)
		dev_err(&data->client->dev, "Failed to create device file(%s)!\n",
				dev_attr_set_delta4.attr.name);

	if (device_create_file(mxt224_noise_test, &dev_attr_set_all_refer) < 0)
		dev_err(&data->client->dev, "Failed to create device file(%s)!\n",
				dev_attr_set_all_refer.attr.name);

	if (device_create_file(mxt224_noise_test, &dev_attr_disp_all_refdata) < 0)
		dev_err(&data->client->dev, "Failed to create device file(%s)!\n",
				dev_attr_disp_all_refdata.attr.name);

	if (device_create_file(mxt224_noise_test, &dev_attr_set_all_delta) < 0)
		dev_err(&data->client->dev, "Failed to create device file(%s)!\n",
				dev_attr_set_all_delta.attr.name);

	if (device_create_file(mxt224_noise_test, &dev_attr_disp_all_deltadata) < 0)
		dev_err(&data->client->dev, "Failed to create device file(%s)!\n",
				dev_attr_disp_all_deltadata.attr.name);

	if (device_create_file(mxt224_noise_test, &dev_attr_set_tsp_name) < 0)
		dev_err(&data->client->dev, "Failed to create device file(%s)!\n",
				dev_attr_set_tsp_name.attr.name);

	if (device_create_file(mxt224_noise_test, &dev_attr_set_tsp_channel) < 0)
		dev_err(&data->client->dev, "Failed to create device file(%s)!\n",
				dev_attr_set_tsp_channel.attr.name);

	if (device_create_file(mxt224_noise_test, &dev_attr_set_threshold) < 0)
		dev_err(&data->client->dev, "Failed to create device file(%s)!\n",
				dev_attr_set_threshold.attr.name);
	/*
	if (device_create_file(mxt224_noise_test, &dev_attr_set_firm_version) < 0)
		dev_err(&data->client->dev, "Failed to create device file(%s)!\n",
				dev_attr_set_firm_version.attr.name);
	*/
	if (device_create_file(mxt224_noise_test, &dev_attr_set_module_off) < 0)
		dev_err(&data->client->dev, "Failed to create device file(%s)!\n",
				dev_attr_set_module_off.attr.name);

	if (device_create_file(mxt224_noise_test, &dev_attr_set_module_on) < 0)
		dev_err(&data->client->dev, "Failed to create device file(%s)!\n",
				dev_attr_set_module_on.attr.name);


	mxt224e_kobject = kobject_create_and_add("mxt224e", kernel_kobj);

	if (!mxt224e_kobject) {
		return -ENOMEM;
	}

	ret = sysfs_create_group(mxt224e_kobject, &mxt224e_interface_group);

	if (ret) {
		kobject_put(mxt224e_kobject);
	}

#if CONFIG_HAS_WAKELOCK
	wake_lock_init(&s2w_wakelock, WAKE_LOCK_SUSPEND, "s2w_wakelock");
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = mxt224_early_suspend;
	data->early_suspend.resume = mxt224_late_resume;
	register_early_suspend(&data->early_suspend);
#endif



	dev_info(&data->client->dev, "vbus_state = %d\n", (int)vbus_state);
	mxt224_ta_probe(vbus_state);

	noise_median.median_on_flag = false;
	noise_median.mferr_setting = false;
	noise_median.mferr_count = 0;

	/* only falling trigger */

	ret = get_object_info(copy_data,
		GEN_MESSAGEPROCESSOR_T5, &size, &object_address);
	read_mem(copy_data, object_address, 8, msg);

	return 0;

err_irq:
#if defined(TOUCH_BOOSTER)
	prcmu_qos_remove_requirement(PRCMU_QOS_APE_OPP, (char *)client->name);
	prcmu_qos_remove_requirement(PRCMU_QOS_DDR_OPP, (char *)client->name);
	prcmu_qos_remove_requirement(PRCMU_QOS_ARM_KHZ, (char *)client->name);
#endif
err_reset:
err_backup:
err_config:
	kfree(data->objects);
err_init_drv:
	gpio_free(data->gpio_read_done);
err_gpio_req:
	data->power_con(false);
	input_unregister_device(input_dev);
err_reg_dev:
err_alloc_dev:
	kfree(data);
	return ret;
}

static int __devexit mxt224_remove(struct i2c_client *client)
{
	struct mxt224_data *data = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif
	free_irq(client->irq, data);
	
#if defined(TOUCH_BOOSTER)
	prcmu_qos_remove_requirement(PRCMU_QOS_APE_OPP, (char *)client->name);
	prcmu_qos_remove_requirement(PRCMU_QOS_DDR_OPP, (char *)client->name);
	prcmu_qos_remove_requirement(PRCMU_QOS_ARM_KHZ, (char *)client->name);
#endif

#if CONFIG_HAS_WAKELOCK
	wake_lock_destroy(&s2w_wakelock);
#endif

	kfree(data->objects);
	gpio_free(data->gpio_read_done);
	data->power_con(false);
	input_unregister_device(data->input_dev);
	kfree(data);

	return 0;
}

static struct i2c_device_id mxt224_idtable[] = {
	{MXT224_DEV_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, mxt224_idtable);

static const struct dev_pm_ops mxt224_pm_ops = {
	.suspend = mxt224_suspend,
	.resume = mxt224_resume,
};

static struct i2c_driver mxt224_i2c_driver = {
	.id_table = mxt224_idtable,
	.probe = mxt224_probe,
	.remove = __devexit_p(mxt224_remove),
	.driver = {
		.owner	= THIS_MODULE,
		.name	= MXT224_DEV_NAME,
		.pm	= &mxt224_pm_ops,
	},
};

static int __init mxt224_init(void)
{
	return i2c_add_driver(&mxt224_i2c_driver);
}

static void __exit mxt224_exit(void)
{
	i2c_del_driver(&mxt224_i2c_driver);
}
module_init(mxt224_init);
module_exit(mxt224_exit);

MODULE_DESCRIPTION("Atmel MaXTouch 224E driver");
MODULE_AUTHOR("Heetae Ahn <heetae82.ahn@samsung.com>");
MODULE_LICENSE("GPL");
