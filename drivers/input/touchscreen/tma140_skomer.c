/* drivers/input/touchscreen/tma140_lucas.c
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>
#include <linux/irq.h>


#include <linux/firmware.h>
#include <linux/uaccess.h> 
#include <linux/rtc.h>
/* firmware - update */

#define MAX_X	480 
#define MAX_Y	800
#define TSP_INT 218//86
#define TSP_SDA 229//85
#define TSP_SCL 230//87

#define TOUCH_EN 22

#define MAX_KEYS	2
#define MAX_USING_FINGER_NUM 2

#define SEC_TSP_FACTORY_TEST

#define NODE_X_NUM 16
#define NODE_Y_NUM 11
#define GLOBAL_IDAC_NUM	22
#define NODE_NUM	(NODE_X_NUM*NODE_Y_NUM)	/* 8X10 */


#define LOCAL_IDAC_START	6
#define GLOBAL_IDAC_START	7

#define __SEND_VIRTUAL_RELEASED__		// force release previous press event, when the next touch is pressed very quickly

#define TSP_EDS_RECOVERY

unsigned int raw_count[NODE_NUM]= {{0,},};;
unsigned int difference[NODE_NUM]= {{0,},};;
unsigned int local_idac[NODE_NUM]= {{0,},};;
unsigned int global_idac[GLOBAL_IDAC_NUM]= {{0,},};;


//#define __TOUCHKEY__

#ifdef __TOUCHKEY__
static const int touchkey_keycodes[] = {
		KEY_MENU,
		KEY_BACK,
		//KEY_HOME,
		//KEY_SEARCH,
};
#endif

#ifdef SEC_TSP_FACTORY_TEST
#define TSP_BUF_SIZE 1024

#define TSP_CMD_STR_LEN 32
#define TSP_CMD_RESULT_STR_LEN 512
#define TSP_CMD_PARAM_NUM 8

#define TSP_SPEC_RAW_COUNT_MIN 50
#define TSP_SPEC_RAW_COUNT_MAX 150

#define TSP_SPEC_DIFFERENCE_MIN 70
#define TSP_SPEC_DIFFERENCE_MAX 130

#define TSP_SPEC_LOCAL_IDAC_MIN 1
#define TSP_SPEC_LOCAL_IDAC_MAX 30

#define TSP_SPEC_GLOBAL_IDAC_MIN 145
#define TSP_SPEC_GLOBAL_IDAC_MAX 295


#endif /* SEC_TSP_FACTORY_TEST */


#define TOUCH_ON 1
#define TOUCH_OFF 0

#define TRUE    1
#define FALSE    0

#define I2C_RETRY_CNT	2

#define __TOUCH_DEBUG__
#define __TOUCH_KMSG__

#ifdef __TOUCH_KMSG__
#define	tma_kmsg(fmt, args...)	printk(KERN_INFO "[TSP][%-18s:%5d]" fmt, __FUNCTION__, __LINE__, ## args)
#else
#define	tma_kmsg(fmt, args...)	do{}while(0)
#endif


#define __TSP_FORCE_UPDATE__
#define USE_THREADED_IRQ	1

//#define __TOUCH_1V8__ 

static struct regulator *touch_regulator=NULL;
#if defined (__TOUCH_1V8__)
static struct regulator *touch_1v8_regulator=NULL;
#endif


#if defined (TSP_EDS_RECOVERY)
static struct workqueue_struct *check_ic_wq;
#endif

#ifdef __TOUCHKEY__
static int touchkey_status[MAX_KEYS];

#define TK_STATUS_PRESS		1
#define TK_STATUS_RELEASE		0
#endif

int tsp_irq;
int st_old;

typedef struct
{
	int8_t id;	/*!< (id>>8) + size */
	int8_t status;/////////////IC
	int8_t z;	/*!< dn>0, up=0, none=-1 */
	int16_t x;			/*!< X */
	int16_t y;			/*!< Y */
} report_finger_info_t;

#if defined (__SEND_VIRTUAL_RELEASED__)
static report_finger_info_t fingerInfo[MAX_USING_FINGER_NUM+2]={0,};
#else
static report_finger_info_t fingerInfo[MAX_USING_FINGER_NUM]={0,};
#endif

enum {
	BUILT_IN = 0,
	UMS,
	REQ_FW,
};

struct tma_platform_data {
	u8 exit_flag;
};

struct touch_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
    	const struct tma_platform_data	*pdata;
	int use_irq;
	struct hrtimer timer;				////////////////////////IC
#if defined (TSP_EDS_RECOVERY)
	struct work_struct  esd_recovery_func;		////////////////////////IC
#endif	
	unsigned char fw_ic_ver;

#if defined(SEC_TSP_FACTORY_TEST)
	struct list_head			cmd_list_head;
	unsigned char cmd_state;
	char			cmd[TSP_CMD_STR_LEN];
	int			cmd_param[TSP_CMD_PARAM_NUM];
	char			cmd_result[TSP_CMD_RESULT_STR_LEN];
	struct mutex			cmd_lock;
	bool			cmd_is_running;

	bool ft_flag;
#endif				/* SEC_TSP_FACTORY_TEST */

	struct early_suspend early_suspend;
};


struct touch_data *ts_global;

/* firmware - update */
static int firmware_ret_val = -1;


unsigned char esd_conter=0;
unsigned char now_tspfw_update = 0;
bool bRun_set_tsp_for_ta_detect = false;

unsigned char tsp_special_update = 0;
static char IsfwUpdate[20]={0};

/* touch information*/
unsigned char touch_vendor_id1 = 0;
unsigned char touch_vendor_id2 = 0;
unsigned char touch_hw_id = 0;
unsigned char touch_fw_ver = 0;

#define TSP_VENDER_ID	0xF0

#define TSP_KERNEL_FW_ID		0x07    //Firmware version in Kernel binary

int tsp_irq_num = 0;
int tsp_workqueue_num = 0;
int tsp_threadedirq_num = 0;

int g_touch_info_x = 0;
int g_touch_info_y = 0;
int g_touch_info_press = 0;

int prev_pressed_num=0;

static int pre_ta_stat = 0;
int tsp_status=0;
int reset_check = 0;	// flag to check in the reset sequecne : 1, or not : 0

static uint8_t raw_min=0;
static uint8_t raw_max=0;
static uint8_t IDAC_min=0;
static uint8_t IDAC_max=0;

extern int cypress_update( int );
int tsp_i2c_read(u8 reg, unsigned char *rbuf, int buf_size);

void set_tsp_for_ta_detect(int);

struct touch_trace_data {
	uint32_t time;
	int16_t fingernum;	
	int8_t status;
	int8_t id;
	uint16_t x;
	uint16_t y;

};
#define MAX_TOUCH_TRACE_NUMBER	10000
static int touch_trace_index = 0;
static struct touch_trace_data touch_trace_info[MAX_TOUCH_TRACE_NUMBER];

void tsp_log(report_finger_info_t *fingerinfo, int i);
struct timespec ts;
struct rtc_time tm;
	
#ifdef __TOUCHKEY__
int touchkey_scan_mode = 0;

#define TOUCHKEY_SPEC_THRESHOLD 18
int touchkey_threshold_value = TOUCHKEY_SPEC_THRESHOLD;
#endif

/* sys fs */
struct class *touch_class;
EXPORT_SYMBOL(touch_class);

#if 0
struct device *firmware_dev;
EXPORT_SYMBOL(firmware_dev);
#else
struct device *sec_touchscreen;
EXPORT_SYMBOL(sec_touchscreen);
#ifdef __TOUCHKEY__
struct device *sec_touchkey;
EXPORT_SYMBOL(sec_touchkey);
#endif // __TOUCHKEY__
#endif

static int read_firmware_version(unsigned char hw_ver_backup, uint8_t *buf_tmp);

static int firm_update_callfc(void);

static ssize_t read_threshold(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firmware_In_Binary(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firmware_In_TSP(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firm_update(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firmware_update_status(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t tsp_panel_rev(struct device *dev, struct device_attribute *attr, char *buf);
#ifdef __TOUCHKEY__
static ssize_t menu_sensitivity_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t back_sensitivity_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t touchkey_sensitivity_power_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size);
static ssize_t read_touchkey_threshold(struct device *dev, struct device_attribute *attr, char *buf);
#endif

static DEVICE_ATTR(tsp_firm_version_phone, S_IRUGO /*0444*/, firmware_In_Binary, NULL);
static DEVICE_ATTR(tsp_firm_version_panel, S_IRUGO /*0444*/, firmware_In_TSP, NULL);
static DEVICE_ATTR(tsp_firm_update, S_IRUGO | S_IWUSR | S_IWGRP /*0664*/, firm_update, firm_update);
static DEVICE_ATTR(tsp_firm_update_status, S_IRUGO /*0444*/, firmware_update_status, firmware_update_status);
static DEVICE_ATTR(tsp_threshold, S_IRUGO /*0444*/, read_threshold, read_threshold);
static DEVICE_ATTR(get_panel_rev, S_IRUGO /*0444*/, tsp_panel_rev, tsp_panel_rev);
#ifdef __TOUCHKEY__
static DEVICE_ATTR(touchkey_menu, S_IRUGO, menu_sensitivity_show, NULL);
static DEVICE_ATTR(touchkey_back, S_IRUGO, back_sensitivity_show, NULL);
static DEVICE_ATTR(touch_sensitivity, S_IRUGO | S_IWUSR | S_IWGRP, NULL, touchkey_sensitivity_power_store);
static DEVICE_ATTR(touchkey_threshold, S_IRUGO, read_touchkey_threshold, NULL);
#endif
/* sys fs */

#if defined(SEC_TSP_FACTORY_TEST)
#define TSP_CMD(name, func) .cmd_name = name, .cmd_func = func

struct tsp_cmd {
	struct list_head	list;
	const char	*cmd_name;
	void	(*cmd_func)(void *device_data);
};

static void fw_update(void *device_data);
static void get_fw_ver_bin(void *device_data);
static void get_fw_ver_ic(void *device_data);
static void get_threshold(void *device_data);
static void module_off_master(void *device_data);
static void module_on_master(void *device_data);
static void module_off_slave(void *device_data);
static void module_on_slave(void *device_data);
static void get_chip_vendor(void *device_data);
static void get_chip_name(void *device_data);
static void get_reference(void *device_data);
static void get_raw_count(void *device_data);
static void get_difference(void *device_data);
static void get_intensity(void *device_data);
static void get_local_idac(void *device_data);
static void get_global_idac(void *device_data);
static void get_x_num(void *device_data);
static void get_y_num(void *device_data);
static void run_reference_read(void *device_data);
static void run_raw_count_read(void *device_data);
static void run_difference_read(void *device_data);
static void run_intensity_read(void *device_data);
static void not_support_cmd(void *device_data);
static void run_raw_node_read(void *device_data);
static void run_global_idac_read(void *device_data);
static void run_local_idac_read(void *device_data);


struct tsp_cmd tsp_cmds[] = {
	{TSP_CMD("fw_update", not_support_cmd),},
	{TSP_CMD("get_fw_ver_bin", get_fw_ver_bin),},
	{TSP_CMD("get_fw_ver_ic", get_fw_ver_ic),},
	{TSP_CMD("get_threshold", get_threshold),},
	{TSP_CMD("module_off_master", not_support_cmd),},
	{TSP_CMD("module_on_master", not_support_cmd),},
	{TSP_CMD("module_off_slave", not_support_cmd),},
	{TSP_CMD("module_on_slave", not_support_cmd),},
	{TSP_CMD("get_chip_vendor", get_chip_vendor),},
	{TSP_CMD("get_chip_name", get_chip_name),},
	{TSP_CMD("run_raw_node_read", run_raw_node_read),},
	{TSP_CMD("get_x_num", get_x_num),},
	{TSP_CMD("get_y_num", get_y_num),},
	{TSP_CMD("get_reference", not_support_cmd),},
	{TSP_CMD("get_raw_count", get_raw_count),},
	{TSP_CMD("get_difference", get_difference),},
	{TSP_CMD("get_intensity", not_support_cmd),},
	{TSP_CMD("get_local_idac", get_local_idac),},	
	{TSP_CMD("get_global_idac", get_global_idac),},
	{TSP_CMD("run_reference_read", not_support_cmd),},
	{TSP_CMD("run_raw_count_read", run_raw_count_read),},
	{TSP_CMD("run_difference_read", run_difference_read),},
	{TSP_CMD("run_intensity_read", not_support_cmd),},
	{TSP_CMD("run_global_idac_read", run_global_idac_read),},
	{TSP_CMD("run_local_idac_read", run_local_idac_read),},	
	{TSP_CMD("not_support_cmd", not_support_cmd),},
};
#endif

static int tsp_testmode = 0;
static int prev_wdog_val = -1;
static int tsp_irq_operation = 0;
static int tsp_releasing = 0;
#ifdef TSP_EDS_RECOVERY
static unsigned int touch_present = 0;
#endif

#define FW_DOWNLOADING "Downloading"
#define FW_DOWNLOAD_COMPLETE "Complete"
#define FW_DOWNLOAD_FAIL "FAIL"
#define FWUP_NOW -1

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ts_early_suspend(struct early_suspend *h);
static void ts_late_resume(struct early_suspend *h);
#endif

extern int bcm_gpio_pull_up(unsigned int gpio, bool up);
extern int bcm_gpio_pull_up_down_enable(unsigned int gpio, bool enable);
extern int set_irq_type(unsigned int irq, unsigned int type);

//extern int tsp_charger_type_status;
extern u8 vbus_state;

void touch_ctrl_regulator(int on_off)
{

	if(!touch_regulator) {
		printk(KERN_ERR "tma140: 3.3v tsp regulator is NULL\n");
		
		touch_regulator = regulator_get(NULL, "v-tsp-3.3");
		if (IS_ERR(touch_regulator)) {
			printk(KERN_ERR
				"tma140: fail to get regulator v3.3 (%d)\n",
						(int)PTR_ERR(touch_regulator));
			goto err_get_reg_3v3;
		}
		regulator_set_voltage(touch_regulator, 3000000, 3000000);
	}
	
#if defined (__TOUCH_1V8__)
	if(!touch_1v8_regulator) {
		printk(KERN_ERR "tma140: 1.8v tsp regulator is NULL\n");
	
		touch_1v8_regulator = regulator_get(NULL, "v-tsp-1.8");
		if (IS_ERR(touch_1v8_regulator)) {
			printk(KERN_ERR "fail to get regulator v1.8 (%d)\n",
						(int)PTR_ERR(touch_1v8_regulator));
			goto err_get_reg_1v8;
		}
		regulator_set_voltage(touch_1v8_regulator, 1800000, 1800000);
	}
#endif

	if (on_off) {
#if defined (__TOUCH_1V8__)
		regulator_enable(touch_1v8_regulator);
#endif
		regulator_enable(touch_regulator);
	} else {
		regulator_disable(touch_regulator);
#if defined (__TOUCH_1V8__)
		regulator_disable(touch_1v8_regulator);
#endif
	}

	printk(KERN_INFO "%s is finished.(%s)\n",
						__func__, (on_off) ? "on" : "off");
						
	return;

#if defined (__TOUCH_1V8__)
err_get_reg_1v8:
#endif
err_get_reg_3v3:
	regulator_put(touch_regulator);
	return;
}
EXPORT_SYMBOL(touch_ctrl_regulator);

int tsp_reset( void )
{
	int ret=1;

#if defined(__TOUCH_DEBUG__)
	printk("[TSP] %s, %d\n", __func__, __LINE__ );
#endif 

	if(reset_check == 0)
	{
		reset_check = 1;

		touch_ctrl_regulator(0);

		gpio_direction_output( TSP_SCL , 0 ); 
		gpio_direction_output( TSP_SDA , 0 ); 
		//gpio_direction_output( TSP_INT , 0 ); 

		msleep(500);

		gpio_direction_output( TSP_SCL , 1 ); 
		gpio_direction_output( TSP_SDA , 1 ); 
		//gpio_direction_output( TSP_INT , 1 ); 

		touch_ctrl_regulator(1);

		msleep(10);

		reset_check = 0;
	}

	return ret;
}

#ifdef __TOUCHKEY__
static void process_key_event(uint8_t tsk_msg)
{
	int i;
	int keycode= 0;
	int st_new;

	printk("[TSP] process_key_event : %d\n", tsk_msg);

	if(	tsk_msg	== 0)
	{
		input_report_key(ts_global->input_dev, st_old, 0);
		for(i = 0; i < MAX_KEYS; i++)
		{
			touchkey_status[i] = TK_STATUS_RELEASE;
		}
		printk("[TSP] release keycode: %4d, keypress: %4d\n", st_old, 0);
	}
	else
	{
		//check each key status
		for(i = 0; i < MAX_KEYS; i++)
		{

			st_new = (tsk_msg>>(i)) & 0x1;
			if (st_new ==1)
			{
				keycode = touchkey_keycodes[i];
				input_report_key(ts_global->input_dev, keycode, 1);
				touchkey_status[i] = TK_STATUS_PRESS;
				printk("[TSP] press keycode: %4d, keypress: %4d\n", keycode, 1);
			}

			st_old = keycode;
		}
	}
}

static void force_release_key()
{
	int i;
//	int keycode= 0;

	printk("[TSP] force_release_key ++\n");

	tsp_releasing = 1;
	for(i = 0; i < MAX_KEYS; i++)
	{
		if(touchkey_status[i] == TK_STATUS_PRESS)
		{
			input_report_key(ts_global->input_dev, touchkey_keycodes[i], 0);
			touchkey_status[i] = TK_STATUS_RELEASE;
			printk("[TSP] release keycode: %4d, keypress: %4d\n", touchkey_keycodes[i], 0);
		}
	}
	tsp_releasing = 0;
	
	printk("[TSP] force_release_key --\n");
}
#endif

static void force_release_touch()
{
	int i;
	int released= 0;

	printk("[TSP] force_release_touch ++\n");

	tsp_releasing = 1;
	for(i = 0; i<MAX_USING_FINGER_NUM; i++)
	{
		if(1 == fingerInfo[i].status)
		{
			fingerInfo[i].status = 0;		
			
			input_mt_slot(ts_global->input_dev, i);
			input_mt_report_slot_state(ts_global->input_dev, MT_TOOL_FINGER, 0);
#if defined(__TOUCH_DEBUG__)			
			printk("[TSP] release touch finger num : %d \n", i);
#endif
			tsp_log(fingerInfo, i);

			released = 1;
		}
	}
	if(released)
	{	
		input_sync(ts_global->input_dev);
	}
	tsp_releasing = 0;
	
	printk("[TSP] force_release_touch --\n");
}


void tsp_log(report_finger_info_t *fingerinfo, int i)
{
#if defined(__TOUCH_DEBUG__)
	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);

	printk("[TSP][%02d:%02d:%02d.%03lu] ",  tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
#endif    

	touch_trace_info[touch_trace_index].time = jiffies;
	touch_trace_info[touch_trace_index].status = fingerInfo[i].status;
	touch_trace_info[touch_trace_index].id = fingerInfo[i].id;
	touch_trace_info[touch_trace_index].x = fingerInfo[i].x;
	touch_trace_info[touch_trace_index].y = fingerInfo[i].y;
	touch_trace_info[touch_trace_index].fingernum=i;

#if defined(__TOUCH_DEBUG__)
	printk("[TSP] i[%d] id[%d] xy[%d, %d] status[%d]\n", i, fingerInfo[i].id, fingerInfo[i].x, fingerInfo[i].y, fingerInfo[i].status);
#else
	if(fingerInfo[i].status==1)
	{
	//	printk("[TSP] finger down\n");
		printk("[TSP] F [1]\n");
	}
	else
	{
		//printk("[TSP] finger up\n");					
		printk("[TSP] F [0]\n");
	}
#endif

	touch_trace_index++;

	if(touch_trace_index >= MAX_TOUCH_TRACE_NUMBER)
	{
		touch_trace_index = 0;
	}
	/* -- due to touch trace -- */

}


static irqreturn_t ts_work_func(int irq, void *dev_id)
{
	int ret=0;
	uint8_t buf[29];// 02h ~ 1Fh
	uint8_t i2c_addr = 0x02;
	int i = 0;
	int pressed_num;
#ifdef __TOUCHKEY__	
	uint8_t buf_key[1];	
	int button_check = 0;
#endif

	if(tsp_testmode || tsp_releasing)
		return IRQ_HANDLED;

	tsp_irq_operation = 1;

	struct touch_data *ts = dev_id;

	ret = tsp_i2c_read( i2c_addr, buf, sizeof(buf));

	if (ret <= 0)
	{
		printk("[TSP] i2c failed : ret=%d, ln=%d\n",ret, __LINE__);
		goto work_func_out;
	}


	//01 번지 하위 4비트는 눌린 finger 갯수
	pressed_num= 0x0f&buf[0];
#ifdef __TOUCHKEY__
	buf_key[0] = buf[25] & 0x03; //information of touch key
	button_check = buf[0] & 0x40;

	if(button_check != 0)
	{
		process_key_event(buf_key[0]);

	}

	//if(button_check == 0)
#endif		
	{

		fingerInfo[0].x = (buf[1] << 8) |buf[2];
		fingerInfo[0].y = (buf[3] << 8) |buf[4];
		fingerInfo[0].z = buf[5];
		fingerInfo[0].id = buf[6] >>4;

		fingerInfo[1].x = (buf[7] << 8) |buf[8];
		fingerInfo[1].y = (buf[9] << 8) |buf[10];
		fingerInfo[1].z = buf[11];
		fingerInfo[1].id = buf[6] & 0xf;

#if defined(__TOUCH_DEBUG__)
		printk("[TSP] pressed finger num [%d]\n", pressed_num);			 		
#endif

#if defined (__SEND_VIRTUAL_RELEASED__)
		if((fingerInfo[2].id != fingerInfo[0].id) && (prev_pressed_num ==1 && pressed_num==1))
		{

			input_mt_slot(ts->input_dev, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);

			fingerInfo[2].status = 0;
#if defined(__TOUCH_DEBUG__)		
			printk("[TSP] send virtual release  xy[%d, %d]\n", fingerInfo[2].x, fingerInfo[2].y);
#endif
 			tsp_log(fingerInfo, 2);
			input_sync(ts->input_dev);
		}
#endif

		/* check touch event */
		for ( i= 0; i<MAX_USING_FINGER_NUM; i++ )
#if defined (__SEND_VIRTUAL_RELEASED__)
		{
			if(fingerInfo[i].id >=1) // press interrupt
			{
				fingerInfo[i].status = 1;


				if( fingerInfo[i].id == fingerInfo[i+2].id &&
					fingerInfo[i].x == fingerInfo[i+2].x &&
					fingerInfo[i].y == fingerInfo[i+2].y &&
					fingerInfo[i].z == fingerInfo[i+2].z )
					continue;

				input_mt_slot(ts->input_dev, i);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);

				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, fingerInfo[i].x);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, fingerInfo[i].y);

				tsp_log(fingerInfo, i);

			}
			else if(fingerInfo[i].id ==0) // release interrupt (only first finger)
			{
				if(fingerInfo[i].status == 1) // prev status is press
				{
					fingerInfo[i].status = 0;

					input_mt_slot(ts->input_dev, i);
					input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);

				tsp_log(fingerInfo, i);
				}
			}

		}
#else	
		{
			//////////////////////////////////////////////////IC
			if(fingerInfo[i].id >=1) // press interrupt
			{
				fingerInfo[i].status = 1;
			}
			else if(fingerInfo[i].id ==0) // release interrupt (only first finger)
			{
				if(fingerInfo[i].status == 1) // prev status is press
					fingerInfo[i].status = 0;
				else if(fingerInfo[i].status == 0) // release already or force release
					fingerInfo[i].status = -1;
			}		

			if(fingerInfo[i].status < 0) continue;
			//////////////////////////////////////////////////IC

			input_mt_slot(ts->input_dev, i);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, fingerInfo[i].status);

			if(fingerInfo[i].status)
			{
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, fingerInfo[i].x);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, fingerInfo[i].y);
			}
#if defined(__TOUCH_DEBUG__)	
			printk("[TSP] i[%d] id[%d] xyz[%d, %d, %d] status[%d]\n", i, fingerInfo[i].id, fingerInfo[i].x, fingerInfo[i].y, fingerInfo[i].z, fingerInfo[i].status);	
#endif
		}
#endif
	}

event_check_out:
	input_sync(ts->input_dev);

#if defined (__SEND_VIRTUAL_RELEASED__)
	fingerInfo[2].x = fingerInfo[0].x;
	fingerInfo[2].y = fingerInfo[0].y;
	fingerInfo[2].z = fingerInfo[0].z;
	fingerInfo[2].id = fingerInfo[0].id;	

	fingerInfo[3].x = fingerInfo[1].x;
	fingerInfo[3].y = fingerInfo[1].y;
	fingerInfo[3].z = fingerInfo[1].z;
	fingerInfo[3].id = fingerInfo[1].id;	

	prev_pressed_num=pressed_num;

#endif	



work_func_out:

	tsp_irq_operation = 0;

	return IRQ_HANDLED;
}


int tsp_i2c_read(u8 reg, unsigned char *rbuf, int buf_size)
{
	int i, ret=-1;
	struct i2c_msg rmsg;
	uint8_t start_reg;

	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		rmsg.addr = ts_global->client->addr;
		rmsg.flags = 0;//I2C_M_WR;
		rmsg.len = 1;
		rmsg.buf = &start_reg;
		start_reg = reg;

		ret = i2c_transfer(ts_global->client->adapter, &rmsg, 1);

		if(ret >= 0) 
		{
			rmsg.flags = I2C_M_RD;
			rmsg.len = buf_size;
			rmsg.buf = rbuf;
			ret = i2c_transfer(ts_global->client->adapter, &rmsg, 1 );

			if (ret >= 0)
				break; // i2c success
		}

		if( i == (I2C_RETRY_CNT - 1) )
		{
			printk("[TSP] Error code : %d, %d\n", __LINE__, ret );
		}
	}

	return ret;
}



void set_tsp_for_ta_detect(int state)
{

	int i, ret=0;
	uint8_t buf1[2] = {0,};
	uint8_t temp;

	printk("[TSP] %s, %d\n", __func__, __LINE__ );
	// defense code to avoid being called from "bcmpmu-accy.c" when ts_global is null
	if ( NULL == ts_global )
		return;

	bRun_set_tsp_for_ta_detect = true;

	if(tsp_status==0)
	{	
		if((tsp_testmode == 0) && (tsp_irq_operation == 0 && (now_tspfw_update == 0)))
		{
			if(state)
			{
				printk("[TSP] [1] set_tsp_for_ta_detect!!! state=1\n");

				for (i = 0; i < I2C_RETRY_CNT; i++)
				{
					buf1[0] = 0x01; //address
					ret = i2c_master_send(ts_global->client, buf1, 1);

					if (ret >= 0)
					{
						ret = i2c_master_recv(ts_global->client, buf1, 1);

						if (ret >= 0)
						{
							temp = buf1[0] | 0x04;//0b0000 0100

							buf1[0] = 0x01;//address
							buf1[1] = temp;//data
							ret = i2c_master_send(ts_global->client, buf1, 2);

							if (ret >= 0)
							{
								printk("[TSP] 01h = 0x%x\n", temp);
								break; // i2c success
							}
						}	
					}

					printk("[TSP] %s, %d, fail\n", __func__, __LINE__ );
				}

				pre_ta_stat = 1;
			}
			else
			{
				printk("[TSP] [2] set_tsp_for_ta_detect!!! state=0\n");

				for (i = 0; i < I2C_RETRY_CNT; i++)
				{
					buf1[0] = 0x01; //address
					ret = i2c_master_send(ts_global->client, buf1, 1);

					if (ret >= 0)
					{
						ret = i2c_master_recv(ts_global->client, buf1, 1);

						if (ret >= 0)
						{
							temp = buf1[0] & 0xFB;//0b1111 1011

							buf1[0] = 0x01;//address
							buf1[1] = temp;//data
							ret = i2c_master_send(ts_global->client, buf1, 2);

							if (ret >= 0)
							{
								printk("[TSP] 01h = 0x%x\n", temp);
								break; // i2c success
							}

						}	
					}

					printk("[TSP] %s, %d, fail\n", __func__, __LINE__ );
				}

				pre_ta_stat = 0;
			}
		}
	}
	bRun_set_tsp_for_ta_detect = false;
}	
EXPORT_SYMBOL(set_tsp_for_ta_detect);

#if defined (TSP_EDS_RECOVERY)
static void check_ic_work_func(struct work_struct *esd_recovery_func)
{
	int ret=0;
	uint8_t buf_esd[1];
	uint8_t wdog_val[1];

	struct touch_data *ts = container_of(esd_recovery_func, struct touch_data, esd_recovery_func);

	buf_esd[0] = 0x1F;
	wdog_val[0] = 1;

	if((tsp_testmode == 0) && (tsp_irq_operation == 0 && (now_tspfw_update == 0)))
	{
		ret = i2c_master_send(ts->client, buf_esd, 1);
		if(ret >=0)
		{
			ret = i2c_master_recv(ts->client, wdog_val, 1);

			if(((wdog_val[0] & 0xFC) >> 2) == (uint8_t)prev_wdog_val)
			{
				printk("[TSP] %s tsp_reset counter = %x, prev = %x\n", __func__, ((wdog_val[0] & 0xFC) >> 2), (uint8_t)prev_wdog_val);
				disable_irq(ts_global->client->irq);				
				tsp_reset();
				enable_irq(ts_global->client->irq);				
				prev_wdog_val = -1;
			}
			else
			{
#if defined(__TOUCH_DEBUG__)
				printk("[TSP] %s, No problem of tsp driver \n", __func__);
#endif			

				prev_wdog_val = (wdog_val[0] & 0xFC) >> 2;
			}			
			esd_conter=0;			
		}
		else//if(ret < 0)
		{
			if(esd_conter==1)
			{
				disable_irq(ts_global->client->irq);
				tsp_reset();
				enable_irq(ts_global->client->irq);				
				printk("[TSP]  %s : tsp_reset() done!\n",__func__);
				esd_conter=0;
			}
			else
			{
				esd_conter++;
#if defined(__TOUCH_DEBUG__)
				printk("[TSP]  %s : esd_conter [%d]\n",__func__, esd_conter);
#endif				
			}
		}

		if( pre_ta_stat != vbus_state )
		{
			set_tsp_for_ta_detect(vbus_state);
		}

	}
	else
	{
#if defined(__TOUCH_DEBUG__)
		printk("[TSP] %s cannot check ESD\n",__func__);
#endif	
	}
}

static enum hrtimer_restart watchdog_timer_func(struct hrtimer *timer)
{
	//printk("[TSP] %s, %d\n", __func__, __LINE__ );

	queue_work(check_ic_wq, &ts_global->esd_recovery_func);
	hrtimer_start(&ts_global->timer, ktime_set(3, 0), HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}

#endif

#ifdef SEC_TSP_FACTORY_TEST
static void set_cmd_result(struct touch_data *info, char *buff, int len)
{
	strncat(info->cmd_result, buff, len);
}

static ssize_t show_close_tsp_test(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, TSP_BUF_SIZE, "%u\n", 0);
}

static void set_default_result(struct touch_data *info)
{
	char delim = ':';

	memset(info->cmd_result, 0x00, ARRAY_SIZE(info->cmd_result));
	memcpy(info->cmd_result, info->cmd, strlen(info->cmd));
	strncat(info->cmd_result, &delim, 1);
}

static void not_support_cmd(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;
	char buff[16] = {0};

	set_default_result(info);
	sprintf(buff, "%s", "NA");
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 4;
	dev_info(&info->client->dev, "%s: \"%s(%d)\"\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
	return;
}

static void fw_update(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	not_support_cmd(info);
}

static void run_raw_node_read(void *device_data) //item 1
{

	printk("[TSP] %s entered. line : %d, \n", __func__,__LINE__);

	struct touch_data *info = (struct touch_data *)device_data;

	int tma140_col_num = NODE_Y_NUM; //0 ~ 7
	int tma140_row_num = NODE_X_NUM;//0 ~ 9

	int  written_bytes = 0 ;	/* & error check */

	uint8_t buf1[2]={0,};
	uint8_t buf2[NODE_NUM]={0,};

	uint16_t ref1[NODE_NUM]={0,};
	uint16_t ref2[NODE_NUM]={0,};
	char buff[TSP_CMD_STR_LEN] = {0};
	u32 max_value = 0, min_value = 0;

	int i,j,k;
	int ret;

	uint8_t i2c_addr;

	uint16_t RAWDATA_MIN = 70;
	uint16_t RAWDATA_MAX = 130;
	uint16_t LIDAC_MIN = 1;
	uint16_t LIDAC_MAX = 30;

	uint8_t test_result = 1;

	tsp_testmode = 1;

	set_default_result(info);


	/////* Raw Value */////
	/////* Enter Raw Data Mode */////
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		buf1[0] = 0x00;//address
		buf1[1] = 0x40;//value
		ret = i2c_master_send(info->client, buf1, 2);

		if (ret >= 0)
			break; // i2c success
	}
	msleep(10);
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		buf1[0] = 0x00;//address
		buf1[1] = 0xC0;//value
		ret = i2c_master_send(info->client, buf1, 2);

		if (ret >= 0)
			break; // i2c success
	}
	msleep(50);

	/////* Read Raw Data */////
	i2c_addr = 0x07;
	tsp_i2c_read( i2c_addr, buf2, sizeof(buf2));

	printk("[TSP] Raw Value : ");
	for(i = 0 ; i < (tma140_col_num * tma140_row_num) ; i++)
	{
		if(i==0)
		{
			min_value=max_value=buf2[i];

		}
		else
		{
			max_value = max(max_value, buf2[i]);
			min_value = min(min_value, buf2[i]);
		}

		printk(" [%d]%3d", i, buf2[i]);
	}
	printk("\n");


	snprintf(buff, sizeof(buff), "%d,%d", min_value, max_value);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));

	/////* Exit Inspection Mode */////
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		buf1[0] = 0x00;//address
		buf1[1] = 0x00;//value
		ret = i2c_master_send(info->client, buf1, 2);	//exit Inspection Mode

		if (ret >= 0)
			break; // i2c success
	}

	mdelay(100);

	tsp_testmode = 0;

#if 0
	/////* Check Result */////
	for(i = 0 ; i < (tma140_col_num * tma140_row_num) ; i++)
	{
		if(ref1[i] < RAWDATA_MIN || ref1[i] > RAWDATA_MAX)
		{
			test_result = 0;
			break;
		}

		if(ref2[i] < LIDAC_MIN || ref2[i] > LIDAC_MAX)
		{
			test_result = 0;
			break;
		}
	}

	printk("[TSP] test_result = %d", test_result);
#endif

}


static void get_fw_ver_bin(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	char buff[16] = {0};

	printk("[TSP] %s, %d\n", __func__, __LINE__ );


	set_default_result(info);
	if (touch_fw_ver == TSP_KERNEL_FW_ID)
	{
        	snprintf(buff, sizeof(buff), "%c%c%.2d%.4d", touch_vendor_id1, touch_vendor_id2, touch_hw_id, touch_fw_ver);
	}
	else	 
	{
		snprintf(buff, sizeof(buff) , "%s", "NG");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = 3;

		dev_info(&info->client->dev, "%s: HW version error: %02x\n", __func__, touch_fw_ver);

		return;
	}

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void get_fw_ver_ic(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	char buff[16] = {0};
	int ver;

	uint8_t i2c_addr = 0x19;
	uint8_t buf_tmp[5] = {0};

	printk("[TSP] %s, %d\n", __func__, __LINE__ );

	set_default_result(info);

    	//msleep(500);
	tsp_i2c_read( i2c_addr, buf_tmp, sizeof(buf_tmp));

	touch_vendor_id1 = buf_tmp[0];
	touch_vendor_id2 = buf_tmp[1];
	touch_hw_id = buf_tmp[3];
	touch_fw_ver = buf_tmp[4];
	printk("[TSP] Vendor=%c%c HW id=%.2d, FW ver=%.4d\n", touch_vendor_id1, touch_vendor_id2, touch_hw_id, touch_fw_ver);
    
	snprintf(buff, sizeof(buff), "%c%c%.2d%.4d", touch_vendor_id1, touch_vendor_id2, touch_hw_id, touch_fw_ver);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}


static void get_threshold(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	printk("[TSP] %s entered. line : %d, \n", __func__,__LINE__);

	uint8_t buf1[2]={0,};
	uint8_t buf2[1]={0,};


	int threshold;

	char buff[TSP_CMD_STR_LEN] = {0};
	u32 max_value = 0, min_value = 0;

	int i,j,k;
	int ret;

	uint8_t i2c_addr;

	tsp_testmode = 1;


	/////* Enter System Information Mode */////
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		buf1[0] = 0x00;//address
		buf1[1] = 0x10;//value
		ret = i2c_master_send(info->client, buf1, 2);

		if (ret >= 0)
			break; // i2c success
	}
	msleep(50);

	/////*  Read Threshold Value */////
	i2c_addr = 0x30;
	tsp_i2c_read( i2c_addr, buf2, sizeof(buf2));

	printk(" [TSP] %d", buf2[0]);

	snprintf(buff, sizeof(buff), "%d", buf2[0]);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));

	/////* Exit System Information Mode */////
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		buf1[0] = 0x00;//address
		buf1[1] = 0x00;//value
		ret = i2c_master_send(info->client, buf1, 2);	//exit Inspection Mode

		if (ret >= 0)
			break; // i2c success
	}

	msleep(100);

	tsp_testmode = 0;

}

static void run_global_idac_read(void *device_data)
{
	printk("[TSP] %s entered. line : %d, \n", __func__,__LINE__);

	struct touch_data *info = (struct touch_data *)device_data;

	set_default_result(info);

	int tma140_col_num = NODE_Y_NUM;
	int tma140_row_num = NODE_X_NUM;

	uint8_t buf1[2]={0,};
	//uint8_t buf2[95]={0,}; // 0 ~ 5 , 6 ~ 85
	uint8_t buf2[6+NODE_X_NUM*NODE_Y_NUM]={0,}; // 0 ~ 5 , 6 ~ 85	
	uint8_t odd_even_detect;

	int i,j,k;
	int ret;

	uint8_t i2c_addr;

	uint16_t RAWDATA_MIN = 70;
	uint16_t RAWDATA_MAX = 130;
	uint16_t LIDAC_MIN = 1;
	uint16_t LIDAC_MAX = 30;

	uint8_t test_result = 1;

	tsp_testmode = 1;

	/////* Global IDAC Value */////
	/////* Enter Local IDAC Data Mode */////

	for (i = 0; i < 10; i++)
	{
		for (j = 0; j < I2C_RETRY_CNT; j++)
		{
			buf1[0] = 0x00;//address
			buf1[1] = 0x60;//value
			ret = i2c_master_send(ts_global->client, buf1, 2);

			if (ret >= 0)
				break; // i2c success
		}
		msleep(400);

		/////* Read Global IDAC Data */////

		i2c_addr = 0x01;

		tsp_i2c_read( i2c_addr, buf2, sizeof(buf2));
		odd_even_detect=buf2[0]>>6;

#if defined(__TOUCH_DEBUG__)
		for (j = 0; j < 6+NODE_X_NUM*NODE_Y_NUM; j++)
			printk("%d ",buf2[j]);

		printk("\n");

		printk("Global IDAC odd_even_detect=%d , buf2[0]=%d\n", odd_even_detect, buf2[0]);
#endif
		if(odd_even_detect%2)
			break;

		msleep(100);			
	}

	printk("[TSP] Global IDAC Value : ");
	for(i = GLOBAL_IDAC_START ; i < GLOBAL_IDAC_START+GLOBAL_IDAC_NUM ; i++)
	{
		j=i-GLOBAL_IDAC_START;
		global_idac[j] = 2*buf2[i];
		printk(" %d", global_idac[j]);
	}
	printk("\n");


	/////* Exit Inspection Mode */////
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{	
		buf1[0] = 0x00;//address
		buf1[1] = 0x00;//value
		ret = i2c_master_send(ts_global->client, buf1, 2);	//exit Inspection Mode

		if (ret >= 0)
			break; // i2c success
	}

	mdelay(100);

	tsp_testmode = 0;

	info->cmd_state = 2;

	/////* Check Result */////
	for(i = GLOBAL_IDAC_START ; i < GLOBAL_IDAC_START+GLOBAL_IDAC_NUM; i++)
	{
		if(buf2[i] < TSP_SPEC_GLOBAL_IDAC_MIN || buf2[i] > TSP_SPEC_GLOBAL_IDAC_MAX)
		{
			test_result = 0;
			break;
		}
	}

	printk("[TSP] test_result = %d", test_result);

}

static void run_local_idac_read(void *device_data)
{
	printk("[TSP] %s entered. line : %d, \n", __func__,__LINE__);

	struct touch_data *info = (struct touch_data *)device_data;

	set_default_result(info);

	int tma140_col_num = NODE_Y_NUM;
	int tma140_row_num = NODE_X_NUM;

	uint8_t buf1[2]={0,};
	//	uint8_t buf2[88]={0,};
	uint8_t buf2[6+NODE_X_NUM*NODE_Y_NUM]={0,}; // 0 ~ 5 , 6 ~ 85

	char buff[TSP_CMD_STR_LEN] = {0};
	u32 max_value = 0, min_value = 0;

	int i,j,k;
	int ret;
	uint8_t odd_even_detect;

	uint8_t i2c_addr;

	uint16_t LIDAC_MIN = 1;
	uint16_t LIDAC_MAX = 30;

	uint8_t test_result = 1;

	tsp_testmode = 1;

	/////* Local IDAC Value */////
	/////* Enter Local IDAC Data Mode */////


	for (i = 0; i < 10; i++)
	{

		for (j = 0; j < I2C_RETRY_CNT; j++)
		{
			buf1[0] = 0x00;//address
			buf1[1] = 0x60;//value
			ret = i2c_master_send(ts_global->client, buf1, 2);

			if (ret >= 0)
				break; // i2c success
		}
		msleep(400);


		/////* Read Local IDAC Data */////

		i2c_addr = 0x01;

		tsp_i2c_read( i2c_addr, buf2, sizeof(buf2));
		odd_even_detect=buf2[0]>>6;

#if defined(__TOUCH_DEBUG__)

		for (j = 0; j < 6+NODE_X_NUM*NODE_Y_NUM; j++)
			printk("%d ", buf2[j]);

		printk("\n");

		printk("Local IDAC, odd_even_detect=%d , buf2[0]=%d\n ", odd_even_detect, buf2[0]);
#endif

		if(odd_even_detect%2 == 0)
			break;

		msleep(100);
	}

	if(i==3 && odd_even_detect%2 )
		printk(" Error get Local IDAC");


	printk("[TSP] Local IDAC Value : ");
	for(i = LOCAL_IDAC_START; i < LOCAL_IDAC_START + (tma140_col_num * tma140_row_num) ; i++)
	{
		j=i-LOCAL_IDAC_START;
		local_idac[j] = buf2[i];
		printk(" %d", local_idac[j]);
	}
	printk("\n");


	/////* Exit Inspection Mode */////
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{	
		buf1[0] = 0x00;//address
		buf1[1] = 0x00;//value
		ret = i2c_master_send(ts_global->client, buf1, 2);	//exit Inspection Mode

		if (ret >= 0)
			break; // i2c success
	}

	mdelay(100);

	tsp_testmode = 0;

	printk("[TSP] Local I-dac Value : ");
	for(i = LOCAL_IDAC_START ; i < LOCAL_IDAC_START + (tma140_col_num * tma140_row_num) ; i++)
	{
		if(i==LOCAL_IDAC_START)
		{
			min_value=max_value=buf2[i];

		}
		else
		{
			max_value = max(max_value, buf2[i]);
			min_value = min(min_value, buf2[i]);
		}

	//	raw_count[i] = buf2[i];
		printk(" [%d]%3d", i, buf2[i]);
	}
	printk("\n");

	snprintf(buff, sizeof(buff), "%d,%d", min_value, max_value);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));

#if 0
	/////* Check Result */////
	for(i = LOCAL_IDAC_START ; i <LOCAL_IDAC_START + (tma140_col_num * tma140_row_num); i++)
	{
		if(buf2[i] < TSP_SPEC_LOCAL_IDAC_MIN || buf2[i] > TSP_SPEC_LOCAL_IDAC_MAX)
		{
			test_result = 0;
			break;
		}
	}
#endif
	if(min_value < TSP_SPEC_LOCAL_IDAC_MIN || max_value > TSP_SPEC_LOCAL_IDAC_MAX)
	{
		test_result = 0;	
	}

	printk("[TSP] test_result = %d", test_result);
}

static void module_off_master(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	not_support_cmd(info);
}

static void module_on_master(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	not_support_cmd(info);
}

static void module_off_slave(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	not_support_cmd(info);
}

static void module_on_slave(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	not_support_cmd(info);
}

static void get_chip_vendor(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	char buff[16] = {0};

	printk("[TSP] %s, %d\n", __func__, __LINE__ );

	set_default_result(info);

	snprintf(buff, sizeof(buff), "%s", "CYPRESS");
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void get_chip_name(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	char buff[16] = {0};

	printk("[TSP] %s, %d\n", __func__, __LINE__ );

	set_default_result(info);

	snprintf(buff, sizeof(buff), "%s", "TMA140");
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static int check_rx_tx_num(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	char buff[TSP_CMD_STR_LEN] = {0};
	int node;

	if (info->cmd_param[0] < 0 ||
			info->cmd_param[0] >= NODE_X_NUM  ||
			info->cmd_param[1] < 0 ||
			info->cmd_param[1] >= NODE_Y_NUM)
	{
		snprintf(buff, sizeof(buff) , "%s", "NG");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = 3;

		dev_info(&info->client->dev, "%s: parameter error: %u,%u\n",
				__func__, info->cmd_param[0],
				info->cmd_param[1]);
		node = -1;
		return node;
	}
	//node = info->cmd_param[1] * NODE_Y_NUM + info->cmd_param[0];
	node = info->cmd_param[0] * NODE_Y_NUM + info->cmd_param[1];
	//node = info->cmd_param[1] * NODE_X_NUM + info->cmd_param[0];
	dev_info(&info->client->dev, "%s: node = %d\n", __func__,
			node);
	return node;

}


static int global_value_check(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	char buff[TSP_CMD_STR_LEN] = {0};
	int value = -1;

	if (info->cmd_param[0] < 0 ||
		info->cmd_param[0] >= NODE_X_NUM  ||
		info->cmd_param[1] < 0 ||
		info->cmd_param[1] >= NODE_Y_NUM) 
	{
		snprintf(buff, sizeof(buff) , "%s", "NG");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = 3;

		dev_info(&info->client->dev, "%s: parameter error: %u,%u\n",
			__func__, info->cmd_param[0],
			info->cmd_param[1]);

		return value;
	}

	value = info->cmd_param[0];
	dev_info(&info->client->dev, "%s: global value = %d\n", __func__,
		value);
	return value;

}



static void get_reference(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	not_support_cmd(info);
}

static void get_raw_count(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	char buff[16] = {0};
	unsigned int val;
	int node;

	set_default_result(info);
	node = check_rx_tx_num(info);

	if (node < 0)
		return;

	val = raw_count[node];
	snprintf(buff, sizeof(buff), "%u", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__, buff,
			strnlen(buff, sizeof(buff)));
}

static void get_difference(void *device_data) // item2
{
	struct touch_data *info = (struct touch_data *)device_data;

	char buff[16] = {0};
	unsigned int val;
	int node;

	set_default_result(info);
	node = check_rx_tx_num(info);

	if (node < 0)
		return;

	val = difference[node];
	snprintf(buff, sizeof(buff), "%u", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__, buff,
			strnlen(buff, sizeof(buff)));
}

static void get_intensity(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	not_support_cmd(info);
}


static void get_local_idac(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	char buff[16] = {0};
	unsigned int val;
	int node;

	set_default_result(info);
	node = check_rx_tx_num(info);

	if (node < 0)
		return;

	val = local_idac[node];
	snprintf(buff, sizeof(buff), "%u", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__, buff,
			strnlen(buff, sizeof(buff)));

}


static void get_global_idac(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	char buff[16] = {0};
	unsigned int val;

	int x_channel_value=0;

#if 0
	int node=0;
	set_default_result(info);
	node = check_rx_tx_num(info);

	if (node < 0)
		return;

	val = global_idac[node];
#else
	set_default_result(info);

	x_channel_value=global_value_check(info);

#endif

	val = global_idac[x_channel_value];

	printk("global_idac[%d]=%d\n",x_channel_value, val );

	snprintf(buff, sizeof(buff), "%u", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__, buff,
			strnlen(buff, sizeof(buff)));


}

static void get_x_num(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	char buff[16] = {0};
	int ver;

	printk("[TSP] %s, x channel=%d\n", __func__ , NODE_X_NUM);

	set_default_result(info);

	snprintf(buff, sizeof(buff), "%d", NODE_X_NUM);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void get_y_num(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	char buff[16] = {0};
	int ver;

	printk("[TSP] %s, y channel=%d\n", __func__, NODE_Y_NUM);

	set_default_result(info);

	snprintf(buff, sizeof(buff), "%d", NODE_Y_NUM);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void run_reference_read(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	not_support_cmd(info);

	/*	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__); */
}

static void run_raw_count_read(void *device_data) //item 1
{
	printk("[TSP] %s entered. line : %d, \n", __func__,__LINE__);

	struct touch_data *info = (struct touch_data *)device_data;

	int tma140_col_num = NODE_Y_NUM; //0 ~ 7
	int tma140_row_num = NODE_X_NUM;//0 ~ 9

	uint8_t buf1[2]={0,};
	uint8_t buf2[NODE_NUM]={0,};

	char buff[TSP_CMD_STR_LEN] = {0};
	u32 max_value = 0, min_value = 0;

	int i,j,k;
	int ret;

	uint8_t i2c_addr;

	uint8_t test_result = 1;

	tsp_testmode = 1;

	set_default_result(info);


	/////* Raw Value */////
	/////* Enter Raw Data Mode */////
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		buf1[0] = 0x00;//address
		buf1[1] = 0x40;//value
		ret = i2c_master_send(info->client, buf1, 2);

		if (ret >= 0)
			break; // i2c success
	}
	msleep(10);
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		buf1[0] = 0x00;//address
		buf1[1] = 0xC0;//value
		ret = i2c_master_send(info->client, buf1, 2);

		if (ret >= 0)
			break; // i2c success
	}
	msleep(50);

	/////* Read Raw Data */////
	i2c_addr = 0x07;
	tsp_i2c_read( i2c_addr, buf2, sizeof(buf2));

	printk("[TSP] Raw Count Value : ");
	for(i = 0 ; i < (tma140_col_num * tma140_row_num) ; i++)
	{
		if(i==0)
		{
			min_value=max_value=buf2[i];

		}
		else
		{
			max_value = max(max_value, buf2[i]);
			min_value = min(min_value, buf2[i]);
		}

		raw_count[i] = buf2[i];
		printk(" [%d]%3d", i, buf2[i]);
	}
	printk("\n");


	snprintf(buff, sizeof(buff), "%d,%d", min_value, max_value);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));

	/////* Exit Inspection Mode */////
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		buf1[0] = 0x00;//address
		buf1[1] = 0x00;//value
		ret = i2c_master_send(info->client, buf1, 2);	//exit Inspection Mode

		if (ret >= 0)
			break; // i2c success
	}

	mdelay(100);

	tsp_testmode = 0;

	/////* Check Result */////
	for(i = 0 ; i < (tma140_col_num * tma140_row_num) ; i++)
	{
		if(buf2[i] < TSP_SPEC_RAW_COUNT_MIN || buf2[i] > TSP_SPEC_RAW_COUNT_MAX)
		{
			test_result = 0;
			break;
		}
	}

	printk("[TSP] test_result = %d", test_result);

}

static void run_difference_read(void *device_data) //item2
{
	printk("[TSP] %s entered. line : %d, \n", __func__,__LINE__);

	struct touch_data *info = (struct touch_data *)device_data;

	set_default_result(info);

	int tma140_col_num = NODE_Y_NUM; //0 ~ 7
	int tma140_row_num = NODE_X_NUM;//0 ~ 9

	uint8_t buf1[2]={0,};
	uint8_t buf2[NODE_NUM]={0,};

	uint8_t test_result = 1;

	int i,j,k;
	int ret;

	uint8_t i2c_addr;

	tsp_testmode = 1;

	/////* Difference Value */////
	/////* Enter Difference Data Mode */////
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		buf1[0] = 0x00;//address
		buf1[1] = 0x50;//value
		ret = i2c_master_send(ts_global->client, buf1, 2);

		if (ret >= 0)
			break; // i2c success
	}
	msleep(10);
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		buf1[0] = 0x00;//address
		buf1[1] = 0xD0;//value
		ret = i2c_master_send(ts_global->client, buf1, 2);

		if (ret >= 0)
			break; // i2c success
	}
	msleep(50);

	/////* Read Difference Data */////
	i2c_addr = 0x07;
	tsp_i2c_read( i2c_addr, buf2, sizeof(buf2));

	for(i = 0 ; i < (tma140_col_num * tma140_row_num) ; i++)
	{
		difference[i] = buf2[i];
	}


	/////* Exit Inspection Mode */////
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{	
		buf1[0] = 0x00;//address
		buf1[1] = 0x00;//value
		ret = i2c_master_send(ts_global->client, buf1, 2);	//exit Inspection Mode

		if (ret >= 0)
			break; // i2c success
	}

	mdelay(100);

	tsp_testmode = 0;

	info->cmd_state = 2;

	/////* Check Result */////
	for(i = 0 ; i < (tma140_col_num * tma140_row_num) ; i++)
	{
		if(buf2[i] < TSP_SPEC_DIFFERENCE_MIN || buf2[i] > TSP_SPEC_DIFFERENCE_MAX)
		{
			test_result = 0;
			break;
		}
	}

	printk("[TSP] test_result = %d", test_result);

}

static void run_intensity_read(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	not_support_cmd(info);
}

static ssize_t store_cmd(struct device *dev, struct device_attribute
		*devattr, const char *buf, size_t count)
{
	struct touch_data *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;

	char *cur, *start, *end;
	char buff[TSP_CMD_STR_LEN] = {0};
	int len, i;
	struct tsp_cmd *tsp_cmd_ptr = NULL;
	char delim = ',';
	bool cmd_found = false;
	int param_cnt = 0;
	int ret;

	if (info->cmd_is_running == true) {
		dev_err(&info->client->dev, "tsp_cmd: other cmd is running.\n");
		goto err_out;
	}


	/* check lock  */
	mutex_lock(&info->cmd_lock);
	info->cmd_is_running = true;
	mutex_unlock(&info->cmd_lock);

	info->cmd_state = 1;

	for (i = 0; i < ARRAY_SIZE(info->cmd_param); i++)
		info->cmd_param[i] = 0;

	len = (int)count;
	if (*(buf + len - 1) == '\n')
		len--;
	memset(info->cmd, 0x00, ARRAY_SIZE(info->cmd));
	memcpy(info->cmd, buf, len);

	cur = strchr(buf, (int)delim);
	if (cur)
		memcpy(buff, buf, cur - buf);
	else
		memcpy(buff, buf, len);

	/* find command */
	list_for_each_entry(tsp_cmd_ptr, &info->cmd_list_head, list) {
		if (!strcmp(buff, tsp_cmd_ptr->cmd_name)) {
			cmd_found = true;
			break;
		}
	}

	/* set not_support_cmd */
	if (!cmd_found) {
		list_for_each_entry(tsp_cmd_ptr, &info->cmd_list_head, list) {
			if (!strcmp("not_support_cmd", tsp_cmd_ptr->cmd_name))
				break;
		}
	}

	/* parsing parameters */
	if (cur && cmd_found) {
		cur++;
		start = cur;
		memset(buff, 0x00, ARRAY_SIZE(buff));
		do {
			if (*cur == delim || cur - buf == len) {
				end = cur;
				memcpy(buff, start, end - start);
				*(buff + strlen(buff)) = '\0';
				ret = kstrtoint(buff, 10,\
						info->cmd_param + param_cnt);
				start = cur + 1;
				memset(buff, 0x00, ARRAY_SIZE(buff));
				param_cnt++;
			}
			cur++;
		} while (cur - buf <= len);
	}

	dev_info(&client->dev, "cmd = %s\n", tsp_cmd_ptr->cmd_name);
	for (i = 0; i < param_cnt; i++)
		dev_info(&client->dev, "cmd param %d= %d\n", i,
				info->cmd_param[i]);

	tsp_cmd_ptr->cmd_func(info);


	err_out:
	return count;
}

static ssize_t show_cmd_status(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	struct touch_data *info = dev_get_drvdata(dev);
	char buff[16] = {0};

	dev_info(&info->client->dev, "tsp cmd: status:%d\n",
			info->cmd_state);

	if (info->cmd_state == 0)
		snprintf(buff, sizeof(buff), "WAITING");

	else if (info->cmd_state == 1)
		snprintf(buff, sizeof(buff), "RUNNING");

	else if (info->cmd_state == 2)
		snprintf(buff, sizeof(buff), "OK");

	else if (info->cmd_state == 3)
		snprintf(buff, sizeof(buff), "FAIL");

	else if (info->cmd_state == 4)
		snprintf(buff, sizeof(buff), "NOT_APPLICABLE");

	return snprintf(buf, TSP_BUF_SIZE, "%s\n", buff);
}

static ssize_t show_cmd_result(struct device *dev, struct device_attribute
		*devattr, char *buf)
{
	struct touch_data *info = dev_get_drvdata(dev);

	dev_info(&info->client->dev, "tsp cmd: result: %s\n", info->cmd_result);

	mutex_lock(&info->cmd_lock);
	info->cmd_is_running = false;
	mutex_unlock(&info->cmd_lock);

	info->cmd_state = 0;

	return snprintf(buf, TSP_BUF_SIZE, "%s\n", info->cmd_result);
}


static DEVICE_ATTR(close_tsp_test, S_IRUGO, show_close_tsp_test, NULL);
static DEVICE_ATTR(cmd, S_IWUSR | S_IWGRP, NULL, store_cmd);
static DEVICE_ATTR(cmd_status, S_IRUGO, show_cmd_status, NULL);
static DEVICE_ATTR(cmd_result, S_IRUGO, show_cmd_result, NULL);


static struct attribute *sec_touch_facotry_attributes[] = {
		&dev_attr_close_tsp_test.attr,
		&dev_attr_cmd.attr,
		&dev_attr_cmd_status.attr,
		&dev_attr_cmd_result.attr,
		NULL,
};

static struct attribute_group sec_touch_factory_attr_group = {
		.attrs = sec_touch_facotry_attributes,
};
#endif /* SEC_TSP_FACTORY_TEST */


extern struct class *sec_class;
static int ts_probe(
		struct i2c_client *client, const struct i2c_device_id *id)
{
      const struct tma_platform_data *pdata = client->dev.platform_data;
	struct touch_data *ts;

	uint8_t buf_tmp[5]={0};
	uint8_t addr[1];	
	int i;
	int ret = 0, key = 0;
	int phone_ver;

#ifdef SEC_TSP_FACTORY_TEST
	struct device *fac_dev_ts;
#endif


	printk("[TSP] %s, %d\n", __func__, __LINE__ );
    
	//	touch_ctrl_regulator(TOUCH_ON);
	//	msleep(100);	
	//touch_ctrl_regulator(TOUCH_OFF);
	//msleep(200);
	touch_ctrl_regulator(TOUCH_ON);
	msleep(250);				// for reading correct IC Vendor ID

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->pdata = pdata;
	if (ts->pdata->exit_flag) {
		dev_err(&client->dev, "exit flag is setted(hats mode)\n",
			ts->pdata->exit_flag);
		goto exit_flag_set;
	}

#if defined (TSP_EDS_RECOVERY)
	INIT_WORK(&ts->esd_recovery_func, check_ic_work_func);
#endif

	ts->client = client;
	i2c_set_clientdata(client, ts);

	ts_global = ts;

	tsp_irq=client->irq;

	printk("[TSP] tsp_irq = %d\n",tsp_irq);

#if 0	//force fw update code
	msleep(1000);

	//temporary code during development
	printk("[TSP] %s, ln:%d, TSP Force Update start!!!\n", __func__,__LINE__);
	touch_fw_ver = 1;

	if(firm_update_callfc() <= 0)
	{
		printk("[TSP] %s, ln:%d, FW update fail!!",__func__,__LINE__);
	}
	else
	{
		printk("[TSP] FW update done!");
		
		msleep(2000);	

		/* check Version */
		if(read_firmware_version(touch_fw_ver, buf_tmp) > 0)
		{
			tma_kmsg("version read SUCCESS!!");
		}
		else
		{
			tma_kmsg("version read FAIL!!");
		}
	}	

#endif	
        
#if 1
		/* Check point - i2c check - start */	
		for (i = 0; i < 2; i++)
		{
			printk("[TSP] %s, %d, send\n", __func__, __LINE__ );
			addr[0] = 0x19; //address

			printk("[TSP] ts_global->client->flags : 0x%2.2x\n", ts_global->client->flags );
			ret = i2c_master_send(ts_global->client, addr, 1);
	
			if (ret >= 0)
			{
				printk("[TSP] %s, %d, receive\n", __func__, __LINE__ );
				ret = i2c_master_recv(ts_global->client, buf_tmp, 5);
				if (ret >= 0)
					break; // i2c success
			}
	
			printk("[TSP] %s, %d, fail\n", __func__, __LINE__ );
		}
#endif
		if(ret >= 0)
		{
			touch_vendor_id1 = buf_tmp[0];
			touch_vendor_id2 = buf_tmp[1];
			touch_hw_id = buf_tmp[3];
			touch_fw_ver = buf_tmp[4];
			printk("[TSP] Vendor=%c%c HW id=%.2d, FW ver=%.4d\n", touch_vendor_id1, touch_vendor_id2, touch_hw_id, touch_fw_ver);
		
#ifdef __TSP_FORCE_UPDATE__ //auto update enable
			if(touch_fw_ver < TSP_KERNEL_FW_ID)
			{
				if(firm_update_callfc() <= 0)
				{
					printk("[TSP] %s, ln:%d, FW update fail!!",__func__,__LINE__);
				}
				else
				{
					printk("[TSP] FW update done!");

					msleep(1000);	

					/* check Version */
					if(read_firmware_version(touch_fw_ver, buf_tmp) > 0)
					{
						tma_kmsg("version read SUCCESS!!");
					}
					else
					{
						tma_kmsg("version read FAIL!!");
					}		
				}				
			}
#endif

		}
		else//if(ret < 0) 
		{
			printk(KERN_ERR "i2c_transfer failed\n");
			printk("[TSP] %s, ln:%d, Failed to register TSP!!!\n\tcheck the i2c line!!!, ret=%d\n", __func__,__LINE__, ret);
			//goto err_check_functionality_failed;

#if 0	//disabled for a case of doing cal in boot time
			i2c_release_client(client); 
			kfree(ts);
			ts = ts_global = NULL;			
			touch_ctrl_regulator(TOUCH_OFF);

			return -ENXIO;
#endif	
		}
		/* Check point - i2c check - end */

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	ts->input_dev->name = "sec_touchscreen";


	ts->input_dev->keybit[BIT_WORD(KEY_POWER)] |= BIT_MASK(KEY_POWER);

	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	//set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(MT_TOOL_FINGER, ts->input_dev->keybit);	// for B type multi touch protocol
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit); /*must be added for ICS*/

	input_mt_init_slots(ts->input_dev, MAX_USING_FINGER_NUM); // for B type multi touch protocol

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, MAX_X, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, MAX_Y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

#ifdef __TOUCHKEY__
	for(key = 0; key < MAX_KEYS ; key++)
		input_set_capability(ts->input_dev, EV_KEY, touchkey_keycodes[key]);

	// for TSK
	for(key = 0; key < MAX_KEYS ; key++)
		touchkey_status[key] = TK_STATUS_RELEASE;
#endif

	/* ts->input_dev->name = ts->keypad_info->name; */
	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk(KERN_ERR "ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	printk("[TSP] %s, irq=%d\n", __func__, client->irq );

	if (client->irq) {
#if USE_THREADED_IRQ
		ret = request_threaded_irq(client->irq, NULL, ts_work_func, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->name, ts);
#else		
		ret = request_irq(client->irq, ts_irq_handler, IRQF_TRIGGER_FALLING, client->name, ts);
#endif

		if (ret == 0)
			ts->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = ts_early_suspend;
	ts->early_suspend.resume = ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	printk(KERN_INFO "ts_probe: Start touchscreen %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

	/* sys fs */

	sec_touchscreen = device_create(sec_class, NULL, 0, ts, "sec_touchscreen");
	if (IS_ERR(sec_touchscreen)) 
	{
		dev_err(&client->dev,"Failed to create device for the sysfs1\n");
		ret = -ENODEV;
	}

	if (device_create_file(sec_touchscreen, &dev_attr_tsp_firm_version_phone) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_firm_version_phone.attr.name);
	if (device_create_file(sec_touchscreen, &dev_attr_tsp_firm_version_panel) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_firm_version_panel.attr.name);
	if (device_create_file(sec_touchscreen, &dev_attr_tsp_firm_update) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_firm_update.attr.name);
	if (device_create_file(sec_touchscreen, &dev_attr_tsp_firm_update_status) < 0)
		pr_err("[TSP] Failed to create device file(%s)!\n", dev_attr_tsp_firm_update_status.attr.name);	
	if (device_create_file(sec_touchscreen, &dev_attr_tsp_threshold) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_threshold.attr.name);
	if (device_create_file(sec_touchscreen, &dev_attr_get_panel_rev) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_get_panel_rev.attr.name);
	
#ifdef __TOUCHKEY__
	sec_touchkey = device_create(sec_class, NULL, 0, ts, "sec_touchkey");
	if (IS_ERR(sec_touchkey)) 
	{
		dev_err(&client->dev,"Failed to create device for the sysfs1\n");
		ret = -ENODEV;
	}

	if (device_create_file(sec_touchkey, &dev_attr_touchkey_menu) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_touchkey_menu.attr.name);
	if (device_create_file(sec_touchkey, &dev_attr_touchkey_back) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_touchkey_back.attr.name);
	if (device_create_file(sec_touchkey, &dev_attr_touch_sensitivity) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_touch_sensitivity.attr.name);
	if (device_create_file(sec_touchkey, &dev_attr_touchkey_threshold) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_touchkey_threshold.attr.name);
#endif

#ifdef SEC_TSP_FACTORY_TEST
	INIT_LIST_HEAD(&ts->cmd_list_head);
	for (i = 0; i < ARRAY_SIZE(tsp_cmds); i++)
		list_add_tail(&tsp_cmds[i].list, &ts->cmd_list_head);

	mutex_init(&ts->cmd_lock);
	ts->cmd_is_running = false;

	fac_dev_ts = device_create(sec_class, NULL, 0, ts, "tsp");
	if (IS_ERR(fac_dev_ts))
		dev_err(&client->dev, "Failed to create device for the sysfs\n");

	ret = sysfs_create_group(&fac_dev_ts->kobj,	&sec_touch_factory_attr_group);
	if (ret)
		dev_err(&client->dev, "Failed to create sysfs group\n");
#endif

#if defined (TSP_EDS_RECOVERY)
	touch_present = 1;
	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = watchdog_timer_func;
	hrtimer_start(&ts->timer, ktime_set(3, 0), HRTIMER_MODE_REL);
#endif


	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
exit_flag_set:    
	kfree(ts);
	ts = ts_global = NULL;
err_alloc_data_failed:
	touch_ctrl_regulator(TOUCH_OFF);
err_check_functionality_failed:
	return ret;
}

static int ts_remove(struct i2c_client *client)
{
	struct touch_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct touch_data *ts = i2c_get_clientdata(client);

	printk("[TSP] %s+\n", __func__ );

	tsp_status=1; 

	if (ts->use_irq)
	{
		disable_irq(client->irq);
	}

#if defined (TSP_EDS_RECOVERY)
	if(touch_present == 1)
		hrtimer_cancel(&ts->timer);
#endif		

	gpio_direction_output( TSP_INT , 1 );
	gpio_direction_output( TSP_SCL , 1 ); 
	gpio_direction_output( TSP_SDA , 1 ); 

	msleep(20);	

	touch_ctrl_regulator(TOUCH_OFF);
	force_release_touch();
	printk("[TSP] %s-\n", __func__ );

	return 0;
}

static int ts_resume(struct i2c_client *client)
{
	int ret;
#if defined (TSP_EDS_RECOVERY)
	struct touch_data *ts = i2c_get_clientdata(client);
#endif	
	uint8_t i2c_addr = 0x19;
	uint8_t buf[5];


	gpio_direction_output( TSP_SCL , 1 ); 
	gpio_direction_output( TSP_SDA , 1 ); 

	gpio_direction_input(TSP_INT);


	touch_ctrl_regulator(TOUCH_ON);

	msleep(250);
/*
	ret = tsp_i2c_read( i2c_addr, buf, sizeof(buf));

	touch_vendor_id1 = buf[0];
	touch_vendor_id2 = buf[1];
	touch_hw_id = buf[3];
	touch_fw_ver = buf[4];
	printk("[TSP] Vendor=%c%c HW id=%.2d, FW ver=%.4d\n", touch_vendor_id1, touch_vendor_id2, touch_hw_id, touch_fw_ver);
*/
	enable_irq(client->irq);

	if(vbus_state == 1)
	{
		set_tsp_for_ta_detect(vbus_state);
	}

#if defined (TSP_EDS_RECOVERY)

	prev_wdog_val = -1;

	if(touch_present == 1)
		hrtimer_start(&ts->timer, ktime_set(3, 0), HRTIMER_MODE_REL);
#endif	

	tsp_status=0; 

	printk("[TSP] %s-\n", __func__ );
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ts_early_suspend(struct early_suspend *h)
{
	struct touch_data *ts;
	ts = container_of(h, struct touch_data, early_suspend);
	ts_suspend(ts->client, PMSG_SUSPEND);
}

static void ts_late_resume(struct early_suspend *h)
{
	struct touch_data *ts;
	ts = container_of(h, struct touch_data, early_suspend);
	ts_resume(ts->client);
}
#endif

static const struct i2c_device_id ts_id[] = {
		{ "cypress-tma140", 0 },
		{ }
};

static struct i2c_driver ts_driver = {
		.probe		= ts_probe,
		.remove		= ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
		.suspend	= ts_suspend,
		.resume		= ts_resume,
#endif
		.id_table	= ts_id,
		.driver = {
				.name	= "cypress-tma140",
		},
};

static int __devinit tsp_driver_init(void)
{

	printk("[TSP] %s\n", __func__ );

	gpio_request(TSP_INT, "ts_irq");
	gpio_direction_input(TSP_INT);

	irq_set_irq_type(gpio_to_irq(TSP_INT), IRQ_TYPE_EDGE_FALLING);
        
	gpio_request(TSP_SCL, "ts_scl");
	gpio_direction_output( TSP_SCL , 1 ); 
	gpio_request(TSP_SDA, "ts_sda");
	gpio_direction_output( TSP_SDA , 1 ); 		

#if defined (TSP_EDS_RECOVERY)
	check_ic_wq = create_singlethread_workqueue("check_ic_wq");	

	if (!check_ic_wq)
		return -ENOMEM;
#endif

	return i2c_add_driver(&ts_driver);
}

static void __exit tsp_driver_exit(void)
{
	if (touch_regulator) 
	{
		regulator_put(touch_regulator);
		touch_regulator = NULL;
	}
#if defined (__TOUCH_1V8__)
	if (touch_1v8_regulator) 
	{
		regulator_put(touch_1v8_regulator);
		touch_1v8_regulator = NULL;
	}
#endif
	i2c_del_driver(&ts_driver);


#if defined (TSP_EDS_RECOVERY)
	if (check_ic_wq)
		destroy_workqueue(check_ic_wq);	
#endif

}

static ssize_t read_threshold(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t buf1[2]={0,};
	uint8_t buf2[1]={0,};

	int threshold;

	char buff[TSP_CMD_STR_LEN] = {0};
	u32 max_value = 0, min_value = 0;

	int i,j,k;
	int ret;

	uint8_t i2c_addr;

	tsp_testmode = 1;

	/////* Enter System Information Mode */////
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		buf1[0] = 0x00;//address
		buf1[1] = 0x10;//value
		ret = i2c_master_send(ts_global->client, buf1, 2);

		if (ret >= 0)
			break; // i2c success
	}
	msleep(50);

	/////*  Read Threshold Value */////
	i2c_addr = 0x30;
	tsp_i2c_read( i2c_addr, buf2, sizeof(buf2));

	printk(" [TSP] %d", buf2[0]);

	/////* Exit System Information Mode */////
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		buf1[0] = 0x00;//address
		buf1[1] = 0x00;//value
		ret = i2c_master_send(ts_global->client, buf1, 2);	//exit Inspection Mode

		if (ret >= 0)
			break; // i2c success
	}

	msleep(100);

	tsp_testmode = 0;

	return sprintf(buf, "%d\n",  buf2[0]);
}


static ssize_t firmware_In_Binary(struct device *dev, struct device_attribute *attr, char *buf)
{

	int phone_ver=0;

     if(touch_fw_ver != TSP_KERNEL_FW_ID)
     {
        phone_ver = touch_fw_ver;
        return sprintf(buf, "%d\n",  touch_fw_ver);
     }
     
	return sprintf(buf, "%c%c%.2d%.4d", touch_vendor_id1, touch_vendor_id2, touch_hw_id, touch_fw_ver);
}


static ssize_t firmware_In_TSP(struct device *dev, struct device_attribute *attr, char *buf)
{

	uint8_t i2c_addr = 0x19;
	uint8_t buf_tmp[5] = {0};
	int phone_ver = 0;

	printk("[TSP] %s\n",__func__);

	tsp_i2c_read( i2c_addr, buf_tmp, sizeof(buf_tmp));

	touch_vendor_id1 = buf_tmp[0];
	touch_vendor_id2 = buf_tmp[1];
	touch_hw_id = buf_tmp[3];
	touch_fw_ver = buf_tmp[4];
	printk("[TSP] Vendor=%c%c HW id=%.2d, FW ver=%.4d\n", touch_vendor_id1, touch_vendor_id2, touch_hw_id, touch_fw_ver);

	return sprintf(buf, "%c%c%.2d%.4d", touch_vendor_id1, touch_vendor_id2, touch_hw_id, touch_fw_ver);
}


static ssize_t tsp_panel_rev(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t i2c_addr = 0x19;
	uint8_t buf_tmp[5] = {0};
	int phone_ver = 0;

	printk("[TSP] %s\n",__func__);

	tsp_i2c_read( i2c_addr, buf_tmp, sizeof(buf_tmp));

	touch_vendor_id1 = buf_tmp[0];
	touch_vendor_id2 = buf_tmp[1];
	touch_hw_id = buf_tmp[3];
	touch_fw_ver = buf_tmp[4];

	return sprintf(buf, "%c%c%.2d%.4d", touch_vendor_id1, touch_vendor_id2, touch_hw_id, touch_fw_ver);
}

#ifdef __TOUCHKEY__
static ssize_t touchkey_sensitivity_power_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size)
{
	int ret = 0;
	uint8_t i2c_addr;
	uint8_t buf1[2]={0,};
	uint8_t buf2[1]={0,};
	int i=0;

	printk("[TSP][%s]++\n",__func__);

	tsp_testmode = 1;

	if( ( !strcmp(buf, "1") || !strcmp(buf, "1\n") ) && (0 == touchkey_scan_mode))
	{
		printk("[TSP] %s, touchkey_sensitivity_power ON !!! \n",__func__);		

		/////* Enter System Information Mode */////
		for (i = 0; i < I2C_RETRY_CNT; i++)
		{
			buf1[0] = 0x00;//address
			buf1[1] = 0x10;//value
			ret = i2c_master_send(ts_global->client, buf1, 2);

			if (ret >= 0){
				break; // i2c success				
			}
		}
		msleep(200);

		/////*  set CMD */////
		for (i = 0; i < I2C_RETRY_CNT; i++)
		{
			buf1[0] = 0x01;//address
			buf1[1] = 0x1F;//value
			ret = i2c_master_send(ts_global->client, buf1, 2);

			if (ret >= 0){
				break; // i2c success
			}
		}
		msleep(200);

		/////* Exit System Information Mode */////
		for (i = 0; i < I2C_RETRY_CNT; i++)
		{
			buf1[0] = 0x00;//address
			buf1[1] = 0x00;//value
			ret = i2c_master_send(ts_global->client, buf1, 2);	//exit Inspection Mode

			if (ret >= 0){
				break; // i2c success
			}
		}

		msleep(1125);

		touchkey_scan_mode = 1;
	}
	else if( ( !strcmp(buf, "0") || !strcmp(buf, "0\n") ) && (1==touchkey_scan_mode))
	{
		printk("[TSP] %s, touchkey_sensitivity_power OFF !!! \n",__func__);		

		disable_irq(ts_global->client->irq);
		force_release_key();
		tsp_reset();
		enable_irq(ts_global->client->irq);	

		touchkey_scan_mode = 0;
	}

	tsp_testmode = 0;	

	printk("[TSP][%s]--\n",__func__);

	//return sprintf(buf, "%d\n",  touchkey_scan_mode);
	return size;
}

static ssize_t read_touchkey_threshold(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n",  touchkey_threshold_value);
}


static ssize_t menu_sensitivity_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u16 local_menu_sensitivity;
	uint8_t i2c_addr;
	uint8_t buf2[1]={0,};

	printk("[TSP] %s, menu_sensitivity start !!! \n",__func__);

	i2c_addr = 0x17;
	tsp_i2c_read( i2c_addr, buf2, sizeof(buf2));

	printk("%s, menu_sensitivity : %d %x!!! \n",__func__,buf2[0],buf2[0]);
	local_menu_sensitivity = buf2[0];

	return sprintf(buf, "%d\n",  local_menu_sensitivity);
}

static ssize_t back_sensitivity_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u16 local_back_sensitivity;
	uint8_t i2c_addr;
	uint8_t buf2[1]={0,};

	printk("[TSP] %s, back_sensitivity start !!! \n",__func__);

	i2c_addr = 0x18;
	tsp_i2c_read( i2c_addr, buf2, sizeof(buf2));

	printk("%s, back_sensitivity : %d %x!!! \n",__func__,buf2[0],buf2[0]);
	local_back_sensitivity = buf2[0];

	return sprintf(buf, "%d\n",  local_back_sensitivity);
}
#endif


#define TMA140_RET_SUCCESS 0x00
int sv_tch_firmware_update = 0;

static int read_firmware_version(unsigned char hw_ver_backup, uint8_t *buf_tmp)
{
	uint8_t addr[1];
	int i;
	int ret;

	for (i = 0; i < 2; i++)
	{
		printk("[TSP] %s, %d, send\n", __func__, __LINE__ );
		addr[0] = 0x19; //address
		ret = i2c_master_send(ts_global->client, addr, 1);

		if (ret >= 0)
		{
			printk("[TSP] %s, %d, receive\n", __func__, __LINE__ );
			ret = i2c_master_recv(ts_global->client, buf_tmp, 5);
			if (ret >= 0)
				break; // i2c success
		}

		printk("[TSP] %s, %d, fail\n", __func__, __LINE__ );
	}

	if(ret >= 0)
	{
		touch_vendor_id1 = buf_tmp[0];
		touch_vendor_id2 = buf_tmp[1];
		touch_hw_id = buf_tmp[3];
		touch_fw_ver = buf_tmp[4];
		printk("[TSP] Vendor=%c%c HW id=%.2d, FW ver=%.4d\n", touch_vendor_id1, touch_vendor_id2, touch_hw_id, touch_fw_ver);

		if(hw_ver_backup == touch_fw_ver)
		{
			printk("[TSP] %s:%d,Version is binary involved version!!\n", __func__,__LINE__);
			return 1;
		}
	}

	printk("[TSP] %s:%d,I2c Fail ret = %d !!\n", __func__,__LINE__, ret);

	return -1;

}

static int firm_update_callfc(void)
{
	uint8_t update_num;

	disable_irq(tsp_irq);
	local_irq_disable();	
	printk("[TSP] disable_irq : %d\n", __LINE__ );
#ifdef TSP_EDS_RECOVERY		
	if(touch_present == 1)
	{
		cancel_work_sync(&ts_global->esd_recovery_func);
		hrtimer_cancel(&ts_global->timer);
	}
#endif
	while( bRun_set_tsp_for_ta_detect )
	{
		msleep(10);
	}
	
	for(update_num = 1; update_num <= 5 ; update_num++)
	{
		sv_tch_firmware_update = cypress_update(touch_fw_ver);

		if(sv_tch_firmware_update == TMA140_RET_SUCCESS)
		{
			firmware_ret_val = 1; //SUCCESS
			//additionally need time to cal in TSP for 5 sec
			msleep(3000);
			sprintf(IsfwUpdate,"%s\n",FW_DOWNLOAD_COMPLETE);
			printk( "[TSP] %s, %d : firmware update SUCCESS !!\n", __func__, __LINE__);
			break;
		}
		else
		{
			printk( "[TSP] %s, %d : firmware update RETRY !!\n", __func__, __LINE__);
			if(update_num == 5)
			{
				firmware_ret_val = -1; //FAIL
				sprintf(IsfwUpdate,"%s\n",FW_DOWNLOAD_FAIL);				
				printk( "[TSP] %s, %d : firmware update FAIL !!\n", __func__, __LINE__);
			}
		}
	}

	printk("[TSP] enable_irq : %d\n", __LINE__ );
	local_irq_enable();
	enable_irq(tsp_irq);
#ifdef TSP_EDS_RECOVERY	
	if(touch_present == 1)
	{
		hrtimer_start(&ts_global->timer, ktime_set(0, 200000000), HRTIMER_MODE_REL);
	}
#endif
	return firmware_ret_val;
}


static ssize_t firm_update(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret=0;
	uint8_t buf_tmp[5]={0,};

	printk("[TSP] %s!\n", __func__);

	sprintf(IsfwUpdate,"%s\n",FW_DOWNLOADING);

	now_tspfw_update = 1;	
	ret = firm_update_callfc();
	if(ret <= 0)
	{
		printk("[TSP] %s, ln:%d, FW update fail!!ret = %d, firmware_ret_val = %d ",__func__,__LINE__, ret, firmware_ret_val);

		firmware_ret_val = -1;
	}
	else
	{
		printk("[TSP] FW update done!");


		/* check Version */
		if(read_firmware_version(touch_fw_ver, buf_tmp) > 0)
		{
			tma_kmsg("version read SUCCESS!!");
		}
		else
		{
			tma_kmsg("version read FAIL!!");
		}		
	}				
	now_tspfw_update = 0;	

	return sprintf(buf, "%d", firmware_ret_val );
}


static ssize_t firmware_update_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("[TSP] %s\n",__func__);

	return sprintf(buf, "%s\n", IsfwUpdate);
}


module_init(tsp_driver_init);
module_exit(tsp_driver_exit);

MODULE_AUTHOR("Cypress");
MODULE_DESCRIPTION("TMA140 Touchscreen Driver");
MODULE_LICENSE("GPL");
