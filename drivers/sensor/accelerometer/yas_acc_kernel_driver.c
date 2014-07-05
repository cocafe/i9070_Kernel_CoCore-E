/*
 * Copyright (c) 2010-2011 Yamaha Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/list.h>
#include <linux/regulator/consumer.h>

#include "yas.h"

#if YAS_ACC_DRIVER == YAS_ACC_DRIVER_ADXL345
#include "yas_acc_driver-adxl34x.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_ADXL346
#include "yas_acc_driver-adxl34x.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMA150
#include "yas_acc_driver-bma150.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMA222
#include "yas_acc_driver-bma222.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMA250
#include "yas_acc_driver-bma25x.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMA254
#include "yas_acc_driver-bma25x.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_KXSD9
#include "yas_acc_driver-kxsd9.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_KXTE9
#include "yas_acc_driver-kxte9.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_KXTF9
#include "yas_acc_driver-kxtf9.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_KXUD9
#include "yas_acc_driver-kxud9.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS331DL
#include "yas_acc_driver-lis331dl.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS331DLH
#include "yas_acc_driver-lis331dlh.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS331DLM
#include "yas_acc_driver-lis331dlm.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS3DH
#include "yas_acc_driver-lis3dh.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_MMA8453Q
#include "yas_acc_driver-mma8453q.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_MMA8452Q
#include "yas_acc_driver-mma8452q.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_KXDM
#include "yas_acc_driver-kxdm.c"
#else
#include "yas_acc_driver-none.c"
#endif

#define YAS_ACC_KERNEL_VERSION                                       "4.4.702a"
#define YAS_ACC_KERNEL_NAME                                     "accelerometer"

/* ABS axes parameter range [um/s^2] (for input event) */
#define GRAVITY_EARTH                                                   9806550
#define ABSMAX_2G                                           (GRAVITY_EARTH * 2)
#define ABSMIN_2G                                          (-GRAVITY_EARTH * 2)

#define delay_to_jiffies(d)                     ((d) ? msecs_to_jiffies(d) : 1)
#define actual_delay(d)                 (jiffies_to_msecs(delay_to_jiffies(d)))

/* -------------------------------------------------------------------------- *
 *  Function prototype declaration
 * -------------------------------------------------------------------------- */
static struct yas_acc_private_data *yas_acc_get_data(void);
static void yas_acc_set_data(struct yas_acc_private_data *);

static int yas_acc_lock(void);
static int yas_acc_unlock(void);
static int yas_acc_i2c_open(void);
static int yas_acc_i2c_close(void);
static int yas_acc_i2c_write(uint8_t, const uint8_t *, int);
static int yas_acc_i2c_read(uint8_t, uint8_t *, int);
static void yas_acc_msleep(int);

static int yas_acc_core_driver_init(struct yas_acc_private_data *);
static void yas_acc_core_driver_fini(struct yas_acc_private_data *);
static int yas_acc_get_enable(struct yas_acc_driver *);
static int yas_acc_set_enable(struct yas_acc_driver *, int);
static int yas_acc_get_delay(struct yas_acc_driver *);
static int yas_acc_set_delay(struct yas_acc_driver *, int);
static int yas_acc_get_position(struct yas_acc_driver *);
static int yas_acc_set_position(struct yas_acc_driver *, int);
static int yas_acc_get_threshold(struct yas_acc_driver *);
static int yas_acc_set_threshold(struct yas_acc_driver *, int);
static int yas_acc_get_filter_enable(struct yas_acc_driver *);
static int yas_acc_set_filter_enable(struct yas_acc_driver *, int);
static int yas_acc_measure(struct yas_acc_driver *, struct yas_acc_data *);
static int yas_acc_input_init(struct yas_acc_private_data *);
static void yas_acc_input_fini(struct yas_acc_private_data *);

static ssize_t yas_acc_enable_show(struct device *, struct device_attribute *
				   , char *);
static ssize_t yas_acc_enable_store(struct device *, struct device_attribute *
				    , const char *, size_t);
static ssize_t yas_acc_delay_show(struct device *, struct device_attribute *
				  , char *);
static ssize_t yas_acc_delay_store(struct device *, struct device_attribute *
				   , const char *, size_t);
static ssize_t yas_acc_position_show(struct device *, struct device_attribute *
				     , char *);
static ssize_t yas_acc_position_store(struct device *
				      , struct device_attribute *
				      , const char *, size_t);
static ssize_t yas_acc_threshold_show(struct device *
				      , struct device_attribute *, char *);
static ssize_t yas_acc_threshold_store(struct device *
				       , struct device_attribute *
				       , const char *, size_t);
static ssize_t yas_acc_filter_enable_show(struct device *
					  , struct device_attribute *, char *);
static ssize_t yas_acc_filter_enable_store(struct device *
					   , struct device_attribute *
					   , const char *, size_t);
static ssize_t yas_acc_wake_store(struct device * , struct device_attribute *
				  , const char *, size_t);
static ssize_t yas_acc_private_data_show(struct device *
					 , struct device_attribute *, char *);
#if DEBUG
static ssize_t yas_acc_debug_reg_show(struct device *
				      , struct device_attribute *, char *);
static int yas_acc_suspend(struct i2c_client *, pm_message_t);
static int yas_acc_resume(struct i2c_client *);
#endif

static void yas_acc_work_func(struct work_struct *);
static int yas_acc_probe(struct i2c_client *, const struct i2c_device_id *);
static int yas_acc_remove(struct i2c_client *);
static int yas_acc_suspend(struct i2c_client *, pm_message_t);
static int yas_acc_resume(struct i2c_client *);

/* -------------------------------------------------------------------------- *
 *  Driver private data
 * -------------------------------------------------------------------------- */
struct yas_acc_private_data {
	struct mutex driver_mutex;
	struct mutex data_mutex;
	struct mutex enable_mutex;
	struct i2c_client *client;
	struct input_dev *input;
	struct yas_acc_driver *driver;
	struct delayed_work work;
	struct yas_acc_data last;
	int suspend_enable;
#if DEBUG
	struct mutex suspend_mutex;
	int suspend;
#endif
#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	struct list_head devfile_list;
#endif
};

struct acc_power_data {
	struct regulator *regulator_vdd;
	struct regulator *regulator_vio;
};

static struct acc_power_data acc_power;

static int acc_power_on(void)
{
	int err = 0;

	if (acc_power.regulator_vdd) {
		err = regulator_enable(acc_power.regulator_vdd);
		if (err) {
			pr_err("%s: Couldn't enable VDD %d\n", __func__, err);
			return err;
		}
	}

	if (acc_power.regulator_vio) {
		err = regulator_enable(acc_power.regulator_vio);
		if (err) {
			pr_err("%s: Couldn't enable VIO %d\n", __func__, err);
			return err;
		}
	}

	msleep(20);
	return err;
}

static int acc_power_off(void)
{
	int err = 0;

	if (acc_power.regulator_vdd) {
		err = regulator_disable(acc_power.regulator_vdd);
		if (err) {
			pr_err("%s: Couldn't disable VDD %d\n", __func__, err);
			return err;
		}
	}

	if (acc_power.regulator_vio) {
		err = regulator_disable(acc_power.regulator_vio);
		if (err) {
			pr_err("%s: Couldn't disable VIO %d\n", __func__, err);
			return err;
		}
	}

	return err;
}

static struct yas_acc_private_data *yas_acc_private_data;
static struct yas_acc_private_data *yas_acc_get_data(void)
{
	return yas_acc_private_data;
}
static void yas_acc_set_data(struct yas_acc_private_data *data)
{
	yas_acc_private_data = data;
}

#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE

#include <linux/miscdevice.h>
#define SENSOR_NAME "accelerometer"
#define MAX_COUNT (64)

struct sensor_device {
	struct list_head list;
	struct mutex lock;
	wait_queue_head_t waitq;
	struct input_event events[MAX_COUNT];
	int head, num_event;
};

static void
get_time_stamp(struct timeval *tv)
{
	struct timespec ts;

	ktime_get_ts(&ts);
	tv->tv_sec = ts.tv_sec;
	tv->tv_usec = ts.tv_nsec / 1000;
}

static void
make_event(struct input_event *ev, int type, int code, int value)
{
	struct timeval tv;

	get_time_stamp(&tv);
	ev->type = type;
	ev->code = code;
	ev->value = value;
	ev->time = tv;
}

static void
make_event_w_time(struct input_event *ev, int type, int code, int value
		  , struct timeval *tv)
{
	ev->type = type;
	ev->code = code;
	ev->value = value;
	ev->time = *tv;
}

static void
sensor_enq(struct sensor_device *kdev, struct input_event *ev)
{
	int idx;

	idx = kdev->head + kdev->num_event;
	if (MAX_COUNT <= idx)
		idx -= MAX_COUNT;

	kdev->events[idx] = *ev;
	kdev->num_event++;
	if (MAX_COUNT < kdev->num_event) {
		kdev->num_event = MAX_COUNT;
		kdev->head++;
		if (MAX_COUNT <= kdev->head)
			kdev->head -= MAX_COUNT;
	}
}

static int
sensor_deq(struct sensor_device *kdev, struct input_event *ev)
{
	if (kdev->num_event == 0)
		return 0;

	*ev = kdev->events[kdev->head];
	kdev->num_event--;
	kdev->head++;
	if (MAX_COUNT <= kdev->head)
		kdev->head -= MAX_COUNT;

	return 1;
}

static void
sensor_event(struct list_head *devlist, struct input_event *ev, int num)
{
	struct sensor_device *kdev;
	int i;

	list_for_each_entry(kdev, devlist, list) {
		mutex_lock(&kdev->lock);
		for (i = 0; i < num; i++)
			sensor_enq(kdev, &ev[i]);
		mutex_unlock(&kdev->lock);
		wake_up_interruptible(&kdev->waitq);
	}
}

static ssize_t
sensor_write(struct file *f, const char __user *buf, size_t count, loff_t *pos)
{
	struct yas_acc_private_data *data = yas_acc_get_data();
	struct sensor_device *kdev;
	struct input_event ev[MAX_COUNT];
	int num, i;

	if (count < sizeof(struct input_event))
		return -EINVAL;

	num = count / sizeof(struct input_event);
	if (MAX_COUNT < num)
		num = MAX_COUNT;

	if (copy_from_user(ev, buf, num * sizeof(struct input_event)))
		return -EFAULT;

	list_for_each_entry(kdev, &data->devfile_list, list) {
		mutex_lock(&kdev->lock);
		for (i = 0; i < num; i++)
			sensor_enq(kdev, &ev[i]);
		mutex_unlock(&kdev->lock);
		wake_up_interruptible(&kdev->waitq);
	}

	return count;
}

static ssize_t
sensor_read(struct file *f, char __user *buf, size_t count, loff_t *pos)
{
	struct sensor_device *kdev = f->private_data;
	int rt, num;
	struct input_event ev[MAX_COUNT];

	if (count < sizeof(struct input_event))
		return -EINVAL;

	rt = wait_event_interruptible(kdev->waitq, kdev->num_event != 0);
	if (rt)
		return rt;

	mutex_lock(&kdev->lock);
	for (num = 0; num < count / sizeof(struct input_event); num++) {
		if (!sensor_deq(kdev, &ev[num]))
			break;
	}
	mutex_unlock(&kdev->lock);

	if (copy_to_user(buf, ev, num * sizeof(struct input_event)))
		return -EFAULT;

	return num * sizeof(struct input_event);
}

static unsigned int
sensor_poll(struct file *f, struct poll_table_struct *wait)
{
	struct sensor_device *kdev = f->private_data;

	poll_wait(f, &kdev->waitq, wait);
	if (kdev->num_event != 0)
		return POLLIN | POLLRDNORM;

	return 0;
}

static int sensor_open(struct inode *inode, struct file *f)
{
	struct yas_acc_private_data *data = yas_acc_get_data();
	struct sensor_device *kdev;

	kdev = kzalloc(sizeof(struct sensor_device), GFP_KERNEL);
	if (!kdev)
		return -ENOMEM;

	mutex_init(&kdev->lock);
	init_waitqueue_head(&kdev->waitq);
	f->private_data = kdev;
	kdev->head = 0;
	kdev->num_event = 0;
	list_add(&kdev->list, &data->devfile_list);

	return 0;
}

static int sensor_release(struct inode *inode, struct file *f)
{
	struct sensor_device *kdev = f->private_data;

	list_del(&kdev->list);
	kfree(kdev);

	return 0;
}

static const struct file_operations sensor_fops = {
	.owner = THIS_MODULE,
	.open = sensor_open,
	.release = sensor_release,
	.write = sensor_write,
	.read = sensor_read,
	.poll = sensor_poll,
};

static struct miscdevice sensor_devfile = {
	.name = SENSOR_NAME,
	.fops = &sensor_fops,
	.minor = MISC_DYNAMIC_MINOR,
};

#endif


/* -------------------------------------------------------------------------- *
 *  Accelerlomete core driver callback function
 * -------------------------------------------------------------------------- */
static int yas_acc_lock(void)
{
	struct yas_acc_private_data *data = yas_acc_get_data();

	mutex_lock(&data->driver_mutex);

	return 0;
}

static int yas_acc_unlock(void)
{
	struct yas_acc_private_data *data = yas_acc_get_data();

	mutex_unlock(&data->driver_mutex);

	return 0;
}

static int yas_acc_i2c_open(void)
{
	return 0;
}

static int yas_acc_i2c_close(void)
{
	return 0;
}

static int yas_acc_i2c_write(uint8_t adr, const uint8_t *buf, int len)
{
	struct yas_acc_private_data *data = yas_acc_get_data();
	struct i2c_msg msg[2];
	char buffer[16];
	uint8_t reg;
	int err;
	int i;

	if (len > 15)
		return -1;

	reg = adr;
	buffer[0] = reg;
	for (i = 0; i < len; i++)
		buffer[i+1] = buf[i];

	msg[0].addr = data->client->addr;
	msg[0].flags = 0;
	msg[0].len = len + 1;
	msg[0].buf = buffer;
	err = i2c_transfer(data->client->adapter, msg, 1);
	if (err != 1) {
		dev_err(&data->client->dev,
		"i2c_transfer() write error: s_addr=%02x, r_addr=%02x, err=%d\n"
			, data->client->addr, adr, err);
		return err;
	}

	return 0;
}

static int yas_acc_i2c_read(uint8_t adr, uint8_t *buf, int len)
{
	struct yas_acc_private_data *data = yas_acc_get_data();
	struct i2c_msg msg[2];
	uint8_t reg;
	int err;

	reg = adr;
	msg[0].addr = data->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;
	msg[1].addr = data->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = buf;

	err = i2c_transfer(data->client->adapter, msg, 2);
	if (err != 2) {
		dev_err(&data->client->dev,
		"i2c_transfer() read error: s_addr=%02x, r_addr=%02x, err=%d\n"
			, data->client->addr, adr, err);
		return err;
	}

	return 0;
}

static void yas_acc_msleep(int msec)
{
	msleep(msec);
}

/* -------------------------------------------------------------------------- *
 *  Accelerometer core driver access function
 * -------------------------------------------------------------------------- */
static int yas_acc_core_driver_init(struct yas_acc_private_data *data)
{
	struct yas_acc_driver_callback *cbk;
	struct yas_acc_driver *driver;
	int err;

	data->driver = driver =
		kzalloc(sizeof(struct yas_acc_driver), GFP_KERNEL);

	if (!driver) {
		err = -ENOMEM;
		return err;
	}

	cbk = &driver->callback;
	cbk->lock = yas_acc_lock;
	cbk->unlock = yas_acc_unlock;
	cbk->device_open = yas_acc_i2c_open;
	cbk->device_close = yas_acc_i2c_close;
	cbk->device_write = yas_acc_i2c_write;
	cbk->device_read = yas_acc_i2c_read;
	cbk->msleep = yas_acc_msleep;

	err = yas_acc_driver_init(driver);
	if (err != YAS_NO_ERROR) {
		kfree(driver);
		return err;
	}

	err = driver->init();
	if (err != YAS_NO_ERROR) {
		kfree(driver);
		return err;
	}

	err = driver->set_position(CONFIG_INPUT_YAS_ACCELEROMETER_POSITION);
	if (err != YAS_NO_ERROR) {
		kfree(driver);
		return err;
	}

	return 0;
}

static void yas_acc_core_driver_fini(struct yas_acc_private_data *data)
{
	struct yas_acc_driver *driver = data->driver;

	driver->term();
	kfree(driver);
}

static int yas_acc_get_enable(struct yas_acc_driver *driver)
{
	int enable;

	enable = driver->get_enable();

	return enable;
}

static int yas_acc_set_enable(struct yas_acc_driver *driver, int enable)
{
	struct yas_acc_private_data *data = yas_acc_get_data();
	int delay = driver->get_delay();

	if (yas_acc_get_enable(data->driver) != enable) {
		if (enable) {
			driver->set_enable(enable);
			acc_power_on();
			schedule_delayed_work(&data->work
					      , delay_to_jiffies(delay) + 1);
		} else {
			cancel_delayed_work_sync(&data->work);
			driver->set_enable(enable);
			acc_power_off();
		}
	}

	return 0;
}

static int yas_acc_get_delay(struct yas_acc_driver *driver)
{
	return driver->get_delay();
}

static int yas_acc_set_delay(struct yas_acc_driver *driver, int delay)
{
	struct yas_acc_private_data *data = yas_acc_get_data();

	mutex_lock(&data->enable_mutex);

	if (yas_acc_get_enable(data->driver)) {
		cancel_delayed_work_sync(&data->work);
		driver->set_delay(actual_delay(delay));
		schedule_delayed_work(&data->work, delay_to_jiffies(delay) + 1);
	} else {
		driver->set_delay(actual_delay(delay));
	}

	mutex_unlock(&data->enable_mutex);

	return 0;
}

static int yas_acc_get_offset(struct yas_acc_driver *driver,
			      struct yas_vector *offset)
{
	return driver->get_offset(offset);
}

static int yas_acc_set_offset(struct yas_acc_driver *driver,
			      struct yas_vector *offset)
{
	return driver->set_offset(offset);
}

static int yas_acc_get_position(struct yas_acc_driver *driver)
{
	return driver->get_position();
}

static int yas_acc_set_position(struct yas_acc_driver *driver, int position)
{
	return driver->set_position(position);
}

static int yas_acc_get_threshold(struct yas_acc_driver *driver)
{
	struct yas_acc_filter filter;

	driver->get_filter(&filter);

	return filter.threshold;
}

static int yas_acc_set_threshold(struct yas_acc_driver *driver, int threshold)
{
	struct yas_acc_filter filter;

	filter.threshold = threshold;

	return driver->set_filter(&filter);
}

static int yas_acc_get_filter_enable(struct yas_acc_driver *driver)
{
	return driver->get_filter_enable();
}

static int yas_acc_set_filter_enable(struct yas_acc_driver *driver, int enable)
{
	return driver->set_filter_enable(enable);
}

static int yas_acc_measure(struct yas_acc_driver *driver,
			   struct yas_acc_data *accel)
{
	int err;

	err = driver->measure(accel);

	return err;
}

/* -------------------------------------------------------------------------- *
 *  Input device interface
 * -------------------------------------------------------------------------- */
static int yas_acc_input_init(struct yas_acc_private_data *data)
{
	struct input_dev *dev;
	int err;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	dev->name = "accelerometer";
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_capability(dev, EV_ABS, ABS_RUDDER);
	input_set_abs_params(dev, ABS_X, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_abs_params(dev, ABS_Y, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_abs_params(dev, ABS_Z, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_drvdata(dev, data);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	data->input = dev;

	return 0;
}

static void yas_acc_input_fini(struct yas_acc_private_data *data)
{
	struct input_dev *dev = data->input;

	input_unregister_device(dev);
	input_free_device(dev);
}

static ssize_t yas_acc_enable_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);
	int enable = 0;

	mutex_lock(&data->enable_mutex);

	enable = yas_acc_get_enable(data->driver);

	mutex_unlock(&data->enable_mutex);

	return sprintf(buf, "%d\n", enable);
}

static ssize_t yas_acc_enable_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);
	unsigned long enable;
	int ret;

	ret = strict_strtoul(buf, 10, &enable);
	if (ret < 0)
		return count;

	mutex_lock(&data->enable_mutex);

	yas_acc_set_enable(data->driver, enable);

	mutex_unlock(&data->enable_mutex);

	return count;
}

static ssize_t yas_acc_delay_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", yas_acc_get_delay(data->driver));
}

static ssize_t yas_acc_delay_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf,
				   size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);
	unsigned long delay;
	int ret;

	ret = strict_strtoul(buf, 10, &delay);
	if (ret < 0)
		return count;

	yas_acc_set_delay(data->driver, delay);

	return count;
}

static ssize_t yas_acc_offset_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);
	struct yas_vector offset;

	yas_acc_get_offset(data->driver, &offset);

	return sprintf(buf, "%d %d %d\n"
		       , offset.v[0], offset.v[1], offset.v[2]);
}

static ssize_t yas_acc_offset_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);
	struct yas_vector offset;

	sscanf(buf, "%d %d %d", &offset.v[0], &offset.v[1], &offset.v[2]);

	yas_acc_set_offset(data->driver, &offset);

	return count;
}

static ssize_t yas_acc_position_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", yas_acc_get_position(data->driver));
}

static ssize_t yas_acc_position_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);
	unsigned long position;
	int ret;

	ret = strict_strtoul(buf, 10, &position);
	if (ret < 0)
		return count;

	yas_acc_set_position(data->driver, position);

	return count;
}

static ssize_t yas_acc_threshold_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", yas_acc_get_threshold(data->driver));
}

static ssize_t yas_acc_threshold_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf,
				       size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);
	unsigned long threshold;
	int ret;

	ret = strict_strtoul(buf, 10, &threshold);
	if (ret < 0)
		return count;

	yas_acc_set_threshold(data->driver, threshold);

	return count;
}

static ssize_t yas_acc_filter_enable_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", yas_acc_get_filter_enable(data->driver));
}

static ssize_t yas_acc_filter_enable_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);
	unsigned long enable;
	int ret;

	ret = strict_strtoul(buf, 10, &enable);
	if (ret < 0)
		return count;

	yas_acc_set_filter_enable(data->driver, enable);

	return count;
}

static ssize_t yas_acc_wake_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	static atomic_t serial = ATOMIC_INIT(0);
#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	struct yas_acc_private_data *data = input_get_drvdata(input);
	struct input_event ev[1];
	make_event(ev, EV_ABS, ABS_MISC, atomic_inc_return(&serial));
	sensor_event(&data->devfile_list, ev, 1);
#else
	input_report_abs(input, ABS_MISC, atomic_inc_return(&serial));
	input_sync(input);
#endif

	return count;
}

static ssize_t yas_acc_private_data_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);
	struct yas_acc_data accel;

	mutex_lock(&data->data_mutex);
	accel = data->last;
	mutex_unlock(&data->data_mutex);

	return sprintf(buf, "%d %d %d\n"
		       , accel.xyz.v[0], accel.xyz.v[1], accel.xyz.v[2]);
}

#if DEBUG
#if YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMA150
#define ADR_MAX (0x16)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMA222 || \
	YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMA250 ||	\
	YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMA254
#define ADR_MAX (0x40)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_KXSD9
#define ADR_MAX (0x0f)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_KXTE9
#define ADR_MAX (0x5c)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_KXTF9
#define ADR_MAX (0x60)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS331DL  || \
	YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS331DLH ||	\
	YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS331DLM
#define ADR_MAX (0x40)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS3DH
#define ADR_MAX (0x3e)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_ADXL345
#define ADR_MAX (0x3a)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_ADXL346
#define ADR_MAX (0x3d)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_MMA8452Q || \
	YAS_ACC_DRIVER == YAS_ACC_DRIVER_MMA8453Q
#define ADR_MAX (0x32)
#else
#define ADR_MAX (0x16)
#endif
static uint8_t reg[ADR_MAX];

static ssize_t yas_acc_debug_reg_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);
	struct i2c_client *client = data->client;
	ssize_t count = 0;
	int ret;
	int i;

	memset(reg, -1, ADR_MAX);
	for (i = 0; i < ADR_MAX; i++) {
		ret = data->driver->get_register(i, &reg[i]);
		if (ret != 0)
			dev_err(&client->dev, "get_register() erorr %d (%d)\n"
				, ret, i);
		else
			count += sprintf(&buf[count], "%02x: %d\n", i, reg[i]);
	}

	return count;
}

static ssize_t yas_acc_debug_suspend_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);
	int suspend = 0;

	mutex_lock(&data->suspend_mutex);

	suspend = sprintf(buf, "%d\n", data->suspend);

	mutex_unlock(&data->suspend_mutex);

	return suspend;
}

static ssize_t yas_acc_debug_suspend_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);
	struct i2c_client *client = data->client;
	unsigned long suspend;
	pm_message_t msg;
	int ret;

	ret = strict_strtoul(buf, 10, &suspend);
	if (ret < 0)
		return count;

	memset(&msg, 0, sizeof(pm_message_t));

	mutex_lock(&data->suspend_mutex);

	if (suspend) {
		yas_acc_suspend(client, msg);
		data->suspend = 1;
	} else {
		yas_acc_resume(client);
		data->suspend = 0;
	}

	mutex_unlock(&data->suspend_mutex);

	return count;
}
#endif /* DEBUG */

static DEVICE_ATTR(enable,
		   S_IRUGO|S_IWUSR|S_IWGRP,
		   yas_acc_enable_show,
		   yas_acc_enable_store
		   );
static DEVICE_ATTR(delay,
		   S_IRUGO|S_IWUSR|S_IWGRP,
		   yas_acc_delay_show,
		   yas_acc_delay_store
		   );
static DEVICE_ATTR(offset,
		   S_IRUGO|S_IWUSR,
		   yas_acc_offset_show,
		   yas_acc_offset_store
		   );
static DEVICE_ATTR(position,
		   S_IRUGO|S_IWUSR,
		   yas_acc_position_show,
		   yas_acc_position_store
		   );
static DEVICE_ATTR(threshold,
		   S_IRUGO|S_IWUSR,
		   yas_acc_threshold_show,
		   yas_acc_threshold_store
		   );
static DEVICE_ATTR(filter_enable,
		   S_IRUGO|S_IWUSR,
		   yas_acc_filter_enable_show,
		   yas_acc_filter_enable_store
		   );
static DEVICE_ATTR(wake,
		   S_IWUSR|S_IWGRP,
		   NULL,
		   yas_acc_wake_store);
static DEVICE_ATTR(data,
		   S_IRUGO,
		   yas_acc_private_data_show,
		   NULL);
#if DEBUG
static DEVICE_ATTR(debug_reg,
		   S_IRUGO,
		   yas_acc_debug_reg_show,
		   NULL
		   );
static DEVICE_ATTR(debug_suspend,
		   S_IRUGO|S_IWUSR,
		   yas_acc_debug_suspend_show,
		   yas_acc_debug_suspend_store
		   );
#endif /* DEBUG */

static struct attribute *yas_acc_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_offset.attr,
	&dev_attr_position.attr,
	&dev_attr_threshold.attr,
	&dev_attr_filter_enable.attr,
	&dev_attr_wake.attr,
	&dev_attr_data.attr,
#if DEBUG
	&dev_attr_debug_reg.attr,
	&dev_attr_debug_suspend.attr,
#endif /* DEBUG */
	NULL
};

static struct attribute_group yas_acc_attribute_group = {
	.attrs = yas_acc_attributes
};

static void yas_acc_work_func(struct work_struct *work)
{
	struct yas_acc_private_data *data \
		= container_of((struct delayed_work *)work,
			       struct yas_acc_private_data,
			       work);
	struct yas_acc_data accel, last;
	unsigned long delay = delay_to_jiffies(yas_acc_get_delay(data->driver));
	static int cnt;
#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	struct input_event ev[4];
	struct timeval tv;
#endif

	accel.xyz.v[0] = accel.xyz.v[1] = accel.xyz.v[2] = 0;
	yas_acc_measure(data->driver, &accel);

	mutex_lock(&data->data_mutex);
	last = data->last;
	mutex_unlock(&data->data_mutex);

#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	get_time_stamp(&tv);
	make_event_w_time(&ev[0], EV_ABS, ABS_X, accel.xyz.v[0], &tv);
	make_event_w_time(&ev[1], EV_ABS, ABS_Y, accel.xyz.v[1], &tv);
	make_event_w_time(&ev[2], EV_ABS, ABS_Z, accel.xyz.v[2], &tv);
	make_event_w_time(&ev[3], EV_SYN, 0, 0, &tv);
	sensor_event(&data->devfile_list, ev, 4);
#else
	input_report_abs(data->input, ABS_X, accel.xyz.v[0]);
	input_report_abs(data->input, ABS_Y, accel.xyz.v[1]);
	input_report_abs(data->input, ABS_Z, accel.xyz.v[2]);
	if (last.xyz.v[0] == accel.xyz.v[0] &&
	    last.xyz.v[1] == accel.xyz.v[1] &&
	    last.xyz.v[2] == accel.xyz.v[2])
		input_report_abs(data->input, ABS_RUDDER, cnt++);
	input_sync(data->input);
#endif

	mutex_lock(&data->data_mutex);
	data->last = accel;
	mutex_unlock(&data->data_mutex);

	schedule_delayed_work(&data->work, delay);
}

static int yas_acc_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct yas_acc_private_data *data;
	int err;

	/* Setup private data */
	data = kzalloc(sizeof(struct yas_acc_private_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto ERR1;
	}
	yas_acc_set_data(data);

	mutex_init(&data->driver_mutex);
	mutex_init(&data->data_mutex);
	mutex_init(&data->enable_mutex);
#if DEBUG
	mutex_init(&data->suspend_mutex);
#endif

	/* regulator output enable/disable control */
	acc_power.regulator_vdd = NULL;
	acc_power.regulator_vio = NULL;

	acc_power.regulator_vdd = regulator_get(&client->dev, "vdd_acc");
	if (IS_ERR(acc_power.regulator_vdd)) {
		err = PTR_ERR(acc_power.regulator_vdd);
		pr_err("%s: failed to get acc_i2c_vdd %d\n", __func__, err);
		goto err_setup_regulator;
	}

	acc_power.regulator_vio = regulator_get(&client->dev, "vio_acc");
	if (IS_ERR(acc_power.regulator_vio)) {
		err = PTR_ERR(acc_power.regulator_vio);
		pr_err("%s: failed to get acc_i2c_vio %d\n", __func__, err);
		goto err_setup_regulator;
	}

	acc_power_on();

	/* Setup i2c client */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto ERR2;
	}
	i2c_set_clientdata(client, data);
	data->client = client;

	/* Setup accelerometer core driver */
	err = yas_acc_core_driver_init(data);
	if (err < 0)
		goto ERR2;

	/* Setup driver interface */
	INIT_DELAYED_WORK(&data->work, yas_acc_work_func);

	/* Setup input device interface */
	err = yas_acc_input_init(data);
	if (err < 0)
		goto ERR3;

	/* Setup sysfs */
	err = sysfs_create_group(&data->input->dev.kobj,
				 &yas_acc_attribute_group);
	if (err < 0)
		goto ERR4;

#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	INIT_LIST_HEAD(&data->devfile_list);
	if (misc_register(&sensor_devfile) < 0)
		goto ERR4;
#endif

	return 0;

ERR4:
	yas_acc_input_fini(data);
ERR3:
	yas_acc_core_driver_fini(data);
ERR2:
	kfree(data);
err_setup_regulator:
	if (acc_power.regulator_vdd) {
		regulator_disable(acc_power.regulator_vdd);
		regulator_put(acc_power.regulator_vdd);
	}
	if (acc_power.regulator_vio) {
		regulator_disable(acc_power.regulator_vio);
		regulator_put(acc_power.regulator_vio);
	}
exit:
	client = NULL;
	pr_err("%s: failed!\n", __func__);
ERR1:
	return err;
}

static int yas_acc_remove(struct i2c_client *client)
{
	struct yas_acc_private_data *data = i2c_get_clientdata(client);
	struct yas_acc_driver *driver = data->driver;

#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	misc_deregister(&sensor_devfile);
#endif
	yas_acc_set_enable(driver, 0);
	sysfs_remove_group(&data->input->dev.kobj, &yas_acc_attribute_group);
	yas_acc_input_fini(data);
	yas_acc_core_driver_fini(data);
	kfree(data);

	return 0;
}

static int yas_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct yas_acc_private_data *data = i2c_get_clientdata(client);
	struct yas_acc_driver *driver = data->driver;

	mutex_lock(&data->enable_mutex);

	data->suspend_enable = yas_acc_get_enable(driver);
	if (data->suspend_enable)
		yas_acc_set_enable(driver, 0);

	mutex_unlock(&data->enable_mutex);

	return 0;
}

static int yas_acc_resume(struct i2c_client *client)
{
	struct yas_acc_private_data *data = i2c_get_clientdata(client);
	struct yas_acc_driver *driver = data->driver;

	mutex_lock(&data->enable_mutex);

	if (data->suspend_enable)
		yas_acc_set_enable(driver, 1);

	mutex_unlock(&data->enable_mutex);

	return 0;
}

static const struct i2c_device_id yas_acc_id[] = {
	{YAS_ACC_KERNEL_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, yas_acc_id);

struct i2c_driver yas_acc_driver = {
	.driver = {
		.name = "accelerometer",
		.owner = THIS_MODULE,
	},
	.probe = yas_acc_probe,
	.remove = yas_acc_remove,
	.suspend = yas_acc_suspend,
	.resume = yas_acc_resume,
	.id_table = yas_acc_id,
};

/* -------------------------------------------------------------------------- *
 *  Module init and exit
 * -------------------------------------------------------------------------- */
static int __init yas_acc_init(void)
{
	return i2c_add_driver(&yas_acc_driver);
}
module_init(yas_acc_init);

static void __exit yas_acc_exit(void)
{
	i2c_del_driver(&yas_acc_driver);
}
module_exit(yas_acc_exit);

MODULE_DESCRIPTION("accelerometer kernel driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(YAS_ACC_KERNEL_VERSION);
