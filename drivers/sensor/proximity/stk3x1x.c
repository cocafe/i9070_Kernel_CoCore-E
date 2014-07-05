/*
 *  stk3x1x.c - Linux kernel modules for SenseTek stk3x1x
 *  
 *
 *  Copyright (C) 2012 Lex Hsieh / SenseTek <lex_hsieh@sitronix.com.tw>
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
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/poll.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/completion.h>
#include <linux/kthread.h>
#include <linux/errno.h>
#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
	#include <linux/earlysuspend.h>
#endif

#include <mach/stk3x1x.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <mach/board-sec-ux500.h>

#define DEVICE_NAME		"stk_ps"
#define DRIVER_VERSION  "2.1"
#define PS_NAME "proximity_sensor"
//#define DRIVER_VERSION  STK_DRIVER_VER

struct stk3x1x_data {
	struct i2c_client *client;
	uint16_t ps_thd_h;
	uint16_t ps_thd_l;
	
	struct mutex io_lock;
	struct input_dev *ps_input_dev;
	int32_t ps_distance_last;
	bool ps_enabled;
	struct wake_lock ps_wakelock;	
	uint8_t wait_reg;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend stk_early_suspend;
#endif
    struct work_struct stk_ps_work;
	struct workqueue_struct *stk_ps_wq;
    int32_t irq;
	int		int_pin;

};


struct stk3x1x_power_data {
	struct regulator *regulator_vdd;
	struct regulator *regulator_vio;
};
static struct stk3x1x_power_data stk3x1x_power;

static int32_t stk3x1x_enable_ps(struct stk3x1x_data *ps_data, uint8_t enable);
static int32_t stk3x1x_set_ps_thd_l(struct stk3x1x_data *ps_data, uint16_t thd_l);
static int32_t stk3x1x_set_ps_thd_h(struct stk3x1x_data *ps_data, uint16_t thd_h);

static int32_t stk3x1x_init_all_reg(struct stk3x1x_data *ps_data, struct stk3x1x_platform_data *plat_data)
{
	int32_t ret;
	uint8_t w_reg;
	
	ps_data->ps_thd_h = plat_data->ps_thd_h;
	ps_data->ps_thd_l = plat_data->ps_thd_l;	
	
	w_reg = plat_data->state_reg;
    ret = i2c_smbus_write_byte_data(ps_data->client, STK_STATE_REG, w_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }	
	w_reg = plat_data->psctrl_reg;
    ret = i2c_smbus_write_byte_data(ps_data->client, STK_PSCTRL_REG, w_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }	
	
	w_reg = plat_data->ledctrl_reg;
    ret = i2c_smbus_write_byte_data(ps_data->client, STK_LEDCTRL_REG, w_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }	
	ps_data->wait_reg = plat_data->wait_reg;
	w_reg = plat_data->wait_reg;
    ret = i2c_smbus_write_byte_data(ps_data->client, STK_WAIT_REG, w_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }	
	stk3x1x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
	stk3x1x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);
	/*
	w_reg = (uint8_t)((ps_data->ps_thd_h & 0xFF00) >> 8);
    ret = i2c_smbus_write_byte_data(ps_data->client, STK_THDH1_PS_REG, w_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }
	w_reg = (uint8_t)(ps_data->ps_thd_h & 0x00FF);	
    ret = i2c_smbus_write_byte_data(ps_data->client, STK_THDH2_PS_REG, w_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }	
	w_reg = (uint8_t)((ps_data->ps_thd_l & 0xFF00) >> 8);	
    ret = i2c_smbus_write_byte_data(ps_data->client, STK_THDL1_PS_REG, w_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }	
	w_reg = (uint8_t)(ps_data->ps_thd_l & 0x00FF);	
    ret = i2c_smbus_write_byte_data(ps_data->client, STK_THDL2_PS_REG, w_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }	
	*/
	w_reg = STK_INT_PS_MODE3;		
    ret = i2c_smbus_write_byte_data(ps_data->client, STK_INT_REG, w_reg);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}		
	return 0;	
}

static int32_t stk3x1x_check_pid(struct stk3x1x_data *ps_data)
{
	int32_t err;
	
	err = i2c_smbus_read_byte_data(ps_data->client,STK_PDT_ID_REG);
	if (err < 0)
	{
		printk(KERN_ERR "%s: read i2c error, err=%d\n", __func__, err);
		return err;
	}
	if (err != STK3X1X_PDT_ID)
	{
		printk(KERN_ERR "%s: invalid product ID number", __func__);
		return -EINVAL;
	}				
	return 0;
}


static int32_t stk3x1x_software_reset(struct stk3x1x_data *ps_data)
{
    int32_t r;
    uint8_t w_reg;
	
    w_reg = 0x7F;
    r = i2c_smbus_write_byte_data(ps_data->client,STK_WAIT_REG,w_reg);
    if (r<0)
    {
        printk(KERN_ERR "%s: software reset: write i2c error\n", __func__);
        return r;
    }
    r = i2c_smbus_read_byte_data(ps_data->client,STK_WAIT_REG);
    if (w_reg != r)
    {
        printk(KERN_ERR "%s: software reset: read-back value is not the same\n", __func__);
        return -1;
    }
    r = i2c_smbus_write_byte_data(ps_data->client,STK_SW_RESET_REG,0);
    if (r<0)
    {
        printk(KERN_ERR "%s: software reset: read error after reset\n", __func__);
        return r;
    }
    msleep(1);
    return 0;
}

static int32_t stk3x1x_set_ps_thd_l(struct stk3x1x_data *ps_data, uint16_t thd_l)
{
    uint8_t temp;
    uint8_t* pSrc = (uint8_t*)&thd_l;
	
    temp = *pSrc;
    *pSrc = *(pSrc+1);
    *(pSrc+1) = temp;
    ps_data->ps_thd_l = thd_l;
	return i2c_smbus_write_word_data(ps_data->client,STK_THDL1_PS_REG,thd_l);
}
static int32_t stk3x1x_set_ps_thd_h(struct stk3x1x_data *ps_data, uint16_t thd_h)
{
    uint8_t temp;
    uint8_t* pSrc = (uint8_t*)&thd_h;
	 
    temp = *pSrc;
    *pSrc = *(pSrc+1);
    *(pSrc+1) = temp;
    ps_data->ps_thd_h = thd_h;
	return i2c_smbus_write_word_data(ps_data->client,STK_THDH1_PS_REG,thd_h);
}

static inline int32_t stk3x1x_get_ps_reading(struct stk3x1x_data *ps_data)
{
	int32_t word_data, tmp_word_data;
	
	tmp_word_data = i2c_smbus_read_word_data(ps_data->client,STK_DATA1_PS_REG);
	if(tmp_word_data < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, tmp_word_data);
		return tmp_word_data;	
	}
	word_data = ((tmp_word_data & 0xFF00) >> 8) | ((tmp_word_data & 0x00FF) << 8) ;
	return word_data;
}
static int32_t stk3x1x_enable_ps(struct stk3x1x_data *ps_data, uint8_t enable)
{
    int32_t ret;
	uint8_t w_state_reg;
	uint8_t curr_ps_enable;	
	int32_t reading;
	int32_t near_far_state;
	
	curr_ps_enable = ps_data->ps_enabled?1:0;	
	if(curr_ps_enable == enable)
		return 0;
		
    ret = i2c_smbus_read_byte_data(ps_data->client, STK_STATE_REG);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
    }		
	w_state_reg = (uint8_t)(ret & (~STK_STATE_EN_PS_MASK)); 
	if(enable)	
		w_state_reg |= STK_STATE_EN_PS_MASK;	
	
    ret = i2c_smbus_write_byte_data(ps_data->client, STK_STATE_REG, w_state_reg);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}
			
    if(enable)
	{
		msleep(1);
		near_far_state = !gpio_get_value(ps_data->int_pin);	
		enable_irq(ps_data->irq);
		ps_data->ps_enabled = true;
		ps_data->ps_distance_last = near_far_state;
		input_report_abs(ps_data->ps_input_dev, ABS_DISTANCE, near_far_state);
		input_sync(ps_data->ps_input_dev);
		wake_lock_timeout(&ps_data->ps_wakelock, 3*HZ);	
		printk(KERN_INFO "%s: ps input event %d cm\n",__func__, near_far_state);			
		reading = stk3x1x_get_ps_reading(ps_data);
		printk(KERN_INFO "%s : ps code = %d, near_far_state=%d\n",__func__,reading, near_far_state);				
	}
	else
	{
		disable_irq(ps_data->irq);
		ps_data->ps_enabled = false;		
	}
	return ret;
}


static ssize_t stk_ps_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
    int32_t reading;
    reading = stk3x1x_get_ps_reading(ps_data);
    return sprintf(buf, "%d\n", reading);
}

static ssize_t stk_ps_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t enable, ret;
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
	
    mutex_lock(&ps_data->io_lock);
	enable = (ps_data->ps_enabled)?1:0;
    mutex_unlock(&ps_data->io_lock);
    ret = i2c_smbus_read_byte_data(ps_data->client,STK_STATE_REG);
    ret = (ret & STK_STATE_EN_PS_MASK)?1:0;
	
	if(enable != ret)
		printk(KERN_ERR " %s: driver and sensor mismatch! driver_enable=0x%x, sensor_enable=%x\n", __func__, enable, ret);
	
	return sprintf(buf, "%d\n", ret);	
}

static ssize_t stk_ps_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
	uint8_t en;
	if (sysfs_streq(buf, "1"))
		en = 1;
	else if (sysfs_streq(buf, "0"))
		en = 0;
	else 
	{
		printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}
    printk(KERN_INFO "%s: Enable PS : %d\n", __func__, en);

    mutex_lock(&ps_data->io_lock);
    stk3x1x_enable_ps(ps_data, en);
    mutex_unlock(&ps_data->io_lock);
    return size;
}

static ssize_t stk_ps_enable_aso_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ret;
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
	
    ret = i2c_smbus_read_byte_data(ps_data->client,STK_STATE_REG);
    ret = (ret & STK_STATE_EN_ASO_MASK)?1:0;
	
	return sprintf(buf, "%d\n", ret);		
}

static ssize_t stk_ps_enable_aso_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
	uint8_t en;
    int32_t ret;
	uint8_t w_state_reg;
	
	if (sysfs_streq(buf, "1"))
		en = 1;
	else if (sysfs_streq(buf, "0"))
		en = 0;
	else 
	{
		printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}
    printk(KERN_INFO "%s: Enable PS ASO : %d\n", __func__, en);
    
    ret = i2c_smbus_read_byte_data(ps_data->client, STK_STATE_REG);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
    }		
	w_state_reg = (uint8_t)(ret & (~STK_STATE_EN_ASO_MASK)); 
	if(en)	
		w_state_reg |= STK_STATE_EN_ASO_MASK;	
	
    ret = i2c_smbus_write_byte_data(ps_data->client, STK_STATE_REG, w_state_reg);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}	
	
	return size;	
}
static ssize_t stk_ps_offset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
    int32_t word_data, tmp_word_data;
	
	tmp_word_data = i2c_smbus_read_word_data(ps_data->client, STK_DATA1_OFFSET_REG);
	if(tmp_word_data < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, tmp_word_data);
		return tmp_word_data;	   
	}
		word_data = ((tmp_word_data & 0xFF00) >> 8) | ((tmp_word_data & 0x00FF) << 8) ;
	return sprintf(buf, "%d\n", word_data);
}
 
static ssize_t stk_ps_offset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
	unsigned long value = 0;
	int ret;
	uint16_t offset;
	
	ret = strict_strtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, ret=0x%x\n", __func__, ret);	
		return ret;	
	}
	if(value > 65535)
	{
		printk(KERN_ERR "%s: invalid value, offset=%ld\n", __func__, value);
		return -EINVAL;
	}
	
	offset = (uint16_t) ((value&0x00FF) << 8) | ((value&0xFF00) >>8);
	ret = i2c_smbus_write_word_data(ps_data->client,STK_DATA1_OFFSET_REG,offset);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}
	return size;
}

static ssize_t stk_ps_distance_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
    int32_t dist=1, ret;

    mutex_lock(&ps_data->io_lock);	
    ret = i2c_smbus_read_byte_data(ps_data->client,STK_FLAG_REG);
    dist = (ret & STK_FLG_NF_MASK)?1:0;	
	
    ps_data->ps_distance_last = dist;
	input_report_abs(ps_data->ps_input_dev, ABS_DISTANCE, dist);
	input_sync(ps_data->ps_input_dev);
    mutex_unlock(&ps_data->io_lock);
	wake_lock_timeout(&ps_data->ps_wakelock, 3*HZ);
	printk(KERN_INFO "%s: ps input event %d cm\n",__func__, dist);		
    return sprintf(buf, "%d\n", dist);
}

static ssize_t stk_ps_distance_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
	unsigned long value = 0;
	int ret;
	ret = strict_strtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, ret=0x%x\n", __func__, ret);	
		return ret;	
	}
    mutex_lock(&ps_data->io_lock);
    ps_data->ps_distance_last = value;
	input_report_abs(ps_data->ps_input_dev, ABS_DISTANCE, value);
	input_sync(ps_data->ps_input_dev);
    mutex_unlock(&ps_data->io_lock);
	wake_lock_timeout(&ps_data->ps_wakelock, 3*HZ);
	printk(KERN_INFO "%s: ps input event %ld cm\n",__func__, value);	
    return size;
}

static ssize_t stk_ps_code_thd_l_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ps_thd_l1_reg, ps_thd_l2_reg;
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
    mutex_lock(&ps_data->io_lock);
    ps_thd_l1_reg = i2c_smbus_read_byte_data(ps_data->client,STK_THDL1_PS_REG);
    if(ps_thd_l1_reg < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, ps_thd_l1_reg);		
		return -EINVAL;		
	}
    ps_thd_l2_reg = i2c_smbus_read_byte_data(ps_data->client,STK_THDL2_PS_REG);
    if(ps_thd_l2_reg < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, ps_thd_l2_reg);		
		return -EINVAL;		
	}
    mutex_unlock(&ps_data->io_lock);
	ps_thd_l1_reg = ps_thd_l1_reg<<8 | ps_thd_l2_reg;
    return sprintf(buf, "%d\n", ps_thd_l1_reg);
}


static ssize_t stk_ps_code_thd_l_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);		
	unsigned long value = 0;
	int ret;
	ret = strict_strtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	    
	}
    mutex_lock(&ps_data->io_lock);
    stk3x1x_set_ps_thd_l(ps_data, value);
    mutex_unlock(&ps_data->io_lock);
    return size;
}

static ssize_t stk_ps_code_thd_h_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ps_thd_h1_reg, ps_thd_h2_reg;
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
    mutex_lock(&ps_data->io_lock);
    ps_thd_h1_reg = i2c_smbus_read_byte_data(ps_data->client,STK_THDH1_PS_REG);
    if(ps_thd_h1_reg < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, ps_thd_h1_reg);		
		return -EINVAL;		
	}
    ps_thd_h2_reg = i2c_smbus_read_byte_data(ps_data->client,STK_THDH2_PS_REG);
    if(ps_thd_h2_reg < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, ps_thd_h2_reg);		
		return -EINVAL;		
	}
    mutex_unlock(&ps_data->io_lock);
	ps_thd_h1_reg = ps_thd_h1_reg<<8 | ps_thd_h2_reg;
    return sprintf(buf, "%d\n", ps_thd_h1_reg);
}


static ssize_t stk_ps_code_thd_h_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);		
	unsigned long value = 0;
	int ret;
	ret = strict_strtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	    
	}
    mutex_lock(&ps_data->io_lock);
    stk3x1x_set_ps_thd_h(ps_data, value);
    mutex_unlock(&ps_data->io_lock);
    return size;
}

static ssize_t stk_all_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ps_reg[27];
	uint8_t cnt;
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
    mutex_lock(&ps_data->io_lock);
	for(cnt=0;cnt<25;cnt++)
	{
		ps_reg[cnt] = i2c_smbus_read_byte_data(ps_data->client, (cnt));
		if(ps_reg[cnt] < 0)
		{
			mutex_unlock(&ps_data->io_lock);
			printk(KERN_ERR "stk_all_reg_show:i2c_smbus_read_byte_data fail, ret=%d", ps_reg[cnt]);	
			return -EINVAL;
		}
		else
		{
			printk(KERN_INFO "reg[0x%2X]=0x%2X\n", cnt, ps_reg[cnt]);
		}
	}
	ps_reg[cnt] = i2c_smbus_read_byte_data(ps_data->client, STK_PDT_ID_REG);
	if(ps_reg[cnt] < 0)
	{
		mutex_unlock(&ps_data->io_lock);
		printk( KERN_ERR "all_reg_show:i2c_smbus_read_byte_data fail, ret=%d", ps_reg[cnt]);	
		return -EINVAL;
	}
	printk( KERN_INFO "reg[0x%x]=0x%2X\n", STK_PDT_ID_REG, ps_reg[cnt]);	
	cnt++;
	ps_reg[cnt] = i2c_smbus_read_byte_data(ps_data->client, STK_RSRVD_REG);
	if(ps_reg[cnt] < 0)
	{
		mutex_unlock(&ps_data->io_lock);
		printk( KERN_ERR "all_reg_show:i2c_smbus_read_byte_data fail, ret=%d", ps_reg[cnt]);	
		return -EINVAL;
	}
	printk( KERN_INFO "reg[0x%x]=0x%2X\n", STK_RSRVD_REG, ps_reg[cnt]);		
    mutex_unlock(&ps_data->io_lock);

    return sprintf(buf, "%2X %2X %2X %2X %2X,%2X %2X %2X %2X %2X,%2X %2X %2X %2X %2X,%2X %2X %2X %2X %2X,%2X %2X %2X %2X %2X,%2X %2X\n", 
		ps_reg[0], ps_reg[1], ps_reg[2], ps_reg[3], ps_reg[4], ps_reg[5], ps_reg[6], ps_reg[7], ps_reg[8], 
		ps_reg[9], ps_reg[10], ps_reg[11], ps_reg[12], ps_reg[13], ps_reg[14], ps_reg[15], ps_reg[16], ps_reg[17], 
		ps_reg[18], ps_reg[19], ps_reg[20], ps_reg[21], ps_reg[22], ps_reg[23], ps_reg[24], ps_reg[25], ps_reg[26]);
}

static ssize_t stk_recv_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}


static ssize_t stk_recv_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned long value = 0;
	int ret;
	int32_t recv_data;	
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
	
	if((ret = strict_strtoul(buf, 16, &value)) < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}
	recv_data = i2c_smbus_read_byte_data(ps_data->client,value);
	printk("%s: reg 0x%x=0x%x\n", __func__, (int)value, recv_data);
	return size;
}


static ssize_t stk_send_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}


static ssize_t stk_send_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int addr, cmd;
	u8 addr_u8, cmd_u8;
	int32_t ret, i;
	char *token[10];
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
	
	for (i = 0; i < 2; i++)
		token[i] = strsep((char **)&buf, " ");
	if((ret = strict_strtoul(token[0], 16, (unsigned long *)&(addr))) < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}
	if((ret = strict_strtoul(token[1], 16, (unsigned long *)&(cmd))) < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}
	printk(KERN_INFO "%s: write reg 0x%x=0x%x\n", __func__, addr, cmd);		

	addr_u8 = (u8) addr;
	cmd_u8 = (u8) cmd;
	//mutex_lock(&ps_data->io_lock);
	ret = i2c_smbus_write_byte_data(ps_data->client,addr_u8,cmd_u8);
	//mutex_unlock(&ps_data->io_lock);
	if (0 != ret)
	{	
		printk(KERN_ERR "%s: i2c_smbus_write_byte_data fail\n", __func__);
		return ret;
	}
	
	return size;
}


static struct device_attribute ps_enable_attribute = __ATTR(enable,0644,stk_ps_enable_show,stk_ps_enable_store);
static struct device_attribute ps_enable_aso_attribute = __ATTR(enable_aso,0644,stk_ps_enable_aso_show,stk_ps_enable_aso_store);
static struct device_attribute ps_distance_attribute = __ATTR(distance,0644,stk_ps_distance_show, stk_ps_distance_store);
static struct device_attribute ps_offset_attribute = __ATTR(offset,0644,stk_ps_offset_show, stk_ps_offset_store);
static struct device_attribute ps_code_attribute = __ATTR(ps_code, 0444, stk_ps_code_show, NULL);
static struct device_attribute ps_code_thd_l_attribute = __ATTR(ps_code_thd_l,0644,stk_ps_code_thd_l_show,stk_ps_code_thd_l_store);
static struct device_attribute ps_code_thd_h_attribute = __ATTR(ps_code_thd_h,0644,stk_ps_code_thd_h_show,stk_ps_code_thd_h_store);
static struct device_attribute recv_attribute = __ATTR(recv,0644,stk_recv_show,stk_recv_store);
static struct device_attribute send_attribute = __ATTR(send,0644,stk_send_show, stk_send_store);
static struct device_attribute all_reg_attribute = __ATTR(all_reg, 0444, stk_all_reg_show, NULL);





static struct attribute *stk_ps_attrs [] =
{
    &ps_enable_attribute.attr,
    &ps_enable_aso_attribute.attr,
    &ps_distance_attribute.attr,
	&ps_offset_attribute.attr,
    &ps_code_attribute.attr,
	&ps_code_thd_l_attribute.attr,
	&ps_code_thd_h_attribute.attr,	
	&recv_attribute.attr,
	&send_attribute.attr,	
	&all_reg_attribute.attr,	
    NULL
};

static struct attribute_group stk_ps_attribute_group = {
	.attrs = stk_ps_attrs,
};

static void stk_work_func(struct work_struct *work)
{
    //int32_t ret;
	int32_t reading;
    //uint8_t w_flag_reg, disable_flag = 0;
    //uint8_t  org_flag_reg;
	//int reg_value;
	struct stk3x1x_data *ps_data = container_of(work, struct stk3x1x_data, stk_ps_work);	
	int32_t near_far_state;
	
    mutex_lock(&ps_data->io_lock);	
	
	near_far_state = !gpio_get_value(ps_data->int_pin);
	
/*	
    org_flag_reg = i2c_smbus_read_byte_data(ps_data->client,STK_FLAG_REG);	
	if(org_flag_reg < 0)
	{
		mutex_unlock(&ps_data->io_lock);
		printk(KERN_ERR "%s: get_status_reg fail, org_flag_reg=%d", __func__, org_flag_reg);
		msleep(30);
		enable_irq(ps_data->irq);
		return; 		
	}	
	reg_value = (org_flag_reg & STK_FLG_NF_MASK)?1:0;
*/	

	ps_data->ps_distance_last = near_far_state;
	input_report_abs(ps_data->ps_input_dev, ABS_DISTANCE, near_far_state);
	input_sync(ps_data->ps_input_dev);
	wake_lock_timeout(&ps_data->ps_wakelock, 3*HZ);
	printk(KERN_INFO "%s: ps input event %d cm\n",__func__, near_far_state);			
	reading = stk3x1x_get_ps_reading(ps_data);
	printk(KERN_INFO "%s : ps code = %d, near_far_state=%d\n",__func__,reading, near_far_state);	
	
	msleep(1);
    enable_irq(ps_data->irq);

    mutex_unlock(&ps_data->io_lock);
}



static irqreturn_t stk_oss_irq_handler(int irq, void *data)
{
	struct stk3x1x_data *pData = data;
	disable_irq_nosync(irq);
	queue_work(pData->stk_ps_wq,&pData->stk_ps_work);
	return IRQ_HANDLED;
}

static int32_t stk3x1x_init_all_setting(struct i2c_client *client, struct stk3x1x_platform_data *plat_data)
{
	int32_t ret;
	struct stk3x1x_data *ps_data = i2c_get_clientdata(client);		
	
	mutex_lock(&ps_data->io_lock);	
	ps_data->ps_enabled = false;
	mutex_unlock(&ps_data->io_lock);
	
	ret = stk3x1x_software_reset(ps_data); 
	if(ret < 0)
	return ret;
	ret = stk3x1x_check_pid(ps_data);
	if(ret < 0)
		return ret;	
	
	ret = stk3x1x_init_all_reg(ps_data, plat_data);
	if(ret < 0)
		return ret;	
    return 0;
}

static int stk3x1x_setup_irq(struct i2c_client *client)
{		
	int irq, err = -EIO;
	struct stk3x1x_data *ps_data = i2c_get_clientdata(client);
	
	irq = gpio_to_irq(ps_data->int_pin);
	printk(KERN_INFO "%s: int pin #=%d, irq=%d\n",__func__, ps_data->int_pin, irq);	
	if (irq <= 0)
	{
		printk(KERN_ERR "irq number is not specified, irq # = %d, int pin=%d\n",irq, ps_data->int_pin);
		return irq;
	}
	ps_data->irq = irq;	
	err = gpio_request(ps_data->int_pin,"stk-int");        
	if(err < 0)
	{
		printk(KERN_ERR "%s: gpio_request, err=%d", __func__, err);
		return err;
	}
	err = gpio_direction_input(ps_data->int_pin);
	if(err < 0)
	{
		printk(KERN_ERR "%s: gpio_direction_input, err=%d", __func__, err);
		return err;
	}	
	err = request_any_context_irq(irq, stk_oss_irq_handler, IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, DEVICE_NAME, ps_data);
	if (err < 0) 
	{
		printk(KERN_WARNING "%s: request_any_context_irq(%d) failed for (%d)\n", __func__, irq, err);		
		goto err_request_any_context_irq;
	}
	disable_irq(irq);
	
	return 0;
err_request_any_context_irq:	
	gpio_free(ps_data->int_pin);		
	return err;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void stk3x1x_early_suspend(struct early_suspend *h)
{
	struct stk3x1x_data *ps_data = container_of(h, struct stk3x1x_data, stk_early_suspend);	
	int err;
	
	printk(KERN_INFO "%s", __func__);
    mutex_lock(&ps_data->io_lock);  		
	  	
	if(ps_data->ps_enabled)
	{
		err = enable_irq_wake(ps_data->irq);	
		if (err)
			printk(KERN_WARNING "%s: set_irq_wake(%d) failed, err=(%d)\n", __func__, ps_data->irq, err);				
	}
	mutex_unlock(&ps_data->io_lock);		
	return;
}

static void stk3x1x_late_resume(struct early_suspend *h)
{
	struct stk3x1x_data *ps_data = container_of(h, struct stk3x1x_data, stk_early_suspend);	
    int err;
	
	printk(KERN_INFO "%s", __func__);	
    mutex_lock(&ps_data->io_lock); 		
	if(ps_data->ps_enabled)
	{
		err = disable_irq_wake(ps_data->irq);	
		if (err)		
			printk(KERN_WARNING "%s: disable_irq_wake(%d) failed, err=(%d)\n", __func__, ps_data->irq, err);		
	}
	mutex_unlock(&ps_data->io_lock);
	return;
}
#endif	//#ifdef CONFIG_HAS_EARLYSUSPEND

#ifdef CONFIG_PM_SLEEP
static int stk3x1x_suspend(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct stk3x1x_data *ps_data = i2c_get_clientdata(client);	
	
	printk(KERN_INFO "%s", __func__);		
	return 0;
}

static int stk3x1x_resume(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct stk3x1x_data *ps_data = i2c_get_clientdata(client);	

	printk(KERN_INFO "%s", __func__);	
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static int stk3x1x_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{


    int err = -ENODEV;

	int ret = 0;
    struct stk3x1x_data *ps_data;
	struct stk3x1x_platform_data *plat_data;
    printk(KERN_INFO "%s: driver version = %s\n", __func__, DRIVER_VERSION);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
    {
        printk(KERN_ERR "%s: No Support for I2C_FUNC_SMBUS_BYTE_DATA\n", __func__);
        return -ENODEV;
    }
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA))
    {
        printk(KERN_ERR "%s: No Support for I2C_FUNC_SMBUS_WORD_DATA\n", __func__);
        return -ENODEV;
    }

	ps_data = kzalloc(sizeof(struct stk3x1x_data),GFP_KERNEL);
	if(!ps_data)
	{
		printk(KERN_ERR "%s: failed to allocate stk3x1x_data\n", __func__);
		return -ENOMEM;
	}


	stk3x1x_power.regulator_vdd = stk3x1x_power.regulator_vio = NULL;
	stk3x1x_power.regulator_vdd = regulator_get(&client->dev, "vdd_proxi");
	if (IS_ERR(stk3x1x_power.regulator_vdd)) {
		ret = PTR_ERR(stk3x1x_power.regulator_vdd);
		pr_err("%s: failed to get vdd_proxi %d\n", __func__, ret);
		goto err_setup_regulator;
	}
	stk3x1x_power.regulator_vio = regulator_get(&client->dev, "vio_proxi");
	if (IS_ERR(stk3x1x_power.regulator_vio)) {
		ret = PTR_ERR(stk3x1x_power.regulator_vio);
		pr_err("%s: failed to get vio_proxi %d\n", __func__, ret);
		goto err_setup_regulator;
		}

	regulator_enable(stk3x1x_power.regulator_vdd);
	mdelay(15);
	regulator_enable(stk3x1x_power.regulator_vio);

	ps_data->client = client;
	i2c_set_clientdata(client,ps_data);
	mutex_init(&ps_data->io_lock);
	wake_lock_init(&ps_data->ps_wakelock,WAKE_LOCK_IDLE,"stk_ps_wakelock");
	
	if(client->dev.platform_data != NULL)		
	{
		plat_data = client->dev.platform_data;	
		ps_data->int_pin = plat_data->int_pin;			
	}
	else
	{
		printk(KERN_INFO "%s: no stk3x1x platform data!\n", __func__);		
	}		
	
	ps_data->ps_input_dev = input_allocate_device();
	if (ps_data->ps_input_dev==NULL)
	{
		printk(KERN_ERR "%s: could not allocate ps device\n", __func__);		
		err = -ENOMEM;
		goto err_ps_input_allocate;		
	}
	ps_data->ps_input_dev->name = PS_NAME;
	set_bit(EV_ABS, ps_data->ps_input_dev->evbit);
	input_set_abs_params(ps_data->ps_input_dev, ABS_DISTANCE, 0,1, 0, 0);

	input_set_drvdata(ps_data->ps_input_dev, ps_data);

	err = input_register_device(ps_data->ps_input_dev);	
	if (err<0)
	{
		printk(KERN_ERR "%s: can not register ps input device\n", __func__);	
		goto err_ps_input_register;
	}

	err = sysfs_create_group(&ps_data->ps_input_dev->dev.kobj, &stk_ps_attribute_group);
	if (err < 0) 
	{
		printk(KERN_ERR "%s:could not create sysfs group for ps\n", __func__);
		goto err_ps_sysfs_create_group;
	}
	input_set_drvdata(ps_data->ps_input_dev, ps_data);	
	ps_data->stk_ps_wq = create_singlethread_workqueue("stk_ps_wq");
	INIT_WORK(&ps_data->stk_ps_work, stk_work_func);
	
	err = stk3x1x_setup_irq(client);
	if(err < 0)
		goto err_stk3x1x_setup_irq;

	if (stk3x1x_init_all_setting(client, plat_data))
		goto err_init_all_setting;			
		
#ifdef CONFIG_HAS_EARLYSUSPEND
	ps_data->stk_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ps_data->stk_early_suspend.suspend = stk3x1x_early_suspend;
	ps_data->stk_early_suspend.resume = stk3x1x_late_resume;
	register_early_suspend(&ps_data->stk_early_suspend);
#endif
	printk(KERN_INFO "%s: probe successfully", __func__);
	return 0;

err_init_all_setting:	
	free_irq(ps_data->irq, ps_data);
	gpio_free(plat_data->int_pin);		
err_stk3x1x_setup_irq:
	destroy_workqueue(ps_data->stk_ps_wq);	
	sysfs_remove_group(&ps_data->ps_input_dev->dev.kobj, &stk_ps_attribute_group);	
err_ps_sysfs_create_group:	
	input_unregister_device(ps_data->ps_input_dev);		
err_ps_input_register:
	input_free_device(ps_data->ps_input_dev);	
err_ps_input_allocate:	
    wake_lock_destroy(&ps_data->ps_wakelock);	
    mutex_destroy(&ps_data->io_lock);
	kfree(ps_data);
    return err;

 err_setup_regulator:
	if (stk3x1x_power.regulator_vdd) {
		regulator_disable(stk3x1x_power.regulator_vdd);
		regulator_put(stk3x1x_power.regulator_vdd);
	}
	if (stk3x1x_power.regulator_vio) {
		regulator_disable(stk3x1x_power.regulator_vio);
		regulator_put(stk3x1x_power.regulator_vio);
	}
	return err;

return err;
}


static int stk3x1x_remove(struct i2c_client *client)
{
	struct stk3x1x_data *ps_data = i2c_get_clientdata(client);
	
	free_irq(ps_data->irq, ps_data);
	gpio_free(ps_data->int_pin);
	destroy_workqueue(ps_data->stk_ps_wq);	
	sysfs_remove_group(&ps_data->ps_input_dev->dev.kobj, &stk_ps_attribute_group);	
	input_unregister_device(ps_data->ps_input_dev);		
	input_free_device(ps_data->ps_input_dev);	
	wake_lock_destroy(&ps_data->ps_wakelock);	
    mutex_destroy(&ps_data->io_lock);
	kfree(ps_data);
	
    return 0;
}

static const struct i2c_device_id stk_ps_id[] =
{
    { "stk_ps", 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, stk_ps_id);

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops stk3x1x_pm_ops = {
	.suspend = stk3x1x_suspend, 
	.resume = stk3x1x_resume
};
#endif	/*	#ifdef CONFIG_PM_SLEEP	*/

static struct i2c_driver stk_ps_driver =
{
    .driver = {
        .name = DEVICE_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM_SLEEP
		.pm = &stk3x1x_pm_ops
#endif	/*	#ifdef CONFIG_PM_SLEEP	*/		
    },
    .probe = stk3x1x_probe,
    .remove = stk3x1x_remove,
    .id_table = stk_ps_id,
};


static int __init stk3x1x_init(void)
{
	int ret;
	
    ret = i2c_add_driver(&stk_ps_driver);

    if (ret)
        return ret;
	
    return 0;
}

static void __exit stk3x1x_exit(void)
{
    i2c_del_driver(&stk_ps_driver);
}

module_init(stk3x1x_init);
module_exit(stk3x1x_exit);
MODULE_AUTHOR("Lex Hsieh <lex_hsieh@sitronix.com.tw>");
MODULE_DESCRIPTION("SenseTek stk3x1x Proximity Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
