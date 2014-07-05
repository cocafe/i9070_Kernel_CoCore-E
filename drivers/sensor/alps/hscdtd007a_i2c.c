/* hscdtd007a_i2c.c
 *
 * GeoMagneticField device driver for I2C (HSCDTD007/HSCDTD008)
 *
 * Copyright (C) 2012 ALPS ELECTRIC CO., LTD. All Rights Reserved.
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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define I2C_RETRY_DELAY  5
#define I2C_RETRIES      5

#define I2C_HSCD_ADDR    (0x0c)    /* 000 1100    */
#define I2C_BUS_NUMBER   4

#define HSCD_DRIVER_NAME "hscd_i2c"

#define HSCD_STB         0x0C
#define HSCD_XOUT        0x10
#define HSCD_YOUT        0x12
#define HSCD_ZOUT        0x14
#define HSCD_XOUT_H      0x11
#define HSCD_XOUT_L      0x10
#define HSCD_YOUT_H      0x13
#define HSCD_YOUT_L      0x12
#define HSCD_ZOUT_H      0x15
#define HSCD_ZOUT_L      0x14

#define HSCD_STATUS      0x18
#define HSCD_CTRL1       0x1b
#define HSCD_CTRL2       0x1c
#define HSCD_CTRL3       0x1d
#define HSCD_CTRL4       0x1e


static struct i2c_driver hscd_driver;
static struct i2c_client *client_hscd = NULL;
#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend hscd_early_suspend_handler;
#endif

static atomic_t flgEna;
static atomic_t delay;
static atomic_t flgSuspend;

static int hscd_i2c_readm(char *rxData, int length)
{
    int err;
    int tries = 0;

    struct i2c_msg msgs[] = {
        {
            .addr  = client_hscd->addr,
            .flags = 0,
            .len   = 1,
            .buf   = rxData,
        },
        {
            .addr  = client_hscd->addr,
            .flags = I2C_M_RD,
            .len   = length,
            .buf   = rxData,
         },
    };

    do {
        err = i2c_transfer(client_hscd->adapter, msgs, 2);
    } while ((err != 2) && (++tries < I2C_RETRIES));

    if (err != 2) {
        dev_err(&client_hscd->adapter->dev, "read transfer error\n");
        err = -EIO;
    } else {
        err = 0;
    }

    return err;
}

static int hscd_i2c_writem(char *txData, int length)
{
    int err;
    int tries = 0;
#ifdef ALPS_DEBUG
    int i;
#endif

    struct i2c_msg msg[] = {
        {
            .addr  = client_hscd->addr,
            .flags = 0,
            .len   = length,
            .buf   = txData,
         },
    };

#ifdef ALPS_DEBUG
    printk("[HSCD] i2c_writem : ");
    for (i=0; i<length;i++) printk("0X%02X, ", txData[i]);
    printk("\n");
#endif

    do {
        err = i2c_transfer(client_hscd->adapter, msg, 1);
    } while ((err != 1) && (++tries < I2C_RETRIES));

    if (err != 1) {
        dev_err(&client_hscd->adapter->dev, "write transfer error\n");
        err = -EIO;
    } else {
        err = 0;
    }

    return err;
}

int hscd_self_test_A(void)
{
    u8 sx[2], cr1[1];

    if (atomic_read(&flgSuspend) == 1) return -1;
    /* Control resister1 backup  */
    cr1[0] = HSCD_CTRL1;
    if (hscd_i2c_readm(cr1, 1)) return 1;
#ifdef ALPS_DEBUG
    else printk("[HSCD] Control resister1 value, %02X\n", cr1[0]);
#endif
    mdelay(1);

    /* Stndby Mode  */
    if (cr1[0] & 0x80) {
        sx[0] = HSCD_CTRL1;
        sx[1] = 0x60;
        if (hscd_i2c_writem(sx, 2)) return 1;
    }

    /* Get inital value of self-test-A register  */
    sx[0] = HSCD_STB;
    hscd_i2c_readm(sx, 1);
    mdelay(1);
    sx[0] = HSCD_STB;
    if (hscd_i2c_readm(sx, 1)) return 1;
#ifdef ALPS_DEBUG
    else printk("[HSCD] self test A register value, %02X\n", sx[0]);
#endif
    if (sx[0] != 0x55) {
        printk("error: self-test-A, initial value is %02X\n", sx[0]);
        return 2;
    }

    /* do self-test*/
    sx[0] = HSCD_CTRL3;
    sx[1] = 0x10;
    if (hscd_i2c_writem(sx, 2)) return 1;
    mdelay(3);

    /* Get 1st value of self-test-A register  */
    sx[0] = HSCD_STB;
    if (hscd_i2c_readm(sx, 1)) return 1;
#ifdef ALPS_DEBUG
    else printk("[HSCD] self test register value, %02X\n", sx[0]);
#endif
    if (sx[0] != 0xAA) {
        printk("error: self-test, 1st value is %02X\n", sx[0]);
        return 3;
    }
    mdelay(3);

    /* Get 2nd value of self-test register  */
    sx[0] = HSCD_STB;
    if (hscd_i2c_readm(sx, 1)) return 1;
#ifdef ALPS_DEBUG
    else printk("[HSCD] self test  register value, %02X\n", sx[0]);
#endif
    if (sx[0] != 0x55) {
        printk("error: self-test, 2nd value is %02X\n", sx[0]);
        return 4;
    }

    /* Active Mode  */
    if (cr1[0] & 0x80) {
        sx[0] = HSCD_CTRL1;
        sx[1] = cr1[0];
        if (hscd_i2c_writem(sx, 2)) return 1;
    }

    return 0;
}

int hscd_self_test_B(void)
{
    if (atomic_read(&flgSuspend) == 1) return -1;
    return 0;
}

int hscd_get_magnetic_field_data(int *xyz)
{
    int err = -1;
    int i;
    u8 sx[6];

    if (atomic_read(&flgSuspend) == 1) return err;
    sx[0] = HSCD_XOUT;
    err = hscd_i2c_readm(sx, 6);
    if (err < 0) return err;
    for (i=0; i<3; i++) {
        xyz[i] = (int) ((short)((sx[2*i+1] << 8) | (sx[2*i])));
    }

#ifdef ALPS_DEBUG
    /*** DEBUG OUTPUT - REMOVE ***/
    printk("Mag_I2C, x:%d, y:%d, z:%d\n",xyz[0], xyz[1], xyz[2]);
    /*** <end> DEBUG OUTPUT - REMOVE ***/
#endif

    return err;
}

void hscd_activate(int flgatm, int flg, int dtime)
{
    u8 buf[2];

    if (flg != 0) flg = 1;

    if (flg) {
        buf[0] = HSCD_CTRL4;                       // 15 bit signed value
        buf[1] = 0x90;
        hscd_i2c_writem(buf, 2);
    }
    mdelay(1);
    
    if      (dtime <=  20) buf[1] = (3<<3);        // 100Hz- 10msec
    else if (dtime <=  70) buf[1] = (2<<3);        //  20Hz- 50msec
    else                   buf[1] = (1<<3);        //  10Hz-100msec
    buf[0]  = HSCD_CTRL1;
    buf[1] |= (flg<<7);
    hscd_i2c_writem(buf, 2);
    mdelay(3);

    if (flgatm) {
        atomic_set(&flgEna, flg);
        atomic_set(&delay, dtime);
    }
}

static void hscd_register_init(void)
{
    int v[3];
    u8  buf[2];

#ifdef ALPS_DEBUG
    printk("[HSCD] register_init\n");
#endif

    buf[0] = HSCD_CTRL3;
    buf[1] = 0x80;
    hscd_i2c_writem(buf, 2);
    mdelay(5);

    atomic_set(&delay, 100);
    hscd_activate(0, 1, atomic_read(&delay));
    hscd_get_magnetic_field_data(v);
    printk("[HSCD] x:%d y:%d z:%d\n", v[0], v[1], v[2]);
    hscd_activate(0, 0, atomic_read(&delay));
}

static int hscd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    printk("[HSCD] probe\n");
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->adapter->dev, "client not i2c capable\n");
        return -ENOMEM;
    }

    client_hscd = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
    if (!client_hscd) {
        dev_err(&client->adapter->dev, "failed to allocate memory for module data\n");
        return -ENOMEM;
    }

    dev_info(&client->adapter->dev, "detected HSCDTD007/008 geomagnetic field sensor\n");

    return 0;
}

static int __devexit hscd_remove(struct i2c_client *client)
{
    printk("[HSCD] remove\n");
    hscd_activate(0, 0, atomic_read(&delay));
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&hscd_early_suspend_handler);
#endif
    kfree(client_hscd);
    return 0;
}

static int hscd_suspend(struct i2c_client *client, pm_message_t mesg)
{
#ifdef ALPS_DEBUG
    printk("[HSCD] suspend\n");
#endif
    atomic_set(&flgSuspend, 1);
    hscd_activate(0, 0, atomic_read(&delay));
    return 0;
}

static int hscd_resume(struct i2c_client *client)
{
#ifdef ALPS_DEBUG
    printk("[HSCD] resume\n");
#endif
    atomic_set(&flgSuspend, 0);
    hscd_activate(0, atomic_read(&flgEna), atomic_read(&delay));
    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void hscd_early_suspend(struct early_suspend *handler)
{
#ifdef ALPS_DEBUG
    printk("[HSCD] early_suspend\n");
#endif
    hscd_suspend(client_hscd, PMSG_SUSPEND);
}

static void hscd_early_resume(struct early_suspend *handler)
{
#ifdef ALPS_DEBUG
    printk("[HSCD] early_resume\n");
#endif
    hscd_resume(client_hscd);
}
#endif

static const struct i2c_device_id ALPS_id[] = {
    { HSCD_DRIVER_NAME, 0 },
    { }
};

static struct i2c_driver hscd_driver = {
    .probe    = hscd_probe,
    .remove   = hscd_remove,
    .id_table = ALPS_id,
    .driver   = {
        .name = HSCD_DRIVER_NAME,
    },
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend  = hscd_suspend,
    .resume   = hscd_resume,
#endif
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend hscd_early_suspend_handler = {
    .suspend = hscd_early_suspend,
    .resume  = hscd_early_resume,
};
#endif

static int __init hscd_init(void)
{
    struct i2c_board_info i2c_info;
    struct i2c_adapter *adapter;
    int rc;

#ifdef ALPS_DEBUG
    printk("[HSCD] init\n");
#endif
    atomic_set(&flgEna, 0);
    atomic_set(&delay, 200);
    atomic_set(&flgSuspend, 0);

    rc = i2c_add_driver(&hscd_driver);
    if (rc != 0) {
        printk("can't add i2c driver\n");
        rc = -ENOTSUPP;
        return rc;
    }

    memset(&i2c_info, 0, sizeof(struct i2c_board_info));
    i2c_info.addr = I2C_HSCD_ADDR;
    strlcpy(i2c_info.type, HSCD_DRIVER_NAME , I2C_NAME_SIZE);

    adapter = i2c_get_adapter(I2C_BUS_NUMBER);
    if (!adapter) {
        printk("can't get i2c adapter %d\n", I2C_BUS_NUMBER);
        rc = -ENOTSUPP;
        goto probe_done;
    }
    client_hscd = i2c_new_device(adapter, &i2c_info);
    client_hscd->adapter->timeout = 0;
    client_hscd->adapter->retries = 0;
  
    i2c_put_adapter(adapter);
    if (!client_hscd) {
        printk("can't add i2c device at 0x%x\n",(unsigned int)i2c_info.addr);
        rc = -ENOTSUPP;  
    }

#ifdef CONFIG_HAS_EARLYSUSPEND
    register_early_suspend(&hscd_early_suspend_handler);
#endif

    hscd_register_init();

#ifdef ALPS_DEBUG
    printk("hscd_open Init end!!!!\n");
#endif
    probe_done: 

    return rc;
}

static void __exit hscd_exit(void)
{
#ifdef ALPS_DEBUG
    printk("[HSCD] exit\n");
#endif
    i2c_del_driver(&hscd_driver);
}

module_init(hscd_init);
module_exit(hscd_exit);

EXPORT_SYMBOL(hscd_self_test_A);
EXPORT_SYMBOL(hscd_self_test_B);
EXPORT_SYMBOL(hscd_get_magnetic_field_data);
EXPORT_SYMBOL(hscd_activate);

MODULE_DESCRIPTION("Alps HSCDTD Device");
MODULE_AUTHOR("ALPS ELECTRIC CO., LTD.");
MODULE_LICENSE("GPL v2");
