/*
 * Copyright (C) 2009 ST-Ericsson SA
 * Copyright (C) 2011 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/amba/bus.h>
#include <linux/amba/pl022.h>
#include <linux/amba/serial.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/i2c-gpio.h>
#include <linux/mfd/ab8500.h>
#include <linux/mfd/abx500.h>
#include <linux/regulator/ab8500.h>
#include <linux/mfd/abx500/ab8500-gpio.h>
#include <linux/input.h>
#ifdef CONFIG_SAMSUNG_JACK
#include <linux/sec_jack.h>
#include <linux/mfd/abx500.h>
#endif
#ifdef CONFIG_BATTERY_SAMSUNG
#include <linux/battery/sec_charging_common.h>
#include <linux/battery/charger/abb_sec_charger.h>
#endif
#include <linux/gpio_keys.h>
#include <linux/delay.h>
#include <linux/mfd/abx500/ab8500-denc.h>
#include <linux/spi/stm_msp.h>
#include <plat/gpio-nomadik.h>

#include <linux/leds.h>
#include <linux/mfd/abx500/ux500_sysctrl.h>
#include <video/ktd253x_bl.h>
#include <../drivers/staging/android/timed_gpio.h>

#include <linux/nfc/pn547.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <plat/i2c.h>
#include <plat/ste_dma40.h>
#include <plat/pincfg.h>

#include <mach/hardware.h>
#include <mach/setup.h>
#include <mach/devices.h>
#include <linux/input/ab8505_micro_usb_iddet.h>
#include <linux/cpuidle-dbx500.h>
#include <mach/irqs.h>
#include <mach/ste-dma40-db8500.h>
#include <linux/mfd/abx500/ab8500-pwmleds.h>
#ifdef CONFIG_PROXIMITY_GP2A
#include <mach/gp2a.h>
#endif
#ifdef CONFIG_PROXIMITY_TMD2672
#include <mach/tmd2672.h>
#endif
#ifdef CONFIG_PROXIMITY_PX3215
#include <mach/px3215.h>
#endif
#ifdef CONFIG_SENSORS_STK3X1X
#include <mach/stk3x1x.h>
#endif
#include <linux/mpu6050_input.h>
#include <mach/crypto-ux500.h>
#include <mach/pm.h>
#include <linux/yas.h>


#include <video/mcde_display.h>

#ifdef CONFIG_DB8500_MLOADER
#include <mach/mloader-dbx500.h>
#endif

#ifdef CONFIG_BT_BCM4334
#include "board-bluetooth-bcm4334.h"
#endif
#include "devices-db8500.h"
#include "board-skomer-regulators.h"
#include "pins.h"
#include "pins-db8500.h"
#include "cpu-db8500.h"
#include "board-mop500.h"	/* using some generic functions defined here */
#include "board-sec-bm.h"
#ifdef CONFIG_STE_WLAN
#include "board-mop500-wlan.h"
#endif
#include <mach/board-sec-ux500.h>
#include <linux/mfd/abx500/ab8500-gpadc.h>
#include <linux/usb_switcher.h>

#include <mach/sec_param.h>
#include <mach/sec_common.h>
#include <mach/sec_log_buf.h>

#ifdef CONFIG_USB_ANDROID
#define PUBLIC_ID_BACKUPRAM1 (U8500_BACKUPRAM1_BASE + 0x0FC0)
#define USB_SERIAL_NUMBER_LEN 31
#endif

#ifndef SSG_CAMERA_ENABLE
#define SSG_CAMERA_ENABLE
#endif

#include <linux/input/mms134s_ts.h>

unsigned int board_id;

unsigned int sec_debug_settings;
int jig_smd = 1;
EXPORT_SYMBOL(jig_smd);
int is_cable_attached;
EXPORT_SYMBOL(is_cable_attached);

int use_ab8505_iddet;
EXPORT_SYMBOL(use_ab8505_iddet);

struct device *gps_dev;
EXPORT_SYMBOL(gps_dev);

u8 hats_state = 0;

#ifdef CONFIG_ANDROID_RAM_CONSOLE

static struct resource ram_console_resource = {
	.name = "ram_console",
	.flags = IORESOURCE_MEM,
};

static struct platform_device ram_console_device = {
	.name = "ram_console",
	.id = -1,
	.num_resources = 0,
	.resource = &ram_console_resource,
};

static int __init ram_console_setup(char *p)
{
	resource_size_t ram_console_size = memparse(p, &p);

	if ((ram_console_size > 0) && (*p == '@')) {
		ram_console_resource.start = memparse(p + 1, &p);
		ram_console_resource.end =
			ram_console_resource.start + ram_console_size - 1;
		ram_console_device.num_resources = 1;
	}
	return 1;
}

__setup("mem_ram_console=", ram_console_setup);

#endif /* CONFIG_ANDROID_RAM_CONSOLE */


#if defined(CONFIG_INPUT_YAS_MAGNETOMETER)
struct yas_platform_data yas_data = {
	.hw_rev = 0,	/* hw gpio value */
};
#endif


#if defined(CONFIG_PROXIMITY_GP2A)
/* -------------------------------------------------------------------------
 * GP2A PROXIMITY SENSOR PLATFORM-SPECIFIC CODE AND DATA
 * ------------------------------------------------------------------------- */
static int __init gp2a_setup(struct device * dev);
static int __init gp2a_teardown(void);
static void gp2a_pwr(bool on);

static struct gp2a_platform_data gp2a_plat_data __initdata = {
	.ps_vout_gpio = PS_INT_HENDRIX_BRINGUP,
	.hw_setup = gp2a_setup,
	.hw_teardown = gp2a_teardown,
	.hw_pwr = gp2a_pwr,
};

static int __init gp2a_setup(struct device * dev)
{
	int err;

	err = gpio_request(PS_VLED_EN_HENDRIX_BRINGUP, "PS VLED EN");
	if (err < 0) {
		 pr_err("PS_VOUT: failed to request GPIO %d,"
                         " err %d\n", PS_VLED_EN_HENDRIX_BRINGUP, err);

                 goto err1;
	}

	return 0;
err1:
	return err;
}

static int __init gp2a_teardown(void)
{
	pr_info("%s, is called\n", __func__);
	return 0;
}

static void gp2a_pwr(bool on)
{
	pr_info("%s, power : %d", __func__, on);

	if (on) {
		gpio_direction_output(PS_VLED_EN_HENDRIX_BRINGUP, 1);
	}
	else {
		gpio_direction_output(PS_VLED_EN_HENDRIX_BRINGUP, 0);
	}
}

#endif
#if defined(CONFIG_PROXIMITY_TMD2672)

/* -------------------------------------------------------------------------
 * TMD2672 PROXIMITY SENSOR PLATFORM-SPECIFIC CODE AND DATA
 * ------------------------------------------------------------------------- */
static int __init tmd2672_setup(void);

static struct tmd2672_platform_data tmd2672_plat_data __initdata = {
	.ps_vout_gpio	= PS_INT_SKOMER_BRINGUP,
	.hw_setup	= tmd2672_setup,
	.alsout		= ADC_AUX2,
};

static int __init tmd2672_setup(void)
{
	int err;

	/* Configure the GPIO for the interrupt */
	err = gpio_request(tmd2672_plat_data.ps_vout_gpio, "PS_VOUT");
	if (err < 0) {
		pr_err("PS_VOUT: failed to request GPIO %d,"
			" err %d\n", tmd2672_plat_data.ps_vout_gpio, err);

		goto err1;
	}

	err = gpio_direction_input(tmd2672_plat_data.ps_vout_gpio);
	if (err < 0) {
		pr_err("PS_VOUT: failed to configure input"
			" direction for GPIO %d, err %d\n",
			tmd2672_plat_data.ps_vout_gpio, err);

		goto err2;
	}

	return 0;

err2:
	gpio_free(tmd2672_plat_data.ps_vout_gpio);
err1:
	return err;
}

#endif

#if defined(CONFIG_PROXIMITY_PX3215)

/* -------------------------------------------------------------------------
 * TMD2672 PROXIMITY SENSOR PLATFORM-SPECIFIC CODE AND DATA
 * ------------------------------------------------------------------------- */
static int __init px3215_setup(void);

static struct px3215_platform_data px3215_plat_data __initdata = {
	.ps_vout_gpio	= PS_INT_SKOMER_BRINGUP,
	.hw_setup	= px3215_setup,
	.alsout		= ADC_AUX2,
};

static int __init px3215_setup(void)
{
	int err;

	/* Configure the GPIO for the interrupt */
	err = gpio_request(px3215_plat_data.ps_vout_gpio, "PS_VOUT");
	if (err < 0) {
		pr_err("PS_VOUT: failed to request GPIO %d,"
			" err %d\n", px3215_plat_data.ps_vout_gpio, err);

		goto err1;
	}

	err = gpio_direction_input(px3215_plat_data.ps_vout_gpio);
	if (err < 0) {
		pr_err("PS_VOUT: failed to configure input"
			" direction for GPIO %d, err %d\n",
			px3215_plat_data.ps_vout_gpio, err);

		goto err2;
	}

	return 0;

err2:
	gpio_free(px3215_plat_data.ps_vout_gpio);
err1:
	return err;
}

#endif

#ifdef CONFIG_SENSORS_STK3X1X
static struct stk3x1x_platform_data stk3x1x_data={
	.state_reg = 0x0, /* disable all */
	.psctrl_reg = 0x31, /* ps_persistance=1, ps_gain=64X, PS_IT=0.391ms */
	.ledctrl_reg = 0xBF, /* 50mA IRDR, 64/64 LED duty */
	.wait_reg = 0x08, /* 50 ms */
	.ps_thd_h = 180, //0x0200,  [HSS]
	.ps_thd_l = 179, //0x01D0,
	.int_pin = PS_INT_SKOMER_BRINGUP,
};
#endif

#if defined(CONFIG_BATTERY_SAMSUNG)
static enum cable_type_t set_cable_status;
int abb_get_cable_status(void) {return (int)set_cable_status; }

void abb_battery_cb(void)
{
	union power_supply_propval value;
	int i, ret = 0;
	struct power_supply *psy;

	pr_info("abb_battery_cb called\n");
	set_cable_status = CABLE_TYPE_NONE;

	for (i = 0; i < 10; i++) {
		psy = power_supply_get_by_name("battery");
		if (psy)
			break;
	}
	if (i == 10) {
		pr_err("%s: fail to get battery ps\n", __func__);
		return;
	}

	value.intval = POWER_SUPPLY_TYPE_BATTERY;

	ret = psy->set_property(psy, POWER_SUPPLY_PROP_ONLINE,
		&value);
	if (ret) {
		pr_err("%s: fail to set power_suppy ONLINE property(%d)\n",
			__func__, ret);
	}
}

void abb_usb_cb(bool attached)
{
	union power_supply_propval value;
	int i, ret = 0;
	struct power_supply *psy;

	pr_info("abb_usb_cb attached %d\n", attached);
	set_cable_status = attached ? CABLE_TYPE_USB : CABLE_TYPE_NONE;

	for (i = 0; i < 10; i++) {
		psy = power_supply_get_by_name("battery");
		if (psy)
			break;
	}
	if (i == 10) {
		pr_err("%s: fail to get battery ps\n", __func__);
		return;
	}

	switch (set_cable_status) {
	case CABLE_TYPE_USB:
		value.intval = POWER_SUPPLY_TYPE_USB;
		break;
	case CABLE_TYPE_NONE:
		value.intval = POWER_SUPPLY_TYPE_BATTERY;
		break;
	default:
		pr_err("%s: invalid cable :%d\n", __func__, set_cable_status);
		return;
	}

	ret = psy->set_property(psy, POWER_SUPPLY_PROP_ONLINE,
		&value);
	if (ret) {
		pr_err("%s: fail to set power_suppy ONLINE property(%d)\n",
			__func__, ret);
	}
}

void abb_charger_cb(bool attached)
{
	union power_supply_propval value;
	int i, ret = 0;
	struct power_supply *psy;

	pr_info("abb_charger_cb attached %d\n", attached);
	set_cable_status = attached ? CABLE_TYPE_AC : CABLE_TYPE_NONE;

	for (i = 0; i < 10; i++) {
		psy = power_supply_get_by_name("battery");
		if (psy)
			break;
	}
	if (i == 10) {
		pr_err("%s: fail to get battery ps\n", __func__);
		return;
	}

	switch (set_cable_status) {
	case CABLE_TYPE_AC:
		value.intval = POWER_SUPPLY_TYPE_MAINS;
		break;
	case CABLE_TYPE_NONE:
		value.intval = POWER_SUPPLY_TYPE_BATTERY;
		break;
	default:
		pr_err("invalid status:%d\n", attached);
		return;
	}

	ret = psy->set_property(psy, POWER_SUPPLY_PROP_ONLINE,
		&value);
	if (ret) {
		pr_err("%s: fail to set power_suppy ONLINE property(%d)\n",
			__func__, ret);
	}
}

void abb_jig_cb(bool attached)
{
	pr_info("abb_jig_cb attached %d\n", attached);

	set_cable_status = attached ? CABLE_TYPE_JIG : CABLE_TYPE_NONE;
}

void abb_uart_cb(bool attached)
{
	pr_info("abb_uart_cb attached %d\n", attached);

	set_cable_status = attached ? CABLE_TYPE_UARTOFF : CABLE_TYPE_NONE;

}

void abb_dock_cb(bool attached)
{
	pr_info("abb_dock_cb attached %d\n", attached);
	set_cable_status = attached ? CABLE_TYPE_CARDOCK : CABLE_TYPE_NONE;
}
#endif 

#if defined(CONFIG_USB_SWITCHER)
static struct usb_switch fsa880_data =	{
		.name					=	"FSA880",
		.id	 				=	0x0	,
		.id_mask				=	0xff	,
		.control_register_default		=	0x05	,
		.control_register_inital_value  	=	0x1e	,
		.connection_changed_interrupt_gpio	=	95	,
		.charger_detect_gpio			=	0xffff 	, /*no charger detect gpio for this device*/
		.valid_device_register_1_bits		=	0xEF	,
		.valid_device_register_2_bits		=	0xFF	,	
		.valid_registers			=	{0,1,1,1,1,0,0,1,0,0,1,1,0,0,0,0, 0, 0, 0, 1, 1  },
};
#endif


static struct pn547_i2c_platform_data rx71_nfc_data = {
	.irq_gpio = NFC_IRQ_SKOMER_BRINGUP,
	.ven_gpio = NFC_EN_SKOMER_BRINGUP,
	.firm_gpio = NFC_FIRM_SKOMER_BRINGUP,
};

static void mms_ts_int_set_pull(bool to_up)
{
	int ret;
	int pull = (to_up) ? NMK_GPIO_PULL_UP : NMK_GPIO_PULL_DOWN;
	ret = nmk_gpio_set_pull(TSP_INT_HENDRIX_BRINGUP, pull);
	if (ret < 0)
		printk(KERN_ERR "%s: fail to set pull xx on interrupt pin\n", __func__);
}
static int mms_ts_pin_configure(bool to_gpios)
{
	/* TOUCH_EN is always an output */
	if (to_gpios) {
		gpio_direction_output(TSP_INT_HENDRIX_BRINGUP, 0);
		nmk_gpio_set_mode(TSP_SCL_HENDRIX_BRINGUP, NMK_GPIO_ALT_GPIO);
		gpio_direction_output(TSP_SCL_HENDRIX_BRINGUP, 0);
		nmk_gpio_set_mode(TSP_SDA_HENDRIX_BRINGUP, NMK_GPIO_ALT_GPIO);
		gpio_direction_output(TSP_SDA_HENDRIX_BRINGUP, 0);
	} else {
		gpio_direction_output(TSP_INT_HENDRIX_BRINGUP, 1);
		gpio_direction_input(TSP_INT_HENDRIX_BRINGUP);
		gpio_direction_output(TSP_SCL_HENDRIX_BRINGUP, 1);
		nmk_gpio_set_mode(TSP_SCL_HENDRIX_BRINGUP, NMK_GPIO_ALT_C);
		gpio_direction_output(TSP_SDA_HENDRIX_BRINGUP, 1);
		nmk_gpio_set_mode(TSP_SDA_HENDRIX_BRINGUP, NMK_GPIO_ALT_C);
	}
	return 0;
}

static void mms_ts_vdd_on(struct device *dev, bool on)
{
	static struct regulator *tsp_reg;
	static struct regulator *io_reg;
	static bool reg_enabled;
	int ret = 0;

	if (!tsp_reg) {
		tsp_reg = regulator_get(dev, "v-tsp-3.3");
		if (IS_ERR(tsp_reg)) {
			printk(KERN_ERR "[%s] Failed to get v-tsp-3.3 regulator for TSP\n",__func__);
			tsp_reg = NULL;
		}
	}

	if (!io_reg) {
		io_reg = regulator_get(dev, "v-tsp-1.8");
		if (IS_ERR(io_reg)) {
			printk(KERN_ERR "[%s] Failed to get v-tsp-1.8 regulator for TSP\n", __func__);
			io_reg = NULL;
		}
	}
        
	if (on & !reg_enabled) {
		ret = regulator_enable(io_reg);
		ret = regulator_enable(tsp_reg);
	}
	else if (reg_enabled) {
		ret = regulator_disable(io_reg);
		ret = regulator_disable(tsp_reg);
	}

	if (ret < 0)
		printk(KERN_ERR "Failed to enable or disable v-tsp_3.3 (%d)\n", ret);
	else
		reg_enabled = on;
}

static unsigned int mms_ts_key_map[] = {KEY_MENU, KEY_BACK};

struct mms_ts_platform_data mms134s_ts_pdata = {
	.max_x      = 480,
	.max_y      = 800,

	.num_rx     = 12,   /* number of RX channels in chip */
	.num_tx     = 22,   /* number of TX channels in chip */
	.fw_ver_reg = 0xF3, /* Register address for Firmware version */
	.max_fingers    = 5,    /* supported by firmware */
	.gpio_sda   = TSP_SDA_HENDRIX_BRINGUP,   /* for Firmware update */
	.gpio_scl   = TSP_SCL_HENDRIX_BRINGUP,
	.gpio_int   = TSP_INT_HENDRIX_BRINGUP,
	.pin_configure  = mms_ts_pin_configure,
	.fw_name_ums    = "/sdcard/firmware/melfas/mms134_ts.fw.bin",
	//  .fw_name_builtin = "MDH_KYLE_EU_R10_VD34.fw",
	.key_map    = mms_ts_key_map,
	.key_nums   = ARRAY_SIZE(mms_ts_key_map),
	.pin_set_pull   = mms_ts_int_set_pull,
	.vdd_on     = mms_ts_vdd_on,
};

void __init mms134s_ts_init(void)
{
	gpio_request(TSP_INT_HENDRIX_BRINGUP, "tsp_int_n");
	gpio_direction_input(TSP_INT_HENDRIX_BRINGUP);

	gpio_request(TSP_SCL_HENDRIX_BRINGUP, "ap_i2c3_scl");
	gpio_request(TSP_SDA_HENDRIX_BRINGUP, "ap_i2c3_sda");
}

static struct i2c_board_info __initdata skomer_bringup_i2c0_devices[] = {
#if defined(CONFIG_PROXIMITY_PX3215)
	{
		/* TMD2672 proximity sensor */
		I2C_BOARD_INFO("dyna", 0x1e),
		/*.irq = GPIO_TO_IRQ(PS_INT_SKOMER_BRINGUP),*/
		.platform_data = &px3215_plat_data,
	},
#endif
#ifdef CONFIG_SENSORS_STK3X1X
	{
		I2C_BOARD_INFO("stk_ps", 0x90 >>1),
		.platform_data = &stk3x1x_data,
	//	.irq = GPIO_TO_IRQ(PS_INT_SKOMER_BRINGUP),
	},
#endif

};

static struct i2c_board_info __initdata skomer_bringup_i2c1_devices[] = {
#if 0
#if defined(CONFIG_USB_SWITCHER)
	{
		I2C_BOARD_INFO("musb", 0x25),
		.platform_data = &fsa880_data ,
		.irq = GPIO_TO_IRQ(JACK_NINT_SKOMER_BRINGUP),
	},
#endif
#endif
};

static struct i2c_board_info __initdata skomer_bringup_i2c2_devices[] = {
#if 0
#if defined(CONFIG_ACCEL_BMA222)
	{
		/* BMA222 accelerometer driver */
		I2C_BOARD_INFO("accelerometer", 0x08),
	},
#endif
#endif
};

static struct i2c_board_info __initdata hendrix_bringup_i2c3_devices[] = {
	{
		I2C_BOARD_INFO("mms_ts", 0x48),
		.platform_data  = &mms134s_ts_pdata,
		.irq = GPIO_TO_IRQ(TSP_INT_HENDRIX_BRINGUP),
	},
};
static struct i2c_gpio_platform_data skomer_gpio_i2c4_data = {
	.sda_pin = SUBPMU_SDA_SKOMER_BRINGUP,
	.scl_pin = SUBPMU_SCL_SKOMER_BRINGUP,
	.udelay = 3,	/* closest to 400KHz */
};

static struct platform_device skomer_gpio_i2c4_pdata = {
	.name = "i2c-gpio",
	.id = 4,
	.dev = {
		.platform_data = &skomer_gpio_i2c4_data,
	},
};

static struct i2c_board_info __initdata skomer_bringup_gpio_i2c4_devices[] = {
{
	/* ncp6914 power management IC for the cameras */
	I2C_BOARD_INFO("ncp6914", 0x10),
	//.platform_data = &ncp6914_plat_data,
},
};

static struct i2c_board_info __initdata skomer_bringup_gpio_i2c4_devices_r0[] = {
#ifdef CONFIG_SENSORS_HSCD
	{
		/* ALPS Magnetometer driver */
		I2C_BOARD_INFO("hscd_i2c", 0x0c),

	},
#endif
};

static struct i2c_gpio_platform_data skomer_gpio_i2c5_data = {
	.sda_pin = AGC_I2C_SDA_SKOMER_BRINGUP,
	.scl_pin = AGC_I2C_SCL_SKOMER_BRINGUP,
	.udelay = 3,	/* closest to 400KHz */
};

static struct platform_device skomer_gpio_i2c5_pdata = {
	.name = "i2c-gpio",
	.id = 5,
	.dev = {
		.platform_data = &skomer_gpio_i2c5_data,
	},
};

static struct i2c_board_info __initdata skomer_bringup_gpio_i2c5_devices[] = {
/* TBD - NFC */
#if defined(CONFIG_PROXIMITY_GP2A)
	{
		/* GP2A proximity sensor */
		I2C_BOARD_INFO(GP2A_I2C_DEVICE_NAME, 0x44),
		.platform_data = &gp2a_plat_data,
	},
#endif
#ifdef CONFIG_SENSORS_STK3X1X
	{
		I2C_BOARD_INFO("stk_ps", 0x90 >>1),
		.platform_data = &stk3x1x_data,
		.irq = GPIO_TO_IRQ(PS_INT_SKOMER_BRINGUP),
	},
#endif
};

static struct i2c_gpio_platform_data skomer_gpio_i2c6_data = {
	.sda_pin = SENSOR_SDA_SKOMER_BRINGUP,
	.scl_pin = SENSOR_SCL_SKOMER_BRINGUP,
	.udelay = 3,	/* closest to 400KHz */
};

static struct platform_device skomer_gpio_i2c6_pdata = {
	.name = "i2c-gpio",
	.id = 6,
	.dev = {
	.platform_data = &skomer_gpio_i2c6_data,
	},
};

static struct i2c_board_info __initdata skomer_bringup_gpio_i2c6_devices[] = {
#ifdef CONFIG_SENSORS_BMA254 
		{
			I2C_BOARD_INFO("bma254", 0x18),
		},
#endif
};

static struct platform_device skomer_gpio_i2c6_pdata_01 = {
	.name = "i2c-gpio",
	.id = 6,
	.dev = {
	.platform_data = &skomer_gpio_i2c6_data,
	},
};

static struct mpu6050_input_platform_data mpu6050_pdata = {
	.orientation = {0, -1, 0,
				1, 0, 0,
				0, 0, 1},
	.acc_cal_path = "/efs/calibration_data",
	.gyro_cal_path = "/efs/gyro_cal_data",
};

static struct i2c_board_info __initdata skomer_bringup_gpio_i2c6_devices_01[] = {
{
			I2C_BOARD_INFO("mpu6050_input", 0x68),
				.irq = GPIO_TO_IRQ(SENSORS_INT_SKOMER_BRINGUP),
				.platform_data = &mpu6050_pdata,
},

};

static struct i2c_board_info __initdata skomer_bringup_gpio_i2c7_devices[] = {
};

static struct i2c_gpio_platform_data skomer_gpio_i2c7_data = {
	.sda_pin = TOUCHKEY_SDA_SKOMER_BRINGUP,
	.scl_pin = TOUCHKEY_SCL_SKOMER_BRINGUP,
	.udelay = 5,
};

static struct platform_device skomer_gpio_i2c7_pdata = {
	.name = "i2c-gpio",
	.id = 7,
	.dev = {
		.platform_data = &skomer_gpio_i2c7_data,
	},
};

static struct i2c_gpio_platform_data skomer_gpio_i2c8_data = {
	.sda_pin = COMP_SDA_SKOMER_BRINGUP,
	.scl_pin = COMP_SCL_SKOMER_BRINGUP,
	.udelay = 3,	/* closest to 400KHz */
};

static struct platform_device skomer_gpio_i2c8_pdata = {
	.name = "i2c-gpio",
	.id = 8,
	.dev = {
		.platform_data = &skomer_gpio_i2c8_data,
	},
};

static struct platform_device alps_pdata = {
	.name = "alps-input",
	.id = -1,
};

static struct i2c_board_info __initdata skomer_bringup_gpio_i2c8_devices[] = {
	{
		/* ALPS Magnetometer driver */
		I2C_BOARD_INFO("hscd_i2c", 0x0c),

},
};

/* I2C GPIO: NFC */
static struct i2c_gpio_platform_data skomer_gpio_i2c9_data = {
	.sda_pin = NFC_SDA_18V_SKOMER_BRINGUP,
	.scl_pin = NFC_SCL_18V_SKOMER_BRINGUP,
	.udelay = 3,	/* closest to 400KHz */
};

static struct platform_device skomer_gpio_i2c9_pdata = {
	.name = "i2c-gpio",
	.id = 9,
	.dev = {
		.platform_data = &skomer_gpio_i2c9_data,
	},
};

static struct i2c_board_info __initdata skomer_bringup_gpio_i2c9_devices[] = {
	{
		/* I2C GPIO: NFC PN547 */
		I2C_BOARD_INFO("pn547", 0x2B), /* 0x28, 0x29, 0x2A, 0x2B */
		.platform_data	= &rx71_nfc_data,
		.irq = GPIO_TO_IRQ(NFC_IRQ_SKOMER_BRINGUP),
	},
};

#ifdef CONFIG_KEYBOARD_GPIO
struct gpio_keys_button skomer_bringup_gpio_keys[] = {
	{
	.code = KEY_HOMEPAGE,		/* input event code (KEY_*, SW_*) */
	.gpio = HOME_KEY_HENDRIX_BRINGUP,
	.active_low = 1,
	.desc = "home_key",
	.type = EV_KEY,		/* input event type (EV_KEY, EV_SW) */
	.wakeup = 1,		/* configure the button as a wake-up source */
	.debounce_interval = 30,	/* debounce ticks interval in msecs */
	.can_disable = false,
	},
	{
	.code = KEY_VOLUMEUP,		/* input event code (KEY_*, SW_*) */
	.gpio = VOL_UP_HENDRIX_BRINGUP,
	.active_low = 1,
	.desc = "volup_key",
	.type = EV_KEY,		/* input event type (EV_KEY, EV_SW) */
	.wakeup = 0,
	.debounce_interval = 30,	/* debounce ticks interval in msecs */
	.can_disable = false,
	},
	{
	.code = KEY_VOLUMEDOWN,		/* input event code (KEY_*, SW_*) */
	.gpio = VOL_DOWN_HENDRIX_BRINGUP,
	.active_low = 1,
	.desc = "voldown_key",
	.type = EV_KEY,		/* input event type (EV_KEY, EV_SW) */
	.wakeup = 0,		/* configure the button as a wake-up source */
	.debounce_interval = 30,	/* debounce ticks interval in msecs */
	.can_disable = false,
	},
};

struct gpio_keys_platform_data skomer_bringup_gpio_data = {
	.buttons = skomer_bringup_gpio_keys,
	.nbuttons = ARRAY_SIZE(skomer_bringup_gpio_keys),
};

struct platform_device skomer_gpio_keys_device = {
	.name = "gpio-keys",
	.dev = {
		.platform_data = &skomer_bringup_gpio_data,
	},
};
#endif


#ifdef CONFIG_USB_ANDROID
/*
static char *usb_functions_adb[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
#ifdef CONFIG_USB_ANDROID_ECM
	"cdc_ethernet",
#endif
};
*/

static char *usb_functions_ums[] = {
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	"usb_mass_storage",
#endif
};

static char *usb_functions_rndis[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
};

static char *usb_functions_phonet[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_PHONET
	"phonet",
#endif
};

static char *usb_functions_ecm[] = {
#ifdef CONFIG_USB_ANDROID_ECM
	"cdc_ethernet",
#endif
};

#ifdef CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE
/* soonyong.cho : Variables for samsung composite
		such as kies, mtp, ums, etc... */
/* kies mode */
static char *usb_functions_acm_mtp[] = {
	"mtp",
	"acm",
};

#ifdef CONFIG_USB_ANDROID_ECM /* Temp !! will  be deleted 2011.04.12 */
/*Temp debug mode */
static char *usb_functions_acm_mtp_adb[] = {
	"mtp",
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
	"cdc_ethernet",
};
#else
/* debug mode */
static char *usb_functions_acm_mtp_adb[] = {
	"mtp",
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
};
#endif

/* mtp only mode */
static char *usb_functions_mtp[] = {
	"mtp",
};

#else /* android original composite*/
static char *usb_functions_ums_adb[] = {
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
};
#endif

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE
/* soonyong.cho :	Every function driver for samsung composite.
 *			Number of enable function features have to
 *			be same as below.
 */
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_ECM
	"cdc_ethernet",
#endif
#ifdef CONFIG_USB_ANDROID_SAMSUNG_MTP
	"mtp",
#endif
#ifdef CONFIG_USB_ANDROID_PHONET
	"phonet",
#endif
#else /* original */
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#endif /* CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE */
};


static struct android_usb_product usb_products[] = {
#ifdef CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE
	/* soonyong.cho : Please modify below value correctly
	if you customize composite */
#ifdef CONFIG_USB_ANDROID_SAMSUNG_ESCAPE /* USE DEVGURU HOST DRIVER */
	{
		.product_id	= SAMSUNG_KIES_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_acm_mtp_adb),
		.functions	= usb_functions_acm_mtp_adb,
		.bDeviceClass		= 0xEF,
		.bDeviceSubClass	= 0x02,
		.bDeviceProtocol	= 0x01,
		.s			= ANDROID_DEBUG_CONFIG_STRING,
		.mode			= USBSTATUS_ADB,
	},
	{
		.product_id		= SAMSUNG_KIES_PRODUCT_ID,
		.num_functions		= ARRAY_SIZE(usb_functions_acm_mtp),
		.functions		= usb_functions_acm_mtp,
		.bDeviceClass		= 0xEF,
		.bDeviceSubClass	= 0x02,
		.bDeviceProtocol	= 0x01,
		.s			= ANDROID_KIES_CONFIG_STRING,
		.mode			= USBSTATUS_SAMSUNG_KIES,
	},
	{
		.product_id		= SAMSUNG_UMS_PRODUCT_ID,
		.num_functions		= ARRAY_SIZE(usb_functions_ums),
		.functions		= usb_functions_ums,
		.bDeviceClass		= USB_CLASS_PER_INTERFACE,
		.bDeviceSubClass	= 0,
		.bDeviceProtocol	= 0,
		.s			= ANDROID_UMS_CONFIG_STRING,
		.mode			= USBSTATUS_UMS,
	},
	{
		.product_id		= SAMSUNG_RNDIS_PRODUCT_ID,
		.num_functions		= ARRAY_SIZE(usb_functions_rndis),
		.functions		= usb_functions_rndis,
#ifdef CONFIG_USB_ANDROID_SAMSUNG_RNDIS_WITH_MS_COMPOSITE
		.bDeviceClass		= 0xEF,
		.bDeviceSubClass	= 0x02,
		.bDeviceProtocol	= 0x01,
#else
#ifdef CONFIG_USB_ANDROID_RNDIS_WCEIS
		.bDeviceClass		= USB_CLASS_WIRELESS_CONTROLLER,
#else
		.bDeviceClass		= USB_CLASS_COMM,
#endif
		.bDeviceSubClass	= 0,
		.bDeviceProtocol	= 0,
#endif
		.s			= ANDROID_RNDIS_CONFIG_STRING,
		.mode			= USBSTATUS_VTP,
	},
#ifdef CONFIG_USB_ANDROID_PHONET
	{
		.product_id		= SAMSUNG_PHONET_PRODUCT_ID,
		.num_functions		= ARRAY_SIZE(usb_functions_phonet),
		.functions		= usb_functions_phonet,
		.bDeviceClass		= USB_CLASS_CDC_DATA,
		.bDeviceSubClass	= 0,
		.bDeviceProtocol	= 0,
		.s			= ANDROID_PHONET_CONFIG_STRING,
		.mode			= USBSTATUS_PHONET,
	},
#endif
	{
		.product_id		= SAMSUNG_MTP_PRODUCT_ID,
		.num_functions		= ARRAY_SIZE(usb_functions_mtp),
		.functions		= usb_functions_mtp,
		.bDeviceClass		= USB_CLASS_PER_INTERFACE,
		.bDeviceSubClass	= 0,
		.bDeviceProtocol	= 0x01,
		.s			= ANDROID_MTP_CONFIG_STRING,
		.mode			= USBSTATUS_MTPONLY,
	},

	/*
	{
		.product_id	= 0x685d,
		.num_functions	= ARRAY_SIZE(usb_functions_adb),
		.functions	= usb_functions_adb,
	},

	*/
#else /* USE MCCI HOST DRIVER */
	{
		.product_id = SAMSUNG_KIES_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_acm_mtp_adb),
		.functions	= usb_functions_acm_mtp_adb,
		.bDeviceClass		= USB_CLASS_COMM,
		.bDeviceSubClass	= 0,
		.bDeviceProtocol	= 0,
		.s			= ANDROID_DEBUG_CONFIG_STRING,
		.mode			= USBSTATUS_ADB,
	},
	{
		.product_id		= SAMSUNG_KIES_PRODUCT_ID,
		.num_functions		= ARRAY_SIZE(usb_functions_acm_mtp),
		.functions		= usb_functions_acm_mtp,
		.bDeviceClass		= USB_CLASS_COMM,
		.bDeviceSubClass	= 0,
		.bDeviceProtocol	= 0,
		.s			= ANDROID_KIES_CONFIG_STRING,
		.mode			= USBSTATUS_SAMSUNG_KIES,
	},
	{
		.product_id		= SAMSUNG_UMS_PRODUCT_ID,
		.num_functions		= ARRAY_SIZE(usb_functions_ums),
		.functions		= usb_functions_ums,
		.bDeviceClass		= USB_CLASS_PER_INTERFACE,
		.bDeviceSubClass	= 0,
		.bDeviceProtocol	= 0,
		.s			= ANDROID_UMS_CONFIG_STRING,
		.mode			= USBSTATUS_UMS,
	},
	{
		.product_id		= SAMSUNG_RNDIS_PRODUCT_ID,
		.num_functions		= ARRAY_SIZE(usb_functions_rndis),
		.functions		= usb_functions_rndis,
#ifdef CONFIG_USB_ANDROID_RNDIS_WCEIS
		.bDeviceClass		= USB_CLASS_WIRELESS_CONTROLLER,
#else
		.bDeviceClass		= USB_CLASS_COMM,
#endif
		.bDeviceSubClass	= 0,
		.bDeviceProtocol	= 0,
		.s			= ANDROID_RNDIS_CONFIG_STRING,
		.mode			= USBSTATUS_VTP,
	},
	{
		.product_id		= SAMSUNG_MTP_PRODUCT_ID,
		.num_functions		= ARRAY_SIZE(usb_functions_mtp),
		.functions		= usb_functions_mtp,
		.bDeviceClass		= USB_CLASS_PER_INTERFACE,
		.bDeviceSubClass	= 0,
		.bDeviceProtocol	= 0x01,
		.s			= ANDROID_MTP_CONFIG_STRING,
		.mode			= USBSTATUS_MTPONLY,
	},
#endif
#else  /* original android composite */
	{
		.product_id = ANDROID_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
	},
	{
		.product_id = ANDROID_ADB_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions	= usb_functions_ums_adb,
	},
#endif
};

static char android_usb_serial_num[USB_SERIAL_NUMBER_LEN] = "0123456789ABCDEF";

static struct android_usb_platform_data android_usb_pdata = {
#ifdef CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE
	.vendor_id	= SAMSUNG_VENDOR_ID,
	.product_id	= SAMSUNG_DEBUG_PRODUCT_ID,
#else
	.vendor_id	= ANDROID_VENDOR_ID,
	.product_id = ANDROID_PRODUCT_ID,
#endif
	.version	= 0x0100,
#ifdef CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE
	.product_name	= "SAMSUNG_Android",
	.manufacturer_name = "SAMSUNG",
#else
	.product_name	= "Android Phone",
	.manufacturer_name = "Android",
#endif
	.serial_number	= android_usb_serial_num,
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
	.platform_data = &android_usb_pdata,
	},
};

#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 2,
	.vendor		= "Samsung",
	.product	= "Android Phone",
	.release	= 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};
#endif /* CONFIG_USB_ANDROID_MASS_STORAGE */

#ifdef CONFIG_USB_ANDROID_ECM
static struct usb_ether_platform_data usb_ecm_pdata = {
	.ethaddr	= {0x02, 0x11, 0x22, 0x33, 0x44, 0x55},
	.vendorID	= 0x04e8,
	.vendorDescr = "Samsung",
};

struct platform_device usb_ecm_device = {
	.name	= "cdc_ethernet",
	.id	= -1,
	.dev	= {
		.platform_data = &usb_ecm_pdata,
	},
};
#endif /* CONFIG_USB_ANDROID_ECM */

#ifdef CONFIG_USB_ANDROID_RNDIS
static struct usb_ether_platform_data usb_rndis_pdata = {
	.ethaddr	= {0x01, 0x11, 0x22, 0x33, 0x44, 0x55},
	.vendorID	= SAMSUNG_VENDOR_ID,
	.vendorDescr = "Samsung",
};

struct platform_device usb_rndis_device = {
	.name = "rndis",
	.id = -1,
	.dev = {
		.platform_data = &usb_rndis_pdata,
	},
};
#endif /* CONFIG_USB_ANDROID_RNDIS */

#ifdef CONFIG_USB_ANDROID_PHONET
static struct usb_ether_platform_data usb_phonet_pdata = {
	.vendorID	= SAMSUNG_VENDOR_ID,	/* PHONET_VENDOR_ID */
	.vendorDescr = "Samsung",
};

struct platform_device usb_phonet_device = {
	.name = "phonet",
	.id = -1,
	.dev = {
		.platform_data = &usb_phonet_pdata,
	},
};
#endif /* CONFIG_USB_ANDROID_PHONET */

#endif /* CONFIG_USB_ANDROID */

#define U8500_I2C_CONTROLLER(id, _slsu, _tft, _rft, clk, t_out, _sm) \
static struct nmk_i2c_controller skomer_i2c##id##_data = { \
	/*				\
	 * slave data setup time, which is	\
	 * 250 ns,100ns,10ns which is 14,6,2	\
	 * respectively for a 48 Mhz	\
	 * i2c clock			\
	 */				\
	.slsu		= _slsu,	\
	/* Tx FIFO threshold */		\
	.tft		= _tft,		\
	/* Rx FIFO threshold */		\
	.rft		= _rft,		\
	/* std. mode operation */	\
	.clk_freq	= clk,		\
	/* Slave response timeout(ms) */\
	.timeout	= t_out,	\
	.sm		= _sm,		\
}

/*
 * The board uses 4 i2c controllers, initialize all of
 * them with slave data setup time of 250 ns,
 * Tx & Rx FIFO threshold values as 1 and standard
 * mode of operation
 */
U8500_I2C_CONTROLLER(0, 0xe, 1, 8, 400000, 200, I2C_FREQ_MODE_FAST);
U8500_I2C_CONTROLLER(1, 0xe, 1, 8, 400000, 200, I2C_FREQ_MODE_FAST);
U8500_I2C_CONTROLLER(2, 0xe, 1, 8, 400000, 200, I2C_FREQ_MODE_FAST);
U8500_I2C_CONTROLLER(3, 0xe, 1, 8, 400000, 200, I2C_FREQ_MODE_FAST);

/*
 * SSP
 */
#define NUM_SPI_CLIENTS 1
static struct pl022_ssp_controller skomer_spi0_data = {
	.bus_id		= SPI023_0_CONTROLLER,
	.num_chipselect	= NUM_SPI_CLIENTS,
};

static struct spi_board_info spi_board_info[] __initdata = {
	{
		.modalias		= "pri_lcd_spi",
		.max_speed_hz		= 1200000,
		.bus_num		= SPI023_0_CONTROLLER,
		.chip_select		= 0,
		.mode			= SPI_MODE_3,
//		.controller_data	= (void *)LCD_CSX_SKOMER_BRINGUP,
	},
};

static struct spi_gpio_platform_data skomer_spi_gpio_data = {
//	.sck		= LCD_CLK_SKOMER_BRINGUP,	/* LCD_CLK */
//	.mosi		= LCD_SDI_SKOMER_BRINGUP,	/* LCD_SDI */
//	.miso		= LCD_SDO_SKOMER_BRINGUP,	/* LCD_SDO */
	.num_chipselect	= 2,
};


static struct platform_device ux500_spi_gpio_device = {
	.name	= "spi_gpio",
	.id	= SPI023_0_CONTROLLER,
	.dev	= {
		.platform_data = &skomer_spi_gpio_data,
	},
};


static struct ab8500_gpio_platform_data ab8500_gpio_pdata = {
	.gpio_base		= AB8500_PIN_GPIO(1),
	.irq_base		= MOP500_AB8500_VIR_GPIO_IRQ_BASE,
	/* initial_pin_config is the initial configuration of ab8500 pins.
	 * The pins can be configured as GPIO or alt functions based
	 * on value present in GpioSel1 to GpioSel6 and AlternatFunction
	 * register. This is the array of 7 configuration settings.
	 * One has to compile time decide these settings. Below is the
	 * explaination of these setting
	 * GpioSel1 = 0xFF => 1-4 should be GPIO input, 5 Res,
	 *		      6-8 should be GPIO input
	 * GpioSel2 = 0xF1 => 9-13 GND, 14-16 NC. 9 is PD GND.
	 * GpioSel3 = 0x80 => Pin GPIO24 is configured as GPIO
	 * GpioSel4 = 0x75 => 25 is SYSCLKREQ8, but NC, should be GPIO
	 * GpioSel5 = 0x7A => Pins GPIO34, GPIO36 to GPIO39 are conf as GPIO
	 * GpioSel6 = 0x02 => 42 is NC
	 * AlternaFunction = 0x03 => If Pins GPIO10 to 13 are not configured
	 * as GPIO then this register selects the alternate fucntions
	 */
	.config_reg = {0xFF, 0xFF, 0x81, 0xFD, 0x7A, 0x02, 0x03},

	/* initial_pin_direction allows for the initial GPIO direction to
	 * be set.
	 */
	.config_direction = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

	/*
	 * initial_pin_pullups allows for the intial configuration of the
	 * GPIO pullup/pulldown configuration.
	 */
	.config_pullups = {0xE0, 0x1F, 0x00, 0x00, 0x80, 0x00},
};

static struct ab8500_gpio_platform_data ab8505_gpio_pdata = {
	.gpio_base		= AB8500_PIN_GPIO(1),
	.irq_base		= MOP500_AB8500_VIR_GPIO_IRQ_BASE,
	/*
	 * config_reg is the initial configuration of ab8505 pins.
	 * The pins can be configured as GPIO or alt functions based
	 * on value present in GpioSel1 to GpioSel6 and AlternatFunction
	 * register. This is the array of 8 configuration settings.
	 * One has to compile time decide these settings. Below is the
	 * explanation of these setting
	 * GpioSel1 = 0x0B => Pin GPIO1 (SysClkReq2)
	 *                    Pin GPIO2 (SysClkReq3)
	 *                    Pin GPIO3 (SysClkReq4)
	 *                    Pin GPIO4 (SysClkReq6) are configured as GPIO
	 * GpioSel2 = 0x26 => Pins GPIO10,11 HW_REV_MOD_0,1
	 * 		      Pin GPIO13, IF_TXD
	 *                    Pin GPIO14, NC
	 *                    Pins GPIO15,16 NotAvail
	 * GpioSel3 = 0x00 => Pins GPIO17-20 AD_Data2, DA_Data2, Fsync2, BitClk2
         *                    Pins GPIO21-24 NA
	 * GpioSel4 = 0x00 => Pins GPIO25, 27-32 NotAvail
	 * GpioSel5 = 0x02 => Pin GPIO34 (ExtCPEna) NC
	 *		      Pin GPIO40 (ModScl) I2C_MODEM_SCL
	 * GpioSel6 = 0x00 => Pin GPIO41 (ModSda) I2C_MODEM_SDA
         *                    Pin GPIO42 NotAvail
	 * GpioSel7 = 0x00 => Pin GPIO50, IF_RXD
         *                    Pins GPIO51 & 60 NotAvail
         *                    Pin GPIO52 (RestHW) RST_AB8505
         *                    Pin GPIO53 (Service) Service_AB8505
	 * AlternatFunction = 0x0C => GPIO13, 50 UartTX, RX
	 *
	 */
	.config_reg     = {0x0B, 0x26, 0x00, 0x00, 0x02, 0x00, 0x00, 0x0C},

	/*
	 * config_direction allows for the initial GPIO direction to
	 * be set.
	 */
	.config_direction  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

	/*
	 * config_pullups allows for the intial configuration of the
	 * GPIO pullup/pulldown configuration.
 	 * GPIO2/3(GpioPud1) = 1 and GPIO10/11(GpioPud2) = 1.
	 * GPIO13(GpioPud2) = 1 and GPIO50(GpioPud7) = 1.
	 */
	.config_pullups    = {0xE7, 0x17, 0x00, 0x00, 0x00, 0x00, 0x06},
};


static struct ab8500_sysctrl_platform_data ab8500_sysctrl_pdata = {
	/*
	 * SysClkReq1RfClkBuf - SysClkReq8RfClkBuf
	 * The initial values should not be changed because of the way
	 * the system works today
	 */
	.initial_req_buf_config
			= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
};


#ifdef CONFIG_SAMSUNG_JACK
static struct sec_jack_zone sec_jack_zones[] = {
	{
		/* adc == 0, default to 3pole if it stays
		 * in this range for 40ms (20ms delays, 2 samples)
		 */
		.adc_high = 0,
		.delay_ms = 20,
		.check_count = 2,
		.jack_type = SEC_HEADSET_3POLE,
	},
	{
		/* 0 < adc <= 660, unstable zone, default to 3pole if it stays
		 * in this range for a 600ms (30ms delays, 20 samples)
		 */
		.adc_high = 660,
		.delay_ms = 30,
		.check_count = 20,
		.jack_type = SEC_HEADSET_3POLE,
	},
	{
		/* 660 < adc <= 1000, unstable zone, default to 4pole if it
		 * stays in this range for 900ms (30ms delays, 30 samples)
		 */
		.adc_high = 1000,
		.delay_ms = 30,
		.check_count = 30,
		.jack_type = SEC_HEADSET_4POLE,
	},
	{
		/* 1000 < adc <= 1850, default to 4 pole if it stays */
		/* in this range for 40ms (20ms delays, 2 samples)
		 */
		.adc_high = 1850,
		.delay_ms = 20,
		.check_count = 2,
		.jack_type = SEC_HEADSET_4POLE,
	},
	{
		/* adc > 1850, unstable zone, default to 3pole if it stays
		 * in this range for a second (10ms delays, 100 samples)
		 */
		.adc_high = 0x7fffffff,
		.delay_ms = 10,
		.check_count = 100,
		.jack_type = SEC_HEADSET_3POLE,
	},
};

/* to support 3-buttons earjack */
static struct sec_jack_buttons_zone sec_jack_buttons_zones[] = {
	{
		/* 0 <= adc <=100, stable zone */
		.code		= KEY_MEDIA,
		.adc_low	= 0,
		.adc_high	= 100,
	},
	{
		/* 101 <= adc <= 225, stable zone */
		.code		= KEY_VOLUMEUP,
		.adc_low	= 101,
		.adc_high	= 225,
	},
	{
		/* 226 <= adc <= 450, stable zone */
		.code		= KEY_VOLUMEDOWN,
		.adc_low	= 226,
		.adc_high	= 450,
	},
};

static int sec_jack_get_adc_value(void)
{
	return ab8500_gpadc_convert(ab8500_gpadc_get(), 5);

}

static void sec_jack_mach_init(struct platform_device *pdev)
{
	int ret = 0;
	/* initialise threshold for ACCDETECT1 comparator
	 * and the debounce for all ACCDETECT comparators */
	ret = abx500_set_register_interruptible(&pdev->dev, AB8500_ECI_AV_ACC,
						0x80, 0x31);
	if (ret < 0)
		pr_err("%s: ab8500 write failed\n", __func__);

	/* initialise threshold for ACCDETECT2 comparator1 and comparator2 */
	ret = abx500_set_register_interruptible(&pdev->dev, AB8500_ECI_AV_ACC,
						0x81, 0xB4);
	if (ret < 0)
		pr_err("%s: ab8500 write failed\n", __func__);

	ret = abx500_set_register_interruptible(&pdev->dev, AB8500_ECI_AV_ACC,
						0x82, 0x33);

	if (ret < 0)
		pr_err("%s: ab8500 write failed\n", __func__);

	/* set output polarity to Gnd when VAMIC1 is disabled */
	ret = abx500_set_register_interruptible(&pdev->dev, AB8500_REGU_CTRL1,
						0x84, 0x1);
	if (ret < 0)
		pr_err("%s: ab8500 write failed\n", __func__);
}

int sec_jack_get_det_level(struct platform_device *pdev)
{
	u8 value = 0;
	int ret = 0;

	ret = abx500_get_register_interruptible(&pdev->dev, AB8500_INTERRUPT, 0x4,
		&value);
	if (ret < 0)
		return ret;

	ret = (value & 0x04) >> 2;
	pr_info("%s: ret=%x\n", __func__, ret);

	return ret;
}

struct sec_jack_platform_data sec_jack_pdata = {
	.get_adc_value = sec_jack_get_adc_value,
	.mach_init = sec_jack_mach_init,
	.get_det_level = sec_jack_get_det_level,
	.zones = sec_jack_zones,
	.num_zones = ARRAY_SIZE(sec_jack_zones),
	.buttons_zones = sec_jack_buttons_zones,
	.num_buttons_zones = ARRAY_SIZE(sec_jack_buttons_zones),
	.det_r = "ACC_DETECT_1DB_R",
	.det_f = "ACC_DETECT_1DB_F",
	.buttons_r = "ACC_DETECT_21DB_R",
	.buttons_f = "ACC_DETECT_21DB_F",
	.regulator_mic_source = "v-amic1"
};
#endif

static struct ab8500_led_pwm leds_pwm_data[] = {

};


struct ab8500_pwmled_platform_data skomer_pwmled_plat_data = {
	.num_pwm = 0,
	.leds = leds_pwm_data,
};

#ifdef CONFIG_MODEM_U8500
static struct platform_device u8500_modem_dev = {
	.name = "u8500-modem",
	.id   = 0,
	.dev  = {
		.platform_data = NULL,
	},
};
#endif


static struct dbx500_cpuidle_platform_data db8500_cpuidle_platform_data = {
	.wakeups = PRCMU_WAKEUP(ARM) | PRCMU_WAKEUP(RTC) | PRCMU_WAKEUP(ABB),
};

struct platform_device db8500_cpuidle_device = {
	.name	= "dbx500-cpuidle",
	.id	= -1,
	.dev	= {
		.platform_data = &db8500_cpuidle_platform_data,
	},
};

static struct dbx500_cpuidle_platform_data db9500_cpuidle_platform_data = {
	.wakeups = PRCMU_WAKEUP(ARM) | PRCMU_WAKEUP(RTC) | PRCMU_WAKEUP(ABB) \
		   | PRCMU_WAKEUP(HSI0),
};

struct platform_device db9500_cpuidle_device = {
	.name	= "dbx500-cpuidle",
	.id	= -1,
	.dev	= {
		.platform_data = &db9500_cpuidle_platform_data,
	},
};

static struct ab8500_platform_data ab8500_platdata = {
	.irq_base	= MOP500_AB8500_IRQ_BASE,
	.regulator	= &skomer_ab8500_regulator_plat_data,
#ifdef CONFIG_BATTERY_SAMSUNG
	.sec_bat	= &sec_battery_pdata,
#else
	.battery	= &ab8500_bm_data,
	.charger	= &ab8500_charger_plat_data,
	.btemp		= &ab8500_btemp_plat_data,
	.fg		= &ab8500_fg_plat_data,
	.chargalg	= &ab8500_chargalg_plat_data,
#endif
	.gpio		= &ab8500_gpio_pdata,
	.sysctrl	= &ab8500_sysctrl_pdata,
//	.pwmled		= &skomer_pwmled_plat_data,
#ifdef CONFIG_INPUT_AB8500_ACCDET
	.accdet = &ab8500_accdet_pdata,
#endif
#ifdef CONFIG_SAMSUNG_JACK
       .accdet = &sec_jack_pdata,
#endif
#ifdef CONFIG_PM
	.pm_power_off = true,
#endif
	.thermal_time_out = 20, /* seconds */
#ifdef CONFIG_INPUT_AB8505_MICRO_USB_DETECT
	.iddet = &iddet_adc_val_list,
#endif
};

static struct resource ab8500_resources[] = {
	[0] = {
		.start = IRQ_DB8500_AB8500,
		.end = IRQ_DB8500_AB8500,
		.flags = IORESOURCE_IRQ
	}
};

static struct platform_device ab8500_device = {
	.name = "ab8500-i2c",
	.id = 0,
	.dev = {
		.platform_data = &ab8500_platdata,
	},
	.num_resources = 1,
	.resource = ab8500_resources,
};

static struct ab8500_platform_data ab8505_platdata = {
	.irq_base	= MOP500_AB8500_IRQ_BASE,
	.regulator	= &skomer_ab8505_regulator_plat_data,
#ifdef CONFIG_BATTERY_SAMSUNG
	.sec_bat = &sec_battery_pdata,
#else
	.battery	= &ab8500_bm_data,
	.charger	= &ab8500_charger_plat_data,
	.btemp		= &ab8500_btemp_plat_data,
	.fg		= &ab8500_fg_plat_data,
	.chargalg	= &ab8500_chargalg_plat_data,
#endif
	.gpio		= &ab8505_gpio_pdata,
	.sysctrl	= &ab8500_sysctrl_pdata,
//	.pwmled		= &skomer_pwmled_plat_data,
#ifdef CONFIG_INPUT_AB8500_ACCDET
	.accdet = &ab8500_accdet_pdata,
#endif
#ifdef CONFIG_SAMSUNG_JACK
       .accdet = &sec_jack_pdata,
#endif
#ifdef CONFIG_PM
	.pm_power_off = true,
#endif
	.thermal_time_out = 20, /* seconds */
#ifdef CONFIG_INPUT_AB8505_MICRO_USB_DETECT
	.iddet = &iddet_adc_val_list,
#endif

};

struct platform_device ab8505_device = {
	.name = "ab8505-i2c",
	.id = 0,
	.dev = {
		.platform_data = &ab8505_platdata,
	},
	.num_resources = 1,
	.resource = ab8500_resources,
};

#ifdef CONFIG_BT_BCM4334
static struct platform_device bcm4334_bluetooth_device = {
	.name = "bcm4334_bluetooth",
	.id = -1,
};
#else
#if (defined CONFIG_RFKILL && !defined CONFIG_CG2900)
static struct platform_device sec_device_rfkill = {
	.name = "bt_rfkill",
	.id = -1,
};
#endif
#endif

static pin_cfg_t mop500_pins_uart0[] = {
	GPIO0_U0_CTSn   | PIN_INPUT_PULLUP,
	GPIO1_U0_RTSn   | PIN_OUTPUT_HIGH,
	GPIO2_U0_RXD    | PIN_INPUT_PULLUP,
	GPIO3_U0_TXD    | PIN_OUTPUT_HIGH,
};

static void ux500_uart0_init(void)
{
	int ret;
    
	ret = nmk_config_pins(mop500_pins_uart0,
			ARRAY_SIZE(mop500_pins_uart0));
	if (ret < 0)
		pr_err("pl011: uart pins_enable failed\n");
}

static void ux500_uart0_exit(void)
{
	int ret;

	ret = nmk_config_pins_sleep(mop500_pins_uart0,
			ARRAY_SIZE(mop500_pins_uart0));
	if (ret < 0)
		pr_err("pl011: uart pins_disable failed\n");
}

static void u8500_uart0_reset(void)
{
	/* UART0 lies in PER1 */
	return u8500_reset_ip(1, PRCC_K_SOFTRST_UART0_MASK);
}

static void u8500_uart1_reset(void)
{
	/* UART1 lies in PER1 */
	return u8500_reset_ip(1, PRCC_K_SOFTRST_UART1_MASK);
}

static void u8500_uart2_reset(void)
{
	/* UART2 lies in PER3 */
	return u8500_reset_ip(3, PRCC_K_SOFTRST_UART2_MASK);
}

static void bt_wake_peer(struct uart_port *port)
{
	printk("@@@@ BT WAKE_PEER\n");
	return;
}

static struct amba_pl011_data uart0_plat = {
#ifdef CONFIG_STE_DMA40_REMOVE
	.dma_filter = stedma40_filter,
	.dma_rx_param = &uart0_dma_cfg_rx,
	.dma_tx_param = &uart0_dma_cfg_tx,
#endif
	.init = ux500_uart0_init,
	.exit = ux500_uart0_exit,
    .reset = u8500_uart0_reset,
#ifdef CONFIG_BT_BCM4334
	.amba_pl011_wake_peer = bcm_bt_lpm_exit_lpm_locked,
#else
	.amba_pl011_wake_peer = NULL,
#endif
};

static struct amba_pl011_data uart1_plat = {
#ifdef CONFIG_STE_DMA40_REMOVE
	.dma_filter = stedma40_filter,
	.dma_rx_param = &uart1_dma_cfg_rx,
	.dma_tx_param = &uart1_dma_cfg_tx,
#endif
	.reset = u8500_uart1_reset,
	.amba_pl011_wake_peer = NULL,
};

static struct amba_pl011_data uart2_plat = {
#ifdef CONFIG_STE_DMA40_REMOVE
	.dma_filter = stedma40_filter,
	.dma_rx_param = &uart2_dma_cfg_rx,
	.dma_tx_param = &uart2_dma_cfg_tx,
#endif
	.reset = u8500_uart2_reset,
	.amba_pl011_wake_peer = NULL,
};


#if defined(CONFIG_BACKLIGHT_KTD253)
/* The following table is used to convert brightness level to the LED
    Current Ratio expressed as (full current) /(n * 32).
    i.e. 1 = 1/32 full current. Zero indicates LED is powered off.
    The table is intended to allow the brightness level to be "tuned"
    to compensate for non-linearity of brightness relative to current.
*/
static const unsigned short ktd253CurrentRatioLookupTable[] = {
0,		/* (0/32)		KTD253_BACKLIGHT_OFF */
39,   /* (1/32)   KTD253_MIN_CURRENT_RATIO */
58,   /* (2/32) */
67,   /* (3/32) */
76,   /* (4/32) */
85,   /* (5/32) */
94,   /* (6/32) */
104,    /* (7/32) */
113,    /* (8/32) */
122,    /* (9/32) */
131,    /* (10/32) */
150,    /* (11/32) */
168,    /* (12/32) */
183,    /* (13/32) */
194,    /* (14/32) */
204,    /* (15/32) */
209,    /* (16/32) */
214,    /* (17/32) */
219,    /* (18/32) */
224,    /* (19/32) */
229,    /* (20/32) */
255,    /* (21/32) */
300,    /* (22/32) */
300,    /* (23/32) */
300,    /* (24/32) */
300,    /* (25/32) */
300,    /* (26/32) */
300,    /* (27/32) */
300,    /* (28/32) */
300,    /* (29/32) */
300,    /* (30/32) */
300,    /* (31/32) */
300     /* (32/32)    KTD253_MAX_CURRENT_RATIO */
};

static struct ktd253x_bl_platform_data hendrix_bl_platform_info = {
	.bl_name			= "pwm-backlight",
	.ctrl_gpio			= LCD_BL_CTRL_HENDRIX_BRINGUP,
	.ctrl_high			= 1,
	.ctrl_low			= 0,
	.max_brightness			= 255,
	.brightness_to_current_ratio	= ktd253CurrentRatioLookupTable,

	/* Control backlight on/off via callback function to synchronise with display on/off */
	.external_bl_control		= true,
};

static struct platform_device hendrix_backlight_device = {
	.name = BL_DRIVER_NAME_KTD253,
	.id = -1,
	.dev = {
		.platform_data = &hendrix_bl_platform_info,
	},
};
#endif




#ifdef CONFIG_ANDROID_TIMED_GPIO
static struct timed_gpio skomer_timed_gpios[] = {
	{
		.name		= "vibrator",
		.gpio		= MOT_EN_SKOMER_BRINGUP,
		.max_timeout	= 10000,
		.active_low	= 0,
	},
};

static struct timed_gpio_platform_data skomer_timed_gpio_pdata = {
	.num_gpios	= ARRAY_SIZE(skomer_timed_gpios),
	.gpios		= skomer_timed_gpios,
};

static struct platform_device skomer_timed_gpios_device = {
	.name = TIMED_GPIO_NAME,
	.id = -1,
	.dev = {
		.platform_data = &skomer_timed_gpio_pdata,
	},
};
#endif

static struct cryp_platform_data u8500_cryp1_platform_data = {
	.mem_to_engine = {
		.dir = STEDMA40_MEM_TO_PERIPH,
		.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
		.dst_dev_type = DB8500_DMA_DEV48_CAC1_TX,
		.src_info.data_width = STEDMA40_WORD_WIDTH,
		.dst_info.data_width = STEDMA40_WORD_WIDTH,
		.mode = STEDMA40_MODE_LOGICAL,
		.src_info.psize = STEDMA40_PSIZE_LOG_4,
		.dst_info.psize = STEDMA40_PSIZE_LOG_4,
	},
	.engine_to_mem = {
		.dir = STEDMA40_PERIPH_TO_MEM,
		.src_dev_type = DB8500_DMA_DEV48_CAC1_RX,
		.dst_dev_type = STEDMA40_DEV_DST_MEMORY,
		.src_info.data_width = STEDMA40_WORD_WIDTH,
		.dst_info.data_width = STEDMA40_WORD_WIDTH,
		.mode = STEDMA40_MODE_LOGICAL,
		.src_info.psize = STEDMA40_PSIZE_LOG_4,
		.dst_info.psize = STEDMA40_PSIZE_LOG_4,
	}
};

static struct stedma40_chan_cfg u8500_hash_dma_cfg_tx = {
	.dir = STEDMA40_MEM_TO_PERIPH,
	.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
	.dst_dev_type = DB8500_DMA_DEV50_HAC1_TX,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
	.mode = STEDMA40_MODE_LOGICAL,
	.src_info.psize = STEDMA40_PSIZE_LOG_16,
	.dst_info.psize = STEDMA40_PSIZE_LOG_16,
};

static struct hash_platform_data u8500_hash1_platform_data = {
	.mem_to_engine = &u8500_hash_dma_cfg_tx,
	.dma_filter = stedma40_filter,
};


static struct platform_device *platform_devs[] __initdata = {
	&u8500_shrm_device,
#ifdef SSG_CAMERA_ENABLE
	&ux500_mmio_device,
#endif
	&ux500_hwmem_device,
#ifdef CONFIG_SPI_GPIO
	&ux500_spi_gpio_device,
#endif
	&ux500_mcde_device,
#ifdef CONFIG_MCDE_DISPLAY_DSI
	&u8500_dsilink_device[0],
	&u8500_dsilink_device[1],
	&u8500_dsilink_device[2],
#endif
	&ux500_b2r2_device,
	&ux500_b2r2_blt_device,
#ifdef CONFIG_STE_TRACE_MODEM
	&u8500_trace_modem,
#endif
	&db8500_mali_gpu_device,
#ifdef CONFIG_MODEM_U8500
	&u8500_modem_dev,
#endif
	&db8500_cpuidle_device,
#ifdef CONFIG_USB_ANDROID
	&android_usb_device,
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	&usb_mass_storage_device,
#endif
#ifdef CONFIG_USB_ANDROID_ECM
	&usb_ecm_device,
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	&usb_rndis_device,
#endif
#ifdef CONFIG_USB_ANDROID_PHONET
	&usb_phonet_device,
#endif
#endif
#ifdef CONFIG_DB8500_MLOADER
	&mloader_fw_device,
#endif
#ifdef CONFIG_BT_BCM4334
	&bcm4334_bluetooth_device,
#else
#if (defined CONFIG_RFKILL && !defined CONFIG_CG2900)
	&sec_device_rfkill,
#endif
#endif
#if defined(CONFIG_BACKLIGHT_KTD253)
	&hendrix_backlight_device,
#endif
#ifdef CONFIG_ANDROID_TIMED_GPIO
	&skomer_timed_gpios_device,
#endif
#if defined(CONFIG_SENSORS_ALPS)
	&alps_pdata,
#endif
#if defined(CONFIG_SENSORS_PX3215)
	&px3215_plat_data,
#endif
};

/* Callback function from display driver used to control backlight on/off */
void hendrix_backlight_on_off(bool on)
{
	if (hendrix_bl_platform_info.external_bl_control && hendrix_bl_platform_info.bl_on_off)
		hendrix_bl_platform_info.bl_on_off(hendrix_bl_platform_info.bd, on);
}

#ifdef CONFIG_INPUT_MPU6050
static void skomer_mpu_init(void)
{
	int intrpt_gpio = SENSORS_INT_SKOMER_BRINGUP;

	gpio_request(intrpt_gpio,"mpu6050_input");
	gpio_direction_input(intrpt_gpio);
}
#endif

#if 0
static void skomer_px3215_init(void)
{
	int intrpt_gpio = PS_INT_SKOMER_BRINGUP;

	gpio_request(intrpt_gpio,"px3215");
	gpio_direction_input(intrpt_gpio);
}
#endif

static void __init skomer_i2c_init(void)
{
//	db8500_add_i2c0(&skomer_i2c0_data);
	db8500_add_i2c1(&skomer_i2c1_data);
	db8500_add_i2c2(&skomer_i2c2_data);
	db8500_add_i2c3(&skomer_i2c3_data);

//	i2c_register_board_info(0,
//		ARRAY_AND_SIZE(skomer_bringup_i2c0_devices));

	i2c_register_board_info(1,
		ARRAY_AND_SIZE(skomer_bringup_i2c1_devices));

	i2c_register_board_info(2,
		ARRAY_AND_SIZE(skomer_bringup_i2c2_devices));

	i2c_register_board_info(3,
		ARRAY_AND_SIZE(hendrix_bringup_i2c3_devices));

	platform_device_register(&skomer_gpio_i2c4_pdata);
	i2c_register_board_info(4,
		ARRAY_AND_SIZE(skomer_bringup_gpio_i2c4_devices));

	platform_device_register(&skomer_gpio_i2c5_pdata);
	i2c_register_board_info(5,
		ARRAY_AND_SIZE(skomer_bringup_gpio_i2c5_devices));

	platform_device_register(&skomer_gpio_i2c6_pdata);
	i2c_register_board_info(6,
	ARRAY_AND_SIZE(skomer_bringup_gpio_i2c6_devices));

	platform_device_register(&skomer_gpio_i2c7_pdata);
	i2c_register_board_info(7,
		ARRAY_AND_SIZE(skomer_bringup_gpio_i2c7_devices));

	platform_device_register(&skomer_gpio_i2c8_pdata);
	i2c_register_board_info(8,
		ARRAY_AND_SIZE(skomer_bringup_gpio_i2c8_devices));

	platform_device_register(&skomer_gpio_i2c9_pdata);
	i2c_register_board_info(9,
		ARRAY_AND_SIZE(skomer_bringup_gpio_i2c9_devices));
}

#ifdef CONFIG_USB_ANDROID
/*
 * Public Id is a Unique number for each board and is stored
 * in Backup RAM at address 0x80151FC0, ..FC4, FC8, FCC and FD0.
 *
 * This function reads the Public Ids from this address and returns
 * a single string, which can be used as serial number for USB.
 * Input parameter - serial_number should be of 'len' bytes long
*/
static void fetch_usb_serial_no(int len)
{
	u32 buf[5];
	void __iomem *backup_ram = NULL;

	backup_ram = ioremap(PUBLIC_ID_BACKUPRAM1, 0x14);

	if (backup_ram) {
		buf[0] = readl(backup_ram);
		buf[1] = readl(backup_ram + 4);
		buf[2] = readl(backup_ram + 8);
		buf[3] = readl(backup_ram + 0x0c);
		buf[4] = readl(backup_ram + 0x10);

		snprintf(android_usb_pdata.serial_number, len+1, "%X%X%X%X%X",
					buf[0], buf[1], buf[2], buf[3], buf[4]);
		iounmap(backup_ram);
	} else {
		printk(KERN_ERR "$$ ioremap failed\n");
	}
}
#endif


static void __init skomer_spi_init(void)
{
	db8500_add_spi0(&skomer_spi0_data);
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
}

static void __init skomer_uart_init(void)
{
	db8500_add_uart0(&uart0_plat);
	db8500_add_uart1(&uart1_plat);
	db8500_add_uart2(&uart2_plat);
}

static void __init u8500_cryp1_hash1_init(void)
{
	db8500_add_cryp1(&u8500_cryp1_platform_data);
	db8500_add_hash1(&u8500_hash1_platform_data);
}


static void __init hendrix_init_machine(void)
{
	sec_common_init();

	sec_common_init_early();

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	if (ram_console_device.num_resources == 1)
		platform_device_register(&ram_console_device);
#endif

	platform_device_register(&db8500_prcmu_device);
	platform_device_register(&u8500_usecase_gov_device);

	u8500_init_devices();

#ifdef CONFIG_USB_ANDROID
	fetch_usb_serial_no(USB_SERIAL_NUMBER_LEN);
#endif

	platform_add_devices(platform_devs, ARRAY_SIZE(platform_devs));

	ssg_pins_init();

	u8500_cryp1_hash1_init();
	skomer_i2c_init();
	skomer_spi_init();
	mop500_msp_init();		/* generic for now */
	skomer_uart_init();

#ifdef CONFIG_INPUT_MPU6050
	skomer_mpu_init();
#endif

#if 0
	skomer_px3215_init();
#endif

#ifdef CONFIG_STE_WLAN
	mop500_wlan_init();
#endif

#ifdef CONFIG_KEYBOARD_GPIO
	platform_device_register(&skomer_gpio_keys_device);
#endif

#ifdef CONFIG_BATTERY_SAMSUNG
	sec_init_battery();
#endif
	platform_device_register(&ab8505_device);

	sec_cam_init();

	sec_common_init_post() ;

	/* This board has full regulator constraints */
	regulator_has_full_constraints();

	mms134s_ts_init();
}

static int __init set_hats_state(char *str)
{
	if (get_option(&str, &hats_state) != 1)
		hats_state = 0;

	return 1;
}
__setup("hats_flag=", set_hats_state);

static int __init jig_smd_status(char *str)
{
	if (get_option(&str, &jig_smd) != 1)
		jig_smd = 0;

	return 1;

}
__setup("jig_smd=", jig_smd_status);

static int __init sec_debug_setup(char *str)
{
	if (get_option(&str, &sec_debug_settings) != 1)
		sec_debug_settings = 0;

	return 1;
}
__setup("debug=", sec_debug_setup);

/* we have equally similar boards with very minimal
 * changes, so we detect the platform during boot
 */
static int __init board_id_setup(char *str)
{
	if (get_option(&str, &board_id) != 1)
		board_id = 0;

	use_ab8505_iddet = 1; //(board_id >= SKOMER_AB8505_IDDET_VER) ? 1 : 0;

	switch (board_id) {
	case 0x0101:
		printk(KERN_INFO "SKOMER Board for Rev0.0\n");
		system_rev = SKOMER_R0_0;
		break;
	case 0x0102:
		printk(KERN_INFO "SKOMER Board for Rev0.1\n");
		system_rev = SKOMER_R0_1;
		break;
	case 0x0103:
		printk(KERN_INFO "SKOMER Board for Rev0.2\n");
		system_rev = SKOMER_R0_2;
		break;
	case 0x0104:
		printk(KERN_INFO "SKOMER Board for Rev0.3\n");
		system_rev = SKOMER_R0_3;
		break;
	default:
		printk(KERN_INFO "Unknown board_id=%c\n", *str);
		system_rev = SKOMER_R0_0;
		break;
	};

	return 1;
}

__setup("board_id=", board_id_setup);

MACHINE_START(SEC_HENDRIX, "SAMSUNG HENDRIX")
	/* Maintainer: SAMSUNG based on ST Ericsson */
	.boot_params	= 0x100,
	.map_io		= u8500_map_io,
	.init_irq	= ux500_init_irq,
	.timer		= &ux500_timer,
	.init_machine	= hendrix_init_machine,
	.restart	= ux500_restart,
MACHINE_END
