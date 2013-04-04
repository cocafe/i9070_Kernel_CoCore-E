/*
 * Copyright (C) Samsung 2012
 *
 * License Terms: GNU General Public License v2
 * Authors: Robert Teather <robert.teather@samsung.com> for Samsung Electronics
 *
 * Board specific file for regulator machine initialization
 *
 */

#include <linux/kernel.h>
#include <linux/regulator/machine.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <linux/amba/bus.h>
#include <linux/mpu.h>
#include <mach/bma023.h>
#if defined(CONFIG_PROXIMITY_GP2A)
#include <mach/gp2a.h>
#endif
#if defined(CONFIG_PROXIMITY_PX3315)
#include <mach/px3315.h>
#endif
#include "board-kyle-regulators.h"

/* ab8500 regulator register initialization used for AB8505 for Rev0.0 */
static struct ab8500_regulator_reg_init
	kyle_r0_0_reg_init[AB8505_NUM_REGULATOR_REGISTERS] = {
	/*
	 * VarmRequestCtrl
	 * VsmpsCRequestCtrl
	 * VsmpsARequestCtrl
	 * VsmpsBRequestCtrl
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUREQUESTCTRL1,	0x00, 0x00),
	/*
	 * VsafeRequestCtrl
	 * VpllRequestCtrl
	 * VanaRequestCtrl	    = HP/LP depending on VxRequest
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUREQUESTCTRL2,	0x30, 0x00),
	/*
	 * Vaux1RequestCtrl         = HP/LP depending on VxRequest
	 * Vaux2RequestCtrl         = HP/LP depending on VxRequest
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUREQUESTCTRL3,	0xf0, 0x00),
	/*
	 * Vaux3RequestCtrl         = HP/LP depending on VxRequest
	 * SwHPReq                  = Control through SWValid disabled
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUREQUESTCTRL4,	0x07, 0x00),
	/*
	 * VsmpsASysClkReq1HPValid
	 * VsmpsBSysClkReq1HPValid
	 * VsafeSysClkReq1HPValid
	 * VanaSysClkReq1HPValid    = disabled
	 * VpllSysClkReq1HPValid
	 * Vaux1SysClkReq1HPValid   = disabled
	 * Vaux2SysClkReq1HPValid   = disabled
	 * Vaux3SysClkReq1HPValid   = disabled
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUSYSCLKREQ1HPVALID1,	0xe8, 0x00),
	/*
	 * VsmpsCSysClkReq1HPValid
	 * VarmSysClkReq1HPValid
	 * VbbSysClkReq1HPValid
	 * VsmpsMSysClkReq1HPValid
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUSYSCLKREQ1HPVALID2, 0x00, 0x00),
	/*
	 * VsmpsAHwHPReq1Valid
	 * VsmpsBHwHPReq1Valid
	 * VsafeHwHPReq1Valid
	 * VanaHwHPReq1Valid	    = disabled
	 * VpllHwHPReq1Valid
	 * Vaux1HwHPreq1Valid	    = disabled
	 * Vaux2HwHPReq1Valid	    = disabled
	 * Vaux3HwHPReqValid	    = disabled
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUHWHPREQ1VALID1,	0xe8, 0x00),
	/*
	 * VsmpsMHwHPReq1Valid
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUHWHPREQ1VALID2,	0x00, 0x00),
	/*
	 * VsmpsAHwHPReq2Valid
	 * VsmpsBHwHPReq2Valid
	 * VsafeHwHPReq2Valid
	 * VanaHwHPReq2Valid	    = disabled
	 * VpllHwHPReq2Valid
	 * Vaux1HwHPReq2Valid	    = disabled
	 * Vaux2HwHPReq2Valid	    = disabled
	 * Vaux3HwHPReq2Valid	    = disabled
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUHWHPREQ2VALID1,	0xe8, 0x00),
	/*
	 * VsmpsMHwHPReq2Valid
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUHWHPREQ2VALID2,	0x00, 0x00),
	/**
	 * VsmpsCSwHPReqValid
	 * VarmSwHPReqValid
	 * VsmpsASwHPReqValid
	 * VsmpsBSwHPReqValid
	 * VsafeSwHPReqValid
	 * VanaSwHPReqValid
	 * VanaSwHPReqValid	    = disabled
	 * VpllSwHPReqValid
	 * Vaux1SwHPReqValid	    = disabled
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUSWHPREQVALID1,	0xa0, 0x00),
	/*
	 * Vaux2SwHPReqValid        = disabled
	 * Vaux3SwHPReqValid        = disabled
	 * VsmpsMSwHPReqValid
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUSWHPREQVALID2,	0x03, 0x00),
	/*
	 * SysClkReq2Valid1         = disabled
	 * SysClkReq3Valid1         = disabled
	 * SysClkReq4Valid1         = disabled
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUSYSCLKREQVALID1,	0x0e, 0x00),
	/*
	 * SysClkReq2Valid2         = disabled
	 * SysClkReq3Valid2         = disabled
	 * SysClkReq4Valid2         = disabled
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUSYSCLKREQVALID2,	0x0e, 0x00),
	/*
	 * Vaux4SwHPReqValid
	 * Vaux4HwHPReq2Valid
	 * Vaux4HwHPReq1Valid
	 * Vaux4SysClkReq1HPValid
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUVAUX4REQVALID,	0x00, 0x00),
	/*
	 * VTVoutEna                = disabled
	 * Vintcore12Ena            = disabled
	 * Vintcore12Sel            = 1.25 V
	 * Vintcore12LP             = inactive (HP)
	 * VTVoutLP                 = inactive (HP)
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUMISC1,		0xfe, 0x10),
	/*
	 * VaudioEna                = disabled
	 * Vaux8Ena                 = disabled
	 * Vamic1Ena                = disabled
	 * Vamic2Ena                = disabled
	 */
	INIT_REGULATOR_REGISTER(AB8505_VAUDIOSUPPLY,		0x1e, 0x00),
	/*
	 * Vamic1_dzout             = high-Z when Vamic1 is disabled
	 * Vamic2_dzout             = high-Z when Vamic2 is disabled
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUCTRL1VAMIC,		0x03, 0x00),
	/*
	 * VsmpsARegu
	 * VsmpsASelCtrl
	 * VsmpsAAutoMode
	 * VsmpsAPWMMode
	 */
	INIT_REGULATOR_REGISTER(AB8505_VSMPSAREGU,		0x00, 0x00),
	/*
	 * VsmpsBRegu
	 * VsmpsBSelCtrl
	 * VsmpsBAutoMode
	 * VsmpsBPWMMode
	 */
	INIT_REGULATOR_REGISTER(AB8505_VSMPSBREGU,		0x00, 0x00),
	/*
	 * VsafeRegu
	 * VsafeSelCtrl
	 * VsafeAutoMode
	 * VsafePWMMode
	 */
	INIT_REGULATOR_REGISTER(AB8505_VSAFEREGU,		0x00, 0x00),
	/*
	 * VPll			    = Hw controlled (NOTE! PRCMU bits)
	 * VanaRegu		    = force on for DSI
	 */
	INIT_REGULATOR_REGISTER(AB8505_VPLLVANAREGU,		0x0f, 0x06),
	/*
	 * No external regulators connected to AB8505
	 */
	INIT_REGULATOR_REGISTER(AB8505_EXTSUPPLYREGU,          0xff, 0x00),
	/*
	 * Vaux1Regu		    = force HP
	 * Vaux2Regu		    = force HP
	 */
	INIT_REGULATOR_REGISTER(AB8505_VAUX12REGU,		0x0f, 0x05),
	/*
	 * Vaux3Regu		    = force off
	 */
	INIT_REGULATOR_REGISTER(AB8505_VRF1VAUX3REGU,		0x03, 0x00),
	/*
	 * VsmpsASel1
	 */
	INIT_REGULATOR_REGISTER(AB8505_VSMPSASEL1,		0x00, 0x00),
	/*
	 * VsmpsASel2
	 */
	INIT_REGULATOR_REGISTER(AB8505_VSMPSASEL2,		0x00, 0x00),
	/*
	 * VsmpsASel3
	 */
	INIT_REGULATOR_REGISTER(AB8505_VSMPSASEL3,		0x00, 0x00),
	/*
	 * VsmpsBSel1
	 */
	INIT_REGULATOR_REGISTER(AB8505_VSMPSBSEL1,		0x00, 0x00),
	/*
	 * VsmpsBSel2
	 */
	INIT_REGULATOR_REGISTER(AB8505_VSMPSBSEL2,		0x00, 0x00),
	/*
	 * VsmpsBSel3
	 */
	INIT_REGULATOR_REGISTER(AB8505_VSMPSBSEL3,		0x00, 0x00),
	/*
	 * VsafeSel1
	 */
	INIT_REGULATOR_REGISTER(AB8505_VSAFESEL1,		0x00, 0x00),
	/*
	 * VsafeSel2
	 */
	INIT_REGULATOR_REGISTER(AB8505_VSAFESEL2,		0x00, 0x00),
	/*
	 * VsafeSel3
	 */
	INIT_REGULATOR_REGISTER(AB8505_VSAFESEL3,		0x00, 0x00),
	/*
	 * Vaux1Sel                 = 3.0 V
	 */
	INIT_REGULATOR_REGISTER(AB8505_VAUX1SEL,		0x0f, 0x0e),
	/*
	 * Vaux2Sel                 = 3.0 V
	 */
	INIT_REGULATOR_REGISTER(AB8505_VAUX2SEL,		0x0f, 0x0e),
	/*
	 * Vaux3Sel                 = 2.91 V
	 */
	INIT_REGULATOR_REGISTER(AB8505_VRF1VAUX3SEL,		0x07, 0x07),
	/*
	 * Vaux4RequestCtrl
	 */
	INIT_REGULATOR_REGISTER(AB8505_VAUX4REQCTRL,		0x03, 0x00),
	/*
	 * Vaux4Regu - off
	 */
	INIT_REGULATOR_REGISTER(AB8505_VAUX4REGU,		0x03, 0x00),
	/*
	 * Vaux4Sel	= 3.3V
	 */
	INIT_REGULATOR_REGISTER(AB8505_VAUX4SEL,		0x0f, 0x0f),
	/*
	 * Vaux1Disch               = short discharge time
	 * Vaux2Disch               = short discharge time
	 * Vaux3Disch               = short discharge time
	 * Vintcore12Disch          = short discharge time
	 * VTVoutDisch              = short discharge time
	 * VaudioDisch              = short discharge time
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUCTRLDISCH,          0xfc, 0x00),
	/*
	 * VanaDisch                = short discharge time
	 * VdmicPullDownEna         = pulldown disabled when Vdmic is disabled
	 * VdmicDisch               = short discharge time
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUCTRLDISCH2,         0x16, 0x00),
	/*
	 * Vaux4Disch		    = short discharge time
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUCTRLDISCH3,	       0x01, 0x00),
	/*
	 * Vaux5Sel - 1.8V
	 * Vaux5LP - HP
	 * Vaux5Ena - enabled
	 * Vaux5Disch - short
	 * Vaux5DisSfst - enabled
	 * Vaux5DisPulld - enabled
	 */
	INIT_REGULATOR_REGISTER(AB8505_CTRLVAUX5,	       0xff, 0x10),
	/*
	 * Vaux6Sel - 1.8V
	 * Vaux6LP - HP
	 * Vaux6Ena - enabled
	 * Vaux6DisPulld - enabled
	 */
	INIT_REGULATOR_REGISTER(AB8505_CTRLVAUX6,	       0x9f, 0x10),
};


/* ab8505 regulator register initialization used for Rev0.1 */
static struct ab8500_regulator_reg_init
	kyle_r0_1_reg_init[AB8505_NUM_REGULATOR_REGISTERS] = {
	/*
	 * VarmRequestCtrl
	 * VsmpsCRequestCtrl
	 * VsmpsARequestCtrl
	 * VsmpsBRequestCtrl
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUREQUESTCTRL1,	0x00, 0x00),
	/*
	 * VsafeRequestCtrl
	 * VpllRequestCtrl
	 * VanaRequestCtrl	    = HP/LP depending on VxRequest
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUREQUESTCTRL2,	0x30, 0x00),
	/*
	 * Vaux1RequestCtrl         = HP/LP depending on VxRequest
	 * Vaux2RequestCtrl         = HP/LP depending on VxRequest
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUREQUESTCTRL3,       0xf0, 0x00),
	/*
	 * Vaux3RequestCtrl         = HP/LP depending on VxRequest
	 * SwHPReq                  = Control through SWValid disabled
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUREQUESTCTRL4,       0x07, 0x00),
	/*
	 * VsmpsASysClkReq1HPValid
	 * VsmpsBSysClkReq1HPValid
	 * VsafeSysClkReq1HPValid
	 * VanaSysClkReq1HPValid    = disabled
	 * VpllSysClkReq1HPValid
	 * Vaux1SysClkReq1HPValid   = disabled
	 * Vaux2SysClkReq1HPValid   = disabled
	 * Vaux3SysClkReq1HPValid   = disabled
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUSYSCLKREQ1HPVALID1,	0xe8, 0x00),
	/*
	 * VsmpsCSysClkReq1HPValid
	 * VarmSysClkReq1HPValid
	 * VbbSysClkReq1HPValid
	 * VsmpsMSysClkReq1HPValid
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUSYSCLKREQ1HPVALID2, 0x00, 0x00),
	/*
	 * VsmpsAHwHPReq1Valid
	 * VsmpsBHwHPReq1Valid
	 * VsafeHwHPReq1Valid
	 * VanaHwHPReq1Valid	    = disabled
	 * VpllHwHPReq1Valid
	 * Vaux1HwHPreq1Valid	    = disabled
	 * Vaux2HwHPReq1Valid	    = disabled
	 * Vaux3HwHPReqValid	    = disabled
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUHWHPREQ1VALID1,	0xe8, 0x00),
	/*
	 * VsmpsMHwHPReq1Valid
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUHWHPREQ1VALID2,	0x00, 0x00),
	/*
	 * VsmpsAHwHPReq2Valid
	 * VsmpsBHwHPReq2Valid
	 * VsafeHwHPReq2Valid
	 * VanaHwHPReq2Valid	    = disabled
	 * VpllHwHPReq2Valid
	 * Vaux1HwHPReq2Valid	    = disabled
	 * Vaux2HwHPReq2Valid	    = disabled
	 * Vaux3HwHPReq2Valid	    = disabled
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUHWHPREQ2VALID1,	0xe8, 0x00),
	/*
	 * VsmpsMHwHPReq2Valid
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUHWHPREQ2VALID2,	0x00, 0x00),
	/**
	 * VsmpsCSwHPReqValid
	 * VarmSwHPReqValid
	 * VsmpsASwHPReqValid
	 * VsmpsBSwHPReqValid
	 * VsafeSwHPReqValid
	 * VanaSwHPReqValid
	 * VanaSwHPReqValid	    = disabled
	 * VpllSwHPReqValid
	 * Vaux1SwHPReqValid	    = disabled
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUSWHPREQVALID1,	0xa0, 0x00),
	/*
	 * Vaux2SwHPReqValid        = disabled
	 * Vaux3SwHPReqValid        = disabled
	 * VsmpsMSwHPReqValid
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUSWHPREQVALID2,	0x03, 0x00),
	/*
	 * SysClkReq2Valid1         = disabled
	 * SysClkReq3Valid1         = disabled
	 * SysClkReq4Valid1         = disabled
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUSYSCLKREQVALID1,	0x0e, 0x00),
	/*
	 * SysClkReq2Valid2         = disabled
	 * SysClkReq3Valid2         = disabled
	 * SysClkReq4Valid2         = disabled
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUSYSCLKREQVALID2,	0x0e, 0x00),
	/*
	 * Vaux4SwHPReqValid
	 * Vaux4HwHPReq2Valid
	 * Vaux4HwHPReq1Valid
	 * Vaux4SysClkReq1HPValid
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUVAUX4REQVALID,	0x00, 0x00),
	/*
	 * VadcEna                  = disabled
	 * VintCore12Ena            = disabled
	 * VintCore12Sel            = 1.25 V
	 * VintCore12LP             = inactive (HP)
	 * VadcLP                   = inactive (HP)
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUMISC1,		0xfe, 0x10),
	/*
	 * VaudioEna                = disabled
	 * Vaux8Ena                 = disabled
	 * Vamic1Ena                = disabled
	 * Vamic2Ena                = disabled
	 */
	INIT_REGULATOR_REGISTER(AB8505_VAUDIOSUPPLY,		0x1e, 0x00),
	/*
	 * Vamic1_dzout             = high-Z when Vamic1 is disabled
	 * Vamic2_dzout             = high-Z when Vamic2 is disabled
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUCTRL1VAMIC,		0x03, 0x00),
	/*
	 * VsmpsARegu
	 * VsmpsASelCtrl
	 * VsmpsAAutoMode
	 * VsmpsAPWMMode
	 */
	INIT_REGULATOR_REGISTER(AB8505_VSMPSAREGU,		0x00, 0x00),
	/*
	 * VsmpsBRegu
	 * VsmpsBSelCtrl
	 * VsmpsBAutoMode
	 * VsmpsBPWMMode
	 */
	INIT_REGULATOR_REGISTER(AB8505_VSMPSBREGU,		0x00, 0x00),
	/*
	 * VsafeRegu
	 * VsafeSelCtrl
	 * VsafeAutoMode
	 * VsafePWMMode
	 */
	INIT_REGULATOR_REGISTER(AB8505_VSAFEREGU,		0x00, 0x00),
	/*
	 * VPll			    = Hw controlled (NOTE! PRCMU bits)
	 * VanaRegu		    = force on for DSI
	 */
	INIT_REGULATOR_REGISTER(AB8505_VPLLVANAREGU,		0x0f, 0x06),
	/*
	 * No external regulators connected to AB8505
	 */
	INIT_REGULATOR_REGISTER(AB8505_EXTSUPPLYREGU,          0xff, 0x00),
	/*
	 * Vaux1Regu                = force off
	 * Vaux2Regu                = force off
	 */
	INIT_REGULATOR_REGISTER(AB8505_VAUX12REGU,             0x0f, 0x00),
	/*
	 * Vaux3Regu		    = force off
	 */
	INIT_REGULATOR_REGISTER(AB8505_VRF1VAUX3REGU,		0x03, 0x00),
	/*
	 * VsmpsASel1
	 */
	INIT_REGULATOR_REGISTER(AB8505_VSMPSASEL1,		0x00, 0x00),
	/*
	 * VsmpsASel2
	 */
	INIT_REGULATOR_REGISTER(AB8505_VSMPSASEL2,		0x00, 0x00),
	/*
	 * VsmpsASel3
	 */
	INIT_REGULATOR_REGISTER(AB8505_VSMPSASEL3,		0x00, 0x00),
	/*
	 * VsmpsBSel1
	 */
	INIT_REGULATOR_REGISTER(AB8505_VSMPSBSEL1,		0x00, 0x00),
	/*
	 * VsmpsBSel2
	 */
	INIT_REGULATOR_REGISTER(AB8505_VSMPSBSEL2,		0x00, 0x00),
	/*
	 * VsmpsBSel3
	 */
	INIT_REGULATOR_REGISTER(AB8505_VSMPSBSEL3,		0x00, 0x00),
	/*
	 * VsafeSel1
	 */
	INIT_REGULATOR_REGISTER(AB8505_VSAFESEL1,		0x00, 0x00),
	/*
	 * VsafeSel2
	 */
	INIT_REGULATOR_REGISTER(AB8505_VSAFESEL2,		0x00, 0x00),
	/*
	 * VsafeSel3
	 */
	INIT_REGULATOR_REGISTER(AB8505_VSAFESEL3,		0x00, 0x00),
	/*
	 * Vaux1Sel                 = 3.0 V
	 */
	INIT_REGULATOR_REGISTER(AB8505_VAUX1SEL,		0x0f, 0x0e),
	/*
	 * Vaux2Sel                 = 3.3 V
	 */
	INIT_REGULATOR_REGISTER(AB8505_VAUX2SEL,		0x0f, 0x0f),
	/*
	 * Vaux3Sel                 = 2.91 V
	 */
	INIT_REGULATOR_REGISTER(AB8505_VRF1VAUX3SEL,		0x07, 0x07),
	/*
	 * Vaux4RequestCtrl
	 */
	INIT_REGULATOR_REGISTER(AB8505_VAUX4REQCTRL,		0x03, 0x00),
	/*
	 * Vaux4Regu - off
	 */
	INIT_REGULATOR_REGISTER(AB8505_VAUX4REGU,		0x03, 0x00),
	/*
	 * Vaux4Sel	= 3.3V
	 */
	INIT_REGULATOR_REGISTER(AB8505_VAUX4SEL,		0x0f, 0x0f),
	/*
	 * Vaux1Disch               = short discharge time
	 * Vaux2Disch               = short discharge time
	 * Vaux3Disch               = short discharge time
	 * Vintcore12Disch          = short discharge time
	 * VTVoutDisch              = short discharge time
	 * VaudioDisch              = short discharge time
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUCTRLDISCH,          0xfc, 0x00),
	/*
	 * VanaDisch                = short discharge time
	 * VdmicPullDownEna         = pulldown disabled when Vdmic is disabled
	 * VdmicDisch               = short discharge time
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUCTRLDISCH2,         0x16, 0x04),
	/*
	 * Vaux4Disch		    = short discharge time
	 */
	INIT_REGULATOR_REGISTER(AB8505_REGUCTRLDISCH3,	       0x01, 0x00),
	/*
	 * Vaux5Sel - 1.8V
	 * Vaux5LP - HP
	 * Vaux5Ena - disabled
	 * Vaux5Disch - short
	 * Vaux5DisSfst - enabled
	 * Vaux5DisPulld - enabled
	 */
	INIT_REGULATOR_REGISTER(AB8505_CTRLVAUX5,	       0xff, 0x00),
	/*
	 * Vaux6Sel - 1.8V
	 * Vaux6LP - HP
	 * Vaux6Ena - disabled
	 * Vaux6DisPulld - enabled
	 */
	INIT_REGULATOR_REGISTER(AB8505_CTRLVAUX6,	       0x9f, 0x00),
};

static struct regulator_consumer_supply sensor_3v_consumers[] = {
	/* Proximity Sensor */
	REGULATOR_SUPPLY("v-prox-vdd", "0-0044"),	/* GP2A */
	REGULATOR_SUPPLY("vdd-proxi", "0-001e"),		/* PX3315 */
	/* Accelerometer Sensor */
	REGULATOR_SUPPLY("vdd-acc", "6-0018"),		/* BMA254 */
	REGULATOR_SUPPLY("v-accel-vdd", "bma022"),	/* BMA222 */
	/* Magnetic Sensor */
	REGULATOR_SUPPLY("vdd_hscdtd", "7-000c"),	/* HSCDTD */
};

static struct regulator_consumer_supply sensor_led_3v_consumers[] = {
	/* Proximity Sensor GP2A  */
	REGULATOR_SUPPLY("v-prox-led", GP2A_I2C_DEVICE_NAME),
};

static struct regulator_consumer_supply vdd_tf_2v91_consumers[] = {
	REGULATOR_SUPPLY("v-SD-STM", "stm"),
	/* External MMC slot power */
	REGULATOR_SUPPLY("vmmc", "sdi0"),
};

static struct regulator_consumer_supply vreg_tsp_a3v3_consumers[] = {
	REGULATOR_SUPPLY("v-tsp-3.3", "3-0048"),
};

/* SENSORS 1V8 */
static struct regulator_consumer_supply sensor_1v8_consumers[] = {
	/* Proximity Sensor  */
	REGULATOR_SUPPLY("v-prox-vio", "0-0044"),		/* GP2A */
	REGULATOR_SUPPLY("vio_proxi", "0-001e"),		/* PX3315 */
	/* Accelerometer Sensor */
	REGULATOR_SUPPLY("vio-acc", "6-0018"),		/* BMA254 */
	REGULATOR_SUPPLY("v-accel-vio", "bma022"),	/* BMA222 */
	/* Magnetic Sensor */
	REGULATOR_SUPPLY("vio_hscdtd", "7-000c"),		/* HSCDTD */
	/* Compass MMC3280 */
	REGULATOR_SUPPLY("v-compass", "mmc3280"),
};

static struct regulator_consumer_supply vreg_lcd_1v8_consumers[] = {
};

static struct regulator_consumer_supply vreg_tsp_1v8_consumers[] = {
	REGULATOR_SUPPLY("v-tsp-1.8", "3-0048"),
};

struct regulator_consumer_supply vdd_key_led_3v3_consumers[] = {
	REGULATOR_SUPPLY("v-keyled-3.3", "leds-regulator"),
};

static struct regulator_consumer_supply ab8505_vaux8_consumers[] = {
};

static struct regulator_consumer_supply ab8505_vadc_consumers[] = {
	/* Internal general-purpose ADC */
	REGULATOR_SUPPLY("vddadc", "ab8500-gpadc.0"),
	/* ADC for charger */
	REGULATOR_SUPPLY("vddadc", "ab8500-charger.0"),
	/* ADC for charger */
	REGULATOR_SUPPLY("vddadc", "ab8500-chargalg.0"),
};

static struct regulator_consumer_supply ab8505_vaudio_consumers[] = {
	/* AB8505 audio codec device */
	REGULATOR_SUPPLY("v-audio", NULL),
};

static struct regulator_consumer_supply ab8505_vamic1_consumers[] = {
	/* AB8505 audio codec device */
	REGULATOR_SUPPLY("v-amic1", NULL),
};

static struct regulator_consumer_supply ab8505_vamic2_consumers[] = {
	/* AB8505 audio codec device */
	REGULATOR_SUPPLY("v-amic2", NULL),
};

static struct regulator_consumer_supply ab8505_vintcore_consumers[] = {
	/* SoC core supply, no device */
	REGULATOR_SUPPLY("v-intcore", NULL),
	/* USB Transceiver */
	REGULATOR_SUPPLY("vddulpivio18", "ab8500-usb.0"),
};

static struct regulator_consumer_supply ab8505_vana_consumers[] = {
	/* DB8500 DSI */
	REGULATOR_SUPPLY("vdddsi1v2", "mcde"),
	REGULATOR_SUPPLY("vdddsi1v2", "dsilink.0"),
	REGULATOR_SUPPLY("vdddsi1v2", "dsilink.1"),
	REGULATOR_SUPPLY("vdddsi1v2", "dsilink.2"),
	/* DB8500 CSI */
	REGULATOR_SUPPLY("vddcsi1v2", "mmio_camera"),

	REGULATOR_SUPPLY("vdddsi1v2", "b2r2_core"),
	REGULATOR_SUPPLY("vdddsi1v2", "b2r2_1_core"),
};

static struct regulator_consumer_supply ab8505_sysclkreq_2_consumers[] = {
};

static struct regulator_consumer_supply ab8505_sysclkreq_4_consumers[] = {
};


/*
 * AB8505 regulators
 * Rev0.0.
 */
struct regulator_init_data kyle_r0_0_regulators[AB8505_NUM_REGULATORS] = {
	/* Consumers of VAUX1 */
	[AB8505_LDO_AUX1] = {
		.constraints = {
			.name = "V-AUX1",
			.min_uV = 3000000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,

		},
		.num_consumer_supplies = ARRAY_SIZE(sensor_3v_consumers),
		.consumer_supplies = sensor_3v_consumers,
	},
	/* Consumers of VAUX2 */
	[AB8505_LDO_AUX2] = {
		.constraints = {
			.name = "V-AUX2",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(sensor_led_3v_consumers),
		.consumer_supplies = sensor_led_3v_consumers,
	},
	/* Consumers of VAUX3 */
	[AB8505_LDO_AUX3] = {
		.constraints = {
			.name = "V-AUX3",
			.min_uV = 1100000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_STATUS |
					  REGULATOR_CHANGE_MODE,
			.valid_modes_mask = REGULATOR_MODE_NORMAL |
					    REGULATOR_MODE_IDLE,
		},
		.num_consumer_supplies = ARRAY_SIZE(vdd_tf_2v91_consumers),
		.consumer_supplies = vdd_tf_2v91_consumers,
	},
	/* Consumers of VAUX4 */
	[AB8505_LDO_AUX4] = {
		.constraints = {
			.name = "V-AUX4",
			.min_uV = 3300000,
			.max_uV = 3300000,
			.apply_uV = 1,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS |
					REGULATOR_CHANGE_MODE,
			.valid_modes_mask = REGULATOR_MODE_NORMAL |
					REGULATOR_MODE_IDLE,
		},
		.num_consumer_supplies = ARRAY_SIZE(vreg_tsp_a3v3_consumers),
		.consumer_supplies = vreg_tsp_a3v3_consumers,
	},
	/* Consumers of VAUX5 */
	[AB8505_LDO_AUX5] = {
		.constraints = {
			.name = "V-AUX5",
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(sensor_1v8_consumers),
		.consumer_supplies = sensor_1v8_consumers,
	},
	/* Consumers of VAUX6 */
	[AB8505_LDO_AUX6] = {
		.constraints = {
			.name = "V-AUX6",
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(vreg_lcd_1v8_consumers),
		.consumer_supplies = vreg_lcd_1v8_consumers,
	},
	/* supply for gpadc, ADC LDO */
	[AB8505_LDO_ADC] = {
		.constraints = {
			.name = "V-ADC",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8505_vadc_consumers),
		.consumer_supplies = ab8505_vadc_consumers,
	},
	/* supply for ab8500-vaudio, VAUDIO LDO */
	[AB8505_LDO_AUDIO] = {
		.constraints = {
			.name = "V-AUD",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8505_vaudio_consumers),
		.consumer_supplies = ab8505_vaudio_consumers,
	},
	/* supply for v-anamic1 VAMic1-LDO */
	[AB8505_LDO_ANAMIC1] = {
		.constraints = {
			.name = "V-AMIC1",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8505_vamic1_consumers),
		.consumer_supplies = ab8505_vamic1_consumers,
	},
	/* supply for v-amic2, VAMIC2 LDO, reuse constants for AMIC1 */
	[AB8505_LDO_ANAMIC2] = {
		.constraints = {
			.name = "V-AMIC2",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8505_vamic2_consumers),
		.consumer_supplies = ab8505_vamic2_consumers,
	},
	/* supply for v-aux8, VAUX8 LDO */
	[AB8505_LDO_AUX8] = {
		.constraints = {
			.name = "V-AUX8",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8505_vaux8_consumers),
		.consumer_supplies = ab8505_vaux8_consumers,
	},
	/* supply for v-intcore12, VINTCORE12 LDO */
	[AB8505_LDO_INTCORE] = {
		.constraints = {
			.name = "V-INTCORE",
			.min_uV = 1250000,
			.max_uV = 1350000,
			.input_uV = 1800000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_STATUS |
					  REGULATOR_CHANGE_MODE |
					  REGULATOR_CHANGE_DRMS,
			.valid_modes_mask = REGULATOR_MODE_NORMAL |
					    REGULATOR_MODE_IDLE,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8505_vintcore_consumers),
		.consumer_supplies = ab8505_vintcore_consumers,
	},
	/* supply for U8500 CSI-DSI, VANA LDO */
	[AB8505_LDO_ANA] = {
		.constraints = {
			.name = "V-CSI-DSI",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.boot_on = 1, // already on for splash screen
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8505_vana_consumers),
		.consumer_supplies = ab8505_vana_consumers,
	},
	/* sysclkreq 2 pin */
	[AB8505_SYSCLKREQ_2] = {
		.constraints = {
			.name = "V-SYSCLKREQ-2",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies =
			ARRAY_SIZE(ab8505_sysclkreq_2_consumers),
		.consumer_supplies = ab8505_sysclkreq_2_consumers,
	},
	/* sysclkreq 4 pin */
	[AB8505_SYSCLKREQ_4] = {
		.constraints = {
			.name = "V-SYSCLKREQ-4",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies =
			ARRAY_SIZE(ab8505_sysclkreq_4_consumers),
		.consumer_supplies = ab8505_sysclkreq_4_consumers,
	},
};

/*
 * Regulators for Rev0.1
 */
struct regulator_init_data kyle_r0_1_regulators[AB8505_NUM_REGULATORS] = {
	/* Consumers of VAUX1 */
	[AB8505_LDO_AUX1] = {
		.constraints = {
			.name = "V-AUX1",
			.min_uV = 3000000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(sensor_3v_consumers),
		.consumer_supplies = sensor_3v_consumers,
	},
	/* Consumers of VAUX2 */
	[AB8505_LDO_AUX2] = {
		.constraints = {
			.name = "V-AUX2",
			.min_uV = 3300000,
			.max_uV = 3300000,
			.apply_uV = 1,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(vreg_tsp_a3v3_consumers),
		.consumer_supplies = vreg_tsp_a3v3_consumers,
	},
	/* Consumers of VAUX3 */
	[AB8505_LDO_AUX3] = {
		.constraints = {
			.name = "V-AUX3",
			.min_uV = 1100000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_STATUS |
					  REGULATOR_CHANGE_MODE,
			.valid_modes_mask = REGULATOR_MODE_NORMAL |
					    REGULATOR_MODE_IDLE,
		},
		.num_consumer_supplies = ARRAY_SIZE(vdd_tf_2v91_consumers),
		.consumer_supplies = vdd_tf_2v91_consumers,
	},
	/* Consumers of VAUX4 */
	[AB8505_LDO_AUX4] = {
		.constraints = {
			.name = "V-AUX4",
			.min_uV = 3300000,
			.max_uV = 3300000,
			.apply_uV = 1,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(vdd_key_led_3v3_consumers),
		.consumer_supplies = vdd_key_led_3v3_consumers,
	},
	/* Consumers of VAUX5 */
	[AB8505_LDO_AUX5] = {
		.constraints = {
			.name = "V-AUX5",
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(vreg_tsp_1v8_consumers),
		.consumer_supplies = vreg_tsp_1v8_consumers,
	},
	/* Consumers of VAUX6 */
	[AB8505_LDO_AUX6] = {
		.constraints = {
			.name = "V-AUX6",
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(sensor_1v8_consumers),
		.consumer_supplies = sensor_1v8_consumers,
	},
	/* supply for gpadc, ADC LDO */
	[AB8505_LDO_ADC] = {
		.constraints = {
			.name = "V-ADC",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8505_vadc_consumers),
		.consumer_supplies = ab8505_vadc_consumers,
	},
	/* supply for ab8500-vaudio, VAUDIO LDO */
	[AB8505_LDO_AUDIO] = {
		.constraints = {
			.name = "V-AUD",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8505_vaudio_consumers),
		.consumer_supplies = ab8505_vaudio_consumers,
	},
	/* supply for v-anamic1 VAMic1-LDO */
	[AB8505_LDO_ANAMIC1] = {
		.constraints = {
			.name = "V-AMIC1",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8505_vamic1_consumers),
		.consumer_supplies = ab8505_vamic1_consumers,
	},
	/* supply for v-amic2, VAMIC2 LDO, reuse constants for AMIC1 */
	[AB8505_LDO_ANAMIC2] = {
		.constraints = {
			.name = "V-AMIC2",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8505_vamic2_consumers),
		.consumer_supplies = ab8505_vamic2_consumers,
	},
	/* supply for v-aux8, VAUX8 LDO */
	[AB8505_LDO_AUX8] = {
		.constraints = {
			.name = "V-AUX8",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8505_vaux8_consumers),
		.consumer_supplies = ab8505_vaux8_consumers,
	},
	/* supply for v-intcore12, VINTCORE12 LDO */
	[AB8505_LDO_INTCORE] = {
		.constraints = {
			.name = "V-INTCORE",
			.min_uV = 1250000,
			.max_uV = 1350000,
			.input_uV = 1800000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_STATUS |
					  REGULATOR_CHANGE_MODE |
					  REGULATOR_CHANGE_DRMS,
			.valid_modes_mask = REGULATOR_MODE_NORMAL |
					    REGULATOR_MODE_IDLE,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8505_vintcore_consumers),
		.consumer_supplies = ab8505_vintcore_consumers,
	},
	/* supply for U8500 CSI-DSI, VANA LDO */
	[AB8505_LDO_ANA] = {
		.constraints = {
			.name = "V-CSI-DSI",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.boot_on = 1, // already on for splash screen
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8505_vana_consumers),
		.consumer_supplies = ab8505_vana_consumers,
	},
	/* sysclkreq 2 pin */
	[AB8505_SYSCLKREQ_2] = {
		.constraints = {
			.name = "V-SYSCLKREQ-2",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies =
			ARRAY_SIZE(ab8505_sysclkreq_2_consumers),
		.consumer_supplies = ab8505_sysclkreq_2_consumers,
	},
	/* sysclkreq 4 pin */
	[AB8505_SYSCLKREQ_4] = {
		.constraints = {
			.name = "V-SYSCLKREQ-4",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies =
			ARRAY_SIZE(ab8505_sysclkreq_4_consumers),
		.consumer_supplies = ab8505_sysclkreq_4_consumers,
	},
};

struct ab8500_regulator_platform_data kyle_r0_0_regulator_plat_data = {
	.reg_init               = kyle_r0_0_reg_init,
	.num_reg_init           = ARRAY_SIZE(kyle_r0_0_reg_init),
	.regulator              = kyle_r0_0_regulators,
	.num_regulator          = ARRAY_SIZE(kyle_r0_0_regulators),
};

struct ab8500_regulator_platform_data kyle_r0_1_regulator_plat_data = {
	.reg_init               = kyle_r0_1_reg_init,
	.num_reg_init           = ARRAY_SIZE(kyle_r0_1_reg_init),
	.regulator              = kyle_r0_1_regulators,
	.num_regulator          = ARRAY_SIZE(kyle_r0_1_regulators),
};

