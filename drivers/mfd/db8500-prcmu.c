/*
 * Copyright (C) STMicroelectronics 2009
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License Terms: GNU General Public License v2
 * Author: Kumar Sanghvi <kumar.sanghvi@stericsson.com>
 * Author: Sundar Iyer <sundar.iyer@stericsson.com>
 * Author: Mattias Nilsson <mattias.i.nilsson@stericsson.com>
 *
 * U8500 PRCM Unit interface driver
 * 
 * DB8500 LiveOPP: 
 *         Huang Ji (cocafe@xda-developers.com) <cocafehj@gmail.com>
 *         mkaluza (mkaluza@xda-developers.com) 
 * 
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/cpufreq.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/bitops.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/mfd/core.h>
#include <linux/mfd/dbx500-prcmu.h>
#include <linux/mfd/ux500_wdt.h>
#include <linux/mfd/dbx500_temp.h>
#include <linux/regulator/db8500-prcmu.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/abx500.h>
#include <linux/wakelock.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/db8500-regs.h>
#include <mach/hardware.h>
#include <mach/prcmu-debug.h>
#include "dbx500-prcmu-regs.h"

#ifdef CONFIG_SAMSUNG_PANIC_DISPLAY_DEVICES
#define PRCMU_I2C_TIMEOUT	0x0F000000
#endif //CONFIG_SAMSUNG_PANIC_DISPLAY_DEVICES

#define CREATE_TRACE_POINTS
#include "dbx500-prcmu-trace.h"

#include <linux/ftrace_event.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
void log_this(u8 pc, char* a, u32 extra1, char* b, u32 extra2);

static char *fw_project_name(u8 project);

/* Offset for the firmware version within the TCPM */
#define PRCMU_FW_VERSION_OFFSET 0xA4

/* Index of different voltages to be used when accessing AVSData */
#define PRCM_AVS_BASE		0x2FC
#define PRCM_AVS_VBB_RET	(PRCM_AVS_BASE + 0x0)
#define PRCM_AVS_VBB_MAX_OPP	(PRCM_AVS_BASE + 0x1)
#define PRCM_AVS_VBB_100_OPP	(PRCM_AVS_BASE + 0x2)
#define PRCM_AVS_VBB_50_OPP	(PRCM_AVS_BASE + 0x3)
#define PRCM_AVS_VARM_MAX_OPP	(PRCM_AVS_BASE + 0x4)
#define PRCM_AVS_VARM_100_OPP	(PRCM_AVS_BASE + 0x5)
#define PRCM_AVS_VARM_50_OPP	(PRCM_AVS_BASE + 0x6)
#define PRCM_AVS_VARM_RET	(PRCM_AVS_BASE + 0x7)
#define PRCM_AVS_VAPE_100_OPP	(PRCM_AVS_BASE + 0x8)
#define PRCM_AVS_VAPE_50_OPP	(PRCM_AVS_BASE + 0x9)
#define PRCM_AVS_VMOD_100_OPP	(PRCM_AVS_BASE + 0xA)
#define PRCM_AVS_VMOD_50_OPP	(PRCM_AVS_BASE + 0xB)
#define PRCM_AVS_VSAFE		(PRCM_AVS_BASE + 0xC)

#define PRCM_AVS_VOLTAGE		0
#define PRCM_AVS_VOLTAGE_MASK		0x3f
#define PRCM_AVS_ISSLOWSTARTUP		6
#define PRCM_AVS_ISSLOWSTARTUP_MASK	(1 << PRCM_AVS_ISSLOWSTARTUP)
#define PRCM_AVS_ISMODEENABLE		7
#define PRCM_AVS_ISMODEENABLE_MASK	(1 << PRCM_AVS_ISMODEENABLE)

#define PRCM_BOOT_STATUS	0xFFF
#define PRCM_ROMCODE_A2P	0xFFE
#define PRCM_ROMCODE_P2A	0xFFD
#define PRCM_XP70_CUR_PWR_STATE 0xFFC      /* 4 BYTES */

#define PRCM_SW_RST_REASON 0xFF8 /* 2 bytes */

#define PRCM_TCDM_VOICE_CALL_FLAG 0xDD4 /* 4 bytes */

#define _PRCM_MBOX_HEADER		0xFE8 /* 16 bytes */
#define PRCM_MBOX_HEADER_REQ_MB0	(_PRCM_MBOX_HEADER + 0x0)
#define PRCM_MBOX_HEADER_REQ_MB1	(_PRCM_MBOX_HEADER + 0x1)
#define PRCM_MBOX_HEADER_REQ_MB2	(_PRCM_MBOX_HEADER + 0x2)
#define PRCM_MBOX_HEADER_REQ_MB3	(_PRCM_MBOX_HEADER + 0x3)
#define PRCM_MBOX_HEADER_REQ_MB4	(_PRCM_MBOX_HEADER + 0x4)
#define PRCM_MBOX_HEADER_REQ_MB5	(_PRCM_MBOX_HEADER + 0x5)
#define PRCM_MBOX_HEADER_ACK_MB0	(_PRCM_MBOX_HEADER + 0x8)

/* Req Mailboxes */
#define PRCM_REQ_MB0 0xFDC /* 12 bytes  */
#define PRCM_REQ_MB1 0xFD0 /* 12 bytes  */
#define PRCM_REQ_MB2 0xFC0 /* 16 bytes  */
#define PRCM_REQ_MB3 0xE4C /* 372 bytes  */
#define PRCM_REQ_MB4 0xE48 /* 4 bytes  */
#define PRCM_REQ_MB5 0xE44 /* 4 bytes  */

/* Ack Mailboxes */
#define PRCM_ACK_MB0 0xE08 /* 52 bytes  */
#define PRCM_ACK_MB1 0xE04 /* 4 bytes */
#define PRCM_ACK_MB2 0xE00 /* 4 bytes */
#define PRCM_ACK_MB3 0xDFC /* 4 bytes */
#define PRCM_ACK_MB4 0xDF8 /* 4 bytes */
#define PRCM_ACK_MB5 0xDF4 /* 4 bytes */

/* Mailbox 0 headers */
#define MB0H_POWER_STATE_TRANS		0
#define MB0H_CONFIG_WAKEUPS_EXE		1
#define MB0H_READ_WAKEUP_ACK		3
#define MB0H_CONFIG_WAKEUPS_SLEEP	4

#define MB0H_WAKEUP_EXE 2
#define MB0H_WAKEUP_SLEEP 5

/* Mailbox 0 REQs */
#define PRCM_REQ_MB0_AP_POWER_STATE	(PRCM_REQ_MB0 + 0x0)
#define PRCM_REQ_MB0_AP_PLL_STATE	(PRCM_REQ_MB0 + 0x1)
#define PRCM_REQ_MB0_ULP_CLOCK_STATE	(PRCM_REQ_MB0 + 0x2)
#define PRCM_REQ_MB0_DO_NOT_WFI		(PRCM_REQ_MB0 + 0x3)
#define PRCM_REQ_MB0_WAKEUP_8500	(PRCM_REQ_MB0 + 0x4)
#define PRCM_REQ_MB0_WAKEUP_4500	(PRCM_REQ_MB0 + 0x8)

/* Mailbox 0 ACKs */
#define PRCM_ACK_MB0_AP_PWRSTTR_STATUS	(PRCM_ACK_MB0 + 0x0)
#define PRCM_ACK_MB0_READ_POINTER	(PRCM_ACK_MB0 + 0x1)
#define PRCM_ACK_MB0_WAKEUP_0_8500	(PRCM_ACK_MB0 + 0x4)
#define PRCM_ACK_MB0_WAKEUP_0_4500	(PRCM_ACK_MB0 + 0x8)
#define PRCM_ACK_MB0_WAKEUP_1_8500	(PRCM_ACK_MB0 + 0x1C)
#define PRCM_ACK_MB0_WAKEUP_1_4500	(PRCM_ACK_MB0 + 0x20)
#define PRCM_ACK_MB0_EVENT_4500_NUMBERS	20

/* Mailbox 1 headers */
#define MB1H_ARM_APE_OPP 0x0
#define MB1H_RESET_MODEM 0x2
#define MB1H_REQUEST_APE_OPP_100_VOLT 0x3
#define MB1H_RELEASE_APE_OPP_100_VOLT 0x4
#define MB1H_RELEASE_USB_WAKEUP 0x5
#define MB1H_PLL_ON_OFF 0x6

/* Mailbox 1 Requests */
#define PRCM_REQ_MB1_ARM_OPP			(PRCM_REQ_MB1 + 0x0)
#define PRCM_REQ_MB1_APE_OPP			(PRCM_REQ_MB1 + 0x1)
#define PRCM_REQ_MB1_PLL_ON_OFF			(PRCM_REQ_MB1 + 0x4)
#define PLL_SOC0_OFF	0x1
#define PLL_SOC0_ON	0x2
#define PLL_SOC1_OFF	0x4
#define PLL_SOC1_ON	0x8

/* Mailbox 1 ACKs */
#define PRCM_ACK_MB1_CURRENT_ARM_OPP	(PRCM_ACK_MB1 + 0x0)
#define PRCM_ACK_MB1_CURRENT_APE_OPP	(PRCM_ACK_MB1 + 0x1)
#define PRCM_ACK_MB1_APE_VOLTAGE_STATUS	(PRCM_ACK_MB1 + 0x2)
#define PRCM_ACK_MB1_DVFS_STATUS	(PRCM_ACK_MB1 + 0x3)

/* Mailbox 2 headers */
#define MB2H_DPS	0x0
#define MB2H_AUTO_PWR	0x1

/* Mailbox 2 REQs */
#define PRCM_REQ_MB2_SVA_MMDSP		(PRCM_REQ_MB2 + 0x0)
#define PRCM_REQ_MB2_SVA_PIPE		(PRCM_REQ_MB2 + 0x1)
#define PRCM_REQ_MB2_SIA_MMDSP		(PRCM_REQ_MB2 + 0x2)
#define PRCM_REQ_MB2_SIA_PIPE		(PRCM_REQ_MB2 + 0x3)
#define PRCM_REQ_MB2_SGA		(PRCM_REQ_MB2 + 0x4)
#define PRCM_REQ_MB2_B2R2_MCDE		(PRCM_REQ_MB2 + 0x5)
#define PRCM_REQ_MB2_ESRAM12		(PRCM_REQ_MB2 + 0x6)
#define PRCM_REQ_MB2_ESRAM34		(PRCM_REQ_MB2 + 0x7)
#define PRCM_REQ_MB2_AUTO_PM_SLEEP	(PRCM_REQ_MB2 + 0x8)
#define PRCM_REQ_MB2_AUTO_PM_IDLE	(PRCM_REQ_MB2 + 0xC)

/* Mailbox 2 ACKs */
#define PRCM_ACK_MB2_DPS_STATUS (PRCM_ACK_MB2 + 0x0)
#define HWACC_PWR_ST_OK 0xFE

/* Mailbox 3 headers */
#define MB3H_ANC	0x0
#define MB3H_SIDETONE	0x1
#define MB3H_SYSCLK	0xE

/* Mailbox 3 Requests */
#define PRCM_REQ_MB3_ANC_FIR_COEFF	(PRCM_REQ_MB3 + 0x0)
#define PRCM_REQ_MB3_ANC_IIR_COEFF	(PRCM_REQ_MB3 + 0x20)
#define PRCM_REQ_MB3_ANC_SHIFTER	(PRCM_REQ_MB3 + 0x60)
#define PRCM_REQ_MB3_ANC_WARP		(PRCM_REQ_MB3 + 0x64)
#define PRCM_REQ_MB3_SIDETONE_FIR_GAIN	(PRCM_REQ_MB3 + 0x68)
#define PRCM_REQ_MB3_SIDETONE_FIR_COEFF	(PRCM_REQ_MB3 + 0x6C)
#define PRCM_REQ_MB3_SYSCLK_MGT		(PRCM_REQ_MB3 + 0x16C)

/* Mailbox 3 ACKs */
#define PRCM_ACK_MB3_TRACE_MSG	(PRCM_ACK_MB3 + 0x00)
#define PRCM_ACK_MB3_LOG_REQ	(PRCM_ACK_MB3 + 0x01)
#define MB3_LOG_REQ_PRCMU_REGS	(1 << 0)
#define MB3_LOG_REQ_TCDM	(1 << 1)
#define MB3_LOG_REQ_AB_REGS	(1 << 2)

/* Mailbox 4 headers */
#define MB4H_DDR_INIT	0x0
#define MB4H_MEM_ST	0x1
#define MB4H_HOTDOG	0x12
#define MB4H_HOTMON	0x13
#define MB4H_HOT_PERIOD	0x14
#define MB4H_A9WDOG_CONF 0x16
#define MB4H_A9WDOG_EN   0x17
#define MB4H_A9WDOG_DIS  0x18
#define MB4H_A9WDOG_LOAD 0x19
#define MB4H_A9WDOG_KICK 0x20

/* Mailbox 4 Requests */
#define PRCM_REQ_MB4_DDR_ST_AP_SLEEP_IDLE	(PRCM_REQ_MB4 + 0x0)
#define PRCM_REQ_MB4_DDR_ST_AP_DEEP_IDLE	(PRCM_REQ_MB4 + 0x1)
#define PRCM_REQ_MB4_ESRAM0_ST			(PRCM_REQ_MB4 + 0x3)
#define PRCM_REQ_MB4_HOTDOG_THRESHOLD		(PRCM_REQ_MB4 + 0x0)
#define PRCM_REQ_MB4_HOTMON_LOW			(PRCM_REQ_MB4 + 0x0)
#define PRCM_REQ_MB4_HOTMON_HIGH		(PRCM_REQ_MB4 + 0x1)
#define PRCM_REQ_MB4_HOTMON_CONFIG		(PRCM_REQ_MB4 + 0x2)
#define PRCM_REQ_MB4_HOT_PERIOD			(PRCM_REQ_MB4 + 0x0)
#define HOTMON_CONFIG_LOW			BIT(0)
#define HOTMON_CONFIG_HIGH			BIT(1)
#define PRCM_REQ_MB4_A9WDOG_0			(PRCM_REQ_MB4 + 0x0)
#define PRCM_REQ_MB4_A9WDOG_1			(PRCM_REQ_MB4 + 0x1)
#define PRCM_REQ_MB4_A9WDOG_2			(PRCM_REQ_MB4 + 0x2)
#define PRCM_REQ_MB4_A9WDOG_3			(PRCM_REQ_MB4 + 0x3)
#define A9WDOG_AUTO_OFF_EN			BIT(7)
#define A9WDOG_AUTO_OFF_DIS			0
#define A9WDOG_ID_MASK				0xf

/* Mailbox 5 Requests */
#define PRCM_REQ_MB5_I2C_SLAVE_OP	(PRCM_REQ_MB5 + 0x0)
#define PRCM_REQ_MB5_I2C_HW_BITS	(PRCM_REQ_MB5 + 0x1)
#define PRCM_REQ_MB5_I2C_REG		(PRCM_REQ_MB5 + 0x2)
#define PRCM_REQ_MB5_I2C_VAL		(PRCM_REQ_MB5 + 0x3)
#define PRCMU_I2C_WRITE(slave) (((slave) << 1) | BIT(6))
#define PRCMU_I2C_READ(slave) (((slave) << 1) | BIT(0) | BIT(6))
#define PRCMU_I2C_STOP_EN		BIT(3)

/* Mailbox 5 ACKs */
#define PRCM_ACK_MB5_I2C_STATUS	(PRCM_ACK_MB5 + 0x1)
#define PRCM_ACK_MB5_I2C_VAL	(PRCM_ACK_MB5 + 0x3)
#define I2C_WR_OK 0x1
#define I2C_RD_OK 0x2

#define NUM_MB 8
#define MBOX_BIT BIT
#define ALL_MBOX_BITS (MBOX_BIT(NUM_MB) - 1)

/*
 * Wakeups/IRQs
 */

#define WAKEUP_BIT_RTC BIT(0)
#define WAKEUP_BIT_RTT0 BIT(1)
#define WAKEUP_BIT_RTT1 BIT(2)
#define WAKEUP_BIT_HSI0 BIT(3)
#define WAKEUP_BIT_HSI1 BIT(4)
#define WAKEUP_BIT_CA_WAKE BIT(5)
#define WAKEUP_BIT_USB BIT(6)
#define WAKEUP_BIT_ABB BIT(7)
#define WAKEUP_BIT_ABB_FIFO BIT(8)
#define WAKEUP_BIT_SYSCLK_OK BIT(9)
#define WAKEUP_BIT_CA_SLEEP BIT(10)
#define WAKEUP_BIT_AC_WAKE_ACK BIT(11)
#define WAKEUP_BIT_SIDE_TONE_OK BIT(12)
#define WAKEUP_BIT_ANC_OK BIT(13)
#define WAKEUP_BIT_SW_ERROR BIT(14)
#define WAKEUP_BIT_AC_SLEEP_ACK BIT(15)
#define WAKEUP_BIT_ARM BIT(17)
#define WAKEUP_BIT_HOTMON_LOW BIT(18)
#define WAKEUP_BIT_HOTMON_HIGH BIT(19)
#define WAKEUP_BIT_MODEM_SW_RESET_REQ BIT(20)
#define WAKEUP_BIT_GPIO0 BIT(23)
#define WAKEUP_BIT_GPIO1 BIT(24)
#define WAKEUP_BIT_GPIO2 BIT(25)
#define WAKEUP_BIT_GPIO3 BIT(26)
#define WAKEUP_BIT_GPIO4 BIT(27)
#define WAKEUP_BIT_GPIO5 BIT(28)
#define WAKEUP_BIT_GPIO6 BIT(29)
#define WAKEUP_BIT_GPIO7 BIT(30)
#define WAKEUP_BIT_GPIO8 BIT(31)

static struct {
	bool valid;
	struct prcmu_fw_version version;
} fw_info;

/*
 * This vector maps irq numbers to the bits in the bit field used in
 * communication with the PRCMU firmware.
 *
 * The reason for having this is to keep the irq numbers contiguous even though
 * the bits in the bit field are not. (The bits also have a tendency to move
 * around, to further complicate matters.)
 */
#define IRQ_INDEX(_name) ((IRQ_PRCMU_##_name) - IRQ_PRCMU_BASE)
#define IRQ_ENTRY(_name)[IRQ_INDEX(_name)] = (WAKEUP_BIT_##_name)
static u32 prcmu_irq_bit[NUM_PRCMU_WAKEUPS] = {
	IRQ_ENTRY(RTC),
	IRQ_ENTRY(RTT0),
	IRQ_ENTRY(RTT1),
	IRQ_ENTRY(HSI0),
	IRQ_ENTRY(HSI1),
	IRQ_ENTRY(CA_WAKE),
	IRQ_ENTRY(USB),
	IRQ_ENTRY(ABB),
	IRQ_ENTRY(ABB_FIFO),
	IRQ_ENTRY(CA_SLEEP),
	IRQ_ENTRY(ARM),
	IRQ_ENTRY(HOTMON_LOW),
	IRQ_ENTRY(HOTMON_HIGH),
	IRQ_ENTRY(MODEM_SW_RESET_REQ),
	IRQ_ENTRY(GPIO0),
	IRQ_ENTRY(GPIO1),
	IRQ_ENTRY(GPIO2),
	IRQ_ENTRY(GPIO3),
	IRQ_ENTRY(GPIO4),
	IRQ_ENTRY(GPIO5),
	IRQ_ENTRY(GPIO6),
	IRQ_ENTRY(GPIO7),
	IRQ_ENTRY(GPIO8)
};

#define VALID_WAKEUPS (BIT(NUM_PRCMU_WAKEUP_INDICES) - 1)
#define WAKEUP_ENTRY(_name)[PRCMU_WAKEUP_INDEX_##_name] = (WAKEUP_BIT_##_name)
static u32 prcmu_wakeup_bit[NUM_PRCMU_WAKEUP_INDICES] = {
	WAKEUP_ENTRY(RTC),
	WAKEUP_ENTRY(RTT0),
	WAKEUP_ENTRY(RTT1),
	WAKEUP_ENTRY(HSI0),
	WAKEUP_ENTRY(HSI1),
	WAKEUP_ENTRY(USB),
	WAKEUP_ENTRY(ABB),
	WAKEUP_ENTRY(ABB_FIFO),
	WAKEUP_ENTRY(ARM)
};

/*
 * mb0_transfer - state needed for mailbox 0 communication.
 * @lock:		The transaction lock.
 * @dbb_events_lock:	A lock used to handle concurrent access to (parts of)
 *			the request data.
 * @mask_work:		Work structure used for (un)masking wakeup interrupts.
 * @req:		Request data that need to persist between requests.
 */
static struct {
	spinlock_t lock;
	spinlock_t dbb_irqs_lock;
	struct work_struct mask_work;
	struct mutex ac_wake_lock;
	struct completion ac_wake_work;
	struct {
		u32 dbb_irqs;
		u32 dbb_wakeups;
		u32 abb_events;
	} req;
} mb0_transfer;

/*
 * mb1_transfer - state needed for mailbox 1 communication.
 * @lock:	The transaction lock.
 * @work:	The transaction completion structure.
 * @ape_opp:	The current APE OPP.
 * @ack:	Reply ("acknowledge") data.
 */
static struct {
	struct mutex lock;
	struct completion work;
	u8 ape_opp;
	struct {
		u8 header;
		u8 arm_opp;
		u8 ape_opp;
		u8 ape_voltage_status;
	} ack;
} mb1_transfer;

/*
 * mb2_transfer - state needed for mailbox 2 communication.
 * @lock:            The transaction lock.
 * @work:            The transaction completion structure.
 * @auto_pm_lock:    The autonomous power management configuration lock.
 * @auto_pm_enabled: A flag indicating whether autonomous PM is enabled.
 * @req:             Request data that need to persist between requests.
 * @ack:             Reply ("acknowledge") data.
 */
static struct {
	struct mutex lock;
	struct completion work;
	spinlock_t auto_pm_lock;
	bool auto_pm_enabled;
	struct {
		u8 status;
	} ack;
} mb2_transfer;

/*
 * mb3_transfer - state needed for mailbox 3 communication.
 * @lock:		The request lock.
 * @sysclk_lock:	A lock used to handle concurrent sysclk requests.
 * @sysclk_work:	Work structure used for sysclk requests.
 */
static struct {
	spinlock_t lock;
	struct mutex sysclk_lock;
	struct completion sysclk_work;
	spinlock_t fw_log_lock;
	struct work_struct fw_log_work;
	u8 fw_log_req;
} mb3_transfer;

/*
 * mb4_transfer - state needed for mailbox 4 communication.
 * @lock:	The transaction lock.
 * @work:	The transaction completion structure.
 */
static struct {
	struct mutex lock;
	struct completion work;
} mb4_transfer;

/*
 * mb5_transfer - state needed for mailbox 5 communication.
 * @lock:	The transaction lock.
 * @work:	The transaction completion structure.
 * @ack:	Reply ("acknowledge") data.
 */
static struct {
	struct mutex lock;
	struct completion work;
	struct {
		u8 status;
		u8 value;
	} ack;
} mb5_transfer;

static atomic_t ac_wake_req_state = ATOMIC_INIT(0);

/* Functions definition */
static void compute_armss_rate(void);

/* Spinlocks */
static DEFINE_SPINLOCK(prcmu_lock);
static DEFINE_SPINLOCK(clkout_lock);

/* Global var to runtime determine TCDM base for v2 or v1 */
static __iomem void *tcdm_base;
static __iomem void *prcmu_base;

/*
 * Copies of the startup values of the reset status register and the SW reset
 * code.
 */
static u32 reset_status_copy;
static u16 reset_code_copy;

struct clk_mgt {
	void __iomem *reg;
	u32 pllsw;
	int branch;
	bool clk38div;
};

enum {
	PLL_RAW,
	PLL_FIX,
	PLL_DIV
};

static DEFINE_SPINLOCK(clk_mgt_lock);

#define CLK_MGT_ENTRY(_name, _branch, _clk38div)[PRCMU_##_name] = \
	{ (PRCM_##_name##_MGT), 0 , _branch, _clk38div}
struct clk_mgt clk_mgt[PRCMU_NUM_REG_CLOCKS] = {
	CLK_MGT_ENTRY(SGACLK, PLL_DIV, false),
	CLK_MGT_ENTRY(UARTCLK, PLL_FIX, true),
	CLK_MGT_ENTRY(MSP02CLK, PLL_FIX, true),
	CLK_MGT_ENTRY(MSP1CLK, PLL_FIX, true),
	CLK_MGT_ENTRY(I2CCLK, PLL_FIX, true),
	CLK_MGT_ENTRY(SDMMCCLK, PLL_DIV, true),
	CLK_MGT_ENTRY(SLIMCLK, PLL_FIX, true),
	CLK_MGT_ENTRY(PER1CLK, PLL_DIV, true),
	CLK_MGT_ENTRY(PER2CLK, PLL_DIV, true),
	CLK_MGT_ENTRY(PER3CLK, PLL_DIV, true),
	CLK_MGT_ENTRY(PER5CLK, PLL_DIV, true),
	CLK_MGT_ENTRY(PER6CLK, PLL_DIV, true),
	CLK_MGT_ENTRY(PER7CLK, PLL_DIV, true),
	CLK_MGT_ENTRY(LCDCLK, PLL_FIX, true),
	CLK_MGT_ENTRY(BMLCLK, PLL_DIV, true),
	CLK_MGT_ENTRY(HSITXCLK, PLL_DIV, true),
	CLK_MGT_ENTRY(HSIRXCLK, PLL_DIV, true),
	CLK_MGT_ENTRY(HDMICLK, PLL_FIX, false),
	CLK_MGT_ENTRY(APEATCLK, PLL_DIV, true),
	CLK_MGT_ENTRY(APETRACECLK, PLL_DIV, true),
	CLK_MGT_ENTRY(MCDECLK, PLL_DIV, true),
	CLK_MGT_ENTRY(IPI2CCLK, PLL_FIX, true),
	CLK_MGT_ENTRY(DSIALTCLK, PLL_FIX, false),
	CLK_MGT_ENTRY(DMACLK, PLL_DIV, true),
	CLK_MGT_ENTRY(B2R2CLK, PLL_DIV, true),
	CLK_MGT_ENTRY(TVCLK, PLL_FIX, true),
	CLK_MGT_ENTRY(SSPCLK, PLL_FIX, true),
	CLK_MGT_ENTRY(RNGCLK, PLL_FIX, true),
	CLK_MGT_ENTRY(UICCCLK, PLL_FIX, false),
};

struct dsiclk {
	u32 divsel_mask;
	u32 divsel_shift;
	u32 divsel;
};

static struct dsiclk dsiclk[2] = {
	{
		.divsel_mask = PRCM_DSI_PLLOUT_SEL_DSI0_PLLOUT_DIVSEL_MASK,
		.divsel_shift = PRCM_DSI_PLLOUT_SEL_DSI0_PLLOUT_DIVSEL_SHIFT,
		.divsel = PRCM_DSI_PLLOUT_SEL_PHI,
	},
	{
		.divsel_mask = PRCM_DSI_PLLOUT_SEL_DSI1_PLLOUT_DIVSEL_MASK,
		.divsel_shift = PRCM_DSI_PLLOUT_SEL_DSI1_PLLOUT_DIVSEL_SHIFT,
		.divsel = PRCM_DSI_PLLOUT_SEL_PHI,
	}
};

struct dsiescclk {
	u32 en;
	u32 div_mask;
	u32 div_shift;
};

static struct dsiescclk dsiescclk[3] = {
	{
		.en = PRCM_DSITVCLK_DIV_DSI0_ESC_CLK_EN,
		.div_mask = PRCM_DSITVCLK_DIV_DSI0_ESC_CLK_DIV_MASK,
		.div_shift = PRCM_DSITVCLK_DIV_DSI0_ESC_CLK_DIV_SHIFT,
	},
	{
		.en = PRCM_DSITVCLK_DIV_DSI1_ESC_CLK_EN,
		.div_mask = PRCM_DSITVCLK_DIV_DSI1_ESC_CLK_DIV_MASK,
		.div_shift = PRCM_DSITVCLK_DIV_DSI1_ESC_CLK_DIV_SHIFT,
	},
	{
		.en = PRCM_DSITVCLK_DIV_DSI2_ESC_CLK_EN,
		.div_mask = PRCM_DSITVCLK_DIV_DSI2_ESC_CLK_DIV_MASK,
		.div_shift = PRCM_DSITVCLK_DIV_DSI2_ESC_CLK_DIV_SHIFT,
	}
};


/*
* Used by MCDE to setup all necessary PRCMU registers
*/
#define PRCMU_RESET_DSIPLL		0x00004000
#define PRCMU_UNCLAMP_DSIPLL		0x00400800

#define PRCMU_CLK_PLL_DIV_SHIFT		0
#define PRCMU_CLK_PLL_SW_SHIFT		5
#define PRCMU_CLK_38			(1 << 9)
#define PRCMU_CLK_38_SRC		(1 << 10)
#define PRCMU_CLK_38_DIV		(1 << 11)

/* PLLDIV=12, PLLSW=4 (PLLDDR) */
#define PRCMU_DSI_CLOCK_SETTING		0x0000008C

/* DPI 50000000 Hz */
#define PRCMU_DPI_CLOCK_SETTING		((1 << PRCMU_CLK_PLL_SW_SHIFT) | \
					  (16 << PRCMU_CLK_PLL_DIV_SHIFT))
#define PRCMU_DSI_LP_CLOCK_SETTING_ULPPLL_ON	0x00000E00
#define PRCMU_DSI_LP_CLOCK_SETTING_ULPPLL_OFF	0x00000A00

/* D=101, N=1, R=4, SELDIV2=0 */
#define PRCMU_PLLDSI_FREQ_SETTING	0x00040165

#define PRCMU_ENABLE_PLLDSI		0x00000001
#define PRCMU_DISABLE_PLLDSI		0x00000000
#define PRCMU_RELEASE_RESET_DSS		0x0000400C
#define PRCMU_DSI_PLLOUT_SEL_SETTING	0x00000202
/* ESC clk, div0=1, div1=1, div2=3 */
#define PRCMU_ENABLE_ESCAPE_CLOCK_DIV	0x07030101
#define PRCMU_DISABLE_ESCAPE_CLOCK_DIV	0x00030101
#define PRCMU_DSI_RESET_SW		0x00000007

#define PRCMU_PLLDSI_LOCKP_LOCKED	0x3

struct wake_lock prcmu_uart_wake_lock;
extern void ux500_ci_dbg_console(void);

int db8500_prcmu_enable_dsipll(void)
{
	int i;

	/* Clear DSIPLL_RESETN */
	writel(PRCMU_RESET_DSIPLL, PRCM_APE_RESETN_CLR);
	/* Unclamp DSIPLL in/out */
	writel(PRCMU_UNCLAMP_DSIPLL, PRCM_MMIP_LS_CLAMP_CLR);

	/* Set DSI PLL FREQ */
	writel(PRCMU_PLLDSI_FREQ_SETTING, PRCM_PLLDSI_FREQ);
	writel(PRCMU_DSI_PLLOUT_SEL_SETTING, PRCM_DSI_PLLOUT_SEL);
	/* Enable Escape clocks */
	writel(PRCMU_ENABLE_ESCAPE_CLOCK_DIV, PRCM_DSITVCLK_DIV);

	/* Start DSI PLL */
	writel(PRCMU_ENABLE_PLLDSI, PRCM_PLLDSI_ENABLE);
	/* Reset DSI PLL */
	writel(PRCMU_DSI_RESET_SW, PRCM_DSI_SW_RESET);
	for (i = 0; i < 10; i++) {
		if ((readl(PRCM_PLLDSI_LOCKP) & PRCMU_PLLDSI_LOCKP_LOCKED)
					== PRCMU_PLLDSI_LOCKP_LOCKED)
			break;
		udelay(100);
	}
	/* Set DSIPLL_RESETN */
	writel(PRCMU_RESET_DSIPLL, PRCM_APE_RESETN_SET);
	return 0;
}

int db8500_prcmu_disable_dsipll(void)
{
	/* Disable dsi pll */
	writel(PRCMU_DISABLE_PLLDSI, PRCM_PLLDSI_ENABLE);
	/* Disable  escapeclock */
	writel(PRCMU_DISABLE_ESCAPE_CLOCK_DIV, PRCM_DSITVCLK_DIV);
	return 0;
}

int db8500_prcmu_set_display_clocks(void)
{
	unsigned long flags;

	spin_lock_irqsave(&clk_mgt_lock, flags);

	/* Grab the HW semaphore. */
	while ((readl(PRCM_SEM) & PRCM_SEM_PRCM_SEM) != 0)
		cpu_relax();

	writel(PRCMU_DSI_CLOCK_SETTING, PRCM_HDMICLK_MGT);

	if (prcmu_is_ulppll_disabled())
		writel(PRCMU_DSI_LP_CLOCK_SETTING_ULPPLL_OFF, PRCM_TVCLK_MGT);
	else
		writel(PRCMU_DSI_LP_CLOCK_SETTING_ULPPLL_ON, PRCM_TVCLK_MGT);

	writel(PRCMU_DPI_CLOCK_SETTING, PRCM_LCDCLK_MGT);

	/* Release the HW semaphore. */
	writel(0, PRCM_SEM);

	spin_unlock_irqrestore(&clk_mgt_lock, flags);

	return 0;
}
#if defined(CONFIG_MACH_SEC_GOLDEN_CHN) || defined(CONFIG_MACH_GAVINI_CHN) || defined(CONFIG_MACH_CODINA_CHN) 
static u32 db8500_prcmu_tcdm_read(unsigned int reg)
{
	return readl(tcdm_base + reg);
}
#endif

static u32 db8500_prcmu_read(unsigned int reg)
{
	return readl(_PRCMU_BASE + reg);
}

static void db8500_prcmu_write(unsigned int reg, u32 value)
{
	unsigned long flags;

	spin_lock_irqsave(&prcmu_lock, flags);
	writel(value, (_PRCMU_BASE + reg));
	spin_unlock_irqrestore(&prcmu_lock, flags);
}

static void db8500_prcmu_write_masked(unsigned int reg, u32 mask, u32 value)
{
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&prcmu_lock, flags);
	val = readl(_PRCMU_BASE + reg);
	val = ((val & ~mask) | (value & mask));
	writel(val, (_PRCMU_BASE + reg));
	spin_unlock_irqrestore(&prcmu_lock, flags);
}

u32 db8500_prcmu_readl(u32 reg)
{
	return readl(prcmu_base + reg);
}

void db8500_prcmu_writel(u32 reg, u32 value)
{
	unsigned long flags;

	spin_lock_irqsave(&prcmu_lock, flags);
	writel(value, prcmu_base + reg);
	spin_unlock_irqrestore(&prcmu_lock, flags);
}

void db8500_prcmu_writel_relaxed(u32 reg, u32 value)
{
	unsigned long flags;

	spin_lock_irqsave(&prcmu_lock, flags);
	writel_relaxed(value, prcmu_base + reg);
	spin_unlock_irqrestore(&prcmu_lock, flags);
}

u32 db8500_prcmu_tcdm_readb(u32 reg)
{
	return readb(tcdm_base + reg);
}

void db8500_prcmu_tcdm_writeb(u32 reg, u32 value)
{
	writeb(value, tcdm_base + reg);
}

u32 db8500_prcmu_tcdm_readl(u32 reg)
{
	return readl(tcdm_base + reg);
}

void db8500_prcmu_tcdm_writel(u32 reg, u32 value)
{
	writel(value, tcdm_base + reg);
}

/*
 * Dump AB8500 registers, PRCMU registers and PRCMU data memory
 * on critical errors.
 */
static void db8500_prcmu_debug_dump(bool dump_abb)
{
	dump_stack();

	/* Dump prcmu registers and data memory */
	prcmu_debug_dump_regs();
	prcmu_debug_dump_data_mem();

	/*
	 * Dump AB8500 registers last since i2c transfer will
	 * cause changes in the prcmu registers and data.
	 */
	if (dump_abb)
		abx500_dump_all_banks();
}

static void db8500_init_prcmu_fw_version(void)
{
	void *tcpm_base = ioremap_nocache(U8500_PRCMU_TCPM_BASE, SZ_4K);

	if (tcpm_base != NULL) {
		u32 version;
		version = readl(tcpm_base + PRCMU_FW_VERSION_OFFSET);
		fw_info.version.project = version & 0xFF;
		fw_info.version.api_version = (version >> 8) & 0xFF;
		fw_info.version.func_version = (version >> 16) & 0xFF;
		fw_info.version.errata = (version >> 24) & 0xFF;

		strncpy(fw_info.version.project_name,
			fw_project_name(fw_info.version.project),
			PRCMU_FW_PROJECT_NAME_LEN);

		fw_info.valid = true;

		iounmap(tcpm_base);
	}
}

static struct prcmu_fw_version *db8500_prcmu_get_fw_version(void)
{
	struct prcmu_fw_version *ver;

	if (fw_info.valid)
		ver = &fw_info.version;
	else
		db8500_init_prcmu_fw_version();

	return fw_info.valid ? ver : NULL;
}

/*
 * it's really redundant job to see this value for ApSleep
 * however, there's a suspicious problem(ER416165) which seems to be related to ApSleep.
 * The same suspicion goes for ER472557.
 * @dedicated usage only on cpuidle
 */
bool prcmu_is_mcdeclk_on(void)
{
	volatile unsigned int reg_value;
	reg_value = readl(PRCM_MCDECLK_MGT);
	return (reg_value & 0x100 /* MCDECLKEN */);
}
EXPORT_SYMBOL(prcmu_is_mcdeclk_on);

bool prcmu_is_mmcclk_on(void)
{
	volatile unsigned int reg_value;
	reg_value = readl(PRCM_SDMMCCLK_MGT);
	return (reg_value & 0x100 /* SDMMCCLKEN */);
}
EXPORT_SYMBOL(prcmu_is_mmcclk_on);

bool prcmu_is_ulppll_disabled(void)
{
	struct prcmu_fw_version *ver;

	ver = db8500_prcmu_get_fw_version();

	if (ver)
		return (ver->project == PRCMU_FW_PROJECT_U8420_SYSCLK);
	else
		return false;
}

/*  frequency available  */
static struct cpufreq_frequency_table *freq_table;

#ifndef CONFIG_DB8500_LIVEOPP
static bool db8500_prcmu_has_arm_maxopp(void)
{
	return (readb(tcdm_base + PRCM_AVS_VARM_MAX_OPP) &
		PRCM_AVS_ISMODEENABLE_MASK) == PRCM_AVS_ISMODEENABLE_MASK;
}
#endif /* CONFIG_DB8500_LIVEOPP */

static void db8500_prcmu_vc(bool enable)
{
	writel((enable ? 0xF : 0), (tcdm_base + PRCM_TCDM_VOICE_CALL_FLAG));
}

/**
 * prcmu_get_boot_status - PRCMU boot status checking
 * Returns: the current PRCMU boot status
 */
int prcmu_get_boot_status(void)
{
	return readb(tcdm_base + PRCM_BOOT_STATUS);
}

/**
 * prcmu_set_rc_a2p - This function is used to run few power state sequences
 * @val: Value to be set, i.e. transition requested
 * Returns: 0 on success, -EINVAL on invalid argument
 *
 * This function is used to run the following power state sequences -
 * any state to ApReset,  ApDeepSleep to ApExecute, ApExecute to ApDeepSleep
 */
int prcmu_set_rc_a2p(enum romcode_write val)
{
	if (val < RDY_2_DS || val > RDY_2_XP70_RST)
		return -EINVAL;
	writeb(val, (tcdm_base + PRCM_ROMCODE_A2P));
	return 0;
}

/**
 * prcmu_get_rc_p2a - This function is used to get power state sequences
 * Returns: the power transition that has last happened
 *
 * This function can return the following transitions-
 * any state to ApReset,  ApDeepSleep to ApExecute, ApExecute to ApDeepSleep
 */
enum romcode_read prcmu_get_rc_p2a(void)
{
	return readb(tcdm_base + PRCM_ROMCODE_P2A);
}

/**
 * prcmu_get_current_mode - Return the current XP70 power mode
 * Returns: Returns the current AP(ARM) power mode: init,
 * apBoot, apExecute, apDeepSleep, apSleep, apIdle, apReset
 */
enum ap_pwrst prcmu_get_xp70_current_state(void)
{
	return readb(tcdm_base + PRCM_XP70_CUR_PWR_STATE);
}

/**
 * db8500_prcmu_config_clkout - Configure one of the programmable clock outputs.
 * @clkout:	The CLKOUT number (0 or 1).
 * @source:	The clock to be used (one of the PRCMU_CLKSRC_*).
 * @div:	The divider to be applied.
 *
 * Configures one of the programmable clock outputs (CLKOUTs).
 * @div should be in the range [1,63] to request a configuration, or 0 to
 * inform that the configuration is no longer requested.
 */
static int db8500_prcmu_config_clkout(u8 clkout, u8 source, u8 div)
{
	static int requests[2];
	int r = 0;
	unsigned long flags;
	u32 val;
	u32 bits;
	u32 mask;
	u32 div_mask;

	BUG_ON(clkout > 1);
	BUG_ON(div > 63);
	BUG_ON((clkout == 0) && (source > PRCMU_CLKSRC_CLK009));

	if (!div && !requests[clkout])
		return -EINVAL;

	switch (clkout) {
	case 0:
		div_mask = PRCM_CLKOCR_CLKODIV0_MASK;
		mask = (PRCM_CLKOCR_CLKODIV0_MASK | PRCM_CLKOCR_CLKOSEL0_MASK);
		bits = ((source << PRCM_CLKOCR_CLKOSEL0_SHIFT) |
			(div << PRCM_CLKOCR_CLKODIV0_SHIFT));
		break;
	case 1:
		div_mask = PRCM_CLKOCR_CLKODIV1_MASK;
		mask = (PRCM_CLKOCR_CLKODIV1_MASK | PRCM_CLKOCR_CLKOSEL1_MASK |
			PRCM_CLKOCR_CLK1TYPE);
		bits = ((source << PRCM_CLKOCR_CLKOSEL1_SHIFT) |
			(div << PRCM_CLKOCR_CLKODIV1_SHIFT));
		break;
	}
	bits &= mask;

	spin_lock_irqsave(&clkout_lock, flags);

	val = readl(PRCM_CLKOCR);
	if (val & div_mask) {
		if (div) {
			if ((val & mask) != bits) {
				r = -EBUSY;
				goto unlock_and_return;
			}
		} else {
			if ((val & mask & ~div_mask) != bits) {
				r = -EINVAL;
				goto unlock_and_return;
			}
		}
	}
	writel((bits | (val & ~mask)), PRCM_CLKOCR);
	requests[clkout] += (div ? 1 : -1);

unlock_and_return:
	spin_unlock_irqrestore(&clkout_lock, flags);

	return r;
}

static u8 fw_trans[] = {
	0x00,/* PRCMU_AP_NO_CHANGE */
	0x01,/* PRCMU_AP_SLEEP */
	0x04,/*	PRCMU_AP_DEEP_SLEEP */
	0x05,/*	PRCMU_AP_IDLE */
	0x07,/*	PRCMU_AP_DEEP_IDLE*/
};

int db8500_prcmu_set_power_state(u8 state, bool keep_ulp_clk, bool keep_ap_pll)
{
	unsigned long flags;

	BUG_ON((state == PRCMU_AP_NO_CHANGE) ||
			(state >= ARRAY_SIZE(fw_trans)));
	spin_lock_irqsave(&mb0_transfer.lock, flags);

	while (readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(0))
		cpu_relax();

	writeb(MB0H_POWER_STATE_TRANS, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB0));
	writeb(fw_trans[state],
			(tcdm_base + PRCM_REQ_MB0_AP_POWER_STATE));
	writeb((keep_ap_pll ? 1 : 0), (tcdm_base + PRCM_REQ_MB0_AP_PLL_STATE));
	writeb((keep_ulp_clk ? 1 : 0),
		(tcdm_base + PRCM_REQ_MB0_ULP_CLOCK_STATE));
	writeb(0, (tcdm_base + PRCM_REQ_MB0_DO_NOT_WFI));
	log_this(100, "state", state, NULL, 0);
	writel(MBOX_BIT(0), PRCM_MBOX_CPU_SET);

	spin_unlock_irqrestore(&mb0_transfer.lock, flags);

	trace_u8500_set_power_state(
			fw_trans[state], keep_ulp_clk, keep_ap_pll);
	return 0;
}

static u8 db8500_prcmu_get_power_state_result(void)
{
	u8 status;
	status = readb(tcdm_base + PRCM_ACK_MB0_AP_PWRSTTR_STATUS);
	trace_u8500_get_power_state_result(status);
	return status;
}

/* This function should only be called while mb0_transfer.lock is held. */
static void config_wakeups(void)
{
	const u8 header[2] = {
		MB0H_CONFIG_WAKEUPS_EXE,
		MB0H_CONFIG_WAKEUPS_SLEEP
	};
	static u32 last_dbb_events;
	static u32 last_abb_events;
	u32 dbb_events;
	u32 abb_events;
	unsigned int i;

	dbb_events = mb0_transfer.req.dbb_irqs | mb0_transfer.req.dbb_wakeups;
	dbb_events |= (WAKEUP_BIT_AC_WAKE_ACK | WAKEUP_BIT_AC_SLEEP_ACK);

	abb_events = mb0_transfer.req.abb_events;

	if ((dbb_events == last_dbb_events) && (abb_events == last_abb_events))
		return;

	for (i = 0; i < 2; i++) {
		while (readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(0))
			cpu_relax();
		writel(dbb_events, (tcdm_base + PRCM_REQ_MB0_WAKEUP_8500));
		writel(abb_events, (tcdm_base + PRCM_REQ_MB0_WAKEUP_4500));
		writeb(header[i], (tcdm_base + PRCM_MBOX_HEADER_REQ_MB0));
		if (i == 0)
			log_this(110, "dbb", dbb_events, "abb", abb_events);
		else
			log_this(111, "ddb", dbb_events, "abb", abb_events);
		writel(MBOX_BIT(0), PRCM_MBOX_CPU_SET);
	}
	last_dbb_events = dbb_events;
	last_abb_events = abb_events;
	trace_u8500_config_wakeups(dbb_events, abb_events);
}

static void db8500_prcmu_enable_wakeups(u32 wakeups)
{
	unsigned long flags;
	u32 bits;
	int i;

	BUG_ON(wakeups != (wakeups & VALID_WAKEUPS));

	for (i = 0, bits = 0; i < NUM_PRCMU_WAKEUP_INDICES; i++) {
		if (wakeups & BIT(i))
			bits |= prcmu_wakeup_bit[i];
	}

	spin_lock_irqsave(&mb0_transfer.lock, flags);

	mb0_transfer.req.dbb_wakeups = bits;
	config_wakeups();

	spin_unlock_irqrestore(&mb0_transfer.lock, flags);
}

static void db8500_prcmu_config_abb_event_readout(u32 abb_events)
{
	unsigned long flags;

	spin_lock_irqsave(&mb0_transfer.lock, flags);

	mb0_transfer.req.abb_events = abb_events;
	config_wakeups();

	spin_unlock_irqrestore(&mb0_transfer.lock, flags);
}

static void db8500_prcmu_get_abb_event_buffer(void __iomem **buf)
{
	if (readb(tcdm_base + PRCM_ACK_MB0_READ_POINTER) & 1)
		*buf = (tcdm_base + PRCM_ACK_MB0_WAKEUP_1_4500);
	else
		*buf = (tcdm_base + PRCM_ACK_MB0_WAKEUP_0_4500);
}

#ifdef CONFIG_DB8500_LIVEOPP
#include <linux/kobject.h>
#include <linux/mfd/db8500-liveopp.h>

#define LIVEOPP_VER		"2.1"
/* #define LIVEOPP_DEBUG	1 */

#define NOCHG			0
#define SET_PLL			1
#define SET_EXT			1
#define SET_VOLT		1

static unsigned int last_arm_idx = 0;

#if LIVEOPP_DEBUG > 1
static int liveopp_start = 0;
#endif

/**
 * PLLARM REGISTER: 0xAABBCCDD
 * AA - OPP_50_enabled
 * BB - divisor
 * CC - unknown
 * DD - multiplier
 *
 * when going from OPP100 to  OPP50 mode, it doesn't set BB and DD, only AA to 01
 * and when going 50 -> 100, only AA=00, so from 700MHz it'll do 1.4GHz
 * OPPMAX -> OPP100/50 sets pll BB, DD and optionally AA (for opp 50)
 * 
 * cocafe: 
 * 	References of PLL register bits: dbx500-prcmu-regs.h L#138
 */

/**
 * VARM REG CHANGES
 * OPP_EXT -> OPP* 0C = 0x12
 * OPP* -> OPPMAX 0B = 0x32
 * OPP* -> OPP100 0B = 0x28
 * NO-OPS:
 *   - OPP* -> OPPEXT
 *   - OPP100 -> OPP50
 *   - OPPMAX -> OPP50
 */

/**
 * Hard-coded Custom ARM Frequency and Voltage Table
 *
 * PRCMU_QOS_DEFAULT_VALUE 	-1
 * PRCMU_QOS_MAX_VALUE 		-2
 */

#define ARM_50_OPP_IDX 		3	/*  400MHz */
#define ARM_100_OPP_IDX 	7	/*  800MHz */
#define ARM_MAX_OPP_IDX 	9	/* 1000MHz */

static struct liveopp_arm_table liveopp_arm[] = {
#ifdef CONFIG_MACH_CODINA
	{ 100000,   99840,  ARM_EXTCLK, SET_EXT, 0x582,   NOCHG, 0x00050168, SET_VOLT, 0x0C, 0x18, 0xDB,  -1,  -1},
	{ 200000,  199680,  ARM_EXTCLK,   NOCHG, 0x581,   NOCHG, 0x00050168, SET_VOLT, 0x0C, 0x18, 0xDB,  -1,  -1},
	{ 300000,  299520,  ARM_50_OPP,   NOCHG, 0x741, SET_PLL, 0x00050127, SET_VOLT, 0x0C, 0x18, 0xDB,  -1,  -1},
	{ 400000,  399360,  ARM_50_OPP,   NOCHG, 0x741, SET_PLL, 0x01050168, SET_VOLT, 0x0C, 0x18, 0xDB,  -1,  -1},
	{ 500000,  499200,  ARM_50_OPP,   NOCHG, 0x741, SET_PLL, 0x0001010D, SET_VOLT, 0x0C, 0x20, 0xDB,  -2,  -2},
	{ 600000,  599040,  ARM_50_OPP,   NOCHG, 0x741, SET_PLL, 0x0005014E, SET_VOLT, 0x0C, 0x20, 0xDB,  -2,  -2},
	{ 700000,  698880,  ARM_50_OPP,   NOCHG, 0x741, SET_PLL, 0x0005015B, SET_VOLT, 0x0C, 0x24, 0xDB,  -2,  -2},
	{ 800000,  798720, ARM_100_OPP,   NOCHG, 0x741, SET_PLL, 0x00050168,    NOCHG, 0x0B, 0x24, 0xDB,  -2,  -2},
	{ 900000,  898560, ARM_100_OPP,   NOCHG, 0x741, SET_PLL, 0x00050175, SET_VOLT, 0x0B, 0x2F, 0xDB,  -2,  -2},
	{1000000,  998400, ARM_100_OPP,   NOCHG, 0x741, SET_PLL, 0x0001011A,    NOCHG, 0x0B, 0x2F, 0xDB,  -2,  -2},
	{1050000, 1049600, ARM_MAX_OPP,   NOCHG, 0x741, SET_PLL, 0x00030152, SET_VOLT, 0x0B, 0x32, 0xDB,  -2,  -2},
	{1100000, 1100800, ARM_MAX_OPP,   NOCHG, 0x741, SET_PLL, 0x00030156, SET_VOLT, 0x0B, 0x36, 0x8F,  -2,  -2},
	{1150000, 1152000, ARM_MAX_OPP,   NOCHG, 0x741, SET_PLL, 0x0001011E, SET_VOLT, 0x0B, 0x36, 0x8F,  -2,  -2},
	{1200000, 1200000, ARM_MAX_OPP,   NOCHG, 0x741, SET_PLL, 0x0004017D, SET_VOLT, 0x0B, 0x39, 0x8F,  -2,  -2},
	{1250000, 1248800, ARM_MAX_OPP,   NOCHG, 0x741, SET_PLL, 0x00040182, SET_VOLT, 0x0B, 0x39, 0x8F,  -2,  -2},
#else
	{ 100000,   99840,  ARM_EXTCLK, SET_EXT, 0x582,   NOCHG, 0x00050168, SET_VOLT, 0x0C, 0x12, 0xDB,  25,  25},
	{ 200000,  199680,  ARM_EXTCLK, SET_EXT, 0x581,   NOCHG, 0x00050168, SET_VOLT, 0x0C, 0x12, 0xDB,  25,  25},
	{ 300000,  299520,  ARM_50_OPP,   NOCHG, 0x741, SET_PLL, 0x0105014E, SET_VOLT, 0x0C, 0x13, 0xDB,  25,  25},
	{ 400000,  399360,  ARM_50_OPP,   NOCHG, 0x741, SET_PLL, 0x01050168, SET_VOLT, 0x0C, 0x14, 0xDB,  25,  50},
	{ 500000,  499200,  ARM_50_OPP,   NOCHG, 0x741, SET_PLL, 0x01050182, SET_VOLT, 0x0C, 0x1B, 0xDB,  25,  50},
	{ 600000,  599040,  ARM_50_OPP,   NOCHG, 0x741, SET_PLL, 0x0105019C, SET_VOLT, 0x0C, 0x20, 0xDB,  50,  50},
	{ 700000,  698880,  ARM_50_OPP,   NOCHG, 0x741, SET_PLL, 0x010501B6, SET_VOLT, 0x0C, 0x22, 0xDB,  50,  50},
	{ 800000,  798720, ARM_100_OPP,   NOCHG, 0x741, SET_PLL, 0x00050168, SET_VOLT, 0x0B, 0x24, 0xDB, 100,  50},
	{ 900000,  898560, ARM_100_OPP,   NOCHG, 0x741, SET_PLL, 0x00050175, SET_VOLT, 0x0B, 0x29, 0xDB, 100, 100},
	{1000000,  998400, ARM_MAX_OPP,   NOCHG, 0x741, SET_PLL, 0x0001011A, SET_VOLT, 0x0B, 0x2F, 0xDB, 100, 100},
	{1050000, 1049600, ARM_MAX_OPP,   NOCHG, 0x741, SET_PLL, 0x00030152, SET_VOLT, 0x0B, 0x34, 0xDB, 100, 100},
	{1100000, 1100800, ARM_MAX_OPP,   NOCHG, 0x741, SET_PLL, 0x00030156, SET_VOLT, 0x0B, 0x3F, 0x8F, 100, 100},
	{1150000, 1152000, ARM_MAX_OPP,   NOCHG, 0x741, SET_PLL, 0x0001011E, SET_VOLT, 0x0B, 0x3F, 0x8F, 100, 100},
	{1200000, 1200000, ARM_MAX_OPP,   NOCHG, 0x741, SET_PLL, 0x0004017D, SET_VOLT, 0x0B, 0x3F, 0x8F, 100, 100},
	{1250000, 1248000, ARM_MAX_OPP,   NOCHG, 0x741, SET_PLL, 0x00040182, SET_VOLT, 0x0B, 0x3F, 0x8F, 100, 100},
#endif /* CONFIG_MACH_CODINA */
};

static const char *armopp_name[] = 
{
	"ARM_OPP_INIT",		/* 0x00 */
	"ARM_NO_CHANGE",	/* 0x01 */
	"ARM_100_OPP", 		/* 0x02 */
	"ARM_50_OPP", 		/* 0x03 */
	"ARM_MAX_OPP", 		/* 0x04 */
	"ARM_MAX_FREQ100OPP", 	/* 0x05 */
	"(null)",		/* 0x06 */
	"ARM_EXTCLK",		/* 0x07 */
};

static int varm_voltage(u8 raw)
{
	if (raw <= 0x35) {
		return (AB8500_VARM_MIN_UV + (raw * AB8500_VARM_STEP_UV));
	} else {
		return AB8500_VARM_MAX_UV;
	}
}

static int pllarm_freq(u32 raw)
{
	int multiple = raw & 0x000000FF;
	int divider = (raw & 0x00FF0000) >> 16;
	int half = (raw & 0x01000000) >> 24;
	int pll;

	pll = (multiple * PLLARM_FREQ_STEPS);
	pll /= divider;

	if (half) {
		pll /= 2;
	}

	return pll;
}

static inline void liveopp_set_armvolt(struct liveopp_arm_table table)
{
	/* Varm */
	prcmu_abb_write(AB8500_REGU_CTRL2, table.varm_sel, &table.varm_raw, 1);

	/* VBBp/VBBn */
	prcmu_abb_write(AB8500_REGU_CTRL2, AB8500_VBBX_REG, &table.vbbx_raw, 1);
}

static inline void liveopp_set_armpll(struct liveopp_arm_table table)
{
	/* ARM PLL */
	db8500_prcmu_writel(PRCMU_PLLARM_REG, table.pllarm_raw);
}

static inline void liveopp_set_armext(struct liveopp_arm_table table)
{
	/* ArmFixClk */
	db8500_prcmu_writel(PRCMU_ARMFIX_REG, table.extarm_raw);
}

static inline void liveopp_update_arm(struct liveopp_arm_table table, bool voltage_first)
{
	if (table.set_volt && voltage_first)
		liveopp_set_armvolt(table);

	if (table.set_pllarm)
		liveopp_set_armpll(table);

	if (table.set_extarm)
		liveopp_set_armext(table);

	if (table.set_volt && !voltage_first)
		liveopp_set_armvolt(table);
}

static inline void liveopp_update_opp(struct liveopp_arm_table table)
{
	if (table.ddr_opp)
		prcmu_qos_update_requirement(PRCMU_QOS_DDR_OPP, "cpufreq",
				     (signed char)table.ddr_opp);
	if (table.ape_opp)
		prcmu_qos_update_requirement(PRCMU_QOS_APE_OPP, "cpufreq",
				     (signed char)table.ape_opp);
}

#define ATTR_RO(_name)	\
	static struct kobj_attribute _name##_interface = __ATTR(_name, 0444, _name##_show, NULL);

#define ATTR_WO(_name)	\
	static struct kobj_attribute _name##_interface = __ATTR(_name, 0220, NULL, _name##_store);

#define ATTR_RW(_name)	\
	static struct kobj_attribute _name##_interface = __ATTR(_name, 0644, _name##_show, _name##_store);

/* 
 * LiveOPP sysfs interfaces
 */

static ssize_t version_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	sprintf(buf, "LiveOPP (%s), cocafe\n", LIVEOPP_VER);

	return strlen(buf);
}

ATTR_RO(version);

static ssize_t arm_extclk_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	u32 r, r2, r3;
	unsigned long rate;

	r = readl(PRCM_ARM_CHGCLKREQ);
	r2 = db8500_prcmu_readl(PRCMU_PLLDDR_REG);

	/* External ARM clock uses PLLDDR */
	/* We use PLLARM function here, they are common */
	rate = pllarm_freq(r2);

	/* Check PRCM_ARM_CHGCLKREQ divider */
	if (!(r & PRCM_ARM_CHGCLKREQ_PRCM_ARM_DIVSEL))
		rate /= 2;

	/* Check PRCM_ARMCLKFIX_MGT divider */
	r3 = readl(PRCM_ARMCLKFIX_MGT);
	r3 &= PRCM_CLK_MGT_CLKPLLDIV_MASK;
	rate /= r3;

	/* PLLDDR belongs to PLL_FIX branch */
	return sprintf(buf, "%lu kHz\n", rate / 2);
}
ATTR_RO(arm_extclk);

static ssize_t arm_pllclk_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d kHz\n", pllarm_freq(db8500_prcmu_readl(PRCMU_PLLARM_REG)));
}
ATTR_RO(arm_pllclk);

static ssize_t arm_varm_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	u8 varm[3];

	prcmu_abb_read(AB8500_REGU_CTRL2, 0x0A, &varm[0], 1);
	prcmu_abb_read(AB8500_REGU_CTRL2, 0x0B, &varm[1], 1);
	prcmu_abb_read(AB8500_REGU_CTRL2, 0x0C, &varm[2], 1);

	sprintf(buf,   "Last_idx: %d\n", last_arm_idx);
	sprintf(buf, "%sVarm_sel: %#04x\n", buf, liveopp_arm[last_arm_idx].varm_sel);
	sprintf(buf, "%sVarm[0x0A, 0x0B, 0x0C] : [%#04x, %#04x ,%#04x]\n", buf, varm[0], varm[1], varm[2]);

	return strlen(buf);
}
ATTR_RO(arm_varm);

static ssize_t arm_step_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf, int _index)
{
	/*
	if (_index >= ARRAY_SIZE(liveopp_arm))
		return sprintf(buf, "Not available\n");
	*/

	sprintf(buf,   "[LiveOPP ARM Step %d]\n\n", _index);
	sprintf(buf, "%sSet EXTARM:\t\t%s\n", buf, liveopp_arm[_index].set_extarm ? "Enabled" : "Disabled");
	sprintf(buf, "%sSet PLLARM:\t\t%s\n", buf, liveopp_arm[_index].set_pllarm ? "Enabled" : "Disabled");
	sprintf(buf, "%sSet Voltage:\t\t%s\n", buf, liveopp_arm[_index].set_volt ? "Enabled" : "Disabled");
	sprintf(buf, "%sFrequency show:\t\t%d kHz\n", buf, liveopp_arm[_index].freq_show);
	sprintf(buf, "%sFrequency real:\t\t%d kHz\n", buf, liveopp_arm[_index].set_pllarm ?
			pllarm_freq(liveopp_arm[_index].pllarm_raw) : liveopp_arm[_index].freq_raw);
	sprintf(buf, "%sArmFix:\t\t\t%#010x\n", buf, liveopp_arm[_index].extarm_raw);
	sprintf(buf, "%sArmPLL:\t\t\t%#010x\n", buf, liveopp_arm[_index].pllarm_raw);
	sprintf(buf, "%sArmOPP:\t\t\t%s (%#04x)\n", buf, armopp_name[(int)liveopp_arm[_index].arm_opp],
								     (int)liveopp_arm[_index].arm_opp);
	sprintf(buf, "%sVarm:\t\t\t%d uV (%#04x)\n", buf, varm_voltage(liveopp_arm[_index].varm_raw),
								     (int)liveopp_arm[_index].varm_raw);
	sprintf(buf, "%sVbbx:\t\t\t%#04x\n", buf, (int)liveopp_arm[_index].vbbx_raw);
	sprintf(buf, "%sQOS_DDR_OPP:\t\t\t%d\n", buf, (int)((signed char)liveopp_arm[_index].ddr_opp));
	sprintf(buf, "%sQOS_APE_OPP:\t\t\t%d\n", buf, (int)((signed char)liveopp_arm[_index].ape_opp));

	return sprintf(buf, "%s\n", buf);
}

static ssize_t arm_step_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count, int _index)
{
	int ret;
	int val;

	/*
	if (_index >= ARRAY_SIZE(liveopp_arm))
		return -EINVAL;
	*/

	if (!strncmp(buf, "set_ext=", 8)) {
		ret = sscanf(&buf[8], "%d", &val);
		if ((!ret) || (val != 0 && val != 1)) {
			pr_err("[LiveOPP] Invalid value\n");
			return -EINVAL;
		}

		liveopp_arm[_index].set_extarm = val;

		return count;
	}

	if (!strncmp(buf, "set_pll=", 8)) {
		ret = sscanf(&buf[8], "%d", &val);
		if ((!ret) || (val != 0 && val != 1)) {
			pr_err("[LiveOPP] Invalid value\n");
			return -EINVAL;
		}

		liveopp_arm[_index].set_pllarm = val;

		return count;
	}

	if (!strncmp(buf, "set_volt=", 9)) {
		ret = sscanf(&buf[9], "%d", &val);
		if ((!ret) || (val != 0 && val != 1)) {
			pr_err("[LiveOPP] Invalid value\n");
			return -EINVAL;
		}

		liveopp_arm[_index].set_volt = val;

		return count;
	}

	if (!strncmp(buf, "opp=", 4)) {
		ret = sscanf(&buf[4], "%d", &val);
		if ((!ret) || (val < 0x00 || val > 0x07)) {
			pr_err("[LiveOPP] Invalid value\n");
			return -EINVAL;
		}

		liveopp_arm[_index].arm_opp = (unsigned char)val;

		return count;
	}

	if (!strncmp(buf, "pll=", 4)) {
		ret = sscanf(&buf[4], "%x", &val);
		if ((!ret)) {
			pr_err("[LiveOPP] Invalid value\n");
			return -EINVAL;
		}

		liveopp_arm[_index].pllarm_raw = val;

		return count;
	}

	if (!strncmp(buf, "ext=", 4)) {
		ret = sscanf(&buf[4], "%x", &val);
		if ((!ret)) {
			pr_err("[LiveOPP] Invalid value\n");
			return -EINVAL;
		}

		liveopp_arm[_index].extarm_raw = val;

		return count;
	}

	if (!strncmp(buf, "varm+", 5)) {
		liveopp_arm[_index].varm_raw ++;

		return count;
	}

	if (!strncmp(buf, "varm-", 5)) {
		liveopp_arm[_index].varm_raw --;

		return count;
	}
	if (!strncmp(buf, "varm=", 5)) {
		ret = sscanf(&buf[5], "%x", &val);
		if ((!ret)) {
			pr_err("[LiveOPP] Invalid value\n");
			return -EINVAL;
		}

		liveopp_arm[_index].varm_raw = val;

		return count;
	}
	if (!strncmp(buf, "vbbx=", 5)) {
		ret = sscanf(&buf[5], "%x", &val);
		if ((!ret)) {
			pr_err("[LiveOPP] Invalid value\n");
			return -EINVAL;
		}

		liveopp_arm[_index].vbbx_raw = val;

		return count;
	}

	if (!strncmp(buf, "apeopp=", 7)) {
		ret = sscanf(&buf[7], "%d", &val);
		if ((!ret) || (val != 25 && val != 50 && val != 100 && val != -1 && val != -2)) {
			pr_err("[LiveOPP] Invalid QOS_APE_OPP value. Enter 25, 50 or 100\n");
			return -EINVAL;
		}

		liveopp_arm[_index].ape_opp = (signed char)val;

		return count;
	}

	if (!strncmp(buf, "ddropp=", 7)) {
		ret = sscanf(&buf[7], "%d", &val);
		if ((!ret) || (val != 25 && val != 50 && val != 100 && val != -1 && val != -2)) {
			pr_err("[LiveOPP] Invalid QOS_DDR_OPP value. Enter 25, 50 or 100\n");
			return -EINVAL;
		}

		liveopp_arm[_index].ddr_opp = (signed char)val;

		return count;
	}

	return count;
}

#define ARM_STEP(_name, _index)											\
static ssize_t _name##_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)			\
{														\
	return arm_step_show(kobj, attr, buf, _index);								\
}														\
														\
static ssize_t _name##_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)	\
{														\
	return arm_step_store(kobj, attr, buf, count, _index);							\
}														\
ATTR_RW(_name);

ARM_STEP(arm_step00, 0);
ARM_STEP(arm_step01, 1);
ARM_STEP(arm_step02, 2);
ARM_STEP(arm_step03, 3);
ARM_STEP(arm_step04, 4);
ARM_STEP(arm_step05, 5);
ARM_STEP(arm_step06, 6);
ARM_STEP(arm_step07, 7);
ARM_STEP(arm_step08, 8);
ARM_STEP(arm_step09, 9);
ARM_STEP(arm_step10, 10);
ARM_STEP(arm_step11, 11);
ARM_STEP(arm_step12, 12);
ARM_STEP(arm_step13, 13);
ARM_STEP(arm_step14, 14);

#if LIVEOPP_DEBUG > 1
static ssize_t liveopp_start_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)		
{
	return sprintf(buf, "%d\n", liveopp_start);
}

static ssize_t liveopp_start_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)	
{
	return sscanf(buf, "%d", &liveopp_start);
}
ATTR_RW(liveopp_start);
#endif

static struct attribute *liveopp_attrs[] = {
#if LIVEOPP_DEBUG > 1
	&liveopp_start_interface.attr, 
#endif
	&version_interface.attr, 
	&arm_extclk_interface.attr, 
	&arm_pllclk_interface.attr, 
	&arm_varm_interface.attr,
	&arm_step00_interface.attr, 
	&arm_step01_interface.attr, 
	&arm_step02_interface.attr, 
	&arm_step03_interface.attr, 
	&arm_step04_interface.attr, 
	&arm_step05_interface.attr, 
	&arm_step06_interface.attr, 
	&arm_step07_interface.attr, 
	&arm_step08_interface.attr, 
	&arm_step09_interface.attr, 
	&arm_step10_interface.attr, 
	&arm_step11_interface.attr, 
	&arm_step12_interface.attr, 
	&arm_step13_interface.attr, 
	&arm_step14_interface.attr, 
	NULL,
};

static struct attribute_group liveopp_interface_group = {
	.attrs = liveopp_attrs,
};

static struct kobject *liveopp_kobject;

#endif /* CONFIG_DB8500_LIVEOPP */

#ifndef CONFIG_DB8500_LIVEOPP
/* db8500-prcmu : hard coded conversion */
static enum arm_opp db8500_idx2opp[] = {
	ARM_EXTCLK,
	ARM_50_OPP,
	ARM_100_OPP,
	ARM_MAX_OPP
};
#endif /* CONFIG_DB8500_LIVEOPP */

#define SET_ARM_OPP_TIMEOUT HZ

/**
 * db8500_prcmu_set_arm_opp - set the appropriate ARM OPP
 * @opp: The new ARM operating point to which transition is to be made
 * Returns: 0 on success, non-zero on failure
 *
 * This function sets the the operating point of the ARM.
 */

static int db8500_prcmu_set_arm_opp(u8 opp)
{
	int r;

	if (opp < ARM_NO_CHANGE || opp > ARM_EXTCLK)
		return -EINVAL;

	trace_u8500_set_arm_opp(opp);
	r = 0;

	mutex_lock(&mb1_transfer.lock);

	while (readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(1))
		cpu_relax();

	writeb(MB1H_ARM_APE_OPP, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB1));
	writeb(opp, (tcdm_base + PRCM_REQ_MB1_ARM_OPP));
	writeb(APE_NO_CHANGE, (tcdm_base + PRCM_REQ_MB1_APE_OPP));

	log_this(120, "OPP", opp, NULL, 0);
	writel(MBOX_BIT(1), PRCM_MBOX_CPU_SET);
	wait_for_completion_timeout(&mb1_transfer.work, SET_ARM_OPP_TIMEOUT);

	if ((mb1_transfer.ack.header != MB1H_ARM_APE_OPP) ||
	    (mb1_transfer.ack.arm_opp != opp)) {
		pr_err("%s: error: timed out (%ds)\n", __func__,
		       SET_ARM_OPP_TIMEOUT / HZ);
		r = -EIO;
	}
	compute_armss_rate();
	mutex_unlock(&mb1_transfer.lock);

	prcmu_debug_arm_opp_log(opp);

	return r;
}

#ifdef CONFIG_DB8500_LIVEOPP
static inline int db8500_prcmu_set_arm_lopp(u8 opp, int idx)
{
	int r;
	struct liveopp_arm_table table = liveopp_arm[idx];
	u8 last_opp = liveopp_arm[last_arm_idx].arm_opp;
	bool voltage_first = (idx > last_arm_idx);

	if (opp < ARM_NO_CHANGE || opp > ARM_EXTCLK)
		return -EINVAL;

	trace_u8500_set_arm_opp(opp);
	r = 0;

	mutex_lock(&mb1_transfer.lock);

	while (readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(1))
		cpu_relax();

	if (opp != last_opp) {
		if (last_opp == ARM_EXTCLK && opp == ARM_50_OPP) {
			table = liveopp_arm[ARM_50_OPP_IDX];
			prcmu_abb_write(AB8500_REGU_CTRL2, table.varm_sel, &table.varm_raw, 1);

			db8500_prcmu_writel(PRCMU_PLLARM_REG, table.pllarm_raw);
			voltage_first = (idx > ARM_50_OPP_IDX);
		}
		else if ((last_opp == ARM_50_OPP || last_opp == ARM_EXTCLK) && (opp == ARM_100_OPP || opp == ARM_MAX_OPP)) {
			if (last_arm_idx != ARM_50_OPP_IDX) {
				table = liveopp_arm[ARM_50_OPP_IDX];
				if (last_arm_idx < ARM_50_OPP_IDX)
					prcmu_abb_write(AB8500_REGU_CTRL2, table.varm_sel, &table.varm_raw, 1);
				db8500_prcmu_writel(PRCMU_PLLARM_REG, table.pllarm_raw);
			}
		}
		else if (last_opp == ARM_100_OPP && opp == ARM_50_OPP) {
			table = liveopp_arm[max(idx, ARM_50_OPP_IDX)];
			prcmu_abb_write(AB8500_REGU_CTRL2, table.varm_sel, &table.varm_raw, 1);
			if (last_arm_idx > ARM_100_OPP_IDX) {
				table = liveopp_arm[ARM_100_OPP_IDX];
				db8500_prcmu_writel(PRCMU_PLLARM_REG, table.pllarm_raw);
			}
			voltage_first = (idx > ARM_50_OPP_IDX);
		}
		else if (last_opp == ARM_MAX_OPP) {
			if (last_arm_idx > ARM_MAX_OPP_IDX) {
				table = liveopp_arm[ARM_MAX_OPP_IDX];
				db8500_prcmu_writel(PRCMU_PLLARM_REG, table.pllarm_raw);
			}
			if (opp == ARM_50_OPP || opp == ARM_EXTCLK) {
				table = liveopp_arm[max(idx, ARM_50_OPP_IDX)];
				prcmu_abb_write(AB8500_REGU_CTRL2, table.varm_sel, &table.varm_raw, 1);
				voltage_first = (idx > ARM_50_OPP_IDX);
			} else
				voltage_first = (idx > ARM_100_OPP_IDX);
		}
		writeb(MB1H_ARM_APE_OPP, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB1));
		writeb(opp, (tcdm_base + PRCM_REQ_MB1_ARM_OPP));
		writeb(APE_NO_CHANGE, (tcdm_base + PRCM_REQ_MB1_APE_OPP));

		log_this(120, "OPP", opp, NULL, 0);
		writel(MBOX_BIT(1), PRCM_MBOX_CPU_SET);
		wait_for_completion_timeout(&mb1_transfer.work, SET_ARM_OPP_TIMEOUT);

		if ((mb1_transfer.ack.header != MB1H_ARM_APE_OPP) ||
		    (mb1_transfer.ack.arm_opp != opp)) {
			pr_err("%s: error: timed out (%ds)\n", __func__,
			       SET_ARM_OPP_TIMEOUT / HZ);
			r = -EIO;
		}
	}
	liveopp_update_arm(liveopp_arm[idx], voltage_first);
	compute_armss_rate();
	mutex_unlock(&mb1_transfer.lock);

	prcmu_debug_arm_opp_log(opp);
	liveopp_update_opp(liveopp_arm[idx]);

	return r;
}
#endif /* CONFIG_DB8500_LIVEOPP */

/**
 * db8500_prcmu_get_arm_opp - get the current ARM OPP
 *
 * Returns: the current ARM OPP
 */
static int db8500_prcmu_get_arm_opp(void)
{
	return readb(tcdm_base + PRCM_ACK_MB1_CURRENT_ARM_OPP);
}

#ifdef CONFIG_DB8500_LIVEOPP
static unsigned long arm_get_rate(void)
{
	unsigned long rate;

	/*  catch early access */
	BUG_ON(!freq_table);

	rate = freq_table[last_arm_idx].frequency;

	return rate * 1000;
}
#else
static unsigned long arm_get_rate(void)
{
	unsigned long rate;
	int i, opp;
	int found = -1;
	opp = db8500_prcmu_get_arm_opp();
	for (i = 0; i < ARRAY_SIZE(db8500_idx2opp); i++)
		if (opp == db8500_idx2opp[i]) {
			found = i;
			break;
		}
	if (found == -1) {
		pr_err("prcmu: Failed to convert arm clk\n");
		found = ARRAY_SIZE(db8500_idx2opp)-1;
	}
	/*  catch early access */
	BUG_ON(!freq_table);
	rate =  freq_table[found].frequency;
	return rate * 1000;
}
#endif /* CONFIG_DB8500_LIVEOPP */
unsigned long (*toto)(void);

#ifdef CONFIG_DB8500_LIVEOPP
static int arm_set_rate(unsigned long rate)
{
	unsigned long frequency = rate / 1000;
	int i;

#if LIVEOPP_DEBUG > 1
	if (!liveopp_start)
		return 0;
#endif

#if LIVEOPP_DEBUG > 0
	pr_info("kHz: %10lu\n", frequency);
#endif

	/*  catch early access */
	BUG_ON(!freq_table);

	for (i = 0; i < ARRAY_SIZE(liveopp_arm); i++) {
		if (frequency == freq_table[i].frequency) {
			db8500_prcmu_set_arm_lopp(liveopp_arm[i].arm_opp, i);
			last_arm_idx = i;

			break;
		}
	}

	return 0;
}
#else
static int arm_set_rate(unsigned long rate)
{
	unsigned long frequency = rate / 1000;
	int found = -1, i;
	/*  catch early access */
	BUG_ON(!freq_table);
	for (i = 0; i < ARRAY_SIZE(db8500_idx2opp); i++)
		if (frequency == freq_table[i].frequency) {
			found = i;
			break;
		}
	if (found == -1)
		pr_err("prcmu: Failed to convert arm clk in opp\n");
	else
		return db8500_prcmu_set_arm_opp(db8500_idx2opp[found]);
	return -1;
}
#endif /* CONFIG_DB8500_LIVEOPP */

/**
 * db8500_prcmu_get_ddr_opp - get the current DDR OPP
 *
 * Returns: the current DDR OPP
 */
static int db8500_prcmu_get_ddr_opp(void)
{
	return readb(PRCM_DDR_SUBSYS_APE_MINBW);
}

/**
 * db8500_set_ddr_opp - set the appropriate DDR OPP
 * @opp: The new DDR operating point to which transition is to be made
 * Returns: 0 on success, non-zero on failure
 *
 * This function sets the operating point of the DDR.
 */
static int db8500_prcmu_set_ddr_opp(u8 opp)
{
	if (opp < DDR_100_OPP || opp > DDR_25_OPP)
		return -EINVAL;
	/* Changing the DDR OPP can hang the hardware pre-v21 */
	if (!cpu_is_u8500v20())
		writeb(opp, PRCM_DDR_SUBSYS_APE_MINBW);

	trace_u8500_set_ddr_opp(opp);
	return 0;
}

/* Divide the frequency of certain clocks by 2 for APE_50_PARTLY_25_OPP. */
static void request_even_slower_clocks(bool enable)
{
	void __iomem *clock_reg[] = {
		PRCM_ACLK_MGT,
		PRCM_DMACLK_MGT
	};
	unsigned long flags;
	unsigned int i;

	spin_lock_irqsave(&clk_mgt_lock, flags);

	/* Grab the HW semaphore. */
	while ((readl(PRCM_SEM) & PRCM_SEM_PRCM_SEM) != 0)
		cpu_relax();

	for (i = 0; i < ARRAY_SIZE(clock_reg); i++) {
		u32 val;
		u32 div;

		val = readl(clock_reg[i]);
		div = (val & PRCM_CLK_MGT_CLKPLLDIV_MASK);
		if (enable) {
			if ((div <= 1) || (div > 15)) {
				pr_err("prcmu: Bad clock divider %d in %s\n",
					div, __func__);
				goto unlock_and_return;
			}
			div <<= 1;
		} else {
			if (div <= 2)
				goto unlock_and_return;
			div >>= 1;
		}
		val = ((val & ~PRCM_CLK_MGT_CLKPLLDIV_MASK) |
			(div & PRCM_CLK_MGT_CLKPLLDIV_MASK));
		writel(val, clock_reg[i]);
	}

unlock_and_return:
	/* Release the HW semaphore. */
	writel(0, PRCM_SEM);

	spin_unlock_irqrestore(&clk_mgt_lock, flags);
}

/**
 * db8500_set_ape_opp - set the appropriate APE OPP
 * @opp: The new APE operating point to which transition is to be made
 * Returns: 0 on success, non-zero on failure
 *
 * This function sets the operating point of the APE.
 */
static int db8500_prcmu_set_ape_opp(u8 opp)
{
	int r = 0;
	u8 prcmu_opp_req;

	trace_u8500_set_ape_opp(opp);
	if (opp == mb1_transfer.ape_opp)
		return 0;

	mutex_lock(&mb1_transfer.lock);

	/* Exit APE_50_PARTLY_25_OPP */
	if (mb1_transfer.ape_opp == APE_50_PARTLY_25_OPP)
		request_even_slower_clocks(false);

	if ((opp != APE_100_OPP) && (mb1_transfer.ape_opp != APE_100_OPP))
		goto skip_message;

	while (readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(1))
		cpu_relax();

	prcmu_opp_req  = (opp == APE_50_PARTLY_25_OPP) ? APE_50_OPP : opp;

	writeb(MB1H_ARM_APE_OPP, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB1));
	writeb(ARM_NO_CHANGE, (tcdm_base + PRCM_REQ_MB1_ARM_OPP));
	writeb(prcmu_opp_req, (tcdm_base + PRCM_REQ_MB1_APE_OPP));

	log_this(130, "OPP", opp, NULL, 0);
	writel(MBOX_BIT(1), PRCM_MBOX_CPU_SET);
	wait_for_completion(&mb1_transfer.work);

	if ((mb1_transfer.ack.header != MB1H_ARM_APE_OPP) ||
		(mb1_transfer.ack.ape_opp != prcmu_opp_req))
		r = -EIO;

skip_message:
	if ((!r && (opp == APE_50_PARTLY_25_OPP)) ||
		/* Set APE_50_PARTLY_25_OPP back in case new opp failed */
		(r && (mb1_transfer.ape_opp == APE_50_PARTLY_25_OPP)))
		request_even_slower_clocks(true);
	if (!r)
		mb1_transfer.ape_opp = opp;

	mutex_unlock(&mb1_transfer.lock);

	return r;
}

/**
 * db8500_prcmu_get_ape_opp - get the current APE OPP
 *
 * Returns: the current APE OPP
 */
static int db8500_prcmu_get_ape_opp(void)
{
	return readb(tcdm_base + PRCM_ACK_MB1_CURRENT_APE_OPP);
}

/**
 * db8500_prcmu_request_ape_opp_100_voltage - Request APE OPP 100% voltage
 * @enable: true to request the higher voltage, false to drop a request.
 *
 * Calls to this function to enable and disable requests must be balanced.
 */
static int db8500_prcmu_request_ape_opp_100_voltage(bool enable)
{
	int r = 0;
	u8 header;
	static unsigned int requests;

	mutex_lock(&mb1_transfer.lock);

	if (enable) {
		if (0 != requests++)
			goto unlock_and_return;
		header = MB1H_REQUEST_APE_OPP_100_VOLT;
	} else {
		if (requests == 0) {
			r = -EIO;
			goto unlock_and_return;
		} else if (1 != requests--) {
			goto unlock_and_return;
		}
		header = MB1H_RELEASE_APE_OPP_100_VOLT;
	}

	while (readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(1))
		cpu_relax();

	writeb(header, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB1));

	log_this(140, "enable", enable, NULL, 0);
	writel(MBOX_BIT(1), PRCM_MBOX_CPU_SET);
	wait_for_completion(&mb1_transfer.work);

	if ((mb1_transfer.ack.header != header) ||
		((mb1_transfer.ack.ape_voltage_status & BIT(0)) != 0))
		r = -EIO;

unlock_and_return:
	mutex_unlock(&mb1_transfer.lock);

	return r;
}

/**
 * prcmu_release_usb_wakeup_state - release the state required by a USB wakeup
 *
 * This function releases the power state requirements of a USB wakeup.
 */
int prcmu_release_usb_wakeup_state(void)
{
	int r = 0;

	mutex_lock(&mb1_transfer.lock);

	while (readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(1))
		cpu_relax();

	writeb(MB1H_RELEASE_USB_WAKEUP,
		(tcdm_base + PRCM_MBOX_HEADER_REQ_MB1));

	log_this(150, NULL, 0, NULL, 0);
	writel(MBOX_BIT(1), PRCM_MBOX_CPU_SET);
	wait_for_completion(&mb1_transfer.work);

	if ((mb1_transfer.ack.header != MB1H_RELEASE_USB_WAKEUP) ||
		((mb1_transfer.ack.ape_voltage_status & BIT(0)) != 0))
		r = -EIO;

	mutex_unlock(&mb1_transfer.lock);

	return r;
}

static int request_pll(u8 clock, bool enable)
{
	int r = 0;

	if (clock == PRCMU_PLLSOC0)
		clock = (enable ? PLL_SOC0_ON : PLL_SOC0_OFF);
	else if (clock == PRCMU_PLLSOC1)
		clock = (enable ? PLL_SOC1_ON : PLL_SOC1_OFF);
	else
		return -EINVAL;

	mutex_lock(&mb1_transfer.lock);

	while (readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(1))
		cpu_relax();

	writeb(MB1H_PLL_ON_OFF, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB1));
	writeb(clock, (tcdm_base + PRCM_REQ_MB1_PLL_ON_OFF));

	log_this(160, "clock", clock, "enable", enable);
	writel(MBOX_BIT(1), PRCM_MBOX_CPU_SET);
	wait_for_completion(&mb1_transfer.work);

	if (mb1_transfer.ack.header != MB1H_PLL_ON_OFF)
		r = -EIO;

	mutex_unlock(&mb1_transfer.lock);

	return r;
}

/**
 * set_epod - set the state of a EPOD (power domain)
 * @epod_id: The EPOD to set
 * @epod_state: The new EPOD state
 *
 * This function sets the state of a EPOD (power domain). It may not be called
 * from interrupt context.
 */
static int set_epod(u16 epod_id, u8 epod_state)
{
	int r = 0;
	bool ram_retention = false;
	int i;

	/* check argument */
	BUG_ON(epod_id >= NUM_EPOD_ID);

	/* set flag if retention is possible */
	switch (epod_id) {
	case EPOD_ID_SVAMMDSP:
	case EPOD_ID_SIAMMDSP:
	case EPOD_ID_ESRAM12:
	case EPOD_ID_ESRAM34:
		ram_retention = true;
		break;
	}

	/* check argument */
	BUG_ON(epod_state > EPOD_STATE_ON);
	BUG_ON(epod_state == EPOD_STATE_RAMRET && !ram_retention);

	trace_u8500_set_epod(epod_id, epod_state);
	/* get lock */
	mutex_lock(&mb2_transfer.lock);

	/* wait for mailbox */
	while (readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(2))
		cpu_relax();

	/* fill in mailbox */
	for (i = 0; i < NUM_EPOD_ID; i++)
		writeb(EPOD_STATE_NO_CHANGE, (tcdm_base + PRCM_REQ_MB2 + i));
	writeb(epod_state, (tcdm_base + PRCM_REQ_MB2 + epod_id));

	writeb(MB2H_DPS, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB2));

	log_this(170, "epod_id", epod_id, "state", epod_state);
	writel(MBOX_BIT(2), PRCM_MBOX_CPU_SET);

	/*
	 * The current firmware version does not handle errors correctly,
	 * and we cannot recover if there is an error.
	 * This is expected to change when the firmware is updated.
	 */
	if (!wait_for_completion_timeout(&mb2_transfer.work,
			msecs_to_jiffies(20000))) {
		pr_err("prcmu: %s timed out (20 s) waiting for a reply.\n",
			__func__);
		r = -EIO;
		db8500_prcmu_debug_dump(true);
		goto unlock_and_return;
	}

	if (mb2_transfer.ack.status != HWACC_PWR_ST_OK)
		r = -EIO;

unlock_and_return:
	mutex_unlock(&mb2_transfer.lock);
	return r;
}
#ifdef CONFIG_SAMSUNG_PANIC_DISPLAY_DEVICES
/**
 * prcmu_set_epod - set the state of a EPOD (power domain) called in Kernel Panic state.
 * @epod_id: The EPOD to set
 * @epod_state: The new EPOD state
 *
 * This function sets the state of a EPOD (power domain). It may not be called
 * from interrupt context.
 */
int prcmu_panic_set_epod(u16 epod_id, u8 epod_state)
{
	int r = 0;
	bool ram_retention = false;
	int i;
	u32 timeout = PRCMU_I2C_TIMEOUT;
	u8 status;

	/* check argument */
	BUG_ON(epod_id >= NUM_EPOD_ID);

	/* set flag if retention is possible */
	switch (epod_id) {
	case EPOD_ID_SVAMMDSP:
	case EPOD_ID_SIAMMDSP:
	case EPOD_ID_ESRAM12:
	case EPOD_ID_ESRAM34:
		ram_retention = true;
		break;
	}

	/* check argument */
	BUG_ON(epod_state > EPOD_STATE_ON);
	BUG_ON(epod_state == EPOD_STATE_RAMRET && !ram_retention);

	if (readl(PRCM_ARM_IT1_VAL) & MBOX_BIT(2)) {
		/* clear mailbox 2 ack irq */
		writel(MBOX_BIT(2), PRCM_ARM_IT1_CLR);
	}

	while ((readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(2)) && timeout--)
		cpu_relax();

	if ( timeout == 0 ){
		pr_emerg("%s: timed out waiting for MBOX2 to be free.\n",__func__);
		return -EIO;
	}

	/* fill in mailbox */
	for (i = 0; i < NUM_EPOD_ID; i++)
		writeb(EPOD_STATE_NO_CHANGE, (tcdm_base + PRCM_REQ_MB2 + i));
	writeb(epod_state, (tcdm_base + PRCM_REQ_MB2 + epod_id));

	writeb(MB2H_DPS, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB2));

	writel(MBOX_BIT(2), (PRCM_MBOX_CPU_SET));

	/*
	 * The current firmware version does not handle errors correctly,
	 * and we cannot recover if there is an error.
	 * This is expected to change when the firmware is updated.
	 */
	timeout = PRCMU_I2C_TIMEOUT;

	while (!(readl(PRCM_ARM_IT1_VAL) & MBOX_BIT(2)) && timeout--)
		cpu_relax();

	if (!timeout) {
		pr_emerg("%s timed out waiting for a reply.\n",__func__);
		return -EIO;
	}

	status = readb(tcdm_base + PRCM_ACK_MB2_DPS_STATUS);

	/* clear mailbox 2 ack irq */
	writel(MBOX_BIT(2), PRCM_ARM_IT1_CLR);

	r = ((status == HWACC_PWR_ST_OK) ? 0 : -EIO);

	return r;
}
#endif

/**
 * db8500_prcmu_configure_auto_pm
 * - Configure autonomous power management.
 * @sleep: Configuration for ApSleep.
 * @idle:  Configuration for ApIdle.
 */
static void db8500_prcmu_configure_auto_pm(struct prcmu_auto_pm_config *sleep,
	struct prcmu_auto_pm_config *idle)
{
	u32 sleep_cfg;
	u32 idle_cfg;
	unsigned long flags;

	BUG_ON((sleep == NULL) || (idle == NULL));

	sleep_cfg = (sleep->sva_auto_pm_enable & 0xF);
	sleep_cfg = ((sleep_cfg << 4) | (sleep->sia_auto_pm_enable & 0xF));
	sleep_cfg = ((sleep_cfg << 8) | (sleep->sva_power_on & 0xFF));
	sleep_cfg = ((sleep_cfg << 8) | (sleep->sia_power_on & 0xFF));
	sleep_cfg = ((sleep_cfg << 4) | (sleep->sva_policy & 0xF));
	sleep_cfg = ((sleep_cfg << 4) | (sleep->sia_policy & 0xF));

	idle_cfg = (idle->sva_auto_pm_enable & 0xF);
	idle_cfg = ((idle_cfg << 4) | (idle->sia_auto_pm_enable & 0xF));
	idle_cfg = ((idle_cfg << 8) | (idle->sva_power_on & 0xFF));
	idle_cfg = ((idle_cfg << 8) | (idle->sia_power_on & 0xFF));
	idle_cfg = ((idle_cfg << 4) | (idle->sva_policy & 0xF));
	idle_cfg = ((idle_cfg << 4) | (idle->sia_policy & 0xF));

	spin_lock_irqsave(&mb2_transfer.auto_pm_lock, flags);

	/*
	 * The autonomous power management configuration is done through
	 * fields in mailbox 2, but these fields are only used as shared
	 * variables - i.e. there is no need to send a message.
	 */
	writel(sleep_cfg, (tcdm_base + PRCM_REQ_MB2_AUTO_PM_SLEEP));
	writel(idle_cfg, (tcdm_base + PRCM_REQ_MB2_AUTO_PM_IDLE));

	mb2_transfer.auto_pm_enabled =
		((sleep->sva_auto_pm_enable == PRCMU_AUTO_PM_ON) ||
		 (sleep->sia_auto_pm_enable == PRCMU_AUTO_PM_ON) ||
		 (idle->sva_auto_pm_enable == PRCMU_AUTO_PM_ON) ||
		 (idle->sia_auto_pm_enable == PRCMU_AUTO_PM_ON));

	spin_unlock_irqrestore(&mb2_transfer.auto_pm_lock, flags);
}

bool prcmu_is_auto_pm_enabled(void)
{
	return mb2_transfer.auto_pm_enabled;
}

static int request_sysclk(bool enable)
{
	int r;
	unsigned long flags;

	r = 0;

	mutex_lock(&mb3_transfer.sysclk_lock);

	spin_lock_irqsave(&mb3_transfer.lock, flags);

	while (readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(3))
		cpu_relax();

	writeb((enable ? ON : OFF), (tcdm_base + PRCM_REQ_MB3_SYSCLK_MGT));

	writeb(MB3H_SYSCLK, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB3));
	log_this(180, "enable", enable, NULL, 0);
	writel(MBOX_BIT(3), PRCM_MBOX_CPU_SET);

	spin_unlock_irqrestore(&mb3_transfer.lock, flags);

	/*
	 * The firmware only sends an ACK if we want to enable the
	 * SysClk, and it succeeds.
	 */
	if (enable && !wait_for_completion_timeout(&mb3_transfer.sysclk_work,
			msecs_to_jiffies(20000))) {
		pr_err("prcmu: %s timed out (20 s) waiting for a reply.\n",
			__func__);
		r = -EIO;
		db8500_prcmu_debug_dump(true);
	}

	mutex_unlock(&mb3_transfer.sysclk_lock);

	return r;
}

#define TIMCLK_CLOCK_RATE_ULPPLL_OFF 32768
#define TIMCLK_CLOCK_RATE_ULPPLL_ON (ROOT_CLOCK_RATE / 16)

static int request_timclk(bool enable)
{
	/* Use 32KHz clock to generate timer clken */
	u32 val;

	if (prcmu_is_ulppll_disabled())
		val = 0;
	else
		val = (PRCM_TCR_DOZE_MODE | PRCM_TCR_TENSEL_MASK);

	if (!enable)
		/*
		* Use TIMCLK but mask it with STOP_TIMER to disable the
		* timer clken.
		*/
		val |= PRCM_TCR_STOP_TIMERS |
			PRCM_TCR_DOZE_MODE |
			PRCM_TCR_TENSEL_MASK;

	writel(val, PRCM_TCR);

	return 0;
}

static int request_clock(u8 clock, bool enable)
{
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&clk_mgt_lock, flags);

	/* Grab the HW semaphore. */
	while ((readl(PRCM_SEM) & PRCM_SEM_PRCM_SEM) != 0)
		cpu_relax();

	val = readl(clk_mgt[clock].reg);
	if (enable) {
		val |= (PRCM_CLK_MGT_CLKEN | clk_mgt[clock].pllsw);
	} else {
		clk_mgt[clock].pllsw = (val & PRCM_CLK_MGT_CLKPLLSW_MASK);
		val &= ~(PRCM_CLK_MGT_CLKEN | PRCM_CLK_MGT_CLKPLLSW_MASK);
	}
	writel(val, clk_mgt[clock].reg);

	/* Release the HW semaphore. */
	writel(0, PRCM_SEM);

	spin_unlock_irqrestore(&clk_mgt_lock, flags);

	return 0;
}

static int request_sga_clock(u8 clock, bool enable)
{
	u32 val;
	int ret;

	if (enable) {
		val = readl(PRCM_CGATING_BYPASS);
		writel(val | PRCM_CGATING_BYPASS_ICN2, PRCM_CGATING_BYPASS);
	}

	ret = request_clock(clock, enable);

	if (!ret && !enable) {
		val = readl(PRCM_CGATING_BYPASS);
		writel(val & ~PRCM_CGATING_BYPASS_ICN2, PRCM_CGATING_BYPASS);
	}

	return ret;
}

static inline bool plldsi_locked(void)
{
	return (readl(PRCM_PLLDSI_LOCKP) &
		(PRCM_PLLDSI_LOCKP_PRCM_PLLDSI_LOCKP10 |
		 PRCM_PLLDSI_LOCKP_PRCM_PLLDSI_LOCKP3)) ==
		(PRCM_PLLDSI_LOCKP_PRCM_PLLDSI_LOCKP10 |
		 PRCM_PLLDSI_LOCKP_PRCM_PLLDSI_LOCKP3);
}

static int request_plldsi(bool enable)
{
	int r = 0;
	u32 val;

	writel((PRCM_MMIP_LS_CLAMP_DSIPLL_CLAMP |
		PRCM_MMIP_LS_CLAMP_DSIPLL_CLAMPI), (enable ?
		PRCM_MMIP_LS_CLAMP_CLR : PRCM_MMIP_LS_CLAMP_SET));

	val = readl(PRCM_PLLDSI_ENABLE);
	if (enable)
		val |= PRCM_PLLDSI_ENABLE_PRCM_PLLDSI_ENABLE;
	else
		val &= ~PRCM_PLLDSI_ENABLE_PRCM_PLLDSI_ENABLE;
	writel(val, PRCM_PLLDSI_ENABLE);

	if (enable) {
		unsigned int i;
		bool locked = plldsi_locked();

		for (i = 10; !locked && (i > 0); --i) {
			udelay(100);
			locked = plldsi_locked();
		}
		if (locked) {
			/* Wait an extra 40 us after the pll is locked */
			udelay(40);
			writel(PRCM_APE_RESETN_DSIPLL_RESETN,
				PRCM_APE_RESETN_SET);
		} else {
			writel((PRCM_MMIP_LS_CLAMP_DSIPLL_CLAMP |
				PRCM_MMIP_LS_CLAMP_DSIPLL_CLAMPI),
				PRCM_MMIP_LS_CLAMP_SET);
			val &= ~PRCM_PLLDSI_ENABLE_PRCM_PLLDSI_ENABLE;
			writel(val, PRCM_PLLDSI_ENABLE);
			r = -EAGAIN;
		}
	} else {
		writel(PRCM_APE_RESETN_DSIPLL_RESETN, PRCM_APE_RESETN_CLR);
		/* Wait an extra 10 us after the pll is disabled */
		udelay(10);
	}
	return r;
}

static int request_dsiclk(u8 n, bool enable)
{
	u32 val;

	val = readl(PRCM_DSI_PLLOUT_SEL);
	val &= ~dsiclk[n].divsel_mask;
	val |= ((enable ? dsiclk[n].divsel : PRCM_DSI_PLLOUT_SEL_OFF) <<
		dsiclk[n].divsel_shift);
	writel(val, PRCM_DSI_PLLOUT_SEL);
	return 0;
}

static int request_dsiescclk(u8 n, bool enable)
{
	u32 val;

	val = readl(PRCM_DSITVCLK_DIV);
	enable ? (val |= dsiescclk[n].en) : (val &= ~dsiescclk[n].en);
	writel(val, PRCM_DSITVCLK_DIV);
	return 0;
}

/*
 * This is a workaround for prcmu firmware bug. While settings EPODs, prcmu
 * firmware enables/disables B2R2 clock. However, this is done without taking
 * PRCM_SEM. Concurrent accesses from APE side to change state of EPODs and
 * B2R2 clock can result in B2R2 clock in unwanted state.
 */
static int request_b2r2_clock(u8 clock, bool enable)
{
	int ret;

	mutex_lock(&mb2_transfer.lock);

	ret = request_clock(clock, enable);

	mutex_unlock(&mb2_transfer.lock);

	return ret;
}

/**
 * db8500_prcmu_request_clock() - Request for a clock to be enabled or disabled.
 * @clock:      The clock for which the request is made.
 * @enable:     Whether the clock should be enabled (true) or disabled (false).
 *
 * This function should only be used by the clock implementation.
 * Do not use it from any other place!
 */
int db8500_prcmu_request_clock(u8 clock, bool enable)
{
	trace_u8500_request_clock(clock, enable);
	if (clock == PRCMU_SGACLK)
		return request_sga_clock(clock, enable);
	else if (clock == PRCMU_B2R2CLK)
		return request_b2r2_clock(clock, enable);
	else if (clock < PRCMU_NUM_REG_CLOCKS)
		return request_clock(clock, enable);
	else if (clock == PRCMU_TIMCLK)
		return request_timclk(enable);
	else if ((clock == PRCMU_DSI0CLK) || (clock == PRCMU_DSI1CLK))
		return request_dsiclk((clock - PRCMU_DSI0CLK), enable);
	else if ((PRCMU_DSI0ESCCLK <= clock) && (clock <= PRCMU_DSI2ESCCLK))
		return request_dsiescclk((clock - PRCMU_DSI0ESCCLK), enable);
	else if (clock == PRCMU_PLLDSI)
		return request_plldsi(enable);
	else if (clock == PRCMU_SYSCLK)
		return request_sysclk(enable);
	else if ((clock == PRCMU_PLLSOC0) || (clock == PRCMU_PLLSOC1))
		return request_pll(clock, enable);
	else
		return -EINVAL;
}

#ifdef CONFIG_SAMSUNG_PANIC_DISPLAY_DEVICES
static int request_panic_pll(u8 clock, bool enable)
{
	int r = 0;

	if (clock == PRCMU_PLLSOC0)
		clock = (enable ? PLL_SOC0_ON : PLL_SOC0_OFF);
	else if (clock == PRCMU_PLLSOC1)
		clock = (enable ? PLL_SOC1_ON : PLL_SOC1_OFF);
	else
		return -EINVAL;

	while (readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(1))
		cpu_relax();

	writeb(MB1H_PLL_ON_OFF, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB1));
	writeb(clock, (tcdm_base + PRCM_REQ_MB1_PLL_ON_OFF));

	writel(MBOX_BIT(1), PRCM_MBOX_CPU_SET);
	wait_for_completion(&mb1_transfer.work);

	if (mb1_transfer.ack.header != MB1H_PLL_ON_OFF)
		r = -EIO;

	return r;
}

static int request_panic_sysclk(bool enable)
{
	int r;
	/* unsigned long flags; */

	r = 0;

	while (readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(3))
		cpu_relax();

	writeb((enable ? ON : OFF), (tcdm_base + PRCM_REQ_MB3_SYSCLK_MGT));

	writeb(MB3H_SYSCLK, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB3));
	writel(MBOX_BIT(3), PRCM_MBOX_CPU_SET);

	/*
	 * The firmware only sends an ACK if we want to enable the
	 * SysClk, and it succeeds.
	 */
	if (enable && !wait_for_completion_timeout(&mb3_transfer.sysclk_work,
			msecs_to_jiffies(20000))) {
		pr_err("prcmu: %s timed out (20 s) waiting for a reply.\n",
			__func__);
		r = -EIO;
		db8500_prcmu_debug_dump(true);
	}

	return r;
}

static int request_panic_clock(u8 clock, bool enable)
{
	u32 val;
	/* unsigned long flags; */

	/* Grab the HW semaphore. */
	while ((readl(PRCM_SEM) & PRCM_SEM_PRCM_SEM) != 0)
		cpu_relax();

	val = readl(clk_mgt[clock].reg);
	if (enable) {
		val |= (PRCM_CLK_MGT_CLKEN | clk_mgt[clock].pllsw);
	} else {
		clk_mgt[clock].pllsw = (val & PRCM_CLK_MGT_CLKPLLSW_MASK);
		val &= ~(PRCM_CLK_MGT_CLKEN | PRCM_CLK_MGT_CLKPLLSW_MASK);
	}
	writel(val, clk_mgt[clock].reg);

	/* Release the HW semaphore. */
	writel(0, PRCM_SEM);
	return 0;
}

static int request_panic_sga_clock(u8 clock, bool enable)
{
	u32 val;
	int ret;

	if (enable) {
		val = readl(PRCM_CGATING_BYPASS);
		writel(val | PRCM_CGATING_BYPASS_ICN2, PRCM_CGATING_BYPASS);
	}

	ret = request_panic_clock(clock, enable);

	if (!ret && !enable) {
		val = readl(PRCM_CGATING_BYPASS);
		writel(val & ~PRCM_CGATING_BYPASS_ICN2, PRCM_CGATING_BYPASS);
	}

	return ret;
}

/**
 * prcmu_request_clock() - Request for a clock to be enabled or disabled.
 * @clock:      The clock for which the request is made.
 * @enable:     Whether the clock should be enabled (true) or disabled (false).
 *
 * This function should only be used by the clock implementation.
 * Do not use it from any other place!
 */
int prcmu_panic_request_clock(u8 clock, bool enable)
{
	if (clock == PRCMU_SGACLK)
		return request_panic_sga_clock(clock, enable);
	else if (clock == PRCMU_B2R2CLK)
		return request_panic_clock(clock, enable);
	else if (clock < PRCMU_NUM_REG_CLOCKS)
		return request_panic_clock(clock, enable);
	else if (clock == PRCMU_TIMCLK)
		return request_timclk(enable);
	else if ((clock == PRCMU_DSI0CLK) || (clock == PRCMU_DSI1CLK))
		return request_dsiclk((clock - PRCMU_DSI0CLK), enable);
	else if ((PRCMU_DSI0ESCCLK <= clock) && (clock <= PRCMU_DSI2ESCCLK))
		return request_dsiescclk((clock - PRCMU_DSI0ESCCLK), enable);
	else if (clock == PRCMU_PLLDSI)
		return request_plldsi(enable);
	else if (clock == PRCMU_SYSCLK)
		return request_panic_sysclk(enable);
	else if ((clock == PRCMU_PLLSOC0) || (clock == PRCMU_PLLSOC1))
		return request_panic_pll(clock, enable);
	else
		return -EINVAL;
}
#endif


static unsigned long pll_rate(void __iomem *reg, unsigned long src_rate,
	int branch)
{
	u64 rate;
	u32 val;
	u32 d;
	u32 div = 1;

	val = readl(reg);

	rate = src_rate;
	rate *= ((val & PRCM_PLL_FREQ_D_MASK) >> PRCM_PLL_FREQ_D_SHIFT);

	d = ((val & PRCM_PLL_FREQ_N_MASK) >> PRCM_PLL_FREQ_N_SHIFT);
	if (d > 1)
		div *= d;

	d = ((val & PRCM_PLL_FREQ_R_MASK) >> PRCM_PLL_FREQ_R_SHIFT);
	if (d > 1)
		div *= d;

	if (val & PRCM_PLL_FREQ_SELDIV2)
		div *= 2;

	if ((branch == PLL_FIX) || ((branch == PLL_DIV) &&
		(val & PRCM_PLL_FREQ_DIV2EN) &&
		((reg == PRCM_PLLSOC0_FREQ) ||
		 (reg == PRCM_PLLARM_FREQ) ||
		 (reg == PRCM_PLLDDR_FREQ))))
		div *= 2;

	(void)do_div(rate, div);

	return (unsigned long)rate;
}

#define ROOT_CLOCK_RATE 38400000

static unsigned long clock_rate(u8 clock)
{
	u32 val;
	u32 pllsw;
	unsigned long rate = ROOT_CLOCK_RATE;

	val = readl(clk_mgt[clock].reg);

	if (val & PRCM_CLK_MGT_CLK38) {
		if (clk_mgt[clock].clk38div && (val & PRCM_CLK_MGT_CLK38DIV))
			rate /= 2;
		return rate;
	}

	val |= clk_mgt[clock].pllsw;
	pllsw = (val & PRCM_CLK_MGT_CLKPLLSW_MASK);

	if (pllsw == PRCM_CLK_MGT_CLKPLLSW_SOC0)
		rate = pll_rate(PRCM_PLLSOC0_FREQ, rate, clk_mgt[clock].branch);
	else if (pllsw == PRCM_CLK_MGT_CLKPLLSW_SOC1)
		rate = pll_rate(PRCM_PLLSOC1_FREQ, rate, clk_mgt[clock].branch);
	else if (pllsw == PRCM_CLK_MGT_CLKPLLSW_DDR)
		rate = pll_rate(PRCM_PLLDDR_FREQ, rate, clk_mgt[clock].branch);
	else
		return 0;

	if ((clock == PRCMU_SGACLK) &&
		(val & PRCM_SGACLK_MGT_SGACLKDIV_BY_2_5_EN)) {
		u64 r = (rate * 10);

		(void)do_div(r, 25);
		return (unsigned long)r;
	}
	val &= PRCM_CLK_MGT_CLKPLLDIV_MASK;
	if (val)
		return rate / val;
	else
		return 0;
}

static unsigned long latest_armss_rate = 0;

static unsigned long armss_rate(void)
{
	return latest_armss_rate;
}

static void compute_armss_rate(void)
{
	u32 r;
	unsigned long rate;

	r = readl(PRCM_ARM_CHGCLKREQ);

	if (r & PRCM_ARM_CHGCLKREQ_PRCM_ARM_CHGCLKREQ) {
		/* External ARMCLKFIX clock */

		rate = pll_rate(PRCM_PLLDDR_FREQ, ROOT_CLOCK_RATE, PLL_FIX);

		/* Check PRCM_ARM_CHGCLKREQ divider */
		if (!(r & PRCM_ARM_CHGCLKREQ_PRCM_ARM_DIVSEL))
			rate /= 2;

		/* Check PRCM_ARMCLKFIX_MGT divider */
		r = readl(PRCM_ARMCLKFIX_MGT);
		r &= PRCM_CLK_MGT_CLKPLLDIV_MASK;
		rate /= r;

	} else { /* ARM PLL */
		rate = pll_rate(PRCM_PLLARM_FREQ, ROOT_CLOCK_RATE, PLL_DIV);
	}

	latest_armss_rate = rate;
}

static unsigned long dsiclk_rate(u8 n)
{
	u32 divsel;
	u32 div = 1;

	divsel = readl(PRCM_DSI_PLLOUT_SEL);
	divsel = ((divsel & dsiclk[n].divsel_mask) >> dsiclk[n].divsel_shift);

	if (divsel == PRCM_DSI_PLLOUT_SEL_OFF)
		divsel = dsiclk[n].divsel;

	switch (divsel) {
	case PRCM_DSI_PLLOUT_SEL_PHI_4:
		div *= 2;
	case PRCM_DSI_PLLOUT_SEL_PHI_2:
		div *= 2;
	case PRCM_DSI_PLLOUT_SEL_PHI:
		return pll_rate(PRCM_PLLDSI_FREQ, clock_rate(PRCMU_HDMICLK),
			PLL_RAW) / div;
	default:
		return 0;
	}
}

static unsigned long dsiescclk_rate(u8 n)
{
	u32 div;

	div = readl(PRCM_DSITVCLK_DIV);
	div = ((div & dsiescclk[n].div_mask) >> (dsiescclk[n].div_shift));
	return clock_rate(PRCMU_TVCLK) / max((u32)1, div);
}

static unsigned long db8500_prcmu_clock_rate(u8 clock)
{
	if (clock < PRCMU_NUM_REG_CLOCKS)
		return clock_rate(clock);
	else if (clock == PRCMU_TIMCLK)
		return prcmu_is_ulppll_disabled()?
			TIMCLK_CLOCK_RATE_ULPPLL_OFF :
			TIMCLK_CLOCK_RATE_ULPPLL_ON;
	else if (clock == PRCMU_SYSCLK)
		return ROOT_CLOCK_RATE;
	else if (clock == PRCMU_PLLSOC0)
		return pll_rate(PRCM_PLLSOC0_FREQ, ROOT_CLOCK_RATE, PLL_RAW);
	else if (clock == PRCMU_PLLSOC1)
		return pll_rate(PRCM_PLLSOC1_FREQ, ROOT_CLOCK_RATE, PLL_RAW);
	else if (clock == PRCMU_ARMSS)
		return armss_rate();
	else if (clock == PRCMU_ARMCLK)
		return arm_get_rate();
	else if (clock == PRCMU_PLLARM)
		return pll_rate(PRCM_PLLARM_FREQ, ROOT_CLOCK_RATE, PLL_RAW);
	else if (clock == PRCMU_PLLDDR)
		return pll_rate(PRCM_PLLDDR_FREQ, ROOT_CLOCK_RATE, PLL_RAW);
	else if (clock == PRCMU_PLLDSI)
		return pll_rate(PRCM_PLLDSI_FREQ, clock_rate(PRCMU_HDMICLK),
			PLL_RAW);
	else if ((clock == PRCMU_DSI0CLK) || (clock == PRCMU_DSI1CLK))
		return dsiclk_rate(clock - PRCMU_DSI0CLK);
	else if ((PRCMU_DSI0ESCCLK <= clock) && (clock <= PRCMU_DSI2ESCCLK))
		return dsiescclk_rate(clock - PRCMU_DSI0ESCCLK);
	else
		BUG_ON(0);
		return 0;
}

static unsigned long clock_source_rate(u32 clk_mgt_val, int branch)
{
	if (clk_mgt_val & PRCM_CLK_MGT_CLK38)
		return ROOT_CLOCK_RATE;
	clk_mgt_val &= PRCM_CLK_MGT_CLKPLLSW_MASK;
	if (clk_mgt_val == PRCM_CLK_MGT_CLKPLLSW_SOC0)
		return pll_rate(PRCM_PLLSOC0_FREQ, ROOT_CLOCK_RATE, branch);
	else if (clk_mgt_val == PRCM_CLK_MGT_CLKPLLSW_SOC1)
		return pll_rate(PRCM_PLLSOC1_FREQ, ROOT_CLOCK_RATE, branch);
	else if (clk_mgt_val == PRCM_CLK_MGT_CLKPLLSW_DDR)
		return pll_rate(PRCM_PLLDDR_FREQ, ROOT_CLOCK_RATE, branch);
	else
		return 0;
}

static u32 clock_divider(unsigned long src_rate, unsigned long rate)
{
	u32 div;

	div = (src_rate / rate);
	if (div == 0)
		return 1;
	if (rate < (src_rate / div))
		div++;
	return div;
}

static long round_clock_rate(u8 clock, unsigned long rate)
{
	u32 val;
	u32 div;
	unsigned long src_rate;
	long rounded_rate;

	val = readl(clk_mgt[clock].reg);
	src_rate = clock_source_rate((val | clk_mgt[clock].pllsw),
		clk_mgt[clock].branch);
	div = clock_divider(src_rate, rate);
	if (val & PRCM_CLK_MGT_CLK38) {
		if (clk_mgt[clock].clk38div) {
			if (div > 2)
				div = 2;
		} else {
			div = 1;
		}
	} else if ((clock == PRCMU_SGACLK) && (div == 3)) {
		u64 r = (src_rate * 10);

		(void)do_div(r, 25);
		if (r <= rate)
			return (unsigned long)r;
	}
	rounded_rate = (src_rate / min(div, (u32)31));

	return rounded_rate;
}

#define MIN_PLL_VCO_RATE 600000000ULL
#define MAX_PLL_VCO_RATE 1680640000ULL

static long round_plldsi_rate(unsigned long rate)
{
	long rounded_rate = 0;
	unsigned long src_rate;
	unsigned long rem;
	u32 r;

	src_rate = clock_rate(PRCMU_HDMICLK);
	rem = rate;

	for (r = 7; (rem > 0) && (r > 0); r--) {
		u64 d;

		d = (r * rate);
		(void)do_div(d, src_rate);
		if (d < 6)
			d = 6;
		else if (d > 255)
			d = 255;
		d *= src_rate;
		if (((2 * d) < (r * MIN_PLL_VCO_RATE)) ||
			((r * MAX_PLL_VCO_RATE) < (2 * d)))
			continue;
		(void)do_div(d, r);
		if (rate < d) {
			if (rounded_rate == 0)
				rounded_rate = (long)d;
			break;
		}
		if ((rate - d) < rem) {
			rem = (rate - d);
			rounded_rate = (long)d;
		}
	}
	return rounded_rate;
}

static long round_dsiclk_rate(unsigned long rate)
{
	u32 div;
	unsigned long src_rate;
	long rounded_rate;

	src_rate = pll_rate(PRCM_PLLDSI_FREQ, clock_rate(PRCMU_HDMICLK),
		PLL_RAW);
	div = clock_divider(src_rate, rate);
	rounded_rate = (src_rate / ((div > 2) ? 4 : div));

	return rounded_rate;
}

static long round_dsiescclk_rate(unsigned long rate)
{
	u32 div;
	unsigned long src_rate;
	long rounded_rate;

	src_rate = clock_rate(PRCMU_TVCLK);
	div = clock_divider(src_rate, rate);
	rounded_rate = (src_rate / min(div, (u32)255));

	return rounded_rate;
}

static long db8500_prcmu_round_clock_rate(u8 clock, unsigned long rate)
{
	if (clock < PRCMU_NUM_REG_CLOCKS)
		return round_clock_rate(clock, rate);
	else if (clock == PRCMU_PLLDSI)
		return round_plldsi_rate(rate);
	else if ((clock == PRCMU_DSI0CLK) || (clock == PRCMU_DSI1CLK))
		return round_dsiclk_rate(rate);
	else if ((PRCMU_DSI0ESCCLK <= clock) && (clock <= PRCMU_DSI2ESCCLK))
		return round_dsiescclk_rate(rate);
	else
		return (long)db8500_prcmu_clock_rate(clock);
}

static void set_clock_rate(u8 clock, unsigned long rate)
{
	u32 val;
	u32 div;
	unsigned long src_rate;
	unsigned long flags;

	spin_lock_irqsave(&clk_mgt_lock, flags);

	/* Grab the HW semaphore. */
	while ((readl(PRCM_SEM) & PRCM_SEM_PRCM_SEM) != 0)
		cpu_relax();

	val = readl(clk_mgt[clock].reg);
	src_rate = clock_source_rate((val | clk_mgt[clock].pllsw),
		clk_mgt[clock].branch);
	div = clock_divider(src_rate, rate);
	if (val & PRCM_CLK_MGT_CLK38) {
		if (clk_mgt[clock].clk38div) {
			if (div > 1)
				val |= PRCM_CLK_MGT_CLK38DIV;
			else
				val &= ~PRCM_CLK_MGT_CLK38DIV;
		}
	} else if (clock == PRCMU_SGACLK) {
		val &= ~(PRCM_CLK_MGT_CLKPLLDIV_MASK |
			PRCM_SGACLK_MGT_SGACLKDIV_BY_2_5_EN);
		if (div == 3) {
			u64 r = (src_rate * 10);

			(void)do_div(r, 25);
			if (r <= rate) {
				val |= PRCM_SGACLK_MGT_SGACLKDIV_BY_2_5_EN;
				div = 0;
			}
		}
		val |= min(div, (u32)31);
	} else {
		val &= ~PRCM_CLK_MGT_CLKPLLDIV_MASK;
		val |= min(div, (u32)31);
	}
	writel(val, clk_mgt[clock].reg);

	/* Release the HW semaphore. */
	writel(0, PRCM_SEM);

	spin_unlock_irqrestore(&clk_mgt_lock, flags);
}

static int set_plldsi_rate(unsigned long rate)
{
	unsigned long src_rate;
	unsigned long rem;
	u32 pll_freq = 0;
	u32 r;

	src_rate = clock_rate(PRCMU_HDMICLK);
	rem = rate;

	for (r = 7; (rem > 0) && (r > 0); r--) {
		u64 d;
		u64 hwrate;

		d = (r * rate);
		(void)do_div(d, src_rate);
		if (d < 6)
			d = 6;
		else if (d > 255)
			d = 255;
		hwrate = (d * src_rate);
		if (((2 * hwrate) < (r * MIN_PLL_VCO_RATE)) ||
			((r * MAX_PLL_VCO_RATE) < (2 * hwrate)))
			continue;
		(void)do_div(hwrate, r);
		if (rate < hwrate) {
			if (pll_freq == 0)
				pll_freq = (((u32)d << PRCM_PLL_FREQ_D_SHIFT) |
					(r << PRCM_PLL_FREQ_R_SHIFT));
			break;
		}
		if ((rate - hwrate) < rem) {
			rem = (rate - hwrate);
			pll_freq = (((u32)d << PRCM_PLL_FREQ_D_SHIFT) |
				(r << PRCM_PLL_FREQ_R_SHIFT));
		}
	}
	if (pll_freq == 0)
		return -EINVAL;

	pll_freq |= (1 << PRCM_PLL_FREQ_N_SHIFT);
	writel(pll_freq, PRCM_PLLDSI_FREQ);

	return 0;
}

static void set_dsiclk_rate(u8 n, unsigned long rate)
{
	u32 val;
	u32 div;

	div = clock_divider(pll_rate(PRCM_PLLDSI_FREQ,
			clock_rate(PRCMU_HDMICLK), PLL_RAW), rate);

	dsiclk[n].divsel = (div == 1) ? PRCM_DSI_PLLOUT_SEL_PHI :
			   (div == 2) ? PRCM_DSI_PLLOUT_SEL_PHI_2 :
			   /* else */	PRCM_DSI_PLLOUT_SEL_PHI_4;

	val = readl(PRCM_DSI_PLLOUT_SEL);
	val &= ~dsiclk[n].divsel_mask;
	val |= (dsiclk[n].divsel << dsiclk[n].divsel_shift);
	writel(val, PRCM_DSI_PLLOUT_SEL);
}

static void set_dsiescclk_rate(u8 n, unsigned long rate)
{
	u32 val;
	u32 div;

	div = clock_divider(clock_rate(PRCMU_TVCLK), rate);
	val = readl(PRCM_DSITVCLK_DIV);
	val &= ~dsiescclk[n].div_mask;
	val |= (min(div, (u32)255) << dsiescclk[n].div_shift);
	writel(val, PRCM_DSITVCLK_DIV);
}

static int db8500_prcmu_set_clock_rate(u8 clock, unsigned long rate)
{
	if (clock < PRCMU_NUM_REG_CLOCKS)
		set_clock_rate(clock, rate);
	else if (clock == PRCMU_PLLDSI)
		return set_plldsi_rate(rate);
	else if (clock == PRCMU_ARMCLK)
		return arm_set_rate(rate);
	else if ((clock == PRCMU_DSI0CLK) || (clock == PRCMU_DSI1CLK))
		set_dsiclk_rate((clock - PRCMU_DSI0CLK), rate);
	else if ((PRCMU_DSI0ESCCLK <= clock) && (clock <= PRCMU_DSI2ESCCLK))
		set_dsiescclk_rate((clock - PRCMU_DSI0ESCCLK), rate);
	trace_u8500_set_clock_rate(clock, rate);
	return 0;
}

static int db8500_prcmu_config_esram0_deep_sleep(u8 state)
{
	if ((state > ESRAM0_DEEP_SLEEP_STATE_RET) ||
	    (state < ESRAM0_DEEP_SLEEP_STATE_OFF))
		return -EINVAL;

	mutex_lock(&mb4_transfer.lock);

	while (readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(4))
		cpu_relax();

	writeb(MB4H_MEM_ST, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB4));
	writeb(((DDR_PWR_STATE_OFFHIGHLAT << 4) | DDR_PWR_STATE_ON),
	       (tcdm_base + PRCM_REQ_MB4_DDR_ST_AP_SLEEP_IDLE));
	writeb(DDR_PWR_STATE_ON,
	       (tcdm_base + PRCM_REQ_MB4_DDR_ST_AP_DEEP_IDLE));
	writeb(state, (tcdm_base + PRCM_REQ_MB4_ESRAM0_ST));

	log_this(190, "state", state, NULL, 0);
	writel(MBOX_BIT(4), PRCM_MBOX_CPU_SET);
	wait_for_completion(&mb4_transfer.work);

	mutex_unlock(&mb4_transfer.lock);

	return 0;
}

static int prcmu_a9wdog(u8 cmd, u8 d0, u8 d1, u8 d2, u8 d3)
{
	trace_u8500_a9_wdog(cmd, d0, d1, d2, d3);

	mutex_lock(&mb4_transfer.lock);

	while (readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(4))
		cpu_relax();

	writeb(d0, (tcdm_base + PRCM_REQ_MB4_A9WDOG_0));
	writeb(d1, (tcdm_base + PRCM_REQ_MB4_A9WDOG_1));
	writeb(d2, (tcdm_base + PRCM_REQ_MB4_A9WDOG_2));
	writeb(d3, (tcdm_base + PRCM_REQ_MB4_A9WDOG_3));

	writeb(cmd, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB4));

	log_this(247, "cmd", cmd, "value", d3 << 24 | d2 << 16 | d1 << 8 | d0);
	writel(MBOX_BIT(4), PRCM_MBOX_CPU_SET);
	wait_for_completion(&mb4_transfer.work);

	mutex_unlock(&mb4_transfer.lock);

	return 0;

}

static int config_a9wdog(u8 num, bool sleep_auto_off)
{
	BUG_ON(num == 0 || num > 0xf);
	return prcmu_a9wdog(MB4H_A9WDOG_CONF, num, 0, 0,
			    sleep_auto_off ? A9WDOG_AUTO_OFF_EN :
			    A9WDOG_AUTO_OFF_DIS);
}

static int enable_a9wdog(u8 id)
{
	return prcmu_a9wdog(MB4H_A9WDOG_EN, id, 0, 0, 0);
}

static int disable_a9wdog(u8 id)
{
	return prcmu_a9wdog(MB4H_A9WDOG_DIS, id, 0, 0, 0);
}

static int kick_a9wdog(u8 id)
{
	return prcmu_a9wdog(MB4H_A9WDOG_KICK, id, 0, 0, 0);
}

/*
 * timeout is 28 bit, in ms.
 */
static int load_a9wdog(u8 id, u32 timeout)
{
	return prcmu_a9wdog(MB4H_A9WDOG_LOAD,
			    (id & A9WDOG_ID_MASK) |
			    /*
			     * Put the lowest 28 bits of timeout at
			     * offset 4. Four first bits are used for id.
			     */
			    (u8)((timeout << 4) & 0xf0),
			    (u8)((timeout >> 4) & 0xff),
			    (u8)((timeout >> 12) & 0xff),
			    (u8)((timeout >> 20) & 0xff));
}

/**
 * db8500_prcmu_abb_read() - Read register value(s) from the ABB.
 * @slave:	The I2C slave address.
 * @reg:	The (start) register address.
 * @value:	The read out value(s).
 * @size:	The number of registers to read.
 *
 * Reads register value(s) from the ABB.
 * @size has to be 1 for the current firmware version.
 */
static int db8500_prcmu_abb_read(u8 slave, u8 reg, u8 *value, u8 size)
{
	int r;

	if (size != 1)
		return -EINVAL;

	mutex_lock(&mb5_transfer.lock);

	while (readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(5))
		cpu_relax();

	writeb(0, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB5));
	writeb(PRCMU_I2C_READ(slave), (tcdm_base + PRCM_REQ_MB5_I2C_SLAVE_OP));
	writeb(PRCMU_I2C_STOP_EN, (tcdm_base + PRCM_REQ_MB5_I2C_HW_BITS));
	writeb(reg, (tcdm_base + PRCM_REQ_MB5_I2C_REG));
	writeb(0, (tcdm_base + PRCM_REQ_MB5_I2C_VAL));

	writel(MBOX_BIT(5), PRCM_MBOX_CPU_SET);

	if (!wait_for_completion_timeout(&mb5_transfer.work,
				msecs_to_jiffies(20000))) {
		pr_err("prcmu: %s timed out (20 s) waiting for a reply.\n",
			__func__);
		r = -EIO;
		db8500_prcmu_debug_dump(false);
	} else {
		r = ((mb5_transfer.ack.status == I2C_RD_OK) ? 0 : -EIO);
	}

	if (!r) {
		*value = mb5_transfer.ack.value;
		if (!(*value == 0 && slave == 0x0E && (reg >= 0x20 && reg < 0x40))) {
			log_this(230, "reg", slave << 8 | reg, "value", *value);
			trace_printk("%02X@%04Xh\n", *value, (slave << 8 | reg));
		}
	} else {
		log_this(230, "reg", slave << 8 | reg, "error", mb5_transfer.ack.status);
		trace_printk("error(%02X)@%04Xh\n", *value, mb5_transfer.ack.status);
	}

	mutex_unlock(&mb5_transfer.lock);

	return r;
}

#define POLLING_TIMEOUT 1000 /* Becomes ~0.1s timeout */
#define TRANSFER_TIMEOUT 30000 /* Becomes ~3s timeout */

/* Only to be used at panic! */
static int db8500_prcmu_abb_read_no_irq(u8 slave, u8 reg, u8 *value, u8 size)
{
	u32 s;
	int r;
	int count = 0;

	if (size != 1)
		return -EINVAL;

	while (readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(5)) {
		udelay(100);
		cpu_relax();
		count++;
		if (count > POLLING_TIMEOUT) {
			pr_err("%s: Error: mailbox 5 busy\n", __func__);
			return -EINVAL;
		}
	}

	writeb(0, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB5));
	writeb(PRCMU_I2C_READ(slave), (tcdm_base + PRCM_REQ_MB5_I2C_SLAVE_OP));
	writeb(PRCMU_I2C_STOP_EN, (tcdm_base + PRCM_REQ_MB5_I2C_HW_BITS));
	writeb(reg, (tcdm_base + PRCM_REQ_MB5_I2C_REG));
	writeb(0, (tcdm_base + PRCM_REQ_MB5_I2C_VAL));

	writel(MBOX_BIT(5), PRCM_MBOX_CPU_SET);

	count = 0;
	do {
		s = (readl(PRCM_ARM_IT1_VAL) & MBOX_BIT(5));
		udelay(100);
		cpu_relax();
		count++;
		if (count > TRANSFER_TIMEOUT) {
			pr_err("%s: Error: i2c transfer timed out\n", __func__);
			return -EINVAL;
		}
	} while(!s);

	mb5_transfer.ack.status = readb(tcdm_base + PRCM_ACK_MB5_I2C_STATUS);
	mb5_transfer.ack.value = readb(tcdm_base + PRCM_ACK_MB5_I2C_VAL);

	writel(MBOX_BIT(5), PRCM_ARM_IT1_CLR);

	r = ((mb5_transfer.ack.status == I2C_RD_OK) ? 0 : -EIO);

	if (!r)
		*value = mb5_transfer.ack.value;

	return r;
}

/* Only to be used before restart! */
static int db8500_prcmu_abb_write_no_irq(u8 slave, u8 reg, u8 *value, u8 size)
{
	int r;
	int count = 0;

	if (size != 1)
		return -EINVAL;

	WARN_ON(!irqs_disabled());

	while (readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(5)) {
		udelay(100);
		cpu_relax();
		count++;
		if (count > POLLING_TIMEOUT) {
			pr_err("%s: Error: mailbox 5 busy\n", __func__);
			return -EINVAL;
		}
	}

	writeb(0, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB5));
	writeb(PRCMU_I2C_WRITE(slave), (tcdm_base + PRCM_REQ_MB5_I2C_SLAVE_OP));
	writeb(PRCMU_I2C_STOP_EN, (tcdm_base + PRCM_REQ_MB5_I2C_HW_BITS));
	writeb(reg, (tcdm_base + PRCM_REQ_MB5_I2C_REG));
	writeb(*value, (tcdm_base + PRCM_REQ_MB5_I2C_VAL));

	writel(MBOX_BIT(5), PRCM_MBOX_CPU_SET);

	count = 0;
	while (!(readl(PRCM_ARM_IT1_VAL) & MBOX_BIT(5))) {
		udelay(100);
		cpu_relax();
		count++;
		if (count > TRANSFER_TIMEOUT) {
			pr_err("%s: Error: i2c transfer timed out\n", __func__);
			return -EINVAL;
		}
	}

	mb5_transfer.ack.status = readb(tcdm_base + PRCM_ACK_MB5_I2C_STATUS);
	mb5_transfer.ack.value = readb(tcdm_base + PRCM_ACK_MB5_I2C_VAL);

	writel(MBOX_BIT(5), PRCM_ARM_IT1_CLR);

	r = ((mb5_transfer.ack.status == I2C_WR_OK) ? 0 : -EIO);

	return r;
}

/**
 * db8500_prcmu_abb_write_masked() - Write masked register value(s) to the ABB.
 * @slave:	The I2C slave address.
 * @reg:	The (start) register address.
 * @value:	The value(s) to write.
 * @mask:	The mask(s) to use.
 * @size:	The number of registers to write.
 *
 * Writes masked register value(s) to the ABB.
 * For each @value, only the bits set to 1 in the corresponding @mask
 * will be written. The other bits are not changed.
 * @size has to be 1 for the current firmware version.
 */
static int db8500_prcmu_abb_write_masked(u8 slave, u8 reg, u8 *value, u8 *mask,
		u8 size)
{
	int r;

	if (size != 1)
		return -EINVAL;

	mutex_lock(&mb5_transfer.lock);

	while (readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(5))
		cpu_relax();

	writeb(~*mask, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB5));
	writeb(PRCMU_I2C_WRITE(slave), (tcdm_base + PRCM_REQ_MB5_I2C_SLAVE_OP));
	writeb(PRCMU_I2C_STOP_EN, (tcdm_base + PRCM_REQ_MB5_I2C_HW_BITS));
	writeb(reg, (tcdm_base + PRCM_REQ_MB5_I2C_REG));
	writeb(*value, (tcdm_base + PRCM_REQ_MB5_I2C_VAL));

	writel(MBOX_BIT(5), PRCM_MBOX_CPU_SET);

	if (!wait_for_completion_timeout(&mb5_transfer.work,
				msecs_to_jiffies(20000))) {
		pr_err("prcmu: %s timed out (20 s) waiting for a reply.\n",
			__func__);
		r = -EIO;
		db8500_prcmu_debug_dump(false);
	} else {
		r = ((mb5_transfer.ack.status == I2C_WR_OK) ? 0 : -EIO);
	}

	if (!r) {
		log_this(240, "reg", slave << 8 | reg, "mask|write", *mask << 16 | *value);
		trace_printk("(%02X&%02X)@%04Xh\n", *value, *mask, (slave << 8 | reg));
	} else {
		log_this(240, "reg", slave << 8 | reg, "error", mb5_transfer.ack.status);
		trace_printk("error(%02X)@%04Xh\n", mb5_transfer.ack.status, (slave << 8 | reg));
	}

	mutex_unlock(&mb5_transfer.lock);

	return r;
}

struct log_entry {
	u64 cpu_clk;
	u32 extra1;
	u32 extra2;
	u8 pc;
	char* a;
	char* b;
};

/* log on non-cacheable buffer */
#ifdef CONFIG_SAMSUNG_LOG_BUF
#include <mach/board-sec-ux500.h>
static DEFINE_SPINLOCK(log_lock);
#define A_LOG_SIZE (6398)
static int log_idx = -1;
static int log_active = 1;
static struct log_entry *a_log;

void* log_buf_prcmu;
EXPORT_SYMBOL(log_buf_prcmu);
const int log_buf_prcmu_entry_size = sizeof(struct log_entry);
EXPORT_SYMBOL(log_buf_prcmu_entry_size);
const int log_buf_prcmu_entry_count = A_LOG_SIZE; 
EXPORT_SYMBOL(log_buf_prcmu_entry_count);
#endif

void log_stop(void)
{
#ifdef CONFIG_SAMSUNG_LOG_BUF
        log_active = 0;
#endif
}
EXPORT_SYMBOL(log_stop);

void log_this(u8 pc, char* a, u32 extra1, char* b, u32 extra2)
{
#ifdef CONFIG_SAMSUNG_LOG_BUF
	u64 cpu_clk = cpu_clock(0);
	unsigned long flags;

	if (!log_active)
		return;

	spin_lock_irqsave(&log_lock, flags);

	if (a_log) {
		log_idx++;

		if ((unsigned int)log_idx >= A_LOG_SIZE)
			log_idx = 0;

		a_log[log_idx].cpu_clk = cpu_clk;
		a_log[log_idx].pc = pc;
		a_log[log_idx].a = a;
		a_log[log_idx].b = b;
		a_log[log_idx].extra1 = extra1;
		a_log[log_idx].extra2 = extra2;

	} else if (log_buf_prcmu) {
		a_log = (struct log_entry *)log_buf_prcmu;
	}

	spin_unlock_irqrestore(&log_lock, flags);
#endif
}
EXPORT_SYMBOL(log_this);

#ifdef CONFIG_SAMSUNG_PANIC_DISPLAY_DEVICES
/**
 * prcmu_panic_abb_read() - Read register value(s) from the ABB under Kernel Panic.
 * @slave:	The I2C slave address.
 * @reg:	The (start) register address.
 * @value:	The read out value(s).
 * @size:	The number of registers to read.
 *
 * Reads register value(s) from the ABB.
 * @size has to be 1 for the current firmware version.
 */
int prcmu_panic_abb_read(u8 slave, u8 reg, u8 *value, u8 size)
{
	int r;
	u32 timeout = PRCMU_I2C_TIMEOUT;
	u8 status;

	if (size != 1)
		return -EINVAL;

	if (readl(PRCM_ARM_IT1_VAL) & MBOX_BIT(5)) {
		/* clear mailbox 5 ack irq */
		writel(MBOX_BIT(5), PRCM_ARM_IT1_CLR);
	}

	while ((readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(5)) && timeout--)
		cpu_relax();

	if ( timeout == 0 ){
		pr_emerg("%s: timed out waiting for MBOX5 to be free.\n",__func__);
		return -EIO;
	}

	writeb(PRCMU_I2C_READ(slave), (tcdm_base + PRCM_REQ_MB5_I2C_SLAVE_OP));
	writeb(PRCMU_I2C_STOP_EN, (tcdm_base + PRCM_REQ_MB5_I2C_HW_BITS));
	writeb(reg, (tcdm_base + PRCM_REQ_MB5_I2C_REG));
	writeb(0, (tcdm_base + PRCM_REQ_MB5_I2C_VAL));

	writel(MBOX_BIT(5), (PRCM_MBOX_CPU_SET));

	timeout = PRCMU_I2C_TIMEOUT;

	while (!(readl(PRCM_ARM_IT1_VAL) & MBOX_BIT(5)) && timeout--)
		cpu_relax();

	if (!timeout) {
		pr_emerg("%s timed out waiting for a reply.\n",__func__);
		return -EIO;
	}

	status = readb(tcdm_base + PRCM_ACK_MB5_I2C_STATUS);
	*value = readb(tcdm_base + PRCM_ACK_MB5_I2C_VAL);

	/* clear mailbox 5 ack irq */
	writel(MBOX_BIT(5), PRCM_ARM_IT1_CLR);

	r = ((status == I2C_RD_OK) ? 0 : -EIO);

	if (r) {
		pr_emerg("%s I2C RD NAK.\n",__func__);
	}

	return r;
}

/**
 * prcmu_panic_abb_write() - Write register value(s) to the ABB under Kernel Panic.
 * @slave:	The I2C slave address.
 * @reg:	The (start) register address.
 * @value:	The value(s) to write.
 * @size:	The number of registers to write.
 *
 * Reads register value(s) from the ABB.
 * @size has to be 1 for the current firmware version.
 */
int prcmu_panic_abb_write(u8 slave, u8 reg, u8 *value, u8 size)
{
	int r;
	u32 timeout = PRCMU_I2C_TIMEOUT;
	u8 status;

	if (size != 1)
		return -EINVAL;

	if (readl(PRCM_ARM_IT1_VAL) & MBOX_BIT(5)) {
		/* clear mailbox 5 ack irq */
		writel(MBOX_BIT(5), PRCM_ARM_IT1_CLR);
	}

	while ((readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(5)) && timeout--)
		cpu_relax();

	if ( timeout == 0 ){
		pr_emerg("%s: timed out waiting for MBOX5 to be free.\n",__func__);
		return -EIO;
	}

	writeb(PRCMU_I2C_WRITE(slave), (tcdm_base + PRCM_REQ_MB5_I2C_SLAVE_OP));
	writeb(PRCMU_I2C_STOP_EN, (tcdm_base + PRCM_REQ_MB5_I2C_HW_BITS));
	writeb(reg, (tcdm_base + PRCM_REQ_MB5_I2C_REG));
	writeb(*value, (tcdm_base + PRCM_REQ_MB5_I2C_VAL));

	writel(MBOX_BIT(5), (PRCM_MBOX_CPU_SET));

	timeout = PRCMU_I2C_TIMEOUT;

	while (!(readl(PRCM_ARM_IT1_VAL) & MBOX_BIT(5)) && timeout--)
		cpu_relax();

	if (!timeout) {
		pr_emerg("%s: timed out waiting for a reply.\n",__func__);
		return -EIO;
	}

	status = readb(tcdm_base + PRCM_ACK_MB5_I2C_STATUS);

	/* clear mailbox 5 ack irq */
	writel(MBOX_BIT(5), PRCM_ARM_IT1_CLR);

	r = ((status == I2C_RD_OK) ? 0 : -EIO);

	return r;
}
#endif

/*
 * db8500_prcmu_abb_write() - Write register value(s) to the ABB.
 * @slave:	The I2C slave address.
 * @reg:	The (start) register address.
 * @value:	The value(s) to write.
 * @size:	The number of registers to write.
 *
 * Writes register value(s) to the ABB.
 * @size has to be 1 for the current firmware version.
 */
static int db8500_prcmu_abb_write(u8 slave, u8 reg, u8 *value, u8 size)
{
	u8 mask = ~0;

	return prcmu_abb_write_masked(slave, reg, value, &mask, size);
}

/**
 * prcmu_ac_wake_req - should be called whenever ARM wants to wakeup Modem
 */
void prcmu_ac_wake_req(void)
{
	bool bitsetretried = false;
	u32 val;
	u32 status;

	mutex_lock(&mb0_transfer.ac_wake_lock);

	val = readl(PRCM_HOSTACCESS_REQ);
	trace_u8500_ac_wake_req(val);
	if (val & PRCM_HOSTACCESS_REQ_HOSTACCESS_REQ)
		goto unlock_and_return;

	if (mb0_transfer.ac_wake_work.done) {
		pr_crit("%s: ac_wake_work was non-zero (%d) on entry!\n",  __func__,
			mb0_transfer.ac_wake_work.done);

		INIT_COMPLETION(mb0_transfer.ac_wake_work);
	}

	atomic_set(&ac_wake_req_state, 1);

retry:
	log_this(20, NULL, 0, NULL, 0);
	/*
	 * Force Modem Wake-up before hostaccess_req ping-pong.
	 * It prevents Modem to enter in Sleep while acking the hostaccess
	 * request. The 31us delay has been calculated by HWI.
	 */
	val |= PRCM_HOSTACCESS_REQ_WAKE_REQ;
	writel(val, PRCM_HOSTACCESS_REQ);

	udelay(31);

	val |= PRCM_HOSTACCESS_REQ_HOSTACCESS_REQ;
	writel(val, PRCM_HOSTACCESS_REQ);
	if (!wait_for_completion_timeout(&mb0_transfer.ac_wake_work,
			msecs_to_jiffies(5000))) {
		if (!bitsetretried &&
		    !(readl_relaxed(PRCM_HOSTACCESS_REQ) & PRCM_HOSTACCESS_REQ_HOSTACCESS_REQ)) {
			bitsetretried = true;
			pr_crit("prcmu: PRCM_HOSTACCESS_REQ bit didn't get set (%#x)?!, try again\n",
				readl_relaxed(PRCM_HOSTACCESS_REQ));
			goto retry;
		}
		db8500_prcmu_debug_dump(true);
		pr_crit("PRCM_HOSTACCESS_REQ %#x bitsetretried %d\n", readl_relaxed(PRCM_HOSTACCESS_REQ), bitsetretried);
		panic("prcmu: %s timed out (5 s) waiting for a reply.\n",
				__func__);
	}

	/* The modem can generate an AC_WAKE_ACK, and then still go to sleep.
	 * As a workaround, we wait, and then check that the modem is indeed
	 * awake (in terms of the value of the PRCM_MOD_AWAKE_STATUS
	 * register, which may not be the whole truth).
	*/

	udelay(400);
	status = readl(PRCM_MOD_AWAKE_STATUS) & BITS(0, 1);
	if (status != (PRCM_MOD_AWAKE_STATUS_PRCM_MOD_AAPD_AWAKE |
			PRCM_MOD_AWAKE_STATUS_PRCM_MOD_COREPD_AWAKE)) {
		pr_err("prcmu: %s received ack, but modem not awake (0x%X).\n",
			__func__, status);
		udelay(1200);

		status = readl(PRCM_MOD_AWAKE_STATUS) & BITS(0, 1);
		if (status != (PRCM_MOD_AWAKE_STATUS_PRCM_MOD_AAPD_AWAKE |
				PRCM_MOD_AWAKE_STATUS_PRCM_MOD_COREPD_AWAKE)) {
			pr_err("prcmu: %s waited, but modem still not awake (0x%X).\n",
			        __func__, status);

			/* Do an ac sleep req first and then retry */
			val &= ~PRCM_HOSTACCESS_REQ_HOSTACCESS_REQ;

			writel(val, (PRCM_HOSTACCESS_REQ));
			if (wait_for_completion_timeout(&mb0_transfer.ac_wake_work,
					msecs_to_jiffies(5000))) {
				goto retry;

			} else {
				db8500_prcmu_debug_dump(true);
				panic("prcmu: %s timed out again (5 s) waiting for a reply.\n",
					__func__);
			}
		} else {
			pr_err("prcmu: %s modem awake after waiting (0x%X)\n",
			       __func__, status);
		}
	}

unlock_and_return:
	mutex_unlock(&mb0_transfer.ac_wake_lock);
}

/**
 * prcmu_ac_sleep_req - called when ARM no longer needs to talk to modem
 */
void prcmu_ac_sleep_req()
{
	u32 val;

	mutex_lock(&mb0_transfer.ac_wake_lock);

	val = readl(PRCM_HOSTACCESS_REQ);
	trace_u8500_ac_sleep_req(val);
	if (!(val & PRCM_HOSTACCESS_REQ_HOSTACCESS_REQ))
		goto unlock_and_return;

	if (mb0_transfer.ac_wake_work.done) {
		pr_crit("%s: ac_wake_work was non-zero (%d) on entry!\n",  __func__,
			mb0_transfer.ac_wake_work.done);

		INIT_COMPLETION(mb0_transfer.ac_wake_work);
	}

	log_this(60, NULL, 0, NULL, 0);
	val &= ~(PRCM_HOSTACCESS_REQ_HOSTACCESS_REQ |
			PRCM_HOSTACCESS_REQ_WAKE_REQ);
	writel(val, PRCM_HOSTACCESS_REQ);

	if (!wait_for_completion_timeout(&mb0_transfer.ac_wake_work,
			msecs_to_jiffies(5000))) {
		db8500_prcmu_debug_dump(true);
		panic("prcmu: %s timed out (5 s) waiting for a reply.\n",
			__func__);
	}

	atomic_set(&ac_wake_req_state, 0);

unlock_and_return:
	mutex_unlock(&mb0_transfer.ac_wake_lock);
}

static bool db8500_prcmu_is_ac_wake_requested(void)
{
	return (atomic_read(&ac_wake_req_state) != 0);
}

/**
 * db8500_prcmu_system_reset - System reset
 *
 * Saves the reset reason code and then sets the APE_SOFTRST register which
 * fires interrupt to fw
 */
#define	PCUT_CTR_AND_STATUS	0x12

static void db8500_prcmu_system_reset(u16 reset_code)
{
	u8 power_cut_reset = 0;
	trace_u8500_system_reset(reset_code);

	/* Disable power-cut feature */
	(void)db8500_prcmu_abb_write_no_irq(AB8500_RTC, PCUT_CTR_AND_STATUS,
						&power_cut_reset, 1);

	writew_relaxed(reset_code, (tcdm_base + PRCM_SW_RST_REASON));
#ifdef CONFIG_SAMSUNG_LOG_BUF
	__write_log(PRCM_APE_SOFTRST);
#endif
	writel_relaxed(1, PRCM_APE_SOFTRST);
	smp_wmb();
}

/**
 * db8500_prcmu_get_reset_code - Retrieve SW reset reason code
 *
 * Retrieves the reset reason code stored by prcmu_system_reset() before
 * last restart.
 */
static u16 db8500_prcmu_get_reset_code(void)
{
	return reset_code_copy;
}

/**
 * db8500_prcmu_get_reset_status - Retrieve reset status
 *
 * Retrieves the value of the reset status register as read at startup.
 */
static u32 db8500_prcmu_get_reset_status(void)
{
	return reset_status_copy;
}

/**
 * db8500_prcmu_reset_modem - ask the PRCMU to reset modem
 */
static void db8500_prcmu_modem_reset(void)
{
	trace_u8500_modem_reset(0);
	mutex_lock(&mb1_transfer.lock);

	while (readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(1))
		cpu_relax();

	writeb(MB1H_RESET_MODEM, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB1));
	log_this(250, NULL, 0, NULL, 0);
	writel(MBOX_BIT(1), PRCM_MBOX_CPU_SET);
	wait_for_completion(&mb1_transfer.work);

	/*
	 * No need to check return from PRCMU as modem should go in reset state
	 * This state is already managed by upper layer
	 */

	mutex_unlock(&mb1_transfer.lock);
}

static void ack_dbb_wakeup(void)
{
	unsigned long flags;

	spin_lock_irqsave(&mb0_transfer.lock, flags);

	while (readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(0))
		cpu_relax();

	writeb(MB0H_READ_WAKEUP_ACK, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB0));
	log_this(30, NULL, 0, NULL, 0);
	writel(MBOX_BIT(0), PRCM_MBOX_CPU_SET);

	spin_unlock_irqrestore(&mb0_transfer.lock, flags);
}

static inline void print_unknown_header_warning(u8 n, u8 header)
{
	pr_warning("prcmu: Unknown message header (%d) in mailbox %d.\n",
		header, n);
}

void uart_wakeunlock(void)
{
        pr_info("UART wakelock is unlocked.\n");
        wake_unlock(&prcmu_uart_wake_lock);
}
EXPORT_SYMBOL(uart_wakeunlock);

static bool read_mailbox_0(void)
{
	bool r;
	u32 ev = 0;
	u32 mask = 0;
	u32 dbb_irqs;
	unsigned int n;
	u8 header;

	header = readb(tcdm_base + PRCM_MBOX_HEADER_ACK_MB0);
	switch (header) {
	case MB0H_WAKEUP_EXE:
	case MB0H_WAKEUP_SLEEP:
		if (readb(tcdm_base + PRCM_ACK_MB0_READ_POINTER) & 1)
			ev = readl(tcdm_base + PRCM_ACK_MB0_WAKEUP_1_8500);
		else
			ev = readl(tcdm_base + PRCM_ACK_MB0_WAKEUP_0_8500);

		if (ev & (WAKEUP_BIT_AC_WAKE_ACK | WAKEUP_BIT_AC_SLEEP_ACK))
			complete(&mb0_transfer.ac_wake_work);
		if (ev & WAKEUP_BIT_SYSCLK_OK)
			complete(&mb3_transfer.sysclk_work);

		log_this(10, "ev", ev, "header", header);
//+ WAKEUP CHECK
		if ((header == MB0H_WAKEUP_SLEEP) && (ev) && (ev != 1)) {
			pr_info("Wakeup Status: 0x%08x\n", ev);
                        if(ev == 0x00800000) {
                                pr_info("Wakeup Status: UART\n");
                                wake_lock_timeout(&prcmu_uart_wake_lock, 20*HZ);
                                ux500_ci_dbg_console();
                        } 			
		}
//- WAKEUP CHECK

		prcmu_debug_register_mbox0_event(ev,
						 (mb0_transfer.req.dbb_irqs |
						  mb0_transfer.req.dbb_wakeups |
						  WAKEUP_BIT_AC_WAKE_ACK |
						  WAKEUP_BIT_AC_SLEEP_ACK |
						  WAKEUP_BIT_SYSCLK_OK));

		mask = mb0_transfer.req.dbb_irqs;
		dbb_irqs = ev & mask;

		for (n = 0; n < NUM_PRCMU_WAKEUPS; n++) {
			if (dbb_irqs & prcmu_irq_bit[n]) {
				log_this(11, "IRQ", IRQ_PRCMU_BASE + n, NULL, 0);
				generic_handle_irq(IRQ_PRCMU_BASE + n);
			}
		}
		r = true;
		break;
	default:
		print_unknown_header_warning(0, header);
		r = false;
		break;
	}
	writel(MBOX_BIT(0), PRCM_ARM_IT1_CLR);
	trace_u8500_irq_mailbox_0(header, ev, mask);

	if (r) {
		unsigned long flags;

		spin_lock_irqsave(&mb0_transfer.lock, flags);

		/* Do not send the ack if MB0 is busy */
		if (!(readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(0))) {
			/* Send ack */
			writeb(MB0H_READ_WAKEUP_ACK, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB0));
			log_this(30, NULL, 0, NULL, 0);
			writel(MBOX_BIT(0), PRCM_MBOX_CPU_SET);
			r = false;
		}
		
		spin_unlock_irqrestore(&mb0_transfer.lock, flags);
	}

	return r;
}

static bool read_mailbox_1(void)
{
	mb1_transfer.ack.header = readb(tcdm_base + PRCM_MBOX_HEADER_REQ_MB1);
	mb1_transfer.ack.arm_opp = readb(tcdm_base +
		PRCM_ACK_MB1_CURRENT_ARM_OPP);
	mb1_transfer.ack.ape_opp = readb(tcdm_base +
		PRCM_ACK_MB1_CURRENT_APE_OPP);
	mb1_transfer.ack.ape_voltage_status = readb(tcdm_base +
		PRCM_ACK_MB1_APE_VOLTAGE_STATUS);
	writel(MBOX_BIT(1), PRCM_ARM_IT1_CLR);
	trace_u8500_irq_mailbox_1(mb1_transfer.ack.header,
		mb1_transfer.ack.arm_opp, mb1_transfer.ack.ape_opp,
		mb1_transfer.ack.ape_voltage_status);
	complete(&mb1_transfer.work);
	return false;
}

static bool read_mailbox_2(void)
{
	mb2_transfer.ack.status = readb(tcdm_base + PRCM_ACK_MB2_DPS_STATUS);
	writel(MBOX_BIT(2), PRCM_ARM_IT1_CLR);
	trace_u8500_irq_mailbox_2(mb2_transfer.ack.status);
	complete(&mb2_transfer.work);
	return false;
}

static void db8500_prcmu_mb3_fw_log_work(struct work_struct *work)
{
	unsigned long flags;
	u8 log_req;

	spin_lock_irqsave(&mb3_transfer.fw_log_lock, flags);
	log_req = mb3_transfer.fw_log_req;
	mb3_transfer.fw_log_req = 0;
	spin_unlock_irqrestore(&mb3_transfer.fw_log_lock, flags);

	if (log_req & MB3_LOG_REQ_PRCMU_REGS)
		prcmu_debug_dump_regs();

	if (log_req & MB3_LOG_REQ_TCDM)
		prcmu_debug_dump_data_mem();

	if (log_req & MB3_LOG_REQ_AB_REGS)
		abx500_dump_all_banks();
}

static bool read_mailbox_3(void)
{
	u8 msg;
	u8 log_req;
	unsigned long flags;

	msg = readb(tcdm_base + PRCM_ACK_MB3_TRACE_MSG);
	printk(KERN_INFO"PRCMU Firmware: msg = 0x%02x\n", msg);

	log_req = readb(tcdm_base + PRCM_ACK_MB3_LOG_REQ);

	writel(MBOX_BIT(3), PRCM_ARM_IT1_CLR);
	trace_u8500_irq_mailbox_3(0);

	if (log_req) {
		spin_lock_irqsave(&mb3_transfer.fw_log_lock, flags);
		mb3_transfer.fw_log_req |= log_req;
		spin_unlock_irqrestore(&mb3_transfer.fw_log_lock, flags);

		schedule_work(&mb3_transfer.fw_log_work);
	}

	return false;
}

static bool read_mailbox_4(void)
{
	u8 header;
	bool do_complete = true;

	header = readb(tcdm_base + PRCM_MBOX_HEADER_REQ_MB4);
	switch (header) {
	case MB4H_MEM_ST:
	case MB4H_HOTDOG:
	case MB4H_HOTMON:
	case MB4H_HOT_PERIOD:
	case MB4H_A9WDOG_CONF:
	case MB4H_A9WDOG_EN:
	case MB4H_A9WDOG_DIS:
	case MB4H_A9WDOG_LOAD:
	case MB4H_A9WDOG_KICK:
		break;
	default:
		print_unknown_header_warning(4, header);
		do_complete = false;
		break;
	}

	writel(MBOX_BIT(4), PRCM_ARM_IT1_CLR);
	trace_u8500_irq_mailbox_4(header);
	if (do_complete)
		complete(&mb4_transfer.work);

	return false;
}

static bool read_mailbox_5(void)
{
	mb5_transfer.ack.status = readb(tcdm_base + PRCM_ACK_MB5_I2C_STATUS);
	mb5_transfer.ack.value = readb(tcdm_base + PRCM_ACK_MB5_I2C_VAL);
	writel(MBOX_BIT(5), PRCM_ARM_IT1_CLR);
	//trace_u8500_irq_mailbox_5(mb5_transfer.ack.status,
	//	mb5_transfer.ack.value);
	complete(&mb5_transfer.work);
	return false;
}

static bool read_mailbox_6(void)
{
	writel(MBOX_BIT(6), PRCM_ARM_IT1_CLR);
	trace_u8500_irq_mailbox_6(0);
	return false;
}

static bool read_mailbox_7(void)
{
	writel(MBOX_BIT(7), PRCM_ARM_IT1_CLR);
	trace_u8500_irq_mailbox_7(0);
	return false;
}

static bool (* const read_mailbox[NUM_MB])(void) = {
	read_mailbox_0,
	read_mailbox_1,
	read_mailbox_2,
	read_mailbox_3,
	read_mailbox_4,
	read_mailbox_5,
	read_mailbox_6,
	read_mailbox_7
};

static irqreturn_t prcmu_irq_handler(int irq, void *data)
{
	u32 bits;
	u8 n;
	irqreturn_t r;

	bits = (readl(PRCM_ARM_IT1_VAL) & ALL_MBOX_BITS);
	if (unlikely(!bits))
		return IRQ_NONE;

	r = IRQ_HANDLED;
	for (n = 0; bits; n++) {
		if (bits & MBOX_BIT(n)) {
			bits -= MBOX_BIT(n);
			if (read_mailbox[n]())
				r = IRQ_WAKE_THREAD;
			prcmu_debug_register_interrupt(n);
		}
	}
	return r;
}

static irqreturn_t prcmu_irq_thread_fn(int irq, void *data)
{
	ack_dbb_wakeup();
	return IRQ_HANDLED;
}

static void prcmu_mask_work(struct work_struct *work)
{
	unsigned long flags;

	spin_lock_irqsave(&mb0_transfer.lock, flags);

	config_wakeups();

	spin_unlock_irqrestore(&mb0_transfer.lock, flags);
}

static void prcmu_irq_mask(struct irq_data *d)
{
	unsigned long flags;

	spin_lock_irqsave(&mb0_transfer.dbb_irqs_lock, flags);

	mb0_transfer.req.dbb_irqs &= ~prcmu_irq_bit[d->irq - IRQ_PRCMU_BASE];

	spin_unlock_irqrestore(&mb0_transfer.dbb_irqs_lock, flags);

	if (d->irq != IRQ_PRCMU_CA_SLEEP)
		schedule_work(&mb0_transfer.mask_work);
}

static void prcmu_irq_unmask(struct irq_data *d)
{
	unsigned long flags;

	spin_lock_irqsave(&mb0_transfer.dbb_irqs_lock, flags);

	mb0_transfer.req.dbb_irqs |= prcmu_irq_bit[d->irq - IRQ_PRCMU_BASE];

	spin_unlock_irqrestore(&mb0_transfer.dbb_irqs_lock, flags);

	if (d->irq != IRQ_PRCMU_CA_SLEEP)
		schedule_work(&mb0_transfer.mask_work);
}

static void noop(struct irq_data *d)
{
}

static struct irq_chip prcmu_irq_chip = {
	.name		= "prcmu",
	.irq_disable	= prcmu_irq_mask,
	.irq_ack	= noop,
	.irq_mask	= prcmu_irq_mask,
	.irq_unmask	= prcmu_irq_unmask,
};

static char *fw_project_name(u8 project)
{
	switch (project) {

	case PRCMU_FW_PROJECT_U8500:
		return "U8500";
	case PRCMU_FW_PROJECT_U8400:
		return "U8400";
	case PRCMU_FW_PROJECT_U9500:
		return "U9500";
	case PRCMU_FW_PROJECT_U8500_MBB:
		return "U8500 MBB";
	case PRCMU_FW_PROJECT_U8500_C1:
		return "U8500 C1";
	case PRCMU_FW_PROJECT_U8500_C2:
		return "U8500 C2";
	case PRCMU_FW_PROJECT_U8500_C3:
		return "U8500 C3";
	case PRCMU_FW_PROJECT_U8500_C4:
		return "U8500 C4";
	case PRCMU_FW_PROJECT_U9500_MBL:
		return "U9500 MBL";
	case PRCMU_FW_PROJECT_U8500_MBL:
		return "U8500 MBL";
	case PRCMU_FW_PROJECT_U8500_MBL2:
		return "U8500 MBL2";
	case PRCMU_FW_PROJECT_U8520:
		return "U8520 MBL";
	case PRCMU_FW_PROJECT_U8420:
		return "U8420";
	case PRCMU_FW_PROJECT_U8420_SYSCLK:
		return "U8420-sysclk";
	case PRCMU_FW_PROJECT_A9420:
		return "A9420";
	default:
		return "Unknown";
	}
}

static inline void db8500_prcmu_set(unsigned int reg, u32 bits)
{
	db8500_prcmu_write_masked(reg, bits, bits);
}

static inline void db8500_prcmu_clear(unsigned int reg, u32 bits)
{
	db8500_prcmu_write_masked(reg, bits, 0);
}


static int db8500_prcmu_enable_spi2(void)
{
	db8500_prcmu_set(DB8500_PRCM_GPIOCR, DB8500_PRCM_GPIOCR_SPI2_SELECT);
	return 0;
}

/**
 * prcmu_disable_spi2 - Disables pin muxing for SPI2 on OtherAlternateC1.
 */
static int db8500_prcmu_disable_spi2(void)
{
	db8500_prcmu_clear(DB8500_PRCM_GPIOCR, DB8500_PRCM_GPIOCR_SPI2_SELECT);
	return 0;
}

/**
 * prcmu_enable_stm_mod_uart - Enables pin muxing for STMMOD
 * and UARTMOD on OtherAlternateC3.
 */
static int db8500_prcmu_enable_stm_mod_uart(void)
{
	db8500_prcmu_set(DB8500_PRCM_GPIOCR,
			(DB8500_PRCM_GPIOCR_DBG_STM_MOD_CMD1 |
			 DB8500_PRCM_GPIOCR_DBG_UARTMOD_CMD0));
	return 0;
}

/**
 * prcmu_disable_stm_mod_uart - Disables pin muxing for STMMOD
 * and UARTMOD on OtherAlternateC3.
 */
static int db8500_prcmu_disable_stm_mod_uart(void)
{
	db8500_prcmu_clear(DB8500_PRCM_GPIOCR,
			(DB8500_PRCM_GPIOCR_DBG_STM_MOD_CMD1 |
			 DB8500_PRCM_GPIOCR_DBG_UARTMOD_CMD0));
	return 0;
}

/**
 * prcmu_enable_stm_ape - Enables pin muxing for STM APE on OtherAlternateC1.
 */
static int db8500_prcmu_enable_stm_ape(void)
{
	db8500_prcmu_set(DB8500_PRCM_GPIOCR,
			DB8500_PRCM_GPIOCR_DBG_STM_APE_CMD);
	return 0;
}

/**
 * prcmu_disable_stm_ape - Disables pin muxing for STM APE on OtherAlternateC1.
 */
static int db8500_prcmu_disable_stm_ape(void)
{
	db8500_prcmu_clear(DB8500_PRCM_GPIOCR,
			DB8500_PRCM_GPIOCR_DBG_STM_APE_CMD);
	return 0;
}

static  struct prcmu_val_data db8500_val_tab[] = {
	{
		.val = APE_OPP,
		.set_val = db8500_prcmu_set_ape_opp,
		.get_val = db8500_prcmu_get_ape_opp,
	},
	{
		.val = DDR_OPP,
		.set_val = db8500_prcmu_set_ddr_opp,
		.get_val = db8500_prcmu_get_ddr_opp,
	},
	{
		.val = ARM_OPP,
		.set_val = db8500_prcmu_set_arm_opp,
		.get_val = db8500_prcmu_get_arm_opp,
	}
};
static struct prcmu_out_data db8500_out_tab[] = {
	{
		.out = SPI2_MUX,
		.enable =  db8500_prcmu_enable_spi2,
		.disable = db8500_prcmu_disable_spi2,
	},
	{
		.out = STM_APE_MUX,
		.enable = db8500_prcmu_enable_stm_ape,
		.disable = db8500_prcmu_disable_stm_ape,
	},
	{
		.out = STM_MOD_UART_MUX,
		.enable = db8500_prcmu_enable_stm_mod_uart,
		.disable = db8500_prcmu_disable_stm_mod_uart,
	}
};

static struct prcmu_early_data db8500_early_fops = {
	/*  system reset  */
	.system_reset = db8500_prcmu_system_reset,

	/*  clock service */
	.config_clkout = db8500_prcmu_config_clkout,
	.request_clock = db8500_prcmu_request_clock,

	/*  direct register access */
#if defined(CONFIG_MACH_SEC_GOLDEN_CHN) || defined(CONFIG_MACH_GAVINI_CHN) || defined(CONFIG_MACH_CODINA_CHN) 
	.tcdm_read = db8500_prcmu_tcdm_read,
#endif
	.read = db8500_prcmu_read,
	.write =  db8500_prcmu_write,
	.write_masked = db8500_prcmu_write_masked,
	/* others */
	.round_clock_rate = db8500_prcmu_round_clock_rate,
	.set_clock_rate = db8500_prcmu_set_clock_rate,
	.clock_rate = db8500_prcmu_clock_rate,
	.get_fw_version = db8500_prcmu_get_fw_version,
	.vc = db8500_prcmu_vc,
};

static struct prcmu_fops_register db8500_early_tab[] = {
	{
		.fops = PRCMU_EARLY,
		.data.pearly = &db8500_early_fops
	},
	{
		.fops = PRCMU_VAL,
		.size = ARRAY_SIZE(db8500_val_tab),
		.data.pval = db8500_val_tab
	},
	{
		.fops = PRCMU_OUT,
		.size = ARRAY_SIZE(db8500_out_tab),
		.data.pout = db8500_out_tab
	}
};

static struct prcmu_fops_register_data db8500_early_data = {
	.size = ARRAY_SIZE(db8500_early_tab),
	.tab = db8500_early_tab
};

struct prcmu_probe_data db8500_probe_fops = {
	/* sysfs soc inf */
	.get_reset_code = db8500_prcmu_get_reset_code,

	/* pm/suspend.c/cpu freq */
	.config_esram0_deep_sleep = db8500_prcmu_config_esram0_deep_sleep,
	.set_power_state = db8500_prcmu_set_power_state,
	.get_power_state_result = db8500_prcmu_get_power_state_result,
	.enable_wakeups = db8500_prcmu_enable_wakeups,
	.is_ac_wake_requested = db8500_prcmu_is_ac_wake_requested,

	/* modem */
	.modem_reset = db8500_prcmu_modem_reset,

	/* no used at all */
	.config_abb_event_readout = db8500_prcmu_config_abb_event_readout,
	.get_abb_event_buffer = db8500_prcmu_get_abb_event_buffer,

	/* abb access */
	.abb_read = db8500_prcmu_abb_read,
	.abb_write = db8500_prcmu_abb_write,
	.get_reset_status = db8500_prcmu_get_reset_status,
	/*  other u8500 specific */
	.request_ape_opp_100_voltage = db8500_prcmu_request_ape_opp_100_voltage,
	.configure_auto_pm = db8500_prcmu_configure_auto_pm,

	/* abb specific access */
	.abb_write_masked = db8500_prcmu_abb_write_masked,

};

static struct prcmu_fops_register db8500_probe_tab[] = {
	{
		.fops = PRCMU_PROBE,
		.data.pprobe = &db8500_probe_fops,
	},
};

struct prcmu_fops_register_data db8500_probe_data = {
	.size = ARRAY_SIZE(db8500_probe_tab),
	.tab = db8500_probe_tab,
};

/*
 * default event tracing
 * ENABLE_FTRACE_BY_DEFAULT
 *  : it should be enabled only if needed,
 *	it will highly impact on system behavior
 */
static int __init late(void)
{
	#ifdef CONFIG_DB8500_LIVEOPP
	int ret;
	#endif /* CONFIG_DB8500_LIVEOPP */
	extern int tracing_update_buffers(void);
#ifdef ENABLE_FTRACE_BY_DEFAULT
	extern int tracing_set_tracer(const char *buf);
	int err;
#endif
	tracing_update_buffers();

	trace_set_clr_event("irq", "irq_handler_entry", 1);
	trace_set_clr_event("irq", "irq_handler_exit", 1);
	//trace_set_clr_event("irq", "softirq_entry", 1);
	//trace_set_clr_event("irq", "softirq_exit", 1);
	trace_set_clr_event("sched", "sched_switch", 1);
	// trace_set_clr_event("sched", "sched_wakeup", 1);
	trace_set_clr_event("workqueue", "workqueue_execute_start", 1);
	trace_set_clr_event("workqueue", "workqueue_execute_end", 1);
	trace_set_clr_event("power", "cpu_frequency", 1);
	trace_set_clr_event("prcmu", NULL, 1);

	#ifdef CONFIG_DB8500_LIVEOPP
	liveopp_kobject = kobject_create_and_add("liveopp", kernel_kobj);
	if (!liveopp_kobject) {
		pr_err("[LiveOPP] Failed to create kobject interface\n");
	}
	ret = sysfs_create_group(liveopp_kobject, &liveopp_interface_group);
	if (ret) {
		kobject_put(liveopp_kobject);
	}
	pr_info("[LiveOPP] Initialized: v%s\n", LIVEOPP_VER);
	#endif /* CONFIG_DB8500_LIVEOPP */

#ifdef ENABLE_FTRACE_BY_DEFAULT
	err = tracing_set_tracer("function");
	if (err)
		pr_info("prcmu : unable to enable function trace\n");
#endif

	return 0;
}
late_initcall(late);

struct prcmu_fops_register_data *__init db8500_prcmu_early_init(void)
{
	unsigned int i;
	void __iomem *sec_base;

	struct prcmu_fw_version *fw_ver;

	fw_ver = db8500_prcmu_get_fw_version();

	if (fw_ver)
		pr_info("PRCMU firmware: %s(%d), version %d.%d.%d\n",
			fw_ver->project_name,
			fw_ver->project,
			fw_ver->api_version, fw_ver->func_version,
			fw_ver->errata);

	tcdm_base = __io_address(U8500_PRCMU_TCDM_BASE);
	prcmu_base = __io_address(U8500_PRCMU_BASE);

	/*
	 * Copy the value of the reset status register and if needed also
	 * the software reset code.
	 */
	sec_base = ioremap_nocache(U8500_PRCMU_SEC_BASE, SZ_4K);
	if (sec_base != NULL) {
		reset_status_copy = readl(sec_base +
			DB8500_SEC_PRCM_RESET_STATUS);
		iounmap(sec_base);
	}
	if (reset_status_copy & DB8500_SEC_PRCM_RESET_STATUS_APE_SOFTWARE_RESET)
		reset_code_copy = readw(tcdm_base + PRCM_SW_RST_REASON);

	spin_lock_init(&mb0_transfer.lock);
	spin_lock_init(&mb0_transfer.dbb_irqs_lock);
	mutex_init(&mb0_transfer.ac_wake_lock);
	init_completion(&mb0_transfer.ac_wake_work);
	mutex_init(&mb1_transfer.lock);
	init_completion(&mb1_transfer.work);
	mb1_transfer.ape_opp = APE_NO_CHANGE;
	mutex_init(&mb2_transfer.lock);
	init_completion(&mb2_transfer.work);
	spin_lock_init(&mb2_transfer.auto_pm_lock);
	spin_lock_init(&mb3_transfer.lock);
	mutex_init(&mb3_transfer.sysclk_lock);
	init_completion(&mb3_transfer.sysclk_work);
	INIT_WORK(&mb3_transfer.fw_log_work, db8500_prcmu_mb3_fw_log_work);
	spin_lock_init(&mb3_transfer.fw_log_lock);
	mutex_init(&mb4_transfer.lock);
	init_completion(&mb4_transfer.work);
	mutex_init(&mb5_transfer.lock);
	init_completion(&mb5_transfer.work);

	INIT_WORK(&mb0_transfer.mask_work, prcmu_mask_work);

	/* Initalize irqs. */
	for (i = 0; i < NUM_PRCMU_WAKEUPS; i++) {
		unsigned int irq;

		irq = IRQ_PRCMU_BASE + i;
		irq_set_chip_and_handler(irq, &prcmu_irq_chip,
					 handle_simple_irq);
		set_irq_flags(irq, IRQF_VALID);
	}
	compute_armss_rate();
	
	wake_lock_init(&prcmu_uart_wake_lock, WAKE_LOCK_SUSPEND, "prcmu_uart_wake_lock");

	/*  early init of dbx500-prcmu */
	return &db8500_early_data;
}

static void __init init_prcm_registers(void)
{
	u32 val;

	val = readl(PRCM_A9PL_FORCE_CLKEN);
	val &= ~(PRCM_A9PL_FORCE_CLKEN_PRCM_A9PL_FORCE_CLKEN |
		PRCM_A9PL_FORCE_CLKEN_PRCM_A9AXI_FORCE_CLKEN);
	writel(val, (PRCM_A9PL_FORCE_CLKEN));
}

void db8500_prcmu_in_panic_mode(void)
{
	db8500_probe_fops.abb_read = db8500_prcmu_abb_read_no_irq;
}

/*
 * Power domain switches (ePODs) modeled as regulators for the DB8500 SoC
 */
static struct regulator_consumer_supply db8500_vape_consumers[] = {
	REGULATOR_SUPPLY("v-ape", NULL),
	REGULATOR_SUPPLY("v-i2c", "nmk-i2c.0"),
	REGULATOR_SUPPLY("v-i2c", "nmk-i2c.1"),
	REGULATOR_SUPPLY("v-i2c", "nmk-i2c.2"),
	REGULATOR_SUPPLY("v-i2c", "nmk-i2c.3"),
	/* "v-mmc" changed to "vcore" in the mainline kernel */
	REGULATOR_SUPPLY("vcore", "sdi0"),
	REGULATOR_SUPPLY("vcore", "sdi1"),
	REGULATOR_SUPPLY("vcore", "sdi2"),
	REGULATOR_SUPPLY("vcore", "sdi3"),
	REGULATOR_SUPPLY("vcore", "sdi4"),
	REGULATOR_SUPPLY("v-dma", "dma40.0"),
	REGULATOR_SUPPLY("v-ape", "ab8500-usb.0"),
	REGULATOR_SUPPLY("v-uart", "uart0"),
	REGULATOR_SUPPLY("v-uart", "uart1"),
	REGULATOR_SUPPLY("v-uart", "uart2"),
	REGULATOR_SUPPLY("v-ape", "nmk-ske-keypad.0"),
	REGULATOR_SUPPLY("v-hsi", "ste_hsi.0"),
};

static struct regulator_consumer_supply db8500_vsmps2_consumers[] = {
	REGULATOR_SUPPLY("musb_1v8", "ab8500-usb.0"),
	/* AV8100 regulator */
	REGULATOR_SUPPLY("hdmi_1v8", "0-0070"),
};

static struct regulator_consumer_supply db8500_b2r2_mcde_consumers[] = {
	REGULATOR_SUPPLY("vsupply", "b2r2_core"),
	REGULATOR_SUPPLY("vsupply", "mcde"),
	REGULATOR_SUPPLY("vsupply", "dsilink.0"),
	REGULATOR_SUPPLY("vsupply", "dsilink.1"),
	REGULATOR_SUPPLY("vsupply", "dsilink.2"),
};

/* SVA MMDSP regulator switch */
static struct regulator_consumer_supply db8500_svammdsp_consumers[] = {
	REGULATOR_SUPPLY("sva-mmdsp", "cm_control"),
};

/* SVA pipe regulator switch */
static struct regulator_consumer_supply db8500_svapipe_consumers[] = {
	REGULATOR_SUPPLY("sva-pipe", "cm_control"),
};

/* SIA MMDSP regulator switch */
static struct regulator_consumer_supply db8500_siammdsp_consumers[] = {
	REGULATOR_SUPPLY("sia-mmdsp", "cm_control"),
};

/* SIA pipe regulator switch */
static struct regulator_consumer_supply db8500_siapipe_consumers[] = {
	REGULATOR_SUPPLY("sia-pipe", "cm_control"),
};

static struct regulator_consumer_supply db8500_sga_consumers[] = {
	REGULATOR_SUPPLY("v-mali", NULL),
};

/* ESRAM1 and 2 regulator switch */
static struct regulator_consumer_supply db8500_esram12_consumers[] = {
	REGULATOR_SUPPLY("esram12", "cm_control"),
};

/* ESRAM3 and 4 regulator switch */
static struct regulator_consumer_supply db8500_esram34_consumers[] = {
	REGULATOR_SUPPLY("v-esram34", "mcde"),
	REGULATOR_SUPPLY("esram34", "cm_control"),
	REGULATOR_SUPPLY("lcla_esram", "dma40.0"),
};

static struct regulator_init_data db8500_regulators[DB8500_NUM_REGULATORS] = {
	[DB8500_REGULATOR_VAPE] = {
		.constraints = {
			.name = "db8500-vape",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.consumer_supplies = db8500_vape_consumers,
		.num_consumer_supplies = ARRAY_SIZE(db8500_vape_consumers),
	},
	[DB8500_REGULATOR_VARM] = {
		.constraints = {
			.name = "db8500-varm",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
	},
	[DB8500_REGULATOR_VMODEM] = {
		.constraints = {
			.name = "db8500-vmodem",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
	},
	[DB8500_REGULATOR_VPLL] = {
		.constraints = {
			.name = "db8500-vpll",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
	},
	[DB8500_REGULATOR_VSMPS1] = {
		.constraints = {
			.name = "db8500-vsmps1",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
	},
	[DB8500_REGULATOR_VSMPS2] = {
		.constraints = {
			.name = "db8500-vsmps2",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.consumer_supplies = db8500_vsmps2_consumers,
		.num_consumer_supplies = ARRAY_SIZE(db8500_vsmps2_consumers),
	},
	[DB8500_REGULATOR_VSMPS3] = {
		.constraints = {
			.name = "db8500-vsmps3",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
	},
	[DB8500_REGULATOR_VRF1] = {
		.constraints = {
			.name = "db8500-vrf1",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
	},
	[DB8500_REGULATOR_SWITCH_SVAMMDSP] = {
		/* dependency to u8500-vape is handled outside regulator framework */
		.constraints = {
			.name = "db8500-sva-mmdsp",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.consumer_supplies = db8500_svammdsp_consumers,
		.num_consumer_supplies = ARRAY_SIZE(db8500_svammdsp_consumers),
	},
	[DB8500_REGULATOR_SWITCH_SVAMMDSPRET] = {
		.constraints = {
			/* "ret" means "retention" */
			.name = "db8500-sva-mmdsp-ret",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
	},
	[DB8500_REGULATOR_SWITCH_SVAPIPE] = {
		/* dependency to u8500-vape is handled outside regulator framework */
		.constraints = {
			.name = "db8500-sva-pipe",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.consumer_supplies = db8500_svapipe_consumers,
		.num_consumer_supplies = ARRAY_SIZE(db8500_svapipe_consumers),
	},
	[DB8500_REGULATOR_SWITCH_SIAMMDSP] = {
		/* dependency to u8500-vape is handled outside regulator framework */
		.constraints = {
			.name = "db8500-sia-mmdsp",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.consumer_supplies = db8500_siammdsp_consumers,
		.num_consumer_supplies = ARRAY_SIZE(db8500_siammdsp_consumers),
	},
	[DB8500_REGULATOR_SWITCH_SIAMMDSPRET] = {
		.constraints = {
			.name = "db8500-sia-mmdsp-ret",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
	},
	[DB8500_REGULATOR_SWITCH_SIAPIPE] = {
		/* dependency to u8500-vape is handled outside regulator framework */
		.constraints = {
			.name = "db8500-sia-pipe",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.consumer_supplies = db8500_siapipe_consumers,
		.num_consumer_supplies = ARRAY_SIZE(db8500_siapipe_consumers),
	},
	[DB8500_REGULATOR_SWITCH_SGA] = {
		.supply_regulator = "db8500-vape",
		.constraints = {
			.name = "db8500-sga",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.consumer_supplies = db8500_sga_consumers,
		.num_consumer_supplies = ARRAY_SIZE(db8500_sga_consumers),

	},
	[DB8500_REGULATOR_SWITCH_B2R2_MCDE] = {
		.supply_regulator = "db8500-vape",
		.constraints = {
			.name = "db8500-b2r2-mcde",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.consumer_supplies = db8500_b2r2_mcde_consumers,
		.num_consumer_supplies = ARRAY_SIZE(db8500_b2r2_mcde_consumers),
	},
	[DB8500_REGULATOR_SWITCH_ESRAM12] = {
		/*
		 * esram12 is set in retention and supplied by Vsafe when Vape is off,
		 * no need to hold Vape
		 */
		.constraints = {
			.name = "db8500-esram12",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.consumer_supplies = db8500_esram12_consumers,
		.num_consumer_supplies = ARRAY_SIZE(db8500_esram12_consumers),
	},
	[DB8500_REGULATOR_SWITCH_ESRAM12RET] = {
		.constraints = {
			.name = "db8500-esram12-ret",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
	},
	[DB8500_REGULATOR_SWITCH_ESRAM34] = {
		/*
		 * esram34 is set in retention and supplied by Vsafe when Vape is off,
		 * no need to hold Vape
		 */
		.constraints = {
			.name = "db8500-esram34",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.consumer_supplies = db8500_esram34_consumers,
		.num_consumer_supplies = ARRAY_SIZE(db8500_esram34_consumers),
	},
	[DB8500_REGULATOR_SWITCH_ESRAM34RET] = {
		.constraints = {
			.name = "db8500-esram34-ret",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
	},
};

static struct db8500_regulator_init_data db8500_regulators_pdata = {
	.set_epod = set_epod,
	.regulators = db8500_regulators,
	.reg_num = DB8500_NUM_REGULATORS
};

static struct ux500_wdt_ops db8500_wdt_ops = {
	.enable = enable_a9wdog,
	.disable = disable_a9wdog,
	.kick = kick_a9wdog,
	.load = load_a9wdog,
	.config = config_a9wdog,
};

static struct dbx500_temp_pdata db8500_temp_pdata = {
	.ops = NULL,
};

static struct resource u8500_thsens_resources[] = {
	{
		.name = "IRQ_HOTMON_LOW",
		.start  = IRQ_PRCMU_HOTMON_LOW,
		.end    = IRQ_PRCMU_HOTMON_LOW,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name = "IRQ_HOTMON_HIGH",
		.start  = IRQ_PRCMU_HOTMON_HIGH,
		.end    = IRQ_PRCMU_HOTMON_HIGH,
		.flags  = IORESOURCE_IRQ,
	},
};


static struct mfd_cell db8500_prcmu_devs[] = {
	{
		.name = "cpufreq-ux500",
		.id = -1,
	},
	{
		.name = "db8500-prcmu-regulators",
		.platform_data = &db8500_regulators_pdata,
		.pdata_size = sizeof(db8500_regulators_pdata),
	},
	{
		.name = "ux500_wdt",
		.id = -1,
		.platform_data = &db8500_wdt_ops,
		.pdata_size = sizeof(db8500_wdt_ops),
	},
	{
		.name = "dbx500_temp",
		.platform_data = &db8500_temp_pdata,
		.pdata_size = sizeof(db8500_temp_pdata),
		.resources       = u8500_thsens_resources,
		.num_resources  = ARRAY_SIZE(u8500_thsens_resources),
	},
	{
		.name = "dbx500-prcmu",
		.platform_data = &db8500_probe_data,
		.pdata_size = sizeof(db8500_probe_data),
	},
};
static struct cpufreq_frequency_table *freq_table;


static void  db8500_prcmu_update_freq(void *pdata)
{
	#ifdef CONFIG_DB8500_LIVEOPP
	int i, pllclk = pllarm_freq(db8500_prcmu_readl(PRCMU_PLLARM_REG));
	#endif /* CONFIG_DB8500_LIVEOPP */

	freq_table =
		(struct cpufreq_frequency_table *)pdata;

	#ifdef CONFIG_DB8500_LIVEOPP
	pr_info("[LiveOPP] Available freqs: %d\n", ARRAY_SIZE(liveopp_arm));
	for (i = 0; i < ARRAY_SIZE(liveopp_arm); i++) {
		freq_table[i].frequency = liveopp_arm[i].freq_show;

		if (liveopp_arm[i].freq_raw == pllclk) {
			pr_info("[LiveOPP] Boot up -> [%d] %dkHz\n", i, pllclk);
			last_arm_idx = i;
		}
	}
	#else /* CONFIG_DB8500_LIVEOPP */
	if  (!db8500_prcmu_has_arm_maxopp())
		return;
	switch (fw_info.version.project) {
	case PRCMU_FW_PROJECT_U8500:
	case PRCMU_FW_PROJECT_U9500:
	case PRCMU_FW_PROJECT_U8420:
	case PRCMU_FW_PROJECT_U8420_SYSCLK:
	case PRCMU_FW_PROJECT_A9420:
	case PRCMU_FW_PROJECT_U8500_MBL:
		freq_table[3].frequency = 1000000;
		break;
	case PRCMU_FW_PROJECT_U8500_C2:
	case PRCMU_FW_PROJECT_U8520:
		freq_table[3].frequency = 1150000;
		break;
	default:
		break;
	}
	#endif /* CONFIG_DB8500_LIVEOPP */
}


/**
 * prcmu_fw_init - arch init call for the Linux PRCMU fw init logic
 *
 */
static int __init db8500_prcmu_probe(struct platform_device *pdev)
{
	int err = 0;

	init_prcm_registers();

	/* Clean up the mailbox interrupts after pre-kernel code. */
	writel(ALL_MBOX_BITS, PRCM_ARM_IT1_CLR);

	err = request_threaded_irq(IRQ_DB8500_PRCMU1, prcmu_irq_handler,
		prcmu_irq_thread_fn, IRQF_NO_SUSPEND, "prcmu", NULL);
	if (err < 0) {
		pr_err("prcmu: Failed to allocate IRQ_DB8500_PRCMU1.\n");
		err = -EBUSY;
		goto no_irq_return;
	}


	db8500_prcmu_config_esram0_deep_sleep(ESRAM0_DEEP_SLEEP_STATE_RET);
	db8500_prcmu_update_freq(dev_get_platdata(&pdev->dev));

	/*  register mfd devices */
	err = mfd_add_devices(&pdev->dev, 0, db8500_prcmu_devs,
			      ARRAY_SIZE(db8500_prcmu_devs), NULL,
			      0);

	if (err)
		pr_err("prcmu: Failed to add subdevices\n");
	else
		pr_info("DB8500 PRCMU initialized\n");

no_irq_return:
	return err;
}

static struct platform_driver db8500_prcmu_driver = {
	.driver = {
		.name = "db8500-prcmu",
		.owner = THIS_MODULE,
	},
};

static int __init db8500_prcmu_init(void)
{
	return platform_driver_probe(&db8500_prcmu_driver, db8500_prcmu_probe);
}

arch_initcall(db8500_prcmu_init);

MODULE_AUTHOR("Mattias Nilsson <mattias.i.nilsson@stericsson.com>");
MODULE_DESCRIPTION("DB8500 PRCM Unit driver");
MODULE_LICENSE("GPL v2");
