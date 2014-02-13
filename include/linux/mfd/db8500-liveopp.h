/*
 * DB8500 Live OPP(Performance Operating Points)
 * 
 * Author: Huang Ji (cocafe@xda-developers.com) <cocafehj@gmail.com>
 * 
 * This program is free software and is provided to you under 
 * the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation, 
 * and any use by you of this program is subject to the terms of such GNU licence.
 * 
 * A copy of the licence is included with the program, 
 * and can also be obtained from Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */


#define PRCMU_ARMFIX_REG		0x000
#define PRCMU_ARMPLL_REG		0x088
#define PRCMU_DDRPLL_REG		0x08C

/*
 * Varm has three voltage selections
 * But Selection 3 is never used
 */
#define AB8500_VARM_SEL1 		0x0B
#define AB8500_VARM_SEL2 		0x0C
#define AB8500_VARM_SEL3 		0x0D
#define AB8500_VBB_REG 			0x11

/* To avoid setting ARM OPP */
#define ARM_IGNORE			0x10

/**
 * struct liveopp_arm_table - Custom frequency and voltage table
 * @freq_show:		Frequency(KHz) showed in userspace, no effect on real clock
 * @freq_raw:		Manually calculated frequency(KHz) for showing, no effect on real clock
 * @arm_opp:		STE ARM OPP index to use, we need to setup ARM OPP first
 * @arm_opp_rec(overy):	Workaround to fix dead issue when scales down from overclocked freqs
 * @armfix_raw:		Raw register value of ARMCLKFIX_MGT in PRCMU
 * @armpll_raw:		Raw register value of PLLARM_FREQ in PRCMU
 * @varm_sel:		Varm voltage selection (1/2/3)
 * @varm_raw:		Raw register value of Varm regulator in AB850x
 * @vbbpn_raw:		Raw register value of Vbbp and Vbbn regulator in AB850x
 * @override_freq:	Whether override the default PRCMU OPP frequency
 * @override_volt:	Whether override the default PRCMU OPP voltage
 */
struct liveopp_arm_table
{
	unsigned int freq_show;
	unsigned int freq_raw;
	unsigned char arm_opp;
	unsigned char arm_opp_rec;
	unsigned int armfix_raw;
	unsigned int armpll_raw;
	unsigned int varm_sel;
	unsigned int varm_raw;
	unsigned int vbbpn_raw;
	int override_freq;
	int override_volt;
};

/* Maximum slots in frequency table */
#define FREQTABLE_MAX_SLOTS		12

enum arm_slots
{
	ARM_SLOT0 = 0,
	ARM_SLOT1 = 1,
	ARM_SLOT2 = 2,
	ARM_SLOT3 = 3,
	ARM_SLOT4 = 4,
	ARM_SLOT5 = 5,
	ARM_SLOT6 = 6,
	ARM_SLOT7 = 7,
	ARM_SLOT8 = 8,
	ARM_SLOT9 = 9,
	ARM_SLOT10 = 10,
	ARM_SLOT11 = 11,
	ARM_SLOT12 = 12,
};

/* Varm in 12.5mV steps */
#define AB8500_VARM_VSEL_MASK 		0x3f	/* 0011 1111 */
#define AB8500_VARM_STEP_UV		12500
#define AB8500_VARM_MIN_UV		700000
#define AB8500_VARM_MAX_UV		1362500

/* PLLARM in 38400KHz steps */
#define PLLARM_FREQ_STEPS		38400

/* Access AB8500 registers */
int liveopp_ab8500_write(u8 bank, u8 reg, u8 value);
int liveopp_ab8500_read(u8 bank, u8 reg);
