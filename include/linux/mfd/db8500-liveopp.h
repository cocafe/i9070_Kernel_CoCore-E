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

#define PRCMU_ARMFIX_REG		0x0000
#define PRCMU_SGACLK_REG		0x0014
#define PRCMU_PLLSOC0_REG		0x0080
#define PRCMU_PLLSOC1_REG		0x0084
#define PRCMU_PLLARM_REG		0x0088
#define PRCMU_PLLDDR_REG		0x008C

/*
 * Varm has three voltage selections
 * But Selection 3 is never used
 */
#define AB8500_VARM_SEL1 		0x0B
#define AB8500_VARM_SEL2 		0x0C
#define AB8500_VARM_SEL3 		0x0D
#define AB8500_VBBX_REG 		0x11

/**
 * struct liveopp_arm_table - Custom frequency and voltage table
 * @freq_show:		Frequency(KHz) showed in userspace, no effect on real clock
 * @freq_raw:		Manually calculated frequency(KHz) for showing, no effect on real clock
 * @arm_opp:		STE ARM OPP index to use, we need to setup ARM OPP first
 * @arm_opp_rec(overy):	Workaround to fix dead issue when scales down from overclocked freqs
 * @extarm_raw:		Raw register value of ARMCLKFIX_MGT in PRCMU
 * @pllarm_raw:		Raw register value of PLLARM_FREQ in PRCMU
 * @varm_sel:		Varm voltage selection (1/2/3)
 * @varm_raw:		Raw register value of Varm regulator in AB850x
 * @vbbx_raw:		Raw register value of Vbbp and Vbbn regulator in AB850x
 * @set_pllarm:		Whether override the default PRCMU OPP frequency
 * @set_volt:		Whether override the default PRCMU OPP voltage
 */
struct liveopp_arm_table
{
	u32 	freq_show;
	u32 	freq_raw;
	u8  	arm_opp;
	u32 	set_extarm;
	u32 	extarm_raw;
	u32 	set_pllarm;
	u32 	pllarm_raw;
	u32 	set_volt;
	u8 	varm_sel;
	u8 	varm_raw;
	u8  	vbbx_raw;
};

/* Varm in 12.5mV steps */
#define AB8500_VARM_VSEL_MASK 		0x3f	/* 0011 1111 */
#define AB8500_VARM_STEP_UV		12500
#define AB8500_VARM_MIN_UV		700000
#define AB8500_VARM_MAX_UV		1362500

/* PLLARM in 38.4MHz steps */
#define PLLARM_FREQ_STEPS		38400
#define PLLARM_MAXOPP			0x0001011A
#define PLLARM_FREQ100OPP		0x00050168
