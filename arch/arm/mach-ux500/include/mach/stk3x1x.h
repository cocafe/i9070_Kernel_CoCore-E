/*
 *
 * $Id: stk3x1x.h
 *
 * Copyright (C) 2012 Lex Hsieh     <lex_hsieh@sitronix.com.tw> 
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 */
#ifndef __STK3X1X_H__
#define __STK3X1X_H__



/* Driver Settings */


/* Define Register Map */
#define STK_STATE_REG 			0x00
#define STK_PSCTRL_REG 			0x01
#define STK_LEDCTRL_REG 			0x03
#define STK_INT_REG 				0x04
#define STK_WAIT_REG 			0x05
#define STK_THDH1_PS_REG 		0x06
#define STK_THDH2_PS_REG 		0x07
#define STK_THDL1_PS_REG 		0x08
#define STK_THDL2_PS_REG 		0x09
#define STK_FLAG_REG 			0x10
#define STK_DATA1_PS_REG	 	0x11
#define STK_DATA2_PS_REG 		0x12
#define STK_DATA1_ALS_REG 		0x13
#define STK_DATA2_ALS_REG 		0x14
#define STK_DATA1_OFFSET_REG 	0x15
#define STK_DATA2_OFFSET_REG 	0x16
#define STK_PDT_ID_REG 			0x3E
#define STK_RSRVD_REG 			0x3F
#define STK_SW_RESET_REG		0x80


/* Define state reg */
#define STK_STATE_EN_AK_SHIFT  	6
#define STK_STATE_EN_ASO_SHIFT  	5
#define STK_STATE_EN_IRO_SHIFT  	4
#define STK_STATE_EN_WAIT_SHIFT  	2
#define STK_STATE_EN_PS_SHIFT  	0

#define STK_STATE_EN_AK_MASK	0x40
#define STK_STATE_EN_ASO_MASK	0x20
#define STK_STATE_EN_IRO_MASK	0x10
#define STK_STATE_EN_WAIT_MASK	0x04
#define STK_STATE_EN_PS_MASK	0x01

/* Define PS ctrl reg */
#define STK_PS_PRS_SHIFT  		6
#define STK_PS_GAIN_SHIFT  		4
#define STK_PS_IT_SHIFT  			0

#define STK_PS_PRS_MASK			0xC0
#define STK_PS_GAIN_MASK			0x30
#define STK_PS_IT_MASK			0x0F
	
/* Define LED ctrl reg */
#define STK_LED_IRDR_SHIFT  		6
#define STK_LED_DT_SHIFT  		0

#define STK_LED_IRDR_MASK		0xC0
#define STK_LED_DT_MASK			0x3F
	
/* Define interrupt reg */
#define STK_INT_OUI_SHIFT  		4
#define STK_INT_PS_SHIFT  			0

#define STK_INT_OUI_MASK			0x10
#define STK_INT_PS_MASK			0x07

#define STK_INT_PS_MODE3			0x03
/* Define flag reg */
#define STK_FLG_PSDR_SHIFT  		6
#define STK_FLG_PSINT_SHIFT  		4
#define STK_FLG_OUI_SHIFT  		2
#define STK_FLG_NF_SHIFT  		0

#define STK_FLG_PSDR_MASK		0x40
#define STK_FLG_PSINT_MASK		0x10
#define STK_FLG_OUT_MASK		0x04
#define STK_FLG_NF_MASK			0x01
	
/* misc define */
#define STK3X1X_PDT_ID	0x33
#define PS_MIN_DELAY    10
#define MIN_ALS_POLL_DELAY_NS	100000000


/* platform data */
struct stk3x1x_platform_data
{
	uint8_t state_reg;
	uint8_t psctrl_reg;
	uint8_t ledctrl_reg;
	uint8_t	wait_reg;	
	uint16_t ps_thd_h;
	uint16_t ps_thd_l;
	int int_pin;
};
	

#endif // __STK3X1X_H__
