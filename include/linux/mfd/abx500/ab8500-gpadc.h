/*
 * Copyright (C) 2010 ST-Ericsson SA
 * Licensed under GPLv2.
 *
 * Author: Arun R Murthy <arun.murthy@stericsson.com>
 * Author: Daniel Willerud <daniel.willerud@stericsson.com>
 * Author: M'boumba Cedric Madianga <cedric.madianga@stericsson.com>
 */

#ifndef	_AB8500_GPADC_H
#define _AB8500_GPADC_H

/* GPADC source: From datasheet(ADCSwSel[4:0] in GPADCCtrl2
 * and ADCHwSel[4:0] in GPADCCtrl3 ) */
#define BAT_CTRL	0x01
#define BTEMP_BALL	0x02
#define MAIN_CHARGER_V	0x03
#define ACC_DETECT1	0x04
#define ACC_DETECT2	0x05
#define ADC_AUX1	0x06
#define ADC_AUX2	0x07
#define MAIN_BAT_V	0x08
#define VBUS_V		0x09
#define MAIN_CHARGER_C	0x0A
#define USB_CHARGER_C	0x0B
#define BK_BAT_V	0x0C
#define DIE_TEMP	0x0D
#define USB_ID		0x0E

#define SAMPLE_1        1
#define SAMPLE_4        4
#define SAMPLE_8        8
#define SAMPLE_16       16
#define RISING_EDGE     0
#define FALLING_EDGE    1

/* Arbitrary ADC conversion type constants */
#define ADC_SW				0
#define ADC_HW				1


struct ab8500_gpadc;

struct ab8500_gpadc *ab8500_gpadc_get(void);
int ab8500_gpadc_read(struct ab8500_gpadc *gpadc, u8 input);
int ab8500_gpadc_sw_hw_convert(struct ab8500_gpadc *gpadc, u8 channel,
		u8 avg_sample, u8 trig_edge, u8 trig_timer, u8 conv_type);
static inline int ab8500_gpadc_convert(struct ab8500_gpadc *gpadc, u8 channel)
{
	return ab8500_gpadc_sw_hw_convert(gpadc, channel,
			SAMPLE_16, 0, 0, ADC_SW);
}

int ab8500_gpadc_read_raw(struct ab8500_gpadc *gpadc, u8 channel,
		u8 avg_sample, u8 trig_edge, u8 trig_timer, u8 conv_type);
int ab8500_gpadc_ad_to_voltage(struct ab8500_gpadc *gpadc,
		u8 channel, int ad_value);

#endif /* _AB8500_GPADC_H */
