#ifndef __GP2A_H__
#define __GP2A_H__

#define I2C_M_WR 0		/* for i2c Write */
#define I2c_M_RD 1		/* for i2c Read */

#define I2C_DF_NOTIFY			0x01	/* for i2c */

#define	GP2A_I2C_NAME	"gp2a"

/* Registers */
#define COMMAND1    0x00
#define COMMAND2    0x01	/* Read&Write */
#define COMMAND3    0x02	/* Read&Write */
#define COMMAND4    0x03	/* Read&Write */
#define INT_LT_LSB  0x04	/* Read&Write */
#define INT_LT_MSB  0x05	/* Read&Write */
#define INT_HT_LSB  0x06	/* Read&Write */
#define INT_HT_MSB  0x07	/* Read&Write */
#define PS_LT_LSB   0x08	/* Read&Write */
#define PS_LT_MSB   0x09	/* Read&Write */
#define PS_HT_LSB   0x0A	/* Read&Write */
#define PS_HT_MSB   0x0B	/* Read&Write */
#define DATA0_LSB   0x0C	/* Read Only */
#define DATA0_MSB   0x0D	/* Read Only */
#define DATA1_LSB   0x0E	/* Read Only */
#define DATA1_MSB   0x0F	/* Read Only */
#define DATA2_LSB   0x10	/* Read Only */
#define DATA2_MSB   0x11	/* Read Only */

#define ADC_BUFFER_NUM	6

/* 16 level for premium model*/

enum {
	LIGHT_DIM   = 0,
	LIGHT_LEVEL1   = 1,
	LIGHT_LEVEL2   = 2,
	LIGHT_LEVEL3   = 3,
	LIGHT_LEVEL4   = 4,
	LIGHT_LEVEL5   = 5,
	LIGHT_LEVEL6   = 6,
	LIGHT_LEVEL7   = 7,
	LIGHT_LEVEL8   = 8,
	LIGHT_LEVEL9   = 9,
	LIGHT_LEVEL10   = 10,
	LIGHT_LEVEL11   = 11,
	LIGHT_LEVEL12   = 12,
	LIGHT_LEVEL13   = 13,
	LIGHT_LEVEL14   = 14,
	LIGHT_LEVEL15   = 15,
	LIGHT_LEVEL16   = 16,
	LIGHT_INIT  = 17,
};

/* prototype */
int opt_i2c_read(u8 reg, unsigned char *rbuf, int len, u16 addr, int adapt_num);
int opt_i2c_write(u8 reg, u8 *val, u16 addr, int adapt_num);

struct gp2a_platform_data {
	void (*power_on) (int);
	int p_out; /* proximity-sensor-output gpio */
	int adapt_num;
	int addr;
};

extern struct class *sensors_class;

#endif
