/*
 * GP2A proximity sensor platform-specific data.
 *
 * Copyright (c) Samsung 2010
 */

#ifndef _GP2A_h_
#define	_GP2A_h_


#define	GP2A_I2C_DEVICE_NAME	"gp2a"

#if defined(CONFIG_MACH_SEC_KYLE)

/* B1 mode */
/*
#define PROX_NONDETECT		0x40
#define PROX_DETECT		0x20
*/
/* B1.5 mode */
#define PROX_NONDETECT		0x2f
#define PROX_DETECT		0x0f

#else
#define PROX_NONDETECT		0x40	/* B1 mode */
#define PROX_DETECT		0x0F
#endif

struct gp2a_platform_data
{
	unsigned int	ps_vout_gpio;
	bool	als_supported;
	int	alsout;
	int (* hw_setup)( struct device * );
	int (* hw_teardown)( void );
	void (* hw_pwr)( bool );
};


#endif
