/*
 * Platform data for the NXP PCA9491 battery charger driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _PCA9491_CHARGER_H_
#define _PCA9491_CHARGER_H_

struct pca9491_platform_data {
	unsigned int	irq_gpio;	/* GPIO pin that's connected to INT# */
	unsigned int	en_gpio;	/* EN GPIO pin */
	unsigned int	iin_cfg;	/* Input Current Limit - uA unit, range:0A~6.5A */
	unsigned int 	ichg_cfg;	/* Charging Current - uA unit, range:0A~6.35A */
	unsigned int	v_float;	/* V_Float Voltage - uV unit, range:4.2V~5.0V */
	unsigned int	topoff_cur;	/* Topoff current -uA unit */
	unsigned int 	snsres;		/* Current sense resister, 0 - 5mOhm, 1 - 10mOhm */

	unsigned int	vbus_ovp;	/* VBUS OVP threshold - uV unit, range:4.2V ~ 6.5V */
	unsigned int	vout_reg;	/* VOUT regulation loop threshold - uV unit, range:4.2V ~ 5.0V */
	unsigned int	vdrop_ovp;	/* VDROP over voltage threshold - uV unit, range:0~1000mV */
	unsigned int	vdrop_alarm;	/* VDROP alarm threshold - uV unit, range:0~1000mV */
};

#endif
