/*
 * Driver for the NXP PCA9491 battery charger.
 *
 * Author: Clark Kim <clark.kim@nxp.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/power_supply.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/rtc.h>

#include <linux/power/pca9491_charger.h>

#if defined (CONFIG_OF)
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif /* CONFIG_OF */

#include <linux/wakelock.h>
#ifdef CONFIG_USBPD_PHY_QCOM
#include <linux/usb/usbpd.h>		// Use Qualcomm USBPD PHY
#endif


#define BITS(_end, _start) ((BIT(_end) - BIT(_start)) + BIT(_end))
#define MASK2SHIFT(_mask)	__ffs(_mask)
#define MIN(a, b)	   ((a < b) ? (a):(b))
#define MAX(a, b)	   ((a > b) ? (a):(b))

//
// Register Map
//
#define PCA9491_REG_BIT_AUTO_INC		BIT(7)

/* System Control Registers */
#define PCA9491_REG_DEVICE_INFO 		0x00	// Device ID, revision
#define PCA9491_BIT_DEVICE_REV			BITS(5,3)
#define PCA9491_BIT_DEVICE_ID			BITS(2,0)

#define PCA9491_REG_EVENT_1_MASK		0x01	// INT mask for EVENT_1 status changes
#define PCA9491_BIT_VBUS_OVP_MASK		BIT(7)
#define PCA9491_BIT_IBUS_REG_MASK		BIT(6)
#define PCA9491_BIT_VBAT_REG_MASK		BIT(5)
#define PCA9491_BIT_IBAT_REG_MASK		BIT(4)
#define PCA9491_BIT_VOUT_REG_MASK		BIT(3)
#define PCA9491_BIT_TBUS_OTP_MASK		BIT(2)
#define PCA9491_BIT_TBAT_OTP_MASK		BIT(1)
#define PCA9491_BIT_IBUS_IREV_MASK		BIT(0)

#define PCA9491_REG_EVENT_2_MASK		0x02	// INT mask for EVENT_2 status changes
#define PCA9491_BIT_WATCHDOG_MASK		BIT(7)
#define PCA9491_BIT_ADC_DONE_MASK		BIT(6)
#define PCA9491_BIT_VDROP_ALM_MASK		BIT(5)
#define PCA9491_BIT_VDROP_OVP_MASK		BIT(4)
#define PCA9491_BIT_VBUS_INSERT_MASK	BIT(3)
#define PCA9491_BIT_BAT_INSERT_MASK		BIT(2)
#define PCA9491_BIT_TSHUT_FLT_MASK		BIT(1)
#define PCA9491_BIT_IOC_FLT_MASK		BIT(0)

#define PCA9491_REG_EVENT_1				0x03	// EVENT_1 status regsiter
#define PCA9491_BIT_VBUS_OVP_FLT		BIT(7)
#define PCA9491_BIT_IBUS_REG_LDO		BIT(6)
#define PCA9491_BIT_VBAT_REG_LDO		BIT(5)
#define PCA9491_BIT_IBAT_REG_LDO		BIT(4)
#define PCA9491_BIT_VOUT_REG_LDO		BIT(3)
#define PCA9491_BIT_TBUS_OTP_FLT		BIT(2)
#define PCA9491_BIT_TBAT_OTP_FLT		BIT(1)
#define PCA9491_BIT_IBUS_IREV_FLT		BIT(0)

#define PCA9491_REG_EVENT_2				0x04	// EVENT_2 status register
#define PCA9491_BIT_WATCHDOG_FLT		BIT(7)
#define PCA9491_BIT_ADC_DONE			BIT(6)
#define PCA9491_BIT_VDROP_ALM_FLT		BIT(5)
#define PCA9491_BIT_VDROP_OVP_FLT		BIT(4)
#define PCA9491_BIT_VBUS_INSERT			BIT(3)
#define PCA9491_BIT_BAT_INSERT			BIT(2)
#define PCA9491_BIT_TSHUT_FLT			BIT(1)
#define PCA9491_BIT_IOC_FLT				BIT(0)

#define PCA9491_REG_EVENT_1_EN			0x05	// EVNET_1 base self protection
#define PCA9491_BIT_VBUS_OVP_EN			BIT(7)
#define PCA9491_BIT_IBUS_REG_EN			BIT(6)
#define PCA9491_BIT_VBAT_REG_EN			BIT(5)
#define PCA9491_BIT_IBAT_REG_EN			BIT(4)
#define PCA9491_BIT_VOUT_REG_EN			BIT(3)
#define PCA9491_BIT_TBUS_OTP_EN			BIT(2)
#define PCA9491_BIT_TBAT_OTP_EN			BIT(1)
#define PCA9491_BIT_VBUS_PD_EN			BIT(0)

#define PCA9491_REG_CONTROL				0x06	// System control register
#define PCA9491_BIT_VDROP_OVP_EN		BIT(7)
#define PCA9491_BIT_DROP_ALM_EN			BIT(6)
#define PCA9491_BIT_SENSE_R				BIT(5)
#define PCA9491_BIT_CHG_EN				BIT(4)
#define PCA9491_BIT_WATCHDOG			BITS(3,2)
#define PCA9491_BIT_IREV_SET			BIT(1)
#define PCA9491_BIT_REG_RST				BIT(0)

#define PCA9491_REG_ADC_CTRL			0x07	// ADC control register
#define PCA9491_BIT_TDIE_ADC_EN			BIT(7)
#define PCA9491_BIT_ADC_EN				BIT(3)
#define PCA9491_BIT_ADC_RATE			BIT(2)
#define PCA9491_BIT_ADC_AVG_EN			BIT(1)
#define PCA9491_BIT_ADC_SAMPLES			BIT(0)

#define PCA9491_REG_SAMPLE_EN			0x08	// ADC channel select register
#define PCA9491_BIT_VBUS_ADC_EN			BIT(7)
#define PCA9491_BIT_IBUS_ADC_EN			BIT(6)
#define PCA9491_BIT_VOUT_ADC_EN			BIT(5)
#define PCA9491_BIT_VDROP_ADC_EN		BIT(4)
#define PCA9491_BIT_VBAT_ADC_EN			BIT(3)
#define PCA9491_BIT_IBAT_ADC_EN			BIT(2)
#define PCA9491_BIT_TBUS_ADC_EN			BIT(1)
#define PCA9491_BIT_TBAT_ADC_EN			BIT(0)

#define PCA9491_REG_PROT_DLY_N_OCP		0x09	// OCP and switch de-glitch control
#define PCA9491_BIT_IBUS_OCP			BITS(7,4)
#define PCA9491_BIT_OCP_RES				BIT(1)
#define PCA9491_BIT_VBUS_OVP_DLY		BIT(0)

/* Protection Control Registers */
#define PCA9491_REG_VBUS_OVP			0x0A	// VBUS OVP threshold
#define PCA9491_BIT_VBUS_OVP			BITS(6,0)

#define PCA9491_REG_VOUT_REG			0x0B	// VOUT OVP threshold
#define PCA9491_BIT_VOUT_REG			BITS(6,0)

#define PCA9491_REG_VDROP_OVP			0x0C	// VDROP OVP threshold
#define PCA9491_BIT_VDROP_OVP			BITS(7,0)

#define PCA9491_REG_VDROP_ALARM			0x0D	// VDROP interrupt signal threshold
#define PCA9491_BIT_VDROP_ALARM			BITS(7,0)

#define PCA9491_REG_VBAT_REG			0x0E	// VBAT constant voltage value
#define PCA9491_BIT_VBAT_REG			BITS(6,0)

#define PCA9491_REG_IBAT_REG			0x0F	// IBAT constant current value
#define PCA9491_BIT_IBAT_REG			BITS(7,0)

#define PCA9491_REG_IBUS_REG			0x10	// IBUS constant current value
#define PCA9491_BIT_IBUS_REG			BITS(7,0)

#define PCA9491_REG_TBUS_OTP			0x11	// TBUS(VBUS) temperature threshold
#define PCA9491_BIT_TBUS_OTP			BITS(6,0)

#define PCA9491_REG_TBAT_OTP			0x12	// TBAT(Battery) temperature threshold
#define PCA9491_BIT_TBAT_OTP			BITS(6,0)

/* ADC Registers */
#define PCA9491_REG_VBUS_ADC_H			0x13	// VBUS 10-bit ADC channel high byte
#define PCA9491_REG_VBUS_ADC_L			0x14	// VBUS 10-bit ADC channel low byte

#define PCA9491_REG_IBUS_ADC_H			0x15	// IBUS 10-bit ADC channel high byte
#define PCA9491_REG_IBUS_ADC_L			0x16	// IBUS 10-bit ADC channel low byte

#define PCA9491_REG_VOUT_ADC_H			0x17	// VOUT 10-bit ADC channel high byte
#define PCA9491_REG_VOUT_ADC_L			0x18	// VOUT 10-bit ADC channel low byte

#define PCA9491_REG_VDROP_ADC_H			0x19	// VDROP 10-bit ADC channel high byte
#define PCA9491_REG_VDROP_ADC_L			0x1A	// VDROP 10-bit ADC channel low byte

#define PCA9491_REG_VBAT_ADC_H			0x1B	// VBAT 10-bit ADC channel high byte
#define PCA9491_REG_VBAT_ADC_L			0x1C	// VBAT 10-bit ADC channel low byte

#define PCA9491_REG_IBAT_ADC_H			0x1D	// IBAT 10-bit ADC channel high byte
#define PCA9491_REG_IBAT_ADC_L			0x1E	// IBAT 10-bit ADC channel low byte

#define PCA9491_REG_TBUS_ADC_H			0x1F	// TBUS 10-bit ADC channel high byte
#define PCA9491_REG_TBUS_ADC_L			0x20	// TBUS 10-bit ADC channel low byte

#define PCA9491_REG_TBAT_ADC_H			0x21	// TBAT 10-bit ADC channel high byte
#define PCA9491_REG_TBAT_ADC_L			0x22	// TBAT 10-bit ADC channel low byte

#define PCA9491_BIT_ADC_POL				BIT(7)
#define PCA9491_BIT_ADC_HI				BITS(4,0)
#define PCA9491_BIT_ADC_LO				BITS(7,0)

#define PCA9491_REG_DIE_TEM_ADC			0x23	// Die temperature 8-bit ADC channel
#define PCA9491_BIT_DIE_TEMP_ADC		BITS(7,0)

#define PCA9491_MAX_REGISTER			PCA9491_REG_DIE_TEM_ADC

// voltage and current step and offset value for registers
#define IBUS_REG_MAX		6500000	// 6.5A
#define IBAT_REG_MAX		6350000	// 6.35A
#define VBAT_REG_MAX		5000000	// 5.0V

#define IBUS_OCP_STEP		500000	// 500mA
#define VBUS_OVP_STEP		25000	// 25mV
#define VOUT_REG_STEP		10000	// 10mV
#define VDROP_OVP_STEP		5000	// 5mV
#define VDROP_ALRM_STEP		5000	// 5mV
#define VBAT_REG_STEP		10000	// 10mV
#define IBAT_REG_STEP		50000	// 50mA
#define IBUS_REG_STEP		50000	// 50mA
#define TBUS_OTP_STEP		20000	// 20mV
#define TBAT_OTP_STEP		20000	// 20mV

#define VBUS_OVP_OFFSET		4200000	// 4200mV
#define VOUT_REG_OFFSET		4200000	// 4200mV
#define VBAT_REG_OFFSET		4200000	// 4200mV


/* Initial value */
#define PCA9491_REG_EVENT_1_MASK_INIT	0x78
#define PCA9491_REG_EVENT_2_MASK_INIT	0xCC
#define PCA9491_REG_EVENT_1_EN_INIT		0xE0
#define PCA9491_REG_CONTROL_INIT		0x02
#define PCA9491_REG_ADC_CTRL_INIT		0x87	// 0x87
#define PCA9491_REG_SAMPLE_EN_INIT		0xFF	// 0xFF

#define PCA9491_SNSRES_5mOhm			0x00
#define PCA9491_SNSRES_10mOhm			PCA9491_BIT_SENSE_R



/* ADC Channel */
enum {
	ADCCH_VBUS = 1,
	ADCCH_IBUS,
	ADCCH_VOUT,
	ADCCH_VDROP,
	ADCCH_VBAT,
	ADCCH_IBAT,
	ADCCH_TBUS,
	ADCCH_TBAT,
	ADCCH_TDIE,
	ADCCH_MAX
};


/* Charging Done Condition */
#define PCA9491_ICHG_DONE	1000000	// 1000mA
#define PCA9491_IIN_DONE	500000	// 500mA

/* Timer defination */
#define PCA9491_VBATMIN_CHECK_T	1000	// 1000ms
#define PCA9491_CCMODE_CHECK1_T	60000	// 60000ms
#define PCA9491_CCMODE_CHECK2_T	10000	// 10000ms
#define PCA9491_CCMODE_CHECK3_T	5000	// 5000ms
#define PCA9491_CVMODE_CHECK_T 	10000	// 10000ms

#define PDMSG_WAIT_T			200		// 200ms
#define PPS_PERIODIC_T			10000	// 10000ms

/* Battery Threshold */
#define PCA9491_DC_VBAT_MIN		3500000	// 3500000uV
/* Input Current Limit default value */
#define PCA9491_IBUS_REG_DFT	3000000	// 3000000uA
/* Charging Current default value */
#define PCA9491_IBAT_REG_DFT	6000000	// 6000000uA
/* Charging Float Voltage default value */
#define PCA9491_VBAT_REG_DFT	4350000	// 4350000uV
/* Topoff current default value */
#define PCA9491_TOPOFF_CURRENT_DFT	500000	// 500000uA
/* Sense Resistance default value */
#define PCA9491_SENSE_R_DFT		1		// 10mOhm
/* VBUS OVP threshold default value */
#define PCA9491_VBUS_OVP_DFT	5500000	// 5500000uV
/* VOUT REG threshold default value */
#define PCA9491_VOUT_REG_DFT	4400000	// 4400000uV
/* VDROP OVP threshold default value */
#define PCA9491_VDROP_OVP_DFT	300000	// 300000uV
/* VDROP alarm threshold default value */
#define PCA9491_VDROP_ALARM_DFT	100000	// 100000uV

/* CC mode 1,2 battery threshold */
#define PCA9491_CC2_VBAT_MIN	4250000 // 4250000uV
#define PCA9491_CC3_VBAT_MIN	4330000	// 4330000uV

/* Maximum TA voltage threshold */
#define PCA9491_TA_MAX_VOL		5500000 // 5500000uV
/* Maximum TA current threshold */
#define PCA9491_TA_MAX_CUR		3000000	// 3000000uA

#define PCA9491_TA_VOL_PRE_OFFSET	100000	// 100000uV
/* Adjust CC mode TA voltage step */
#define PCA9491_TA_VOL_STEP_ADJ_CC	40000	// 40000uV
/* Pre CV mode TA voltage step */
#define PCA9491_TA_VOL_STEP_PRE_CV	20000	// 20000uV

/* PD Message Voltage and Current Step */
#define PDMSG_PPS_VOL_STEP		20000	// 20mV
#define PDMSG_PPS_CUR_STEP		50000	// 50mA


/* Direct Charging State */
enum {
	DC_STATE_NO_CHARGING,	/* no charigng */
	DC_STATE_CHECK_VBAT,	/* check min battery level */
	DC_STATE_PRESET_DC, 	/* Preset Direct Charging configuration */
	DC_STATE_ADJUST_CC,		/* Adjust CC mode */
	DC_STATE_START_CC,		/* Start CC mode */
	DC_STATE_CHECK_CC,		/* Check CC mode status */
	DC_STATE_START_CV,		/* Start CV mode */
	DC_STATE_CHECK_CV,		/* Check CV mode status */
	DC_STATE_CHARGING_DONE,	/* Charging Done */
	DC_STATE_MAX,
};


/* CC Mode Status */
enum {
	CCMODE_CHG_LOOP,
	CCMODE_VFLT_LOOP,
	CCMODE_IIN_LOOP,
	CCMODE_LOOP_INACTIVE,
	CCMODE_VIN_UVLO,
};

/* CV Mode Status */
enum {
	CVMODE_CHG_LOOP,
	CVMODE_VFLT_LOOP,
	CVMODE_IIN_LOOP,
	CVMODE_LOOP_INACTIVE,
	CVMODE_VIN_UVLO,
	CVMODE_CHG_DONE,
};

/* Timer ID */
enum {
	TIMER_ID_NONE,
	TIMER_VBATMIN_CHECK,
	TIMER_ENTER_ADJ_CCMODE,
	TIMER_CCMODE_CHECK,
	TIMER_ENTER_CVMODE,
	TIMER_CVMODE_CHECK,
	TIMER_PDMSG_SEND,
};

/* PD Message Type */
enum {
	PD_MSG_REQUEST_APDO,
	PD_MSG_REQUEST_FIXED_PDO,
};


/**
 * struct pca9491_charger - pca9491 charger instance
 * @monitor_wake_lock: lock to enter the suspend mode
 * @lock: protects concurrent access to online variables
 * @dev: pointer to device
 * @regmap: pointer to driver regmap
 * @mains: power_supply instance for AC/DC power
 * @work: timer work for charging
 * @timer_id:timer id for charging
 * @timer_period:timer period for charing timer
 * @last_update_time: system update time
 * @irq: pca9491 interrupt
 * @pps_work: pps periodic request work
 * @mains_online: is AC/DC input connected
 * @charging_state: direct charging state
 * @ta_cur: AC/DC(TA) current, uA
 * @ta_vol: AC/DC(TA) voltage, uV
 * @ta_objpos: AC/DC(TA) PDO object position
 * @pre_iin_adc: previous input current adc, uV
 * @ta_max_cur: AC/DC(TA) maximum request current
 * @ta_max_vol: AC/DC(TA) maximum request voltage
 * @pdata: pointer to platform data
 */
struct pca9491_charger {
	struct wake_lock	monitor_wake_lock;
	struct mutex		lock;
	struct device		*dev;
	struct regmap		*regmap;
	struct power_supply	*mains;
	struct delayed_work work;
	unsigned int		timer_id;
	unsigned long      	timer_period;
	unsigned long		last_update_time;
	int 				irq;
	
	struct delayed_work	pps_work;
#ifdef CONFIG_USBPD_PHY_QCOM
	struct usbpd 		*pd;
#endif

	bool				mains_online;
	unsigned int 		charging_state;
	
	unsigned int		ta_cur;
	unsigned int		ta_vol;
	unsigned int		ta_objpos;

	int					pre_iin_adc;	// Previous IIN ADC in CC adjust mode
	unsigned int		ta_max_cur;		// TA maximum current
	unsigned int		ta_max_vol;		// TA maximum voltage

	struct pca9491_platform_data *pdata;
};


#ifdef CONFIG_USBPD_PHY_QCOM
static int pca9491_usbpd_setup(struct pca9491_charger *pca9491);
#endif


/*******************************/
/* Switching charger control function */
/*******************************/
/* This function needs some modification by a customer */
static int pca9491_set_switching_charger( bool enable, 
												unsigned int input_current, 
												unsigned int charging_current, 
												unsigned int vfloat)
{
	int ret;
	struct power_supply *psy_swcharger;
	union power_supply_propval val;

	pr_info("%s: enable=%d, iin=%d, ichg=%d, vfloat=%d\n", 
		__func__, enable, input_current, charging_current, vfloat);
	
	/* Insert Code */

	/* Get power supply name */
#ifdef CONFIG_USBPD_PHY_QCOM
	psy_swcharger = power_supply_get_by_name("usb");
#else
	psy_swcharger = power_supply_get_by_name("sw-charger");
#endif
	if (psy_swcharger == NULL) {
		ret = -EINVAL;
		goto error;
	}
	
	if (enable == true)
	{
		// Set Switching charger //
#ifndef CONFIG_USBPD_PHY_QCOM
		/* input current */
		val.intval = input_current;
		ret = power_supply_set_property(psy_swcharger, POWER_SUPPLY_PROP_CURRENT_MAX, &val);
		if (ret < 0)
			goto error;
		/* charigng current */
		val.intval = charging_current;
		ret = power_supply_set_property(psy_swcharger, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, &val);
		if (ret < 0)
			goto error;
		/* vfloat voltage */
		val.intval = vfloat;
		ret = power_supply_set_property(psy_swcharger, POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE, &val);
		if (ret < 0)
			goto error;
#endif
		/* enable charger */
#ifdef CONFIG_USBPD_PHY_QCOM
		val.intval = enable;
		ret = power_supply_set_property(psy_swcharger, POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
		if (ret < 0)
			goto error;

		psy_swcharger = power_supply_get_by_name("main");
		if (psy_swcharger == NULL) {
				ret = -ENODEV;
				goto error;
		}
		/* input_current */
		val.intval = input_current;
		ret = power_supply_set_property(psy_swcharger, POWER_SUPPLY_PROP_CURRENT_MAX, &val);
		if (ret < 0)
			goto error;
#else
		/* it depends on customer's code */
		//val.intval = enable;
		//ret = power_supply_set_property(psy_swcharger, POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
		//if (ret < 0)
		//	goto error;
#endif
	}
	else
	{
		/* disable charger */
#ifdef CONFIG_USBPD_PHY_QCOM
		val.intval = enable;
		ret = power_supply_set_property(psy_swcharger, POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
		if (ret < 0)
			goto error;
		
		psy_swcharger = power_supply_get_by_name("main");
		if (psy_swcharger == NULL) {
				ret = -ENODEV;
				goto error;
		}
		/* input_current */
		val.intval = input_current;
		ret = power_supply_set_property(psy_swcharger, POWER_SUPPLY_PROP_CURRENT_MAX, &val);
		if (ret < 0)
			goto error;
#else
		/* it depends on customer's code */
		/* Todo */
		//val.intval = enable;
		//ret = psy_swcharger->set_property(psy_swcharger, POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
		//if (ret < 0)
		//	return ret;
#endif
	}

error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

static int pca9491_get_switching_charger_is_enabled(int *enable)
{
	int ret;
	struct power_supply *psy_swcharger;
	union power_supply_propval val;

	/* Get power supply name */
#ifdef CONFIG_USBPD_PHY_QCOM
	psy_swcharger = power_supply_get_by_name("usb");
#else
	psy_swcharger = power_supply_get_by_name("sw-charger");
#endif
	if (psy_swcharger == NULL) {
		ret = -EINVAL;
		goto error;
	}

	ret = power_supply_get_property(psy_swcharger,
				POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (ret < 0) {
		pr_err("%s, Unable to read PMI CHARGING_ENABLED: %d\n", __func__, ret);
		goto error;
	}

	*enable = val.intval;
	
error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}



/******************/
/* Send PD message */
/******************/
/* This function needs some modification by a customer */
static int pca9491_send_pd_message(struct pca9491_charger *pca9491, unsigned int msg_type)
{
	//int ret;
#ifdef CONFIG_USBPD_PHY_QCOM
	u32 msg_buf[3];	/* Data Buffer */
#else
	u8 msg_buf[12];	/* Data Buffer */
	unsigned int max_cur;
#endif
	unsigned int op_cur, out_vol;
	int ret = 0;

#ifdef CONFIG_USBPD_PHY_QCOM
	/* check the phandle */
	if (pca9491->pd == NULL) {
		pr_info("%s: get phandle\n", __func__);
		ret = pca9491_usbpd_setup(pca9491);
		if (ret != 0) {
			dev_err(pca9491->dev, "Error usbpd setup!\n");
			pca9491->pd = NULL;
			return ret;
		}
	}
#endif
	
	pr_info("%s: msg_type=%d, ta_cur=%d, ta_vol=%d, ta_objpos=%d\n", 
		__func__, msg_type, pca9491->ta_cur, pca9491->ta_vol, pca9491->ta_objpos);
		
	switch (msg_type) {
	case PD_MSG_REQUEST_APDO:
		/* Cancel pps request timer */
		cancel_delayed_work(&pca9491->pps_work);
#ifdef CONFIG_USBPD_PHY_QCOM
		ret = usbpd_request_pdo(pca9491->pd, pca9491->ta_objpos, pca9491->ta_vol, pca9491->ta_cur);
		if (ret == -EBUSY) {
			/* wait 100ms */
			msleep(100);
			/* try again */
			ret = usbpd_request_pdo(pca9491->pd, pca9491->ta_objpos, pca9491->ta_vol, pca9491->ta_cur);
		}
#else
		op_cur = pca9491->ta_cur/PDMSG_PPS_CUR_STEP;	// Operating Current 50mA units
		out_vol = pca9491->ta_vol/PDMSG_PPS_VOL_STEP;	// Output Voltage in 20mV units
		msg_buf[0] = op_cur & 0x7F;			// Operating Current 50mA units - B6...0
		msg_buf[1] = (out_vol<<1) & 0xFE;	// Output Voltage in 20mV units - B19..(B15)..B9
		msg_buf[2] = (out_vol>>7) & 0x0F;	// Output Voltage in 20mV units - B19..(B16)..B9,
		msg_buf[3] = pca9491->ta_objpos<<4;	// Object Position - B30...B28
		
		/* Send the PD messge to CC/PD chip */
		/* Todo - insert code */
#endif
		/* Start pps request timer */
		if (ret == 0) {
			schedule_delayed_work(&pca9491->pps_work, msecs_to_jiffies(PPS_PERIODIC_T));
		}
		break;
		
	case PD_MSG_REQUEST_FIXED_PDO:
		cancel_delayed_work(&pca9491->pps_work);
#ifdef CONFIG_USBPD_PHY_QCOM
		ret = usbpd_request_pdo(pca9491->pd, pca9491->ta_objpos, pca9491->ta_vol, pca9491->ta_cur);
		if (ret == -EBUSY) {
			/* wait 100ms */
			msleep(100);
			/* try again */
			ret = usbpd_request_pdo(pca9491->pd, pca9491->ta_objpos, pca9491->ta_vol, pca9491->ta_cur);
		}
#else
		max_cur = pca9491->ta_cur/10000; 	// Maximum Operation Current 10mA units
		op_cur = max_cur;					// Operating Current 10mA units
		msg_buf[0] = max_cur & 0xFF;		// Maximum Operation Current -B9..(7)..0
		msg_buf[1] = ((max_cur>>8) & 0x03) | ((op_cur<<2) & 0xFC);	// Operating Current - B19..(15)..10
		msg_buf[2] = ((op_cur>>6) & 0x0F);	// Operating Current - B19..(16)..10, Unchunked Extended Messages Supported  - not support
		msg_buf[3] = pca9491->ta_objpos<<4;	// Object Position - B30...B28
		
		/* Send the PD messge to CC/PD chip */
		/* Todo - insert code */
#endif
		break;
		
	default:
		break;
	}

	return ret;
}


/* ADC Read function */
static int pca9491_read_adc(struct pca9491_charger *pca9491, u8 adc_ch)
{
	unsigned int reg_data[2];
	unsigned int raw_adc;	// raw ADC value
	u8 adcch_reg;	// ADC channel register
	int sign_flag;	// sign bit
	int conv_adc;	// conversion ADC value
	int ret;
	
	switch (adc_ch)	{
	case ADCCH_VBUS:
		adcch_reg = PCA9491_REG_VBUS_ADC_H;
		break;

	case ADCCH_IBUS:
		adcch_reg = PCA9491_REG_IBUS_ADC_H;
		break;

	case ADCCH_VOUT:
		adcch_reg = PCA9491_REG_VOUT_ADC_H;
		break;

	case ADCCH_VDROP:
		adcch_reg = PCA9491_REG_VDROP_ADC_H;
		break;

	case ADCCH_VBAT:
		adcch_reg = PCA9491_REG_VBAT_ADC_H;
		break;

	case ADCCH_IBAT:
		adcch_reg = PCA9491_REG_IBAT_ADC_H;
		break;

	case ADCCH_TBUS:
		adcch_reg = PCA9491_REG_TBUS_ADC_H;
		break;

	case ADCCH_TBAT:
		adcch_reg = PCA9491_REG_TBAT_ADC_H;
		break;

	case ADCCH_TDIE:
		adcch_reg = PCA9491_REG_DIE_TEM_ADC;
		break;

	default:
		conv_adc = -EINVAL;
		goto error;
	}

	if (adc_ch == ADCCH_TDIE) {
		/* 8bit ADC */
		ret = regmap_read(pca9491->regmap, adcch_reg, &reg_data[0]);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		raw_adc = reg_data[0];
		conv_adc = raw_adc;
	} else {
		/* 10bit ADC */
		ret = regmap_bulk_read(pca9491->regmap, adcch_reg | PCA9491_REG_BIT_AUTO_INC, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		raw_adc = ((reg_data[0] & PCA9491_BIT_ADC_HI)<<8) | reg_data[1];
		sign_flag = (reg_data[0] & PCA9491_BIT_ADC_POL);
		if (sign_flag == PCA9491_BIT_ADC_POL)
			conv_adc = -raw_adc;
		else
			conv_adc = raw_adc;
		conv_adc = conv_adc*1000;	// uV/uA unit
	}

error:
	pr_info("%s: adc_ch=%d, convert_val=%d\n", __func__, adc_ch, conv_adc);

	return conv_adc;
}


static int pca9491_set_vfloat(struct pca9491_charger *pca9491, unsigned int v_float)
{
	int ret;
	unsigned int val;

	pr_info("%s: vfloat=%d\n", __func__, v_float);
	
	/* v float voltage */
	val = (v_float - VBAT_REG_OFFSET)/VBAT_REG_STEP;
	ret = regmap_write(pca9491->regmap, PCA9491_REG_VBAT_REG, val);
	return ret;
}

static int pca9491_set_charging_current(struct pca9491_charger *pca9491, unsigned int ichg)
{
	int ret;
	unsigned int val;
	
	pr_info("%s: ichg=%d\n", __func__, ichg);

	/* charging current */
	if (ichg > IBAT_REG_MAX)
		ichg = IBAT_REG_MAX;
	val = ichg/IBAT_REG_STEP;
	ret = regmap_write(pca9491->regmap, PCA9491_REG_IBAT_REG, val);
	return ret;
}

static int pca9491_set_input_current(struct pca9491_charger *pca9491, unsigned int iin)
{
	int ret;
	unsigned int val;

	pr_info("%s: iin=%d\n", __func__, iin);

	/* input current */
	if (iin > IBUS_REG_MAX)
		iin = IBUS_REG_MAX;
	val = iin/IBUS_REG_STEP;
	ret = regmap_write(pca9491->regmap, PCA9491_REG_IBUS_REG, val);
	return ret;
}

static int pca9491_set_charging(struct pca9491_charger *pca9491, bool enable)
{
	int ret, val;

	pr_info("%s: enable=%d\n", __func__, enable);
	
	/* Enable Charge */
	val = (enable == true) ? PCA9491_BIT_CHG_EN : 0;
	ret = regmap_update_bits(pca9491->regmap, PCA9491_REG_CONTROL, PCA9491_BIT_CHG_EN, val);
	return ret;
}

/* Stop charging */
static int pca9491_stop_charging(struct pca9491_charger *pca9491)
{
	int ret;

	pr_info("%s: ======START=======\n", __func__);

	/* cancel timer */
	cancel_delayed_work(&pca9491->work);
	cancel_delayed_work(&pca9491->pps_work);

	pca9491->charging_state = DC_STATE_NO_CHARGING;

	/* disable charging enable bit */
	ret = pca9491_set_charging(pca9491, false);
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;	
}


/* Check CC Mode status */
static int pca9491_check_ccmode_status(struct pca9491_charger *pca9491)
{
	unsigned int reg_val;
	int ret;

	/* Read EVENT_1 */
	ret = regmap_read(pca9491->regmap, PCA9491_REG_EVENT_1, &reg_val);
	if (ret < 0)
		goto error;
	
	/* Check EVENT_1 */
	if (reg_val & PCA9491_BIT_IBUS_REG_LDO)
		ret = CCMODE_IIN_LOOP;
	else if (reg_val & PCA9491_BIT_VBAT_REG_LDO)
		ret = CCMODE_VFLT_LOOP;
	else if (reg_val & PCA9491_BIT_IBAT_REG_LDO)
		ret = CCMODE_CHG_LOOP;
	else if (reg_val & PCA9491_BIT_VOUT_REG_LDO)
		ret = CCMODE_CHG_LOOP;
	else
		ret = CCMODE_LOOP_INACTIVE;

error:
	pr_info("%s: CCMODE Status=%d\n", __func__, ret);
	return ret;
}


/* Check CVMode Status */
static int pca9491_check_cvmode_status(struct pca9491_charger *pca9491)
{
	unsigned int val;
	int ret, iin;

	/* Read IBUS ADC */
	iin = pca9491_read_adc(pca9491, ADCCH_IBUS);
	if (iin < 0) {
		ret = iin;
		goto error;
	}

	/* Check TopOff Current */
	if (iin < pca9491->pdata->topoff_cur) {
		/* Direct Charging Done */
		ret = CVMODE_CHG_DONE;
	} else {
		/* It doesn't reach top-off condition yet */
		
		/* Read EVENT_1 */
		ret = regmap_read(pca9491->regmap, PCA9491_REG_EVENT_1, &val);
		if (ret < 0)
			goto error;		

		/* Check EVENT_1 */
		if (val & PCA9491_BIT_IBUS_REG_LDO)
			ret = CVMODE_IIN_LOOP;
		else if (val & PCA9491_BIT_VBAT_REG_LDO)
			ret = CVMODE_VFLT_LOOP;
		else if (val & PCA9491_BIT_IBAT_REG_LDO)
			ret = CVMODE_CHG_LOOP;
		else if (val & PCA9491_BIT_VOUT_REG_LDO)
			ret = CVMODE_CHG_LOOP;
		else
			ret = CVMODE_LOOP_INACTIVE; /* Any LOOP is inactive */
	}
	
error:
	pr_info("%s: CVMODE Status=%d\n", __func__, ret);
	return ret;
}


/* Check Fault Event condition */
static bool pca9491_check_fault(u8 *val)
{
	bool result = false;
	
	/* Check Fault bit of EVENT_1 */
	if (val[0] & PCA9491_BIT_VBUS_OVP_FLT) {
		pr_info("%s: VBUS_OVP fault\n", __func__);
		result = true;
	}
	if (val[0] & PCA9491_BIT_TBUS_OTP_FLT) {
		pr_info("%s: TBUS_OTP fault\n", __func__);
		result |= true;
	}
	if (val[0] & PCA9491_BIT_TBAT_OTP_FLT) {
		pr_info("%s: TBAT_OTP fault\n", __func__);
		result |= true;
	}
	if (val[0] & PCA9491_BIT_IBUS_IREV_FLT)	{
		pr_info("%s: IREV fault\n", __func__);
		result |= true;
	}
	/* Check Fault bit of EVENT_2 */
	if (val[1] & PCA9491_BIT_WATCHDOG_FLT) {
		pr_info("%s: Watchdog fault\n", __func__);
		result |= true;
	}
	if (val[2] & PCA9491_BIT_VDROP_ALM_FLT)	{
		pr_info("%s: VDROP alarm fault\n", __func__);
		result |= true;
	}
	if (val[2] & PCA9491_BIT_VDROP_OVP_FLT)	{
		pr_info("%s: VDROP_OVP fault\n", __func__);
		result |= true;
	}
	if (val[2] & PCA9491_BIT_TSHUT_FLT)	{
		pr_info("%s: TSHUT fault\n", __func__);
		result |= true;
	}
	if (val[2] & PCA9491_BIT_IOC_FLT) {
		pr_info("%s: IOC fault\n", __func__);
		result |= true;
	}
	
	return result;
}


/* Enable PCA9491 chip and initialize it or disable it */
static int pca9491_enable_chip(struct pca9491_charger *pca9491, bool enable)
{
	unsigned int val;
	int ret;

	pr_info("%s: ======START=======\n", __func__);
	
	if (enable == true)	{
		/* Set EN pin to High */
		ret = gpio_direction_output(pca9491->pdata->en_gpio, 1);
		if (ret < 0)
			goto error;
		
		msleep(1);	// 1ms delay

		/* Initialize ADC mode */
		val = PCA9491_REG_ADC_CTRL_INIT;
		ret = regmap_write(pca9491->regmap, PCA9491_REG_ADC_CTRL, val);
		if (ret < 0)
			goto error;
		
		val = PCA9491_REG_SAMPLE_EN_INIT;
		ret = regmap_write(pca9491->regmap, PCA9491_REG_SAMPLE_EN, val);
		if (ret < 0)
			goto error;
		
		msleep(1);	// 1ms delay
		
		ret = regmap_update_bits(pca9491->regmap, PCA9491_REG_ADC_CTRL, 
								PCA9491_BIT_ADC_EN, PCA9491_BIT_ADC_EN);
		if (ret < 0)
			goto error;
	}
	else {
		/* Set EN pin to Low */
		ret = gpio_direction_output(pca9491->pdata->en_gpio, 1);
		if (ret < 0)
			goto error;
	}

error:
	pr_info("%s: enable=%d, ret=%d\n", __func__, enable, ret);
	return ret;
}


/* Initial Setup for the direct charging */
static int pca9491_init_setup(struct pca9491_charger *pca9491)
{
	unsigned int val;
	int ret;

	pr_info("%s: ======START=======\n", __func__);
	
	/* 1. Set the EVENT Mask registers */
	// EVENT_1_MASK = 0x78
	val = PCA9491_REG_EVENT_1_MASK_INIT;
	ret = regmap_write(pca9491->regmap, PCA9491_REG_EVENT_1_MASK, val);
	if (ret < 0)
		goto error;

	// EVENT_2_MASK = 0xCC
	val = PCA9491_REG_EVENT_2_MASK_INIT;
	ret = regmap_write(pca9491->regmap, PCA9491_REG_EVENT_2_MASK, val);
	if (ret < 0)
		goto error;

	// EVENT_1_EN = 0xE0
	val = PCA9491_REG_EVENT_1_EN_INIT;
	ret = regmap_write(pca9491->regmap, PCA9491_REG_EVENT_1_EN, val);
	if (ret < 0)
		goto error;

	/* 2. Set CONTROL register */
	// CONTROL = 0x02
	val = PCA9491_REG_CONTROL_INIT;
	ret = regmap_write(pca9491->regmap, PCA9491_REG_CONTROL, val);
	if (ret < 0)
		goto error;

	/* 3. Set Protection registers */
	// IBUS_OCP = 20% of IBUS
	val = ((pca9491->pdata->iin_cfg*120/100)/IBUS_OCP_STEP + 1)<<4;
	ret = regmap_update_bits(pca9491->regmap, PCA9491_REG_PROT_DLY_N_OCP, 
							PCA9491_BIT_IBUS_OCP, val);
	if (ret < 0)
		goto error;
	
	// VBUS_OVP = 20% of VREG
	val = ((pca9491->pdata->v_float*120/100 - VBUS_OVP_OFFSET)/VBUS_OVP_STEP + 1);
	ret = regmap_write(pca9491->regmap, PCA9491_REG_VBUS_OVP, val);
	if (ret < 0)
		goto error;

	/* 4. Set Regulation */
	// VOUT_REG = 5V
	val = (pca9491->pdata->vout_reg - VOUT_REG_OFFSET)/VOUT_REG_STEP;
	ret = regmap_write(pca9491->regmap, PCA9491_REG_VOUT_REG, val);
	if (ret < 0)
		goto error;

	/* 5. Vdrop OVP */
	val = pca9491->pdata->vdrop_ovp;
	ret = regmap_write(pca9491->regmap, PCA9491_REG_VDROP_OVP, val);
	if (ret < 0)
		goto error;

	/* 6. Select Sense Resistor */
	val = pca9491->pdata->snsres << MASK2SHIFT(PCA9491_BIT_SENSE_R);
	ret = regmap_update_bits(pca9491->regmap, PCA9491_REG_CONTROL, 
							PCA9491_BIT_SENSE_R, val);
	if (ret < 0)
		goto error;

error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}


/* 2:1 Direct Charging Adjust CC MODE control */
static int pca9491_charge_adjust_ccmode(struct pca9491_charger *pca9491)
{
	int ret, iin;
	int vbatt, val;

	pr_info("%s: ======START=======\n", __func__);

	pca9491->charging_state = DC_STATE_ADJUST_CC;

	ret = pca9491_check_ccmode_status(pca9491);
	if (ret < 0)
		goto error;

	switch(ret) {
	case CCMODE_IIN_LOOP:
	case CCMODE_CHG_LOOP:
		/* Decrease TA current (50mA) */
		pca9491->ta_cur = pca9491->ta_cur - PDMSG_PPS_CUR_STEP;
		pr_info("%s: CC adjust End: ta_cur=%d\n", __func__, pca9491->ta_cur);
		/* Read VBAT ADC */
		vbatt = pca9491_read_adc(pca9491, ADCCH_VBAT);
		/* TA voltage = TA voltage + (VFLOAT - VBAT_ADC) */
		pca9491->ta_vol = pca9491->ta_vol + (pca9491->pdata->v_float - vbatt);
		val = pca9491->ta_vol/PDMSG_PPS_VOL_STEP;
		pca9491->ta_vol = val*PDMSG_PPS_VOL_STEP;
		if (pca9491->ta_vol > PCA9491_TA_MAX_VOL)
			pca9491->ta_vol = PCA9491_TA_MAX_VOL;
		pr_info("%s: CC adjust End: ta_vol=%d\n", __func__, pca9491->ta_vol);
		
		/* Send PD Message */
		ret = pca9491_send_pd_message(pca9491, PD_MSG_REQUEST_APDO);
		if (ret < 0)
			goto error;

		mutex_lock(&pca9491->lock);
		/* End TA voltage and current adjustment */
		/* go to CC mode */
		pca9491->charging_state = DC_STATE_START_CC;
		pca9491->timer_id = TIMER_PDMSG_SEND;
		pca9491->timer_period = PDMSG_WAIT_T;
		mutex_unlock(&pca9491->lock);
		schedule_delayed_work(&pca9491->work, msecs_to_jiffies(pca9491->timer_period));
		break;
		
	case CCMODE_VFLT_LOOP:
		/* go to CV mode */
		mutex_lock(&pca9491->lock);
		pca9491->timer_id = TIMER_ENTER_CVMODE;
		pca9491->timer_period = 0;
		mutex_unlock(&pca9491->lock);
		schedule_delayed_work(&pca9491->work, msecs_to_jiffies(pca9491->timer_period));
		break;

	case CCMODE_LOOP_INACTIVE:
		/* Check IIN ADC with IIN_CFG */
		iin = pca9491_read_adc(pca9491, ADCCH_IBUS);
		/* IIN_ADC > IIN_CFG -40mA ? */
		if (iin > (pca9491->pdata->iin_cfg - 40000)) {
			/* Input current is already over IIN_CFG */
			/* End TA voltage and current adjustment */
			/* go to CC mode */
			pca9491->charging_state = DC_STATE_START_CC;

			/* Read VBAT ADC */
			vbatt = pca9491_read_adc(pca9491, ADCCH_VBAT);
			/* TA voltage = TA voltage + (VFLOAT - VBAT_ADC) */
			pca9491->ta_vol = pca9491->ta_vol + (pca9491->pdata->v_float - vbatt);
			val = pca9491->ta_vol/PDMSG_PPS_VOL_STEP;
			pca9491->ta_vol = val*PDMSG_PPS_VOL_STEP;
			if (pca9491->ta_vol > PCA9491_TA_MAX_VOL)
				pca9491->ta_vol = PCA9491_TA_MAX_VOL;
			pr_info("%s: CC adjust End: IIN_ADC=%d, ta_vol=%d\n", __func__, iin, pca9491->ta_vol);	
		} else {
			/* IIN_ADC > pre_IIN_ADC + 20mA ? */
			if (iin > (pca9491->pre_iin_adc + 20000)) {
				/* TA voltage too low to enter TA CC mode, so we should increae TA voltage */
				pca9491->ta_vol = pca9491->ta_vol + PCA9491_TA_VOL_STEP_ADJ_CC;
				if (pca9491->ta_vol > PCA9491_TA_MAX_VOL)
					pca9491->ta_vol = PCA9491_TA_MAX_VOL;
				pr_info("%s: CC adjust Cont: ta_vol=%d\n", __func__, pca9491->ta_vol);
			} else {
				/* Check Max TA current */
				if (pca9491->ta_cur == pca9491->ta_max_cur) {
					/* TA current is already max value */
					/* Check TA voltage */
					if (pca9491->ta_vol == pca9491->ta_max_vol) {
						/* TA voltage is already max value */
						/* Change charging status to CC mode */
						pca9491->charging_state = DC_STATE_START_CC;
						pr_info("%s: CC adjust End: MAX value, ta_vol=%d, ta_cur=%d\n", 
							__func__, pca9491->ta_vol, pca9491->ta_cur);
					} else {
						/* TA voltage too low to enter TA CC mode, so we should increae TA voltage */
						pca9491->ta_vol = pca9491->ta_vol + PCA9491_TA_VOL_STEP_ADJ_CC;
						if (pca9491->ta_vol > PCA9491_TA_MAX_VOL)
							pca9491->ta_vol = PCA9491_TA_MAX_VOL;
						pr_info("%s: CC adjust Cont: ta_vol=%d\n", __func__, pca9491->ta_vol);
					}
				} else {
					/* Increase TA current (50mA) */
					pca9491->ta_cur = pca9491->ta_cur + PDMSG_PPS_CUR_STEP;
					if (pca9491->ta_cur > PCA9491_TA_MAX_CUR)
						pca9491->ta_cur = PCA9491_TA_MAX_CUR;
					pr_info("%s: CC adjust Cont: ta_cur=%d\n", __func__, pca9491->ta_cur);
				}
			}
		}

		/* Save the current iin adc  */
		pca9491->pre_iin_adc = iin;

		/* Send PD Message */
		ret = pca9491_send_pd_message(pca9491, PD_MSG_REQUEST_APDO);
		if (ret < 0)
			goto error;

		mutex_lock(&pca9491->lock);
		pca9491->timer_id = TIMER_PDMSG_SEND;
		pca9491->timer_period = PDMSG_WAIT_T;
		mutex_unlock(&pca9491->lock);
		schedule_delayed_work(&pca9491->work, msecs_to_jiffies(pca9491->timer_period));
		break;

	case CCMODE_VIN_UVLO:
		/* Stop Charging */
		ret = pca9491_set_charging(pca9491, false);
		if (ret < 0)
			goto error;
		break;

	default:
		break;
	}

	ret = 0;
	
error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}


/* 2:1 Direct Charging CC MODE control */
static int pca9491_charge_ccmode(struct pca9491_charger *pca9491)
{
	int ret, vbat;

	pr_info("%s: ======START=======\n", __func__);

	pca9491->charging_state = DC_STATE_CHECK_CC;

	ret = pca9491_check_ccmode_status(pca9491);
	if (ret < 0)
		goto error;

	switch(ret)	{
	case CCMODE_LOOP_INACTIVE:
		/* Read VBAT ADC */
		vbat = pca9491_read_adc(pca9491, ADCCH_VBAT);
		if (vbat < 0)
			goto error;
		
		/* Set timer */
		mutex_lock(&pca9491->lock);
		if (vbat < PCA9491_CC2_VBAT_MIN) {
			/* Set 60s timer */
			pca9491->timer_period = PCA9491_CCMODE_CHECK1_T;
		} else if (vbat < PCA9491_CC3_VBAT_MIN) {
			/* Set 10s timer */
			pca9491->timer_period = PCA9491_CCMODE_CHECK2_T;
		} else {
			/* Set 5s timer */
			pca9491->timer_period = PCA9491_CCMODE_CHECK3_T;
		}
		pca9491->timer_id = TIMER_CCMODE_CHECK;
		mutex_unlock(&pca9491->lock);
		schedule_delayed_work(&pca9491->work, msecs_to_jiffies(pca9491->timer_period));
		break;

	case CCMODE_VFLT_LOOP:
		/* go to CV mode */
		mutex_lock(&pca9491->lock);
		pca9491->timer_id = TIMER_ENTER_CVMODE;
		pca9491->timer_period = 0;
		mutex_unlock(&pca9491->lock);
		schedule_delayed_work(&pca9491->work, msecs_to_jiffies(pca9491->timer_period));
		break;

	case CCMODE_IIN_LOOP:
	case CCMODE_CHG_LOOP:
		/* Decrease TA current (50mA) */
		pca9491->ta_cur = pca9491->ta_cur - PDMSG_PPS_CUR_STEP;

		pr_info("%s: CC LOOP: ta_cur=%d\n", __func__, pca9491->ta_cur);
		/* Send PD Message */
		ret = pca9491_send_pd_message(pca9491, PD_MSG_REQUEST_APDO);
		if (ret < 0)
			goto error;

		mutex_lock(&pca9491->lock);
		pca9491->timer_id = TIMER_PDMSG_SEND;
		pca9491->timer_period = PDMSG_WAIT_T;
		mutex_unlock(&pca9491->lock);
		schedule_delayed_work(&pca9491->work, msecs_to_jiffies(pca9491->timer_period));
		break;			

	case CCMODE_VIN_UVLO:
		/* Stop Charging */
		ret = pca9491_set_charging(pca9491, false);
		if (ret < 0)
			goto error;
		break;

	default:
		break;
	}

	ret = 0;

error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/* 2:1 Direct Charging Start CV MODE control - Pre CV MODE */
static int pca9491_charge_start_cvmode(struct pca9491_charger *pca9491)
{
	int ret;

	pr_info("%s: ======START=======\n", __func__);

	pca9491->charging_state = DC_STATE_START_CV;
	
	ret = pca9491_check_cvmode_status(pca9491);

	if (ret < 0)
		goto error;
	
	switch(ret)	{
	case CVMODE_CHG_LOOP:
	case CVMODE_IIN_LOOP:
		/* Decrease TA current (50mA) */
		pca9491->ta_cur = pca9491->ta_cur - PDMSG_PPS_CUR_STEP;
		/* Send PD Message */
		ret = pca9491_send_pd_message(pca9491, PD_MSG_REQUEST_APDO);
		if (ret < 0)
			goto error;

		pr_info("%s: PreCV Cont: ta_cur=%d\n", __func__, pca9491->ta_cur);

		mutex_lock(&pca9491->lock);
		pca9491->timer_id = TIMER_PDMSG_SEND;
		pca9491->timer_period = PDMSG_WAIT_T;
		mutex_unlock(&pca9491->lock);
		schedule_delayed_work(&pca9491->work, msecs_to_jiffies(pca9491->timer_period));
		break;

	case CVMODE_VFLT_LOOP:
		/* Decrease TA voltage (20mV) */
		pca9491->ta_vol = pca9491->ta_vol - PCA9491_TA_VOL_STEP_PRE_CV;
		/* Send PD Message */
		ret = pca9491_send_pd_message(pca9491, PD_MSG_REQUEST_APDO);
		if (ret < 0)
			goto error;

		pr_info("%s: PreCV Cont: ta_vol=%d\n", __func__, pca9491->ta_vol);

		mutex_lock(&pca9491->lock);
		pca9491->timer_id = TIMER_PDMSG_SEND;
		pca9491->timer_period = PDMSG_WAIT_T;
		mutex_unlock(&pca9491->lock);
		schedule_delayed_work(&pca9491->work, msecs_to_jiffies(pca9491->timer_period));
		break;

	case CVMODE_LOOP_INACTIVE:
		/* Exit Pre CV mode */
		/* Go to CV mode */
		pr_info("%s: PreCV End: ta_cur=%d\n", __func__, pca9491->ta_cur);
		
		mutex_lock(&pca9491->lock);
		pca9491->timer_id = TIMER_CVMODE_CHECK;
		pca9491->timer_period = 0;
		mutex_unlock(&pca9491->lock);
		schedule_delayed_work(&pca9491->work, msecs_to_jiffies(pca9491->timer_period));
		break;

	case CVMODE_VIN_UVLO:
		/* Stop Charging */
		ret = pca9491_set_charging(pca9491, false);
		if (ret < 0)
			goto error;
		break;
		
	default:
		break;
	}

	ret = 0;
	
error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/* 2:1 Direct Charging CV MODE control */
static int pca9491_charge_cvmode(struct pca9491_charger *pca9491)
{
	int ret;

	pr_info("%s: ======START=======\n", __func__);

	pca9491->charging_state = DC_STATE_CHECK_CV;
	
	ret = pca9491_check_cvmode_status(pca9491);
	if (ret < 0)
		goto error;
	
	switch(ret)	{
	case CVMODE_CHG_DONE:
		/* Charging Done */
		/* Disable PCA9491 */
		ret = pca9491_set_charging(pca9491, false);
		if (ret < 0)
			goto error;

		/* Disable interrupt */
		disable_irq(pca9491->irq);
		
		/* Disable Chip */
		pca9491_enable_chip(pca9491, false);
		
		/* Change charging status */
		pca9491->charging_state = DC_STATE_CHARGING_DONE;
		
		/* Set TA voltage to fixed 5V */
		pca9491->ta_vol = 5000000;
		/* Set TA current to maximum 3A */
		pca9491->ta_cur = 3000000;
		
		/* Send PD Message */
		pca9491->ta_objpos = 1;	// PDO1 - fixed 5V
		ret = pca9491_send_pd_message(pca9491, PD_MSG_REQUEST_FIXED_PDO);
		if (ret < 0)
			goto error;

		pr_info("%s: CV Done\n", __func__);

		mutex_lock(&pca9491->lock);
		pca9491->timer_id = TIMER_PDMSG_SEND;
		pca9491->timer_period = PDMSG_WAIT_T;
		mutex_unlock(&pca9491->lock);
		schedule_delayed_work(&pca9491->work, msecs_to_jiffies(pca9491->timer_period));
		break;

	case CVMODE_CHG_LOOP:
	case CVMODE_IIN_LOOP:
		/* Decrease TA current (50mA) */
		pca9491->ta_cur = pca9491->ta_cur - PDMSG_PPS_CUR_STEP;
		/* Send PD Message */
		ret = pca9491_send_pd_message(pca9491, PD_MSG_REQUEST_APDO);
		if (ret < 0)
			goto error;

		pr_info("%s: CV LOOP, Cont: ta_cur=%d\n", __func__, pca9491->ta_cur);

		mutex_lock(&pca9491->lock);
		pca9491->timer_id = TIMER_PDMSG_SEND;
		pca9491->timer_period = PDMSG_WAIT_T;
		mutex_unlock(&pca9491->lock);
		schedule_delayed_work(&pca9491->work, msecs_to_jiffies(pca9491->timer_period));
		break;

	case CVMODE_VFLT_LOOP:
		/* Decrease TA voltage */
		pca9491->ta_vol = pca9491->ta_vol - PDMSG_PPS_VOL_STEP;
		/* Send PD Message */
		ret = pca9491_send_pd_message(pca9491, PD_MSG_REQUEST_APDO);
		if (ret < 0)
			goto error;

		pr_info("%s: CV VFLOAT, Cont: ta_vol=%d\n", __func__, pca9491->ta_vol);

		mutex_lock(&pca9491->lock);
		pca9491->timer_id = TIMER_PDMSG_SEND;
		pca9491->timer_period = PDMSG_WAIT_T;
		mutex_unlock(&pca9491->lock);
		schedule_delayed_work(&pca9491->work, msecs_to_jiffies(pca9491->timer_period));
		break;

	case CVMODE_LOOP_INACTIVE:
		/* Set timer */
		mutex_lock(&pca9491->lock);
		pca9491->timer_id = TIMER_CVMODE_CHECK;
		pca9491->timer_period = PCA9491_CVMODE_CHECK_T;
		mutex_unlock(&pca9491->lock);
		schedule_delayed_work(&pca9491->work, msecs_to_jiffies(pca9491->timer_period));
		break;

	case CVMODE_VIN_UVLO:
		/* Stop Charging */
		ret = pca9491_set_charging(pca9491, false);
		if (ret < 0)
			goto error;
		break;

	default:
		break;
	}

	ret = 0;
	
error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}


/* Preset TA voltage and current for Direct Charging Mode */
static int pca9491_preset_dcmode(struct pca9491_charger *pca9491)
{
	int vbat;
	unsigned int val;
	int ret = 0;

	pr_info("%s: ======START=======\n", __func__);

	pca9491->charging_state = DC_STATE_PRESET_DC;

	/* Read VBAT ADC */
	vbat = pca9491_read_adc(pca9491, ADCCH_VBAT);

	/* Save Charging Configuration */

	/* clear the previous IIN ADC value */
	pca9491->pre_iin_adc = 0;

	/* Set TA voltage to VBAT_ADC + 100 mV */
	pca9491->ta_vol = MAX(PCA9491_DC_VBAT_MIN, (vbat + PCA9491_TA_VOL_PRE_OFFSET));
	val = pca9491->ta_vol/PDMSG_PPS_VOL_STEP;	/* PPS voltage resolution is 20mV */
	pca9491->ta_vol = val*PDMSG_PPS_VOL_STEP;
	/* Set TA current to IIN */
	pca9491->ta_cur = pca9491->pdata->iin_cfg;
	val = pca9491->ta_cur/PDMSG_PPS_CUR_STEP;	/* PPS current resolution is 50mV */
	pca9491->ta_cur = val*PDMSG_PPS_CUR_STEP;
	pca9491->ta_objpos = 0;	/* Search the proper object position of PDO */
	/* Set max TA voltage and current */
	pca9491->ta_max_vol = PCA9491_TA_MAX_VOL;
	pca9491->ta_max_cur = PCA9491_TA_MAX_CUR;
	
	ret = pca9491_send_pd_message(pca9491, PD_MSG_REQUEST_APDO);
	if (ret < 0)
		goto error;

	pr_info("%s: Preset DC, ta_vol=%d, ta_cur=%d\n", 
		__func__, pca9491->ta_vol, pca9491->ta_cur);

	mutex_lock(&pca9491->lock);
	pca9491->timer_id = TIMER_PDMSG_SEND;
	pca9491->timer_period = PDMSG_WAIT_T;
	mutex_unlock(&pca9491->lock);
	schedule_delayed_work(&pca9491->work, msecs_to_jiffies(pca9491->timer_period));

error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}


/* Preset direct charging configuration */
static int pca9491_preset_config(struct pca9491_charger *pca9491)
{
	int ret = 0;
	
	pr_info("%s: ======START=======\n", __func__);
	
	pca9491->charging_state = DC_STATE_PRESET_DC;
	
	/* Set IIN_CFG */
	ret = pca9491_set_input_current(pca9491, pca9491->pdata->iin_cfg);
	if (ret < 0)
		goto error;

	/* Set ICHG_CFG */
	ret = pca9491_set_charging_current(pca9491, pca9491->pdata->ichg_cfg);
	if (ret < 0)
		goto error;

	/* Set VFLOAT */
	ret = pca9491_set_vfloat(pca9491, pca9491->pdata->v_float);
	if (ret < 0)
		goto error;

	/* Enable PCA9491 */	
	ret = pca9491_set_charging(pca9491, true);
	if (ret < 0)
		goto error;

	/* Enable interrupt */
	enable_irq(pca9491->irq);
	
	/* Go to Adjust CC mode after 100ms*/
	mutex_lock(&pca9491->lock);
	pca9491->timer_id = TIMER_ENTER_ADJ_CCMODE;
	pca9491->timer_period = 100;
	mutex_unlock(&pca9491->lock);
	schedule_delayed_work(&pca9491->work, msecs_to_jiffies(pca9491->timer_period));
	ret = 0;
	
error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret; 
}


/* Enter direct charging algorithm */
static int pca9491_start_direct_charging(struct pca9491_charger *pca9491)
{
	int ret;

	pr_info("%s: =========START=========\n", __func__);

	/* init set up */
	ret = pca9491_init_setup(pca9491);
	if (ret < 0)
		goto error;

	/* wake lock */
	wake_lock(&pca9491->monitor_wake_lock);

	/* Preset charging configuration and TA condition */
	ret = pca9491_preset_dcmode(pca9491);
	if (ret < 0)
		pca9491_stop_charging(pca9491);
error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}


/* Check Vbat minimum level to start direct charging */
static int pca9491_check_vbatmin(struct pca9491_charger *pca9491)
{
	int ret, vbat;

	pr_info("%s: =========START=========\n", __func__);

	pca9491->charging_state = DC_STATE_CHECK_VBAT;

	/* Check Vbat */
	vbat = pca9491_read_adc(pca9491, ADCCH_VBAT);
	if (vbat > PCA9491_DC_VBAT_MIN)	{
		/* Start Direct Charging */
		/* Read switching charger status */
		int enable;
		ret = pca9491_get_switching_charger_is_enabled(&enable);
		if (ret < 0) {
			/* Start Direct Charging again after 1sec */
			mutex_lock(&pca9491->lock);
			pca9491->timer_id = TIMER_VBATMIN_CHECK;
			pca9491->timer_period = PCA9491_VBATMIN_CHECK_T;
			mutex_unlock(&pca9491->lock);
			schedule_delayed_work(&pca9491->work, msecs_to_jiffies(pca9491->timer_period));
			goto error;
		}
		
		if (enable == 0) {
			/* already disabled switching charger */
			/* enable direct charging */
			ret = pca9491_start_direct_charging(pca9491);
			if (ret < 0) {
				/* Start Direct Charging again after 1sec */
				mutex_lock(&pca9491->lock);
				pca9491->timer_id = TIMER_VBATMIN_CHECK;
				pca9491->timer_period = PCA9491_VBATMIN_CHECK_T;
				mutex_unlock(&pca9491->lock);
				schedule_delayed_work(&pca9491->work, msecs_to_jiffies(pca9491->timer_period));
				goto error;
			}
		} else {
			/* now switching charger is enabled */
			/* disable switching charger first */
			ret = pca9491_set_switching_charger(false, 0, 0, 0);

			/* Wait 1sec for stopping switching charger */
			mutex_lock(&pca9491->lock);
			pca9491->timer_id = TIMER_VBATMIN_CHECK;
			pca9491->timer_period = PCA9491_VBATMIN_CHECK_T;
			mutex_unlock(&pca9491->lock);
			schedule_delayed_work(&pca9491->work, msecs_to_jiffies(pca9491->timer_period));
		}
	} else {
		/* Start 1sec timer for battery check */
		mutex_lock(&pca9491->lock);
		pca9491->timer_id = TIMER_VBATMIN_CHECK;
		pca9491->timer_period = PCA9491_VBATMIN_CHECK_T;
		mutex_unlock(&pca9491->lock);
		schedule_delayed_work(&pca9491->work, msecs_to_jiffies(pca9491->timer_period));
		ret = 0;
	}

error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/* Get the current rtc time */
static int get_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
		       __FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
		       CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
		       CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}
	rtc_tm_to_time(&tm, now_tm_sec);

 close_time:
	rtc_class_close(rtc);
	return rc;
}

/* delayed work function for charging timer */
static void pca9491_timer_work(struct work_struct *work)
{
	struct pca9491_charger *pca9491 = container_of(work, struct pca9491_charger,
						 work.work);
	int ret;

	get_current_time(&pca9491->last_update_time);

	pr_info("%s: timer id=%d, charging_state=%d, last_update_time=%lu\n", 
		__func__, pca9491->timer_id, pca9491->charging_state, pca9491->last_update_time);
	
	switch (pca9491->timer_id) {
	case TIMER_VBATMIN_CHECK:
		pca9491_check_vbatmin(pca9491);
		break;

	case TIMER_ENTER_ADJ_CCMODE:
		ret = pca9491_charge_adjust_ccmode(pca9491);
		if (ret < 0)
			goto error;
		break;			
					
	case TIMER_CCMODE_CHECK:
		ret = pca9491_charge_ccmode(pca9491);
		if (ret < 0)
			goto error;
		break;

	case TIMER_ENTER_CVMODE:
		/* Enter Pre-CV mode */
		ret = pca9491_charge_start_cvmode(pca9491);
		if (ret < 0)
			goto error;
		break;

	case TIMER_CVMODE_CHECK:
		ret = pca9491_charge_cvmode(pca9491);
		if (ret < 0)
			goto error;
		break;

	case TIMER_PDMSG_SEND:
		/* Enter here after sending PD message */
		/* Changing TA voltage */
		
		/* check the charging status */
		if (pca9491->charging_state == DC_STATE_PRESET_DC) {
			/* preset pca9491 configuration */
			ret = pca9491_preset_config(pca9491);
			if (ret < 0)
				goto error;
		} else if (pca9491->charging_state == DC_STATE_ADJUST_CC) {
			/* Adjust CC mode */
			ret = pca9491_charge_adjust_ccmode(pca9491);
			if (ret < 0)
				goto error;
		} else if (pca9491->charging_state == DC_STATE_START_CC) {
			/* Start CC mode */
			/* interrupt enable here if we use interrupt method */
			ret = pca9491_charge_ccmode(pca9491);
			if (ret < 0)
				goto error;
		} else if (pca9491->charging_state == DC_STATE_CHECK_CC) {
			/* Check CC mode */
			ret = pca9491_charge_ccmode(pca9491);
			if (ret < 0)
				goto error;
		} else if (pca9491->charging_state == DC_STATE_START_CV) {
			/* Start CV mode - pre CV mode */
			ret = pca9491_charge_start_cvmode(pca9491);
			if (ret < 0)
				goto error;
		} else if (pca9491->charging_state == DC_STATE_CHECK_CV) {
			/* Check CV mode */
			ret = pca9491_charge_cvmode(pca9491);
			if (ret < 0)
				goto error;
		} else if (pca9491->charging_state == DC_STATE_CHARGING_DONE) {
			/* Timer ID is none */
			mutex_lock(&pca9491->lock);
			pca9491->timer_id = TIMER_ID_NONE;
			mutex_unlock(&pca9491->lock);
			/* Enable Switching Charger */
			ret = pca9491_set_switching_charger(true, pca9491->pdata->iin_cfg, 
												pca9491->pdata->ichg_cfg, 
												pca9491->pdata->v_float);
			if (ret < 0)
				goto error;
			/* wake unlock */
			wake_unlock(&pca9491->monitor_wake_lock);
		}
		break;
		
	default:
		break;
	}

error:
	pca9491_stop_charging(pca9491);
	return;
}


/* delayed work function for pps periodic timer */
static void pca9491_pps_request_work(struct work_struct *work)
{
	struct pca9491_charger *pca9491 = container_of(work, struct pca9491_charger,
						 pps_work.work);

	int ret = 0;
	
	pr_info("%s: pps_work_start\n", __func__);

	/* Send PD message */
	ret = pca9491_send_pd_message(pca9491, PD_MSG_REQUEST_APDO);
	pr_info("%s: End, ret=%d\n", __func__, ret);
}

static int pca9491_hw_init(struct pca9491_charger *pca9491)
{
	int ret;

	pr_info("%s: =========START=========\n", __func__);

	/* Set EN pin to Low - disable */
	ret = gpio_direction_output(pca9491->pdata->en_gpio, 0);

	return ret;
}

/* pca9491 interrupt handler */
static irqreturn_t pca9491_interrupt_handler(int irq, void *data)
{
	struct pca9491_charger *pca9491 = data;
	u8 event[4];			/* EVENT_1_MASK, EVENT_2_MASK, EVENT_1, EVENT_2 */
	u8 masked_event[2];
	bool handled = false;
	int ret;

	pr_info("%s: =========START=========\n", __func__);

	/* Read EVENT_1, 2 and mask*/
	ret = regmap_bulk_read(pca9491->regmap, (PCA9491_REG_EVENT_1_MASK | PCA9491_REG_BIT_AUTO_INC), 
							event, 4);
	if (ret < 0) {
		pr_info("%s: reading EVENT_X failed=%d\n", __func__, ret);
		return IRQ_NONE;
	}

	pr_info("%s: event_1=0x%2x, event_2=0x%2x\n", __func__, event[2], event[3]);


	/* Check Interrupt */
	masked_event[0] = event[2] & !event[0];	/* EVENT 1 */
	masked_event[1] = event[3] & !event[1];	/* EVENT 2 */

	if (pca9491_check_fault(masked_event) == true) {
		/* Stop Charging */
		pca9491_stop_charging(pca9491);
		handled = true;
	}

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

/* initialize interrupt */
static int pca9491_irq_init(struct pca9491_charger *pca9491,
			   struct i2c_client *client)
{
	const struct pca9491_platform_data *pdata = pca9491->pdata;
	int ret, irq;

	pr_info("%s: =========START=========\n", __func__);

	irq = gpio_to_irq(pdata->irq_gpio);
	pca9491->irq = irq;
	
	ret = gpio_request_one(pdata->irq_gpio, GPIOF_IN, client->name);
	if (ret < 0)
		goto fail;

	ret = request_threaded_irq(irq, NULL, pca9491_interrupt_handler,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   client->name, pca9491);
	if (ret < 0)
		goto fail_gpio;

	/* Disable interrupt */
	disable_irq(irq);
	
	client->irq = irq;
	return 0;

fail_gpio:
	gpio_free(pdata->irq_gpio);
fail:
	client->irq = 0;
	return ret;
}


/*
 * Returns the constant charge current programmed
 * into the charger in uA.
 */
static int get_const_charge_current(struct pca9491_charger *pca9491)
{
	int ret, intval;
	unsigned int val;

	if (!pca9491->mains_online)
		return -ENODATA;

	ret = regmap_read(pca9491->regmap, PCA9491_REG_IBAT_REG, &val);
	if (ret < 0)
		return ret;

	intval = val * IBAT_REG_STEP;

	return intval;
}

/*
 * Returns the constant charge voltage programmed
 * into the charger in uV.
 */
static int get_const_charge_voltage(struct pca9491_charger *pca9491)
{
	int ret, intval;
	unsigned int val;

	if (!pca9491->mains_online)
		return -ENODATA;

	ret = regmap_read(pca9491->regmap, PCA9491_REG_VBAT_REG, &val);
	if (ret < 0)
		return ret;
	
	intval = (val * VBAT_REG_STEP) + VBAT_REG_OFFSET;

	return intval;
}

/*
 * Returns the input current limit programmed
 * into the charger in uA.
 */
static int get_input_current_limit(struct pca9491_charger *pca9491)
{
	int ret, intval;
	unsigned int val;

	if (!pca9491->mains_online)
		return -ENODATA;

	ret = regmap_read(pca9491->regmap, PCA9491_REG_IBUS_REG, &val);
	if (ret < 0)
		return ret;

	intval = val * IBUS_REG_STEP;

	return intval;
}

/*
 * Returns the enable or disable value.
 * into 1 or 0.
 */
static int get_charging_enabled(struct pca9491_charger *pca9491)
{
	int ret, intval;
	unsigned int val;
	
	ret = regmap_read(pca9491->regmap, PCA9491_REG_CONTROL, &val);
	if (ret < 0)
		return ret;

	intval = (val & PCA9491_BIT_CHG_EN) ? 1 : 0;

	return intval;
}

static int pca9491_mains_set_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     const union power_supply_propval *val)
{
	struct pca9491_charger *pca9491 = power_supply_get_drvdata(psy);
	int ret;

	pr_info("%s: =========START=========\n", __func__);

	switch (prop) {
	/* Todo - Insert code */
	/* It needs modification by a customer */
	/* The customer make a decision to start charging and stop charging property */
	
	case POWER_SUPPLY_PROP_ONLINE:	/* need to change property */
		if (val->intval == 0) {
			// Stop Direct charging 
			cancel_delayed_work(&pca9491->work);
			cancel_delayed_work(&pca9491->pps_work);
			mutex_lock(&pca9491->lock);
			pca9491->timer_id = TIMER_ID_NONE;
			pca9491->timer_period = 0;
			mutex_unlock(&pca9491->lock);
			wake_unlock(&pca9491->monitor_wake_lock);

			ret = pca9491_set_charging(pca9491, false);
			if (ret < 0)
				goto error;
		} else {
			// Start Direct charging
			ret = pca9491_start_direct_charging(pca9491);
			if (ret < 0)
				goto error;
		}
		break;

	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		if (val->intval == 0) {
			// Stop Direct Charging
			cancel_delayed_work(&pca9491->work);
			cancel_delayed_work(&pca9491->pps_work);
			mutex_lock(&pca9491->lock);
			pca9491->timer_id = TIMER_ID_NONE;
			pca9491->timer_period = 0;
			mutex_unlock(&pca9491->lock);
			wake_unlock(&pca9491->monitor_wake_lock);

			ret = pca9491_set_charging(pca9491, false);
			if (ret < 0)
				goto error;
		} else {
			// Start Direct Charging
			/* Start 1sec timer for battery check */
			mutex_lock(&pca9491->lock);
			pca9491->timer_id = TIMER_VBATMIN_CHECK;
			pca9491->timer_period = 5000;	/* The dealy time for PD state goes to PE_SNK_STATE */
			mutex_unlock(&pca9491->lock);
			schedule_delayed_work(&pca9491->work, msecs_to_jiffies(pca9491->timer_period));
			ret = 0;
		}
		break;

	default:
		ret = -EINVAL;
		break;
	}

error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}


static int pca9491_mains_get_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     union power_supply_propval *val)
{
	struct pca9491_charger *pca9491 = power_supply_get_drvdata(psy);
	int ret;

	pr_info("%s: =========START=========\n", __func__);

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = pca9491->mains_online;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = get_const_charge_voltage(pca9491);
		if (ret < 0)
			return ret;
		else
			val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = get_const_charge_current(pca9491);
		if (ret < 0)
			return ret;
		else
			val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		ret = get_charging_enabled(pca9491);
		if (ret < 0)
			return ret;
		else
			val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = get_input_current_limit(pca9491);
		if (ret < 0)
			return ret;
		else
			val->intval = ret;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property pca9491_mains_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
};


static const struct regmap_config pca9491_regmap = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= PCA9491_MAX_REGISTER,
};


static const struct power_supply_desc pca9491_mains_desc = {
	.name		= "pca9491-mains",
	.type		= POWER_SUPPLY_TYPE_MAINS,
	.get_property	= pca9491_mains_get_property,
	.set_property 	= pca9491_mains_set_property,
	.properties	= pca9491_mains_properties,
	.num_properties	= ARRAY_SIZE(pca9491_mains_properties),
};


#if defined(CONFIG_OF)
static int of_pca9491_dt(struct device *dev, struct pca9491_platform_data *pdata)
{
	struct device_node *np_pca9491 = dev->of_node;
	int ret;
	if(!np_pca9491)
		return -EINVAL;

	/* irq gpio */
	pdata->irq_gpio = of_get_named_gpio(np_pca9491, "pca9491,irq-gpio", 0);
	pr_info("%s: irq-gpio: %u \n", __func__, pdata->irq_gpio);

	/* en gpio */
	pdata->en_gpio = of_get_named_gpio(np_pca9491, "pca9491,en-gpio", 0);
	pr_info("%s: en-gpio: %u \n", __func__, pdata->en_gpio);
	
	/* input current limit */
	ret = of_property_read_u32(np_pca9491, "pca9491,input-current-limit",
						   &pdata->iin_cfg);
	if (ret) {
		pr_info("%s: pca9491,input-current-limit is Empty\n", __func__);
		pdata->iin_cfg = PCA9491_IBUS_REG_DFT;
	}
	pr_info("%s: pca9491,iin_cfg is %d\n", __func__, pdata->iin_cfg);

	/* charging current */
	ret = of_property_read_u32(np_pca9491, "pca9491,charging-current",
							   &pdata->ichg_cfg);
	if (ret) {
		pr_info("%s: pca9491,charging-current is Empty\n", __func__);
		pdata->ichg_cfg = PCA9491_IBAT_REG_DFT;
	}
	pr_info("%s: pca9491,ichg_cfg is %d\n", __func__, pdata->ichg_cfg);

	/* charging float voltage */
	ret = of_property_read_u32(np_pca9491, "pca9491,float-voltage",
							   &pdata->v_float);
	if (ret) {
		pr_info("%s: pca9491,float-voltage is Empty\n", __func__);
		pdata->v_float = PCA9491_VBAT_REG_DFT;
	}
	pr_info("%s: pca9491,v_float is %d\n", __func__, pdata->v_float);

	/* top-off current */
	ret = of_property_read_u32(np_pca9491, "pca9491,topoff-current",
							   &pdata->topoff_cur);
	if (ret) {
		pr_info("%s: pca9491,topoff current is Empty\n", __func__);
		pdata->topoff_cur = PCA9491_TOPOFF_CURRENT_DFT;
	}
	pr_info("%s: pca9491,topoff current is %d\n", __func__, pdata->topoff_cur);

	/* sense resistance */
	ret = of_property_read_u32(np_pca9491, "pca9491,sense-resistor",
							   &pdata->snsres);
	if (ret) {
		pr_info("%s: pca9491,sense-resistor is Empty\n", __func__);
		pdata->snsres = PCA9491_SENSE_R_DFT;
	}
	pr_info("%s: pca9491,snsres is %d\n", __func__, pdata->snsres);

	/* vbus over voltage threshold */
	ret = of_property_read_u32(np_pca9491, "pca9491,vbus-ovp",
							   &pdata->vbus_ovp);
	if (ret) {
		pr_info("%s: pca9491,vbus-ovp is Empty\n", __func__);
		pdata->vbus_ovp = PCA9491_VBUS_OVP_DFT;
	}
	pr_info("%s: pca9491,vbus-ovp is %d\n", __func__, pdata->vbus_ovp);

	/* vout regulator loop threshold */
	ret = of_property_read_u32(np_pca9491, "pca9491,vout-reg",
							   &pdata->vout_reg);
	if (ret) {
		pr_info("%s: pca9491,vout-reg is Empty\n", __func__);
		pdata->vout_reg = PCA9491_VOUT_REG_DFT;
	}
	pr_info("%s: pca9491,vout-reg is %d\n", __func__, pdata->vout_reg);

	/* vdrop over voltage threshold */
	ret = of_property_read_u32(np_pca9491, "pca9491,vdrop-ovp",
							   &pdata->vdrop_ovp);
	if (ret) {
		pr_info("%s: pca9491,vdrop-ovp is Empty\n", __func__);
		pdata->vdrop_ovp = PCA9491_VDROP_OVP_DFT;
	}
	pr_info("%s: pca9491,vdrop-ovp is %d\n", __func__, pdata->vdrop_ovp);

	/* vdrop alarm threshold */
	ret = of_property_read_u32(np_pca9491, "pca9491,vdrop-alarm",
							   &pdata->vdrop_alarm);
	if (ret) {
		pr_info("%s: pca9491,vdrop-alarm is Empty\n", __func__);
		pdata->vdrop_alarm = PCA9491_VDROP_ALARM_DFT;
	}
	pr_info("%s: pca9491,vdrop-alarm is %d\n", __func__, pdata->vdrop_alarm);

	return 0;
}
#else
static int of_pca9491_dt(struct device *dev, struct pca9491_platform_data *pdata)
{
	return 0;
}
#endif /* CONFIG_OF */

#ifdef CONFIG_USBPD_PHY_QCOM
static int pca9491_usbpd_setup(struct pca9491_charger *pca9491)
{
	int ret = 0;
	const char *pd_phandle = "usbpd-phy";

	pca9491->pd = devm_usbpd_get_by_phandle(pca9491->dev, pd_phandle);

	if (IS_ERR(pca9491->pd)) {
		pr_err("get_usbpd phandle failed (%ld)\n",
				PTR_ERR(pca9491->pd));
		return PTR_ERR(pca9491->pd);
	}

	return ret;
}
#endif

static int pca9491_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{	
	static char *battery[] = { "pca9491-battery" };
	struct power_supply_config mains_cfg = {};
	struct pca9491_platform_data *pdata;
	struct device *dev = &client->dev;
	struct pca9491_charger *pca9491_chg;
	int ret;

	pr_info("%s: =========START=========\n", __func__);

	pca9491_chg = devm_kzalloc(dev, sizeof(*pca9491_chg), GFP_KERNEL);
	if (!pca9491_chg)
		return -ENOMEM;

#if defined(CONFIG_OF)
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev, sizeof(struct pca9491_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory \n");
			return -ENOMEM;
		}

		ret = of_pca9491_dt(&client->dev, pdata);
		if (ret < 0){
			dev_err(&client->dev, "Failed to get device of_node \n");
			return -ENOMEM;
		}

		client->dev.platform_data = pdata;
	} else {
		pdata = client->dev.platform_data;
	}
#else
	pdata = dev->platform_data;
#endif
	if (!pdata)
		return -EINVAL;

	i2c_set_clientdata(client, pca9491_chg);

	mutex_init(&pca9491_chg->lock);
	pca9491_chg->dev = &client->dev;
	pca9491_chg->pdata = pdata;
	pca9491_chg->charging_state = DC_STATE_NO_CHARGING;

	wake_lock_init(&pca9491_chg->monitor_wake_lock, WAKE_LOCK_SUSPEND,
		       "pca9491-charger-monitor");

	/* initialize work */
	INIT_DELAYED_WORK(&pca9491_chg->work, pca9491_timer_work);
	pca9491_chg->timer_id = TIMER_ID_NONE;
	pca9491_chg->timer_period = 0;

	INIT_DELAYED_WORK(&pca9491_chg->pps_work, pca9491_pps_request_work);

	pca9491_chg->regmap = devm_regmap_init_i2c(client, &pca9491_regmap);
	if (IS_ERR(pca9491_chg->regmap))
		return PTR_ERR(pca9491_chg->regmap);

#ifdef CONFIG_USBPD_PHY_QCOM
	if (pca9491_usbpd_setup(pca9491_chg)) {
		dev_err(dev, "Error usbpd setup!\n");
		pca9491_chg->pd = NULL;
	}
#endif

	ret = pca9491_hw_init(pca9491_chg);
	if (ret < 0)
		return ret;

	mains_cfg.supplied_to = battery;
	mains_cfg.num_supplicants = ARRAY_SIZE(battery);
	mains_cfg.drv_data = pca9491_chg;
	pca9491_chg->mains = power_supply_register(dev, &pca9491_mains_desc,
					   &mains_cfg);

	if (IS_ERR(pca9491_chg->mains))
		return PTR_ERR(pca9491_chg->mains);

	/*
	 * Interrupt pin is optional. If it is connected, we setup the
	 * interrupt support here.
	 */
	if (pdata->irq_gpio >= 0) {
		ret = pca9491_irq_init(pca9491_chg, client);
		if (ret < 0) {
			dev_warn(dev, "failed to initialize IRQ: %d\n", ret);
			dev_warn(dev, "disabling IRQ support\n");
		}
	}

	pr_info("%s: =========END=========\n", __func__);

	return 0;
}

static int pca9491_remove(struct i2c_client *client)
{
	struct pca9491_charger *pca9491_chg = i2c_get_clientdata(client);

	if (client->irq) {
		free_irq(client->irq, pca9491_chg);
		gpio_free(pca9491_chg->pdata->irq_gpio);
	}

	wake_lock_destroy(&pca9491_chg->monitor_wake_lock);
	power_supply_unregister(pca9491_chg->mains);
	return 0;
}

static const struct i2c_device_id pca9491_id[] = {
	{ "pca9491", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pca9491_id);

#if defined(CONFIG_OF)
static struct of_device_id pca9491_i2c_dt_ids[] = {
	{ .compatible = "nxp,pca9491" },
	{ },
};
MODULE_DEVICE_TABLE(of, pca9491_i2c_dt_ids);
#endif /* CONFIG_OF */

#if defined(CONFIG_PM)
static void pca9491_check_and_update_charging_timer(struct pca9491_charger *pca9491)
{
	unsigned long current_time = 0, next_update_time, time_left;

	get_current_time(&current_time);
	
	if (pca9491->timer_id != TIMER_ID_NONE)	{
		next_update_time = pca9491->last_update_time + (pca9491->timer_period / 1000);	// unit is second

		pr_info("%s: current_time=%ld, next_update_time=%ld\n", __func__, current_time, next_update_time);

		if (next_update_time > current_time)
			time_left = next_update_time - current_time;
		else
			time_left = 0;

		mutex_lock(&pca9491->lock);
		pca9491->timer_period = time_left * 1000;	// ms unit
		mutex_unlock(&pca9491->lock);
		schedule_delayed_work(&pca9491->work, msecs_to_jiffies(pca9491->timer_period));
		pr_info("%s: timer_id=%d, time_period=%ld\n", __func__, pca9491->timer_id, pca9491->timer_period);
	}
	pca9491->last_update_time = current_time;
}


static int pca9491_suspend(struct device *dev)
{
	struct pca9491_charger *pca9491 = dev_get_drvdata(dev);

	pr_info("%s: cancel delayed work\n", __func__);

	/* cancel delayed_work */
	cancel_delayed_work(&pca9491->work);
	return 0;
}

static int pca9491_resume(struct device *dev)
{
	struct pca9491_charger *pca9491 = dev_get_drvdata(dev);

	pr_info("%s: update_timer\n", __func__);

	/* Update the current timer */
	pca9491_check_and_update_charging_timer(pca9491);

	return 0;
}
#else
#define pca9491_suspend		NULL
#define pca9491_resume		NULL
#endif

const struct dev_pm_ops pca9491_pm_ops = {
	.suspend = pca9491_suspend,
	.resume = pca9491_resume,
};

static struct i2c_driver pca9491_driver = {
	.driver = {
		.name = "pca9491",
#if defined(CONFIG_OF)
		.of_match_table = pca9491_i2c_dt_ids,
#endif /* CONFIG_OF */
#if defined(CONFIG_PM)
		.pm = &pca9491_pm_ops,
#endif
	},
	.probe        = pca9491_probe,
	.remove       = pca9491_remove,
	.id_table     = pca9491_id,
};

module_i2c_driver(pca9491_driver);

MODULE_AUTHOR("Clark Kim <clark.kim@nxp.com>");
MODULE_DESCRIPTION("PCA9491 charger driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
