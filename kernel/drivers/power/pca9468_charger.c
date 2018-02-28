/*
 * Driver for the NXP PCA9468 battery charger.
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

#include <linux/power/pca9468_charger.h>

#if defined (CONFIG_OF)
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif /* CONFIG_OF */

#include <linux/wakelock.h>
#ifdef CONFIG_USBPD_PHY_QCOM
#include <linux/usb/usbpd.h>		// Use Qualcomm USBPD PHY
#endif

#define CONFIG_USBPD_SUPPORT_PPS	// Support USBPD 3.0 APDO
#define CONFIG_PCA9468_HW_REV_B0	// HW rev B0

#define BITS(_end, _start) ((BIT(_end) - BIT(_start)) + BIT(_end))
#define MASK2SHIFT(_mask)	__ffs(_mask)
#define MIN(a, b)	   ((a < b) ? (a):(b))

//
// Register Map
//
#define PCA9468_REG_DEVICE_INFO 		0x00	// Device ID, revision
#define PCA9468_BIT_DEV_REV				BITS(7,4)
#define PCA9468_BIT_DEV_ID				BITS(3,0)

#define PCA9468_REG_INT1				0x01	// Interrupt register
#define PCA9468_BIT_V_OK_INT			BIT(7)
#define PCA9468_BIT_NTC_TEMP_INT		BIT(6)
#define PCA9468_BIT_CHG_PHASE_INT		BIT(5)
#define PCA9468_BIT_CTRL_LIMIT_INT		BIT(3)
#define PCA9468_BIT_TEMP_REG_INT		BIT(2)
#define PCA9468_BIT_ADC_DONE_INT		BIT(1)
#define PCA9468_BIT_TIMER_INT			BIT(0)

#define PCA9468_REG_INT1_MSK			0x02	// INT mask for INT1 register
#define PCA9468_BIT_V_OK_M				BIT(7)
#define PCA9468_BIT_NTC_TEMP_M			BIT(6)
#define PCA9468_BIT_CHG_PHASE_M			BIT(5)
#define PCA9468_BIT_CTRL_LIMIT_M		BIT(3)
#define PCA9468_BIT_TEMP_REG_M			BIT(2)
#define PCA9468_BIT_ADC_DONE_M			BIT(1)
#define PCA9468_BIT_TIMER_M				BIT(0)

#define PCA9468_REG_INT1_STS			0x03	// INT1 status regsiter
#define PCA9468_BIT_V_OK_STS			BIT(7)
#define PCA9468_BIT_NTC_TEMP_STS		BIT(6)
#define PCA9468_BIT_CHG_PHASE_STS		BIT(5)
#define PCA9468_BIT_CTRL_LIMIT_STS		BIT(3)
#define PCA9468_BIT_TEMP_REG_STS		BIT(2)
#define PCA9468_BIT_ADC_DONE_STS		BIT(1)
#define PCA9468_BIT_TIMER_STS			BIT(0)

#define PCA9468_REG_STS_A				0x04	// INT1 status register
#define PCA9468_BIT_IIN_LOOP_STS		BIT(7)
#define PCA9468_BIT_CHG_LOOP_STS		BIT(6)
#define PCA9468_BIT_VFLT_LOOP_STS		BIT(5)
#define PCA9468_BIT_CFLY_SHORT_STS		BIT(4)
#define PCA9468_BIT_VOUT_UV_STS			BIT(3)
#define PCA9468_BIT_VBAT_OV_STS			BIT(2)
#define PCA9468_BIT_VIN_OV_STS			BIT(1)
#define PCA9468_BIT_VIN_UV_STS			BIT(0)

#define PCA9468_REG_STS_B				0x05	// INT1 status register
#define PCA9468_BIT_BATT_MISS_STS		BIT(7)
#define PCA9468_BIT_OCP_FAST_STS		BIT(6)
#define PCA9468_BIT_OCP_AVG_STS			BIT(5)
#define PCA9468_BIT_ACTIVE_STATE_STS	BIT(4)
#define PCA9468_BIT_SHUTDOWN_STATE_STS	BIT(3)
#define PCA9468_BIT_STANDBY_STATE_STS	BIT(2)
#define PCA9468_BIT_CHARGE_TIMER_STS	BIT(1)
#define PCA9468_BIT_WATCHDOG_TIMER_STS	BIT(0)

#define PCA9468_REG_STS_C				0x06	// IIN status
#define PCA9468_BIT_IIN_STS				BITS(7,2)

#define PCA9468_REG_STS_D				0x07	// ICHG status
#define PCA9468_BIT_ICHG_STS			BITS(7,1)

#define PCA9468_REG_STS_ADC_1			0x08	// ADC register
#define PCA9468_BIT_ADC_IIN7_0			BITS(7,0)

#define PCA9468_REG_STS_ADC_2			0x09	// ADC register
#define PCA9468_BIT_ADC_IOUT5_0			BITS(7,2)
#define PCA9468_BIT_ADC_IIN9_8			BITS(1,0)

#define PCA9468_REG_STS_ADC_3			0x0A	// ADC register
#define PCA9468_BIT_ADC_VIN3_0			BITS(7,4)
#define PCA9468_BIT_ADC_IOUT9_6			BITS(3,0)

#define PCA9468_REG_STS_ADC_4			0x0B	// ADC register
#define PCA9468_BIT_ADC_VOUT1_0			BITS(7,6)
#define PCA9468_BIT_ADC_VIN9_4			BITS(5,0)

#define PCA9468_REG_STS_ADC_5			0x0C	// ADC register
#define PCA9468_BIT_ADC_VOUT9_2			BITS(7,0)

#define PCA9468_REG_STS_ADC_6			0x0D	// ADC register
#define PCA9468_BIT_ADC_VBAT7_0			BITS(7,0)

#define PCA9468_REG_STS_ADC_7			0x0E	// ADC register
#define PCA9468_BIT_ADC_DIETEMP5_0		BITS(7,2)
#define PCA9468_BIT_ADC_VBAT9_8			BITS(1,0)

#define PCA9468_REG_STS_ADC_8			0x0F	// ADC register
#define PCA9468_BIT_ADC_NTCV3_0			BITS(7,4)
#define PCA9468_BIT_ADC_DIETEMP9_6		BITS(3,0)

#define PCA9468_REG_STS_ADC_9			0x10	// ADC register
#define PCA9468_BIT_ADC_NTCV9_4			BITS(5,0)

#define PCA9468_REG_ICHG_CTRL			0x20	// Change current configuration
#define PCA9468_BIT_ICHG_SS				BIT(7)
#define PCA9468_BIT_ICHG_CFG			BITS(6,0)

#define PCA9468_REG_IIN_CTRL			0x21	// Input current configuration
#define PCA9468_BIT_LIMIT_INCREMENT_EN	BIT(7)
#define PCA9468_BIT_IIN_SS				BIT(6)
#define PCA9468_BIT_IIN_CFG				BITS(5,0)

#define PCA9468_REG_START_CTRL			0x22	// Device initialization configuration
#define PCA9468_BIT_SNSRES				BIT(7)
#define PCA9468_BIT_EN_CFG				BIT(6)
#define PCA9468_BIT_STANDBY_EN			BIT(5)
#define PCA9468_BIT_REV_IIN_DET			BIT(4)
#define PCA9468_BIT_FSW_CFG				BITS(3,0)

#define PCA9468_REG_ADC_CTRL			0x23	// ADC configuration
#define PCA9468_BIT_FORCE_ADC_MODE		BITS(7,6)
#define PCA9468_BIT_ADC_SHUTDOWN_CFG	BIT(5)
#define PCA9468_BIT_HIBERNATE_DELAY		BITS(4,3)

#define PCA9468_REG_ADC_CFG				0x24	// ADC channel configuration
#define PCA9468_BIT_CH7_EN				BIT(7)
#define PCA9468_BIT_CH6_EN				BIT(6)
#define PCA9468_BIT_CH5_EN				BIT(5)
#define PCA9468_BIT_CH4_EN				BIT(4)
#define PCA9468_BIT_CH3_EN				BIT(3)
#define PCA9468_BIT_CH2_EN				BIT(2)
#define PCA9468_BIT_CH1_EN				BIT(1)

#define PCA9468_REG_TEMP_CTRL			0x25	// Temperature configuration
#define PCA9468_BIT_TEMP_REG			BITS(7,6)
#define PCA9468_BIT_TEMP_DELTA			BITS(5,4)
#define PCA9468_BIT_TEMP_REG_EN			BIT(3)
#define PCA9468_BIT_NTC_PROTECTION_EN	BIT(2)
#define PCA9468_BIT_TEMP_MAX_EN			BIT(1)

#define PCA9468_REG_PWR_COLLAPSE		0x26	// Power collapse configuration
#define PCA9468_BIT_UV_DELTA			BITS(7,6)
#define PCA9468_BIT_IIN_FORCE_COUNT		BIT(4)
#define PCA9468_BIT_BAT_MISS_DET_EN		BIT(3)

#define PCA9468_REG_V_FLOAT				0x27	// Voltage regulation configuration
#define PCA9468_BIT_V_FLOAT				BITS(7,0)

#define PCA9468_REG_SAFETY_CTRL			0x28	// Safety configuration
#define PCA9468_BIT_WATCHDOG_EN			BIT(7)
#define PCA9468_BIT_WATCHDOG_CFG		BITS(6,5)
#define PCA9468_BIT_CHG_TIMER_EN		BIT(4)
#define PCA9468_BIT_CHG_TIMER_CFG		BITS(3,2)
#define PCA9468_BIT_OV_DELTA			BITS(1,0)

#define PCA9468_REG_NTC_TH_1			0x29	// Thermistor threshold configuration
#define PCA9468_BIT_NTC_THRESHOLD7_0	BITS(7,0)

#define PCA9468_REG_NTC_TH_2			0x2A	// Thermistor threshold configuration
#define PCA9468_BIT_NTC_THRESHOLD9_8	BITS(1,0)

#define PCA9468_REG_ADC_ACCESS			0x30

#define PCA9468_REG_ADC_IMPROVE			0x3D
#define PCA9468_BIT_ADC_IIN_IMP			BIT(3)

#define PCA9468_REG_ADC_MODE			0x3F
#define PCA9468_BIT_ADC_MODE			BIT(4)

#define PCA9468_MAX_REGISTER			PCA9468_REG_ADC_MODE


#define PCA9468_IIN_CFG(_input_current)	(_input_current/100000)			// input current, unit - uA
#define PCA9468_ICHG_CFG(_chg_current)	(_chg_current/100000)			// charging current, uint - uA
#define PCA9468_V_FLOAT(_v_float)		((_v_float/1000 - 3725)/5)		// v_float voltage, unit - uV

#define PCA9468_SNSRES_5mOhm			0x00
#define PCA9468_SNSRES_10mOhm			PCA9468_BIT_SNSRES

/* VIN Overvoltage setting from 2*VOUT */
enum {
	OV_DELTA_10P,
	OV_DELTA_20P,
	OV_DELTA_30P,
	OV_DELTA_40P,
};

/* Switching frequency */
enum {
	FSW_CFG_833KHZ,
	FSW_CFG_893KHZ,
	FSW_CFG_935KHZ,
	FSW_CFG_980KHZ,
	FSW_CFG_1020KHZ,
	FSW_CFG_1080KHZ,
	FSW_CFG_1120KHZ,
	FSW_CFG_1160KHZ,
	FSW_CFG_440KHZ,
	FSW_CFG_490KHZ,
	FSW_CFG_540KHZ,
	FSW_CFG_590KHZ,
	FSW_CFG_630KHZ,
	FSW_CFG_680KHZ,
	FSW_CFG_730KHZ,
	FSW_CFG_780KHZ
};

/* Enable pin polarity selection */
#define PCA9468_EN_ACTIVE_H		0x00
#define PCA9468_EN_ACTIVE_L		PCA9468_BIT_EN_CFG
#define PCA9468_STANDBY_FORCED	PCA9468_BIT_STANDBY_EN
#define PCA9468_STANDBY_DONOT	0

/* ADC Channel */
enum {
	ADCCH_VOUT = 1,
	ADCCH_VIN,
	ADCCH_VBAT,
	ADCCH_ICHG,
	ADCCH_IIN,
	ADCCH_DIETEMP,
	ADCCH_NTC,
	ADCCH_MAX
};

/* ADC step */
#define VIN_STEP		16000	// 16mV(16000uV) LSB, Range(0V ~ 16.368V)
#define VBAT_STEP		5000	// 5mV(5000uV) LSB, Range(0V ~ 5.115V)
#define IIN_STEP		4890 	// 4.89mA(4890uA) LSB, Range(0A ~ 5A)
#define ICHG_STEP		9780 	// 9.78mA(9780uA) LSB, Range(0A ~ 10A)
#define DIETEMP_STEP  	435		// 0.435C LSB, Range(-25C ~ 160C)
#define DIETEMP_DENOM	1000	// 1000, denominator
#define DIETEMP_MIN 	-25  	// -25C
#define DIETEMP_MAX		160		// 160C
#define VOUT_STEP		5000 	// 5mV(5000uV) LSB, Range(0V ~ 5.115V)
#define NTCV_STEP		2346 	// 2.346mV(2346uV) LSB, Range(0V ~ 2.4V)

/* Charging Done Condition */
#define PCA9468_ICHG_DONE	1000000	// 1000mA
#define PCA9468_IIN_DONE	500000	// 500mA

/* Timer defination */
#define PCA9468_VBATMIN_CHECK_T	1000	// 1000ms
#define PCA9468_CCMODE_CHECK1_T	60000	// 60000ms
#define PCA9468_CCMODE_CHECK2_T	10000	// 10000ms
#define PCA9468_CCMODE_CHECK3_T	5000	// 5000ms
#define PCA9468_CVMODE_CHECK_T 	10000	// 10000ms
#define PCA9468_PDMSG_VDM_T		500		// 500ms
#define PCA9468_PPS_PERIODIC_T	10000	// 10000ms

/* Battery Threshold */
#define PCA9468_DC_VBAT_MIN		3500000	// 3500000uV
/* Input Current Limit default value */
#define PCA9468_IIN_CFG_DFT		2000000	// 2000000uA
/* Charging Current default value */
#define PCA9468_ICHG_CFG_DFT	6000000	// 6000000uA
/* Charging Float Voltage default value */
#define PCA9468_VFLOAT_DFT		4350000	// 4350000uV
/* Sense Resistance default value */
#define PCA9468_SENSE_R_DFT		1		// 10mOhm

/* CC mode 1,2 battery threshold */
#define PCA9468_CC2_VBAT_MIN	4250000 // 4250000uV
#define PCA9468_CC3_VBAT_MIN	4330000	// 4330000uV

/* Maximum TA voltage threshold */
#define PCA9468_TA_MAX_VOL		9800000 // 9800000uV
/* Maximum TA current threshold */
#define PCA9468_TA_MAX_CUR		2500000	// 2500000uA

#define PCA9468_TA_VOL_PRE_OFFSET	200000	// 200000uV
/* Adjust CC mode TA voltage step */
#define PCA9468_TA_VOL_STEP_ADJ_CC	40000	// 40000uV
/* Pre CV mode TA voltage step */
#define PCA9468_TA_VOL_STEP_PRE_CV	20000	// 20000uV

/* PD Message Voltage and Current Step */
#define PD_MSG_TA_VOL_STEP			20000	// 20mV
#define PD_MSG_TA_CUR_STEP			50000	// 50mA

/* INT1 Register Buffer */
enum {
	REG_INT1,
	REG_INT1_MSK,
	REG_INT1_STS,
	REG_INT1_MAX
};

/* STS Register Buffer */
enum {
	REG_STS_A,
	REG_STS_B,
	REG_STS_C,
	REG_STS_D,
	REG_STS_MAX
};

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
	CCMODE_LOOP_INACTIVE1,
	CCMODE_LOOP_INACTIVE2,
	CCMODE_LOOP_INACTIVE3,
	CCMODE_VIN_UVLO,
};

/* CV Mode Status */
enum {
	CVMODE_CHG_LOOP,
	CVMODE_VFLT_LOOP,
	CVMODE_IIN_LOOP,
	CVMODE_LOOP_INACTIVE,
	CVMODE_CHG_DONE,
	CVMODE_VIN_UVLO,
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
	PD_MSG_VDM_SET_VOL,
	PD_MSG_VDM_SET_CUR,
	PD_MSG_VDM_RFC_EXE,
};


/**
 * struct pca9468_charger - pca9468 charger instance
 * @monitor_wake_lock: lock to enter the suspend mode
 * @lock: protects concurrent access to online variables
 * @dev: pointer to device
 * @regmap: pointer to driver regmap
 * @mains: power_supply instance for AC/DC power
 * @chg_work: timer work for charging
 * @mains_online: is AC/DC input connected
 * @charging_state: direct charging state
 * @ta_cur: AC/DC(TA) current, uA
 * @ta_vol: AC/DC(TA) voltage, uV
 * @ta_objpos: AC/DC(TA) PDO object position
 * @ta_hr: TA Headroom voltage, uV
 * @vin1_adc: VIN1 adc value for calculating Rcable, uV
 * @iin1_adc: IIN1 adc value for calculating Rcable, uA
 * @rcable: Cable resistance from AC/DC to VIN, mOhm
 * @pdata: pointer to platform data
 */
struct pca9468_charger {
	struct wake_lock	monitor_wake_lock;
	struct mutex		lock;
	struct device		*dev;
	struct regmap		*regmap;
#if (LINUX_VERSION_CODE > KERNEL_VERSION(4, 1, 0))
	struct power_supply	*mains;
#else
	struct power_supply	mains;
#endif
	struct delayed_work work;
	unsigned int		timer_id;
	unsigned long      	timer_period;
	unsigned long		last_update_time;

	struct delayed_work	pps_work;	// PPS periodic request work
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

	struct pca9468_platform_data *pdata;
};


#ifdef CONFIG_USBPD_PHY_QCOM
static int pca9468_usbpd_setup(struct pca9468_charger *pca9468);
#endif


/*******************************/
/* Switching charger control function */
/*******************************/
/* This function needs some modification by a customer */
static int pca9468_set_switching_charger( bool enable, 
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
	
	if (enable == true)	{
		// Set Switching charger //
#if (LINUX_VERSION_CODE > KERNEL_VERSION(4, 1, 0))
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
#else
#ifndef CONFIG_USBPD_PHY_QCOM
		/* input current */
		val.intval = input_current;
		ret = psy_swcharger->set_property(psy_swcharger, POWER_SUPPLY_PROP_CURRENT_MAX, &val);
		if (ret < 0)
			goto error;
		/* charigng current */
		val.intval = charging_current;
		ret = psy_swcharger->set_property(psy_swcharger, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, &val);
		if (ret < 0)
			goto error;
		/* vfloat voltage */
		val.intval = vfloat;
		ret = psy_swcharger->set_property(psy_swcharger, POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE, &val);
		if (ret < 0)
			goto error;
#endif
		/* enable charger */
#ifdef CONFIG_USBPD_PHY_QCOM
		val.intval = enable;
		ret = psy_swcharger->set_property(psy_swcharger, POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
		if (ret < 0)
			goto error;
#else
		/* it depends on customer's code */
		//val.intval = enable;
		//ret = psy_swcharger->set_property(psy_swcharger, POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
		//if (ret < 0)
		//	goto error;
#endif
#endif
	} else {
		/* disable charger */
#if (LINUX_VERSION_CODE > KERNEL_VERSION(4, 1, 0))
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
#else
#ifdef CONFIG_USBPD_PHY_QCOM
		val.intval = enable;
		ret = psy_swcharger->set_property(psy_swcharger, POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
		if (ret < 0)
			goto error;
#else
		/* it depends on customer's code */
		/* Todo */
		//val.intval = enable;
		//ret = psy_swcharger->set_property(psy_swcharger, POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
		//if (ret < 0)
		//	goto error;
#endif
#endif
	}

error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

static int pca9468_get_switching_charger_is_enabled(int *enable)
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
static int pca9468_send_pd_message(struct pca9468_charger *pca9468, unsigned int msg_type)
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
	if (pca9468->pd == NULL) {
		pr_info("%s: get phandle\n", __func__);
		ret = pca9468_usbpd_setup(pca9468);
		if (ret != 0) {
			dev_err(pca9468->dev, "Error usbpd setup!\n");
			pca9468->pd = NULL;
			return ret;
		}
	}
#endif
	
	pr_info("%s: msg_type=%d, ta_cur=%d, ta_vol=%d, ta_objpos=%d\n", 
		__func__, msg_type, pca9468->ta_cur, pca9468->ta_vol, pca9468->ta_objpos);
		
	switch (msg_type) {
	case PD_MSG_REQUEST_APDO:
		/* Cancel pps request timer */
		cancel_delayed_work(&pca9468->pps_work);
#ifdef CONFIG_USBPD_PHY_QCOM
		ret = usbpd_request_pdo(pca9468->pd, pca9468->ta_objpos, pca9468->ta_vol, pca9468->ta_cur);
		if (ret == -EBUSY) {
			/* wait 100ms */
			msleep(100);
			/* try again */
			ret = usbpd_request_pdo(pca9468->pd, pca9468->ta_objpos, pca9468->ta_vol, pca9468->ta_cur);
		}
#else
		op_cur = pca9468->ta_cur/50000;		// Operating Current 50mA units
		out_vol = pca9468->ta_vol/20000;	// Output Voltage in 20mV units
		msg_buf[0] = op_cur & 0x7F;			// Operating Current 50mA units - B6...0
		msg_buf[1] = (out_vol<<1) & 0xFE;	// Output Voltage in 20mV units - B19..(B15)..B9
		msg_buf[2] = (out_vol>>7) & 0x0F;	// Output Voltage in 20mV units - B19..(B16)..B9,
		msg_buf[3] = pca9468->ta_objpos<<4;	// Object Position - B30...B28
#endif
		/* Start pps request timer */
		if (ret == 0) {
			schedule_delayed_work(&pca9468->pps_work, msecs_to_jiffies(PCA9468_PPS_PERIODIC_T));
		}
		break;
		
	case PD_MSG_REQUEST_FIXED_PDO:
		cancel_delayed_work(&pca9468->pps_work);
#ifdef CONFIG_USBPD_PHY_QCOM
		ret = usbpd_request_pdo(pca9468->pd, pca9468->ta_objpos, pca9468->ta_vol, pca9468->ta_cur);
		if (ret == -EBUSY) {
			/* wait 100ms */
			msleep(100);
			/* try again */
			ret = usbpd_request_pdo(pca9468->pd, pca9468->ta_objpos, pca9468->ta_vol, pca9468->ta_cur);
		}
#else
		max_cur = pca9468->ta_cur/10000; 	// Maximum Operation Current 10mA units
		op_cur = max_cur;					// Operating Current 10mA units
		msg_buf[0] = max_cur & 0xFF;		// Maximum Operation Current -B9..(7)..0
		msg_buf[1] = ((max_cur>>8) & 0x03) | ((op_cur<<2) & 0xFC);	// Operating Current - B19..(15)..10
		msg_buf[2] = ((op_cur>>6) & 0x0F);	// Operating Current - B19..(16)..10, Unchunked Extended Messages Supported  - not support
		msg_buf[3] = pca9468->ta_objpos<<4;	// Object Position - B30...B28
#endif
		break;
		
	case PD_MSG_VDM_SET_VOL:
		out_vol = pca9468->ta_vol/1000;		// Output Voltage in 1mV units(2bytes)
#ifdef CONFIG_USBPD_PHY_QCOM
		/* VDM Header - 4bytes */
		msg_buf[0] = 0x1FC95243;	// VID - NXP VID:0x1FC9, Command - 0x5243
		/* Vender defined Message */
		msg_buf[1] = 0x00000400;	// Offset, Offset - 2byte, NrBytes - 1bytes, OPCODE - WRITE
		msg_buf[2] = (out_vol<<8) | 0x000B; // Voltage(2bytes), Function ID(2bytes): Set Voltage - 0x000B
		ret = usbpd_send_vdm(pca9468->pd, msg_buf[0], &msg_buf[1], 2);
		if (ret == -EBUSY) {
			/* wait 100ms */
			msleep(100);
			/* try again */
			ret = usbpd_send_vdm(pca9468->pd, msg_buf[0], &msg_buf[1], 2);
		}
#else
		/* VDM Header - 4bytes */
		msg_buf[0] = 0x43;	// Commnad - 0x5243
		msg_buf[1] = 0x52;
		msg_buf[2] = 0xC9;	// VID - NXP VID 0x1FC9
		msg_buf[3] = 0x1F;
		/* Vender defined Message */
		msg_buf[4] = 0x00; 	// OPCODE - WRITE
		msg_buf[5] = 0x04;	// NrBytes - 1bytes
		msg_buf[6] = 0x00;	// Offset - 2bytes
		msg_buf[7] = 0x00;	// Offset
		
		msg_buf[8] = 0x0B;	// function ID - 2bytes
		msg_buf[9] = 0x00;	// Set Voltage - 0x000B

		msg_buf[10] = (out_vol & 0xFF);		// Voltage(2bytes)
		msg_buf[11] = (out_vol>>8) & 0xFF; 	// Voltage
#endif
		break;

	case PD_MSG_VDM_SET_CUR:
		op_cur = pca9468->ta_cur/1000;		// Output Voltage in 1mA units(2bytes)
#ifdef CONFIG_USBPD_PHY_QCOM
		/* VDM Header - 4bytes */
		msg_buf[0] = 0x1FC95243;	// VID - NXP VID:0x1FC9, Command - 0x5243
		/* Vender defined Message */
		msg_buf[1] = 0x00000400;	// Offset, Offset - 2byte, NrBytes - 1bytes, OPCODE - WRITE
		msg_buf[2] = (op_cur<<8) | 0x000E; // Voltage(2bytes), Function ID(2bytes): Set Current - 0x000E
		ret = usbpd_send_vdm(pca9468->pd, msg_buf[0], &msg_buf[1], 2);
		if (ret == -EBUSY) {
			/* wait 100ms */
			msleep(100);
			/* try again */
			ret = usbpd_send_vdm(pca9468->pd, msg_buf[0], &msg_buf[1], 2);
		}
#else
		/* VDM Header - 4bytes */
		msg_buf[0] = 0x43;	// Commnad - 0x5243
		msg_buf[1] = 0x52;
		msg_buf[2] = 0xC9;	// VID - NXP VID 0x1FC9
		msg_buf[3] = 0x1F;
		/* Vender defined Message */
		msg_buf[4] = 0x00; 	// OPCODE - WRITE
		msg_buf[5] = 0x04;	// NrBytes - 1bytes
		msg_buf[6] = 0x00;	// Offset - 2bytes
		msg_buf[7] = 0x00;	// Offset
		
		msg_buf[8] = 0x0E;	// function ID - 2bytes
		msg_buf[9] = 0x00;	// Set Current - 0x000E

		msg_buf[10] = (op_cur & 0xFF);		// Current(2bytes)
		msg_buf[11] = (op_cur>>8) & 0xFF; 	// Current
#endif
		break;

	case PD_MSG_VDM_RFC_EXE:
#ifdef CONFIG_USBPD_PHY_QCOM
		/* VDM Header - 4bytes */
		msg_buf[0] = 0x1FC95243;	// VID - NXP VID:0x1FC9, Command - 0x5243
		/* Vender defined Message */
		msg_buf[1] = 0x00000002;	// Offset, Offset - 2byte, NrBytes - 1bytes, OPCODE - EXE
		ret = usbpd_send_vdm(pca9468->pd, msg_buf[0], &msg_buf[1], 1);
		if (ret == -EBUSY) {
			/* wait 100ms */
			msleep(100);
			/* try again */
			ret = usbpd_send_vdm(pca9468->pd, msg_buf[0], &msg_buf[1], 2);
		}
#else
		/* VDM Header - 4bytes */
		msg_buf[0] = 0x43;	// Commnad - 0x5243
		msg_buf[1] = 0x52;
		msg_buf[2] = 0xC9;	// VID - NXP VID 0x1FC9
		msg_buf[3] = 0x1F;
		/* Vender defined Message */
		msg_buf[4] = 0x02; 	// OPCODE - EXE
		msg_buf[5] = 0x00;	// NrBytes - 1bytes
		msg_buf[6] = 0x00;	// Offset - 2bytes
		msg_buf[7] = 0x00;	// Offset
#endif
		break;

	default:
		break;
	}

	/* Send the PD messge to CC/PD chip */
	/* Todo - insert code */
	return ret;
}

/* ADC Read function */
static int pca9468_read_adc(struct pca9468_charger *pca9468, u8 adc_ch)
{
	u8 reg_data[2];
	u16 raw_adc;	// raw ADC value
	int conv_adc;	// conversion ADC value
	int ret;
	u8 rsense; /* sense resistance */
	
	switch (adc_ch)
	{
	case ADCCH_VOUT:
		// Write ADC Channel - Disable
		reg_data[0] = (u8) ~PCA9468_BIT_CH1_EN;
		ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_CFG, reg_data[0]);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		
		// Wait 120us
		udelay(120);	// need 120us

		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_4, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_VOUT9_2)<<2) | ((reg_data[0] & PCA9468_BIT_ADC_VOUT1_0)>>6);
		conv_adc = raw_adc * VOUT_STEP;	// unit - uV
		break;
	
	case ADCCH_VIN:
		// Write ADC Channel - Disable
		reg_data[0] = (u8) ~PCA9468_BIT_CH2_EN;
		ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_CFG, reg_data[0]);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		
		// Wait 120us
		udelay(120);	// need 120us

		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_3, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_VIN9_4)<<4) | ((reg_data[0] & PCA9468_BIT_ADC_VIN3_0)>>4);
		conv_adc = raw_adc * VIN_STEP;	// uint - uV
		break;

	case ADCCH_VBAT:
		// Write ADC Channel - Disable
		reg_data[0] = (u8) ~PCA9468_BIT_CH3_EN;
		ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_CFG, reg_data[0]);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		
		// Wait 120us
		udelay(120);	// need 120us

		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_6, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_VBAT9_8)<<8) | ((reg_data[0] & PCA9468_BIT_ADC_VBAT7_0)>>0);
		conv_adc = raw_adc * VBAT_STEP;		// unit - uV
		break;

	case ADCCH_ICHG:
		// Write ADC Channel - Disable
		reg_data[0] = (u8) ~PCA9468_BIT_CH4_EN;
		ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_CFG, reg_data[0]);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		
		// Wait 120us
		udelay(120);	// need 120us

		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_2, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC
		rsense = (pca9468->pdata->snsres == 0) ? 5 : 10;	// snsres : 0 - 5mOhm, 1 - 10mOhm
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_IOUT9_6)<<6) | ((reg_data[0] & PCA9468_BIT_ADC_IOUT5_0)>>2);
		conv_adc = raw_adc * ICHG_STEP;	// unit - uA
		break;

	case ADCCH_IIN:
		// Write ADC Channel - Disable
		reg_data[0] = (u8) ~PCA9468_BIT_CH5_EN;
		ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_CFG, reg_data[0]);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		
		// Wait 120us
		udelay(120);	// need 120us

		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_1, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_IIN9_8)<<8) | ((reg_data[0] & PCA9468_BIT_ADC_IIN7_0)>>0);
		conv_adc = raw_adc * IIN_STEP;		// unit - uV
		break;

	
	case ADCCH_DIETEMP:
		// Write ADC Channel - Disable
		reg_data[0] = (u8) ~PCA9468_BIT_CH6_EN;
		ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_CFG, reg_data[0]);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		
		// Wait 120us
		udelay(120);	// need 120us

		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_7, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_DIETEMP9_6)<<6) | ((reg_data[0] & PCA9468_BIT_ADC_DIETEMP5_0)>>2);
		conv_adc = (935 - raw_adc)*DIETEMP_STEP/DIETEMP_DENOM; 	// Temp = (935-rawadc)*0.435, unit - C
		if (conv_adc > DIETEMP_MAX) {
			conv_adc = DIETEMP_MAX;
		} else if (conv_adc < DIETEMP_MIN) {
			conv_adc = DIETEMP_MIN;
		}
		break;

	case ADCCH_NTC:
		// Write ADC Channel - Disable
		reg_data[0] = (u8) ~PCA9468_BIT_CH7_EN;
		ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_CFG, reg_data[0]);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		
		// Wait 120us
		udelay(120);	// need 120us

		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_8, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_NTCV9_4)<<4) | ((reg_data[0] & PCA9468_BIT_ADC_NTCV3_0)>>4);
		conv_adc = raw_adc * NTCV_STEP;		// unit - uV
		break;

	default:
		conv_adc = -EINVAL;
		break;
	}

error:
	// Write ADC Channel -Enable All
	reg_data[0] = 0xFF;
	ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_CFG, reg_data[0]);

	pr_info("%s: adc_ch=%d, convert_val=%d\n", __func__, adc_ch, conv_adc);

	return conv_adc;
}


static int pca9468_set_vfloat(struct pca9468_charger *pca9468, unsigned int v_float)
{
	int ret, val;

	pr_info("%s: vfloat=%d\n", __func__, v_float);

	/* v float voltage */
	val = PCA9468_V_FLOAT(v_float);
	ret = regmap_write(pca9468->regmap, PCA9468_REG_V_FLOAT, val);
	return ret;
}

static int pca9468_set_charging_current(struct pca9468_charger *pca9468, unsigned int ichg)
{
	int ret, val;

	pr_info("%s: ichg=%d\n", __func__, ichg);

	/* charging current */
	if (ichg > 8000000)
		ichg = 8000000;
	val = PCA9468_ICHG_CFG(ichg);
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_ICHG_CTRL, PCA9468_BIT_ICHG_CFG, val);
	return ret;
}

static int pca9468_set_input_current(struct pca9468_charger *pca9468, unsigned int iin)
{
	int ret, val;

	pr_info("%s: iin=%d\n", __func__, iin);


	/* input current */
	if (iin > 5000000)
		iin = 5000000;
	val = PCA9468_IIN_CFG(iin);

	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_IIN_CTRL, PCA9468_BIT_IIN_CFG, val);
	return ret;
}

static int pca9468_set_charging(struct pca9468_charger *pca9468, bool enable)
{
	int ret, val;

	pr_info("%s: enable=%d\n", __func__, enable);

	if (enable == true) {
		/* Improve adc */
		val = 0x5B;
		ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_ACCESS, val);
		if (ret < 0)
			goto error;
		ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_ADC_IMPROVE, PCA9468_BIT_ADC_IIN_IMP, 0);
		if (ret < 0)
			goto error;
	}
	
	/* Enable PCA9468 */
	val = (enable == true) ? (PCA9468_EN_ACTIVE_L | PCA9468_STANDBY_DONOT): (PCA9468_EN_ACTIVE_H | PCA9468_STANDBY_FORCED);
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL, PCA9468_BIT_EN_CFG | PCA9468_BIT_STANDBY_EN, val);
	if (ret < 0)
		goto error;
	
	if (enable == true) {
		/* Wait 50ms */
		msleep(50);
		ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_ADC_IMPROVE, PCA9468_BIT_ADC_IIN_IMP, PCA9468_BIT_ADC_IIN_IMP);
		if (ret  < 0)
			goto error;
		
		val = 0x00;
		ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_ACCESS, val);
	}

error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}


/* Check CC Mode status */
static int pca9468_check_ccmode_status(struct pca9468_charger *pca9468)
{
	unsigned int reg_val;
	int ret, vbat;
	
	/* Read STS_A */
	ret = regmap_read(pca9468->regmap, PCA9468_REG_STS_A, &reg_val);
	if (ret < 0)
		goto error;

	/* Read VBAT ADC */
	vbat = pca9468_read_adc(pca9468, ADCCH_VBAT);

	/* Check STS_A */
	if (reg_val & PCA9468_BIT_VIN_UV_STS) {
		ret = CCMODE_VIN_UVLO;
	} else if (reg_val & PCA9468_BIT_CHG_LOOP_STS) {
		ret = CCMODE_CHG_LOOP;
	} else if (reg_val & PCA9468_BIT_VFLT_LOOP_STS) {
		ret = CCMODE_VFLT_LOOP;
	} else if (reg_val & PCA9468_BIT_IIN_LOOP_STS) {
		ret = CCMODE_IIN_LOOP;
	} else {
		if (vbat < PCA9468_CC2_VBAT_MIN)
			ret = CCMODE_LOOP_INACTIVE1;
		else if (vbat < PCA9468_CC3_VBAT_MIN)
			ret = CCMODE_LOOP_INACTIVE2;
		else
			ret = CCMODE_LOOP_INACTIVE3;
	}

error:
	pr_info("%s: CCMODE Status=%d\n", __func__, ret);
	return ret;
}


/* Check CVMode Status */
static int pca9468_check_cvmode_status(struct pca9468_charger *pca9468)
{
	unsigned int val, iin;
	int ret;

	if (pca9468->charging_state == DC_STATE_START_CV) {
		/* Read STS_A */
		ret = regmap_read(pca9468->regmap, PCA9468_REG_STS_A, &val);
		if (ret < 0)
			goto error;
		/* Check STS_A */
		if (val & PCA9468_BIT_CHG_LOOP_STS)	{
			ret = CVMODE_CHG_LOOP;
		} else if (val & PCA9468_BIT_VFLT_LOOP_STS) {
			ret = CVMODE_VFLT_LOOP;
		} else if (val & PCA9468_BIT_IIN_LOOP_STS) {
			ret = CVMODE_IIN_LOOP;
		} else if (val & PCA9468_BIT_VIN_UV_STS) {
			ret = CVMODE_VIN_UVLO;
		} else {
			/* Any LOOP is inactive */
			ret = CVMODE_LOOP_INACTIVE;
		}
	} else {
		/* Read IIN ADC */
		iin = pca9468_read_adc(pca9468, ADCCH_IIN);
		if (iin < 0) {
			ret = iin;
			goto error;
		}
		
		/* Check IIN < 0.5A */
		if (iin < PCA9468_IIN_DONE) {
			/* Direct Charging Done */
			ret = CVMODE_CHG_DONE;
		} else {
			/* It doesn't reach top-off condition yet */
			
			/* Read STS_A */
			ret = regmap_read(pca9468->regmap, PCA9468_REG_STS_A, &val);
			if (ret < 0)
				goto error;
			/* Check STS_A */
			if (val & PCA9468_BIT_CHG_LOOP_STS) {
				ret = CVMODE_CHG_LOOP;
			} else if (val & PCA9468_BIT_VFLT_LOOP_STS) {
				ret = CVMODE_VFLT_LOOP;
			} else if (val & PCA9468_BIT_IIN_LOOP_STS) {
				ret = CVMODE_IIN_LOOP;
			} else if (val & PCA9468_BIT_VIN_UV_STS) {
				ret = CVMODE_VIN_UVLO;
			} else {
				/* Any LOOP is inactive */
				ret = CVMODE_LOOP_INACTIVE;
			}
		}
	}
	
error:
	pr_info("%s: CVMODE Status=%d\n", __func__, ret);
	return ret;
}


/* 2:1 Direct Charging Adjust CC MODE control */
static int pca9468_charge_adjust_ccmode(struct pca9468_charger *pca9468)
{
	int ret, iin;
	int vbatt;

	pr_info("%s: ======START=======\n", __func__);

	pca9468->charging_state = DC_STATE_ADJUST_CC;

	ret = pca9468_check_ccmode_status(pca9468);
	if (ret < 0)
		goto error;

	switch(ret) {
	case CCMODE_IIN_LOOP:
	case CCMODE_CHG_LOOP:
		/* Decrease TA current (50mA) */
		pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;
		pr_info("%s: CC adjust End: ta_cur=%d\n", __func__, pca9468->ta_cur);
		/* Read VBAT ADC */
		vbatt = pca9468_read_adc(pca9468, ADCCH_VBAT);
		/* TA voltage = TA voltage + (VFLOAT - VBAT_ADC)*2 */
		pca9468->ta_vol = pca9468->ta_vol + (pca9468->pdata->v_float - vbatt)*2;
		if (pca9468->ta_vol > PCA9468_TA_MAX_VOL)
			pca9468->ta_vol = PCA9468_TA_MAX_VOL;
		pr_info("%s: CC adjust End: ta_vol=%d\n", __func__, pca9468->ta_vol);
		/* Send PD Message */
#ifdef CONFIG_USBPD_SUPPORT_PPS
		ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
		if (ret < 0)
			goto error;
#else
		ret = pca9468_send_pd_message(pca9468, PD_MSG_VDM_SET_CUR);
		if (ret < 0)
			goto error;
		ret = pca9468_send_pd_message(pca9468, PD_MSG_VDM_RFC_EXE);
		if (ret < 0)
			goto error;
		ret = pca9468_send_pd_message(pca9468, PD_MSG_VDM_SET_VOL);
		if (ret < 0)
			goto error;
		ret = pca9468_send_pd_message(pca9468, PD_MSG_VDM_RFC_EXE);
		if (ret < 0)
			goto error;
#endif
		mutex_lock(&pca9468->lock);
		/* End TA voltage and current adjustment */
		/* go to CC mode */
		pca9468->charging_state = DC_STATE_START_CC;
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = PCA9468_PDMSG_VDM_T;
		mutex_unlock(&pca9468->lock);
		schedule_delayed_work(&pca9468->work, msecs_to_jiffies(pca9468->timer_period));
		break;
		
	case CCMODE_VFLT_LOOP:
		/* go to CV mode */
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_ENTER_CVMODE;
		pca9468->timer_period = 0;
		mutex_unlock(&pca9468->lock);
		schedule_delayed_work(&pca9468->work, msecs_to_jiffies(pca9468->timer_period));
		break;

	case CCMODE_LOOP_INACTIVE1:
	case CCMODE_LOOP_INACTIVE2:
	case CCMODE_LOOP_INACTIVE3:
		/* Check IIN ADC with IIN_CFG */
		iin = pca9468_read_adc(pca9468, ADCCH_IIN);
		/* IIN_ADC > IIN_CFG -40mA ? */
		if (iin > (pca9468->pdata->iin_cfg - 40000)) {
			/* Input current is already over IIN_CFG */
			/* End TA voltage and current adjustment */
			/* go to CC mode */
			pca9468->charging_state = DC_STATE_START_CC;

			/* Read VBAT ADC */
			vbatt = pca9468_read_adc(pca9468, ADCCH_VBAT);
			/* TA voltage = TA voltage + (VFLOAT - VBAT_ADC)*2 */
			pca9468->ta_vol = pca9468->ta_vol + (pca9468->pdata->v_float - vbatt)*2;
			if (pca9468->ta_vol > PCA9468_TA_MAX_VOL)
				pca9468->ta_vol = PCA9468_TA_MAX_VOL;
			pr_info("%s: CC adjust End: IIN_ADC=%d, ta_vol=%d\n", __func__, iin, pca9468->ta_vol);	
		} else {
			/* IIN_ADC > pre_IIN_ADC + 20mA ? */
			if (iin > (pca9468->pre_iin_adc + 20000)) {
				/* TA voltage too low to enter TA CC mode, so we should increae TA voltage */
				pca9468->ta_vol = pca9468->ta_vol + PCA9468_TA_VOL_STEP_ADJ_CC;
				if (pca9468->ta_vol > PCA9468_TA_MAX_VOL)
					pca9468->ta_vol = PCA9468_TA_MAX_VOL;
				pr_info("%s: CC adjust Cont: ta_vol=%d\n", __func__, pca9468->ta_vol);
			} else {
				/* Check Max TA current */
				if (pca9468->ta_cur == PCA9468_TA_MAX_CUR) {
					/* TA current is already max value */
					/* Check TA voltage */
					if (pca9468->ta_vol == PCA9468_TA_MAX_VOL) {
						/* TA voltage is already max value */
						/* Change charging status */
						pca9468->charging_state = DC_STATE_START_CC;
						pr_info("%s: CC adjust End: MAX value, ta_vol=%d, ta_cur=%d\n", 
							__func__, pca9468->ta_vol, pca9468->ta_cur);
					} else {
						/* TA voltage too low to enter TA CC mode, so we should increae TA voltage */
						pca9468->ta_vol = pca9468->ta_vol + PCA9468_TA_VOL_STEP_ADJ_CC;
						if (pca9468->ta_vol > PCA9468_TA_MAX_VOL)
							pca9468->ta_vol = PCA9468_TA_MAX_VOL;
						pr_info("%s: CC adjust Cont: ta_vol=%d\n", __func__, pca9468->ta_vol);
					}
				} else {
					/* Increase TA current (50mA) */
					pca9468->ta_cur = pca9468->ta_cur + PD_MSG_TA_CUR_STEP;
					if (pca9468->ta_cur > PCA9468_TA_MAX_CUR)
						pca9468->ta_cur = PCA9468_TA_MAX_CUR;
					pr_info("%s: CC adjust Cont: ta_cur=%d\n", __func__, pca9468->ta_cur);
				}
			}					
		}

		/* Save the current iin adc  */
		pca9468->pre_iin_adc = iin;

		/* Send PD Message */
#ifdef CONFIG_USBPD_SUPPORT_PPS
		ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
		if (ret < 0)
			goto error;
#else
		ret = pca9468_send_pd_message(pca9468, PD_MSG_VDM_SET_CUR);
		if (ret < 0)
			goto error;
		ret = pca9468_send_pd_message(pca9468, PD_MSG_VDM_RFC_EXE);
		if (ret < 0)
			goto error;
		ret = pca9468_send_pd_message(pca9468, PD_MSG_VDM_SET_VOL);
		if (ret < 0)
			goto error;
		ret = pca9468_send_pd_message(pca9468, PD_MSG_VDM_RFC_EXE);
		if (ret < 0)
			goto error;
#endif
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = PCA9468_PDMSG_VDM_T;
		mutex_unlock(&pca9468->lock);
		schedule_delayed_work(&pca9468->work, msecs_to_jiffies(pca9468->timer_period));
		break;

	case CCMODE_VIN_UVLO:
		/* Stop Charging */
		ret = pca9468_set_charging(pca9468, false);
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
static int pca9468_charge_ccmode(struct pca9468_charger *pca9468)
{
	int ret;

	pr_info("%s: ======START=======\n", __func__);

	pca9468->charging_state = DC_STATE_CHECK_CC;

	ret = pca9468_check_ccmode_status(pca9468);
	if (ret < 0)
		goto error;

	switch(ret)	{
	case CCMODE_LOOP_INACTIVE1:
	case CCMODE_LOOP_INACTIVE2:
	case CCMODE_LOOP_INACTIVE3:
		/* Set timer */
		mutex_lock(&pca9468->lock);
		if (ret == CCMODE_LOOP_INACTIVE1) {
			/* Set 60s timer */
			pca9468->timer_period = PCA9468_CCMODE_CHECK1_T;
		} else if (ret == CCMODE_LOOP_INACTIVE2) {
			/* Set 10s timer */
			pca9468->timer_period = PCA9468_CCMODE_CHECK2_T;
		} else {
			/* Set 5s timer */
			pca9468->timer_period = PCA9468_CCMODE_CHECK3_T;
		}
		pca9468->timer_id = TIMER_CCMODE_CHECK;
		mutex_unlock(&pca9468->lock);
		schedule_delayed_work(&pca9468->work, msecs_to_jiffies(pca9468->timer_period));
		break;

	case CCMODE_VFLT_LOOP:
		/* go to CV mode */
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_ENTER_CVMODE;
		pca9468->timer_period = 0;
		mutex_unlock(&pca9468->lock);
		schedule_delayed_work(&pca9468->work, msecs_to_jiffies(pca9468->timer_period));
		break;

	case CCMODE_IIN_LOOP:
	case CCMODE_CHG_LOOP:
		/* Decrease TA current (50mA) */
		pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;

		pr_info("%s: CC LOOP: ta_cur=%d\n", __func__, pca9468->ta_cur);
		/* Send PD Message */
#ifdef CONFIG_USBPD_SUPPORT_PPS
		ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
		if (ret < 0)
			goto error;
#else
		ret = pca9468_send_pd_message(pca9468, PD_MSG_VDM_SET_CUR);
		if (ret < 0)
			goto error;
		ret = pca9468_send_pd_message(pca9468, PD_MSG_VDM_RFC_EXE);
		if (ret < 0)
			goto error;
#endif
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = PCA9468_PDMSG_VDM_T;
		mutex_unlock(&pca9468->lock);
		schedule_delayed_work(&pca9468->work, msecs_to_jiffies(pca9468->timer_period));
		break;			

	case CCMODE_VIN_UVLO:
		/* Stop Charging */
		ret = pca9468_set_charging(pca9468, false);
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
static int pca9468_charge_start_cvmode(struct pca9468_charger *pca9468)
{
	int ret, cvmode;

	pr_info("%s: ======START=======\n", __func__);

	pca9468->charging_state = DC_STATE_START_CV;
	
	cvmode = pca9468_check_cvmode_status(pca9468);
	if (cvmode < 0) {
		ret = cvmode;
		goto error;
	}
	
	switch(cvmode) {
	case CVMODE_CHG_LOOP:
	case CVMODE_IIN_LOOP:
		/* Decrease TA current (50mA) */
		pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;
		/* Send PD Message */
#ifdef CONFIG_USBPD_SUPPORT_PPS
		ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
		if (ret < 0)
			goto error;
#else
		ret = pca9468_send_pd_message(pca9468, PD_MSG_VDM_SET_CUR);
		if (ret < 0)
			goto error;
		ret = pca9468_send_pd_message(pca9468, PD_MSG_VDM_RFC_EXE);
		if (ret < 0)
			goto error;
#endif
		pr_info("%s: PreCV Cont: ta_cur=%d\n", __func__, pca9468->ta_cur);

		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = PCA9468_PDMSG_VDM_T;
		mutex_unlock(&pca9468->lock);
		schedule_delayed_work(&pca9468->work, msecs_to_jiffies(pca9468->timer_period));
		break;

	case CVMODE_VFLT_LOOP:
		/* Decrease TA voltage (60mV) */
		pca9468->ta_vol = pca9468->ta_vol - PCA9468_TA_VOL_STEP_PRE_CV;
		/* Send PD Message */
#ifdef CONFIG_USBPD_SUPPORT_PPS
		ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
		if (ret < 0)
			goto error;
#else
		ret = pca9468_send_pd_message(pca9468, PD_MSG_VDM_SET_VOL);
		if (ret < 0)
			goto error;
		ret = pca9468_send_pd_message(pca9468, PD_MSG_VDM_RFC_EXE);
		if (ret < 0)
			goto error;
#endif
		pr_info("%s: PreCV Cont: ta_vol=%d\n", __func__, pca9468->ta_vol);

		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = PCA9468_PDMSG_VDM_T;
		mutex_unlock(&pca9468->lock);
		schedule_delayed_work(&pca9468->work, msecs_to_jiffies(pca9468->timer_period));
		break;

	case CVMODE_LOOP_INACTIVE:
		/* Exit Pre CV mode */
		/* Go to CV mode */
		pr_info("%s: PreCV End: ta_cur=%d\n", __func__, pca9468->ta_cur);
		
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_CVMODE_CHECK;
		pca9468->timer_period = 0;
		mutex_unlock(&pca9468->lock);
		schedule_delayed_work(&pca9468->work, msecs_to_jiffies(pca9468->timer_period));
		break;

	case CVMODE_VIN_UVLO:
		/* Stop Charging */
		ret = pca9468_set_charging(pca9468, false);
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
static int pca9468_charge_cvmode(struct pca9468_charger *pca9468)
{
	int ret, cvmode;

	pr_info("%s: ======START=======\n", __func__);

	pca9468->charging_state = DC_STATE_CHECK_CV;
	
	cvmode = pca9468_check_cvmode_status(pca9468);
	if (cvmode < 0) {
		ret = cvmode;
		goto error;
	}
	
	switch(cvmode) {
	case CVMODE_CHG_DONE:
		/* Charging Done */
		/* Disable PCA9468 */
		ret = pca9468_set_charging(pca9468, false);
		if (ret < 0)
			goto error;

		/* Change charging status */
		pca9468->charging_state = DC_STATE_CHARGING_DONE;
		
		/* Set TA voltage to fixed 5V */
		pca9468->ta_vol = 5000000;
		/* Set TA current to maximum 3A */
		pca9468->ta_cur = 3000000;
		
		/* Send PD Message */
		pca9468->ta_objpos = 1;	// PDO1 - fixed 5V
		ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_FIXED_PDO);
		if (ret < 0)
			goto error;

		pr_info("%s: CV Done\n", __func__);

		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = PCA9468_PDMSG_VDM_T;
		mutex_unlock(&pca9468->lock);
		schedule_delayed_work(&pca9468->work, msecs_to_jiffies(pca9468->timer_period));
		break;

	case CVMODE_CHG_LOOP:
	case CVMODE_IIN_LOOP:
		/* Decrease TA current (50mA) */
		pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;
		/* Send PD Message */
#ifdef CONFIG_USBPD_SUPPORT_PPS
		ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
		if (ret < 0)
			goto error;
#else
		ret = pca9468_send_pd_message(pca9468, PD_MSG_VDM_SET_CUR);
		if (ret < 0)
			goto error;
		ret = pca9468_send_pd_message(pca9468, PD_MSG_VDM_RFC_EXE);
		if (ret < 0)
			goto error;
#endif
		pr_info("%s: CV LOOP, Cont: ta_cur=%d\n", __func__, pca9468->ta_cur);

		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = PCA9468_PDMSG_VDM_T;
		mutex_unlock(&pca9468->lock);
		schedule_delayed_work(&pca9468->work, msecs_to_jiffies(pca9468->timer_period));
		break;

	case CVMODE_VFLT_LOOP:
		/* Decrease TA voltage */
		pca9468->ta_vol = pca9468->ta_vol - PD_MSG_TA_VOL_STEP;
		/* Send PD Message */
#ifdef CONFIG_USBPD_SUPPORT_PPS
		ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
		if (ret < 0)
			goto error;
#else
		ret = pca9468_send_pd_message(pca9468, PD_MSG_VDM_SET_VOL);
		if (ret < 0)
			goto error;
		ret = pca9468_send_pd_message(pca9468, PD_MSG_VDM_RFC_EXE);
		if (ret < 0)
			goto error;
#endif
		pr_info("%s: CV VFLOAT, Cont: ta_vol=%d\n", __func__, pca9468->ta_vol);

		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = PCA9468_PDMSG_VDM_T;
		mutex_unlock(&pca9468->lock);
		schedule_delayed_work(&pca9468->work, msecs_to_jiffies(pca9468->timer_period));
		break;

	case CVMODE_LOOP_INACTIVE:
		/* Set timer */
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_CVMODE_CHECK;
		pca9468->timer_period = PCA9468_CVMODE_CHECK_T;
		mutex_unlock(&pca9468->lock);
		schedule_delayed_work(&pca9468->work, msecs_to_jiffies(pca9468->timer_period));
		break;

	case CVMODE_VIN_UVLO:
		/* Stop Charging */
		ret = pca9468_set_charging(pca9468, false);
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
static int pca9468_preset_dcmode(struct pca9468_charger *pca9468)
{
	int vbat;
	unsigned int val;
	int ret = 0;

	pr_info("%s: ======START=======\n", __func__);

	pca9468->charging_state = DC_STATE_PRESET_DC;

	/* Read VBAT ADC */
	vbat = pca9468_read_adc(pca9468, ADCCH_VBAT);

	/* Save Charging Configuration */

	/* clear the previous IIN ADC value */
	pca9468->pre_iin_adc = 0;

	/* Set TA voltage to (2*VBAT_ADC + 200 mV) */
	pca9468->ta_vol = 2*vbat + PCA9468_TA_VOL_PRE_OFFSET;
	val = pca9468->ta_vol/PD_MSG_TA_VOL_STEP;	/* PPS voltage resolution is 20mV */
	pca9468->ta_vol = val*PD_MSG_TA_VOL_STEP;
	/* Set TA current to IIN */
	pca9468->ta_cur = pca9468->pdata->iin_cfg;
	val = pca9468->ta_cur/PD_MSG_TA_CUR_STEP;	/* PPS current resolution is 50mV */
	pca9468->ta_cur = val*PD_MSG_TA_CUR_STEP;
	pca9468->ta_objpos = 0;	/* Search the proper object position of PDO */
	
#ifdef CONFIG_USBPD_SUPPORT_PPS
	ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
	if (ret < 0)
		goto error;
#else
	ret = pca9468_send_pd_message(pca9468, PD_MSG_VDM_SET_VOL);
	if (ret < 0)
		goto error;
	ret = pca9468_send_pd_message(pca9468, PD_MSG_VDM_RFC_EXE);
	if (ret < 0)
		goto error;
#endif
	pr_info("%s: Preset DC, ta_vol=%d, ta_cur=%d\n", 
		__func__, pca9468->ta_vol, pca9468->ta_cur);

	mutex_lock(&pca9468->lock);
	pca9468->timer_id = TIMER_PDMSG_SEND;
	pca9468->timer_period = PCA9468_PDMSG_VDM_T;
	mutex_unlock(&pca9468->lock);
	schedule_delayed_work(&pca9468->work, msecs_to_jiffies(pca9468->timer_period));

error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}


/* Preset direct charging configuration */
static int pca9468_preset_config(struct pca9468_charger *pca9468)
{
	int ret = 0;
	
	pr_info("%s: ======START=======\n", __func__);
	
	pca9468->charging_state = DC_STATE_PRESET_DC;

	/* Set IIN_CFG */
	ret = pca9468_set_input_current(pca9468, pca9468->pdata->iin_cfg);
	if (ret < 0)
		goto error;

	/* Set ICHG_CFG */
	ret = pca9468_set_charging_current(pca9468, pca9468->pdata->ichg_cfg);
	if (ret < 0)
		goto error;

	/* Set VFLOAT */
	ret = pca9468_set_vfloat(pca9468, pca9468->pdata->v_float);
	if (ret < 0)
		goto error;

	/* Enable PCA9468 */	
	ret = pca9468_set_charging(pca9468, true);
	if (ret < 0)
		goto error;

	/* Go to Adjust CC mode after 150ms*/
	mutex_lock(&pca9468->lock);
	pca9468->timer_id = TIMER_ENTER_ADJ_CCMODE;
	pca9468->timer_period = 150;
	mutex_unlock(&pca9468->lock);
	schedule_delayed_work(&pca9468->work, msecs_to_jiffies(pca9468->timer_period));
	ret = 0;
	
error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret; 
}


/* Enter direct charging algorithm */
static int pca9468_start_direct_charging(struct pca9468_charger *pca9468)
{
	int ret;
	unsigned int val;

	pr_info("%s: =========START=========\n", __func__);

	/* Set OV_DELTA to 40% */
	val = OV_DELTA_40P << MASK2SHIFT(PCA9468_BIT_OV_DELTA);
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_SAFETY_CTRL,
							PCA9468_BIT_OV_DELTA, val);
	if (ret < 0)
			return ret;

	/* Set Switching Frequency to 980KHz */
	val = 0x03;	// 980KHz
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
							PCA9468_BIT_FSW_CFG, val);
	if (ret < 0)
		return ret;

	/* current sense resistance */
	val = pca9468->pdata->snsres << MASK2SHIFT(PCA9468_BIT_SNSRES);
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
							PCA9468_BIT_SNSRES, val);
	if (ret < 0)
		return ret;

	/* wake lock */
	wake_lock(&pca9468->monitor_wake_lock);

	/* Preset charging configuration and TA condition */
	ret = pca9468_preset_dcmode(pca9468);

	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}


/* Check Vbat minimum level to start direct charging */
static int pca9468_check_vbatmin(struct pca9468_charger *pca9468)
{
	unsigned int vbat;
	int ret;

	pr_info("%s: =========START=========\n", __func__);

	pca9468->charging_state = DC_STATE_CHECK_VBAT;

	/* Check Vbat */
	vbat = pca9468_read_adc(pca9468, ADCCH_VBAT);
	if (vbat < 0) {
		ret = vbat;
	}
	if (vbat > PCA9468_DC_VBAT_MIN) {
		/* Start Direct Charging */
		/* Read switching charger status */
		int enable;
		ret = pca9468_get_switching_charger_is_enabled(&enable);
		if (ret < 0) {
			/* Start Direct Charging again after 1sec */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_VBATMIN_CHECK;
			pca9468->timer_period = PCA9468_VBATMIN_CHECK_T;
			mutex_unlock(&pca9468->lock);
			schedule_delayed_work(&pca9468->work, msecs_to_jiffies(pca9468->timer_period));
			goto error;
		}
		
		if (enable == 0) {
			/* already disabled switching charger */
			/* enable direct charging */
			ret = pca9468_start_direct_charging(pca9468);
			if (ret < 0) {
				/* Start Direct Charging again after 1sec */
				mutex_lock(&pca9468->lock);
				pca9468->timer_id = TIMER_VBATMIN_CHECK;
				pca9468->timer_period = PCA9468_VBATMIN_CHECK_T;
				mutex_unlock(&pca9468->lock);
				schedule_delayed_work(&pca9468->work, msecs_to_jiffies(pca9468->timer_period));
				goto error;
			}
		} else {
			/* now switching charger is enabled */
			/* disable switching charger first */
			ret = pca9468_set_switching_charger(false, 0, 0, 0);

			/* Wait 1sec for stopping switching charger */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_VBATMIN_CHECK;
			pca9468->timer_period = PCA9468_VBATMIN_CHECK_T;
			mutex_unlock(&pca9468->lock);
			schedule_delayed_work(&pca9468->work, msecs_to_jiffies(pca9468->timer_period));
		}
	}else {
		/* Start 1sec timer for battery check */
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_VBATMIN_CHECK;
		pca9468->timer_period = PCA9468_VBATMIN_CHECK_T;
		mutex_unlock(&pca9468->lock);
		schedule_delayed_work(&pca9468->work, msecs_to_jiffies(pca9468->timer_period));
		ret = 0;
	}

error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}


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
static void pca9468_timer_work(struct work_struct *work)
{
	struct pca9468_charger *pca9468 = container_of(work, struct pca9468_charger,
						 work.work);

	get_current_time(&pca9468->last_update_time);

	pr_info("%s: timer id=%d, charging_state=%d, last_update_time=%lu\n", 
		__func__, pca9468->timer_id, pca9468->charging_state, pca9468->last_update_time);
	
	switch (pca9468->timer_id) {
	case TIMER_VBATMIN_CHECK:
		pca9468_check_vbatmin(pca9468);
		break;

	case TIMER_ENTER_ADJ_CCMODE:
		pca9468_charge_adjust_ccmode(pca9468);
		break;			
					
	case TIMER_CCMODE_CHECK:
		pca9468_charge_ccmode(pca9468);
		break;

	case TIMER_ENTER_CVMODE:
		/* Enter Pre-CV mode */
		pca9468_charge_start_cvmode(pca9468);
		break;

	case TIMER_CVMODE_CHECK:
		pca9468_charge_cvmode(pca9468);
		break;

	case TIMER_PDMSG_SEND:
		/* Enter here after sending PD message */
		/* Changing TA voltage */
		
		/* check the charging status */
		if (pca9468->charging_state == DC_STATE_PRESET_DC) {
			/* preset pca9468 configuration */
			pca9468_preset_config(pca9468);
		} else if (pca9468->charging_state == DC_STATE_ADJUST_CC) {
			/* Adjust CC mode */
			pca9468_charge_adjust_ccmode(pca9468);
		} else if (pca9468->charging_state == DC_STATE_START_CC) {
			/* Start CC mode */
			/* interrupt enable here if we use interrupt method */
			pca9468_charge_ccmode(pca9468);
		} else if (pca9468->charging_state == DC_STATE_CHECK_CC) {
			/* Check CC mode */
			pca9468_charge_ccmode(pca9468);
		} else if (pca9468->charging_state == DC_STATE_START_CV) {
			/* Start CV mode - pre CV mode */
			pca9468_charge_start_cvmode(pca9468);
		} else if (pca9468->charging_state == DC_STATE_CHECK_CV) {
			/* Check CV mode */
			pca9468_charge_cvmode(pca9468);
		} else if (pca9468->charging_state == DC_STATE_CHARGING_DONE) {
			/* Timer ID is none */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_ID_NONE;
			mutex_unlock(&pca9468->lock);
			/* Enable Switching Charger */
			pca9468_set_switching_charger(true, pca9468->pdata->iin_cfg, 
										pca9468->pdata->ichg_cfg, pca9468->pdata->v_float);
			/* wake unlock */
			wake_unlock(&pca9468->monitor_wake_lock);
		}
		break;
		
	default:
		break;
	}
}


/* delayed work function for pps periodic timer */
static void pca9468_pps_request_work(struct work_struct *work)
{
	struct pca9468_charger *pca9468 = container_of(work, struct pca9468_charger,
						 pps_work.work);

	int ret = 0;
	
	pr_info("%s: pps_work_start\n", __func__);

	/* Send PD message */
	ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
	pr_info("%s: End, ret=%d\n", __func__, ret);
}

static int pca9468_hw_init(struct pca9468_charger *pca9468)
{
	unsigned int val;
	int ret;

	pr_info("%s: =========START=========\n", __func__);

	/*
	 * Program the platform specific configuration values to the device
	 * first.
	 */

	/* Set OV_DELTA to 40% */
	val = OV_DELTA_40P << MASK2SHIFT(PCA9468_BIT_OV_DELTA);
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_SAFETY_CTRL,
						 	PCA9468_BIT_OV_DELTA, val);
	if (ret < 0)
		return ret;

	/* Set Switching Frequencey to 980KHz */
	val = FSW_CFG_980KHZ << MASK2SHIFT(PCA9468_BIT_FSW_CFG);
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
						 	PCA9468_BIT_FSW_CFG, val);
	if (ret < 0)
		return ret;

	/* current sense resistance */
	val = pca9468->pdata->snsres << MASK2SHIFT(PCA9468_BIT_SNSRES);
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
						 	PCA9468_BIT_SNSRES, val);
	if (ret < 0)
		return ret;

	/* clear LIMIT_INCREMENT_EN */
	val = 0;
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_IIN_CTRL,
						 	PCA9468_BIT_LIMIT_INCREMENT_EN, val);
	if (ret < 0)
		return ret;
	
#ifdef CONFIG_PCA9468_HW_REV_B0
	/* Set the ADC channel */
	val = 0xFF;
	ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_CFG, val);
	if (ret < 0)
		return ret;

	/* Set the test register */
	// ADC Mode change
	// [0x30] = 0x5B	// Open test register
	// [0x3F] = 0x10
	// [0x30] = 0x00	// Close test register
	val = 0x5B;
	ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_ACCESS, val);
	if (ret < 0)
		return ret;
	val = 0x10;
	ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_MODE, val);
	if (ret < 0)
		return ret;
	val = 0x00;
	ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_ACCESS, val);	
	if (ret < 0)
		return ret;
#endif
	
	/* input current - uA*/	
	ret = pca9468_set_input_current(pca9468, pca9468->pdata->iin_cfg);
	if (ret < 0)
		return ret;

	/* charging current */
	ret = pca9468_set_charging_current(pca9468, pca9468->pdata->ichg_cfg);
	if (ret < 0)
		return ret;

	/* v float voltage */
	ret = pca9468_set_vfloat(pca9468, pca9468->pdata->v_float);
	if (ret < 0)
		return ret;
	
	return ret;
}


static irqreturn_t pca9468_interrupt_handler(int irq, void *data)
{
	struct pca9468_charger *pca9468 = data;
	u8 int1[REG_INT1_MAX], sts[REG_STS_MAX];	/* INT1, INT1_MSK, INT1_STS, STS_A, B, C, D */
	u8 masked_int;	/* masked int */
	bool handled = false;
	int ret;

	/* Read INT1, INT1_MSK, INT1_STS */
	ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_INT1, int1, 3);
	if (ret < 0) {
		dev_warn(pca9468->dev, "reading INT1_X failed\n");
		return IRQ_NONE;
	}

	/* Read STS_A, B, C, D */
	ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_A, sts, 4);
	if (ret < 0) {
		dev_warn(pca9468->dev, "reading STS_X failed\n");
		return IRQ_NONE;
	}

	pr_info("%s: int1=0x%2x, int1_sts=0x%2x, sts_a=0x%2x\n", __func__, 
			int1[REG_INT1], int1[REG_INT1_STS], sts[REG_STS_A]);

	/* Check Interrupt */
	masked_int = int1[REG_INT1] & !int1[REG_INT1_MSK];
	if (masked_int & PCA9468_BIT_V_OK_INT) {
		/* V_OK interrupt happened */
		mutex_lock(&pca9468->lock);
		pca9468->mains_online = (int1[REG_INT1_STS] & PCA9468_BIT_V_OK_STS) ? true : false;
		mutex_unlock(&pca9468->lock);
#if (LINUX_VERSION_CODE > KERNEL_VERSION(4, 1, 0))
		power_supply_changed(pca9468->mains);
#else
		power_supply_changed(&pca9468->mains);
#endif
		handled = true;
	}

	if (masked_int & PCA9468_BIT_NTC_TEMP_INT) {
		/* NTC_TEMP interrupt happened */
		if (int1[REG_INT1_STS] & PCA9468_BIT_NTC_TEMP_STS) {
			/* above NTC_THRESHOLD */
			dev_err(pca9468->dev, "charging stopped due to NTC threshold voltage\n");
		}
		handled = true;
	}

	if (masked_int & PCA9468_BIT_CHG_PHASE_INT) {
		/* CHG_PHASE interrupt happened */
		if (int1[REG_INT1_STS] & PCA9468_BIT_CHG_PHASE_STS) {
			/* Any of loops is active*/
			if (sts[REG_STS_A] & PCA9468_BIT_VFLT_LOOP_STS)	{
				/* V_FLOAT loop is in regulation */
				pr_info("%s: V_FLOAT loop interrupt\n", __func__);
				/* Disable CHG_PHASE_M */
				ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_INT1_MSK, 
										PCA9468_BIT_CHG_PHASE_M, PCA9468_BIT_CHG_PHASE_M);
				if (ret < 0) {
					handled = false;
					return handled;
				}
				/* Go to Pre CV Mode */
				pca9468->timer_id = TIMER_ENTER_CVMODE;
				pca9468->timer_period = 10;
				schedule_delayed_work(&pca9468->work, msecs_to_jiffies(pca9468->timer_period));
			} else if (sts[REG_STS_A] & PCA9468_BIT_IIN_LOOP_STS) {
				/* IIN loop or ICHG loop is in regulation */
				pr_info("%s: IIN loop interrupt\n", __func__);
			} else if (sts[REG_STS_A] & PCA9468_BIT_CHG_LOOP_STS) {
				/* ICHG loop is in regulation */
				pr_info("%s: ICHG loop interrupt\n", __func__);
			}
		}
		handled = true;
	}

	if (masked_int & PCA9468_BIT_CTRL_LIMIT_INT) {
		/* CTRL_LIMIT interrupt happened */
		if (int1[REG_INT1_STS] & PCA9468_BIT_CTRL_LIMIT_STS) {
			/* No Loop is active or OCP */
			if (sts[REG_STS_B] & PCA9468_BIT_OCP_FAST_STS) {
				/* Input fast over current */
				dev_err(pca9468->dev, "IIN > 50A instantaneously\n");
			}
			if (sts[REG_STS_B] & PCA9468_BIT_OCP_AVG_STS) {
				/* Input average over current */
				dev_err(pca9468->dev, "IIN > IIN_CFG*150percent\n");
			}
		}
		handled = true;
	}

	if (masked_int & PCA9468_BIT_TEMP_REG_INT) {
		/* TEMP_REG interrupt happened */
		if (int1[REG_INT1_STS] & PCA9468_BIT_TEMP_REG_STS) {
			/* Device is in temperature regulation */
			dev_err(pca9468->dev, "Device is in temperature regulation\n");
		}
		handled = true;
	}

	if (masked_int & PCA9468_BIT_ADC_DONE_INT) {
		/* ADC complete interrupt happened */
		dev_dbg(pca9468->dev, "ADC has been completed\n");
		handled = true;
	}

	if (masked_int & PCA9468_BIT_TIMER_INT) {
		/* Timer falut interrupt happened */
		if (int1[REG_INT1_STS] & PCA9468_BIT_TIMER_STS) {
			if (sts[REG_STS_B] & PCA9468_BIT_CHARGE_TIMER_STS) {
				/* Charger timer is expired */
				dev_err(pca9468->dev, "Charger timer is expired\n");
			}
			if (sts[REG_STS_B] & PCA9468_BIT_WATCHDOG_TIMER_STS) {
				/* Watchdog timer is expired */
				dev_err(pca9468->dev, "Watchdog timer is expired\n");
			}
		}
		handled = true;
	}

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static int pca9468_irq_init(struct pca9468_charger *pca9468,
			   struct i2c_client *client)
{
	const struct pca9468_platform_data *pdata = pca9468->pdata;
	int ret, msk, irq;

	pr_info("%s: =========START=========\n", __func__);

	irq = gpio_to_irq(pdata->irq_gpio);

	ret = gpio_request_one(pdata->irq_gpio, GPIOF_IN, client->name);
	if (ret < 0)
		goto fail;

	ret = request_threaded_irq(irq, NULL, pca9468_interrupt_handler,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   client->name, pca9468);
	if (ret < 0)
		goto fail_gpio;

	/*
	 * Configure the Mask Register for interrupts: disable all interrupts by default.
	 */
	msk = (PCA9468_BIT_V_OK_M | 
		   PCA9468_BIT_NTC_TEMP_M | 
		   PCA9468_BIT_CHG_PHASE_M |
		   PCA9468_BIT_CTRL_LIMIT_M |
		   PCA9468_BIT_TEMP_REG_M |
		   PCA9468_BIT_ADC_DONE_M |
		   PCA9468_BIT_TIMER_M);
	ret = regmap_write(pca9468->regmap, PCA9468_REG_INT1_MSK, msk);
	if (ret < 0)
		goto fail_wirte;
	
	client->irq = irq;
	return 0;

fail_wirte:
	free_irq(irq, pca9468);
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
static int get_const_charge_current(struct pca9468_charger *pca9468)
{
	int ret, intval;
	unsigned int val;

	if (!pca9468->mains_online)
		return -ENODATA;

	ret = regmap_read(pca9468->regmap, PCA9468_REG_ICHG_CTRL, &val);
	if (ret < 0)
		return ret;

	intval = (val & PCA9468_BIT_ICHG_CFG) * 100000;

	return intval;
}

/*
 * Returns the constant charge voltage programmed
 * into the charger in uV.
 */
static int get_const_charge_voltage(struct pca9468_charger *pca9468)
{
	int ret, intval;
	unsigned int val;

	if (!pca9468->mains_online)
		return -ENODATA;

	ret = regmap_read(pca9468->regmap, PCA9468_REG_V_FLOAT, &val);
	if (ret < 0)
		return ret;
	
	intval = (val * 5 + 3725) * 1000;

	return intval;
}

/*
 * Returns the enable or disable value.
 * into 1 or 0.
 */
static int get_charging_enabled(struct pca9468_charger *pca9468)
{
	int ret, intval;
	unsigned int val;
	
	ret = regmap_read(pca9468->regmap, PCA9468_REG_START_CTRL, &val);
	if (ret < 0)
		return ret;

	intval = (val & PCA9468_BIT_STANDBY_EN) ? 0 : 1;

	return intval;
}

static int pca9468_mains_set_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     const union power_supply_propval *val)
{
#if (LINUX_VERSION_CODE > KERNEL_VERSION(4, 1, 0))
	struct pca9468_charger *pca9468 = power_supply_get_drvdata(psy);
#else
	struct pca9468_charger *pca9468 =
		container_of(psy, struct pca9468_charger, mains);
#endif
	int ret;

	pr_info("%s: =========START=========\n", __func__);

	switch (prop) {
	/* Todo - Insert code */
	/* It needs modification by a customer */
	/* The customer make a decision to start charging and stop charging property */
	
	case POWER_SUPPLY_PROP_ONLINE:	/* need to change property */
		if (val->intval == 0) {
			// Stop Direct charging 
			cancel_delayed_work(&pca9468->work);
			cancel_delayed_work(&pca9468->pps_work);
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_ID_NONE;
			pca9468->timer_period = 0;
			mutex_unlock(&pca9468->lock);
			wake_unlock(&pca9468->monitor_wake_lock);

			ret = pca9468_set_charging(pca9468, false);
			if (ret < 0)
				goto error;
		} else {
			// Start Direct charging
			ret = pca9468_start_direct_charging(pca9468);
			if (ret < 0)
				goto error;
		}
		break;

	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		if (val->intval == 0) {
			// Stop Direct Charging
			cancel_delayed_work(&pca9468->work);
			cancel_delayed_work(&pca9468->pps_work);
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_ID_NONE;
			pca9468->timer_period = 0;
			mutex_unlock(&pca9468->lock);
			wake_unlock(&pca9468->monitor_wake_lock);

			ret = pca9468_set_charging(pca9468, false);
			if (ret < 0)
				goto error;
		} else {
			// Start Direct Charging
			/* Start 1sec timer for battery check */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_VBATMIN_CHECK;
			pca9468->timer_period = 5000;	/* The dealy time for PD state goes to PE_SNK_STATE */
			mutex_unlock(&pca9468->lock);
			schedule_delayed_work(&pca9468->work, msecs_to_jiffies(pca9468->timer_period));
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


static int pca9468_mains_get_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     union power_supply_propval *val)
{
	int ret;
#if (LINUX_VERSION_CODE > KERNEL_VERSION(4, 1, 0))
	struct pca9468_charger *pca9468 = power_supply_get_drvdata(psy);
#else
	struct pca9468_charger *pca9468 =
		container_of(psy, struct pca9468_charger, mains);
#endif	

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = pca9468->mains_online;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = get_const_charge_voltage(pca9468);
		if (ret < 0)
			return ret;
		else
			val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = get_const_charge_current(pca9468);
		if (ret < 0)
			return ret;
		else
			val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		ret = get_charging_enabled(pca9468);
		if (ret < 0)
			return ret;
		else
			val->intval = ret;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property pca9468_mains_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
};


static const struct regmap_config pca9468_regmap = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= PCA9468_MAX_REGISTER,
};

#if (LINUX_VERSION_CODE > KERNEL_VERSION(4, 1, 0))
static const struct power_supply_desc pca9468_mains_desc = {
	.name		= "pca9468-mains",
	.type		= POWER_SUPPLY_TYPE_MAINS,
	.get_property	= pca9468_mains_get_property,
	.set_property 	= pca9468_mains_set_property,
	.properties	= pca9468_mains_properties,
	.num_properties	= ARRAY_SIZE(pca9468_mains_properties),
};
#endif

#if defined(CONFIG_OF)
static int of_pca9468_dt(struct device *dev, struct pca9468_platform_data *pdata)
{
	struct device_node *np_pca9468 = dev->of_node;
	int ret;
	if(!np_pca9468)
		return -EINVAL;

	/* irq gpio */
	pdata->irq_gpio = of_get_named_gpio(np_pca9468, "pca9468,irq-gpio", 0);
	pr_info("%s: irq-gpio: %u \n", __func__, pdata->irq_gpio);

	/* input current limit */
	ret = of_property_read_u32(np_pca9468, "pca9468,input-current-limit",
						   &pdata->iin_cfg);
	if (ret) {
		pr_info("%s: pca9468,input-current-limit is Empty\n", __func__);
		pdata->iin_cfg = PCA9468_IIN_CFG_DFT;
	}
	pr_info("%s: pca9468,iin_cfg is %d\n", __func__, pdata->iin_cfg);

	/* charging current */
	ret = of_property_read_u32(np_pca9468, "pca9468,charging-current",
							   &pdata->ichg_cfg);
	if (ret) {
		pr_info("%s: pca9468,charging-current is Empty\n", __func__);
		pdata->ichg_cfg = PCA9468_ICHG_CFG_DFT;
	}
	pr_info("%s: pca9468,ichg_cfg is %d\n", __func__, pdata->ichg_cfg);

	/* charging float voltage */
	ret = of_property_read_u32(np_pca9468, "pca9468,float-voltage",
							   &pdata->v_float);
	if (ret) {
		pr_info("%s: pca9468,float-voltage is Empty\n", __func__);
		pdata->v_float = PCA9468_VFLOAT_DFT;
	}
	pr_info("%s: pca9468,v_float is %d\n", __func__, pdata->v_float);

	/* sense resistance */
	ret = of_property_read_u32(np_pca9468, "pca9468,sense-resistance",
							   &pdata->snsres);
	if (ret) {
		pr_info("%s: pca9468,sense-resistance is Empty\n", __func__);
		pdata->snsres = PCA9468_SENSE_R_DFT;
	}
	pr_info("%s: pca9468,snsres is %d\n", __func__, pdata->snsres);

	return 0;
}
#else
static int of_pca9468_dt(struct device *dev, struct pca9468_platform_data *pdata)
{
	return 0;
}
#endif /* CONFIG_OF */

#ifdef CONFIG_USBPD_PHY_QCOM
static int pca9468_usbpd_setup(struct pca9468_charger *pca9468)
{
	int ret = 0;
	const char *pd_phandle = "usbpd-phy";

	pca9468->pd = devm_usbpd_get_by_phandle(pca9468->dev, pd_phandle);

	if (IS_ERR(pca9468->pd)) {
		pr_err("get_usbpd phandle failed (%ld)\n",
				PTR_ERR(pca9468->pd));
		return PTR_ERR(pca9468->pd);
	}

	return ret;
}
#endif

static int pca9468_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{	
#if (LINUX_VERSION_CODE > KERNEL_VERSION(4, 1, 0))
	static char *battery[] = { "pca9468-battery" };
	struct power_supply_config mains_cfg = {};
#endif
	struct pca9468_platform_data *pdata;
	struct device *dev = &client->dev;
	struct pca9468_charger *pca9468_chg;
	int ret;

	pr_info("%s: =========START=========\n", __func__);

	pca9468_chg = devm_kzalloc(dev, sizeof(*pca9468_chg), GFP_KERNEL);
	if (!pca9468_chg)
		return -ENOMEM;

#if defined(CONFIG_OF)
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev, sizeof(struct pca9468_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory \n");
			return -ENOMEM;
		}

		ret = of_pca9468_dt(&client->dev, pdata);
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

	i2c_set_clientdata(client, pca9468_chg);

	mutex_init(&pca9468_chg->lock);
	pca9468_chg->dev = &client->dev;
	pca9468_chg->pdata = pdata;
	pca9468_chg->charging_state = DC_STATE_NO_CHARGING;

	wake_lock_init(&pca9468_chg->monitor_wake_lock, WAKE_LOCK_SUSPEND,
		       "pca9468-charger-monitor");

	/* initialize work */
	INIT_DELAYED_WORK(&pca9468_chg->work, pca9468_timer_work);
	pca9468_chg->timer_id = TIMER_ID_NONE;
	pca9468_chg->timer_period = 0;

	INIT_DELAYED_WORK(&pca9468_chg->pps_work, pca9468_pps_request_work);

	pca9468_chg->regmap = devm_regmap_init_i2c(client, &pca9468_regmap);
	if (IS_ERR(pca9468_chg->regmap))
		return PTR_ERR(pca9468_chg->regmap);

#ifdef CONFIG_USBPD_PHY_QCOM
	if (pca9468_usbpd_setup(pca9468_chg)) {
		dev_err(dev, "Error usbpd setup!\n");
		pca9468_chg->pd = NULL;
	}
#endif

	ret = pca9468_hw_init(pca9468_chg);
	if (ret < 0)
		return ret;

#if (LINUX_VERSION_CODE > KERNEL_VERSION(4, 1, 0))
	mains_cfg.supplied_to = battery;
	mains_cfg.num_supplicants = ARRAY_SIZE(battery);
	mains_cfg.drv_data = pca9468_chg;
	pca9468_chg->mains = power_supply_register(dev, &pca9468_mains_desc,
					   &mains_cfg);

	if (IS_ERR(pca9468_chg->mains))
		return PTR_ERR(pca9468_chg->mains);
#else
	pca9468_chg->mains.name		= "pca9468-mains";
	pca9468_chg->mains.type		= POWER_SUPPLY_TYPE_MAINS;
	pca9468_chg->mains.get_property	= pca9468_mains_get_property;
	pca9468_chg->mains.set_property	= pca9468_mains_set_property;
	pca9468_chg->mains.properties	= pca9468_mains_properties;
	pca9468_chg->mains.num_properties	= ARRAY_SIZE(pca9468_mains_properties);

	ret = power_supply_register(dev, &pca9468_chg->mains);
	if (ret) {
		dev_err(dev, "failed: power supply register\n");
		return ret;
	}
#endif	
	


	/*
	 * Interrupt pin is optional. If it is connected, we setup the
	 * interrupt support here.
	 */
	if (pdata->irq_gpio >= 0) {
		ret = pca9468_irq_init(pca9468_chg, client);
		if (ret < 0) {
			dev_warn(dev, "failed to initialize IRQ: %d\n", ret);
			dev_warn(dev, "disabling IRQ support\n");
		}
	}

	pr_info("%s: =========END=========\n", __func__);

	return 0;
}

static int pca9468_remove(struct i2c_client *client)
{
	struct pca9468_charger *pca9468_chg = i2c_get_clientdata(client);

	if (client->irq) {
		free_irq(client->irq, pca9468_chg);
		gpio_free(pca9468_chg->pdata->irq_gpio);
	}

	wake_lock_destroy(&pca9468_chg->monitor_wake_lock);
#if (LINUX_VERSION_CODE > KERNEL_VERSION(4, 1, 0))
	power_supply_unregister(pca9468_chg->mains);
#else
	power_supply_unregister(&pca9468_chg->mains);
#endif
	return 0;
}

static const struct i2c_device_id pca9468_id[] = {
	{ "pca9468", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pca9468_id);

#if defined(CONFIG_OF)
static struct of_device_id pca9468_i2c_dt_ids[] = {
	{ .compatible = "nxp,pca9468" },
	{ },
};
MODULE_DEVICE_TABLE(of, pca9468_i2c_dt_ids);
#endif /* CONFIG_OF */

#if defined(CONFIG_PM)
static void pca9468_check_and_update_charging_timer(struct pca9468_charger *pca9468)
{
	unsigned long current_time = 0, next_update_time, time_left;

	get_current_time(&current_time);
	
	if (pca9468->timer_id != TIMER_ID_NONE)	{
		next_update_time = pca9468->last_update_time + (pca9468->timer_period / 1000);	// unit is second

		pr_info("%s: current_time=%ld, next_update_time=%ld\n", __func__, current_time, next_update_time);

		if (next_update_time > current_time)
			time_left = next_update_time - current_time;
		else
			time_left = 0;

		mutex_lock(&pca9468->lock);
		pca9468->timer_period = time_left * 1000;	// ms unit
		mutex_unlock(&pca9468->lock);
		schedule_delayed_work(&pca9468->work, msecs_to_jiffies(pca9468->timer_period));
		pr_info("%s: timer_id=%d, time_period=%ld\n", __func__, pca9468->timer_id, pca9468->timer_period);
	}
	pca9468->last_update_time = current_time;
}


static int pca9468_suspend(struct device *dev)
{
	struct pca9468_charger *pca9468 = dev_get_drvdata(dev);

	pr_info("%s: cancel delayed work\n", __func__);

	/* cancel delayed_work */
	cancel_delayed_work(&pca9468->work);
	return 0;
}

static int pca9468_resume(struct device *dev)
{
	struct pca9468_charger *pca9468 = dev_get_drvdata(dev);

	pr_info("%s: update_timer\n", __func__);

	/* Update the current timer */
	pca9468_check_and_update_charging_timer(pca9468);

	return 0;
}
#else
#define pca9468_suspend		NULL
#define pca9468_resume		NULL
#endif

const struct dev_pm_ops pca9468_pm_ops = {
	.suspend = pca9468_suspend,
	.resume = pca9468_resume,
};

static struct i2c_driver pca9468_driver = {
	.driver = {
		.name = "pca9468",
#if defined(CONFIG_OF)
		.of_match_table = pca9468_i2c_dt_ids,
#endif /* CONFIG_OF */
#if defined(CONFIG_PM)
		.pm = &pca9468_pm_ops,
#endif
	},
	.probe        = pca9468_probe,
	.remove       = pca9468_remove,
	.id_table     = pca9468_id,
};

module_i2c_driver(pca9468_driver);

MODULE_AUTHOR("Clark Kim <clark.kim@nxp.com>");
MODULE_DESCRIPTION("PCA9468 charger driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.8.1");
