/*
 * nx30p6039.c - NXP High Voltage Over Voltage Protection Load Switch device driver
 *
 * Copyright (C) 2017 NXP
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
#include <linux/pm_runtime.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#if defined (CONFIG_OF)
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif /* CONFIG_OF */

#define BITS(_end, _start) ((BIT(_end) - BIT(_start)) + BIT(_end))
#define MASK2SHIFT(_mask)	__ffs(_mask)

//#define CONFIG_NX30P6093A	// NX30P6093A version

//
// Register Map
//
#define NX30P6093_REG_DEVICE_ID 		0x00	// Device ID, revision
#define NX30P6093_BIT_VENDOR_ID			BITS(7,3)
#define NX30P6093_BIT_VERSION_ID		BITS(2,0)

#define NX30P6093_REG_ENABLE			0x01	// Enable register
#define NX30P6093_BIT_VOUT_EN			BIT(7)
#define NX30P6093_BIT_DETC_EN			BIT(6)

#define NX30P6093_REG_STS				0x02	// Status  register
#define NX30P6093_BIT_PWRON_STS			BIT(7)
#define NX30P6093_BIT_OVER_TAG_STS		BIT(6)
#define NX30P6093_BIT_TMR_OUT_STS		BIT(5)
#define NX30P6093_BIT_SWON_STS			BIT(4)

#define NX30P6093_REG_FLAG				0x03	// Flag register
#define NX30P6093_BIT_OV_FLG			BIT(2)
#define NX30P6093_BIT_OC_FLG			BIT(1)
#define NX30P6093_BIT_OT_FLG			BIT(0)

#define NX30P6093_REG_INT_M				0x04	// Interrupt Mask register
#define NX30P6093_BIT_PWRON_STS			BIT(7)
#define NX30P6093_BIT_OVER_TAG_STS		BIT(6)
#define NX30P6093_BIT_TMR_OUT_STS		BIT(5)
#define NX30P6093_BIT_SWON_STS			BIT(4)
#define NX30P6093_BIT_OV_FLG			BIT(2)
#define NX30P6093_BIT_OC_FLG			BIT(1)
#define NX30P6093_BIT_OT_FLG			BIT(0)

#define NX30P6093_REG_OVLO_TRIG			0x05	// OVLO Trigger level register
#define NX30P6093_BIT_RNG				BITS(7,5)
#define NX30P6093_BIT_OV_SEL			BIT(3)
#define NX30P6093_BIT_OV				BITS(1,0)

#define NX30P6093_REG_ISRC_TO_VIN		0x06	// Isource to VIN register
#define NX30P6093_BIT_ISRC				BITS(3,0)

#define NX30P6093_REG_ISRC_WORKINGTIME	0x07	// Isource working time register
#define NX30P6093_BIT_TDET				BITS(7,4)
#define NX30P6093_BIT_DUTY				BITS(3,0)

#define NX30P6093_REG_VOL_TO_VIN		0x08	// Voltage to VIN register
#define NX30P6093_BIT_VIN7				BITS(7,0)

#define NX30P6093_REG_SET_TAG_ON_VIN	0x09	// Set Tag on VIN register
#define NX30P6093_BIT_TVIN7				BITS(7,0)

#define NX30P6093_REG_ADD_OVP			0x0E	// Additional OVP register
#define NX30P6093_BIT_AOVP				BITS(1,0)

#define NX30P6093_REG_SLEW_RATE_TUNE	0x0F	// Slew rate Tune register
#define NX30P6093_BIT_SRT				BITS(2,0)

#define NX30P6093_MAX_REGISTER			NX30P6093_REG_SLEW_RATE_TUNE


/* Customer Definition */
#define NX30P6093_VIN_TAG_RESOLUTION	10547	// 10546.875uV
#define NX30P6093_VIN_TAG_1P82V			0xAD	// 1.82V


/*ISRC set */
enum {
	ISRC_0UA,
	ISRC_1UA,
	ISRC_2UA,
	ISRC_3UA,
	ISRC_4UA,
	ISRC_5UA,
	ISRC_10UA,
	ISRC_20UA,
	ISRC_50UA,
	ISRC_100UA,
	ISRC_200UA,
	ISRC_500UA,
	ISRC_1000UA,
	ISRC_2000UA,
	ISRC_5000UA,
	ISRC_10000UA,
};

/*TDET set */
enum {
	TDET_200US,
	TDET_400US,
	TDET_1000US,
	TDET_2000US,
	TDET_4000US,
	TDET_10000US,
	TDET_20000US,
	TDET_40000US,
	TDET_100000US,
	TDET_200000US,
	TDET_400000US,
	TDET_1000000US,
	TDET_2000000US,
	TDET_4000000US,
	TDET_10000000US,
	TDET_ALWAYSON,
};

/*DUTY set */
enum {
	DUTY_SINGLEPULSE,
	DUTY_10MS,
	DUTY_20MS,
	DUTY_50MS,
	DUTY_100MS,
	DUTY_200MS,
	DUTY_500MS,
	DUTY_1000MS,
	DUTY_2000MS,
	DUTY_3000MS,
	DUTY_6000MS,
	DUTY_12000MS,
	DUTY_30000MS,
	DUTY_60000MS,
	DUTY_120000MS,
	DUTY_300000MS,
};


/*
 * struct nx30p6093_platform_data - nx30p6093 platform data
 * @ irq_gpio : gpio pin for /IRQ
 * @ en_gpio : gpio pin for /EN
 */
struct nx30p6093_platform_data {
	unsigned int	irq_gpio;	/* GPIO pin that's connected to INT# */
	unsigned int 	en_gpio;	/* GPIO pin that's connneted to EN# */
	unsigned int 	vin_tag;	/* VIN TAG voltage - unit : uV */
};

/**
 * struct nx30p6093_ovp - nx30p6093 ovp instance
 * @lock: protects concurrent access to online variables
 * @dev: pointer to device
 * @regmap: pointer to driver regmap
 * @pdata: pointer to platform data
 */
struct nx30p6093_ovp {
	struct mutex		lock;
	struct device		*dev;
	struct regmap		*regmap;
	struct nx30p6093_platform_data *pdata;
};


/*******************************/
/* Moisture detection function */
/*******************************/
/* This function needs some modification by a customer */
static int nx30p6093_moisture_detection_enable(struct nx30p6093_ovp *nx30p6093)
{
	int val, ret;

	pr_info("%s: =========START=========\n", __func__);

	val = 1 << MASK2SHIFT(NX30P6093_BIT_DETC_EN);  // set DETC_EN
	ret = regmap_update_bits(nx30p6093->regmap, NX30P6093_REG_ENABLE,
							NX30P6093_BIT_DETC_EN, val);
	if (ret < 0)
		return ret;
	
	udelay(2000); // 2ms delay for wakeup time
	
	// Tdet = 10ms
	val = TDET_10000US << MASK2SHIFT(NX30P6093_BIT_TDET);
	ret = regmap_update_bits(nx30p6093->regmap, NX30P6093_REG_ISRC_WORKINGTIME,
							NX30P6093_BIT_TDET, val);
	if (ret < 0)
		return ret;
	
	// Tduty = Single pulse
	val = DUTY_SINGLEPULSE << MASK2SHIFT(NX30P6093_BIT_DUTY);
	ret = regmap_update_bits(nx30p6093->regmap, NX30P6093_REG_ISRC_WORKINGTIME,
							NX30P6093_BIT_DUTY, val);
	if (ret < 0)
		return ret;
	
	// VIN Tag
	val = nx30p6093->pdata->vin_tag/NX30P6093_VIN_TAG_RESOLUTION;
	ret = regmap_update_bits(nx30p6093->regmap, NX30P6093_REG_SET_TAG_ON_VIN,
							NX30P6093_BIT_TVIN7, val);
	if (ret < 0)
		return ret;
	
	// Isource = 2000uA
	val = ISRC_2000UA << MASK2SHIFT(NX30P6093_BIT_ISRC);
	ret = regmap_update_bits(nx30p6093->regmap, NX30P6093_REG_ISRC_TO_VIN,
							NX30P6093_BIT_ISRC, val);
	
	pr_info("%s: End, ret=%d\n", __func__, ret);
	
	return ret;
}


static int nx30p6093_moisture_detection_disable(struct nx30p6093_ovp *nx30p6093)
{	
	int val, ret;
	
	pr_info("%s: =========START=========\n", __func__);
	
	val = 0;  // set DETC_EN
	ret = regmap_update_bits(nx30p6093->regmap, NX30P6093_REG_ENABLE,
							NX30P6093_BIT_DETC_EN, val);
	
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}


static ssize_t nx30p6093_set_moisture_detection(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct nx30p6093_ovp *nx30p6093 = dev_get_drvdata(dev);

	if (!strncmp(buf, "enable", 6)) {
		nx30p6093_moisture_detection_enable(nx30p6093);
	}
	else if (!strncmp(buf, "disable", 7)){
		nx30p6093_moisture_detection_disable(nx30p6093);
	}

	return count;
}

static DEVICE_ATTR(moisture, S_IWUSR, NULL, nx30p6093_set_moisture_detection);

static struct attribute *nx30p6093_attributes[] = {
	&dev_attr_moisture.attr,
	NULL
};

static const struct attribute_group nx30p6093_group = {
	.attrs = nx30p6093_attributes,
};


static irqreturn_t nx30p6093_interrupt_handler(int irq, void *data)
{
	struct nx30p6093_ovp *nx30p6093 = data;
	unsigned int sts, flag, int_mask;	/* status and int_mask registers */
	u8 masked_int;	/* masked interrupt */
	bool handled = false;
	int ret;

	pr_info("%s: =========START=========\n", __func__);


	/* Read status register */
	ret = regmap_read(nx30p6093->regmap, NX30P6093_REG_STS, &sts);
	if (ret < 0) {
		pr_err("reading status register failed\n");
		return IRQ_NONE;
	}

	/* Read flag register */
	ret = regmap_read(nx30p6093->regmap, NX30P6093_REG_FLAG, &flag);
	if (ret < 0) {
		pr_err("reading flag register failed\n");
		return IRQ_NONE;
	}
	
	/* Read int mask register */
	ret = regmap_read(nx30p6093->regmap, NX30P6093_REG_INT_M, &int_mask);
	if (ret < 0) {
		pr_err("reading STS_X failed\n");
		return IRQ_NONE;
	}
	
	/* Check Interrupt */
	masked_int = ( sts | flag ) & int_mask;
	if (masked_int & NX30P6093_BIT_PWRON_STS) {
		/* Charger plugged in.
		H/W disables impedance detection */
		handled = true;
	}
#if defined(CONFIG_NX30P6093A)
	else if (masked_int & NX30P6093_BIT_OVER_TAG_STS) {
		/* NX30P6093A_BIT_OVER_TAG_STS is 1 */
		/*VIN is not good and < 300kohm.
		need to report to OS system for the abnormal */
		pr_info("VIN is not good and Impedance < 300KOhm\n");
		handled = true;		
	}
#else
    else if (masked_int & NX30P6093_BIT_TMR_OUT_STS) {
		if (masked_int & NX30P6093_BIT_OVER_TAG_STS) {
			/*VIN is good and Impedance > 100kohm */
			pr_info("VIN is good and Impedance > 100KOhm\n");
		}
		else {
			/* NX30P6093_BIT_OVER_TAG_STS is 0 */
			/*VIN is not good and < 100kohm.
			need to report to OS system for the abnormal */
			pr_info("VIN is not good and Impedance < 100KOhm\n");
		}
		handled = true;
	}
#endif

	if (masked_int & NX30P6093_BIT_OV_FLG) {
		/* Over Voltage Protection */
		pr_info("Over Voltage Protection triggered\n");
		handled = true;
	}
	if (masked_int & NX30P6093_BIT_OC_FLG) {
		/* Over Current Protection */
		pr_info("Over Current Protection triggered\n");
		handled = true;
	}
	if (masked_int & NX30P6093_BIT_OT_FLG) {
		/* Over Temperature Protection */
		pr_info("Over Temperature Protection triggered\n");
		handled = true;
	}

	pr_info("%s: End, handled=%d\n", __func__, handled);
	
	return handled ? IRQ_HANDLED : IRQ_NONE;
}


static int nx30p6093_irq_init(struct nx30p6093_ovp *nx30p6093,
			   struct i2c_client *client)
{
	const struct nx30p6093_platform_data *pdata = nx30p6093->pdata;
	unsigned int msk;
	int ret, irq;

	pr_info("%s: =========START=========\n", __func__);

	irq = gpio_to_irq(pdata->irq_gpio);

	ret = gpio_request_one(pdata->irq_gpio, GPIOF_IN, client->name);
	if (ret < 0)
		goto fail;

	ret = request_threaded_irq(irq, NULL, nx30p6093_interrupt_handler,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   client->name, nx30p6093);
	if (ret < 0)
		goto fail_gpio;

	/*
	 * Configure the Mask Register for interrupts: enable all interrupts by default.
	 */
#if defined(CONFIG_NX30P6093A)
	/* Disable TMR_OUT_STS interrupt not to wake up periodically */
	msk = ( NX30P6093_BIT_PWRON_STS |
			NX30P6093_BIT_OVER_TAG_STS |
			NX30P6093_BIT_SWON_STS |
			NX30P6093_BIT_OV_FLG |
			NX30P6093_BIT_OC_FLG |
			NX30P6093_BIT_OT_FLG );
#else
	msk = ( NX30P6093_BIT_PWRON_STS |
		    NX30P6093_BIT_OVER_TAG_STS |
		    NX30P6093_BIT_TMR_OUT_STS |
		    NX30P6093_BIT_SWON_STS |
		    NX30P6093_BIT_OV_FLG |
		    NX30P6093_BIT_OC_FLG |
		    NX30P6093_BIT_OT_FLG );
#endif
	ret = regmap_write(nx30p6093->regmap, NX30P6093_REG_INT_M, msk);
	if (ret < 0)
		goto fail_wirte;
	
	client->irq = irq;
	return 0;

fail_wirte:
	free_irq(irq, nx30p6093);
fail_gpio:
	gpio_free(pdata->irq_gpio);
fail:
	client->irq = 0;

	pr_info("%s: End, ret=%d\n", __func__, ret);
	
	return ret;
}

static const struct regmap_config nx30p6093_regmap = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= NX30P6093_MAX_REGISTER,
};


#if defined(CONFIG_OF)
static int of_nx30p6093_dt(struct device *dev, struct nx30p6093_platform_data *pdata)
{
	struct device_node *np_nx30p6093 = dev->of_node;
	int ret;
	
	if(!np_nx30p6093)
		return -EINVAL;

	/* irq gpio */
	pdata->irq_gpio = of_get_named_gpio(np_nx30p6093, "nx30p6093,irq-gpio", 0);
	pr_info("%s: irq-gpio: %u \n", __func__, pdata->irq_gpio);

	/* en gpio */
	pdata->en_gpio = of_get_named_gpio(np_nx30p6093, "nx30p6093,en-gpio", 0);
	pr_info("%s: en-gpio: %u \n", __func__, pdata->en_gpio);

	/* Vin Tag voltage */
	ret = of_property_read_u32(np_nx30p6093, "nx30p6093,vin-tag-voltage",
						   &pdata->vin_tag);
	if (ret) {
		pr_info("%s: vin-tag-voltage is Empty\n", __func__);
		pdata->vin_tag = NX30P6093_VIN_TAG_1P82V;
	}
	pr_info("%s: vin-tag-voltage: %u(uV)\n", __func__, pdata->vin_tag);
	return 0;
}
#else
static int of_nx30p6093_dt(struct device *dev, struct pca9468_platform_data *pdata)
{
	return 0;
}
#endif /* CONFIG_OF */

static int nx30p6093_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct nx30p6093_platform_data *pdata;
	struct device *dev = &client->dev;
	struct nx30p6093_ovp *nx30p6093;
	int ret;

	pr_info("%s: =========START=========\n", __func__);

	nx30p6093 = devm_kzalloc(dev, sizeof(*nx30p6093), GFP_KERNEL);
	if (!nx30p6093)
		return -ENOMEM;

#if defined(CONFIG_OF)
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev, sizeof(struct nx30p6093_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			pr_err("Failed to allocate memory \n");
			return -ENOMEM;
		}

		ret = of_nx30p6093_dt(&client->dev, pdata);
		if (ret < 0){
			pr_err("Failed to get device of_node \n");
			return -ENOMEM;
		}

		client->dev.platform_data = pdata;
	} 
	else {
		pdata = client->dev.platform_data;
	}
#else
	pdata = dev->platform_data;
#endif
	if (!pdata)
		return -EINVAL;

	i2c_set_clientdata(client, nx30p6093);

	mutex_init(&nx30p6093->lock);
	nx30p6093->dev = &client->dev;
	nx30p6093->pdata = pdata;

	nx30p6093->regmap = devm_regmap_init_i2c(client, &nx30p6093_regmap);
	if (IS_ERR(nx30p6093->regmap))
		return PTR_ERR(nx30p6093->regmap);

	/*
	 * Interrupt pin is optional. If it is connected, we setup the
	 * interrupt support here.
	 */
	if (pdata->irq_gpio >= 0) {
		ret = nx30p6093_irq_init(nx30p6093, client);
		if (ret < 0) {
			pr_err("failed to initialize IRQ: %d\n", ret);
			pr_err("disabling IRQ support\n");
		}
	}

	/* Set /en gpio pin */
	ret = devm_gpio_request(nx30p6093->dev, pdata->en_gpio, "en_gpio");
	if (ret) {
		pr_err("gpio_request (pin) failed\n");
		return ret;
	}

	/* set /en gpio pin to low - enable */	
	if (gpio_is_valid(nx30p6093->pdata->en_gpio)) {
		ret = gpio_direction_output(nx30p6093->pdata->en_gpio, 0);
		if (ret) {
			pr_err("unable to set dir for en gpio\n");
			return ret;
		}
	}

	/* Set the initial OVP voltage */
	/* OVLO Threshold : 11.05V, RNG - +800mV */
	ret = regmap_write(nx30p6093->regmap, NX30P6093_REG_OVLO_TRIG, 0x79);
	if (ret < 0) {
		pr_err("failed to set OVLO_TRIG: %d\n", ret);
		return ret;
	}

	ret = sysfs_create_group(&client->dev.kobj, &nx30p6093_group);
	if (ret) {
		pr_err("failed to create nx30p6093 attribute group\n");
		return ret;
	}

	/* Enable moisture detect */
	ret = nx30p6093_moisture_detection_enable(nx30p6093);
	if (ret < 0)
		return ret;

	pr_info("%s: =========END=========\n", __func__);

	return 0;
}

static int nx30p6093_remove(struct i2c_client *client)
{
	struct nx30p6093_ovp *nx30p6093 = i2c_get_clientdata(client);

	if (client->irq) {
		free_irq(client->irq, nx30p6093);
		gpio_free(nx30p6093->pdata->irq_gpio);
	}

	return 0;
}

static const struct i2c_device_id nx30p6093_id[] = {
	{ "nx30p6093", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nx30p6093_id);

#if defined(CONFIG_OF)
static struct of_device_id nx30p6093_i2c_dt_ids[] = {
	{ .compatible = "nxp,nx30p6093" },
	{ },
};
MODULE_DEVICE_TABLE(of, nx30p6093_i2c_dt_ids);
#endif /* CONFIG_OF */


#if defined(CONFIG_PM)
static int nx30p6093_suspend(struct device *dev)
{
	struct nx30p6093_ovp *nx30p6093 = dev_get_drvdata(dev);

	pr_info("%s: disable /en pin\n", __func__);

	/* disable /en gpio */
	gpio_direction_output(nx30p6093->pdata->en_gpio, 1);
	return 0;
}

static int nx30p6093_resume(struct device *dev)
{
	struct nx30p6093_ovp *nx30p6093 = dev_get_drvdata(dev);

	pr_info("%s: enable /en pin\n", __func__);

	/* enable /en gpio */
	gpio_direction_output(nx30p6093->pdata->en_gpio, 0);
	return 0;
}
#else
#define nx30p6093_suspend		NULL
#define nx30p6093_resume		NULL
#endif

const struct dev_pm_ops nx30p6093_pm_ops = {
	.suspend = nx30p6093_suspend,
	.resume = nx30p6093_resume,
};

static struct i2c_driver nx30p6093_driver = {
	.driver = {
		.name = "nx30p6093",
#if defined(CONFIG_OF)
		.of_match_table = nx30p6093_i2c_dt_ids,
#endif /* CONFIG_OF */
#if defined(CONFIG_PM)
		.pm = &nx30p6093_pm_ops,
#endif
	},
	.probe        = nx30p6093_probe,
	.remove       = nx30p6093_remove,
	.id_table     = nx30p6093_id,
};

module_i2c_driver(nx30p6093_driver);

MODULE_AUTHOR("Clark Kim <clark.kim@nxp.com>");
MODULE_DESCRIPTION("NX30P6093 OVP driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.3");
