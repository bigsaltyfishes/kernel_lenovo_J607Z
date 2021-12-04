/*
 * Copyright (C) 2011 ST-Ericsson SA.
 * Copyright (C) 2009 Motorola, Inc.
 *
 * License Terms: GNU General Public License v2
 *
 * Simple driver for National Semiconductor sgm Backlight driver chip
 *
 * Author: Shreshtha Kumar SAHU <shreshthakumar.sahu@stericsson.com>
 * based on leds-sgm.c by Dan Murphy <D.Murphy@motorola.com>
 */

#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/backlight.h>


/**
 * struct sgm_data
 * @led_dev: led class device
 * @client: i2c client
 * @pdata: sgm platform data
 * @mode: mode of operation - manual, ALS, PWM
 * @regulator: regulator
 * @brightness: previous brightness value
 * @enable: regulator is enabled
 */


#define LP6280_NAME "lp6280-bl"

#define LP6280_DEV "lp6280-bl"

struct lp6280_data {
	struct led_classdev led_dev;
	struct i2c_client *client;
	struct device dev;
	struct i2c_adapter *adapter;
	unsigned short addr;
	struct mutex lock;
	bool enable;
	u8 full_scale_current;
	bool brt_code_enable;
	u16 *brt_code_table;
	int hwen_gpio;
	int enn_gpio;
	bool using_lsb;
	unsigned int ramp_on_time;
	unsigned int ramp_off_time;
	unsigned int i2c_trans_dim;
	unsigned int channel;
	unsigned int ovp_level;
	unsigned int frequency;
	unsigned int induct_current;
	unsigned int flash_current;
	unsigned int flash_timeout;
};

int i2c_lp6280_write(struct i2c_client *client, uint8_t command, uint8_t data)
{
	int retry/*, loop_i*/;
	uint8_t buf[1 + 1];
	uint8_t toRetry = 5;
	
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1 + 1,
			.buf = buf,
		}
	};
	buf[0] = command;
	buf[1] = data;
	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		//msleep(20);
	}
	
	pr_info("i2c_lp6280_write client->addr = %x \n",client->addr);
	if (retry == toRetry) {
		pr_info("%s: i2c_write_block retry over %d\n",
			__func__, toRetry);
		return -EIO;
	}
	return 0;

}

static int lp6280_init_registers(struct lp6280_data *drvdata)
{
	int ret = 0;
	
	ret |= i2c_lp6280_write(drvdata->client,0x00, 0x11);
	ret |= i2c_lp6280_write(drvdata->client,0x01, 0x11);
	return ret;
}

static void lp6280_get_dt_data(struct device *dev, struct lp6280_data *drvdata)
{
	struct device_node *np = dev->of_node;

	drvdata->hwen_gpio = of_get_named_gpio(np, "lp6280,hwen-gpio", 0);
	pr_info("%s drvdata->hwen_gpio --<%d>\n", __func__, drvdata->hwen_gpio);

	drvdata->enn_gpio = of_get_named_gpio(np, "lp6280,enn-gpio", 0);
	pr_info("%s drvdata->enn_gpio --<%d>\n", __func__, drvdata->enn_gpio);

}

static void lp6280_hwen_pin_ctrl(int gpio,int en)
{
	if (gpio_is_valid(gpio)) {
		if (en) {
			pr_info("hwen pin %d ,is going to be high!\n", gpio);
			gpio_set_value(gpio, true);
			usleep_range(3500, 4000);
		} else {
			pr_info("hwen pin is %d going to be low!\n", gpio);
			gpio_set_value(gpio, false);
			usleep_range(1000, 2000);
		}
	}
}


static int lp6280_gpio_init(int gpio,int value)
{

	int ret;
	if (gpio_is_valid(gpio)) {
		ret = gpio_request(gpio, "hwen_gpio");
		if (ret < 0) {
			pr_err("failed to request gpio\n");
			return -1;
		}
		ret = gpio_direction_output(gpio, 1);
		pr_info(" request gpio init\n");
		if (ret < 0) {
			pr_err("failed to set output");
			gpio_free(gpio);
			return ret;
		}
		pr_info("gpio %d is valid!\n",gpio);
		lp6280_hwen_pin_ctrl(gpio,value);
	}

	return 0;
}

static int lp6280_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct lp6280_data *drvdata;
	int err = 0;
    
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s : I2C_FUNC_I2C not supported\n", __func__);
		err = -EIO;
		goto err_out;
	}

	if (!client->dev.of_node) {
		pr_err("%s : no device node\n", __func__);
		err = -ENOMEM;
		goto err_out;
	}

	drvdata = kzalloc(sizeof(struct lp6280_data), GFP_KERNEL);
	if (drvdata == NULL) {
		pr_err("%s : kzalloc failed\n", __func__);
		err = -ENOMEM;
		goto err_out;
	}

	drvdata->client = client;
	drvdata->adapter = client->adapter;
	drvdata->addr = client->addr;
	drvdata->enable = true;
	drvdata->led_dev.default_trigger = "bkl-trigger";
	drvdata->led_dev.name = LP6280_DEV;
	mutex_init(&drvdata->lock);
	lp6280_get_dt_data(&client->dev, drvdata);
	i2c_set_clientdata(client, drvdata);
	lp6280_gpio_init(drvdata->hwen_gpio,1);
	usleep_range(1000, 2000);
	lp6280_gpio_init(drvdata->enn_gpio,1);

	err = led_classdev_register(&client->dev, &drvdata->led_dev);
	if (err < 0) {
		pr_err("%s : Register led class failed\n", __func__);
		err = -ENODEV;
		goto err_init;
	}

	lp6280_init_registers(drvdata);
	pr_info("%s exit\n", __func__);
	return 0;

err_init:
	kfree(drvdata);
err_out:
	return err;
}

 static int lp6280_remove(struct i2c_client *client)
{
		struct lp6280_data *drvdata = i2c_get_clientdata(client);
		led_classdev_unregister(&drvdata->led_dev);
		kfree(drvdata);
		return 0;
}

static const struct i2c_device_id lp6280_id[] = {
	{LP6280_NAME, 0},
	{}
};
static struct of_device_id match_table[] = {
			{.compatible = "lp6280,lp6280-bl",}
};

MODULE_DEVICE_TABLE(i2c, lp6280_id);

static struct i2c_driver lp6280_i2c_driver = {
	.probe = lp6280_probe,
	.remove = lp6280_remove,
	.id_table = lp6280_id,
	.driver = {
		.name = LP6280_NAME,
		.owner = THIS_MODULE,
		.of_match_table = match_table,
	},
};


module_i2c_driver(lp6280_i2c_driver);

MODULE_DESCRIPTION("Back Light driver for lp6280");
MODULE_LICENSE("GPL v2");
