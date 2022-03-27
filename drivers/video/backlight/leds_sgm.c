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


#define SGM_NAME "sgm-bl"

#define SGM_LED_DEV "sgm-bl"

struct sgm_data {
	struct led_classdev led_dev;
	struct i2c_client *client;
	struct device dev;
	struct i2c_adapter *adapter;
	unsigned short addr;
	struct mutex lock;
	struct work_struct work;
	enum led_brightness brightness;
	bool enable;
	u8 pwm_cfg;
	u8 full_scale_current;
	bool brt_code_enable;
	u16 *brt_code_table;
	int hwen_gpio;
	unsigned int  pwm_mode;
	bool using_lsb;
	unsigned int pwm_period;
	unsigned int full_scale_led;
	unsigned int ramp_on_time;
	unsigned int ramp_off_time;
	unsigned int pwm_trans_dim;
	unsigned int i2c_trans_dim;
	unsigned int channel;
	unsigned int ovp_level;
	unsigned int frequency;
	unsigned int default_brightness;
	unsigned int max_brightness;
	unsigned int induct_current;
	unsigned int flash_current;
	unsigned int flash_timeout;
	unsigned int bl_map;
	struct backlight_device *bl_dev;

};

#define MAX_BRIGHTNESS 4096
struct sgm_data *g_sgm_data;

int i2c_sgm_write(struct i2c_client *client, uint8_t command, uint8_t data)
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

		if (retry == toRetry) {
		pr_info("%s: i2c_write_block retry over %d\n",
			__func__, toRetry);
		return -EIO;
	}
	return 0;

}

static int sgm_init_registers(struct sgm_data *drvdata)
{
	int ret = 0;
	
	ret |= i2c_sgm_write(drvdata->client,0x11, 0x40);
	ret |= i2c_sgm_write(drvdata->client,0x19, 0xCC);
	ret |= i2c_sgm_write(drvdata->client,0x1A, 0x0C);

	return ret;
}

static int sgm_backlight_enable(struct sgm_data *drvdata)
{
	pr_info("%s enter.\n", __func__);
	drvdata->enable = true;

	return 0;
}

int sgm_brightness_set(struct sgm_data *drvdata, int brt_val)
{
	int err;
	if (drvdata->enable == false) {
		sgm_init_registers(drvdata);
		sgm_backlight_enable(drvdata);
	}

	if (brt_val < 0)
		brt_val =0;

	if (brt_val > MAX_BRIGHTNESS)
		brt_val = MAX_BRIGHTNESS;

	/* set the brightness in brightness control register*/
	err |= i2c_sgm_write(drvdata->client, 0x19, (brt_val >> 4));
	err |= i2c_sgm_write(drvdata->client, 0x1A, (brt_val & 0xf));

	drvdata->brightness = brt_val;
	if (drvdata->brightness == 0)
		drvdata->enable = false;
	return err;

}
static int sgm_bl_get_brightness(struct backlight_device *bl_dev)
{
		return bl_dev->props.brightness;
}

static inline int sgm_bl_update_status(struct backlight_device *bl_dev)
{
		struct sgm_data *drvdata = bl_get_data(bl_dev);
		int brt;

		if (bl_dev->props.state & BL_CORE_SUSPENDED)
				bl_dev->props.brightness = 0;

		brt = bl_dev->props.brightness;
		/*
		 * Brightness register should always be written
		 * not only register based mode but also in PWM mode.
		 */
		return sgm_brightness_set(drvdata, brt);
}

int sgm_backlight_device_set_brightness(struct backlight_device *bl_dev,
				    unsigned long brightness)
{
	int rc = -ENXIO;
	//struct sgm_data *drvdata = bl_get_data(bl_dev);

	mutex_lock(&bl_dev->ops_lock);
	if (bl_dev->ops) {
			if (brightness > bl_dev->props.max_brightness)
			brightness = bl_dev->props.max_brightness;

			pr_debug("set brightness to %lu\n", brightness);
			bl_dev->props.brightness = brightness;
			rc = sgm_bl_update_status(bl_dev);
		}
	mutex_unlock(&bl_dev->ops_lock);

	//backlight_generate_event(bl_dev, BACKLIGHT_UPDATE_SYSFS);

	return rc;
}
EXPORT_SYMBOL(sgm_backlight_device_set_brightness);

static const struct backlight_ops sgm_bl_ops = {
		.update_status = sgm_bl_update_status,
		.get_brightness = sgm_bl_get_brightness,
};

static void __sgm_work(struct sgm_data *led,
				enum led_brightness value)
{
	mutex_lock(&led->lock);
	sgm_brightness_set(led, value);
	mutex_unlock(&led->lock);
}

static void sgm_work(struct work_struct *work)
{
	struct sgm_data *drvdata = container_of(work,
					struct sgm_data, work);

	__sgm_work(drvdata, drvdata->led_dev.brightness);
}

static void sgm_set_brightness(struct led_classdev *led_cdev,
			enum led_brightness brt_val)
{
	struct sgm_data *drvdata;

	drvdata = container_of(led_cdev, struct sgm_data, led_dev);
	schedule_work(&drvdata->work);
}

static void sgm_get_dt_data(struct device *dev, struct sgm_data *drvdata)
{
	struct device_node *np = dev->of_node;
	//u32 bl_channel, temp;

	drvdata->hwen_gpio = of_get_named_gpio(np, "sgm,hwen-gpio", 0);
	pr_info("%s drvdata->hwen_gpio --<%d>\n", __func__, drvdata->hwen_gpio);

}


static int sgm_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct sgm_data *drvdata;
	struct backlight_device *bl_dev;
	struct backlight_properties props;

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

	drvdata = kzalloc(sizeof(struct sgm_data), GFP_KERNEL);
	if (drvdata == NULL) {
		pr_err("%s : kzalloc failed\n", __func__);
		err = -ENOMEM;
		goto err_out;
	}

	drvdata->client = client;
	drvdata->adapter = client->adapter;
	drvdata->addr = client->addr;
	drvdata->brightness = LED_OFF;
	drvdata->enable = true;
	drvdata->led_dev.default_trigger = "bkl-trigger";
	drvdata->led_dev.name = SGM_LED_DEV;
	drvdata->led_dev.brightness_set = sgm_set_brightness;
	drvdata->led_dev.max_brightness = MAX_BRIGHTNESS;
	mutex_init(&drvdata->lock);
	INIT_WORK(&drvdata->work, sgm_work);
	sgm_get_dt_data(&client->dev, drvdata);
	i2c_set_clientdata(client, drvdata);

	err = led_classdev_register(&client->dev, &drvdata->led_dev);
	if (err < 0) {
		pr_err("%s : Register led class failed\n", __func__);
		err = -ENODEV;
		goto err_init;
	}

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.brightness = MAX_BRIGHTNESS;
	props.max_brightness = MAX_BRIGHTNESS;
	bl_dev = backlight_device_register(SGM_NAME, &client->dev,
					drvdata, &sgm_bl_ops, &props);

	g_sgm_data = drvdata;

	sgm_init_registers(drvdata);
	pr_info("%s exit\n", __func__);
	return 0;

err_init:
	kfree(drvdata);
err_out:
	return err;
}

 static int sgm_remove(struct i2c_client *client)
{
		struct sgm_data *drvdata = i2c_get_clientdata(client);
		led_classdev_unregister(&drvdata->led_dev);
		kfree(drvdata);
		return 0;
}

static const struct i2c_device_id sgm_id[] = {
	{SGM_NAME, 0},
	{}
};
static struct of_device_id match_table[] = {
			{.compatible = "sgm,sgm-bl",}
};

MODULE_DEVICE_TABLE(i2c, sgm_id);

static struct i2c_driver sgm_i2c_driver = {
	.probe = sgm_probe,
	.remove = sgm_remove,
	.id_table = sgm_id,
	.driver = {
		.name = SGM_NAME,
		.owner = THIS_MODULE,
		.of_match_table = match_table,
	},
};


module_i2c_driver(sgm_i2c_driver);

MODULE_DESCRIPTION("Back Light driver for sgm");
MODULE_LICENSE("GPL v2");
