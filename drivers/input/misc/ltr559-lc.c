/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>
#include <linux/ctype.h>
#include <linux/pm_runtime.h>
#include <linux/device.h>
#include <linux/input/ltr559.h>
#include <linux/irq.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>
#include <linux/regulator/consumer.h>

#define SENSOR_NAME			"proximity"
#define LTR559_DRV_NAME		"ltr559"
#define LTR559_MANUFAC_ID	0x05

#define VENDOR_NAME				"lite-on"
#define LTR559_SENSOR_NAME		"ltr559als"
#define DRIVER_VERSION		"1.0"

#define SYS_AUTHORITY		(S_IRUGO|S_IWUGO)

#define PS_OFFSET_DATA_LEN 6

struct ltr559_data {

	struct i2c_client *client;
	struct input_dev *input_dev_als;
	struct input_dev *input_dev_ps;
	struct sensors_classdev als_cdev;
	struct sensors_classdev ps_cdev;

	/* pinctrl data*/
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	struct pinctrl_state *pin_sleep;

	struct ltr559_platform_data *platform_data;

	/* regulator data */
	bool power_state;
	struct regulator *vdd;
	struct regulator *vio;

	/* interrupt type is level-style */
	struct mutex lockw;
	struct mutex op_lock;

	struct delayed_work ps_work;
	struct delayed_work als_work;

	u8 ps_open_state;
	u8 als_open_state;

	u16 irq;

	u32 last_lux;

	bool ps_covered;
	u8   ps_offset_data_count;
	u16  ps_offset_data[PS_OFFSET_DATA_LEN];
	u16  ps_offset;
};

struct ltr559_reg {
	const char *name;
	u8 addr;
	u16 defval;
	u16 curval;
};

enum ltr559_reg_tbl{
	REG_ALS_CONTR,
	REG_PS_CONTR,
	REG_ALS_PS_STATUS,
	REG_INTERRUPT,
	REG_PS_LED,
	REG_PS_N_PULSES,
	REG_PS_MEAS_RATE,
	REG_ALS_MEAS_RATE,
	REG_MANUFACTURER_ID,
	REG_INTERRUPT_PERSIST,
	REG_PS_THRES_LOW,
	REG_PS_THRES_UP,
	REG_ALS_THRES_LOW,
	REG_ALS_THRES_UP,
	REG_ALS_DATA_CH1,
	REG_ALS_DATA_CH0,
	REG_PS_DATA
};

static int ltr559_als_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable);
static int ltr559_ps_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable);

static  struct ltr559_reg reg_tbl[] = {
		{
				.name   = "ALS_CONTR",
				.addr   = 0x80,
				.defval = 0x00,
				.curval = 0x19,
		},
		{
				.name = "PS_CONTR",
				.addr = 0x81,
				.defval = 0x00,
				.curval = 0x23, /* PS active with saturation indicator enabled. */
		},
		{
				.name = "ALS_PS_STATUS",
				.addr = 0x8c,
				.defval = 0x00,
				.curval = 0x00,
		},
		{
				.name = "INTERRUPT",
				.addr = 0x8f,
				.defval = 0x00,
				.curval = 0x01,
		},
		{
				.name = "PS_LED",
				.addr = 0x82,
				.defval = 0x7f,
				.curval = 0x7b,
		},
		{
				.name = "PS_N_PULSES",
				.addr = 0x83,
				.defval = 0x01,
				.curval = 0x02,
		},
		{
				.name = "PS_MEAS_RATE",
				.addr = 0x84,
				.defval = 0x02,
				.curval = 0x00,
		},
		{
				.name = "ALS_MEAS_RATE",
				.addr = 0x85,
				.defval = 0x03,
				.curval = 0x02,	/* 200ms */
		},
		{
				.name = "MANUFACTURER_ID",
				.addr = 0x87,
				.defval = 0x05,
				.curval = 0x05,
		},
		{
				.name = "INTERRUPT_PERSIST",
				.addr = 0x9e,
				.defval = 0x00,
				.curval = 0x13, /* Two consecutive PS values out of threshold range
				                   trigger PS interrupt. */
		},
		{
				.name = "PS_THRES_LOW",
				.addr = 0x92,
				.defval = 0x0000,
				.curval = 0x0000,
		},
		{
				.name = "PS_THRES_UP",
				.addr = 0x90,
				.defval = 0x07ff,
				.curval = 0x07ff,
		},
		{
				.name = "ALS_THRES_LOW",
				.addr = 0x99,
				.defval = 0x0000,
				.curval = 0x0000,
		},
		{
				.name = "ALS_THRES_UP",
				.addr = 0x97,
				.defval = 0xffff,
				.curval = 0x0000,
		},
		{
				.name = "ALS_DATA_CH1",
				.addr = 0x88,
				.defval = 0x0000,
				.curval = 0x0000,
		},
		{
				.name = "ALS_DATA_CH0",
				.addr = 0x8a,
				.defval = 0x0000,
				.curval = 0x0000,
		},
		{
				.name = "PS_DATA",
				.addr = 0x8d,
				.defval = 0x0000,
				.curval = 0x0000,
		},
};

static struct sensors_classdev sensors_light_cdev = {
	.name = "liteon-light",
	.vendor = "liteon",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "60000",
	.resolution = "0.0125",
	.sensor_power = "0.20",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev sensors_proximity_cdev = {
	.name = "liteon-proximity",
	.vendor = "liteon",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

/* To determine the noise of the proximity sensor, three different series
   of 10'000 measurements each have been performed:
   (1) With an object exactly 5cm away from the sensor.
   (2) With an object somewhat close to the sensor.
   (3) With an object very close to the sensor.
   The collected data are assumed to be normally distributed, which leads to
   the following mean values µ (which correspond to the sensor values) and
   standard deviations σ (which are measures for the noise levels):
           (1)       (2)        (3)
   µ:   41.24    462.00    1485.62
   σ:    2.493     3.655      6.640
   The measurements indicate a linear relationship between sensor value and
   noise level. Use a linear fit of the obtained results to calculate σ for
   arbitrary sensor values.
   To ensure a strong separation of the hysteresis levels, the difference
   between the sensor value and the upper threshold has been chosen to be equal
   to 2*6σ (as a function of the current sensor value). The lower threshold is
   half way between the sensor value and the upper threshold.
   Value limits:
   (a) The lower threshold is at least 2*6σ above zero.
   (b) The sensor is not to be triggered by an object more than 5cm away
       from it.
   (c) The upper threshold is at least 6σ below the maximum value, which
       is 0x07ff.
   From all this one can derive the formulas below. */

/* Returns a threshold that lies 6σ below val. */
static u16 ltr559_ps_thres_below(u32 val)
{
	if (val <= 14)
		return 0;
	if (val >= 0x07ff)
		return 1997;
	/* return -15.65 + 0.98372*val; */
	return (-925600 + 64404*val) >> 16;
}

/* Returns a threshold that lies 6σ above val. */
static u16 ltr559_ps_thres_low(u32 val)
{
	if (val <= 14)
		return 28;
	if (val >= 1905)
		return 1952;
	/* return 15.65 + 1.01628*val; */
	return (925600 + 66668*val) >> 16;
}

/* Returns a threshold that lies 2*6σ above val. */
static u16 ltr559_ps_thres_up(u32 val)
{
	if (val <= 14)
		return 42;
	if (val >= 1905)
		return 1999;
	/* return 31.30 + 1.03256*val; */
	return (1851201 + 67801*val) >> 16;
}

static s16 ltr559_ps_read(struct i2c_client *client)
{
	static bool saturation_logged = false;
	int ret;

	ret = i2c_smbus_read_word_data(client, LTR559_PS_DATA_0);

	/* 0x8000 is the saturation flag bit. */
	if (ret & 0x8000) {
		/* While the sensor is saturated, the interrupt will
		   trigger very frequently. Don't spam the log. */
		if(!saturation_logged) {
			pr_warning("%s: Proximity sensor is saturated, presumably due"
				" to very bright ambient light.\n", __func__);
			saturation_logged = true;
		}
		return -ENOMSG;
	}
	saturation_logged = false;

	/* 0x07ff is the maximum value. */
	if (ret > 0x07ff || ret < 0) {
		pr_err("%s: Error while reading data from proximity sensor"
			" (i2c_smbus_read_word_data returned %i).\n", __func__, ret);
		return -EBADMSG;
	}

	return ret;
}

static int ltr559_chip_reset(struct i2c_client *client)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, LTR559_PS_CONTR, MODE_PS_StdBy);/* ps standby mode */
	if(ret < 0)
	{
		dev_err(&client->dev, "%s write error\n",__func__);
		return ret;
	}
	ret = i2c_smbus_write_byte_data(client, LTR559_ALS_CONTR, 0x02);    /* reset */
	if(ret < 0)
	{
		dev_err(&client->dev, "%s reset chip fail\n",__func__);
		return ret;
	}

	return ret;
}

static void ltr559_ps_set_thresholds(struct i2c_client *client, u16 thres_low, u16 thres_up)
{
	i2c_smbus_write_word_data(client, LTR559_PS_THRES_LOW_0, thres_low);
	i2c_smbus_write_word_data(client, LTR559_PS_THRES_UP_0, thres_up);
}

static int ltr559_ps_enable(struct i2c_client *client, int on)
{
	struct ltr559_data *data = i2c_get_clientdata(client);
	int ret;

	if (on) {

		ret = i2c_smbus_write_byte_data(client, LTR559_PS_CONTR, reg_tbl[REG_PS_CONTR].curval);
		if(ret < 0) {
			pr_err("%s: Failed to enable proximity sensor. Error while writing to register"
				" REG_PS_CONTR (i2c_smbus_write_byte_data returned %i).\n", __func__, ret);
			return -EFAULT;
		ret = i2c_smbus_read_byte_data(client, LTR559_PS_CONTR);
		if(ret != reg_tbl[REG_PS_CONTR].curval) {
			pr_err("%s: Failed to enable proximity sensor. Error while writing to register"
				" REG_PS_CONTR (i2c_smbus_read_byte_data returned %i, but must be %i).\n",
				__func__, ret, reg_tbl[REG_PS_CONTR].curval);
		}

		msleep(WAKEUP_DELAY);

		ret = ltr559_ps_read(client);

		if (data->ps_offset_data_count == 0) { // First activation.

			if (ret < 0) {
				pr_warning("%s: Received no or invalid sensor data during initialization"
					" (ltr559_ps_read returned %i). Offset will now be set to its maximum"
					" value (=1905).\n", __func__, ret);
				data->ps_offset = 1905;
			}
			else if (ret > 1905)
				data->ps_offset = 1905;
			else
				data->ps_offset = ret;

			/* Assume that the sensor is not covered, even if this is not true.
			   After the first interrupt ps_covered will be set suitably.
			   However, in most cases, the sensor will work as expected right
			   away (before the first interrupt is triggered).
			   Only when the sensor value is high (meaning that the sensor is
			   covered), but still below the highest possible upper threshold,
			   the sensor state is set incorrectly until the first interrupt
			   (which will trigger at the smallest movement). To observe this
			   flaw, you would have to keep the sensor covered below the highest
			   possible upper threshold without movement whenever it is enabled.
			   Fortunately, this is highly unlikely in typical use cases.
			   The flaw cannot be eliminated without assuming the offset value,
			   which is not desirable (e.g. it is unknow if the user applied a
			   screen protector that covers the sensor, which raises the offset). */
			data->ps_covered = false;

			data->platform_data->prox_hysteresis_threshold = ltr559_ps_thres_below(data->ps_offset);
			data->platform_data->prox_threshold = ltr559_ps_thres_up(data->ps_offset);

		} else {

			if (ret <= (int) ltr559_ps_thres_up(data->ps_offset)) {

				data->ps_covered = false;

				data->platform_data->prox_hysteresis_threshold = ltr559_ps_thres_below(data->ps_offset);
				data->platform_data->prox_threshold = ltr559_ps_thres_up(data->ps_offset);

				if (ret < 0)
					pr_warning("%s: Received no or invalid sensor data while waking up"
						" proximity sensor (ltr559_ps_read returned %i). It will now"
						" be assumed that the sensor is not covered.\n", __func__, ret);

			} else {

				data->ps_covered = true;

				data->platform_data->prox_hysteresis_threshold = ltr559_ps_thres_low(data->ps_offset);
				data->platform_data->prox_threshold = 0x07ff;
			}
		}

		data->ps_offset_data_count = 1;
		data->ps_offset_data[0] = data->ps_offset;

		ltr559_ps_set_thresholds(client,
			data->platform_data->prox_hysteresis_threshold,
			data->platform_data->prox_threshold);

		input_report_abs(data->input_dev_ps, ABS_DISTANCE, data->ps_covered ? 0 : 1);

		pr_info("%s: Proximity sensor enabled (state: %s, offset=%i, thres_low=%i,"
			" thres_up=%i).\n", __func__, data->ps_covered ? "covered" : "not covered",
			data->ps_offset, data->platform_data->prox_hysteresis_threshold,
			data->platform_data->prox_threshold);
	} else {
		ret = i2c_smbus_write_byte_data(client, LTR559_PS_CONTR, MODE_PS_StdBy);
		if(ret < 0) {
			pr_err("%s: Failed to disable proximity sensor. Error while writing to"
				" register REG_PS_CONTR (i2c_smbus_write_byte_data returned %i).\n",
				__func__, ret);
		}
		ret = i2c_smbus_read_byte_data(client, LTR559_PS_CONTR);
		if(ret != reg_tbl[REG_PS_CONTR].defval) {
			pr_err("%s: Failed to disable proximity sensor. Error while writing to"
				" register REG_PS_CONTR (i2c_smbus_read_byte_data returned %i, but"
				" must be %i).\n", __func__, ret, reg_tbl[REG_PS_CONTR].defval);
		}
	}
	return 0;
}

/*
 * Ambient Light Sensor Congfig
 */
static int ltr559_als_enable(struct i2c_client *client, int on)
{
	struct ltr559_data *data = i2c_get_clientdata(client);
	int ret;

	if (on) {
		ret = i2c_smbus_write_byte_data(client, LTR559_ALS_CONTR, reg_tbl[REG_ALS_CONTR].curval);
		if(ret < 0)
		{
			dev_err(&client->dev, "%s write error\n",__func__);
			return ret;
		}
		msleep(WAKEUP_DELAY);
		ret |= i2c_smbus_read_byte_data(client, LTR559_ALS_DATA_CH0_1);
		cancel_delayed_work_sync(&data->als_work);
		schedule_delayed_work(&data->als_work,msecs_to_jiffies(data->platform_data->als_poll_interval));
	} else {
		cancel_delayed_work_sync(&data->als_work);
		ret = i2c_smbus_write_byte_data(client, LTR559_ALS_CONTR, MODE_ALS_StdBy);
		if(ret < 0)
		{
			dev_err(&client->dev, "%s write error\n",__func__);
			return ret;
		}
	}

	pr_err("%s: enable=(%d) ret=%d\n", __func__, on, ret);
	return ret;
}

static int ltr559_als_read(struct i2c_client *client)
{
		int alsval_ch0;
		int alsval_ch1;
		int luxdata;
		int ch1_co, ch0_co, ratio;
		int ch0_c[4] = {17743,42785,5926,0};
		int ch1_c[4] = {-11059,19548,-1185,0};

		alsval_ch1 = i2c_smbus_read_word_data(client, LTR559_ALS_DATA_CH1_0);//0x88
		if (alsval_ch1 < 0)
				return -EIO;

		alsval_ch0 = i2c_smbus_read_word_data(client, LTR559_ALS_DATA_CH0_0);//0x8a
		if (alsval_ch0 < 0 )
				return -EIO;

		if ((alsval_ch0 + alsval_ch1) == 0){
				ratio = 1000;
		}else{
			ratio = alsval_ch1 * 1000 / (alsval_ch1 + alsval_ch0);
		}

		if (ratio < 450) {
				ch0_co = ch0_c[0];
				ch1_co = ch1_c[0];
		} else if ((ratio >= 450) && (ratio < 640)) {
				ch0_co = ch0_c[1];
				ch1_co = ch1_c[1];
		} else if ((ratio >= 640) && (ratio < 850)) {
				ch0_co = ch0_c[2];
				ch1_co = ch1_c[2];
		} else if (ratio >= 850) {
				ch0_co = ch0_c[3];
				ch1_co = ch1_c[3];
		}
		luxdata = (alsval_ch0 * ch0_co - alsval_ch1 * ch1_co) / 10000;
		return luxdata;
}

static void ltr559_ps_work_func(struct work_struct *work)
{
	struct ltr559_data *data = container_of(work, struct ltr559_data, ps_work.work);
	struct i2c_client *client = data->client;
	bool ps_state_changed = false;
	int ret;
	u8 i;

	mutex_lock(&data->op_lock);

	/* LTR559_ALS_PS_STATUS register (1 byte):
	   Bit 0: New unread proximity data is available.
	   Bit 1: Proximity sensor interrupt conditions are met.
	   Bits 2-7 hold ALS status information. */
	ret = i2c_smbus_read_byte_data(client, LTR559_ALS_PS_STATUS);
	if ((ret & 0x03) != 0x03)
		goto workout;

	ret = ltr559_ps_read(client);

	if (ret < 0) {

		/* If there is no data, don't touch offset and force uncovered state. */

		if (data->ps_covered) {

			data->ps_covered = false;
			ps_state_changed = true;

			data->platform_data->prox_hysteresis_threshold = ltr559_ps_thres_below(data->ps_offset);
			data->platform_data->prox_threshold = ltr559_ps_thres_up(data->ps_offset);
		}
	} else {

		if (ret > data->platform_data->prox_threshold) {

			data->ps_covered = true;
			ps_state_changed = true;

			data->platform_data->prox_hysteresis_threshold = ltr559_ps_thres_low(data->ps_offset);
			data->platform_data->prox_threshold = 0x07ff;

		} else if (ret < data->platform_data->prox_hysteresis_threshold) {

			data->ps_offset = ret;

			if (data->ps_covered) {

				data->ps_covered = false;
				ps_state_changed = true;

				/* Recalculate offset as the mean value of the last few sensor values
				   that triggered the interrupt via the lower threshold. */

				if (data->ps_offset_data_count >= PS_OFFSET_DATA_LEN)
					data->ps_offset_data_count = PS_OFFSET_DATA_LEN - 1;

				for (i = data->ps_offset_data_count; i > 0; i--) {
					data->ps_offset_data[i] = data->ps_offset_data[i-1];
					data->ps_offset += data->ps_offset_data[i];
				}

				data->ps_offset_data_count++;
				data->ps_offset_data[0] = ret;

				data->ps_offset /= data->ps_offset_data_count;

			} else {

				data->ps_offset_data_count = 1;
				data->ps_offset_data[0] = data->ps_offset;

				pr_info("%s: Proximity sensor offset adjusted (offset=%i, thres_low=%i, thres_up=%i).\n",
					__func__, data->ps_offset, data->platform_data->prox_hysteresis_threshold,
					data->platform_data->prox_threshold);
			}

			data->platform_data->prox_hysteresis_threshold = ltr559_ps_thres_below(data->ps_offset);
			data->platform_data->prox_threshold = ltr559_ps_thres_up(data->ps_offset);
		}
	}

	ltr559_ps_set_thresholds(client,
		data->platform_data->prox_hysteresis_threshold,
		data->platform_data->prox_threshold);

	if (ps_state_changed) {

		input_report_abs(data->input_dev_ps, ABS_DISTANCE, data->ps_covered ? 0 : 1);
		input_sync(data->input_dev_ps);

		pr_info("%s: Proximity sensor is now %s (offset=%i, thres_low=%i, thres_up=%i).\n",
			__func__, data->ps_covered ? "covered" : "uncovered", data->ps_offset,
			data->platform_data->prox_hysteresis_threshold, data->platform_data->prox_threshold);
	}

workout:

	enable_irq(data->irq);
	mutex_unlock(&data->op_lock);
}

static void ltr559_als_work_func(struct work_struct *work)
{
	struct ltr559_data *data = container_of(work, struct ltr559_data, als_work.work);
	struct i2c_client *client=data->client;
	int als_ps_status;
	int als_data;

	mutex_lock(&data->op_lock);

	if(!data->als_open_state)
		goto workout;

	als_ps_status = i2c_smbus_read_byte_data(client, LTR559_ALS_PS_STATUS);
	if (als_ps_status < 0)
		goto workout;


	if ((data->als_open_state == 1) && (als_ps_status & 0x04)) {
		als_data = ltr559_als_read(client);
		if (als_data > 50000)
			als_data = 50000;

		if ((als_data >= 0) && (als_data != data->last_lux)) {
			data->last_lux = als_data;
			input_report_abs(data->input_dev_als, ABS_MISC, als_data);
			input_sync(data->input_dev_als);
		}
	}

	schedule_delayed_work(&data->als_work,msecs_to_jiffies(data->platform_data->als_poll_interval));
workout:
	mutex_unlock(&data->op_lock);
}

static irqreturn_t ltr559_irq_handler(int irq, void *arg)
{
	struct ltr559_data *data = (struct ltr559_data *)arg;

	if (NULL == data)
		return IRQ_HANDLED;
	disable_irq_nosync(data->irq);
	schedule_delayed_work(&data->ps_work, 0);
	return IRQ_HANDLED;
}

static int ltr559_gpio_irq(struct ltr559_data *data)
{
        struct device_node *np = data->client->dev.of_node;
	 int err = 0;

        data->platform_data->int_gpio = of_get_named_gpio_flags(np, "ltr,irq-gpio", 0, &data->platform_data->irq_gpio_flags);
        if (data->platform_data->int_gpio < 0)
                return -EIO;

        if (gpio_is_valid(data->platform_data->int_gpio)) {
                err = gpio_request(data->platform_data->int_gpio, "ltr559_irq_gpio");
                if (err) {
                        printk( "%s irq gpio request failed\n",__func__);
			return -EINTR;
                }

                err = gpio_direction_input(data->platform_data->int_gpio);
                if (err) {
                        printk("%s set_direction for irq gpio failed\n",__func__);
			return -EIO;
                }
        }

	data->irq = data->client->irq = gpio_to_irq(data->platform_data->int_gpio);

        if (request_irq(data->irq, ltr559_irq_handler, IRQ_TYPE_LEVEL_LOW/*IRQF_DISABLED|IRQ_TYPE_EDGE_FALLING*/,
                LTR559_DRV_NAME, data)) {
                printk("%s Could not allocate ltr559_INT !\n", __func__);
		return -EINTR;
        }

        irq_set_irq_wake(data->irq, 1);

        printk(KERN_INFO "%s: INT No. %d", __func__, data->irq);
        return 0;
}


static void ltr559_gpio_irq_free(struct ltr559_data *data)
{
	free_irq(data->irq, data);
	gpio_free(data->platform_data->int_gpio);
}

static ssize_t ltr559_show_enable_ps(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ltr559_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", data->ps_open_state);
}

static ssize_t ltr559_store_enable_ps(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	/* If proximity work,then ALS must be enable */
	unsigned long val;
	char *after;
	struct ltr559_data *data = dev_get_drvdata(dev);

	val = simple_strtoul(buf, &after, 10);

	printk(KERN_INFO "enable 559 PS sensor -> %ld\n", val);

	mutex_lock(&data->lockw);
	ltr559_ps_set_enable(&data->ps_cdev, (unsigned int)val);
	mutex_unlock(&data->lockw);

	return size;
}

static ssize_t ltr559_show_enable_als(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ltr559_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", data->als_open_state);
}

static ssize_t ltr559_store_enable_als(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	/* If proximity work,then ALS must be enable */
	unsigned long val;
	char *after;
	struct ltr559_data *data = dev_get_drvdata(dev);

	val = simple_strtoul(buf, &after, 10);

	printk(KERN_INFO "enable 559 ALS sensor -> %ld\n", val);

	mutex_lock(&data->lockw);
	ltr559_als_set_enable(&data->als_cdev, (unsigned int)val);
	mutex_unlock(&data->lockw);
	return size;
}

static ssize_t ltr559_driver_info_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
		return sprintf(buf, "Chip: %s %s\nVersion: %s\n",
						VENDOR_NAME, LTR559_SENSOR_NAME, DRIVER_VERSION);
}

static ssize_t ltr559_show_debug_regs(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 val,high,low;
	int i;
	char *after;
	struct ltr559_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

	after = buf;

	after += sprintf(after, "%-17s%5s%14s%16s\n", "Register Name", "address", "default", "current");
	for (i = 0; i < sizeof(reg_tbl)/sizeof(reg_tbl[0]); i++) {
			if (reg_tbl[i].name == NULL || reg_tbl[i].addr == 0) {
					break;
			}
			if (i < 10) {
					val = i2c_smbus_read_byte_data(client, reg_tbl[i].addr);
					after += sprintf(after, "%-20s0x%02x\t  0x%02x\t\t  0x%02x\n", reg_tbl[i].name, reg_tbl[i].addr, reg_tbl[i].defval, val);
			} else {
					low = i2c_smbus_read_byte_data(client, reg_tbl[i].addr);
					high = i2c_smbus_read_byte_data(client, reg_tbl[i].addr+1);
					after += sprintf(after, "%-20s0x%02x\t0x%04x\t\t0x%04x\n", reg_tbl[i].name, reg_tbl[i].addr, reg_tbl[i].defval,(high << 8) + low);
			}
	}
	after += sprintf(after, "\nYou can echo '0xaa=0xbb' to set the value 0xbb to the register of address 0xaa.\n ");

	return (after - buf);
}

static ssize_t ltr559_store_debug_regs(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	/* If proximity work,then ALS must be enable */
	char *after, direct;
	u8 addr, val;
	struct ltr559_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

	addr = simple_strtoul(buf, &after, 16);
	direct = *after;
	val = simple_strtoul((after+1), &after, 16);

	if (!((addr >= 0x80 && addr <= 0x93)
				|| (addr >= 0x97 && addr <= 0x9e)))
		return -EINVAL;

	mutex_lock(&data->lockw);
	if (direct == '=')
		i2c_smbus_write_byte_data(client, addr, val);
	else
		printk("%s: register(0x%02x) is: 0x%02x\n", __func__, addr, i2c_smbus_read_byte_data(client, addr));
	mutex_unlock(&data->lockw);

	return (after - buf);
}

static ssize_t ltr559_show_adc_data(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ltr559_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int ps_data;

	ps_data = i2c_smbus_read_word_data(client, LTR559_PS_DATA_0);
	if (ps_data < 0)
		return -EINVAL;
	else
		return sprintf(buf, "%d\n", ps_data);
}

static ssize_t ltr559_show_lux_data(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int lux;
	struct ltr559_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

	lux = ltr559_als_read(client);

	return sprintf(buf, "%d\n", lux);
}

static DEVICE_ATTR(debug_regs, SYS_AUTHORITY, ltr559_show_debug_regs,
				ltr559_store_debug_regs);
static DEVICE_ATTR(enable_als_sensor, SYS_AUTHORITY, ltr559_show_enable_als,
				ltr559_store_enable_als);
static DEVICE_ATTR(enable, SYS_AUTHORITY, ltr559_show_enable_ps,
				ltr559_store_enable_ps);
static DEVICE_ATTR(info, S_IRUGO, ltr559_driver_info_show, NULL);
static DEVICE_ATTR(raw_adc, S_IRUGO, ltr559_show_adc_data, NULL);
static DEVICE_ATTR(lux_adc, S_IRUGO, ltr559_show_lux_data, NULL);

static struct attribute *ltr559_attributes[] = {
		&dev_attr_enable.attr,
		&dev_attr_info.attr,
		&dev_attr_enable_als_sensor.attr,
		&dev_attr_debug_regs.attr,
		&dev_attr_raw_adc.attr,
		&dev_attr_lux_adc.attr,
		NULL,
};

static const struct attribute_group ltr559_attr_group = {
		.attrs = ltr559_attributes,
};

static int ltr559_als_set_poll_delay(struct ltr559_data *data,unsigned int delay)
{
	mutex_lock(&data->op_lock);

	delay = clamp((int)delay,1,1000);

	if (data->platform_data->als_poll_interval != delay) {
			data->platform_data->als_poll_interval = delay;
	}

	if(!data->als_open_state)
		return -ESRCH;

	pr_info("%s poll_interval=%d\n",__func__,data->platform_data->als_poll_interval);
	cancel_delayed_work_sync(&data->als_work);
	schedule_delayed_work(&data->als_work,msecs_to_jiffies(data->platform_data->als_poll_interval));
	mutex_unlock(&data->op_lock);
	return 0;
}

static int ltr559_als_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct ltr559_data *data = container_of(sensors_cdev, struct ltr559_data, als_cdev);
	int ret = 0;

	if ((enable != 0) && (enable != 1)) {
		pr_err("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}

	ret = ltr559_als_enable(data->client, enable);
	if(ret < 0){
		pr_err("%s: enable(%d) failed!\n", __func__, enable);
		return -EFAULT;
	}

	data->als_open_state = enable;
	pr_err("%s: enable=(%d), data->als_open_state=%d\n", __func__, enable, data->als_open_state);
	return ret;
}
static int ltr559_als_poll_delay(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	struct ltr559_data *data = container_of(sensors_cdev, struct ltr559_data, als_cdev);
	ltr559_als_set_poll_delay(data, delay_msec);
	return 0;
}

static int ltr559_ps_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct ltr559_data *data = container_of(sensors_cdev, struct ltr559_data, ps_cdev);
	int ret;

	if ((enable != 0) && (enable != 1)) {
		pr_err("%s: Invalid parameter (got enable=%i, but must be 0 or 1).\n", __func__, enable);
		return -EINVAL;
	}

	ret = ltr559_ps_enable(data->client, enable);
	if(ret < 0){
		pr_err("%s: Failed to %s the proximity sensor (ltr559_ps_enable returned %i).\n",
			__func__, enable ? "enable" : "disable", ret);
		return ret;
	}

	data->ps_open_state = enable;
	return 0;
}

static int ltr559_suspend(struct device *dev)
{
	struct ltr559_data *data = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&data->lockw);
	ret |= ltr559_als_enable(data->client, 0);
	mutex_unlock(&data->lockw);
	return ret;
}

static int ltr559_resume(struct device *dev)
{
	struct ltr559_data *data = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&data->lockw);
	if (data->als_open_state == 1)
		ret = ltr559_als_enable(data->client, 1);
	mutex_unlock(&data->lockw);
	return ret;
}

static int ltr559_check_chip_id(struct i2c_client *client)
{
	int id;

	id = i2c_smbus_read_byte_data(client, LTR559_MANUFACTURER_ID);
	printk("%s read the  LTR559_MANUFAC_ID is 0x%x\n", __func__, id);
	if (id != LTR559_MANUFAC_ID) {
		return -EINVAL;
	}
	return 0;
}

int ltr559_device_init(struct i2c_client *client)
{
	int retval = 0;
	int i;

	retval = i2c_smbus_write_byte_data(client, LTR559_ALS_CONTR, 0x02);    /* reset chip */
	if(retval < 0)
	{
		printk("%s   i2c_smbus_write_byte_data(LTR559_ALS_CONTR, 0x02);  ERROR !!!.\n",__func__);
		return -EIO;
	}

	msleep(WAKEUP_DELAY);
	for (i = 2; i < sizeof(reg_tbl)/sizeof(reg_tbl[0]); i++) {
		if (reg_tbl[i].name == NULL || reg_tbl[i].addr == 0) {
				break;
		}
		if (reg_tbl[i].defval != reg_tbl[i].curval) {
			if (i < 10) {
				retval = i2c_smbus_write_byte_data(client, reg_tbl[i].addr, reg_tbl[i].curval);
				if(retval < 0)
				{
					dev_err(&client->dev, "%s write error\n",__func__);
					return retval;
				}
			} else {
				retval = i2c_smbus_write_byte_data(client, reg_tbl[i].addr, reg_tbl[i].curval & 0xff);
				if(retval < 0)
				{
					dev_err(&client->dev, "%s write error\n",__func__);
					return retval;
				}
				retval = i2c_smbus_write_byte_data(client, reg_tbl[i].addr + 1, reg_tbl[i].curval >> 8);
				if(retval < 0)
				{
					dev_err(&client->dev, "%s write error\n",__func__);
					return retval;
				}
			}
		}
	}

	return retval;
}

static int sensor_regulator_configure(struct ltr559_data *data, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0, LTR559_VDD_MAX_UV);
		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0, LTR559_VIO_MAX_UV);
		regulator_put(data->vio);
	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			dev_err(&data->client->dev,"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd, LTR559_VDD_MIN_UV, LTR559_VDD_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,"Regulator set failed vdd rc=%d\n",rc);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio)) {
			rc = PTR_ERR(data->vio);
			dev_err(&data->client->dev,"Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			rc = regulator_set_voltage(data->vio, LTR559_VIO_MIN_UV, LTR559_VIO_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev, "Regulator set failed vio rc=%d\n", rc);
				goto reg_vio_put;
			}
		}
	}

	return 0;
reg_vio_put:
	regulator_put(data->vio);

reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, LTR559_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}

static int sensor_regulator_power_on(struct ltr559_data *data, bool on)
{
	int rc = 0;

	if (!on) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev, "Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_disable(data->vio);
		if (rc) {
			dev_err(&data->client->dev, "Regulator vio disable failed rc=%d\n", rc);
			rc = regulator_enable(data->vdd);
			dev_err(&data->client->dev, "Regulator vio re-enabled rc=%d\n", rc);
			/*
			 * Successfully re-enable regulator.
			 * Enter poweron delay and returns error.
			 */
			if (!rc) {
				rc = -EBUSY;
				goto enable_delay;
			}
		}
		return rc;
	} else {
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev, "Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_enable(data->vio);
		if (rc) {
			dev_err(&data->client->dev, "Regulator vio enable failed rc=%d\n", rc);
			regulator_disable(data->vdd);
			return rc;
		}
	}

enable_delay:
	msleep(130);
	dev_dbg(&data->client->dev, "Sensor regulator power on =%d\n", on);
	return rc;
}

static int sensor_platform_hw_power_onoff(struct ltr559_data *data, bool on)
{
	int err = 0;

	if (data->power_state != on) {
		if(on){
			if(!IS_ERR_OR_NULL(data->pinctrl)){
				err = pinctrl_select_state(data->pinctrl, data->pin_default);
				if (err){
					dev_err(&data->client->dev, "Can't select pinctrl state on=%d\n", on);
					goto power_out;
				}
			}

			err = sensor_regulator_configure(data, true);
			if(err){
				dev_err(&data->client->dev, "unable to configure regulator on=%d\n", on);
				goto power_out;
			}

			err = sensor_regulator_power_on(data, true);
			if (err){
				dev_err(&data->client->dev, "Can't configure regulator on=%d\n", on);
				goto power_out;
			}

			data->power_state = true;
		}else{
			if(!IS_ERR_OR_NULL(data->pinctrl)){
				err = pinctrl_select_state(data->pinctrl, data->pin_sleep);
				if (err){
					dev_err(&data->client->dev, "Can't select pinctrl state on=%d\n", on);
					goto power_out;
				}
			}

			err = sensor_regulator_power_on(data, false);
			if (err){
				dev_err(&data->client->dev, "Can't configure regulator on=%d\n", on);
				goto power_out;
			}

			err = sensor_regulator_configure(data, false);
			if(err){
				dev_err(&data->client->dev, "unable to configure regulator on=%d\n", on);
				goto power_out;
			}

			data->power_state = false;
		}
	}
power_out:
	return err;
}

static int ltr559_pinctrl_init(struct ltr559_data *data)
{
	struct i2c_client *client = data->client;

	data->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		dev_err(&client->dev, "Failed to get pinctrl\n");
		return PTR_ERR(data->pinctrl);
	}

	data->pin_default =
		pinctrl_lookup_state(data->pinctrl, "default");
	if (IS_ERR_OR_NULL(data->pin_default)) {
		dev_err(&client->dev, "Failed to look up default state\n");
		return PTR_ERR(data->pin_default);
	}

	data->pin_sleep =
		pinctrl_lookup_state(data->pinctrl, "sleep");
	if (IS_ERR_OR_NULL(data->pin_sleep)) {
		dev_err(&client->dev, "Failed to look up sleep state\n");
		return PTR_ERR(data->pin_sleep);
	}

	return 0;
}

static int ltr559_parse_dt(struct device *dev, struct ltr559_data *data)
{
	struct ltr559_platform_data *pdata = data->platform_data;
	struct device_node *np = dev->of_node;
	unsigned int tmp;
	int rc = 0;

	/* ps tuning data*/
	rc = of_property_read_u32(np, "ltr,ps-threshold", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read ps threshold\n");
		return rc;
	}
	pdata->prox_threshold = tmp;

	rc = of_property_read_u32(np, "ltr,ps-hysteresis-threshold", &tmp);
	 if (rc) {
		dev_err(dev, "Unable to read ps hysteresis threshold\n");
		return rc;
	}
	pdata->prox_hysteresis_threshold = tmp;

	rc = of_property_read_u32(np, "ltr,als-polling-time", &tmp);
	 if (rc) {
		dev_err(dev, "Unable to read ps hysteresis threshold\n");
		return rc;
	}
	pdata->als_poll_interval = tmp;

	return 0;
}

int ltr559_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct ltr559_data *data;
	struct ltr559_platform_data *pdata;
	int ret = 0;

	/* check i2c*/
	if (!i2c_check_functionality(adapter,I2C_FUNC_SMBUS_WRITE_BYTE | I2C_FUNC_SMBUS_READ_BYTE_DATA)) {
			dev_err(&client->dev, "LTR-559ALS functionality check failed.\n");
			return -EIO;
	}

	/* platform data memory allocation*/
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct ltr559_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		client->dev.platform_data = pdata;
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			dev_err(&client->dev, "No platform data\n");
			return -ENODEV;
		}
	}

	/* data memory allocation */
	data = kzalloc(sizeof(struct ltr559_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "kzalloc failed\n");
		ret = -ENOMEM;
		goto exit_kfree_pdata;
	}
	data->client = client;
	data->platform_data = pdata;
	ret = ltr559_parse_dt(&client->dev, data);
	if (ret) {
		dev_err(&client->dev, "can't parse platform data\n");
		ret = -EFAULT;
		goto exit_kfree_data;
	}

	/* pinctrl initialization */
	ret = ltr559_pinctrl_init(data);
	if (ret) {
		ret = -EFAULT;
		dev_err(&client->dev, "Can't initialize pinctrl\n");
		goto exit_kfree_data;
	}

	/* power initialization */
	ret = sensor_platform_hw_power_onoff(data, true);
	if (ret) {
		ret = -ENOEXEC;
		dev_err(&client->dev,"power on fail\n");
		goto exit_kfree_data;
	}

	/* set client data as ltr559_data*/
	i2c_set_clientdata(client, data);

	ret = ltr559_check_chip_id(client);
	if(ret){
		ret = -ENXIO;
		dev_err(&client->dev,"the manufacture id is not match\n");
		goto exit_power_off;
	}

	ret = ltr559_device_init(client);
	if (ret) {
		ret = -ENXIO;
		dev_err(&client->dev,"device init failed\n");
		goto exit_power_off;
	}

	/* request gpio and irq */
	ret = ltr559_gpio_irq(data);
	if (ret) {
		ret = -ENXIO;
		dev_err(&client->dev,"gpio_irq failed\n");
		goto exit_chip_reset;
	}

	/* Register Input Device */
	data->input_dev_als = input_allocate_device();
	if (!data->input_dev_als) {
		ret = -ENOMEM;
		dev_err(&client->dev,"Failed to allocate input device als\n");
		goto exit_free_irq;
	}

	data->input_dev_ps = input_allocate_device();
	if (!data->input_dev_ps) {
		ret = -ENOMEM;
		dev_err(&client->dev,"Failed to allocate input device ps\n");
		goto exit_free_dev_als;
	}

	set_bit(EV_ABS, data->input_dev_als->evbit);
	set_bit(EV_ABS, data->input_dev_ps->evbit);

	input_set_abs_params(data->input_dev_als, ABS_MISC, 0, 65535, 0, 0);
	input_set_abs_params(data->input_dev_ps, ABS_DISTANCE, 0, 1, 0, 0);

	data->input_dev_als->name = "ltr559-ls";
	data->input_dev_ps->name = "ltr559-ps";
	data->input_dev_als->id.bustype = BUS_I2C;
	data->input_dev_als->dev.parent =&data->client->dev;
	data->input_dev_ps->id.bustype = BUS_I2C;
	data->input_dev_ps->dev.parent =&data->client->dev;

	input_set_drvdata(data->input_dev_als, data);
	input_set_drvdata(data->input_dev_ps, data);

	ret = input_register_device(data->input_dev_als);
	if (ret) {
		ret = -ENOMEM;
		dev_err(&client->dev,"Unable to register input device als: %s\n",data->input_dev_als->name);
		goto exit_free_dev_ps;
	}

	ret = input_register_device(data->input_dev_ps);
	if (ret) {
		ret = -ENOMEM;
		dev_err(&client->dev,"Unable to register input device ps: %s\n",data->input_dev_ps->name);
		goto exit_unregister_dev_als;
	}
	printk("%s: Register input device success.\n", __func__);

	/* init delayed works */
	INIT_DELAYED_WORK(&data->ps_work, ltr559_ps_work_func);
	INIT_DELAYED_WORK(&data->als_work, ltr559_als_work_func);

	/* init mutex */
	mutex_init(&data->lockw);
	mutex_init(&data->op_lock);

	/* create sysfs group */
	ret = sysfs_create_group(&client->dev.kobj, &ltr559_attr_group);

    ret = sysfs_create_group(&data->input_dev_als->dev.kobj, &ltr559_attr_group);
	ret = sysfs_create_group(&data->input_dev_ps->dev.kobj, &ltr559_attr_group);

	if (ret){
		ret = -EROFS;
		dev_err(&client->dev,"Unable to create sysfs group.\n");
		goto exit_unregister_dev_ps;
	}

	/* Register sensors class */
	data->als_cdev = sensors_light_cdev;
	data->als_cdev.sensors_enable = ltr559_als_set_enable;
	data->als_cdev.sensors_poll_delay = ltr559_als_poll_delay;
	data->ps_cdev = sensors_proximity_cdev;
	data->ps_cdev.sensors_enable = ltr559_ps_set_enable;
	data->ps_cdev.sensors_poll_delay = NULL;

	ret = sensors_classdev_register(&client->dev, &data->als_cdev);
	if(ret) {
		ret = -EROFS;
		dev_err(&client->dev,"Unable to register to als sensor class.\n");
		goto exit_remove_sysfs_group;
	}

	ret = sensors_classdev_register(&client->dev, &data->ps_cdev);
	if(ret) {
		ret = -EROFS;
		dev_err(&client->dev,"Unable to register to ps sensor class.\n");
		goto exit_unregister_als_class;
	}

	/* Initialize data offset and thresholds during boot. */
	data->ps_offset_data_count = 0;
	ltr559_ps_enable(client, 1);
	ltr559_ps_enable(client, 0);

	dev_dbg(&client->dev, "Probe success.\n");
	return 0;

exit_unregister_als_class:
	sensors_classdev_unregister(&data->als_cdev);
exit_remove_sysfs_group:
	sysfs_remove_group(&client->dev.kobj, &ltr559_attr_group);
	sysfs_remove_group(&data->input_dev_als->dev.kobj, &ltr559_attr_group);
	sysfs_remove_group(&data->input_dev_ps->dev.kobj, &ltr559_attr_group);
exit_unregister_dev_ps:
	input_unregister_device(data->input_dev_ps);
exit_unregister_dev_als:
	input_unregister_device(data->input_dev_als);
exit_free_dev_ps:
	if (data->input_dev_ps)
			input_free_device(data->input_dev_ps);
exit_free_dev_als:
	if (data->input_dev_als)
			input_free_device(data->input_dev_als);
exit_free_irq:
	ltr559_gpio_irq_free(data);
exit_chip_reset:
	ltr559_chip_reset(client);
exit_power_off:
	sensor_platform_hw_power_onoff(data, false);

exit_kfree_pdata:
	if (pdata && (client->dev.of_node))
		devm_kfree(&client->dev, pdata);
	data->platform_data= NULL;
    pdata = NULL;

exit_kfree_data:
	kfree(data);

	return ret;
}

static int ltr559_remove(struct i2c_client *client)
{
	struct ltr559_data *data = i2c_get_clientdata(client);
	struct ltr559_platform_data *pdata=data->platform_data;

	if (data == NULL || pdata == NULL)
		return 0;

	ltr559_ps_enable(client, 0);
	ltr559_als_enable(client, 0);
	input_unregister_device(data->input_dev_als);
	input_unregister_device(data->input_dev_ps);

	input_free_device(data->input_dev_als);
	input_free_device(data->input_dev_ps);
	ltr559_gpio_irq_free(data);

	sysfs_remove_group(&client->dev.kobj, &ltr559_attr_group);
	sysfs_remove_group(&data->input_dev_als->dev.kobj, &ltr559_attr_group);
	sysfs_remove_group(&data->input_dev_ps->dev.kobj, &ltr559_attr_group);

	cancel_delayed_work_sync(&data->ps_work);
	cancel_delayed_work_sync(&data->als_work);

	if (pdata && (client->dev.of_node))
		devm_kfree(&client->dev, pdata);
	pdata = NULL;

	kfree(data);
	data = NULL;

	return 0;
}

static struct i2c_device_id ltr559_id[] = {
		{"ltr559", 0},
		{}
};

static struct of_device_id ltr_match_table[] = {
		{ .compatible = "ltr,ltr559",},
		{ },
};

MODULE_DEVICE_TABLE(i2c, ltr559_id);
static SIMPLE_DEV_PM_OPS(ltr559_pm_ops, ltr559_suspend, ltr559_resume);
static struct i2c_driver ltr559_driver = {
		.driver = {
				.name = LTR559_DRV_NAME,
				.owner = THIS_MODULE,
				.pm = &ltr559_pm_ops,
				.of_match_table = ltr_match_table,
		},
		.probe = ltr559_probe,
		.remove = ltr559_remove,
		.id_table = ltr559_id,
};

static int ltr559_driver_init(void)
{
		pr_info("Driver ltr559 init.\n");
		return i2c_add_driver(&ltr559_driver);
};

static void ltr559_driver_exit(void)
{
		pr_info("Unload ltr559 module...\n");
		i2c_del_driver(&ltr559_driver);
}

module_init(ltr559_driver_init);
module_exit(ltr559_driver_exit);
MODULE_AUTHOR("Lite-On Technology Corp.");
MODULE_DESCRIPTION("Lite-On LTR-559 Proximity and Light Sensor Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
