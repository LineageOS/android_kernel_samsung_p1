/* arch/arm/mach-s5pv210/p1-vibrator.c
 *
 * Copyright (C) 2010 Samsung Electronics Co. Ltd. All Rights Reserved.
 * Author: Rom Lemarchand <rlemarchand@sta.samsung.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/hrtimer.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/wakelock.h>
#include <linux/mutex.h>
#include <linux/clk.h>
#include <linux/workqueue.h>

#include <asm/mach-types.h>

#include <../../../drivers/staging/android/timed_output.h>

#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
static struct regulator *regulator_motor;

#define GPD0_TOUT_1		(0x2 << 4)
#define VIB_EN			S5PV210_GPJ1(3)
#define VIB_PWM			S5PV210_GPD0(1)

#ifdef CONFIG_SAMSUNG_P1C

#define PWM_PERIOD              44640
#define PWM_DUTY_MAX            42408
#define PWM_DUTY_MIN			21204

#else

#define PWM_PERIOD              44540
#define PWM_DUTY_MAX            44500
#define PWM_DUTY_MIN			22250

#endif

#define MAX_TIMEOUT		10000 /* 10s */

static unsigned int multiplier = (PWM_DUTY_MAX - PWM_DUTY_MIN) / 100;
static unsigned int pwm_duty = 100;
static unsigned int pwm_duty_value = PWM_DUTY_MAX;

static struct vibrator {
	struct wake_lock wklock;
	struct pwm_device *pwm_dev;
	struct hrtimer timer;
	struct mutex lock;
	struct work_struct work;
} vibdata;

static void p1_vibrator_off(void)
{
	s3c_gpio_cfgpin(VIB_PWM, S3C_GPIO_OUTPUT);
	regulator_disable(regulator_motor);
	pwm_disable(vibdata.pwm_dev);
	gpio_direction_output(VIB_EN, 0);
	wake_unlock(&vibdata.wklock);
}

static int p1_vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibdata.timer)) {
		ktime_t r = hrtimer_get_remaining(&vibdata.timer);
		return ktime_to_ms(r);
	}

	return 0;
}

static void p1_vibrator_enable(struct timed_output_dev *dev, int value)
{
	mutex_lock(&vibdata.lock);

	/* cancel previous timer and set GPIO according to value */
	hrtimer_cancel(&vibdata.timer);
	cancel_work_sync(&vibdata.work);
	if (value) {
		wake_lock(&vibdata.wklock);
		s3c_gpio_cfgpin(VIB_PWM, S3C_GPIO_SFN(2));
		regulator_enable(regulator_motor);
		pwm_config(vibdata.pwm_dev, pwm_duty_value, PWM_PERIOD);
		pwm_enable(vibdata.pwm_dev);
		gpio_direction_output(VIB_EN, 1);

		if (value > 0) {
			if (value > MAX_TIMEOUT)
				value = MAX_TIMEOUT;

			hrtimer_start(&vibdata.timer,
				ns_to_ktime((u64)value * NSEC_PER_MSEC),
				HRTIMER_MODE_REL);
		}
	} else
		p1_vibrator_off();

	mutex_unlock(&vibdata.lock);
}

static ssize_t p1_vibrator_set_duty(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	sscanf(buf, "%d\n", &pwm_duty);

	if (pwm_duty >= 0 && pwm_duty <= 100) 
		pwm_duty_value = (pwm_duty * multiplier) + PWM_DUTY_MIN;

	return size;
}
static ssize_t p1_vibrator_show_duty(struct device *dev,
					struct device_attribute *attr,
					const char *buf)
{
	return sprintf(buf, "%d", pwm_duty);
}
static DEVICE_ATTR(pwm_duty, S_IRUGO | S_IWUGO, p1_vibrator_show_duty, p1_vibrator_set_duty);
static struct attribute *pwm_duty_attributes[] = {
	&dev_attr_pwm_duty,
	NULL
};
static struct attribute_group pwm_duty_group = {
	.attrs = pwm_duty_attributes,
};
static struct miscdevice pwm_duty_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "pwm_duty",
};

static struct timed_output_dev to_dev = {
	.name		= "vibrator",
	.get_time	= p1_vibrator_get_time,
	.enable		= p1_vibrator_enable,
};

static enum hrtimer_restart p1_vibrator_timer_func(struct hrtimer *timer)
{
	schedule_work(&vibdata.work);
	return HRTIMER_NORESTART;
}

static void p1_vibrator_work(struct work_struct *work)
{
	p1_vibrator_off();
}

static int __init p1_init_vibrator(void)
{
	int ret = 0;

	hrtimer_init(&vibdata.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibdata.timer.function = p1_vibrator_timer_func;
	INIT_WORK(&vibdata.work, p1_vibrator_work);

    if (IS_ERR_OR_NULL(regulator_motor))
    {
        regulator_motor = regulator_get(NULL, "vcc_motor");
        if (IS_ERR_OR_NULL(regulator_motor))
        {
            pr_err("failed to get motor regulator");
            return -EINVAL;
        }
    }

	ret = gpio_request(VIB_EN, "VIB_EN");
	if (ret < 0)
		return ret;

	vibdata.pwm_dev = pwm_request(1, "VIB_PWM");
	if (IS_ERR(vibdata.pwm_dev)) {
		ret = PTR_ERR(vibdata.pwm_dev);
		goto err_pwm_req;
	}

	wake_lock_init(&vibdata.wklock, WAKE_LOCK_SUSPEND, "vibrator");
	mutex_init(&vibdata.lock);

	ret = timed_output_dev_register(&to_dev);
	if (ret < 0)
		goto err_to_dev_reg;

	if (misc_register(&pwm_duty_device)) {
		printk("%s misc_register(pwm_duty) failed\n", __FUNCTION__);
	} else {
		if (sysfs_create_group(&pwm_duty_device.this_device->kobj, &pwm_duty_group)) {
			printk("failed to create sysfs group for device pwm_duty\n");
		}
	}

	return 0;

err_to_dev_reg:
	mutex_destroy(&vibdata.lock);
	wake_lock_destroy(&vibdata.wklock);
	pwm_free(vibdata.pwm_dev);
err_pwm_req:
	gpio_free(VIB_EN);
	return ret;
}

device_initcall(p1_init_vibrator);
