/*
 * scu_pic.c - thermal monitoring, fan control, and fault LED control.
 *
 * Copyright (C) 2012 Guenter Roeck
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/watchdog.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/leds.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/workqueue.h>
#include <linux/firmware.h>
#include <linux/ihex.h>
#include "scu_pic.h"

/* Intentionally break things during the PIC firmware update process. */
enum {
	_fault_point_min = 0,
	FAULT_POINT_EMBED_CRC,
	FAULT_POINT_DURING_VERIFY,
	FAULT_POINT_BEFORE_VERIFY,
	FAULT_POINT_DURING_WRITE,
	FAULT_POINT_BEFORE_WRITE,
	FAULT_POINT_DURING_BLANK_CHECK,
	FAULT_POINT_BEFORE_BLANK_CHECK,
	FAULT_POINT_DURING_ERASE,
	FAULT_POINT_BEFORE_ERASE,
	_fault_point_max,

	_FAULT_POINT_MIN = (_fault_point_min + 1),
	_FAULT_POINT_MAX = (_fault_point_max - 1),
};

static const unsigned short normal_i2c[] = { 0x20, I2C_CLIENT_END };

#define DEFAULT_POLL_RATE	200	/* in ms */

static int poll_rate = DEFAULT_POLL_RATE;
module_param(poll_rate, int, 0644);
MODULE_PARM_DESC(poll_rate,
		 "Reset state poll rate, in milli-seconds. Set to 0 to disable.");

#define NBYTES_BUILD_DATE (32)

struct scu_pic_data {
	struct i2c_client *client;
	struct mutex i2c_lock;
	struct kref kref;
	struct list_head list;		/* member of scu_pic_data_list */

	u8 version_major;
	u8 version_minor;
	char build_date[NBYTES_BUILD_DATE];

	u8 version_major_bootloader;
	u8 version_minor_bootloader;
	char build_date_bootloader[NBYTES_BUILD_DATE];

	/* worker */
	struct delayed_work work;
	struct workqueue_struct *workqueue;
	bool reset_pin_state;

	/* hwmon */
	struct device *hwmon_dev;
	u8 fan_contr_model;
	u8 fan_contr_rev;

	bool valid;			/* true if following fields are valid */
	unsigned long last_updated;	/* In jiffies */

        u8 thermal_fault_state;
	u8 fan[2];
	u8 pwm_state;	/* same for both fans, only one control available */
	u8 pwm;		/* for manual control, only off/on supported */
	u8 temp[5][2];  /* local, remote, ps, bottom, top respectively */

	/* wdt data */
	struct watchdog_device wdt_dev;
	struct timer_list wdt_timer;
	unsigned long wdt_lastping;

	/* led data */
	struct led_classdev cdev;

	/* firmware update */
	bool in_bootloader;
	int update_progress;
	int update_total;
	unsigned int fault_point;
};

/*
 * Global data pointer list with all scu_pic devices, so that we can find
 * our device data. When using misc_register there is no other method
 * to get to ones device data from the open fop.
 */
static LIST_HEAD(scu_pic_data_list);
/* Note this lock not only protects list access, but also data.kref access */
static DEFINE_MUTEX(scu_pic_data_mutex);

static void scu_pic_release_resources(struct kref *ref)
{
	struct scu_pic_data *data =
		container_of(ref, struct scu_pic_data, kref);

	kfree(data);
}

#define WDT_IN_USE			0
#define WDT_EXPECT_CLOSE		1

#define SCU_PIC_WDT_TIMEOUT	300		/* 5 minutes */

static int nowayout = WATCHDOG_NOWAYOUT;

module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started");

/*
 * Note that these accessors implement an I2C protocol which is somewhat
 * non-standard and sub-optimal.
 *
 * The PIC device being accessed as an I2C slave has a firmware implementation
 * which expects to see the device address twice on the bus, hence each of the
 * transfer buffers below includes 'client->addr' as the very first byte.
 *
 * In addition, the new PIC bootloader firmware does not handle I2C restart
 * conditions (possibly due to the polled nature of the slave I2C
 * implementation in the bootloader?).  Thus, all accesses must be decomposed
 * into separate read/write operations bounded by a start and stop condition.
 *
 * This results in bus traffic which looks like:
 *
 *     addressed read:  S addr addr subaddr P S addr addr data P
 *     addressed write: S addr addr subaddr data P
 *
 * Given that there are already units fielded which make use of this existing
 * implementation (and said units are not easily field-upgradable), support for
 * this peculiarity needs to be maintained going forward.
 */

static int scu_pic_read_byte(struct i2c_client const * const client,
			     u8 const reg)
{
	struct device const * const dev = &client->dev;
	u8 buf[2] = { client->addr, reg };
	int err;

	err = i2c_master_send(client, buf, ARRAY_SIZE(buf));
	if (err < 0)
		return err;

	err = i2c_master_recv(client, buf, 1);
	if (err < 0)
		return err;

	dev_dbg(dev, "scu_pic_read_byte() 0x%02x = 0x%02x\n", reg, buf[0]);
	return buf[0];
}

static int scu_pic_write_byte(struct i2c_client const * const client,
			      u8 const reg, u8 const value)
{
	struct device const * const dev = &client->dev;
	u8 buf[3] = { client->addr, reg, value };
	int err;

	err = i2c_master_send(client, buf, ARRAY_SIZE(buf));
	if (err < 0)
		return err;

	dev_dbg(dev, "scu_pic_write_byte() 0x%02x = 0x%02x\n", reg, value);
	return 0;
}

static int scu_pic_read_build_date(struct i2c_client const * const client,
				   u8 const reg, u8 * const buf,
				   size_t const nbytes)
{
	struct scu_pic_data * const data = i2c_get_clientdata(client);
	bool have_stx = false;
	int err;
	int i;

	if ((reg != I2C_GET_SCU_PIC_BUILD_DATE) &&
			(reg != I2C_GET_SCU_PIC_BOOTLOADER_BUILD_DATE))
		return -EINVAL;

	/*
	 * Data returned is in the form "\x0221-Oct-14 09:16:05\x03".
	 *
	 * The below parsing logic assumes that the provided buffer is at least
	 * large enough that we'll see an STX before nbytes have been read, at
	 * which point we reset the loop count, and read data until we receive
	 * ETX or the buffer is full.
	 *
	 * This is preferable to looping indefinitely in cases where the I2C
	 * implementation in the PIC firmware returns the same garbage byte ad
	 * infinitum.
	 */
	mutex_lock(&data->i2c_lock);
	i = 0;
	while (i < nbytes) {
		err = scu_pic_read_byte(client, reg);
		if (err < 0) {
			goto exit_unlock;
		} else if (err == 0x02) {
			if (have_stx) {
				err = -EINVAL;
				goto exit_unlock;
			}

			have_stx = true;
			i = 0;
			continue;
		} else if (err == 0x03) {
			if (have_stx) {
				buf[i] = '\0';
				err = 0;
				break;
			}
		} else if (have_stx) {
			buf[i] = err;
		}

		i++;
	}

	buf[nbytes - 1] = '\0';

exit_unlock:
	mutex_unlock(&data->i2c_lock);
	return err;
}

/* hardware monitoring */

static int get_thermal_override_state(struct i2c_client *client)
{
	struct scu_pic_data * const data = i2c_get_clientdata(client);
	int err;

	/*
	 * Get the thermal override state from the PIC
	 */
	mutex_lock(&data->i2c_lock);
	err = scu_pic_read_byte(client,
				I2C_GET_SCU_PIC_THERMAL_OVERRIDE_STATE);
	mutex_unlock(&data->i2c_lock);

	return err;
}

static ssize_t show_thermal_override_state(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int state = get_thermal_override_state(to_i2c_client(dev));

	if (state < 0)
		return state;

	return sprintf(buf, "%d\n", state);
}

static DEVICE_ATTR(thermal_override_state, S_IRUGO,
		   show_thermal_override_state, NULL);

static int get_reset_pin_state(struct i2c_client *client)
{
	struct scu_pic_data * const data = i2c_get_clientdata(client);
	int err;

	/*
	 * Get the reset pin state from the PIC
	 */
	mutex_lock(&data->i2c_lock);
	err = scu_pic_read_byte(client, I2C_GET_SCU_PIC_RESET_PIN_STATE);
	mutex_unlock(&data->i2c_lock);

	return err;
}

static ssize_t show_reset_pin_state(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	int state = get_reset_pin_state(to_i2c_client(dev));

	if (state < 0)
		return state;

	return sprintf(buf, "%d\n", state);
}

static DEVICE_ATTR(reset_pin_state, S_IRUGO, show_reset_pin_state, NULL);

static int get_reset_reason(struct i2c_client *client)
{
	struct scu_pic_data * const data = i2c_get_clientdata(client);
	int err;

	/*
	 * Get the reset reason from the PIC
	 */
	mutex_lock(&data->i2c_lock);
	err = scu_pic_read_byte(client, I2C_GET_SCU_PIC_RESET_REASON);
	mutex_unlock(&data->i2c_lock);

	return err;
}


static ssize_t show_reset_reason(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int reason = get_reset_reason(to_i2c_client(dev));

	if (reason < 0)
		return reason;

	return sprintf(buf, "%d\n", reason);
}

static DEVICE_ATTR(reset_reason, S_IRUGO, show_reset_reason, NULL);


static struct scu_pic_data *scu_pic_update_device(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct scu_pic_data *data = i2c_get_clientdata(client);
	int reg;

	mutex_lock(&data->i2c_lock);

	/*
	 * Default ADC sampling rate is 1 second, so wait a little longer
	 * before updating data.
	 */
	if (time_after(jiffies, data->last_updated + HZ + 1) || !data->valid) {
		data->fan[0] = scu_pic_read_byte(client,
						 I2C_GET_SCU_PIC_FAN1_SPEED);
		data->fan[1] = scu_pic_read_byte(client,
						 I2C_GET_SCU_PIC_FAN2_SPEED);
		reg = scu_pic_read_byte(client,
					I2C_GET_SCU_PIC_THERMAL_CONTROL_STATE);
		if (reg == 1) {		/* auto */
			data->pwm_state = 2;
			data->pwm = 255;
		} else {		/* manual */
			data->pwm_state = 1;
			/* Assume pwm is 0 if fans are off. */
			if (data->fan[0] == 0 && data->fan[1] == 0)
				data->pwm = 0;
			else
				data->pwm = 255;
		}
                data->thermal_fault_state =
			scu_pic_read_byte(client, I2C_GET_SCU_PIC_THERMAL_FAULT_STATE);
		data->temp[0][0] =
			scu_pic_read_byte(client, I2C_GET_SCU_PIC_LOCAL_TEMP);
		data->temp[1][0] =
			scu_pic_read_byte(client, I2C_GET_SCU_PIC_REMOTE_TEMP);

		data->temp[2][0] =
			scu_pic_read_byte(client, I2C_GET_SCU_PIC_LM75_PS_TEMP_H);
		data->temp[2][1] =
			scu_pic_read_byte(client, I2C_GET_SCU_PIC_LM75_PS_TEMP_L);

		data->temp[3][0] =
			scu_pic_read_byte(client, I2C_GET_SCU_PIC_LM75_BOTTOM_AIRFLOW_TEMP_H);
		data->temp[3][1] =
			scu_pic_read_byte(client, I2C_GET_SCU_PIC_LM75_BOTTOM_AIRFLOW_TEMP_L);

		data->temp[4][0] =
			scu_pic_read_byte(client, I2C_GET_SCU_PIC_LM75_TOP_AIRFLOW_TEMP_H);
		data->temp[4][1] =
			scu_pic_read_byte(client, I2C_GET_SCU_PIC_LM75_TOP_AIRFLOW_TEMP_L);

		data->last_updated = jiffies;
		data->valid = 1;
	}
	mutex_unlock(&data->i2c_lock);

	return data;
}

static ssize_t show_thermal_fault_state(struct device *dev,
			struct device_attribute *attr, char *buf)
{
    struct scu_pic_data * data = scu_pic_update_device(dev);
    return sprintf(buf, "%d\n", data->thermal_fault_state);
}

static DEVICE_ATTR(thermal_fault_state, S_IRUGO, show_thermal_fault_state, NULL);

static ssize_t show_pwm(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct scu_pic_data *data = scu_pic_update_device(dev);

	return sprintf(buf, "%d\n", data->pwm);
}

static ssize_t set_pwm(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct scu_pic_data *data = i2c_get_clientdata(client);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;
	if (val > 255)
		return -EINVAL;

	mutex_lock(&data->i2c_lock);
	/* Can not override automatic control, ignore write if enabled */
	if (data->pwm_state == 2)
		val = 255;
	data->pwm = val ? 255 : 0;
	scu_pic_write_byte(client, I2C_SET_SCU_PIC_FAN_STATE, val ? 1 : 0);
	mutex_unlock(&data->i2c_lock);
	return count;
}

static DEVICE_ATTR(pwm1, S_IRUGO | S_IWUSR, show_pwm, set_pwm);
static DEVICE_ATTR(pwm2, S_IRUGO | S_IWUSR, show_pwm, set_pwm);

static ssize_t show_pwm_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct scu_pic_data *data = scu_pic_update_device(dev);

	return sprintf(buf, "%d\n", data->pwm_state);
}

static ssize_t set_pwm_enable(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct scu_pic_data *data = i2c_get_clientdata(client);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;
	if (val == 0 || val > 2)
		return -EINVAL;

	mutex_lock(&data->i2c_lock);
	data->pwm_state = val;
	scu_pic_write_byte(client, I2C_SET_SCU_PIC_THERMAL_CONTROL_STATE,
			val == 2 ? 1 : 0);
	/* Manual control turns off fans, reflect in pwm value. */
	data->pwm = val == 2 ? 255 : 0;
	mutex_unlock(&data->i2c_lock);
	return count;
}

static DEVICE_ATTR(pwm1_enable, S_IRUGO | S_IWUSR, show_pwm_enable,
		   set_pwm_enable);
static DEVICE_ATTR(pwm2_enable, S_IRUGO | S_IWUSR, show_pwm_enable,
		   set_pwm_enable);

static int fan_from_reg(struct scu_pic_data *data, u8 reg)
{
	int speed = 0;
	int mult;

	if (reg == 0 || reg == 0xff)
		return 0;

	switch (data->fan_contr_model) {
	case FAN_CONTR_MODEL_ADM1031:
		mult = data->pwm_state == 2 ? 30 : 60;
		speed = DIV_ROUND_CLOSEST(11250 * mult, reg);
		break;
	case FAN_CONTR_MODEL_MAX6639:
		speed = DIV_ROUND_CLOSEST(8000 * 30, reg);
		break;
	default:
		break;
	}
	return speed;
}

static ssize_t show_fan(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int nr = to_sensor_dev_attr(attr)->index;
	struct scu_pic_data *data = scu_pic_update_device(dev);

	return sprintf(buf, "%d\n", fan_from_reg(data, data->fan[nr]));
}

static ssize_t show_fan_div(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct scu_pic_data *data = scu_pic_update_device(dev);
	int div = data->pwm_state == 2 ? 2 : 1;

	return sprintf(buf, "%d\n", div);
}

static SENSOR_DEVICE_ATTR(fan1_input, S_IRUGO, show_fan, NULL, 0);
static SENSOR_DEVICE_ATTR(fan2_input, S_IRUGO, show_fan, NULL, 1);
static SENSOR_DEVICE_ATTR(fan1_div, S_IRUGO, show_fan_div, NULL, 0);
static SENSOR_DEVICE_ATTR(fan2_div, S_IRUGO, show_fan_div, NULL, 1);

static ssize_t show_temp(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	int nr = to_sensor_dev_attr(attr)->index;
	struct scu_pic_data *data = scu_pic_update_device(dev);
	int temp;

	/*
	 * Temperature is always considered as a 9-bit value with 0.5 degree
	 * Celsius resolution (i.e. as for an LM75) even for ADM1031 / MAX6639
	 * readings.
	 */
	if (data->temp[nr][0] != 0xFF || data->temp[nr][1] != 0xFF)
		temp = ((s16)(data->temp[nr][0] << 8 | data->temp[nr][1]) >> 7) * 500;
	else
		temp = -128000;

	return sprintf(buf, "%d\n", temp);
}

static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO,	show_temp, NULL, 0);
static SENSOR_DEVICE_ATTR(temp2_input, S_IRUGO,	show_temp, NULL, 1);
static SENSOR_DEVICE_ATTR(temp3_input, S_IRUGO,	show_temp, NULL, 2);
static SENSOR_DEVICE_ATTR(temp4_input, S_IRUGO,	show_temp, NULL, 3);
static SENSOR_DEVICE_ATTR(temp5_input, S_IRUGO,	show_temp, NULL, 4);

static ssize_t show_label(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	char const * const LABEL[] = {
		"local",
		"remote",
		"power_supply",
		"front",
		"back",
	};

	int nr = to_sensor_dev_attr(attr)->index;

	return sprintf(buf, "%s\n", LABEL[nr]);
}

static SENSOR_DEVICE_ATTR(temp1_label, S_IRUGO,	show_label, NULL, 0);
static SENSOR_DEVICE_ATTR(temp2_label, S_IRUGO,	show_label, NULL, 1);
static SENSOR_DEVICE_ATTR(temp3_label, S_IRUGO,	show_label, NULL, 2);
static SENSOR_DEVICE_ATTR(temp4_label, S_IRUGO,	show_label, NULL, 3);
static SENSOR_DEVICE_ATTR(temp5_label, S_IRUGO,	show_label, NULL, 4);

static ssize_t show_build_date(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct scu_pic_data *data = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%s\n",
		       data->build_date[0] ? data->build_date : "Unknown");
}

static DEVICE_ATTR(build_date, S_IRUGO, show_build_date, NULL);

static ssize_t show_build_date_bootloader(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct scu_pic_data *data = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%s\n",
		       data->build_date_bootloader[0] ? data->build_date_bootloader : "Unknown");
}

static DEVICE_ATTR(build_date_bootloader, S_IRUGO, show_build_date_bootloader, NULL);

static ssize_t show_version(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct scu_pic_data *data = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%u.%02u\n", data->version_major,
		       data->version_minor);
}

static DEVICE_ATTR(version, S_IRUGO, show_version, NULL);

static ssize_t show_version_bootloader(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct scu_pic_data *data = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%u.%02u\n", data->version_major_bootloader,
		       data->version_minor_bootloader);
}

static DEVICE_ATTR(version_bootloader, S_IRUGO, show_version_bootloader, NULL);

static int bootload_jump_to_application(struct i2c_client const * const client);
static struct i2c_client *scu_pic_reset_client;

/*
 * If linked into the kernel, the following function will replace
 * mach_reboot_fixups() in arch/x86/kernel/reboot.c.
 */
void mach_reboot_fixups(void)
{
	/* Don't bother about mutexes or error checking here. */
	if (scu_pic_reset_client) {
		struct scu_pic_data const * const data =
			i2c_get_clientdata(scu_pic_reset_client);

		if (data->in_bootloader)
			(void)bootload_jump_to_application(scu_pic_reset_client);
		else
			(void)scu_pic_write_byte(scu_pic_reset_client,
					   I2C_SET_SCU_PIC_RESET_HOST, 1);
	}

	/* is 20 ms enough time ? */
	mdelay(20);
}

static ssize_t show_reset(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0\n");
}

static ssize_t set_reset(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;
	if (val != 1)
		return -EINVAL;

	mach_reboot_fixups();
	return count;
}

static DEVICE_ATTR(reset, S_IRUGO | S_IWUSR, show_reset, set_reset);

/* firmware update */

#define SCU_PIC_FIRMWARE_NAME         "scu_pic.fw"
#define SCU_PIC_APPLICATION_BASE      (0x1000)  /* In units of 16-bit words. */
#define SCU_PIC_APPLICATION_NWORDS    (0x1000)  /* In units of 16-bit words. */
#define SCU_PIC_APPLICATION_END \
	(SCU_PIC_APPLICATION_BASE + SCU_PIC_APPLICATION_NWORDS)

#define BOOTLOAD_REG_ADDRESS_POINTER  (0x01)
#define BOOTLOAD_REG_DATA_DOWNLOAD    (0x02)
#define BOOTLOAD_REG_FLASH_READ       (0x03)
#define BOOTLOAD_REG_FLASH_ERASE      (0x04)
#define BOOTLOAD_REG_FLASH_WRITE      (0x05)
#define BOOTLOAD_REG_APPLICATION_JUMP (0x06)
#define BOOTLOAD_REG_EMBED_CRC        (0x07)

#if 0  /* Unused, but retained as part of the baseline interface to the PIC. */
static int bootload_get_address_pointer(struct i2c_client const * const client,
					u16 * const address)
{
	struct device const * const dev = &client->dev;
	u8 buf[2] = { client->addr, BOOTLOAD_REG_ADDRESS_POINTER };
	int err;

	err = i2c_master_send(client, buf, 2);
	if (err < 0)
		return err;

	err = i2c_master_recv(client, buf, ARRAY_SIZE(buf));
	if (err < 0)
		return err;

	*address = ((u16)buf[0] << 8) | (u16)buf[1];
	dev_dbg(dev, "bootload_get_address_pointer() @ 0x%04x\n", *address);
	return 0;
}
#endif

static int bootload_set_address_pointer(struct i2c_client const * const client,
					u16 const address)
{
	struct device const * const dev = &client->dev;
	u8 buf[4] = { client->addr, BOOTLOAD_REG_ADDRESS_POINTER, (address >> 8), address };
	int err;

	err = i2c_master_send(client, buf, ARRAY_SIZE(buf));
	if (err < 0)
		return err;

	dev_dbg(dev, "bootload_set_address_pointer(0x%04x)\n", address);
	return 0;
}

static int bootload_data_download(struct i2c_client const * const client,
				  u8 const * const data, size_t const nbytes)
{
	struct device const * const dev = &client->dev;
	u8 buf[18] = {
		client->addr, BOOTLOAD_REG_DATA_DOWNLOAD,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	};
	int err;

	if (!data || (nbytes > 16))
		return -EINVAL;

	(void)memcpy(&buf[2], data, nbytes);

	err = i2c_master_send(client, buf, ARRAY_SIZE(buf));
	if (err < 0)
		return err;

	dev_dbg(dev, "bootload_data_download() transferred %d\n", nbytes);
	return 0;
}

static int bootload_flash_read(struct i2c_client const * const client,
			       u8 * const data, size_t const nbytes)
{
	struct device const * const dev = &client->dev;
	u8 buf[17] = { client->addr, BOOTLOAD_REG_FLASH_READ };
	int err;

	if ((!data) || (nbytes > 16))
		return -EINVAL;

	err = i2c_master_send(client, buf, 2);
	if (err < 0)
		return err;

	err = i2c_master_recv(client, buf, 16);
	if (err < 0)
		return err;

	(void)memcpy(data, buf, nbytes);

	udelay(100);
	dev_dbg(dev, "bootload_flash_read() read %d\n", nbytes);
	return 0;
}

static int bootload_flash_erase(struct i2c_client const * const client)
{
	struct device const * const dev = &client->dev;
	u8 buf[18] = {
		client->addr, BOOTLOAD_REG_DATA_DOWNLOAD,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	};
	int err;

	/*
	 * Erase is implemented as a "write" of blank data values.  The PIC
	 * actually erases on-the-fly as new values are written to flash, but
	 * we blank everything out just to be safe (rather than possibly leave
	 * random opcodes in flash from previous, larger, firmware images).
	 */

	err = i2c_master_send(client, buf, ARRAY_SIZE(buf));
	if (err < 0)
		return err;

	buf[1] = BOOTLOAD_REG_FLASH_WRITE;
	err = i2c_master_send(client, buf, 2);
	if (err < 0)
		return err;

	err = i2c_master_recv(client, buf, 1);
	if (err < 0)
		return err;

	udelay(100);
	dev_dbg(dev, "bootload_flash_erase() = 0x%02x\n", buf[0]);
	return buf[0];
}

static int bootload_flash_write(struct i2c_client const * const client)
{
	struct device const * const dev = &client->dev;
	u8 buf[2] = { client->addr, BOOTLOAD_REG_FLASH_WRITE };
	int err;

	err = i2c_master_send(client, buf, ARRAY_SIZE(buf));
	if (err < 0)
		return err;

	err = i2c_master_recv(client, buf, 1);
	if (err < 0)
		return err;

	udelay(100);
	dev_dbg(dev, "bootload_flash_write() = 0x%02x\n", buf[0]);
	return buf[0];
}

static int bootload_jump_to_application(struct i2c_client const * const client)
{
	struct device const * const dev = &client->dev;
	u8 buf[2] = { client->addr, BOOTLOAD_REG_APPLICATION_JUMP };
	int err;

	dev_info(dev, "%s: Switching to PIC application & rebooting...\n", __func__);

	err = i2c_master_send(client, buf, ARRAY_SIZE(buf));
	if (err < 0)
		return err;

	err = i2c_master_recv(client, buf, 1);
	if (err < 0)
		return err;

	dev_dbg(dev, "bootload_jump_to_application() = 0x%02x\n", buf[0]);
	return buf[0];
}

static int bootload_embed_crc(struct i2c_client const * const client)
{
	struct device const * const dev = &client->dev;
	u8 buf[2] = { client->addr, BOOTLOAD_REG_EMBED_CRC };
	int err;

	dev_info(dev, "%s: Embedding CRC into PIC EEPROM...\n", __func__);

	err = i2c_master_send(client, buf, ARRAY_SIZE(buf));
	if (err < 0)
		return err;

	err = i2c_master_recv(client, buf, 1);
	if (err < 0)
		return err;

	dev_dbg(dev, "bootload_embed_crc() = 0x%02x\n", buf[0]);
	return buf[0];
}

static int load_firmware(struct device * const dev, u8 * const buf,
			 size_t const nbytes_buf)
{
	struct firmware const *fw;
	struct ihex_binrec const *rec;
	size_t nrecords = 0;
	size_t nbytes = 0;
	int err;
	int i;

	/*
	 * The SCU PIC includes 14-bit flash program memory; initialize the
	 * entire buffer with the "erased" value (0x3FFF) prior to loading
	 * firmware chunks into the buffer.
	 */
	for (i = 0; i < nbytes_buf;) {
		buf[i++] = 0xFF;
		buf[i++] = 0x3F;
	}

	/* Obtain a handle to the firmware file. */
	err = request_ihex_firmware(&fw, SCU_PIC_FIRMWARE_NAME, dev);
	if (err) {
		dev_err(dev, "Firmware request for '%s' failed (%d).\n",
				SCU_PIC_FIRMWARE_NAME, err);
		return err;
	}

	rec = (struct ihex_binrec const *)fw->data;
	while (rec) {
		u32 const addr_base = (SCU_PIC_APPLICATION_BASE << 1);
		u32 const addr_end = (SCU_PIC_APPLICATION_END << 1) - 1;
		u32 addr = be32_to_cpu(rec->addr);
		u16 len = be16_to_cpu(rec->len);

		if (((addr + len) < addr_base) || (addr > addr_end)) {
			/* Skip. */
			dev_dbg(dev, "Skipped load of %d bytes @ %04x.\n", len,
					addr);
		} else {
			u8 const *src = rec->data;
			u8 *dst;

			if (addr < addr_base) {
				int delta = (addr_base - addr);

				addr += delta;
				BUG_ON(addr != addr_base);
				src += delta;
				BUG_ON(src > (rec->data + len));
				len -= delta;
				BUG_ON((int)len <= 0);
			}

			if ((addr + len) > addr_end) {
				len = ((addr_end - addr) + 1);
				BUG_ON((int)len <= 0);
				BUG_ON((int)len > 16);
			}

			dst = buf + (addr - addr_base);
			BUG_ON((dst + len) > (buf + nbytes_buf));

			(void)memcpy(dst, src, len);
			nbytes += len;

			dev_dbg(dev, "Loaded %d bytes @ %04x (offset %04x).\n",
				len, addr, (addr - addr_base));
		}

		nrecords++;
		rec = ihex_next_binrec(rec);
	}

	dev_info(dev, "%s: Loaded firmware from '%s' (%zu/%zu bytes in %zu records).\n",
		 __func__, SCU_PIC_FIRMWARE_NAME, nbytes, fw->size, nrecords);

	release_firmware(fw);

	return nbytes;
}


/*
 * The bootloader returns "magic" major and minor versions of 'B' and 'L',
 * respectively, when queried.  This allows us to determine when the jump to
 * bootloader mode is complete and the PIC I2C slave interface is again active.
 */

#define BOOTLOADER_MAGIC_MAJOR ('B')
#define BOOTLOADER_MAGIC_MINOR ('L')

static int exec_bootloader(struct device const * const dev)
{
	struct i2c_client const * const client = to_i2c_client(dev);
	struct scu_pic_data * const data = i2c_get_clientdata(client);
	int retry = 5;

	dev_info(dev, "%s: Switching to PIC bootloader...\n", __func__);

	do {
		int major, minor;

		(void)scu_pic_write_byte(client, I2C_SET_SCU_PIC_RESET_TO_BOOTLOADER, 1);
		usleep_range(50000, 100000);
		major = scu_pic_read_byte(client, I2C_GET_SCU_PIC_FIRMWARE_REV_MAJOR);
		minor = scu_pic_read_byte(client, I2C_GET_SCU_PIC_FIRMWARE_REV_MINOR);
		if (major == BOOTLOADER_MAGIC_MAJOR && minor == BOOTLOADER_MAGIC_MINOR) {
			dev_info(dev, "%s: Bootloader started successfully.\n", __func__);
			data->in_bootloader = true;
			break;
		}
	} while (--retry);

	return retry ? 0 : -EIO;
}

static int erase_flash(struct device const * const dev, u16 const address,
		       size_t const nwords)
{
	struct i2c_client const * const client = to_i2c_client(dev);
	struct scu_pic_data *data = i2c_get_clientdata(client);
	size_t nwords_erased;
	int err;

	/* Flash operations are all 8-word multiples and 8-word aligned. */
	BUG_ON(address & 0x7);
	BUG_ON(nwords & 0x7);

	dev_info(dev, "%s: Erasing firmware flash segment...\n", __func__);

	/* Load 'base address' into the address pointer. */
	err = bootload_set_address_pointer(client, address);
	if (err) {
		dev_err(dev, "Set Address Pointer operation failed.\n");
		goto out;
	}

	/* Erase. */
	for (nwords_erased = 0; nwords_erased < nwords; nwords_erased += 8) {
		err = bootload_flash_erase(client);
		if (err) {
			dev_err(dev, "Erase Flash operation failed.\n");
			goto out;
		}

#if defined(CONFIG_SCU_PIC_FAULT_INJECTION)
		if ((data->fault_point >= FAULT_POINT_DURING_ERASE) &&
				(nwords_erased > (nwords / 2))) {
			dev_info(dev, "Synthetic fault during flash erase.\n");
			err = -EIO;
			goto out;
		}
#endif

		data->update_progress += 8;
	}

out:
	return err;
}

static int blank_check_flash(struct device const * const dev, u16 const address,
		       size_t const nwords)
{
	struct i2c_client const * const client = to_i2c_client(dev);
	struct scu_pic_data *data = i2c_get_clientdata(client);
	size_t nwords_checked;
	int err;

	/* Flash operations are all 8-word multiples and 8-word aligned. */
	BUG_ON(address & 0x7);
	BUG_ON(nwords & 0x7);

	dev_info(dev, "%s: Blank checking firmware flash segment...\n", __func__);

	/* Load 'base address' into the address pointer. */
	err = bootload_set_address_pointer(client, address);
	if (err) {
		dev_err(dev, "Set Address Pointer operation failed.\n");
		goto out;
	}

	/* Blank check. */
	for (nwords_checked = 0; nwords_checked < nwords; nwords_checked += 8) {
		u8 buf[16] = { 0 };
		size_t i;

		err = bootload_flash_read(client, buf, ARRAY_SIZE(buf));
		if (err) {
			dev_err(dev, "Read Flash operation failed.\n");
			goto out;
		}

		for (i = 0; i < ARRAY_SIZE(buf); i += 2) {
			u16 val = ((u16)buf[i + 1] << 8) | (u16)buf[i];

			if (val != 0x3FFF) {
				dev_err(dev, "Flash blank check failed at offset 0x%04x (read 0x%04x).\n",
					(address + nwords_checked + i), val);
				err = -EIO;
				goto out;
			}
		}

#if defined(CONFIG_SCU_PIC_FAULT_INJECTION)
		if ((data->fault_point >= FAULT_POINT_DURING_BLANK_CHECK) &&
				(nwords_checked > (nwords / 2))) {
			dev_info(dev, "Synthetic fault during flash blank check.\n");
			err = -EIO;
			goto out;
		}
#endif

		data->update_progress += 8;
	}

out:
	return err;
}

static int write_flash(struct device const * const dev, u16 const address,
		       size_t const nwords, u8 const * const buf,
		       size_t const nbytes_buf)
{
	struct i2c_client const * const client = to_i2c_client(dev);
	struct scu_pic_data *data = i2c_get_clientdata(client);
	size_t nwords_written;
	int err;

	/* Flash operations are all 8-word multiples and 8-word aligned. */
	BUG_ON(address & 0x7);
	BUG_ON(nwords & 0x7);

	dev_info(dev, "%s: Writing firmware data...\n", __func__);

	/* Buffer size we're about to write needs to match the segment size. */
	if (nbytes_buf != (nwords << 1)) {
		dev_err(dev, "Flash write failed - invalid source buffer size (0x%04x).\n", nbytes_buf);
		err = -EINVAL;
		goto out;
	}

	/* Load 'base address' into the address pointer. */
	err = bootload_set_address_pointer(client, address);
	if (err) {
		dev_err(dev, "Set Address Pointer operation failed.\n");
		goto out;
	}

	/* Write. */
	for (nwords_written = 0; nwords_written < nwords; nwords_written += 8) {
		err = bootload_data_download(client, &buf[nwords_written << 1], 16);
		if (err) {
			dev_err(dev, "Data Download operation failed.\n");
			goto out;
		}

		err = bootload_flash_write(client);
		if (err) {
			dev_err(dev, "Flash Write operation failed.\n");
			goto out;
		}

#if defined(CONFIG_SCU_PIC_FAULT_INJECTION)
		if ((data->fault_point >= FAULT_POINT_DURING_WRITE) &&
				(nwords_written > (nwords / 2))) {
			dev_info(dev, "Synthetic fault during flash write.\n");
			err = -EIO;
			goto out;
		}
#endif

		data->update_progress += 8;
	}

out:
	return err;
}

static int verify_flash(struct device const * const dev, u16 const address,
			size_t const nwords, u8 const * const buf,
			size_t const nbytes_buf)
{
	struct i2c_client const * const client = to_i2c_client(dev);
	struct scu_pic_data *data = i2c_get_clientdata(client);
	size_t nwords_checked;
	int err;

	/* Flash operations are all 8-word multiples and 8-word aligned. */
	BUG_ON(address & 0x7);
	BUG_ON(nwords & 0x7);

	dev_info(dev, "%s: Verifying firmware data...\n", __func__);

	/* Buffer size we're about to write needs to match the segment size. */
	if (nbytes_buf != (nwords << 1)) {
		dev_err(dev, "Flash verify failed - invalid source buffer size (0x%04x).\n", nbytes_buf);
		err = -EINVAL;
		goto out;
	}

	/* Load 'base address' into the address pointer. */
	err = bootload_set_address_pointer(client, address);
	if (err) {
		dev_err(dev, "Set Address Pointer operation failed.\n");
		goto out;
	}

	/* Verify. */
	for (nwords_checked = 0; nwords_checked < nwords; nwords_checked += 8) {
		u8 buf_r[16] = { 0 };
		size_t i;

		err = bootload_flash_read(client, buf_r, ARRAY_SIZE(buf_r));
		if (err) {
			dev_err(dev, "Flash Read operation failed.\n");
			goto out;
		}

		for (i = 0; i < ARRAY_SIZE(buf_r); i += 2) {
			u16 val_w = ((u16)buf[(nwords_checked << 1) + i + 1] << 8) |
				    (u16)buf[(nwords_checked << 1) + i];
			u16 val_r = ((u16)buf_r[i + 1] << 8) | (u16)buf_r[i];

			if (val_r != val_w) {
				dev_err(dev, "Flash verify failed at offset 0x%04x (expected 0x%04x, read 0x%04x).\n",
					(address + nwords_checked + i), val_w, val_r);
				err = -EIO;
				goto out;
			}
		}

#if defined(CONFIG_SCU_PIC_FAULT_INJECTION)
		if ((data->fault_point >= FAULT_POINT_DURING_VERIFY) &&
				(nwords_checked > (nwords / 2))) {
			dev_info(dev, "Synthetic fault during flash verify.\n");
			err = -EIO;
			goto out;
		}
#endif

		data->update_progress += 8;
	}

out:
	return err;
}

static ssize_t update_firmware(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct scu_pic_data *data = i2c_get_clientdata(client);
	u8 *fw_buf;
	size_t nbytes_fw_buf = (SCU_PIC_APPLICATION_NWORDS << 1);
	int err;

	fw_buf = kmalloc(nbytes_fw_buf, GFP_KERNEL);
	if (!fw_buf)
		return -ENOMEM;

	dev_info(dev, "%s: Firmware update started...\n", __func__);
	dev_info(dev, "%s: Firmware flash segment is %u words at offset 0x%04x.\n",
		 __func__, SCU_PIC_APPLICATION_NWORDS, SCU_PIC_APPLICATION_BASE);

	err = load_firmware(dev, fw_buf, nbytes_fw_buf);
	if (err < 0)
		goto exit_free;

	mutex_lock(&data->i2c_lock);
	data->update_progress = 0;

	err = exec_bootloader(dev);
	if (err < 0) {
		data->update_progress = err;
		goto exit_unlock;
	}

#if defined(CONFIG_SCU_PIC_FAULT_INJECTION)
	if (data->fault_point >= FAULT_POINT_BEFORE_ERASE) {
		dev_info(dev, "Synthetic fault before flash erase.\n");
		err = -EIO;
		data->update_progress = err;
		goto exit_unlock;
	}
#endif

	err = erase_flash(dev, SCU_PIC_APPLICATION_BASE,
			  SCU_PIC_APPLICATION_NWORDS);
	if (err < 0) {
		data->update_progress = err;
		goto exit_unlock;
	}

#if defined(CONFIG_SCU_PIC_FAULT_INJECTION)
	if (data->fault_point >= FAULT_POINT_BEFORE_BLANK_CHECK) {
		dev_info(dev, "Synthetic fault before flash blank check.\n");
		err = -EIO;
		data->update_progress = err;
		goto exit_unlock;
	}
#endif

	err = blank_check_flash(dev, SCU_PIC_APPLICATION_BASE,
				SCU_PIC_APPLICATION_NWORDS);
	if (err < 0) {
		data->update_progress = err;
		goto exit_unlock;
	}

#if defined(CONFIG_SCU_PIC_FAULT_INJECTION)
	if (data->fault_point >= FAULT_POINT_BEFORE_WRITE) {
		dev_info(dev, "Synthetic fault before flash write.\n");
		err = -EIO;
		data->update_progress = err;
		goto exit_unlock;
	}
#endif

	err = write_flash(dev, SCU_PIC_APPLICATION_BASE,
			  SCU_PIC_APPLICATION_NWORDS, fw_buf, nbytes_fw_buf);
	if (err < 0) {
		data->update_progress = err;
		goto exit_unlock;
	}

#if defined(CONFIG_SCU_PIC_FAULT_INJECTION)
	if (data->fault_point >= FAULT_POINT_BEFORE_VERIFY) {
		dev_info(dev, "Synthetic fault before flash verify.\n");
		err = -EIO;
		data->update_progress = err;
		goto exit_unlock;
	}
#endif

	err = verify_flash(dev, SCU_PIC_APPLICATION_BASE,
			  SCU_PIC_APPLICATION_NWORDS, fw_buf, nbytes_fw_buf);
	if (err < 0) {
		data->update_progress = err;
		goto exit_unlock;
	}

#if defined(CONFIG_SCU_PIC_FAULT_INJECTION)
	if (data->fault_point >= FAULT_POINT_EMBED_CRC) {
		dev_info(dev, "Synthetic fault before embedding CRC.\n");
		err = -EIO;
		data->update_progress = err;
		goto exit_unlock;
	}
#endif

	err = bootload_embed_crc(client);
	if (err < 0) {
		data->update_progress = err;
		goto exit_unlock;
	}

	data->update_progress = 0;

exit_unlock:
	mutex_unlock(&data->i2c_lock);
exit_free:
	kfree(fw_buf);
	return err ? err : 0;
}

static ssize_t set_update_firmware(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	unsigned long val;
	int err;

	err = kstrtoul(buf, 0, &val);
	if (err)
		return err;
	if (val != 1)
		return -EINVAL;

	err = update_firmware(dev);
	return err ? err : count;
}

static DEVICE_ATTR(update_firmware, S_IWUSR, NULL, set_update_firmware);

static ssize_t get_update_firmware_status(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct scu_pic_data *data = i2c_get_clientdata(to_i2c_client(dev));
	int percent_complete;

	/* FIXME - This really ought to have its own mutex; we can't really
	 *         co-opt i2c_lock because it's held the entire time firmware
	 *         update is in progress... */

	if (data->update_progress < 0)
		return sprintf(buf, "%d\n", data->update_progress);

	percent_complete = (data->update_progress * 100) / data->update_total;
	return sprintf(buf, "%d\n", percent_complete);
}

static DEVICE_ATTR(update_firmware_status, S_IRUGO, get_update_firmware_status,
		   NULL);

#if defined(CONFIG_SCU_PIC_FAULT_INJECTION)
static ssize_t get_update_firmware_fault(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct scu_pic_data *data = i2c_get_clientdata(to_i2c_client(dev));

	/* FIXME - This really ought to have its own mutex; we can't really
	 *         co-opt i2c_lock because it's held the entire time firmware
	 *         update is in progress... */

	return sprintf(buf, "%u\n", data->fault_point);
}

static ssize_t set_update_firmware_fault(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct scu_pic_data *data = i2c_get_clientdata(to_i2c_client(dev));
	unsigned long val;
	int err;

	err = kstrtoul(buf, 0, &val);
	if (err)
		return err;
	if (val > _FAULT_POINT_MAX)
		return -EINVAL;

	data->fault_point = val;

	return err ? err : count;
}

static DEVICE_ATTR(update_firmware_fault, S_IRUGO | S_IWUSR,
		   get_update_firmware_fault, set_update_firmware_fault);
#endif

static struct attribute *scu_pic_attributes_v4[] = {
	&sensor_dev_attr_fan1_input.dev_attr.attr,
	&sensor_dev_attr_fan1_div.dev_attr.attr,
	&sensor_dev_attr_fan2_input.dev_attr.attr,
	&sensor_dev_attr_fan2_div.dev_attr.attr,
	&dev_attr_pwm1.attr,
	&dev_attr_pwm1_enable.attr,
	&dev_attr_pwm2.attr,
	&dev_attr_pwm2_enable.attr,
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp2_input.dev_attr.attr,
	&sensor_dev_attr_temp1_label.dev_attr.attr,
	&sensor_dev_attr_temp2_label.dev_attr.attr,
	&dev_attr_reset.attr,
	&dev_attr_reset_reason.attr,
	&dev_attr_version.attr,
	NULL
};

static const struct attribute_group scu_pic_group_v4 = {
	.attrs = scu_pic_attributes_v4,
};

static struct attribute *scu_pic_attributes_v6[] = {
	&sensor_dev_attr_fan1_input.dev_attr.attr,
	&sensor_dev_attr_fan1_div.dev_attr.attr,
	&sensor_dev_attr_fan2_input.dev_attr.attr,
	&sensor_dev_attr_fan2_div.dev_attr.attr,
	&dev_attr_pwm1.attr,
	&dev_attr_pwm1_enable.attr,
	&dev_attr_pwm2.attr,
	&dev_attr_pwm2_enable.attr,
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp2_input.dev_attr.attr,
	&sensor_dev_attr_temp3_input.dev_attr.attr,
	&sensor_dev_attr_temp4_input.dev_attr.attr,
	&sensor_dev_attr_temp5_input.dev_attr.attr,
	&sensor_dev_attr_temp1_label.dev_attr.attr,
	&sensor_dev_attr_temp2_label.dev_attr.attr,
	&sensor_dev_attr_temp3_label.dev_attr.attr,
	&sensor_dev_attr_temp4_label.dev_attr.attr,
	&sensor_dev_attr_temp5_label.dev_attr.attr,
	&dev_attr_reset.attr,
	&dev_attr_reset_reason.attr,
	&dev_attr_reset_pin_state.attr,
	&dev_attr_thermal_override_state.attr,
        &dev_attr_thermal_fault_state.attr,
	&dev_attr_update_firmware.attr,
#if defined(CONFIG_SCU_PIC_FAULT_INJECTION)
	&dev_attr_update_firmware_fault.attr,
#endif
	&dev_attr_update_firmware_status.attr,
	&dev_attr_version.attr,
	&dev_attr_version_bootloader.attr,
	&dev_attr_build_date.attr,
	&dev_attr_build_date_bootloader.attr,
	NULL
};

static const struct attribute_group scu_pic_group_v6 = {
	.attrs = scu_pic_attributes_v6,
};

static struct attribute *scu_pic_attributes_bootloader[] = {
	&dev_attr_reset.attr,
	&dev_attr_update_firmware.attr,
#if defined(CONFIG_SCU_PIC_FAULT_INJECTION)
	&dev_attr_update_firmware_fault.attr,
#endif
	&dev_attr_update_firmware_status.attr,
	&dev_attr_version_bootloader.attr,
	&dev_attr_build_date_bootloader.attr,
	NULL
};

static const struct attribute_group scu_pic_group_bootloader = {
	.attrs = scu_pic_attributes_bootloader,
};

static int scu_pic_hwmon_init_v4(struct i2c_client *client)
{
	struct scu_pic_data *data = i2c_get_clientdata(client);
	struct device *dev = &client->dev;
	int err;

	/* Register sysfs hooks */
	err = sysfs_create_group(&dev->kobj, &scu_pic_group_v4);
	if (err)
		return err;

	data->hwmon_dev = hwmon_device_register(dev);
	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		goto exit_remove;
	}
	return 0;

exit_remove:
	sysfs_remove_group(&dev->kobj, &scu_pic_group_v4);
	return err;
}

static void scu_pic_hwmon_exit_v4(struct i2c_client *client)
{
	struct scu_pic_data *data = i2c_get_clientdata(client);

	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_group(&client->dev.kobj, &scu_pic_group_v4);
}

static void scu_pic_hwmon_exit_v5(struct i2c_client *client)
{
	return scu_pic_hwmon_exit_v4(client);
}

static int scu_pic_hwmon_init_v6(struct i2c_client *client)
{
	struct scu_pic_data *data = i2c_get_clientdata(client);
	struct device *dev = &client->dev;
	int err;

	/* Register sysfs hooks */
	err = sysfs_create_group(&dev->kobj, &scu_pic_group_v6);
	if (err)
		return err;

	data->hwmon_dev = hwmon_device_register(dev);
	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		goto exit_remove;
	}
	return 0;

exit_remove:
	sysfs_remove_group(&dev->kobj, &scu_pic_group_v6);
	return err;
}

static void scu_pic_hwmon_exit_v6(struct i2c_client *client)
{
	struct scu_pic_data *data = i2c_get_clientdata(client);

	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_group(&client->dev.kobj, &scu_pic_group_v6);
}

static int scu_pic_hwmon_init_bootloader(struct i2c_client *client)
{
	struct scu_pic_data *data = i2c_get_clientdata(client);
	struct device *dev = &client->dev;
	int err;

	/* Register sysfs hooks */
	err = sysfs_create_group(&dev->kobj, &scu_pic_group_bootloader);
	if (err)
		return err;

	data->hwmon_dev = hwmon_device_register(dev);
	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		goto exit_remove;
	}
	return 0;

exit_remove:
	sysfs_remove_group(&dev->kobj, &scu_pic_group_bootloader);
	return err;
}

static void scu_pic_hwmon_exit_bootloader(struct i2c_client *client)
{
	struct scu_pic_data *data = i2c_get_clientdata(client);

	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_group(&client->dev.kobj, &scu_pic_group_bootloader);
}

/* led */

static void scu_pic_led_set(struct led_classdev *led_cdev,
			enum led_brightness brightness)
{
	struct scu_pic_data *data
		    = container_of(led_cdev, struct scu_pic_data, cdev);

	mutex_lock(&data->i2c_lock);
	scu_pic_write_byte(data->client, I2C_SET_SCU_PIC_FAULT_LED_STATE,
			brightness);
	mutex_unlock(&data->i2c_lock);
}

static enum led_brightness scu_pic_led_get(struct led_classdev *led_cdev)
{
	struct scu_pic_data *data
		    = container_of(led_cdev, struct scu_pic_data, cdev);
	int ret;

	mutex_lock(&data->i2c_lock);
	ret = scu_pic_read_byte(data->client, I2C_GET_SCU_PIC_FAULT_LED_STATE);
	mutex_unlock(&data->i2c_lock);
	return ret;
}

static int scu_pic_led_init_v4(struct i2c_client *client)
{
	struct scu_pic_data *data = i2c_get_clientdata(client);

	data->cdev.name = "scu_status:r:Fault";
	data->cdev.brightness_set = scu_pic_led_set;
	data->cdev.brightness_get = scu_pic_led_get;
	data->cdev.max_brightness = 1;
	data->cdev.flags = LED_CORE_SUSPENDRESUME;
	return led_classdev_register(&client->dev, &data->cdev);
}

static int scu_pic_led_init_v6(struct i2c_client *client)
{
	return scu_pic_led_init_v4(client);
}

static void scu_pic_led_exit_v4(struct i2c_client *client)
{
	struct scu_pic_data *data = i2c_get_clientdata(client);

	led_classdev_unregister(&data->cdev);
}

static void scu_pic_led_exit_v5(struct i2c_client *client)
{
	return scu_pic_led_exit_v4(client);
}

static void scu_pic_led_exit_v6(struct i2c_client *client)
{
	return scu_pic_led_exit_v4(client);
}

static void scu_pic_led_exit_bootloader(struct i2c_client *client)
{
}

/* wdt */

static int scu_pic_wdt_ping(struct watchdog_device *wdev)
{
	struct scu_pic_data *data = watchdog_get_drvdata(wdev);

	mutex_lock(&data->i2c_lock);
	/* Any host communication resets watchdog */
	if (data->client)
		(void)scu_pic_read_byte(data->client,
					I2C_GET_SCU_PIC_WDT_STATE);
	mod_timer(&data->wdt_timer,
		  jiffies + min_t(unsigned long, SCU_PIC_WDT_TIMEOUT / 2,
				  wdev->timeout) * HZ);
	data->wdt_lastping = jiffies;
	mutex_unlock(&data->i2c_lock);

	return 0;
}

static int scu_pic_wdt_start(struct watchdog_device *wdev)
{
	struct scu_pic_data *data = watchdog_get_drvdata(wdev);

	mutex_lock(&data->i2c_lock);
	if (data->client)
		scu_pic_write_byte(data->client, I2C_SET_SCU_PIC_WDT_STATE, 1);
	mod_timer(&data->wdt_timer, jiffies + min_t(unsigned long,
						    SCU_PIC_WDT_TIMEOUT / 2,
						    wdev->timeout) * HZ);
	mutex_unlock(&data->i2c_lock);

	return 0;
}

static int scu_pic_wdt_stop(struct watchdog_device *wdev)
{
	struct scu_pic_data *data = watchdog_get_drvdata(wdev);

	mutex_lock(&data->i2c_lock);
	if (data->client)
		scu_pic_write_byte(data->client, I2C_SET_SCU_PIC_WDT_STATE, 0);
	del_timer(&data->wdt_timer);
	mutex_unlock(&data->i2c_lock);

	return 0;
}

static int scu_pic_wdt_set_timeout(struct watchdog_device *wdev, unsigned int t)
{
	wdev->timeout = t;
	scu_pic_wdt_ping(wdev);

	return 0;
}

static void scu_pic_wdt_timerfunc(unsigned long d)
{
	struct watchdog_device *wdev = (struct watchdog_device *)d;
	struct scu_pic_data *data = watchdog_get_drvdata(wdev);

	if (time_after(jiffies, data->wdt_lastping + wdev->timeout * HZ)) {
		pr_crit("Software watchdog timeout: Initiating system reboot.\n");
		emergency_restart();
	}
	scu_pic_wdt_ping(wdev);
}

static const struct watchdog_info scu_pic_wdt_ident = {
	.options	= WDIOF_MAGICCLOSE | WDIOF_KEEPALIVEPING |
			  WDIOF_SETTIMEOUT,
	.identity	= "SCU Pic Watchdog",
};

static const struct watchdog_ops scu_pic_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= scu_pic_wdt_start,
	.stop		= scu_pic_wdt_stop,
	.ping		= scu_pic_wdt_ping,
	.set_timeout	= scu_pic_wdt_set_timeout,
};

static int scu_pic_wdt_init_v4(struct i2c_client *client)
{
	struct scu_pic_data *data = i2c_get_clientdata(client);
	int ret = 0;

	mutex_lock(&scu_pic_data_mutex);

#if defined(CONFIG_SCU_PIC_MODULE)
	/* Disable watchdog at startup if running as module */
	scu_pic_write_byte(data->client, I2C_SET_scu_pic_WDT_STATE, 0);
#endif

	watchdog_set_drvdata(&data->wdt_dev, data);

	data->wdt_timer.function = scu_pic_wdt_timerfunc;
	data->wdt_timer.data = (unsigned long)&data->wdt_dev;
	init_timer(&data->wdt_timer);

	data->wdt_dev.info = &scu_pic_wdt_ident;
	data->wdt_dev.ops = &scu_pic_wdt_ops;
	data->wdt_dev.timeout = SCU_PIC_WDT_TIMEOUT;
	data->wdt_dev.min_timeout = 1;
	data->wdt_dev.max_timeout = 0xffff;
	watchdog_set_nowayout(&data->wdt_dev, nowayout);
	data->wdt_dev.parent = client->dev.parent;

	ret = watchdog_register_device(&data->wdt_dev);
	if (ret)
		goto error;
	list_add(&data->list, &scu_pic_data_list);
error:
	mutex_unlock(&scu_pic_data_mutex);
	return 0;
}

static int scu_pic_wdt_init_v6(struct i2c_client *client)
{
	return scu_pic_wdt_init_v4(client);
}

static int scu_pic_wdt_exit_v4(struct i2c_client *client)
{
	struct scu_pic_data *data = i2c_get_clientdata(client);

	del_timer(&data->wdt_timer);

	watchdog_unregister_device(&data->wdt_dev);
	mutex_lock(&scu_pic_data_mutex);
	list_del(&data->list);
	mutex_unlock(&scu_pic_data_mutex);

	/* Tell watchdog code that client is gone */
	mutex_lock(&data->i2c_lock);
	data->client = NULL;
	mutex_unlock(&data->i2c_lock);

	return 0;
}

static int scu_pic_wdt_exit_v5(struct i2c_client *client)
{
	return scu_pic_wdt_exit_v4(client);
}

static int scu_pic_wdt_exit_v6(struct i2c_client *client)
{
	return scu_pic_wdt_exit_v4(client);
}

static int scu_pic_wdt_exit_bootloader(struct i2c_client *client)
{
	return 0;
}

/* reset poll worker */

static void scu_pic_reset_poll(struct work_struct *work)
{
	struct scu_pic_data *data = container_of(to_delayed_work(work),
						 struct scu_pic_data, work);
	int state;

	state = get_reset_pin_state(data->client);
	if (state >= 0) {
		if (!!state != data->reset_pin_state)
			sysfs_notify(&data->client->dev.kobj, NULL,
				     "reset_pin_state");
		data->reset_pin_state = state;
	}
	queue_delayed_work(data->workqueue, &data->work,
			   msecs_to_jiffies(poll_rate));
}

/* device level functions */

static int probe_v4(struct i2c_client * const client)
{
	struct scu_pic_data *data = i2c_get_clientdata(client);
	struct device *dev = &client->dev;
	int err;

	/* Old firmware, default to ADM1031 */
	data->fan_contr_model = FAN_CONTR_MODEL_ADM1031;

	err = scu_pic_wdt_init_v4(client);
	if (err) {
		dev_err(dev, "Failed to init watchdog (%d)\n", err);
		return err;
	}

	err = scu_pic_led_init_v4(client);
	if (err) {
		dev_err(dev, "Failed to init leds (%d)\n", err);
		goto exit_wdt;
	}

	err = scu_pic_hwmon_init_v4(client);
	if (err) {
		dev_err(dev, "Failed to init hwmon (%d)\n", err);
		goto exit_led;
	}

	if (poll_rate) {
		data->workqueue = create_singlethread_workqueue("scu-pic-poll");
		if (data->workqueue == NULL) {
			err = -ENOMEM;
			goto exit_hwmon;
		}
		INIT_DELAYED_WORK(&data->work, scu_pic_reset_poll);
		queue_delayed_work(data->workqueue, &data->work,
				   msecs_to_jiffies(poll_rate));
	}

	dev_info(dev, "Firmware revision %d.%02d.\n",
		 data->version_major, data->version_minor);

	dev_info(dev, "Fan controller model 0x%02x, revision 0x%02x.\n",
		 data->fan_contr_model, data->fan_contr_rev);

	return 0;

exit_hwmon:
	scu_pic_hwmon_exit_v4(client);
exit_led:
	scu_pic_led_exit_v4(client);
exit_wdt:
	scu_pic_wdt_exit_v4(client);
	return err;
}

static int probe_v5(struct i2c_client * const client)
{
	return probe_v4(client);
}

static int probe_v6(struct i2c_client * const client)
{
	struct scu_pic_data *data = i2c_get_clientdata(client);
	struct device *dev = &client->dev;
	int major, minor;
	int model, rev;
	int err;

	major = scu_pic_read_byte(client, I2C_GET_SCU_PIC_BOOTLOADER_VERSION_MAJOR);
	minor = scu_pic_read_byte(client, I2C_GET_SCU_PIC_BOOTLOADER_VERSION_MINOR);

	if ((major < 0) || (minor < 0)) {
		dev_err(dev, "Bootloader major %d minor %d\n", major, minor);
		return -ENODEV;
	}

	data->version_major_bootloader = major;
	data->version_minor_bootloader = minor;

	err = scu_pic_read_build_date(client,
				      I2C_GET_SCU_PIC_BUILD_DATE,
				      data->build_date,
				      NBYTES_BUILD_DATE);
	if (err < 0) {
		dev_err(dev, "Failed to read PIC application build date(%d).\n", err);
		(void)memset(data->build_date, '\0', NBYTES_BUILD_DATE);
	}

	err = scu_pic_read_build_date(client,
				     I2C_GET_SCU_PIC_BOOTLOADER_BUILD_DATE,
				     data->build_date_bootloader,
				     NBYTES_BUILD_DATE);
	if (err < 0) {
		dev_err(dev, "Failed to read PIC bootloader build date (%d).\n", err);
		(void)memset(data->build_date_bootloader, '\0', NBYTES_BUILD_DATE);
	}

	model = scu_pic_read_byte(client,
				   I2C_GET_SCU_PIC_FAN_CONTR_MODEL);
	if (model < 0 || model == 0xff) {
		dev_err(dev,
			"Failed to read fan controller model (%d)\n",
			model);
		model = FAN_CONTR_MODEL_ADM1031;
	}
	rev = scu_pic_read_byte(client,
				 I2C_GET_SCU_PIC_FAN_CONTR_REV);
	if (rev < 0 || rev == 0xff) {
		dev_err(dev,
			"Failed to read fan controller revision (%d)\n",
			rev);
		rev = 0;
	}
	data->fan_contr_model = model;
	data->fan_contr_rev = rev;

	err = scu_pic_wdt_init_v6(client);
	if (err) {
		dev_err(dev, "Failed to init watchdog (%d)\n", err);
		return err;
	}

	err = scu_pic_led_init_v6(client);
	if (err) {
		dev_err(dev, "Failed to init leds (%d)\n", err);
		goto exit_wdt;
	}

	err = scu_pic_hwmon_init_v6(client);
	if (err) {
		dev_err(dev, "Failed to init hwmon (%d)\n", err);
		goto exit_led;
	}

	if (poll_rate) {
		data->workqueue = create_singlethread_workqueue("scu-pic-poll");
		if (data->workqueue == NULL) {
			err = -ENOMEM;
			goto exit_hwmon;
		}
		INIT_DELAYED_WORK(&data->work, scu_pic_reset_poll);
		queue_delayed_work(data->workqueue, &data->work,
				   msecs_to_jiffies(poll_rate));
	}

	dev_info(dev, "Firmware revision %d.%02d, built %s.\n",
		 data->version_major, data->version_minor,
		 data->build_date[0] ?  data->build_date : "Unknown");

	dev_info(dev, "Bootloader revision %d.%02d, built %s.\n",
		 data->version_major_bootloader, data->version_minor_bootloader,
		 data->build_date_bootloader[0] ?  data->build_date_bootloader : "Unknown");

	dev_info(dev, "Fan controller model 0x%02x, revision 0x%02x.\n",
		 data->fan_contr_model, data->fan_contr_rev);

#if defined(CONFIG_SCU_PIC_FAULT_INJECTION)
	dev_info(dev, "Synthetic fault injection support is ENABLED.\n");
#endif

	return 0;

exit_hwmon:
	scu_pic_hwmon_exit_v6(client);
exit_led:
	scu_pic_led_exit_v6(client);
exit_wdt:
	scu_pic_wdt_exit_v6(client);
	return err;
}

static int probe_bootloader(struct i2c_client * const client)
{
	struct scu_pic_data *data = i2c_get_clientdata(client);
	struct device *dev = &client->dev;
	int major, minor;
	int err;

	major = scu_pic_read_byte(client, I2C_GET_SCU_PIC_BOOTLOADER_VERSION_MAJOR);
	minor = scu_pic_read_byte(client, I2C_GET_SCU_PIC_BOOTLOADER_VERSION_MINOR);

	if ((major < 0) || (minor < 0)) {
		dev_err(dev, "Bootloader major %d minor %d\n", major, minor);
		return -ENODEV;
	}

	data->version_major_bootloader = major;
	data->version_minor_bootloader = minor;

	err = scu_pic_read_build_date(client,
				     I2C_GET_SCU_PIC_BOOTLOADER_BUILD_DATE,
				     data->build_date_bootloader,
				     NBYTES_BUILD_DATE);
	if (err < 0) {
		dev_err(dev, "Failed to read PIC bootloader build date (%d).\n", err);
		(void)memset(data->build_date_bootloader, '\0', NBYTES_BUILD_DATE);
	}

	err = scu_pic_hwmon_init_bootloader(client);
	if (err) {
		dev_err(dev, "Failed to init hwmon (%d)\n", err);
		return err;
	}

	dev_info(dev, "Firmware not present or corrupt.\n");
	dev_info(dev, "Bootloader revision %d.%02d, built %s.\n",
		 data->version_major_bootloader, data->version_minor_bootloader,
		 data->build_date_bootloader[0] ?  data->build_date_bootloader : "Unknown");

#if defined(CONFIG_SCU_PIC_FAULT_INJECTION)
	dev_info(dev, "Synthetic fault injection support is ENABLED.\n");
#endif

	return 0;
}

static int scu_pic_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct scu_pic_data *data;
	int major, minor;
	int err;

	major = scu_pic_read_byte(client, I2C_GET_SCU_PIC_FIRMWARE_REV_MAJOR);
	minor = scu_pic_read_byte(client, I2C_GET_SCU_PIC_FIRMWARE_REV_MINOR);

	if (major < 0 || minor < 0) {
		dev_err(dev, "major %d minor %d\n", major, minor);
		return -ENODEV;
	}

	data = kzalloc(sizeof(struct scu_pic_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	client->flags &= ~I2C_CLIENT_PEC;	/* PEC is not supported */

	i2c_set_clientdata(client, data);
	data->client = client;
	scu_pic_reset_client = client;

	mutex_init(&data->i2c_lock);
	kref_init(&data->kref);

	data->version_major = major;
	data->version_minor = minor;

	/* erase + blank check + write + verify */
	data->update_total = (SCU_PIC_APPLICATION_NWORDS * 4);

	/*
	 * If the FW is older than major version 6, there is no
	 * support for reading the fan controller model or rev. So
	 * just assume that it is the older ADM1031 board.
	 */
	if (data->version_major == BOOTLOADER_MAGIC_MAJOR &&
	    data->version_minor == BOOTLOADER_MAGIC_MINOR) {
		data->version_major = 0;
		data->version_minor = 0;
		data->in_bootloader = true;
		err = probe_bootloader(client);
	} else if (data->version_major == 4) {
		err = probe_v4(client);
	} else if (data->version_major == 5) {
		err = probe_v5(client);
	} else if (data->version_major == 6) {
		err = probe_v6(client);
	} else {
		err = -ENODEV;
	}

	if (err) {
		dev_err(dev, "Hardware probe failed.\n");
		goto exit_free;
	}

	return 0;

exit_free:
	kfree(data);
	return err;
}

static int scu_pic_remove(struct i2c_client *client)
{
	struct scu_pic_data *data = i2c_get_clientdata(client);

	scu_pic_reset_client = NULL;
	cancel_delayed_work_sync(&data->work);

	if (data->version_major == 4) {
		scu_pic_hwmon_exit_v4(client);
		scu_pic_led_exit_v4(client);
		scu_pic_wdt_exit_v4(client);
	} else if (data->version_major == 5) {
		scu_pic_hwmon_exit_v5(client);
		scu_pic_led_exit_v5(client);
		scu_pic_wdt_exit_v5(client);
	} else if (data->version_major == 6) {
		scu_pic_hwmon_exit_v6(client);
		scu_pic_led_exit_v6(client);
		scu_pic_wdt_exit_v6(client);
	} else if (data->in_bootloader) {
		scu_pic_hwmon_exit_bootloader(client);
		scu_pic_led_exit_bootloader(client);
		scu_pic_wdt_exit_bootloader(client);
	}

	mutex_lock(&scu_pic_data_mutex);
	kref_put(&data->kref, scu_pic_release_resources);
	mutex_unlock(&scu_pic_data_mutex);

	return 0;
}

static const struct i2c_device_id scu_pic_id[] = {
	{ "scu_pic", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, scu_pic_id);

static struct i2c_driver scu_pic_driver = {
	.driver = {
		.name = "scu_pic",
	},
	.probe		= scu_pic_probe,
	.remove		= scu_pic_remove,
	.id_table	= scu_pic_id,
	.address_list	= normal_i2c,
};

module_i2c_driver(scu_pic_driver);

MODULE_AUTHOR("Guenter Roeck <linux@roeck-us.net>");
MODULE_DESCRIPTION("SCU PIC driver");
MODULE_LICENSE("GPL");
