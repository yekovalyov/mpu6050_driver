#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/acpi.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>


#include "mpu6050-regs.h"


struct mpu6050_data {
	struct i2c_client *client;
	struct mutex  mutex;
	int accel_values[3];
	int gyro_values[3];
	int temperature;
};


static struct mpu6050_data g_mpu6050_data;

static int mpu6050_read_data(void)
{
	int temp;

	if (g_mpu6050_data.client == 0) {
		printk(KERN_ERR"mpu6050: client has not been initialized");
		return -ENODEV;
	}

	/* accel */
	g_mpu6050_data.accel_values[0] = (s16)((u16)i2c_smbus_read_word_swapped(g_mpu6050_data.client, REG_ACCEL_XOUT_H));
	g_mpu6050_data.accel_values[1] = (s16)((u16)i2c_smbus_read_word_swapped(g_mpu6050_data.client, REG_ACCEL_YOUT_H));
	g_mpu6050_data.accel_values[2] = (s16)((u16)i2c_smbus_read_word_swapped(g_mpu6050_data.client, REG_ACCEL_ZOUT_H));
	/* gyro */
	g_mpu6050_data.gyro_values[0] = (s16)((u16)i2c_smbus_read_word_swapped(g_mpu6050_data.client, REG_GYRO_XOUT_H));
	g_mpu6050_data.gyro_values[1] = (s16)((u16)i2c_smbus_read_word_swapped(g_mpu6050_data.client, REG_GYRO_YOUT_H));
	g_mpu6050_data.gyro_values[2] = (s16)((u16)i2c_smbus_read_word_swapped(g_mpu6050_data.client, REG_GYRO_ZOUT_H));
	/* temp */
	/* Temperature in degrees C = (TEMP_OUT Register Value  as a signed quantity)/340 + 36.53 */
	temp = (s16)((u16)i2c_smbus_read_word_swapped(g_mpu6050_data.client, REG_TEMP_OUT_H));
	g_mpu6050_data.temperature = (temp + 12420 + 170) / 340;

	printk(KERN_INFO "mpu6050: sensor data read:\n");
	printk(KERN_INFO "mpu6050: ACCEL[X,Y,Z] = [%d, %d, %d]\n",
		g_mpu6050_data.accel_values[0],
		g_mpu6050_data.accel_values[1],
		g_mpu6050_data.accel_values[2]);
	printk(KERN_INFO "mpu6050: GYRO[X,Y,Z] = [%d, %d, %d]\n",
		g_mpu6050_data.gyro_values[0],
		g_mpu6050_data.gyro_values[1],
		g_mpu6050_data.gyro_values[2]);
	printk(KERN_INFO "mpu6050: TEMP = %d\n", g_mpu6050_data.temperature);
	return 0;
}


static ssize_t accel_x_show(struct class *class, struct class_attribute *attr, char *buf)
{
	mpu6050_read_data();

	sprintf(buf, "%d\n", g_mpu6050_data.accel_values[0]);
	return strlen(buf);
}

static ssize_t accel_y_show(struct class *class, struct class_attribute *attr, char *buf)
{
	mpu6050_read_data();

	sprintf(buf, "%d\n", g_mpu6050_data.accel_values[1]);
	return strlen(buf);
}

static ssize_t accel_z_show(struct class *class, struct class_attribute *attr, char *buf)
{
	mpu6050_read_data();

	sprintf(buf, "%d\n", g_mpu6050_data.accel_values[2]);
	return strlen(buf);
}

static ssize_t gyro_x_show(struct class *class, struct class_attribute *attr, char *buf)
{
	mpu6050_read_data();

	sprintf(buf, "%d\n", g_mpu6050_data.gyro_values[0]);
	return strlen(buf);
}

static ssize_t gyro_y_show(struct class *class, struct class_attribute *attr, char *buf)
{
	mpu6050_read_data();

	sprintf(buf, "%d\n", g_mpu6050_data.gyro_values[0]);
	return strlen(buf);
}

static ssize_t gyro_z_show(struct class *class, struct class_attribute *attr, char *buf)
{
	mpu6050_read_data();

	sprintf(buf, "%d\n", g_mpu6050_data.gyro_values[0]);
	return strlen(buf);
}

static ssize_t temperature_show(struct class *class, struct class_attribute *attr, char *buf)
{
	if (mpu6050_read_data()) {
		*buf = '\0';
	} else {
		sprintf(buf, "%d\n", g_mpu6050_data.temperature);
	}
	return strlen(buf);
}


CLASS_ATTR_RO(accel_x);
CLASS_ATTR_RO(accel_y);
CLASS_ATTR_RO(accel_z);
CLASS_ATTR_RO(gyro_x);
CLASS_ATTR_RO(gyro_y);
CLASS_ATTR_RO(gyro_z);
CLASS_ATTR_RO(temperature);

static struct class *attr_class = 0;



static int mpu6050_init(void)
{
	int ret;

	/* Create i2c driver */
	/*
	ret = i2c_add_driver(&mpu6050_i2c_driver);
	if (ret) {
		printk(KERN_ERR "mpu6050: failed to add new i2c driver: %d\n", ret);
		return ret;
	}
	printk(KERN_INFO "mpu6050: i2c driver created\n");
	*/
	/* Create class */
	attr_class = class_create(THIS_MODULE, "mpu6050");
	if (IS_ERR(attr_class)) {
		ret = PTR_ERR(attr_class);
		printk(KERN_ERR "mpu6050: failed to create sysfs class: %d\n", ret);
		return ret;
	}
	printk(KERN_INFO "mpu6050: sysfs class created\n");

	/* Create accel_x */
	ret = class_create_file(attr_class, &class_attr_accel_x);
	if (ret) {
		printk(KERN_ERR "mpu6050: failed to create sysfs class attribute accel_x: %d\n", ret);
		return ret;
	}
	/* Create accel_y */
	ret = class_create_file(attr_class, &class_attr_accel_y);
	if (ret) {
		printk(KERN_ERR "mpu6050: failed to create sysfs class attribute accel_y: %d\n", ret);
		return ret;
	}
	/* Create accel_z */
	ret = class_create_file(attr_class, &class_attr_accel_z);
	if (ret) {
		printk(KERN_ERR "mpu6050: failed to create sysfs class attribute accel_z: %d\n", ret);
		return ret;
	}
	/* Create gyro_x */
	ret = class_create_file(attr_class, &class_attr_gyro_x);
	if (ret) {
		printk(KERN_ERR "mpu6050: failed to create sysfs class attribute gyro_x: %d\n", ret);
		return ret;
	}
	/* Create gyro_y */
	ret = class_create_file(attr_class, &class_attr_gyro_y);
	if (ret) {
		printk(KERN_ERR "mpu6050: failed to create sysfs class attribute gyro_y: %d\n", ret);
		return ret;
	}
	/* Create gyro_z */
	ret = class_create_file(attr_class, &class_attr_gyro_z);
	if (ret) {
		printk(KERN_ERR "mpu6050: failed to create sysfs class attribute gyro_z: %d\n", ret);
		return ret;
	}
	/* Create temperature */
	ret = class_create_file(attr_class, &class_attr_temperature);
	if (ret) {
		printk(KERN_ERR "mpu6050: failed to create sysfs class attribute temperature: %d\n", ret);
		return ret;
	}

	printk(KERN_INFO "mpu6050: sysfs class attributes created\n");
	printk(KERN_INFO "mpu6050: module loaded\n");
	return 0;
}

static void mpu6050_exit(void)
{
	if (attr_class) {
		class_remove_file(attr_class, &class_attr_accel_x);
		class_remove_file(attr_class, &class_attr_accel_y);
		class_remove_file(attr_class, &class_attr_accel_z);
		class_remove_file(attr_class, &class_attr_gyro_x);
		class_remove_file(attr_class, &class_attr_gyro_y);
		class_remove_file(attr_class, &class_attr_gyro_z);
		class_remove_file(attr_class, &class_attr_temperature);
		printk(KERN_INFO "mpu6050: sysfs class attributes removed\n");


		class_destroy(attr_class);
		printk(KERN_INFO "mpu6050: sysfs class destroyed\n");
	}
/*
	i2c_del_driver(&mpu6050_i2c_driver);
*/
	printk(KERN_INFO "mpu6050: i2c driver deleted\n");

	printk(KERN_INFO "mpu6050: module exited\n");
}


static int mpu6050_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct mpu6050_data *data;
	struct iio_dev *indio_dev;
	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(data));

	if (indio_dev == NULL)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->client = client;
	mutex_init(&data->mutex);
	i2c_set_clientdata(client, indio_dev);

	mpu6050_init();

	printk(KERN_INFO "mpu6050: i2c client address is 0x%X\n", client->addr);
	g_mpu6050_data.client = client;

	/* Read who_am_i register */
	ret = i2c_smbus_read_byte_data(client, REG_WHO_AM_I);
	if (IS_ERR_VALUE(ret)) {
		printk(KERN_ERR "mpu6050: i2c_smbus_read_byte_data() failed with error: %d\n", ret);
		return ret;
	}
	if (ret != MPU6050_WHO_AM_I) {
		printk(KERN_ERR "mpu6050: wrong i2c device found: expected 0x%X, found 0x%X\n", MPU6050_WHO_AM_I, ret);
		return -1;
	}
	printk(KERN_INFO "mpu6050: i2c mpu6050 device found, WHO_AM_I register value = 0x%X\n", ret);

	/* Setup the device */
	/* No error handling here! */
	i2c_smbus_write_byte_data(client, REG_CONFIG, 0);
	i2c_smbus_write_byte_data(client, REG_GYRO_CONFIG, 0);
	i2c_smbus_write_byte_data(client, REG_ACCEL_CONFIG, 0);
	i2c_smbus_write_byte_data(client, REG_FIFO_EN, 0);
	i2c_smbus_write_byte_data(client, REG_INT_PIN_CFG, 0);
	i2c_smbus_write_byte_data(client, REG_INT_ENABLE, 0);
	i2c_smbus_write_byte_data(client, REG_USER_CTRL, 0);
	i2c_smbus_write_byte_data(client, REG_PWR_MGMT_1, 0);
	i2c_smbus_write_byte_data(client, REG_PWR_MGMT_2, 0);

	printk(KERN_INFO "mpu6050: i2c driver probed\n");
	return 0;
}

static int mpu6050_remove(struct i2c_client *client)
{
	g_mpu6050_data.client = 0;

	mpu6050_exit();
	printk(KERN_INFO "mpu6050: i2c driver removed\n");
	return 0;
}


static int mpu6050_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = (struct iio_dev *)i2c_get_clientdata(to_i2c_client(dev));
	struct mpu6050_data *data = iio_priv(indio_dev);
	int ret = 0;

	mutex_lock(&data->mutex);
		/* TODO: set sleep mode */
	mutex_unlock(&data->mutex);
	return ret;
}

static int mpu6050_resume(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct mpu6050_data *data = iio_priv(indio_dev);
	int ret = 0;

	mutex_lock(&data->mutex);
		/* TODO: start sensor */
	mutex_unlock(&data->mutex);

	return ret;
}

static const struct dev_pm_ops mpu6050_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mpu6050_suspend, mpu6050_resume)
};

static const struct acpi_device_id mpu6050_acpi_match[] = {
	{"MPU6050", 0},
	{},
};
MODULE_DEVICE_TABLE(acpi, mpu6050_acpi_match);

static const struct of_device_id mpu6050_of_match[] = {
	{ .compatible = "gl,mpu6050", },
	{}
};
MODULE_DEVICE_TABLE(of, mpu6050_of_match);

static const struct i2c_device_id mpu6050_id[] = {
	{"mpu6050", 0},
	{ },
};

MODULE_DEVICE_TABLE(i2c, mpu6050_id);

static struct i2c_driver mpu6050_driver = {
	.driver = {
		.name = "mpu6050",
		.acpi_match_table = ACPI_PTR(mpu6050_acpi_match),
		.of_match_table = of_match_ptr(mpu6050_of_match),
		.pm = &mpu6050_pm_ops,
	},
	.probe = mpu6050_probe,
	.remove = mpu6050_remove,
	.id_table = mpu6050_id,
};

module_i2c_driver(mpu6050_driver);


MODULE_AUTHOR("Andriy.Khulap <andriy.khulap@globallogic.com>");
MODULE_AUTHOR("Yevgen Kovalyov<yevgen.kovalyov@globallogic.com>");
MODULE_DESCRIPTION("mpu6050 I2C acc&gyro");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.2");
