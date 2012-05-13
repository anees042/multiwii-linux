/*
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version. see <http://www.gnu.org/licenses/>
 *
 * Char device wrapper for the mi2c-i2c kernel module example.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>

#include "mi2c.h"
#include "mi2c-i2c.h"

#include "sensors.h"

#define USER_BUFF_SIZE 128



#define NUM_DEVICES 2


struct i2c_board_info mi2c_board_info[NUM_DEVICES] = {

		{ I2C_BOARD_INFO(ITG3200_DEVICE, ITG3200_ADDRESS), }
		,
		{ I2C_BOARD_INFO(ARDUINO_DEVICE, ARDUINO_ADDRESS), }
};

struct mi2c_dev {
	dev_t devt;
	struct cdev cdev;
	struct semaphore sem;
	struct class *class;
	char *user_buff;
};

static struct mi2c_dev mi2c_dev;


/* 
 * See the arduino_i2c_slave.pde under the arduino_i2c_slave directory
 * for the arduino code. Basically, the arduino is running a simple program
 * where he responds to 1 byte commands with a two-byte response.
 */
static int arduino_run_command(unsigned char cmd, unsigned int *val) {
	int result;
	unsigned char buff[2];

	buff[0] = cmd;

	// send cmd
	result = mi2c_i2c_write(ARDUINO_I2C, buff, 1);

	if (result != 1)
		return result;

	buff[0] = 0;
	buff[1] = 0;

	// read reply
	result = mi2c_i2c_read(ARDUINO_I2C, buff, 2);

	if (result != 2)
		return result;

	*val = (buff[0] << 8) | buff[1];

	return 0;
}

static int itg3200_init(void) {

	udelay(100);

	mi2c_i2c_write_reg(ITG3200_I2C, 0x3E, 0x80); //register: Power Management  --  value: reset device
	udelay(5);
	mi2c_i2c_write_reg(ITG3200_I2C, 0x15, ITG3200_SMPLRT_DIV); //register: Sample Rate Divider  -- default value = 0: OK
	udelay(5);
	mi2c_i2c_write_reg(ITG3200_I2C, 0x16, 0x18 + ITG3200_DLPF_CFG); //register: DLPF_CFG - low pass filter configuration
	udelay(5);
	mi2c_i2c_write_reg(ITG3200_I2C, 0x3E, 0x03); //register: Power Management  --  value: PLL with Z Gyro reference
	udelay(100);

	return 0;
}

static int itg3200_read_gyro(int16_t *temp, int16_t *gx, int16_t *gy, int16_t *gz) {

	int result;
	int8_t data[8];

	result = mi2c_i2c_reads(ITG3200_I2C,ITG3200_REG_TEMP_OUT_H, 8,data);

	if (result != 8)
		return -1;

	*temp = data[1]& 0xff;
	*temp = *temp + (data[0] << 8);
	*temp = *temp+13200;
	*temp = *temp /28;
	*temp = *temp +350;


	//*temp= 35.0 + ((float) ( ((data[1] & 0xff )+ (data[0] << 8 )) + 13200)) / 280;

	*gx = data[3]& 0xff;
	*gx = *gx+(data[2] << 8);
	*gx = *gx /4;
	*gy = data[5]& 0xff;
	*gy = *gy +(data[4] << 8);
	*gy = *gy /4;
	*gz = data[7]& 0xff;
	*gz = *gz+(data[6] << 8);
	*gz = *gz /4;
	return result;
}

static ssize_t mi2c_read(struct file *filp, char __user *buff,
		size_t count, loff_t *offp)
{
	ssize_t status;
	size_t len;
//	unsigned int addr, val;
//	unsigned char cmd;
	int16_t temp,gx,gy, gz;

	/* 
	 Generic user progs like cat will continue calling until we
	 return zero. So if *offp != 0, we know this is at least the
	 second call.
	 */
	//	if (*offp > 0)
	//		return 0;
	if (down_interruptible(&mi2c_dev.sem))
		return -ERESTARTSYS;

	memset(mi2c_dev.user_buff, 0, USER_BUFF_SIZE);
	len = 0;

	//	/* Arduino device  */
	//	addr = mi2c_i2c_get_address(ARDUINO_I2C);
	//	val = 0;
	//	cmd = 0x02;
	//
	//	if (arduino_run_command(cmd, &val) < 0)
	//		printk(KERN_ALERT "Read of Arduino at 0x%02X failed\n", addr);
	//	else
	//		len += sprintf(mi2c_dev.user_buff + len,
	//				"Arduino at 0x%02X responded to 0x%02X with 0x%04X\n",
	//				addr, cmd, val);

	/* read imu*/
	if (itg3200_read_gyro( &temp,&gx,&gy, &gz ) != 8){
		printk(KERN_ALERT "Read of itg3200 failed\n");
	}else{
		len += sprintf(mi2c_dev.user_buff + len,
				"temp=%i\t gx=%d\t gy=%d\t gz=%d\n",
				temp,gx,gy,gz);
	}


	len = strlen(mi2c_dev.user_buff);

	if (len > count)
		len = count;

	if (copy_to_user(buff, mi2c_dev.user_buff, len)) {
		status = -EFAULT;
		goto mi2c_read_done;
	}

	*offp += len;
	status = len;

	mi2c_read_done:

	up(&mi2c_dev.sem);

	return status;
}

static int mi2c_open(struct inode *inode, struct file *filp) {
	int status = 0;

	if (down_interruptible(&mi2c_dev.sem))
		return -ERESTARTSYS;

	if (!mi2c_dev.user_buff) {
		mi2c_dev.user_buff = kmalloc(USER_BUFF_SIZE, GFP_KERNEL);

		if (!mi2c_dev.user_buff) {
			printk(KERN_ALERT
					"mi2c_open: user_buff alloc failed\n");

			status = -ENOMEM;
		}
	}

	up(&mi2c_dev.sem);

	return status;
}

static const struct file_operations mi2c_fops = { .owner = THIS_MODULE, .open =
		mi2c_open, .read = mi2c_read, };

static int __init mi2c_init_cdev(void)
{
	int error;

	mi2c_dev.devt = MKDEV(0, 0);

	error = alloc_chrdev_region(&mi2c_dev.devt, 0, 1, DRIVER_NAME);
	if (error < 0) {
		printk(KERN_ALERT
				"alloc_chrdev_region() failed: error = %d \n",
				error);

		return -1;
	}

	cdev_init(&mi2c_dev.cdev, &mi2c_fops);
	mi2c_dev.cdev.owner = THIS_MODULE;

	error = cdev_add(&mi2c_dev.cdev, mi2c_dev.devt, 1);
	if (error) {
		printk(KERN_ALERT "cdev_add() failed: error = %d\n", error);
		unregister_chrdev_region(mi2c_dev.devt, 1);
		return -1;
	}

	return 0;
}

static int __init mi2c_init_class(void)
{
	mi2c_dev.class = class_create(THIS_MODULE, DRIVER_NAME);

	if (!mi2c_dev.class) {
		printk(KERN_ALERT "class_create(mi2c) failed\n");
		return -1;
	}

	if (!device_create(mi2c_dev.class, NULL, mi2c_dev.devt,
			NULL, DRIVER_NAME)) {
		class_destroy(mi2c_dev.class);
		return -1;
	}

	return 0;
}

static int __init mi2c_init(void)
{
	printk(KERN_INFO "mi2c_init()\n");

	memset(&mi2c_dev, 0, sizeof(struct mi2c_dev));

	sema_init(&mi2c_dev.sem, 1);

	if (mi2c_init_cdev() < 0)
		goto init_fail_1;

	if (mi2c_init_class() < 0)
		goto init_fail_2;

	if (mi2c_init_i2c(NUM_DEVICES,mi2c_board_info) < 0)
		goto init_fail_3;

	itg3200_init();

	return 0;

	init_fail_3:
	device_destroy(mi2c_dev.class, mi2c_dev.devt);
	class_destroy(mi2c_dev.class);

	init_fail_2:
	cdev_del(&mi2c_dev.cdev);
	unregister_chrdev_region(mi2c_dev.devt, 1);

	init_fail_1:

	return -1;
}
module_init( mi2c_init);

static void __exit mi2c_exit(void)
{
	printk(KERN_INFO "mi2c_exit()\n");

	mi2c_cleanup_i2c();

	device_destroy(mi2c_dev.class, mi2c_dev.devt);
	class_destroy(mi2c_dev.class);

	cdev_del(&mi2c_dev.cdev);
	unregister_chrdev_region(mi2c_dev.devt, 1);

	if (mi2c_dev.user_buff)
		kfree(mi2c_dev.user_buff);
}
module_exit( mi2c_exit);

MODULE_AUTHOR("Scott Ellis");
MODULE_AUTHOR("Trey Marc");
MODULE_DESCRIPTION("mi2c driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION("0.1.1");

