/*
 * http://www.kernel.org/doc/Documentation/i2c/writing-clients
 *
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

#include "mi2c-lib.h"
#include "mi2c.h"
#include "mi2c-i2c.h"


#include "sensors.h"

#define USER_BUFF_SIZE 128

#define NUM_DEVICES 2

struct i2c_board_info mi2c_board_info[NUM_DEVICES] = {

		{ I2C_BOARD_INFO(ITG3200_NAME, ITG3200_ADDRESS), },
		{ I2C_BOARD_INFO(BMA180_NAME, BMA180_ADDRESS), }
};

#define NUM_DEVICES2 1
struct i2c_board_info mi2c_board_info2[NUM_DEVICES2] = {

		{ I2C_BOARD_INFO(ARDUINO_NAME, ARDUINO_ADDRESS), }
};




struct mi2c_dev {
	dev_t devt;
	struct cdev cdev;
	struct semaphore sem;
	struct class *class;
	char *user_buff;
};

static struct mi2c_dev mi2c_dev;

#define BUFFER_LENGH  64

static char sensor_read[BUFFER_LENGH]={0};

//static float   magCal[3] = {1.0,1.0,1.0};

///*
// * See the arduino_i2c_slave.pde under the arduino_i2c_slave directory
// * for the arduino code. Basically, the arduino is running a simple program
// * where he responds to 1 byte commands with a two-byte response.
// */
//static int arduino_run_command(unsigned int *val) {
//	int result;
//	unsigned char buff[16] = { 250, 250, 250, 250, 250, 250, 250, 250, 250, 250,
//			250, 250, 250, 250, 250, 250 };
//
//	// send cmd
//	result = mi2c_i2c_write(ARDUINO, buff, 16);
//
//	if (result != 16)
//		return result;
//
//	buff[0] = 0;
//	buff[1] = 0;
//
//	// read reply
//	result = mi2c_i2c_read(ARDUINO, buff, 2);
//
//	if (result != 2)
//		return result;
//
//	*val = (buff[0] << 8) | buff[1];
//
//	return 0;
//}

//
//void getADC() {
//  i2c_getSixRawADC(HMC58XX,HMC58XX_DATA_REGISTER);
//  #if defined(HMC5843)
//    MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
//                     ((rawADC[2]<<8) | rawADC[3]) ,
//                     ((rawADC[4]<<8) | rawADC[5]) );
//  #endif
//  #if defined (HMC5883)
//    MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
//                     ((rawADC[4]<<8) | rawADC[5]) ,
//                     ((rawADC[2]<<8) | rawADC[3]) );
//  #endif
//}

static int hmc58xx_init(void) {

   udelay(100);
   // force positiveBias
   mi2c_i2c_write_reg(HMC58XX_ADDRESS ,0x00 ,0x71 ); //Configuration Register A  -- 0 11 100 01  num samples: 8 ; output rate: 15Hz ; positive bias
   udelay(50);
   // set gains for calibration
   mi2c_i2c_write_reg(HMC58XX_ADDRESS ,0x01 ,0x60 ); //Configuration Register B  -- 011 00000    configuration gain 2.5Ga
   mi2c_i2c_write_reg(HMC58XX_ADDRESS ,0x02 ,0x01 ); //Mode register             -- 000000 01    single Conversion Mode

   // read values from the compass -  self test operation
   // by placing the mode register into single-measurement mode (0x01), two data acquisition cycles will be made on each magnetic vector.
   // The first acquisition values will be subtracted from the second acquisition, and the net measurement will be placed into the data output registers
   udelay(100);

   //TODO
    // getADC();

   udelay(10);

   //TODO
//   #if defined(HMC5883)
//     magCal[ROLL]  =  1160.0 / abs(magADC[ROLL]);
//     magCal[PITCH] =  1160.0 / abs(magADC[PITCH]);
//     magCal[YAW]   =  1080.0 / abs(magADC[YAW]);
//   #else
//     magCal[ROLL]  =  1000.0 / abs(magADC[ROLL]);
//     magCal[PITCH] =  1000.0 / abs(magADC[PITCH]);
//     magCal[YAW]   =  1000.0 / abs(magADC[YAW]);
//   #endif
//     magInit = 1;

   // leave test mode
     mi2c_i2c_write_reg(HMC58XX_ADDRESS ,0x00 ,0x70 ); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
     mi2c_i2c_write_reg(HMC58XX_ADDRESS ,0x01 ,0x20 ); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
     mi2c_i2c_write_reg(HMC58XX_ADDRESS ,0x02 ,0x00 ); //Mode register             -- 000000 00    continuous Conversion Mode

return 1;
 }

static int bma180_init (void) {
	uint8_t control[1]={0};
	//printk(KERN_INFO "bma180_init 1");
	udelay(10);
	//default range 2G: 1G = 4096 unit.
	mi2c_i2c_write_reg(BMA180,0x0D,(1<<4)); // register: ctrl_reg0  -- value: set bit ee_w to 1 to enable writing
	udelay(5);
	mi2c_i2c_read_reg(BMA180, 0x20,control);
	control[0] = control[0] & 0x0F;        // save tcs register
	control[0] = control[0] | (0x01 << 4); // register: bw_tcs reg: bits 4-7 to set bw -- value: set low pass filter to 20Hz
	mi2c_i2c_write_reg(BMA180, 0x20, control[0]);
	udelay(5);

	mi2c_i2c_read_reg(BMA180, 0x30,control);
	control[0] = control[0] & 0xFC;        // save tco_z register
	control[0] = control[0] | 0x00;        // set mode_config to 0
	mi2c_i2c_write_reg(BMA180, 0x30, control[0]);
	udelay(5);
	mi2c_i2c_read_reg(BMA180, 0x35,control);
	control[0] = control[0] & 0xF1;        // save offset_x and smp_skip register
	control[0] = control[0] | (0x05 << 1); // set range to 8G
	mi2c_i2c_write_reg(BMA180, 0x35, control[0]);
	udelay(5);

//	// TODO acc_1G = 255;
	return 1;
}

static int mi2c_read_raw(uint8_t sensor,uint8_t reg,int8_t *data) {

	int result;
	result = mi2c_i2c_read_regs(sensor, reg, 6, data);

	if (result != 6){
		printk(KERN_ALERT "faile to read 6 raw : %d\n",result);
		return -1;
	}


	return result;
}

static int itg3200_init(void) {

	udelay(100);

	mi2c_i2c_write_reg(ITG3200, ITG3200_REG_POWER_MGMT, 0x80); // reset device
	udelay(5);
	mi2c_i2c_write_reg(ITG3200, ITG3200_REG_SAMPLE_RATE_DIV,
			ITG3200_SMPLRT_DIV); // set sample rate
	udelay(5);
	mi2c_i2c_write_reg(ITG3200, ITG3200_REG_LP_FULL_SCALE,
			0x18 + ITG3200_DLPF_CFG); //low pass filter configuration
	udelay(5);
	mi2c_i2c_write_reg(ITG3200, ITG3200_REG_POWER_MGMT, 0x03); //register: Power Management  --  value: PLL with Z Gyro reference
	udelay(100);

	return 1;
}


static ssize_t mi2c_read(struct file *filp, char __user *buff,
		size_t count, loff_t *offp)
{
	ssize_t status;
	size_t lenn;
	uint8_t read = 0;
	uint8_t data[6] = {0};

	/* 
	 Generic user progs like cat will continue calling until we
	 return zero. So if *offp != 0, we know this is at least the
	 second call.
	 */
//			if (*offp > 0)
//				return 0;
	if (down_interruptible(&mi2c_dev.sem))
		return -ERESTARTSYS;

	memset(mi2c_dev.user_buff, 0, USER_BUFF_SIZE);
	lenn = 0;


	switch (sensor_read[0]) {
	case SENSOR_GYRO:
		// int16 X, Y, Z
		read = mi2c_read_raw( ITG3200,ITG3200_REG_GYRO_XOUT_H, data );

		break;
	case SENSOR_ACC:
		read = mi2c_read_raw(BMA180,0x02, data );

		break;
	case SENSOR_MAG:

		break;
	case SENSOR_BARO:

		break;
	case SENSOR_SONAR:

		break;
	default:
		printk(KERN_ALERT "Unknown sensor_read : %d\n",sensor_read[0]);
		break;
	}


	if(read == 6) {

				lenn += sprintf(mi2c_dev.user_buff + lenn,
						"%01c%01c%01c%01c%01c%01c",
						data[0],data[1],data[2],data[3],data[4],data[5]);
			}


	if (copy_to_user(buff, mi2c_dev.user_buff, 6)) {
		status = -EFAULT;
		goto mi2c_read_done;
	}

	*offp += lenn;
	status = lenn;

	mi2c_read_done:

	up(&mi2c_dev.sem);

	return status;
}


/*  Called when a process writes to dev file: echo "hi" > /dev/hello */
static ssize_t mi2c_write(struct file *filp, const char *buffer, size_t length,loff_t *f_pos) {

	uint8_t i;

//#ifdef DEBUG
//	printk(KERN_INFO "device_write(%p,%s,%d)", file, buffer, length);
//#endif
	i =length ;
	if (i>BUFFER_LENGH){
		i = BUFFER_LENGH;
	}
	/* function to copy kernel space buffer to user space*/
	if ( copy_from_user(sensor_read,buffer,i) != 0 )
	printk( "kernel->userspace copy failed!\n" );

//	printk(KERN_INFO "device_write , sensor_read=%s", sensor_read);
	return strlen(sensor_read);

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
		mi2c_open, .read = mi2c_read, .write = mi2c_write };

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
	bma180_init();

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


MODULE_AUTHOR("Trey Marc");
MODULE_DESCRIPTION("multiwii i2c driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION("0.0.1");

