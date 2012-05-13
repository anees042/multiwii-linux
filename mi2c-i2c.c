/*
 * An example of how to implement an i2c Linux driver as a loadable module with
 * dynamic device registration. 
 *
 * This driver was meant to be included as a module in a bigger driver which
 * is most likely why you would want to use a kernel i2c solution.
 *
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/module.h>

#include "mi2c.h"


static int  num_devices;

static struct i2c_client *mi2c_i2c_client[MAX_MI2C_DEVICES];


int mi2c_i2c_get_address(unsigned int device_id) {
	if (device_id >= num_devices)
		return -EINVAL;

	if (!mi2c_i2c_client[device_id])
		return -ENODEV;

	return mi2c_i2c_client[device_id]->addr;
}

int mi2c_i2c_write(unsigned int device_id, unsigned char *buf, int count) {
	if (device_id >= num_devices)
		return -EINVAL;

	if (!mi2c_i2c_client[device_id])
		return -ENODEV;

	return i2c_master_send(mi2c_i2c_client[device_id], buf, count);
}

int mi2c_i2c_write_reg(unsigned int device_id, unsigned char reg,
		unsigned char val) {
	unsigned char cmd[1];
	int i = 0;
	cmd[0] = reg;
	i = mi2c_i2c_write(device_id, cmd, 1);
	cmd[0] = val;
	i += mi2c_i2c_write(device_id, cmd, 1);
	if (i !=2 )
				dev_err(&(mi2c_i2c_client[device_id]->dev), "I2C WRITE error\n");
	return i == 2;
}

int mi2c_i2c_read(unsigned int device_id, unsigned char *buf, int count) {
	if (device_id >= num_devices)
		return -EINVAL;

	if (!mi2c_i2c_client[device_id])
		return -ENODEV;

	return i2c_master_recv(mi2c_i2c_client[device_id], buf, count);
}

int mi2c_i2c_read_reg(unsigned int device_id, unsigned char reg,
		unsigned char *val) {
	unsigned char cmd[1];
	int i = 0;
	cmd[0] = reg;
	i = mi2c_i2c_write(device_id, cmd, 1);
	i += mi2c_i2c_read(device_id, val, 1);
	if (i !=2 )
			dev_err(&(mi2c_i2c_client[device_id]->dev), "I2C READ error\n");
	return i == 2;
}

int mi2c_i2c_reads(unsigned int device_id,int reg,int count, int8_t *data)
{

	int r=0;

	r = i2c_smbus_read_i2c_block_data(mi2c_i2c_client[device_id],
			reg,count, data);
	if (r != count)
		dev_err(&(mi2c_i2c_client[device_id]->dev), "I2C READ error\n");
	return r;

}

static int __init
mi2c_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
	printk(KERN_INFO "%s driver registered for device at address 0x%02x\n",
			client->name, client->addr);
	return 0;
}

static int __exit
mi2c_i2c_remove(struct i2c_client *client) {
	printk(KERN_INFO "removing %s driver for device at address 0x%02x\n",
			client->name, client->addr);

	return 0;
}


/* i2 devices used by the driver   */
static const struct i2c_device_id mi2c_id[] = {
		{ ITG3200_DEVICE, 0 },
		{ARDUINO_DEVICE, 0 },
		{ }, };

MODULE_DEVICE_TABLE( i2c, mi2c_id);

static struct i2c_driver mi2c_i2c_driver __refdata = {
		.driver = {
				.name = DRIVER_NAME,
				.owner = THIS_MODULE,
		},
		.id_table = mi2c_id,
		.probe = mi2c_i2c_probe,
		.remove = mi2c_i2c_remove,
};


// init i2c devices
int __init mi2c_init_i2c(int numdevices, struct i2c_board_info * board_info )
{
	int i, ret;
	struct i2c_adapter *adapter;


	num_devices =numdevices;

	/* register our driver */
	ret = i2c_add_driver(&mi2c_i2c_driver);
	if (ret) {
		printk(KERN_ALERT "Error registering i2c driver\n");
		return ret;
	}


	/* get bus */
	adapter = i2c_get_adapter(I2C_BUS_2);

	if (!adapter) {
		printk(KERN_ALERT "i2c_get_adapter(%d) failed\n", I2C_BUS_2);
		return -1;
	}

	/* add our devices */
	for (i = 0; i < num_devices; i++) {
		mi2c_i2c_client[i] = i2c_new_device(adapter,&board_info[i]);

		if (!mi2c_i2c_client[i]) {
			printk(KERN_ALERT "i2c_new_device failed\n");
			break;
		} else {

		}
	}

	i2c_put_adapter(adapter);

	return i == num_devices ? 0 : -1;
}

void __exit mi2c_cleanup_i2c(void)
{
	int i;

	for (i = 0; i < num_devices; i++) {
		if (mi2c_i2c_client[i]) {
			i2c_unregister_device(mi2c_i2c_client[i]);
			mi2c_i2c_client[i] = NULL;
		}
	}

	i2c_del_driver(&mi2c_i2c_driver);
}

