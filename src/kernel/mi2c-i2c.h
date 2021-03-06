/*
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version. see <http://www.gnu.org/licenses/>
 *
 * I2C functions for the mi2c driver
 */

#ifndef MI2C_I2C_H
#define MI2C_I2C_H

int mi2c_i2c_get_address(unsigned int device_id);

int mi2c_i2c_write(unsigned int device_id, unsigned char *buf, int count);
int mi2c_i2c_write_reg(unsigned int device_id, unsigned char reg,unsigned char buf);


int mi2c_i2c_read(unsigned int device_id, unsigned char *buf, int count);
int mi2c_i2c_read_reg(unsigned int device_id, unsigned char reg, unsigned char *val);
int mi2c_i2c_read_regs(unsigned int device_id,const int reg ,const int count, int8_t *data);

int mi2c_init_i2c(int num_devices,struct i2c_board_info * board_info);


void mi2c_cleanup_i2c(void);

#endif

