/*
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version. see <http://www.gnu.org/licenses/>
 *
 * Char device wrapper for the mi2c-i2c kernel module example.
 *
 */
#ifndef MI2C_H
#define MI2C_H

#define DRIVER_NAME "mi2c"

#define I2C_BUS_3	3 //	H9 pin 19 20
#define I2C_BUS_2	2 //	H9 pin 17 18

#define MAX_MI2C_DEVICES 80

#define ARDUINO_DEVICE	"arduino"
#define ARDUINO_ADDRESS		0x10

#define ITG3200_DEVICE	"itg3200"
#define ITG3200_ADDRESS		0x69

#endif

