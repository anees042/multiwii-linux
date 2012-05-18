/*
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version. see <http://www.gnu.org/licenses/>
 *
 * sensors.c
 *
 */

#include "../kernel/mi2c-lib.h"
#include "multiwii.h"
#include "util.h"
//      #include <sys/types.h>
       #include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

//typedef enum SensorsType {
#define	DEVICE_FD 	"/dev/mi2c"

//
//} SensorsType;

// TODO remove after testing
extern int16_t rcData[8];
extern uint8_t armed;

extern uint8_t calibratingM;
extern uint16_t calibratingA;
extern uint16_t calibratingG;

static uint32_t fd_device;

int sensor_read_rawADC(uint8_t sensor, int16_t * raw) {

	uint8_t data[6] = { 0 };

	if (fd_device <= 0) {
		return -1;
	}

	int k = write(fd_device, &sensor, 1);
	if ((k != 1)) {
		printf("error sending sensor req to device : %s (%d)\n",
				strerror(errno), errno);
		return -1;
	}
	k = read(fd_device, data, 6);

	if ((k != 6)) {
		printf("k = %i", k);
		printf("error: %s (%d)\n", strerror(errno), errno);
	} else {

		raw[0] = data[1] & 0xff;
		raw[0] = raw[0] + (data[0] << 8);
		raw[0] = raw[0] / 4;
		raw[1] = data[3] & 0xff;
		raw[1] = raw[1] + (data[2] << 8);
		raw[1] = raw[1] / 4;
		raw[2] = data[5] & 0xff;
		raw[2] = raw[2] + (data[4] << 8);
		raw[2] = raw[2] / 4;

	}

//	printf("raw[0] =%i\n", raw[0]);
//	printf("raw[1] =%i\n", raw[1]);
//	printf("raw[2] =%i\n", raw[2]);
return 1;
}

// ****************
//      GYRO
// ****************

void GYRO_Common() {
	DEBUG("GYRO_Common")
			static int16_t previousGyroADC[3] = { 0, 0, 0 };
	static int32_t g[3];
	uint8_t axis;

#if defined MMGYRO
	// Moving Average Gyros by Magnetron1
	//---------------------------------------------------
	static int16_t mediaMobileGyroADC[3][MMGYROVECTORLENGHT];
	static int32_t mediaMobileGyroADCSum[3];
	static uint8_t mediaMobileGyroIDX;
	//---------------------------------------------------
#endif

	if (calibratingG > 0) {
		for (axis = 0; axis < 3; axis++) {
			// Reset g[axis] at start of calibration
			if (calibratingG == 400)
				g[axis] = 0;
			// Sum up 400 readings
			g[axis] += gyroADC[axis];
			// Clear global variables for next reading
			gyroADC[axis] = 0;
			gyroZero[axis] = 0;
			if (calibratingG == 1) {
				gyroZero[axis] = g[axis] / 400;
				delay(650);
			}
		}
		calibratingG--;
	} else {
		// TODO remove after testing - for armed and throttle
		armed = 1;
		rcData[THROTTLE] = 1500;
		printf("rcData[THROTTLE] = %i\n", rcData[THROTTLE]);

	}

#ifdef MMGYRO
	mediaMobileGyroIDX = ++mediaMobileGyroIDX % MMGYROVECTORLENGHT;
	for (axis = 0; axis < 3; axis++) {
		gyroADC[axis] -= gyroZero[axis];
		mediaMobileGyroADCSum[axis] -= mediaMobileGyroADC[axis][mediaMobileGyroIDX];
		//anti gyro glitch, limit the variation between two consecutive readings
		mediaMobileGyroADC[axis][mediaMobileGyroIDX] = constrain(gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800);
		mediaMobileGyroADCSum[axis] += mediaMobileGyroADC[axis][mediaMobileGyroIDX];
		gyroADC[axis] = mediaMobileGyroADCSum[axis] / MMGYROVECTORLENGHT;
#else
		for (axis = 0; axis < 3; axis++) {
			gyroADC[axis] -= gyroZero[axis];
			//anti gyro glitch, limit the variation between two consecutive readings
			gyroADC[axis] = constrain(gyroADC[axis], previousGyroADC[axis] - 800,
					previousGyroADC[axis] + 800);
#endif
			previousGyroADC[axis] = gyroADC[axis];
		}
	}

	void sensors_gyro_read() {
		if (sensor_read_rawADC(SENSOR_GYRO, gyroADC) == -1) {
			GYRO_Common();
		} else {

		}
		DEBUG("sensors_gyro_read done")
	}

	// ****************
	//     ACC
	// ****************
	void ACC_Common() {
		DEBUG("ACC_Common")
			static int32_t a[3];

		if (calibratingA > 0) {
			for (uint8_t axis = 0; axis < 3; axis++) {
				// Reset a[axis] at start of calibration
				if (calibratingA == 400)
					a[axis] = 0;
				// Sum up 400 readings
				a[axis] += accADC[axis];
				// Clear global variables for next reading
				accADC[axis] = 0;
				cfg.accZero[axis] = 0;
			}
			DEBUG("calibratingA middle")
			// Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
			if (calibratingA == 1) {
				cfg.accZero[ROLL] = a[ROLL] / 400;
				cfg.accZero[PITCH] = a[PITCH] / 400;
				cfg.accZero[YAW] = a[YAW] / 400 - acc_1G; // for nunchuk 200=1G
				cfg.accTrim[ROLL] = 0;
				cfg.accTrim[PITCH] = 0;
				// TODO save only changed parameter
				//config_save(); // write accZero in EEPROM
			}
			calibratingA--;
		}

		DEBUG("calibratingA done")
		accADC[ROLL] -= cfg.accZero[ROLL];
		accADC[PITCH] -= cfg.accZero[PITCH];
		accADC[YAW] -= cfg.accZero[YAW];
	}

	void sensors_acc_read() {

		if (sensor_read_rawADC(SENSOR_ACC, accADC) == -1) {

		} else {
			ACC_Common();
		}
		DEBUG("sensors_acc_read done")
	}

	void sensors_baro_read() {
		DEBUG("sensors_baro_read")
	}


	#define MAG_UPDATE_INTERVAL 100000

	void sensors_mag_read() {
		DEBUG("sensors_mag_read")

		static uint32_t t, tCal = 0;
		static int16_t magZeroTempMin[3];
		static int16_t magZeroTempMax[3];
		uint8_t axis;

		if (currentTime < t)
			return; //each read is spaced by 100ms
		t = currentTime + MAG_UPDATE_INTERVAL;


		sensor_read_rawADC(SENSOR_MAG, magADC);

		if (calibratingM == 1) {
			tCal = t;
			for (axis = 0; axis < 3; axis++) {
				cfg.magZero[axis] = 0;
				magZeroTempMin[axis] = magADC[axis];
				magZeroTempMax[axis] = magADC[axis];
			}
			calibratingM = 0;
		}

		magADC[ROLL] -= cfg.magZero[ROLL];
		magADC[PITCH] -= cfg.magZero[PITCH];
		magADC[YAW] -= cfg.magZero[YAW];

		if (tCal != 0) {
			if ((t - tCal) < 30000000) { // 30s: you have 30s to turn the multi in all directions
				LEDPIN_TOGGLE;
				for (axis = 0; axis < 3; axis++) {
					if (magADC[axis] < magZeroTempMin[axis])
						magZeroTempMin[axis] = magADC[axis];
					if (magADC[axis] > magZeroTempMax[axis])
						magZeroTempMax[axis] = magADC[axis];
				}
			} else {
				tCal = 0;
				for (axis = 0; axis < 3; axis++)
					cfg.magZero[axis] =
							(magZeroTempMin[axis] + magZeroTempMax[axis]) / 2;
				// TODO save only changed parameter
				//config_save();
			}
		}
	}

	int8_t sensors_gyro_init(void);
	int8_t sensors_baro_init(void);
	int8_t sensors_mag_init(void);
	int8_t sensors_acc_init(void);

	int8_t sensors_gyro_init() {
		DEBUG("sensors_gyro_init")
			return 1;
	}
	int8_t sensors_baro_init() {
		DEBUG("sensors_baro_init")
			return 1;
	}
	int8_t sensors_mag_init() {
		DEBUG("sensors_mag_init")
			return 1;
	}

	int8_t sensors_acc_init() {
		DEBUG("sensors_acc_init")
			acc_25deg = acc_1G * 0.423;
		return 1;
	}

	int8_t sensors_sonar_init() {
		DEBUG("sensors_sonar_init")
			return 1;
	}

	int8_t sensors_init() {

		DEBUG("sensors_gyro_init")

			char filename[40];

		sprintf(filename, DEVICE_FD);
		if ((fd_device = open(filename, O_RDONLY)) < 0) {
			printf("Failed to open the sensors %s ", filename);
			printf("error: %s (%d)\n", strerror(errno), errno);
			return (-1);
		}

		if (cfg.GYRO)
			if (sensors_gyro_init() == -1)
				return -1;
		if (cfg.BARO)
			sensors_baro_init();
		if (cfg.MAG)
			sensors_mag_init();
		if (cfg.ACC)
			sensors_acc_init();
		if (cfg.SONAR)
			sensors_sonar_init();

		return 1;
	}
