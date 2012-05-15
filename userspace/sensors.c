/*
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version. see <http://www.gnu.org/licenses/>
 *
 * sensors.c
 *
 */
#include "multiwii.h"
#include "util.h"

extern uint8_t calibratingM;
extern uint16_t calibratingA;
extern uint16_t calibratingG;

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
//	i2c_getSixRawADC(0 | 0);
	GYRO_Common();
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
				config_save(); // write accZero in EEPROM
			}
			calibratingA--;
		}

		DEBUG("calibratingA done")
		accADC[ROLL] -= cfg.accZero[ROLL];
		accADC[PITCH] -= cfg.accZero[PITCH];
		accADC[YAW] -= cfg.accZero[YAW];
}
void sensors_acc_read() {
		//i2c_getSixRawADC(0, 0);
		ACC_Common();
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

	//Device_Mag_getADC();
	//TODO magADC[] -> input

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
				cfg.magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis])
				/ 2;
			config_save();
		}
	}
}



void sensors_gyro_init(void);
void sensors_baro_init(void);
void sensors_mag_init(void);
void sensors_acc_init(void);

void sensors_gyro_init(){
	DEBUG("sensors_gyro_init")
}
void sensors_baro_init(){
	DEBUG("sensors_baro_init")
}
void sensors_mag_init(){
	DEBUG("sensors_mag_init")
}

void sensors_acc_init(){
	DEBUG("sensors_acc_init")
	acc_25deg = acc_1G * 0.423;
}

void sensors_sonar_init(){}


void sensors_init() {
	if (cfg.GYRO) sensors_gyro_init();
	if (cfg.BARO) sensors_baro_init();
	if (cfg.MAG) sensors_mag_init();
	if (cfg.ACC) sensors_acc_init();
	if (cfg.SONAR) sensors_sonar_init();
}
