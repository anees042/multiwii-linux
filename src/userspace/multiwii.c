/*
 * MultiWiiCopter - Linux Fork
 *
 * http://multiwii.googlecode.com/svn/trunk/MultiWii - r759
 * http://afrodevices.googlecode.com/svn -r142
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version. see <http://www.gnu.org/licenses/>
 *
 *
 *
 */

#include "def.h"
#include "util.h"
#include "multiwii.h"
#include "output.h"
#include "imu.h"
#include "rc.h"
#include "serial.h"
#include "sensors.h"
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

//uint8_t mixerConfiguration;
uint8_t armed = 0;
int16_t axisPID[3];
int16_t heading;

int16_t rcData[8]; // interval [1000;2000]
int16_t rcCommand[4]; // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW

uint64_t currentTime = 0;
uint64_t previousTime = 0;

static uint16_t cycleTime = 0; // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
uint16_t calibratingA = 0; // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
uint8_t calibratingM = 0;
uint16_t calibratingG;

uint16_t acc_1G; // this is the 1G measured acceleration
int16_t acc_25deg;

static uint8_t accMode = 0; // if level mode is a activated
static uint8_t magMode = 0; // if compass heading hold is a activated
static uint8_t baroMode = 0; // if altitude hold is activated
static uint8_t headFreeMode = 0; // if head free mode is a activated
uint8_t passThruMode = 0; // if passthrough mode is activated
static int16_t headFreeModeHold;

static int16_t accTrim[2] = { 0, 0 };
static int16_t magHold;
static uint8_t calibratedACC = 0;

static uint8_t okToArm = 0;
static uint8_t rcOptions[CHECKBOXITEMS];

// absolute pressure sensor
static int32_t BaroAlt;
static int32_t EstAlt; // in cm
static int16_t BaroPID = 0;
static int32_t AltHold;
static int16_t errorAltitudeI = 0;

// sonar
//static int16_t  sonarAlt; //to think about the unit

//for log
static uint16_t cycleTimeMax = 0; // highest ever cycle timen
static uint16_t cycleTimeMin = 65535; // lowest ever cycle timen
static uint32_t armedTime = 0;
static int32_t BAROaltStart = 0; // offset value from powerup
static int32_t BAROaltMax = 0; // maximum value

volatile int16_t failsafeCnt = 0;
static int16_t failsafeEvents = 0;

 int16_t lookupPitchRollRC[6]; // lookup table for expo & RC rate PITCH+ROLL
 int16_t lookupThrottleRC[11]; // lookup table for expo & mid THROTTLE

// **********************
// EEPROM & LCD functions
// **********************
//static uint8_t P8[8], I8[8], D8[8]; // 8 bits is much faster and the code is much shorter
static uint8_t dynP8[3], dynD8[3];
static uint8_t rollPitchRate;
static uint8_t yawRate;
static uint8_t dynThrPID;
static uint16_t activate[CHECKBOXITEMS];

void annexCode() { // this code is excetuted at each loop and won't interfere with control loop if it lasts less than 650 microseconds
	static uint32_t calibratedAccTime;
	uint16_t tmp, tmp2;

	uint8_t axis, prop1, prop2;

#define BREAKPOINT 1500
	// PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
	if (rcData[THROTTLE] < BREAKPOINT) {
		prop2 = 100;
	} else {
		if (rcData[THROTTLE] < 2000) {
			prop2 = 100
					- (uint16_t) dynThrPID * (rcData[THROTTLE] - BREAKPOINT)
					/ (2000 - BREAKPOINT);
		} else {
			prop2 = 100 - dynThrPID;
		}
	}

	for (axis = 0; axis < 3; axis++) {
		tmp = min(abs(rcData[axis]-MIDRC),500);
#if defined(DEADBAND)
		if (tmp>DEADBAND) {tmp -= DEADBAND;}
		else {tmp=0;}
#endif
		if (axis != 2) { //ROLL & PITCH
			tmp2 = tmp / 100;
			rcCommand[axis] = lookupPitchRollRC[tmp2]
			                                    + (tmp - tmp2 * 100)
			                                    * (lookupPitchRollRC[tmp2 + 1]
			                                                         - lookupPitchRollRC[tmp2]) / 100;
			prop1 = 100 - (uint16_t) rollPitchRate * tmp / 500;
			prop1 = (uint16_t) prop1 * prop2 / 100;
		} else { // YAW
			rcCommand[axis] = tmp;
			prop1 = 100 - (uint16_t) yawRate * tmp / 500;
		}
		dynP8[axis] = (uint16_t) cfg.P8[axis] * prop1 / 100;
		dynD8[axis] = (uint16_t) cfg.D8[axis] * prop1 / 100;
		if (rcData[axis] < MIDRC)
			rcCommand[axis] = -rcCommand[axis];
	}
	tmp = constrain(rcData[THROTTLE],MINCHECK,2000);
	tmp = (uint32_t) (tmp - MINCHECK) * 1000 / (2000 - MINCHECK); // [MINCHECK;2000] -> [0;1000]
	tmp2 = tmp / 100;
	rcCommand[THROTTLE] = lookupThrottleRC[tmp2]
	                                       + (tmp - tmp2 * 100)
	                                       * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2])
	                                       / 100; // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]

	if (headFreeMode) { //to optimize
		float radDiff = (heading - headFreeModeHold) * 0.0174533f; // where PI/180 ~= 0.0174533
		float cosDiff = cos(radDiff);
		float sinDiff = sin(radDiff);
		int16_t rcCommand_PITCH = rcCommand[PITCH] * cosDiff
				+ rcCommand[ROLL] * sinDiff;
		rcCommand[ROLL] = rcCommand[ROLL] * cosDiff
				- rcCommand[PITCH] * sinDiff;
		rcCommand[PITCH] = rcCommand_PITCH;
	}

	if ((calibratingA > 0 && (cfg.ACC)) || (calibratingG > 0)) { // Calibration phasis
		LEDPIN_TOGGLE;
	} else {
		if (calibratedACC == 1) {
			LEDPIN_OFF;
		}
		if (armed) {
			LEDPIN_ON;
		}
	}

	if (currentTime > calibratedAccTime) {
		if (smallAngle25 == 0) {
			calibratedACC = 0; // the multi uses ACC and is not calibrated or is too much inclinated
			LEDPIN_TOGGLE;
			calibratedAccTime = currentTime + 500000;
		} else
			calibratedACC = 1;
	}

	serial_com();

#if defined(LOG_VALUES) && (LOG_VALUES == 2)
	if (cycleTime > cycleTimeMax) cycleTimeMax = cycleTime; // remember highscore
	if (cycleTime < cycleTimeMin) cycleTimeMin = cycleTime;// remember lowscore
#endif

#ifdef LOG_VALUES
	if (armed) armedTime += (uint32_t)cycleTime;
	if (cfg.BARO) {
		if (armed == 0) {
			BAROaltStart = BaroAlt;
			BAROaltMax = BaroAlt;
		} else {
			if (BaroAlt > BAROaltMax) BAROaltMax = BaroAlt;
		}
	}
#endif
}

int8_t setup() {
	previousTime = micros();
	output_init();
	config_init();
	rc_init();


#if defined(GIMBAL)
	calibratingA = 400;
#endif
	calibratingG = 400;

	// TODO pass cfg and calibrating to imu_init
	if (imu_init() == -1){
		return FAILURE_IMU_INIT;
	}

	return 1;
}

// ******** Main Loop *********
void loop() {
	uint8_t axis, i;
	int16_t error, errorAngle;
	int16_t delta, deltaSum;
	int16_t PTerm, ITerm, DTerm;
	static int16_t lastGyro[3] = { 0, 0, 0 };
	static int16_t delta1[3], delta2[3];
	static int16_t errorGyroI[3] = { 0, 0, 0 };
	static int16_t errorAngleI[2] = { 0, 0 };
	static uint32_t rcTime = 0;
	static int16_t initialThrottleHold;

	/*
	 * 50Hz RC loop
	 */
	if (currentTime > rcTime) {

		rcTime = currentTime + 20000;

		rc_read();

		if (cfg.FAILSAFE) { //Failsafe routine - added by MIS
			if (failsafeCnt > (5 * FAILSAVE_DELAY) && armed == 1) {
				for (i = 0; i < 3; i++)
					rcData[i] = MIDRC;
				rcData[THROTTLE] = FAILSAVE_THR0TTLE;
				if (failsafeCnt > 5 * (FAILSAVE_DELAY + FAILSAVE_OFF_DELAY)) {
					armed = 0; // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
					okToArm = 0; // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
				}
				failsafeEvents++;
			}
			failsafeCnt++;
		}

		for (i = 0; i < CHECKBOXITEMS; i++) {
			rcOptions[i] = (((rcData[AUX1] < 1300)
					| (1300 < rcData[AUX1] && rcData[AUX1] < 1700) << 1
					| (rcData[AUX1] > 1700) << 2 | (rcData[AUX2] < 1300) << 3
					| (1300 < rcData[AUX2] && rcData[AUX2] < 1700) << 4
					| (rcData[AUX2] > 1700) << 5 | (rcData[AUX3] < 1300) << 6
					| (1300 < rcData[AUX3] && rcData[AUX3] < 1700) << 7
					| (rcData[AUX3] > 1700) << 8 | (rcData[AUX4] < 1300) << 9
					| (1300 < rcData[AUX4] && rcData[AUX4] < 1700) << 10
					| (rcData[AUX4] > 1700) << 11) & activate[i]) > 0;
		}

		// note: if FAILSAFE is disable, failsafeCnt > 5*FAILSAVE_DELAY is always false
		// TODO this belong to rc_input
		if ((rcOptions[BOXACC] || (failsafeCnt > 5 * FAILSAVE_DELAY))
				&& (cfg.ACC)) {
			// bumpless transfer to Level mode
			if (!accMode) {
				errorAngleI[ROLL] = 0;
				errorAngleI[PITCH] = 0;
				accMode = 1;
			}
		} else
			accMode = 0; // failsave support

		if (rcOptions[BOXARM] == 0)
			okToArm = 1;
		if (accMode == 1) {
			STABLEPIN_ON;
		} else {
			STABLEPIN_OFF;
		}

		if (cfg.BARO) {
			if (rcOptions[BOXBARO]) {
				if (baroMode == 0) {
					baroMode = 1;
					AltHold = EstAlt;
					initialThrottleHold = rcCommand[THROTTLE];
					errorAltitudeI = 0;
					BaroPID=0;
				}
			} else baroMode = 0;
		}
		if (cfg.MAG) {
			if (rcOptions[BOXMAG]) {
				if (magMode == 0) {
					magMode = 1;
					magHold = heading;
				}
			} else
				magMode = 0;
			if (rcOptions[BOXHEADFREE]) {
				if (headFreeMode == 0) {
					headFreeMode = 1;
				}
			} else
				headFreeMode = 0;
		}

		if (rcOptions[BOXPASSTHRU]) {
			passThruMode = 1;
		} else {
			passThruMode = 0;
		}

	} else {

		// not in rc loop

		static uint8_t taskOrder = 0; // never call all functions in the same loop, to avoid high delay spikes
		switch (taskOrder++ % 3) {
		case 0:
			if (cfg.MAG) {
				sensors_mag_read();
			}
			break;
		case 1:
			if (cfg.BARO) {
				sensors_baro_read();
			}
			break;
		case 2:
			if (cfg.BARO) {
				imu_baro_compute();
			}
			break;

		}
	}

	imu_compute();

	// Measure loop rate just afer reading the sensors
	currentTime = micros();
	cycleTime = currentTime - previousTime;
	previousTime = currentTime;

	if (cfg.MAG) {
		if (abs(rcCommand[YAW]) < 70 && magMode) {
			int16_t dif = heading - magHold;
			if (dif <= -180)
				dif += 360;
			if (dif >= +180)
				dif -= 360;
			if (smallAngle25)
				rcCommand[YAW] -= dif * cfg.P8[PIDMAG] / 30; // 18 deg
		} else {
			magHold = heading;
		}
		DEBUG("Mag end")
	}

	if (cfg.BARO) {
		if (baroMode) {
			if (abs(rcCommand[THROTTLE]-initialThrottleHold)>20) {
				baroMode = 0; // so that a new althold reference is defined
			}
			rcCommand[THROTTLE] = initialThrottleHold + BaroPID;
		}
		DEBUG("Baro end")
	}

	DEBUG("PID Begin")
	//**** PITCH & ROLL & YAW PID ****
	for (axis = 0; axis < 3; axis++) {
		if ((accMode == 1) && (axis < 2)) { //LEVEL MODE
			// 50 degrees max inclination
			// errorAngle = constrain(2*rcCommand[axis] - GPS_angle[axis],-500,+500) - angle[axis] + accTrim[axis]; //16 bits is ok here
			errorAngle = constrain(2*rcCommand[axis] ,-500,+500 ) - angle[axis]
			                                                              + accTrim[axis]; //16 bits is ok here
			//#ifdef LEVEL_PDF
			//			PTerm      = -(int32_t)angle[axis]*P8[PIDLEVEL]/100 ;
			//#else
			PTerm = (int32_t) errorAngle * cfg.P8[PIDLEVEL] / 100; // 32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
			//#endif
			PTerm = constrain(PTerm,-cfg.D8[PIDLEVEL]*5,+cfg.D8[PIDLEVEL]*5);

			errorAngleI[axis] =
					constrain(errorAngleI[axis]+errorAngle,-10000,+10000); // WindUp     //16 bits is ok here
			ITerm = ((int32_t) errorAngleI[axis] * cfg.I8[PIDLEVEL]) >> 12; // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result
		} else { //ACRO MODE or YAW axis
			if (abs(rcCommand[axis]) < 350)
				error = rcCommand[axis] * 10 * 8 / cfg.P8[axis]; // 16 bits is needed for calculation: 350*10*8 = 28000      16 bits is ok for result if P8>2 (P>0.2)
			else
				error = (int32_t) rcCommand[axis] * 10 * 8 / cfg.P8[axis]; // 32 bits is needed for calculation: 500*5*10*8 = 200000   16 bits is ok for result if P8>2 (P>0.2)
			error -= gyroData[axis];

			PTerm = rcCommand[axis];

			errorGyroI[axis] = constrain(errorGyroI[axis]+error,-16000,+16000); // WindUp   16 bits is ok here
			if (abs(gyroData[axis]) > 640)
				errorGyroI[axis] = 0;
			ITerm = (errorGyroI[axis] / 125 * cfg.I8[axis]) >> 6; // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000
		}
		if (abs(gyroData[axis]) < 160)
			PTerm -= gyroData[axis] * dynP8[axis] / 10 / 8; // 16 bits is needed for calculation   160*200 = 32000         16 bits is ok for result
		else
			PTerm -= (int32_t) gyroData[axis] * dynP8[axis] / 10 / 8; // 32 bits is needed for calculation

		delta = gyroData[axis] - lastGyro[axis]; // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
		lastGyro[axis] = gyroData[axis];
		deltaSum = delta1[axis] + delta2[axis] + delta;
		delta2[axis] = delta1[axis];
		delta1[axis] = delta;

		if (abs(deltaSum) < 640)
			DTerm = (deltaSum * dynD8[axis]) >> 5; // 16 bits is needed for calculation 640*50 = 32000           16 bits is ok for result
		else
			DTerm = ((int32_t) deltaSum * dynD8[axis]) >> 5; // 32 bits is needed for calculation

		axisPID[axis] = PTerm + ITerm - DTerm;
	}
	DEBUG("PID End")
	output_mix_cmd();

}

