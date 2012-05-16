/*
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version. see <http://www.gnu.org/licenses/>
 *
 * imu.c
 *
 *
 * Simplified IMU based on "Complementary Filter"
 * Inspired by http://starlino.com/imu_guide.html
 *
 * adapted by ziss_dm : http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
 *
 * The following ideas was used in this project:
 * 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
 * 2) Small-angle approximation: http://en.wikipedia.org/wiki/Small-angle_approximation
 * 3) C. Hastings approximation for atan2()
 * 4) Optimization tricks: http://www.hackersdelight.org/
 *
 * Currently Magnetometer uses separate CF which is used only
 * for heading approximation.
 *
 *
 */

#include <stdint.h>
#include <stdlib.h>
#include "util.h"
#include "multiwii.h"
#include "sensors.h"

//******  advanced users settings *******************
/* Set the Low Pass Filter factor for ACC */
/* Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time*/
/* Comment this if  you do not want filter at all.*/
#define ACC_LPF_FACTOR 100

/* Set the Low Pass Filter factor for Magnetometer */
/* Increasing this value would reduce Magnetometer noise (not visible in GUI), but would increase Magnetometer lag time*/
/* Comment this if  you do not want filter at all.*/
/* Default WMC value: n/a*/
//#define MG_LPF_FACTOR 4

/* Set the Gyro Weight for Gyro/Acc complementary filter */
/* Increasing this value would reduce and delay Acc influence on the output of the filter*/
/* Default WMC value: 300*/
#define GYR_CMPF_FACTOR 400.0f

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter */
/* Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
/* Default WMC value: n/a*/
#define GYR_CMPFM_FACTOR 200.0f

//****** end of advanced users settings *************

#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))
//#if GYRO
#define GYRO_SCALE ((2380 * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f)) //should be 2279.44 but 2380 gives better result
// +-2000/sec deg scale
//#define GYRO_SCALE ((200.0f * PI)/((32768.0f / 5.0f / 4.0f ) * 180.0f * 1000000.0f) * 1.5f)
// +- 200/sec deg scale
// 1.5 is emperical, not sure what it means
// should be in rad/sec
//#else
//#define GYRO_SCALE (1.0f/200e6f)
//// empirical, depends on WMP on IDG datasheet, tied of deg/ms sensibility
//// !!!!should be adjusted to the rad/sec
//#endif

int16_t gyroADC[3], accADC[3], magADC[3], accSmooth[3];
int16_t acc_25deg = 0;
int32_t BaroAlt;
int32_t EstAlt;             // in cm
int16_t BaroPID = 0;
int32_t AltHold;
int16_t errorAltitudeI = 0;
uint16_t acc_1G;         // this is the 1G measured acceleration

int16_t gyroData[3] = { 0, 0, 0 };
int16_t gyroZero[3] = { 0, 0, 0 };
int16_t angle[2] = { 0, 0 };     // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
int8_t smallAngle25 = 1;

int16_t  annex650_overrun_count = 0;

int8_t imu_init(){
	return sensors_init();
}


void getEstimatedAttitude(void);

void imu_compute () {
	uint8_t axis;
	static int16_t gyroADCprevious[3] = {0,0,0};
	int16_t gyroADCp[3];
	int16_t gyroADCinter[3];
	static uint32_t timeInterleave = 0;

	//TODO removed : we separate the 2 situations because reading gyro values with a gyro only setup can be acchieved at a higher rate
	//gyro+nunchuk: we must wait for a quite high delay betwwen 2 reads to get both WM+ and Nunchuk data. It works with 3ms
	//gyro only: the delay to read 2 consecutive values can be reduced to only 0.65ms
	{
		if( cfg.ACC){
			sensors_acc_read();
			getEstimatedAttitude();
		}

		sensors_gyro_read();

		for (axis = 0; axis < 3; axis++){
			gyroADCp[axis] =  gyroADC[axis];
		}
		timeInterleave=micros();
		annexCode();
		if ((micros()-timeInterleave)>650) {
			annex650_overrun_count++;
		} else {
			while((micros()-timeInterleave)<650){ usleep(50);}; //empirical, interleaving delay between 2 consecutive reads
		}

		sensors_gyro_read();

		for (axis = 0; axis < 3; axis++) {
			gyroADCinter[axis] =  gyroADC[axis]+gyroADCp[axis];
			// empirical, we take a weighted value of the current and the previous values
			gyroData[axis] = (gyroADCinter[axis]+gyroADCprevious[axis])/3;
			gyroADCprevious[axis] = gyroADCinter[axis]/2;
			if (!cfg.ACC) accADC[axis]=0;
		}
	}

	#if defined(GYRO_SMOOTHING)
		static uint8_t Smoothing[3]  = GYRO_SMOOTHING; // How much to smoothen with per axis
		static int16_t gyroSmooth[3] = {0,0,0};
		for (axis = 0; axis < 3; axis++) {
			gyroData[axis] = (gyroSmooth[axis]*(Smoothing[axis]-1)+gyroData[axis])/Smoothing[axis];
			gyroSmooth[axis] = gyroData[axis];
		}
	#elif defined(TRI)
		static int16_t gyroYawSmooth = 0;
		gyroData[YAW] = (gyroYawSmooth*2+gyroData[YAW])/3;
		gyroYawSmooth = gyroData[YAW];
	#endif
		DEBUG("imu_compute end");
}





// Small angle approximation
#define ssin(val) (val)
#define scos(val) 1.0f

typedef struct fp_vector {
	float X;
	float Y;
	float Z;
} t_fp_vector_def;

typedef union {
	float   A[3];
	t_fp_vector_def V;
} t_fp_vector;

#define fp_is_neg(val) ((((uint8_t*)&val)[3] & 0x80) != 0)

int16_t _atan2(float y, float x){

	float z = y / x;
	int16_t zi = abs((int16_t)(z * 100));
	int8_t y_neg = fp_is_neg(y);
	if ( zi < 100 ){
		if (zi > 10)
			z = z / (1.0f + 0.28f * z * z);
		if (fp_is_neg(x)) {
			if (y_neg) z -= PI;
			else z += PI;
		}
	} else {
		z = (PI / 2.0f) - z / (z * z + 0.28f);
		if (y_neg) z -= PI;
	}
	z *= (180.0f / PI * 10);
	return z;
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v,float* delta) {
	struct fp_vector v_tmp = *v;
	v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;
	v->X += delta[ROLL]  * v_tmp.Z - delta[YAW]   * v_tmp.Y;
	v->Y += delta[PITCH] * v_tmp.Z + delta[YAW]   * v_tmp.X;
}

void getEstimatedAttitude(){
	DEBUG("getEstimatedAttitude begin")
	uint8_t axis;
	int32_t accMag = 0;
	static t_fp_vector EstG;
	static t_fp_vector EstM;

	#if defined(MG_LPF_FACTOR)
		static int16_t mgSmooth[3];
	#endif
	#if defined(ACC_LPF_FACTOR)
		static float accLPF[3];
	#endif
	static uint16_t previousT;
	uint16_t currentT = micros();
	float scale, deltaGyroAngle[3];

	scale = (currentT - previousT) * GYRO_SCALE;
	previousT = currentT;

	// Initialization
	for (axis = 0; axis < 3; axis++) {
		deltaGyroAngle[axis] = gyroADC[axis]  * scale;
		#if defined(ACC_LPF_FACTOR)
			accLPF[axis] = accLPF[axis] * (1.0f - (1.0f/ACC_LPF_FACTOR)) + accADC[axis] * (1.0f/ACC_LPF_FACTOR);
			accSmooth[axis] = accLPF[axis];
			#define ACC_VALUE accSmooth[axis]
		#else
			accSmooth[axis] = accADC[axis];
			#define ACC_VALUE accADC[axis]
		#endif
			DEBUG("axe gyro done")
		//    accMag += (ACC_VALUE * 10 / (int16_t)acc_1G) * (ACC_VALUE * 10 / (int16_t)acc_1G);
		accMag += (int32_t)ACC_VALUE*ACC_VALUE ;
			DEBUG("begin axe mag ")
			if( cfg.MAG){
			#if defined(MG_LPF_FACTOR)
				mgSmooth[axis] = (mgSmooth[axis] * (MG_LPF_FACTOR - 1) + magADC[axis]) / MG_LPF_FACTOR; // LPF for Magnetometer values
				#define MAG_VALUE mgSmooth[axis]
			#else
				#define MAG_VALUE magADC[axis]
			#endif
				DEBUG("axe mag done")
			}
	}

	if (acc_1G == 0){
		acc_1G = 250;
	}
	accMag = accMag*100/((int32_t)acc_1G*acc_1G);

	rotateV(&EstG.V,deltaGyroAngle);
	DEBUG("rotate EstG done")
	if (cfg.MAG){
		rotateV(&EstM.V,deltaGyroAngle);
		DEBUG("rotate EstM done")
	}

	if ( abs(accSmooth[ROLL])<acc_25deg && abs(accSmooth[PITCH])<acc_25deg && accSmooth[YAW]>0)
		smallAngle25 = 1;
	else
		smallAngle25 = 0;

	DEBUG("Gyro drift correction begin")
	// Apply complimentary filter (Gyro drift correction)
	// If accel magnitude >1.4G or <0.6G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
	// To do that, we just skip filter, as EstV already rotated by Gyro
	if ( ( 36 < accMag && accMag < 196 ) || smallAngle25 )
		for (axis = 0; axis < 3; axis++) {
			int16_t acc = ACC_VALUE;
			EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + acc) * INV_GYR_CMPF_FACTOR;
		}
	if (cfg.MAG){
		for (axis = 0; axis < 3; axis++)
			EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR  + MAG_VALUE) * INV_GYR_CMPFM_FACTOR;
			}

	DEBUG("angle begin")
	// Attitude of the estimated vector
	angle[ROLL]  =  _atan2(EstG.V.X , EstG.V.Z) ;
	angle[PITCH] =  _atan2(EstG.V.Y , EstG.V.Z) ;
	if (cfg.MAG){
	DEBUG("heading begin")
		// Attitude of the cross product vector GxM
		heading = _atan2( EstG.V.X * EstM.V.Z - EstG.V.Z * EstM.V.X , EstG.V.Z * EstM.V.Y - EstG.V.Y * EstM.V.Z  ) / 10;
	}
}

#define BARO_UPDATE_INTERVAL 25000    // 40hz update rate (20hz LPF on acc)
#define BARO_INIT_DELAY      4000000  // 4 sec initialization delay
#define BARO_TAB_SIZE   40

void imu_baro_compute(){
	uint8_t index;
	static uint32_t deadLine = BARO_INIT_DELAY;

	static int16_t BaroHistTab[BARO_TAB_SIZE];
	static int8_t BaroHistIdx;
	static int32_t BaroHigh,BaroLow;
	int32_t temp32;
	int16_t last;

	if (currentTime < deadLine) return;
	deadLine = currentTime + BARO_UPDATE_INTERVAL;

	//**** Alt. Set Point stabilization PID ****
	//calculate speed for D calculation
	last = BaroHistTab[BaroHistIdx];
	BaroHistTab[BaroHistIdx] = BaroAlt/10;
	BaroHigh += BaroHistTab[BaroHistIdx];
	index = (BaroHistIdx + (BARO_TAB_SIZE/2))%BARO_TAB_SIZE;
	BaroHigh -= BaroHistTab[index];
	BaroLow  += BaroHistTab[index];
	BaroLow  -= last;

	BaroHistIdx++;
	if (BaroHistIdx == BARO_TAB_SIZE) BaroHistIdx = 0;

	BaroPID = 0;
	//D
	temp32 = cfg.D8[PIDALT]*(BaroHigh - BaroLow) / 40;
	BaroPID-=temp32;

	EstAlt = BaroHigh*10/(BARO_TAB_SIZE/2);

	temp32 = AltHold - EstAlt;
	if (abs(temp32) < 10 && abs(BaroPID) < 10) BaroPID = 0;  //remove small D parametr to reduce noise near zero position

	//P
	BaroPID += cfg.P8[PIDALT]*constrain(temp32,(-2)*cfg.P8[PIDALT],2*cfg.P8[PIDALT])/100;
	BaroPID = constrain(BaroPID,-150,+150); //sum of P and D should be in range 150

	//I
	errorAltitudeI += temp32*cfg.I8[PIDALT]/50;
	errorAltitudeI = constrain(errorAltitudeI,-30000,30000);
	temp32 = errorAltitudeI / 500; //I in range +/-60
	BaroPID+=temp32;
}
