/*
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version. see <http://www.gnu.org/licenses/>
 *
 * Sensors definition.
 *
 */

#ifndef BGSENSORS_H
#define BGSENSORS_H


/*
 *  Arduino Motor/Servo slave
 */
#define ARDUINO 		1
#define ARDUINO_ADDRESS	0x10




/*
 * Gyro ITG3200
 */
#define ITG3200 		0
#define ITG3200_ADDRESS	0x69


//ITG3200 and ITG3205 Gyro LPF setting
#if defined(ITG3200_LPF_256HZ) || defined(ITG3200_LPF_188HZ) || defined(ITG3200_LPF_98HZ) || defined(ITG3200_LPF_42HZ) || defined(ITG3200_LPF_20HZ) || defined(ITG3200_LPF_10HZ)
#if defined(ITG3200_LPF_256HZ)
#define ITG3200_SMPLRT_DIV 0    //8000Hz
#define ITG3200_DLPF_CFG   0
#endif
#if defined(ITG3200_LPF_188HZ)
#define ITG3200_SMPLRT_DIV 0    //1000Hz
#define ITG3200_DLPF_CFG   1
#endif
#if defined(ITG3200_LPF_98HZ)
#define ITG3200_SMPLRT_DIV 0
#define ITG3200_DLPF_CFG   2
#endif
#if defined(ITG3200_LPF_42HZ)
#define ITG3200_SMPLRT_DIV 0
#define ITG3200_DLPF_CFG   3
#endif
#if defined(ITG3200_LPF_20HZ)
#define ITG3200_SMPLRT_DIV 0
#define ITG3200_DLPF_CFG   4
#endif
#if defined(ITG3200_LPF_10HZ)
#define ITG3200_SMPLRT_DIV 0
#define ITG3200_DLPF_CFG   5
#endif
#else
//Default settings LPF 256Hz/8000Hz sample
#define ITG3200_SMPLRT_DIV 0    //8000Hz
#define ITG3200_DLPF_CFG   0
#endif


#define ITG3200_REG_TEMP_OUT_H			0x1B
#define ITG3200_REG_TEMP_OUT_L			0x1C
#define ITG3200_REG_GYRO_XOUT_H			0x1D
#define ITG3200_REG_GYRO_XOUT_L			0x1E
#define ITG3200_REG_GYRO_YOUT_H			0x1F
#define ITG3200_REG_GYRO_YOUT_L			0x20
#define ITG3200_REG_GYRO_ZOUT_H			0x21
#define ITG3200_REG_GYRO_ZOUT_L			0x22

#define ITG3200_REG_POWER_MGMT			0x3E
#define ITG3200_REG_SAMPLE_RATE_DIV		0x15

#define ITG3200_REG_LP_FULL_SCALE		0x16

#define ITG3200_REG_IRQ					0x17

#define ITG3200_REG_IRQ_STATUS			0x1A

//++ merge in




/**************************************************************************************/
/***************      IMU Sensor definitions and Orientations      ********************/
/**************************************************************************************/


//please submit any correction to this list.
#if defined(FFIMUv1)
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5843
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
  #define BMA180_ADDRESS 0x80
  #define ITG3200_ADDRESS 0XD0
#endif

#if defined(FFIMUv2)
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  Y; magADC[PITCH]  = -X; magADC[YAW]  = -Z;}
  #define BMA180_ADDRESS 0x80
  #define ITG3200_ADDRESS 0XD0
#endif

#if defined(FREEIMUv1)
  #define ITG3200
  #define ADXL345
  #define HMC5843
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X;  magADC[PITCH] =  Y; magADC[YAW]  = -Z;}
  #define ADXL345_ADDRESS 0xA6
#endif

#if defined(FREEIMUv03)
  #define ITG3200
  #define ADXL345 // this is actually an ADXL346 but that's just the same as ADXL345
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
  #define ADXL345_ADDRESS 0xA6
#endif

#if defined(FREEIMUv035) || defined(FREEIMUv035_MS) || defined(FREEIMUv035_BMP)
  #define ITG3200
  #define BMA180
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
  #if defined(FREEIMUv035_MS)
    #define MS561101BA
  #elif defined(FREEIMUv035_BMP)
    #define BMP085
  #endif
#endif

#if defined(FREEIMUv04)
 #define FREEIMUv043
#endif

#if defined(FREEIMUv043)
  #define MPU6050
  #define HMC5883
  #define MS561101BA
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
  #define MPU6050_I2C_AUX_MASTER // MAG connected to the AUX I2C bus of MPU6050
#endif

#if defined(NANOWII)
  #define MPU6050
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -Y; accADC[PITCH]  =  X; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = -X; gyroADC[PITCH] = -Y; gyroADC[YAW] = -Z;}
#endif

#if defined(PIPO)
  #define L3G4200D
  #define ADXL345
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] =  Y; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  Y; magADC[PITCH]  = -X; magADC[YAW]  =  Z;}
  #define ADXL345_ADDRESS 0xA6
#endif

#if defined(QUADRINO)
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
  #define BMA180_ADDRESS 0x80
  #define ITG3200_ADDRESS 0XD0
#endif

#if defined(QUADRINO_ZOOM)
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
  #define BMA180_ADDRESS 0x80
  #define ITG3200_ADDRESS 0XD0
#endif

#if defined(QUADRINO_ZOOM_MS)
  #define ITG3200
  #define BMA180
  #define MS561101BA
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
  #define BMA180_ADDRESS 0x80
  #define ITG3200_ADDRESS 0XD0
#endif

#if defined(ALLINONE)
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
  #define BMA180_ADDRESS 0x82
#endif

#if defined(AEROQUADSHIELDv2) // to confirm
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5843
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] =  Y; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
  #define ITG3200_ADDRESS 0XD2
#endif

#if defined(ATAVRSBIN1)
  #define ITG3200
  #define BMA020        //Actually it's a BMA150, but this is a drop in replacement for the discountinued BMA020
  #define AK8975
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  Y; accADC[PITCH]  = -X; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] =  Y; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = -Y; magADC[PITCH]  = -X; magADC[YAW]  =  Z;}
#endif

#if defined(SIRIUS)
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
  #define BMA180_ADDRESS 0x80
  #define ITG3200_ADDRESS 0XD0
#endif

#if defined(SIRIUS600)
  #define BMA180
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
  #define BMA180_ADDRESS 0x80
#endif

#if defined(MINIWII)
  #define ITG3200
  #define BMA180
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
#endif

#if defined(CITRUSv2_1)
  #define ITG3200
  #define ADXL345
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z) {accADC[ROLL] = -X; accADC[PITCH] = -Y; accADC[YAW] = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z) {magADC[ROLL] = X; magADC[PITCH] = Y; magADC[YAW] = -Z;}
  #define ITG3200_ADDRESS 0XD0
#endif

#if defined(CHERRY6DOFv1_0)
  #define MPU6050
  #define ACC_ORIENTATION(Y, X, Z)  {accADC[ROLL]  = -Y; accADC[PITCH]  = -X; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(Y, X, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = -Y; gyroADC[YAW] = -Z;}
#endif

#if defined(DROTEK_10DOF) || defined(DROTEK_10DOF_MS)
  #define ITG3200
  #define BMA180
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
  #define ITG3200_ADDRESS 0XD2
  #if defined(DROTEK_10DOF_MS)
    #define MS561101BA
  #elif defined(DROTEK_10DOF)
    #define BMP085
  #endif
#endif

#if defined(DROTEK_6DOFv2)
  #define ITG3200
  #define BMA180
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -Y; accADC[PITCH]  =  X; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = -X; gyroADC[PITCH] = -Y; gyroADC[YAW] = -Z;}
  #define ITG3200_ADDRESS 0XD2
#endif

#if defined(DROTEK_6DOF_MPU)
  #define MPU6050
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -Y; accADC[PITCH]  =  X; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = -X; gyroADC[PITCH] = -Y; gyroADC[YAW] = -Z;}
  #define MPU6050_ADDRESS 0xD2
#endif

#if defined(MONGOOSE1_0)
  #define ITG3200
  #define ADXL345
  #define BMP085
  #define HMC5883
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = -Y; gyroADC[PITCH] =  X; gyroADC[YAW] = -Z;}
  #define ACC_ORIENTATION(Y, X, Z)  {accADC[ROLL]  =  Y; accADC[PITCH]  =  X; accADC[YAW]  =  Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = -X; magADC[PITCH]  = -Y; magADC[YAW]  = -Z;}
  #define ADXL345_ADDRESS 0xA6
  #define ITG3200_ADDRESS 0XD0
  #define BMP085_ADDRESS 0xEE
#endif

#if defined(CRIUS_LITE)
  #define ITG3200
  #define ADXL345
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
#endif

#if defined(CRIUS_SE)
  #define ITG3200
  #define BMA180
  #define HMC5883
  #define BMP085
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
#endif



// ************************************************************************************************************
// default board orientation and setup
// ************************************************************************************************************

#if !defined(ACC_ORIENTATION)
#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
#endif
#if !defined(GYRO_ORIENTATION)
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
#endif
#if !defined(MAG_ORIENTATION)
#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = X; magADC[PITCH]  = Y; magADC[YAW]  = Z;}
#endif


//++ merge end
#endif
