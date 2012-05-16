/*
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version. see <http://www.gnu.org/licenses/>
 *
 * config.h
 *
 */

#ifndef MWI_CONFIG_H
#define MWI_CONFIG_H



/* Motor minthrottle
 *
 * Set the minimum throttle command sent to the ESC (Electronic Speed Controller)
 * This is the minimum value that allow motors to run at a idle speed  */
//#define MINTHROTTLE 1300 // for Turnigy Plush ESCs 10A
//#define MINTHROTTLE 1120 // for Super Simple ESCs 10A
//#define MINTHROTTLE 1220
#define MINTHROTTLE 1150 


/* YAW_DIRECTION
 *
 * if you want to reverse the yaw correction direction*/
//#define YAW_DIRECTION -1
#define YAW_DIRECTION 1


/**************************************************************************************/
/***************               advanced users settings             ********************/
/**************************************************************************************/


 
/**************************************************************************************/
/*****************          boards and sensor definitions            ******************/
/**************************************************************************************/

/***************************    Combined IMU Boards    ********************************/
/* if you use a specific sensor board:
   please submit any correction to this list.
     Note from Alex: I only own some boards
                     for other boards, I'm not sure, the info was gathered via rc forums, be cautious */
//#define FFIMUv1         // first 9DOF+baro board from Jussi, with HMC5843                   <- confirmed by Alex
//#define FFIMUv2         // second version of 9DOF+baro board from Jussi, with HMC5883       <- confirmed by Alex
//#define FREEIMUv1       // v0.1 & v0.2 & v0.3 version of 9DOF board from Fabio
//#define FREEIMUv03      // FreeIMU v0.3 and v0.3.1
//#define FREEIMUv035     // FreeIMU v0.3.5 no baro
//#define FREEIMUv035_MS  // FreeIMU v0.3.5_MS                                                <- confirmed by Alex
//#define FREEIMUv035_BMP // FreeIMU v0.3.5_BMP
//#define FREEIMUv04      // FreeIMU v0.4 with MPU6050, HMC5883L, MS561101BA                  <- confirmed by Alex
//#define FREEIMUv043     // same as FREEIMUv04 with final MPU6050 (with the right ACC scale)
//#define NANOWII         // the smallest multiwii FC based on MPU6050 + pro micro based proc <- confirmed by Alex
//#define PIPO            // 9DOF board from erazz
//#define QUADRINO        // full FC board 9DOF+baro board from witespy  with BMP085 baro     <- confirmed by Alex
//#define QUADRINO_ZOOM   // full FC board 9DOF+baro board from witespy  second edition
//#define QUADRINO_ZOOM_MS// full FC board 9DOF+baro board from witespy  second edition       <- confirmed by Alex
//#define ALLINONE        // full FC board or standalone 9DOF+baro board from CSG_EU
//#define AEROQUADSHIELDv2
//#define ATAVRSBIN1      // Atmel 9DOF (Contribution by EOSBandi). requires 3.3V power.
//#define SIRIUS          // Sirius Navigator IMU                                             <- confirmed by Alex
//#define SIRIUS600       // Sirius Navigator IMU  using the WMP for the gyro
//#define MINIWII         // Jussi's MiniWii Flight Controller                                <- confirmed by Alex
//#define CITRUSv2_1      // CITRUS from qcrc.ca
//#define CHERRY6DOFv1_0
//#define DROTEK_10DOF    // Drotek 10DOF with ITG3200, BMA180, HMC5883, BMP085, w or w/o LLC
//#define DROTEK_10DOF_MS // Drotek 10DOF with ITG3200, BMA180, HMC5883, MS5611, LLC
//#define DROTEK_6DOFv2   // Drotek 6DOF v2
//#define DROTEK_6DOF_MPU // Drotek 6DOF with MPU6050
//#define MONGOOSE1_0     // mongoose 1.0    http://store.ckdevices.com/
//#define CRIUS_LITE      // Crius MultiWii Lite
//#define CRIUS_SE        // Crius MultiWii SE
//#define OPENLRSv2MULTI  // OpenLRS v2 Multi Rc Receiver board including ITG3205 and ADXL345
//#define BOARD_PROTO_1   // with MPU6050 + HMC5883L + MS baro
//#define BOARD_PROTO_2   // with MPU6050 + slave  MAG3110 + MS baro


/***************************    independent sensors    ********************************/
//leave it commented if you already checked a specific board above
/* I2C gyroscope */
//#define ITG3200
//#define L3G4200D
//#define MPU6050       //combo + ACC

/* I2C accelerometer */
//#define MMA745
//#define ADXL345
//#define BMA020
//#define BMA180
//#define NUNCHACK  // if you want to use the nunckuk as a standalone I2C ACC without WMP
//#define LIS3LV02
//#define LSM303DLx_ACC

/* I2C barometer */
//#define BMP085
//#define MS561101BA

/* I2C magnetometer */
//#define HMC5843
//#define HMC5883
//#define AK8975
//#define MAG3110

// use the Devantech SRF i2c sensors, SRF08, SRF02
// (for now, there is no difference in the SRF0x code, but we may want to differentiate in the future.)
//#define SRF02
//#define SRF08
//#define SRF10
//#define SRF23


/*****************************    Gyro smoothing    **********************************/
/* GYRO_SMOOTHING. In case you cannot reduce vibrations _and_ _after_ you have tried the low pass filter options, you
   may try this gyro smoothing via averaging. Not suitable for multicopters!
   Good results for helicopter, airplanes and flying wings (foamies) with lots of vibrations.*/
//#define GYRO_SMOOTHING {20, 20, 3}    // separate averaging ranges for roll, pitch, yaw


/************************    Moving Average Gyros    **********************************/
//#define MMGYRO                         // Active Moving Average Function for Gyros
//#define MMGYROVECTORLENGHT 10          // Lenght of Moving Average Vector
// Moving Average ServoGimbal Signal Output
//#define MMSERVOGIMBAL                  // Active Output Moving Average Function for Servos Gimbal
//#define MMSERVOGIMBALVECTORLENGHT 32   // Lenght of Moving Average Vector



/**************************************************************************************/
/***************                 Failsave settings                 ********************/
/**************************************************************************************/
/*
   Failsafe check pulse on THROTTLE channel. If the pulse is OFF (on only THROTTLE or on all channels) the failsafe procedure is initiated.
   After FAILSAVE_DELAY time of pulse absence, the level mode is on (if ACC or nunchuk is avaliable), PITCH, ROLL and YAW is centered
   and THROTTLE is set to FAILSAVE_THR0TTLE value. You must set this value to descending about 1m/s or so for best results. 
   This value is depended from your configuration, AUW and some other params. 
   Next, afrer FAILSAVE_OFF_DELAY the copter is disarmed, and motors is stopped.
   If RC pulse coming back before reached FAILSAVE_OFF_DELAY time, after the small quard time the RC control is returned to normal.
   If you use serial sum PPM, the sum converter must completly turn off the PPM SUM pusles for this FailSafe functionality.*/

#define FAILSAVE_DELAY     10                     // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example
#define FAILSAVE_OFF_DELAY 200                    // Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example
#define FAILSAVE_THR0TTLE  (MINTHROTTLE + 200)    // Throttle level used for landing - may be relative to MINTHROTTLE - as in this case


/**************************************************************************************/
/***************             Special Throttle settings             ********************/
/**************************************************************************************/

/* this is the value for the ESCs when they are not armed
   in some cases, this value must be lowered down to 900 for some specific ESCs */
#define MINCOMMAND 1000

/* this is the maximum value for the ESCs at full power
   this value can be increased up to 2000 */
#define MAXTHROTTLE 1850


/* Pseudo-derivative conrtroller for level mode (experimental)
   Additional information: http://www.multiwii.com/forum/viewtopic.php?f=8&t=503 */
//#define LEVEL_PDF

/* introduce a deadband around the stick center
   Must be greater than zero, comment if you dont want a deadband on roll, pitch and yaw */
//#define DEADBAND 6


/**************************************************************************************/
/***********************     motor, servo and other presets     ***********************/
/**************************************************************************************/
/* motors will not spin when the throttle command is in low position
   this is an alternative method to stop immediately the motors */
//#define MOTOR_STOP

/* some radios have not a neutral point centered on 1500. can be changed here */
#define MIDRC 1500

/* The following lines apply only for a pitch/roll tilt stabilization system
   Uncomment the first line to activate it */
//#define SERVO_MIX_TILT              //  Simple CameraGimbal By Bledy http://youtu.be/zKGr6iR54vM
//#define SERVO_TILT
#define TILT_PITCH_MIN    1020    //servo travel min, don't set it below 1020
#define TILT_PITCH_MAX    2000    //servo travel max, max value=2000
#define TILT_PITCH_MIDDLE 1500    //servo neutral value
#define TILT_PITCH_PROP   10      //servo proportional (tied to angle) ; can be negative to invert movement
#define TILT_ROLL_MIN     1020
#define TILT_ROLL_MAX     2000
#define TILT_ROLL_MIDDLE  1500
#define TILT_ROLL_PROP    10

/* experimental
   camera trigger function : activated via Rc Options in the GUI, servo output=A2 on promini */
//#define CAMTRIG
#define CAM_SERVO_HIGH 2000  // the position of HIGH state servo
#define CAM_SERVO_LOW 1020   // the position of LOW state servo
#define CAM_TIME_HIGH 1000   // the duration of HIGH state servo expressed in ms
#define CAM_TIME_LOW 1000    // the duration of LOW state servo expressed in ms

/* you can change the tricopter servo travel here */
#define TRI_YAW_CONSTRAINT_MIN 1020
#define TRI_YAW_CONSTRAINT_MAX 2000
#define TRI_YAW_MIDDLE 1500 // tail servo center pos. - use this for initial trim; later trim midpoint via LCD

/* Flying Wing: you can change change servo orientation and servo min/max values here */
/* valid for all flight modes, even passThrough mode */
/* need to setup servo directions here; no need to swap servos amongst channels at rx */ 
#define PITCH_DIRECTION_L 1 // left servo - pitch orientation
#define PITCH_DIRECTION_R -1  // right servo - pitch orientation (opposite sign to PITCH_DIRECTION_L, if servos are mounted in mirrored orientation)
#define ROLL_DIRECTION_L 1 // left servo - roll orientation
#define ROLL_DIRECTION_R 1  // right servo - roll orientation  (same sign as ROLL_DIRECTION_L, if servos are mounted in mirrored orientation)
#define WING_LEFT_MID  1500 // left servo center pos. - use this for initial trim; later trim midpoint via LCD
#define WING_RIGHT_MID 1500 // right servo center pos. - use this for initial trim; later trim midpoint via LCD
#define WING_LEFT_MIN  1020 // limit servo travel range must be inside [1020;2000]
#define WING_LEFT_MAX  2000 // limit servo travel range must be inside [1020;2000]
#define WING_RIGHT_MIN 1020 // limit servo travel range must be inside [1020;2000]
#define WING_RIGHT_MAX 2000 // limit servo travel range must be inside [1020;2000]

//***********************************************************************************************//
//******************************* !!!!  Airplane Settings  !!!! *********************************//
//***********************************************************************************************//
// Howto setup =>>> http://fotoflygarn.blogspot.com/2012/03/how-to-setup-multiwii-airplane-same.html

#define SERVO_OFFSET     {  0,   0,   0,  0,   0,   0,  0,   0 } // Adjust Servo MID Offset & Swash angles 
#define SERVO_RATES      {100, 100, 100, 100, 100, 100, 100, 100} // Rates in 0-100% 
#define SERVO_DIRECTION  { -1,   1,   1,   -1,  1,   1,   1,   1 } // Invert servos by setting -1 
 
#define FLAP_CHANNEL     AUX4       // Define the Channel to controll Flaps with.If used.
#define FLAP_EP      { 1500, 1650 } // Endpooints for flaps on a 2 way switch else set {1020,2000} and program in radio.
#define FLAP_INVERT    { 1, -1 }    // Change direction om flaps { Wing1, Wing2 }



//***********************************************************************************************//
//****************************** !!!!  Hellicopter Settings  !!!! *******************************//
//***********************************************************************************************//
// Channel to controll CollectivePitch
#define COLLECTIVE_PITCH      THROTTLE   
// Set Maximum available movement for the servos. Depending on modell.
#define SERVO_ENDPOINT_HIGH {2000,2000,2000,2000,2000,2000,2000,2000};
#define SERVO_ENDPOINT_LOW  {1020,1020,1020,1020,1020,1020,1020,1020};

// Limit the range of Collective Pitch. 100% is Full Range each way and position for Zero Pitch
#define COLLECTIVE_RANGE { 80, 1500, 80 }// {Min%, ZeroPitch, Max%}.
#define YAW_CENTER             1500      // Use servo[5] SERVO_ENDPOINT_HIGH/LOW for the endpoits.
#define YAWMOTOR                0       // If a motor is use as YAW Set to 1 else set to 0.

// Limit Maximum controll for Roll & Nick  in 0-100%  
#define CONTROLL_RANGE   { 100, 100 }      //  { ROLL,PITCH }
//*************************************************************************************************// 


//TODO use runtime cfg for log and debug
/********************************************************************/
/****           diagnostics                                      ****/
/********************************************************************/

/* to log values like max loop time and others to come */
/* logging values are visible via LCD config */
/* set to 2, if you want powerconsumption on a per motor basis (this uses the big array and is a memory hog, if POWERMETER <> PM_SOFT) */
//#define LOG_VALUES 1

/* to add debugging code */
/* not needed and not recommended for normal operation */
/* will add extra code that may slow down the main loop or make copter non-flyable */
//#define DEBUG

#endif
