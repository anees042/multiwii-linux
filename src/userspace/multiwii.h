/*
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version. see <http://www.gnu.org/licenses/>
 *
 * multiwii.h
 *
 */

#ifndef MWI_H
#define MWI_H

#include <stdint.h>

#define  VERSION  200

/*********** RC alias *****************/
#define ROLL       0
#define PITCH      1
#define YAW        2
#define THROTTLE   3
#define AUX1       4
#define AUX2       5
#define AUX3       6
#define AUX4       7

#define PIDALT     3
#define PIDVEL     4
#define PIDGPS     5
#define PIDLEVEL   6
#define PIDMAG     7

#define BOXACC       0
#define BOXBARO      1
#define BOXMAG       2
#define BOXCAMSTAB   3
#define BOXCAMTRIG   4
#define BOXARM       5
#define BOXGPSHOME   6
#define BOXGPSHOLD   7
#define BOXPASSTHRU  8
#define BOXHEADFREE  9
#define BOXBEEPERON  10

#define CHECKBOXITEMS 11
#define PIDITEMS 8


#define LEDPIN_PINMODE
#define LEDPIN_TOGGLE
#define LEDPIN_OFF
#define LEDPIN_ON

#define STABLEPIN_ON
#define STABLEPIN_OFF

// ******************
// rc functions
// ******************
#define MINCHECK 1100
#define MAXCHECK 1900


//
//
//enum {
//    GIMBAL_TILTONLY = 1,                        // In standalone gimbal mode, this switches PPM port into a single-channel PWM input
//    GIMBAL_LEVEL_STAB = 2                       // In quadx/quadp with camstab enabled, ignore CAM1/CAM2 inputs and just stabilize level.
//};
//
//

typedef struct config_t {
    uint8_t version;

    uint8_t mixerConfiguration;
    int8_t flap_channel ;
    uint8_t FAILSAFE;
    uint8_t GYRO;
    uint8_t ACC;
    uint8_t BARO;
    uint8_t MAG;
    uint8_t SONAR;

    uint16_t motor_disarmed_value;

    uint8_t P8[8];
    uint8_t I8[8];
    uint8_t D8[8];
    uint8_t rcRate8;
    uint8_t rcExpo8;
    uint8_t rollPitchRate;
    uint8_t yawRate;
    uint8_t dynThrPID;

    // sensors
    int16_t accZero[3];
    int16_t magZero[3];
    int16_t accTrim[2];

    // battery monitor
    uint8_t vbatscale;                      // adjust this to match battery voltage to reported value
    uint8_t vbatmaxcellvoltage;             // maximum voltage per cell, used for auto-detecting battery voltage in 0.1V units, default is 43 (4.3V)
    uint8_t vbatmincellvoltage;             // minimum voltage per cell, this triggers battery out alarms, in 0.1V units, default is 33 (3.3V)

    // rc related
    uint8_t deadband;                       // introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
    uint8_t activate1[CHECKBOXITEMS];
    uint8_t activate2[CHECKBOXITEMS];
    uint16_t midrc;                         // Some radios have not a neutral point centered on 1500. can be changed here

    // camera gimbal settings
    uint8_t gimbal_flags;                   // To be determined
    int8_t gimbal_gain_pitch;               // Amount of servo gain per angle of inclination for pitch (can be negative to invert movement)
    int8_t gimbal_gain_roll;                // Amount of servo gain per angle of inclination for roll (can be negative to invert movement)


    uint8_t thrMid8 ;
    uint8_t thrExpo8 ;

} config_t;
//
extern config_t cfg;
//
//extern int16_t lookupRX[7];
//extern uint8_t powerTrigger1;

extern int16_t rcData[8];
//
//extern uint8_t useSpektrum;
//extern volatile uint8_t rcFrameComplete;
//

//  mw.c
extern int16_t annex650_overrun_count;
extern uint64_t currentTime;
extern uint64_t previousTime;

// IMU/Gyro/Acc
extern int8_t smallAngle25;
extern int16_t angle[2];
extern int16_t gyroADC[3], accADC[3], magADC[3];
extern int16_t gyroData[3];
extern int16_t gyroZero[3];
extern int16_t accSmooth[3];
//extern int16_t accZero[3];
//extern int16_t magZero[3];
extern uint16_t acc_1G;
extern int16_t acc_25deg;
extern int16_t heading;
//extern volatile int16_t failsafeCnt;
//
//// config.c
//void readEEPROM(void);
//void checkFirstTime(void);
//void writeParams(void);
//
//// multiwii.c
//void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat);
void annexCode(void);
//

//
//// serial.c
//void serialCom(void);

#endif




