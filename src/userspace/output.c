/*
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version. see <http://www.gnu.org/licenses/>
 *
 * output.c
 *
 */
#include "util.h"
#include "multiwii.h"
#include "def.h"

#include <stdio.h>
#include <stdlib.h>

#define SERVO_MIN 1020           // limit servo travel range must be inside [1020;2000]
#define SERVO_MAX 2000           // limit servo travel range must be inside [1020;2000]

int16_t motor[8];
int16_t servo[8] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };
uint8_t useServo = 0;
uint8_t numberMotor = 4;

static uint16_t wing_left_mid  = WING_LEFT_MID;
static uint16_t wing_right_mid = WING_RIGHT_MID;
static uint16_t tri_yaw_middle = TRI_YAW_MIDDLE;

extern int16_t axisPID[3];
extern int16_t rcCommand[4];
extern int16_t angle[2];
extern uint8_t armed;
//extern uint8_t cfg.mixerConfiguration;
extern uint8_t passThruMode;
// Common controlls for Helicopters
int16_t heliRoll,heliNick;
int16_t collRange[3] = COLLECTIVE_RANGE;
static int16_t   collective;
static int16_t   servoEndpiont[8][2];

/*
 * Send the Servos values to devices
 */
void output_servo_write() {
	DEBUG("output_servo_write")


        								if (!useServo)
        									return;

	// TODO SEND OUTPUT->  servo[i];
	if (cfg.mixerConfiguration == MULTITYPE_TRI || cfg.mixerConfiguration == MULTITYPE_BI) {
		/* One servo on Motor #4 */
		//pwmWrite(4, servo[4]);
		if (cfg.mixerConfiguration == MULTITYPE_BI){
			//  pwmWrite(5, servo[5]);
			;}
	} else {
		/* Two servos for camstab or FLYING_WING */
		//pwmWrite(4, servo[4]);
		//pwmWrite(5, servo[5]);
	}
}

/*
 * Send the motors values to devices
 */
void output_motor_write() { // [1000;2000] => [125;250]
	DEBUG("output_motor_write")

									for (uint8_t i =0;i<numberMotor;i++){
										// TODO SEND OUTPUT-> motor[i];

									}

}

/*
 * Send the <mc> value to all Motors
 */
void output_motor_write_all(int16_t mc) {   // Sends commands to all motors
	for (uint8_t i =0;i<numberMotor;i++)
		motor[i]=mc;
	output_motor_write();
}

void output_init_servo() {

	// set each servo to his default position
}

/*
 * Initialize the servos and motors
 */
void output_init() {

	if (cfg.mixerConfiguration == MULTITYPE_BI || cfg.mixerConfiguration == MULTITYPE_TRI || cfg.mixerConfiguration == MULTITYPE_GIMBAL || cfg.mixerConfiguration == MULTITYPE_FLYING_WING)
		useServo = 1;

#if defined(SERVO_TILT) || defined(CAMTRIG)
	useServo = 1;
#endif

	switch (cfg.mixerConfiguration) {
	case MULTITYPE_GIMBAL:
		numberMotor = 0;
		break;
	case MULTITYPE_FLYING_WING:
		numberMotor = 1;
		break;
	case MULTITYPE_BI:
		numberMotor = 2;
		break;
	case MULTITYPE_TRI:
		numberMotor = 3;
		break;

	case MULTITYPE_QUADP:
	case MULTITYPE_QUADX:
	case MULTITYPE_Y4:
		numberMotor = 4;
		break;

	case MULTITYPE_Y6:
	case MULTITYPE_HEX6:
	case MULTITYPE_HEX6X:
		numberMotor = 6;
		break;

	case MULTITYPE_OCTOX8:
	case MULTITYPE_OCTOFLATP:
	case MULTITYPE_OCTOFLATX:
		numberMotor = 8;
		break;
	}
	output_init_servo();

	output_motor_write_all(MINCOMMAND);
	delay(300);

}

void output_mix_heli(){


	static int16_t   servoHigh[8] = SERVO_ENDPOINT_HIGH; // HIGHpoint on servo
	static int16_t   servoLow[8]  = SERVO_ENDPOINT_LOW ; // LOWpoint on servo

	/***************************
	 * servo settings Heli.
	 ***************************/
	for(int i=0; i<8; i++){  //  Set rates using endpoints.
		servoEndpiont[i][0] = servoLow[i];  //Min
		servoEndpiont[i][1] = servoHigh[i]; //Max
	}

	// Limit Collective range up/down
	int16_t collect = rcData[COLLECTIVE_PITCH]-collRange[1];
	if   (collect>0) {
		collective = collect * (collRange[2]*0.01);
	} else{
		collective = collect * (collRange[0]*0.01);
	}

	if(passThruMode){ // Use Rcdata Without sensors
		heliRoll=  rcCommand[ROLL] ;
		heliNick=  rcCommand[PITCH];
	} else{ // Assisted modes
		heliRoll= axisPID[ROLL];
		heliNick= axisPID[PITCH];
	}

	// Limit Maximum Rates for Heli
	int16_t cRange[2] = CONTROLL_RANGE;
	heliRoll*=cRange[0]*0.01;
	heliNick*=cRange[1]*0.01;


	// Yaw is common for Heli 90 & 120
	uint16_t yawControll =  YAW_CENTER + (axisPID[YAW]*YAW_DIRECTION);

	/* Throttle & YAW
	 ********************
	  Handeled in common functions for Heli */
	if (!armed){
		servo[7] = 900; // Kill throttle when disarmed
		if (YAWMOTOR){servo[5] =  MINCOMMAND;} else {servo[5] =  yawControll; } // Kill YAWMOTOR when disarmed
	}else {
		servo[7]  = rcData[THROTTLE]; //   50hz ESC or servo
		if (YAWMOTOR && rcData[THROTTLE] < MINTHROTTLE){servo[5] =  MINCOMMAND;}
		else{ servo[5] =  yawControll; }     // YawSero
	}
}



#define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z

#define HeliXPIDMIX(Z,Y,X) collRange[1]+collective*Z + heliNick*Y +  heliRoll*X

/*
 *  Mixes and send the computed stabilize values to the Motors & Servos
 */
void output_mix_cmd() {

	DEBUG("output_send_cmd")

							int16_t maxMotor;
	//	uint8_t i;


	// Common parts for Plane and Heli
	static int16_t   servoMid[8];                        // Midpoint on servo
	static uint8_t   servoTravel[8] = SERVO_RATES;       // Rates in 0-100%
	static int8_t    Mid[8] = SERVO_OFFSET;
	static int8_t    servoReverse[8] = SERVO_DIRECTION ; // Inverted servos
	static int16_t   servoLimit[8][2]; // Holds servoLimit data
	// Flapperon Controll
	int16_t flaps[2]={0,0  };

	if (numberMotor > 3) {
		//prevent "yaw jump" during yaw correction
		axisPID[YAW] = constrain(axisPID[YAW], -100 - abs(rcCommand[YAW]), +100 + abs(rcCommand[YAW]));
	}

	switch (cfg.mixerConfiguration) {
	case MULTITYPE_BI:
		motor[0] = PIDMIX(+1, 0, 0);        //LEFT
		motor[1] = PIDMIX(-1, 0, 0);        //RIGHT
		servo[4] = constrain(1500 + (YAW_DIRECTION * axisPID[YAW]) + axisPID[PITCH], 1020, 2000);   //LEFT
		servo[5] = constrain(1500 + (YAW_DIRECTION * axisPID[YAW]) - axisPID[PITCH], 1020, 2000);   //RIGHT
		break;

	case MULTITYPE_TRI:
		motor[0] = PIDMIX(0, +4 / 3, 0);    //REAR
		motor[1] = PIDMIX(-1, -2 / 3, 0);   //RIGHT
		motor[2] = PIDMIX(+1, -2 / 3, 0);   //LEFT
		servo[4] = constrain(TRI_YAW_MIDDLE + YAW_DIRECTION * axisPID[YAW], TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX);        //REAR
		break;

	case MULTITYPE_QUADP:
		motor[0] = PIDMIX(0, +1, -1);       //REAR
		motor[1] = PIDMIX(-1, 0, +1);       //RIGHT
		motor[2] = PIDMIX(+1, 0, +1);       //LEFT
		motor[3] = PIDMIX(0, -1, -1);       //FRONT
		break;

	case MULTITYPE_QUADX:
		motor[0] = PIDMIX(-1, +1, -1);      //REAR_R
		motor[1] = PIDMIX(-1, -1, +1);      //FRONT_R
		motor[2] = PIDMIX(+1, +1, +1);      //REAR_L
		motor[3] = PIDMIX(+1, -1, -1);      //FRONT_L
		break;

	case MULTITYPE_Y4:
		motor[0] = PIDMIX(+0, +1, -1);      //REAR_1 CW
		motor[1] = PIDMIX(-1, -1, 0);       //FRONT_R CCW
		motor[2] = PIDMIX(+0, +1, +1);      //REAR_2 CCW
		motor[3] = PIDMIX(+1, -1, 0);       //FRONT_L CW
		break;

	case MULTITYPE_Y6:
		motor[0] = PIDMIX(+0, +4 / 3, +1);  //REAR
		motor[1] = PIDMIX(-1, -2 / 3, -1);  //RIGHT
		motor[2] = PIDMIX(+1, -2 / 3, -1);  //LEFT
		motor[3] = PIDMIX(+0, +4 / 3, -1);  //UNDER_REAR
		motor[4] = PIDMIX(-1, -2 / 3, +1);  //UNDER_RIGHT
		motor[5] = PIDMIX(+1, -2 / 3, +1);  //UNDER_LEFT
		break;

	case MULTITYPE_HEX6:
		motor[0] = PIDMIX(-1 / 2, +1 / 2, +1);      //REAR_R
		motor[1] = PIDMIX(-1 / 2, -1 / 2, -1);      //FRONT_R
		motor[2] = PIDMIX(+1 / 2, +1 / 2, +1);      //REAR_L
		motor[3] = PIDMIX(+1 / 2, -1 / 2, -1);      //FRONT_L
		motor[4] = PIDMIX(+0, -1, +1);      //FRONT
		motor[5] = PIDMIX(+0, +1, -1);      //REAR
		break;

	case MULTITYPE_HEX6X:
		motor[0] = PIDMIX(-1 / 2, +1 / 2, +1);      //REAR_R
		motor[1] = PIDMIX(-1 / 2, -1 / 2, +1);      //FRONT_R
		motor[2] = PIDMIX(+1 / 2, +1 / 2, -1);      //REAR_L
		motor[3] = PIDMIX(+1 / 2, -1 / 2, -1);      //FRONT_L
		motor[4] = PIDMIX(-1, +0, -1);      //RIGHT
		motor[5] = PIDMIX(+1, +0, +1);      //LEFT
		break;

	case MULTITYPE_OCTOX8:
		motor[0] = PIDMIX(-1, +1, -1);      //REAR_R
		motor[1] = PIDMIX(-1, -1, +1);      //FRONT_R
		motor[2] = PIDMIX(+1, +1, +1);      //REAR_L
		motor[3] = PIDMIX(+1, -1, -1);      //FRONT_L
		motor[4] = PIDMIX(-1, +1, +1);      //UNDER_REAR_R
		motor[5] = PIDMIX(-1, -1, -1);      //UNDER_FRONT_R
		motor[6] = PIDMIX(+1, +1, -1);      //UNDER_REAR_L
		motor[7] = PIDMIX(+1, -1, +1);      //UNDER_FRONT_L
		break;

	case MULTITYPE_OCTOFLATP:
		motor[0] = PIDMIX(+7 / 10, -7 / 10, +1);    //FRONT_L
		motor[1] = PIDMIX(-7 / 10, -7 / 10, +1);    //FRONT_R
		motor[2] = PIDMIX(-7 / 10, +7 / 10, +1);    //REAR_R
		motor[3] = PIDMIX(+7 / 10, +7 / 10, +1);    //REAR_L
		motor[4] = PIDMIX(+0, -1, -1);      //FRONT
		motor[5] = PIDMIX(-1, +0, -1);      //RIGHT
		motor[6] = PIDMIX(+0, +1, -1);      //REAR
		motor[7] = PIDMIX(+1, +0, -1);      //LEFT
		break;

	case MULTITYPE_OCTOFLATX:
		motor[0] = PIDMIX(+1, -1 / 2, +1);  //MIDFRONT_L
		motor[1] = PIDMIX(-1 / 2, -1, +1);  //FRONT_R
		motor[2] = PIDMIX(-1, +1 / 2, +1);  //MIDREAR_R
		motor[3] = PIDMIX(+1 / 2, +1, +1);  //REAR_L
		motor[4] = PIDMIX(+1 / 2, -1, -1);  //FRONT_L
		motor[5] = PIDMIX(-1, -1 / 2, -1);  //MIDFRONT_R
		motor[6] = PIDMIX(-1 / 2, +1, -1);  //REAR_R
		motor[7] = PIDMIX(+1, +1 / 2, -1);  //MIDREAR_L
		break;
	case MULTITYPE_VTAIL4:
		motor[0] = PIDMIX(+0,+1, -1/2); //REAR_R
		motor[1] = PIDMIX(-1, -1, +0);  //FRONT_R
		motor[2] = PIDMIX(+0,+1, +1/2); //REAR_L
		motor[3] = PIDMIX(+1, -1, -0);  //FRONT_L
		break;

	case MULTITYPE_GIMBAL:
		servo[1] = constrain(TILT_PITCH_MIDDLE + cfg.gimbal_gain_pitch * angle[PITCH] / 16 + rcCommand[PITCH], TILT_PITCH_MIN, TILT_PITCH_MAX);
		servo[2] = constrain(TILT_ROLL_MIDDLE + cfg.gimbal_gain_roll * angle[ROLL] / 16 + rcCommand[ROLL], TILT_ROLL_MIN, TILT_ROLL_MAX);
		break;

	case MULTITYPE_AIRPLANE:

		//servo endpoints Airplane.

		for(int i=0; i<8; i++){  //  Set rates with 0 - 100%.
			servoMid[i]     =MIDRC + Mid[i];
			servoLimit[i][0]=servoMid[i]-((servoMid[i]-SERVO_MIN)   *(servoTravel[i]*0.01));
			servoLimit[i][1]=servoMid[i]+((SERVO_MAX - servoMid[i]) *(servoTravel[i]*0.01));
		}

		// servo[7] is programmed with safty features to avoid motorstarts when ardu reset..
		// All other servos go to center at reset..  Half throttle can be dangerus
		// Only use servo[7] as motorcontrol if motor is used in the setup            */
		if (!armed){
			servo[7] =  MINCOMMAND; // Kill throttle when disarmed
		} else {
			servo[7] =  rcData[THROTTLE];
		}


		if ( cfg.flap_channel ){
			int8_t flapinv[2] = FLAP_INVERT;
			static int16_t F_Endpoint[2] = FLAP_EP;
			int16_t flap = (MIDRC- constrain(rcData[FLAP_CHANNEL],F_Endpoint[0],F_Endpoint[1]));
			for(int i=0; i<2; i++){flaps[i] = flap * flapinv[i] ;}
		}
		if(passThruMode){   // Direct passthru from RX
			servo[3]  = servoMid[3]+((rcCommand[ROLL] + flaps[0]) *servoReverse[3]);     //   Wing 1
			servo[4]  = servoMid[4]+((rcCommand[ROLL] + flaps[1]) *servoReverse[4]);     //   Wing 2
			servo[5]  = servoMid[5]+(rcCommand[YAW]               *servoReverse[5]);     //   Rudder
			servo[6]  = servoMid[6]+(rcCommand[PITCH]             *servoReverse[6]);     //   Elevator
		}else{
			// Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
			servo[3]  =(servoMid[3] + ((axisPID[ROLL] + flaps[0]) *servoReverse[3]));   //   Wing 1
			servo[4]  =(servoMid[4] + ((axisPID[ROLL] + flaps[1]) *servoReverse[4]));   //   Wing 2
			servo[5]  =(servoMid[5] + (axisPID[YAW]               *servoReverse[5]));   //   Rudder
			servo[6]  =(servoMid[6] + (axisPID[PITCH]             *servoReverse[6]));   //   Elevator
		}
		// ServoRates
		for(uint8_t i=3;i<8;i++){
			servo[i]  = map(servo[i], SERVO_MIN, SERVO_MAX,servoLimit[i][0],servoLimit[i][1]);
			servo[i]  = constrain( servo[i], SERVO_MIN, SERVO_MAX);
		}
		break;
	case MULTITYPE_FLYING_WING:
		motor[0] = rcCommand[THROTTLE];
		if (passThruMode) {// do not use sensors for correction, simple 2 channel mixing
			servo[0]  = PITCH_DIRECTION_L * (rcData[PITCH]-MIDRC) + ROLL_DIRECTION_L * (rcData[ROLL]-MIDRC);
			servo[1]  = PITCH_DIRECTION_R * (rcData[PITCH]-MIDRC) + ROLL_DIRECTION_R * (rcData[ROLL]-MIDRC);
		} else { // use sensors to correct (gyro only or gyro+acc according to aux1/aux2 configuration
			servo[0]  = PITCH_DIRECTION_L * axisPID[PITCH]        + ROLL_DIRECTION_L * axisPID[ROLL];
			servo[1]  = PITCH_DIRECTION_R * axisPID[PITCH]        + ROLL_DIRECTION_R * axisPID[ROLL];
		}
		servo[0]  = constrain(servo[0] + wing_left_mid , WING_LEFT_MIN,  WING_LEFT_MAX );
		servo[1]  = constrain(servo[1] + wing_right_mid, WING_RIGHT_MIN, WING_RIGHT_MAX);
		break;
	case MULTITYPE_HELI_90_DEG:
		output_mix_heli();
		servo[3]  = HeliXPIDMIX( +0, +1, -0) ;      //     NICK  servo
		servo[4]  = HeliXPIDMIX( +0, +0, +1) ;      //     ROLL servo
		servo[6]  = HeliXPIDMIX( +1, +0, +0) ;      //     COLLECTIVE  servo
		for(uint8_t i=3;i<8;i++){
			servo[i]  = constrain( servo[i], servoEndpiont[i][0], servoEndpiont[i][1] );
		}
		break;
	case MULTITYPE_HELI_120_CCPM:
		output_mix_heli();
		servo[3]  =  HeliXPIDMIX( +1, -1  , +0) ;   //    NICK  servo
		servo[4]  =  HeliXPIDMIX( +1, +1/2, +1) ;   //    LEFT servo
		servo[6]  =  HeliXPIDMIX( +1, +1/2, -1) ;   //    RIGHT  servo
		for(uint8_t i=3;i<8;i++){
			servo[i]  = constrain( servo[i], servoEndpiont[i][0], servoEndpiont[i][1] );
		}
		break;

	}


	/****************                Cam stabilize Sevos             ******************/
#if defined(SERVO_TILT)

	servo[2] = TILT_PITCH_MIDDLE + rcData[AUX3]-1500;
	servo[3]  = TILT_ROLL_MIDDLE  + rcData[AUX4]-1500;
	if (rcOptions[BOXCAMSTAB]) {
		servo[2] += TILT_PITCH_PROP * angle[PITCH] /16 ;
		servo[3]  += TILT_ROLL_PROP  * angle[ROLL]  /16 ;
	}
	servo[2] = constrain(servo[2], TILT_PITCH_MIN, TILT_PITCH_MAX);
	servo[3]  = constrain(servo[3] , TILT_ROLL_MIN, TILT_ROLL_MAX  );
#endif


	/************************************************************************************************************/
	// Bledi Experimentals
	/************************************************************************************************************/
#ifdef SERVO_MIX_TILT
	// Simple CameraGimbal By Bledy http://youtu.be/zKGr6iR54vM
	if (rcOptions[BOXCAMSTAB]) {
		servo[0] = constrain(TILT_PITCH_MIDDLE - (-TILT_ROLL_PROP) * angle[PITCH] /16 - TILT_ROLL_PROP * angle[ROLL] /16 , TILT_PITCH_MIN, TILT_PITCH_MAX);
		servo[1] = constrain(TILT_ROLL_MIDDLE + (-TILT_ROLL_PROP) * angle[PITCH] /16 - TILT_ROLL_PROP * angle[ROLL] /16 , TILT_ROLL_MIN, TILT_ROLL_MAX);
	} else {

		servo[0] = constrain(TILT_PITCH_MIDDLE  + rcData[AUX3]-1500 , TILT_PITCH_MIN, TILT_PITCH_MAX);
		servo[1] = constrain(TILT_ROLL_MIDDLE   + rcData[AUX4]-1500,  TILT_ROLL_MIN, TILT_ROLL_MAX);
	}
#endif
	/************************************************************************************************************/
	// End of Bledi Experimentals
	/************************************************************************************************************/



	/****************                    Cam trigger Sevo                ******************/
#if defined(CAMTRIG)
	static uint8_t camCycle = 0;
	static uint8_t camState = 0;
	static uint32_t camTime = 0;
	if (camCycle==1) {
		if (camState == 0) {
			servo[2] = CAM_SERVO_HIGH;
			camState = 1;
			camTime = millis();
		} else if (camState == 1) {
			if ( (millis() - camTime) > CAM_TIME_HIGH ) {
				servo[2] = CAM_SERVO_LOW;
				camState = 2;
				camTime = millis();
			}
		} else { //camState ==2
			if ( (millis() - camTime) > CAM_TIME_LOW ) {
				camState = 0;
				camCycle = 0;
			}
		}
	}
	if (rcOptions[BOXCAMTRIG]) camCycle=1;
#endif


	/****************                Filter the Motors values                ******************/
	maxMotor=motor[0];
	for(int i=1;i< numberMotor;i++)
		if (motor[i]>maxMotor) maxMotor=motor[i];

	for (int i = 0; i < numberMotor; i++) {
		if (maxMotor > MAXTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
			motor[i] -= maxMotor - MAXTHROTTLE;
		motor[i] = constrain(motor[i], MINTHROTTLE, MAXTHROTTLE);
		if ((rcData[THROTTLE]) < MINCHECK){

			motor[i] = cfg.motor_disarmed_value;



		}
		if (armed == 0)
			motor[i] = MINCOMMAND;
	}
	output_servo_write();
	output_motor_write();
}





