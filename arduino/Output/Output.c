
/*
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version. see <http://www.gnu.org/licenses/>
 * 
 * Simple i2c Output slave for the linux i2c driver example mi2c.
 *
 */

#include <Wire.h>

#include "def.h"
/* this is the value for the ESCs when they are not armed
   in some cases, this value must be lowered down to 900 for some specific ESCs */
#define MINCOMMAND 1000




#define NUMBER_MOTOR 8

#define I2C_SLAVE_ADDRESS 0x10
#define BUFFER_SIZE 128

uint8_t i2c_buffer[BUFFER_SIZE];
int cmd;

/**************************************************************************************/
/***************                  Motor Pin order                  ********************/
/**************************************************************************************/
// since we are uing the PWM generation in a direct way, the pin order is just to inizialie the right pins 
// its not possible to change a PWM output pin just by changing the order
#if defined(PROMINI)
uint8_t PWM_PIN[8] = {9,10,11,3,6,5,A2,12};   //for a quad+: rear,right,left,front
#endif
#if defined(PROMICRO)
#if !defined(HWPWM6)
#if !defined(TEENSY20)
uint8_t PWM_PIN[8] = {9,10,5,6,4,A2,A0,A1};   //for a quad+: rear,right,left,front
#else
uint8_t PWM_PIN[8] = {14,15,9,12,22,18,16,17};   //for a quad+: rear,right,left,front
#endif
#else
#if !defined(TEENSY20)
uint8_t PWM_PIN[8] = {9,10,5,6,11,13,A0,A1};   //for a quad+: rear,right,left,front
#else
uint8_t PWM_PIN[8] = {14,15,9,12,4,10,16,17};   //for a quad+: rear,right,left,front
#endif
#endif
#endif
#if defined(MEGA)
uint8_t PWM_PIN[8] = {3,5,6,2,7,8,9,10};      //for a quad+: rear,right,left,front   //+ for y6: 7:under right  8:under left
#endif

/**************************************************************************************/
/***************         Software PWM & Servo variables            ********************/
/**************************************************************************************/
#if !defined(PROMICRO) || defined(HWPWM6)
#if defined(SERVO)
#if defined(AIRPLANE)|| defined(HELICOPTER)
// To prevent motor to start at reset. atomicServo[7]=5 or 249 if reversed servo
volatile uint8_t atomicServo[8] = {125,125,125,125,125,125,125,5};
#else
volatile uint8_t atomicServo[8] = {125,125,125,125,125,125,125,125};
#endif
#endif
#if (NUMBER_MOTOR > 4)
//for HEX Y6 and HEX6/HEX6X flat for promini
volatile uint8_t atomicPWM_PIN5_lowState;
volatile uint8_t atomicPWM_PIN5_highState;
volatile uint8_t atomicPWM_PIN6_lowState;
volatile uint8_t atomicPWM_PIN6_highState;
#endif
#if (NUMBER_MOTOR > 6)
//for OCTO on promini
volatile uint8_t atomicPWM_PINA2_lowState;
volatile uint8_t atomicPWM_PINA2_highState;
volatile uint8_t atomicPWM_PIN12_lowState;
volatile uint8_t atomicPWM_PIN12_highState;
#endif
#else
#if defined(SERVO)
volatile uint16_t atomicServo[8] = {8000,8000,8000,8000,8000,8000,8000,8000};
#endif
#if (NUMBER_MOTOR > 4)
//for HEX Y6 and HEX6/HEX6X and for Promicro
volatile uint16_t atomicPWM_PIN5_lowState;
volatile uint16_t atomicPWM_PIN5_highState;
volatile uint16_t atomicPWM_PIN6_lowState;
volatile uint16_t atomicPWM_PIN6_highState;
#endif
#if (NUMBER_MOTOR > 6)
//for OCTO on Promicro
volatile uint16_t atomicPWM_PINA2_lowState;
volatile uint16_t atomicPWM_PINA2_highState;
volatile uint16_t atomicPWM_PIN12_lowState;
volatile uint16_t atomicPWM_PIN12_highState;
#endif
#endif

/**************************************************************************************/
/***************   Writes the Servos values to the needed format   ********************/
/**************************************************************************************/
void writeServos() {

	for(uint8_t i = 0; i < 2; i++){
#if !defined(PROMICRO) || defined(HWPWM6)
		atomicServo[i] = (servo[i]-1000)>>2;
#else
		atomicServo[i] = (servo[i]-1000)<<4;
#endif
	}

}

/**************************************************************************************/
/************  Writes the Motors values to the PWM compare register  ******************/
/**************************************************************************************/
void writeMotors() { // [1000;2000] => [125;250]

	/****************  Specific PWM Timers & Registers for the MEGA's   *******************/
#if defined(MEGA)// [1000:2000] => [8000:16000] for timer 3 & 4 for mega
#if (NUMBER_MOTOR > 0)
#ifndef EXT_MOTOR_RANGE
	OCR3C = motor[0]<<3; //  pin 3
#else
	OCR3C = ((motor[0]<<4) - 16000) + 128;
#endif
#endif
#if (NUMBER_MOTOR > 1)
#ifndef EXT_MOTOR_RANGE
	OCR3A = motor[1]<<3; //  pin 5
#else
	OCR3A = ((motor[1]<<4) - 16000) + 128;
#endif
#endif
#if (NUMBER_MOTOR > 2)
#ifndef EXT_MOTOR_RANGE
	OCR4A = motor[2]<<3; //  pin 6
#else
	OCR4A = ((motor[2]<<4) - 16000) + 128;
#endif
#endif
#if (NUMBER_MOTOR > 3)
#ifndef EXT_MOTOR_RANGE
	OCR3B = motor[3]<<3; //  pin 2
#else
	OCR3B = ((motor[3]<<4) - 16000) + 128;
#endif
#endif
#if (NUMBER_MOTOR > 4)
#ifndef EXT_MOTOR_RANGE
	OCR4B = motor[4]<<3; //  pin 7
	OCR4C = motor[5]<<3; //  pin 8
#else
	OCR4B = ((motor[4]<<4) - 16000) + 128;
	OCR4C = ((motor[5]<<4) - 16000) + 128;
#endif
#endif
#if (NUMBER_MOTOR > 6)
#ifndef EXT_MOTOR_RANGE
	OCR2B = motor[6]>>3; //  pin 9
	OCR2A = motor[7]>>3; //  pin 10
#else
	OCR2B = ((motor[6]>>2) - 250) + 2);
	OCR2A = ((motor[7]>>2) - 250) + 2);
#endif
#endif
#endif

	/******** Specific PWM Timers & Registers for the atmega32u4 (Promicro)   ************/
#if defined(PROMICRO)
#if (NUMBER_MOTOR > 0) // Timer 1 A & B [1000:2000] => [8000:16000]
	OCR1A = motor[0]<<3; //  pin 9
#endif
#if (NUMBER_MOTOR > 1)
	OCR1B = motor[1]<<3; //  pin 10
#endif
#if (NUMBER_MOTOR > 2) // Timer 4 A & D [1000:2000] => [1000:2000]
#if !defined(HWPWM6)
	// to write values > 255 to timer 4 A/B we need to split the bytes
	static uint8_t pwm4_HBA;
	static uint16_t pwm4_LBA; // high and low byte for timer 4 A
	pwm4_LBA = 2047-motor[2]; pwm4_HBA = 0; // channel A is inverted
	while(pwm4_LBA > 255){
		pwm4_HBA++;
		pwm4_LBA-=256;
	}
	TC4H = pwm4_HBA; OCR4A = pwm4_LBA; //  pin 5
#else
	OCR3A = motor[2]<<3; //  pin 5
#endif
#endif
#if (NUMBER_MOTOR > 3)
	static uint8_t pwm4_HBD;
	static uint16_t pwm4_LBD; // high and low byte for timer 4 D
	pwm4_LBD = motor[3]; pwm4_HBD = 0;
	while(pwm4_LBD > 255){
		pwm4_HBD++;
		pwm4_LBD-=256;
	}
	TC4H = pwm4_HBD; OCR4D = pwm4_LBD; //  pin 6
#endif
#if (NUMBER_MOTOR > 4)
#if !defined(HWPWM6)
#if (NUMBER_MOTOR == 6) && !defined(SERVO)
	atomicPWM_PIN5_highState = motor[4]<<3;
	atomicPWM_PIN5_lowState = 16383-atomicPWM_PIN5_highState;
	atomicPWM_PIN6_highState = motor[5]<<3;
	atomicPWM_PIN6_lowState = 16383-atomicPWM_PIN6_highState;
#else
	atomicPWM_PIN5_highState = ((motor[4]-1000)<<4)+320;
	atomicPWM_PIN5_lowState = 15743-atomicPWM_PIN5_highState;
	atomicPWM_PIN6_highState = ((motor[5]-1000)<<4)+320;
	atomicPWM_PIN6_lowState = 15743-atomicPWM_PIN6_highState;
#endif
#else
	OCR1C = motor[4]<<3; //  pin 11
	static uint8_t pwm4_HBA;
	static uint16_t pwm4_LBA; // high and low byte for timer 4 A
	pwm4_LBA = motor[5]; pwm4_HBA = 0;
	while(pwm4_LBA > 255){
		pwm4_HBA++;
		pwm4_LBA-=256;
	}
	TC4H = pwm4_HBA; OCR4A = pwm4_LBA; //  pin 13
#endif
#endif
#if (NUMBER_MOTOR > 6)
#if !defined(HWPWM6)
	atomicPWM_PINA2_highState = ((motor[6]-1000)<<4)+320;
	atomicPWM_PINA2_lowState = 15743-atomicPWM_PINA2_highState;
	atomicPWM_PIN12_highState = ((motor[7]-1000)<<4)+320;
	atomicPWM_PIN12_lowState = 15743-atomicPWM_PIN12_highState;
#else
	atomicPWM_PINA2_highState = ((motor[6]-1000)>>2)+5;
	atomicPWM_PINA2_lowState = 245-atomicPWM_PINA2_highState;
	atomicPWM_PIN12_highState = ((motor[7]-1000)>>2)+5;
	atomicPWM_PIN12_lowState = 245-atomicPWM_PIN12_highState;
#endif
#endif
#endif

	/********  Specific PWM Timers & Registers for the atmega328P (Promini)   ************/
#if defined(PROMINI)
#if (F_CPU > 8000000L)
#if (NUMBER_MOTOR > 0)
#ifndef EXT_MOTOR_RANGE
	OCR1A = motor[0]>>3; //  pin 9
#else
	OCR1A = ((motor[0]>>2) - 250) + 2;
#endif
#endif
#if (NUMBER_MOTOR > 1)
#ifndef EXT_MOTOR_RANGE
	OCR1B = motor[1]>>3; //  pin 10
#else
	OCR1B = ((motor[1]>>2) - 250) + 2;
#endif
#endif
#if (NUMBER_MOTOR > 2)
#ifndef EXT_MOTOR_RANGE
	OCR2A = motor[2]>>3; //  pin 11
#else
	OCR2A = ((motor[2]>>2) - 250) + 2;
#endif
#endif
#if (NUMBER_MOTOR > 3)
#ifndef EXT_MOTOR_RANGE
	OCR2B = motor[3]>>3; //  pin 3
#else
	OCR2B = ((motor[3]>>2) - 250) + 2;
#endif
#endif
#if (NUMBER_MOTOR > 4)
#if (NUMBER_MOTOR == 6) && !defined(SERVO)
#ifndef EXT_MOTOR_RANGE
	atomicPWM_PIN6_highState = motor[4]>>3;
	atomicPWM_PIN5_highState = motor[5]>>3;
#else
	atomicPWM_PIN6_highState = ((motor[4]>>2) - 250) + 2;
	atomicPWM_PIN5_highState = ((motor[5]>>2) - 250) + 2;
#endif
	atomicPWM_PIN6_lowState  = 255-atomicPWM_PIN6_highState;
	atomicPWM_PIN5_lowState  = 255-atomicPWM_PIN5_highState;
#else //note: EXT_MOTOR_RANGE not possible here
	atomicPWM_PIN6_highState = ((motor[4]-1000)>>2)+5;
	atomicPWM_PIN6_lowState  = 245-atomicPWM_PIN6_highState;
	atomicPWM_PIN5_highState = ((motor[5]-1000)>>2)+100;
	atomicPWM_PIN5_lowState  = 245-atomicPWM_PIN5_highState;
#endif
#endif
#if (NUMBER_MOTOR > 6) //note: EXT_MOTOR_RANGE not possible here
	atomicPWM_PINA2_highState = ((motor[6]-1000)>>2)+5;
	atomicPWM_PINA2_lowState  = 245-atomicPWM_PINA2_highState;
	atomicPWM_PIN12_highState = ((motor[7]-1000)>>2)+5;
	atomicPWM_PIN12_lowState  = 245-atomicPWM_PIN12_highState;
#endif

#else // 8mhz

#if (NUMBER_MOTOR > 0) // Timer 1 A & B [1000:2000] => [8000:16000]
	OCR1A = motor[0]<<2; //  pin 9
#endif
#if (NUMBER_MOTOR > 1)
	OCR1B = motor[1]<<2; //  pin 10
#endif
#if (NUMBER_MOTOR > 2)
#ifndef EXT_MOTOR_RANGE
	OCR2A = motor[2]>>3; //  pin 11
#else
	OCR2A = ((motor[2]>>2) - 250) + 2;
#endif
#endif
#if (NUMBER_MOTOR > 3)
#ifndef EXT_MOTOR_RANGE
	OCR2B = motor[3]>>3; //  pin 3
#else
	OCR2B = ((motor[3]>>2) - 250) + 2;
#endif
#endif
#if (NUMBER_MOTOR > 4) //note: EXT_MOTOR_RANGE not possible here
	atomicPWM_PIN6_highState = ((motor[4]-1000)>>3)+3 ;
	atomicPWM_PIN6_lowState  = 122-atomicPWM_PIN6_highState;
	atomicPWM_PIN5_highState = ((motor[5]-1000)>>3)+3;
	atomicPWM_PIN5_lowState  = 122-atomicPWM_PIN5_highState;
#endif
#if (NUMBER_MOTOR > 6) //note: EXT_MOTOR_RANGE not possible here
	atomicPWM_PINA2_highState = ((motor[6]-1000)>>3)+3;
	atomicPWM_PINA2_lowState  = 122-atomicPWM_PINA2_highState;
	atomicPWM_PIN12_highState = ((motor[7]-1000)>>3)+3;
	atomicPWM_PIN12_lowState  = 122-atomicPWM_PIN12_highState;
#endif
#endif
#endif
}

/**************************************************************************************/
/************          Writes the mincommand to all Motors           ******************/
/**************************************************************************************/
void writeAllMotors(int16_t mc) {   // Sends commands to all motors
	for (uint8_t i =0;i<NUMBER_MOTOR;i++)
		motor[i]=mc;
	writeMotors();
}

/**************************************************************************************/
/************        Initialize the PWM Timers and Registers         ******************/
/**************************************************************************************/
void initOutput() {

	/****************            mark all PWM pins as Output             ******************/
	for(uint8_t i=0;i<NUMBER_MOTOR;i++)
		pinMode(PWM_PIN[i],OUTPUT);

	/****************  Specific PWM Timers & Registers for the MEGA's    ******************/
#if defined(MEGA)
#if (NUMBER_MOTOR > 0)
	// init 16bit timer 3
	TCCR3A |= (1<<WGM31); // phase correct mode
	TCCR3A &= ~(1<<WGM30);
	TCCR3B |= (1<<WGM33);
	TCCR3B &= ~(1<<CS31); // no prescaler
	ICR3   |= 0x3FFF; // TOP to 16383;

	TCCR3A |= _BV(COM3C1); // connect pin 3 to timer 3 channel C
#endif
#if (NUMBER_MOTOR > 1)
	TCCR3A |= _BV(COM3A1); // connect pin 5 to timer 3 channel A
#endif
#if (NUMBER_MOTOR > 2)
	// init 16bit timer 4
	TCCR4A |= (1<<WGM41); // phase correct mode
	TCCR4A &= ~(1<<WGM40);
	TCCR4B |= (1<<WGM43);
	TCCR4B &= ~(1<<CS41); // no prescaler
	ICR4   |= 0x3FFF; // TOP to 16383;

	TCCR4A |= _BV(COM4A1); // connect pin 6 to timer 4 channel A
#endif
#if (NUMBER_MOTOR > 3)
	TCCR3A |= _BV(COM3B1); // connect pin 2 to timer 3 channel B
#endif
#if (NUMBER_MOTOR > 4)
	TCCR4A |= _BV(COM4B1); // connect pin 7 to timer 4 channel B
	TCCR4A |= _BV(COM4C1); // connect pin 8 to timer 4 channel C
#endif
#if (NUMBER_MOTOR > 6)
	// timer 2 is a 8bit timer so we cant change its range
	TCCR2A |= _BV(COM2B1); // connect pin 9 to timer 2 channel B
	TCCR2A |= _BV(COM2A1); // connect pin 10 to timer 2 channel A
#endif
#endif

	/******** Specific PWM Timers & Registers for the atmega32u4 (Promicro)   ************/
#if defined(PROMICRO)
#if (NUMBER_MOTOR > 0)
	TCCR1A |= (1<<WGM11); // phase correct mode & no prescaler
	TCCR1A &= ~(1<<WGM10);
	TCCR1B &= ~(1<<WGM12) &  ~(1<<CS11) & ~(1<<CS12);
	TCCR1B |= (1<<WGM13) | (1<<CS10);
	ICR1   |= 0x3FFF; // TOP to 16383;
	TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
#endif
#if (NUMBER_MOTOR > 1)
	TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
#endif
#if (NUMBER_MOTOR > 2)
#if !defined(HWPWM6) // timer 4A
	TCCR4E |= (1<<ENHC4); // enhanced pwm mode
	TCCR4B &= ~(1<<CS41); TCCR4B |= (1<<CS42)|(1<<CS40); // prescaler to 16
	TCCR4D |= (1<<WGM40); TC4H = 0x3; OCR4C = 0xFF; // phase and frequency correct mode & top to 1023 but with enhanced pwm mode we have 2047
	TCCR4A |= (1<<COM4A0)|(1<<PWM4A); // connect pin 5 to timer 4 channel A
#else // timer 3A
	TCCR3A |= (1<<WGM31); // phase correct mode & no prescaler
	TCCR3A &= ~(1<<WGM30);
	TCCR3B &= ~(1<<WGM32) &  ~(1<<CS31) & ~(1<<CS32);
	TCCR3B |= (1<<WGM33) | (1<<CS30);
	ICR3   |= 0x3FFF; // TOP to 16383;
	TCCR3A |= _BV(COM3A1); // connect pin 5 to timer 3 channel A
#endif
#endif
#if (NUMBER_MOTOR > 3)
#if defined(HWPWM6)
	TCCR4E |= (1<<ENHC4); // enhanced pwm mode
	TCCR4B &= ~(1<<CS41); TCCR4B |= (1<<CS42)|(1<<CS40); // prescaler to 16
	TCCR4D |= (1<<WGM40); TC4H = 0x3; OCR4C = 0xFF; // phase and frequency correct mode & top to 1023 but with enhanced pwm mode we have 2047
#endif
	TCCR4C |= (1<<COM4D1)|(1<<PWM4D); // connect pin 6 to timer 4 channel D
#endif
#if (NUMBER_MOTOR > 4)
#if defined(HWPWM6)
	TCCR1A |= _BV(COM1C1); // connect pin 11 to timer 1 channel C
	TCCR4A |= (1<<COM4A1)|(1<<PWM4A); // connect pin 13 to timer 4 channel A
#else
	initializeSoftPWM();
#endif
#endif
#if (NUMBER_MOTOR > 6)
#if defined(HWPWM6)
	initializeSoftPWM();
#endif
#endif
#endif

	/********  Specific PWM Timers & Registers for the atmega328P (Promini)   ************/
#if defined(PROMINI)
#if (F_CPU > 8000000L)
#if (NUMBER_MOTOR > 0)
	TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
#endif
#if (NUMBER_MOTOR > 1)
	TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
#endif
#if (NUMBER_MOTOR > 2)
	TCCR2A |= _BV(COM2A1); // connect pin 11 to timer 2 channel A
#endif
#if (NUMBER_MOTOR > 3)
	TCCR2A |= _BV(COM2B1); // connect pin 3 to timer 2 channel B
#endif
#if (NUMBER_MOTOR > 5)  // PIN 5 & 6 or A0 & A1
	initializeSoftPWM();
#if defined(A0_A1_PIN_HEX) || (NUMBER_MOTOR > 6)
	pinMode(5,INPUT);pinMode(6,INPUT);     // we reactivate the INPUT affectation for these two PINs
	pinMode(A0,OUTPUT);pinMode(A1,OUTPUT);
#endif
#endif

#else
#if (NUMBER_MOTOR > 0)

	TCCR1A |= (1<<WGM11); TCCR1A &= ~(1<<WGM10); TCCR1B |= (1<<WGM13);  // phase correct mode
	TCCR1B &= ~(1<<CS11); ICR1 |= 0x2000; // no prescaler & TOP to 16383;
	TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
#endif
#if (NUMBER_MOTOR > 1)
	TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
#endif
#if (NUMBER_MOTOR > 2)
	TCCR2A |= _BV(COM2A1) | _BV(CS21) | _BV(CS20); // connect pin 11 to timer 2 channel A prescale32
#endif
#if (NUMBER_MOTOR > 3)
	TCCR2A |= _BV(COM2B1) ; // connect pin 3 to timer 2 channel B
#endif
#if (NUMBER_MOTOR > 5)  // PIN 5 & 6 or A0 & A1
	initializeSoftPWM();
#if defined(A0_A1_PIN_HEX) || (NUMBER_MOTOR > 6)
	pinMode(5,INPUT);pinMode(6,INPUT);     // we reactivate the INPUT affectation for these two PINs
	pinMode(A0,OUTPUT);pinMode(A1,OUTPUT);
#endif
#endif

#endif

#endif

	writeAllMotors(MINCOMMAND);
	delay(300);
#if defined(SERVO)
	initializeServo();
#endif
}

#if defined(SERVO)
/**************************************************************************************/
/************                Initialize the PWM Servos               ******************/
/**************************************************************************************/
void initializeServo() {
#if (PRI_SERVO_FROM == 1) || (SEC_SERVO_FROM == 1)
	SERVO_1_PINMODE;
#endif
#if (PRI_SERVO_FROM <= 2 && PRI_SERVO_TO >= 2) || (SEC_SERVO_FROM <= 2 && SEC_SERVO_TO >= 2)
	SERVO_2_PINMODE;
#endif
#if (PRI_SERVO_FROM <= 3 && PRI_SERVO_TO >= 3) || (SEC_SERVO_FROM <= 3 && SEC_SERVO_TO >= 3)
	SERVO_3_PINMODE;
#endif
#if (PRI_SERVO_FROM <= 4 && PRI_SERVO_TO >= 4) || (SEC_SERVO_FROM <= 4 && SEC_SERVO_TO >= 4)
	SERVO_4_PINMODE;
#endif
#if (PRI_SERVO_FROM <= 5 && PRI_SERVO_TO >= 5) || (SEC_SERVO_FROM <= 5 && SEC_SERVO_TO >= 5)
	SERVO_5_PINMODE;
#endif
#if (PRI_SERVO_FROM <= 6 && PRI_SERVO_TO >= 6) || (SEC_SERVO_FROM <= 6 && SEC_SERVO_TO >= 6)
	SERVO_6_PINMODE;
#endif
#if (PRI_SERVO_FROM <= 7 && PRI_SERVO_TO >= 7) || (SEC_SERVO_FROM <= 7 && SEC_SERVO_TO >= 7)
	SERVO_7_PINMODE;
#endif
#if (PRI_SERVO_FROM <= 8 && PRI_SERVO_TO >= 8) || (SEC_SERVO_FROM <= 8 && SEC_SERVO_TO >= 8)
	SERVO_8_PINMODE;
#endif

#if !defined(PROMICRO) || defined(HWPWM6)
	TCCR0A = 0; // normal counting mode
	TIMSK0 |= (1<<OCIE0A); // Enable CTC interrupt
#else
	TCCR3A &= ~(1<<WGM30) & ~(1<<WGM31); //normal counting & no prescaler
	TCCR3B &= ~(1<<WGM32) & ~(1<<CS31) & ~(1<<CS32) & ~(1<<WGM33);
	TCCR3B |= (1<<CS30);
	TIMSK3 |= (1<<OCIE3A); // Enable CTC interrupt
#endif
}

/**************************************************************************************/
/************              Servo software PWM generation             ******************/
/**************************************************************************************/
#if !defined(PROMICRO) || defined(HWPWM6)
#define SERVO_ISR TIMER0_COMPA_vect
#define SERVO_CHANNEL OCR0A
#if (F_CPU > 8000000L)
#define SERVO_1K_US 250
#else
#define SERVO_1K_US 125
#endif

#else
#define SERVO_ISR TIMER3_COMPA_vect
#define SERVO_CHANNEL OCR3A
#define SERVO_1K_US 16000
#endif

// prescaler is set by default to 64 on Timer0
// Duemilanove : 16MHz / 64 => 4 us
// 256 steps = 1 counter cycle = 1024 us
ISR(SERVO_ISR) {
	static uint8_t state = 0;
	static uint8_t count;
	if (state == 0) {
#if (PRI_SERVO_FROM == 1) || (SEC_SERVO_FROM == 1)
		SERVO_1_PIN_HIGH;
#endif
		SERVO_CHANNEL+= SERVO_1K_US; // 1000 us
		state++ ;
	} else if (state == 1) {
		SERVO_CHANNEL+= atomicServo[0]; // 1000 + [0-1020] us
		state++;
	} else if (state == 2) {
#if (PRI_SERVO_FROM == 1) || (SEC_SERVO_FROM == 1)
		SERVO_1_PIN_LOW;
#endif
#if (PRI_SERVO_FROM <= 2 && PRI_SERVO_TO >= 2) || (SEC_SERVO_FROM <= 2 && SEC_SERVO_TO >= 2)
		SERVO_2_PIN_HIGH;
#endif
		SERVO_CHANNEL+= SERVO_1K_US; // 1000 us
		state++;
	} else if (state == 3) {
		SERVO_CHANNEL+= atomicServo[1]; // 1000 + [0-1020] us
		state++;
	} else if (state == 4) {
#if (PRI_SERVO_FROM <= 2 && PRI_SERVO_TO >= 2) || (SEC_SERVO_FROM <= 2 && SEC_SERVO_TO >= 2)
		SERVO_2_PIN_LOW;
#endif
#if (PRI_SERVO_FROM <= 3 && PRI_SERVO_TO >= 3) || (SEC_SERVO_FROM <= 3 && SEC_SERVO_TO >= 3)
		SERVO_3_PIN_HIGH;
#endif
		state++;
		SERVO_CHANNEL+= SERVO_1K_US; // 1000 us
	} else if (state == 5) {
		SERVO_CHANNEL+= atomicServo[2]; // 1000 + [0-1020] us
		state++;
	} else if (state == 6) {
#if (PRI_SERVO_FROM <= 3 && PRI_SERVO_TO >= 3) || (SEC_SERVO_FROM <= 3 && SEC_SERVO_TO >= 3)
		SERVO_3_PIN_LOW;
#endif
#if (PRI_SERVO_FROM <= 4 && PRI_SERVO_TO >= 4) || (SEC_SERVO_FROM <= 4 && SEC_SERVO_TO >= 4)
		SERVO_4_PIN_HIGH;
#endif
		state++;
		SERVO_CHANNEL+= SERVO_1K_US; // 1000 us
	} else if (state == 7) {
		SERVO_CHANNEL+= atomicServo[3]; // 1000 + [0-1020] us
		state++;
	} else if (state == 8) {
#if (PRI_SERVO_FROM <= 4 && PRI_SERVO_TO >= 4) || (SEC_SERVO_FROM <= 4 && SEC_SERVO_TO >= 4)
		SERVO_4_PIN_LOW;
#endif
#if (PRI_SERVO_FROM <= 5 && PRI_SERVO_TO >= 5) || (SEC_SERVO_FROM <= 5 && SEC_SERVO_TO >= 5)
		SERVO_5_PIN_HIGH;;
#endif
		state++;
		SERVO_CHANNEL+= SERVO_1K_US; // 1000 us
	} else if (state == 9) {
		SERVO_CHANNEL+= atomicServo[4]; // 1000 + [0-1020] us
		state++;
	} else if (state == 10) {
#if (PRI_SERVO_FROM <= 5 && PRI_SERVO_TO >= 5) || (SEC_SERVO_FROM <= 5 && SEC_SERVO_TO >= 5)
		SERVO_5_PIN_LOW;
#endif
#if (PRI_SERVO_FROM <= 6 && PRI_SERVO_TO >= 6) || (SEC_SERVO_FROM <= 6 && SEC_SERVO_TO >= 6)
		SERVO_6_PIN_HIGH;;
#endif
		state++;
		SERVO_CHANNEL+= SERVO_1K_US; // 1000 us
	} else if (state == 11) {
		SERVO_CHANNEL+= atomicServo[5]; // 1000 + [0-1020] us
		state++;
	} else if (state == 12) {
#if (PRI_SERVO_FROM <= 6 && PRI_SERVO_TO >= 6) || (SEC_SERVO_FROM <= 6 && SEC_SERVO_TO >= 6)
		SERVO_6_PIN_LOW;
#endif
#if (PRI_SERVO_FROM <= 7 && PRI_SERVO_TO >= 7) || (SEC_SERVO_FROM <= 7 && SEC_SERVO_TO >= 7)
		SERVO_7_PIN_HIGH;
#endif
		state++;
		SERVO_CHANNEL+= SERVO_1K_US; // 1000 us
	} else if (state == 13) {
		SERVO_CHANNEL+= atomicServo[6]; // 1000 + [0-1020] us
		state++;
	} else if (state == 14) {
#if (PRI_SERVO_FROM <= 7 && PRI_SERVO_TO >= 7) || (SEC_SERVO_FROM <= 7 && SEC_SERVO_TO >= 7)
		SERVO_7_PIN_LOW;
#endif
#if (PRI_SERVO_FROM <= 8 && PRI_SERVO_TO >= 8) || (SEC_SERVO_FROM <= 8 && SEC_SERVO_TO >= 8)
		SERVO_8_PIN_HIGH;
#endif
		state++;
		SERVO_CHANNEL+= SERVO_1K_US; // 1000 us
	} else if (state == 15) {
		SERVO_CHANNEL+= atomicServo[7]; // 1000 + [0-1020] us
		state++;
	} else if (state == 16) {
#if (PRI_SERVO_FROM <= 8 && PRI_SERVO_TO >= 8) || (SEC_SERVO_FROM <= 8 && SEC_SERVO_TO >= 8)
		SERVO_8_PIN_LOW;
#endif
		count = 2;
		state++;
		SERVO_CHANNEL+= SERVO_1K_US; // 1000 us
	} else if (state == 17) {
		if (count > 0) count--;
		else state = 0;
		SERVO_CHANNEL+= SERVO_1K_US;
	}
}
#endif

#if (NUMBER_MOTOR > 4) && (defined(PROMINI) || defined(PROMICRO))

#if !defined(PROMICRO) || defined(HWPWM6)
#define SOFT_PWM_ISR1 TIMER0_COMPB_vect
#define SOFT_PWM_ISR2 TIMER0_COMPA_vect
#define SOFT_PWM_CHANNEL1 OCR0B
#define SOFT_PWM_CHANNEL2 OCR0A
#else
#define SOFT_PWM_ISR1 TIMER3_COMPB_vect
#define SOFT_PWM_ISR2 TIMER3_COMPC_vect
#define SOFT_PWM_CHANNEL1 OCR3B
#define SOFT_PWM_CHANNEL2 OCR3C
#endif

void initializeSoftPWM() {
#if !defined(PROMICRO) || defined(HWPWM6)
	TCCR0A = 0; // normal counting mode
#if (NUMBER_MOTOR > 4) && !defined(HWPWM6)
	TIMSK0 |= (1<<OCIE0B); // Enable CTC interrupt
#endif
#if (NUMBER_MOTOR > 6)
	TIMSK0 |= (1<<OCIE0A);
#endif
#else
	TCCR3A &= ~(1<<WGM30); // normal counting mode
	TCCR3B &= ~(1<<CS31); // no prescaler
	TIMSK3 |= (1<<OCIE3B); // Enable CTC interrupt
#if (NUMBER_MOTOR > 6)
	TIMSK3 |= (1<<OCIE3C);
#endif
#endif
}

/****************               Motor SW PWM ISR's                 ******************/
// hexa with old but sometimes better SW PWM method
// for setups without servos
#if (NUMBER_MOTOR == 6) && (!defined(SERVO) && !defined(HWPWM6))
ISR(SOFT_PWM_ISR1) {
	static uint8_t state = 0;
	if(state == 0){
		SOFT_PWM_1_PIN_HIGH;
		SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_highState;
		state = 1;
	}else if(state == 1){
		SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_highState;
		state = 2;
	}else if(state == 2){
		SOFT_PWM_1_PIN_LOW;
		SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_lowState;
		state = 3;
	}else if(state == 3){
		SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_lowState;
		state = 0;
	}
}
ISR(SOFT_PWM_ISR2) {
	static uint8_t state = 0;
	if(state == 0){
		SOFT_PWM_2_PIN_HIGH;
		SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_highState;
		state = 1;
	}else if(state == 1){
		SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_highState;
		state = 2;
	}else if(state == 2){
		SOFT_PWM_2_PIN_LOW;
		SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_lowState;
		state = 3;
	}else if(state == 3){
		SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_lowState;
		state = 0;
	}
}
#else
#if (NUMBER_MOTOR > 4) && !defined(HWPWM6)
// HEXA with just OCR0B
ISR(SOFT_PWM_ISR1) {
	static uint8_t state = 0;
	if(state == 0){
		SOFT_PWM_1_PIN_HIGH;
		SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_highState;
		state = 1;
	}else if(state == 1){
		SOFT_PWM_2_PIN_LOW;
		SOFT_PWM_CHANNEL1 += atomicPWM_PIN6_lowState;
		state = 2;
	}else if(state == 2){
		SOFT_PWM_2_PIN_HIGH;
		SOFT_PWM_CHANNEL1 += atomicPWM_PIN6_highState;
		state = 3;
	}else if(state == 3){
		SOFT_PWM_1_PIN_LOW;
		SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_lowState;
		state = 0;
	}
}
#endif
//the same with digital PIN A2 & 12 OCR0A counter for OCTO
#if (NUMBER_MOTOR > 6)
ISR(SOFT_PWM_ISR2) {
	static uint8_t state = 0;
	if(state == 0){
		SOFT_PWM_3_PIN_HIGH;
		SOFT_PWM_CHANNEL2 += atomicPWM_PINA2_highState;
		state = 1;
	}else if(state == 1){
		SOFT_PWM_4_PIN_LOW;
		SOFT_PWM_CHANNEL2 += atomicPWM_PIN12_lowState;
		state = 2;
	}else if(state == 2){
		SOFT_PWM_4_PIN_HIGH;
		SOFT_PWM_CHANNEL2 += atomicPWM_PIN12_highState;
		state = 3;
	}else if(state == 3){
		SOFT_PWM_3_PIN_LOW;
		SOFT_PWM_CHANNEL2 += atomicPWM_PINA2_lowState;
		state = 0;
	}
}
#endif
#endif
#endif



/*
        This gets called when the master does a write.
 */
void onRecieveHandler(int numBytes)
{
	//        digitalWrite(led, HIGH);
	/* eat all the bytes, just for demo purposes, we don't use them*/
	for (int i = 0; i < numBytes && i < BUFFER_SIZE; i++) {
		i2c_buffer[i] = Wire.receive();
	}

	for (int i = 0; i < ( NUMBER_MOTOR *2) && i < BUFFER_SIZE; i++) {
		motor[i] = (i2c_buffer[i]<<8) |(i2c_buffer[i+1]&0xff);
	}

}

/*
        This gets called when the master does a read.
        We are simulating a one byte write command followed
        by a two byte response from slave behavior.
 */
void onRequestHandler()
{
	//        digitalWrite(led, HIGH);
	//        if (cmd == 0x01) {
	//          i2c_buffer[0] = 0x1e;
	//          i2c_buffer[1] = 0x55;
	//        }
	//        else if (cmd == 0x02) {
	//          i2c_buffer[0] = 0xbe;
	//          i2c_buffer[1] = 0xef;
	//        }
	//        else {
	i2c_buffer[0] = 0xa5;
	i2c_buffer[1] = 0x81;
	//        }

	Wire.send(i2c_buffer, 2);
}

void begin_i2c()
{
	Wire.begin(I2C_SLAVE_ADDRESS);
	Wire.onReceive(onRecieveHandler);
	Wire.onRequest(onRequestHandler);
}

void setup()
{
	Serial.begin(115200);
	initOutput();
	pinMode(13, OUTPUT);
	begin_i2c();
	Serial.println("init done");
}

void loop()
{
	Serial.print("motor : ");
	for (int i = 0; i < NUMBER_MOTOR && i < BUFFER_SIZE; i++) {
		Serial.print(motor[i]);

	}
}
