


#define __AVR_ATmega328P__
/**************************************************************************************/
/***************             Proc specific definitions             ********************/
/**************************************************************************************/
// Proc auto detection
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  #define PROMINI
#endif
#if defined(__AVR_ATmega32U4__) || #defined(TEENSY20)
  #define PROMICRO
#endif
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
  #define MEGA
#endif

/**************************   atmega328P (Promini)  ************************************/
#if defined(PROMINI)
  #if !defined(MONGOOSE1_0)
    #define LEDPIN_PINMODE             pinMode (13, OUTPUT);
    #define LEDPIN_TOGGLE              PINB |= 1<<5;     //switch LEDPIN state (digital PIN 13)
    #define LEDPIN_OFF                 PORTB &= ~(1<<5);
    #define LEDPIN_ON                  PORTB |= (1<<5);
  #endif
  #if !defined(RCAUXPIN8) 
    #if !defined(MONGOOSE1_0)
      #define BUZZERPIN_PINMODE          pinMode (8, OUTPUT);
      #define BUZZERPIN_ON               PORTB |= 1;
      #define BUZZERPIN_OFF              PORTB &= ~1;
    #endif
  #else
    #define BUZZERPIN_PINMODE          ;
    #define BUZZERPIN_ON               ;
    #define BUZZERPIN_OFF              ;
    #define RCAUXPIN
  #endif
  #if !defined(RCAUXPIN12) && !defined(DISABLE_POWER_PIN)
    #define POWERPIN_PINMODE           pinMode (12, OUTPUT);
    #define POWERPIN_ON                PORTB |= 1<<4;
    #define POWERPIN_OFF               PORTB &= ~(1<<4); //switch OFF WMP, digital PIN 12
  #else
    #define POWERPIN_PINMODE           ;
    #define POWERPIN_ON                ;
    #define POWERPIN_OFF               ;
  #endif
  #if defined(RCAUXPIN12)
    #define RCAUXPIN
  #endif
  #define I2C_PULLUPS_ENABLE         PORTC |= 1<<4; PORTC |= 1<<5;   // PIN A4&A5 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTC &= ~(1<<4); PORTC &= ~(1<<5);
  #if !defined(MONGOOSE1_0)
    #define PINMODE_LCD                pinMode(0, OUTPUT);
    #define LCDPIN_OFF                 PORTD &= ~1; //switch OFF digital PIN 0
    #define LCDPIN_ON                  PORTD |= 1;
    #define STABLEPIN_PINMODE          ;
    #define STABLEPIN_ON               ;
    #define STABLEPIN_OFF              ;
  #endif 
  #define PPM_PIN_INTERRUPT          attachInterrupt(0, rxInt, RISING); //PIN 0
  #define SPEK_SERIAL_VECT           USART_RX_vect
  #define SPEK_DATA_REG              UDR0
  //RX PIN assignment inside the port //for PORTD
  #define THROTTLEPIN                2
  #define ROLLPIN                    4
  #define PITCHPIN                   5
  #define YAWPIN                     6
  #define AUX1PIN                    7
  #define AUX2PIN                    0 // optional PIN 8 or PIN 12
  #define AUX3PIN                    1 // unused 
  #define AUX4PIN                    3 // unused 
    
  #define PCINT_PIN_COUNT            5
  #define PCINT_RX_BITS              (1<<2),(1<<4),(1<<5),(1<<6),(1<<7)
  #define PCINT_RX_PORT              PORTD
  #define PCINT_RX_MASK              PCMSK2
  #define PCIR_PORT_BIT              (1<<2)
  #define RX_PC_INTERRUPT            PCINT2_vect
  #define RX_PCINT_PIN_PORT          PIND
  #define ISR_UART                   ISR(USART_UDRE_vect)
  #define V_BATPIN                   A3    // Analog PIN 3
  #define PSENSORPIN                 A2    // Analog PIN 2
  
  #if defined(A0_A1_PIN_HEX) && (NUMBER_MOTOR < 8)
    #define SOFT_PWM_1_PIN_HIGH        PORTC |= 1<<0;
    #define SOFT_PWM_1_PIN_LOW         PORTC &= ~(1<<0);
    #define SOFT_PWM_2_PIN_HIGH        PORTC |= 1<<1;
    #define SOFT_PWM_2_PIN_LOW         PORTC &= ~(1<<1);  
  #else
    #define SOFT_PWM_1_PIN_HIGH        PORTD |= 1<<5;
    #define SOFT_PWM_1_PIN_LOW         PORTD &= ~(1<<5);
    #define SOFT_PWM_2_PIN_HIGH        PORTD |= 1<<6;
    #define SOFT_PWM_2_PIN_LOW         PORTD &= ~(1<<6);
  #endif
  #define SOFT_PWM_3_PIN_HIGH        PORTC |= 1<<2;
  #define SOFT_PWM_3_PIN_LOW         PORTC &= ~(1<<2);
  #define SOFT_PWM_4_PIN_HIGH        PORTB |= 1<<4;
  #define SOFT_PWM_4_PIN_LOW         PORTB &= ~(1<<4);
  
  #define SERVO_1_PINMODE            pinMode(A0,OUTPUT); // TILT_PITCH - WING left
  #define SERVO_1_PIN_HIGH           PORTC |= 1<<0;
  #define SERVO_1_PIN_LOW            PORTC &= ~(1<<0);
  #define SERVO_2_PINMODE            pinMode(A1,OUTPUT); // TILT_ROLL  - WING right
  #define SERVO_2_PIN_HIGH           PORTC |= 1<<1;
  #define SERVO_2_PIN_LOW            PORTC &= ~(1<<1);
  #define SERVO_3_PINMODE            pinMode(A2,OUTPUT); // CAM TRIG  - alt TILT_PITCH
  #define SERVO_3_PIN_HIGH           PORTC |= 1<<2;
  #define SERVO_3_PIN_LOW            PORTC &= ~(1<<2);
  #if !defined(MONGOOSE1_0)
    #define SERVO_4_PINMODE            pinMode(12,OUTPUT); // new       - alt TILT_ROLL
    #define SERVO_4_PIN_HIGH           PORTB |= 1<<4;
    #define SERVO_4_PIN_LOW            PORTB &= ~(1<<4);
  #endif
  #define SERVO_5_PINMODE            pinMode(11,OUTPUT); // BI LEFT
  #define SERVO_5_PIN_HIGH           PORTB |= 1<<3;
  #define SERVO_5_PIN_LOW            PORTB &= ~(1<<3);
  #define SERVO_6_PINMODE            pinMode(3,OUTPUT);  // TRI REAR - BI RIGHT
  #define SERVO_6_PIN_HIGH           PORTD|= 1<<3;
  #define SERVO_6_PIN_LOW            PORTD &= ~(1<<3);
  #define SERVO_7_PINMODE            pinMode(10,OUTPUT); // new
  #define SERVO_7_PIN_HIGH           PORTB |= 1<<2;
  #define SERVO_7_PIN_LOW            PORTB &= ~(1<<2);
  #define SERVO_8_PINMODE            pinMode(9,OUTPUT); // new
  #define SERVO_8_PIN_HIGH           PORTB |= 1<<1;
  #define SERVO_8_PIN_LOW            PORTB &= ~(1<<1);
#endif

/**************************  atmega32u4 (Promicro)  ***********************************/
#if defined(PROMICRO)
  #if !defined(TEENSY20)
    #define LEDPIN_PINMODE             //
    #define LEDPIN_TOGGLE              PIND |= 1<<5;     //switch LEDPIN state (Port D5)
    #if !defined(PROMICRO10)
      #define LEDPIN_OFF                 PORTD |= (1<<5);
      #define LEDPIN_ON                  PORTD &= ~(1<<5);  
    #else
      #define LEDPIN_OFF                PORTD &= ~(1<<5);
      #define LEDPIN_ON                 PORTD |= (1<<5);
    #endif
  #else
    #define LEDPIN_PINMODE           DDRD |= (1<<6);
    #define LEDPIN_OFF               PORTD &= ~(1<<6);
    #define LEDPIN_ON                PORTD |= (1<<6);   
    #define LEDPIN_TOGGLE            PIND |= 1<<6;     //switch LEDPIN state (Port D6)  
  #endif
  #if defined(D8BUZZER)
    #define BUZZERPIN_PINMODE          DDRB |= (1<<4);
    #define BUZZERPIN_ON               PORTB |= 1<<4;
    #define BUZZERPIN_OFF              PORTB &= ~(1<<4); 
  #elif defined(A32U4ALLPINS)
    #define BUZZERPIN_PINMODE          DDRD |= (1<<4);
    #define BUZZERPIN_ON               PORTD |= 1<<4;
    #define BUZZERPIN_OFF              PORTD &= ~(1<<4);    
  #else
    #define BUZZERPIN_PINMODE          DDRD |= (1<<3);
    #define BUZZERPIN_ON               PORTD |= 1<<3;
    #define BUZZERPIN_OFF              PORTD &= ~(1<<3); 
  #endif
  #define POWERPIN_PINMODE           //
  #define POWERPIN_ON                //
  #define POWERPIN_OFF               //
  #define I2C_PULLUPS_ENABLE         PORTD |= 1<<0; PORTD |= 1<<1;   // PIN 2&3 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1);
  #define PINMODE_LCD                DDRD |= (1<<2);
  #define LCDPIN_OFF                 PORTD &= ~1;
  #define LCDPIN_ON                  PORTD |= 1;
  #define STABLEPIN_PINMODE          ;
  #define STABLEPIN_ON               ;
  #define STABLEPIN_OFF              ;
  #define PPM_PIN_INTERRUPT          DDRE &= ~(1 << 6);PORTE |= (1 << 6);EIMSK |= (1 << INT6);EICRB |= (1 << ISC61)|(1 << ISC60);
  #define SPEK_SERIAL_VECT           USART1_RX_vect
  #define SPEK_DATA_REG              UDR1
  #define USB_CDC_TX                 3
  #define USB_CDC_RX                 2
  
  //soft PWM Pins  
  #define SOFT_PWM_1_PIN_HIGH        PORTD |= 1<<4;
  #define SOFT_PWM_1_PIN_LOW         PORTD &= ~(1<<4);
  #define SOFT_PWM_2_PIN_HIGH        PORTF |= 1<<5;
  #define SOFT_PWM_2_PIN_LOW         PORTF &= ~(1<<5);
  #define SOFT_PWM_3_PIN_HIGH        PORTF |= 1<<7;
  #define SOFT_PWM_3_PIN_LOW         PORTF &= ~(1<<7);
  #define SOFT_PWM_4_PIN_HIGH        PORTF |= 1<<6;
  #define SOFT_PWM_4_PIN_LOW         PORTF &= ~(1<<6);
  
  // Servos
  #define SERVO_1_PINMODE   DDRF |= (1<<7); // A0
  #define SERVO_1_PIN_HIGH  PORTF|= 1<<7;
  #define SERVO_1_PIN_LOW   PORTF &= ~(1<<7);
  #define SERVO_2_PINMODE   DDRF |= (1<<6); // A1
  #define SERVO_2_PIN_HIGH  PORTF |= 1<<6;
  #define SERVO_2_PIN_LOW   PORTF &= ~(1<<6);
  #define SERVO_3_PINMODE   DDRF |= (1<<5); // A2
  #define SERVO_3_PIN_HIGH  PORTF |= 1<<5;
  #define SERVO_3_PIN_LOW   PORTF &= ~(1<<5);
  #if !defined(A32U4ALLPINS)
    #define SERVO_4_PINMODE   DDRD |= (1<<4); // 4
    #define SERVO_4_PIN_HIGH  PORTD |= 1<<4;
    #define SERVO_4_PIN_LOW   PORTD &= ~(1<<4);
  #else
    #define SERVO_4_PINMODE   DDRF |= (1<<4); // A3
    #define SERVO_4_PIN_HIGH  PORTF |= 1<<4;
    #define SERVO_4_PIN_LOW   PORTF &= ~(1<<4);  
  #endif
  #define SERVO_5_PINMODE   DDRD |= (1<<7); // 6
  #define SERVO_5_PIN_HIGH  PORTD |= 1<<7;
  #define SERVO_5_PIN_LOW   PORTD &= ~(1<<7);
  #define SERVO_6_PINMODE   DDRC |= (1<<6); // 5
  #define SERVO_6_PIN_HIGH  PORTC|= 1<<6;
  #define SERVO_6_PIN_LOW   PORTC &= ~(1<<6);
  #define SERVO_7_PINMODE   DDRB |= (1<<6); // 10
  #define SERVO_7_PIN_HIGH  PORTB |= 1<<6;
  #define SERVO_7_PIN_LOW   PORTB &= ~(1<<6);
  #define SERVO_8_PINMODE   DDRB |= (1<<5); // 9
  #define SERVO_8_PIN_HIGH  PORTB |= 1<<5;
  #define SERVO_8_PIN_LOW   PORTB &= ~(1<<5);
  
  //Standart RX
  #define THROTTLEPIN                  3
  #if defined(A32U4ALLPINS)
    #define ROLLPIN                    6
    #define PITCHPIN                   2
    #define YAWPIN                     4
    #define AUX1PIN                    5
  #else
    #define ROLLPIN                    4
    #define PITCHPIN                   5
    #define YAWPIN                     2
    #define AUX1PIN                    6
  #endif
  #define AUX2PIN                      7 
  #define AUX3PIN                      1 // unused 
  #define AUX4PIN                      0 // unused 
  #if !defined(RCAUX2PIND17)
    #define PCINT_PIN_COUNT          4
    #define PCINT_RX_BITS            (1<<1),(1<<2),(1<<3),(1<<4)
  #else
    #define PCINT_PIN_COUNT          5 // one more bit (PB0) is added in RX code
    #define PCINT_RX_BITS            (1<<1),(1<<2),(1<<3),(1<<4),(1<<0)
  #endif
  #define PCINT_RX_PORT                PORTB
  #define PCINT_RX_MASK                PCMSK0
  #define PCIR_PORT_BIT                (1<<0)
  #define RX_PC_INTERRUPT              PCINT0_vect
  #define RX_PCINT_PIN_PORT            PINB
  
  #define ISR_UART                    ISR(USART_UDRE_vect)
  #if !defined(A32U4ALLPINS) && !defined(TEENSY20)
    #define V_BATPIN                  A3    // Analog PIN 3
  #elif defined(A32U4ALLPINS)
    #define V_BATPIN                  A4    // Analog PIN 4
  #else
    #define V_BATPIN                  A2    // Analog PIN 3
  #endif
  #if !defined(TEENSY20)
    #define PSENSORPIN                A2    // Analog PIN 2 
  #else
    #define PSENSORPIN                A2    // Analog PIN 2 
  #endif
#endif

/**************************  all the Mega types  ***********************************/
#if defined(MEGA)
  #define LEDPIN_PINMODE             pinMode (13, OUTPUT);pinMode (30, OUTPUT);
  #define LEDPIN_TOGGLE              PINB  |= (1<<7); PINC  |= (1<<7);
  #define LEDPIN_ON                  PORTB |= (1<<7); PORTC |= (1<<7);
  #define LEDPIN_OFF                 PORTB &= ~(1<<7);PORTC &= ~(1<<7);
  #define BUZZERPIN_PINMODE          pinMode (32, OUTPUT);
  #define BUZZERPIN_ON               PORTC |= 1<<5;
  #define BUZZERPIN_OFF              PORTC &= ~(1<<5);
  #if !defined(DISABLE_POWER_PIN)
    #define POWERPIN_PINMODE           pinMode (37, OUTPUT);
    #define POWERPIN_ON                PORTC |= 1<<0;
    #define POWERPIN_OFF               PORTC &= ~(1<<0);
  #else
    #define POWERPIN_PINMODE           ;
    #define POWERPIN_ON                ;
    #define POWERPIN_OFF               ;
  #endif
  #define I2C_PULLUPS_ENABLE         PORTD |= 1<<0; PORTD |= 1<<1;       // PIN 20&21 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1);
  #define PINMODE_LCD                pinMode(0, OUTPUT);
  #define LCDPIN_OFF                 PORTE &= ~1; //switch OFF digital PIN 0
  #define LCDPIN_ON                  PORTE |= 1;
  #define STABLEPIN_PINMODE          pinMode (31, OUTPUT);
  #define STABLEPIN_ON               PORTC |= 1<<6;
  #define STABLEPIN_OFF              PORTC &= ~(1<<6);

  #define PPM_PIN_INTERRUPT          attachInterrupt(4, rxInt, RISING);  //PIN 19, also used for Spektrum satellite option
  #define SPEK_SERIAL_VECT           USART1_RX_vect
  #define SPEK_DATA_REG              UDR1
  //RX PIN assignment inside the port //for PORTK
  #define THROTTLEPIN                0  //PIN 62 =  PIN A8
  #define ROLLPIN                    1  //PIN 63 =  PIN A9
  #define PITCHPIN                   2  //PIN 64 =  PIN A10
  #define YAWPIN                     3  //PIN 65 =  PIN A11
  #define AUX1PIN                    4  //PIN 66 =  PIN A12
  #define AUX2PIN                    5  //PIN 67 =  PIN A13
  #define AUX3PIN                    6  //PIN 68 =  PIN A14
  #define AUX4PIN                    7  //PIN 69 =  PIN A15
  #define V_BATPIN                   A0    // Analog PIN 0
  #define PSENSORPIN                 A2    // Analog PIN 2
  #define PCINT_PIN_COUNT            8
  #define PCINT_RX_BITS              (1<<2),(1<<4),(1<<5),(1<<6),(1<<7),(1<<0),(1<<1),(1<<3)
  #define PCINT_RX_PORT              PORTK
  #define PCINT_RX_MASK              PCMSK2
  #define PCIR_PORT_BIT              (1<<2)
  #define RX_PC_INTERRUPT            PCINT2_vect
  #define RX_PCINT_PIN_PORT          PINK
   
  #define ISR_UART                   ISR(USART0_UDRE_vect)
  
  #define SERVO_1_PINMODE            pinMode(34,OUTPUT);pinMode(44,OUTPUT); // TILT_PITCH - WING left
  #define SERVO_1_PIN_HIGH           PORTC |= 1<<3;PORTL |= 1<<5;
  #define SERVO_1_PIN_LOW            PORTC &= ~(1<<3);PORTL &= ~(1<<5);
  #define SERVO_2_PINMODE            pinMode(35,OUTPUT);pinMode(45,OUTPUT); // TILT_ROLL  - WING right
  #define SERVO_2_PIN_HIGH           PORTC |= 1<<2;PORTL |= 1<<4;
  #define SERVO_2_PIN_LOW            PORTC &= ~(1<<2);PORTL &= ~(1<<4);
  #define SERVO_3_PINMODE            pinMode(33,OUTPUT); pinMode(46,OUTPUT); // CAM TRIG  - alt TILT_PITCH
  #define SERVO_3_PIN_HIGH           PORTC |= 1<<4;PORTL |= 1<<3;
  #define SERVO_3_PIN_LOW            PORTC &= ~(1<<4);PORTL &= ~(1<<3);
  #define SERVO_4_PINMODE            pinMode (37, OUTPUT);                   // new       - alt TILT_ROLL
  #define SERVO_4_PIN_HIGH           PORTC |= 1<<0;
  #define SERVO_4_PIN_LOW            PORTC &= ~(1<<0);
  #define SERVO_5_PINMODE            pinMode(6,OUTPUT);                      // BI LEFT
  #define SERVO_5_PIN_HIGH           PORTH |= 1<<3;
  #define SERVO_5_PIN_LOW            PORTH &= ~(1<<3);
  #define SERVO_6_PINMODE            pinMode(2,OUTPUT);                      // TRI REAR - BI RIGHT
  #define SERVO_6_PIN_HIGH           PORTE |= 1<<4;
  #define SERVO_6_PIN_LOW            PORTE &= ~(1<<4);
  #define SERVO_7_PINMODE            pinMode(5,OUTPUT);                      // new
  #define SERVO_7_PIN_HIGH           PORTE |= 1<<3;
  #define SERVO_7_PIN_LOW            PORTE &= ~(1<<3);
  #define SERVO_8_PINMODE            pinMode(3,OUTPUT);                      // new
  #define SERVO_8_PIN_HIGH           PORTE |= 1<<5;
  #define SERVO_8_PIN_LOW            PORTE &= ~(1<<5);
#endif


// special defines for the Mongose IMU board 
// note: that may be moved to the IMU Orientations because this are board defines .. not Proc

#if defined(MONGOOSE1_0)  // basically it's a PROMINI without some PINS => same code as a PROMINI board except PIN definition
                          // note: to avoid too much dubble code there are now just the differencies
  // http://www.multiwii.com/forum/viewtopic.php?f=6&t=627
  #define LEDPIN_PINMODE             pinMode (4, OUTPUT);
  #define LEDPIN_TOGGLE              PIND |= 1<<4;     //switch LEDPIN state (digital PIN 13)
  #define LEDPIN_OFF                 PORTD &= ~(1<<4);  
  #define LEDPIN_ON                  PORTD |= (1<<4);     
  #define SPEK_BAUD_SET              UCSR0A  = (1<<U2X0); UBRR0H = ((F_CPU  / 4 / 115200 -1) / 2) >> 8; UBRR0L = ((F_CPU  / 4 / 115200 -1) / 2);
  #define SPEK_SERIAL_INTERRUPT      UCSR0B |= (1<<RXEN0)|(1<<RXCIE0);

  /* Unavailable pins on MONGOOSE1_0 */
  #define BUZZERPIN_PINMODE          ; // D8
  #define BUZZERPIN_ON               ;
  #define BUZZERPIN_OFF              ;
  #define POWERPIN_PINMODE           ; // D12
  #define POWERPIN_ON                ;
  #define POWERPIN_OFF               ;
  #define STABLEPIN_PINMODE          ; //
  #define STABLEPIN_ON               ;
  #define STABLEPIN_OFF              ; 
  #define PINMODE_LCD                ; //
  #define LCDPIN_OFF                 ;
  #define LCDPIN_ON                  ; 
  
  
  #define SERVO_4_PINMODE            ;                   // Not available
  #define SERVO_4_PIN_HIGH           ;
  #define SERVO_4_PIN_LOW            ;
#endif





