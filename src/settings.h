//
//  settings.h
//  Stepper32
//
//  Created by Ruedi Heimlicher on 07.05.2020.
//  Copyright © 2020 Ruedi Heimlicher. All rights reserved.
//

#ifndef settings_h
#define settings_h



#define USB_DATENBREITE 32

#define TIMER0_STARTWERT   0x40

#define LOOPLEDDDR          DDRD
#define LOOPLEDPORT         PORTD
#define LOOPLED             6       //6 

#define TASTENDDR           DDRF
#define TASTENPORT          PORTF
#define TASTENPIN             PINF


// Stepper A


 
 
 

 
#define DC_PWM              22

// 10,11,12: SPI
// 18, 19: I2C


// SPI
#define OSZIPORT           PORTD
#define OSZIPORTDDR        DDRD
#define OSZIPORTPIN        PIND
#define OSZI_PULS_A        0
#define OSZI_PULS_B        1

#define OSZIALO OSZIPORT &= ~(1<<OSZI_PULS_A)
#define OSZIAHI OSZIPORT |= (1<<OSZI_PULS_A)
#define OSZIATOG OSZIPORT ^= (1<<OSZI_PULS_A)

#define OSZIBLO OSZIPORT &= ~(1<<OSZI_PULS_B)
#define OSZIBHI OSZIPORT |= (1<<OSZI_PULS_B)
#define OSZIBTOG OSZIPORT ^= (1<<OSZI_PULS_B)
// SPI


//#define TASTE0_PIN            0   // HALT-Bit Motor A
//#define TASTE1_PIN            1


#define STEPDUR 250

#define STARTDELAYBIT       0
#define HICOUNTBIT          1

#define LOAD_NEXT          5  // Bit fuer Laden des naechsten Abschnitts in der loop
#define LOAD_LAST          6  // Bit fuer Laden des letzten Abschnitts in der loop

// auf Stepperport 1


// Auf Stepperport 2
#define END_C0          6       //  Bit fuer Endanschlag C0 
#define END_D0          7       //           Endanschlag D0 

// auf Stepperport 1
#define END_A0_PIN          6       //  PIN fuer Endanschlag A0 
#define END_B0_PIN          7 		//           Endanschlag B0 


// Auf Stepperport 2
#define END_C0_PIN          6       //  PIN fuer Endanschlag C0 
#define END_D0_PIN          7 		//           Endanschlag D0 


#define RICHTUNG_A   0
#define RICHTUNG_B   1
#define RICHTUNG_C   2
#define RICHTUNG_D   3

#define MOTOR_A 0
#define MOTOR_B 1
#define MOTOR_C 2
#define MOTOR_D 3


#define HALT_PIN           0

#define COUNT_A            0 // 4      // Motorstatus:   Schritte von Motor A zaehlen
#define COUNT_B            1 // 5      // Motorstatus:   Schritte von Motor B zaehlen

#define COUNT_C            2 // 4      // Motorstatus:   Schritte von Motor C zaehlen
#define COUNT_D            3 // 5      // Motorstatus:   Schritte von Motor D zaehlen

#define COUNT_END          6
#define COUNT_LAST         7


#define TIMER_ON           1 // Bit fuer timerfunktion start

#define DC                  7    // DC ON: LO
#define STROM               4    // Stepperstrom ON: HI

#define GO_HOME            7     // Bit fuer befehl beginn home auf cncstatus
#define DC_DIVIDER         1      // teilt die pwm-Frequenz in ISR

#define USB_SEND  0 


// Ringbuffer
#define RINGBUFFERTIEFE    4
#define READYBIT           0        // buffer kann Daten aufnehmen
#define FULLBIT            1        // Buffer ist voll
#define STARTBIT           2        // Buffer ist geladen
#define RINGBUFFERBIT      3        // Ringbuffer wird verwendet
#define LASTBIT            4        // Letzter Abschnitt  ist geladen
#define ENDBIT             5        // Letzter Abschnitt  ist abgearbeitet
#define STOPBIT            6        // Ablauf stoppen
#define FIRSTBIT           7





#define THREAD_COUNT_BIT   0

#define TIMERINTERVALL 128

// Ramp
#define RAMP_OK      1 // Ramp einschalten
#define RAMPFAKTOR   2 // Verlaengerung der delayzeit
#define RAMPZEIT     800 // Mindestdauer fuer Ramp

#define RAMPSTARTBIT 1
#define RAMPENDBIT 2
#define RAMPEND0BIT 3 // Beginn der Endrampe
#define RAMPOKBIT    7
#define RAMPSCHRITT  1


#define DEVICE_MILL  1
#define DEVICE_JOY  2

#define VORZEICHEN_X   0
#define VORZEICHEN_Y   1


// Zeichnen
#define ACHSE0_BYTE_H   0
#define ACHSE0_BYTE_L   1
#define ACHSE0_START_BYTE_H   2
#define ACHSE0_START_BYTE_L   3

 
#define ACHSE1_BYTE_H   4
#define ACHSE1_BYTE_L   5
#define ACHSE1_START_BYTE_H   6
#define ACHSE1_START_BYTE_L   7

#define ACHSE2_BYTE_H   8
#define ACHSE2_BYTE_L   9
#define ACHSE2_START_BYTE_H   10
#define ACHSE2_START_BYTE_L   11

#define  ACHSE3_BYTE_H  12
#define  ACHSE3_BYTE_L  13

#define  INDEX_BYTE_H  18
#define  INDEX_BYTE_L  19

/*
#define TASTE1     67
#define TASTE2     109
#define TASTE3     163
#define TASTE4     253
#define TASTE5     360
#define TASTE6     484
#define TASTE7     628
#define TASTE8     742
#define TASTE9     827
#define TASTEL     899
#define TASTE0     946
#define TASTER     993
*/
/*

#define TASTEX  10
#define TASTE1  56
#define TASTE2  78
#define TASTE3  101
#define TASTE4  124
#define TASTE5  154
#define TASTE6  182
#define TASTE7  203
#define TASTE8  220
#define TASTE9  240

*/
//#define TASTEL     250
//#define TASTE0     250
//#define TASTER     250

#define TASTE0				0   // HALT-PIN Motor A
#define TASTE1				1


// von Mill32

//#define OSZI_PULS_A        9


#define STARTIMPULSDAUER   100 // Beginn Rampe
#define ENDIMPULSDAUER     20
#define TASTENENDIMPULSDAUER     20

# define RAMPDELAY 80 // delay fuerr Reduktion Impulsdauer


// begin Ringbuffer
#define RINGBUFFERTIEFE 4
#define READYBIT   0       // buffer kann Daten aufnehmen
#define FULLBIT   1        // Buffer ist voll
#define STARTBIT   2       // Buffer ist geladen
#define RINGBUFFERBIT 3    // Ringbuffer wird verwendet
#define LASTBIT   4         // Letzter Abschnitt  ist geladen
#define ENDBIT   5          // Letzter Abschnitt  ist abgearbeitet
#define STOPBIT   6        // Ablauf stoppen
#define FIRSTBIT   7

#define TASTENDDR           DDRF
#define TASTENPORT          PORTF
#define TASTENPIN          PINF

#define TASTE0_PIN				0   // HALT-PIN Motor A
#define TASTE1_PIN				1
/*
#define  HYP_BYTE_H  22 // Hypotenuse
#define  HYP_BYTE_L 23


#define  STEPS_BYTE_H  26
#define  STEPS_BYTE_L  27
*/

// CNC12
#define CMD_PORT            PORTD   //    PORTB
#define CMD_DDR             DDRD    //    DDRB
#define CMD_PIN             PIND    //    PINB

#define LOAD_NEXT          5  // Bit fuer Laden des naechsten Abschnitts in der loop
#define LOAD_LAST          6  // Bit fuer Laden des letzten Abschnitts in der loop

// auf Stepperport 1
#define END_A0_PIN          6       //  PIN fuer Endanschlag A0 
#define END_B0_PIN          7 		//           Endanschlag B0 


// Auf Stepperport 2
#define END_C0_PIN          6       //  PIN fuer Endanschlag C0 
#define END_D0_PIN          7 		//           Endanschlag D0 


#define RICHTUNG_A	0
#define RICHTUNG_B	1
#define RICHTUNG_C	2
#define RICHTUNG_D	3

// Seite 1

#define STEPPERPORT_1	PORTC
#define STEPPERDDR_1    DDRC
#define STEPPERPIN_1    PINC

#define MA_STEP         0
#define MA_RI           1
#define MA_EN           2

#define MB_STEP         3
#define MB_RI           4
#define MB_EN           5

#define END_A0          6           // Bit fuer Endanschlag bei A0
#define END_B0          7           // Bit fuer Endanschlag bei A1


// Seite 2

#define STEPPERPORT_2      PORTB
#define STEPPERDDR_2       DDRB
#define STEPPERPIN_2       PINB

#define MC_STEP            0           // PIN auf Stepperport 2
#define MC_RI              1
#define MC_EN              2

#define MD_STEP            3           // PIN auf Stepperport 2
#define MD_RI              4
#define MD_EN              5

#define END_C0             6           // Anschlagstatus:  Bit fuer Endanschlag bei C0
#define END_D0             7           // Anschlagstatus:  Bit fuer Endanschlag bei D0



#endif /* settings_h */
