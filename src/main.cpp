#include <Arduino.h>

#include <string.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "main.h"
#include "settings.h"
#include "lcd.h"
#include "bresenham.h"
#include "adc.c"


elapsedMillis msUntilNextSend;
unsigned int packetCount = 0;
uint8_t startwert = 0;

elapsedMillis sinceblink;
elapsedMillis sincelcd;
elapsedMillis sinceusb;

elapsedMillis sinceload; // Zeitdauer der Anzeige des Potentialwertes

elapsedMicros sincelaststep;

elapsedMillis sincelastthread;
elapsedMicros sincelastimpuls;
uint16_t cncdelaycounter = 0;


#define TEST 1

uint8_t buffer[USB_DATENBREITE] = {};
uint8_t sendbuffer[USB_DATENBREITE] = {};

volatile uint16_t usb_recv_counter = 0;

// Ringbuffer
uint8_t CNCDaten[RINGBUFFERTIEFE][USB_DATENBREITE];
uint8_t CDCStringArray[RINGBUFFERTIEFE];

uint16_t abschnittnummer = 0;
uint16_t endposition = 0xFFFF;
uint16_t ladeposition = 0;

// volatile uint16_t          globalaktuelleladeposition = 0;
uint16_t aktuelleladeposition = 0;
volatile uint8_t ringbufferstatus = 0x00;

uint8_t aktuellelage = 0; 

uint16_t Abschnitte = 0;
uint16_t AbschnittCounter = 0;
volatile uint8_t liniencounter = 0;
// end Ringbuffer
volatile uint16_t steps = 0;

volatile uint16_t korrekturintervallx = 0;
volatile uint16_t korrekturintervally = 0;

volatile uint16_t korrekturintervallcounterx = 0;
volatile uint16_t korrekturintervallcountery = 0;

volatile uint16_t korrekturcounterx = 0;
volatile uint16_t korrekturcountery = 0;

volatile uint8_t vorzeichen = 0;

uint8_t motorsteps = 48;
uint8_t micro = 1;

volatile uint16_t loadtime = 0;

volatile uint8_t timer0startwert = TIMER0_STARTWERT;

volatile uint16_t timer2Counter = 0;
volatile uint8_t cncstatus = 0x00;
volatile uint8_t sendstatus = 0x00;

volatile uint8_t usbstatus = 0x00;
static volatile uint8_t motorstatus = 0x00;
static volatile uint8_t anschlagstatus = 0x00;
volatile uint8_t           anschlagcounter=0;

volatile uint8_t taskstatus = 0x00; // Anzeige running
#define TASK    1
#define RUNNING    2

// bresenham

volatile uint8_t bresenhamstatus = 0x00; // relevanter motor, in Abschnittladen:bres gesetzt

volatile uint16_t bresenham_errAB = 0; // error A,B
volatile uint16_t bresenham_e2AB = 0;  // check A,B

volatile uint16_t bresenham_errCD = 0;
volatile uint16_t bresenham_e2CD = 0;

volatile uint16_t StepStartA = 0; // startwert am Anfang des Abschnittes
volatile uint16_t StepStartC = 0;

// Seite A
volatile int16_t xA, yA, tA, dxA, dyA, incxA, incyA, pdxA, pdyA, ddxA, ddyA, deltaslowdirectionA, deltafastdirectionA, errA;

volatile uint16_t deltafastdelayA = 0; // aktueller delay
volatile uint16_t bres_delayA = 0;     // steps fuer fastdirection
volatile uint16_t bres_counterA = 0;   // zaehler fuer fastdirection

uint16_t stepdurA = 0;
uint16_t stepdurB = 0;
uint16_t stepdurC = 0;
uint16_t stepdurD = 0;

// Seite B
volatile int16_t xB, yB, tB, dxB, dyB, incxB, incyB, pdxB, pdyB, ddxB, ddyB, deltaslowdirectionB, deltafastdirectionB, errB;

volatile uint16_t deltafastdelayB = 0; // aktueller delay
volatile uint16_t bres_delayB = 0;     // steps fuer fastdirection
volatile uint16_t bres_counterB = 0;   // zaehler fuer fastdirection

#define BRES_MOTORA  0
#define BRES_MOTORB  1
#define BRES_MOTORC  2
#define BRES_MOTORD  3

uint16_t errpos = 0;
// bresenham end

volatile uint8_t timerstatus = 0;

volatile uint8_t status = 0;

volatile uint8_t pfeilstatus = 0;
volatile uint8_t tastaturstatus = 0;

volatile uint8_t PWM = 0;
static volatile uint8_t pwmposition = 0;
static volatile uint8_t pwmdivider = 0;

// CNC

volatile uint16_t CounterA = 0; // Zaehler fuer Delay von Motor A
volatile uint16_t CounterB = 0; // Zaehler fuer Delay von Motor B
volatile uint16_t CounterC = 0; // Zaehler fuer Delay von Motor C
volatile uint16_t CounterD = 0; // Zaehler fuer Delay von Motor D

volatile uint32_t DelayA = 24; // Delay von Motor A
volatile uint32_t DelayB = 24; // Delay von Motor B
volatile uint32_t DelayC = 24; // Delay von Motor C
volatile uint32_t DelayD = 24; // Delay von Motor D

volatile uint32_t StepCounterA = 0; // Zaehler fuer Schritte von Motor A
volatile uint32_t StepCounterB = 0; // Zaehler fuer Schritte von Motor B
volatile uint32_t StepCounterC = 0; // Zaehler fuer Schritte von Motor C
volatile uint32_t StepCounterD = 0; // Zaehler fuer Schritte von Motor D

volatile uint8_t richtung = 0;
volatile uint8_t homestatus = 0;

volatile uint8_t parallelcounter = 0;
volatile uint8_t parallelstatus = 0; // Status des Thread

volatile uint16_t timerintervall = TIMERINTERVALL;
volatile uint16_t timerintervall_SLOW = 0; // Intervall klein
volatile uint16_t timerintervall_FAST = 0; // Intervall gross

// Ramp

volatile uint16_t ramptimerintervall = TIMERINTERVALL;

volatile uint8_t rampstatus = 0;
volatile uint32_t rampstepstart = 0; // Stepcounter am Anfang
uint8_t rampschritt = 2;
volatile uint16_t rampbreite = 10; // anzahl Schritte der Ramp. Wird beim Start bestimmt und fuer das Ende verwendet

volatile uint32_t rampendstep = 0; // Beginn der Endramp. Wird in Abschnittladen bestimmt

uint8_t richtungstatus = 0;
uint8_t oldrichtungstatus = 0;
#define AXNEG 0
#define AYNEG 1
#define BXNEG 4
#define BYNEG 5

int16_t lastdax = 0; // letzte Werte fuer schritte x, y. Fuer berechnung gradient
int16_t lastday = 0;
#define TASTATURPIN  PF1

uint16_t tastenwert = 0;
uint16_t tastaturcounter = 0;
uint16_t prellcounter = 0;

uint16_t taste = 0xFF;
uint8_t tastencounter = 0;
uint8_t TastaturCount=0;
uint8_t Taste=0;
uint8_t analogtastaturstatus = 0;
#define TASTE_OFF  0
#define TASTE_ON  1

uint16_t TastenStatus=0;
uint16_t Tastenprellen=0x1F;



void startTimer2(void)
{
   timerstatus |= (1<<TIMER_ON);
   //timer2
   TCNT2   = 0; 
   //	TCCR2A |= (1 << WGM21);    // Configure timer 2 for CTC mode 
   TCCR2B |= (1 << CS20);     // Start timer at Fcpu/8 
   //	TIMSK2 |= (1 << OCIE2A);   // Enable CTC interrupt 
   TIMSK2 |= (1 << TOIE2);    // Enable OV interrupt 
   //OCR2A   = 5;             // Set CTC compare value with a prescaler of 64 
   TCCR2A = 0x00;
   
   sei();
}

void stopTimer2(void)
{
   TCCR2B = 0;
   timerstatus &= ~(1<<TIMER_ON);
}

ISR (TIMER2_OVF_vect) 
{ 
	timer2Counter +=1;
   
   if (PWM) // Draht soll heiss sein. 
   {
   }
   else
   {
      pwmposition =0;
   }

	if (timer2Counter >= 14) 
	{
       
      if (CounterA)  CounterA-=1;
      if (CounterB) 	CounterB-=1;
      if (CounterC)  CounterC-=1;
      if (CounterD)  CounterD-=1;
      
      
      if (timerstatus & (1<<TIMER_ON))
      {
         // OSZI_A_LO(); // 100us
         
         if (bres_delayA)
         {
            bres_delayA-=1;
         }
         
         if (bres_delayB)
         {
            bres_delayB-=1;
         }
         
      
         
      } 
     
      
      
      if (PWM)
      {
         pwmposition ++;
      }
      else
      {
         pwmposition =0;
      }

		timer2Counter = 0; 
        //OSZIBTOG ;
	} 
	TCNT2 = 10;							// ergibt 2 kHz fuer Timertakt
}

uint8_t  AbschnittLaden_bres(const uint8_t* AbschnittDaten) // 22us
{
   stopTimer2();
 //  lcd_gotoxy(15,0);
 //  lcd_puts("    ");
   
   uint8_t returnwert=0;

   /*         
    Reihenfolge der Daten:
    0    schritteax lb
    1    schritteax hb
    2    schritteay lb
    3    schritteay hb
    
    4    delayax lb
    5    delayax hb
    6    delayay lb
    7    delayay hb
    
    8    schrittebx lb
    9    schrittebx hb
    10    schritteby lb
    11    schritteby hb
    
    12    delaybx lb
    13    delaybx hb
    14    delayby lb
    15    delayby hb
    
    
    16   (8)    code
    17   (9)    position // Beschreibung der Lage im Schnittpolygon:first, last, ...
    18   (10)   indexh     // Nummer des Abschnitts
    19   (11)   indexl   
    
    20     pwm
    
    21   motorstatus // relevanter Motor fuer Abschnitt
    
    22   zoomfaktor
    
    25   steps
    26   micro
    
    */         
   
   motorsteps = AbschnittDaten[25];
   
   micro = AbschnittDaten[26];
   
   uint16_t index = (AbschnittDaten[18] << 8) | AbschnittDaten[19];
   
   if (AbschnittDaten[35] == 1)
   {
      // Serial.printf("+++ +++ +++ \t\t\t index: %d AbschnittLaden_bres WENDEPUNKT \n",index);
      rampstatus |=(1<<RAMPOKBIT);
   }
   
   // pwm-rate
   PWM = AbschnittDaten[20];
   //// Serial.printf("AbschnittLaden_4M steps: %d micro: %d PWM: %d\n",steps,micro,PWM);
   //// Serial.printf("AbschnittLaden_bres start \n");
   //**   analogWrite(DC_PWM, PWM);
   
   //// Serial.printf("AbschnittLaden_bres AbschnittDaten eingang index: %d\n", index);
   
   
   /*
   for(uint8_t i=0;i<27;i++) // 5 us ohne printf, 10ms mit printf
   { 
      // Serial.printf("%d \t",AbschnittDaten[i]);
   }
   // Serial.printf("\n");
   
    //  // Serial.printf("AbschnittLaden_4M steps: %d micro: %d\n",motorsteps,micro);
   
   //   lage = AbschnittDaten[9]; // Start: 1, innerhalb: 0, Ende: 2
    */
   
   int lage = 0;
   lage = AbschnittDaten[17]; // Start: 1, innerhalb: 0, Ende: 2
  // // Serial.printf("******* *********   AbschnittLaden_bres lage: %d\n",lage);
  // // Serial.printf("AbschnittLaden_bres lage: %d\n",lage);
   if (lage & 0x01) // erstes Element
   {
      returnwert=1;
   }
   if (lage & 0x02) // letztes Element
   {
      returnwert=2;
   }
   
   richtung=0;
   
   // Motor A
   //digitalWriteFast(MA_EN,LOW); // Pololu ON
   STEPPERPORT_1 &= ~(1<<MA_EN); // Pololu ON
   uint8_t dataL=0;
   uint8_t dataH=0;
   
   uint8_t delayL=0;
   uint8_t delayH=0;
   
   dataL=AbschnittDaten[0];
   dataH=AbschnittDaten[1];
   
   //lcd_gotoxy(17,0);
   int8_t vz = 1;
   if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1<<RICHTUNG_A); // Rueckwarts
      //digitalWriteFast(MA_RI, LOW); // PIN fuer Treiber stellen
      STEPPERPORT_1 &= ~(1<< MA_RI); // PIN fuer Treiber stellen
      vz = -1;
      //lcd_putc('r');
   }
   else 
   {
      richtung &= ~(1<<RICHTUNG_A);
      //digitalWriteFast(MA_RI, HIGH);
      STEPPERPORT_1 |= (1<< MA_RI);
      //lcd_putc('v');   // Vorwaerts
   }
   
   dataH &= (0x7F); // bit 8 entfernen
   StepCounterA = dataL | (dataH << 8);      //    

   StepCounterA *= micro;
   StepStartA = StepCounterA;
      
   delayL=AbschnittDaten[4];
   delayH=AbschnittDaten[5];
   
   DelayA = delayL | (delayH << 8);
   
   CounterA = DelayA;
   
   // Motor B
   STEPPERPORT_1 &= ~(1<<MB_EN);   // Pololu ON
   
   dataL=AbschnittDaten[2];
   dataH=AbschnittDaten[3];
   //lcd_gotoxy(19,1);
   vz = 1;
   if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1<<RICHTUNG_B); // Rueckwarts
      //digitalWriteFast(MB_RI, LOW);      //lcd_putc('r');
      STEPPERPORT_1 &= ~(1<< MB_RI);
      vz = -1;
   }
   else 
   {
      richtung &= ~(1<<RICHTUNG_B);
      STEPPERPORT_1 |= (1<< MB_RI);
   }
   
   dataH &= (0x7F);
    
   StepCounterB = dataL | (dataH <<8);
   
   StepCounterB *= micro;
   
   
    DelayB = (AbschnittDaten[7]<<8) | AbschnittDaten[6];
   
   
   // Serial.printf("\nAbschnittLaden_bres index: %d StepCounterA  : %d DelayA: %d StepCounterB: %d DelayB: %d\n",index,StepCounterA, DelayA, StepCounterB, DelayB);

   
   CounterB = DelayB;
   
   
    // Motor C
   //digitalWriteFast(MC_EN,LOW);    // Pololu ON
   STEPPERPORT_2 &= ~(1<<MC_EN); // Pololu ON
   //CounterC=0;
   dataL=0;
   dataH=0;
   
   delayL=0;
   delayH=0;
   
   dataL=AbschnittDaten[8];
   dataH=AbschnittDaten[9];
   //// Serial.printf("AbschnittLaden_4M C datah: %d\n",dataH);
   //richtung=0;
   if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1<<RICHTUNG_C); // Rueckwarts
      //digitalWriteFast(MC_RI, LOW);
      STEPPERPORT_2 &= ~(1<< MC_RI);
      //// Serial.printf("AbschnittLaden_4M C negativ\n");
   }
   else 
   {
      richtung &= ~(1<<RICHTUNG_C);
      //digitalWriteFast(MA_RI, HIGH);
      STEPPERPORT_2 |= (1<< MC_RI);
      //// Serial.printf("AbschnittLaden_4M C positiv\n");
   }
   
   dataH &= (0x7F);
//   StepCounterC = dataH;      // HByte
//   StepCounterC <<= 8;      // shift 8
//   StepCounterC += dataL;   // +LByte
   
   StepCounterC = dataL | (dataH << 8);
   StepCounterC  *= micro;
   
   StepStartC = StepCounterC;
   delayL=AbschnittDaten[12];
   delayH=AbschnittDaten[13];
   
//   DelayC = delayH;
//   DelayC <<=8;
//   DelayC += delayL;
   
   DelayC = delayL | (delayH <<8);
   
   CounterC = DelayC;

   // Motor D
   STEPPERPORT_2 &= ~(1<<MD_EN); // Pololu ON
   dataL=0;
   dataH=0;
   
   delayL = 0;
   delayH = 0;
   
   dataL = AbschnittDaten[10];
   dataH = AbschnittDaten[11];
   //// Serial.printf("AbschnittLaden_4M D datah: %d\n",dataH);
   if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1<<RICHTUNG_D); // Rueckwarts
      STEPPERPORT_2 &= ~(1<<MD_RI); // Rueckwarts
      //// Serial.printf("AbschnittLaden_4M D negativ\n");
   }
   else 
   {
      richtung &= ~(1<<RICHTUNG_D);
      STEPPERPORT_2 |= (1<< MD_RI);
      //// Serial.printf("AbschnittLaden_4M D positiv\n");
   }
   
   dataH &= (0x7F);
   
   StepCounterD = dataL | (dataH << 8); 
   StepCounterD  *= micro;
   
   delayL=AbschnittDaten[14];
   delayH=AbschnittDaten[15];
   
   DelayD = delayL | (delayH <<8);
   
   //// Serial.printf("AbschnittLaden_4M StepCounterD: %d DelayD: %d\n",StepCounterD,DelayD);
   CounterD = DelayD;
   
   //  ****
   //  Bresenham
   //  ***
   //// Serial.printf("AbschnittLaden_bres vor bresenham: StepCounterA: %d StepCounterB: %d\n",StepCounterA,StepCounterB);
   deltafastdirectionA = 0;
   deltaslowdirectionA = 0;
   deltafastdirectionB = 0;
   deltaslowdirectionB = 0;
   deltafastdelayA = 0;
   deltafastdelayB = 0;

   bresenhamstatus = 0;
   // bresenham Seite A
   
   // relevanten Motor setzen
   if (StepCounterA > StepCounterB)
   {
      bresenhamstatus |= (1<<BRES_MOTORA);
      //
      pdxA = 1;
      pdyA = 0;
      ddxA = 1;
      ddyA = 1;
      deltaslowdirectionA = StepCounterB;
      deltafastdirectionA = StepCounterA;
      deltafastdelayA = DelayA;
 //     // Serial.printf("AbschnittLaden_bres  A > B\n");
   }
   else
   {
      bresenhamstatus |= (1<<BRES_MOTORB);
      //
      pdxA = 0;
      pdyA = 1;
      ddxA = 1;
      ddyA = 1;
      deltaslowdirectionA = StepCounterA;
      deltafastdirectionA = StepCounterB;
      deltafastdelayA = DelayB;
 //     // Serial.printf("AbschnittLaden_bres  A < B\n");
   }
   // aktuelle Werte einsetzen
   bres_delayA = deltafastdelayA; // aktueller delay in fastdir
   bres_counterA = deltafastdirectionA; // aktueller counter fuer steps
   
   if(rampstatus & (1<<RAMPOKBIT))
   {
      // Serial.printf("AbschnittLaden_bres index: %d set RAMPSTARTBIT\n",index);
      rampstatus |= (1<<RAMPSTARTBIT);
      errpos = 0;
      ramptimerintervall += (ramptimerintervall/4*3);
      
      //delayTimer.update(ramptimerintervall);
   }
   
   xA = StepCounterA; // 
   yA = StepCounterB;

   errA = deltafastdirectionA/2;
   
  // // Serial.printf("AbschnittLaden_bres deltafastdirectionA: %d deltaslowdirectionA: %d  deltafastdelayA: %d errA: %d bres_counterA: %d bres_delayA: %d\n",deltafastdirectionA,deltaslowdirectionA, deltafastdelayA,errA,bres_counterA,bres_delayA);

   // bresenham Seite B
   
    
   
   // relevanten Motor setzen
   if (StepCounterC > StepCounterD)
   {
      bresenhamstatus |= (1<<BRES_MOTORC);
      //
      pdxB = 1;
      pdyB = 0;
      ddxB = 1;
      ddyB = 1;
      deltaslowdirectionB = StepCounterD;
      deltafastdirectionB = StepCounterC;
      deltafastdelayB = DelayC;
      //// Serial.printf("AbschnittLaden_bres  C > D\n");
   }
   else
   {
      bresenhamstatus |= (1<<BRES_MOTORD);
      //
      pdxB = 0;
      pdyB = 1;
      ddxB = 1;
      ddyB = 1;
      deltaslowdirectionB = StepCounterC;
      deltafastdirectionB = StepCounterD;
      deltafastdelayB = DelayD;
      //// Serial.printf("AbschnittLaden_bres  C < D\n");
   }
   // aktuelle Werte einsetzen
   bres_delayB = deltafastdelayB; // aktueller delay in fastdir
   bres_counterB = deltafastdirectionB; // aktueller counter fuer steps
   
   xB = StepCounterC; // 
   yB = StepCounterD;

   errB = deltafastdirectionB/2;
   
     {
   
   timerintervall_FAST = TIMERINTERVALL;
   //  OSZI_B_LO();
   }
   
   
   // motorstatus: welcher Motor ist relevant
   motorstatus = AbschnittDaten[21];
   
   // richtung change
   
  // rampstatus |=(1<<RAMPOKBIT);

   
   startTimer2();
   
   //// Serial.printf("\nAbschnittLaden_bres end aktuellelage: %d \n",returnwert);
     return returnwert;
 
}

void AnschlagVonMotor(const uint8_t motor) // Schlitten ist am Anschlag
{
   //NSLog(@"AnschlagVonMotor: %d anschlagstatus am Beginn: %d",motor, anschlagstatus);
   if (richtung & (1<<(RICHTUNG_A + motor))) // Richtung war auf Anschlag A0 zu         
   {
      anschlagcounter ++;
       // MARK: END_A0 + motor
      if (!(anschlagstatus &(1<< (END_A0 + motor)))) // Bit noch nicht gesetzt
      {
         cli();
         PWM = 0;
         lcd_gotoxy(12,2);
         lcd_putc('A' + motor);
         lcd_putc('0');
         anschlagstatus |= (1<< (END_A0 + motor));      // Bit fuer Anschlag A0+motor setzen
         //anschlagstatus |= (1<< (END_A0 + motor + 4)); 
   
        // NSLog(@"anschlagstatus gesetzt: %d cncstatus: %d" anschlagstatus, cncstatus);
         //cncstatus |=  (1<<GO_HOME);
         if (cncstatus & (1<<GO_HOME)) // nur eigene Seite abstellen
         {
   // ********************************* Start HOME *****************
            // Zuerst kommt der Schlitten am Anschalg A oder C an
            
            lcd_gotoxy(15,0);
            lcd_puts("home");
           // Zuerst horizonal auf Anschlag
            switch (motor) // Stepperport 1
            {
               case 0:
               {
                  
               }
                                    
            }//switch motor
            //lcd_gotoxy(10,1);
            //lcd_putc('L');
            //lcd_putint2(ladeposition);
            
            sendbuffer[0]=0xB5 + motor; // HOME Ankunft melden
            cncstatus |= (1<<motor); 
            
             
            if (motor<2) // Stepperport 1
            {
               //lcd_gotoxy(0,2);
               
               //lcd_puts("P1 M");
               //lcd_putint1(motor);

               STEPPERPORT_1 |= (1<<(MA_EN + motor)); // Motor 0 ODER 1 OFF // andere Richtung kommt anschliessend von master
               
               if (anschlagstatus &(1<< END_A0)) // Anschlag von Motor A               
               {
                  //lcd_gotoxy(6,3);
                  //lcd_puts("A0");
                  //StepCounterA=0; 
                  //StepCounterB=0; 
                  //   deltafastdirectionB = 0;
                  //  deltaslowdirectionB = 0;
               }
                  
                  if (anschlagstatus &(1<< END_B0)) // Anschlag von Motor B, NACH Motor A
                  {
  
                     //lcd_gotoxy(8,3);
                     //lcd_puts("B0");
                     //StepCounterB=0; 
                  }
               
               // 
               //StepCounterB=0; 
                //             CounterA=0xFFFF;
                //             CounterB=0xFFFF;
               
            }
            else // Stepperport 2
            {
               //lcd_gotoxy(0,3);
               //lcd_puts("P2 M");
               //lcd_putint1(motor);
               

               STEPPERPORT_2 |= (1<<(MA_EN + motor));     // Motor 2,3 OFF
               //STEPPERPORT_1 |= (1<<(MA_EN + motor - 2)); // Paralleler Motor 0,1 OFF
               
               if (anschlagstatus &(1<< END_C0)) // Anschlag von Motor C               
               {
                  //lcd_gotoxy(6,3);
                  //lcd_puts("C0");
                  //StepCounterC=0; 
                  //StepCounterD=0;
                 
               }
                  
   
                  if (anschlagstatus &(1<< END_D0)) // Anschlag von Motor D, NACH Motor C
                  {
                     //lcd_gotoxy(8,3);
                     //lcd_puts("D0");
                     StepCounterD=0; 
                  }
               
              
              // StepCounterD=0;
               //               CounterC=0xFFFF;
               //               CounterD=0xFFFF;
               
            } // end Stepperport 2
         
    //        cncstatus &= ~(1<<GO_HOME);
            //ladeposition=0;
    //        AbschnittCounter++;
            //sendbuffer[0]=0xEA;
            
    //        lcd_putc('C');
    //        lcd_putint(cncstatus);

 //           lcd_puthex(STEPPERPIN_1);
//            lcd_puthex(STEPPERPIN_2);
            
            sendbuffer[7]=abschnittnummer; // lo
            sendbuffer[8]=ladeposition;
            sendbuffer[22] = cncstatus;
            sendbuffer[19] = anschlagstatus;
            
            sendbuffer[23] = (StepCounterA & 0xFF0)>>8;
            sendbuffer[24] = StepCounterA & 0x00FF;
            sendbuffer[25] = (StepCounterB & 0xFF0)>>8;
            sendbuffer[26] = StepCounterB & 0x00FF;
            sendbuffer[27] = (StepCounterC & 0xFF0)>>8;
            sendbuffer[28] = StepCounterC & 0x00FF;
            sendbuffer[29] = (StepCounterD & 0xFF0)>>8;
            sendbuffer[30] = StepCounterD & 0x00FF;
            lcd_gotoxy(0,0);
            lcd_puts("code ");
            lcd_gotoxy(6+motor,0);
            lcd_puthex(sendbuffer[0]);
 //           usb_rawhid_send((void*)sendbuffer, 50); // 220518 diff
            sei();
            
  //          cncstatus &= ~(1<<GO_HOME);
            
            
            
    //        richtung &= ~(1<<(RICHTUNG_A + motor)); // Richtung umschalten // 220518 diff
// ********************************* End HOME *****************
         } // end HOME
         
         else           // beide Seiten abstellen, Vorgang unterbrechen
         {    
            lcd_gotoxy(10,0);
            lcd_puts("both");
            cncstatus=0;
            sendbuffer[0]=0xA5 + motor;
            
            if (motor<2) // Stepperport 1
            {
               deltafastdirectionA = 0;
               deltafastdirectionB = 0;
               deltaslowdirectionA = 0;
               deltaslowdirectionB = 0;
               
               
               STEPPERPORT_1 |= (1<<(MA_EN + motor));     // Motor 0,1 OFF
               STEPPERPORT_2 |= (1<<(MC_EN + motor)); // Paralleler Motor 2,3 OFF
               
               
            } // end Stepperport 1
            else // Stepperport 2
            {
               deltafastdirectionA = 0;
               deltafastdirectionB = 0;
               deltaslowdirectionA = 0;
               deltaslowdirectionB = 0;
              
               
               STEPPERPORT_2 |= (1<<(MA_EN + motor));     // Motor 2,3 OFF
               STEPPERPORT_1 |= (1<<(MC_EN + motor )); // Paralleler Motor 0,1 OFF
            }
            
            // Alles abstellen
            /*
            StepCounterA=0;
            StepCounterB=0;
            StepCounterC=0;
            StepCounterD=0;
            */
            /*
            CounterA = 0;
            CounterB = 0;
            CounterC = 0;
            CounterD = 0;
            */
            ladeposition=0;
            motorstatus=0;
             
            sendbuffer[5]=abschnittnummer;
            sendbuffer[6]=ladeposition;
            sendbuffer[22] = cncstatus;
            lcd_gotoxy(0,0);
            lcd_puts("code ");
            lcd_gotoxy(6+motor,0);
            lcd_puthex(sendbuffer[0]);

            //usb_rawhid_send((void*)sendbuffer, 50);
            RawHID.send(sendbuffer, 50); // 
            sei();
             richtung &= ~(1<<(RICHTUNG_A + motor)); // Richtung umschalten
            
         } // both
         
         sei();
      } // END_A0 + motor
      else
      {
         
      }
      
   } // richtung war auf Anschlag zu
   /*
   else  // richtung ist von Anschlag weg
   {
       if ((anschlagstatus &(1<< (END_A0 + motor))))
      {
         anschlagstatus &= ~(1<< (END_A0 + motor)); // Bit fuer Anschlag X0 zuruecksetzen
         lcd_gotoxy(12,2);
         lcd_putc('x');
         lcd_putc('x');
      }
      else
      {
      }
      
   }
   */
}


void slaveinit(void)
{
	
	LOOPLEDDDR |=(1<<LOOPLED);
	LOOPLEDPORT |= (1<<LOOPLED);	// HI

	STEPPERDDR_1 |= (1<<MA_STEP);
	STEPPERPORT_1 |= (1<<MA_STEP);	// HI
	
	STEPPERDDR_1 |= (1 << MA_RI);
	STEPPERPORT_1 |= (1 << MA_RI);	// HI
   
	STEPPERDDR_1 |= (1 << MA_EN);
	STEPPERPORT_1 |= (1 << MA_EN);	// HI
	
	STEPPERDDR_1 |= (1 << MB_STEP);
	STEPPERPORT_1 |= (1 << MB_STEP); // HI
	
	STEPPERDDR_1 |= (1 << MB_RI);
	STEPPERPORT_1 |= (1 << MB_RI);	// HI
	
	STEPPERDDR_1 |= (1 << MB_EN);
	STEPPERPORT_1 |= (1 << MB_EN); // LO
   
   //Seite 2
	STEPPERDDR_2 |= (1<<MC_STEP);
	STEPPERPORT_2 |= (1<<MC_STEP);	// HI
	
	STEPPERDDR_2 |= (1 << MC_RI);
	STEPPERPORT_1 |= (1 << MC_RI);	// HI
   
	STEPPERDDR_2 |= (1 << MC_EN);
	STEPPERPORT_2 |= (1 << MC_EN);	// HI
	
	STEPPERDDR_2 |= (1 << MD_STEP);
	STEPPERPORT_2 |= (1 << MD_STEP); // HI
	
	STEPPERDDR_2 |= (1 << MD_RI);
	STEPPERPORT_2 |= (1 << MD_RI);	// HI
	
	STEPPERDDR_2 |= (1 << MD_EN);
   STEPPERPORT_2 |= (1 << MD_EN);	// HI
   
	
	//Pin 0 von   als Ausgang fuer OSZI
	OSZIPORTDDR |= (1<<OSZI_PULS_A);	//Pin 0 von  als Ausgang fuer LED TWI
    OSZIPORT |= (1<<OSZI_PULS_A);		// HI
	
    OSZIPORTDDR |= (1<<OSZI_PULS_B);		//Pin 1 von  als Ausgang fuer LED TWI
    OSZIPORT |= (1<<OSZI_PULS_B);		//Pin   von   als Ausgang fuer OSZI
	

	TASTENDDR &= ~(1<<TASTE0);	//Bit 0 von PORT B als Eingang für Taste 0
	TASTENPORT |= (1<<TASTE0);	//Pull-up

//	DDRB &= ~(1<<PORTB1);	//Bit 1 von PORT B als Eingang für Taste 1
//	PORTB |= (1<<PORTB1);	//Pull-up
	

	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);		//Pin 4 von PORT D als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 5 von PORT D als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT D als Ausgang fuer LCD

	//Versuch mit init von CNC 12
//	CMD_DDR &= ~(1<<END_A0_PIN);			//	Bit 0 von PORT B als Eingang für Endanschlag A0
//	CMD_PORT |= (1<<END_A0_PIN);			// Pull-up

//   CMD_DDR &= ~(1<<END_B0_PIN);			//	Bit 1 von PORT B als Eingang für Endanschlag A1
//	CMD_PORT |= (1<<END_B0_PIN);			// Pull-up

//	CMD_DDR &= ~(1<<PORTB1);			// Bit 1 von PORT B als Eingang für Taste 1
//	CMD_PORT |= (1<<PORTB1);			//	Pull-up
	
   DDRD |= (1<<PORTD6);
  PORTD |= (1<<PORTD6);
   
   
   // Anschlaege
   
   STEPPERDDR_1 &= ~(1<<END_A0_PIN);			//	Eingang für Endanschlag A0
	STEPPERPORT_1 |= (1<<END_A0_PIN);			// Pull-up
   
	STEPPERDDR_1 &= ~(1<<END_B0_PIN);			//	Eingang für Endanschlag B0
	STEPPERPORT_1 |= (1<<END_B0_PIN);			// Pull-up
   
   
   STEPPERDDR_2 &= ~(1<<END_C0_PIN);			//	Eingang für Endanschlag C0
	STEPPERPORT_2 |= (1<<END_C0_PIN);			// Pull-up
   
   STEPPERDDR_2 &= ~(1<<END_D0_PIN);			//	Eingang für Endanschlag D0
	STEPPERPORT_2 |= (1<<END_D0_PIN);			// Pull-up
   

   
   
   CMD_DDR |= (1<<DC);                       // DC-PWM-Ausgang
   CMD_PORT &= ~(1<<DC);                      // LO
   
   CMD_DDR |= (1<<STROM);                    // Stepperstrom-Ausgang, Active HI
   CMD_PORT |= (1<<STROM);                   // HI
}

void timer2 (uint8_t wert) 
{
    //timer2
   TCNT2   = 0; 
   //   TCCR2A |= (1 << WGM21);    // Configure timer 2 for CTC mode 
   TCCR2B |= (1 << CS20);     // Start timer at Fcpu/8 
   //   TIMSK2 |= (1 << OCIE2A);   // Enable CTC interrupt 
   TIMSK2 |= (1 << TOIE2);    // Enable OV interrupt 
   //OCR2A   = 5;             // Set CTC compare value with a prescaler of 64 
   TCCR2A = 0x00;

}


void setup() 
{
  // put your setup code here, to run once:
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);

  // von TWI_Slave

  uint16_t count=0;
    
   // set for 16 MHz clock
   CPU_PRESCALE(0);
   slaveinit();
   lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
   lcd_puts("Guten Tag\0");
   _delay_ms(1000);
   lcd_cls();
  uint8_t Tastenwert=0;
   uint8_t TastaturCount=0;
   
   uint16_t TastenStatus=0;
   uint16_t Tastencount=0;
   uint16_t Tastenprellen=0x01F;

   uint16_t loopcount0=0;
   uint8_t loopcount1=0;
   
   initADC(1);

  timer2(4);


// end TWI_Slave


}

void loop() 
{
  int n;
   n = RawHID.recv(buffer, 0); // 0 timeout = do not wait
  if (n > 0) 
  {
    // the computer sent a message.  Display the bits
    // of the first byte on pin 0 to 7.  Ignore the
    // other 63 bytes!
    Serial.print(F("Received packet, first byte: "));
    Serial.println((int)buffer[0]);
    for (int i=0; i<8; i++) 
    {
       sendbuffer[i] = buffer[i];
    }
    sendbuffer[16] = 0xBD; // set the first byte to 0, so we can see if the computer changes it
  // put your main code here, to run repeatedly:
}
  if (msUntilNextSend > 400) 
  {
    digitalWrite(6, !digitalRead(6)); // toggle pin 6 so we can see when we send a packet
    // it's time to send a packet to the computer.  Fill
    // the first byte with a count of how many packets we've sent.
    // The computer can use this to detect if any packets are lost.
    //sendbuffer[63] = startwert++;
    RawHID.send(sendbuffer, 0); // 0 timeout = do not wait
    msUntilNextSend = 0;
    Serial.print(F("Sent packet, first byte: "));
    Serial.println((int)sendbuffer[0]);
  }
}

