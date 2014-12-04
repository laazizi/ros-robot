/* Free Software (Beer and Speech) */
/* Please give credit where credit is due. */
/* No stupid legalize, no warranty expressed or implied. */
/* This software is for those who love to create instead of bicker and hamper innovation. */
/* Author: Andrew Jalics */

#ifndef __MEGA_QUADRATURE_ENCODER_COUNTER__
#define __MEGA_QUADRATURE_ENCODER_COUNTER__ 1

#include "Arduino.h"

class MegaEncoderCounter
{
   public:
      MegaEncoderCounter( unsigned char countMode=4 );

      void          XAxisReset( );
      unsigned long XAxisGetCount( );
      void          YAxisReset( );
      unsigned long YAxisGetCount( );

      void          switchCountMode( unsigned char countMode );

   private:
      unsigned long count;
      unsigned char busByte;
};

#define MEGA_QUADRATURE_ENCODER_COUNTER_PIN_XY   37
#define MEGA_QUADRATURE_ENCODER_COUNTER_PIN_OE   36 // Active LOW
#define MEGA_QUADRATURE_ENCODER_COUNTER_PIN_EN1  35
#define MEGA_QUADRATURE_ENCODER_COUNTER_PIN_EN2  34
#define MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1 33
#define MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2 32
#define MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTX 31 // Active LOW
#define MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTY 30 // Active LOW

// The following comments are for reference only...
// (Comments are based on Arduino Mega 1280, should work for all Arduino Megas in theory...)

// Arduino Mega Digital Pin 30 [ATMEL ATMEGA PORTC 7] -> HCTL 2032 Pin 11 RSTY - Active LOW
// Arduino Mega Digital Pin 31 [ATMEL ATMEGA PORTC 6] -> HCTL 2032 Pin 12 RSTX - Active LOW
// Arduino Mega Digital Pin 32 [ATMEL ATMEGA PORTC 5] -> HCTL 2032 Pin 26 SEL2
// Arduino Mega Digital Pin 33 [ATMEL ATMEGA PORTC 4] -> HCTL 2032 Pin  6 SEL1
// Arduino Mega Digital Pin 34 [ATMEL ATMEGA PORTC 3] -> HCTL 2032 Pin  3 EN2
// Arduino Mega Digital Pin 35 [ATMEL ATMEGA PORTC 2] -> HCTL 2032 Pin  2 EN1
// Arduino Mega Digital Pin 36 [ATMEL ATMEGA PORTC 1] -> HCTL 2032 Pin  7 OE   - Active LOW
// Arduino Mega Digital Pin 37 [ATMEL ATMEGA PORTC 0] -> HCTL 2032 Pin 32 X/Y 

// Arduino Mega Digital Pin 22 [ATMEL ATMEGA PORTA 0] <- HCTL 2032 Pin  1 D0
// Arduino Mega Digital Pin 23 [ATMEL ATMEGA PORTA 1] <- HCTL 2032 Pin 15 D1
// Arduino Mega Digital Pin 24 [ATMEL ATMEGA PORTA 2] <- HCTL 2032 Pin 14 D2
// Arduino Mega Digital Pin 25 [ATMEL ATMEGA PORTA 3] <- HCTL 2032 Pin 13 D3
// Arduino Mega Digital Pin 26 [ATMEL ATMEGA PORTA 4] <- HCTL 2032 Pin 12 D4
// Arduino Mega Digital Pin 27 [ATMEL ATMEGA PORTA 5] <- HCTL 2032 Pin 11 D5
// Arduino Mega Digital Pin 28 [ATMEL ATMEGA PORTA 6] <- HCTL 2032 Pin 10 D6
// Arduino Mega Digital Pin 29 [ATMEL ATMEGA PORTA 7] <- HCTL 2032 Pin  9 D7

// HCTL-2032 Count Mode Info
// Count Mode Illegal Mode EN1 LOW  EN2 LOW
// Count Mode   4X         EN1 HIGH EN2 LOW
// Count Mode   2X         EN1 LOW  EN2 HIGH
// Count Mode   1X         EN1 HIGH EN2 HIGH

// HCTL-2032 Byte Selected Info
// Byte Selected MSB SEL1  LOW SEL2 HIGH
// Byte Selected 2nd SEL1 HIGH SEL2 HIGH
// Byte Selected 3rd SEL1  LOW SEL2 LOW
// Byte Selected LSB SEL1 HIGH SEL2 LOW

// HCTL-2032 X/Y Info
// XY LOW  X Axis AKA 1st Axis
// XY HIGH Y Axis AKA 2nd Axis

// Quadrature Encoder connections US Digital
// orange solid  5V
// blue solid    A
// orange stripe GND
// blue stripe   B

// Quadrature Encoder connections  HEDM-5500
// Pin 1 GND
// Pin 2 NC
// Pin 3 A
// Pin 4 5V
// Pin 5 B

// Maxon Motor HEDM-5500 Quadrature Encoder connections
// white  5V
// brown  GND
// green  A
// yellow B

#endif
