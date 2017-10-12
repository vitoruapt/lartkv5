/* Free Software (Beer and Speech) */
/* Please give credit where credit is due. */
/* No stupid legalize, no warranty expressed or implied. */
/* This software is for those who love to create instead of bicker and hamper innovation. */
/* Author: Andrew Jalics */

/* Please see header file (uno_Counter.h) and Avago HCTL-2032 Datasheet for more technical details */

#include "WProgram.h"
#include "uno_Counter.h"

uno_Counter::uno_Counter(unsigned char countMode)
{
   //DDRA = B00000000;   // sets Arduino Mega (ATMEL ATMEGA) Digital pins 22(PORTA0) to 29(PORTA7) as inputs from HCTL-2032 - D0 to D7 

	
	//defnition of ports to reciev information
	pinMode(uno_Counter_PIN_0, INPUT);
	pinMode(uno_Counter_PIN_1, INPUT);
	pinMode(uno_Counter_PIN_2, INPUT);
	pinMode(uno_Counter_PIN_3, INPUT);
	pinMode(uno_Counter_PIN_4, INPUT);
	pinMode(uno_Counter_PIN_5, INPUT);
	pinMode(uno_Counter_PIN_6, INPUT);
	pinMode(uno_Counter_PIN_7, INPUT);
	
	
	

   //definition of port that defines values
   pinMode(uno_Counter_PIN_OE,   OUTPUT);
   pinMode(uno_Counter_PIN_SEL1, OUTPUT);
   pinMode(uno_Counter_PIN_SEL2, OUTPUT);
   pinMode(uno_Counter_PIN_RSTX, OUTPUT);
   
   // XY LOW  X Axis AKA 1st Axis
   // XY HIGH Y Axis AKA 2nd Axis
   //digitalWrite(uno_Counter_PIN_XY, LOW);

   digitalWrite(uno_Counter_PIN_OE, HIGH);  // Active LOW

  // switchCountMode( countMode );

   // Byte Selected MSB SEL1  LOW SEL2 HIGH
   // Byte Selected 2nd SEL1 HIGH SEL2 HIGH
   // Byte Selected 3rd SEL1  LOW SEL2 LOW
   // Byte Selected LSB SEL1 HIGH SEL2 LOW
   digitalWrite(uno_Counter_PIN_SEL1, LOW);
   digitalWrite(uno_Counter_PIN_SEL2, HIGH);

   digitalWrite(uno_Counter_PIN_RSTX, HIGH);  // Active LOW
 
   XAxisReset( );
  
}

// Communicates with a HCTL-2032 IC to get reset the X encoder count
// see Avago/Agilent/HP HCTL-2032 PDF for details
void uno_Counter::XAxisReset( )
{
   digitalWrite(uno_Counter_PIN_RSTX, LOW);
   delayMicroseconds(1);
   digitalWrite(uno_Counter_PIN_RSTX, HIGH);
   delayMicroseconds(1);
}

// Communicates with a HCTL-2032 IC to get the X Axis encoder count via an 8bit parallel bus
// see Avago/Agilent/HP HCTL-2032 Datasheet PDF for details
unsigned long uno_Counter::XAxisGetCount( )
{
   //digitalWrite(uno_Counter_PIN_XY,   LOW);
   digitalWrite(uno_Counter_PIN_OE,   LOW);
   digitalWrite(uno_Counter_PIN_SEL1, LOW);
   digitalWrite(uno_Counter_PIN_SEL2, HIGH);
   delayMicroseconds(1);
   busByte = ReadCount();
   count   = busByte;
   count <<= 8;

   digitalWrite(uno_Counter_PIN_SEL1, HIGH);
   digitalWrite(uno_Counter_PIN_SEL2, HIGH);
   delayMicroseconds(1);
   busByte = ReadCount();
   count  += busByte;
   count <<= 8;

   digitalWrite(uno_Counter_PIN_SEL1, LOW);
   digitalWrite(uno_Counter_PIN_SEL2, LOW);
   delayMicroseconds(1);
   busByte = ReadCount();
   count  += busByte;
   count <<= 8;

   digitalWrite(uno_Counter_PIN_SEL1, HIGH);
   digitalWrite(uno_Counter_PIN_SEL2, LOW);
   delayMicroseconds(1);
   busByte = ReadCount();
   count  += busByte;

   digitalWrite(uno_Counter_PIN_OE,  HIGH);

   return count;
}
/*
// Communicates with a HCTL-2032 IC to get reset the Y encoder count
// see Avago/Agilent/HP HCTL-2032 PDF for details
void uno_Counter::YAxisReset( )
{
   digitalWrite(uno_Counter_PIN_RSTY, LOW);
   delayMicroseconds(1);
   digitalWrite(uno_Counter_PIN_RSTY, HIGH);
   delayMicroseconds(1);
}

// Communicates with a HCTL-2032 IC to get the Y Axis encoder count via an 8bit parallel bus
// see Avago/Agilent/HP HCTL-2032 PDF for details
unsigned long uno_Counter::YAxisGetCount( )
{
   digitalWrite(uno_Counter_PIN_XY,   HIGH);
   digitalWrite(uno_Counter_PIN_OE,   LOW);
   digitalWrite(uno_Counter_PIN_SEL1, LOW);
   digitalWrite(uno_Counter_PIN_SEL2, HIGH);
   delayMicroseconds(1);
   busByte = ReadCount();
   count   = busByte;
   count <<= 8;

   digitalWrite(uno_Counter_PIN_SEL1, HIGH);
   digitalWrite(uno_Counter_PIN_SEL2, HIGH);
   delayMicroseconds(1);
   busByte = ReadCount();
   count  += busByte;
   count <<= 8;

   digitalWrite(uno_Counter_PIN_SEL1, LOW);
   digitalWrite(uno_Counter_PIN_SEL2, LOW);
   delayMicroseconds(1);
   busByte = ReadCount();
   count  += busByte;
   count <<= 8;

   digitalWrite(uno_Counter_PIN_SEL1, HIGH);
   digitalWrite(uno_Counter_PIN_SEL2, LOW);
   delayMicroseconds(1);
   busByte = ReadCount();
   count  += busByte;

   digitalWrite(uno_Counter_PIN_OE,  HIGH);

   return count;
}

// Communicates with a HCTL-2032 IC to set the count mode
// see Avago/Agilent/HP HCTL-2032 PDF for details
void uno_Counter::switchCountMode( unsigned char countMode )
{
   // Count Mode Illegal Mode EN1 LOW  EN2 LOW
   // Count Mode   4X         EN1 HIGH EN2 LOW
   // Count Mode   2X         EN1 LOW  EN2 HIGH
   // Count Mode   1X         EN1 HIGH EN2 HIGH
   switch(countMode)
   {
      case 1: // 1X Count Mode
         digitalWrite(uno_Counter_PIN_EN1, HIGH);
         digitalWrite(uno_Counter_PIN_EN2, HIGH);
         break;
      case 2: // 2X Count Mode
         digitalWrite(uno_Counter_PIN_EN1, LOW);
         digitalWrite(uno_Counter_PIN_EN2, HIGH);
         break;
      case 4: // 4X Count Mode is the default
      default:
         digitalWrite(uno_Counter_PIN_EN1, HIGH);
         digitalWrite(uno_Counter_PIN_EN2, LOW);
         break;
   }
   delayMicroseconds(1);
}
*/
byte uno_Counter::ReadCount(void)
{
	byte count_value=0b00000000;
	
	byte p0 = digitalRead(uno_Counter_PIN_0);
	byte p1 = digitalRead(uno_Counter_PIN_1);
	byte p2 = digitalRead(uno_Counter_PIN_2);
	byte p3 = digitalRead(uno_Counter_PIN_3);
	byte p4 = digitalRead(uno_Counter_PIN_4);
	byte p5 = digitalRead(uno_Counter_PIN_5);
	byte p6 = digitalRead(uno_Counter_PIN_6);
	byte p7 = digitalRead(uno_Counter_PIN_7);
	
	
	//change of ports state
	if(p0)
	{count_value = count_value | 0b00000001;}
	
	if(p1)
	{count_value = count_value | 0b00000010;}
	
	if(p2)
	{count_value = count_value | 0b00000100;}
	
	if(p3)
	{count_value = count_value | 0b00001000;}
	
	if(p4)
	{count_value = count_value | 0b00010000;}
	
	if(p5)
	{count_value = count_value | 0b00100000;}
	
	if(p6)
	{count_value = count_value | 0b01000000;}
	
	if(p7)
	{count_value = count_value | 0b10000000;}
	
	
	
	
	return count_value;
}
