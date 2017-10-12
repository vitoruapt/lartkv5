#include <uno_Counter.h>

uno_Counter uno_Counter(4); // Initializes the Mega Encoder Counter in the 4X Count Mode

char incomingByte;
unsigned long count;
unsigned char iter;

void setup()
{
   Serial.begin(9600);
   RobogaiaMegaQuadratureEncoderCounterShieldDemoHelp();
   Serial.println(" [Power up or Reset of Arduino Occured]");
   Serial.println("Robogaia UNO Quadrature Encoder Counter Shield Demo -> ");
}

void loop() 
{ 
   if (Serial.available() > 0) 
   {
      incomingByte = Serial.read();
      if(incomingByte == '\r')
      {
         return;
      }
      switch(incomingByte)
      {
         case 'u':
            Serial.println("Resetting the quadrature encoder X counter");
            uno_Counter.XAxisReset();
            break;
         case 'i':
            Serial.println("Reading the quadrature encoder X counter");
            count = uno_Counter.XAxisGetCount();
            Serial.print("Decimal    : ");
            Serial.print(count);
            Serial.println(" ");
            break;
         case 'o':
            Serial.println("Reading the quadrature encoder X counter extended");
            for( iter = 0 ; iter < 255 ; iter++)
            {
               count = uno_Counter.XAxisGetCount();
               Serial.print("Decimal    : ");
               Serial.print(count);
               Serial.println(" ");
               delay(100);
            }
            break;
         default:
            Serial.println("Unknown Entry Event");
            RobogaiaMegaQuadratureEncoderCounterShieldDemoHelp();
            break;
      }
      Serial.println("Robogaia UNO Quadrature Encoder Counter Shield Demo -> ");
   }
   
}

void RobogaiaMegaQuadratureEncoderCounterShieldDemoHelp()
{
   Serial.println("Robogaia Mega Quadrature Encoder Counter Shield Demo"); 
   Serial.println(" u -                    Reset X Axis quadrature encoder counter");
   Serial.println(" i -                    Read X Axis quadrature encoder counter");   
   Serial.println(" o -                    Read X Axis quadrature encoder counter extended, Reads and Prints the count 255 times over 25 seconds"); 
   Serial.println(" h -                    Shows this Help Screen");   
}