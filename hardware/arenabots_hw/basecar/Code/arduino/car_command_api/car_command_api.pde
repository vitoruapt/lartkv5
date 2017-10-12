/********************************************************
 * Car Command - interface with the base traction and
 * steer motor, using a l293e
 ********************************************************/

#include <PID_Beta6.h>
#include <XBee.h>
#include <avr/sleep.h>

double setpoint, input, output, error;
int inputSetpoint;
int reachedFlag;
/////////////
int serial_input;
char inputBytes [7];
char* inputPointer;
/////////////
int motorSpeed; //minimal to start moving (no load?)
int steerDirection; //minimal to start moving (no load?)
int tolerance; //error accepted for steering
boolean directionFlag;
//// xbee related /////
XBee xbee = XBee();
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();
//// timer related //////
long startTime; // start time for stop watch
long elapsedTime; // elapsed time for stop watch

//outputs
const int enableTraction = 8; // H-bridge enable pin
const int enableSteer = 7; // H-bridge enable pin
const int tractionPinA = 5;    // H-bridge pin 2
const int tractionPinB = 6;    // H-bridge leg 9
const int steerPinA = 9;    // H-bridge pin 12
const int steerPinB = 10;    // H-bridge leg 19
const int ledPin = 13;      // LED 
const int statusLed = 12;   //xbee signaling
const int wakePin = 2;                 // pin used for waking up

double Kp=3, Ki=3, Kd=1;
//double Kp=2.5, Ki=3.5, Kd=1; //only for subaru

//Specify the links and initial tuning parameters
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd);

void setup()
{
	// set IOs
	pinMode(wakePin, INPUT);

	pinMode(tractionPinA, OUTPUT); 
	pinMode(tractionPinB, OUTPUT);
	pinMode(steerPinA, OUTPUT); 
	pinMode(steerPinB, OUTPUT); 
	pinMode(enableTraction, OUTPUT);
	pinMode(enableSteer, OUTPUT);
	pinMode(ledPin, OUTPUT);
	pinMode(statusLed, OUTPUT);

	// start serial
	//xbee.begin(19200);
	xbee.begin(9600);

	//turn the PID on
	myPID.SetMode(AUTO);
	myPID.SetOutputLimits(-255,255);
	myPID.SetInputLimits(260,760);
	myPID.SetSampleTime(10);

	// set enablePin high so that motor can turn on:
	digitalWrite(enableTraction, HIGH); 
	digitalWrite(enableSteer, HIGH); 

	//initialize vars
	input = analogRead(0);
	setpoint = 270;//new 3.3v ref. steering center (may change value)
	reachedFlag = 0;
	motorSpeed = 0; //start speed
	directionFlag = 1;
	tolerance = 20; //20 in old ref

	// use interrupt 0 (pin 2) and run function
	// wakeUpNow when pin 2 gets LOW
	startTime = millis();

	blink(ledPin, 3, 100); //program starts
}

void loop()
{
        //Serial.println("looping");
        //Serial.println();
	//blink(ledPin, 2, 100); //program starts
        //delay(500);
	elapsedTime = millis() - startTime;
	
	if (elapsedTime >= 10000)
	{
		//Serial.println("Timer: Entering Sleep mode");
		delay(100);     // this delay is needed for the sleep 
		sleepNow();     // sleep function called here
	}
	
	xbee.readPacket();
    
	if (xbee.getResponse().isAvailable())
	{
                //Serial.println("available");
                //Serial.println();
	//if received message restart timer  
		startTime = millis();
		// got something
		if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE)
		{
			// got a zb rx packet
			// now fill our zb rx class
			xbee.getResponse().getZBRxResponse(rx);
			if (rx.getOption() == ZB_PACKET_ACKNOWLEDGED)
			{
			// the sender got an ACK
			//Serial.println("message received");
			//blink(statusLed, 2, 100);
			}
			else
			{
				// we got it (obviously) but sender didn't get an ACK
				//blink(statusLed, 5, 100);
			}

			//check dir signal and receive value
			if ((rx.getData(0) & 0x01)==0x01)
			{
				//Serial.println("negative direction");
				//setpoint = (int)(-1*rx.getData(1) + 330);
				setpoint = (int)(-1*rx.getData(1) + 510);
				//Serial.println(setpoint,DEC);
				//Serial.println();
			}
			else
			{
				//Serial.println("positive direction");
				//setpoint = (int)(rx.getData(1) + 330);
				setpoint = (int)(rx.getData(1) + 510);
				//Serial.println(setpoint,DEC);
				//Serial.println();
			}          
			
			//check speed signal and receive value
			if ((rx.getData(0) & 0x02)==0x02){
				//Serial.println("negative speed");
				motorSpeed = (int)(-1*rx.getData(2));
				//Serial.println(motorSpeed,DEC);
				//Serial.println();
			}
			else
			{
			//Serial.println("positive speed");
			motorSpeed = (int)(rx.getData(2));
			//Serial.println(motorSpeed,DEC);
			//Serial.println();
		}       
		} 
	}   
  
	/////////// traction part ///////////
	//send speed command according to motorSpeed value
	if (motorSpeed >= 0)
	{
		analogWrite(tractionPinA, 0);   
		analogWrite(tractionPinB, motorSpeed);
	} 
	else
	{
		analogWrite(tractionPinA, -motorSpeed);
		analogWrite(tractionPinB, 0);
	}
  
	////////// steer part //////////////
	input = analogRead(0);
	/*Serial.print("Input: ");
	Serial.print(input);
	Serial.print(",");  
	//Serial.println();*/

        /*Serial.print("Setpoint: ");
	Serial.print(setpoint);
	Serial.print(",");  
	//Serial.println();*/
	
	//compute error (to avoid control if within a limit)
	error = abs(input - setpoint);
	/*Serial.print("Error: ");
	Serial.print(error);
	//Serial.print(",");  
	Serial.println();*/

        myPID.Compute();
	/*Serial.print("Output: ");
	Serial.print(output);
	Serial.print(",");  
	//Serial.println();*/
	
	if(error < tolerance && reachedFlag == 0)
	{
		//Serial.println("SetR");
		reachedFlag = 1;
	}
	else
	{
		if (output < 0)
		{
			//Serial.println("spinning clockwise" );
			//Serial.println();    
			analogWrite(steerPinA, abs(output));
			analogWrite(steerPinB, 0);
		}
		else
		{
			//Serial.println("spinning counterclockwise" );    
			//Serial.println(); 
			analogWrite(steerPinA, 0);
			analogWrite(steerPinB, abs(output));
		}
	}
}

/*
  blinks an LED
 */
void blink(int whatPin, int howManyTimes, int milliSecs)
{
	int i = 0;
	for ( i = 0; i < howManyTimes; i++)
	{
		digitalWrite(whatPin, HIGH);
		delay(milliSecs/2);
		digitalWrite(whatPin, LOW);
		delay(milliSecs/2);
	}
}

// here the interrupt is handled after wakeup
void wakeUpNow()        
{
        //Serial.println("waking up");
        //Serial.println();
	digitalWrite(enableTraction, HIGH); 
	digitalWrite(enableSteer, HIGH);
	startTime = millis();
  // execute code here after wake-up before returning to the loop() function
  // timers and code using timers (serial.print and more...) will not work here.
  // we don't really need to execute any special functions here, since we
  // just want the thing to wake up
}

// here we put the arduino to sleep
void sleepNow()         
{
        //Serial.println("sleeping");
        //Serial.println();  
	blink(ledPin, 5, 50); //program starts
	digitalWrite(enableTraction, LOW); 
	digitalWrite(enableSteer, LOW); 
	
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
        sleep_enable();          // enables the sleep bit in the mcucr register

        attachInterrupt(0,wakeUpNow, LOW); //wakeUpNow when pin 2 gets LOW 
        sleep_mode();            // here the device is actually put to sleep!!
                             // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
        sleep_disable();         // first thing after waking from sleep:
        detachInterrupt(0);      // disables interrupt 0 on pin 2 so the 
}


