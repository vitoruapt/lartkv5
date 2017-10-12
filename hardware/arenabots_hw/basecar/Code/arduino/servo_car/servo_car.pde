/*
  Analog input, analog output, serial output
 
 Reads an analog input pin, maps the result to a range from 0 to 255
 and uses the result to set the pulsewidth modulation (PWM) of an output pin.
 Also prints the results to the serial monitor.
 
 The circuit:
 * potentiometer connected to analog pin 0.
   Center pin of the potentiometer goes to the analog pin.
   side pins of the potentiometer go to +5V and ground
 * LED connected from digital pin 9 to ground
 
 created 29 Dec. 2008
 by Tom Igoe
 
 */

// These constants won't change.  They're used to give names
// to the pins used:
const int potentiometer = 0;  // Analog input pin that the potentiometer is attached to
const int led = 13; // Analog output pin that the LED is attached to

const int analogOutPin10 = 5; // Analog output pin that the LED is attached to
const int analogOutPin11 = 6; // Analog output pin that the LED is attached to

const int upSpeed = 2;
const int downSpeed = 3;

int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)

double reference = 86;
double error = 0;
double integrated_error = 0;
double kp = 0;
double ki = 0;
double signal = 0;
double sign = 1;

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 
  
  pinMode(potentiometer, INPUT); 
  pinMode(led, INPUT);
  pinMode(upSpeed, INPUT); 
  pinMode(downSpeed, INPUT);   
  pinMode(analogOutPin10, INPUT); 
  pinMode(analogOutPin11, INPUT);

  // set enablePin high so that motor can turn on:
  digitalWrite(enablePin, HIGH); 
}

void loop() {
  // read the analog in value:
  sensorValue = analogRead(potentiometer);            
  // map it to the range of the analog out:
  outputValue = map(sensorValue, 0, 1023, 0, 255);  
  // change the analog out value:
  analogWrite(led, outputValue);           

  // print the results to the serial monitor:
  Serial.print("sensor = " );                       
  Serial.print(sensorValue);      
  Serial.print(", sensor mapped = ");      
  Serial.println(outputValue);
  Serial.print(", reference = ");      
  Serial.println(reference);
  
/*  
if (digitalRead(upSpeed) == HIGH && motorSpeed < 255){
  motorSpeed = motorSpeed + 5;      
  // print the results to the serial monitor:
  Serial.print("up speed, new value = " );                       
  Serial.println(motorSpeed);      
}

if (digitalRead(downSpeed) == HIGH && motorSpeed > 0){
  motorSpeed = motorSpeed - 5;      
  // print the results to the serial monitor:
  Serial.print("down speed, new value = " );                       
  Serial.println(motorSpeed);
}
*/
  

  error = reference - (double)outputValue;
  sign = (integrated_error+error)/(abs(integrated_error+error)+0.00001);  
  Serial.print("sign = ");      
  Serial.println(sign);
  integrated_error = sign*abs(integrated_error + error);
  
  
  //while outbounds
  if(abs(error) > 10){
    
    Serial.println();
    Serial.print("error: ");  
    Serial.println(error); 
    
    Serial.print("integrated error: ");  
    Serial.println(integrated_error); 
    
    kp = 255/120;
    ki = 0.1;
    
    signal = abs((kp*error)+(ki*integrated_error));
    signal = min(signal,255);
    Serial.print("signal: ");  
    Serial.println(signal);     
    
    //spin counterclockwise
    if(error < 0){
      Serial.println("spinning counterclockwise" );
      analogWrite(analogOutPin11, (int)signal);
      analogWrite(analogOutPin10, 0); 
  
      //stop
      //delay(50);
      //analogWrite(analogOutPin10, 0); 
      //analogWrite(analogOutPin11, 0);

    }
    //else, spin clockwise
    else if(error > 0){
      Serial.println("spinning clockwise" );    
      analogWrite(analogOutPin10, (int)signal);  
      analogWrite(analogOutPin11, 0);

      //stop
      //delay(50);
      //analogWrite(analogOutPin10, 0); 
      //analogWrite(analogOutPin11, 0);
    }

  }
  //else error is inside threshold, change reference
  else{
    Serial.println("reference reached!!!!!!!!!!!!!!!!!!!" );
    analogWrite(analogOutPin11, 0); 
    analogWrite(analogOutPin10, 0);
    delay(2000);
       
    if (reference == 86){
      reference = 45;
      Serial.println("reference changed");
    }
    else{
      reference = 86;
      Serial.println("reference changed");      
    }
  }
  

  // wait 10 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(10);                     
}
