/********************************************************
 * Car Command - interface with the base traction and
 * steer motor, using a l293e
 ********************************************************/

#include <PID_Beta6.h>

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

//inputs
const int switchDirection = 2;    // switch direction pushbutton
const int upSpeed = 3;      //pushbutton: increase speed
const int downSpeed = 4;    //pushbutton: decrease speed

//outputs
const int enableTraction = 7; // H-bridge enable pin
const int enableSteer = 8; // H-bridge enable pin
const int tractionPinA = 5;    // H-bridge pin 2
const int tractionPinB = 6;    // H-bridge leg 9
const int steerPinA = 9;    // H-bridge pin 12
const int steerPinB = 10;    // H-bridge leg 19
const int ledPin = 13;      // LED 

//Specify the links and initial tuning parameters
PID myPID(&input, &output, &setpoint,3,3,1);

void setup() {

  // set the switch as an input:
  pinMode(switchDirection, INPUT); 
  pinMode(upSpeed, INPUT); 
  pinMode(downSpeed, INPUT); 

  // set all the other pins you're using as outputs:
  pinMode(tractionPinA, OUTPUT); 
  pinMode(tractionPinB, OUTPUT);
  pinMode(steerPinA, OUTPUT); 
  pinMode(steerPinB, OUTPUT); 
  pinMode(enableTraction, OUTPUT);
  pinMode(enableSteer, OUTPUT);
  pinMode(ledPin, OUTPUT);

  //start serial and variables
  Serial.begin(9600);//Serial.begin(9600);

  //turn the PID on
  myPID.SetMode(AUTO);
  myPID.SetOutputLimits(-255,255);
  myPID.SetInputLimits(160,490);
  myPID.SetSampleTime(10);

   // set enablePin high so that motor can turn on:
  digitalWrite(enableTraction, HIGH); 
  digitalWrite(enableSteer, HIGH); 

  //initialize vars
  input = analogRead(0);
  setpoint = 180; //334 old center (may change value)
  reachedFlag = 0;
  motorSpeed = 0; //start speed
  directionFlag = 1;
  tolerance = 20;

  blink(ledPin, 3, 100); //program starts
}

void loop() {
  
  serialReader(); //serial_input = serialReader();
  /*if (serial_input != -1){
    Serial.println("serial input:");
    Serial.println(inputBytes);
    Serial.println(inputBytes[0]);
    delay(1000);
  }*/

  
  if(inputBytes[0] == 's'){
    //Serial.println("is:");
    inputPointer = inputBytes;
    inputPointer++;
    motorSpeed = atoi(inputPointer);
    Serial.println("ms");
    Serial.println(motorSpeed);
    //delay(3000);
  }
  
  if(inputBytes[0] == 'd'){
    //Serial.println("is:");
    inputPointer = inputBytes;
    inputPointer++;
    setpoint = atoi(inputPointer) + 330;
    Serial.println("sd");
    Serial.println(setpoint);
    //delay(3000);
  }
  
  /////////// traction part ///////////

  //if speedup button is pressed
  if (digitalRead(upSpeed) == HIGH){
    speedup();    
  }

  //if speeddown button is pressed
  if (digitalRead(downSpeed) == HIGH){
    speeddown();
  }
  
  // if the switch is pressed, motor will change direction:
  //if (digitalRead(switchDirection) == HIGH) {
  //  delay(100);
  //  directionFlag = !directionFlag;
  //}
  
  //send speed command according to motorSpeed value
  if (motorSpeed >= 0){
    analogWrite(tractionPinA, 0);   
    analogWrite(tractionPinB, motorSpeed);
  } 
  else {
    analogWrite(tractionPinA, -motorSpeed);
    analogWrite(tractionPinB, 0);
  }
  
  ////////// steer part //////////////
  
  input = analogRead(0);
  //Serial.print("Input: ");
  //Serial.print(input);
  //Serial.print("\t");  
  //Serial.println();
  
  myPID.Compute();
  //Serial.print("Output: ");
  //Serial.print(output);
  //Serial.print("\t");  
  //Serial.println();
  
  //compute error (to avoid control if within a limit)
  error = input - setpoint;
  //Serial.print("Error: ");
  //Serial.print(error);
  //Serial.print("\t");  
  //Serial.println(); 

  if(abs(error) < tolerance && reachedFlag == 0){
    Serial.println("SetR");
    reachedFlag = 1;
  }
    else{
    if (output < 0){
      //Serial.println("spinning clockwise" );
      //Serial.println();    
      analogWrite(steerPinA, abs(output));
      analogWrite(steerPinB, 0);
    }
    else{
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
void blink(int whatPin, int howManyTimes, int milliSecs) {
  int i = 0;
  for ( i = 0; i < howManyTimes; i++) {
    digitalWrite(whatPin, HIGH);
    delay(milliSecs/2);
    digitalWrite(whatPin, LOW);
    delay(milliSecs/2);
  }
}

/*
  speed up
 */
void speedup(void) {
  if(motorSpeed <= 250){
    motorSpeed = motorSpeed + 5;      
    // print the results to the serial monitor:
    Serial.print("up speed, new value = " );                       
    Serial.println(motorSpeed);
    delay(100);    
  }
}

/*
  speed down
 */
void speeddown(void) {
  if(motorSpeed >= 5){
    motorSpeed = motorSpeed - 5;      
     // print the results to the serial monitor:
    Serial.print("down speed, new value = " );                       
    Serial.println(motorSpeed);      
    delay(100);
  }
}

/*
  speed down
*/

void serialReader(){
  int i, serAva;                           // i is a counter, serAva hold number of serial available
  //char inputBytes [7];                 // Array hold input bytes
       
  if (Serial.available()>0)            // Check to see if there are any serial input
  {
    delay(5);                              // Delay for terminal to finish transmitted
                                              // 5mS work great for 9600 baud (increase this number for slower baud)
    serAva = Serial.available();  // Read number of input bytes
    
    for (i=0; i<serAva; i++){       // Load input bytes into array
      inputBytes[i] = Serial.read();
    }
    
    inputBytes[i] =  '\0';             // Put NULL character at the end
    //return *inputBytes;
    
    }
  else
    inputBytes[0] =  '\0';               // Put NULL character at the end if there is no input
}

