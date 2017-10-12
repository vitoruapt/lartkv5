/********************************************************
 * PID Simple Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID_Beta6.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output, error;
double counter;
int inputSetpoint;
int reachedFlag = 0;

const int enablePin = 7; //7 and 10    // H-bridge enable pin

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,3,3,1);

void setup()
{
  //start serial and variables
  Serial.begin(9600);
  Input = analogRead(0);
  Setpoint = 180;// right in 5v aref 

  //turn the PID on
  myPID.SetMode(AUTO);
  myPID.SetOutputLimits(-255,255);
  myPID.SetInputLimits(170,490);
  myPID.SetSampleTime(10); //10 miliseconds

  // set enablePin high so that motor can turn on:
  digitalWrite(enablePin, HIGH);
  digitalWrite(8,HIGH); //set other motor enabled.
  counter = 0;
}

void loop(){
  inputSetpoint = serReadInt();
  if(inputSetpoint>0){
    Setpoint = (double)inputSetpoint;
    Serial.println(Setpoint);
    reachedFlag = 0;
  }
  
    
    
  //Serial.println("Setpoint: ");
  //Serial.print(Setpoint);
  //Serial.print("\t"); 
 
  //Input = analogRead(0);
  //Serial.print("Input: ");
  //Serial.print(Input);
  //Serial.print("\t");  
  //Serial.println();
  
  myPID.Compute();
  //Serial.print("Output: ");
  //Serial.print(Output);
  //Serial.print("\t");  
  //Serial.println();
  
  //compute error (to avoid control if within a limit)
  error = Input - Setpoint;
  //Serial.print("Error: ");
  //Serial.print(error);
  //Serial.print("\t");  
  //Serial.println(); 

  if(abs(error) < 20 && reachedFlag == 0){
    Serial.println("SetR");
    reachedFlag = 1;
    //counter++;
    //digitalWrite(enablePin, LOW);
    //delay(5000);
    
    //if (counter > 1000){
      /*counter = 0;
      switch((int)Setpoint){
        case 334:
          Setpoint = 180; //170
          //digitalWrite(enablePin, HIGH);
          break;
        case 180:
          Setpoint = 480; //485
          //digitalWrite(enablePin, HIGH);
          break;
        case 480:
          Setpoint = 334;
          //digitalWrite(enablePin, HIGH);
          break;
        }
      }*/
    }
    else{
    if (Output < 0){
      //Serial.println("spinning clockwise" );
      //Serial.println();    
      //analogWrite(6, abs(Output)); //6 and 8
      //analogWrite(5, 0); //5 and 9
      analogWrite(9, abs(Output)); //6 and 8
      analogWrite(10, 0); //5 and 9
    }
    else{
      //Serial.println("spinning counterclockwise" );    
      //Serial.println(); 
      //analogWrite(5, abs(Output));  //6 and 8
      //analogWrite(6, 0);  //5 and 9
      analogWrite(10, abs(Output));  //6 and 8
      analogWrite(9, 0);  //5 and 9      
    }
  }
}

int serReadInt(){
  int i, serAva;                           // i is a counter, serAva hold number of serial available
  char inputBytes [7];                 // Array hold input bytes
  char * inputBytesPtr = &inputBytes[0];  // Pointer to the first element of the array
     
  if (Serial.available()>0)            // Check to see if there are any serial input
  {
    delay(5);                              // Delay for terminal to finish transmitted
                                              // 5mS work great for 9600 baud (increase this number for slower baud)
    serAva = Serial.available();  // Read number of input bytes
    for (i=0; i<serAva; i++)       // Load input bytes into array
      inputBytes[i] = Serial.read();
    inputBytes[i] =  '\0';             // Put NULL character at the end
    return atoi(inputBytesPtr);    // Call atoi function and return result
  }
  else
    return -1;                           // Return -1 if there is no input
}


