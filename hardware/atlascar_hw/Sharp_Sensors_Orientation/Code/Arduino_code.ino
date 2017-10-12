#include "Timer.h"
#include <math.h>

int ledPin = 13;      // select the pin for the LED

boolean lled = HIGH;
Timer t;
double freq = 40;

int N = 512;
int mem[512];
int n=0;

// variable to store the value coming from the sensor
int aa0, aa1, aa2, aa3, aa4, aa5, aa6, aa7;

void sendStatus()
{
  //lled=!lled;
  //digitalWrite(ledPin, lled);  
  // Ler A0
  String msg = "0 ";
  msg += analogRead(A0);
  
  msg += " 1 ";
  msg += analogRead(A1);
  
  msg += " 2 ";
  msg += analogRead(A2);
  
  msg += " 3 ";
  msg += analogRead(A3);
  
  msg += " 4 ";
  msg += analogRead(A4);
  
  msg += " 5 ";
  msg += analogRead(A5);
  
  msg += " 6 ";
  msg += analogRead(A6);
  
  msg += " 7 ";
  msg += analogRead(A7);
  
 
  
  Serial.println(msg);
  
  //volt=getvolt(sensorValue);
 // dist=getdist(volt);
 /*if(n==N+1)
 {
   int i=0;
   for(i=0;i<=N;i++)
   {
      Serial.print("A0:");
      Serial.print(mem[i]);
      Serial.print("\n");
   }
  n=0;
 }
 
 mem[n]=sensorValue;
 n++;*/
  // Ler A1
  /*sensorValue1= analogRead(A1);
  //volt=getvolt(sensorValue);
 // dist=getdist(volt);
  Serial.print("A1:");
  Serial.print(sensorValue1);
  Serial.print("\n");*/
  
  /*
  // Ler A2
  sensorValue= analogRead(A2);
  volt=getvolt(sensorValue);
 // dist=getdist(volt);
  Serial.print("A2:");
  Serial.print(volt);
  Serial.print("\n");
  // Ler A3
  sensorValue= analogRead(A3);
  volt=getvolt(sensorValue);
 // dist=getdist(volt);
  Serial.print("A3:");
  Serial.print(volt);
  Serial.print("\n");*/
  
}

void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
  //memset(mem,0,sizeof(int)*2048);
  //Schedule the send function using the timer class
  t.every(1000/freq, sendStatus);
}

void loop() {

  t.update();
}





