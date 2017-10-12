

#include <Wire.h>//Wire library for communication with the potenciometer
#include <SPI.h>//SPI library for communication with the ethernet chip
#include <Ethernet.h>//Ethernet library
#include <Servo.h>//Servo library for servo actuator control
#include <string.h>
#include "Timer.h"

//Potenciomter adress
#define I2C_POT_ADDRESS 0b0101100
//Write command
#define I2C_WRITE_BIT 0b11111110
//Read command
#define I2C_READ_BIT 0b11111111

#define RECEIVE_BUFFER_SIZE 100

//Configuration adress 0 of the potenciometer
#define MSG_MEM_TCON0 0x4d	//0x4c
//Configuration adress 1 of the potenciometer
#define MSG_MEM_TCON1 0xAd
//Adress of the potenciomter 0 in the chip
#define MSG_MEM_WP0 0x0c	//0x2c
//Adress of the potenciomter 1 in the chip
#define MSG_MEM_WP1 0x1c	//0x3c
//Adress of the potenciomter 2 in the chip
#define MSG_MEM_WP2 0x6c	//0x8c
//Adress of the potenciomter 3 in the chip
#define MSG_MEM_WP3 0x7c	//0x9c

#define CMD_READ 0b11111111
#define CMD_INC 0b11110111
#define CMD_DEC 0b11111011
#define CMD_WRITE 0b11110011

#define AUTO 1
#define MANUAL 0

byte mac[] = { 
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };//mac adress
IPAddress ip(10,0,0,55);//Ip adress of the arduino
IPAddress gateway(0,0,0,0);//Gateway of the net
IPAddress subnet(255, 255, 255, 0);//Subnet mask

EthernetServer server(1000);
boolean gotAMessage = false; // whether or not you got a message from the client yet

Servo srv;//Servo class
const int mode_selection_pin = 8;//Input to use as the mode selection
const int borboleta_input = A1;//Input pin for the analog borboleta value
const int pedal_input = A2;//Input pin for the analog pedal value
const int centralina_pot = 3;
const int centralina_min_value_pot = 1;
const int servo_value_pot = 2;

//borboleta min= 0.5 max 2.8
double b_min = 1.45;
double b_max = 1.72;

//centralina min 0.4 max 3.4
double c_min = 0.98; //->
double c_max = 4.0;

//define the limits for the conversion to the centralina potenciometer limits  
double cp_min = 0;//this value corresponds to c_min
double cp_max = 255;//this value to c_max

//limits for the conversion from borboleta value to servo potenciomter value
double sp_min = 255;// 255 - 0.23V (servo min)
double sp_max = 0;// 0 - 1.0V (servo max)

//pedal potenciometer min and max values, also ethernet
double ctl_min = 0;
double ctl_max = 3.8;

double command_value_auto=0;
int mode = MANUAL;
double last_control=0;

char receive_buffer[RECEIVE_BUFFER_SIZE];
char subject[RECEIVE_BUFFER_SIZE];
char complement[RECEIVE_BUFFER_SIZE];

double throttle = 0;
double pedal = 0;

double freq = 50;
Timer t;
EthernetClient *pclient;

void SetPotValue(char id,int val)
{
  int ret=0;

  Wire.beginTransmission(byte(I2C_POT_ADDRESS));

  byte cmd;
  byte vl;

  switch(id)
  {
  case 0:
    cmd=byte(MSG_MEM_WP0 & CMD_WRITE);
    break;

  case 1:
    cmd=byte(MSG_MEM_WP1 & CMD_WRITE);
    break;

  case 2:
    cmd=byte(MSG_MEM_WP2 & CMD_WRITE);
    break;

  case 3:
    cmd=byte(MSG_MEM_WP3 & CMD_WRITE);
    break;
  }

  if(val & 0b0000000100000000)
    cmd=cmd | 0b00000001;
  else
    cmd=cmd | 0b00000000;

  vl=val & 0b11111111;

  Wire.write(cmd);      // sets register pointer to the command register (0x00)
  Wire.write(vl);

  ret=Wire.endTransmission();
  if(ret!=0)
  {
    //Serial.print("COM Error! ");
    Serial.println(ret);
  }
}

byte ReadPotValue(char id)
{
  Wire.beginTransmission(byte(I2C_POT_ADDRESS)); // transmit to

  switch(id)
  {
  case 0:
    Wire.write(byte(MSG_MEM_WP0 & CMD_READ));      // sets register pointer to the command register (0x00)  
    break;

  case 1:
    Wire.write(byte(MSG_MEM_WP1 & CMD_READ));      // sets register pointer to the command register (0x00)  
    break;

  case 2:
    Wire.write(byte(MSG_MEM_WP2 & CMD_READ));      // sets register pointer to the command register (0x00)  
    break;

  case 3:
    Wire.write(byte(MSG_MEM_WP3 & CMD_READ));      // sets register pointer to the command register (0x00)  
    break;
  }

  Wire.endTransmission();

  Wire.requestFrom(I2C_POT_ADDRESS, 2);

  byte c;
  while(Wire.available())    // slave may send less than requested
    c = Wire.read();    // receive a byte as character

  return c;
}

double map_local(double val,double aMin,double aMax,double bMin,double bMax)
{
  //create a calibration line using the min and maximum espected values
  double m = (bMax-bMin)/(aMax-aMin);
  double b = bMax-m*aMax;
  return val*m+b;
}

double ReadBorboletaPotenciometer(void)
{
  int value=analogRead(borboleta_input);

  double b_value = (5./1024.)*value + 0.04;//the residual addition is just a small correction

  return b_value;
}

double ReadPedalPotenciometer(void)
{
  int value=analogRead(pedal_input);


  double p_value = (5./1024.)*value + 0.04;//the residual addition is just a small correction
  //p_value in volts, must revert it

  double corrected = -p_value + 5.0;

  corrected = map_local(corrected,0.60,4.43,ctl_min,ctl_max);

  if(corrected<ctl_min)
    corrected=ctl_min;

  if(corrected>ctl_max)
    corrected=ctl_max;

  return corrected;
}

void SetServoPWM(double control_value)
{
  static bool att=true;

  if(control_value<ctl_min)
    control_value=ctl_min;

  if(control_value>ctl_max)
    control_value=ctl_max;  

  //Calculate the correct angular value for the servo
  double val = map_local(control_value,ctl_min,ctl_max,98,160);
  //Set the servo position


  double signal_value = map_local(control_value,ctl_min,ctl_max,37,170.7); 

  //set the centralina potenciometer value
  if( abs(control_value-ctl_min)<0.1*(ctl_max-ctl_min) && att==true)
  {
    srv.detach();
    att=false;
    val=98;
    signal_value=37;
  }

  if(control_value>ctl_min)
  {
    if(att==false)
    {
      srv.attach(2);
      att=true;
    }
    srv.write((int)val);
  }

  SetPotValue(centralina_pot,(unsigned int)signal_value);
}

void sendStatus()
{
  pclient->print("mode ");
  pclient->print(mode);
  pclient->print(" throttle ");
  pclient->print(throttle);
  pclient->print(" pedal ");
  pclient->println(pedal);
}

void interpreterMessage(char* msg)
{
  if(strncmp(msg,"set",3)==0)
  {
    sscanf(msg,"%*s %s %s",subject,complement);

    if(strncmp(subject,"throttle",8)==0)
    {
      command_value_auto = atof(complement);
      return;
    }

    if(strncmp(subject,"mode",4)==0)
    {
      mode = atoi(complement);
      return;
    };

  }
}


void setup()
{
  // start serial communication at 9600bps
  Serial.begin(9600);
  //delay(100);

  //Serial.println("Start Setup");

  //Serial.print("Start ethernet ... ");
  Ethernet.begin(mac, ip, gateway, subnet);//configure the ethernet chip
  server.begin();  // start listening for clients
  // Serial.println("Done");

  //Serial.print("Start i2c ... ");
  Wire.begin();// join i2c bus (address optional for master)
  //SetPotValue(centralina_min_value_pot,233);//Set the centralina min value potenciomter, 249 - 0.40v
  SetPotValue(centralina_min_value_pot,255);//Set the centralina min value potenciomter, 249 - 0.40v

  srv.attach(2);//attach the servo class to an output
  SetServoPWM(ctl_min);

  mode = MANUAL;

  //Schedule the send function using the timer class
  t.every(1000/freq, sendStatus);

  Serial.println("Done");
}

void loop()
{
  /*Fichas:
   
   Borboleta
   1 - GND (red)
   2 - 5V (orange)
   3 - Signal (input, black), Pin A1
   4 - GND (brown)
   
   Centralina
   1 - GND (yellow)
   2 - GND (brown)
   3 - output (green), Pot3, for this pot 0 corresponds to the min voltage and 255 to the max
   4 - NC (white)
   
   Pot1 is the centralina min value
   
   Pedal
   1 - GND (brown)
   2 - 5V (orange)
   3 - Signal (input, black), Pin A2
   4 - GND (red)
   
   Servo controler
   min potenciomter - black
   max potenciomter - red
   signal potenciomter - white, Pot2, for this pot 255 corresponds to the min voltage and 0 to the max
   0V power - black
   5V power - red
   pwm signal - white, Pin2
   
   Digital input: auto/manual
   Pin 7 (yellow loose cable)
   
   Pot0 is not used
   */

  // wait for a new client:
  EthernetClient client = server.available();
  pclient = &client;

  if(client)
  {
    //While the connection is active
    while(client.connected())
    {
      //If we get any data, read it and interpreter the message
      if(client.available()>0)
      {
        //Number of bytes received
        int count=0;
        //While there are bytes avaliable and the buffer is not full
        while(client.available()>0 && count<RECEIVE_BUFFER_SIZE)
          receive_buffer[count++]=client.read();

        //Mark the end of the message
        receive_buffer[count]='\0';

        //Interpreter the received message
        interpreterMessage(receive_buffer);
      }

      //Update the timer class
      t.update();

      //Read the borboleta potenciometer signal 
      throttle = ReadBorboletaPotenciometer();

      if (mode==AUTO)
      {
        SetServoPWM(command_value_auto);
      } 
      else if (mode==MANUAL)
      {
        //if manual, read the control value from the pedal potenciometer
        pedal = ReadPedalPotenciometer();
        //Create PWM value using the control value
        SetServoPWM(pedal);

      }
    }
  }

  //Read the borboleta potenciometer signal 
  throttle = ReadBorboletaPotenciometer();

  //if manual, read the control value from the pedal potenciometer
  pedal = ReadPedalPotenciometer();
  //Create PWM value using the control value
  SetServoPWM(pedal);
}


