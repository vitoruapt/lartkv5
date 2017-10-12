#include <SPI.h>
#include <Ethernet.h>
#include <uno_Counter.h>
#include <TimerOne.h>

uno_Counter uno_Counter(4); // Initializes the Uno Encoder Counter in 4X Count Mode

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
//byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
byte mac[] = {0x00, 0x23, 0x54, 0x5c, 0x2c, 0x16 };
byte ip[] = { 10, 0, 0, 31 }; // ip in lan

// Initialize the Ethernet server library
// with the IP address and port you want to use 
// (port 80 is default for HTTP):
Server server(120);

//variables used
unsigned long wait;
int timer_count;

String  msg;

double ct, lt, vps, pps, dt , dp, Speed , pi=3.14159265, D;

long cp,lp;

String floatToString(double number, uint8_t digits) //função que convert em Strings para que se possa enviar por porta serie os dados
{ 
  String resultString = "";
  // Handle negative numbers
  if (number < 0.0)
  {
     resultString += "-";
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  resultString += int_part;

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    resultString += "."; 

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    resultString += toPrint;
    remainder -= toPrint; 
  } 
  return resultString;
}

void timerIsr() //função que é chamada pelo interrupt
{
      // Toggle LED
    // analogWrite(A2,155);
    
     ct=millis();
     cp=uno_Counter.XAxisGetCount();
     timer_count=1;
     // Serial.println(ct-lt);
      //lt=ct;
}



void setup()
{  
  //pinMode(A2, OUTPUT);  
  
  timer_count=0;
  //Definiço de um timer interno
  wait=100000; //frequency of the actualization
  Timer1.initialize(wait); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
  Timer1.attachInterrupt( timerIsr ); // attach the service routine here
  
  
  // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);
  //server.begin();
  
  Serial.begin(9600);
   
  
  // Diameter of the wheel
  D=0.28*2.;
 
}

void loop()
{
   //analogWrite(A2,0);
   
  // listen for incoming clients 
  Client client = server.available();
  
  // if (client) {    boolean currentLineIsBlank = true;
     if (client.connected()) {
      if (client.available()) {


     //   ct=millis();
       // cp=uno_Counter.XAxisGetCount();
        
                    if(timer_count==1)
                    {
        
                           if( abs(cp-lp) < 100000 )
                           {
                            //Serial.println("correu a situaço do ciclo if");  
                            dt=(double)(ct-lt)/1000.; //diviso por 100 para que o valor esteja em segundos
                            dp=(double)(cp-lp)/4.; //devido ao modo de contagem do HCTL2022.
                            
                            pps=dp/dt; //pulsos por segundo
                            
                            vps=pps/50.; //rotaçao por segundo
                            
                            
                            Speed=vps*pi*D;
                            
                            
                           }
                    
                  
            
                    
                        lt=ct;
                        lp=cp;
                        
                        //delay(wait);
                      
                        msg="AC ";
                        msg+=floatToString(cp,0);
                        msg+=" P ";
                        msg+=floatToString(pps,2);
                        msg+=" R ";
                        msg+=floatToString(vps,2);
                        msg+=" S ";
                        msg+=floatToString(Speed,4);
                        msg+=" B";
                        
                          // msg= 'A' + 'C'+ cp +'P'+ pps + 'R'+ vps + 'S'+ Speed+ 'B' ; 
                           msg[0]=0x02; //inicio de mensagem
                           msg[msg.length()-1]=0x03; //fim de mensagem
                           
                    timer_count=0;
                  }
                  
              //Serial.println(msg);
              client.println(msg);

        }

   }
  
}

