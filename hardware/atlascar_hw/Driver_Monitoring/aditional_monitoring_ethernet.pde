#include <SPI.h>
#include <Ethernet.h>

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
//byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
byte mac[] = {0x00, 0x23, 0x54, 0x5c, 0x2c, 0x16 };
byte ip[] = { 10, 0, 0, 30 }; // ip in lan
String  s0,s1,s2, s3, s5, s6, s7, s8, s9, msg;
// Initialize the Ethernet server library
// with the IP address and port you want to use 
// (port 80 is default for HTTP):
Server server(120);
int fsrPin0 = 0, fsrPin1 = 1, fsrPin2 = 2  ;     // the FSR and 10K pulldown are connected to a0
String   fsrReading0,  fsrReading1, fsrReading2;     // the analog reading from the FSR resistor divider
 
          
void setup()
{
  
  Ethernet.begin(mac, ip);
  server.begin();

  
  pinMode(2,INPUT);
  pinMode(3,INPUT);  
  pinMode(5,INPUT);
  pinMode(6,INPUT);
  pinMode(7,INPUT);
  pinMode(8,INPUT);
  pinMode(9,INPUT);
  

}

void loop()
{
 
  Client client = server.available();
  
  // if (client) {
       if (client.connected()) {
      	if (client.available()) {
       
        fsrReading0 = analogRead(fsrPin0);  
         fsrReading1 = analogRead(fsrPin1); 
          fsrReading2 = analogRead(fsrPin2); 
        
        
          s2 = digitalRead(2);
          s3 = digitalRead(3);
          s5 = digitalRead(5);
           
          s6 = digitalRead(6);
          s7 = digitalRead(7);
          s8 = digitalRead(8);
           
          s9 = digitalRead(9);


        
           msg= 'A' +  s2  + s3 + s5 + s6 + s7 + s8 + s9 + 'T'+ fsrReading2 +'C'+fsrReading0 + 'B' + fsrReading1 +'B' ; 
           msg[0]=0x02;
           msg[msg.length()-1]=0x03;
           
                 client.println(msg);
         
        }

   }
   delay(10);
  
}

