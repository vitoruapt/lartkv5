void setup() {
  Serial.begin(9600);
  Serial2.begin(19200);
}
 
void loop() {
  // read from port 0, send to port 2:
  if (Serial.available()) {
     int inByte = Serial.read();
     
//     Serial.println('A', BYTE);   // send a capital A
//     Serial.println("0,0,0");
//
//     Serial.print("from usb I received: ");
//     Serial.println(inByte, DEC);
     Serial.println(inByte, BYTE);
//     Serial.println(inByte);
//     Serial.println("finished");
     
     //Serial2.print(inByte);
     //delay(300);
   }
 
//   if (Serial2.available()) {
//     int inByte = Serial2.read();
//     Serial.print("Outside-Valor lido: ");
//     Serial.println(inByte,BYTE);
//     Serial2.print(inByte); //send in back
//   }
}
