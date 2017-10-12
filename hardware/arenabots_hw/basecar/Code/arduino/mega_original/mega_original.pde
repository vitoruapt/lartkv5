void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);

}
 
void loop() {
  // read from port 0, send to port 2:
  if (Serial.available()) {
     int inByte = Serial.read();
     Serial.print("Porta 0-Valor lido: ");
     Serial.println(inByte,BYTE);
     Serial3.println(inByte,BYTE);
     //Serial2.println(inByte,BYTE);
     //Serial2.println('A',BYTE);
   }
 
//   if (Serial2.available()) {
//     int inByte = Serial2.read();
//     Serial.print("Outside-Valor lido: ");
//     Serial.println(inByte,BYTE);
//     Serial2.print(inByte); //send in back
//   }
}
