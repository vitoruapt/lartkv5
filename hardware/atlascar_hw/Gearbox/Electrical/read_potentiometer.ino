void setup()
{
  Serial.begin(9600);
}

void loop(void)
{
  delay(1000);
  Serial.print("yy:");
  Serial.println(analogRead(A0));
  delay(1000);
  Serial.print("xx:");      
  Serial.println(analogRead(A1));
  
}
