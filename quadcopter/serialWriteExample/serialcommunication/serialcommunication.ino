#define LED 13

void setup()
{
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
}

void loop()
{
  if (Serial.available() > 0)
 { 
   digitalWrite(LED,HIGH);
   Serial.println(Serial.read()-'0');
 }
 //digitalWrite(LED,LOW);
}
