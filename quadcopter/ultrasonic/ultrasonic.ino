  #include<Servo.h>	

const int trigPin = 11;
const int echoPin = 10;
int i=0;
int _motor_A = 3;
int _motor_B = 5;
int _motor_C = 6;
int _motor_D = 9;
int _ultraSonicA_E = 2;
int _ultraSonicA_T = 4;
int _ultraSonicB_E = 7;
int _ultraSonicB_T = 8;
int _ultraSonicC_E = 12;
int _ultraSonicC_T = 13;
int _ultraSonicD_E = A0;
int _ultraSonicD_T = A1;
int _ultraSonicDown_E = A2; //
int _ultraSonicDown_T = A3; //USING THESE
int _ax, _ay, _az;
int _gx, _gy, _gz;
Servo A;
Servo B;
Servo C;
Servo D;

void setup(){

A.attach(_motor_A);
B.attach(_motor_B);
C.attach(_motor_C);
D.attach(_motor_D);
///////////////////////////////////////////////
//good to go
Serial.begin(9600);

arm(); 
qspeed(23.7);
delay(3000);
turnOff();

while(true){}

/*delay(3500);
turnOff();*/



}





void loop(){
    // establish variables for duration of the ping, 
  // and the distance result in inches and centimeters:
  long duration, inches, cm;
 
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
 
  // convert the time into a distance
 // inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);
  if(cm<5)
  {
    turnOff();
  }
  else
  {
  if(30-cm/15<21)
  {
    qspeed(21);
  }
  else
  {
   qspeed(30-cm/15<21);
  }
  }
  delay(10);
 i++;
 if(i==5000)
 {
   turnOff();
   while(true){}
   
 }
 Serial.println(getADistance());
  delay(10);
  
}//Do not write here!
long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}






///////////////////////////////////////////////////////////////////////////

void arm()
{
  delay(1000);
A.writeMicroseconds(1500);
B.writeMicroseconds(1500);
C.writeMicroseconds(1500);
D.writeMicroseconds(1500);
Serial.println ("Arming.... ");
delay (4000);
Serial.println ("Done!");
}

void turnOff()
{
A.writeMicroseconds(1500);
B.writeMicroseconds(1500);
C.writeMicroseconds(1500);
D.writeMicroseconds(1500);
Serial.println ("Turned off");
}

void writeA(float speed)
{
A.writeMicroseconds(speed*5+1500);
}

void writeB(float speed)
{
B.writeMicroseconds(speed*5+1500);
}

void writeC(float speed)
{
C.writeMicroseconds(speed*5+1500);
}

void writeD(float speed)
{
D.writeMicroseconds(speed*5+1500);
}

void qspeed(float speed)
{
A.writeMicroseconds(speed*5+1500);
B.writeMicroseconds(speed*5+1500);
C.writeMicroseconds(speed*5+1500);
D.writeMicroseconds(speed*5+1500);
}

void twist(int speed)
{
A.writeMicroseconds(speed*5+1500);
C.writeMicroseconds(speed*5+1500);
}

void manual(int a_Speed, int b_Speed, int c_Speed, int d_Speed)
{
A.writeMicroseconds(a_Speed*5+1500);
B.writeMicroseconds(b_Speed*5+1500);
C.writeMicroseconds(c_Speed*5+1500);
D.writeMicroseconds(d_Speed*5+1500);
}

void hover()
{
}

void land()
{

}

void takeOff()
{

}



int getADistance()
{
digitalWrite(4, LOW); 
delayMicroseconds(2); 

digitalWrite(4, HIGH);
delayMicroseconds(10); 

digitalWrite(4, LOW);

return pulseIn(2, HIGH)/58.2;
}

int getBDistance()
{
digitalWrite(8, LOW); 
delayMicroseconds(2); 

digitalWrite(8, HIGH);
delayMicroseconds(10); 

digitalWrite(8, LOW);

return pulseIn(7, HIGH)/58.2;
}

int getCDistance()
{
digitalWrite(13, LOW); 
delayMicroseconds(2); 

digitalWrite(13, HIGH);
delayMicroseconds(10); 

digitalWrite(13, LOW);

return pulseIn(12, HIGH)/58.2;
}

int getDDistance()
{
digitalWrite(A1, LOW); 
delayMicroseconds(2); 

digitalWrite(A1, HIGH);
delayMicroseconds(10); 

digitalWrite(A1, LOW);

return pulseIn(A0, HIGH)/58.2;
}

int getDownDistance()
{
digitalWrite(A3, LOW); 
delayMicroseconds(2); 

digitalWrite(A3, HIGH);
delayMicroseconds(10); 

digitalWrite(A3, LOW);
Serial.println("HELLLO");
return pulseIn(A2, HIGH)/58.2; 
}

void accel(float time, float startSpeed, float endSpeed) //In time use seconds
{
if(endSpeed > startSpeed){

for (float i = 1500+startSpeed*5; i <= 1500+endSpeed*5; i++)
{

A.writeMicroseconds (i);
B.writeMicroseconds (i);
C.writeMicroseconds (i);
D.writeMicroseconds (i);
delay(time*200/(endSpeed-startSpeed));
}



}
else
{
Serial.println("Negtive acceleration");
for (float i = 1500+startSpeed*5; i >= 1500+endSpeed*5; i--)//Start speed is first than we reduce its amount one at a time until it equals the endSpeed
{

A.writeMicroseconds (i);
B.writeMicroseconds (i);
C.writeMicroseconds (i);
D.writeMicroseconds (i);
delay(time*200/(startSpeed-endSpeed));

}
}
}




