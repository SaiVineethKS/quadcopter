#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#define sampleTime 10//millis!!!
#define kp 1/16.55 //Bigger less kp original:4 this is p!!! 1/20 1/40
#define ki 10 * sampleTime //Bigger less kp original:4 this is p!!! 1/20 1/40
#define kd 0 / sampleTime //Bigger less kp original:4 this is p!!! 1/20 1/40

//////////////////////////////////////////////////////////////////
//altitude pid
#define Hkp 1/12
#define Hki 1/9000 * sampleTime //Bigger less kp original:4 this is p!!! 1/20 1/40
#define Hkd 1/3 / sampleTime //Bigger less kp original:4 this is p!!! 1/20 1/40
//////////////////////////////////////////////////////////////////
#include<Servo.h>


int res=20;
int _motor_A = 5;
int _motor_B = 3;
int _motor_C = 9;
int _motor_D = 6;
int _ultraSonicA_E = 2;
int _ultraSonicA_T = 4;
int _ultraSonicB_E = 7;
int _ultraSonicB_T = 8;
int _ultraSonicC_E = 12;
int _ultraSonicC_T = 13;
int _ultraSonicD_E = A0;
int _ultraSonicD_T = A1;
int _ultraSonicDown_E = A2;
int _ultraSonicDown_T = A3;
Servo A;
Servo B;
Servo C;
Servo D;


Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
int16_t accX, accY, accZ;
int16_t tempRaw;
int16_t gyroX, gyroY, gyroZ;
double lastSpeed = 0;
double accXangle, accYangle; // Angle calculate using the accelerometer
double temp; // Temperature
double gyroXangle, gyroYangle; // Angle calculate using the gyro
double compAngleX, compAngleY; // Calculate the angle using a complementary filter
double kalAngleX, kalAngleY; // Calculate the angle using a Kalman filter
double errorX, errorY;
uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

void setup() {
 initStuff();
 average();
 errorX = 180;
 errorY = 180;
 //arm();
timer = micros();
///////////////////////////////////////////////
//good to go
lastSpeed = 23.850;
//pid(20, 23.8500, 0, 0);
//turnOff();
altitude(15, 10000000);
}

void loop() {
 Serial.println(getHight());
}

void average()
{
  
  int sumX = 0;
  int sumY = 0;
  for(int c=0;c<res;c++)
  {
    updateAngles();
    sumX+= kalAngleX;
    sumY+= kalAngleY;
  }
  kalAngleX = sumX / res;
  kalAngleY = sumY / res;
  //Serial.print("KalX: ");
  //Serial.println(kalAngleX);
  //Serial.print("KalY: ");
  //Serial.println(kalAngleY);
}
void updateAngles()
{
  /* Update all the values */
while (i2cRead(0x3B, i2cData, 14));
accX = ((i2cData[0] << 8) | i2cData[1]);
accY = ((i2cData[2] << 8) | i2cData[3]);
accZ = ((i2cData[4] << 8) | i2cData[5]);
tempRaw = ((i2cData[6] << 8) | i2cData[7]);
gyroX = ((i2cData[8] << 8) | i2cData[9]);
gyroY = ((i2cData[10] << 8) | i2cData[11]);
gyroZ = ((i2cData[12] << 8) | i2cData[13]);

// atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
// We then convert it to 0 to 2π and then from radians to degrees
accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;

double gyroXrate = (double)gyroX / 131.0;
double gyroYrate = -((double)gyroY / 131.0);
gyroXangle += gyroXrate * ((double)(micros() - timer) / 1000000); // Calculate gyro angle without any filter
gyroYangle += gyroYrate * ((double)(micros() - timer) / 1000000);
//gyroXangle += kalmanX.getRate()*((double)(micros()-timer)/1000000); // Calculate gyro angle using the unbiased rate
//gyroYangle += kalmanY.getRate()*((double)(micros()-timer)/1000000);

compAngleX = (0.93 * (compAngleX + (gyroXrate * (double)(micros() - timer) / 1000000))) + (0.07 * accXangle); // Calculate the angle using a Complimentary filter
compAngleY = (0.93 * (compAngleY + (gyroYrate * (double)(micros() - timer) / 1000000))) + (0.07 * accYangle);

kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros() - timer) / 1000000); // Calculate the angle using a Kalman filter
kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros() - timer) / 1000000);
timer = micros();

temp = ((double)tempRaw + 12412.0) / 340.0;
kalAngleX = kalAngleX - errorX;
kalAngleY = kalAngleY - errorY;
}







void arm()
{
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
 D.writeMicroseconds(1500);
B.writeMicroseconds(1500);
A.writeMicroseconds(1500);
C.writeMicroseconds(1500);

Serial.println ("Turned off");
}

void qspeed(float speed)
{
  lastSpeed = speed;
A.writeMicroseconds(speed*5+1500);
B.writeMicroseconds(speed*5+1500);
C.writeMicroseconds(speed*5+1500);
D.writeMicroseconds(speed*5+1500);
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

void initStuff(){
 
Serial.begin(115200);
Wire.begin();
TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

while (i2cRead(0x75, i2cData, 1));
if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
Serial.print(F("Error reading sensor"));
while (1);
}

delay(100); // Wait for sensor to stabilize

/* Set kalman and gyro starting angle */
while (i2cRead(0x3B, i2cData, 6));
accX = ((i2cData[0] << 8) | i2cData[1]);
accY = ((i2cData[2] << 8) | i2cData[3]);
accZ = ((i2cData[4] << 8) | i2cData[5]);
// atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
// We then convert it to 0 to 2π and then from radians to degrees
accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;
accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;

kalmanX.setAngle(accXangle); // Set starting angle
kalmanY.setAngle(accYangle);
gyroXangle = accXangle;
gyroYangle = accYangle;
compAngleX = accXangle;
compAngleY = accYangle;
A.attach(_motor_A);
B.attach(_motor_B);
C.attach(_motor_C);
D.attach(_motor_D);
}




void pid(double time, double minSpeed, double xAngle,double yAngle){
  lastSpeed = minSpeed;
  int start = millis();
  double sumIX = 0;
  double sumIY = 0;
  average();
  int lastX = kalAngleX;
  int lastY = kalAngleY;
 while((millis()-start) < (time * 1000)){
average();


double errorX = xAngle-kalAngleX;
double errorY = yAngle-kalAngleY;
sumIX += errorX * ki;
sumIY += errorY * ki;
double pidX = errorX * kp + sumIX  - (kalAngleX-lastX) * kd;
double pidY = errorY * kp + sumIY - (kalAngleY-lastY) * kd;
double a = minSpeed + pidY;
double b = minSpeed - pidX;
double c = minSpeed - pidY;
double d = minSpeed + pidX;
if(a<20){
  a = 20;
}
if(b<20){
  b = 20;
}
if(c<20){
  c = 20;
}
if(d<20){
  d = 20;
}

Serial.print("A:");
Serial.print(a);Serial.print("\t");
writeA(a);
Serial.print("B:");
Serial.print(b);Serial.print("\t");
writeB(b);
Serial.print("C:");
Serial.print(c);Serial.print("\t");
writeC(c);
Serial.print("D:");
Serial.print(d);Serial.println("\t");
writeD(d);
//Serial.print(temp);Serial.print("\t");

  
//minSpeed-=(1.0/time*(minSpeed-20)/100.0);//Decreases the motors speed in relation to time and minSpeed



lastX = errorX;
lastY = errorY;
/*Serial.print("minSpeed: ");
Serial.println(minSpeed);*/
delay(sampleTime);

}
}



long getHight(){
  long duration, inches, cm;
 
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(11, OUTPUT);
  digitalWrite(11, LOW);
  delayMicroseconds(2);
  digitalWrite(11, HIGH);
  delayMicroseconds(10);
  digitalWrite(11, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(10, INPUT);
  duration = pulseIn(10, HIGH);
 
  // convert the time into a distance
 // inches = microsecondsToInches(duration);
  cm = duration / 29 / 2;
  delay(10);
  average();
  long hight;
  if (abs(kalAngleX) > 25 || abs(kalAngleY) > 25){
    return 0;
  }else{
 return cos(abs(kalAngleX)*DEG_TO_RAD) * (cm * cos(abs(kalAngleY)*DEG_TO_RAD)); 
  }
}

long getDistance(){
  long duration, inches, cm;
 
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(11, OUTPUT);
  digitalWrite(11, LOW);
  delayMicroseconds(2);
  digitalWrite(11, HIGH);
  delayMicroseconds(10);
  digitalWrite(11, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(10, INPUT);
  duration = pulseIn(10, HIGH);
 
  // convert the time into a distance
 // inches = microsecondsToInches(duration);
  cm = duration / 29 / 2;
  delay(10);
  average();
  return cm;
}


void altitude(int wantedHight, double time){
     
  double pidSpeed = lastSpeed;
  double start = millis();
  double error = 0;
  double sumIHight = 0;
  int hight = getHight();
  if (hight == 0){
    hight = getDistance();
  }
  double lastHight = hight;
  while((millis()-start) < (time * 1000)){
     
    hight = getHight();
    if (hight == 0){
      hight = getDistance();
    }
    
    error = wantedHight - hight;
    sumIHight += error * Hkp;
    double pidValue = error * Hkp + sumIHight  - (hight - lastHight) * Hkd;
    pid(0.1, pidSpeed + pidValue, 0, 0);
    lastHight = hight;
  }
}
