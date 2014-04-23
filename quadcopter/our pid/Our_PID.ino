#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#define kp 1/300 //Bigger less kp original:4 this is p!!! 1/20 1/40
#define ki 0  //Bigger less kp original:4 this is p!!! 1/20 1/40
#define kd 0 //Bigger less kp original:4 this is p!!! 1/20 1/40

//////////////////////////////////////////////////////////////////
//altitude pid
#define Hkp 1/9  
#define Hki 1/9000 * sampleTime //Bigger less kp original:4 this is p!!! 1/20 1/40
#define Hkd 1/5 / sampleTime //Bigger less kp original:4 this is p!!! 1/20 1/40
//////////////////////////////////////////////////////////////////

int res=20;
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
 Serial.println("appears");
 average();
 arm();
timer = micros();
///////////////////////////////////////////////
//good to go
lastSpeed = 24;
pid(999999999, 24, 0, 0);

turnOff();
//altitude(15, 10000000);
}

void loop() {
 //Serial.println(getHight());
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
kalAngleX += 3;
kalAngleY -= 1;

kalAngleX -= 180;
kalAngleY -= 180;
}







void arm()
{
Serial.println ("Arming.... ");
  for(int i = 0; i < 32000; i++){
   qspeed(0);
  delayMicroseconds(125);
  }
  Serial.println ("Done!");
}



void turnOff()
{
 qspeed(0);

Serial.println ("Turned off");
}

void qspeed(float MotorSpeed)
{
  writeA(MotorSpeed);
  writeB(MotorSpeed);
  writeC(MotorSpeed);
  writeD(MotorSpeed);
}


void writeA(float MotorSpeed){
 if (MotorSpeed == 0){
     analogWrite(5, 1400);
 }else if(MotorSpeed <= 100){
     analogWrite(5, (1669+((MotorSpeed/100)*120)));
 }else{
     analogWrite(5, 1790);
 }
}

void writeB(float MotorSpeed){
 if (MotorSpeed == 0){
     analogWrite(3, 1400);
 }else if(MotorSpeed <= 100){
     analogWrite(3, (1669+((MotorSpeed/100)*120)));
 }else{
     analogWrite(3, 1790);
 }
}

void writeC(float MotorSpeed){
 if (MotorSpeed == 0){
     analogWrite(9, 1400);
 }else if(MotorSpeed <= 100){
     analogWrite(9, (1669+((MotorSpeed/100)*120)));
 }else{
     analogWrite(9, 1790);
 }
}

void writeD(float MotorSpeed){
 if (MotorSpeed == 0){
     analogWrite(6, 1400);
 }else if(MotorSpeed <= 100){
     analogWrite(6, (1669+((MotorSpeed/100)*120)));
 }else{
     analogWrite(6, 1790);
 }
}

void initStuff(){
 
Serial.begin(115200);
Wire.begin();
TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
Serial.println("1");
while (i2cWrite(0x19, i2cData, 4, false)){
  Serial.println("init 1 faild");
  delay(10);
} // Write to all four registers at once
Serial.println("2");
while (i2cWrite(0x6B, 0x01, true)){
    Serial.println("init 2 faild");
  delay(10);
} // PLL with X axis gyroscope reference and disable sleep mode
Serial.println("3");
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
Serial.println("Inintalized succsefully");
}




void pid(double time, double minSpeed, double xAngle,double yAngle){
  lastSpeed = minSpeed;
  int start = millis();
  double sumIX = 0;
  double sumIY = 0;
  average();
  int lastX = xAngle-kalAngleX;
  int lastY = yAngle-kalAngleY;
  double errorX,errorY,pidX,pidY,a,b,c,d;
 while((millis()-start) < (time * 1000)){
average();


 errorX = xAngle-kalAngleX;
 errorY = yAngle-kalAngleY;
sumIX += errorX * ki;
sumIY += errorY * ki;
 pidX = errorX * kp + sumIX*sampleTime - (errorX - lastX)/sampleTime *kd;
 pidY = errorY * kp + sumIY*sampleTime - (errorY - lastY)/sampleTime *kd;
 a = minSpeed - pidY;
 b = minSpeed + pidX;
 c = minSpeed + pidY;
 d = minSpeed - pidX;


if (kalAngleY > 100 || kalAngleY < -100 || kalAngleX > 100 || kalAngleX < -100){
  //turnOff();
  return;
}
//Serial.print("A:");
//Serial.print(a);Serial.print("\t");
writeA(a);
//Serial.print("B:");
//Serial.print(b);Serial.print("\t");
//writeB(b);
//Serial.print("C:");
//Serial.print(c);Serial.print("\t");
writeC(c);
//Serial.print("D:");
//Serial.print(d);Serial.println("\t");
//Serial.print(kalAngleX);Serial.println("\t");
Serial.println(kalAngleY);

//writeD(d);
//Serial.print(temp);Serial.print("\t");

  
//minSpeed-=(1.0/time*(minSpeed-20)/100.0);//Decreases the motors speed in relation to time and minSpeed



lastX = errorX;
lastY = errorY;
/*Serial.print("minSpeed: ");
Serial.println(minSpeed);*/

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
