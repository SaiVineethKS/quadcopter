#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter


#define mistakeRange 0 //orig:3 this is d!!!
#define change 20 //Bigger less change original:4 this is p!!! 20
#include<Servo.h>	

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
double minSpeed = 24.2000; // Max Battery Life- to hover about 23.5
double time = 2.0;
Servo A;
Servo B;
Servo C;
Servo D;


int i = 0;

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
int16_t accX, accY, accZ;
int16_t tempRaw;
int16_t gyroX, gyroY, gyroZ;

double accXangle, accYangle; // Angle calculate using the accelerometer
double temp; // Temperature
double gyroXangle, gyroYangle; // Angle calculate using the gyro
double compAngleX, compAngleY; // Calculate the angle using a complementary filter
double kalAngleX, kalAngleY; // Calculate the angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

void setup() {
  
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
///////////////////////////////////////////////
//good to go
arm();
timer = micros();

}

void loop() {
 
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
kalAngleX = 180 - kalAngleX;
kalAngleY = 180 - kalAngleY;

Serial.print(kalAngleX); Serial.print("\t");

Serial.print("\t");

Serial.print(kalAngleY); Serial.print("\t");
double a = minSpeed+(kalAngleY/change);//What the motor speeds will be with angles applyied
double b = minSpeed+(kalAngleX/change);
double c = minSpeed+(kalAngleY/change);
double d = minSpeed+(kalAngleX/change);
if (a > minSpeed+mistakeRange){

c = minSpeed+abs(kalAngleY/change);


a = minSpeed;
}else if(a > minSpeed-mistakeRange && a < minSpeed+mistakeRange){
  a = minSpeed;
  
  c = minSpeed;
  
}else {
a = minSpeed+abs(kalAngleY/change);

c = minSpeed;

}

if (d > minSpeed+mistakeRange){
b = minSpeed+abs(kalAngleX/change);

d = minSpeed;

}else if(d > minSpeed-mistakeRange && d < minSpeed+mistakeRange){
b = minSpeed;

d = minSpeed;

}else{
d = minSpeed+abs(kalAngleX/change);

b = minSpeed;

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

i++;



/*Serial.print("minSpeed: ");
Serial.println(minSpeed);*/
Serial.println(i);
delay(10);

if(i >= time*100){
  turnOff();
 while(true){
   //Serial.println("end");
 } 
}
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
