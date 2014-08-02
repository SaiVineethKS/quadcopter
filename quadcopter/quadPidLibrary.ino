#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif
#include <VirtualWire.h>
#include<String>

//Define Variables we'll be connecting to
double SetpointX, pidX, SetpointY, pidY;
MPU6050 accelgyro;
double lastTime, timer, temp, tempRaw, deltaTimer;
int16_t ax, ay, az;
int16_t gx, gy, gz;
double accXangle, accYangle, gyroXangle, gyroYangle, kalAngleX, kalAngleY, compAngleX, compAngleY;
//Specify the links and initial tuning parameters
PID xPID(&kalAngleX, &pidX, &SetpointX,0.1,0.025,0.0002, DIRECT);//0.1, 0.025, 0.0002
PID yPID(&kalAngleY, &pidY, &SetpointY,0.2,0.025,0.0002, DIRECT);
double baseSpeed = 0;
byte stringmessage[VW_MAX_MESSAGE_LEN]; // a buffer to store the incoming messages
byte stringmessageLength = VW_MAX_MESSAGE_LEN; // the size of the message

void setup()
{
  //initialize the variables we're linked to
  xPID.SetSampleTime(1);///in millis
  yPID.SetSampleTime(1);///in millis
  xPID.SetOutputLimits(-10, 10);
  yPID.SetOutputLimits(-10, 10);
  SetpointX = 0;
  SetpointY = 0;
  //turn the PID on
  xPID.SetMode(AUTOMATIC);
  yPID.SetMode(AUTOMATIC);
  lastTime = micros();  
  timer = micros();
  initStuff();
  updateAngles();
  arm();
}

void loop()
{
  updateAngles();
  updateRadio();
  xPID.Compute();
  yPID.Compute();
  Serial.print("    ");
  Serial.println(pidX);
  writeB(baseSpeed+pidY);
  writeD(baseSpeed-pidY);
  
}

void initStuff(){
   // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(100000, true);
    #endif
    Serial.begin(115200);

    // initialize device
    Serial.println("Initializing MPU6050...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    vw_setup(2000); // Bits per sec
    vw_rx_start(); // Start the receiver
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

void updateAngles(){
  deltaTimer = micros() - timer;
  Serial.print(1000000/deltaTimer);
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  accXangle = (atan2(ax, az) + PI) * RAD_TO_DEG;
  accYangle = (atan2(ax, az) + PI) * RAD_TO_DEG;
  double gyroXrate = (double)gx / 131.0;
  double gyroYrate = -((double)gy / 131.0);
  //gyroXangle += gyroXrate * deltaTimer / 1000000; // Calculate gyro angle without any filter
  //gyroYangle += gyroYrate * deltaTimer / 1000000;
  compAngleX = (0.93 * (compAngleX + (gyroXrate * deltaTimer / 1000000))) + (0.07 * accXangle); // Calculate the angle using a Complimentary filter
  compAngleY = (0.93 * (compAngleY + (gyroYrate * deltaTimer / 1000000))) + (0.07 * accYangle);
  kalAngleX = compAngleX - 180;//maybe need to change offset becuse of the orentation of the imu on the quad.
  kalAngleY = compAngleY -180;
  timer = micros();
}

void updateRadio(){
  String str="";
  char stringm;
  int num;
  if (vw_get_message(stringmessage, &stringmessageLength)) // Non-blocking
    {//woohoo recieved date
    for (int i = 0; i < stringmessageLength; i++)
    {
      str += (char)stringmessage[i];
    }
    baseSpeed = str.toInt();
  }
}
