#define Pu 2.7e6
#define kp 1/15.5//1/9//1/5//37.5//0.044 18
#define ki 0//1/100//kp*1/Pu//0.000000001//1/900000
#define kd 0//d1/80//1/20//1/20//1/80//-1/20//1/10//3000
//p - 1/13.5


#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
  
  #include <VirtualWire.h>
#include<String>


#endif
MPU6050 accelgyro;
double lastTime, timer, temp, tempRaw, deltaTimer;
int16_t ax, ay, az;
int16_t gx, gy, gz;
double accXangle, accYangle, gyroXangle, gyroYangle, kalAngleX, kalAngleY, compAngleX, compAngleY;
int minSpeed = 24;
//Communication
byte stringmessage[VW_MAX_MESSAGE_LEN]; // a buffer to store the incoming messages
byte stringmessageLength = VW_MAX_MESSAGE_LEN; // the size of the message
//End of com

void setup() {
  //Serial.begin(9600);
//Serial.println("Device is ready");
vw_setup(2000); // Bits per sec
vw_rx_start(); // Start the receiver

   lastTime = micros();  
   timer = micros();
   initStuff();
   updateAngles();
   arm();
   pid(10, 0, 0);//Time/Angles
  turnOff();
}

void loop() {/*
    updateAngles();
    Serial.println(kalAngleX);
    //Serial.println((micros()-lastTime));
    lastTime = micros();
    */
    
}
boolean com()//Recieve data!
{
  //Serial.println("asfd");
  String str="";
  char stringm;
  int num;
if (vw_get_message(stringmessage, &stringmessageLength)) // Non-blocking
{
  
Serial.print("Received: ");
for (int i = 0; i < stringmessageLength; i++)
{
stringm= (char)stringmessage[i];
Serial.print(stringm);
str += stringm;

}
num=str.toInt();
minSpeed = num;
Serial.println(num);
if(num==0)
{
  return false;
}
str="";
return true;
}
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
accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
accXangle = (atan2(ax, az) + PI) * RAD_TO_DEG;
accYangle = (atan2(ax, az) + PI) * RAD_TO_DEG;
double gyroXrate = (double)gx / 131.0;
double gyroYrate = -((double)gy / 131.0);
gyroXangle += gyroXrate * deltaTimer / 1000000; // Calculate gyro angle without any filter
gyroYangle += gyroYrate * deltaTimer / 1000000;
compAngleX = (0.93 * (compAngleX + (gyroXrate * deltaTimer / 1000000))) + (0.07 * accXangle); // Calculate the angle using a Complimentary filter
compAngleY = (0.93 * (compAngleY + (gyroYrate * deltaTimer / 1000000))) + (0.07 * accYangle);
kalAngleX = compAngleX - 170;
kalAngleY = compAngleY -181;
timer = micros();
}



void pid(double time, double xAngle,double yAngle){
  
  int start = millis();
  double sumIX = 0;
  double sumIY = 0;
  updateAngles();
  int lastX = xAngle-kalAngleX;
  int lastY = yAngle-kalAngleY;
  double errorX,errorY,pidX,pidY,a,b,c,d, deltaMicros;
 while((millis()-start) < (time * 1000)){
   if(accelgyro.testConnection()){
updateAngles();
/*if(!com())
{
  turnOff();
  return;
}*/
//Get DATA
 errorX = xAngle-kalAngleX;
 errorY = yAngle-kalAngleY;
sumIX += errorX * ki;
sumIY += errorY * ki;
deltaMicros = micros()-lastTime;
 pidX = errorX  * kp + sumIX*deltaMicros - ((errorX - lastX)/deltaMicros) *kd;
 pidY = errorY*  kp + sumIY*deltaMicros - ((errorY - lastY)/deltaMicros) *kd;
 //if(errorY > 13 || errorY < -13){
     //pidY += errorX * 1; 
 //}
 a = minSpeed - pidY;
 b = minSpeed + pidX;
 c = minSpeed + pidY;
 d = minSpeed - pidX;
    
    if (a < minSpeed){
        a = minSpeed;
        
    }else{
     c = minSpeed; //asd,jkhasdkhszckjsahkausjdlhj
    }

//if ((kalAngleY > 100 || kalAngleY < -100 || kalAngleX > 100 || kalAngleX < -100) /*|| accelgyro.testConnection()*//*adds 500 micros*/){
  //turnOff();
  //return;
//}
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
//Serial.println(kalAngleY);
//writeD(d);
//Serial.println(a);
  
Serial.println(a);
//Serial.println(deltaMicros);
lastX = errorX;
lastY = errorY;
lastTime = micros();

/*Serial.print("minSpeed: ");
Serial.println(minSpeed);*/
   }
   else{
    turnOff();
   return; 
   }
}
}

