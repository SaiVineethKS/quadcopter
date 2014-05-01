#define kp 1/25
#define ki 0
#define kd 0



#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif
MPU6050 accelgyro;
double lastTime, timer, temp, tempRaw;
int16_t ax, ay, az;
int16_t gx, gy, gz;
double accXangle, accYangle, gyroXangle, gyroYangle, kalAngleX, kalAngleY, compAngleX, compAngleY;

void setup() {
   initStuff();
   updateAngles();
   arm();
   lastTime = micros();  
   timer = micros();
   pid(999999999, 24, 0, 0);
}

void loop() {/*
    updateAngles();
    Serial.println(kalAngleX);
    //Serial.println((micros()-lastTime));
    lastTime = micros();
    */
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
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
accXangle = (atan2(ax, az) + PI) * RAD_TO_DEG;
accYangle = (atan2(ax, az) + PI) * RAD_TO_DEG;

double gyroXrate = (double)gx / 131.0;
double gyroYrate = -((double)gy / 131.0);
gyroXangle += gyroXrate * ((double)(micros() - timer) / 1000000); // Calculate gyro angle without any filter
gyroYangle += gyroYrate * ((double)(micros() - timer) / 1000000);
//gyroXangle += kalmanX.getRate()*((double)(micros()-timer)/1000000); // Calculate gyro angle using the unbiased rate
//gyroYangle += kalmanY.getRate()*((double)(micros()-timer)/1000000);

compAngleX = (0.93 * (compAngleX + (gyroXrate * (double)(micros() - timer) / 1000000))) + (0.07 * accXangle); // Calculate the angle using a Complimentary filter
compAngleY = (0.93 * (compAngleY + (gyroYrate * (double)(micros() - timer) / 1000000))) + (0.07 * accYangle);


timer = micros();

temp = ((double)tempRaw + 12412.0) / 340.0;
kalAngleX = compAngleX;
kalAngleY = compAngleY;
kalAngleX += 3;
kalAngleY -= 1;
kalAngleX -= 180;
kalAngleY -= 180; 
}



void pid(double time, double minSpeed, double xAngle,double yAngle){
  int start = millis();
  double sumIX = 0;
  double sumIY = 0;
  updateAngles();
  int lastX = xAngle-kalAngleX;
  int lastY = yAngle-kalAngleY;
  double errorX,errorY,pidX,pidY,a,b,c,d;
 while((millis()-start) < (time * 1000)){
updateAngles();


 errorX = xAngle-kalAngleX;
 errorY = yAngle-kalAngleY;
sumIX += errorX * ki;
sumIY += errorY * ki;
 pidX = errorX * kp + sumIX*((micros()-lastTime) / 1000) - (errorX - lastX)/((micros()-lastTime) / 1000) *kd;
 pidY = errorY * kp + sumIY*((micros()-lastTime) / 1000) - (errorY - lastY)/((micros()-lastTime) / 1000) *kd;
 a = minSpeed - pidY;
 b = minSpeed + pidX;
 c = minSpeed + pidY;
 d = minSpeed - pidX;


if (kalAngleY > 100 || kalAngleY < -100 || kalAngleX > 100 || kalAngleX < -100){
  //turnOff();
  //return;
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
//Serial.println(kalAngleY);

//writeD(d);
//Serial.print(temp);Serial.print("\t");

  
//minSpeed-=(1.0/time*(minSpeed-20)/100.0);//Decreases the motors speed in relation to time and minSpeed

Serial.println(micros()-lastTime);

lastX = errorX;
lastY = errorY;
lastTime = micros();
/*Serial.print("minSpeed: ");
Serial.println(minSpeed);*/

}
}


