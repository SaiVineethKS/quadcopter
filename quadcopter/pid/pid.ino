#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include <PID_v1.h>
#include<Servo.h>	
#define time 0.5
#define checks 20.0
double KP = 0.065, KI = 0, KD = 0;//Bigger - More extreme changes P=0.035,I=0.2,D=0.001
double Setpoint = 0.0, Input, Output;
double Setpoint1 = 0.0, Input1, Output1;
PID myPID(&Input, &Output, &Setpoint,KP,KI,KD, DIRECT);
PID myPID1(&Input1, &Output1, &Setpoint1,KP,KI,KD, DIRECT);
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

int i=0;


Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
int16_t accX, accY, accZ;
int16_t tempRaw;
int16_t gyroX, gyroY, gyroZ;
double minSpeed = 21;
double minusX, minusY;
double accXangle, accYangle; // Angle calculate using the accelerometer
double temp; // Temperature
double gyroXangle, gyroYangle; // Angle calculate using the gyro
double compAngleX, compAngleY; // Calculate the angle using a complementary filter
double kalAngleX, kalAngleY; // Calculate the angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

void setup() {
  Serial.begin(115200);
  myPID.SetMode(AUTOMATIC);
  myPID1.SetMode(AUTOMATIC);
  initSensors();
A.attach(_motor_A);
B.attach(_motor_B);
C.attach(_motor_C);
D.attach(_motor_D);
timer = micros();
updateAngles();
minusX = 0;//abs(kalAngleX-180);
minusY = 0;//abs(kalAngleY-180);
myPID.SetSampleTime(10);
myPID.SetOutputLimits(minSpeed, minSpeed + 5);
myPID1.SetSampleTime(10);
myPID1.SetOutputLimits(minSpeed, minSpeed + 5);
///////////////////////////////////////////////
//good to go
arm();


}

void loop() { //Red-A & C
  /* Update all the values */
  average();
  Input = kalAngleX;
  Serial.print("KalX: ");
  Serial.println(Input);
  if (Input < 0){
  myPID.Compute();
  
//  writeB(Output);
  Serial.print("B: ");
  Serial.print(Output);
  Serial.print("    ");
  Serial.print("D: ");
  Serial.println(25-abs(Output-25));
//  writeD(25-abs(Output-25));
  }else{
    Input = Input - 2 * Input;
    myPID.Compute();
    Serial.print("B: ");
  Serial.print(minSpeed-abs(Output-minSpeed));
//  writeB(minSpeed-abs(Output-minSpeed));
  Serial.print("    ");
  Serial.print("D: ");
  Serial.print(Output);
  Serial.print("    ");
//  writeD(Output);
  }
  Input1 = kalAngleY;
  Serial.print("KalY: ");
  Serial.println(Input1);
  if (Input1 < 0){
  myPID1.Compute();
  Serial.print("C: ");
  Serial.print(Output1);
  writeC(Output1);
  Serial.print("    ");
  Serial.print("A: ");
  Serial.println(minSpeed-abs(Output1-minSpeed));
  writeA(minSpeed-abs(Output1-minSpeed));
  }else{
    Input1 = Input1 - 2 * Input1;
    myPID1.Compute();
    Serial.print("C: ");
  Serial.print(minSpeed-abs(Output1-minSpeed));
  writeC(minSpeed-abs(Output1-minSpeed));
  Serial.print("    ");
  Serial.print("A: ");
  Serial.println(Output1);
  writeA(Output1);
  }
  i++;
  if(i>100*time)
  {
    turnOff();
    while(true){}
  }
  delay(10);
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
A.writeMicroseconds(1500);
B.writeMicroseconds(1500);
C.writeMicroseconds(1500);
D.writeMicroseconds(1500);
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


void initSensors(){
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
  turnOff();
  
    Serial.print(F("Error reading sensor"));
    while(true){}
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
}
void average()
{
  
  int sumX = 0;
  int sumY = 0;
  for(int c=0;c<checks;c++)
  {
    updateAngles();
    sumX+= kalAngleX;
    sumY+= kalAngleY;
  }
  kalAngleX = sumX / checks;
  kalAngleY = sumY / checks;

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
kalAngleX = kalAngleX - 180;
kalAngleY = kalAngleY - 180;
}
