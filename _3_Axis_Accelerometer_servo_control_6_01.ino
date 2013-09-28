//_3_Axis_Accelerometer_servo_control_6_02
//Code for RCV3AXIS accelerometer board released as
//California Roll by eel electronics
//eelele.com
//working rev for The California Roll - 3AXIS RC-CAR v15.7
//6.02 added auto level during 4 second boot

//Include Libraries
#include <Servo.h>
#include <math.h>
#include <Wire.h>
#include <MMA8453_n0m1.h>

//Protos and vars
MMA8453_n0m1 accel;

float Q_angle  =  0.9; //0.4 
float Q_gyro   =  0.9;  //0.4
float R_angle  =  180;  //0.01
int looptime = 5;
float newRate = 0.2;//values between 0.1(slower) to 0.5(faster) work well

//Define Pins
const int servoPin_fr = 5;
const int servoPin_fl = 6;
const int servoPin_br = 9;
const int servoPin_bl = 10;
const int servoMin = 800;
const int servoMax = 2200;

int j;
float scaledResult;
float xpos = 0, ypos = 0, zpos = 0;
volatile int xNEUTRAL = 90;
volatile int yNEUTRAL = 90;

//Create Servo Object
Servo fServoL;
Servo fServoR;
Servo bServoL;
Servo bServoR;

//timing vars for auto cal during startup
volatile unsigned long time = millis();
volatile unsigned long tapTimeout = 4000;

void calMODE()//this calibrates the position of the device during startup
{
  time = millis();
  while(time < tapTimeout)
  {
    accel.update();
    xpos = map(accel.x(), -1024, 1024, -90, 90);  
    ypos = map(accel.y(), -1024, 1024, -90, 90);
    xNEUTRAL = (90+xpos);
    yNEUTRAL = (ypos);
    time = millis();
  }
  return;
}

void moveMODE()
{
  while(true)
  {
    accel.update();
    xpos = map(accel.x(), -1024, 1024, -90, 90);//the accel.y and accel.x can be switched so the boards  
    ypos = map(accel.y(), -1024, 1024, -90, 90);// can be installed lengthwise on the vehicle
    zpos = map(accel.z(), -1024, 1024, 30, -10);
    xpos=kalmanCalculateX(xpos,newRate,looptime);
    ypos=kalmanCalculateY(ypos,newRate,looptime);
    zpos=kalmanCalculateZ(zpos,newRate,looptime);
    int fR = (xNEUTRAL+xpos+ypos-yNEUTRAL);
    int fL = (xNEUTRAL-xpos+ypos-yNEUTRAL);
    int bR = (xNEUTRAL+xpos-ypos+yNEUTRAL);
    int bL = (xNEUTRAL-xpos-ypos+yNEUTRAL);
    fR = constrain(fR,0,180);
    fL = constrain(fL,0,180);
    bR = constrain(bR,0,180);
    bL = constrain(bL,0,180);
    fServoR.write(fR);
    fServoL.write(fL);
    bServoR.write(bR);
    bServoL.write(bL);
  }  
}

void setup()
{
  time = millis();
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode (10, OUTPUT);
  digitalWrite(5, HIGH);
  digitalWrite(6, HIGH);
  digitalWrite(9, HIGH);
  digitalWrite(10, HIGH);
  accel.setI2CAddr(0x1C); //alt 0x1D if SAO pulled HIGH // default is 0x1C
  //I2c.begin();
  fServoR.attach(servoPin_fr);
  fServoL.attach(servoPin_fl);
  bServoR.attach(servoPin_br);
  bServoL.attach(servoPin_bl);
}

void loop()
{
  accel.dataMode(true, 2);
  calMODE();//first two seconds after powerup are waiting for tap and calibrate
  accel.dataMode(true, 2); //enable highRes 10bit, 2g range [2g,4g,8g]
  moveMODE();//ready for
}