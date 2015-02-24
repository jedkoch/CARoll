//Include Libraries
#include <Servo.h>
#include <math.h>
#include <Wire.h>
#include "MMA8453_n0m1.h"

//Protos and vars
MMA8453_n0m1 accel;
//LiquidTWI2 lcd(0);

//variables for Kalman filtering
float newRate = 0.3;//1.4//values between 0.1(slower) to 0.8(faster) work well
int looptime = 12; //46 // 50 is quick. low values (e.g. 2) are much slower but more precise.
float R_angle  =  1.8;  //1.10 //180
float Q_angle  =  0.25; //0.6 //0.9 
float Q_gyro   =  0.27;  //0.6//0.9
int SETgForce = 2;//gforce range 2, 4, 8
int gForce = 1;//this sets the SETgForce var. eg 1 = SETgForce = 0-2 range, 2 = SETgForce = 0-4 range, 3 = SETgForce = 0-8 range  
int gRange =128;

//Define Pins
const int servoPin_fr = 9;
const int servoPin_fl = 10;
const int servoPin_br = 6;
const int servoPin_bl = 5;

int j;
//float scaledResult;
volatile float xpos = 0.0, ypos = 0.0, zpos = 0.0;
float frNEUTRAL = 20.0, flNEUTRAL = 20.0, brNEUTRAL = 20.0, blNEUTRAL = 20.0;
int zOffset=0;

//Create Servo Object
Servo fServoL;
Servo fServoR;
Servo bServoL;
Servo bServoR;
float fR = 20.0;
float fL = 20.0;
float bR = 20.0;
float bL = 20.0;

//unused - timing vars
unsigned long time = millis();
unsigned long previousMillis = 0;
//int interval = 4;//default = 2; how often the servos update in milliseconds

//controller Vars
int isSelectPressed = 0;
volatile int menuItem = 0;
volatile int incValue = 0;

//for memory purposes. int stores a 16-bit (2-byte) value. This yields a range of -32,768 to 32,767
int eepromInts[15];
float eepromFloats[7];
//storeBuffer(); //enable during first upload

//performance test vars
byte digit = 0;
unsigned long freqMult = 0;

void setup()
{
  previousMillis = millis();
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(servoPin_fr, OUTPUT);
  pinMode(servoPin_fl, OUTPUT);
  pinMode(servoPin_br, OUTPUT);
  pinMode (servoPin_bl, OUTPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
  //digitalWrite(servoPin_fr, HIGH);
  //digitalWrite(servoPin_fl, HIGH);
  //digitalWrite(servoPin_br, HIGH);
  //digitalWrite(servoPin_bl, HIGH);
  fServoR.attach(servoPin_fr);
  fServoL.attach(servoPin_fl);
  bServoR.attach(servoPin_br);
  bServoL.attach(servoPin_bl);
  
  accel.setI2CAddr(0x1C); //alt 0x1D if SAO pulled HIGH // default is 0x1C
  accel.dataMode(true, gForce); //enable/disable highRes bitrate, G range [2,4,8] 

  //MMA8452Q:
  //The sensitivity is represented in counts/g. In 2g mode the sensitivity is 1024 counts/g. 
  //In 4g mode the sensitivity is 512 counts/g and in 8g mode the sensitivity is 256 counts/g.
  //
  //When the full-scale is set to 2g, the measurement range is -2g to +1.999g, and each count 
  //corresponds to 1g/1024 (1 mg) at 12-bits resolution. When the full-scale is set to 8g, the
  //measurement range is -8g to +7.996g, and each count corresponds to 1g/256 (3.9 mg) at 
  //12-bits resolution. The resolution is reduced by a factor of 16 if only the 8-bit results 
  //are used.

  //MMA8453Q
  //When the full-scale is set to 2g, the measurement range is -2g to +1.9961g, and each count 
  //corresponds to 1g/256 (3.9 mg) at 10-bits resolution. When the full-scale is set to 8g, 
  //the measurement range is -8g to +7.9844g, and each count corresponds to 1g/64 (15.6 mg) 
  //at 10-bits resolution. The resolution is reduced by a factor of 4 if only the 8-bit results are used.

}

void loop()
{
  ledServoTest();
  accelerometerLoop();
}

void accelerometerLoop()
{
  int g = (gRange*SETgForce)*3;
  accel.update();
  xpos = accel.x();//the accel.y and accel.x can be switched so the boards  
  ypos = accel.y();// can be installed lengthwise on the vehicle
  //zpos = accel.z()*-1.0;
  xpos=kalmanCalculateX(xpos,newRate,looptime);
  ypos=kalmanCalculateY(ypos,newRate,looptime);
  //zpos=kalmanCalculateZ(zpos,newRate,looptime);
  
  fR = ((xpos*-1.0)- ypos);
  fL = ((xpos*-1.0)+ ypos); //fL = (xpos+(ypos*-1.0));
    
  bR = (xpos-ypos);            //(256+256+256) = gRange(64,128,256) * setGForce(8,4,2) and the negative version
  bL = (xpos+ypos);
  
  
  fR = map(fR, -g, g, 0.1, 185.0)+frNEUTRAL;
  fL = map(fL, -g, g, 0.1, 185.0)+flNEUTRAL;
  bR = map(bR, -g, g, 0.1, 185.0)+brNEUTRAL;
  bL = map(bL, -g, g, 0.1, 185.0)+flNEUTRAL;
  
  fServoR.write(fR);
  fServoL.write(fL);
  bServoR.write(bR);
  bServoL.write(bL);
}

void ledServoTest()
{
  if(xpos<=3.0 && xpos>=-3.0 && ypos<=3.0 && ypos>=-3.0)//if the board is level the white light will come on
  {
    digitalWrite(2, HIGH);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
  }
  else if((xpos>3.0 && xpos<75.0) || (xpos<-3.0 && xpos>-75.0))//the blue light is for medium g-force
  {
    digitalWrite(2, LOW);
    digitalWrite(3, HIGH);
    digitalWrite(4, LOW);
  }
  else if(xpos>=75.0 || xpos<=-75.0)//the red light is for high g-force
  {
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(4, HIGH);
  }
}










































