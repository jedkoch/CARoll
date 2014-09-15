//6.03 to 6.07 is for the v15.8.2 3AXIS board with DIG2 PWMDIG3 DIG4 connected to board mounted LEDs.
//6.07 is clean running system with flashers at boot and color change for higher g-force

//Include Libraries
#include <Servo.h>
#include <math.h>
#include <Wire.h>
#include <MMA8453_n0m1.h>

//Protos and vars
MMA8453_n0m1 accel;

float Q_angle  =  0.9; //0.9 
float Q_gyro   =  0.9;  //0.9
float R_angle  =  180;  //180
int looptime = 50; //low values (e.g. 2) are much slower but more precise.
float newRate = 0.5;//values between 0.1(slower) to 0.8(faster) work well

//Define Pins
const int servoPin_fr = 5;
const int servoPin_fl = 6;
const int servoPin_br = 9;
const int servoPin_bl = 10;
const int servoMin = 1000;
const int servoMax = 2000;

int j;
//float scaledResult;
float xpos = 0, ypos = 0, zpos = 0;
volatile int xNEUTRAL = 90;
volatile int yNEUTRAL = 90;

//Create Servo Object
Servo fServoL;
Servo fServoR;
Servo bServoL;
Servo bServoR;

//unused - timing vars
unsigned long time = millis();
unsigned long previousMillis = 0;
int interval = 2;//how often the servos update in milliseconds

void setup()
{
  previousMillis = millis();
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  pinMode(9, OUTPUT);
  pinMode (10, OUTPUT);
  digitalWrite(5, HIGH);
  digitalWrite(6, HIGH);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
  digitalWrite(9, HIGH);
  digitalWrite(10, HIGH);
  accel.setI2CAddr(0x1C); //alt 0x1D if SAO pulled HIGH // default is 0x1C
  accel.dataMode(false, 2); //enable/disable highRes bitrate, G range [2,4,8] 

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

  fServoR.attach(servoPin_fr);
  fServoL.attach(servoPin_fl);
  bServoR.attach(servoPin_br);
  bServoL.attach(servoPin_bl);
}

void loop()
{
  accel.update();
  xpos = map(accel.y(), -512, 512, -60, 60);//the accel.y and accel.x are switched so the boards  
  ypos = map(accel.x(), -512, 512, -60, 60);// can be installed lengthwise on the vehicle
  zpos = map(accel.z(), -512, 512, 30, -20);
  xpos=kalmanCalculateX(xpos,newRate,looptime);
  ypos=kalmanCalculateY(ypos,newRate,looptime);
  zpos=kalmanCalculateZ(zpos,newRate,looptime);
  int fR = (xNEUTRAL+xpos+ypos+zpos);
  int fL = (xNEUTRAL-xpos+ypos-zpos);
  int bR = (xNEUTRAL+xpos-ypos+zpos);
  int bL = (xNEUTRAL-xpos-ypos-zpos);
  fR = constrain(fR,0,180);
  fL = constrain(fL,0,180);
  bR = constrain(bR,0,180);
  bL = constrain(bL,0,180);
  fServoR.write(fR);
  fServoL.write(fL);
  bServoR.write(bR);
  bServoL.write(bL);
  ledServoTest();
}


void ledServoTest()
{
  if(ypos<=1 && ypos>=-1)//if the board is level the white light will come on
  {
    digitalWrite(2, HIGH);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
  }
  else if((ypos>1 && ypos<25) || (ypos<-1 && ypos>-25))//the blue light is for medium g-force
  {
    digitalWrite(2, LOW);
    digitalWrite(3, HIGH);
    digitalWrite(4, LOW);
  }
  else if(ypos>=25 || ypos<=-25)//the red light is for high g-force
  {
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(4, HIGH);
  }
}
