CARoll
======

CARoll is an active suspension system for RC scale vehicles. It is an AVR + accelerometer board that is programmed to help keep your RC Car, Quadcopter, camera, or autonomous robot level. It is based on a accelerometer and Arduino Compatible microcontroller combination. The CARoll reads accelerometer data over an I2C bus (that has been broken out along the top pins for further expansion); It uses this data to control 4 servo outputs for the purpose of creating a stable, platform (i.e. chassis.)

Dependencies:
MMA8453_n0m1.h


![My image](http://eeltronix.com/RCV3AXIS-onGitHub-600px.jpg)

The ISP port is compatible with the Arduino IDE (emulating an Arduino Mini Pro 16Mhz 5V with Atmega328p) 
An external AVR programmer or Arduino in ISP mode can be used to download software changes/updates.
