// MAIN FILE 
// Arduino Uno code used to IMU sensor data and encoder infomation
// for a model mars rover. sensor data is parsed and sent to the 
// master raspberry pi over serial
// author: 470333496, 2019

// INCLUDES
#include "encoders.h"
#include "sensors.h"
#include "MPU9250.h"

// DEFINES
// time step for the angular velocity integral
#define dt 0.004096 

//GLOBALS

// objects for reading IMU data
MPU9250 IMU(Wire,0x68);
int status;

// angular velocity, as measured by the gyroscope
double yawRaw;
double yawCali;

//SETUP
void setup() {
  // setup serial
  Serial.begin(115200);
  while(!Serial) {}
  
  //setup encoders
  setupGlobals();
  noInterrupts();
  setupTimer1();
  setupTimer2();
  setupInterrupts();
  interrupts();
  yawRaw = 0;
  yawCali = 0;

  // setup IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    while(1) {}
  }
  
}

//MAIN LOOP
void loop() {

  // integrate the gyro data at regular intervals, set when timer 2 overflows
  if(isIntegrateSensors == 1)
  {
    // introducing a negative to match right hand rule of robot body frame
    yawRaw +=  - IMU.getGyroZ_rads() * dt; 
    yawCali = 14.081*yawRaw;
    isIntegrateSensors = 0;
  }

  // read the IMU data every loop
  IMU.readSensor();
  
  // send serial data at regular intervals to the host PC
  if(isSerialReady == 1)
  {
    // left and right encoder measurements are sent as time between rising edges
    // in timer ticks of timer 1. (16MHz clock with 8 prescale)
    // the median is sent to reduce the impact of noise

    // left wheel encoder data in timer ticks
    Serial.print(medianFinder.GetMedian(left, BUFF));
    Serial.print(" "); 

    // right wheel encoder data in timer ticks
    Serial.print(medianFinder.GetMedian(right, BUFF));
    Serial.print(" "); 

    // calibrated prientation about z axis, from gyroscope
    Serial.println(yawCali, 6); 
    Serial.print(" "); 

    // angular velocity around z axis, from gyroscope
    Serial.println(IMU.getGyroZ_rads(),6);      // gyro z

    isSerialReady = 0;
    
  }



}
