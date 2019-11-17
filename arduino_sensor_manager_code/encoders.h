// implementation of code used to measure the quadrature encoder
// channels of a model mars rover
// aero3760, 2019
// 470333486

#ifndef _ENCODERS_H_
#define _ENCODERS_H_

// INCLUDES
#include "QuickMedianLib.h"

// DEFINES
// set the size and inx mask for circular buffers
// used to store sensor data, ensuring sensor data buffer
// does not grow unchecked into memory
#define BUFF 64
#define BUFF_MASK BUFF-1

// GLOBALS

// variables for storing most recent encoder 
// measurement. Time between consecuting rising 
// edge in timer ticks 
long int dTLeft = 0;
long int dTRight = 0;

// circular buffers for saving 64 most recent
// encoder measurements
long int left[BUFF];
long int right[BUFF];
int leftInx;
int rightInx;

// counts for the number of free counting timer overflows
long int leftTimerOF = 0;
long int rightTimerOF = 0;
long int timer1OF = 0;

// pins used for the encoder channels
int leftPin = 2;
int rightPin = 3;

// timer counter at previous interrupt
long int prevTimeRight = 0;
long int prevTimeLeft = 0;

// flags for transmitting sensor data to master
// raspberry pi
int isSerialReady = 0;
int serialCount = 0;

// object to find the median of encoder array
QuickMedian<long int> medianFinder;

// INTERRUPTS

// free counting timer used to measure the time between rising edges
// of the wheel encoders
ISR(TIMER1_OVF_vect) 
{
  noInterrupts();
  ++timer1OF;
  
  // check if the wheels are stationary
  if(timer1OF - leftTimerOF > 1)
  {
    dTLeft = 0;
    prevTimeLeft = 0;
    leftTimerOF = timer1OF;

    // start filling the encoder buffer with stationary measurements
    left[leftInx] = dTLeft;
    ++leftInx;
    leftInx &= BUFF_MASK;
  }
  if(timer1OF - rightTimerOF > 1)
  {
    dTRight = 0;
    prevTimeRight = 0;
    rightTimerOF = timer1OF;

    // start filling the encoder buffer with stationary measurements
    right[rightInx] = dTRight;
    ++rightInx;
    rightInx &= BUFF_MASK;
  }
  
  // ready to send next lot sensor of data to raspberry pi
  ++serialCount;
  if(serialCount == 10)
  {
    isSerialReady = 1; 
    serialCount = 0;
  }
  	
  interrupts();
}  

// rising edge interrpupt of left wheel encder channel
void leftISR()
{
  // get the difference between the time of this rising edge and the 
  // previous rising edge, accounting for the free counting timer overflow
  long int currentTime = TCNT1;
  if(currentTime > prevTimeRight)
  {
    dTLeft = currentTime - prevTimeLeft;
  }
  else
  {
    dTLeft = 65536 + currentTime - prevTimeLeft;
  }

  // fill the time in the encoder data buffer
  left[leftInx] = dTLeft;
  ++leftInx;
  leftInx &= BUFF_MASK;
  
  // update the previous overflow and timer records
  prevTimeLeft = currentTime;
  leftTimerOF = timer1OF;
}

// rising edge interrpupt of right wheel encder channel
void rightISR()
{

  // get the difference between the time of this rising edge and the 
  // previous rising edge, accounting for the free counting timer overflow
  long int currentTime = TCNT1;
  if(currentTime > prevTimeRight)
  {
    dTRight = currentTime - prevTimeRight;
  }
  else
  {
    dTRight = 65536 + currentTime - prevTimeRight;
  }

  // fill the data in the encoder array
  right[rightInx] = dTRight;
  ++rightInx;
  rightInx &= BUFF_MASK;
  
  // update the previous overflow and timer records
  prevTimeRight = currentTime;
  rightTimerOF = timer1OF;
}

// FUNCTIONS
void setupTimer1()
{
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  timer1OF = 0;      // intilise the overflow counter
  
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  
  TCCR1B |= (0 << CS22) | (1 << CS21) | (0 << CS20);   // 8 prescaler. (16MHz base freq)
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
}

// setup rising edge interrupts on the encoder channels
void setupInterrupts()
{
  noInterrupts();           // disable all interrupts
  pinMode(leftPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftPin), leftISR, CHANGE);
  pinMode(rightPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rightPin), rightISR, CHANGE);
  interrupts();             // enable all interrupts
}

// set the default values of the global variables
void setupGlobals()
{
  prevTimeLeft = 0;
  prevTimeRight = 0;
  leftPin = 2;
  rightPin = 3;  
  dTLeft = 0;
  dTRight = 0;
  leftTimerOF = 0;
  rightTimerOF = 0;
  timer1OF = 0;
  isSerialReady = 0;
  serialCount = 0;
  for(int i = 0; i<BUFF; ++i)
  {
    left[leftInx] = 0;
    right[rightInx] = 0;  
  }
  leftInx = 0;
  rightInx = 0;
}

#endif
