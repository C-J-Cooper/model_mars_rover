// implementation of code used to setup the timers used
// in the sensor code for a model mars rover
// aero3760, 2019
// 470333486

#ifndef _SENSORS_H_
#define _SENSORS_H_

// flag used to detect that a timer 2 interrupt has 
// occured and integrate sensor data (e.g. gyroscope 
// angular velocity) at refular intervals
int isIntegrateSensors;

// timer 2 overflow, interrupt subroutine
ISR(TIMER2_OVF_vect) 
{
  noInterrupts();
  // set flag to integrate the sensor data
  isIntegrateSensors = 1;
  interrupts();
}  

// timer 2 setup, used to intergrate sensor data on overflow
void setupTimer2(void)
{
  // disable interrupts
  noInterrupts();
  
  // leave parameters to default and clear the timer
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  
  // set prescale to 1, 16MHz base clock frequency
  TCCR2B |= (0 << CS22) | (0 << CS21) | (1 << CS20);

  // enable overflow timer
  TIMSK2 |= (1 << TOIE1);

  // by default, set the integrate flag to false. will set
  // on first timer 2 overflow interrupt\
  isIntegrateSensors = 0;

  // enable all interrupts
  interrupts();             
}

#endif