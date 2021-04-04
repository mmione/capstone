// Using PID_v1.h,  see example code at https://github.com/br3ttb/Arduino-PID-Library/
#include <PID_v1.h>

#define PIN_OUTPUT 3

// SETPOINT: our GOAL angle
// INPUT: angle reading
// OUTPUT: PWM pin of motor. 
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{
  //initialize the variables we're linked to
  Input = 0;
  Setpoint = 0; // This is the angle we are always trying to target.

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  
}

void loop()
{
  // We don't get our inputs this simply-- we either need to assign Input 
  Input = analogRead(PIN_INPUT);
  myPID.Compute();
  // Compute() function simply calculates and "puts" what it thinks
  // is the "right" value to the motor. 
  
  analogWrite(PWM_PIN, Output);
}
