#include "wheel.h"
#include <PID_v1.h>

// PID variables
double SetpointR, SetpointL, setR, setL;
double Kp = 255, Ki = 600, Kd = 10;
PID PIDR(&omegaR, &setR, &SetpointR, Kp, Ki, Kd, DIRECT);
PID PIDL(&omegaL, &setL, &SetpointL, Kp, Ki, Kd, DIRECT);

void setup()
{
  Serial.begin(9600);

  wheelSetup();

  SetpointR = 0;
  SetpointL = -1;
  PIDR.SetMode(AUTOMATIC);
  PIDL.SetMode(AUTOMATIC);
  PIDR.SetOutputLimits(-255, 255);
  PIDL.SetOutputLimits(-255, 255);
}

void loop()
{
  PIDR.Compute();
  PIDL.Compute();
  motorWrite(RIGHT, setR);
  motorWrite(LEFT, setL);
  Serial.println(SetpointL);
}
