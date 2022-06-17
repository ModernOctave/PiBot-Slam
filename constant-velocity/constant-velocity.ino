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
  SetpointL = 0.3;
  PIDR.SetMode(AUTOMATIC);
  PIDL.SetMode(AUTOMATIC);
}

void loop()
{
  PIDR.Compute();
  PIDL.Compute();
  motorWrite(RIGHT, setR);
  motorWrite(LEFT, setL);
}
