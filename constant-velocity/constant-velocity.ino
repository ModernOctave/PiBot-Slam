#include "wheel.h"
#include <PID_v1.h>

// PID variables
double SetpointR, SetpointL, setR, setL;
double Kp=2, Ki=5, Kd=1;
PID PIDR(&omegaR, &setR, &SetpointR, Kp, Ki, Kd, DIRECT);
PID PIDL(&omegaL, &setL, &SetpointL, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);

  wheelSetup();

  SetpointR = 1;
  SetpointL = 1;
  PIDR.SetMode(AUTOMATIC);
}

void loop() {
  PIDR.Compute();
  motorWrite(RIGHT, setR);
  // motorWrite(LEFT, setL);
}
