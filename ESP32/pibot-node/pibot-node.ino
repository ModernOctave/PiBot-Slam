#include "wheel.h"
#include "ros-node.h"
#include <PID_v1.h>

// PID variables (0,300,0)
double SetpointR = 0, SetpointL = 0, setR = 0, setL = 0;
double Kp = 50, Ki = 300, Kd = 0;
PID PIDR(&omegaR, &setR, &SetpointR, Kp, Ki, Kd, DIRECT);
PID PIDL(&omegaL, &setL, &SetpointL, Kp, Ki, Kd, DIRECT);

void setup()
{
  Serial.begin(BAUD);

  // Wheel driver setup
  wheelSetup();

  // PID setup
  PIDR.SetMode(AUTOMATIC);
  PIDL.SetMode(AUTOMATIC);
  PIDR.SetOutputLimits(-255, 255);
  PIDL.SetOutputLimits(-255, 255);

  // ROS node setup
  rosSetup();
}

void loop()
{
  // Calculate Setpoints in revolutions per second
  // We get value in radians per second by kinematics equations
  // 2*pi radians per revolution
  // Hence divide by 2*pi to get revolutions per second
  SetpointR = (x * 100 + z * AXIL_LENGTH / 2) / (WHEELDIAMETER / 2) / (2 * PI);
  SetpointL = (x * 100 - z * AXIL_LENGTH / 2) / (WHEELDIAMETER / 2) / (2 * PI);

  PIDR.Compute();
  PIDL.Compute();
  motorWrite(RIGHT, setR);
  motorWrite(LEFT, setL);

  rosLoop();
}
