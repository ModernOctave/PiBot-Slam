#define BAUD 115200
#include "wheel.h"

int start, end, diff;
float cpcR, cpcL;

void setup()
{
  Serial.begin(BAUD);

  // Wheel driver setup
  wheelSetup();

  Serial.setTimeout(1000 * 120);
}

void loop()
{
  Serial.print("Right encoder: \t");
  Serial.print(encoderR);
  Serial.print("\tLeft encoder: \t");
  Serial.println(encoderL);
}
