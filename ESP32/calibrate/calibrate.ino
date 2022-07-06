#define BAUD 115200
#include "wheel.h"

int start, end, diff;
float cpcR, cpcL;

void setup()
{
  Serial.begin(BAUD);

  // Wheel driver setup
  wheelSetup();

  Serial.setTimeout(1000*120);
}

void loop()
{
    // Calibrate right
    cpcR = 0;
    for (size_t i = 0; i < 10; i++)
    {
    
    start = encoderR;
    Serial.println("Rotate right wheel for 10 revolutions");
    Serial.println("Press enter after each one");
    Serial.readStringUntil('\n');
    end = encoderR;
    diff = end - start;
    cpcR += (diff) / (PI * WHEELDIAMETER);

    }

    // Calibrate left
    cpcL = 0;
    for (size_t i = 0; i < 10; i++)
    {
        
    start = encoderL;
    Serial.println("Rotate left wheel for 10 revolutions");
    Serial.println("Press enter after each one");
    Serial.readStringUntil('\n');
    end = encoderL;
    diff = end - start;
    cpcL += (diff) / (PI * WHEELDIAMETER);

    }

    Serial.println();
    Serial.print("Right cpc is: ");
    Serial.println(cpcR/10);

    Serial.println();
    Serial.print("Left cpc is: ");
    Serial.println(cpcL/10);
}
