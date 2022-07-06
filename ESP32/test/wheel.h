#define RIGHT 0
#define LEFT 1
#define M_PI 3.14159265358979323846
#define WHEELDIAMETER 6.5
#define AXIL_LENGTH 19.5

// Encoder
#define M1A 36
#define M1B 39
#define M2A 34
#define M2B 35
#define COUNTTPERCM 9.9
#define OMEGASAMPLEFREQ 5

int encoderR = 0;
double prev_distanceR = 0;
double omegaR = 0, displacementR = 0, velocityR = 0;
int encoderL = 0;
double prev_distanceL = 0;
double omegaL = 0, displacementL = 0, velocityL = 0;
double base_linear_disp = 0, base_angular_disp = 0;
double pose_x = 0, pose_y = 0, pose_theta = 0;

void updateEncoderR();
void updateEncoderL();
void IRAM_ATTR updateOmega();


// Setup
void wheelSetup() {
  // Encoder
  pinMode(M1A, INPUT);
  pinMode(M1B, INPUT);
  pinMode(M2A, INPUT);
  pinMode(M2B, INPUT);
  attachInterrupt(digitalPinToInterrupt(M1A), updateEncoderR, RISING);
  attachInterrupt(digitalPinToInterrupt(M2A), updateEncoderL, RISING);
}


// Encoder functions
void updateEncoderR() {
  if (digitalRead(M1B) == HIGH) {
    encoderR++;
  } else {
    encoderR--;
  }
}

void updateEncoderL() {
  if (digitalRead(M2B) == LOW) {
    encoderL++;
  } else {
    encoderL--;
  }
}

double getDist(int motor) {
  if (motor == RIGHT) {
    return (double)encoderR/COUNTTPERCM;
  }
  else {
    return (double)encoderL/COUNTTPERCM;
  }
}