#define RIGHT 0
#define LEFT 1


// Motor
#define ENA 25
#define IN1 26
#define IN2 27
#define IN3 14
#define IN4 12
#define ENB 13


// Encoder
#define M1A 36
#define M1B 39
#define M2A 34
#define M2B 35
#define COUNTTPERCM 9.9

int encoderR = 0;
int prev_distanceR = 0;
double omegaR = 0;
int encoderL = 0;
int prev_distanceL = 0;
double omegaL = 0;

void updateEncoderR();
void updateEncoderL();
void IRAM_ATTR updateOmega();


// Timer
hw_timer_t *timer = NULL;


// Setup
void wheelSetup() {
  // Motor
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Encoder
  pinMode(M1A, INPUT);
  pinMode(M1B, INPUT);
  pinMode(M2A, INPUT);
  pinMode(M2B, INPUT);
  attachInterrupt(digitalPinToInterrupt(M1A), updateEncoderR, RISING);
  attachInterrupt(digitalPinToInterrupt(M2A), updateEncoderL, RISING);

  // Timer
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &updateOmega, true);
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmEnable(timer);
}


// Motor functions
void motorWrite(int motor, int speed) {
  int enable, in_a, in_b;
  speed = max(min(speed, 100), -100);
  speed = map(speed, 0, 100, 0, 255);
  if (motor == RIGHT) {
    enable = ENA;
    in_a = IN1;
    in_b = IN2;
  }
  else {
    enable = ENB;
    in_a = IN3;
    in_b = IN4;
  }

  if (speed >= 0) {
    analogWrite(enable, speed);
    digitalWrite(in_a, HIGH);
    digitalWrite(in_b, LOW);
  }
  else {
    analogWrite(enable, speed);
    digitalWrite(in_a, LOW);
    digitalWrite(in_b, HIGH);
  }
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

int getDist(int motor) {
  if (motor == RIGHT) {
    return encoderR/COUNTTPERCM/100;
  }
  else {
    return encoderL/COUNTTPERCM/100;
  }
}

void IRAM_ATTR updateOmega(){
  int distanceR = getDist(RIGHT);
  int distanceL = getDist(LEFT);
  omegaR = (distanceR - prev_distanceR);
  omegaL = (distanceL - prev_distanceL);
  prev_distanceR = distanceR;
  prev_distanceL = distanceL;
}