#define M1A 36
#define M1B 39
#define M2A 34
#define M2B 35

#define COUNTTPERCM 9.9

int encoder1 = 0;
int encoder2 = 0;

void setup() {
  Serial.begin(9600); 

  pinMode(M1A, INPUT);
  pinMode(M1B, INPUT);
  pinMode(M2A, INPUT);
  pinMode(M2B, INPUT);

  attachInterrupt(digitalPinToInterrupt(M1A), updateEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(M2A), updateEncoderL, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(encoder1/COUNTTPERCM);
  Serial.print("\t");
  Serial.println(encoder2/COUNTTPERCM);
}

void updateEncoder1() {
  if (digitalRead(M1B) == HIGH) {
    encoder1++;
  } else {
    encoder1--;
  }
}

void updateEncoderL() {
  if (digitalRead(M2B) == LOW) {
    encoder2++;
  } else {
    encoder2--;
  }
}