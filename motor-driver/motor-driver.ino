#define ENA 25
#define IN1 26
#define IN2 27
#define IN3 14
#define IN4 12
#define ENB 13

void setup() {
  // put your setup code here, to run once:
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  motor_set_speed(0, 100);
  motor_set_speed(1, 100);
}

void motor_set_speed(int motor, int speed) {
  int enable, in_a, in_b;
  speed = map(speed, 0, 100, 0, 255);
  if (motor == 0) {
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
