#include <Servo.h>

// --- Servo Pins ---
#define SERVO1_PIN A0
#define SERVO2_PIN A1
#define SERVO3_PIN A2
#define SERVO4_PIN A3
Servo servo1, servo2, servo3, servo4;

// --- Motor 1 (Left) ---
#define RPWM1 3
#define LPWM1 5
#define R_EN1 4
#define L_EN1 2

// --- Motor 2 (Right) ---
#define RPWM2 6
#define LPWM2 11
#define R_EN2 12
#define L_EN2 7

// --- Encoder Pins ---
#define ENCODER1_CLK A5
#define ENCODER1_DT  A4
#define ENCODER2_CLK 9
#define ENCODER2_DT  8

// --- Robot Parameters ---
#define WHEEL_RADIUS 0.12  // 5 cm
#define WHEEL_BASE 0.31    // 31 cm
#define MAX_SPEED 2     // m/s
#define LOOP_INTERVAL 100  // ms
#define ENCODER_TICKS_PER_REV 39

long count1 = 0;
long count2 = 0;

int lastState1 = 0;
int lastState2 = 0;
unsigned long last_loop_time = 0;

float Vx = 0.0;
float Wz = 0.0;

void setup() {
  Serial.begin(9600);

  pinMode(R_EN1, OUTPUT); digitalWrite(R_EN1, HIGH);
  pinMode(L_EN1, OUTPUT); digitalWrite(L_EN1, HIGH);
  pinMode(RPWM1, OUTPUT); pinMode(LPWM1, OUTPUT);

  pinMode(R_EN2, OUTPUT); digitalWrite(R_EN2, HIGH);
  pinMode(L_EN2, OUTPUT); digitalWrite(L_EN2, HIGH);
  pinMode(RPWM2, OUTPUT); pinMode(LPWM2, OUTPUT);

  pinMode(ENCODER1_CLK, INPUT);
  pinMode(ENCODER1_DT, INPUT);
  pinMode(ENCODER2_CLK, INPUT);
  pinMode(ENCODER2_DT, INPUT);

  lastState1 = digitalRead(ENCODER1_CLK);
  lastState2 = digitalRead(ENCODER2_CLK);

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);

  last_loop_time = millis();
}

void loop() {
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);

  // --- Encoder Update ---
  int newState1 = digitalRead(ENCODER1_CLK);
  if (newState1 != lastState1) {
    if (digitalRead(ENCODER1_DT) != newState1) count1++;
    else count1--;
    lastState1 = newState1;
  }

  int newState2 = digitalRead(ENCODER2_CLK);
  if (newState2 != lastState2) {
    if (digitalRead(ENCODER2_DT) != newState2) count2++;
    else count2--;
    lastState2 = newState2;
  }

  // --- ROS cmd_vel Parsing ---
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n'); // Read the full line
    int spaceIndex = command.indexOf(' ');
    if (spaceIndex != -1) {
      String linear_str = command.substring(0, spaceIndex);
      String angular_str = command.substring(spaceIndex + 1);

      Vx = linear_str.toFloat();
      Wz = angular_str.toFloat();
    }
  }

  // --- Speed Feedback ---
  if (millis() - last_loop_time >= LOOP_INTERVAL) {
    float delta_t = (millis() - last_loop_time) / 1000.0;
    last_loop_time = millis();
      processVelocity(Vx, Wz); // Control Motors

    // float left_speed = (count1) / 10.4;
    // float right_speed = (count2) /10.4;

    // Serial.print("speed ");
    // Serial.print(left_speed);
    // Serial.print(" ");
    // Serial.print(right_speed);
    // Serial.print(" ");
    // Serial.println(LOOP_INTERVAL / 1000.0);

    count1 = 0;
    count2 = 0;
  }
}

// --- Helpers ---
int velocityToPWM(float velocity) {
  return constrain((int)((velocity / MAX_SPEED) * 255), -255, 255);
}

void setMotor(int RPWM, int LPWM, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    analogWrite(RPWM, speed);
    analogWrite(LPWM, 0);
  } else {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, -speed); // Motor needs positive value to turn in reverse
  }
}

void processVelocity(float linear, float angular) {
  // Calculate left and right wheel speeds
  float left_speed = linear - (angular * WHEEL_BASE / 2);
  float right_speed = linear + (angular * WHEEL_BASE / 2);
    Serial.print("speed ");
    Serial.print(left_speed);
    Serial.print(" ");
    Serial.print(right_speed);
    Serial.print(" ");
    Serial.println(LOOP_INTERVAL / 1000.0);
  // Convert speeds to PWM values
  int pwm_left = velocityToPWM(left_speed);
  int pwm_right = velocityToPWM(right_speed);

  // Set motors
  setMotor(RPWM1, LPWM1, pwm_left);
  setMotor(RPWM2, LPWM2, pwm_right);
}
