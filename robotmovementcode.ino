#include <Servo.h>
#include <math.h>

#define L1 31.0
#define L2 27.0
#define JOYSTICK_BUTTON_PIN 2

Servo servoBase, servoShoulder, servoElbow, servoWrist, servoGripper;

int currentBase = 90;
int currentShoulder = 90;
int currentElbow = 90;
int currentWrist = 90;
int currentGripper = 90;

const int smoothDelay = 10;

void setup() {
  Serial.begin(9600);
  pinMode(JOYSTICK_BUTTON_PIN, INPUT_PULLUP);

  servoBase.attach(3);
  servoShoulder.attach(4);
  servoElbow.attach(5);
  servoWrist.attach(6);
  servoGripper.attach(7);

  moveServoSmooth(servoBase, currentBase, currentBase);
  moveServoSmooth(servoShoulder, currentShoulder, currentShoulder);
  moveServoSmooth(servoElbow, currentElbow, currentElbow);
  moveServoSmooth(servoWrist, currentWrist, currentWrist);
  moveServoSmooth(servoGripper, currentGripper, currentGripper);
}

void loop() {
  if (digitalRead(JOYSTICK_BUTTON_PIN) == LOW) {
    delay(200);
    while (digitalRead(JOYSTICK_BUTTON_PIN) == LOW);
    pickAndPlace();
  }
}

void moveServoSmooth(Servo &servo, int &currentAngle, int targetAngle) {
  targetAngle = constrain(targetAngle, 0, 180);
  int step = (targetAngle > currentAngle) ? 1 : -1;
  for (int pos = currentAngle; pos != targetAngle; pos += step) {
    servo.write(pos);
    delay(smoothDelay);
  }
  servo.write(targetAngle);
  currentAngle = targetAngle;
}

void computeAndMoveIK(float x, float y) {
  float dist = sqrt(x * x + y * y);
  if (dist > (L1 + L2)) return;

  float cosTheta2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  float theta2 = acos(cosTheta2);
  float k1 = L1 + L2 * cos(theta2);
  float k2 = L2 * sin(theta2);
  float theta1 = atan2(y, x) - atan2(k2, k1);
  float thetaWrist = -(theta1 + theta2);

  int angleShoulder = constrain(degrees(theta1), 0, 180);
  int angleElbow = constrain(degrees(theta2), 0, 180);
  int angleWrist = constrain(degrees(thetaWrist), 0, 180);

  moveServoSmooth(servoShoulder, currentShoulder, angleShoulder);
  moveServoSmooth(servoElbow, currentElbow, angleElbow);
  moveServoSmooth(servoWrist, currentWrist, angleWrist);
}

void pickAndPlace() {
  moveServoSmooth(servoBase, currentBase, 60);
  computeAndMoveIK(40.0, 10.0);
  delay(300);

  moveServoSmooth(servoGripper, currentGripper, 40);
  delay(500);

  computeAndMoveIK(40.0, 20.0);
  delay(500);

  moveServoSmooth(servoBase, currentBase, 120);
  delay(500);

  computeAndMoveIK(40.0, 10.0);
  delay(500);

  moveServoSmooth(servoGripper, currentGripper, 90);
  delay(500);

  computeAndMoveIK(35.0, 20.0);
  moveServoSmooth(servoBase, currentBase, 90);
}