#include <Servo.h>

const int rightArmPin = 38;
const unsigned long FORWARD = 1000;
const unsigned long STOP = 1500;
const unsigned long REVERSE = 2000;
const unsigned long HOLD_OFF_TIME = 8000;

Servo rightArm;
unsigned long _timeOfLastCommand;

void setup() {
  delay(5);
  pinMode(rightArmPin, OUTPUT);
  digitalWrite(rightArmPin, LOW);
  rightArm.attach(rightArmPin, 800, 2200);
  rightArm.writeMicroseconds(STOP);
  delay(5);
}

unsigned long previousMillis = 0;

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis <= 1000) forward();
  else if (currentMillis - previousMillis <= 2000) reverse();
  else if (currentMillis - previousMillis <= 3000) previousMillis = currentMillis;
}

void forward() {
  checkHoldOffTime();
  rightArm.writeMicroseconds(FORWARD);
  _timeOfLastCommand = millis();
}

void reverse() {
  checkHoldOffTime();
  rightArm.writeMicroseconds(REVERSE);
  _timeOfLastCommand = millis();
}

void checkHoldOffTime() {
  unsigned long timeSinceStartUp = millis();
  if (timeSinceStartUp <= _timeOfLastCommand) {
    delay(HOLD_OFF_TIME);
  }
  unsigned long timeSinceLastCommand = timeSinceStartUp - _timeOfLastCommand;
  if (timeSinceLastCommand < HOLD_OFF_TIME) {
    delay(HOLD_OFF_TIME - timeSinceLastCommand);
  } 
}
