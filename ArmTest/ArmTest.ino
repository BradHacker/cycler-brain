const int RIGHT_ARM_PIN = 16;
const int LEFT_ARM_PIN = 17;

const unsigned long FORWARD = 1000;
const unsigned long REVERSE = 2000;
const unsigned long STOP = 1500;

unsigned long rightArmState = STOP;
unsigned long leftArmState = STOP;

int rightPWMState = LOW;
int leftPWMState = LOW;

unsigned long pulseGutter = 6000;

unsigned long rightArmMillis = 0;
unsigned long leftArmMillis = 0;

unsigned long totalMillis = 0;
int numPulses = 0;

void setup() {
  pinMode(RIGHT_ARM_PIN, OUTPUT);
  pinMode(LEFT_ARM_PIN, OUTPUT);
  digitalWrite(RIGHT_ARM_PIN, LOW);
  digitalWrite(LEFT_ARM_PIN, LOW);

//  Serial.begin(9600);
//  Serial.println("Arm test booted up...");
}

unsigned long previousMillis = 0;

void loop() {
  unsigned long currentMillis = millis();
  unsigned long avgPulseLength = totalMillis / (unsigned long) numPulses;
//  if (currentMillis - previousMillis >= 4000) {
//    rightArmState = STOP;
//    leftArmState = STOP;
//    previousMillis = currentMillis;
////    Serial.println("STOP");
//    Serial.println(avgPulseLength);
//    totalMillis = 0;
//    numPulses = 0;
//  }
//  else if (currentMillis - previousMillis >= 3000) {
//    rightArmState = REVERSE;
//    leftArmState = REVERSE;
////    Serial.println("REVERSE");
//    Serial.println(avgPulseLength);
//    totalMillis = 0;
//    numPulses = 0;
//  }
//  else if (currentMillis - previousMillis >= 2000) {
//    rightArmState = STOP;
//    leftArmState = STOP;
////    Serial.println("STOP");
//    Serial.println(avgPulseLength);
//    totalMillis = 0;
//    numPulses = 0;
//  }
//  else if (currentMillis - previousMillis >= 1000) {
//    rightArmState = FORWARD;
//    leftArmState = FORWARD;
////    Serial.println("FORWARD");
//    Serial.println(avgPulseLength);
//    totalMillis = 0;
//    numPulses = 0;
//  }
    if (currentMillis - previousMillis >= 1000) {
      previousMillis = currentMillis;
      if (rightArmState == FORWARD) {
        rightArmState = REVERSE;
        leftArmState = REVERSE;
      } else if (rightArmState == REVERSE) {
        rightArmState = STOP;
        leftArmState = STOP;
      } else {
        rightArmState = REVERSE;
        leftArmState = REVERSE;
      }
    }
//  if (numPulses > 0 && currentMillis % 500 == 0) Serial.println(totalMillis / (long) numPulses);
  // Right Arm
  driveArm(RIGHT_ARM_PIN, rightArmState);
  // Left Arm
  driveArm(LEFT_ARM_PIN, leftArmState);
}

void driveArm(int side, unsigned long armState) {
  unsigned long currentMillis = millis();
  
  if (side == LEFT_ARM_PIN) {
    if (leftPWMState == LOW) {
      if ((currentMillis - leftArmMillis) * (unsigned long) 1000 > pulseGutter) {
        leftPWMState = HIGH;
        leftArmMillis = currentMillis;
      }
    } else {
      if ((currentMillis - leftArmMillis) * (unsigned long) 1000 >= armState) {
        totalMillis += currentMillis - leftArmMillis;
        numPulses++;
        leftPWMState = LOW;
        leftArmMillis = currentMillis;
      }
    }
    digitalWrite(side, leftPWMState);
  } else {
    if (rightPWMState == LOW) {
      if ((currentMillis - rightArmMillis) * (unsigned long) 1000 > pulseGutter) {
        rightPWMState = HIGH;
        rightArmMillis = currentMillis;
      }
    } else {
      if ((currentMillis - rightArmMillis) * (unsigned long) 1000 >= armState) {
        rightPWMState = LOW;
        rightArmMillis = currentMillis;
      }
    }
    digitalWrite(side, rightPWMState);
  }
}
