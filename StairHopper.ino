#include <Servo.h>
//#include <HB25MotorControl.h>

#include <SPI.h>
#include <Adafruit_BLE_UART.h>

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART bluefruit = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

long millisAtRecieved = 0;

char LEFT_UP[5] = {'A','L','_','U','P'};
char LEFT_STOP[5] = {'A','L','S','T','P'};
char LEFT_DOWN[5] = {'A','L','D','W','N'};
char RIGHT_UP[5] = {'A','R','_','U','P'};
char RIGHT_STOP[5] = {'A','R','S','T','P'};
char RIGHT_DOWN[5] = {'A','R','D','W','N'};

char FORWARD[5] = {'F','O','R','W','A'};
char REVERSE[5] = {'B','A','C','K','W'};
char TURN_LEFT[5] = {'D','R','I','V', 'L'};
char TURN_RIGHT[5] = {'D','R','I','V', 'R'};
char DRIVE_STOP[5] = {'D','S','T','O','P'};

// Forward - Down B516 | Up B507
// Reverse - Down B615 | Up B606
// Turn Left - Down B714 | Up B705
// Turn Right - Down B813 | Up B804


const int L_RPWM = 4;
const int L_LPWM = 3;
const int R_RPWM = 6;
const int R_LPWM = 5;

const int RIGHT_ARM_PIN = 16;
const int LEFT_ARM_PIN = 17;

const int HEAD_PIN = 7;

const int LEFT_LEG = 0;
const int RIGHT_LEG = 1;
const int LEG_FORWARD = 0;
const int LEG_REVERSE = 1;

//const int ARM_UP = -500;
//const int ARM_DOWN = 500;
//const int STOP = 0;
const int ARM_UP = 1000;
const int ARM_DOWN = 2000;
const int STOP = 1500;

const int SPEED = 1;

int rightArmState = STOP;
int leftArmState = STOP;

int rightPWMState = LOW;
int leftPWMState = LOW;

unsigned long rightArmMillis = 0;
unsigned long leftArmMillis = 0;

//HB25MotorControl leftArmControl(LEFT_ARM_PIN);
//HB25MotorControl rightArmControl(RIGHT_ARM_PIN);
//Servo leftArm;
//Servo rightArm;

Servo head;
int headAngle = 90;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("yo poppi bootin up...");

  bluefruit.setDeviceName("WM BOT");
  bluefruit.begin();

  head.attach(HEAD_PIN);

  pinMode(R_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);
  pinMode(L_RPWM, OUTPUT);
  pinMode(L_LPWM, OUTPUT);
  digitalWrite(R_RPWM, LOW);
  digitalWrite(R_LPWM, LOW);
  digitalWrite(L_RPWM, LOW);
  digitalWrite(L_LPWM, LOW);

//  leftArmControl.begin();
//  rightArmControl.begin();
  pinMode(RIGHT_ARM_PIN, OUTPUT);
  pinMode(LEFT_ARM_PIN, OUTPUT);
  digitalWrite(RIGHT_ARM_PIN, LOW);
  digitalWrite(LEFT_ARM_PIN, LOW);
//  leftArm.attach(LEFT_ARM_PIN);
//  rightArm.attach(RIGHT_ARM_PIN);
}

aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;
boolean locked = false;
long previousMillis = 0;
long timeOpen = 3000;

void loop() {
  bluefruit.pollACI();

  // Ask what is our current status
  aci_evt_opcode_t status = bluefruit.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
        Serial.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) {
        Serial.println(F("* Connected!"));
    }
    if (status == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Disconnected or advertising timed out"));
        
    }
    // OK set the last status change to this one
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {
    // Lets see if there's any data for us!
    boolean hasData = false;
    if (bluefruit.available()) {
//      Serial.println("Data Available");
      hasData = true;
    }
    
    // OK while we still have something to read, get a character and print it out
    char keyCode[5] = {0,0,0,0,0};
    int loopNum = 0;
    while (bluefruit.available()) {
      int loopNum = 5 - bluefruit.available();
      char c = bluefruit.read();
      //Serial.print(c);
      keyCode[loopNum] = c;
    }

    if (hasData) {
      //Serial.println(keyCode[0]);
      processData(keyCode);
    }
    // Right Arm
    driveArm(RIGHT_ARM_PIN, rightArmState);
    // Left Arm
    driveArm(LEFT_ARM_PIN, leftArmState);
    // Drive Head
//    if (rightArmState == ARM_DOWN) headAngle++;
//    if (rightArmState == ARM_UP) headAngle--;
    

    if (Serial.available()) {
//      // Read a line from Serial
//      Serial.setTimeout(100); // 100 millisecond timeout
//      String s = Serial.readString();
//
//      // We need to convert the line to bytes, no more than 20 at this time
//      uint8_t sendbuffer[20];
//      s.getBytes(sendbuffer, 20);
//      char sendbuffersize = min(20, s.length());
//
//      Serial.print(F("\n* Sending -> \"")); Serial.print((char *)sendbuffer); Serial.println("\"");
//
//      // write the data
//      bluefruit.write(sendbuffer, sendbuffersize);
//        headAngle = Serial.parseInt();
//        Serial.println("Setting angle to " + headAngle);
    }
  }
}

boolean isCodeEqual(char *a, char *b) {
  for (int n = 0; n < 5; n++) {
    if (a[n] != b[n]) return false;
  }
  return true;
}

void processData(char code[5]) {
//  Serial.print("Processing code: ");
//  for (int i = 0; i < 5; i++) {
//    Serial.print(code[i]);
//  }
//  Serial.print("\n");
//  Serial.println(isCodeEqual(code, UP_PRESS) == true);
  if (code[0] == 'H') {
    char codeAngle[4] = {code[2], code[3], code[4]};
    headAngle = atoi(codeAngle);
//    Serial.println(headAngle);
    head.write(headAngle);
  }
  else if (code[0] == 'L') {
    char codeSpeed[4] = {code[1], code[2], code[3], code[4]};
    int pwmSpeed = atoi(codeSpeed);
    driveLeg(LEFT_LEG, pwmSpeed);
  }
  else if (code[0] == 'R') {
    char codeSpeed[4] = {code[1], code[2], code[3], code[4]};
    int pwmSpeed = atoi(codeSpeed);
    driveLeg(RIGHT_LEG, pwmSpeed);
  }
  else {
    if (isCodeEqual(code, RIGHT_UP)) {
      rightArmState = ARM_UP;
//      digitalWrite(8, HIGH);
//      Serial.println("R_UP");
//      driveArm(RIGHT_ARM_PIN, ARM_UP);
    }
    if (isCodeEqual(code, RIGHT_STOP)) {
      rightArmState = STOP;
//      digitalWrite(8, LOW);
//      driveArm(RIGHT_ARM_PIN, rightArmState);
    }
    if (isCodeEqual(code, RIGHT_DOWN)) {
      rightArmState = ARM_DOWN;
//      digitalWrite(8, HIGH);
//      driveArm(RIGHT_ARM_PIN, rightArmState);
    }
    if (isCodeEqual(code, LEFT_UP)) {
      leftArmState = ARM_UP;
//      digitalWrite(1, HIGH);
//      driveArm(LEFT_ARM_PIN, leftArmState);
    }
    if (isCodeEqual(code, LEFT_STOP)) {
      leftArmState = STOP;
//      digitalWrite(1, LOW);
//      driveArm(LEFT_ARM_PIN, leftArmState);
    }
    if (isCodeEqual(code, LEFT_DOWN)) {
      leftArmState = ARM_DOWN;
//      digitalWrite(1, HIGH);
//      driveArm(LEFT_ARM_PIN, leftArmState);
    }
    if (isCodeEqual(code, FORWARD)) {
      Serial.println("FORWARD");
      driveLeg(LEFT_LEG, LEG_FORWARD);
      driveLeg(RIGHT_LEG, LEG_FORWARD);
    }
    if (isCodeEqual(code, TURN_LEFT)) {
      Serial.println("LEFT");
      driveLeg(LEFT_LEG, LEG_REVERSE);
      driveLeg(RIGHT_LEG, LEG_FORWARD);
    }
    if (isCodeEqual(code, TURN_RIGHT)) {
      Serial.println("RIGHT");
      driveLeg(LEFT_LEG, LEG_FORWARD);
      driveLeg(RIGHT_LEG, LEG_REVERSE);
    }
    if (isCodeEqual(code, REVERSE)) {
      Serial.println("REVERSE");
      driveLeg(LEFT_LEG, LEG_REVERSE);
      driveLeg(RIGHT_LEG, LEG_REVERSE);
    }
    if (isCodeEqual(code, DRIVE_STOP)) {
      Serial.println("STOP");
      digitalWrite(R_RPWM, LOW);
      digitalWrite(R_LPWM, LOW);
      digitalWrite(L_RPWM, LOW);
      digitalWrite(L_LPWM, LOW);
    }
  }
}

void driveArm(int side, int armState) {
  unsigned long currentMillis = millis();
  
  if (side == LEFT_ARM_PIN) {
    if (leftPWMState = LOW) {
      if (currentMillis - leftArmMillis > 5250) {
        leftPWMState = HIGH;
        leftArmMillis = currentMillis;
      }
    } else {
      if (currentMillis - leftArmMillis >= armState) {
        leftPWMState = LOW;
        leftArmMillis = currentMillis;
      }
    }
    digitalWrite(side, leftPWMState);
  } else {
    if (rightPWMState = LOW) {
      if (currentMillis - rightArmMillis > 5250) {
        rightPWMState = HIGH;
        rightArmMillis = currentMillis;
      }
    } else {
      if (currentMillis - rightArmMillis >= armState) {
        rightPWMState = LOW;
        rightArmMillis = currentMillis;
      }
    }
    digitalWrite(side, rightPWMState);
  }
}

void driveLeg(int side, int legSpeed) {
  int lpwm;
  int rpwm;
  if (side == RIGHT_LEG) {
    lpwm = R_LPWM;
    rpwm = R_RPWM;
  } else {
    lpwm = L_LPWM;
    rpwm = L_RPWM;
  }

  if (legSpeed > 0) {
    digitalWrite(lpwm, SPEED);
    digitalWrite(rpwm, 0);
  } else if (legSpeed < 0) {
    digitalWrite(lpwm, 0);
    digitalWrite(rpwm, -SPEED);
  } else {
    digitalWrite(lpwm, LOW);
    digitalWrite(rpwm, LOW);
  }
}
