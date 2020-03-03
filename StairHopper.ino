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
const int AL_RPWM = 22;
const int AL_LPWM = 23;
const int AR_RPWM = 21;
const int AR_LPWM = 20;

const int RIGHT_ARM = 0;
const int LEFT_ARM = 1;

const int HEAD_PIN = 7;

const int LEFT_LEG = 0;
const int RIGHT_LEG = 1;
const int LEG_FORWARD = 255;
const int LEG_REVERSE = 0;

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
  pinMode(R_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);
  pinMode(L_RPWM, OUTPUT);
  pinMode(L_LPWM, OUTPUT);
  digitalWrite(R_RPWM, LOW);
  digitalWrite(R_LPWM, LOW);
  digitalWrite(L_RPWM, LOW);
  digitalWrite(L_LPWM, LOW);

//  pinMode(AR_RPWM, OUTPUT);
//  pinMode(AR_LPWM, OUTPUT);
//  pinMode(AL_RPWM, OUTPUT);
//  pinMode(AL_LPWM, OUTPUT);
//  digitalWrite(AR_RPWM, LOW);
//  digitalWrite(AR_LPWM, LOW);
//  digitalWrite(AL_RPWM, LOW);
//  digitalWrite(AL_LPWM, LOW);

  Serial.begin(9600);
  Serial.println("Cycler is booting up...");

  bluefruit.setDeviceName("WM BOT");
  bluefruit.begin();

  head.attach(HEAD_PIN);
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
//    driveArm(RIGHT_ARM_PIN, rightArmState);
    // Left Arm
//    driveArm(LEFT_ARM_PIN, leftArmState);
    

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
  Serial.print("Processing code: ");
  for (int i = 0; i < 5; i++) {
    Serial.print(code[i]);
  }
  Serial.print("\n");
  if (code[0] == 'H') {
    char codeAngle[4] = {code[2], code[3], code[4]};
    headAngle = atoi(codeAngle);
//    Serial.println(headAngle);
    head.write(headAngle);
  }
  else {
//    if (isCodeEqual(code, RIGHT_UP)) {
//      driveArm(RIGHT_ARM, ARM_UP);
//    }
//    if (isCodeEqual(code, RIGHT_STOP)) {
//      digitalWrite(AR_RPWM, LOW);
//      digitalWrite(AR_LPWM, LOW);
//    }
//    if (isCodeEqual(code, RIGHT_DOWN)) {
//      driveArm(RIGHT_ARM, ARM_DOWN);
//    }
//    if (isCodeEqual(code, LEFT_UP)) {
//      driveArm(LEFT_ARM, ARM_UP);
//    }
//    if (isCodeEqual(code, LEFT_STOP)) {
//      digitalWrite(AL_RPWM, LOW);
//      digitalWrite(AL_LPWM, LOW);
//    }
//    if (isCodeEqual(code, LEFT_DOWN)) {
//      driveArm(LEFT_ARM, ARM_DOWN);
//    }
    if (isCodeEqual(code, FORWARD)) {
      driveLeg(LEFT_LEG, LEG_FORWARD);
      driveLeg(RIGHT_LEG, LEG_FORWARD);
    }
    if (isCodeEqual(code, TURN_LEFT)) {
      driveLeg(LEFT_LEG, LEG_REVERSE);
      driveLeg(RIGHT_LEG, LEG_FORWARD);
    }
    if (isCodeEqual(code, TURN_RIGHT)) {
      driveLeg(LEFT_LEG, LEG_FORWARD);
      driveLeg(RIGHT_LEG, LEG_REVERSE);
    }
    if (isCodeEqual(code, REVERSE)) {
      driveLeg(LEFT_LEG, LEG_REVERSE);
      driveLeg(RIGHT_LEG, LEG_REVERSE);
    }
    if (isCodeEqual(code, DRIVE_STOP)) {
      digitalWrite(R_RPWM, LOW);
      digitalWrite(R_LPWM, LOW);
      digitalWrite(L_RPWM, LOW);
      digitalWrite(L_LPWM, LOW);
    }
  }
}

//void driveArm(int aside, int adir) {
//  int alpwm;
//  int arpwm;
//  if (aside == RIGHT_ARM) {
//    alpwm = AR_LPWM;
//    arpwm = AR_RPWM;
//  } else {
//    alpwm = AL_LPWM;
//    arpwm = AL_RPWM;
//  }
//
//  if (adir == ARM_UP) {
//    digitalWrite(arpwm, SPEED);
//    digitalWrite(alpwm, 0);
//  } else if (adir == ARM_DOWN) {
//    digitalWrite(arpwm, 0);
//    digitalWrite(alpwm, SPEED);
//  } else {
//    digitalWrite(alpwm, LOW);
//    digitalWrite(arpwm, LOW);
//  }
//}

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

  if (legSpeed == LEG_FORWARD) {
    digitalWrite(rpwm, SPEED);
    digitalWrite(lpwm, 0);
  } else if (legSpeed == LEG_REVERSE) {
    digitalWrite(rpwm, 0);
    digitalWrite(lpwm, SPEED);
  } else {
    digitalWrite(lpwm, LOW);
    digitalWrite(rpwm, LOW);
  }
}
