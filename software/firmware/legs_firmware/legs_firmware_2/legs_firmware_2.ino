#include <SoftwareSerial.h>

// === Serial Communication (to RPi) ===
#define RX_PIN 6
#define TX_PIN 5
SoftwareSerial RPiSerial(RX_PIN, TX_PIN); // RX, TX

// === Motor Driver Pins (No enable pins used) ===
#define IN1 8
#define IN2 7
#define IN3 12
#define IN4 11

// === Ultrasonic Sensors ===
#define TRIG_FRONT 4
#define ECHO_FRONT 3
#define TRIG_BOTTOM A0
#define ECHO_BOTTOM A1

// === Safety Distance (cm) ===
#define FRONT_LIMIT 20
#define PIT_LIMIT   10

// State tracking
char lastCmd = 'S';          
bool safetyStop = false;     
unsigned long safetyStart = 0; 

// Function prototypes
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void stopMotors();
long getDistance(int trigPin, int echoPin);
void executeCommand(char cmd);
void dance();

void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_BOTTOM, OUTPUT); pinMode(ECHO_BOTTOM, INPUT);

  stopMotors();

  RPiSerial.begin(38400);
  Serial.begin(9600);
  Serial.println("Rover Ready. Commands: F,B,L,R,S");
}

void loop() {
  long distFront = getDistance(TRIG_FRONT, ECHO_FRONT);
  long distBottom = getDistance(TRIG_BOTTOM, ECHO_BOTTOM);

  bool obstacle = (distFront > 0 && distFront < FRONT_LIMIT);
  bool pit      = (distBottom > PIT_LIMIT);

  // === Safety check ===
  if (obstacle || pit) {
    if (!safetyStop) {
      stopMotors();
      safetyStop = true;
      safetyStart = millis();
      Serial.println("STOP: Safety triggered!");
      RPiSerial.println("STOP: Safety triggered!");
    }
  }

  if (safetyStop) {
    // Ensure minimum 1 sec hard stop
    if (millis() - safetyStart >= 1000) {
      // Check again
      distFront = getDistance(TRIG_FRONT, ECHO_FRONT);
      distBottom = getDistance(TRIG_BOTTOM, ECHO_BOTTOM);
      obstacle = (distFront > 0 && distFront < FRONT_LIMIT);
      pit      = (distBottom > PIT_LIMIT);

      if (!obstacle && !pit) {
        // Safe → resume
        safetyStop = false;
        if (lastCmd != 'S') {
          executeCommand(lastCmd);
          Serial.println("Safe -> Resuming last command...");
        }
      } 
      // else → remain stopped (do NOT reset timer here, keeps rover stopped indefinitely)
    }
  } else {
    // Process new commands only if safe
    if (RPiSerial.available()) {
      char cmd = RPiSerial.read();
      executeCommand(cmd);
    }
  }
}

// === Execute a command ===
void executeCommand(char cmd) {
  switch (cmd) {
    case 'F': moveForward();  lastCmd = 'F'; Serial.println("Forward"); break;
    case 'B': moveBackward(); lastCmd = 'B'; Serial.println("Backward"); break;
    case 'L': turnLeft();     lastCmd = 'L'; Serial.println("Left"); break;
    case 'R': turnRight();    lastCmd = 'R'; Serial.println("Right"); break;
    case 'S': stopMotors();   lastCmd = 'S'; Serial.println("Stop (manual)"); break;
    case 'D': dance(); break;
    default: Serial.println("Unknown Command"); break;
  }
}

// === Motor Control ===
void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}
void moveBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}
void turnLeft() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}
void turnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}
void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// === Ultrasonic ===
long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000);
  if (duration == 0) return -1;
  return duration * 0.034 / 2;
}

void dance() {
  Serial.println("💃 Starting dance!");

  // Step 1: Wiggle forward-backward
  for (int i = 0; i < 3; i++) {
    moveForward();
    delay(300);
    moveBackward();
    delay(300);
  }

  // Step 2: Spin in place (clockwise)
  for (int i = 0; i < 2; i++) {
    turnRight();
    delay(500);
  }

  // Step 3: Spin in place (counter-clockwise)
  for (int i = 0; i < 2; i++) {
    turnLeft();
    delay(500);
  }

  // Step 4: Quick forward-backward hops
  moveForward(); delay(200);
  moveBackward(); delay(200);
  moveForward(); delay(200);
  moveBackward(); delay(200);

  // Step 5: Stop and wiggle wheels slightly
  stopMotors();
  delay(200);
  moveForward(); delay(100);
  moveBackward(); delay(100);
  stopMotors();

  Serial.println("💃 Dance finished!");
}

