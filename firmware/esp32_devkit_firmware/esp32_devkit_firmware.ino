/*
 * ==============================================================================
 * ESP32 NodeMCU-32S / DevKit (30-PIN) - Wireless Smart Waste Robot Controller
 * ==============================================================================
 *
 *               [ ANTENNA / TOP ]
 *          (EN) [ ]            [ ] D23 (GPIO 23) -> [Free]
 *          (VP) [ ]            [ ] D22 (GPIO 22) -> PCA9685 SCL (I2C Clock)
 *          (VN) [ ]            [ ] TX0 (GPIO 1)  [Skip]
 *          D34  [ ]            [ ] RX0 (GPIO 3)  [Skip]
 *          D35  [ ]            [ ] D21 (GPIO 21) -> PCA9685 SDA (I2C Data)
 *  IN3 Fwd D32  [ ]            [ ] D19 (GPIO 19) -> [Free]
 *  IN4 Rev D33  [ ]            [ ] D18 (GPIO 18) -> HC-SR04 ECHO (via divider)
 *  IN1 Fwd D25  [ ]            [ ] D5  (GPIO 5)  -> HC-SR04 TRIG (Ultrasonic Trigger)
 *  IN2 Rev D26  [ ]            [ ] D17 (GPIO 17) -> [Free]
 *  ENA Spd D27  [ ]            [ ] D16 (GPIO 16) -> [Free]
 *       [Free] D14 [ ]         [ ] D4  (GPIO 4)  -> [Free]
 *       [Free] D12 [ ]         [ ] D2  (GPIO 2)  -> [Free]
 *  ENB Spd D13  [ ]            [ ] D15 (GPIO 15) -> [Free]
 *  GND     GND  [ ]            [ ] GND (GND)     -> Common GND (Battery - & ESP32 GND)
 *  VIN     VIN  [ ]            [ ] 3V3 (3.3V)    -> PCA9685 VCC (Logic Power)
 *               [  MICRO USB   ]
 *
 * MOTOR WIRING (L298N Dual H-Bridge Control):
 *   L298N IN1  -> ESP32 D25 (Left Motors Forward)
 *   L298N IN2  -> ESP32 D26 (Left Motors Reverse)
 *   L298N IN3  -> ESP32 D32 (Right Motors Forward)
 *   L298N IN4  -> ESP32 D33 (Right Motors Reverse)
 *   L298N ENA  -> ESP32 D27 (Left Speed Enable)
 *   L298N ENB  -> ESP32 D13 (Right Speed Enable)
 *   L298N 12V  -> Battery (+) 7.4V - 12V
 *   L298N GND  -> Battery (-) AND ESP32 GND (Common Ground)
 * ==============================================================================
 */

#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include "esp_wifi.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ------------------------------------------------------------------------------
// 1. WI-FI & HIGH-SPEED UDP CONFIGURATION (Port 8888)
// ------------------------------------------------------------------------------
const char *ssid         = "AADHIDHYA";
const char *password     = "12345678";

// Standalone Access Point Fallback (Active if Hotspot is offline)
const char *ap_ssid      = "SmartWasteDevKit";
const char *ap_password  = "12345678";
const IPAddress ap_IP    (192, 168, 4, 1);
const IPAddress ap_gateway(192, 168, 4, 1);
const IPAddress ap_subnet(255, 255, 255, 0);

const unsigned int UDP_PORT = 8888;
WiFiUDP udp;
char incomingPacket[256];
IPAddress lastSenderIP(192, 168, 137, 1);
uint16_t  lastSenderPort = 8888;
bool hasRemoteSender = false;

void sendResponse(const String &msg) {
  if (hasRemoteSender) {
    udp.beginPacket(lastSenderIP, lastSenderPort);
    udp.write((const uint8_t*)msg.c_str(), msg.length());
    udp.endPacket();
  }
  if (WiFi.status() == WL_CONNECTED) {
    IPAddress gw = WiFi.gatewayIP();
    if (gw != IPAddress(0, 0, 0, 0)) {
      udp.beginPacket(gw, UDP_PORT);
      udp.write((const uint8_t*)msg.c_str(), msg.length());
      udp.endPacket();
    }
    udp.beginPacket(IPAddress(192, 168, 137, 255), UDP_PORT);
    udp.write((const uint8_t*)msg.c_str(), msg.length());
    udp.endPacket();
  }
  Serial.println(msg);
}

// ------------------------------------------------------------------------------
// 2. MOTOR DRIVER HARDWARE PIN DEFINITIONS (L298N Dual H-Bridge Control)
// ------------------------------------------------------------------------------
// Exact physical connections:
const int pinIN1 = 25; // Left Motors Forward  (GPIO 25 -> IN1)
const int pinIN2 = 26; // Left Motors Reverse  (GPIO 26 -> IN2)
const int pinIN3 = 32; // Right Motors Forward (GPIO 32 -> IN3)
const int pinIN4 = 33; // Right Motors Reverse (GPIO 33 -> IN4)
const int pinENA = 27; // Left Speed Enable    (GPIO 27 -> ENA)
const int pinENB = 13; // Right Speed Enable   (GPIO 13 -> ENB)

// Drive configuration
int motorSpeed = 255; // Default full power (255 / 255)
bool invertLeftDrive  = false;
bool invertRightDrive = false;
bool isDrivingForward = false;
bool isMotorsRunning  = false;

inline void setLeftEnable(int spd) {
  int val = constrain(spd, 0, 255);
  if (val <= 0) {
    digitalWrite(pinENA, LOW);
  } else if (val >= 250) {
    digitalWrite(pinENA, HIGH);
  } else {
    analogWrite(pinENA, val);
  }
}

inline void setRightEnable(int spd) {
  int val = constrain(spd, 0, 255);
  if (val <= 0) {
    digitalWrite(pinENB, LOW);
  } else if (val >= 250) {
    digitalWrite(pinENB, HIGH);
  } else {
    analogWrite(pinENB, val);
  }
}

void setLeftDrive(int speed) {
  if (invertLeftDrive) speed = -speed;
  int pwm = constrain(abs(speed), 0, 255);

  if (speed > 0) {
    // Left Forward: IN1=HIGH, IN2=LOW, ENA=PWM
    digitalWrite(pinIN2, LOW);
    digitalWrite(pinIN1, HIGH);
    setLeftEnable(pwm);
  } else if (speed < 0) {
    // Left Reverse: IN1=LOW, IN2=HIGH, ENA=PWM
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, HIGH);
    setLeftEnable(pwm);
  } else {
    // Left Stop: IN1=LOW, IN2=LOW, ENA=LOW
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, LOW);
    setLeftEnable(0);
  }
}

void setRightDrive(int speed) {
  if (invertRightDrive) speed = -speed;
  int pwm = constrain(abs(speed), 0, 255);

  if (speed > 0) {
    // Right Forward: IN3=HIGH, IN4=LOW, ENB=PWM
    digitalWrite(pinIN4, LOW);
    digitalWrite(pinIN3, HIGH);
    setRightEnable(pwm);
  } else if (speed < 0) {
    // Right Reverse: IN3=LOW, IN4=HIGH, ENB=PWM
    digitalWrite(pinIN3, LOW);
    digitalWrite(pinIN4, HIGH);
    setRightEnable(pwm);
  } else {
    // Right Stop: IN3=LOW, IN4=LOW, ENB=LOW
    digitalWrite(pinIN3, LOW);
    digitalWrite(pinIN4, LOW);
    setRightEnable(0);
  }
}

void moveForward() {
  isDrivingForward = true;
  isMotorsRunning  = true;
  Serial.printf(">>> [MOTORS] FORWARD [W] speed=%d/255\n", motorSpeed);
  setLeftDrive(motorSpeed);
  setRightDrive(motorSpeed);
}

void moveBackward() {
  isDrivingForward = false;
  isMotorsRunning  = true;
  Serial.printf(">>> [MOTORS] BACKWARD [S] speed=%d/255\n", motorSpeed);
  setLeftDrive(-motorSpeed);
  setRightDrive(-motorSpeed);
}

void steerLeft() {
  isDrivingForward = false;
  isMotorsRunning  = true;
  Serial.printf(">>> [MOTORS] PIVOT LEFT [A] speed=%d/255\n", motorSpeed);
  setLeftDrive(-motorSpeed);
  setRightDrive(motorSpeed);
}

void steerRight() {
  isDrivingForward = false;
  isMotorsRunning  = true;
  Serial.printf(">>> [MOTORS] PIVOT RIGHT [D] speed=%d/255\n", motorSpeed);
  setLeftDrive(motorSpeed);
  setRightDrive(-motorSpeed);
}

void forwardCurveLeft() {
  isDrivingForward = true;
  isMotorsRunning  = true;
  Serial.println(">>> [MOTORS] CURVE LEFT [Q]");
  setLeftDrive(0);
  setRightDrive(motorSpeed);
}

void forwardCurveRight() {
  isDrivingForward = true;
  isMotorsRunning  = true;
  Serial.println(">>> [MOTORS] CURVE RIGHT [E]");
  setLeftDrive(motorSpeed);
  setRightDrive(0);
}

void backCurveLeft() {
  isDrivingForward = false;
  isMotorsRunning  = true;
  Serial.println(">>> [MOTORS] BACK LEFT [Z]");
  setLeftDrive(0);
  setRightDrive(-motorSpeed);
}

void backCurveRight() {
  isDrivingForward = false;
  isMotorsRunning  = true;
  Serial.println(">>> [MOTORS] BACK RIGHT [C]");
  setLeftDrive(-motorSpeed);
  setRightDrive(0);
}

void stopMotors() {
  isDrivingForward = false;
  isMotorsRunning  = false;
  digitalWrite(pinIN1, LOW);
  digitalWrite(pinIN2, LOW);
  digitalWrite(pinIN3, LOW);
  digitalWrite(pinIN4, LOW);
  digitalWrite(pinENA, LOW);
  digitalWrite(pinENB, LOW);
}

// Safety Watchdog: Stops motors if no command received within 5 seconds
unsigned long lastPacketTime = 0;
const unsigned long SAFETY_TIMEOUT_MS = 5000;

// ------------------------------------------------------------------------------
// 3. HC-SR04 ULTRASONIC SENSOR
// ------------------------------------------------------------------------------
#define TRIG_PIN 5
#define ECHO_PIN 18

const float OBSTACLE_STOP_DISTANCE_CM = 15.0;
float lastMeasuredDistanceCm = -1.0;
bool obstacleNear = false;
bool autoBrakeEnabled = true;

float readUltrasonicDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  if (duration == 0 || duration >= 25000) return -1.0;
  float dist = (duration * 0.0343f) / 2.0f;
  // Ignore noise or disconnected readings (< 3cm or > 400cm)
  if (dist < 3.0f || dist > 400.0f) return -1.0;
  return dist;
}

unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 1000;

void sendTelemetry() {
  float dist = readUltrasonicDistanceCm();
  lastMeasuredDistanceCm = dist;
  obstacleNear = (dist >= 3.0f && dist <= OBSTACLE_STOP_DISTANCE_CM);

  if (isDrivingForward && autoBrakeEnabled && obstacleNear) {
    Serial.printf("⚠️ [OBSTACLE] Distance = %.1f cm! Auto-Braking!\n", dist);
    stopMotors();
  }

  char teleBuf[64];
  snprintf(teleBuf, sizeof(teleBuf), "TELEM:DIST=%.1f,OBST=%d", 
           dist, obstacleNear ? 1 : 0);
  sendResponse(teleBuf);
}

// ------------------------------------------------------------------------------
// 4. PCA9685 16-CHANNEL SERVO DRIVER (6-DOF Robotic Arm)
// ------------------------------------------------------------------------------
#define CH_BASE         0 // M1 Base Rotation
#define CH_SHOULDER     1 // M2 Shoulder
#define CH_ELBOW        2 // M3 Elbow
#define CH_WRIST_PITCH  3 // M4 Wrist Pitch
#define CH_WRIST_ROLL   4 // M5 Wrist Roll
#define CH_GRIPPER      5 // M6 Claw / Gripper

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);
#define SERVO_FREQ 50

// Safe calibrated pulse limits per channel (0 to 5) at 50Hz (4096 counts per 20ms)
// 0 deg (~610us -> 125 counts) to 180 deg (~2410us -> 495 counts)
// Prevents internal mechanical gear end-stop crashing, jitter, and motor stall
const int servoMinPulses[6] = { 125, 125, 130, 130, 130, 130 }; // 0 deg
const int servoMaxPulses[6] = { 495, 495, 490, 485, 485, 485 }; // 180 deg

int currentAngles[6] = { 90, 100, 90, 90, 90, 30 }; // Initial angles
bool pcaReady = false;

bool ensurePcaReady() {
  if (pcaReady) return true;
  Wire.setTimeOut(50);
  Wire.beginTransmission(0x40);
  byte err = Wire.endTransmission();
  if (err == 0) {
    pcaReady = true;
    pca.begin();
    pca.setPWMFreq(SERVO_FREQ);
    Serial.println("✅ [PCA9685] Connected & armed at 0x40!");
    return true;
  }
  return false;
}

// Direct Degree Setting (0 to 180 deg)
void setServoAngle(uint8_t channel, int angle) {
  if (channel >= 6 || !ensurePcaReady()) return;
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, servoMinPulses[channel], servoMaxPulses[channel]);
  pca.setPWM(channel, 0, pulse);
  currentAngles[channel] = angle;
}

// Smooth Degree Interpolation
void smoothMoveServo(uint8_t channel, int targetAngle, int stepDelay = 8) {
  if (channel >= 6 || !ensurePcaReady()) return;
  targetAngle = constrain(targetAngle, 0, 180);
  int startAngle = currentAngles[channel];
  if (startAngle < 0) startAngle = 90;
  if (startAngle == targetAngle) {
    setServoAngle(channel, targetAngle);
    return;
  }
  int step = (targetAngle > startAngle) ? 1 : -1;
  for (int a = startAngle; a != targetAngle; a += step) {
    int pulse = map(a, 0, 180, servoMinPulses[channel], servoMaxPulses[channel]);
    pca.setPWM(channel, 0, pulse);
    delay(stepDelay);
  }
  setServoAngle(channel, targetAngle);
}

void armHome() {
  if (!ensurePcaReady()) return;
  Serial.println(">>> [ARM] Moving to HOME position (Exact Degrees)...");
  smoothMoveServo(CH_GRIPPER, 30, 6);     // M6 Open Claw (30 deg)
  smoothMoveServo(CH_WRIST_ROLL, 90, 6);  // M5 Wrist Roll Level (90 deg)
  smoothMoveServo(CH_WRIST_PITCH, 90, 6); // M4 Wrist Pitch Level (90 deg)
  smoothMoveServo(CH_ELBOW, 90, 8);       // M3 Elbow Level (90 deg)
  smoothMoveServo(CH_SHOULDER, 100, 8);   // M2 Shoulder Ready (100 deg)
  smoothMoveServo(CH_BASE, 90, 8);        // M1 Base Center (90 deg)
}

inline bool isDegradableWaste(String wasteClass) {
  wasteClass.toUpperCase(); wasteClass.trim();
  return (wasteClass == "BANANA" || wasteClass == "BANANA PEEL" || wasteClass == "BANANA_PEEL" || wasteClass == "DEGRADABLE" || wasteClass == "ORGANIC" || wasteClass == "FOOD");
}

void dropIntoRightDegradableBin() {
  // 1. Rotate Base M1 to Right Bin (175 deg)
  smoothMoveServo(CH_BASE, 175, 8);
  delay(150);
  // 2. Extend over right bin
  smoothMoveServo(CH_SHOULDER, 65, 8);
  smoothMoveServo(CH_ELBOW, 75, 8);
  smoothMoveServo(CH_WRIST_PITCH, 80, 8);
  delay(250);
  // 3. Open Claw to drop
  smoothMoveServo(CH_GRIPPER, 30, 8);
  delay(300);
  // 4. Retract arm
  smoothMoveServo(CH_SHOULDER, 105, 8);
  smoothMoveServo(CH_ELBOW, 95, 8);
  smoothMoveServo(CH_WRIST_PITCH, 90, 8);
  delay(150);
  // 5. Return Base M1 to Center (90 deg)
  smoothMoveServo(CH_BASE, 90, 8);
  delay(150);
}

void dropIntoLeftNonDegradableBin() {
  // 1. Rotate Base M1 to Left Bin (5 deg)
  smoothMoveServo(CH_BASE, 5, 8);
  delay(150);
  // 2. Extend over left bin
  smoothMoveServo(CH_SHOULDER, 65, 8);
  smoothMoveServo(CH_ELBOW, 75, 8);
  smoothMoveServo(CH_WRIST_PITCH, 80, 8);
  delay(250);
  // 3. Open Claw to drop
  smoothMoveServo(CH_GRIPPER, 30, 8);
  delay(300);
  // 4. Retract arm
  smoothMoveServo(CH_SHOULDER, 105, 8);
  smoothMoveServo(CH_ELBOW, 95, 8);
  smoothMoveServo(CH_WRIST_PITCH, 90, 8);
  delay(150);
  // 5. Return Base M1 to Center (90 deg)
  smoothMoveServo(CH_BASE, 90, 8);
  delay(150);
}

void executeSmartTargetPickSequence(String wasteClass, String offsetDir, int offsetMs) {
  if (!ensurePcaReady()) return;
  stopMotors();
  bool isDeg = isDegradableWaste(wasteClass);

  // Calculate Base M1 aim angle in degrees (Center is 90 deg)
  int baseAimAngle = 90;
  if (offsetDir == "LEFT") {
    baseAimAngle = 65; // Aim left
  } else if (offsetDir == "RIGHT") {
    baseAimAngle = 115; // Aim right
  }

  // 1. Aim Base M1 towards the detected waste
  smoothMoveServo(CH_BASE, baseAimAngle, 8);
  smoothMoveServo(CH_GRIPPER, 30, 6); // Claw open
  smoothMoveServo(CH_WRIST_ROLL, 90, 6);
  delay(100);

  // 2. Reach down to ground pick level
  smoothMoveServo(CH_SHOULDER, 50, 10);
  smoothMoveServo(CH_ELBOW, 60, 10);
  smoothMoveServo(CH_WRIST_PITCH, 70, 8);
  delay(300);

  // 3. Clamp Gripper firmly onto waste
  smoothMoveServo(CH_GRIPPER, 105, 8);
  delay(350);

  // 4. Lift arm up with waste
  smoothMoveServo(CH_SHOULDER, 110, 10);
  smoothMoveServo(CH_ELBOW, 100, 10);
  smoothMoveServo(CH_WRIST_PITCH, 90, 8);
  delay(200);

  // 5. Deposit into designated bin
  if (isDeg) {
    dropIntoRightDegradableBin();
  } else {
    dropIntoLeftNonDegradableBin();
  }

  // 6. Return Home
  armHome();
}

// ------------------------------------------------------------------------------
// 5. COMMAND EXECUTION DISPATCHER
// ------------------------------------------------------------------------------
void executeCommand(String cmd) {
  // ── Unified Movement Commands (Single-key + Web UI CMD: prefix) ────────────
  if (cmd == "W" || cmd == "w" || cmd == "CMD:FORWARD") {
    moveForward();
    sendResponse("ACK:FORWARD");
    return;
  } else if (cmd == "S" || cmd == "s" || cmd == "CMD:BACKWARD" || cmd == "CMD:REVERSE") {
    moveBackward();
    sendResponse("ACK:BACKWARD");
    return;
  } else if (cmd == "A" || cmd == "a" || cmd == "CMD:LEFT") {
    steerLeft();
    sendResponse("ACK:LEFT");
    return;
  } else if (cmd == "D" || cmd == "d" || cmd == "CMD:RIGHT") {
    steerRight();
    sendResponse("ACK:RIGHT");
    return;
  } else if (cmd == "Q" || cmd == "q" || cmd == "CMD:FORWARD_LEFT") {
    forwardCurveLeft();
    sendResponse("ACK:FWDL");
    return;
  } else if (cmd == "E" || cmd == "e" || cmd == "CMD:FORWARD_RIGHT") {
    forwardCurveRight();
    sendResponse("ACK:FWDR");
    return;
  } else if (cmd == "Z" || cmd == "z" || cmd == "CMD:BACK_LEFT" || cmd == "CMD:BACKWARD_LEFT" || cmd == "CMD:REVERSE_LEFT") {
    backCurveLeft();
    sendResponse("ACK:BACKL");
    return;
  } else if (cmd == "C" || cmd == "c" || cmd == "CMD:BACK_RIGHT" || cmd == "CMD:BACKWARD_RIGHT" || cmd == "CMD:REVERSE_RIGHT") {
    backCurveRight();
    sendResponse("ACK:BACKR");
    return;
  } else if (cmd == "X" || cmd == "x" || cmd == " " || cmd == "CMD:STOP") {
    stopMotors();
    sendResponse("ACK:STOP");
    return;

  // Diagnostics & Tests
  } else if (cmd == "CMD:TEST:LEFT") {
    stopMotors(); setLeftDrive(motorSpeed); delay(1200); stopMotors();
    Serial.println("✅ [DIAGNOSTIC] Left Motor Test Complete");
  } else if (cmd == "CMD:TEST:RIGHT") {
    stopMotors(); setRightDrive(motorSpeed); delay(1200); stopMotors();
    Serial.println("✅ [DIAGNOSTIC] Right Motor Test Complete");
  } else if (cmd == "CMD:TEST:PIN25") {
    stopMotors(); digitalWrite(pinENA, HIGH); digitalWrite(pinIN1, HIGH); delay(1200); stopMotors();
    Serial.println("✅ [TEST] GPIO 25 (IN1) + ENA Driven HIGH 1.2s");
  } else if (cmd == "CMD:TEST:PIN26") {
    stopMotors(); digitalWrite(pinENA, HIGH); digitalWrite(pinIN2, HIGH); delay(1200); stopMotors();
    Serial.println("✅ [TEST] GPIO 26 (IN2) + ENA Driven HIGH 1.2s");
  } else if (cmd == "CMD:TEST:PIN32") {
    stopMotors(); digitalWrite(pinENB, HIGH); digitalWrite(pinIN3, HIGH); delay(1200); stopMotors();
    Serial.println("✅ [TEST] GPIO 32 (IN3) + ENB Driven HIGH 1.2s");
  } else if (cmd == "CMD:TEST:PIN33") {
    stopMotors(); digitalWrite(pinENB, HIGH); digitalWrite(pinIN4, HIGH); delay(1200); stopMotors();
    Serial.println("✅ [TEST] GPIO 33 (IN4) + ENB Driven HIGH 1.2s");

  // Motor Inversion Flags
  } else if (cmd == "CMD:INVERT:LEFT") {
    invertLeftDrive = !invertLeftDrive;
    Serial.printf("⚡ [CONFIG] Invert Left Motor: %s\n", invertLeftDrive ? "ON" : "OFF");
    sendResponse(invertLeftDrive ? "CONFIG:INVERT_LEFT_ON" : "CONFIG:INVERT_LEFT_OFF");
  } else if (cmd == "CMD:INVERT:RIGHT") {
    invertRightDrive = !invertRightDrive;
    Serial.printf("⚡ [CONFIG] Invert Right Motor: %s\n", invertRightDrive ? "ON" : "OFF");
    sendResponse(invertRightDrive ? "CONFIG:INVERT_RIGHT_ON" : "CONFIG:INVERT_RIGHT_OFF");

  // Auto-Brake Safety Toggle
  } else if (cmd == "CMD:AUTOBRAKE:OFF") {
    autoBrakeEnabled = false;
    Serial.println("⚡ [CONFIG] Auto-Brake DISABLED");
    sendResponse("CONFIG:AUTOBRAKE_OFF");
  } else if (cmd == "CMD:AUTOBRAKE:ON") {
    autoBrakeEnabled = true;
    Serial.println("⚡ [CONFIG] Auto-Brake ENABLED");
    sendResponse("CONFIG:AUTOBRAKE_ON");

  // Speed Control
  } else if (cmd.startsWith("CMD:SPEED:") || cmd.startsWith("CMD:THROTTLE:")) {
    int val = cmd.startsWith("CMD:SPEED:") ? cmd.substring(10).toInt() : cmd.substring(13).toInt();
    if (val <= 0) {
      motorSpeed = 0;
    } else {
      val = constrain(val, 1, 100);
      motorSpeed = map(val, 1, 100, 130, 255);
    }
    if (isMotorsRunning) {
      if (isDrivingForward) moveForward();
      else moveBackward();
    }
    Serial.printf("⚡ [THROTTLE] Input=%d%% -> motorSpeed=%d/255\n", val, motorSpeed);

  // Arm Commands
  } else if (cmd == "ARM:HOME") {
    armHome();
  } else if (cmd.startsWith("ARM:PICK_TARGET:")) {
    String rest = cmd.substring(16);
    int c1 = rest.indexOf(':');
    if (c1 > 0) {
      String item = rest.substring(0, c1);
      String remain = rest.substring(c1 + 1);
      int c2 = remain.indexOf(':');
      if (c2 > 0) {
        String dir = remain.substring(0, c2);
        int ms = remain.substring(c2 + 1).toInt();
        executeSmartTargetPickSequence(item, dir, ms);
      } else {
        executeSmartTargetPickSequence(item, "CENTER", 0);
      }
    } else {
      executeSmartTargetPickSequence(rest, "CENTER", 0);
    }
  } else if (cmd.startsWith("ARM:PICK:")) {
    executeSmartTargetPickSequence(cmd.substring(9), "CENTER", 0);
  } else if (cmd == "ARM:BIN:RIGHT" || cmd == "ARM:BIN:DEGRADABLE") {
    dropIntoRightDegradableBin();
    armHome();
  } else if (cmd == "ARM:BIN:LEFT" || cmd == "ARM:BIN:NON_DEGRADABLE") {
    dropIntoLeftNonDegradableBin();
    armHome();
  } else if (cmd == "ARM:CLAW:OPEN" || cmd == "ARM:GRIPPER:OPEN") {
    smoothMoveServo(CH_GRIPPER, 30, 6);
  } else if (cmd == "ARM:CLAW:CLOSE" || cmd == "ARM:GRIPPER:CLOSE") {
    smoothMoveServo(CH_GRIPPER, 105, 6);
  } else if (cmd.startsWith("ARM:SERVO:") || cmd.startsWith("ARM:JOINT:")) {
    int prefixLen = 10;
    int firstColon = cmd.indexOf(':', prefixLen);
    if (firstColon > 0) {
      int ch = cmd.substring(prefixLen, firstColon).toInt();
      int val = cmd.substring(firstColon + 1).toInt();
      val = constrain(val, 0, 180);
      setServoAngle(ch, val);
      Serial.printf("🦾 [SERVO] Ch %d (M%d) -> %d deg\n", ch, ch + 1, val);
      sendResponse("ACK:SERVO:" + String(ch) + ":" + String(val));
    }
  } else if (cmd.startsWith("ARM:BASE:")) {
    String param = cmd.substring(9);
    int deg = 90;
    if (param == "LEFT" || param.startsWith("LEFT") || param == "CCW") {
      deg = constrain(currentAngles[CH_BASE] - 15, 0, 180);
    } else if (param == "RIGHT" || param.startsWith("RIGHT") || param == "CW") {
      deg = constrain(currentAngles[CH_BASE] + 15, 0, 180);
    } else if (param == "CENTER" || param == "STOP") {
      deg = 90;
    } else {
      deg = constrain(param.toInt(), 0, 180);
    }
    setServoAngle(CH_BASE, deg);
    Serial.printf("🦾 [BASE M1] -> %d deg\n", deg);
    sendResponse("ACK:BASE:" + String(deg));

  // Telemetry & Ping
  } else if (cmd == "CMD:TELEMETRY" || cmd == "CMD:GET_SENSORS") {
    sendTelemetry();
  } else if (cmd == "CMD:PING" || cmd == "PING") {
    String pong = "TELEM:PONG,IP=" + WiFi.localIP().toString() + ",OBST=" + String(obstacleNear ? 1 : 0);
    sendResponse(pong);
  } else if (cmd == "CMD:I2C_SCAN") {
    Wire.setTimeOut(50);
    Wire.beginTransmission(0x40);
    byte err = Wire.endTransmission();
    if (err == 0) {
      pcaReady = true;
      pca.begin();
      pca.setPWMFreq(SERVO_FREQ);
      sendResponse("I2C_STATUS:PCA9685_FOUND_AT_0x40");
    } else {
      sendResponse("I2C_STATUS:FAILED_CODE_" + String(err));
    }
  }
}

// ------------------------------------------------------------------------------
// 6. SETUP
// ------------------------------------------------------------------------------
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout resets

  Serial.begin(115200);
  delay(500);
  Serial.println("\n========================================================");
  Serial.println(" Smart Waste Robot Controller - Clean Standard Firmware");
  Serial.println("========================================================");

  // Setup Motor Pins: IN1(25), IN2(26), IN3(32), IN4(33), ENA(27), ENB(13)
  pinMode(pinIN1, OUTPUT);
  pinMode(pinIN2, OUTPUT);
  pinMode(pinIN3, OUTPUT);
  pinMode(pinIN4, OUTPUT);
  pinMode(pinENA, OUTPUT);
  pinMode(pinENB, OUTPUT);

  stopMotors();

  // Setup Ultrasonic Sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Setup I2C for PCA9685
  Wire.setTimeOut(50);
  Wire.begin(21, 22, 100000);
  delay(100);
  ensurePcaReady();
  if (pcaReady) armHome();

  // Wi-Fi Setup
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.setTxPower(WIFI_POWER_15dBm);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname("ESP32-DEVKIT");

  Serial.printf("[WiFi] Connecting to '%s'...\n", ssid);
  WiFi.begin(ssid, password);

  int retries = 0;
  while ((WiFi.status() != WL_CONNECTED || WiFi.localIP().toString() == "0.0.0.0") && retries < 25) {
    delay(400);
    Serial.print(".");
    retries++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED && WiFi.localIP().toString() != "0.0.0.0") {
    Serial.printf("📶 WI-FI CONNECTED: %s (IP: %s)\n", ssid, WiFi.localIP().toString().c_str());
  } else {
    Serial.println("⚠️ Starting Fallback Access Point...");
    WiFi.softAPConfig(ap_IP, ap_gateway, ap_subnet);
    WiFi.softAP(ap_ssid, ap_password);
    Serial.printf("📶 FALLBACK AP: %s (IP: %s)\n", ap_ssid, WiFi.softAPIP().toString().c_str());
  }

  MDNS.begin("smartwasterobot");
  udp.begin(UDP_PORT);
  Serial.printf("[UDP] Listening on port %d\n", UDP_PORT);

  sendResponse("ROBOT_ONLINE:DEVKIT");
}

// ------------------------------------------------------------------------------
// 7. MAIN LOOP
// ------------------------------------------------------------------------------
void loop() {
  // Rapid 100ms auto-brake check when moving forward
  if (isDrivingForward && autoBrakeEnabled) {
    static unsigned long lastBrakeCheck = 0;
    if (millis() - lastBrakeCheck > 100) {
      lastBrakeCheck = millis();
      float d = readUltrasonicDistanceCm();
      if (d >= 3.0f && d <= OBSTACLE_STOP_DISTANCE_CM) {
        Serial.printf("🚨 [AUTO-BRAKE] Obstacle at %.1f cm! Stopped.\n", d);
        stopMotors();
      }
    }
  }

  // Periodic telemetry
  if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryTime = millis();
    sendTelemetry();
  }

  // Wi-Fi auto-reconnect
  if (WiFi.status() != WL_CONNECTED) {
    static unsigned long lastReconn = 0;
    if (millis() - lastReconn > 5000) {
      lastReconn = millis();
      WiFi.reconnect();
    }
  }

  // Receive UDP packets
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) {
      incomingPacket[len] = '\0';
      String cmd = String(incomingPacket);
      cmd.trim();
      if (cmd.length() > 0) {
        lastSenderIP = udp.remoteIP();
        lastSenderPort = udp.remotePort();
        hasRemoteSender = true;
        lastPacketTime = millis();
        executeCommand(cmd);
      }
    }
  }

  // Receive USB Serial commands
  if (Serial.available()) {
    String usbCmd = Serial.readStringUntil('\n');
    usbCmd.trim();
    if (usbCmd.length() > 0) {
      lastPacketTime = millis();
      executeCommand(usbCmd);
    }
  }

  // Safety Watchdog: stop motors if no command in 5 seconds
  if (isMotorsRunning && lastPacketTime > 0 && (millis() - lastPacketTime > SAFETY_TIMEOUT_MS)) {
    stopMotors();
    isMotorsRunning = false;
    Serial.println("⚠️ [SAFETY] 5s timeout — All motors stopped.");
  }
}
