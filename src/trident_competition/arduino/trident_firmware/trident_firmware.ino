/*
 * Arduino Mega Firmware for Trident Competition Robot
 * 
 * Hardware:
 * - 6 Analog IR sensors (A0-A5)
 * - 2 N20 motors with PWM control
 * - Servo gripper
 * - Buzzer
 * 
 * Communication: JSON over Serial (115200 baud) with Raspberry Pi
 */

#include <ArduinoJson.h>
#include <Servo.h>

// ============= PIN DEFINITIONS =============
// IR Sensors (Analog)
const int IR_PINS[6] = {A0, A1, A2, A3, A4, A5};

// Motor Control (PWM on direction pins - no separate PWM)
// Right Motor
const int MOTOR_RIGHT_AIN1 = 5;    // PWM direction pin 1
const int MOTOR_RIGHT_AIN2 = 6;    // PWM direction pin 2

// Left Motor
const int MOTOR_LEFT_BIN1 = 7;     // PWM direction pin 1
const int MOTOR_LEFT_BIN2 = 8;     // PWM direction pin 2

// Gripper Servo
const int GRIPPER_PIN = 9;

// Buzzer
const int BUZZER_PIN = 3;

// ============= CONSTANTS =============
const int SERIAL_BAUD = 115200;
const int IR_SEND_RATE = 50;  // Hz
const int GRIPPER_CLOSE_ANGLE = 30;
const int GRIPPER_OPEN_ANGLE = 90;
const int BUZZER_DURATION = 300;  // ms
const int BUZZER_PAUSE = 500;     // ms

// ============= CALIBRATION DATA =============
// Values from calibration test
const int CAL_MIN[6] = {44, 42, 44, 43, 43, 41};
const int CAL_MAX[6] = {335, 314, 439, 394, 374, 237};

// ============= GLOBAL VARIABLES =============
Servo gripperServo;
unsigned long lastIRSendTime = 0;
int irSensorValues[6] = {0};

// ============= SETUP =============
void setup() {
  // Initialize serial
  Serial.begin(SERIAL_BAUD);
  
  // Initialize IR sensors
  for (int i = 0; i < 6; i++) {
    pinMode(IR_PINS[i], INPUT);
  }
  
  // Initialize motor pins
  pinMode(MOTOR_RIGHT_AIN1, OUTPUT);
  pinMode(MOTOR_RIGHT_AIN2, OUTPUT);
  pinMode(MOTOR_LEFT_BIN1, OUTPUT);
  pinMode(MOTOR_LEFT_BIN2, OUTPUT);
  
  // Initialize gripper
  gripperServo.attach(GRIPPER_PIN);
  gripperServo.write(GRIPPER_OPEN_ANGLE);
  
  // Initialize buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Startup beep
  beep(1);
  
  Serial.println("{\"type\":\"status\",\"message\":\"Arduino initialized\"}");
}

// ============= MAIN LOOP =============
void loop() {
  // Read IR sensors and send periodically
  unsigned long currentTime = millis();
  if (currentTime - lastIRSendTime >= (1000 / IR_SEND_RATE)) {
    readAndSendIRSensors();
    lastIRSendTime = currentTime;
  }
  
  // Process incoming serial commands
  if (Serial.available() > 0) {
    processSerialCommand();
  }
}

// ============= IR SENSOR FUNCTIONS =============
void readAndSendIRSensors() {
  // Read all IR sensors
  for (int i = 0; i < 6; i++) {
    int raw = analogRead(IR_PINS[i]);
    // Map raw reading to 0-1000 based on calibration
    int calibrated = map(raw, CAL_MIN[i], CAL_MAX[i], 0, 1000);
    irSensorValues[i] = constrain(calibrated, 0, 1000);
  }
  
  // Send as JSON
  StaticJsonDocument<200> doc;
  doc["type"] = "ir_sensors";
  JsonArray values = doc.createNestedArray("values");
  for (int i = 0; i < 6; i++) {
    values.add(irSensorValues[i]);
  }
  
  serializeJson(doc, Serial);
  Serial.println();
}

// ============= SERIAL COMMUNICATION =============
void processSerialCommand() {
  String jsonString = Serial.readStringUntil('\n');
  
  // Parse JSON
  StaticJsonDocument<300> doc;
  DeserializationError error = deserializeJson(doc, jsonString);
  
  if (error) {
    sendError("JSON parse error");
    return;
  }
  
  // Get command type
  const char* type = doc["type"];
  
  if (strcmp(type, "motor") == 0) {
    handleMotorCommand(doc);
  }
  else if (strcmp(type, "gripper") == 0) {
    handleGripperCommand(doc);
  }
  else if (strcmp(type, "buzzer") == 0) {
    handleBuzzerCommand(doc);
  }
  else {
    sendError("Unknown command type");
  }
}

void sendError(const char* message) {
  StaticJsonDocument<100> doc;
  doc["type"] = "error";
  doc["message"] = message;
  serializeJson(doc, Serial);
  Serial.println();
}

// ============= MOTOR CONTROL =============
void handleMotorCommand(JsonDocument& doc) {
  int leftSpeed = doc["left_speed"] | 0;
  int rightSpeed = doc["right_speed"] | 0;
  const char* leftDir = doc["left_direction"] | "forward";
  const char* rightDir = doc["right_direction"] | "forward";
  
  // Set left motor
  setMotor(MOTOR_LEFT_BIN1, MOTOR_LEFT_BIN2,
           leftSpeed, strcmp(leftDir, "forward") == 0);
  
  // Set right motor
  setMotor(MOTOR_RIGHT_AIN1, MOTOR_RIGHT_AIN2,
           rightSpeed, strcmp(rightDir, "forward") == 0);
}

void setMotor(int ain1Pin, int ain2Pin, int speed, bool forward) {
  // Clamp speed to 0-255
  speed = constrain(speed, 0, 255);
  
  // Set direction and speed via PWM on direction pins
  if (forward) {
    analogWrite(ain1Pin, speed);
    analogWrite(ain2Pin, 0);
  } else {
    analogWrite(ain1Pin, 0);
    analogWrite(ain2Pin, speed);
  }
}

void stopMotors() {
  analogWrite(MOTOR_RIGHT_AIN1, 0);
  analogWrite(MOTOR_RIGHT_AIN2, 0);
  analogWrite(MOTOR_LEFT_BIN1, 0);
  analogWrite(MOTOR_LEFT_BIN2, 0);
}

// ============= GRIPPER CONTROL =============
void handleGripperCommand(JsonDocument& doc) {
  const char* action = doc["action"];
  
  if (strcmp(action, "open") == 0) {
    gripperServo.write(GRIPPER_OPEN_ANGLE);
    sendGripperFeedback("open", GRIPPER_OPEN_ANGLE);
  }
  else if (strcmp(action, "close") == 0) {
    gripperServo.write(GRIPPER_CLOSE_ANGLE);
    sendGripperFeedback("close", GRIPPER_CLOSE_ANGLE);
  }
}

void sendGripperFeedback(const char* action, int position) {
  StaticJsonDocument<150> doc;
  doc["type"] = "gripper_feedback";
  doc["action"] = action;
  doc["position"] = position;
  
  serializeJson(doc, Serial);
  Serial.println();
}

// ============= BUZZER CONTROL =============
void handleBuzzerCommand(JsonDocument& doc) {
  const char* pattern = doc["pattern"];
  
  if (strcmp(pattern, "single") == 0) {
    beep(1);
  }
  else if (strcmp(pattern, "double") == 0) {
    beep(2);
  }
}

void beep(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(BUZZER_DURATION);
    digitalWrite(BUZZER_PIN, LOW);
    
    if (i < count - 1) {
      delay(BUZZER_PAUSE);
    }
  }
}
