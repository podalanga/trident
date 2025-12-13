/*
 * Calibrated Line Follower Firmware
 * * Updates:
 * - Added Calibration Arrays (Min/Max) based on your data
 * - Added "Normalization" step (map all sensors to 0-1000)
 * - Set initial safe speeds/PID values
 */

// ============= PIN DEFINITIONS =============
const int IR_PINS[6] = {A0, A1, A2, A3, A4, A5};

// Motor Pins
// NOTE: On Arduino Uno, Pins 7 & 8 are NOT PWM. 
// If using Uno, move Left Motor to pins 3, 9, 10, or 11 for speed control.
const int MOTOR_RIGHT_AIN1 = 5; 
const int MOTOR_RIGHT_AIN2 = 6;
const int MOTOR_LEFT_BIN1 = 7;   
const int MOTOR_LEFT_BIN2 = 8;

// ============= CALIBRATION DATA =============
// Values from your calibration test
const int CAL_MIN[6] = {35,33,35,35,34,32};
const int CAL_MAX[6] = {308,338,792,557,760,806};

// ============= PID CONSTANTS =============
// Start with these conservative values
float Kp = 20.0;   // Controls how sharply it turns (Start: 20-30)
float Ki = 0.0;    // Usually 0 for simple line following
float Kd = 0.0;    // Dampens oscillation (Start: 5-15)

int baseSpeed = 100;    // Safe starting speed (0-255)
int maxSpeed = 255;    // Cap speed to prevent runaways

// ============= GLOBAL VARIABLES =============
int irValues[6];       // Holds normalized (0-1000) values
float previousError = 0;
float integral = 0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  
  for (int i = 0; i < 6; i++) pinMode(IR_PINS[i], INPUT);
  
  pinMode(MOTOR_LEFT_BIN1, OUTPUT);
  pinMode(MOTOR_LEFT_BIN2, OUTPUT);
  pinMode(MOTOR_RIGHT_AIN1, OUTPUT);    
  pinMode(MOTOR_RIGHT_AIN2, OUTPUT);
  
  // Stop initially
  stopMotors();
  
  Serial.println("=== Calibrated Line Follower ===");
  delay(2000); // 2s delay to put robot on line
  lastTime = millis();
}

void loop() {
  // 1. Read & Normalize Sensors
  readCalibratedSensors();
  
  // 2. Calculate Position
  float error = calculateLineError();
  
  // 3. PID Calculations
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; 
  if(dt == 0) dt = 0.001; // Prevent divide by zero
  lastTime = currentTime;
  
  float pidOutput = computePID(error, dt);
  
  // 4. Motor Control
  int leftSpeed = baseSpeed + pidOutput;
  int rightSpeed = baseSpeed - pidOutput;
  
  leftSpeed = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);
  
  setMotorSpeed(leftSpeed, rightSpeed);
  
  // Debug output
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    printDebugInfo(error, pidOutput, leftSpeed, rightSpeed);
    lastPrint = millis();
  }
}

// ============= SENSOR FUNCTIONS =============
void readCalibratedSensors() {
  for (int i = 0; i < 6; i++) {
    int raw = analogRead(IR_PINS[i]);
    
    // Map raw reading to 0-1000 based on YOUR calibration
    // map(value, low, high, newLow, newHigh)
    int calibrated = map(raw, CAL_MIN[i], CAL_MAX[i], 0, 1000);
    
    // Constraint to keep it clean (in case of outliers)
    irValues[i] = constrain(calibrated, 0, 1000);
  }
}

float calculateLineError() {
  long weightedSum = 0;
  long totalValue = 0;
  
  // Standard Weighted Average
  for (int i = 0; i < 6; i++) {
    weightedSum += (long)irValues[i] * i * 1000; // Multiply by 1000 for precision
    totalValue += irValues[i];
  }
  
  // Dead reckoning: If we lose the line, keep turning in the direction of the last known error
  if (totalValue < 500) { 
    if (previousError > 0) return 3.0;  // Hard right
    if (previousError < 0) return -3.0; // Hard left
    return 0; 
  }
  
  // Result is 0 to 5000, Center is 2500
  long position = weightedSum / totalValue;
  
  // Map to -2.5 to +2.5 for easier PID math
  float error = (position - 2500) / 1000.0;
  
  return error;
}

// ============= PID CONTROLLER =============
float computePID(float error, float dt) {
  float P = Kp * error;
  
  integral += error * dt;
  integral = constrain(integral, -50, 50);
  float I = Ki * integral;
  
  float D = 0.0;
  if (dt > 0) D = Kd * (error - previousError) / dt;
  
  previousError = error;
  
  return P + I + D;
}

// ============= MOTOR CONTROL =============
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Left Motor
  if (leftSpeed >= 0) {
    analogWrite(MOTOR_LEFT_BIN1, leftSpeed);
    analogWrite(MOTOR_LEFT_BIN2, 0);
  } else {
    analogWrite(MOTOR_LEFT_BIN1, 0);
    analogWrite(MOTOR_LEFT_BIN2, abs(leftSpeed));
  }
  
  // Right Motor
  if (rightSpeed >= 0) {
    analogWrite(MOTOR_RIGHT_AIN1, rightSpeed);
    analogWrite(MOTOR_RIGHT_AIN2, 0);
  } else {
    analogWrite(MOTOR_RIGHT_AIN1, 0);
    analogWrite(MOTOR_RIGHT_AIN2, abs(rightSpeed));
  }
}

void stopMotors() {
  analogWrite(MOTOR_LEFT_BIN1, 0);
  analogWrite(MOTOR_LEFT_BIN2, 0);
  analogWrite(MOTOR_RIGHT_AIN1, 0);
  analogWrite(MOTOR_RIGHT_AIN2, 0);
}

void printDebugInfo(float error, float pid, int l, int r) {
  Serial.print("Err: "); Serial.print(error);
  Serial.print(" | PID: "); Serial.print(pid);
  Serial.print(" | L: "); Serial.print(l);
  Serial.print(" R: "); Serial.println(r);
}