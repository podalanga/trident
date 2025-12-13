/*
 * IR Sensor Array Calibration (Gain Report Edition)
 * * Instructions:
 * 1. Send 'w' -> Place on WHITE surface (Wait 10s)
 * 2. Send 'b' -> Move over BLACK line (Wait 10s) -> Automatically prints GAINS
 */

const int IR_PINS[6] = {A0, A1, A2, A3, A4, A5};
const int NUM_SENSORS = 6;
const unsigned long CALIBRATION_TIME = 10000; // 10 seconds

struct SensorData {
  int rawValue;
  int minValue;      // From White
  int maxValue;      // From Black
  int calibrated;    // 0-1000
};

SensorData sensors[NUM_SENSORS];

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(IR_PINS[i], INPUT);
    sensors[i].minValue = 1023; // Default high
    sensors[i].maxValue = 0;    // Default low
  }
  Serial.println("\n=== Ready ===");
  Serial.println("'w' = White Calib (10s)");
  Serial.println("'b' = Black Calib (10s) -> Shows Gains");
  delay(1000);
}

void loop() {
  // 1. Read and Map Data
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensors[i].rawValue = analogRead(IR_PINS[i]);
    int range = sensors[i].maxValue - sensors[i].minValue;
    if (range > 0) {
      sensors[i].calibrated = map(sensors[i].rawValue, sensors[i].minValue, sensors[i].maxValue, 0, 1000);
      sensors[i].calibrated = constrain(sensors[i].calibrated, 0, 1000);
    } else {
      sensors[i].calibrated = 0;
    }
  }

  // 2. Print Live Data (so you know it's running)
  printLiveValues();

  // 3. Check Commands
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'w' || cmd == 'W') calibrateWhite();
    if (cmd == 'b' || cmd == 'B') calibrateBlack();
  }
  
  delay(100); 
}

// ================= FUNCTIONS =================

void calibrateWhite() {
  Serial.println("\n\n[STARTED] Calibrating WHITE (10s)...");
  
  // Reset Min Values
  for(int i=0; i<NUM_SENSORS; i++) sensors[i].minValue = 1023;

  unsigned long startTime = millis();
  while (millis() - startTime < CALIBRATION_TIME) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      int val = analogRead(IR_PINS[i]);
      if (val < sensors[i].minValue) sensors[i].minValue = val;
    }
  }
  Serial.println("[DONE] White values saved.\n");
}

void calibrateBlack() {
  Serial.println("\n\n[STARTED] Calibrating BLACK (10s)...");
  Serial.println(">> Move sensors over the line now <<");
  
  // Reset Max Values
  for(int i=0; i<NUM_SENSORS; i++) sensors[i].maxValue = 0;

  unsigned long startTime = millis();
  while (millis() - startTime < CALIBRATION_TIME) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      int val = analogRead(IR_PINS[i]);
      if (val > sensors[i].maxValue) sensors[i].maxValue = val;
    }
  }
  
  // STOP and Show Gains immediately
  printGainReport();
  
  // Short pause so you can read the report before loop restarts
  delay(3000); 
}

void printGainReport() {
  Serial.println("\n\n========== SENSOR GAINS ==========");
  Serial.println("ID\tMin\tMax\tGain(Range)");
  Serial.println("----------------------------------");
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    int gain = sensors[i].maxValue - sensors[i].minValue;
    
    Serial.print("S"); Serial.print(i); Serial.print("\t");
    Serial.print(sensors[i].minValue); Serial.print("\t");
    Serial.print(sensors[i].maxValue); Serial.print("\t");
    Serial.print(gain); 
    
    if(gain < 100) Serial.println("\t(BAD!)");
    else Serial.println("\t(OK)");
  }
  Serial.println("==================================\n");
}

void printLiveValues() {
  Serial.print("Vals: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensors[i].calibrated);
    Serial.print("\t");
  }
  Serial.println();
}