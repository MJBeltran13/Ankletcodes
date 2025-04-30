#include <Arduino_BMI270_BMM150.h>
#include <Wire.h>
#include <ArduinoBLE.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define SENSOR_SERVICE_UUID "19B10000-E8F2-537E-4F6C-D104768A1214"
#define SENSOR_CHARACTERISTIC_UUID "19B10001-E8F2-537E-4F6C-D104768A1214"
BLEService sensorService(SENSOR_SERVICE_UUID);
BLECharacteristic sensorCharacteristic(SENSOR_CHARACTERISTIC_UUID, BLENotify, 20);

// Button pins
#define BUTTON_UP 2     // Increase values / Select
#define BUTTON_DOWN 3   // Decrease values 
#define BUTTON_MODE 4   // Mode switch / Confirm

// Button states
bool currentButtonUp = HIGH;     // Current debounced state
bool currentButtonDown = HIGH;
bool currentButtonMode = HIGH;
bool lastButtonUp = HIGH;        // Previous debounced state 
bool lastButtonDown = HIGH;
bool lastButtonMode = HIGH;
bool rawButtonUp = HIGH;         // Raw reading
bool rawButtonDown = HIGH;
bool rawButtonMode = HIGH;
unsigned long lastButtonUpTime = 0;    // Last time this button changed
unsigned long lastButtonDownTime = 0;
unsigned long lastButtonModeTime = 0;
unsigned long lastButtonActionTime = 0; // Last time an action was performed
const unsigned long BUTTON_DEBOUNCE_TIME = 100;  // Longer debounce time for stability
const unsigned long REPEAT_FIRST_TIME = 800;     // Longer first repeat delay
const unsigned long REPEAT_TIME = 300;           // Subsequent repeat time

// Device states
enum DeviceState {
  STATE_WEIGHT_SETUP,
  STATE_DURATION_SETUP,
  STATE_READY,
  STATE_TRACKING,
  STATE_RESULTS
};
DeviceState currentState = STATE_WEIGHT_SETUP;

const float STEP_THRESHOLD = 3.0;
const float STRIDE_LENGTH = 0.7;
const int STEP_DEBOUNCE_TIME = 300;      // Renamed to avoid conflict
const float STEPPING_THRESHOLD = 3.3;
const float RUNNING_THRESHOLD = 3.9;
const float VELOCITY_DECAY = 0.95;
const float ACCEL_BIAS = 0.01;

unsigned long lastStepTime = 0;
unsigned long measurementStartTime = 0;
unsigned long lastSampleTime = 0;
unsigned long userDurationMillis = 60000;
unsigned long lastDisplayUpdateTime = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 500;  // Update display every 500ms

int stepCount = 0;
bool wasInStep = false;
bool measurementStarted = false;
bool measurementComplete = false;
bool bleConnected = false;

float velocityX = 0, velocityY = 0;
float displacementX = 0, displacementY = 0;
float totalDisplacement = 0;

int userWeight = 60;
int userDurationMinutes = 1;

void setup() {
  Serial.begin(115200);
  
  // Setup button pins
  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  pinMode(BUTTON_MODE, INPUT_PULLUP);
  
  // Initialize display first for status updates
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    while (1);
  }
  
  displayStatus("Starting...", "Initializing sensors");
  
  // Initialize IMU
  if (!IMU.begin()) {
    displayStatus("ERROR", "IMU initialization failed");
    while (1);
  }
  
  displayStatus("Starting...", "Initializing BLE");
  
  // Initialize BLE
  if (!BLE.begin()) {
    displayStatus("ERROR", "BLE initialization failed");
    while (1);
  }

  // Configure BLE
  BLE.setLocalName("SensorData");
  BLE.setAdvertisedService(sensorService);
  sensorService.addCharacteristic(sensorCharacteristic);
  BLE.addService(sensorService);
  
  displayStatus("Advertising...", "Waiting for connection");
  BLE.advertise();

  // Start with weight setup
  displayWeightSetup();
}

// Proper button debouncing and handling
void checkButtons() {
  unsigned long currentMillis = millis();
  
  // Read current button states
  bool readingUp = digitalRead(BUTTON_UP);
  bool readingDown = digitalRead(BUTTON_DOWN);
  bool readingMode = digitalRead(BUTTON_MODE);
  
  // Store current debounced button states before updating
  lastButtonUp = currentButtonUp;
  lastButtonDown = currentButtonDown; 
  lastButtonMode = currentButtonMode;
  
  // UP button debouncing
  if (readingUp != rawButtonUp) {
    lastButtonUpTime = currentMillis;
    rawButtonUp = readingUp;
  }
  
  if ((currentMillis - lastButtonUpTime) > BUTTON_DEBOUNCE_TIME) {
    currentButtonUp = rawButtonUp;
  }
  
  // DOWN button debouncing
  if (readingDown != rawButtonDown) {
    lastButtonDownTime = currentMillis;
    rawButtonDown = readingDown;
  }
  
  if ((currentMillis - lastButtonDownTime) > BUTTON_DEBOUNCE_TIME) {
    currentButtonDown = rawButtonDown;
  }
  
  // MODE button debouncing  
  if (readingMode != rawButtonMode) {
    lastButtonModeTime = currentMillis;
    rawButtonMode = readingMode;
  }
  
  if ((currentMillis - lastButtonModeTime) > BUTTON_DEBOUNCE_TIME) {
    currentButtonMode = rawButtonMode;
  }
  
  // Button press detection and handling (transition from HIGH to LOW)
  
  // UP button - Handle initial press and repeats
  if (currentButtonUp == LOW) {
    if (lastButtonUp == HIGH || 
        (currentMillis - lastButtonUpTime > REPEAT_FIRST_TIME && 
         currentMillis - lastButtonActionTime > REPEAT_TIME)) {
      // Either initial press or repeat time has elapsed
      handleUpButton();
      lastButtonActionTime = currentMillis;
    }
  }
  
  // DOWN button - Handle initial press and repeats
  if (currentButtonDown == LOW) {
    if (lastButtonDown == HIGH || 
        (currentMillis - lastButtonDownTime > REPEAT_FIRST_TIME && 
         currentMillis - lastButtonActionTime > REPEAT_TIME)) {
      // Either initial press or repeat time has elapsed
      handleDownButton();
      lastButtonActionTime = currentMillis;
    }
  }
  
  // MODE button - Only trigger on new press, no repeats
  if (currentButtonMode == LOW && lastButtonMode == HIGH) {
    handleModeButton();
  }
}

// Handle UP button press based on current state
void handleUpButton() {
  Serial.println("UP button pressed");  // Debug output
  
  switch (currentState) {
    case STATE_WEIGHT_SETUP:
      userWeight += 5;
      if (userWeight > 150) userWeight = 150; // Maximum weight
      displayWeightSetup();
      break;
      
    case STATE_DURATION_SETUP:
      userDurationMinutes += 1;
      if (userDurationMinutes > 60) userDurationMinutes = 60; // Maximum duration
      userDurationMillis = userDurationMinutes * 60000UL;
      displayDurationSetup();
      break;
      
    case STATE_RESULTS:
      // Reset and go back to ready state
      resetMeasurement();
      currentState = STATE_READY;
      displayStatus("Ready", "Waiting for movement");
      break;
  }
}

// Handle DOWN button press based on current state
void handleDownButton() {
  Serial.println("DOWN button pressed");  // Debug output
  
  switch (currentState) {
    case STATE_WEIGHT_SETUP:
      userWeight -= 5;
      if (userWeight < 30) userWeight = 30; // Minimum weight
      displayWeightSetup();
      break;
      
    case STATE_DURATION_SETUP:
      userDurationMinutes -= 1;
      if (userDurationMinutes < 1) userDurationMinutes = 1; // Minimum duration
      userDurationMillis = userDurationMinutes * 60000UL;
      displayDurationSetup();
      break;
  }
}

// Handle MODE button press based on current state
void handleModeButton() {
  Serial.println("MODE button pressed");  // Debug output
  
  switch (currentState) {
    case STATE_WEIGHT_SETUP:
      currentState = STATE_DURATION_SETUP;
      displayDurationSetup();
      break;
      
    case STATE_DURATION_SETUP:
      currentState = STATE_READY;
      displayStatus("Ready", "Waiting for movement");
      break;
      
    case STATE_READY:
      // Manual start (instead of auto-detection)
      if (bleConnected) {
        startMeasurement();
      }
      break;
      
    case STATE_TRACKING:
      // Force end measurement
      sendResults();
      break;
      
    case STATE_RESULTS:
      resetMeasurement();
      currentState = STATE_READY;
      displayStatus("Ready", "Waiting for movement");
      break;
  }
}

// Display weight setup screen
void displayWeightSetup() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Header
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("WEIGHT SETUP");
  display.drawLine(0, 10, display.width(), 10, SSD1306_WHITE);
  
  // Weight value (large)
  display.setTextSize(2);
  display.setCursor(35, 20);
  display.print(userWeight);
  display.println(" kg");
  
  // Visual weight slider
  int sliderWidth = display.width() - 20;
  int sliderPos = map(userWeight, 30, 150, 0, sliderWidth);
  display.drawRect(10, 40, sliderWidth, 8, SSD1306_WHITE);
  display.fillRect(10, 40, sliderPos, 8, SSD1306_WHITE);
  
 
  
  // Controls help
  display.drawLine(0, 52, display.width(), 52, SSD1306_WHITE);
  display.setCursor(0, 55);
  display.print("UP/DOWN");
  
  display.setCursor(90, 55);
  display.print("MODE: Next");
  
  display.display();
}

// Display duration setup screen
void displayDurationSetup() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Header
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("DURATION SETUP");
  display.drawLine(0, 10, display.width(), 10, SSD1306_WHITE);
  
  // Duration value (large)
  display.setTextSize(2);
  display.setCursor(35, 20);
  display.print(userDurationMinutes);
  display.println(" min");
  
  // Controls help
  display.setTextSize(1);
  display.setCursor(0, 45);
  display.println("UP/DOWN");
  display.setCursor(0, 55);
  display.println("SET");
  
  display.display();
}

// Helper function to display status messages
void displayStatus(String line1, String line2) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  
  // Display header
  display.setCursor(0, 0);
  display.println("ANKLET DEVICE");
  display.drawLine(0, 10, display.width(), 10, SSD1306_WHITE);
  
  // Display main status
  display.setCursor(0, 16);
  display.setTextSize(1);
  display.println(line1);
  
  // Display secondary status
  display.setCursor(0, 28);
  display.println(line2);
  
  // Display connection status at bottom
  display.drawLine(0, 52, display.width(), 52, SSD1306_WHITE);
  display.setCursor(0, 54);
  display.println(bleConnected ? "BLE: Connected" : "BLE: Advertising");
  
  display.display();
}

// Display live measurement data
void displayLiveData() {
  if (millis() - lastDisplayUpdateTime < DISPLAY_UPDATE_INTERVAL) return;
  
  lastDisplayUpdateTime = millis();
  
  unsigned long elapsedTime = millis() - measurementStartTime;
  unsigned long remainingTime = userDurationMillis > elapsedTime ? userDurationMillis - elapsedTime : 0;
  int remainingSeconds = remainingTime / 1000;
  
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Title
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("TRACKING ACTIVE");
  display.drawLine(0, 10, display.width(), 10, SSD1306_WHITE);
  
  // Steps and time
  display.setCursor(0, 16);
  display.print("Steps: ");
  display.println(stepCount);
  
  display.setCursor(0, 26);
  display.print("Time: ");
  display.print(remainingSeconds / 60);
  display.print(":");
  if (remainingSeconds % 60 < 10) display.print("0");
  display.print(remainingSeconds % 60);
  display.println(" left");
  
  // Movement type
  display.setCursor(0, 36);
  display.print("Mode: ");
  display.println(determineMovementType());
  
  // Progress bar
  int progressWidth = map(elapsedTime, 0, userDurationMillis, 0, display.width());
  display.drawRect(0, 52, display.width(), 10, SSD1306_WHITE);
  display.fillRect(0, 52, progressWidth, 10, SSD1306_WHITE);
  
  display.display();
}

// Start measurement manually
void startMeasurement() {
  measurementStarted = true;
  measurementStartTime = millis();
  measurementComplete = false;
  currentState = STATE_TRACKING;
  Serial.println("Measurement started manually...");
  displayStatus("Measurement", "Started!");
  delay(500); // Show start message briefly
}

// Reset measurement values
void resetMeasurement() {
  stepCount = 0;
  totalDisplacement = 0;
  displacementX = 0;
  displacementY = 0;
  velocityX = 0;
  velocityY = 0;
  measurementComplete = false;
  measurementStarted = false;
  lastSampleTime = 0;
}

float calculateSpeed(int steps, unsigned long timeMs) {
  float timeMinutes = timeMs / 60000.0;
  float stepsPerMinute = steps / timeMinutes;
  return (stepsPerMinute * STRIDE_LENGTH * 60.0) / 1000.0;
}

void updateDisplacement(float accelX, float accelY, unsigned long currentTime) {
  float dt = (currentTime - lastSampleTime) / 1000.0;
  if (lastSampleTime == 0) dt = 0;
  if (dt > 0 && dt < 0.1) {
    float filteredAccelX = accelX - ACCEL_BIAS;
    float filteredAccelY = accelY - ACCEL_BIAS;
    velocityX = (velocityX + filteredAccelX * dt) * VELOCITY_DECAY;
    velocityY = (velocityY + filteredAccelY * dt) * VELOCITY_DECAY;
    displacementX += velocityX * dt;
    displacementY += velocityY * dt;
    totalDisplacement = sqrt(displacementX * displacementX + displacementY * displacementY);
  }
  lastSampleTime = currentTime;
}

String determineMovementType() {
  if (totalDisplacement < STEPPING_THRESHOLD) return "jumping";
  else if (totalDisplacement > RUNNING_THRESHOLD) return "Running";
  else return "Walking";
}

float calculateMET(String type) {
  if (type == "Running") return 8.0;
  if (type == "Walking") return 3.5;
  return 2.3;
}

float calculateCalories(float MET, int weightKg, int durationMinutes) {
  return (MET * 3.5 * weightKg / 200.0) * durationMinutes;
}

void sendResults() {
  currentState = STATE_RESULTS;
  
  float speed = calculateSpeed(stepCount, userDurationMillis);
  String type = determineMovementType();
  float MET = calculateMET(type);
  float calories = calculateCalories(MET, userWeight, userDurationMinutes);

  String dataString = String(stepCount) + "," + String(speed, 2) + "," +
                      String(totalDisplacement, 2) + "," + type;
  sensorCharacteristic.writeValue(dataString.c_str());

  Serial.println("--- Final Results ---");
  Serial.println("Activity: " + type);

  if (type == "jumping") {
    Serial.println("Steps: " + String(stepCount));
    Serial.println("Calories: " + String(calories, 2) + " kcal");
  } else if (type == "Running") {
    Serial.println("Speed: " + String(speed, 2) + " km/h");
    Serial.println("Steps: " + String(stepCount));
    Serial.println("Calories: " + String(calories, 2) + " kcal");
  } else {
    Serial.println("Steps: " + String(stepCount));
    Serial.println("Speed: " + String(speed, 2) + " km/h");
    Serial.println("Displacement: " + String(totalDisplacement, 2) + " m");
    Serial.println("Calories: " + String(calories, 2) + " kcal");
  }
  Serial.println("---------------------");

  // Display final results
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  
  // Header
  display.setCursor(0, 0);
  display.println("WORKOUT COMPLETE");
  display.drawLine(0, 10, display.width(), 10, SSD1306_WHITE);
  
  // Results
  display.setCursor(0, 14);
  display.println("Activity: " + type);

  int yPos = 24;
  if (type == "jumping") {
    display.setCursor(0, yPos); display.print("Steps: "); display.println(stepCount); yPos += 10;
    display.setCursor(0, yPos); display.print("Cal: "); display.print(calories, 1); display.println(" kcal");
  } else if (type == "Running") {
    display.setCursor(0, yPos); display.print("Speed: "); display.print(speed, 1); display.println(" km/h"); yPos += 10;
    display.setCursor(0, yPos); display.print("Steps: "); display.println(stepCount); yPos += 10;
    display.setCursor(0, yPos); display.print("Cal: "); display.print(calories, 1); display.println(" kcal");
  } else {
    display.setCursor(0, yPos); display.print("Steps: "); display.println(stepCount); yPos += 10;
    display.setCursor(0, yPos); display.print("Speed: "); display.print(speed, 1); display.println(" km/h"); yPos += 10;
    display.setCursor(0, yPos); display.print("Dist: "); display.print(totalDisplacement, 1); display.println(" m"); yPos += 10;
    display.setCursor(0, yPos); display.print("Cal: "); display.print(calories, 1); display.println(" kcal");
  }
  
  // Footer
  display.setCursor(0, 54);
  display.println("MODE: New Session");
  
  display.display();
}

void loop() {
  // Check for button inputs
  checkButtons();
  
  // Handle BLE
  BLEDevice central = BLE.central();
  
  if (central) {
    if (!bleConnected) {
      bleConnected = true;
      displayStatus("Connected!", "Ready to start");
    }
    
    while (central.connected()) {
      // Continue checking for button inputs while connected
      checkButtons();
      
      float accelX, accelY, accelZ;
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(accelX, accelY, accelZ);
        float magnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
        unsigned long currentTime = millis();

        // Auto-start detection (if in ready state)
        if (currentState == STATE_READY && !measurementStarted && magnitude > STEP_THRESHOLD) {
          startMeasurement();
        }

        if (currentState == STATE_TRACKING && measurementStarted && !measurementComplete) {
          updateDisplacement(accelX, accelY, currentTime);
          
          // Update live display
          displayLiveData();
          
          if (currentTime - measurementStartTime >= userDurationMillis) {
            sendResults();
            continue;
          }

          if (currentTime - lastStepTime >= STEP_DEBOUNCE_TIME) {
            if (magnitude > STEP_THRESHOLD && !wasInStep) {
              wasInStep = true;
            } else if (magnitude < (STEP_THRESHOLD * 0.8) && wasInStep) {
              wasInStep = false;
              stepCount++;
              lastStepTime = currentTime;
            }
          }
        }
      }
      delay(10);
    }
    
    // Handle disconnect
    bleConnected = false;
    displayStatus("Disconnected", "Waiting for connection");
  }
}
