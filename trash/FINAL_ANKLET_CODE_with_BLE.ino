#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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
  STATE_CALIBRATION,  // New state for calibration
  STATE_READY,
  STATE_TRACKING,
  STATE_RESULTS
};
DeviceState currentState = STATE_WEIGHT_SETUP;

// Movement detection thresholds
const float STEP_THRESHOLD = 2.0;        // Minimum acceleration to detect a step (in m/s²)
const float STRIDE_LENGTH = 0.7;         // Average stride length in meters
const int STEP_DEBOUNCE_TIME = 400;      // Increased debounce time for better step detection
const float STEPPING_THRESHOLD = 1.0;    // Lowered threshold for walking detection
const float RUNNING_THRESHOLD = 3.0;     // Increased threshold for running detection
const float VELOCITY_DECAY = 0.95;       // Adjusted for more stable readings
const float ACCEL_BIAS = 0.001;          // Small offset to correct accelerometer bias (in m/s²)

// Constants for improved jumping detection
const float VERTICAL_JUMP_THRESHOLD = 2.5;      // Lowered threshold for easier jump detection
const float HORIZONTAL_LIMIT_FOR_JUMP = 2.0;    // Increased to allow more natural movement during jumps
const float DISPLACEMENT_DECAY_RATE = 0.94;     // Faster decay for more accurate readings
const float JUMPING_DISPLACEMENT_LIMIT = 5.0;   // Adjusted limit for jump detection
const unsigned long JUMP_PATTERN_TIMEOUT = 2000; // Increased timeout for better pattern detection
const unsigned long JUMP_RESET_INTERVAL = 3000;  // Increased reset interval
const unsigned long JUMP_DEBOUNCE_TIME = 200;   // Reduced debounce time for more responsive detection

// Constants for jump detection
const float JUMP_START_THRESHOLD = 2.5;    // Lowered threshold for more reliable detection
const float JUMP_LAND_THRESHOLD = 1;     // Added specific landing threshold
const float JUMP_MIN_TIME = 100;            // Increased minimum time to avoid false positives
const float JUMP_MAX_TIME = 800;          // Decreased maximum time for better accuracy

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

float velocityX = 0, velocityY = 0;
float displacementX = 0, displacementY = 0;
float totalDisplacement = 0;

int userWeight = 60;
int userDurationMinutes = 1;

// Add jump detection tracking
bool hasJumpingPattern = false;
float verticalAccelMax = 0;
unsigned long jumpPatternDuration = 0;
unsigned long lastJumpDetectTime = 0;
unsigned long consecutiveJumpingTime = 0;

// Calibration variables
float calibrationX = 0, calibrationY = 0, calibrationZ = 0;
float gyroCalibX = 0, gyroCalibY = 0, gyroCalibZ = 0;
const int CALIBRATION_SAMPLES = 50;
int calibrationCount = 0;
float calibrationSumX = 0, calibrationSumY = 0, calibrationSumZ = 0;
float gyroSumX = 0, gyroSumY = 0, gyroSumZ = 0;
bool calibrationComplete = false;

// Orientation tracking
enum DeviceOrientation {
  ORIENTATION_NORMAL_Z_UP,
  ORIENTATION_UPSIDE_DOWN,
  ORIENTATION_HORIZONTAL_X_POS,
  ORIENTATION_HORIZONTAL_X_NEG,
  ORIENTATION_VERTICAL_Y_POS,
  ORIENTATION_VERTICAL_Y_NEG,
  ORIENTATION_TILTED
};
DeviceOrientation currentOrientation = ORIENTATION_TILTED;
float gravityVector[3] = {0, 0, 0}; // Stores primary gravity direction

// Add these variables for better jump state management
unsigned long lastJumpTime = 0;
bool isInJumpState = false;
int consecutiveJumps = 0;

// Jump state tracking
bool inJumpTakeoff = false;
bool inJumpLanding = false;
unsigned long jumpStartTime = 0;
float maxJumpForce = 0;

// Add jump counter variable at the top with other counters
int jumpCount = 0;

// BLE Service and Characteristics UUIDs - must match Python code
BLEService sensorService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEUnsignedIntCharacteristic stepCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEUnsignedIntCharacteristic jumpCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

// Function to determine device orientation from calibration data
DeviceOrientation determineOrientation(float accX, float accY, float accZ) {
  // Find the axis with the strongest gravity component
  float absX = abs(accX);
  float absY = abs(accY);
  float absZ = abs(accZ);
  
  const float GRAVITY_THRESHOLD = 0.8; // Minimum gravity component to determine orientation
  
  // Z-axis orientations
  if (absZ > absX && absZ > absY && absZ > GRAVITY_THRESHOLD) {
    gravityVector[0] = 0;
    gravityVector[1] = 0;
    gravityVector[2] = (accZ > 0) ? 1.0 : -1.0;
    return (accZ > 0) ? ORIENTATION_NORMAL_Z_UP : ORIENTATION_UPSIDE_DOWN;
  } 
  // X-axis orientations
  else if (absX > absY && absX > absZ && absX > GRAVITY_THRESHOLD) {
    gravityVector[0] = (accX > 0) ? 1.0 : -1.0;
    gravityVector[1] = 0;
    gravityVector[2] = 0;
    return (accX > 0) ? ORIENTATION_HORIZONTAL_X_POS : ORIENTATION_HORIZONTAL_X_NEG;
  } 
  // Y-axis orientations
  else if (absY > absX && absY > absZ && absY > GRAVITY_THRESHOLD) {
    gravityVector[0] = 0;
    gravityVector[1] = (accY > 0) ? 1.0 : -1.0;
    gravityVector[2] = 0;
    return (accY > 0) ? ORIENTATION_VERTICAL_Y_POS : ORIENTATION_VERTICAL_Y_NEG;
  }
  
  // If no clear orientation detected, it's tilted
  // Calculate an approximate gravity vector for tilted orientation
  float magnitude = sqrt(accX*accX + accY*accY + accZ*accZ);
  if (magnitude > 0.1) {
    gravityVector[0] = accX / magnitude;
    gravityVector[1] = accY / magnitude;
    gravityVector[2] = accZ / magnitude;
  }
  
  return ORIENTATION_TILTED;
}

void setup() {
  Serial.begin(115200);
  
  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }

  // Set BLE name and advertised service
  BLE.setLocalName("SensorData");
  BLE.setAdvertisedService(sensorService);
  
  // Add characteristics to the service
  sensorService.addCharacteristic(stepCharacteristic);
  sensorService.addCharacteristic(jumpCharacteristic);
  
  // Add the service
  BLE.addService(sensorService);
  
  // Set initial values
  stepCharacteristic.writeValue(0);
  jumpCharacteristic.writeValue(0);
  
  // Start advertising
  BLE.advertise();
  Serial.println("BLE initialized and advertising!");
  
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
  
  // Start with weight setup
  displayWeightSetup();
  
  Serial.println("Anklet Ready");
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
      userWeight += 1;
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
      displayStatus("Ready", "Press MODE to start");
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
      currentState = STATE_CALIBRATION;  // Go to calibration instead of ready
      calibrationCount = 0;  // Reset calibration
      calibrationSumX = 0;
      calibrationSumY = 0;
      calibrationSumZ = 0;
      gyroSumX = 0;
      gyroSumY = 0;
      gyroSumZ = 0;
      calibrationComplete = false;
      displayCalibration();
      break;
      
    case STATE_CALIBRATION:
      if (calibrationComplete) {
        currentState = STATE_READY;
        displayStatus("Ready", "Press MODE to start");
      }
      break;
      
    case STATE_READY:
      startMeasurement();
      break;
      
    case STATE_TRACKING:
      sendResults();
      break;
      
    case STATE_RESULTS:
      resetMeasurement();
      currentState = STATE_READY;
      displayStatus("Ready", "Press MODE to start");
      break;
  }
}

// Display weight setup screen
void displayWeightSetup() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Header
  display.setTextSize(2);
  display.setCursor(25, 5);
  display.print("WEIGHT");
  
  // Weight value
  display.setTextSize(3);
  display.setCursor(35, 25);
  display.print(userWeight);
  
  // Unit and controls
  display.setTextSize(1);
  display.setCursor(100, 32);
  display.print("kg");
  display.setCursor(0, 56);
  display.print("UP/DN:Change  MODE:Next");
  
  display.display();
}

// Display duration setup screen
void displayDurationSetup() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Header
  display.setTextSize(2);
  display.setCursor(15, 5);
  display.print("DURATION");
  
  // Duration value
  display.setTextSize(3);
  display.setCursor(35, 25);
  display.print(userDurationMinutes);
  
  // Unit and controls
  display.setTextSize(1);
  display.setCursor(100, 32);
  display.print("m");
  display.setCursor(0, 56);
  display.print("UP/DN:Change  MODE:Next");
  
  display.display();
}

// Display calibration screen
void displayCalibration() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Header
  display.setTextSize(1);
  display.setCursor(30, 5);
  display.print("CALIBRATING");
  
  if (!calibrationComplete) {
    // Progress bar
    display.drawRect(14, 30, 100, 8, SSD1306_WHITE);
    int progressWidth = map(calibrationCount, 0, CALIBRATION_SAMPLES, 0, 98);
    display.fillRect(15, 31, progressWidth, 6, SSD1306_WHITE);
    
    // Percentage
    display.setCursor(50, 45);
    int progress = (calibrationCount * 100) / CALIBRATION_SAMPLES;
    display.print(progress);
    display.print("%");
  } else {
    display.setCursor(20, 30);
    display.print("READY - Press MODE");
  }
  
  display.display();
}

// Display live measurement data
void displayLiveData() {
  if (millis() - lastDisplayUpdateTime < DISPLAY_UPDATE_INTERVAL) return;
  lastDisplayUpdateTime = millis();
  
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Time remaining
  unsigned long elapsedTime = millis() - measurementStartTime;
  unsigned long remainingTime = userDurationMillis > elapsedTime ? userDurationMillis - elapsedTime : 0;
  int remainingSeconds = remainingTime / 1000;
  int remainingMinutes = remainingSeconds / 60;
  remainingSeconds %= 60;
  
  // Activity type (large)
  display.setTextSize(2);
  String type = determineMovementType();
  int typeX = (128 - (type.length() * 12)) / 2;  // Center text (12 pixels per char at size 2)
  display.setCursor(typeX, 2);
  display.print(type);
  
  // Stats (medium size)
  display.setTextSize(2);
  
  // Steps/Jumps (depending on activity)
  if (type == "Jumping") {
    display.setCursor(5, 25);
    display.print(jumpCount);
    display.setTextSize(1);
    display.print(" JUMPS");
  } else {
    display.setCursor(5, 25);
    display.print(stepCount);
    display.setTextSize(1);
    display.print(" STEPS");
  }
  
  // Time remaining
  display.setTextSize(2);
  display.setCursor(25, 45);
  if (remainingMinutes < 10) display.print("0");
  display.print(remainingMinutes);
  display.print(":");
  if (remainingSeconds < 10) display.print("0");
  display.print(remainingSeconds);
  
  display.display();
}

// Display results screen
void displayResults(String type, String intensityLevel, float speed, float calories) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Activity type
  display.setTextSize(2);
  int typeX = (128 - (type.length() * 12)) / 2;
  display.setCursor(typeX, 0);
  display.print(type);
  
  // Stats
  display.setTextSize(1);
  
  if (type == "Jumping") {
    // Jumps
    display.setCursor(5, 20);
    display.print("Jumps: ");
    display.print(jumpCount);
    
    // Jump rate
    display.setCursor(5, 32);
    unsigned long elapsedTimeMinutes = (millis() - measurementStartTime) / 60000.0;
    if (elapsedTimeMinutes < 0.1) elapsedTimeMinutes = 0.1;
    float jumpsPerMinute = jumpCount / (float)elapsedTimeMinutes;
    display.print(jumpsPerMinute, 1);
    display.print("/min");
  } else {
    // Steps
    display.setCursor(5, 20);
    display.print("Steps: ");
    display.print(stepCount);
    
    // Speed
    display.setCursor(5, 32);
    display.print(speed, 1);
    display.print(" km/h");
  }
  
  // Intensity and calories
  display.setCursor(5, 44);
  display.print(intensityLevel);
  display.setCursor(5, 56);
  display.print(calories, 1);
  display.print(" kcal");
  
  display.display();
}

// Helper function to display status messages
void displayStatus(String line1, String line2) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // First line (larger)
  display.setTextSize(2);
  int x1 = (128 - (line1.length() * 12)) / 2;
  display.setCursor(x1, 10);
  display.print(line1);
  
  // Second line (smaller)
  display.setTextSize(1);
  int x2 = (128 - (line2.length() * 6)) / 2;
  display.setCursor(x2, 40);
  display.print(line2);
  
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
  jumpCount = 0;
  
  totalDisplacement = 0;
  displacementX = 0;
  displacementY = 0;
  velocityX = 0;
  velocityY = 0;
  measurementComplete = false;
  measurementStarted = false;
  lastSampleTime = 0;
  hasJumpingPattern = false;
  verticalAccelMax = 0;
  jumpPatternDuration = 0;
}

float calculateSpeed(int steps, unsigned long timeMs) {
  float timeMinutes = timeMs / 60000.0;
  float stepsPerMinute = steps / timeMinutes;
  return (stepsPerMinute * STRIDE_LENGTH * 60.0) / 1000.0;
}

// Function to detect and process jumps
bool processJumpDetection(float verticalAccel, unsigned long currentTime) {
    // Debug output
    if (abs(verticalAccel) > 1.0) {
        Serial.print("Vertical Accel: ");
        Serial.println(verticalAccel);
    }

    // Track maximum force during jump
    maxJumpForce = max(maxJumpForce, abs(verticalAccel));

    // Detect jump takeoff when vertical acceleration exceeds threshold
    if (!isInJumpState && verticalAccel > JUMP_START_THRESHOLD) {
        Serial.println("Jump takeoff detected!");
        Serial.print("Jump force: ");
        Serial.println(verticalAccel);
        
        isInJumpState = true;
        jumpStartTime = currentTime;
        return false; // Don't count the jump yet, wait for landing
    }
    
    // Detect landing when in jump state and acceleration goes negative
    if (isInJumpState && verticalAccel < JUMP_LAND_THRESHOLD) {
        Serial.println("Jump landing detected!");
        
        // Reset states
        isInJumpState = false;
        lastJumpTime = currentTime;
        maxJumpForce = 0;
        jumpCount++; // Increment jump counter
        
        // Update jump characteristic when jump is detected
        jumpCharacteristic.writeValue(jumpCount);
        
        return true; // Jump completed
    }

    // Reset jump state if we've been in the air too long (timeout)
    if (isInJumpState && (currentTime - jumpStartTime > JUMP_MAX_TIME)) {
        isInJumpState = false;
        maxJumpForce = 0;
    }

    return false;
}

// Modified updateDisplacement function
void updateDisplacement(float accelX, float accelY, float accelZ, unsigned long currentTime) {
    // Remove gravity component based on orientation
    float adjustedX = accelX - (calibrationX * gravityVector[0]);
    float adjustedY = accelY - (calibrationY * gravityVector[1]);
    float adjustedZ = accelZ - (calibrationZ * gravityVector[2]);
    
    float dt = (currentTime - lastSampleTime) / 1000.0;
    if (lastSampleTime == 0) dt = 0;
    
    if (dt > 0 && dt < 0.1) {
        // Calculate vertical component based on orientation
        float verticalAccel;
        
        switch (currentOrientation) {
            case ORIENTATION_NORMAL_Z_UP:
                verticalAccel = adjustedZ;
                break;
            case ORIENTATION_UPSIDE_DOWN:
                verticalAccel = -adjustedZ;
                break;
            case ORIENTATION_HORIZONTAL_X_POS:
                verticalAccel = adjustedX;
                break;
            case ORIENTATION_HORIZONTAL_X_NEG:
                verticalAccel = -adjustedX;
                break;
            case ORIENTATION_VERTICAL_Y_POS:
                verticalAccel = adjustedY;
                break;
            case ORIENTATION_VERTICAL_Y_NEG:
                verticalAccel = -adjustedY;
                break;
            default:
                // For tilted orientation, use the dominant axis
                if (abs(adjustedZ) > abs(adjustedX) && abs(adjustedZ) > abs(adjustedY))
                    verticalAccel = adjustedZ;
                else if (abs(adjustedX) > abs(adjustedY))
                    verticalAccel = adjustedX;
                else
                    verticalAccel = adjustedY;
                break;
        }

        // Process jump detection
        if (processJumpDetection(verticalAccel, currentTime)) {
            // Jump completed (landing detected)
            consecutiveJumps++;
            hasJumpingPattern = true;
            lastJumpDetectTime = currentTime;
            stepCount++; // Count jump as a step
        }

        // Reset jump pattern if no jumps for a while
        if (currentTime - lastJumpDetectTime > JUMP_RESET_INTERVAL) {
            if (!isInJumpState) { // Only reset if we're not currently in a jump
                consecutiveJumps = 0;
                hasJumpingPattern = false;
            }
        }

        // Only update displacement calculations if not in a jump state
        if (!isInJumpState && !hasJumpingPattern) {
            float horizontalAccel = sqrt(
                adjustedX * adjustedX + 
                adjustedY * adjustedY + 
                adjustedZ * adjustedZ) - abs(verticalAccel);
            
            // Apply bias correction
            horizontalAccel = max(0.0f, horizontalAccel - ACCEL_BIAS);
            
            // Update velocity and displacement
            velocityX = (velocityX + horizontalAccel * dt) * VELOCITY_DECAY;
            if (abs(velocityX) > 0) {
                displacementX += velocityX * dt;
                totalDisplacement = abs(displacementX);
            }
        }
    }
    
    lastSampleTime = currentTime;
}

// Modified determineMovementType function
String determineMovementType() {
    // Check for jumping first - requires 3 or more jumps
    if (jumpCount >= 3) {
        return "Jumping";
    }
    
    // Calculate current speed
    float currentSpeed = calculateSpeed(stepCount, millis() - measurementStartTime);
    
    // Running if speed is more than 5 km/h
    if (currentSpeed > 5.0) {
        return "Running";
    }
    
    // Default to walking
    return "Walking";
}

float calculateMET(String type) {
  if (type == "Running") {
    float speed = calculateSpeed(stepCount, userDurationMillis);
    
    // More accurate MET values based on running speed (kph)
    if (speed < 6.4) return 6.5;
    else if (speed < 6.9) return 6.5;
    else if (speed < 8.0) return 7.8;
    else if (speed < 8.8) return 8.5;
    else if (speed < 9.6) return 9.0;
    else if (speed < 10.8) return 9.3;
    else if (speed < 11.2) return 10.5;
    else if (speed < 12.0) return 11.0;
    else if (speed < 12.8) return 11.8;
    else if (speed < 13.8) return 12.0;
    else if (speed < 14.5) return 12.5;
    else if (speed < 15.0) return 13.0;
    else if (speed < 16.0) return 14.8;
    else if (speed < 17.7) return 14.8;
    else if (speed < 19.3) return 16.8;
    else if (speed < 21.0) return 18.5;
    else if (speed < 22.5) return 19.8;
    else return 23.0;
  }
  else if (type == "Jumping") {
    // Calculate jumps per minute based on step count and elapsed time
    unsigned long elapsedTimeMinutes = (millis() - measurementStartTime) / 60000.0;
    if (elapsedTimeMinutes < 0.1) elapsedTimeMinutes = 0.1; // Prevent division by zero
    float jumpsPerMinute = stepCount / elapsedTimeMinutes;
    
    // MET values based on jumps per minute for jumping activity
    if (jumpsPerMinute < 80) return 8.3;
    else if (jumpsPerMinute < 100) return 10.0;
    else if (jumpsPerMinute < 120) return 11.8;
    else return 12.3; // 120+ jumps per minute
  }
  
  // Default walking MET
  return 3.5;
}

float calculateCalories(float MET, int weightKg, int durationMinutes) {
  // More accurate formula: MET * 3.5 * weightKg / 200.0 * durationMinutes
  // This formula considers:
  // - MET value (metabolic equivalent of task)
  // - 3.5 ml/kg/min (oxygen consumption at rest)
  // - User weight in kg
  // - 200.0 conversion factor (1 liter O2 ~ 5 kcal)
  // - Duration in minutes
  
  // Calculate effective duration (actual elapsed time, not just the set duration)
  float effectiveDuration = durationMinutes;
  if (measurementStarted) {
    unsigned long elapsedMs = millis() - measurementStartTime;
    effectiveDuration = elapsedMs / 60000.0;
  }
  
  // Use the higher of the two durations to avoid underestimation
  if (effectiveDuration < 0.1) effectiveDuration = 0.1; // Prevent division by zero
  
  // Apply the formula with the effective duration
  return (MET * 3.5 * weightKg / 200.0) * effectiveDuration;
}

// Update calibration values with orientation detection
void updateCalibration(float accelX, float accelY, float accelZ) {
  if (currentState == STATE_CALIBRATION && !calibrationComplete) {
    float gyroX, gyroY, gyroZ;
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gyroX, gyroY, gyroZ);
      gyroSumX += gyroX;
      gyroSumY += gyroY;
      gyroSumZ += gyroZ;
    }
    
    calibrationSumX += accelX;
    calibrationSumY += accelY;
    calibrationSumZ += accelZ;
    calibrationCount++;
    
    if (calibrationCount >= CALIBRATION_SAMPLES) {
      calibrationX = calibrationSumX / CALIBRATION_SAMPLES;
      calibrationY = calibrationSumY / CALIBRATION_SAMPLES;
      calibrationZ = calibrationSumZ / CALIBRATION_SAMPLES;
      
      gyroCalibX = gyroSumX / CALIBRATION_SAMPLES;
      gyroCalibY = gyroSumY / CALIBRATION_SAMPLES;
      gyroCalibZ = gyroSumZ / CALIBRATION_SAMPLES;
      
      // Determine device orientation
      currentOrientation = determineOrientation(calibrationX, calibrationY, calibrationZ);
      
      calibrationComplete = true;
      Serial.println("Calibration complete:");
      Serial.print("Accel X: "); Serial.println(calibrationX);
      Serial.print("Accel Y: "); Serial.println(calibrationY);
      Serial.print("Accel Z: "); Serial.println(calibrationZ);
    }
    displayCalibration();
  }
}

void sendResults() {
  currentState = STATE_RESULTS;
  
  float speed = calculateSpeed(stepCount, userDurationMillis);
  String type = determineMovementType();
  float MET = calculateMET(type);
  float calories = calculateCalories(MET, userWeight, userDurationMinutes);
  
  // Final update of BLE characteristics
  stepCharacteristic.writeValue(stepCount);
  jumpCharacteristic.writeValue(jumpCount);
  
  // Calculate intensity metrics
  String intensityLevel = "";
  if (type == "Running") {
    if (speed < 8.0) intensityLevel = "Light";
    else if (speed < 11.0) intensityLevel = "Moderate";
    else if (speed < 14.0) intensityLevel = "Vigorous";
    else intensityLevel = "High";
  } else if (type == "Jumping") {
    unsigned long elapsedTimeMinutes = (millis() - measurementStartTime) / 60000.0;
    if (elapsedTimeMinutes < 0.1) elapsedTimeMinutes = 0.1;
    float jumpsPerMinute = jumpCount / (float)elapsedTimeMinutes;
    
    if (jumpsPerMinute < 80) intensityLevel = "Light";
    else if (jumpsPerMinute < 100) intensityLevel = "Moderate";
    else if (jumpsPerMinute < 120) intensityLevel = "Vigorous";
    else intensityLevel = "High";
  } else {
    if (speed < 4.0) intensityLevel = "Light";
    else if (speed < 5.5) intensityLevel = "Moderate";
    else intensityLevel = "Brisk";
  }

  // Output to Serial
  Serial.println("--- Final Results ---");
  Serial.println("Activity: " + type);
  Serial.println("Intensity: " + intensityLevel);
  Serial.println("MET Value: " + String(MET, 1));

  if (type == "Jumping") {
    unsigned long elapsedTimeMinutes = (millis() - measurementStartTime) / 60000.0;
    if (elapsedTimeMinutes < 0.1) elapsedTimeMinutes = 0.1;
    float jumpsPerMinute = jumpCount / (float)elapsedTimeMinutes;
    
    Serial.println("Jumps: " + String(jumpCount));
    Serial.println("Jumps/min: " + String(jumpsPerMinute, 1));
    Serial.println("Calories: " + String(calories, 2) + " kcal");
  } else {
    Serial.println("Steps: " + String(stepCount));
    Serial.println("Speed: " + String(speed, 2) + " km/h");
    Serial.println("Calories: " + String(calories, 2) + " kcal");
  }
  Serial.println("---------------------");

  // Display results using new minimal layout
  displayResults(type, intensityLevel, speed, calories);
}

void loop() {
  // Handle BLE central device connections/disconnections
  BLEDevice central = BLE.central();
  
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    
    while (central.connected()) {
      // Check for button inputs
      checkButtons();
      
      // Handle sensor data
      float accelX, accelY, accelZ;
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(accelX, accelY, accelZ);
        float magnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
        unsigned long currentTime = millis();

        // Handle calibration if in calibration state
        if (currentState == STATE_CALIBRATION) {
          updateCalibration(accelX, accelY, accelZ);
          continue;  // Skip other processing during calibration
        }

        // Auto-start detection (if in ready state)
        if (currentState == STATE_READY && !measurementStarted && magnitude > STEP_THRESHOLD) {
          startMeasurement();
        }

        if (currentState == STATE_TRACKING && measurementStarted && !measurementComplete) {
          // Pass all acceleration components to the improved function
          updateDisplacement(accelX, accelY, accelZ, currentTime);
          
          // Update live display
          displayLiveData();
          
          // Update BLE characteristics with notifications
          if (stepCharacteristic.written()) {
            Serial.print("Steps sent via BLE: ");
            Serial.println(stepCount);
          }
          stepCharacteristic.writeValue(stepCount);
          
          if (jumpCharacteristic.written()) {
            Serial.print("Jumps sent via BLE: ");
            Serial.println(jumpCount);
          }
          jumpCharacteristic.writeValue(jumpCount);
          
          if (currentTime - measurementStartTime >= userDurationMillis) {
            sendResults();
          } else if (currentTime - lastStepTime >= STEP_DEBOUNCE_TIME) {
            // Only detect steps if we're not in a jumping state
            if (!isInJumpState) {
              if (magnitude > STEP_THRESHOLD && !wasInStep) {
                wasInStep = true;
              } else if (magnitude < (STEP_THRESHOLD * 0.8) && wasInStep) {
                wasInStep = false;
                stepCount++;
                lastStepTime = currentTime;
                
                // Update step characteristic when step is detected
                stepCharacteristic.writeValue(stepCount);
                Serial.print("Step detected and sent via BLE: ");
                Serial.println(stepCount);
              }
            }
          }
        }
      }
      
      delay(10);
    }
    
    Serial.println("Disconnected from central device");
  }
  
  // Poll BLE events
  BLE.poll();
  
  delay(10);
}
