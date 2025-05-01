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
  STATE_READY,
  STATE_TRACKING,
  STATE_RESULTS
};
DeviceState currentState = STATE_WEIGHT_SETUP;

// Movement detection thresholds
const float STEP_THRESHOLD = 2.2;        // Minimum acceleration to detect a step (in m/s²)
                                         // Balanced threshold for step detection
                                         // Typical walking: 2.0-3.0, Running: 3.0-4.0

const float STRIDE_LENGTH = 0.7;         // Average stride length in meters
                                         // Used to calculate speed and distance
                                         // Adjust based on user's height/stride

const int STEP_DEBOUNCE_TIME = 300;      // Minimum time between step detections (in ms)
                                         // Balanced time to prevent double-counting
                                         // Increase if counting too many steps, decrease if missing steps

const float STEPPING_THRESHOLD = 1.2;    // Acceleration threshold to distinguish walking (in m/s²)
                                         // Balanced threshold for walking detection
                                         // Below this: No significant movement
                                         // Above this: Walking detected

const float RUNNING_THRESHOLD = 2.5;     // Acceleration threshold to distinguish running (in m/s²)
                                         // Balanced threshold for running detection
                                         // Below this: Walking
                                         // Above this: Running

const float VELOCITY_DECAY = 0.98;       // Rate at which velocity decreases over time (0.0-1.0)
                                         // Higher values = slower decay, more sensitive to movement
                                         // Lower values = faster decay, more stable but less sensitive

const float ACCEL_BIAS = 0.001;          // Small offset to correct accelerometer bias (in m/s²)
                                        // Helps reduce drift in displacement calculations
                                        // Adjust based on your specific accelerometer's characteristics

// Constants for improved jumping detection
const float VERTICAL_JUMP_THRESHOLD = 3.5;      // Minimum vertical acceleration to qualify as jumping
                                                // Lowered to detect more jumps while avoiding false positives
const float HORIZONTAL_LIMIT_FOR_JUMP = 1.2;    // Maximum horizontal acceleration during jumping
                                                // Balanced limit to distinguish from walking
const float DISPLACEMENT_DECAY_RATE = 0.96;     // Faster displacement reduction rate
const float JUMPING_DISPLACEMENT_LIMIT = 6.0;   // Lower limit on displacement
                                                // Helps detect jumps more easily
const unsigned long JUMP_PATTERN_TIMEOUT = 1200; // Reset jump pattern after 1.2 seconds
                                                // Gives enough time to detect jumps
const unsigned long JUMP_RESET_INTERVAL = 2500;  // Reset accumulated displacement after 2.5 seconds
                                                // Maintains jump state longer

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
unsigned long lastJumpDetectTime = 0;    // Add back this declaration
unsigned long consecutiveJumpingTime = 0; // Add back this declaration



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
  
  // Start with weight setup
  displayWeightSetup();
  
  Serial.println("Anklet Ready - No BLE Version");
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
      currentState = STATE_READY;
      displayStatus("Ready", "Press MODE to start");
      break;
      
    case STATE_READY:
      // Manual start
      startMeasurement();
      break;
      
    case STATE_TRACKING:
      // Force end measurement
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
  
  // Fancy header with icon
  display.drawRect(0, 0, display.width(), 14, SSD1306_WHITE);
  display.fillRect(0, 0, display.width(), 14, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(32, 3);
  display.print("WEIGHT SETUP");
  
  // Draw small weight icon
  display.drawRect(12, 3, 8, 8, SSD1306_BLACK);
  display.drawLine(15, 1, 15, 3, SSD1306_BLACK);
  display.drawLine(17, 1, 17, 3, SSD1306_BLACK);
  
  // Weight value (large with kg units smaller) - centered
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(3);
  
  // Calculate center position based on weight value width
  int centerX = 64; // Center of 128px wide display
  int digitWidth = 18; // Width of each digit in size 3 font
  int weightDigits = (userWeight < 100) ? 2 : 3;
  int startPos = centerX - ((weightDigits * digitWidth) / 2);
  
  display.setCursor(startPos, 20);
  display.print(userWeight);
  
  display.setTextSize(1);
  display.setCursor(startPos + (weightDigits * digitWidth) + 2, 26);
  display.print("kg");
  
  // Visual weight slider with better styling
  int sliderWidth = display.width() - 20;
  int sliderPos = map(userWeight, 30, 150, 0, sliderWidth);
  display.drawRect(10, 40, sliderWidth, 8, SSD1306_WHITE);
  display.fillRect(10, 40, sliderPos, 8, SSD1306_WHITE);
  
  // Add a marker on the filled part
  if (sliderPos > 3 && sliderPos < sliderWidth - 3) {
    display.drawLine(10 + sliderPos, 38, 10 + sliderPos, 50, SSD1306_WHITE);
    display.drawLine(9 + sliderPos, 39, 9 + sliderPos, 49, SSD1306_WHITE);
  }
  
  // Controls help in a button-like appearance
  display.drawRect(0, 53, 62, 11, SSD1306_WHITE);
  display.setCursor(4, 55);
  display.print("UP/DOWN");
  
  display.drawRect(66, 53, 62, 11, SSD1306_WHITE);
  display.setCursor(80, 55);
  display.print("MODE");
  
  display.display();
}

// Display duration setup screen
void displayDurationSetup() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Fancy header with icon
  display.drawRect(0, 0, display.width(), 14, SSD1306_WHITE);
  display.fillRect(0, 0, display.width(), 14, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(24, 3);
  display.print("DURATION SETUP");
  
  // Draw small clock icon
  display.drawCircle(12, 7, 5, SSD1306_BLACK);
  display.drawLine(12, 7, 12, 4, SSD1306_BLACK);
  display.drawLine(12, 7, 15, 7, SSD1306_BLACK);
  
  // Duration value (large with min units smaller)
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(3);
  display.setCursor(userDurationMinutes < 10 ? 40 : 30, 20);
  display.print(userDurationMinutes);
  
  display.setTextSize(1);
  display.setCursor(userDurationMinutes < 10 ? 64 : 54, 26);
  display.print("min");
  
  // Duration slider with ticks
  int sliderWidth = display.width() - 20;
  int sliderPos = map(userDurationMinutes, 1, 60, 0, sliderWidth);
  display.drawRect(10, 40, sliderWidth, 8, SSD1306_WHITE);
  display.fillRect(10, 40, sliderPos, 8, SSD1306_WHITE);
  
  // Add tick marks
  for (int i = 0; i <= 6; i++) {
    int tickPos = map(i * 10, 0, 60, 0, sliderWidth);
    display.drawLine(10 + tickPos, 48, 10 + tickPos, 50, SSD1306_WHITE);
  }
  
  // Add a marker on the filled part
  if (sliderPos > 3 && sliderPos < sliderWidth - 3) {
    display.drawLine(10 + sliderPos, 38, 10 + sliderPos, 50, SSD1306_WHITE);
    display.drawLine(9 + sliderPos, 39, 9 + sliderPos, 49, SSD1306_WHITE);
  }
  
  // Min/Max indicators
  display.setTextSize(1);
  
  // Controls help in a button-like appearance
  display.drawRect(0, 53, 62, 11, SSD1306_WHITE);
  display.setCursor(4, 55);
  display.print("UP/DOWN");
  
  display.drawRect(66, 53, 62, 11, SSD1306_WHITE);
  display.setCursor(80, 55);
  display.print("MODE");
  
  display.display();
}

// Helper function to display status messages
void displayStatus(String line1, String line2) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Fancy header with logo
  display.drawRect(0, 0, display.width(), 14, SSD1306_WHITE);
  display.fillRect(0, 0, display.width(), 14, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(30, 3);
  display.print("ANKLET DEVICE");
  
  // Draw small anklet icon
  display.drawLine(10, 5, 20, 5, SSD1306_BLACK);
  display.drawLine(10, 5, 10, 9, SSD1306_BLACK);
  display.drawLine(20, 5, 20, 9, SSD1306_BLACK);
  display.drawLine(10, 9, 20, 9, SSD1306_BLACK);
  
  // Main status with icon
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  
  // Display main status with appropriate icon
  display.setCursor(20, 18);
  display.setTextSize(1);
  display.println(line1);
  
  // Draw status indicator icon
  if (line1.indexOf("Ready") >= 0) {
    // Ready icon (checkmark)
    display.drawCircle(10, 20, 6, SSD1306_WHITE);
    display.drawLine(7, 20, 9, 22, SSD1306_WHITE);
    display.drawLine(9, 22, 13, 18, SSD1306_WHITE);
  } else if (line1.indexOf("ERROR") >= 0) {
    // Error icon (X)
    display.drawCircle(10, 20, 6, SSD1306_WHITE);
    display.drawLine(7, 17, 13, 23, SSD1306_WHITE);
    display.drawLine(13, 17, 7, 23, SSD1306_WHITE);
  } else if (line1.indexOf("Starting") >= 0) {
    // Starting icon (loading circle)
    display.drawCircle(10, 20, 6, SSD1306_WHITE);
    display.drawLine(10, 20, 10, 15, SSD1306_WHITE);
  } else if (line1.indexOf("Measure") >= 0) {
    // Measurement icon (running figure)
    display.drawLine(7, 17, 9, 21, SSD1306_WHITE);
    display.drawLine(9, 21, 11, 17, SSD1306_WHITE);
    display.drawLine(11, 19, 13, 23, SSD1306_WHITE);
    display.drawCircle(9, 15, 2, SSD1306_WHITE);
  } else {
    // Default icon (info)
    display.drawCircle(10, 20, 6, SSD1306_WHITE);
    display.drawPixel(10, 17, SSD1306_WHITE);
    display.drawLine(10, 19, 10, 23, SSD1306_WHITE);
  }
  
  // Display secondary status in a box
  display.drawRect(0, 27, display.width(), 14, SSD1306_WHITE);
  display.setCursor(3, 30);
  display.println(line2);
  
  // Bottom status bar with standalone indicator
  display.fillRect(0, 54, display.width(), 10, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setCursor(30, 55);

  
  // Battery indicator (mockup)
  display.drawRect(110, 55, 12, 7, SSD1306_BLACK);
  display.drawRect(122, 57, 2, 3, SSD1306_BLACK);
  display.fillRect(111, 56, 10, 5, SSD1306_BLACK);
  
  display.display();
}

// Display live measurement data
void displayLiveData() {
  if (millis() - lastDisplayUpdateTime < DISPLAY_UPDATE_INTERVAL) return;
  
  lastDisplayUpdateTime = millis();
  
  unsigned long elapsedTime = millis() - measurementStartTime;
  unsigned long remainingTime = userDurationMillis > elapsedTime ? userDurationMillis - elapsedTime : 0;
  int remainingSeconds = remainingTime / 1000;
  int remainingMinutes = remainingSeconds / 60;
  remainingSeconds %= 60;
  
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Animated header
  display.fillRect(0, 0, display.width(), 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(32, 2);
  display.print("TRACKING ACTIVE");
  
  // Draw animated running figure based on time - moved left slightly
  int animFrame = (elapsedTime / 250) % 4;
  switch(animFrame) {
    case 0:
      display.drawLine(8, 2, 6, 5, SSD1306_BLACK);
      display.drawLine(8, 2, 10, 5, SSD1306_BLACK);
      display.drawLine(8, 2, 8, 8, SSD1306_BLACK);
      display.drawLine(8, 8, 6, 11, SSD1306_BLACK);
      display.drawLine(8, 8, 10, 11, SSD1306_BLACK);
      break;
    case 1:
      display.drawLine(8, 2, 5, 6, SSD1306_BLACK);
      display.drawLine(8, 2, 11, 6, SSD1306_BLACK);
      display.drawLine(8, 2, 8, 8, SSD1306_BLACK);
      display.drawLine(8, 8, 5, 10, SSD1306_BLACK);
      display.drawLine(8, 8, 11, 10, SSD1306_BLACK);
      break;
    case 2:
      display.drawLine(8, 2, 4, 5, SSD1306_BLACK);
      display.drawLine(8, 2, 12, 5, SSD1306_BLACK);
      display.drawLine(8, 2, 8, 8, SSD1306_BLACK);
      display.drawLine(8, 8, 4, 11, SSD1306_BLACK);
      display.drawLine(8, 8, 12, 11, SSD1306_BLACK);
      break;
    case 3:
      display.drawLine(8, 2, 5, 4, SSD1306_BLACK);
      display.drawLine(8, 2, 11, 4, SSD1306_BLACK);
      display.drawLine(8, 2, 8, 8, SSD1306_BLACK);
      display.drawLine(8, 8, 5, 8, SSD1306_BLACK);
      display.drawLine(8, 8, 11, 8, SSD1306_BLACK);
      break;
  }
  
  display.setTextColor(SSD1306_WHITE);
  
  // Step counter with icon - moved to left side
  display.drawCircle(8, 19, 5, SSD1306_WHITE);
  display.drawLine(8, 16, 8, 19, SSD1306_WHITE); 
  display.drawLine(8, 19, 10, 21, SSD1306_WHITE);
  
  // Steps count - moved right to prevent overlap with icon
  display.setTextSize(1);
  display.setCursor(16, 16);
  display.print("Steps: ");
  display.print(stepCount);
  
  // Time remaining with countdown display - moved to fit better
  display.drawRect(75, 15, 50, 12, SSD1306_WHITE);
  display.fillRect(76, 16, 48, 10, SSD1306_BLACK);
  display.setCursor(82, 17);
  
  // Digital clock style time display
  if (remainingMinutes < 10) display.print("0");
  display.print(remainingMinutes);
  display.print(":");
  if (remainingSeconds < 10) display.print("0");
  display.print(remainingSeconds);
  
  // Movement type with icon - adjusted positioning
  String moveType = determineMovementType();
  display.setCursor(16, 30);
  display.print("Mode: ");
  display.print(moveType);
  
  // Type-specific icon - moved to align with other icons
  if (moveType == "Jumping") {
    display.drawLine(8, 30, 8, 33, SSD1306_WHITE); // Body
    display.drawLine(6, 35, 8, 33, SSD1306_WHITE);  // Legs spread
    display.drawLine(10, 35, 8, 33, SSD1306_WHITE);
    display.drawLine(5, 31, 8, 33, SSD1306_WHITE);  // Arms up
    display.drawLine(11, 31, 8, 33, SSD1306_WHITE);
  } else if (moveType == "Running") {
    display.drawLine(5, 30, 8, 32, SSD1306_WHITE);  // Forward leg
    display.drawLine(8, 32, 11, 30, SSD1306_WHITE); // Back leg
    display.drawLine(8, 32, 8, 28, SSD1306_WHITE); // Body
    display.drawLine(8, 28, 6, 29, SSD1306_WHITE);  // Arms
    display.drawLine(8, 28, 10, 29, SSD1306_WHITE);
  } else { // Walking
    display.drawLine(6, 33, 8, 30, SSD1306_WHITE);  // Legs
    display.drawLine(8, 30, 10, 33, SSD1306_WHITE);
    display.drawLine(8, 30, 8, 27, SSD1306_WHITE); // Body
    display.drawLine(6, 29, 8, 30, SSD1306_WHITE);  // Arms
    display.drawLine(8, 30, 10, 29, SSD1306_WHITE);
  }
  
  // Current statistics in box - adjusted to prevent clipping
  display.drawRect(0, 40, display.width(), 12, SSD1306_WHITE);
  display.setCursor(4, 42);
  
  // Show appropriate stats based on movement type
  if (moveType == "Jumping") {
    display.print("Calories: ");
    float calories = calculateCalories(calculateMET(moveType), userWeight, (int)(elapsedTime / 60000.0));
    display.print(calories, 1);
    display.print(" kcal");
  } else {
    display.print("Speed: ");
    float currentSpeed = calculateSpeed(stepCount, elapsedTime > 0 ? elapsedTime : 1);
    display.print(currentSpeed, 1);
    display.print(" km/h");
  }
  
  // Progress bar with percentage - moved up from edge
  int progressWidth = map(elapsedTime, 0, userDurationMillis, 0, display.width() - 40);
  int percent = map(elapsedTime, 0, userDurationMillis, 0, 100);
  
  display.drawRect(0, 53, display.width() - 35, 10, SSD1306_WHITE);
  display.fillRect(1, 54, progressWidth, 8, SSD1306_WHITE);
  
  // Add percentage text - moved to align better
  display.setCursor(display.width() - 30, 55);
  display.print(percent);
  display.print("%");
  
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
  hasJumpingPattern = false;
  verticalAccelMax = 0;
  jumpPatternDuration = 0;
}

float calculateSpeed(int steps, unsigned long timeMs) {
  float timeMinutes = timeMs / 60000.0;
  float stepsPerMinute = steps / timeMinutes;
  return (stepsPerMinute * STRIDE_LENGTH * 60.0) / 1000.0;
}

// Determine movement type with improved jump detection
String determineMovementType() {
    // Debug data
    Serial.print("DISP: ");
    Serial.print(totalDisplacement);
    Serial.print(" Jump: ");
    Serial.println(hasJumpingPattern ? "YES" : "NO");
    
    // More sensitive jumping detection
    if (hasJumpingPattern || verticalAccelMax > VERTICAL_JUMP_THRESHOLD * 1.1) {
        return "Jumping";
    }
    
    // Force running detection if displacement is above threshold
    if (totalDisplacement > RUNNING_THRESHOLD) {
        return "Running";
    }
    
    // Detect walking if there's any movement
    if (totalDisplacement > STEPPING_THRESHOLD) {
        return "Walking";
    }
    
    return "Walking";  // Default to walking if no other movement detected
}

// Improved updateDisplacement function
void updateDisplacement(float accelX, float accelY, float accelZ, unsigned long currentTime) {
  float dt = (currentTime - lastSampleTime) / 1000.0;
  if (lastSampleTime == 0) dt = 0;
  
  if (dt > 0 && dt < 0.1) {
    // Only update if we have a reasonable time interval
    
    // Track maximum vertical acceleration for jump detection
    if (abs(accelZ) > verticalAccelMax) {
      verticalAccelMax = abs(accelZ);
    }
    
    // Apply bias correction - different for each axis
    float filteredAccelX = accelX - ACCEL_BIAS;
    float filteredAccelY = accelY - ACCEL_BIAS;
    
    // Calculate horizontal and vertical components
    float horizontalAccel = sqrt(filteredAccelX * filteredAccelX + filteredAccelY * filteredAccelY);
    float verticalAccel = abs(accelZ);
    
    // Simplified jumping detection logic - extremely sensitive
    bool currentlyJumping = false;
    
    // Detect jumping if vertical acceleration is above threshold
    if (verticalAccel > VERTICAL_JUMP_THRESHOLD) {
        currentlyJumping = true;
        lastJumpDetectTime = currentTime;
        hasJumpingPattern = true;  // Immediately set jumping pattern
        
        // Start or continue tracking jump patterns
        if (jumpPatternDuration == 0) {
            jumpPatternDuration = currentTime;
        }
        
        // Track consecutive jumping time
        if (consecutiveJumpingTime == 0) {
            consecutiveJumpingTime = currentTime;
        }
    }
    
    // Reset jumping state more gradually
    if (currentTime - lastJumpDetectTime > JUMP_PATTERN_TIMEOUT) {
        if (currentTime - lastJumpDetectTime > JUMP_PATTERN_TIMEOUT * 3) {
            jumpPatternDuration = 0;
            hasJumpingPattern = false;
        }
    }
    
    // Force velocity decay based on movement type
    float currentDecay;
    
    // Use much stronger decay when jumping is detected
    if (currentlyJumping || hasJumpingPattern) {
      currentDecay = VELOCITY_DECAY * 0.5; // Much stronger damping during jumps
      
      // Nearly eliminate horizontal acceleration during jumping
      filteredAccelX *= 0.1;
      filteredAccelY *= 0.1;
      
      // Check for extended jumping and reset displacement
      if (currentTime - consecutiveJumpingTime > JUMP_RESET_INTERVAL) {
        // After continuous jumping, force displacement reduction
        displacementX *= DISPLACEMENT_DECAY_RATE;
        displacementY *= DISPLACEMENT_DECAY_RATE;
      }
      
      // Force displacement to stay under the jump displacement limit
      if (totalDisplacement > JUMPING_DISPLACEMENT_LIMIT) {
        float scaleFactor = JUMPING_DISPLACEMENT_LIMIT / totalDisplacement;
        displacementX *= scaleFactor;
        displacementY *= scaleFactor;
      }
    } else {
      currentDecay = VELOCITY_DECAY; 
      consecutiveJumpingTime = 0; // Reset consecutive jumping time
    }
    
    // Apply static thresholds to filter out noise
    if (abs(filteredAccelX) < 0.2) filteredAccelX = 0;
    if (abs(filteredAccelY) < 0.2) filteredAccelY = 0;
    
    // Update velocity with filtered acceleration
    velocityX = (velocityX + filteredAccelX * dt) * currentDecay;
    velocityY = (velocityY + filteredAccelY * dt) * currentDecay;
    
    // Apply stronger velocity thresholds to prevent drift
    if (abs(velocityX) < 0.1) velocityX = 0;
    if (abs(velocityY) < 0.1) velocityY = 0;
    
    // Update displacement only if we have significant velocity
    if (abs(velocityX) > 0 || abs(velocityY) > 0) {
      displacementX += velocityX * dt;
      displacementY += velocityY * dt;
    }
    
    // Calculate total displacement
    totalDisplacement = sqrt(displacementX * displacementX + displacementY * displacementY);
    
    // Periodically reset max vertical acceleration for continued monitoring
    static unsigned long lastResetTime = 0;
    if (currentTime - lastResetTime > 1000) { // Reset once per second
      verticalAccelMax = 0;
      lastResetTime = currentTime;
    }
  }
  
  lastSampleTime = currentTime;
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

  Serial.println("--- Final Results ---");
  Serial.println("Activity: " + type);

  if (type == "Jumping") {
    Serial.println("Jumps: " + String(stepCount));
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

  // Display final results with fancy animations
  display.clearDisplay();
  
  // Draw celebratory animation
  for (int i = 0; i < 3; i++) {
    // Frame 1: Starburst
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    
    // Draw radiating lines
    for (int angle = 0; angle < 360; angle += 30) {
      float radians = angle * PI / 180.0;
      int x1 = 64 + 10 * cos(radians);
      int y1 = 32 + 10 * sin(radians);
      int x2 = 64 + 20 * cos(radians);
      int y2 = 32 + 20 * sin(radians);
      display.drawLine(x1, y1, x2, y2, SSD1306_WHITE);
    }
    
    display.setTextSize(2);
    display.setCursor(20, 24); // Centered text better
    display.print("COMPLETE!");
    display.display();
    delay(150);
    
    // Frame 2: Circle
    display.clearDisplay();
    display.drawCircle(64, 32, 20, SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(20, 24); // Centered text better
    display.print("COMPLETE!");
    display.display();
    delay(150);
  }
  
  // Show final results screen
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Banner header
  display.fillRect(0, 0, display.width(), 15, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(20, 4); // Moved text right to prevent overlap with icon
  display.print("WORKOUT COMPLETE!");
  
  // Trophy icon
  display.drawRect(2, 3, 6, 9, SSD1306_BLACK);
  display.drawLine(3, 3, 3, 1, SSD1306_BLACK);
  display.drawLine(6, 3, 6, 1, SSD1306_BLACK);
  display.drawLine(3, 1, 6, 1, SSD1306_BLACK);
  
  // Results panel
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  
  // Activity type with icon - adjusted spacing
  display.setCursor(20, 18);
  display.print("Activity: ");
  display.print(type);
  
  // Activity icon - moved left slightly
  if (type == "Jumping") {
    display.drawLine(10, 17, 10, 20, SSD1306_WHITE);
    display.drawLine(8, 22, 10, 20, SSD1306_WHITE);
    display.drawLine(12, 22, 10, 20, SSD1306_WHITE);
    display.drawLine(7, 18, 10, 20, SSD1306_WHITE);
    display.drawLine(13, 18, 10, 20, SSD1306_WHITE);
  } else if (type == "Running") {
    display.drawLine(7, 17, 10, 19, SSD1306_WHITE);
    display.drawLine(10, 19, 13, 17, SSD1306_WHITE);
    display.drawLine(10, 19, 10, 15, SSD1306_WHITE);
    display.drawCircle(10, 14, 2, SSD1306_WHITE);
  } else { // Walking
    display.drawLine(8, 20, 10, 17, SSD1306_WHITE);
    display.drawLine(10, 17, 12, 20, SSD1306_WHITE);
    display.drawLine(10, 17, 10, 14, SSD1306_WHITE);
    display.drawCircle(10, 13, 2, SSD1306_WHITE);
  }
  
  // Stylish separators
  display.drawLine(0, 26, display.width(), 26, SSD1306_WHITE);
  
  // Results with icons - increased spacing between items
  int yPos = 30;
  
  if (type == "Jumping") {
    // Jumps count
    display.drawCircle(10, yPos, 4, SSD1306_WHITE);
    display.drawLine(10, yPos-2, 10, yPos+2, SSD1306_WHITE);
    display.setCursor(18, yPos-2);
    display.print("Jumps: ");
    display.print(stepCount);
    yPos += 12; // Increased spacing
    
    // Calories
    display.drawRect(8, yPos-2, 5, 7, SSD1306_WHITE);
    display.drawLine(10, yPos-4, 10, yPos-2, SSD1306_WHITE);
    display.setCursor(18, yPos-2);
    display.print("Cal: ");
    display.print(calories, 1);
    display.print(" kcal");
  } else if (type == "Running") {
    // Speed with speedometer icon
    display.drawCircle(10, yPos, 4, SSD1306_WHITE);
    display.drawLine(10, yPos, 10+3, yPos-2, SSD1306_WHITE);
    display.setCursor(18, yPos-2);
    display.print("Speed: ");
    display.print(speed, 1);
    display.print(" km/h");
    yPos += 12; // Increased spacing
    
    // Steps with footprint icon
    display.drawCircle(9, yPos-1, 2, SSD1306_WHITE);
    display.drawCircle(12, yPos+1, 2, SSD1306_WHITE);
    display.setCursor(18, yPos-2);
    display.print("Steps: ");
    display.print(stepCount);
    yPos += 12; // Increased spacing
    
    // Calories with fire icon
    display.drawLine(8, yPos, 10, yPos-4, SSD1306_WHITE);
    display.drawLine(10, yPos-4, 12, yPos, SSD1306_WHITE);
    display.drawLine(10, yPos, 10, yPos-3, SSD1306_WHITE);
    display.setCursor(18, yPos-2);
    display.print("Cal: ");
    display.print(calories, 1);
    display.print(" kcal");
  } else {
    // Steps with footprint icon
    display.drawCircle(9, yPos-1, 2, SSD1306_WHITE);
    display.drawCircle(12, yPos+1, 2, SSD1306_WHITE);
    display.setCursor(18, yPos-2);
    display.print("Steps: ");
    display.print(stepCount);
    yPos += 12; // Increased spacing
    
    // Speed with speedometer icon
    display.drawCircle(10, yPos, 4, SSD1306_WHITE);
    display.drawLine(10, yPos, 10+3, yPos-2, SSD1306_WHITE);
    display.setCursor(18, yPos-2);
    display.print("Speed: ");
    display.print(speed, 1);
    display.print(" km/h");
    yPos += 12; // Increased spacing
    
    // Distance with ruler icon
    display.drawLine(6, yPos, 14, yPos, SSD1306_WHITE);
    display.drawLine(7, yPos-2, 7, yPos+2, SSD1306_WHITE);
    display.drawLine(13, yPos-2, 13, yPos+2, SSD1306_WHITE);
    display.setCursor(18, yPos-2);
    display.print("Dist: ");
    display.print(totalDisplacement, 1);
    
    // Skip calories if not enough room
  }
  

  
  display.display();
}

void loop() {
  // Check for button inputs
  checkButtons();
  
  // Handle sensor data
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
      // Pass all acceleration components to the improved function
      updateDisplacement(accelX, accelY, accelZ, currentTime);
      
      // Update live display
      displayLiveData();
      
      if (currentTime - measurementStartTime >= userDurationMillis) {
        sendResults();
      } else if (currentTime - lastStepTime >= STEP_DEBOUNCE_TIME) {
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
