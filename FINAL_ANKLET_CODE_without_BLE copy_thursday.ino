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
                                        // Lowered for better walking detection

const float STRIDE_LENGTH = 0.7;         // Average stride length in meters
                                        // Used to calculate speed and distance

const int STEP_DEBOUNCE_TIME = 400;      // Increased debounce time for better step detection

const float STEPPING_THRESHOLD = 1.0;    // Lowered threshold for walking detection

const float RUNNING_THRESHOLD = 3.0;     // Increased threshold for running detection

const float VELOCITY_DECAY = 0.95;       // Adjusted for more stable readings

const float ACCEL_BIAS = 0.001;          // Small offset to correct accelerometer bias (in m/s²)
                                        // Helps reduce drift in displacement calculations
                                        // Adjust based on your specific accelerometer's characteristics

// Constants for improved jumping detection
const float VERTICAL_JUMP_THRESHOLD = 4.0;      // Increased threshold for jump detection
const float HORIZONTAL_LIMIT_FOR_JUMP = 1.0;    // Lowered to better distinguish from walking
const float DISPLACEMENT_DECAY_RATE = 0.94;     // Faster decay for more accurate readings
const float JUMPING_DISPLACEMENT_LIMIT = 5.0;   // Adjusted limit for jump detection
const unsigned long JUMP_PATTERN_TIMEOUT = 1000; // Shorter timeout for better pattern detection
const unsigned long JUMP_RESET_INTERVAL = 2000;  // Shorter reset interval

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

// Display calibration screen with orientation info
void displayCalibration() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Header with icon
  display.fillRect(0, 0, display.width(), 16, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(32, 4);
  display.print("CALIBRATING");
  
  // Calibration icon in header
  display.drawCircle(16, 8, 6, SSD1306_BLACK);
  display.drawLine(16, 8, 16, 4, SSD1306_BLACK);
  display.drawLine(16, 8, 19, 8, SSD1306_BLACK);
  
  display.setTextColor(SSD1306_WHITE);
  
  if (!calibrationComplete) {
    // Status message
    display.setTextSize(1);
    display.setCursor(4, 20);
    display.print("Please hold still");
    
    // Progress circle animation
    int centerX = 64;
    int centerY = 38;
    int radius = 12;
    float progress = (float)calibrationCount / CALIBRATION_SAMPLES;
    int endAngle = (int)(progress * 360);
    
    // Draw progress circle
    for (int angle = 0; angle < endAngle; angle += 15) {
      float radians = angle * PI / 180.0;
      int x1 = centerX + (radius - 2) * cos(radians);
      int y1 = centerY + (radius - 2) * sin(radians);
      int x2 = centerX + (radius + 2) * cos(radians);
      int y2 = centerY + (radius + 2) * sin(radians);
      display.drawLine(x1, y1, x2, y2, SSD1306_WHITE);
    }
    
    // Progress percentage in circle
    display.setTextSize(1);
    int progress_num = (calibrationCount * 100) / CALIBRATION_SAMPLES;
    display.setCursor(centerX - (progress_num < 10 ? 3 : (progress_num < 100 ? 6 : 9)), centerY - 3);
    display.print(progress_num);
    display.print("%");
    
    // Progress bar
    display.drawRect(14, 54, 100, 6, SSD1306_WHITE);
    int progressWidth = map(calibrationCount, 0, CALIBRATION_SAMPLES, 0, 98);
    display.fillRect(15, 55, progressWidth, 4, SSD1306_WHITE);
    
  } else {
    // Show orientation with visual indicator
    display.setTextSize(1);
    display.setCursor(4, 20);
    display.print("Position:");
    
    String orientText;
    int iconX = 64;
    int iconY = 38;
    
    switch (currentOrientation) {
      case ORIENTATION_NORMAL_Z_UP:
        orientText = "Normal";
        // Up arrow
        display.fillTriangle(iconX, iconY-8, iconX-6, iconY, iconX+6, iconY, SSD1306_WHITE);
        break;
      case ORIENTATION_UPSIDE_DOWN:
        orientText = "Upside down";
        // Down arrow
        display.fillTriangle(iconX, iconY+8, iconX-6, iconY, iconX+6, iconY, SSD1306_WHITE);
        break;
      case ORIENTATION_HORIZONTAL_X_POS:
        orientText = "Horizontal +";
        // Right arrow
        display.fillTriangle(iconX+8, iconY, iconX, iconY-6, iconX, iconY+6, SSD1306_WHITE);
        break;
      case ORIENTATION_HORIZONTAL_X_NEG:
        orientText = "Horizontal -";
        // Left arrow
        display.fillTriangle(iconX-8, iconY, iconX, iconY-6, iconX, iconY+6, SSD1306_WHITE);
        break;
      case ORIENTATION_VERTICAL_Y_POS:
      case ORIENTATION_VERTICAL_Y_NEG:
        orientText = "Vertical";
        // Double-ended arrow
        display.drawLine(iconX, iconY-8, iconX, iconY+8, SSD1306_WHITE);
        display.fillTriangle(iconX, iconY-8, iconX-4, iconY-4, iconX+4, iconY-4, SSD1306_WHITE);
        display.fillTriangle(iconX, iconY+8, iconX-4, iconY+4, iconX+4, iconY+4, SSD1306_WHITE);
        break;
      default:
        orientText = "Tilted";
        // Circular arrow
        display.drawCircle(iconX, iconY, 6, SSD1306_WHITE);
        display.fillTriangle(iconX+6, iconY-2, iconX+6, iconY+2, iconX+9, iconY, SSD1306_WHITE);
        break;
    }
    
    // Display orientation text
    display.setTextSize(1);
    display.setCursor(4, 32);
    display.print(orientText);
    
    // Bottom instruction
    display.drawRect(4, 54, 120, 9, SSD1306_WHITE);
    display.setCursor(8, 55);
    display.print("Press MODE to continue");
  }
  
  display.display();
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
      Serial.print("Orientation: ");
      Serial.println(currentOrientation == ORIENTATION_NORMAL_Z_UP ? "Normal Z Up" :
                    currentOrientation == ORIENTATION_UPSIDE_DOWN ? "Upside Down" :
                    currentOrientation == ORIENTATION_HORIZONTAL_X_POS ? "Horizontal X Pos" :
                    currentOrientation == ORIENTATION_HORIZONTAL_X_NEG ? "Horizontal X Neg" :
                    currentOrientation == ORIENTATION_VERTICAL_Y_POS ? "Vertical Y Pos" :
                    currentOrientation == ORIENTATION_VERTICAL_Y_NEG ? "Vertical Y Neg" : "Tilted");
    }
    displayCalibration();
  }
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
  
  // Current statistics in box - adjusted to use more space
  display.drawRect(0, 30, display.width(), 12, SSD1306_WHITE);
  display.setCursor(4, 32);
  
  // Show appropriate stats based on movement type
  String moveType = determineMovementType();
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
    
    // First check for jumping - it has priority
    if (hasJumpingPattern && verticalAccelMax > VERTICAL_JUMP_THRESHOLD) {
        return "Jumping";
    }
    
    // Calculate average speed for the last period
    float currentSpeed = calculateSpeed(stepCount, millis() - measurementStartTime);
    
    // Running detection with stricter conditions
    if (totalDisplacement > RUNNING_THRESHOLD && currentSpeed > 8.0) {
        return "Running";
    }
    
    // Default to walking for all other movement
    return "Walking";
}

// Modified updateDisplacement to handle different orientations
void updateDisplacement(float accelX, float accelY, float accelZ, unsigned long currentTime) {
  // Remove gravity component based on orientation
  float adjustedX = accelX - (calibrationX * gravityVector[0]);
  float adjustedY = accelY - (calibrationY * gravityVector[1]);
  float adjustedZ = accelZ - (calibrationZ * gravityVector[2]);
  
  float dt = (currentTime - lastSampleTime) / 1000.0;
  if (lastSampleTime == 0) dt = 0;
  
  if (dt > 0 && dt < 0.1) {
    // Track maximum acceleration based on orientation
    float verticalComponent;
    
    switch (currentOrientation) {
      case ORIENTATION_NORMAL_Z_UP:
      case ORIENTATION_UPSIDE_DOWN:
        verticalComponent = abs(adjustedZ);
        break;
      case ORIENTATION_HORIZONTAL_X_POS:
      case ORIENTATION_HORIZONTAL_X_NEG:
        verticalComponent = abs(adjustedX);
        break;
      case ORIENTATION_VERTICAL_Y_POS:
      case ORIENTATION_VERTICAL_Y_NEG:
        verticalComponent = abs(adjustedY);
        break;
      case ORIENTATION_TILTED:
      default:
        // For tilted orientation, use the magnitude of all components
        verticalComponent = sqrt(adjustedX * adjustedX + adjustedY * adjustedY + adjustedZ * adjustedZ);
        break;
    }
    
    if (verticalComponent > verticalAccelMax) {
      verticalAccelMax = verticalComponent;
    }
    
    // Calculate horizontal components based on orientation
    float horizontalAccel;
    
    switch (currentOrientation) {
      case ORIENTATION_NORMAL_Z_UP:
      case ORIENTATION_UPSIDE_DOWN:
        horizontalAccel = sqrt(adjustedX * adjustedX + adjustedY * adjustedY);
        break;
      case ORIENTATION_HORIZONTAL_X_POS:
      case ORIENTATION_HORIZONTAL_X_NEG:
        horizontalAccel = sqrt(adjustedY * adjustedY + adjustedZ * adjustedZ);
        break;
      case ORIENTATION_VERTICAL_Y_POS:
      case ORIENTATION_VERTICAL_Y_NEG:
        horizontalAccel = sqrt(adjustedX * adjustedX + adjustedZ * adjustedZ);
        break;
      case ORIENTATION_TILTED:
      default:
        // For tilted, use the planar component perpendicular to gravity vector
        float gx = gravityVector[0], gy = gravityVector[1], gz = gravityVector[2];
        float dot = adjustedX * gx + adjustedY * gy + adjustedZ * gz;
        float projX = dot * gx;
        float projY = dot * gy;
        float projZ = dot * gz;
        float perpX = adjustedX - projX;
        float perpY = adjustedY - projY;
        float perpZ = adjustedZ - projZ;
        horizontalAccel = sqrt(perpX * perpX + perpY * perpY + perpZ * perpZ);
        break;
    }
    
    // Apply bias correction
    horizontalAccel = max(0.0f, horizontalAccel - ACCEL_BIAS);
    
    // Update velocity with orientation-adjusted acceleration
    velocityX = (velocityX + horizontalAccel * dt) * VELOCITY_DECAY;
    velocityY = 0; // We'll use single-axis velocity for simplicity
    
    // Update displacement
    if (abs(velocityX) > 0) {
      displacementX += velocityX * dt;
      totalDisplacement = abs(displacementX); // Use absolute value for total displacement
    }
    
    // Jumping detection based on orientation
    bool currentlyJumping = false;
    if (verticalComponent > VERTICAL_JUMP_THRESHOLD) {
      currentlyJumping = true;
      lastJumpDetectTime = currentTime;
      hasJumpingPattern = true;
      
      if (jumpPatternDuration == 0) {
        jumpPatternDuration = currentTime;
      }
      if (consecutiveJumpingTime == 0) {
        consecutiveJumpingTime = currentTime;
      }
    }
    
    // Reset jumping state gradually
    if (currentTime - lastJumpDetectTime > JUMP_PATTERN_TIMEOUT) {
      if (currentTime - lastJumpDetectTime > JUMP_PATTERN_TIMEOUT * 3) {
        jumpPatternDuration = 0;
        hasJumpingPattern = false;
      }
    }
  }
  
  lastSampleTime = currentTime;
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

void sendResults() {
  currentState = STATE_RESULTS;
  
  float speed = calculateSpeed(stepCount, userDurationMillis);
  String type = determineMovementType();
  float MET = calculateMET(type);
  float calories = calculateCalories(MET, userWeight, userDurationMinutes);
  
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
    float jumpsPerMinute = stepCount / elapsedTimeMinutes;
    
    if (jumpsPerMinute < 80) intensityLevel = "Light";
    else if (jumpsPerMinute < 100) intensityLevel = "Moderate";
    else if (jumpsPerMinute < 120) intensityLevel = "Vigorous";
    else intensityLevel = "High";
  } else {
    if (speed < 4.0) intensityLevel = "Light";
    else if (speed < 5.5) intensityLevel = "Moderate";
    else intensityLevel = "Brisk";
  }

  Serial.println("--- Final Results ---");
  Serial.println("Activity: " + type);
  Serial.println("Intensity: " + intensityLevel);
  Serial.println("MET Value: " + String(MET, 1));

  if (type == "Jumping") {
    unsigned long elapsedTimeMinutes = (millis() - measurementStartTime) / 60000.0;
    if (elapsedTimeMinutes < 0.1) elapsedTimeMinutes = 0.1;
    float jumpsPerMinute = stepCount / elapsedTimeMinutes;
    
    Serial.println("Jumps: " + String(stepCount));
    Serial.println("Jumps/min: " + String(jumpsPerMinute, 1));
    Serial.println("Calories: " + String(calories, 2) + " kcal");
  } else if (type == "Running") {
    Serial.println("Speed: " + String(speed, 2) + " km/h");
    Serial.println("Steps: " + String(stepCount));
    Serial.println("Calories: " + String(calories, 2) + " kcal");
  } else {
    Serial.println("Steps: " + String(stepCount));
    Serial.println("Speed: " + String(speed, 2) + " km/h");
    Serial.println("Displacement: " + String(totalDisplacement, 2) + " m");
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
  display.print(" (");
  display.print(intensityLevel);
  display.print(")");
  
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
  
  // Add MET value display
  display.setCursor(18, yPos-2);
  display.print("MET: ");
  display.print(MET, 1);
  yPos += 8;
  
  if (type == "Jumping") {
    // Jumps count
    display.drawCircle(10, yPos, 4, SSD1306_WHITE);
    display.drawLine(10, yPos-2, 10, yPos+2, SSD1306_WHITE);
    display.setCursor(18, yPos-2);
    display.print("Jumps: ");
    display.print(stepCount);
    yPos += 8;
    
    // Jumps per minute
    unsigned long elapsedTimeMinutes = (millis() - measurementStartTime) / 60000.0;
    if (elapsedTimeMinutes < 0.1) elapsedTimeMinutes = 0.1;
    float jumpsPerMinute = stepCount / elapsedTimeMinutes;
    display.setCursor(18, yPos-2);
    display.print("Rate: ");
    display.print(jumpsPerMinute, 1);
    display.print("/min");
    yPos += 8;
    
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
    yPos += 8;
    
    // Steps with footprint icon
    display.drawCircle(9, yPos-1, 2, SSD1306_WHITE);
    display.drawCircle(12, yPos+1, 2, SSD1306_WHITE);
    display.setCursor(18, yPos-2);
    display.print("Steps: ");
    display.print(stepCount);
    yPos += 8;
    
    // Calories with fire icon
    display.drawLine(8, yPos, 10, yPos-4, SSD1306_WHITE);
    display.drawLine(10, yPos-4, 12, yPos, SSD1306_WHITE);
    display.drawLine(10, yPos, 10, yPos-3, SSD1306_WHITE);
    display.setCursor(18, yPos-2);
    display.print("Cal: ");
    display.print(calories, 1);
    display.print(" kcal");
  } else { // Walking
    // Steps with footprint icon
    display.drawCircle(9, yPos-1, 2, SSD1306_WHITE);
    display.drawCircle(12, yPos+1, 2, SSD1306_WHITE);
    display.setCursor(18, yPos-2);
    display.print("Steps: ");
    display.print(stepCount);
    yPos += 8;
    
    // Speed with speedometer icon
    display.drawCircle(10, yPos, 4, SSD1306_WHITE);
    display.drawLine(10, yPos, 10+3, yPos-2, SSD1306_WHITE);
    display.setCursor(18, yPos-2);
    display.print("Speed: ");
    display.print(speed, 1);
    display.print(" km/h");
    yPos += 8;
    
    // Distance with ruler icon
    display.drawLine(6, yPos, 14, yPos, SSD1306_WHITE);
    display.drawLine(7, yPos-2, 7, yPos+2, SSD1306_WHITE);
    display.drawLine(13, yPos-2, 13, yPos+2, SSD1306_WHITE);
    display.setCursor(18, yPos-2);
    display.print("Dist: ");
    display.print(totalDisplacement, 1);
    display.print(" m");
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

    // Handle calibration if in calibration state
    if (currentState == STATE_CALIBRATION) {
      updateCalibration(accelX, accelY, accelZ);
      return;  // Skip other processing during calibration
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
