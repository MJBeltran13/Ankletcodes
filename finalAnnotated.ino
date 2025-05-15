// required libraries
#include <Arduino_BMI270_BMM150.h>  // Arduino Nano 33 BLE Sense Rev2 sensors
#include <Wire.h>                   // I2C communication
#include <Adafruit_GFX.h>           // Adafruit displays
#include <Adafruit_SSD1306.h>       // Specific library for SSD1306 OLED displays
#include <mbed.h>                   // Mbed OS core libraries
#include <rtos.h>                   // Part of Mbed OS

// screen resolution - 128x64 pixels
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1 // No reset pin connected 

// Adafruit_SSD1306 object named display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// RTOS (real-time operating system) objects
rtos::Thread thread_buttons;  // Monitoring pushbutton inputs
rtos::Thread thread_sensors;  // Reading motion sensors
rtos::Thread thread_display;  // Updating OLED
rtos::Mutex displayMutex;     // Only one thread updates the OLED
rtos::Mutex sensorDataMutex;  // Thread-safe access to shared sensor data

// Thread timing constants
const uint32_t BUTTONS_THREAD_DELAY = 50;   // Check buttons every 50 ms
const uint32_t SENSORS_THREAD_DELAY = 10;   // Process sensors every 10 ms
const uint32_t DISPLAY_THREAD_DELAY = 100;  // Update display every 100 ms

// Button pins
#define BUTTON_UP 2     // Increase values / Select
#define BUTTON_DOWN 3   // Decrease values 
#define BUTTON_MODE 4   // Mode switch / Confirm

// Button states

// Current debounced state
bool currentButtonUp = HIGH;
bool currentButtonDown = HIGH;
bool currentButtonMode = HIGH;

// Previous debounced state, detects state changes
bool lastButtonUp = HIGH;
bool lastButtonDown = HIGH;
bool lastButtonMode = HIGH;

// Raw reading - digitalRead() value
bool rawButtonUp = HIGH;
bool rawButtonDown = HIGH;
bool rawButtonMode = HIGH;

// Last time this button changed
unsigned long lastButtonUpTime = 0;
unsigned long lastButtonDownTime = 0;
unsigned long lastButtonModeTime = 0;
unsigned long lastButtonActionTime = 0; // Last time an action was performed

const unsigned long BUTTON_DEBOUNCE_TIME = 100;   // 100 ms delay, longer debounce time for stability
const unsigned long REPEAT_FIRST_TIME = 800;      // First repeated action fires after 800 ms. Longer first repeat delay
const unsigned long REPEAT_TIME = 300;            // Subsequent repeat time - every 300 ms as long as it's still held

// Device states
enum DeviceState {
  STATE_WEIGHT_SETUP,   // Set weight using buttons
  STATE_DURATION_SETUP, // Set duration using buttons
  STATE_CALIBRATION,    // Pre-tracking calibration
  STATE_READY,          // Ready to begin
  STATE_TRACKING,       // Real-time data collection and motion analysis
  STATE_RESULTS         // Summary of results
};

DeviceState currentState = STATE_WEIGHT_SETUP;

// Movement detection thresholds
const float STEP_THRESHOLD = 2.0;        // Minimum acceleration to detect a step (in m/s²)
const float STRIDE_LENGTH = 0.6;         // Average stride length in meters
const int STEP_DEBOUNCE_TIME = 400;      // Minimum time (ms) between valid steps
const float VELOCITY_DECAY = 0.95;       // Smooths out estimated velocity by decaying it gradually
const float ACCEL_BIAS = 0.001;          // Small offset to correct accelerometer bias (in m/s²)

// Constants for improved jumping detection
const float VERTICAL_JUMP_THRESHOLD = 2.5;        // Minimum upward vertical accel (in m/s²) required to start detecting a jump
const float HORIZONTAL_LIMIT_FOR_JUMP = 2.0;      // Maximum horizontal accel allowed during a jump
const float DISPLACEMENT_DECAY_RATE = 0.94;       // Controls how fast calculated displacement decays over time. Faster decay for more accurate readings
const float JUMPING_DISPLACEMENT_LIMIT = 5.0;     // Maximum acceptable vertical displacement to qualify as a jump
const unsigned long JUMP_PATTERN_TIMEOUT = 2000;  // Time (ms) to complete a jump pattern (launch → airborne → land)
const unsigned long JUMP_RESET_INTERVAL = 3000;   // Time after which jump detection variables reset
const unsigned long JUMP_DEBOUNCE_TIME = 200;     // Minimum time between two valid jump detections

// Constants for jump detection
const float JUMP_START_THRESHOLD = 2.5;   // Acceleration threshold to initiate a jump detection 
const float JUMP_LAND_THRESHOLD = 0.1;    // Low acceleration threshold for landing detection
const float JUMP_MIN_TIME = 100;          // Minimum time between takeoff and landing
const float JUMP_MAX_TIME = 800;          // Maximum time between jump start and landing

unsigned long lastStepTime = 0;           // Timestamp of the last detected step
unsigned long measurementStartTime = 0;   // Beginning of the tracking session
unsigned long lastSampleTime = 0;         // Last time sensor data was processed
unsigned long userDurationMillis = 60000; // User-defined tracking duration, in milliseconds
unsigned long lastDisplayUpdateTime = 0;  // Timestamp of the last display refresh

const unsigned long DISPLAY_UPDATE_INTERVAL = 500;  // Update display every 500 ms

int stepCount = 0;                // Tracks the total number of detected steps
bool wasInStep = false;           // Tracks whether the last cycle was inside a valid step
bool measurementStarted = false;  // Indicates whether the user has begun the exercise tracking phase
bool measurementComplete = false; // Marks the end of a tracking session

float velocityX = 0, velocityY = 0;         // Estimate the horizontal velocity in the X and Y directions
float displacementX = 0, displacementY = 0; // Estimates of horizontal displacement
float totalDisplacement = 0;                // Combined measure of total horizontal displacement

int userWeight = 60;          // User’s weight in kilograms, default is 60 kg
int userDurationMinutes = 1;  // User's intended duration (min) for the tracking session, default is 1 minute

// Add jump detection tracking
bool hasJumpingPattern = false;           // Indicates whether a potential jumping pattern is currently being detected
float verticalAccelMax = 0;               // Stores the maximum vertical acceleration seen during a potential jump
unsigned long jumpPatternDuration = 0;    // Measures how long the current jumping pattern has been active
unsigned long lastJumpDetectTime = 0;     // Timestamp of the last valid jump detection
unsigned long consecutiveJumpingTime = 0; // Tracks how long continuous jumping has occurred

// Calibration variables
float calibrationX = 0, calibrationY = 0, calibrationZ = 0;           // Stores the average offsets (biases) for the accelerometer in the X, Y, and Z axes.
float calibrationSumX = 0, calibrationSumY = 0, calibrationSumZ = 0;  // Used to accumulate sensor readings over several samples for averaging
int calibrationCount = 0;
float gyroCalibX = 0, gyroCalibY = 0, gyroCalibZ = 0;                 // Same as calibrationSum, but for gyro
float gyroSumX = 0, gyroSumY = 0, gyroSumZ = 0;
const int CALIBRATION_SAMPLES = 50;                                   // Number of samples used to compute the average bias
bool calibrationComplete = false;                                     // Set to true when calibration is done.

// Orientation tracking
enum DeviceOrientation {
  ORIENTATION_NORMAL_Z_UP,      // Device is flat with Z-axis pointing up (screen up)
  ORIENTATION_UPSIDE_DOWN,      // Device is flat with Z-axis pointing down (screen down)
  ORIENTATION_HORIZONTAL_X_POS, // Device is on its side, X-axis up
  ORIENTATION_HORIZONTAL_X_NEG, // Device is on its side, X-axis down
  ORIENTATION_VERTICAL_Y_POS,   // Device is vertical, Y-axis up
  ORIENTATION_VERTICAL_Y_NEG,   // Device is vertical, Y-axis down
  ORIENTATION_TILTED            // Catch-all for angled or uncertain positions
};

DeviceOrientation currentOrientation = ORIENTATION_TILTED;

float gravityVector[3] = {0, 0, 0}; // Stores gravity direction - from the accelerometer

// Variables for better jump state management
unsigned long lastJumpTime = 0; // Stores the timestamp of the last confirmed jump
bool isInJumpState = false;     // True when the system has detected jump initiation but is waiting for landing
int consecutiveJumps = 0;       // Tracks the number of sequential jumps without exceeding JUMP_RESET_INTERVAL

// Jump state tracking
bool inJumpTakeoff = false;       // Potential jump takeoff has been detected but the jump hasn’t yet completed
bool inJumpLanding = false;       // System is waiting for landing after detecting a takeoff
unsigned long jumpStartTime = 0;  // Timestamp of when takeoff occurred
float maxJumpForce = 0;           // Stores the peak acceleration during the takeoff phase

int jumpCount = 0;    // Jump counter for the total number of confirmed jumps. Increments only after a full jump cycle (takeoff + airtime + landing)

// Thread function declarations
void buttonsThreadFunc();
void sensorsThreadFunc();
void displayThreadFunc();

// Function to determine device orientation from calibration data
DeviceOrientation determineOrientation(float accX, float accY, float accZ) {
  
  // Find the axis with the strongest gravity component
  float absX = abs(accX);
  float absY = abs(accY);
  float absZ = abs(accZ);
  
  const float GRAVITY_THRESHOLD = 0.8; // Minimum gravity component to determine orientation
  
  // Z-axis orientations
  // Upright or upside down
  if (absZ > absX && absZ > absY && absZ > GRAVITY_THRESHOLD) {
    gravityVector[0] = 0;
    gravityVector[1] = 0;
    gravityVector[2] = (accZ > 0) ? 1.0 : -1.0;
    return (accZ > 0) ? ORIENTATION_NORMAL_Z_UP : ORIENTATION_UPSIDE_DOWN;
  } 
  // X-axis orientations
  // Left or right
  else if (absX > absY && absX > absZ && absX > GRAVITY_THRESHOLD) {
    gravityVector[0] = (accX > 0) ? 1.0 : -1.0;
    gravityVector[1] = 0;
    gravityVector[2] = 0;
    return (accX > 0) ? ORIENTATION_HORIZONTAL_X_POS : ORIENTATION_HORIZONTAL_X_NEG;
  } 
  // Y-axis orientations
  // Top or bottom
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
  
  // Setup button pins - inputs with internal pull-up resistors
  // HIGH by default and LOW when pressed
  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  pinMode(BUTTON_MODE, INPUT_PULLUP);
  
  // Initialize display first for status updates
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    // Print if initialization fails
    Serial.println("SSD1306 allocation failed");
    while (1);
  }
  
  // Show (custom) splash screen
  displaySplashScreen();
  delay(2000);  // Show splash screen for 2 seconds
  
  displayStatus("Starting...", "Initializing sensors");
  
  // Initialize IMU
  if (!IMU.begin()) {
    // Print if initialization fails
    displayStatus("ERROR", "IMU initialization failed");
    while (1);
  }
  
  // Start with weight setup
  displayWeightSetup();
  
  // Start RTOS threads
  thread_buttons.start(buttonsThreadFunc);
  thread_sensors.start(sensorsThreadFunc);
  thread_display.start(displayThreadFunc);
  
  Serial.println("Anklet Ready with RTOS")  // Fully initialized
}

// Display splash screen with logo
void displaySplashScreen() {
  display.clearDisplay();
  
  // Draw a border around the screen - rectangle frame
  display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
  
  // Draw the title
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(15, 5);
  display.print("ANKLET");
  
  // Draw a (horizontal) separator line
  display.drawFastHLine(10, 25, 108, SSD1306_WHITE);
  
  // Draw the subtitle
  display.setTextSize(1);
  display.setCursor(10, 30);
  display.print("SmartMotion Tracker");
  
  // Draw simple shoe/anklet symbol
  display.drawRoundRect(45, 40, 38, 18, 5, SSD1306_WHITE);
  display.drawLine(45, 50, 30, 45, SSD1306_WHITE);
  display.drawLine(83, 50, 98, 45, SSD1306_WHITE);
  
  display.display();  // Refreshes the screen and render all the drawings and text 
}

// Proper button debouncing and handling
void checkButtons() {
  unsigned long currentMillis = millis(); // Stores the current time in ms since the Arduino started
  
  // Read current (raw electrical state) button states
  bool readingUp = digitalRead(BUTTON_UP);
  bool readingDown = digitalRead(BUTTON_DOWN);
  bool readingMode = digitalRead(BUTTON_MODE);
  
  // Store current debounced button states before updating - to detect transitions like HIGH → LOW
  lastButtonUp = currentButtonUp;
  lastButtonDown = currentButtonDown; 
  lastButtonMode = currentButtonMode;
  
  // Debouncing - if the raw reading changes, update the last change time

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
    handleModeButton(); // Changes modes or states
  }
}

// Handle UP button press based on current state
void handleUpButton() {
  Serial.println("UP button pressed");                        // Debug output
  
  switch (currentState) {
    case STATE_WEIGHT_SETUP:
      userWeight += 1;                                        // Increment of 1
      if (userWeight > 150) userWeight = 150;                 // Maximum weight is 150 kg
      displayWeightSetup();
      break;
      
    case STATE_DURATION_SETUP:
      userDurationMinutes += 1;                               // Increment of 1
      if (userDurationMinutes > 60) userDurationMinutes = 60; // Maximum duration is 60 min
      userDurationMillis = userDurationMinutes * 60000UL;
      displayDurationSetup();
      break;
      
    case STATE_RESULTS:
      resetMeasurement();                                     // Reset and go back to ready state
      currentState = STATE_READY;                             // user can start again
      displayStatus("Ready", "Press MODE to start");
      break;
  }
}

// Handle DOWN button press based on current state
void handleDownButton() {
  Serial.println("DOWN button pressed");                      // Debug output
  
  switch (currentState) {
    case STATE_WEIGHT_SETUP:
      userWeight -= 5;                                        // Decrease by 5 kg
      if (userWeight < 30) userWeight = 30;                   // Minimum weight is 30 kg
      displayWeightSetup();
      break;
      
    case STATE_DURATION_SETUP:
      userDurationMinutes -= 1;                               // Decrease by 1 min
      if (userDurationMinutes < 1) userDurationMinutes = 1;   // Minimum duration is 1 min
      userDurationMillis = userDurationMinutes * 60000UL;
      displayDurationSetup();
      break;
  }
}

// Handle MODE button press based on current state
void handleModeButton() {
  Serial.println("MODE button pressed");                      // Debug output
  
  switch (currentState) {
    case STATE_WEIGHT_SETUP:
      currentState = STATE_DURATION_SETUP;                    // Switch to STATE_DURATION_SETUP
      displayDurationSetup();
      break;
      
    case STATE_DURATION_SETUP:
      currentState = STATE_CALIBRATION;                       // Go to calibration instead of ready
      
      // Reset calibration
      calibrationCount = 0;
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
        currentState = STATE_READY;                           // Switch to STATE_READY
        displayStatus("Ready", "Press MODE to start");
      }
      break;
      
    case STATE_READY:
      startMeasurement();                                     // Begin the activity tracking process.
      break;
      
    case STATE_TRACKING:
      sendResults();                                          // When tracking is ongoing, pressing MODE sends or shows the results
      break;
      
    case STATE_RESULTS:
      resetMeasurement();
      currentState = STATE_READY;                             // Back to STATE_READY
      displayStatus("Ready", "Press MODE to start");
      break;
  }
}

// Display weight setup screen
void displayWeightSetup() {
  displayMutex.lock();  // Locks the display so no other thread modifies the OLED
  
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Header with (rectangle) border
  display.drawRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(26, 2);
  display.print("WEIGHT SETUP");
  
  // Weight value - centered
  display.setTextSize(2);
  display.setCursor(46, 20);
  display.print(userWeight);
  display.setTextSize(1);
  display.print(" kg");
  
  // Draw a (horizontal) separator line
  display.drawFastHLine(0, 43, 128, SSD1306_WHITE);
  display.setCursor(20, 45);
  display.print("UP/DOWN: Change");
  display.setCursor(35, 55);
  display.print("MODE:Next");
  
  display.display();
  
  displayMutex.unlock();  // Releases the display so other threads can now use it
}

// Display duration setup screen
void displayDurationSetup() {
  displayMutex.lock();  // Locks the display so no other thread modifies the OLED
  
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Header with (rectangle) border
  display.drawRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(20, 2);
  display.print("DURATION SETUP");
  
  // Duration value - centered
  display.setTextSize(2);
  display.setCursor(46, 20);
  display.print(userDurationMinutes);
  display.setTextSize(1);
  display.print(" min");
  
  // Draw a (horizontal) separator line
  display.drawFastHLine(0, 43, 128, SSD1306_WHITE);
  display.setCursor(20, 45);
  display.print("UP/DOWN: Change");
  display.setCursor(35, 55);
  display.print("MODE:Next");
  
  display.display();
  
  displayMutex.unlock();  // Releases the display so other threads can now use it
}

// Display calibration screen
void displayCalibration() {
  displayMutex.lock();  // Locks the display so no other thread modifies the OLED
  
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Header with (rectangle) border
  display.drawRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(30, 2);
  display.print("CALIBRATING");
  
  if (!calibrationComplete) {
    display.setCursor(14, 18);
    display.print("Please hold still");
    
    // Progress percentage with outline
    display.setCursor(55, 30);
    int progress_num = (calibrationCount * 100) / CALIBRATION_SAMPLES;        // Calculates the calibration progress percentage
    display.print(progress_num);
    display.print("%");
    
    // Progress bar
    display.drawRect(14, 40, 100, 8, SSD1306_WHITE);
    int progressWidth = map(calibrationCount, 0, CALIBRATION_SAMPLES, 0, 98); // map() function adjusts the width of the filled bar based on the current calibration progress
    display.fillRect(15, 41, progressWidth, 6, SSD1306_WHITE);
  } 
  
  else {
    display.setCursor(5, 20);
    display.print("Calibration complete");
    display.drawLine(8, 28, 120, 28, SSD1306_WHITE);  // Draw a (horizontal) separator line
    display.setCursor(35, 35);
    display.print("Press MODE");
    display.setCursor(32, 45);
    display.print("to continue");
  }
  
  display.display();
  
  displayMutex.unlock();  // Releases the display so other threads can now use it
}

// Helper function to display status messages
void displayStatus(String line1, String line2) {
  displayMutex.lock();  // Locks the display so no other thread modifies the OLED
  
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Header with (rectangle) border
  display.drawRect(0, 0, 128, 16, SSD1306_WHITE);
  display.setTextSize(1);
  
  // Center text in header box
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(line1, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((128 - w) / 2, 4);
  display.print(line1);
  
  // Message in middle of screen
  display.getTextBounds(line2, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((128 - w) / 2, 30);
  display.print(line2);
  
  // Add decorative elements
  display.drawRect(10, 25, 108, 20, SSD1306_WHITE);
  display.drawFastHLine(0, 55, 128, SSD1306_WHITE);
  
  display.display();
  
  displayMutex.unlock();  // Releases the display so other threads can now use it
}

// Display live measurement data
void displayLiveData() {
  // Don't lock display mutex if we're not going to update
  if (millis() - lastDisplayUpdateTime < DISPLAY_UPDATE_INTERVAL) return; // Display is updated only at a certain interval, preventing excessive updates.
  
  lastDisplayUpdateTime = millis(); // Updates the time of the last display update to the current time (millis())
  
  displayMutex.lock();
  sensorDataMutex.lock(); // Lock while reading shared data
  
  // Calculate Elapsed and Remaining Time
  unsigned long elapsedTime = millis() - measurementStartTime;
  unsigned long remainingTime = userDurationMillis > elapsedTime ? userDurationMillis - elapsedTime : 0;
  int remainingSeconds = remainingTime / 1000;
  int remainingMinutes = remainingSeconds / 60;
  remainingSeconds %= 60;
  
  // Local copies of data that might be modified by other threads
  int localStepCount = stepCount;
  int localJumpCount = jumpCount;
  String movementType = determineMovementType();
  
  sensorDataMutex.unlock(); // Unlock as soon as we've read the data
  
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Header with border
  display.drawRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(38, 2);
  display.print("TRACKING");
  
  // Activity border
  display.drawRect(0, 42, 128, 12, SSD1306_WHITE);
  display.setCursor(2, 44);
  display.print(movementType);
  
  // Steps and Jumps on the left side
  display.setCursor(5, 16);
  display.print("Steps:");
  display.print(localStepCount);
  
  display.setCursor(5, 31);
  display.print("Jumps:");
  display.print(localJumpCount);
  
  // Time remaining on the right side
  display.setCursor(75, 16);
  display.print("Time:");
  
  // Time value 
  char timeBuffer[6];
  sprintf(timeBuffer, "%02d:%02d", remainingMinutes, remainingSeconds);
  display.setCursor(75, 26);
  display.print(timeBuffer);
  
  // Progress bar at bottom
  display.drawRect(0, 60, 128, 4, SSD1306_WHITE);
  int progressWidth = map(elapsedTime, 0, userDurationMillis, 0, 126);
  display.fillRect(1, 61, progressWidth, 2, SSD1306_WHITE);
  
  display.display();
  
  displayMutex.unlock();
}

// Start measurement manually
void startMeasurement() {
  measurementStarted = true;          // Measurement has been manually started
  measurementStartTime = millis();    // Sets the start time for the measurement to the current time
  measurementComplete = false;        // Sets the measurement as incomplete at the start
  
  currentState = STATE_TRACKING;      // Switch to STATE_TRACKING
  
  Serial.println("Measurement started manually...");
  displayStatus("Measurement", "Started!");
  delay(500); // Show start message briefly
}

// Reset measurement values and state - to prepare the system for a new tracking session
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

// Speed calculation
float calculateSpeed(int steps, unsigned long timeMs) {
  float timeMinutes = timeMs / 60000.0;                     // Convert milliseconds to minutes
  float stepsPerMinute = steps / timeMinutes;               // Steps per minute
  return (stepsPerMinute * STRIDE_LENGTH * 60.0) / 1000.0;  // Speed formula using average stride length
}

// Function to detect and process jumps
bool processJumpDetection(float verticalAccel, unsigned long currentTime) {
    // Debug output
    if (abs(verticalAccel) > 1.0) {
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
    
    // Detect landing when in jump state and vertical acceleration goes less than landing threshold
    if (isInJumpState && verticalAccel < JUMP_LAND_THRESHOLD) {
        Serial.println("Jump landing detected!");
        Serial.print("Jump force: ");
        Serial.println(verticalAccel);
        
        // Reset states
        isInJumpState = false;
        lastJumpTime = currentTime;
        maxJumpForce = 0;
        jumpCount++; // Increment jump counter
        
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
    
    float dt = (currentTime - lastSampleTime) / 1000.0; // Calculates time difference between the current and last sample
    if (lastSampleTime == 0) dt = 0;
    
    if (dt > 0 && dt < 0.1) { // Only process updates if the time step is positive and less than 100 ms
        // Calculate vertical component based on orientation
        float verticalAccel;
        
        switch (currentOrientation) { // Selects the vertical acceleration component based on the current orientation
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
            float horizontalAccel = sqrt( // Calculates the magnitude of horizontal movement
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
    
    lastSampleTime = currentTime; // Saves current time to calculate dt on the next loop
}

// Modified determineMovementType function
String determineMovementType() {
    // Calculate current speed in km/h
    float currentSpeed = calculateSpeed(stepCount, millis() - measurementStartTime);
    
    // Running if speed is more than 5 km/h
    if (currentSpeed > 5.0) {
        return "Running";
    }
    
    // Jumping if 3 or more jumps have been detected
    if (jumpCount >= 3) {
        return "Jumping";
    }
    
    // Default to walking
    return "Walking";
}

// MET value based on the movement
float calculateMET(String type) {
  if (type == "Running") {
    float speed = calculateSpeed(stepCount, userDurationMillis);
    
    // MET values based on running speed (kph)
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
    float jumpsPerMinute = jumpCount / elapsedTimeMinutes;
    
    // MET values based on jumps per minute for jumping activity
    if (jumpsPerMinute < 80) return 8.3;
    else if (jumpsPerMinute < 100) return 10.0;
    else if (jumpsPerMinute < 120) return 11.8;
    else return 12.3; // 120+ jumps per minute
  }
  
  // Default walking MET
  return 3.5;
}

// Calorie burn calculation
float calculateCalories(float MET, int weightKg, int durationMinutes) {
  // Formula: MET * 3.5 * weightKg / 200.0 * durationMinutes
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
  // Calibrates if in calibration state
  if (currentState == STATE_CALIBRATION && !calibrationComplete) {
    // Reads gyroscope values only if they are available
    float gyroX, gyroY, gyroZ;
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gyroX, gyroY, gyroZ);
      gyroSumX += gyroX;
      gyroSumY += gyroY;
      gyroSumZ += gyroZ;
    }
    
    // Adds current acceleration values to the sum
    calibrationSumX += accelX;
    calibrationSumY += accelY;
    calibrationSumZ += accelZ;
    calibrationCount++;
    
    // Check if enough samples are collected
    if (calibrationCount >= CALIBRATION_SAMPLES) {
      
      // Calculates the average accelerometer and gyroscope values
      calibrationX = calibrationSumX / CALIBRATION_SAMPLES;
      calibrationY = calibrationSumY / CALIBRATION_SAMPLES;
      calibrationZ = calibrationSumZ / CALIBRATION_SAMPLES;
      
      gyroCalibX = gyroSumX / CALIBRATION_SAMPLES;
      gyroCalibY = gyroSumY / CALIBRATION_SAMPLES;
      gyroCalibZ = gyroSumZ / CALIBRATION_SAMPLES;
      
      // Determine device orientation
      currentOrientation = determineOrientation(calibrationX, calibrationY, calibrationZ);
      
      calibrationComplete = true; // Calibration is finished so the system can proceed
      Serial.println("Calibration complete:");
      Serial.print("Accel X: "); Serial.println(calibrationX);
      Serial.print("Accel Y: "); Serial.println(calibrationY);
      Serial.print("Accel Z: "); Serial.println(calibrationZ);
    }
    displayCalibration();
  }
}

// Display and send results
void sendResults() {
  sensorDataMutex.lock();
  currentState = STATE_RESULTS; // Sets the current application state to STATE_RESULTS
  
  // Make local copies of shared data
  int localStepCount = stepCount;
  int localJumpCount = jumpCount;
  unsigned long localMeasurementStartTime = measurementStartTime;
  sensorDataMutex.unlock();
  
  // Calculates speed, activity type, MET value, and calories burned
  float speed = calculateSpeed(localStepCount, userDurationMillis);
  String type = determineMovementType();
  float MET = calculateMET(type);
  float calories = calculateCalories(MET, userWeight, userDurationMinutes);
  
  // Calculate intensity metrics
  String intensityLevel = ""; // Empty intensity descriptor
  
  if (type == "Running") {
    if (speed < 8.0) intensityLevel = "Light";
    else if (speed < 11.0) intensityLevel = "Moderate";
    else if (speed < 14.0) intensityLevel = "Vigorous";
    else intensityLevel = "High";
  } 
  
  else if (type == "Jumping") {
    unsigned long elapsedTimeMinutes = (millis() - localMeasurementStartTime) / 60000.0;
    if (elapsedTimeMinutes < 0.1) elapsedTimeMinutes = 0.1;
    float jumpsPerMinute = localJumpCount / (float)elapsedTimeMinutes;
    
    if (jumpsPerMinute < 80) intensityLevel = "Light";
    else if (jumpsPerMinute < 100) intensityLevel = "Moderate";
    else if (jumpsPerMinute < 120) intensityLevel = "Vigorous";
    else intensityLevel = "High";
  } 
  
  else {  // Walking
    if (speed < 4.0) intensityLevel = "Light";
    else if (speed < 5.5) intensityLevel = "Moderate";
    else intensityLevel = "Brisk";
  }

  // Output to Serial Monitor
  Serial.println("--- Final Results ---");
  Serial.println("Activity: " + type);
  Serial.println("Intensity: " + intensityLevel);
  Serial.println("MET Value: " + String(MET, 1));

  if (type == "Jumping") {
    unsigned long elapsedTimeMinutes = (millis() - localMeasurementStartTime) / 60000.0;
    if (elapsedTimeMinutes < 0.1) elapsedTimeMinutes = 0.1;
    float jumpsPerMinute = localJumpCount / (float)elapsedTimeMinutes;
    
    Serial.println("Jumps: " + String(localJumpCount));
    Serial.println("Jumps/min: " + String(jumpsPerMinute, 1));
    Serial.println("Calories: " + String(calories, 2) + " kcal");
  } 
  
  else {
    Serial.println("Steps: " + String(localStepCount));
    Serial.println("Speed: " + String(speed, 2) + " km/h");
    Serial.println("Calories: " + String(calories, 2) + " kcal");
  }
  
  Serial.println("---------------------");

  // Display results screen
  displayMutex.lock();
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Header with border
  display.drawRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(40, 2);
  display.print("RESULTS");
  
  // Activity type and intensity
  display.drawRect(0, 14, 128, 12, SSD1306_WHITE);
  display.setCursor(2, 16);
  display.print(type);
  display.print(" (");
  display.print(intensityLevel);
  display.print(")");
  
  // Stats - in separate sections
  if (type == "Jumping") {
    display.setCursor(5, 29);
    display.print("Jumps: ");
    display.print(localJumpCount);
    
    unsigned long elapsedTimeMinutes = (millis() - localMeasurementStartTime) / 60000.0;
    if (elapsedTimeMinutes < 0.1) elapsedTimeMinutes = 0.1;
    float jumpsPerMinute = localJumpCount / (float)elapsedTimeMinutes;
    
    display.setCursor(70, 29);
    display.print(jumpsPerMinute, 1);
    display.print("/min");
  } 
  
  else {
    display.setCursor(5, 29);
    display.print("Steps: ");
    display.print(localStepCount);
    
    display.setCursor(70, 29);
    display.print(speed, 1);
    display.print(" km/h");
  }
  
  // Calories in a box at bottom
  display.drawRect(14, 40, 100, 15, SSD1306_WHITE);
  display.setCursor(20, 45);
  display.print("Calories: ");
  display.print(calories, 1);
  
  // Instructions
  display.setCursor(0, 56);
  display.print("Press MODE to continue");
  
  display.display();
  displayMutex.unlock();
}

// Button handling thread function
void buttonsThreadFunc() {
  while (true) {
    checkButtons(); // Button debouncing and state detection
    rtos::ThisThread::sleep_for(BUTTONS_THREAD_DELAY);  // Prevents the thread from running too frequently and using unnecessary CPU time
  }
}

// Sensor processing thread function
void sensorsThreadFunc() {
  while (true) {  // Loop that runs indefinitely in its own thread
    float accelX, accelY, accelZ;
    
    if (IMU.accelerationAvailable()) {  // Read acceleration
      sensorDataMutex.lock();
      
      IMU.readAcceleration(accelX, accelY, accelZ); // Reads the X, Y, Z acceleration values from the sensor
      float magnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);  // Calculates the total acceleration magnitude (vector length) using the Euclidean formula
      unsigned long currentTime = millis();

      // Handle calibration if in calibration state
      if (currentState == STATE_CALIBRATION) {
        updateCalibration(accelX, accelY, accelZ);
      }
      
      // If the system is in tracking mode and a measurement is in progress but not yet complete, proceed with motion analysis
      else if (currentState == STATE_TRACKING && measurementStarted && !measurementComplete) {
        // Process movement based on accelerometer data
        updateDisplacement(accelX, accelY, accelZ, currentTime);
        
        // If the time since the start of the measurement exceeds the user-defined duration, trigger sendResults() to finalize data
        if (currentTime - measurementStartTime >= userDurationMillis) {
          sendResults();
        } 
        
        // Only attempt to detect steps if enough time has passed since the last step (debouncing to avoid false positives)
        else if (currentTime - lastStepTime >= STEP_DEBOUNCE_TIME) {
          
            // If acceleration magnitude exceeds the step threshold, and a step wasn’t already in progress, mark a step-in-progress
            if (magnitude > STEP_THRESHOLD && !wasInStep) {
              wasInStep = true;
            } 
            
            // If magnitude falls below 80% of the step threshold and a step was in progress, register the step
            else if (magnitude < (STEP_THRESHOLD * 0.8) && wasInStep) {
              wasInStep = false;

              Serial.print("Step detected! Magnitude: ");
              Serial.println(magnitude);
              
              stepCount++;
              lastStepTime = currentTime;
            }
        }
      }
      
      sensorDataMutex.unlock();
    }
    
    rtos::ThisThread::sleep_for(SENSORS_THREAD_DELAY);  // Pauses the thread for a defined time
  }
}

// Display update thread function
void displayThreadFunc() {
  while (true) {
    displayMutex.lock();
    
    // Show real-time data
    if (currentState == STATE_TRACKING && measurementStarted && !measurementComplete) {
      displayLiveData();
    }
    
    displayMutex.unlock();
    rtos::ThisThread::sleep_for(DISPLAY_THREAD_DELAY);  // Adds a delay between updates
  }
}

void loop() {
  // Main loop is now empty as tasks are handled by RTOS threads
  delay(1000);
}