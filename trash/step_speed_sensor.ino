#include <Arduino_BMI270_BMM150.h>
#include <Wire.h>
#include <ArduinoBLE.h>

// BLE Service and Characteristic UUIDs
#define SENSOR_SERVICE_UUID "19B10000-E8F2-537E-4F6C-D104768A1214"
#define SENSOR_CHARACTERISTIC_UUID "19B10001-E8F2-537E-4F6C-D104768A1214"

// BLE Service and Characteristic
BLEService sensorService(SENSOR_SERVICE_UUID);
BLECharacteristic sensorCharacteristic(SENSOR_CHARACTERISTIC_UUID, BLENotify, 20);

// Step detection parameters
const float STEP_THRESHOLD = 3.0;      // Acceleration threshold for step detection
const float STRIDE_LENGTH = 0.7;       // Average stride length in meters
const int DEBOUNCE_TIME = 300;        // Minimum time between steps (ms)
const unsigned long ONE_MINUTE = 10000; // 60 seconds in milliseconds

// Movement classification thresholds
const float STEPPING_THRESHOLD = 3.3;  // Maximum displacement for stepping in place (meters)
const float RUNNING_THRESHOLD = 3.9;   // Minimum displacement for running (meters)

// Raw displacement calculation parameters
const float VELOCITY_DECAY = 0.95;     // Basic velocity decay
const float ACCEL_BIAS = 0.01;         // Basic acceleration bias

// Variables for step and speed calculation
unsigned long lastStepTime = 0;
unsigned long measurementStartTime = 0;
unsigned long lastSampleTime = 0;
int stepCount = 0;
float currentSpeed = 0.0;
bool wasInStep = false;
bool measurementStarted = false;
bool measurementComplete = false;

// Variables for displacement calculation
float velocityX = 0;
float velocityY = 0;
float displacementX = 0;
float displacementY = 0;
float lastAccelX = 0;
float lastAccelY = 0;
float totalDisplacement = 0;

void setup() {
  Serial.begin(115200);  // Initialize serial but don't wait for it

  // Initialize IMU with maximum sensitivity
  if (!IMU.begin()) {
    while (1); // If IMU fails, stop here
  }

  // Initialize BLE
  if (!BLE.begin()) {
    while (1); // If BLE fails, stop here
  }

  // Set advertised local name and service UUID
  BLE.setLocalName("SensorData");
  BLE.setAdvertisedService(sensorService);

  // Add characteristic to service
  sensorService.addCharacteristic(sensorCharacteristic);

  // Add service
  BLE.addService(sensorService);

  // Start advertising
  BLE.advertise();

  Serial.println("Ready to start measurement");
  Serial.println("Waiting for movement to start timing...");
}

float calculateSpeed(int steps, unsigned long timeMs) {
  // Convert time to minutes
  float timeMinutes = timeMs / 60000.0;

  // Calculate steps per minute
  float stepsPerMinute = steps / timeMinutes;

  // Calculate speed in km/h
  float speedKmh = (stepsPerMinute * STRIDE_LENGTH * 60.0) / 1000.0;

  return speedKmh;
}

void updateDisplacement(float accelX, float accelY, unsigned long currentTime) {
  // Calculate time delta in seconds
  float dt = (currentTime - lastSampleTime) / 1000.0;
  if (lastSampleTime == 0) {
    dt = 0;
  }

  // Only update if we have a reasonable time delta
  if (dt > 0 && dt < 0.1) {
    // Apply basic bias compensation
    float filteredAccelX = accelX - ACCEL_BIAS;
    float filteredAccelY = accelY - ACCEL_BIAS;

    // Integrate acceleration to get velocity
    velocityX = (velocityX + filteredAccelX * dt) * VELOCITY_DECAY;
    velocityY = (velocityY + filteredAccelY * dt) * VELOCITY_DECAY;

    // Integrate velocity to get displacement
    displacementX += velocityX * dt;
    displacementY += velocityY * dt;

    // Calculate total displacement
    totalDisplacement = sqrt(displacementX * displacementX + displacementY * displacementY);
  }

  lastSampleTime = currentTime;
  lastAccelX = accelX;
  lastAccelY = accelY;
}

String determineMovementType() {
  // Debug information
  Serial.print("Debug - Displacement: ");
  Serial.print(totalDisplacement, 2);
  Serial.print("m, Steps: ");
  Serial.println(stepCount);

  // Simple classification based on displacement
  if (totalDisplacement < STEPPING_THRESHOLD) {
    return "Stepping in place";
  } else if (totalDisplacement > RUNNING_THRESHOLD) {
    return "Running";
  } else {
    return "Walking";
  }
}

void sendResults() {
  // Calculate final speed
  currentSpeed = calculateSpeed(stepCount, ONE_MINUTE);
  String movementType = determineMovementType();

  // Create data string with step count, speed, and movement type
  String dataString = String(stepCount) + "," + String(currentSpeed, 2) + "," +
                      String(totalDisplacement, 2) + "," + movementType;
  sensorCharacteristic.writeValue(dataString.c_str());

  // Print final results
  Serial.println("\n--- Final Results ---");
  Serial.print("Total Steps: ");
  Serial.println(stepCount);
  Serial.print("Average Speed: ");
  Serial.print(currentSpeed, 2);
  Serial.println(" km/h");
  Serial.print("Total Displacement: ");
  Serial.print(totalDisplacement, 2);
  Serial.println(" meters");
  Serial.print("Movement Type: ");
  Serial.println(movementType);
  Serial.println("------------------");

  // Reset for next measurement
  stepCount = 0;
  totalDisplacement = 0;
  displacementX = 0;
  displacementY = 0;
  velocityX = 0;
  velocityY = 0;
  measurementComplete = true;
  measurementStarted = false;
  lastSampleTime = 0;
}

void loop() {
  // Wait for a BLE central to connect
  BLEDevice central = BLE.central();

  if (central) {
    while (central.connected()) {
      float accelX, accelY, accelZ;

      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(accelX, accelY, accelZ);

        // Calculate magnitude of acceleration
        float accelMagnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
        unsigned long currentTime = millis();

        // Start measurement when first movement is detected
        if (!measurementStarted && accelMagnitude > STEP_THRESHOLD) {
          measurementStarted = true;
          measurementStartTime = currentTime;
          measurementComplete = false;
          Serial.println("Movement detected! Starting 1-minute measurement...");
          Serial.println("Keep walking...");
        }

        // Only process steps if measurement is ongoing
        if (measurementStarted && !measurementComplete) {
          // Update displacement calculation
          updateDisplacement(accelX, accelY, currentTime);

          // Check if one minute has elapsed
          if (currentTime - measurementStartTime >= ONE_MINUTE) {
            sendResults();
            continue;
          }

          // Print remaining time every 10 seconds
          if ((currentTime - measurementStartTime) % 10000 == 0) {
            int remainingSeconds = (ONE_MINUTE - (currentTime - measurementStartTime)) / 1000;
            Serial.print("Time remaining: ");
            Serial.print(remainingSeconds);
            Serial.println(" seconds");

            // Print current displacement
            Serial.print("Current displacement: ");
            Serial.print(totalDisplacement, 2);
            Serial.println(" meters");
          }

          // Check if enough time has passed since last step
          if (currentTime - lastStepTime < DEBOUNCE_TIME) {
            continue;
          }

          // Step detection using state machine
          if (accelMagnitude > STEP_THRESHOLD && !wasInStep) {
            wasInStep = true;
          }
          else if (accelMagnitude < (STEP_THRESHOLD * 0.8) && wasInStep) {
            // Step completed
            wasInStep = false;
            stepCount++;
            lastStepTime = currentTime;

            // Print current step count
            Serial.print("Steps: ");
            Serial.println(stepCount);
          }
        }
      }

      delay(10); // Small delay to prevent overwhelming the connection
    }
  }
}