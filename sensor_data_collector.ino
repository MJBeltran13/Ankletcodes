#include <Arduino_BMI270_BMM150.h>
#include <Wire.h>
#include <ArduinoBLE.h>

// BLE Service and Characteristic UUIDs
#define SENSOR_SERVICE_UUID "19B10000-E8F2-537E-4F6C-D104768A1214"
#define SENSOR_CHARACTERISTIC_UUID "19B10001-E8F2-537E-4F6C-D104768A1214"

// BLE Service and Characteristic
BLEService sensorService(SENSOR_SERVICE_UUID);
BLECharacteristic sensorCharacteristic(SENSOR_CHARACTERISTIC_UUID, BLENotify, 40);

// Jump detection variables
const float JUMP_THRESHOLD = 0.8;      // Lowered threshold for easier detection
const int DEBOUNCE_TIME = 500;        // Reduced debounce time
unsigned long lastJumpTime = 0;
int jumpCount = 0;
bool wasInAir = false;

// Speed detection variables
const int SAMPLES_FOR_SPEED = 10;
float accelSamples[SAMPLES_FOR_SPEED][3];
int sampleIndex = 0;
unsigned long lastSpeedUpdateTime = 0;
const unsigned long SPEED_UPDATE_INTERVAL = 1000; // Update speed every 1 second
float currentSpeed = 0.0; // Speed in km/h
float previousAccelMagnitudes[3] = {0.0, 0.0, 0.0}; // Store previous values to detect actual movement

// Combined data variables
bool dataUpdated = false;

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
}

// Calculate speed from acceleration samples
float calculateSpeed() {
  float sumX = 0, sumY = 0;
  
  // Calculate the sum of horizontal accelerations (X and Y axis)
  for (int i = 0; i < SAMPLES_FOR_SPEED; i++) {
    sumX += accelSamples[i][0]; // X-axis
    sumY += accelSamples[i][1]; // Y-axis
  }
  
  // Average acceleration on horizontal plane
  float avgAccelX = sumX / SAMPLES_FOR_SPEED;
  float avgAccelY = sumY / SAMPLES_FOR_SPEED;
  
  // Calculate magnitude of acceleration vector
  float accelMagnitude = sqrt(sq(avgAccelX) + sq(avgAccelY));
  
  // Check for actual movement by comparing with previous magnitudes
  // Shift values in the array
  float variability = 0;
  for (int i = 2; i > 0; i--) {
    previousAccelMagnitudes[i] = previousAccelMagnitudes[i-1];
    variability += abs(previousAccelMagnitudes[i] - previousAccelMagnitudes[i-1]);
  }
  previousAccelMagnitudes[0] = accelMagnitude;
  
  // Apply a higher threshold to filter out noise when standing still
  // Also check for variability in readings - real movement has changing acceleration
  if (accelMagnitude < 0.25 || variability < 0.05) {
    return 0.0;
  }
  
  // Simple conversion from acceleration to speed with improved calibration
  float calibrationFactor = 2.8; // Reduced calibration factor
  
  return accelMagnitude * calibrationFactor;
}

// Send updated data via BLE
void sendUpdatedData() {
  // Format data as simple comma-separated values: "jumps,speed"
  // This format is easier to parse on the receiving end
  char dataBuffer[40];
  sprintf(dataBuffer, "%d,%.1f", jumpCount, currentSpeed);
  
  // Send the combined data
  sensorCharacteristic.writeValue(dataBuffer);
  
  Serial.print("BLE Sent: ");
  Serial.println(dataBuffer);
  
  dataUpdated = false;
}

void loop() {
  // Wait for a BLE central to connect
  BLEDevice central = BLE.central();
  
  if (central) {
    while (central.connected()) {
      float accelX, accelY, accelZ;
      
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(accelX, accelY, accelZ);
        
        // Store acceleration sample for speed calculation
        accelSamples[sampleIndex][0] = accelX;
        accelSamples[sampleIndex][1] = accelY;
        accelSamples[sampleIndex][2] = accelZ;
        sampleIndex = (sampleIndex + 1) % SAMPLES_FOR_SPEED;
        
        // Print acceleration for debugging
        Serial.print("AccelZ: ");
        Serial.println(accelZ);
        
        unsigned long currentTime = millis();
        
        // Check if enough time has passed since last jump
        if (currentTime - lastJumpTime < DEBOUNCE_TIME) {
          continue;
        }

        // Jump detection
        if (accelZ < -JUMP_THRESHOLD && !wasInAir) {
          wasInAir = true;
          Serial.println("Takeoff detected!");
        } 
        else if (accelZ > 0 && wasInAir) {
          // Jump completed
          wasInAir = false;
          jumpCount++;
          lastJumpTime = currentTime;
          
          Serial.print("JUMP! Count: ");
          Serial.println(jumpCount);
          
          dataUpdated = true;
        }
        
        // Update speed at regular intervals
        if (currentTime - lastSpeedUpdateTime >= SPEED_UPDATE_INTERVAL) {
          currentSpeed = calculateSpeed();
          lastSpeedUpdateTime = currentTime;
          
          // Enhanced speed display
          Serial.println("-------------------------");
          Serial.print("SPEED: ");
          Serial.print(currentSpeed, 1);
          Serial.println(" km/h");
          
          // Visual indicator for speed
          Serial.print("Indicator: ");
          for (int i = 0; i < min(int(currentSpeed), 20); i++) {
            Serial.print(">");
          }
          Serial.println("");
          Serial.println("-------------------------");
          
          dataUpdated = true;
        }
        
        // Send combined data if anything was updated
        if (dataUpdated) {
          sendUpdatedData();
        }
      }
      
      delay(10); // Small delay to prevent overwhelming the connection
    }
  }
} 