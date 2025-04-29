#include <Arduino_BMI270_BMM150.h>
#include <Wire.h>
#include <ArduinoBLE.h>

// BLE Service and Characteristic UUIDs
#define SENSOR_SERVICE_UUID "19B10000-E8F2-537E-4F6C-D104768A1214"
#define SENSOR_CHARACTERISTIC_UUID "19B10001-E8F2-537E-4F6C-D104768A1214"

// BLE Service and Characteristic
BLEService sensorService(SENSOR_SERVICE_UUID);
BLECharacteristic sensorCharacteristic(SENSOR_CHARACTERISTIC_UUID, BLENotify, 20);

// Jump detection variables
const float JUMP_THRESHOLD = 0.8;      // Lowered threshold for easier detection
const int DEBOUNCE_TIME = 500;        // Reduced debounce time
unsigned long lastJumpTime = 0;
int jumpCount = 0;
bool wasInAir = false;

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

void loop() {
  // Wait for a BLE central to connect
  BLEDevice central = BLE.central();
  
  if (central) {
    while (central.connected()) {
      float accelX, accelY, accelZ;
      
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(accelX, accelY, accelZ);
        
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
          
          // Send the jump count
          String jumpData = String(jumpCount);
          sensorCharacteristic.writeValue(jumpData.c_str());
          
          Serial.print("JUMP! Count: ");
          Serial.println(jumpCount);
        }
      }
      
      delay(10); // Small delay to prevent overwhelming the connection
    }
  }
} 