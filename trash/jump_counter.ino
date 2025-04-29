#include <Arduino_BMI270_BMM150.h>
#include <Wire.h>
#include <ArduinoBLE.h>

// BLE Service and Characteristic UUIDs
#define JUMP_SERVICE_UUID "19B10000-E8F2-537E-4F6C-D104768A1214"
#define JUMP_CHARACTERISTIC_UUID "19B10001-E8F2-537E-4F6C-D104768A1214"

// BLE Service and Characteristic
BLEService jumpService(JUMP_SERVICE_UUID);
BLECharacteristic jumpCharacteristic(JUMP_CHARACTERISTIC_UUID, BLENotify, 20);

// Jump detection parameters
const float JUMP_THRESHOLD = 1.5;  // Acceleration threshold for jump detection
const unsigned long DEBOUNCE_TIME = 500;  // Minimum time between jumps (ms)
unsigned long lastJumpTime = 0;
int jumpCount = 0;

// Variables for smoothing
const int SMOOTHING_WINDOW = 5;
float accelHistory[SMOOTHING_WINDOW];
int historyIndex = 0;

void setup() {
  Serial.begin(115200);
  
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }

  // Set advertised local name and service UUID
  BLE.setLocalName("JumpCounter");
  BLE.setAdvertisedService(jumpService);

  // Add characteristic to service
  jumpService.addCharacteristic(jumpCharacteristic);

  // Add service
  BLE.addService(jumpService);

  // Start advertising
  BLE.advertise();
  Serial.println("Jump Counter active, waiting for connections...");

  // Initialize smoothing array
  for (int i = 0; i < SMOOTHING_WINDOW; i++) {
    accelHistory[i] = 0;
  }
}

float getSmoothedAcceleration() {
  float accelX, accelY, accelZ;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accelX, accelY, accelZ);
    
    // Update smoothing history
    accelHistory[historyIndex] = accelZ;
    historyIndex = (historyIndex + 1) % SMOOTHING_WINDOW;
    
    // Calculate smoothed value
    float smoothed = 0;
    for (int i = 0; i < SMOOTHING_WINDOW; i++) {
      smoothed += accelHistory[i];
    }
    return smoothed / SMOOTHING_WINDOW;
  }
  return 0;
}

void detectJump() {
  float smoothedAccel = getSmoothedAcceleration();
  unsigned long currentTime = millis();
  
  // Check if we're above threshold and enough time has passed since last jump
  if (smoothedAccel > JUMP_THRESHOLD && (currentTime - lastJumpTime) > DEBOUNCE_TIME) {
    jumpCount++;
    lastJumpTime = currentTime;
    
    // Send jump count over BLE
    String jumpData = String(jumpCount);
    jumpCharacteristic.writeValue(jumpData.c_str());
    
    // Also print to Serial for debugging
    Serial.print("Jump detected! Total jumps: ");
    Serial.println(jumpCount);
  }
}

void loop() {
  // Wait for a BLE central to connect
  BLEDevice central = BLE.central();
  
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected()) {
      detectJump();
      delay(10);  // Small delay to prevent overwhelming the system
    }
    
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
} 