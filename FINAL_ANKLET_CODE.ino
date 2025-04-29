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

const float STEP_THRESHOLD = 3.0;
const float STRIDE_LENGTH = 0.7;
const int DEBOUNCE_TIME = 300;
const float STEPPING_THRESHOLD = 3.3;
const float RUNNING_THRESHOLD = 3.9;
const float VELOCITY_DECAY = 0.95;
const float ACCEL_BIAS = 0.01;

unsigned long lastStepTime = 0;
unsigned long measurementStartTime = 0;
unsigned long lastSampleTime = 0;
unsigned long userDurationMillis = 60000;
int stepCount = 0;
bool wasInStep = false;
bool measurementStarted = false;
bool measurementComplete = false;

float velocityX = 0, velocityY = 0;
float displacementX = 0, displacementY = 0;
float totalDisplacement = 0;

int userWeight = 60;
int userDurationMinutes = 1;

void setup() {
  Serial.begin(115200);
  if (!IMU.begin()) while (1);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while (1);
  if (!BLE.begin()) while (1);

  BLE.setLocalName("SensorData");
  BLE.setAdvertisedService(sensorService);
  sensorService.addCharacteristic(sensorCharacteristic);
  BLE.addService(sensorService);
  BLE.advertise();

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Enter Weight:");
  display.println("Use buttons (mock)");
  display.display();

  delay(3000); // Mock button input
  userWeight = 60; // Simulate user input

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Enter Duration:");
  display.println("Use buttons (mock)");
  display.display();

  delay(3000);
  userDurationMinutes = 1; // Simulate duration input
  userDurationMillis = userDurationMinutes * 60000UL;

  Serial.println("Ready to start measurement");
  Serial.println("Waiting for movement...");
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
  if (totalDisplacement < STEPPING_THRESHOLD) return "Stepping in place";
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
  float speed = calculateSpeed(stepCount, userDurationMillis);
  String type = determineMovementType();
  float MET = calculateMET(type);
  float calories = calculateCalories(MET, userWeight, userDurationMinutes);

  String dataString = String(stepCount) + "," + String(speed, 2) + "," +
                      String(totalDisplacement, 2) + "," + type;
  sensorCharacteristic.writeValue(dataString.c_str());

  Serial.println("--- Final Results ---");
  Serial.println("Activity: " + type);

  if (type == "Stepping in place") {
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

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("Result:");
  display.println("Type: " + type);

  if (type == "Stepping in place") {
    display.print("Steps: "); display.println(stepCount);
    display.print("Cal: "); display.print(calories, 1); display.println(" kcal");
  } else if (type == "Running") {
    display.print("Speed: "); display.print(speed, 1); display.println(" km/h");
    display.print("Steps: "); display.println(stepCount);
    display.print("Cal: "); display.print(calories, 1); display.println(" kcal");
  } else {
    display.print("Steps: "); display.println(stepCount);
    display.print("Speed: "); display.print(speed, 1); display.println(" km/h");
    display.print("Dist: "); display.print(totalDisplacement, 1); display.println(" m");
    display.print("Cal: "); display.print(calories, 1); display.println(" kcal");
  }
  display.display();

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
  BLEDevice central = BLE.central();
  if (central) {
    while (central.connected()) {
      float accelX, accelY, accelZ;
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(accelX, accelY, accelZ);
        float magnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
        unsigned long currentTime = millis();

        if (!measurementStarted && magnitude > STEP_THRESHOLD) {
          measurementStarted = true;
          measurementStartTime = currentTime;
          measurementComplete = false;
          Serial.println("Measurement started...");
        }

        if (measurementStarted && !measurementComplete) {
          updateDisplacement(accelX, accelY, currentTime);
          if (currentTime - measurementStartTime >= userDurationMillis) {
            sendResults();
            continue;
          }

          if (currentTime - lastStepTime >= DEBOUNCE_TIME) {
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
  }
}
