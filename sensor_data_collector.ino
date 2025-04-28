#include <Arduino_BMI270_BMM150.h>
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}

void loop() {
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float magX, magY, magZ;
  
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accelX, accelY, accelZ);
  }
  
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
  }
  
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(magX, magY, magZ);
  }
  
  // Print raw data in CSV format
  Serial.print(millis());
  Serial.print(",");
  Serial.print(accelX, 6);
  Serial.print(",");
  Serial.print(accelY, 6);
  Serial.print(",");
  Serial.print(accelZ, 6);
  Serial.print(",");
  Serial.print(gyroX, 6);
  Serial.print(",");
  Serial.print(gyroY, 6);
  Serial.print(",");
  Serial.print(gyroZ, 6);
  Serial.print(",");
  Serial.print(magX, 6);
  Serial.print(",");
  Serial.print(magY, 6);
  Serial.print(",");
  Serial.println(magZ, 6);
  
  delay(10); // Adjust delay as needed
} 