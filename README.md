# Anklet - Smart Motion Tracker

## Overview
Anklet is a sophisticated motion tracking device built using Arduino Nano 33 BLE Sense Rev2. It accurately tracks various physical activities including walking, running, and jumping, providing real-time feedback and detailed activity analysis.

## Features
- **Multi-Activity Tracking**
  - Walking detection with step counting
  - Running detection with speed calculation
  - Jump detection with jump count and intensity analysis
  
- **Smart Calibration**
  - Automatic device orientation detection
  - Gravity vector compensation
  - Motion sensor calibration
  
- **Real-time Monitoring**
  - OLED display for live activity data
  - Current activity type detection
  - Step/jump counter
  - Remaining time display
  - Progress bar
  
- **Comprehensive Results**
  - Activity type classification
  - Intensity level assessment
  - Speed calculation (for walking/running)
  - Jumps per minute (for jumping)
  - Calorie burn estimation using MET values
  
- **User Configuration**
  - Adjustable user weight (30-150 kg)
  - Configurable session duration (1-60 minutes)
  
## Hardware Requirements
- Arduino Nano 33 BLE Sense Rev2
- SSD1306 OLED Display (128x64 pixels)
- 3 Push Buttons
  - UP button (Pin 2)
  - DOWN button (Pin 3)
  - MODE button (Pin 4)

## Dependencies
```
Arduino_BMI270_BMM150  // IMU sensor library
Wire                   // I2C communication
Adafruit_GFX          // Graphics library
Adafruit_SSD1306      // OLED display driver
mbed                  // Core OS library
rtos                  // Real-time operating system
```

## Implementation Details
- **RTOS Implementation**
  - Multi-threaded operation for responsive UI
  - Separate threads for:
    - Button handling
    - Sensor processing
    - Display updates
  - Thread-safe data access using mutexes

- **Activity Detection**
  - Walking/Running: Based on step frequency and acceleration patterns
  - Jumping: Vertical acceleration analysis with pattern recognition
  - Intensity levels: Light, Moderate, Vigorous, High

- **Calorie Calculation**
  - Uses MET (Metabolic Equivalent of Task) values
  - Considers activity type, intensity, and user weight
  - Dynamic calculation based on actual activity duration

## Usage
1. **Initial Setup**
   - Set your weight using UP/DOWN buttons
   - Configure session duration
   - Wait for device calibration
   
2. **Activity Tracking**
   - Press MODE to start tracking
   - Real-time display shows:
     - Current activity type
     - Step/jump count
     - Remaining time
     - Progress bar
   
3. **View Results**
   - Session automatically ends after set duration
   - Or press MODE to end early
   - Results show:
     - Activity summary
     - Intensity level
     - Performance metrics
     - Calories burned

## Technical Notes
- Step detection threshold: 2.0 m/s²
- Jump detection threshold: 2.5 m/s²
- Average stride length: 0.6 meters
- Display updates every 500ms
- Button debounce time: 100ms
- Sensor sampling rate: 10ms

## Future Improvements
- Bluetooth connectivity for data export
- Extended activity recognition
- Custom activity profiles
- Battery optimization
- Data logging capabilities

## License
[Add your license information here]

## Contributors
[Add contributor information here] 