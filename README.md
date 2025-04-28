# IMU Sensor Data Collection System

This project collects raw data from the BMI270 accelerometer/gyroscope and BMM150 magnetometer on the Arduino Nano 33 BLE Sense Rev2 and saves it to CSV files using Python. The system includes tools to analyze ankle movement data, detect jumps, and calculate running speed.

## Hardware Requirements

- Arduino Nano 33 BLE Sense Rev2

## Software Requirements

- Arduino IDE with Arduino_BMI270_BMM150 library installed
- Python 3.x
- PySerial library
- For data analysis: pandas, numpy, matplotlib, scipy

## Setup Instructions

1. **Arduino Setup**
   - Connect your Arduino Nano 33 BLE Sense Rev2 to your computer
   - Open `sensor_data_collector.ino` in the Arduino IDE
   - Upload the sketch to your Arduino

2. **Python Setup**
   - Install the required Python packages:
     ```
     pip install pyserial pandas numpy matplotlib scipy
     ```

## Usage

### Data Collection

1. Make sure your Arduino is connected to your computer
2. Run the Python script:
   ```
   python data_collector.py
   ```
3. The script will:
   - Connect to the Arduino via serial port (default: COM3)
   - Create a new CSV file with timestamp in the filename
   - Collect data for 1 minute
   - Save all data to the CSV file in the `sensor_data` directory

### Sample Data

If you don't have actual data yet, you can generate realistic sample data:

1. Run the sample data generator:
   ```
   python generate_sample_data.py
   ```
2. This will create a CSV file in the `sensor_data` directory with:
   - 1 minute of simulated ankle sensor data
   - 30 jumps distributed throughout the recording
   - Running at a speed of approximately 5 km/h
   - Visualization of the generated data

### Ankle Movement Analysis

The project includes ankle movement analysis for jump detection and speed calculation:

1. Run the analysis script:
   ```
   python process_ankle_data.py
   ```
2. The script will:
   - Analyze the sample data in the sensor_data directory
   - Prompt you to process all files in the sensor_data directory
   - Generate visualizations showing acceleration and speed
   - Count the number of jumps detected
   - Calculate running speed in km/h
   - Save analysis results in the `analysis_results` directory

3. To view the results summary:
   ```
   python show_results.py
   ```

## Data Format

The CSV files contain the following columns:
- timestamp (milliseconds since Arduino boot)
- accelX, accelY, accelZ (acceleration in g)
- gyroX, gyroY, gyroZ (angular velocity in degrees per second)
- magX, magY, magZ (magnetic field strength in μT)

## Analysis Features

- Jump detection using vertical acceleration peaks
- Speed calculation through integration of acceleration data
- Running speed in km/h based on horizontal acceleration
- Low-pass filtering to reduce noise
- Visualization of acceleration patterns and speed
- Summary statistics for multiple data files

## Customization

- To change the data collection duration, modify the `60` in `while time.time() - start_time < 60:` in `data_collector.py`
- To change the sampling rate, modify the `delay(10)` in `sensor_data_collector.ino`
- If your Arduino is connected to a different port, change `COM3` in `data_collector.py` to match your port
- To adjust jump detection sensitivity, modify the `threshold` parameter in the `detect_jumps` function in `process_ankle_data.py`
- To modify the sample data parameters, adjust the variables in `generate_sample_data.py` 