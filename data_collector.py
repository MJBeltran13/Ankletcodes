import serial
import time
import csv
from datetime import datetime
import os
import numpy as np

def get_next_filename():
    base_dir = "sensor_data"
    if not os.path.exists(base_dir):
        os.makedirs(base_dir)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(base_dir, f"sensor_data_{timestamp}.csv")

def main():
    # Configure serial port - adjust COM port as needed
    ser = serial.Serial('COM3', 115200, timeout=1)
    print("Waiting for data...")
    
    # Wait for Arduino to initialize
    time.sleep(2)
    
    # Create headers for CSV
    headers = ['timestamp', 'accelX', 'accelY', 'accelZ', 
               'gyroX', 'gyroY', 'gyroZ', 
               'magX', 'magY', 'magZ']
    
    # Create new CSV file
    filename = get_next_filename()
    print(f"Creating new file: {filename}")
    
    # Keep track of timestamps for sampling rate calculation
    all_timestamps = []
    data_count = 0
    collection_start_time = time.time()
    
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(headers)
        
        start_time = time.time()
        while time.time() - start_time < 60:  # Run for 1 minute
            if ser.in_waiting:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    try:
                        # Split the line into values and write to CSV
                        values = [float(x) for x in line.split(',')]
                        writer.writerow(values)
                        
                        # Store timestamp for sampling rate calculation
                        all_timestamps.append(time.time())
                        data_count += 1
                        
                    except ValueError:
                        print(f"Error parsing line: {line}")
                        continue
    
    ser.close()
    
    # Calculate sampling rate
    collection_duration = time.time() - collection_start_time
    samples_per_second = data_count / collection_duration
    
    # If we have timestamps from the Arduino itself, use those for a more accurate calculation
    arduino_sample_rate = None
    if data_count > 1 and len(all_timestamps) > 1:
        # Calculate intervals between samples
        intervals = np.diff(all_timestamps)
        avg_interval = np.mean(intervals)
        arduino_sample_rate = 1.0 / avg_interval
    
    print("\nData Collection Summary:")
    print(f"Data collection complete! Collected {data_count} samples in {collection_duration:.2f} seconds.")
    print(f"Average sampling rate: {samples_per_second:.2f} Hz")
    if arduino_sample_rate:
        print(f"Measured sampling rate from timestamps: {arduino_sample_rate:.2f} Hz")
    
    # Save sampling rate info to a metadata file
    meta_file = filename.replace('.csv', '_meta.txt')
    with open(meta_file, 'w') as f:
        f.write(f"Data Collection Summary:\n")
        f.write(f"Filename: {filename}\n")
        f.write(f"Collection time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Duration: {collection_duration:.2f} seconds\n")
        f.write(f"Total samples: {data_count}\n")
        f.write(f"Average sampling rate: {samples_per_second:.2f} Hz\n")
        if arduino_sample_rate:
            f.write(f"Measured sampling rate from timestamps: {arduino_sample_rate:.2f} Hz\n")
    
    print(f"Metadata saved to {meta_file}")

if __name__ == "__main__":
    main() 