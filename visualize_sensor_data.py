import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy.signal import butter, filtfilt

def apply_low_pass_filter(data, cutoff=1.0, fs=32.9, order=4):
    """Apply a low-pass filter to remove noise"""
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data)

def visualize_sensor_data():
    # Path to the sensor data directory
    data_dir = "sensor_data"
    
    # Check if directory exists
    if not os.path.exists(data_dir):
        print(f"Error: Directory '{data_dir}' not found")
        return
    
    # List all CSV files in the directory
    csv_files = [f for f in os.listdir(data_dir) if f.endswith('.csv')]
    
    if not csv_files:
        print("No sensor data files found in the directory")
        return
    
    # Create output directory for visualizations
    output_dir = "data_visualizations"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Process each CSV file
    for csv_file in csv_files:
        file_path = os.path.join(data_dir, csv_file)
        print(f"Processing {csv_file}...")
        
        # Read the CSV file
        data = pd.read_csv(file_path)
        
        # Create time array in seconds
        time = np.arange(len(data)) / 32.9  # 32.9 Hz sampling rate
        
        # Create figure with subplots
        plt.figure(figsize=(15, 12))
        
        # Plot 1: Accelerometer Data
        plt.subplot(3, 1, 1)
        plt.plot(time, data['accelX'], label='X', alpha=0.7)
        plt.plot(time, data['accelY'], label='Y', alpha=0.7)
        plt.plot(time, data['accelZ'], label='Z', alpha=0.7)
        plt.title('Accelerometer Data')
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (g)')
        plt.legend()
        plt.grid(True)
        
        # Plot 2: Gyroscope Data
        plt.subplot(3, 1, 2)
        plt.plot(time, data['gyroX'], label='X', alpha=0.7)
        plt.plot(time, data['gyroY'], label='Y', alpha=0.7)
        plt.plot(time, data['gyroZ'], label='Z', alpha=0.7)
        plt.title('Gyroscope Data')
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Velocity (deg/s)')
        plt.legend()
        plt.grid(True)
        
        # Plot 3: Magnetometer Data
        plt.subplot(3, 1, 3)
        plt.plot(time, data['magX'], label='X', alpha=0.7)
        plt.plot(time, data['magY'], label='Y', alpha=0.7)
        plt.plot(time, data['magZ'], label='Z', alpha=0.7)
        plt.title('Magnetometer Data')
        plt.xlabel('Time (s)')
        plt.ylabel('Magnetic Field (uT)')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        
        # Save the figure
        output_file = os.path.join(output_dir, f"{os.path.splitext(csv_file)[0]}_visualization.png")
        plt.savefig(output_file)
        print(f"Saved visualization to {output_file}")
        
        # Show the plot
        plt.show()
        
        # Create filtered data visualization
        plt.figure(figsize=(15, 12))
        
        # Plot 1: Filtered Accelerometer Data
        plt.subplot(3, 1, 1)
        plt.plot(time, apply_low_pass_filter(data['accelX']), label='X', alpha=0.7)
        plt.plot(time, apply_low_pass_filter(data['accelY']), label='Y', alpha=0.7)
        plt.plot(time, apply_low_pass_filter(data['accelZ']), label='Z', alpha=0.7)
        plt.title('Filtered Accelerometer Data')
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (g)')
        plt.legend()
        plt.grid(True)
        
        # Plot 2: Filtered Gyroscope Data
        plt.subplot(3, 1, 2)
        plt.plot(time, apply_low_pass_filter(data['gyroX']), label='X', alpha=0.7)
        plt.plot(time, apply_low_pass_filter(data['gyroY']), label='Y', alpha=0.7)
        plt.plot(time, apply_low_pass_filter(data['gyroZ']), label='Z', alpha=0.7)
        plt.title('Filtered Gyroscope Data')
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Velocity (deg/s)')
        plt.legend()
        plt.grid(True)
        
        # Plot 3: Filtered Magnetometer Data
        plt.subplot(3, 1, 3)
        plt.plot(time, apply_low_pass_filter(data['magX']), label='X', alpha=0.7)
        plt.plot(time, apply_low_pass_filter(data['magY']), label='Y', alpha=0.7)
        plt.plot(time, apply_low_pass_filter(data['magZ']), label='Z', alpha=0.7)
        plt.title('Filtered Magnetometer Data')
        plt.xlabel('Time (s)')
        plt.ylabel('Magnetic Field (uT)')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        
        # Save the filtered visualization
        output_file = os.path.join(output_dir, f"{os.path.splitext(csv_file)[0]}_filtered_visualization.png")
        plt.savefig(output_file)
        print(f"Saved filtered visualization to {output_file}")
        
        # Show the filtered plot
        plt.show()

if __name__ == "__main__":
    visualize_sensor_data() 