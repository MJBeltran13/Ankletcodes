import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
from scipy.signal import find_peaks, butter, filtfilt

def load_csv_data(file_path):
    """Load sensor data from CSV file"""
    return pd.read_csv(file_path)

def apply_low_pass_filter(data, cutoff=1.0, fs=32.9, order=4):
    """Apply a low-pass filter to remove noise"""
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data)

def detect_jumps(accel_z, threshold_high=None, threshold_low=None, min_distance=10):
    """Detect jumps based on Z-axis acceleration patterns
    
    For ankle-mounted sensors, jumps typically show as:
    1. Initial positive acceleration (push-off)
    2. Followed by near-zero or negative acceleration (in flight)
    3. Then large negative acceleration (landing)
    
    We'll identify jumps by finding significant positive peaks, which indicate push-off.
    """
    # If thresholds not provided, calculate based on data
    if threshold_high is None:
        # Calculate threshold based on data statistics
        std_dev = np.std(accel_z)
        threshold_high = 2.0 * std_dev  # More sensitive threshold for lower sample rate
    
    # Find positive peaks (takeoff phase)
    takeoff_peaks, _ = find_peaks(accel_z, height=threshold_high, distance=min_distance)
    
    print(f"Jump detection threshold: {threshold_high:.3f}")
    print(f"Found {len(takeoff_peaks)} potential jumps")
    
    return takeoff_peaks

def calculate_vertical_speed(accel_data, timestamps, gravity=9.81):
    """Calculate vertical speed from Z-axis acceleration data"""
    # Calculate time differences between samples
    dt = np.diff(timestamps)
    dt = np.append(dt, dt[-1])  # Add the last dt to make lengths match
    
    # Scale acceleration to m/s²
    accel_z_without_gravity = accel_data * gravity
    
    # Apply a high-pass filter to remove drift (essentially zeroing the mean)
    accel_z_without_gravity = accel_z_without_gravity - np.mean(accel_z_without_gravity)
    
    # Integrate acceleration to get velocity
    velocity = np.zeros_like(accel_z_without_gravity)
    
    for i in range(1, len(velocity)):
        velocity[i] = velocity[i-1] + accel_z_without_gravity[i] * dt[i-1]
        # Apply drift correction - slowly return to zero
        velocity[i] *= 0.995  # Increased decay factor for lower sample rate
    
    return velocity

def calculate_running_speed(accel_x, accel_y, timestamps, gravity=9.81):
    """Calculate running speed in km/h from horizontal acceleration components"""
    # Calculate time differences between samples
    dt = np.diff(timestamps)
    dt = np.append(dt, dt[-1])  # Add the last dt to make lengths match
    
    # Scale acceleration to m/s²
    accel_x_ms2 = accel_x * gravity
    accel_y_ms2 = accel_y * gravity
    
    # Remove mean to eliminate static offset
    accel_x_ms2 = accel_x_ms2 - np.mean(accel_x_ms2)
    accel_y_ms2 = accel_y_ms2 - np.mean(accel_y_ms2)
    
    # Calculate horizontal acceleration magnitude
    accel_horizontal = np.sqrt(accel_x_ms2**2 + accel_y_ms2**2)
    
    # Apply low-pass filter to horizontal acceleration
    accel_horizontal_filtered = apply_low_pass_filter(accel_horizontal, cutoff=0.5, fs=32.9)
    
    # Integrate acceleration to get velocity
    velocity = np.zeros_like(accel_horizontal_filtered)
    
    for i in range(1, len(velocity)):
        velocity[i] = velocity[i-1] + accel_horizontal_filtered[i] * dt[i-1]
        # Apply drift correction - slowly return to zero
        velocity[i] *= 0.995  # Increased decay factor for lower sample rate
    
    # For ankle sensors, we need to account for the rotational movement
    # Simple calibration - approximately map to expected speed range
    velocity_calibrated = velocity * 0.15  # Adjusted scale factor for lower sample rate
    
    # Convert m/s to km/h
    velocity_kmh = velocity_calibrated * 3.6
    
    # Ensure non-negative speed
    velocity_kmh = np.maximum(velocity_kmh, 0)
    
    return velocity_kmh

def analyze_ankle_data(file_path):
    """Analyze ankle movement data to calculate speed and count jumps"""
    # Load data
    data = load_csv_data(file_path)
    
    # Use the known sample rate
    sample_rate = 32.9  # Hz
    
    print(f"Using sample rate: {sample_rate:.1f} Hz")
    
    # Apply low-pass filter to acceleration data
    accel_z_filtered = apply_low_pass_filter(data['accelZ'], fs=sample_rate)
    accel_x_filtered = apply_low_pass_filter(data['accelX'], fs=sample_rate)
    accel_y_filtered = apply_low_pass_filter(data['accelY'], fs=sample_rate)
    
    # Detect jumps with adjusted parameters
    jump_indices = detect_jumps(accel_z_filtered, min_distance=10)
    num_jumps = len(jump_indices)
    
    # Calculate vertical speed
    vertical_speed = calculate_vertical_speed(accel_z_filtered, data['timestamp'])
    
    # Calculate running speed in km/h
    running_speed_kmh = calculate_running_speed(accel_x_filtered, accel_y_filtered, data['timestamp'])
    
    # Calculate max speeds
    max_vertical_speed = np.max(np.abs(vertical_speed))
    max_running_speed = np.max(running_speed_kmh)
    
    # Average only where running speed is non-negligible (> 0.5 km/h)
    is_running = running_speed_kmh > 0.5
    avg_running_speed = np.mean(running_speed_kmh[is_running]) if np.any(is_running) else 0
    
    # Plot results
    plt.figure(figsize=(12, 12))
    
    # Plot 1: Acceleration
    plt.subplot(3, 1, 1)
    plt.plot(data['timestamp'], accel_z_filtered, label='Z (vertical)')
    plt.scatter(data['timestamp'][jump_indices], accel_z_filtered[jump_indices], 
                color='red', marker='o', label=f'Jumps ({num_jumps})')
    plt.xlabel('Time (ms)')
    plt.ylabel('Vertical Acceleration (g)')
    plt.title('Ankle Movement - Vertical Acceleration')
    plt.legend()
    plt.grid(True)
    
    # Plot 2: Vertical Speed
    plt.subplot(3, 1, 2)
    plt.plot(data['timestamp'], vertical_speed)
    plt.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    plt.xlabel('Time (ms)')
    plt.ylabel('Vertical Speed (m/s)')
    plt.title(f'Ankle Vertical Speed (Max: {max_vertical_speed:.2f} m/s)')
    plt.grid(True)
    
    # Plot 3: Running Speed
    plt.subplot(3, 1, 3)
    plt.plot(data['timestamp'], running_speed_kmh)
    plt.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    plt.xlabel('Time (ms)')
    plt.ylabel('Running Speed (km/h)')
    plt.title(f'Running Speed (Max: {max_running_speed:.2f} km/h, Avg: {avg_running_speed:.2f} km/h)')
    plt.grid(True)
    
    plt.tight_layout()
    
    # Save the figure
    output_dir = "analysis_results"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Get base filename without extension
    base_name = os.path.splitext(os.path.basename(file_path))[0]
    plt.savefig(os.path.join(output_dir, f"{base_name}_analysis.png"))
    
    # Print analysis results
    print(f"Analysis Results for {file_path}:")
    print(f"Number of jumps detected: {num_jumps}")
    print(f"Maximum vertical speed: {max_vertical_speed:.2f} m/s")
    print(f"Maximum running speed: {max_running_speed:.2f} km/h")
    print(f"Average running speed: {avg_running_speed:.2f} km/h")
    
    return {
        "num_jumps": num_jumps,
        "max_vertical_speed": max_vertical_speed,
        "max_running_speed": max_running_speed,
        "avg_running_speed": avg_running_speed,
        "jump_timestamps": data['timestamp'][jump_indices].tolist(),
        "filename": os.path.basename(file_path)
    }

def process_all_files():
    """Process all CSV files in the sensor_data directory"""
    sensor_data_dir = "sensor_data"
    
    if not os.path.exists(sensor_data_dir):
        print(f"Error: Directory '{sensor_data_dir}' not found")
        return
    
    results = []
    
    for filename in os.listdir(sensor_data_dir):
        if filename.endswith(".csv"):
            file_path = os.path.join(sensor_data_dir, filename)
            print(f"Processing {filename}...")
            result = analyze_ankle_data(file_path)
            results.append(result)
    
    # Save summary results to CSV
    if results:
        summary_df = pd.DataFrame(results)
        summary_df.to_csv("analysis_results/summary_results.csv", index=False)
        print(f"Summary saved to analysis_results/summary_results.csv")

if __name__ == "__main__":
    # Process all files in the sensor_data directory
    process_all_files()