import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt
import traceback

try:
    print("Starting sample data generation...")
    
    # Create sample data directory if it doesn't exist
    if not os.path.exists("sensor_data"):
        os.makedirs("sensor_data")
        print("Created sensor_data directory")
    
    # Parameters
    duration_ms = 60000  # 1 minute in milliseconds
    sampling_rate = 100  # Hz
    num_jumps = 30
    running_speed_kmh = 5.0
    filename = "sensor_data/ankle_jumps_running_sample.csv"
    
    # Generate timestamps (100 Hz sampling rate = 10ms intervals)
    timestamps = np.arange(0, duration_ms, 1000 / sampling_rate)
    num_samples = len(timestamps)
    
    print(f"Generating {num_samples} samples at {sampling_rate}Hz...")
    
    # Initialize arrays with gravity effect (-1g in Z direction when standing upright)
    accel_x = np.zeros(num_samples)  # No gravity component in X initially
    accel_y = np.zeros(num_samples)  # No gravity component in Y initially
    accel_z = np.ones(num_samples) * (-1.0)  # -1g in Z direction (gravity)
    
    # Add running pattern (roughly 5 km/h)
    # Convert km/h to m/s
    running_speed_ms = running_speed_kmh / 3.6  # 5 km/h = 1.39 m/s
    
    # Calculate cadence (steps per minute) based on stride length
    # Approx stride length for walking/slow running = 0.8m
    stride_length = 0.8  # meters
    steps_per_second = running_speed_ms / stride_length
    steps_per_minute = steps_per_second * 60
    
    # Generate running pattern with appropriate frequency
    running_freq = steps_per_minute / 60  # Hz
    t_sec = timestamps / 1000.0  # Convert to seconds
    
    # Add running motion (sine waves for forward and vertical motion)
    # X-axis: forward motion (with appropriate acceleration to achieve 5 km/h)
    running_phase = 2 * np.pi * running_freq * t_sec
    accel_x += 0.2 * np.sin(running_phase)  # Forward acceleration component
    
    # Z-axis: vertical bouncing during running (smaller amplitude than jumps)
    accel_z += 0.15 * np.abs(np.sin(running_phase))  # Vertical component of running
    
    print("Added running patterns")
    
    # Add 30 jumps distributed throughout the minute
    jump_times_ms = np.linspace(2000, 58000, num_jumps)  # Distribute jumps between 2s and 58s
    jump_indices = [int(t * sampling_rate / 1000) for t in jump_times_ms]
    
    # Create jumping motion - make jumps more pronounced
    jump_window = 40  # samples (400ms at 100Hz)
    jump_height = 3.0  # increased peak acceleration in g to make jumps more obvious
    
    for idx in jump_indices:
        if idx < num_samples - jump_window:
            # Create a more pronounced jump acceleration pattern
            # First: push-off phase (stronger positive acceleration)
            takeoff = np.hanning(jump_window)[:jump_window//3] * jump_height
            # Then: flight phase (near zero, slight negative for air resistance)
            flight = np.ones(jump_window//3) * -0.95  # In flight, only gravity applies
            # Finally: landing phase (stronger negative acceleration spike)
            landing = -np.hanning(jump_window)[:jump_window//3] * (jump_height * 1.5)
            
            # Combine phases
            jump_pattern = np.concatenate([takeoff, flight, landing])
            
            # Apply jump pattern to z-acceleration
            end_idx = min(idx + len(jump_pattern), num_samples)
            pattern_length = end_idx - idx
            accel_z[idx:end_idx] += jump_pattern[:pattern_length]
    
    print(f"Added {num_jumps} jumps to the data")
    
    # Add some noise to make it realistic
    accel_x += np.random.normal(0, 0.05, num_samples)
    accel_y += np.random.normal(0, 0.05, num_samples)
    accel_z += np.random.normal(0, 0.05, num_samples)
    
    # Generate gyroscope data (rotation rates during movement)
    gyro_x = np.random.normal(0, 0.2, num_samples)  # Minimal rotation in X
    gyro_y = np.random.normal(0, 0.2, num_samples)  # Minimal rotation in Y
    gyro_z = np.random.normal(0, 0.5, num_samples)  # More rotation around Z axis
    
    # Add rotational patterns during jumps and running
    for idx in jump_indices:
        if idx < num_samples - 20:
            gyro_x[idx:idx+20] += np.sin(np.linspace(0, np.pi, 20)) * 2  # Rotation during jump
            gyro_y[idx:idx+20] += np.sin(np.linspace(0, np.pi, 20)) * 1.5  # Rotation during jump
    
    # Add running cadence to gyroscope
    gyro_x += 0.3 * np.sin(running_phase)
    gyro_y += 0.4 * np.sin(running_phase)
    gyro_z += 0.2 * np.sin(running_phase)
    
    print("Generated gyroscope data")
    
    # Generate magnetometer data (typical values for indoor environment)
    mag_x = np.ones(num_samples) * 25 + np.random.normal(0, 1, num_samples)
    mag_y = np.ones(num_samples) * (-12) + np.random.normal(0, 1, num_samples)
    mag_z = np.ones(num_samples) * 8 + np.random.normal(0, 1, num_samples)
    
    # Create DataFrame
    data = pd.DataFrame({
        'timestamp': timestamps,
        'accelX': accel_x,
        'accelY': accel_y,
        'accelZ': accel_z,
        'gyroX': gyro_x,
        'gyroY': gyro_y,
        'gyroZ': gyro_z,
        'magX': mag_x,
        'magY': mag_y,
        'magZ': mag_z
    })
    
    print(f"Created DataFrame with {len(data)} rows")
    
    # Save to CSV
    data.to_csv(filename, index=False)
    print(f"Sample data created at {filename}")
    print(f"- Duration: 60 seconds (1 minute)")
    print(f"- Sample rate: {sampling_rate} Hz")
    print(f"- Number of jumps: {num_jumps}")
    print(f"- Running speed: {running_speed_kmh} km/h")
    
    # Plot the data for verification
    plt.figure(figsize=(12, 9))
    
    # Acceleration plot
    plt.subplot(2, 1, 1)
    plt.plot(timestamps/1000, accel_x, label='X (forward)')
    plt.plot(timestamps/1000, accel_y, label='Y (sideways)')
    plt.plot(timestamps/1000, accel_z, label='Z (vertical)')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Acceleration (g)')
    plt.title('Ankle Acceleration (30 Jumps + Running at 5 km/h)')
    plt.legend()
    plt.grid(True)
    
    # Highlight jump locations
    for jt in jump_times_ms/1000:
        plt.axvline(x=jt, color='r', alpha=0.2)
    
    # Gyroscope plot
    plt.subplot(2, 1, 2)
    plt.plot(timestamps/1000, gyro_x, label='X')
    plt.plot(timestamps/1000, gyro_y, label='Y')
    plt.plot(timestamps/1000, gyro_z, label='Z')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Angular velocity (deg/s)')
    plt.title('Gyroscope Data')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig("sensor_data/sample_data_visualization.png")
    print("Visualization saved to sensor_data/sample_data_visualization.png")
    
    # Don't show plot (it's better to just save it when running from command line)
    # plt.show() 

except Exception as e:
    print(f"Error: {str(e)}")
    print(traceback.format_exc()) 