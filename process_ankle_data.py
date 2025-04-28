import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
from scipy.signal import find_peaks, butter, filtfilt

def load_csv_data(file_path):
    """Load sensor data from CSV file"""
    return pd.read_csv(file_path)

def apply_low_pass_filter(data, cutoff=2.0, fs=32.9, order=4):
    """Apply a low-pass filter to remove noise"""
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data)

def calculate_activity_level(accel_data):
    """Calculate the overall activity level in the data"""
    # Calculate z-score normalized data
    norm_data = (accel_data - np.mean(accel_data)) / np.std(accel_data)
    
    # Calculate amplitude of movement
    amplitude = np.max(norm_data) - np.min(norm_data)
    
    # Calculate variance and standard deviation
    variance = np.var(norm_data)
    std_dev = np.std(norm_data)
    
    # Calculate energy (sum of squared values)
    energy = np.sum(norm_data**2) / len(norm_data)
    
    return {
        'amplitude': amplitude,
        'variance': variance,
        'std_dev': std_dev,
        'energy': energy
    }

def is_jumping_activity(accel_data, threshold=1.5):
    """Determine if the acceleration data contains jumping activity"""
    # Calculate activity metrics
    activity = calculate_activity_level(accel_data)
    
    # Print the activity metrics for debugging
    print(f"Activity metrics: amplitude={activity['amplitude']:.2f}, "
          f"variance={activity['variance']:.2f}, energy={activity['energy']:.2f}")
    
    # Adjust jumping detection thresholds
    is_jumping = (activity['energy'] > 0.8 and 
                 activity['amplitude'] > 4.0 and
                 activity['variance'] > 0.5)
    
    # Alternative check for jump patterns
    # Check for distinctive jump pattern: high peaks with significant drops
    if not is_jumping:
        # Normalize the data
        norm_data = (accel_data - np.mean(accel_data)) / np.std(accel_data)
        
        # Look for large vertical movements (typical of jumps)
        peaks, _ = find_peaks(norm_data, prominence=1.0, distance=10)
        valleys, _ = find_peaks(-norm_data, prominence=1.0, distance=10)
        
        # Check if there are clear take-off and landing patterns
        if len(peaks) >= 2 and len(valleys) >= 2:
            # Calculate the range of peaks and valleys
            peak_valley_range = np.max(norm_data[peaks]) - np.min(norm_data[valleys])
            
            # If the range is large, it's likely jumping
            if peak_valley_range > 3.5:
                is_jumping = True
                print(f"Jump pattern detected with peak-valley range: {peak_valley_range:.2f}")
    
    print(f"Is jumping activity: {is_jumping}")
    return is_jumping

def detect_steps(accel_z):
    """
    Detect steps based on Z-axis acceleration patterns.
    
    For ankle-mounted sensors, steps typically show as:
    1. Small oscillations in the vertical direction
    2. More frequent and regular than jumps
    3. Lower amplitude than jumps
    """
    # First, normalize the data to have zero mean
    accel_z_norm = accel_z - np.mean(accel_z)
    
    # Calculate signal statistics
    std_dev = np.std(accel_z_norm)
    
    # Parameters for step detection - more selective
    min_distance = 25  # About 750ms at 32.9Hz (typical interval between steps)
    
    # Find primary peaks (step phase) with more selective parameters
    peaks, _ = find_peaks(accel_z_norm, 
                          prominence=0.35 * std_dev,  # Higher prominence threshold
                          distance=min_distance,
                          width=2)  # Require wider peaks
    
    # Further filter peaks based on regularity
    if len(peaks) > 2:
        # Calculate intervals between steps
        intervals = np.diff(peaks)
        
        # Calculate mean and standard deviation of intervals
        mean_interval = np.mean(intervals)
        std_interval = np.std(intervals)
        
        # Keep only peaks that have reasonable intervals
        filtered_peaks = [peaks[0]]  # Keep first peak
        
        for i in range(1, len(peaks)):
            curr_interval = peaks[i] - filtered_peaks[-1]
            
            # Check if this interval is reasonable (within 30% of mean interval)
            if abs(curr_interval - mean_interval) <= 0.3 * mean_interval:
                filtered_peaks.append(peaks[i])
        
        peaks = np.array(filtered_peaks)
    
    print(f"Found {len(peaks)} potential steps")
    
    return np.array(peaks)

def detect_jumps(accel_z):
    """
    Detect jumps based on Z-axis acceleration patterns using threshold-based detection.
    Similar to Arduino BMI270 implementation.
    
    Args:
        accel_z: Z-axis acceleration data in g units
    Returns:
        Arrays of jump indices, takeoff points, and landing points
    """
    # Convert acceleration to m/s^2
    accel_z_ms2 = accel_z * 9.80665
    
    # Initialize arrays for storing jump information
    takeoff_points = []
    landing_points = []
    
    # State variables
    in_air = False
    takeoff_idx = 0
    
    # Thresholds (similar to Arduino code but slightly adjusted for noise)
    TAKEOFF_THRESHOLD = 12.0  # m/s^2 (slightly lower than Arduino for better sensitivity)
    LANDING_THRESHOLD = -12.0  # m/s^2
    MIN_HANG_TIME = 100  # ms
    MAX_HANG_TIME = 1000  # ms
    
    # Detect jumps
    for i in range(len(accel_z_ms2)):
        if not in_air and accel_z_ms2[i] > TAKEOFF_THRESHOLD:
            # Takeoff detected
            in_air = True
            takeoff_idx = i
        
        elif in_air and accel_z_ms2[i] < LANDING_THRESHOLD:
            # Landing detected - calculate hang time
            landing_idx = i
            
            # Calculate hang time in milliseconds
            # Note: at 32.9Hz sample rate, each index represents ~30.4ms
            hang_time = (landing_idx - takeoff_idx) * (1000.0 / 32.9)
            
            # Only count as jump if hang time is reasonable
            if MIN_HANG_TIME <= hang_time <= MAX_HANG_TIME:
                takeoff_points.append(takeoff_idx)
                landing_points.append(landing_idx)
            
            in_air = False
    
    # Convert to numpy arrays
    takeoff_points = np.array(takeoff_points)
    landing_points = np.array(landing_points)
    
    print(f"Found {len(takeoff_points)} distinct jumps")
    
    return takeoff_points, takeoff_points, landing_points

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
    
    # Scale velocity to more realistic values
    max_abs_velocity = np.max(np.abs(velocity))
    if max_abs_velocity > 0:
        velocity = velocity * (5.0 / max_abs_velocity)  # Scale to max ~5 m/s which is realistic for jumps
    
    return velocity

def calculate_movement_metrics(accel_z, timestamps, peak_indices):
    """Calculate metrics for each movement (step or jump)"""
    movement_metrics = []
    
    for i in range(len(peak_indices)):
        peak = peak_indices[i]
        
        # Calculate step/jump power
        accel_z_norm = accel_z - np.mean(accel_z)
        power = abs(accel_z_norm[peak])
        
        # Approximate duration (use fixed value for steps)
        duration = 300  # ms
        
        movement_metrics.append({
            'peak_idx': peak,
            'power': power,
            'duration_ms': duration
        })
    
    return movement_metrics

def calculate_walking_speed(step_indices, timestamps, stride_length=0.7):
    """
    Calculate walking speed based on step cadence and stride length.
    
    Args:
        step_indices: Indices of detected steps
        timestamps: Array of timestamp values
        stride_length: Average stride length in meters (default 0.7m)
    
    Returns:
        Dictionary with walking speed metrics
    """
    if len(step_indices) < 2:
        return {
            'avg_speed_kmh': 0,
            'max_speed_kmh': 0,
            'speeds': []
        }
    
    # Calculate time between steps in seconds
    step_times = []
    speeds = []
    
    for i in range(1, len(step_indices)):
        current_step = step_indices[i]
        prev_step = step_indices[i-1]
        
        # Time between steps in seconds
        time_diff_sec = (timestamps[current_step] - timestamps[prev_step]) / 1000.0
        
        if time_diff_sec > 0:
            # Calculate speed for this step (stride length / time)
            speed_ms = stride_length / time_diff_sec
            
            # Convert to km/h
            speed_kmh = speed_ms * 3.6
            
            step_times.append(time_diff_sec)
            speeds.append(speed_kmh)
    
    # Calculate average and maximum speeds
    avg_speed_kmh = np.mean(speeds) if speeds else 0
    max_speed_kmh = np.max(speeds) if speeds else 0
    
    return {
        'avg_speed_kmh': avg_speed_kmh,
        'max_speed_kmh': max_speed_kmh,
        'speeds': speeds
    }

def analyze_ankle_data(file_path):
    """Analyze ankle movement data to detect activity type and movement patterns"""
    # Load data
    data = load_csv_data(file_path)
    
    # Use the known sample rate
    sample_rate = 32.9  # Hz
    
    print(f"Using sample rate: {sample_rate:.1f} Hz")
    
    # Apply low-pass filter to acceleration data - higher cutoff for jumps
    accel_z_filtered = apply_low_pass_filter(data['accelZ'], fs=sample_rate, cutoff=3.0)
    
    # Determine if the activity contains jumps
    is_jumping = is_jumping_activity(accel_z_filtered)
    
    if is_jumping:
        # Detect jumps with threshold-based detection
        jump_indices, takeoff_points, landing_points = detect_jumps(accel_z_filtered)
        num_movements = len(jump_indices)
        movement_type = "jumps"
        
        # Calculate jump metrics
        if num_movements > 0:
            jump_metrics = []
            for i in range(len(takeoff_points)):
                takeoff = takeoff_points[i]
                landing = landing_points[i]
                
                # Calculate hang time in seconds
                hang_time_ms = (data['timestamp'][landing] - data['timestamp'][takeoff])
                hang_time_sec = hang_time_ms / 1000.0
                
                # Calculate jump height using hang time formula (same as Arduino code)
                # h = 1/2 * g * (t/2)^2 where t is hang time
                jump_height = 0.5 * 9.80665 * pow(hang_time_sec / 2, 2)
                
                # Calculate jump power from takeoff acceleration
                takeoff_accel = accel_z_filtered[takeoff] * 9.80665  # Convert to m/s^2
                
                jump_metrics.append({
                    'takeoff_idx': takeoff,
                    'landing_idx': landing,
                    'duration_ms': hang_time_ms,
                    'height_m': jump_height,
                    'takeoff_accel': takeoff_accel
                })
            
            movement_metrics = jump_metrics
            
            # Calculate average and max jump heights
            avg_height = np.mean([metric['height_m'] for metric in jump_metrics])
            max_height = np.max([metric['height_m'] for metric in jump_metrics])
            
            # Calculate vertical speed
            vertical_speed = calculate_vertical_speed(accel_z_filtered, data['timestamp'])
            max_vertical_speed = np.max(np.abs(vertical_speed))
            
            # Calculate walking speed
            walking_speed = calculate_walking_speed(jump_indices, data['timestamp'])
            
            # Plot results for jumps
            plt.figure(figsize=(15, 15))
            
            # Plot 1: Acceleration with Jumps
            plt.subplot(3, 1, 1)
            plt.plot(data['timestamp'], accel_z_filtered, label='Z (vertical)')
            plt.scatter(data['timestamp'][takeoff_points], accel_z_filtered[takeoff_points], 
                        color='green', marker='^', label=f'Takeoff ({num_movements})')
            plt.scatter(data['timestamp'][landing_points], accel_z_filtered[landing_points],
                        color='red', marker='v', label='Landing')
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
            plt.title(f'Vertical Speed (Max: {max_vertical_speed:.2f} m/s)')
            plt.grid(True)
            
            # Plot 3: Jump Heights
            heights = [metric['height_m'] for metric in jump_metrics]
            jump_numbers = range(1, len(heights) + 1)
            
            plt.subplot(3, 1, 3)
            plt.bar(jump_numbers, heights)
            plt.axhline(y=avg_height, color='r', linestyle='--', label=f'Avg: {avg_height:.2f}m')
            plt.xlabel('Jump Number')
            plt.ylabel('Jump Height (m)')
            plt.title(f'Jump Heights (Max: {max_height:.2f}m, Avg: {avg_height:.2f}m)')
            plt.grid(True, axis='y')
            plt.legend()
        else:
            # Handle case with no jumps detected
            vertical_speed = calculate_vertical_speed(accel_z_filtered, data['timestamp'])
            max_vertical_speed = np.max(np.abs(vertical_speed))
            avg_height = 0
            max_height = 0
            
            # Calculate walking speed
            walking_speed = calculate_walking_speed(jump_indices, data['timestamp'])
            
            # Plot results for no jumps
            plt.figure(figsize=(10, 10))
            
            plt.subplot(2, 1, 1)
            plt.plot(data['timestamp'], accel_z_filtered, label='Z (vertical)')
            plt.xlabel('Time (ms)')
            plt.ylabel('Vertical Acceleration (g)')
            plt.title('Ankle Movement - No Jumps Detected')
            plt.legend()
            plt.grid(True)
            
            plt.subplot(2, 1, 2)
            plt.plot(data['timestamp'], vertical_speed)
            plt.axhline(y=0, color='k', linestyle='-', alpha=0.3)
            plt.xlabel('Time (ms)')
            plt.ylabel('Vertical Speed (m/s)')
            plt.title(f'Vertical Speed (Max: {max_vertical_speed:.2f} m/s)')
            plt.grid(True)
    else:
        # Detect steps instead of jumps
        step_indices = detect_steps(accel_z_filtered)
        num_movements = len(step_indices)
        movement_type = "steps"
        
        # Calculate step metrics
        movement_metrics = calculate_movement_metrics(accel_z_filtered, data['timestamp'], step_indices)
        
        # Calculate cadence (steps per minute)
        if num_movements > 1 and len(data['timestamp']) > 1:
            total_time_sec = (data['timestamp'].iloc[-1] - data['timestamp'].iloc[0]) / 1000.0
            cadence = (num_movements / total_time_sec) * 60  # steps per minute
        else:
            cadence = 0
        
        # Calculate walking speed
        walking_speed = calculate_walking_speed(step_indices, data['timestamp'])
        
        # Calculate vertical speed for consistency
        vertical_speed = calculate_vertical_speed(accel_z_filtered, data['timestamp'])
        max_vertical_speed = np.max(np.abs(vertical_speed))
        
        # For step analysis, we don't have jump heights
        avg_height = 0
        max_height = 0
        
        # Plot results for steps
        plt.figure(figsize=(15, 15))
        
        # Plot 1: Acceleration with Steps
        plt.subplot(3, 1, 1)
        plt.plot(data['timestamp'], accel_z_filtered, label='Z (vertical)')
        plt.scatter(data['timestamp'][step_indices], accel_z_filtered[step_indices], 
                    color='blue', marker='o', label=f'Steps ({num_movements})')
        plt.xlabel('Time (ms)')
        plt.ylabel('Vertical Acceleration (g)')
        plt.title(f'Ankle Movement - Step Detection (Cadence: {cadence:.1f} steps/min)')
        plt.legend()
        plt.grid(True)
        
        # Plot 2: Vertical Speed
        plt.subplot(3, 1, 2)
        plt.plot(data['timestamp'], vertical_speed)
        plt.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        plt.xlabel('Time (ms)')
        plt.ylabel('Vertical Speed (m/s)')
        plt.title(f'Vertical Speed (Max: {max_vertical_speed:.2f} m/s)')
        plt.grid(True)
        
        # Plot 3: Walking Speed (if steps detected)
        if num_movements > 1:
            plt.subplot(3, 1, 3)
            # Create timestamps for speed plot (midpoint between consecutive steps)
            speed_timestamps = []
            for i in range(1, len(step_indices)):
                speed_timestamps.append((data['timestamp'][step_indices[i]] + 
                                        data['timestamp'][step_indices[i-1]]) / 2)
            
            plt.plot(speed_timestamps, walking_speed['speeds'], marker='o')
            plt.axhline(y=walking_speed['avg_speed_kmh'], color='r', linestyle='--', 
                        label=f'Avg: {walking_speed["avg_speed_kmh"]:.2f} km/h')
            plt.xlabel('Time (ms)')
            plt.ylabel('Walking Speed (km/h)')
            plt.title(f'Walking Speed (Max: {walking_speed["max_speed_kmh"]:.2f} km/h, Avg: {walking_speed["avg_speed_kmh"]:.2f} km/h)')
            plt.grid(True)
            plt.legend()
    
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
    print(f"Detected movement: {movement_type}")
    print(f"Number of {movement_type} detected: {num_movements}")
    print(f"Maximum vertical speed: {max_vertical_speed:.2f} m/s")
    
    if is_jumping and num_movements > 0:
        print(f"Average jump height: {avg_height:.2f} m")
        print(f"Maximum jump height: {max_height:.2f} m")
    elif not is_jumping and num_movements > 0:
        print(f"Cadence: {cadence:.1f} steps/minute")
        print(f"Average walking speed: {walking_speed['avg_speed_kmh']:.2f} km/h")
        print(f"Maximum walking speed: {walking_speed['max_speed_kmh']:.2f} km/h")
    
    # Save detailed metrics to CSV
    if num_movements > 0:
        metrics_df = pd.DataFrame(movement_metrics)
        if is_jumping:
            metrics_df['jump_number'] = range(1, len(metrics_df) + 1)
        else:
            metrics_df['step_number'] = range(1, len(metrics_df) + 1)
        
        metrics_df.to_csv(os.path.join(output_dir, f"{base_name}_metrics.csv"), index=False)
        print(f"Movement metrics saved to {os.path.join(output_dir, f'{base_name}_metrics.csv')}")
    
    return {
        "movement_type": movement_type,
        "num_movements": num_movements,
        "max_vertical_speed": max_vertical_speed,
        "filename": os.path.basename(file_path),
        "walking_speed_avg_kmh": walking_speed['avg_speed_kmh'] if movement_type == "steps" else 0,
        "walking_speed_max_kmh": walking_speed['max_speed_kmh'] if movement_type == "steps" else 0
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
    process_all_files()