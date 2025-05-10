import serial
import time
import csv
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import threading
import queue
import argparse
import random
from matplotlib.widgets import TextBox
from scipy.signal import find_peaks
from collections import deque
import matplotlib.patches as mpatches
from matplotlib.widgets import Button

class SensorDataVisualizer:
    def __init__(self, port='COM3', baud_rate=115200, simulation_mode=False):
        self.port = port
        self.baud_rate = baud_rate
        self.serial = None
        self.data_queue = queue.Queue()
        self.is_running = True
        self.simulation_mode = simulation_mode
        
        # Simulation variables
        self.current_steps = 0
        self.current_speed = 0
        self.current_displacement = 0
        self.current_accel_x = 0
        self.current_accel_y = 0
        self.current_accel_z = 9.81  # Starting with gravity
        self.activities = ['walking', 'running', 'jumping', 'standing']
        self.current_activity = 'standing'
        self.activity_duration = 0
        self.activity_change_threshold = 50
        
        # Orientation variables
        self.orientation = "normal"  # normal, upside_down, vertical, horizontal
        self.orientation_change_counter = 0
        self.orientation_change_threshold = 100  # Change orientation every ~10 seconds
        self.auto_change_orientation = False  # Start with manual orientation control
        
        # Activity detection
        self.accel_buffer_size = 20
        self.accel_buffer_x = deque(maxlen=self.accel_buffer_size)
        self.accel_buffer_y = deque(maxlen=self.accel_buffer_size)
        self.accel_buffer_z = deque(maxlen=self.accel_buffer_size)
        self.detected_activity = 'initializing'
        self.detection_accuracy = 0.0
        self.total_detections = 0
        self.correct_detections = 0
        
        # Data storage
        self.timestamps = []
        self.steps = []
        self.speeds = []
        self.activities = []
        self.displacements = []
        self.accel_x = []
        self.accel_y = []
        self.accel_z = []
        self.orientations = []
        
        # Initialize plots
        plt.style.use('dark_background')
        
        # Create display window
        self.display_fig = plt.figure(figsize=(8, 10), num='Sensor Display')
        self.display_ax = self.display_fig.add_subplot(111)
        self.display_ax.axis('off')
        self.display_text = self.display_ax.text(0.5, 0.5, '',
                                               horizontalalignment='center',
                                               verticalalignment='center',
                                               fontfamily='monospace',
                                               fontsize=12,
                                               color='lime')
        self.display_fig.canvas.manager.set_window_title('Sensor Display')
        
        # Add orientation control buttons if in simulation mode
        if simulation_mode:
            self.setup_orientation_controls()
            
        # Create graphs window
        self.graph_fig = plt.figure(figsize=(12, 10), num='Sensor Graphs')
        self.gs = self.graph_fig.add_gridspec(4, 1, height_ratios=[3, 3, 3, 3])
        
        # Create main plots
        self.ax1 = self.graph_fig.add_subplot(self.gs[0])
        self.ax2 = self.graph_fig.add_subplot(self.gs[1])
        self.ax3 = self.graph_fig.add_subplot(self.gs[2])
        self.ax4 = self.graph_fig.add_subplot(self.gs[3])
        
        self.graph_fig.suptitle('Simulated Sensor Data' if simulation_mode else 'Real-time Sensor Data')
        self.graph_fig.canvas.manager.set_window_title('Sensor Graphs')
        
        # Setup key press event for orientation control
        if simulation_mode:
            self.display_fig.canvas.mpl_connect('key_press_event', self.on_key_press)
            self.graph_fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        # Setup plots
        self.setup_plots()
        
        # Create CSV file for data logging
        self.csv_filename = f"sensor_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        with open(self.csv_filename, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Timestamp', 'Steps', 'Speed', 'Activity', 'Displacement', 
                            'AccelX', 'AccelY', 'AccelZ', 'DetectedActivity', 'Accuracy', 'Orientation'])

    def setup_orientation_controls(self):
        """Add orientation control buttons to the display figure."""
        # Create a new axis for the control panel
        control_ax = self.display_fig.add_axes([0.1, 0.05, 0.8, 0.3])
        control_ax.axis('off')
        
        # Title for the control panel
        control_ax.text(0.5, 0.95, 'ORIENTATION CONTROLS (or use keyboard keys 1-7)',
                      horizontalalignment='center',
                      verticalalignment='top',
                      fontsize=10,
                      color='white')
        
        # Create a legend showing keyboard shortcuts
        orientations = [
            ('1', 'Normal (Z up)'),
            ('2', 'Upside down (Z down)'),
            ('3', 'Horizontal X+'),
            ('4', 'Horizontal X-'),
            ('5', 'Vertical Y+'),
            ('6', 'Vertical Y-'),
            ('7', 'Tilted'),
            ('A', 'Auto-change'),
            ('S', 'Space = Activity change')
        ]
        
        # Create legend entries
        for i, (key, description) in enumerate(orientations):
            row = i // 3
            col = i % 3
            x_pos = 0.16 + col * 0.33
            y_pos = 0.75 - row * 0.15
            
            # Key circle
            circle = mpatches.Circle((x_pos - 0.06, y_pos), 0.02, color='yellow')
            control_ax.add_patch(circle)
            control_ax.text(x_pos - 0.06, y_pos, key, 
                         horizontalalignment='center',
                         verticalalignment='center',
                         fontsize=9, 
                         color='black',
                         fontweight='bold')
            
            # Description
            control_ax.text(x_pos + 0.04, y_pos, description, 
                         horizontalalignment='left',
                         verticalalignment='center',
                         fontsize=9, 
                         color='white')
    
    def on_key_press(self, event):
        """Handle key press events for orientation control."""
        if not self.simulation_mode:
            return
            
        if event.key == '1':
            self.orientation = "normal"
            print("Changed to NORMAL orientation (Z up)")
        elif event.key == '2':
            self.orientation = "upside_down"
            print("Changed to UPSIDE DOWN orientation (Z down)")
        elif event.key == '3':
            self.orientation = "horizontal_x+"
            print("Changed to HORIZONTAL X+ orientation")
        elif event.key == '4':
            self.orientation = "horizontal_x-"
            print("Changed to HORIZONTAL X- orientation")
        elif event.key == '5':
            self.orientation = "vertical_y+"
            print("Changed to VERTICAL Y+ orientation")
        elif event.key == '6':
            self.orientation = "vertical_y-"
            print("Changed to VERTICAL Y- orientation")
        elif event.key == '7':
            self.orientation = "tilted"
            print("Changed to TILTED orientation")
        elif event.key == 'a':
            self.auto_change_orientation = not self.auto_change_orientation
            status = "enabled" if self.auto_change_orientation else "disabled"
            print(f"Auto-change orientation {status}")
        elif event.key == ' ':
            # Change activity on spacebar
            self.current_activity = random.choice(['walking', 'running', 'jumping', 'standing'])
            print(f"Changed activity to: {self.current_activity.upper()}")

    def setup_plots(self):
        # Steps plot
        self.ax1.set_title('Step Count')
        self.ax1.set_ylabel('Steps')
        self.steps_line, = self.ax1.plot([], [], 'g-')
        
        # Speed plot
        self.ax2.set_title('Speed')
        self.ax2.set_ylabel('km/h')
        self.speed_line, = self.ax2.plot([], [], 'b-')
        
        # Displacement plot
        self.ax3.set_title('Displacement')
        self.ax3.set_ylabel('meters')
        self.displacement_line, = self.ax3.plot([], [], 'r-')
        
        # Acceleration plot
        self.ax4.set_title('Acceleration')
        self.ax4.set_ylabel('m/s²')
        self.accel_x_line, = self.ax4.plot([], [], 'r-', label='X')
        self.accel_y_line, = self.ax4.plot([], [], 'g-', label='Y')
        self.accel_z_line, = self.ax4.plot([], [], 'b-', label='Z')
        self.ax4.legend()
        
        # Adjust layout
        self.graph_fig.tight_layout()
        self.display_fig.tight_layout()

    def connect_serial(self):
        try:
            self.serial = serial.Serial(self.port, self.baud_rate)
            print(f"Connected to {self.port}")
            return True
        except serial.SerialException as e:
            print(f"Error connecting to {self.port}: {e}")
            return False

    def read_serial_data(self):
        while self.is_running:
            if self.serial and self.serial.in_waiting:
                try:
                    line = self.serial.readline().decode('utf-8').strip()
                    if line:
                        # Parse the data
                        # Expected format: steps,speed,displacement,activity
                        data = line.split(',')
                        if len(data) == 4:
                            timestamp = time.time()
                            steps = int(data[0])
                            speed = float(data[1])
                            displacement = float(data[2])
                            activity = data[3]
                            
                            # Add to queue for plotting
                            self.data_queue.put((timestamp, steps, speed, activity, displacement))
                            
                            # Save to CSV
                            with open(self.csv_filename, 'a', newline='') as file:
                                writer = csv.writer(file)
                                writer.writerow([timestamp, steps, speed, activity, displacement])
                            
                except Exception as e:
                    print(f"Error reading data: {e}")
            time.sleep(0.01)

    def detect_orientation(self, accel_x, accel_y, accel_z):
        """Detect the orientation of the device based on gravity vector."""
        # Calculate the magnitude of acceleration
        magnitude = np.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        
        # Normalize the acceleration values
        if magnitude > 0:
            x_norm = accel_x / magnitude
            y_norm = accel_y / magnitude
            z_norm = accel_z / magnitude
        else:
            return "normal"  # Default to normal if no significant acceleration
        
        # Determine orientation based on gravity direction
        if z_norm > 0.7:
            return "normal"  # Z is pointing up (normal orientation)
        elif z_norm < -0.7:
            return "upside_down"  # Z is pointing down
        elif x_norm > 0.7:
            return "horizontal_x+"  # X is pointing up
        elif x_norm < -0.7:
            return "horizontal_x-"  # X is pointing down
        elif y_norm > 0.7:
            return "vertical_y+"  # Y is pointing up
        elif y_norm < -0.7:
            return "vertical_y-"  # Y is pointing down
        else:
            return "tilted"  # Some other orientation

    def adjust_data_for_orientation(self, x, y, z, orientation):
        """Transform acceleration data based on device orientation."""
        if orientation == "normal":
            return x, y, z
        elif orientation == "upside_down":
            return -x, -y, -z
        elif orientation == "horizontal_x+":
            return z, y, -x
        elif orientation == "horizontal_x-":
            return -z, y, x
        elif orientation == "vertical_y+":
            return x, z, -y
        elif orientation == "vertical_y-":
            return x, -z, y
        else:  # tilted or other
            return x, y, z  # No adjustment for tilted

    def detect_activity_from_accelerometer(self, accel_x, accel_y, accel_z):
        # Detect orientation
        orientation = self.detect_orientation(accel_x, accel_y, accel_z)
        
        # Adjust acceleration values based on orientation
        adj_x, adj_y, adj_z = self.adjust_data_for_orientation(accel_x, accel_y, accel_z, orientation)
        
        # Add adjusted values to buffer
        self.accel_buffer_x.append(adj_x)
        self.accel_buffer_y.append(adj_y)
        self.accel_buffer_z.append(adj_z)
        
        # Store orientation for display
        self.orientation = orientation
        
        # Wait until buffer is filled
        if len(self.accel_buffer_x) < self.accel_buffer_size:
            return "initializing"
        
        # Calculate features
        x_std = np.std(self.accel_buffer_x)
        y_std = np.std(self.accel_buffer_y)
        z_std = np.std(self.accel_buffer_z)
        
        total_variance = x_std + y_std + z_std
        
        # Simple classification based on variance and patterns
        if total_variance < 0.3:
            activity = "standing"
        elif 0.3 <= total_variance < 1.0:
            activity = "walking"
        elif total_variance < 2.5:
            activity = "running"
        else:
            # For any orientation, look for large variations in the predominant axis
            max_std = max(x_std, y_std, z_std)
            if max_std > 1.5:
                activity = "jumping"
            else:
                activity = "running"
        
        # Track accuracy (only in simulation mode)
        if self.simulation_mode:
            self.total_detections += 1
            if activity == self.current_activity:
                self.correct_detections += 1
            self.detection_accuracy = (self.correct_detections / self.total_detections) * 100
            
        return activity

    def update_display(self, steps, speed, activity, displacement, accel_x, accel_y, accel_z):
        # Update activity detection
        self.detected_activity = self.detect_activity_from_accelerometer(accel_x, accel_y, accel_z)
        
        # Color code for accuracy
        accuracy_color = "\033[32m"  # Green
        if self.detected_activity != activity:
            accuracy_color = "\033[31m"  # Red
        
        # Display simulated vs detected activities only in simulation mode
        activity_comparison = ""
        if self.simulation_mode:
            activity_comparison = f"""║ Actual: {activity.upper():^20} ║
║ Detected: {self.detected_activity.upper():^18} ║
║ Detection Accuracy: {self.detection_accuracy:^11.1f}% ║"""
        
        # Get orientation description
        orientation_desc = self.get_orientation_description(self.orientation)
        orientation_display = f"""║──────────────────────────────────║
║ Device Orientation:              ║
║ {orientation_desc:^30} ║"""
        
        # Show auto/manual mode
        if self.simulation_mode:
            mode = "AUTO" if self.auto_change_orientation else "MANUAL"
            orientation_display += f"""
║ Mode: {mode:^26} ║"""
        
        display_text = f"""
╔══════════════════════════════════╗
║ Activity: {self.detected_activity.upper():^20} ║
{activity_comparison if activity_comparison else ""}
{orientation_display}
║──────────────────────────────────║
║ Steps: {steps:^24d} ║
║ Speed: {speed:^22.2f} km/h ║
║ Distance: {displacement:^20.2f} m ║
║──────────────────────────────────║
║ Acceleration (m/s²):             ║
║ X: {accel_x:^24.2f} ║
║ Y: {accel_y:^24.2f} ║
║ Z: {accel_z:^24.2f} ║
╚══════════════════════════════════╝
"""
        self.display_text.set_text(display_text)

    def get_orientation_description(self, orientation):
        """Get a user-friendly description of the orientation."""
        descriptions = {
            "normal": "NORMAL (Z UP)",
            "upside_down": "UPSIDE DOWN (Z DOWN)",
            "horizontal_x+": "HORIZONTAL (X UP)",
            "horizontal_x-": "HORIZONTAL (X DOWN)",
            "vertical_y+": "VERTICAL (Y UP)",
            "vertical_y-": "VERTICAL (Y DOWN)",
            "tilted": "TILTED"
        }
        return descriptions.get(orientation, orientation.upper())

    def generate_activity_acceleration(self):
        # Periodically change orientation in auto mode
        if self.simulation_mode and self.auto_change_orientation:
            self.orientation_change_counter += 1
            if self.orientation_change_counter >= self.orientation_change_threshold:
                self.orientation_change_counter = 0
                # Randomly select a new orientation
                possible_orientations = ["normal", "upside_down", "horizontal_x+", "horizontal_x-", 
                                        "vertical_y+", "vertical_y-"]
                chosen_orientation = random.choice(possible_orientations)
                self.orientation = chosen_orientation
                print(f"Auto-changed device orientation to: {self.get_orientation_description(chosen_orientation)}")
        
        # Generate accelerations based on activity
        if self.current_activity == 'standing':
            # Standing: minimal movement, mostly gravity
            base_x = random.uniform(-0.1, 0.1)
            base_y = random.uniform(-0.1, 0.1)
            base_z = 9.81 + random.uniform(-0.1, 0.1)
        elif self.current_activity == 'walking':
            # Walking: periodic motion in all axes
            t = time.time()
            base_x = 0.5 * np.sin(2 * np.pi * t) + random.uniform(-0.2, 0.2)
            base_y = 0.5 * np.cos(2 * np.pi * t) + random.uniform(-0.2, 0.2)
            base_z = 9.81 + np.sin(4 * np.pi * t) + random.uniform(-0.3, 0.3)
        elif self.current_activity == 'running':
            # Running: stronger periodic motion
            t = time.time()
            base_x = 1.5 * np.sin(3 * np.pi * t) + random.uniform(-0.4, 0.4)
            base_y = 1.5 * np.cos(3 * np.pi * t) + random.uniform(-0.4, 0.4)
            base_z = 9.81 + 2 * np.sin(6 * np.pi * t) + random.uniform(-0.5, 0.5)
        else:  # jumping
            # Jumping: sudden changes in vertical acceleration
            if random.random() < 0.1:  # 10% chance of jump
                base_z = random.uniform(15, 20)
            else:
                base_z = 9.81 + random.uniform(-1, 1)
            base_x = random.uniform(-2, 2)
            base_y = random.uniform(-2, 2)
            
        # Apply orientation transformation to the simulated data
        if hasattr(self, 'orientation') and self.orientation != "normal":
            # Transform accelerations based on orientation
            if self.orientation == "upside_down":
                self.current_accel_x = -base_x
                self.current_accel_y = -base_y
                self.current_accel_z = -base_z
            elif self.orientation == "horizontal_x+":
                self.current_accel_x = base_z - 9.81
                self.current_accel_y = base_y
                self.current_accel_z = -base_x
            elif self.orientation == "horizontal_x-":
                self.current_accel_x = -(base_z - 9.81)
                self.current_accel_y = base_y
                self.current_accel_z = base_x
            elif self.orientation == "vertical_y+":
                self.current_accel_x = base_x
                self.current_accel_y = base_z - 9.81
                self.current_accel_z = -base_y
            elif self.orientation == "vertical_y-":
                self.current_accel_x = base_x
                self.current_accel_y = -(base_z - 9.81)
                self.current_accel_z = base_y
            elif self.orientation == "tilted":
                # Create a random tilt
                angle_x = np.pi/4 * random.uniform(-1, 1)  # +/- 45 degrees
                angle_y = np.pi/4 * random.uniform(-1, 1)  # +/- 45 degrees
                
                # Rotate the acceleration vector
                sin_x, cos_x = np.sin(angle_x), np.cos(angle_x)
                sin_y, cos_y = np.sin(angle_y), np.cos(angle_y)
                
                # Apply rotation matrices
                # First rotate around x-axis
                y_rot = base_y * cos_x - base_z * sin_x
                z_rot = base_y * sin_x + base_z * cos_x
                
                # Then rotate around y-axis
                x_rot = base_x * cos_y + z_rot * sin_y
                z_rot_final = -base_x * sin_y + z_rot * cos_y
                
                self.current_accel_x = x_rot
                self.current_accel_y = y_rot
                self.current_accel_z = z_rot_final
            else:
                self.current_accel_x = base_x
                self.current_accel_y = base_y
                self.current_accel_z = base_z
        else:
            # Normal orientation
            self.current_accel_x = base_x
            self.current_accel_y = base_y
            self.current_accel_z = base_z

    def generate_dummy_data(self):
        while self.is_running:
            # Simulate activity changes
            self.activity_duration += 1
            if self.activity_duration >= self.activity_change_threshold:
                self.current_activity = random.choice(['walking', 'running', 'jumping', 'standing'])
                self.activity_duration = 0
            
            # Generate acceleration data based on activity
            self.generate_activity_acceleration()
                
            # Generate other data based on activity
            if self.current_activity == 'standing':
                speed_change = random.uniform(-0.1, 0.1)
                step_change = 0
            elif self.current_activity == 'walking':
                speed_change = random.uniform(-0.2, 0.5)
                step_change = random.randint(1, 2)
            elif self.current_activity == 'running':
                speed_change = random.uniform(-0.3, 1.0)
                step_change = random.randint(2, 4)
            else:  # jumping
                speed_change = random.uniform(-0.5, 1.5)
                step_change = random.randint(0, 2)
                
            # Update current values
            self.current_speed = max(0, min(20, self.current_speed + speed_change))
            self.current_steps += step_change
            self.current_displacement += self.current_speed * 0.1
            
            # Detect activity
            detected = self.detect_activity_from_accelerometer(
                self.current_accel_x, 
                self.current_accel_y, 
                self.current_accel_z
            )
            
            # Add to queue
            self.data_queue.put((
                time.time(),
                self.current_steps,
                self.current_speed,
                self.current_activity,
                self.current_displacement,
                self.current_accel_x,
                self.current_accel_y,
                self.current_accel_z
            ))
            
            # Save to CSV
            with open(self.csv_filename, 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([time.time(), self.current_steps, self.current_speed,
                               self.current_activity, self.current_displacement,
                               self.current_accel_x, self.current_accel_y, self.current_accel_z,
                               detected, self.detection_accuracy, self.orientation])
            
            time.sleep(0.1)

    def update_plot(self, frame):
        while not self.data_queue.empty():
            timestamp, steps, speed, activity, displacement, accel_x, accel_y, accel_z = self.data_queue.get()
            
            self.timestamps.append(timestamp)
            self.steps.append(steps)
            self.speeds.append(speed)
            self.displacements.append(displacement)
            self.accel_x.append(accel_x)
            self.accel_y.append(accel_y)
            self.accel_z.append(accel_z)
            
            # Update display
            self.update_display(steps, speed, activity, displacement, accel_x, accel_y, accel_z)
            
            # Keep last 100 points
            if len(self.timestamps) > 100:
                self.timestamps = self.timestamps[-100:]
                self.steps = self.steps[-100:]
                self.speeds = self.speeds[-100:]
                self.displacements = self.displacements[-100:]
                self.accel_x = self.accel_x[-100:]
                self.accel_y = self.accel_y[-100:]
                self.accel_z = self.accel_z[-100:]
            
            # Update plots
            time_points = np.array(self.timestamps) - self.timestamps[0]
            
            self.steps_line.set_data(time_points, self.steps)
            self.speed_line.set_data(time_points, self.speeds)
            self.displacement_line.set_data(time_points, self.displacements)
            self.accel_x_line.set_data(time_points, self.accel_x)
            self.accel_y_line.set_data(time_points, self.accel_y)
            self.accel_z_line.set_data(time_points, self.accel_z)
            
            # Adjust axes
            for ax in [self.ax1, self.ax2, self.ax3, self.ax4]:
                ax.relim()
                ax.autoscale_view()
                ax.set_xlim(min(time_points), max(time_points))

            # Draw both figures
            self.display_fig.canvas.draw()
            self.graph_fig.canvas.draw()
        
        return [self.display_text]

    def run(self):
        if self.simulation_mode:
            # Start simulation thread
            sim_thread = threading.Thread(target=self.generate_dummy_data)
            sim_thread.daemon = True
            sim_thread.start()
        else:
            if not self.connect_serial():
                return
            # Start serial reading thread
            serial_thread = threading.Thread(target=self.read_serial_data)
            serial_thread.daemon = True
            serial_thread.start()
        
        # Start animation for both windows
        anim = FuncAnimation(self.display_fig, self.update_plot, interval=50, blit=True)
        plt.show()
        
        # Cleanup
        self.is_running = False
        if self.serial:
            self.serial.close()

    def generate_summary(self):
        if len(self.steps) == 0:
            return
        
        print("\nSession Summary:")
        print("-" * 20)
        print(f"Total Steps: {max(self.steps)}")
        print(f"Max Speed: {max(self.speeds):.2f} km/h")
        print(f"Average Speed: {np.mean(self.speeds):.2f} km/h")
        print(f"Total Distance: {max(self.displacements):.2f} m")
        print(f"Max Acceleration (m/s²):")
        print(f"  X: {max(map(abs, self.accel_x)):.2f}")
        print(f"  Y: {max(map(abs, self.accel_y)):.2f}")
        print(f"  Z: {max(map(abs, self.accel_z)):.2f}")
        
        if self.simulation_mode:
            print(f"Activity Detection Accuracy: {self.detection_accuracy:.1f}%")
            
        print(f"Duration: {(self.timestamps[-1] - self.timestamps[0]):.2f} seconds")
        print(f"Data saved to: {self.csv_filename}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Sensor Data Visualizer')
    parser.add_argument('--port', type=str, default='COM3', help='Serial port (default: COM3)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--simulation', action='store_true', help='Run in simulation mode with dummy data')
    
    args = parser.parse_args()
    
    visualizer = SensorDataVisualizer(port=args.port, baud_rate=args.baud, simulation_mode=args.simulation)
    try:
        visualizer.run()
    except KeyboardInterrupt:
        print("\nStopping visualization...")
    finally:
        visualizer.generate_summary() 