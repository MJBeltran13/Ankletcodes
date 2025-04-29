import asyncio
from bleak import BleakClient, BleakScanner
import time
from datetime import datetime

# UUIDs must match those in the Arduino code
SENSOR_SERVICE_UUID = "19B10000-E8F2-537E-4F6C-D104768A1214"
SENSOR_CHARACTERISTIC_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"

# Constants for speed calculation
STRIDE_LENGTH_METERS = 0.7  # Average stride length - can be adjusted
WINDOW_SIZE_SECONDS = 60    # Window size for calculating steps per minute

class StepTracker:
    def __init__(self, stride_length=STRIDE_LENGTH_METERS):
        self.stride_length = stride_length
        self.steps = []
        self.last_calculation_time = time.time()
    
    def add_step(self):
        current_time = time.time()
        self.steps.append(current_time)
        
        # Remove steps older than WINDOW_SIZE_SECONDS
        cutoff_time = current_time - WINDOW_SIZE_SECONDS
        self.steps = [step for step in self.steps if step > cutoff_time]
    
    def calculate_speed(self):
        current_time = time.time()
        
        # Only calculate every second to avoid too frequent updates
        if current_time - self.last_calculation_time < 1.0:
            return None
            
        self.last_calculation_time = current_time
        
        if len(self.steps) < 2:
            return 0.0
            
        # Calculate steps per minute
        time_window = min(WINDOW_SIZE_SECONDS, self.steps[-1] - self.steps[0])
        steps_per_minute = (len(self.steps) / time_window) * 60
        
        # Calculate speed in meters per minute
        speed_mpm = steps_per_minute * self.stride_length
        
        # Convert to km/h
        speed_kmh = (speed_mpm * 60) / 1000
        
        return speed_kmh

async def connect_to_device():
    step_tracker = StepTracker()
    
    while True:
        try:
            print("Searching for SensorData device...")
            
            # Scan for devices
            devices = await BleakScanner.discover()
            target_device = None
            
            # Find our device by name
            for device in devices:
                if device.name == "SensorData":
                    target_device = device
                    break
            
            if not target_device:
                print("Could not find SensorData device. Retrying in 5 seconds...")
                await asyncio.sleep(5)
                continue
            
            print(f"Found device: {target_device.name} ({target_device.address})")
            
            def notification_handler(sender, data):
                # Convert bytes to string and process
                data_str = data.decode('utf-8')
                try:
                    step_count = int(data_str)
                    step_tracker.add_step()
                    speed = step_tracker.calculate_speed()
                    
                    if speed is not None:
                        current_time = datetime.now().strftime("%H:%M:%S")
                        print(f"[{current_time}] Steps: {step_count}, Current Speed: {speed:.2f} km/h")
                except ValueError:
                    print(f"Received non-numeric data: {data_str}")
            
            async with BleakClient(target_device.address) as client:
                print("Connected to device")
                
                # Enable notifications
                await client.start_notify(SENSOR_CHARACTERISTIC_UUID, notification_handler)
                
                # Keep the connection alive
                while True:
                    if not client.is_connected:
                        print("Connection lost. Reconnecting...")
                        break
                    await asyncio.sleep(1)
                    
        except Exception as e:
            print(f"Error: {e}")
            print("Retrying in 5 seconds...")
            await asyncio.sleep(5)

async def main():
    await connect_to_device()

if __name__ == "__main__":
    print("Starting BLE Sensor Receiver...")
    print(f"Using stride length: {STRIDE_LENGTH_METERS} meters")
    print(f"Calculating speed over {WINDOW_SIZE_SECONDS} second window")
    asyncio.run(main()) 