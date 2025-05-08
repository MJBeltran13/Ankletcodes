import asyncio
from bleak import BleakClient, BleakScanner
import time
from datetime import datetime
import struct
import logging

# Set up logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

# UUIDs must match those in the Arduino code
SENSOR_SERVICE_UUID = "19B10000-E8F2-537E-4F6C-D104768A1214"
STEP_CHARACTERISTIC_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"
JUMP_CHARACTERISTIC_UUID = "19B10002-E8F2-537E-4F6C-D104768A1214"

# Constants for speed calculation
STRIDE_LENGTH_METERS = 0.7  # Average stride length - can be adjusted
WINDOW_SIZE_SECONDS = 60    # Window size for calculating steps per minute

class ActivityTracker:
    def __init__(self, stride_length=STRIDE_LENGTH_METERS):
        self.stride_length = stride_length
        self.steps = []
        self.last_calculation_time = time.time()
        self.total_steps = 0
        self.total_jumps = 0
    
    def add_step(self, step_count):
        current_time = time.time()
        logger.debug(f"Adding step count: {step_count}")
        
        # Only add to time series if step count has increased
        if step_count > self.total_steps:
            self.steps.append(current_time)
            self.total_steps = step_count
            logger.debug(f"Updated total steps to: {self.total_steps}")
        
        # Remove steps older than WINDOW_SIZE_SECONDS
        cutoff_time = current_time - WINDOW_SIZE_SECONDS
        self.steps = [step for step in self.steps if step > cutoff_time]
    
    def update_jumps(self, jump_count):
        logger.debug(f"Updating jump count to: {jump_count}")
        self.total_jumps = jump_count
    
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
        
        logger.debug(f"Calculated speed: {speed_kmh:.2f} km/h")
        return speed_kmh

async def connect_to_device():
    activity_tracker = ActivityTracker()
    last_step_count = 0
    last_jump_count = 0
    
    while True:
        try:
            logger.info("Searching for SensorData device...")
            
            # Scan for devices
            devices = await BleakScanner.discover()
            logger.debug(f"Found {len(devices)} devices")
            
            target_device = None
            
            # Find our device by name
            for device in devices:
                logger.debug(f"Device found: {device.name} ({device.address})")
                if device.name and device.name.lower() == "sensordata":
                    target_device = device
                    break
            
            if not target_device:
                logger.warning("Could not find SensorData device. Retrying in 5 seconds...")
                await asyncio.sleep(5)
                continue
            
            logger.info(f"Found device: {target_device.name} ({target_device.address})")
            
            def notification_handler(sender, data):
                nonlocal last_step_count, last_jump_count
                try:
                    # Convert bytes to integer
                    value = int.from_bytes(data, byteorder='little', signed=False)
                    logger.debug(f"Received notification from {sender.uuid}: {value}")
                    
                    # Determine which characteristic sent the notification
                    if sender.uuid == STEP_CHARACTERISTIC_UUID:
                        if value != last_step_count:
                            activity_tracker.add_step(value)
                            last_step_count = value
                            speed = activity_tracker.calculate_speed()
                            
                            if speed is not None:
                                current_time = datetime.now().strftime("%H:%M:%S")
                                print(f"[{current_time}] Steps: {value}, Speed: {speed:.2f} km/h, Jumps: {activity_tracker.total_jumps}")
                    
                    elif sender.uuid == JUMP_CHARACTERISTIC_UUID:
                        if value != last_jump_count:
                            activity_tracker.update_jumps(value)
                            last_jump_count = value
                            current_time = datetime.now().strftime("%H:%M:%S")
                            print(f"[{current_time}] Jump detected! Total jumps: {value}")
                
                except Exception as e:
                    logger.error(f"Error processing data: {data.hex()}")
                    logger.error(f"Error details: {str(e)}")
            
            async with BleakClient(target_device.address, timeout=20.0) as client:
                logger.info("Connected to device")
                
                # Get all services
                services = await client.get_services()
                for service in services:
                    logger.debug(f"Service found: {service.uuid}")
                    for char in service.characteristics:
                        logger.debug(f"  Characteristic: {char.uuid}, Properties: {char.properties}")
                
                # Enable notifications for both characteristics
                await client.start_notify(STEP_CHARACTERISTIC_UUID, notification_handler)
                logger.info("Enabled step notifications")
                await client.start_notify(JUMP_CHARACTERISTIC_UUID, notification_handler)
                logger.info("Enabled jump notifications")
                
                # Keep the connection alive
                while True:
                    if not client.is_connected:
                        logger.warning("Connection lost. Reconnecting...")
                        break
                    await asyncio.sleep(1)
                    
        except Exception as e:
            logger.error(f"Error: {e}")
            logger.error("Retrying in 5 seconds...")
            await asyncio.sleep(5)

async def main():
    logger.info("Starting BLE Sensor Receiver...")
    logger.info(f"Using stride length: {STRIDE_LENGTH_METERS} meters")
    logger.info(f"Calculating speed over {WINDOW_SIZE_SECONDS} second window")
    logger.info("Waiting for device connection...")
    
    try:
        await connect_to_device()
    except KeyboardInterrupt:
        logger.info("\nReceiver stopped by user")
    except Exception as e:
        logger.error(f"\nUnexpected error: {e}")
        logger.error("Receiver stopped")

if __name__ == "__main__":
    asyncio.run(main()) 