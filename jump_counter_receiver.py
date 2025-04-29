import asyncio
from bleak import BleakClient, BleakScanner
import time

# UUIDs must match those in the Arduino code
SENSOR_SERVICE_UUID = "19B10000-E8F2-537E-4F6C-D104768A1214"
SENSOR_CHARACTERISTIC_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"

async def connect_to_device():
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
                try:
                    # Convert bytes to string and get jump count
                    jump_count = int(data.decode('utf-8'))
                    print(f"Total jumps: {jump_count}")
                except Exception as e:
                    print(f"Error parsing data: {e}")
            
            async with BleakClient(target_device.address) as client:
                print("Connected to device")
                print("Waiting for jumps...")
                
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
    asyncio.run(main()) 