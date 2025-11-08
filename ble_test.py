"""
BLE Connection and Service Discovery Test
Connects to specified device and discovers all services and characteristics.
"""

import asyncio
from bleak import BleakClient, BleakScanner

# Device name to connect to - change this as needed
DEVICE_NAME = "RoboArm3"

async def connect_and_discover():
    """Connect to specified device and discover its services"""
    
    print(f"Scanning for {DEVICE_NAME}...")
    
    # First, find the target device
    devices = await BleakScanner.discover(timeout=10.0)
    robot_device = None
    
    for device in devices:
        if device.name == DEVICE_NAME:
            robot_device = device
            break
    
    if not robot_device:
        print(f"{DEVICE_NAME} not found. Make sure it's powered on and advertising.")
        return
    
    print(f"Found {DEVICE_NAME} at {robot_device.address}")
    print("Connecting...")
    
    # Connect to the device
    async with BleakClient(robot_device.address) as client:
        if not client.is_connected:
            print("Failed to connect!")
            return
            
        print("Connected successfully!")
        print("=" * 50)
        
        # Discover all services
        services = client.services
        
        print("Discovered services:")
        print()
        
        for service in services:
            print(f"Service: {service.uuid}")
            print(f"  Description: {service.description}")
            
            # List all characteristics for this service
            if service.characteristics:
                print(f"  Characteristics ({len(service.characteristics)}):")
                for char in service.characteristics:
                    properties = []
                    if "read" in char.properties:
                        properties.append("READ")
                    if "write" in char.properties:
                        properties.append("WRITE")
                    if "write-without-response" in char.properties:
                        properties.append("WRITE_NO_RESP")
                    if "notify" in char.properties:
                        properties.append("NOTIFY")
                    if "indicate" in char.properties:
                        properties.append("INDICATE")
                    
                    prop_str = " | ".join(properties) if properties else "None"
                    
                    print(f"    - {char.uuid}")
                    print(f"      Properties: {prop_str}")
                    print(f"      Description: {char.description}")
                    
                    # List descriptors if any
                    if char.descriptors:
                        print(f"      Descriptors:")
                        for desc in char.descriptors:
                            print(f"        * {desc.uuid}: {desc.description}")
                    print()
            else:
                print("  No characteristics found")
            
            print("-" * 40)
        
        print("Discovery complete!")

if __name__ == "__main__":
    try:
        asyncio.run(connect_and_discover())
    except KeyboardInterrupt:
        print("\nOperation interrupted by user.")
    except Exception as e:
        print(f"Error: {e}")
        print(f"Make sure {DEVICE_NAME} is powered on and Bluetooth is enabled.")
