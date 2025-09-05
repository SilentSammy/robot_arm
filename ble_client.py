from bleak import BleakClient, BleakScanner
import asyncio
import sys

# Base UUID for the service and characteristics
BASE_UUID = "12345678-1234-5678-1234-56789abcdef"
SERVICE_UUID = f"{BASE_UUID}0"
DEVICE_NAME = "RoboArm"  # Match the name in the server

_char_uuids = []  # Will store discovered characteristic UUIDs

def get_char_uuid(index):
    """Get the UUID for the characteristic at the given index"""
    return _char_uuids[index]

async def find_device():
    print(f"Scanning for {DEVICE_NAME}...")
    device = await BleakScanner.find_device_by_filter(
        lambda d, ad: d.name and d.name.lower() == DEVICE_NAME.lower()
    )
    
    if not device:
        print(f"Could not find {DEVICE_NAME}")
        return None
        
    print(f"Found {DEVICE_NAME} at {device.address}")
    return device

async def main():
    device = await find_device()
    if not device:
        return

    async with BleakClient(device) as client:
        print(f"Connected to {device.name}")
        
        # Get all characteristics to determine how many controls are available
        # Find our service and its characteristics
        global _char_uuids
        _char_uuids = []
        
        for service in client.services:
            if service.uuid.lower() == SERVICE_UUID.lower():
                chars = service.characteristics
                _char_uuids = [char.uuid for char in chars]
                break
        else:  # No service found
            chars = []
            
        if not chars:
            print("No controls found!")
            return
            
        print(f"\nFound {len(chars)} controls:")
        print("Control Index -> UUID mapping:")
        for i, uuid in enumerate(_char_uuids):
            print(f"  {i} -> {uuid}")
        print("\nFormat: <index> <value> (use index shown above)")
        print("Example: '0 100' sets control 0 to value 100")
        print("Enter 'x' to exit\n")

        while True:
            try:
                cmd = input("Enter command: ").strip().lower()
                if cmd == 'x':
                    break
                    
                # Parse input in format: "control_number value"
                parts = cmd.split()
                if len(parts) != 2:
                    print("Invalid format. Use: <control_number> <value>")
                    continue
                    
                control_num, value = parts
                control_num = int(control_num)
                value = int(value)
                
                if control_num < 0 or control_num >= len(chars):
                    print(f"Invalid control number. Use 0-{len(chars)-1}")
                    continue
                    
                if value < 0 or value > 255:
                    print("Invalid value. Use 0-255")
                    continue
                    
                # Write to the appropriate characteristic
                char_uuid = get_char_uuid(control_num)
                await client.write_gatt_char(char_uuid, bytes([value]))
                print(f"Control {control_num} set to {value}")
                
            except ValueError:
                print("Invalid input. Use numbers only.")
            except Exception as e:
                print(f"Error: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        print("Done.")