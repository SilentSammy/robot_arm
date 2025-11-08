"""
Minimalistic BLE Scanner
Scans for nearby Bluetooth Low Energy devices and displays their name and address.
"""

import asyncio
from bleak import BleakScanner

async def scan_ble_devices():
    """Scan for BLE devices and display results"""
    print("Scanning for BLE devices...")
    print("-" * 50)
    
    # Scan for 10 seconds
    devices = await BleakScanner.discover(timeout=10.0)
    
    if not devices:
        print("No BLE devices found.")
        return
    
    print(f"Found {len(devices)} device(s):")
    print()
    
    # Filter out devices with no name (Unknown devices)
    named_devices = [device for device in devices if device.name]
    
    if not named_devices:
        print("No named BLE devices found (filtered out 'Unknown' devices).")
        return
    
    print(f"Found {len(named_devices)} named device(s) (filtered out {len(devices) - len(named_devices)} unknown):")
    print()
    
    for device in named_devices:
        name = device.name
        address = device.address
        
        print(f"Name: {name}")
        print(f"MAC Address: {address}")
        
        # Try multiple methods to get service UUIDs
        uuids_found = []
        
        # Method 1: Check advertisement data
        try:
            if hasattr(device, 'details') and hasattr(device.details, 'adv'):
                adv_data = device.details.adv
                if hasattr(adv_data, 'service_uuids') and adv_data.service_uuids:
                    uuids_found.extend(adv_data.service_uuids)
        except:
            pass
        
        # Method 2: Try to access any metadata-like attributes
        try:
            if hasattr(device, 'metadata'):
                metadata = getattr(device, 'metadata')
                if metadata and isinstance(metadata, dict) and 'uuids' in metadata:
                    uuids_found.extend(metadata['uuids'])
        except:
            pass
        
        # Method 3: Try direct service_uuids attribute
        try:
            if hasattr(device, 'service_uuids'):
                service_uuids = getattr(device, 'service_uuids')
                if service_uuids:
                    uuids_found.extend(service_uuids)
        except:
            pass
        
        # Method 4: Check all attributes that might contain UUIDs
        try:
            for attr_name in dir(device):
                if 'uuid' in attr_name.lower() and not attr_name.startswith('_'):
                    attr_value = getattr(device, attr_name)
                    if attr_value:
                        if isinstance(attr_value, (list, tuple)):
                            uuids_found.extend(attr_value)
                        elif isinstance(attr_value, str):
                            uuids_found.append(attr_value)
        except:
            pass
        
        # Display results
        if uuids_found:
            # Remove duplicates while preserving order
            unique_uuids = []
            for uuid in uuids_found:
                if uuid not in unique_uuids:
                    unique_uuids.append(uuid)
            print(f"Service UUIDs:")
            for uuid in unique_uuids:
                print(f"  - {uuid}")
        else:
            print("Service UUIDs: None detected in advertisement")
            print("Note: Device may still be connectable - UUIDs discoverable after connection")
        
        print("-" * 30)

if __name__ == "__main__":
    try:
        asyncio.run(scan_ble_devices())
    except KeyboardInterrupt:
        print("\nScan interrupted by user.")
    except Exception as e:
        print(f"Error: {e}")
        print("Make sure Bluetooth is enabled and bleak is installed: pip install bleak")
