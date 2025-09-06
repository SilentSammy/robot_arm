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

class ArmBLEClient:
    def __init__(self):
        self.device = None
        self.client = None

    async def connect(self):
        self.device = await find_device()
        if not self.device:
            raise Exception("Device not found")

        self.client = BleakClient(self.device)
        await self.client.connect()
        print(f"Connected to {self.device.name}")
        
        # Get all characteristics to determine how many controls are available
        # Find our service and its characteristics
        global _char_uuids
        _char_uuids = []
        
        for service in self.client.services:
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

    async def disconnect(self):
        if self.client:
            await self.client.disconnect()
            print("Disconnected.")

    async def set_led(self, state):
        """Set the LED state on the device."""
        if len(_char_uuids) == 0:
            print("No characteristic UUIDs found. Cannot set LED state.")
            return
        
        # Assuming the first characteristic is the LED control
        led_char_uuid = _char_uuids[0]  
        value = 1 if state else 0
        await self.client.write_gatt_char(led_char_uuid, bytes([value]))

async def main():
    from input_manager.input_man import rising_edge, falling_edge
    arm = ArmBLEClient()
    await arm.connect()
    print("Connected. Press X to toggle LED, Q to quit. Use WASD for vx/vy.")
    led_on = False
    VXY_STEP = 15  # Step size for WASD
    vx = 0
    vy = 0
    prev_vx = None
    prev_vy = None
    try:
        while True:
            # LED control
            if rising_edge('x'):
                await arm.set_led(True)
                led_on = True
                print("LED ON")
            if falling_edge('x'):
                await arm.set_led(False)
                led_on = False
                print("LED OFF")

            # WASD velocity control
            # W/S control vy (+100/-100), A/D control vx (-100/+100)
            # On key press, set to value; on key release, set to 0
            # Priority: if both W/S or A/D pressed, last pressed wins
            # vx: -100 (A), +100 (D), 0 (release)
            # vy: +100 (W), -100 (S), 0 (release)

            # Handle vx
            if rising_edge('a'):
                vx = -100 if VXY_STEP >= 100 else -VXY_STEP
            elif rising_edge('d'):
                vx = 100 if VXY_STEP >= 100 else VXY_STEP
            if falling_edge('a') or falling_edge('d'):
                vx = 0

            # Handle vy
            if rising_edge('w'):
                vy = 100 if VXY_STEP >= 100 else VXY_STEP
            elif rising_edge('s'):
                vy = -100 if VXY_STEP >= 100 else -VXY_STEP
            if falling_edge('w') or falling_edge('s'):
                vy = 0

            def map_vel(val):
                val = max(-100, min(100, val))
                return int(round((val + 100) * 255 / 200))

            # Only send if changed
            if vx != prev_vx:
                if len(_char_uuids) > 1:
                    await arm.client.write_gatt_char(_char_uuids[1], bytes([map_vel(vx)]))
                    print(f"VX set to {vx}")
                prev_vx = vx
            if vy != prev_vy:
                if len(_char_uuids) > 2:
                    await arm.client.write_gatt_char(_char_uuids[2], bytes([map_vel(vy)]))
                    print(f"VY set to {vy}")
                prev_vy = vy

            await asyncio.sleep(0.01)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        await arm.set_led(False)
        await arm.disconnect()
        print("Disconnected.")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        print("Done.")