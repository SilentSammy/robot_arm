from bleak import BleakClient, BleakScanner
import asyncio
import sys

# Conversion helpers
def from_norm(norm):
    """Convert 0.0-1.0 float to 0-255 int"""
    val = int(round(norm * 256))
    return max(0, min(255, val))

def from_bipolar(bipolar):
    """Convert -1.0-1.0 float to 0-255 int"""
    val = int(round((bipolar * 128) + 128))
    return max(0, min(255, val))

def map_vel(val):
    """Map velocity from -100..100 to 0..255 range"""
    val = max(-1.0, min(1.0, val/100.0)) # Normalize to -1..1
    return from_bipolar(val)

class ArmBLEClient:
    def __init__(self, device_name="RoboArm", base_uuid="12345678-1234-5678-1234-56789abcdef"):
        self.device_name = device_name
        self.base_uuid = base_uuid
        self.device = None
        self.client = None
        
        # Cache last sent mapped values to avoid redundant BLE writes
        self._cached_char_values = {}
    
    # Communication methods
    async def find_device(self):
        print(f"Scanning for {self.device_name}...")
        device = await BleakScanner.find_device_by_filter( lambda d, ad: d.name and d.name.lower() == self.device_name.lower() )
        if not device:
            print(f"Could not find {self.device_name}")
            return None
        print(f"Found {self.device_name} at {device.address}")
        return device

    async def connect(self):
        self.device = await self.find_device()
        if not self.device:
            raise Exception("Device not found")

        self.client = BleakClient(self.device)
        await self.client.connect()
        print(f"Connected to {self.device.name}")

    async def disconnect(self):
        if self.client:
            await self.client.disconnect()
            print("Disconnected.")

    async def set_char(self, char_uuid, value):
        if self._cached_char_values.get(char_uuid) == value:
            # print("No change, skipping write") # Commented to avoid spamming output
            return  # No change, skip write
        await self.client.write_gatt_char(char_uuid, bytes([value]))
        self._cached_char_values[char_uuid] = value

    # Setters for each control
    async def set_led(self, state):
        """Set the LED state on the device."""
        await self.set_char(self.base_uuid + "1", int(state))

    async def set_vx(self, vx):
        """Set X velocity (-100 to 100)"""
        mapped = map_vel(vx)
        await self.set_char(self.base_uuid + "2", mapped)

    async def set_vy(self, vy):
        """Set Y velocity (-100 to 100)"""
        mapped = map_vel(vy)
        await self.set_char(self.base_uuid + "3", mapped)

    async def set_w(self, w):
        """Set angular velocity w in degrees/sec (expected range -100..100)."""
        mapped = map_vel(w)
        await self.set_char(self.base_uuid + "4", mapped)


async def main_loop():
    from input_manager.input_man import rising_edge, falling_edge, is_pressed

    arm = ArmBLEClient()
    await arm.connect()
    print("Connected")

    xy_mag = 15 # cm/s per key press
    try:
        while True:
            if rising_edge('x'):
                # schedule fire-and-forget on the running loop
                await arm.set_led(True)
            if falling_edge('x'):
                await arm.set_led(False)
            
            # X axis (A/D)
            vx = int(is_pressed('d')) - int(is_pressed('a'))
            vx *= xy_mag
            await arm.set_vx(vx)

            # Y axis (W/S)
            vy = int(is_pressed('w')) - int(is_pressed('s'))
            vy *= xy_mag
            await arm.set_vy(vy)

            # Angular velocity (Q/E) - Q = negative, E = positive
            w = int(is_pressed('q')) - int(is_pressed('e'))
            w *= 60
            await arm.set_w(w)

            await asyncio.sleep(0.01)   # keep the loop responsive
    finally:
        await arm.set_vx(0)
        await arm.set_vy(0)
        await arm.set_w(0)
        await arm.disconnect()

if __name__ == "__main__":
    asyncio.run(main_loop())
