from bleak import BleakClient, BleakScanner
import asyncio
import sys

# Same UUIDs as in the MicroPython server
LED_SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
LED_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef1"
DEVICE_NAME = "MicroLED"

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

        while True:
            cmd = input("Enter command (0=OFF, 1=ON, x=exit): ").strip().lower()
            if cmd == 'x':
                break
            elif cmd in ('0', '1'):
                value = bytes([int(cmd)])
                await client.write_gatt_char(LED_CHAR_UUID, value)
                print(f"LED set to {cmd}")
            else:
                print("Invalid command")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        print("Done.")