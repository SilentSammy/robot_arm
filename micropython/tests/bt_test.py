# Bluetooth Classic Echo Test - Uppercase
import bts
import time

print("Starting Bluetooth Classic Echo Server...")

# Initialize Bluetooth slave
bts.init("ESP32-ECHO", "1234")
print("Bluetooth initialized: ESP32-ECHO, PIN: 1234")

# Start Bluetooth
if bts.up():
    print("Bluetooth started successfully!")
    print("Ready for pairing and connections...")
else:
    print("Failed to start Bluetooth!")

# Main echo loop
while True:
    try:
        # Check for incoming data
        if bts.data() > 0:
            # Read all available data as string
            msg = bts.get_str(bts.data())
            if msg:
                print(f"Received: '{msg}'")
                
                # Convert to uppercase and send back
                reply = msg.upper()
                bts.send_str(reply)
                print(f"Sent back: '{reply}'")
        
        # Small delay to prevent excessive CPU usage
        time.sleep_ms(10)
        
    except KeyboardInterrupt:
        print("\nStopping echo server...")
        break
    except Exception as e:
        print(f"Error: {e}")
        time.sleep(1)

print("Echo server stopped.")