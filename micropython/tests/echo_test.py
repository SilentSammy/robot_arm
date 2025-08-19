# Main loop that imports Bluetooth serial functions
import bt_serial
import time

# Initialize Bluetooth
if bt_serial.init_bluetooth("ESP32-MAIN", "1234"):
    bt_serial.println("Bluetooth initialization successful!")
else:
    bt_serial.println("Bluetooth initialization failed!")

bt_serial.println("=== Main Echo Loop ===")
bt_serial.println("Send data via Bluetooth - will echo back in uppercase")
bt_serial.println("Press Ctrl+C to exit")
bt_serial.println("")

# Main echo loop
while True:
    try:
        # Check for incoming data
        if bt_serial.available():
            print("[DEBUG] Data available, checking for lines...")
            # Read complete lines from Bluetooth
            line = bt_serial.read_line()
            print(f"[DEBUG] read_line() returned: {repr(line)}")
            if line:  # read_line() returns None if no complete line ready
                bt_serial.println(f"Received: '{line}'")
                # Convert to uppercase and send back
                reply = line.upper()
                # Send directly via bts for immediate response
                import bts
                bts.send_str(reply)
                bt_serial.println(f"Sent back: '{reply}'")
            else:
                print("[DEBUG] No complete line ready yet")
        
        # Small delay to prevent excessive CPU usage
        time.sleep(0.01)
        
    except KeyboardInterrupt:
        bt_serial.println("\nMain loop stopped by user")
        break
    except Exception as e:
        bt_serial.println(f"Error: {e}")
        time.sleep(1)

bt_serial.println("Main loop completed.")
