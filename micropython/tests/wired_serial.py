# Standalone Wired Serial Test - Non-blocking Line Echo
import sys
import select
import time

def available():
    """Check if wired serial data is available (non-blocking)"""
    try:
        # Use select to check if stdin has data available
        ready, _, _ = select.select([sys.stdin], [], [], 0)
        return len(ready) > 0
    except:
        return False

def read_line():
    """Read a complete line from wired serial if available"""
    try:
        if available():
            line = sys.stdin.readline().strip()
            return line
    except:
        pass
    return None

print("=== MicroPython Wired Serial Line Echo Test ===")
print("Type a line and press Enter - it will be echoed back in uppercase")
print("Press Ctrl+C to exit")
print("")

# Main echo loop
while True:
    try:
        # Check if a complete line is available
        line = read_line()
        if line:
            # Echo back in uppercase
            print(f"ECHO: {line.upper()}")
        
        # Small delay to prevent excessive CPU usage
        time.sleep(0.01)
        
    except KeyboardInterrupt:
        print("\nTest stopped by user")
        break
    except Exception as e:
        print(f"Error: {e}")
        time.sleep(1)

print("Test completed.")
