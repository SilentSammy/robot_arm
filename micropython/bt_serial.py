# Bluetooth Classic Echo Test - Uppercase
import bts
import time

# Global line buffer for accumulating partial lines
_line_buffer = ""

def available():
    """Check if Bluetooth data is available (non-blocking)"""
    return bts.data() > 0

def read_line(terminators=['\n', '\r', ';'], filter_enabled=True):
    """Read a complete line from Bluetooth using custom terminators, returns None if line not ready yet"""
    global _line_buffer
    
    if available():
        raw_data = bts.get_str(bts.data())
        if raw_data:
            if not filter_enabled:
                # Add raw data to buffer
                _line_buffer += raw_data
            else:
                # Filter out null bytes but keep terminators for line detection
                filtered_data = raw_data.replace('\x00', '')
                _line_buffer += filtered_data
            
            # Check if we have any terminator in the buffer
            terminator_found = None
            terminator_pos = -1
            
            for term in terminators:
                pos = _line_buffer.find(term)
                if pos != -1 and (terminator_pos == -1 or pos < terminator_pos):
                    terminator_found = term
                    terminator_pos = pos
            
            if terminator_found is not None:
                # Extract the complete line up to the terminator
                complete_line = _line_buffer[:terminator_pos]
                
                # Keep remaining data in buffer (after the terminator)
                _line_buffer = _line_buffer[terminator_pos + len(terminator_found):]
                
                if filter_enabled:
                    # Apply filtering to the complete line
                    cleaned_line = complete_line.strip()
                    if cleaned_line and len(cleaned_line) > 0:
                        return cleaned_line
                    else:
                        return None  # Filtered out empty line
                else:
                    # Return raw line (but still stripped of whitespace)
                    return complete_line.strip()
    
    return None  # No complete line ready yet

def read(filter_enabled=True):
    """Read all available data from Bluetooth, optionally filtered for meaningful content"""
    if available():
        raw_data = bts.get_str(bts.data())
        if raw_data:
            if filter_enabled:
                # Filter out control characters and meaningless data
                cleaned_data = raw_data.strip().replace('\x00', '').replace('\r', '').replace('\n', '')
                # Only return if there's actual content
                if cleaned_data and len(cleaned_data) > 0:
                    return cleaned_data
                else:
                    return None  # Filtered out
            else:
                # Return raw data without filtering
                return raw_data
    return None

def println(msg):
    """Print message with newline to both serial and Bluetooth (if ready)"""
    print(msg)
    if 'BT_READY' in globals() and BT_READY:
        try:
            bts.send_str(str(msg) + "\n")
        except:
            pass  # Ignore BT errors

def print_msg(msg):
    """Print message without newline to both serial and Bluetooth (if ready)"""
    print(msg, end='')
    if 'BT_READY' in globals() and BT_READY:
        try:
            bts.send_str(str(msg))
        except:
            pass  # Ignore BT errors

def init_bluetooth(device_name="ESP32-ECHO", pin="1234"):
    """Initialize and start Bluetooth Classic"""
    global BT_READY
    println("Starting Bluetooth Classic initialization...")
    
    # Initialize Bluetooth slave
    bts.init(device_name, pin)
    println(f"Bluetooth initialized: {device_name}, PIN: {pin}")
    
    # Start Bluetooth
    BT_READY = False
    if bts.up():
        BT_READY = True
        println("Bluetooth started successfully!")
        println("Ready for pairing and connections...")
        return True
    else:
        println("Failed to start Bluetooth!")
        return False

# Initialize BT_READY as False by default
BT_READY = False