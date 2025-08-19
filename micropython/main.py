# Main loop that imports Bluetooth serial functions
import bt_serial
import time
from machine import Pin

# Initialize LED pin
led = Pin(2, Pin.OUT)  # Built-in LED on GPIO 2

# Command handler functions
def handle_led(args):
    """Handle LED command: LED:0 or LED:1"""
    if len(args) != 1:
        return False
    
    try:
        state = int(args[0])
        if state == 0:
            led.off()
        elif state == 1:
            led.on()
        else:
            return False
    except ValueError:
        return False
    
    return True

# Command dispatch table - maps command names to handler functions
command_handlers = {
    "LED": handle_led,
    # Add more commands here as needed
    # "SERVO": handle_servo,
    # "SENSOR": handle_sensor,
}

# Initialize Bluetooth
if bt_serial.init_bluetooth("ESP32-MAIN", "1234"):
    pass  # Silent success like Arduino
else:
    bt_serial.println("ERR")

bt_serial.println("Ready")

def parse_cmd(input_str):
    """Parse command string into function and arguments"""
    result = {"function": "", "args": [], "valid": False}
    
    input_str = input_str.strip()
    if not input_str:
        return result
    
    colon_index = input_str.find(':')
    
    if colon_index < 0:
        # No colon found - argumentless command
        result["function"] = input_str.upper()
        result["valid"] = True
        return result
    
    if colon_index == 0:
        return result  # Can't start with colon
    
    result["function"] = input_str[:colon_index].upper()
    
    arg_string = input_str[colon_index + 1:]
    if not arg_string:
        # Colon but no arguments - still valid
        result["valid"] = True
        return result
    
    # Split arguments by comma
    args = [arg.strip() for arg in arg_string.split(',') if arg.strip()]
    result["args"] = args
    result["valid"] = True
    return result

def run_cmd(command_str):
    """Execute a parsed command string using dispatch table"""
    cmd = parse_cmd(command_str)
    
    if not cmd["valid"]:
        bt_serial.println("ERR")
        return False
    
    function = cmd["function"]
    args = cmd["args"]
    
    # Look up command handler in dispatch table
    if function in command_handlers:
        try:
            return command_handlers[function](args)
        except Exception as e:
            bt_serial.println("ERR")
            return False
    else:
        bt_serial.println("ERR")
        return False

def run_cmd_loop():
    """Main command processing loop - can be stopped to return to REPL"""
    bt_serial.println("Command loop started - Ctrl+C to stop and return to REPL")
    
    while True:
        try:
            # Check for incoming data
            if bt_serial.available():
                # Read complete lines from Bluetooth
                line = bt_serial.read_line()
                if line:  # read_line() returns None if no complete line ready
                    if not run_cmd(line):
                        bt_serial.println("ERR")
            
            # Small delay to prevent excessive CPU usage
            time.sleep(0.01)
            
        except KeyboardInterrupt:
            bt_serial.println("Command loop stopped - REPL available")
            break
        except Exception as e:
            time.sleep(1)

# Auto-start the command loop
run_cmd_loop()
