# Command processing system module
# Provides reusable command parsing and execution framework

import bt_serial
import time

class CommandProcessor:
    def __init__(self, command_handlers, bt_name="ESP32-MAIN", bt_pin="1234", auto_init_bt=True):
        """
        Initialize command processor with custom handlers.
        
        Args:
            command_handlers: Dictionary mapping command names to handler functions
            bt_name: Bluetooth device name
            bt_pin: Bluetooth PIN
            auto_init_bt: Whether to automatically initialize Bluetooth
        """
        self.command_handlers = command_handlers
        self.bt_name = bt_name
        self.bt_pin = bt_pin
        
        if auto_init_bt:
            self.init_bluetooth()
    
    def init_bluetooth(self):
        """Initialize Bluetooth communication"""
        if bt_serial.init_bluetooth(self.bt_name, self.bt_pin):
            pass  # Silent success like Arduino
        else:
            bt_serial.println("ERR")
        bt_serial.println("Ready")
    
    def parse_cmd(self, input_str):
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
    
    def run_cmd(self, command_str):
        """Execute a parsed command string using dispatch table"""
        cmd = self.parse_cmd(command_str)
        
        if not cmd["valid"]:
            bt_serial.println("ERR")
            return False
        
        function = cmd["function"]
        args = cmd["args"]
        
        # Look up command handler in dispatch table
        if function in self.command_handlers:
            try:
                return self.command_handlers[function](args)
            except Exception as e:
                bt_serial.println("ERR")
                return False
        else:
            bt_serial.println("ERR")
            return False
    
    def run_cmd_loop(self):
        """Main command processing loop - can be stopped to return to REPL"""
        bt_serial.println("Command loop started - Ctrl+C to stop and return to REPL")
        
        while True:
            try:
                # Check for incoming data
                if bt_serial.available():
                    # Read complete lines from Bluetooth
                    line = bt_serial.read_line()
                    if line:  # read_line() returns None if no complete line ready
                        if not self.run_cmd(line):
                            bt_serial.println("ERR")
                
                # Small delay to prevent excessive CPU usage
                time.sleep(0.01)
                
            except KeyboardInterrupt:
                bt_serial.println("Command loop stopped - REPL available")
                break
            except Exception as e:
                time.sleep(1)
    
    def add_handler(self, command_name, handler_function):
        """Add a new command handler at runtime"""
        self.command_handlers[command_name.upper()] = handler_function
    
    def remove_handler(self, command_name):
        """Remove a command handler at runtime"""
        if command_name.upper() in self.command_handlers:
            del self.command_handlers[command_name.upper()]

# Simple usage example:
# from cmd_system import create_processor
# 
# def handle_led(args):
#     # LED handler code
#     return True
# 
# handlers = {"LED": handle_led}
# processor = create_processor(handlers)
# processor.run_cmd_loop()
