import serial
import time
import threading

class RobotArm:
    def __init__(self, port="COM10", baud_rate=115200, timeout=1, on_message=None):
        """
        Initialize RobotArm client
        
        Args:
            port: COM port for serial connection
            baud_rate: Serial communication speed (should match Arduino: 115200)
            timeout: Serial timeout in seconds
            on_message: Callback function for received messages (optional)
        """
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial = None
        self.on_message = on_message
        
        # Command-string-based change detection
        self.last_command = None
        
        # Rate limiting
        self.last_send_time = 0
        self.min_send_interval = 0.05  # 50ms minimum between sends
        
        # Background reading thread
        self._running = False
        self._read_thread = None
        
        # Connect to the robot
        self._connect()
    
    def _connect(self):
        """Establish serial connection to the robot"""
        try:
            self.serial = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)
            print(f"Connected to robot arm on {self.port}")
            self._start_reading()
            
            # Wait for Arduino to initialize and send "Ready"
            time.sleep(2)
            
        except serial.SerialException as e:
            print(f"Failed to connect to {self.port}: {e}")
            raise
    
    def _start_reading(self):
        """Start background thread to read serial messages"""
        if self.on_message and not self._running:
            self._running = True
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()
    
    def _read_loop(self):
        """Background thread loop to read serial messages"""
        while self._running and self.serial and self.serial.is_open:
            try:
                if self.serial.in_waiting > 0:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line and self.on_message:
                        self.on_message(line)
                time.sleep(0.01)  # Small delay to prevent busy waiting
            except Exception:
                # Silently handle read errors (connection issues, etc.)
                break

    def disconnect(self):
        """Close the serial connection"""
        if self.serial and self.serial.is_open:
            # Stop the reading thread
            self._running = False
            if self._read_thread and self._read_thread.is_alive():
                self._read_thread.join(timeout=1)
            
            self.serial.close()
            print("Disconnected from robot arm")
    
    def __enter__(self):
        """Context manager entry"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - ensures cleanup"""
        self.disconnect()
    
    def is_connected(self):
        """Check if serial connection is active"""
        return self.serial and self.serial.is_open
    
    # --- Command methods ---
    def _send(self, command_string, force=False):
        """
        Send a command to the robot with change detection and rate limiting
        
        Args:
            command_string: String command to send (e.g., "MOV:0,90,5")
            force: If True, bypass change detection and rate limiting
        
        Returns:
            bool: True if command was sent, False if skipped
        """
        if not self.serial or not self.serial.is_open:
            print("Serial connection not available")
            return False
        
        # Change detection - skip if same command as last time
        if not force and command_string == self.last_command:
            return False
        
        # Rate limiting
        current_time = time.time()
        if not force and (current_time - self.last_send_time) < self.min_send_interval:
            return False
        
        try:
            # Send command with newline (Arduino expects line-terminated input)
            print(f"{command_string}")
            self.serial.write((command_string + '\n').encode())
            self.last_command = command_string
            self.last_send_time = current_time
            return True
        except Exception as e:
            print(f"Error sending command '{command_string}': {e}")
            return False

    def move_to(self, joints):
        # joints can be: int -> angle, speed OR dict {joint: (angle, speed)}
        if isinstance(joints, dict):
            commands = [f"MOV:{joint},{angle},{speed}" for joint, (angle, speed) in joints.items()]
        else:
            # Single joint case - should be called as move_to({0: (90, 5)})
            raise ValueError("move_to requires dict format: {joint: (angle, speed)}")
        
        multi_command = '\n'.join(commands)
        return self._send(multi_command)
    
    def snap_to(self, joints):
        # joints: dict {joint: angle}
        if isinstance(joints, dict):
            commands = [f"SET:{joint},{angle}" for joint, angle in joints.items()]
        else:
            raise ValueError("snap_to requires dict format: {joint: angle}")
        
        multi_command = '\n'.join(commands)
        return self._send(multi_command)
    
    def set_velocity(self, joints):
        # joints: dict {joint: velocity}
        if isinstance(joints, dict):
            commands = [f"VEL:{joint},{velocity}" for joint, velocity in joints.items()]
        else:
            raise ValueError("set_velocity requires dict format: {joint: velocity}")
        
        multi_command = '\n'.join(commands)
        return self._send(multi_command)
    
    def move_by(self, joints, force=True):
        # joints: dict {joint: (increment, speed)}
        if isinstance(joints, dict):
            commands = [f"ADD:{joint},{increment},{speed}" for joint, (increment, speed) in joints.items()]
        else:
            raise ValueError("move_by requires dict format: {joint: (increment, speed)}")
        
        multi_command = '\n'.join(commands)
        return self._send(multi_command, force=force)
    
    def snap_by(self, joints, force=True):
        # joints: dict {joint: increment}
        if isinstance(joints, dict):
            commands = [f"JMP:{joint},{increment}" for joint, increment in joints.items()]
        else:
            raise ValueError("snap_by requires dict format: {joint: increment}")
        
        multi_command = '\n'.join(commands)
        return self._send(multi_command, force=force)

    def stop(self):
        """Stop all joints by setting velocity to 0"""
        return self.set_velocity({joint: 0 for joint in range(3)})

# Example usage
if __name__ == '__main__':
    def handle_robot_message(message):
        print(f"Robot: {message}")
    
    # Create robot arm client with message callback
    with RobotArm(port="COM10", on_message=handle_robot_message) as arm:
        pass # Start a REPL session here
