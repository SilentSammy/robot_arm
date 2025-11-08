"""
Brazo Simple - Interfaz funcional para control de brazo robótico
Módulo educativo que expone funciones simples para controlar el brazo sin usar POO directamente.
"""

# Importar todas las clases y instancias necesarias
from machine import Pin, PWM
import machine
import time
import math
import bluetooth
from micropython import const

# Global BLE setup
DEVICE_NAME = "RoboArm1"
_ble = bluetooth.BLE()
_ble.active(True)

# Global state
control_callbacks = {}  # Dictionary mapping control IDs to callback functions
_connections = set()    # Set of connected devices
_handles = {}           # Dictionary mapping control IDs to characteristic handles

def _ble_irq(event, data):
    """Handle BLE events through interrupts"""
    if event == 1:  # _IRQ_CENTRAL_CONNECT
        conn_handle, _, _ = data
        _connections.add(conn_handle)
        print('Client connected')
        
    elif event == 2:  # _IRQ_CENTRAL_DISCONNECT
        conn_handle, _, _ = data
        _connections.remove(conn_handle)
        print('Client disconnected')
        _advertise()
        
    elif event == 3:  # _IRQ_GATTS_WRITE
        conn_handle, value_handle = data
        if conn_handle in _connections:
            # Find which control ID this handle belongs to
            for control_id, handle in _handles.items():
                if handle == value_handle:
                    payload = _ble.gatts_read(value_handle)
                    if payload and control_id in control_callbacks:
                        # Call the corresponding callback with the integer value
                        control_callbacks[control_id](payload[0])
                    break

def _advertise(interval_us=100000):
    """Start advertising the BLE service"""
    name = DEVICE_NAME.encode()
    adv_data = bytearray([
        0x02, 0x01, 0x06,  # General discoverable mode
        len(name) + 1, 0x09]) + name  # Complete Local Name
    _ble.gap_advertise(interval_us, adv_data)

def stop():
    """Stop the BLE server and clean up"""
    # Stop advertising
    _ble.gap_advertise(None)
    
    # Remove IRQ handler
    _ble.irq(None)
    
    # Disconnect any active connections
    for conn_handle in _connections.copy():  # Use copy since we're modifying the set
        _ble.gap_disconnect(conn_handle)
        _connections.remove(conn_handle)
    
    print("BLE Server stopped")

def start():
    """Initialize and start the BLE server"""
    global _handles
    
    # Ensure clean state
    stop()
    
    if not control_callbacks:
        raise ValueError("No callbacks defined!")
        
    _ble.config(gap_name=DEVICE_NAME)
    
    # Create characteristics for each callback
    characteristics = []
    control_ids = sorted(control_callbacks.keys())  # Sort for consistent ordering
    
    for control_id in control_ids:
        uuid = bluetooth.UUID('12345678-1234-5678-1234-56789abcdef' + hex(control_id)[2:])
        characteristics.append(
            (uuid, bluetooth.FLAG_READ | bluetooth.FLAG_WRITE | bluetooth.FLAG_NOTIFY)
        )
    
    # Register all characteristics under one service
    service_handles = _ble.gatts_register_services(((
        bluetooth.UUID('12345678-1234-5678-1234-56789abcdef0'),  # Service UUID
        tuple(characteristics),
    ),))
    
    # Map control IDs to their handles and initialize to 0
    _handles.clear()
    for i, control_id in enumerate(control_ids):
        handle = service_handles[0][i]
        _handles[control_id] = handle
        _ble.gatts_write(handle, bytes([0]))
        
    # Set up event handler and start advertising
    _ble.irq(_ble_irq)
    _advertise()
    
    print(f"BLE Server started with {len(control_callbacks)} controls")
    print(f"Advertising as {DEVICE_NAME}")

# Conversion helpers
def to_norm(value):
    """Convert 0-255 to 0.0-1.0 float"""
    return value / 256.0

def to_bipolar(value):
    """Convert 0-255 to -1.0-1.0 float"""
    return (value - 128) / 128.0

class Encoder:
    def __init__(self, pin_a, pin_b):
        """
        Hardware quadrature encoder using ESP32's PCNT (Pulse Counter) peripheral.
        Much more accurate and reliable than software-based counting.
        pin_a: Channel A pin
        pin_b: Channel B pin  
        """
        try:
            # Use ESP32 hardware pulse counter for quadrature encoding
            from machine import Pin
            from esp32 import PCNT
            
            # Create PCNT unit for quadrature decoding
            self.pcnt = PCNT(0, pin_a, pin_b, count_mode=PCNT.MODE_QUAD)
            self.pcnt.value(0)  # Start at zero
            self.hardware_encoder = True
            
        except (ImportError, AttributeError):
            # Fallback to software implementation if hardware not available
            self.pin_a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
            self.pin_b = Pin(pin_b, Pin.IN, Pin.PULL_UP)
            self.count = 0
            self.last_a = self.pin_a.value()
            self.pin_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._irq_handler)
            self.hardware_encoder = False
    
    def _irq_handler(self, pin):
        """Software fallback interrupt handler"""
        current_a = self.pin_a.value()
        if self.last_a != current_a:
            current_b = self.pin_b.value()
            if current_a == current_b:
                self.count += 1
            else:
                self.count -= 1
            self.last_a = current_a
    
    def get_count(self):
        """Get current encoder count"""
        if self.hardware_encoder:
            return self.pcnt.value()
        else:
            return self.count
    
    def zero(self):
        """Zero the encoder at current position"""
        if self.hardware_encoder:
            self.pcnt.value(0)
        else:
            self.count = 0
    
    def __repr__(self):
        return f"Encoder: {self.get_count()} counts ({'HW' if self.hardware_encoder else 'SW'})"

class DCMotor:
    def __init__(self, ena_pin, in1_pin, in2_pin, freq=48, power_limit=0.25):
        """
        DC Motor control using 3-pin system: ENA (PWM), IN1, IN2.
        ena_pin: PWM pin for speed control
        in1_pin: Digital pin for direction control 1
        in2_pin: Digital pin for direction control 2
        freq: PWM frequency (default 32Hz)
        power_limit: Maximum power limit (default 0.2 = 20%)
        """
        self.ena = PWM(Pin(ena_pin), freq=freq)
        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)
        self.power_limit = power_limit
        
        # Initialize to stopped state
        self.set_power(0)
    
    def set_power(self, value):
        """
        Set motor speed and direction.
        value: float from -1 (full reverse) to 1 (full forward)
        Note: Value will be clamped to power_limit (always positive)
        """
        limit = abs(self.power_limit)
        value = max(-1, min(1, value))  # Clamp to [-1, 1] first
        value = max(-limit, min(limit, value))  # Apply power limit (always positive)
        duty = int(abs(value) * 1023)
        
        if value > 0:
            # Forward direction
            self.in1.on()
            self.in2.off()
            self.ena.duty(duty)
        elif value < 0:
            # Reverse direction
            self.in1.off()
            self.in2.on()
            self.ena.duty(duty)
        else:
            # Stop
            self.in1.off()
            self.in2.off()
            self.ena.duty(0)
    
    def stop(self):
        """Stop the motor"""
        self.set_power(0)

class ServoMotor:
    # Class field for save granularity (degrees)
    SAVE_GRANULARITY = 20
    ANGLES_FILE = "servo_angles.csv"
    
    def __init__(self, pin, scale=1.0, offset=0, start_angle=0):
        # Store configuration first so _load_saved_angle can work
        self.pin = pin
        self.scale = scale
        self.offset = offset
        
        # Load saved angle or use start_angle
        saved_angle = self._load_saved_angle()
        initial_angle = saved_angle if saved_angle is not None else start_angle
        
        # Calculate the target servo angle and duty cycle before creating PWM
        target_servo_angle = (initial_angle * scale) + offset
        target_servo_angle = max(0, min(180, target_servo_angle))  # Clamp
        
        # Calculate target duty cycle
        min_us = 600
        max_us = 2500
        us_per_duty = 20000 / 1023
        min_duty = int(min_us / us_per_duty)
        max_duty = int(max_us / us_per_duty)
        target_duty = int(min_duty + (max_duty - min_duty) * target_servo_angle / 180)
        
        # Create PWM and immediately set to target duty to prevent glitches
        self.pwm = PWM(Pin(pin), freq=50, duty=target_duty)
        
        # Store PWM configuration
        self.min_us = min_us
        self.max_us = max_us
        self.us_per_duty = us_per_duty
        self.last_saved_angle = None
        
        # Initialize current_angle
        self.current_angle = initial_angle

    def _load_saved_angle(self):
        """Load saved angle for this servo from CSV file"""
        try:
            with open(self.ANGLES_FILE, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#'):
                        parts = line.split(',')
                        if len(parts) == 2:
                            pin_str, angle_str = parts
                            if int(pin_str.strip()) == self.pin:
                                return float(angle_str.strip())
        except:
            # File doesn't exist, malformed, or any other error - return None
            pass
        return None
    
    def _save_angle(self, angle):
        """Save current angle to CSV file if granularity threshold is crossed"""
        # Check if we should save (rounded to granularity)
        current_rounded = round(angle / self.SAVE_GRANULARITY) * self.SAVE_GRANULARITY
        if self.last_saved_angle == current_rounded:
            return  # No need to save, same granularity boundary
            
        # Load existing angles
        angles = {}
        try:
            with open(self.ANGLES_FILE, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#'):
                        parts = line.split(',')
                        if len(parts) == 2:
                            pin_str, angle_str = parts
                            angles[int(pin_str.strip())] = float(angle_str.strip())
        except:
            # File doesn't exist or error - start with empty dict
            pass
        
        # Update this servo's angle
        angles[self.pin] = angle
        
        # Write back to file
        try:
            with open(self.ANGLES_FILE, 'w') as f:
                f.write("# Servo angles: pin, angle\n")
                for pin in sorted(angles.keys()):
                    f.write(f"{pin},{angles[pin]}\n")
            # Only update last_saved_angle if file write was successful
            self.last_saved_angle = current_rounded
        except:
            # Failed to save - continue without crashing
            pass
    
    def _write(self, angle):
        """
        Set servo to specified angle (0-180 degrees).
        Uses standard servo PWM timing for ESP32 hardware PWM.
        """
        # Clamp angle to 0-180 range
        angle = max(0, min(180, angle))
        
        # Calculate duty cycle directly from angle
        min_duty = int(self.min_us / self.us_per_duty)
        max_duty = int(self.max_us / self.us_per_duty)
        duty = int(min_duty + (max_duty - min_duty) * angle / 180)
        self.pwm.duty(duty)

    def write(self, user_angle):
        converted_angle = self.convert_angle(user_angle)
        self._write(converted_angle)
        self.current_angle = user_angle  # Track the actual user angle written
        
        # Always attempt to save - _save_angle will handle granularity check
        self._save_angle(user_angle)

    def convert_angle(self, user_angle):
        transformed_angle = (user_angle * self.scale) + self.offset
        return transformed_angle

    def revert_angle(self, servo_angle):
        user_angle = (servo_angle - self.offset) / self.scale
        return user_angle

    @property
    def min_user_angle(self):
        angle1 = self.revert_angle(0)
        angle2 = self.revert_angle(180)
        return min(angle1, angle2)

    @property
    def max_user_angle(self):
        angle1 = self.revert_angle(0)
        angle2 = self.revert_angle(180)
        return max(angle1, angle2)

class EncodedMotor:
    def __init__(self, motor, encoder, counts_per_rev=740, zero_offset=0, max_power=0.25, kP=0.05, kD=0.001):
        self.motor = motor
        self.encoder = encoder
        self.counts_per_rev = counts_per_rev  # Encoder counts per full revolution
        self.zero_offset = zero_offset        # Encoder count at 0 degrees
        start_count = encoder.get_count()
        self._current_target_counts = float(start_count)  # Use float for smooth increments
        self._final_target_counts = float(start_count)    # The user sets this but indirectly, in degrees
        self._speed_counts = counts_per_rev / 4
        self.tmr = None
        self.last_update = None
        self.max_power = max_power
        self.kP = kP
        self.kD = kD
    
    @property
    def speed(self):
        # Speed in counts/sec
        return self._speed_counts

    @speed.setter
    def speed(self, value):
        # Accepts counts/sec directly
        self._speed_counts = abs(value)

    @property
    def final_target(self):
        # Final target in encoder counts
        return self._final_target_counts

    @final_target.setter
    def final_target(self, counts):
        self._final_target_counts = int(counts)

    @property
    def enabled(self):
        return self.tmr is not None

    def enable(self):
        if not self.enabled:
            now_count = self.encoder.get_count()
            self._current_target_counts = now_count
            self._final_target_counts = now_count
            self.tmr = machine.Timer(2)
            # Use a named callback instead of lambda for reliability
            def _timer_cb(t):
                self.update()
            self.tmr.init(period=20, mode=machine.Timer.PERIODIC, callback=_timer_cb)

    def disable(self):
        if self.enabled:
            self.tmr.deinit()
            self.tmr = None
            self.last_update = None
            self.motor.set_power(0)  # Stop motor

    def update(self):
        # Speed-limited position update, then PD control to chase _current_target_counts
        now = time.ticks_ms()
        if self.last_update is None:
            self.last_update = now
            return
        dt = time.ticks_diff(now, self.last_update) / 1000.0
        self.last_update = now
        max_dt = 0.1
        min_dt = 0.005
        if dt > max_dt or dt < min_dt:
            # Ignore updates with too large or too small dt
            return

        # Debug: print dt, max_step, delta
        delta = self._final_target_counts - self._current_target_counts
        max_step = self._speed_counts * dt
        if abs(delta) <= max_step:
            self._current_target_counts = self._final_target_counts
        else:
            self._current_target_counts += max_step if delta > 0 else -max_step
        # Snap exactly if within 1 count
        if abs(self._final_target_counts - self._current_target_counts) < 1e-3:
            self._current_target_counts = self._final_target_counts

        # Set motor power
        power = self.control(dt)
        self.motor.set_power(power)

    def control(self, dt):
        pos = self.encoder.get_count()
        error = int(round(self._current_target_counts)) - pos
        # Derivative on measurement: d_error = -(pos - last_pos) / dt
        if not hasattr(self, '_last_pos'):
            self._last_pos = pos
        d_error = -(pos - self._last_pos) / dt if dt > 0 else 0
        self._last_pos = pos
        power = self.kP * error + self.kD * d_error
        # Clamp power to [-max_power, max_power]
        maxp = abs(self.max_power)
        power = max(-maxp, min(maxp, power))
        return power
    
    # Helper methods
    def snap_by(self, counts):
        self._final_target_counts += counts
        self._current_target_counts = self._final_target_counts
    
    def snap_to(self, counts):
        # Snap to the equivalent position within one revolution, choosing the shortest path
        # counts: desired absolute position (can be any int)
        current = self.encoder.get_count()
        rev = self.counts_per_rev
        # Find the wrapped target closest to current
        # Normalize both to [0, rev)
        target_mod = counts % rev
        current_mod = current % rev
        # Compute difference in both directions
        diff = target_mod - current_mod
        # Wrap to [-rev/2, rev/2)
        if diff > rev/2:
            diff -= rev
        elif diff < -rev/2:
            diff += rev
        # The new target is current + diff (shortest path)
        new_target = current + diff
        self._final_target_counts = new_target
        self._current_target_counts = self._final_target_counts

    def set_vel(self, counts_per_sec):
        self._speed_counts = abs(counts_per_sec)
        prev_delta = self._final_target_counts - self._current_target_counts
        prev_sign = 0 if prev_delta == 0 else (1 if prev_delta > 0 else -1)
        new_sign = 0 if counts_per_sec == 0 else (1 if counts_per_sec > 0 else -1)

        if counts_per_sec > 0:
            self._final_target_counts = float('inf')
        elif counts_per_sec < 0:
            self._final_target_counts = float('-inf')
        else:
            now = self.encoder.get_count()
            self._final_target_counts = now
            self._current_target_counts = now
            return

        # If direction changed (ignoring zero), reset _current_target_counts to encoder position
        if prev_sign != 0 and new_sign != 0 and prev_sign != new_sign:
            self._current_target_counts = self.encoder.get_count()

    # Conversion methods
    def counts_to_degs(self, counts):
        """Convert encoder counts to degrees."""
        return counts * 360.0 / self.counts_per_rev

    def degs_to_counts(self, degrees):
        """Convert degrees to encoder counts."""
        return degrees * self.counts_per_rev / 360.0


class Joint:
    def __init__(self, servo):
        self.servo = servo
        self.target = servo.current_angle  # Initial target (will be reset on enable)
        self.speed = 0
        self.last_update = None
        self.tmr = None

    @property
    def enabled(self):
        """Check if joint control is enabled"""
        return self.tmr is not None

    def enable(self):
        if not self.enabled:
            # SAFETY: Always reset target to current position to prevent sudden jumps
            self.target = self.servo.current_angle
            self.tmr = machine.Timer(0)  # Use Timer 0 for all joints
            self.tmr.init(period=20, mode=machine.Timer.PERIODIC, callback=lambda t: self.update())

    def disable(self):
        if self.enabled:
            self.tmr.deinit()
            self.tmr = None
            self.last_update = None

    def write(self, angle):
        self.servo.write(angle)
        # No need to track current_angle here - ServoMotor handles it

    @property
    def current_angle(self):
        """Get current angle from the servo motor"""
        return self.servo.current_angle

    def update(self):
        # Calculate time since last update
        now = time.ticks_ms()
        if self.last_update is None:
            self.last_update = now
            return
        dt = time.ticks_diff(now, self.last_update) / 1000.0  # Convert to seconds
        self.last_update = now
        # Clamp dt to prevent large jumps (e.g., timer delay or missed callback)
        max_dt = 0.1  # 100 ms max step
        if dt > max_dt:
            dt = max_dt

        # Update servo position based on speed
        dtheta = self.speed * dt
        next_angle = self.current_angle
        if self.current_angle < self.target:
            next_angle = min(self.target, next_angle + dtheta)
        else:
            next_angle = max(self.target, next_angle - dtheta)

        # Write
        self.write(next_angle)

    # Helpers to set target and speed
    def snap_to(self, angle):
        self.target = angle
        self.speed = float('inf')  # Snap to position immediately
    
    def snap_by(self, increment):
        self.target += increment
        self.speed = float('inf')  # Snap to new position immediately
    
    def move_to(self, angle, speed=10):
        self.target = angle
        self.speed = speed  # Set speed for movement
    
    def move_by(self, increment, speed=10):
        self.target += increment
        self.speed = speed  # Set speed for movement
    
    def set_vel(self, avel):
        self.speed = abs(avel)  # Use magnitude for speed
        self.target = self.servo.max_user_angle if avel >= 0 else self.servo.min_user_angle  # Direction determines target

class Arm2D:
    def __init__(self, shoulder, elbow, arm_len=15, forearm_len=15):
        self.shoulder = shoulder
        self.elbow = elbow
        self.arm_len = arm_len
        self.forearm_len = forearm_len

        self.target = (arm_len, 0)  # Default target position
        self.speed = 0
        self.last_update = None
        self.tmr = None
        
        # Workspace limits
        self.x_min = 9
        self.x_max = 30
        self.y_min = -7
        self.y_max = 30

    @property
    def enabled(self):
        """Check if arm control is enabled"""
        return self.tmr is not None
    
    def _validate_position(self, x, y):
        """Validate that (x,y) is within allowed workspace limits"""
        if not (self.x_min <= x <= self.x_max):
            raise ValueError(f"X position {x} outside valid range [{self.x_min}, {self.x_max}]")
        if not (self.y_min <= y <= self.y_max):
            raise ValueError(f"Y position {y} outside valid range [{self.y_min}, {self.y_max}]")
        return True

    def FK(self):
        """
        Returns (x, y) position of the end effector using forward kinematics.
        Assumes shoulder and elbow are Joint objects.
        """
        # Get current angles in degrees
        theta1 = self.shoulder.current_angle
        theta2 = self.elbow.current_angle
        # Convert to radians
        t1 = math.radians(theta1)
        t2 = math.radians(theta2)
        # FK equations
        x = self.arm_len * math.cos(t1) + self.forearm_len * math.cos(t1 + t2)
        y = self.arm_len * math.sin(t1) + self.forearm_len * math.sin(t1 + t2)
        return (x, y)

    def IK(self, x, y):
        """
        Inverse kinematics to calculate angles for given (x, y) position.
        Returns (theta1, theta2) in degrees.
        """
        y= -y  # Invert y for correct orientation
        # Calculate distance to target point
        d = math.sqrt(x**2 + y**2)
        if d > self.arm_len + self.forearm_len:
            raise ValueError("Target out of reach")
        
        # Law of cosines
        cos_theta2 = (d**2 - self.arm_len**2 - self.forearm_len**2) / (2 * self.arm_len * self.forearm_len)
        theta2 = math.degrees(math.acos(cos_theta2))
        
        k1 = self.arm_len + self.forearm_len * cos_theta2
        k2 = self.forearm_len * math.sin(math.radians(theta2))
        theta1 = math.degrees(math.atan2(y, x) - math.atan2(k2, k1))
        
        return (-theta1, -theta2)

    def set(self, x, y):
        """
        Move arm to specified (x, y) position using inverse kinematics.
        """
        self._validate_position(x, y)
        theta1, theta2 = self.IK(x, y)
        self.shoulder.write(theta1)
        self.elbow.write(theta2)
    
    def enable(self):
        """
        Start periodic updates to move arm towards target position.
        """
        if not self.enabled:
            # SAFETY: Disable individual joints to avoid control conflicts
            if self.shoulder.enabled:
                self.shoulder.disable()
            if self.elbow.enabled:
                self.elbow.disable()
                
            # SAFETY: Always reset target to current position to prevent sudden jumps
            self.target = self.FK()  # Recalculate from current joint angles
            self.tmr = machine.Timer(1)  # Use Timer 1 for Arm2D
            self.tmr.init(period=20, mode=machine.Timer.PERIODIC, callback=lambda t: self.update())
            
    def disable(self):
        """
        Stop periodic updates.
        """
        if self.enabled:
            self.tmr.deinit()
            self.tmr = None
            self.last_update = None

    def update(self):
        """
        Update arm position based on current target and speed.
        """
        # SAFETY: Auto-disable if any joint is enabled to avoid control conflicts
        if self.shoulder.enabled or self.elbow.enabled:
            self.disable()
            return

        # Calculate time since last update
        now = time.ticks_ms()
        if self.last_update is None:
            self.last_update = now
            return
        dt = time.ticks_diff(now, self.last_update) / 1000.0  # Convert to seconds
        self.last_update = now
        # Clamp dt to prevent large jumps (e.g., timer delay or missed callback)
        max_dt = 0.1  # 100 ms max step
        if dt > max_dt:
            dt = max_dt

        # Update position based on speed
        current_x, current_y = self.FK()
        target_x, target_y = self.target

        # Calculate direction vector to target
        dx_total = target_x - current_x
        dy_total = target_y - current_y
        distance_to_target = math.sqrt(dx_total*dx_total + dy_total*dy_total)

        if distance_to_target < 0.01:  # Close enough to target
            return

        # Calculate step size based on speed and time
        step_size = self.speed * dt

        # Normalize direction and scale by step size
        dx = (dx_total / distance_to_target) * step_size
        dy = (dy_total / distance_to_target) * step_size

        # Don't overshoot the target
        if step_size >= distance_to_target:
            next_x, next_y = target_x, target_y
        else:
            next_x = current_x + dx
            next_y = current_y + dy

        # Try to move to new position, handle unreachable targets gracefully
        try:
            self.set(next_x, next_y)
        except ValueError:
            # Target unreachable or out of bounds - stop at current position
            self.target = self.FK()
            self.speed = 0

    # Cartesian control methods similar to Joint class
    def snap_to(self, x=None, y=None):
        """Snap arm to (x,y) position immediately. If x or y is None, use current value."""
        # Stop any active velocity movement first
        self.set_vels(vx=0, vy=0)
        # Now use FK() to get actual current position
        current_x, current_y = self.FK()
        if x is None:
            x = current_x
        if y is None:
            y = current_y
        self._validate_position(x, y)
        self.target = (x, y)
        self.speed = float('inf')

    def snap_by(self, dx=0, dy=0):
        """Snap arm by relative offset immediately"""
        # Stop any active velocity movement first
        self.set_vels(vx=0, vy=0)
        # Now use FK() to get actual current position
        current_x, current_y = self.FK()
        new_x, new_y = current_x + dx, current_y + dy
        self._validate_position(new_x, new_y)
        self.target = (new_x, new_y)
        self.speed = float('inf')

    def move_to(self, x=None, y=None, speed=10):
        """Move arm to (x,y) position at specified speed. If x or y is None, use current value."""
        # Stop any active velocity movement first
        self.set_vels(vx=0, vy=0)
        # Now use FK() to get actual current position
        current_x, current_y = self.FK()
        if x is None:
            x = current_x
        if y is None:
            y = current_y
        self._validate_position(x, y)
        self.target = (x, y)
        self.speed = speed
    
    def move_by(self, dx=0, dy=0, speed=10):
        """Move arm by relative offset at specified speed"""
        # Stop any active velocity movement first
        self.set_vels(vx=0, vy=0)
        # Now use FK() to get actual current position
        current_x, current_y = self.FK()
        # Skip tiny movements that are likely precision errors
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return
        new_x, new_y = current_x + dx, current_y + dy
        self._validate_position(new_x, new_y)
        self.target = (new_x, new_y)
        self.speed = speed

    def set_vels(self, vx=None, vy=None):
        """Set cartesian velocity - arm moves continuously in direction of velocity vector.
        If vx or vy is None, maintain that component of the current velocity."""
        
        # Update components, keeping current value if None
        final_vx = vx if vx is not None else self.get_vels()[0]
        final_vy = vy if vy is not None else self.get_vels()[1]
        
        # Compute new velocity
        self.speed = math.sqrt(final_vx*final_vx + final_vy*final_vy)
        if self.speed > 0:
            # Set target to a point far in the direction of velocity
            scale = 1000  # Large distance
            current_x, current_y = self.FK()
            self.target = (current_x + final_vx/self.speed * scale, 
                         current_y + final_vy/self.speed * scale)
        else:
            # Zero velocity - stop at current position
            self.target = self.FK()

    def get_vels(self):
        """Get current cartesian velocity components (vx, vy)"""
        if self.speed > 0:  # Currently moving
            current_x, current_y = self.FK()
            # Extract current velocity direction
            dx = self.target[0] - current_x
            dy = self.target[1] - current_y
            # Normalize to get unit vector, then scale by current speed
            d = math.sqrt(dx*dx + dy*dy)
            if d < 0.01:  # Target is (almost) at current position
                return (0, 0)
            return (dx/d * self.speed, dy/d * self.speed)
        else:  # Not moving
            return (0, 0)


# Inicializar motores y sensores
# Initialize arm (servo motors and joints)
servo_a = ServoMotor(18, start_angle=60)  # GPIO18 - excellent PWM pin for servos
servo_b = ServoMotor(19, scale=-1.0, start_angle=-120)  # GPIO19 - excellent PWM pin for servos
shoulder = Joint(servo_a)
elbow = Joint(servo_b)
arm = Arm2D(shoulder, elbow)
arm.enable()

# Initialize platform (DC motor and encoder)
motor = DCMotor(ena_pin=25, in1_pin=26, in2_pin=27)
encoder = Encoder(pin_a=32, pin_b=33)  # GPIO32/33 for encoder channels A/B
platform = EncodedMotor(motor, encoder)
platform.enable()

# Initialize electromagnet
mag = Pin(15, Pin.OUT)

# Initialize LED
led = Pin(2, Pin.OUT)  # GPIO2 - onboard LED on most ESP32 boards


# =============================================================================
# CONTROL INDIVIDUAL DE ARTICULACIONES (Joint control)
# =============================================================================

def moverHombro(angulo, velocidad=10):
    """
    Mueve el hombro a un ángulo específico en grados.
    
    Parámetros:
    - angulo: Ángulo objetivo en grados
    - velocidad: Velocidad de movimiento (grados/segundo)
    """
    shoulder.enable()  # Habilita control individual del hombro
    shoulder.move_to(angulo, velocidad)

def moverCodo(angulo, velocidad=10):
    """
    Mueve el codo a un ángulo específico en grados.
    
    Parámetros:
    - angulo: Ángulo objetivo en grados
    - velocidad: Velocidad de movimiento (grados/segundo)
    """
    elbow.enable()  # Habilita control individual del codo
    elbow.move_to(angulo, velocidad)

def incHombro(incremento, velocidad=10):
    """
    Incrementa la posición del hombro por un valor relativo.
    
    Parámetros:
    - incremento: Cambio en grados (puede ser negativo)
    - velocidad: Velocidad de movimiento (grados/segundo)
    """
    shoulder.enable()
    shoulder.move_by(incremento, velocidad)

def incCodo(incremento, velocidad=10):
    """
    Incrementa la posición del codo por un valor relativo.
    
    Parámetros:
    - incremento: Cambio en grados (puede ser negativo)
    - velocidad: Velocidad de movimiento (grados/segundo)
    """
    elbow.enable()
    elbow.move_by(incremento, velocidad)

def velHombro(velocidad_angular):
    """
    Establece velocidad constante del hombro.
    
    Parámetros:
    - velocidad_angular: Velocidad en grados/segundo (+ o -)
    """
    shoulder.enable()
    shoulder.set_vel(velocidad_angular)

def velCodo(velocidad_angular):
    """
    Establece velocidad constante del codo.
    
    Parámetros:
    - velocidad_angular: Velocidad en grados/segundo (+ o -)
    """
    elbow.enable()
    elbow.set_vel(velocidad_angular)

# =============================================================================
# CONTROL COORDINADO DEL BRAZO (Arm2D control)
# =============================================================================

def moverBrazo(x, y, velocidad=10):
    """
    Mueve la punta del brazo a coordenadas cartesianas específicas.
    
    Parámetros:
    - x: Coordenada X en centímetros
    - y: Coordenada Y en centímetros  
    - velocidad: Velocidad de movimiento (cm/segundo)
    """
    arm.enable()  # Habilita control coordinado del brazo
    arm.move_to(x, y, velocidad)

def incBrazo(dx, dy, velocidad=10):
    """
    Incrementa la posición de la punta del brazo por valores relativos.
    
    Parámetros:
    - dx: Cambio en X (centímetros, puede ser negativo)
    - dy: Cambio en Y (centímetros, puede ser negativo)
    - velocidad: Velocidad de movimiento (cm/segundo)
    """
    arm.enable()
    arm.move_by(dx, dy, velocidad)

def velBrazo(vx, vy):
    """
    Establece velocidades constantes del brazo en coordenadas cartesianas.
    
    Parámetros:
    - vx: Velocidad en X (cm/segundo)
    - vy: Velocidad en Y (cm/segundo)
    """
    arm.enable()
    arm.set_vels(vx, vy)

def posicionBrazo():
    """
    Obtiene la posición actual de la punta del brazo.
    
    Retorna:
    - tupla (x, y) con coordenadas en centímetros
    """
    return arm.FK()

# =============================================================================
# CONTROL DE PLATAFORMA GIRATORIA
# =============================================================================

def moverPlataforma(angulo, velocidad=10):
    """
    Mueve la plataforma giratoria a un ángulo específico.
    
    Parámetros:
    - angulo: Ángulo objetivo en grados
    - velocidad: Velocidad de movimiento (grados/segundo)
    """
    platform.enable()
    # Convertir ángulo a counts y establecer velocidad
    counts = platform.degs_to_counts(angulo)
    vel_counts = platform.degs_to_counts(velocidad)
    platform.speed = vel_counts
    platform.snap_to(counts)

def incPlataforma(incremento, velocidad=10):
    """
    Incrementa la posición de la plataforma por un valor relativo.
    
    Parámetros:
    - incremento: Cambio en grados (puede ser negativo)
    - velocidad: Velocidad de movimiento (grados/segundo)  
    """
    platform.enable()
    # Convertir incremento a counts y establecer velocidad
    inc_counts = platform.degs_to_counts(incremento)
    vel_counts = platform.degs_to_counts(velocidad)
    platform.speed = vel_counts
    platform.snap_by(inc_counts)

def velPlataforma(velocidad_angular):
    """
    Establece velocidad constante de la plataforma giratoria.
    
    Parámetros:
    - velocidad_angular: Velocidad en grados/segundo (+ o -)
    """
    platform.enable()
    platform.set_vel(velocidad_angular)

# =============================================================================
# FUNCIONES DE UTILIDAD Y CONTROL GENERAL
# =============================================================================

def habilitarBrazo():
    """Habilita el control coordinado del brazo (deshabilita control individual)"""
    arm.enable()

def deshabilitarBrazo():
    """Deshabilita el control coordinado del brazo"""
    arm.disable()

def habilitarPlataforma():
    """Habilita el control de la plataforma giratoria"""
    platform.enable()

def deshabilitarPlataforma():
    """Deshabilita el control de la plataforma giratoria"""
    platform.disable()

def estadoBrazo():
    """
    Muestra información de depuración sobre el estado del brazo.
    """
    pos_actual = arm.FK()
    print(f"Posición actual: ({pos_actual[0]:.2f}, {pos_actual[1]:.2f}) cm")
    print(f"Objetivo: {arm.target}")
    print(f"Velocidad: {arm.speed}")
    print(f"Brazo habilitado: {arm.enabled}")
    print(f"Hombro habilitado: {shoulder.enabled}")
    print(f"Codo habilitado: {elbow.enabled}")

def encenderLED():
    """Enciende el LED integrado"""
    led.value(1)

def apagarLED():
    """Apaga el LED integrado"""
    led.value(0)

def encenderIman():
    """Activa el electroimán"""
    mag.value(1)

def apagarIman():
    """Desactiva el electroimán"""
    mag.value(0)

# =============================================================================
# EJEMPLOS DE USO
# =============================================================================

def ejemplo_basico():
    """
    Función de ejemplo que demuestra el uso básico del módulo.
    """
    print("=== Ejemplo de uso del módulo brazo_simple ===")
    
    # Mostrar posición inicial
    print("Posición inicial:")
    estadoBrazo()
    
    # Mover hombro individualmente
    print("\nMoviendo hombro a 45 grados...")
    moverHombro(45, velocidad=20)
    
    # Esperar un poco y mover el brazo coordinado
    print("\nMoviendo brazo a coordenadas (20, 5)...")
    moverBrazo(20, 5, velocidad=15)
    
    # Incrementar posición
    print("\nIncrementando posición del brazo...")
    incBrazo(dx=2, dy=1, velocidad=10)
    
    print("\nEjemplo completado. Use estadoBrazo() para ver el estado actual.")

def set_led(value):
    """Handle LED control (0-255, but treat as binary)"""
    led.value(1 if value else 0)

def set_vx(value):
    """Handle X velocity control (-128 to 127 cm/s)"""
    vx = to_bipolar(value) * 128
    arm.set_vels(vx=vx)  # Update only X component

    # Debug
    print(f"Value: {value}, Vx: {vx}, Arm target: {arm.target}, Arm pos: {arm.FK()}, speed: {arm.speed}")

def set_vy(value):
    """Handle Y velocity control (-128 to 127 cm/s)"""
    vy = to_bipolar(value) * 128
    arm.set_vels(vy=vy)  # Update only Y component

def set_w(value):
    """Handle angular velocity control"""
    w = to_bipolar(value)
    # w_counts = platform.degs_to_counts(w * 128)  # Scale to ±128 deg/s
    # platform.set_vel(w_counts)
    platform.disable()
    motor.set_power(w)  # Direct control for simplicity

def set_mag(value):
    """Handle electromagnet control (0-255, but treat as binary)"""
    mag.value(1 if value else 0)

def inc_x(value):
    """Increment X position by a small amount (-30 to 30 cm)"""
    delta = to_bipolar(value) * 32  # Scale to ±32 cm
    arm.move_by(dx=delta)
    
    # Debug
    print(f"Value: {value}, Delta X: {delta}, Arm target: {arm.target}, Arm pos: {arm.FK()}, speed: {arm.speed}")

def inc_y(value):
    """Increment Y position by a small amount (-30 to 30 cm)"""
    delta = to_bipolar(value) * 32  # Scale to ±32 cm
    arm.move_by(dy=delta)

def inc_theta(value):
    """Increment angle by a small amount (-90 to 90 degrees)"""
    delta = to_bipolar(value) * 128  # Scale to ±128 degrees
    platform.snap_by(delta)

# Set up BLE server
# bs.DEVICE_NAME = "RoboArm"
control_callbacks = {
    1: set_led,
    2: set_vx,
    3: set_vy,
    4: set_w,
    5: set_mag,
    6: inc_x,
    7: inc_y,
    8: inc_theta,
}

if __name__ == "__main__":
    print("Módulo brazo_simple cargado correctamente")
    print("Funciones disponibles:")
    print("- Control individual: moverHombro(), moverCodo(), incHombro(), incCodo(), velHombro(), velCodo()")
    print("- Control coordinado: moverBrazo(), incBrazo(), velBrazo(), posicionBrazo()")
    print("- Plataforma: moverPlataforma(), incPlataforma(), velPlataforma()")
    print("- Utilidades: estadoBrazo(), encenderLED(), apagarLED(), encenderIman(), apagarIman()")
    print("- Ejecute ejemplo_basico() para ver una demostración")
    start()
