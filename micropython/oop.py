import machine
from machine import Pin, PWM
import time
import math

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
    def __init__(self, ena_pin, in1_pin, in2_pin, freq=32):
        """
        DC Motor control using 3-pin system: ENA (PWM), IN1, IN2.
        ena_pin: PWM pin for speed control
        in1_pin: Digital pin for direction control 1
        in2_pin: Digital pin for direction control 2
        freq: PWM frequency (default 32Hz)
        """
        self.ena = PWM(Pin(ena_pin), freq=freq)
        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)
        
        # Initialize to stopped state
        self.set_power(0)
    
    def set_power(self, value):
        """
        Set motor speed and direction.
        value: float from -1 (full reverse) to 1 (full forward)
        """
        value = max(-1, min(1, value))  # Clamp value
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
    
    def forward(self, power=1.0):
        """Move forward at specified power (0-1)"""
        self.set_power(abs(power))
    
    def reverse(self, power=1.0):
        """Move reverse at specified power (0-1)"""
        self.set_power(-abs(power))

class ServoMotor:
    # Class field for save granularity (degrees)
    SAVE_GRANULARITY = 10
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
            self.tmr.init(period=10, mode=machine.Timer.PERIODIC, callback=lambda t: self.update())

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
    
    def move_to(self, angle, speed):
        self.target = angle
        self.speed = speed  # Set speed for movement
    
    def move_by(self, increment, speed):
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

    @property
    def enabled(self):
        """Check if arm control is enabled"""
        return self.tmr is not None

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
            self.tmr.init(period=10, mode=machine.Timer.PERIODIC, callback=lambda t: self.update())
            
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
            # Target unreachable - stop at current position
            self.target = self.FK()
            self.speed = 0

    # Cartesian control methods similar to Joint class
    def snap_to(self, x, y):
        """Snap arm to (x,y) position immediately"""
        self.target = (x, y)
        self.speed = float('inf')
    
    def snap_by(self, dx, dy):
        """Snap arm by relative offset immediately"""
        current_x, current_y = self.target
        self.target = (current_x + dx, current_y + dy)
        self.speed = float('inf')
    
    def move_to(self, x, y, speed):
        """Move arm to (x,y) position at specified speed"""
        self.target = (x, y)
        self.speed = speed
    
    def move_by(self, dx, dy, speed):
        """Move arm by relative offset at specified speed"""
        current_x, current_y = self.target
        self.target = (current_x + dx, current_y + dy)
        self.speed = speed
    
    def set_vels(self, vx, vy):
        """Set cartesian velocity - arm moves continuously in direction of velocity vector"""
        self.speed = math.sqrt(vx*vx + vy*vy)  # Magnitude of velocity
        if self.speed > 0:
            # Set target to a point far in the direction of velocity
            current_x, current_y = self.FK()
            # Normalize velocity and project far out
            scale = 1000  # Large distance
            self.target = (current_x + vx/self.speed * scale, current_y + vy/self.speed * scale)
        else:
            # Zero velocity - stop at current position
            self.target = self.FK()
