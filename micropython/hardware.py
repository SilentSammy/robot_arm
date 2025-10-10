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
