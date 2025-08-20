import machine
from machine import Pin, PWM
import time
import math
from oop import ServoMotor, Joint, Arm2D

class Encoder:
    def __init__(self, pin_a, pin_b):
        """
        Software quadrature encoder using interrupt-based counting.
        pin_a: Channel A pin (main pulse signal)
        pin_b: Channel B pin (for direction detection)
        """
        self.pin_a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self.pin_b = Pin(pin_b, Pin.IN, Pin.PULL_UP)
        self.count = 0
        self.last_a = self.pin_a.value()
        
        # Set up interrupt on pin A (rising and falling edges)
        self.pin_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._irq_handler)
    
    def _irq_handler(self, pin):
        """Interrupt handler for encoder pin A changes - optimized for speed"""
        # Read both pins once to avoid multiple GPIO reads
        current_a = self.pin_a.value()
        
        # Only process if A actually changed (debounce noise)
        if self.last_a != current_a:
            current_b = self.pin_b.value()
            
            # Optimized quadrature decoding
            if current_a == current_b:
                self.count += 1  # Clockwise
            else:
                self.count -= 1  # Counter-clockwise
            
            self.last_a = current_a
    
    def get_count(self):
        """Get current encoder count (raw pulses)"""
        return self.count
    
    def zero(self):
        """Zero the encoder at current position"""
        self.count = 0
    
    def __repr__(self):
        return f"Encoder: {self.get_count()} counts"

def set_motor(value):
    """
    Set motor speed and direction using 3-pin system: ENA (PWM), IN1, IN2.
    value: float from -1 (full reverse) to 1 (full forward)
    """
    value = max(-1, min(1, value))  # Clamp value
    duty = int(abs(value) * 1023)
    
    if value > 0:
        # Forward direction
        IN1.on()
        IN2.off()
        ENA.duty(duty)
    elif value < 0:
        # Reverse direction
        IN1.off()
        IN2.on()
        ENA.duty(duty)
    else:
        # Stop
        IN1.off()
        IN2.off()
        ENA.duty(0)

# Initialize servo motors and joints
servo_a = ServoMotor(18)  # GPIO18 - excellent PWM pin for servos
servo_b = ServoMotor(19, scale=-1.0)  # GPIO19 - excellent PWM pin for servos  
shoulder = Joint(servo_a)
elbow = Joint(servo_b)
arm = Arm2D(shoulder, elbow)

# Motor control pins (3-pin system: ENA as PWM, IN1/IN2 as digital)
# Option 1: Grouped pins on one side (recommended)
ENA = PWM(Pin(25), freq=32)
IN1 = Pin(26, Pin.OUT)
IN2 = Pin(27, Pin.OUT)

# Motor encoder pins (2-channel quadrature encoder)
encoder = Encoder(pin_a=32, pin_b=33)  # GPIO32/33 for encoder channels A/B
print(f"Encoder initialized: {encoder}")

