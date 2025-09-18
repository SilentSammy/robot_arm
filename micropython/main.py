import machine
from machine import Pin, PWM
import time
import math
from hardware import ServoMotor, Encoder, DCMotor
from servo_kin import Joint, Arm2D
from motor_kin import EncodedMotor
import ble_server as bs

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

def set_led(value):
    """Handle LED control (0-255, but treat as binary)"""
    led.value(1 if value else 0)

def set_vx(value):
    """Handle X velocity control (-100 to 100 cm/s)"""
    vx = bs.to_bipolar(value) * 100
    arm.set_vels(vx=vx)  # Update only X component

    # Debug arm target and speed
    print(f"Arm target: {arm.target}, speed: {arm.speed}")

def set_vy(value):
    """Handle Y velocity control (-100 to 100 cm/s)"""
    vy = bs.to_bipolar(value) * 100
    arm.set_vels(vy=vy)  # Update only Y component

def set_w(value):
    """Handle angular velocity control"""
    w = bs.to_bipolar(value)
    w_counts = platform.degs_to_counts(w * 100)  # Scale to ±100 deg/s
    platform.set_vel(w_counts)

def set_mag(value):
    """Handle electromagnet control (0-255, but treat as binary)"""
    mag.value(1 if value else 0)

def inc_x(value):
    """Increment X position by a small amount (-30 to 30 cm)"""
    delta = bs.to_bipolar(value) * 30  # Scale to ±30 cm
    arm.move_by(dx=delta)
    
    # Debug arm target and speed
    print(f"Arm target: {arm.target}, speed: {arm.speed}")

def inc_y(value):
    """Increment Y position by a small amount (-30 to 30 cm)"""
    delta = bs.to_bipolar(value) * 30  # Scale to ±30 cm
    arm.move_by(dy=delta)

def inc_theta(value):
    """Increment angle by a small amount (-90 to 90 degrees)"""
    delta = bs.to_bipolar(value) * 90  # Scale to ±90 degrees
    platform.snap_by(delta)

# Set up BLE server
bs.DEVICE_NAME = "RoboArm"
bs.control_callbacks = {
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
    print("Starting BLE server...")
    bs.start()
