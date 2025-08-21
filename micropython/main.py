import machine
from machine import Pin, PWM
import time
import math
from oop import ServoMotor, Joint, Arm2D, Encoder, DCMotor
from cmd_system import CommandProcessor

# Initialize servo motors and joints
servo_a = ServoMotor(18)  # GPIO18 - excellent PWM pin for servos
servo_b = ServoMotor(19, scale=-1.0)  # GPIO19 - excellent PWM pin for servos  
shoulder = Joint(servo_a)
elbow = Joint(servo_b)
arm = Arm2D(shoulder, elbow)
arm.enable()

# Initialize platform (DC motor and encoder)
motor = DCMotor(ena_pin=25, in1_pin=26, in2_pin=27)
encoder = Encoder(pin_a=32, pin_b=33)  # GPIO32/33 for encoder channels A/B

# Command handler functions
# Store velocity components for cartesian control
vx_current = 0.0
vy_current = 0.0

def handle_motor(args):
    """Handle W command: W:0.5 (sets motor power from -1 to 1)"""
    if len(args) != 1:
        return False
    
    try:
        power = float(args[0])
        if power < -1 or power > 1:
            return False
        motor.set_power(power)
    except ValueError:
        return False
    
    return True

def handle_vx(args):
    """Handle VX command: VX:0.5 (sets X velocity component)"""
    global vx_current, vy_current
    if len(args) != 1:
        return False
    
    try:
        vx_current = float(args[0])
        arm.set_vels(vx_current, vy_current)
    except ValueError:
        return False
    
    return True

def handle_vy(args):
    """Handle VY command: VY:0.3 (sets Y velocity component)"""
    global vx_current, vy_current
    if len(args) != 1:
        return False
    
    try:
        vy_current = float(args[0])
        arm.set_vels(vx_current, vy_current)
    except ValueError:
        return False
    
    return True

# Command dispatch table - maps command names to handler functions
command_handlers = {
    "W": handle_motor,
    "VX": handle_vx,
    "VY": handle_vy,
    # Add more commands here as needed
}

# Create command processor and start command loop
processor = CommandProcessor(command_handlers)
processor.run_cmd_loop()
