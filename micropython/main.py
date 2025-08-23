import machine
from machine import Pin, PWM
import time
import math
from hardware import ServoMotor, Encoder, DCMotor
from control import Joint, Arm2D
import web

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

# Initialize LED for web demo
led = Pin(2, Pin.OUT)  # GPIO2 - onboard LED on most ESP32 boards
led_state = False

# Track current arm velocities for partial updates
current_vx = 0.0
current_vy = 0.0

def toggle_led(request):
    """Toggle LED endpoint for web interface testing"""
    global led_state
    led_state = not led_state
    led.value(led_state)
    return {
        "led_state": "on" if led_state else "off",
        "message": f"LED turned {'on' if led_state else 'off'}"
    }

def control(request):
    """Control endpoint for motor power and arm velocities"""
    global current_vx, current_vy
    
    params = request.get('params', {})
    print(f"Control request: {params}")  # Debug: show incoming parameters
    response_data = {}
    
    # Handle motor power (w parameter)
    if 'w' in params:
        try:
            motor_power = float(params['w'])
            motor_power = max(-1.0, min(1.0, motor_power))  # Clamp to [-1, 1]
            print(f"Setting motor power: {motor_power}")  # Debug: motor control
            motor.set_power(motor_power)  # TODO: Uncomment for testing
            response_data['motor_power'] = motor_power
        except ValueError:
            response_data['motor_error'] = "Invalid motor power value"
    
    # Handle arm velocities (x and y parameters)
    vx_updated = False
    vy_updated = False
    
    if 'x' in params:
        try:
            current_vx = float(params['x'])
            vx_updated = True
            response_data['vx'] = current_vx
        except ValueError:
            response_data['vx_error'] = "Invalid x velocity value"
    
    if 'y' in params:
        try:
            current_vy = float(params['y'])
            vy_updated = True
            response_data['vy'] = current_vy
        except ValueError:
            response_data['vy_error'] = "Invalid y velocity value"
    
    # Set arm velocities if either x or y was updated
    if vx_updated or vy_updated:
        print(f"Setting arm velocities: vx={current_vx}, vy={current_vy}")  # Debug: arm control
        arm.set_vels(current_vx, current_vy)  # TODO: Uncomment for testing
        response_data['arm_vels'] = [current_vx, current_vy]
    
    # Add current state to response
    response_data['status'] = 'ok'
    
    return response_data

# Define web endpoints
endpoints = {
    "toggle": toggle_led,
    "control": control,
}

# Uncomment the following lines to start web server
web.connect_wifi()
print("Starting web server...")
web.start_webserver(endpoints)
