import machine
from machine import Pin, PWM
import time
import math
from oop import ServoMotor, Joint, Arm2D, Encoder, DCMotor
import web

# Initialize servo motors and joints
servo_a = ServoMotor(18)  # GPIO18 - excellent PWM pin for servos
servo_b = ServoMotor(19, scale=-1.0)  # GPIO19 - excellent PWM pin for servos
shoulder = Joint(servo_a)
elbow = Joint(servo_b)
arm = Arm2D(shoulder, elbow)

# Initialize platform (DC motor and encoder)
motor = DCMotor(ena_pin=25, in1_pin=26, in2_pin=27)
encoder = Encoder(pin_a=32, pin_b=33)  # GPIO32/33 for encoder channels A/B

# Initialize LED for web demo
led = Pin(2, Pin.OUT)  # GPIO2 - onboard LED on most ESP32 boards
led_state = False

def toggle_led(request):
    """Toggle LED endpoint for web interface testing"""
    global led_state
    led_state = not led_state
    led.value(led_state)
    return {
        "led_state": "on" if led_state else "off",
        "message": f"LED turned {'on' if led_state else 'off'}"
    }

# Define web endpoints
endpoints = {
    "toggle": toggle_led,
}

# Uncomment the following lines to start web server
web.connect_wifi()
print("Starting web server...")
web.start_webserver(endpoints)
