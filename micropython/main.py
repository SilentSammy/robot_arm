import machine
from machine import Pin, PWM
import time
import math
from hardware import ServoMotor, Encoder, DCMotor
from servo_kin import Joint, Arm2D
from motor_kin import EncodedMotor
import tcp_server as tcp

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

# Initialize LED for TCP demo
led = Pin(2, Pin.OUT)  # GPIO2 - onboard LED on most ESP32 boards
led_state = False

# Track current arm velocities for partial updates
current_vx = 0.0
current_vy = 0.0

def plat_stats():
    print(platform._current_target_counts)
    print(platform.encoder.get_count())

def led_command(args):
    global led_state
    # LED: 1 (on), 0 (off), toggle if no arg or arg is None
    if not args or args[0] is None:
        led_state = not led_state
        led.value(led_state)
        return str(int(led_state))
    try:
        val = int(args[0])
        led_state = bool(val)
        led.value(led_state)
        return str(int(led_state))
    except Exception:
        return 'ERR'

def vel_command(args):
    # VEL: vx, vy, w (vx=arm x vel, vy=arm y vel, w=platform deg/s)
    global current_vx, current_vy
    try:
        vx = float(args[0]) if len(args) > 0 and args[0] is not None else None
        vy = float(args[1]) if len(args) > 1 and args[1] is not None else None
        w_deg = float(args[2]) if len(args) > 2 and args[2] is not None else None
    except Exception:
        return 'ERR'
    if vx is not None:
        current_vx = vx
    if vy is not None:
        current_vy = vy
    if (vx is not None) or (vy is not None):
        arm.set_vels(current_vx, current_vy)
    if w_deg is not None:
        w_counts = platform.degs_to_counts(w_deg)
        platform.set_vel(w_counts)
    return 'OK'

def dp_command(args):
    # DP: dx, dy, dt (all optional, just numbers, default 0)
    try:
        dx = int(float(args[0])) if len(args) > 0 and args[0] is not None else 0
        dy = int(float(args[1])) if len(args) > 1 and args[1] is not None else 0
        dt = float(args[2]) if len(args) > 2 and args[2] is not None else 0
    except Exception:
        return 'ERR'
    arm.move_by(dx, dy)
    dt_counts = platform.degs_to_counts(dt)
    platform.snap_by(dt_counts)
    return 'OK'

def mag_command(args):
    # MAG: 1 (on), 0 (off), toggle if no arg or arg is None
    current = mag.value()
    if not args or args[0] is None:
        new_state = 0 if current else 1
    else:
        try:
            new_state = 1 if int(args[0]) else 0
        except Exception:
            return 'ERR'
    mag.value(new_state)
    return str(mag.value())

def spin_command(args):
    # SPIN[:speed] (default 100)
    speed = 100
    if args and args[0]:
        try:
            speed = int(float(args[0]))
        except Exception:
            return 'ERR'
    platform.set_vel(speed)
    return 'OK'

def stop_command(args):
    # STOP
    platform.set_vel(0)
    return 'OK'


# Set up TCP command map for the tcp_server module
tcp.command_map = {
    'LED': led_command,
    'VEL': vel_command,
    'DP': dp_command,
    'MAG': mag_command,
    'SPIN': spin_command,
    'STOP': stop_command,
}

if __name__ == "__main__":
    # Connect to WiFi using wifi.txt (if needed)
    tcp.connect_to_wifi_from_file("wifi.txt")
    print("Starting TCP server...")
    tcp.start(port=12345)
