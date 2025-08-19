import machine
from machine import Pin, PWM
import time
import math

class ServoMotor:
    def __init__(self, pin, scale=1.0, offset=0):
        self.pwm = PWM(Pin(pin), freq=50)
        # Standard servo PWM values for ESP32 hardware PWM
        self.min_us = 600  # pulse width for 0 degrees
        self.max_us = 2500  # pulse width for 180 degrees
        self.us_per_duty = 20000 / 1023  # 20ms period, 1023 max duty cycle
        # Remove the ESP8266 factor correction - ESP32 doesn't need it
        
        self.scale = scale
        self.offset = offset
    
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
        self.target = servo.revert_angle(90)  # Default target angle
        self.current_angle = self.target
        self.speed = 0
        self.last_update = None
        self.tmr = None

    def enable(self):
        if self.tmr is None:
            self.tmr = machine.Timer(0)
            self.tmr.init(period=10, mode=machine.Timer.PERIODIC, callback=lambda t: self.update())

    def disable(self):
        if self.tmr is not None:
            self.tmr.deinit()
            self.tmr = None
            self.last_update = None

    def write(self, angle):
        self.servo.write(angle)
        self.current_angle = angle

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
    
    def set_angular_vel(self, avel):
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
        if self.tmr is None:
            self.tmr = machine.Timer(0)
            self.tmr.init(period=10, mode=machine.Timer.PERIODIC, callback=lambda t: self.update())
            
    def disable(self):
        """
        Stop periodic updates.
        """
        if self.tmr is not None:
            self.tmr.deinit()
            self.tmr = None
            self.last_update = None

    def update(self):
        """
        Update arm position based on current target and speed.
        """
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
    
    def set_cartesian_vel(self, vx, vy):
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
