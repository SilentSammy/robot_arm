import machine
from machine import Pin, PWM
import time
import math

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
            # Target unreachable - stop at current position
            self.target = self.FK()
            self.speed = 0

    # Cartesian control methods similar to Joint class
    def snap_to(self, x=None, y=None):
        """Snap arm to (x,y) position immediately. If x or y is None, use current value."""
        current_x, current_y = self.target
        if x is None:
            x = current_x
        if y is None:
            y = current_y
        self.target = (x, y)
        self.speed = float('inf')

    def snap_by(self, dx=0, dy=0):
        """Snap arm by relative offset immediately"""
        current_x, current_y = self.target
        self.target = (current_x + dx, current_y + dy)
        self.speed = float('inf')

    def move_to(self, x=None, y=None, speed=10):
        """Move arm to (x,y) position at specified speed. If x or y is None, use current value."""
        current_x, current_y = self.target
        if x is None:
            x = current_x
        if y is None:
            y = current_y
        self.target = (x, y)
        self.speed = speed
    
    def move_by(self, dx=0, dy=0, speed=10):
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

class EncodedMotor:
    def __init__(self, motor, encoder, max_power=0.25):
        self.motor = motor
        self.encoder = encoder
        self.target = encoder.get_count()
        self.tmr = None
        self.last_update = None
        self.max_power = max_power

    @property
    def enabled(self):
        return self.tmr is not None

    def enable(self):
        if not self.enabled:
            self.target = self.encoder.get_count()  # Reset target to current position
            self.tmr = machine.Timer(2)  # Use a virtual timer (or pick a unique number)
            self.tmr.init(period=20, mode=machine.Timer.PERIODIC, callback=lambda t: self.update())

    def disable(self):
        if self.enabled:
            self.tmr.deinit()
            self.tmr = None
            self.last_update = None

    def update(self):
        # Simple PD control to reach target encoder count
        now = time.ticks_ms()
        if self.last_update is None:
            self.last_update = now
            return
        dt = time.ticks_diff(now, self.last_update) / 1000.0
        self.last_update = now
        max_dt = 0.1
        if dt > max_dt:
            dt = max_dt

        # PD control
        kP = 0.02  # Proportional gain (tune as needed)
        kD = 0.001  # Derivative gain (tune as needed)
        pos = self.encoder.get_count()
        error = self.target - pos
        d_error = 0
        if not hasattr(self, '_last_error'):
            self._last_error = error
        else:
            d_error = (error - self._last_error) / dt if dt > 0 else 0
            self._last_error = error
        power = kP * error + kD * d_error
        # Clamp power to [-max_power, max_power]
        maxp = abs(self.max_power)
        power = max(-maxp, min(maxp, power))
        self.motor.set_power(power)
    
