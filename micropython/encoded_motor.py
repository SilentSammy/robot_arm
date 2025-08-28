import machine
from machine import Pin, PWM
import time
import math

class EncodedMotor:
    def __init__(self, motor, encoder, counts_per_rev=740, zero_offset=0, max_power=0.25, kP=0.05, kD=0.003):
        self.motor = motor
        self.encoder = encoder
        self.counts_per_rev = counts_per_rev  # Encoder counts per full revolution
        self.zero_offset = zero_offset        # Encoder count at 0 degrees
        self._target_counts = encoder.get_count()
        self.tmr = None
        self.last_update = None
        self.max_power = max_power
        self.kP = kP
        self.kD = kD

    @property
    def target(self):
        """Get/set target in degrees (converted to internal encoder counts)"""
        # Convert counts to degrees
        return (self._target_counts - self.zero_offset) * 360.0 / self.counts_per_rev

    @target.setter
    def target(self, deg):
        # Convert degrees to counts
        self._target_counts = int(round(self.zero_offset + (deg * self.counts_per_rev / 360.0)))

    @property
    def enabled(self):
        return self.tmr is not None

    def enable(self):
        if not self.enabled:
            self._target_counts = self.encoder.get_count()  # Reset target to current position
            self.tmr = machine.Timer(2)  # Use a virtual timer (or pick a unique number)
            self.tmr.init(period=20, mode=machine.Timer.PERIODIC, callback=lambda t: self.update())

    def disable(self):
        if self.enabled:
            self.tmr.deinit()
            self.tmr = None
            self.last_update = None
            self.motor.set_power(0)  # Stop motor

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

        # PD control (derivative on measurement to reduce kick)
        pos = self.encoder.get_count()
        error = self._target_counts - pos
        # Derivative on measurement: d_error = -(pos - last_pos) / dt
        if not hasattr(self, '_last_pos'):
            self._last_pos = pos
        d_error = -(pos - self._last_pos) / dt if dt > 0 else 0
        self._last_pos = pos
        power = self.kP * error + self.kD * d_error
        # Clamp power to [-max_power, max_power]
        maxp = abs(self.max_power)
        power = max(-maxp, min(maxp, power))
        self.motor.set_power(power)
