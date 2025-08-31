import machine
from machine import Pin, PWM
import time
import math

class EncodedMotor:
    def __init__(self, motor, encoder, counts_per_rev=740, zero_offset=0, max_power=0.25, kP=0.05, kD=0.001):
        self.motor = motor
        self.encoder = encoder
        self.counts_per_rev = counts_per_rev  # Encoder counts per full revolution
        self.zero_offset = zero_offset        # Encoder count at 0 degrees
        start_count = encoder.get_count()
        self._current_target_counts = float(start_count)  # Use float for smooth increments
        self._final_target_counts = float(start_count)    # The user sets this but indirectly, in degrees
        self._speed_counts = counts_per_rev / 4
        self.tmr = None
        self.last_update = None
        self.max_power = max_power
        self.kP = kP
        self.kD = kD
    
    @property
    def speed(self):
        # Speed in counts/sec
        return self._speed_counts

    @speed.setter
    def speed(self, value):
        # Accepts counts/sec directly
        self._speed_counts = abs(value)

    @property
    def final_target(self):
        # Final target in encoder counts
        return self._final_target_counts

    @final_target.setter
    def final_target(self, counts):
        self._final_target_counts = int(counts)

    @property
    def enabled(self):
        return self.tmr is not None

    def enable(self):
        if not self.enabled:
            now_count = self.encoder.get_count()
            self._current_target_counts = now_count
            self._final_target_counts = now_count
            self.tmr = machine.Timer(2)
            # Use a named callback instead of lambda for reliability
            def _timer_cb(t):
                self.update()
            self.tmr.init(period=20, mode=machine.Timer.PERIODIC, callback=_timer_cb)

    def disable(self):
        if self.enabled:
            self.tmr.deinit()
            self.tmr = None
            self.last_update = None
            self.motor.set_power(0)  # Stop motor

    def update(self):
        # Speed-limited position update, then PD control to chase _current_target_counts
        now = time.ticks_ms()
        if self.last_update is None:
            self.last_update = now
            return
        dt = time.ticks_diff(now, self.last_update) / 1000.0
        self.last_update = now
        max_dt = 0.1
        min_dt = 0.005
        if dt > max_dt or dt < min_dt:
            # Ignore updates with too large or too small dt
            return

        # Debug: print dt, max_step, delta
        delta = self._final_target_counts - self._current_target_counts
        max_step = self._speed_counts * dt
        if abs(delta) <= max_step:
            self._current_target_counts = self._final_target_counts
        else:
            self._current_target_counts += max_step if delta > 0 else -max_step
        # Snap exactly if within 1 count
        if abs(self._final_target_counts - self._current_target_counts) < 1e-3:
            self._current_target_counts = self._final_target_counts

        # Set motor power
        power = self.control(dt)
        self.motor.set_power(power)

    def control(self, dt):
        pos = self.encoder.get_count()
        error = int(round(self._current_target_counts)) - pos
        # Derivative on measurement: d_error = -(pos - last_pos) / dt
        if not hasattr(self, '_last_pos'):
            self._last_pos = pos
        d_error = -(pos - self._last_pos) / dt if dt > 0 else 0
        self._last_pos = pos
        power = self.kP * error + self.kD * d_error
        # Clamp power to [-max_power, max_power]
        maxp = abs(self.max_power)
        power = max(-maxp, min(maxp, power))
        return power
    
    # Helper methods
    def snap_by(self, counts):
        self._final_target_counts += counts
        self._current_target_counts = self._final_target_counts
    
    def snap_to(self, counts):
        # Snap to the equivalent position within one revolution, choosing the shortest path
        # counts: desired absolute position (can be any int)
        current = self.encoder.get_count()
        rev = self.counts_per_rev
        # Find the wrapped target closest to current
        # Normalize both to [0, rev)
        target_mod = counts % rev
        current_mod = current % rev
        # Compute difference in both directions
        diff = target_mod - current_mod
        # Wrap to [-rev/2, rev/2)
        if diff > rev/2:
            diff -= rev
        elif diff < -rev/2:
            diff += rev
        # The new target is current + diff (shortest path)
        new_target = current + diff
        self._final_target_counts = new_target
        self._current_target_counts = self._final_target_counts

    def set_vel(self, counts_per_sec):
        self._speed_counts = abs(counts_per_sec)
        prev_delta = self._final_target_counts - self._current_target_counts
        prev_sign = 0 if prev_delta == 0 else (1 if prev_delta > 0 else -1)
        new_sign = 0 if counts_per_sec == 0 else (1 if counts_per_sec > 0 else -1)

        if counts_per_sec > 0:
            self._final_target_counts = float('inf')
        elif counts_per_sec < 0:
            self._final_target_counts = float('-inf')
        else:
            now = self.encoder.get_count()
            self._final_target_counts = now
            self._current_target_counts = now
            return

        # If direction changed (ignoring zero), reset _current_target_counts to encoder position
        if prev_sign != 0 and new_sign != 0 and prev_sign != new_sign:
            self._current_target_counts = self.encoder.get_count()

    # Conversion methods
    def counts_to_degs(self, counts):
        """Convert encoder counts to degrees."""
        return counts * 360.0 / self.counts_per_rev

    def degs_to_counts(self, degrees):
        """Convert degrees to encoder counts."""
        return degrees * self.counts_per_rev / 360.0
