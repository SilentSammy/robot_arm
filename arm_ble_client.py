from bleak import BleakClient, BleakScanner
import asyncio
import sys
from pynput import keyboard
import inputs
import importlib
import threading
import time

# ——————————————————————————————————————————————————————————————
# Global gamepad state
_pressed_buttons       = set()   # buttons currently down
_just_pressed_buttons  = set()   # buttons that went down since last query
_just_released_buttons = set()   # buttons that went up since last query
_toggles               = {}      # button_name -> bool
_axis_states           = {}      # axis_name -> raw value

# Global keyboard state
pressed_keys = set()
just_pressed_keys = set()
just_released_keys = set()
toggles = {}

# Map raw ev.code → friendly button name
_CODE_TO_NAME = {
    "BTN_SOUTH":     "A",
    "BTN_EAST":      "B",
    "BTN_NORTH":     "Y",
    "BTN_WEST":      "X",
    "BTN_TL":        "LB",
    "BTN_TR":        "RB",
    "BTN_TL2":       "LT",
    "BTN_TR2":       "RT",
    "BTN_SELECT":    "SLCT",
    "BTN_START":     "START",
    "BTN_MODE":      "GUIDE",
    "BTN_THUMBL":    "L3",
    "BTN_THUMBR":    "R3",
    "BTN_DPAD_UP":   "DPAD_UP",
    "BTN_DPAD_DOWN": "DPAD_DOWN",
    "BTN_DPAD_LEFT": "DPAD_LEFT",
    "BTN_DPAD_RIGHT":"DPAD_RIGHT",
}

# Map raw absolute codes → friendly axis names
_ABS_TO_NAME = {
    'ABS_X':     'LX',      # left stick X
    'ABS_Y':     'LY',      # left stick Y
    'ABS_RX':    'RX',      # right stick X
    'ABS_RY':    'RY',      # right stick Y
    'ABS_Z':     'LT',      # left trigger
    'ABS_RZ':    'RT',      # right trigger
    'ABS_HAT0X': 'DPAD_X',  # D-pad horizontal
    'ABS_HAT0Y': 'DPAD_Y',  # D-pad vertical
}

# Track disconnect status to throttle warnings
_warned_disconnected = False

# ——————————————————————————————————————————————————————————————
# Internal helpers
def _repr_button(code: str) -> str:
    """Return a friendly name for a raw event code."""
    return _CODE_TO_NAME.get(code, code)

# ——————————————————————————————————————————————————————————————
# Background event loop

def _gamepad_event_loop():
    global _warned_disconnected
    while True:
        try:
            events = inputs.get_gamepad()
        except Exception:
            if not _warned_disconnected:
                print("Gamepad not connected. Waiting for connection...")
                _warned_disconnected = True
            importlib.reload(inputs)
            # clear stale state
            _pressed_buttons.clear()
            _axis_states.clear()
            time.sleep(1)
            continue
        if _warned_disconnected:
            print("Gamepad connected.")
            _warned_disconnected = False

        for ev in events:
            # DIGITAL BUTTONS
            if ev.ev_type == 'Key':
                name = _repr_button(ev.code)
                if ev.state == 1:
                    if name not in _pressed_buttons:
                        _just_pressed_buttons.add(name)
                        _toggles[name] = not _toggles.get(name, False)
                    _pressed_buttons.add(name)
                else:
                    if name in _pressed_buttons:
                        _just_released_buttons.add(name)
                    _pressed_buttons.discard(name)

            # ANALOG AXES & BINARY MAPPINGS
            elif ev.ev_type == 'Absolute':
                axis = _ABS_TO_NAME.get(ev.code)
                if not axis:
                    continue
                value = ev.state
                # update raw axis state
                _axis_states[axis] = value

                # TRIGGERS as binary buttons
                if axis in ('LT', 'RT'):
                    if value > 0:
                        if axis not in _pressed_buttons:
                            _just_pressed_buttons.add(axis)
                        _pressed_buttons.add(axis)
                    else:
                        if axis in _pressed_buttons:
                            _just_released_buttons.add(axis)
                        _pressed_buttons.discard(axis)

                # D-PAD as binary buttons
                elif axis == 'DPAD_X':
                    # left
                    if value < 0:
                        if 'DPAD_LEFT' not in _pressed_buttons:
                            _just_pressed_buttons.add('DPAD_LEFT')
                        _pressed_buttons.add('DPAD_LEFT')
                    else:
                        if 'DPAD_LEFT' in _pressed_buttons:
                            _just_released_buttons.add('DPAD_LEFT')
                        _pressed_buttons.discard('DPAD_LEFT')
                    # right
                    if value > 0:
                        if 'DPAD_RIGHT' not in _pressed_buttons:
                            _just_pressed_buttons.add('DPAD_RIGHT')
                        _pressed_buttons.add('DPAD_RIGHT')
                    else:
                        if 'DPAD_RIGHT' in _pressed_buttons:
                            _just_released_buttons.add('DPAD_RIGHT')
                        _pressed_buttons.discard('DPAD_RIGHT')

                elif axis == 'DPAD_Y':
                    # up
                    if value < 0:
                        if 'DPAD_UP' not in _pressed_buttons:
                            _just_pressed_buttons.add('DPAD_UP')
                        _pressed_buttons.add('DPAD_UP')
                    else:
                        if 'DPAD_UP' in _pressed_buttons:
                            _just_released_buttons.add('DPAD_UP')
                        _pressed_buttons.discard('DPAD_UP')
                    # down
                    if value > 0:
                        if 'DPAD_DOWN' not in _pressed_buttons:
                            _just_pressed_buttons.add('DPAD_DOWN')
                        _pressed_buttons.add('DPAD_DOWN')
                    else:
                        if 'DPAD_DOWN' in _pressed_buttons:
                            _just_released_buttons.add('DPAD_DOWN')
                        _pressed_buttons.discard('DPAD_DOWN')
        # end for events
# end while

# Start the gamepad event listener thread
_listener = threading.Thread(target=_gamepad_event_loop, daemon=True)
_listener.start()

# Start the keyboard listener
def _on_press(key):
    key_repr = key if isinstance(key, str) else key.char if hasattr(key, 'char') else str(key)
    # Filter out None values before adding to sets
    if key_repr is None:
        return
    # Only mark as just pressed if it wasn't already noted as down.
    if key_repr not in pressed_keys:
        just_pressed_keys.add(key_repr)
    pressed_keys.add(key_repr)
    # Toggle state if applicable.
    if key_repr in toggles:
        toggles[key_repr] = not toggles[key_repr]

def _on_release(key):
    key_repr = key if isinstance(key, str) else key.char if hasattr(key, 'char') else str(key)
    # Filter out None values
    if key_repr is None:
        return
    pressed_keys.discard(key_repr)
    # Mark the key as just released for falling edge detection.
    just_released_keys.add(key_repr)

keyboard.Listener(on_press=_on_press, on_release=_on_release).start()

# ——————————————————————————————————————————————————————————————
# Public API (unified for both gamepad and keyboard)
def is_pressed(input_name):
    """True as long as the given button/key is held down. Checks gamepad first, then keyboard."""
    # Check gamepad buttons first
    if input_name in _pressed_buttons:
        return True
    # Fallback to keyboard
    key_repr = input_name if isinstance(input_name, str) else input_name.char if hasattr(input_name, 'char') else str(input_name)
    return key_repr in pressed_keys

def is_toggled(input_name):
    """Flip-flop state for each press. Checks gamepad first, then keyboard."""
    # Check gamepad toggles first
    if input_name in _toggles:
        return _toggles[input_name]
    # Fallback to keyboard
    key_repr = input_name if isinstance(input_name, str) else input_name.char if hasattr(input_name, 'char') else str(input_name)
    if key_repr not in toggles:
        toggles[key_repr] = False
    return toggles.get(key_repr, False)

def rising_edge(input_name):
    """True exactly once when the button/key first goes down. Checks gamepad first, then keyboard."""
    # Check gamepad just pressed first
    if input_name in _just_pressed_buttons:
        _just_pressed_buttons.remove(input_name)
        return True
    # Fallback to keyboard
    key_repr = input_name if isinstance(input_name, str) else input_name.char if hasattr(input_name, 'char') else str(input_name)
    if key_repr in just_pressed_keys:
        just_pressed_keys.remove(key_repr)
        return True
    return False

def falling_edge(input_name):
    """True exactly once when the button/key first goes up. Checks gamepad first, then keyboard."""
    # Check gamepad just released first
    if input_name in _just_released_buttons:
        _just_released_buttons.remove(input_name)
        return True
    # Fallback to keyboard
    key_repr = input_name if isinstance(input_name, str) else input_name.char if hasattr(input_name, 'char') else str(input_name)
    if key_repr in just_released_keys:
        just_released_keys.remove(key_repr)
        return True
    return False

def get_axis(axis_name: str, normalize: bool = True) -> float:
    """Return axis state. Can be normalized to -1,+1 for sticks and 0,1 for triggers."""
    val = _axis_states.get(axis_name, 0)
    if not normalize:
        return val
    # sticks: -32768..32767 -> -1.0..1.0
    if axis_name in ("LX", "LY", "RX", "RY"):
        normalized = val / (32767.0 if val >= 0 else 32768.0)
        return round(normalized, 1)
    # triggers: 0..255 or 0..1023 -> 0.0..1.0
    if axis_name in ("LT", "RT"):
        maxv = 255 if val <= 255 else 1023
        return val / maxv
    # D-pad: already -1,0,1
    if axis_name in ("DPAD_X", "DPAD_Y"):
        return val
    return val

# Helpers for common use cases
def get_bipolar_ctrl(high_key=None, low_key=None, high_game=None, low_game=None):
    """Returns -1.0 to +1.0. Combines gamepad and keyboard inputs with clamping.
    high_game/low_game can be button names or axis names (LX, LY, RX, RY, etc.)."""
    # Check if high_game/low_game are axes or buttons
    is_high_axis = high_game in _ABS_TO_NAME.values() if high_game else False
    is_low_axis = low_game in _ABS_TO_NAME.values() if low_game else False
    high_val = get_axis(high_game) if is_high_axis else int(is_pressed(high_game or ''))
    low_val = get_axis(low_game) if is_low_axis else int(is_pressed(low_game or ''))
    game_in = high_val - low_val
    key_in = int(is_pressed(high_key or '')) - int(is_pressed(low_key or ''))
    result = game_in + key_in
    return float(max(-1, min(1, result)))


# Conversion helpers
def from_norm(norm):
    """Convert 0.0-1.0 float to 0-255 int"""
    val = int(round(norm * 256))
    return max(0, min(255, val))

def from_bipolar(bipolar):
    """Convert -1.0-1.0 float to 0-255 int"""
    val = int(round((bipolar * 128) + 128))
    return max(0, min(255, val))

def map_vel(val):
    """Map velocity from -128..127 to 0..255 range"""
    rng = 128
    val = max(-1.0, min(1.0, val/rng)) # Normalize to -1..1
    return from_bipolar(val)

class ArmBLEClient:
    def __init__(self, device_name="RoboArm3", base_uuid="12345678-1234-5678-1234-56789abcdef"):
        self.device_name = device_name
        self.base_uuid = base_uuid
        self.device = None
        self.client = None
        
        # Cache last sent mapped values to avoid redundant BLE writes
        self._cached_char_values = {}
    
    # Communication methods
    async def find_device(self):
        print(f"Scanning for {self.device_name}...")
        device = await BleakScanner.find_device_by_filter( lambda d, ad: d.name and d.name.lower() == self.device_name.lower() )
        if not device:
            print(f"Could not find {self.device_name}")
            return None
        print(f"Found {self.device_name} at {device.address}")
        return device

    async def connect(self):
        self.device = await self.find_device()
        if not self.device:
            raise Exception("Device not found")

        self.client = BleakClient(self.device)
        await self.client.connect()
        print(f"Connected to {self.device.name}")

    async def disconnect(self):
        if self.client:
            await self.client.disconnect()
            print("Disconnected.")

    async def set_char(self, char_uuid, value, force=False):
        if not force and self._cached_char_values.get(char_uuid) == value:
            # print("No change, skipping write") # Commented to avoid spamming output
            return  # No change, skip write
        await self.client.write_gatt_char(char_uuid, bytes([value]))
        self._cached_char_values[char_uuid] = value

    # Setters for each control
    async def set_led(self, state):
        """Set the LED state on the device."""
        await self.set_char(self.base_uuid + "1", int(state))

    async def set_mag(self, state):
        """Set the magnet state on the device."""
        await self.set_char(self.base_uuid + "5", int(state))

    async def set_vx(self, vx):
        """Set X velocity (-100 to 100)"""
        mapped = map_vel(vx)
        await self.set_char(self.base_uuid + "2", mapped)

    async def set_vy(self, vy):
        """Set Y velocity (-100 to 100)"""
        mapped = map_vel(vy)
        await self.set_char(self.base_uuid + "3", mapped)

    async def set_w(self, w):
        """Set angular velocity w in degrees/sec (expected range -100..100)."""
        mapped = map_vel(w)
        await self.set_char(self.base_uuid + "4", mapped)

    async def inc_x(self, delta_cm):
        """Increment X position by a small amount (-32 to 32 cm)"""
        mapped = from_bipolar(max(-1.0, min(1.0, delta_cm / 32.0)))
        await self.set_char(self.base_uuid + "6", mapped, force=True)
    
    async def inc_y(self, delta_cm):
        """Increment Y position by a small amount (-32 to 32 cm)"""
        mapped = from_bipolar(max(-1.0, min(1.0, delta_cm / 32.0)))
        await self.set_char(self.base_uuid + "7", mapped, force=True)
    
    async def inc_theta(self, delta_deg):
        """Increment angle by a small amount (-128 to 128 degrees)"""
        mapped = from_bipolar(max(-1.0, min(1.0, delta_deg / 128.0)))
        await self.set_char(self.base_uuid + "8", mapped, force=True)

async def main_loop():
    arm = ArmBLEClient()
    await arm.connect()
    print("Connected")

    try:
        while True:
            await arm.set_led(is_pressed('x') or is_pressed('X'))

            await arm.set_mag(is_pressed('A') or is_pressed('m'))

            # Check for c key rising edge to clear pending edges when switching modes
            c_rising = rising_edge('c')
            
            if not is_pressed('c'): # Velocity control mode
                vxy_mag = 15 # cm/s per key press
                w_mag = 60  # deg/s per key press

                # X axis (A/D)
                vx = get_bipolar_ctrl('d', 'a', 'LX')
                vx *= vxy_mag
                await arm.set_vx(vx)

                # Y axis (W/S)
                vy = get_bipolar_ctrl('w', 's', 'LY')
                vy *= vxy_mag
                await arm.set_vy(vy)

                # Angular velocity (Q/E) - Q = negative, E = positive
                w = get_bipolar_ctrl('e', 'q', 'RX')
                w *= w_mag
                await arm.set_w(w)
            else: # Incremental position mode
                # If we just pressed 'c', consume any lingering rising edges to avoid phantom keypresses
                if c_rising:
                    # Consume any pending rising edges from the control keys
                    rising_edge('a')
                    rising_edge('d')
                    rising_edge('w')
                    rising_edge('s')
                    rising_edge('q')
                    rising_edge('e')
                inc_xy_mag = 1 # cm per key press
                inc_theta_mag = 5 # degrees per key press
                if rising_edge('a'):
                    await arm.inc_x(-inc_xy_mag)
                if rising_edge('d'):
                    await arm.inc_x(inc_xy_mag)
                if rising_edge('w'):
                    await arm.inc_y(inc_xy_mag)
                if rising_edge('s'):
                    await arm.inc_y(-inc_xy_mag)
                if rising_edge('q'):
                    await arm.inc_theta(-inc_theta_mag)
                if rising_edge('e'):
                    await arm.inc_theta(inc_theta_mag)

            await asyncio.sleep(0.01)   # keep the loop responsive
    finally:
        await arm.set_vx(0)
        await arm.set_vy(0)
        await arm.set_w(0)
        await arm.disconnect()

if __name__ == "__main__":
    asyncio.run(main_loop())
