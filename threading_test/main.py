import _thread
from machine import Pin

# Pin 2 on ESP32
led = Pin(2, Pin.OUT)

# Globals for blink control
_blink_should_run = False
_blink_thread_running = False

def _blink_loop():
    global _blink_should_run, _blink_thread_running
    _blink_thread_running = True
    while _blink_should_run:
        led.value(1)
        import time; time.sleep(0.5)
        led.value(0)
        import time; time.sleep(0.5)
    led.value(0)
    _blink_thread_running = False

def start_blink():
    global _blink_should_run, _blink_thread_running
    if _blink_thread_running:
        return  # Already running
    _blink_should_run = True
    _thread.start_new_thread(_blink_loop, ())

def stop_blink():
    global _blink_should_run
    _blink_should_run = False

# No free code, will start from REPL command
