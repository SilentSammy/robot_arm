from machine import Pin, PWM
import ble_server as bs

# Example usage:
led_pin = Pin(2, Pin.OUT)  # GPIO2 is the built-in LED
pwm_led = PWM(led_pin)     # Create PWM object
pwm_led.freq(1000)         # Set frequency to 1kHz

def set_led(value):
    # Convert 0-255 to 0.0-1.0, then to 0-1023 (ESP32 PWM range)
    brightness = bs.to_norm(value)  # 0.0-1.0
    duty = int(brightness * 1023)   # ESP32 PWM is 10-bit (0-1023)
    pwm_led.duty(duty)
    print(f'LED brightness: {brightness:.1%}')

# Add the callback with a specific control ID
bs.control_callbacks[0] = set_led  # LED control will always be ID 0

# Start the server
bs.start()
