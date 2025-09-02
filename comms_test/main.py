import machine
from tcp_server import start_tcp_server

led = machine.Pin(2, machine.Pin.OUT)

def handle_led(args):
    if not args:
        return 'ERR: No value'
    try:
        val = int(args[0])
        led.value(1 if val else 0)
        return f'LED set to {val}'
    except Exception as e:
        return f'ERR: {e}'

command_map = {
    'LED': handle_led,
}

start_tcp_server(command_map)
