import machine
import tcp_server as tcp

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

tcp.connect_wifi("SammyPC", "12345678")
tcp.command_map = {
    'LED': handle_led,
}

# To start the TCP server in the background, call from REPL:
# tcp.start()
