import machine
from micropython import const
import bluetooth

# BLE configs
DEVICE_NAME = 'MicroLED'
# Custom UUIDs for our service and characteristic
LED_SERVICE_UUID = bluetooth.UUID('12345678-1234-5678-1234-56789abcdef0')
LED_CHAR_UUID = bluetooth.UUID('12345678-1234-5678-1234-56789abcdef1')

led = machine.Pin(2, machine.Pin.OUT)
ble = bluetooth.BLE()
ble.active(True)

class LEDService:
    def __init__(self, ble):
        # Create internal GATT server
        self._ble = ble
        self._ble.config(gap_name=DEVICE_NAME)
        self._handles = self._ble.gatts_register_services(((
            LED_SERVICE_UUID,
            (
                (bluetooth.UUID(LED_CHAR_UUID),
                 bluetooth.FLAG_READ | bluetooth.FLAG_WRITE | bluetooth.FLAG_NOTIFY),
            ),
        ),))
        self._ble.gatts_write(self._handles[0][0], bytes([0]))  # Initial LED state
        self._ble.irq(self._irq)
        self._connections = set()
        self._write_callback = None
        self._payload = None
        
        # Start advertising
        self._advertise()

    def _irq(self, event, data):
        if event == 1: # _IRQ_CENTRAL_CONNECT
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
            print('Client connected')
            
        elif event == 2: # _IRQ_CENTRAL_DISCONNECT
            conn_handle, _, _ = data
            self._connections.remove(conn_handle)
            print('Client disconnected')
            # Start advertising again
            self._advertise()
            
        elif event == 3: # _IRQ_GATTS_WRITE
            conn_handle, value_handle = data
            if conn_handle in self._connections and value_handle == self._handles[0][0]:
                self._payload = self._ble.gatts_read(value_handle)
                if self._write_callback:
                    self._write_callback(self._payload)

    def _advertise(self, interval_us=100000):
        adv_data = bytearray([
            0x02, 0x01, 0x06,  # General discoverable mode
            0x09, 0x09]) + DEVICE_NAME.encode() # Complete local name
        self._ble.gap_advertise(interval_us, adv_data)

    def on_write(self, callback):
        self._write_callback = callback

def led_callback(payload):
    if payload:
        val = payload[0]
        led.value(1 if val else 0)
        print('LED set to:', val)

# Initialize the LED service
led_service = LEDService(ble)
led_service.on_write(led_callback)

print(f'BLE LED Service started, advertising as {DEVICE_NAME}')
print('Connect to control LED')
