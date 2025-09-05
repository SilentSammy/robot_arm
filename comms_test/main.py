import machine
import bluetooth
from micropython import const

# Global BLE setup
DEVICE_NAME = "MicroLED"
ble = bluetooth.BLE()
ble.active(True)

# Global state
control_callbacks = []  # List of callback functions
connections = set()     # Set of connected devices
handles = []           # Characteristic handles

def _ble_irq(event, data):
    """Handle BLE events through interrupts"""
    if event == 1:  # _IRQ_CENTRAL_CONNECT
        conn_handle, _, _ = data
        connections.add(conn_handle)
        print('Client connected')
        
    elif event == 2:  # _IRQ_CENTRAL_DISCONNECT
        conn_handle, _, _ = data
        connections.remove(conn_handle)
        print('Client disconnected')
        _advertise()
        
    elif event == 3:  # _IRQ_GATTS_WRITE
        conn_handle, value_handle = data
        if conn_handle in connections:
            try:
                # Find which characteristic was written to
                char_index = handles[0].index(value_handle)
                payload = ble.gatts_read(value_handle)
                if payload:
                    # Call the corresponding callback with the integer value
                    control_callbacks[char_index](payload[0])
            except ValueError:
                pass  # Handle not found

def _advertise(interval_us=100000):
    """Start advertising the BLE service"""
    adv_data = bytearray([
        0x02, 0x01, 0x06,  # General discoverable mode
        0x09, 0x09]) + DEVICE_NAME.encode()
    ble.gap_advertise(interval_us, adv_data)

def stop():
    """Stop the BLE server and clean up"""
    # Stop advertising
    ble.gap_advertise(None)
    
    # Remove IRQ handler
    ble.irq(None)
    
    # Disconnect any active connections
    for conn_handle in connections.copy():  # Use copy since we're modifying the set
        ble.gap_disconnect(conn_handle)
        connections.remove(conn_handle)
    
    print("BLE Server stopped")

def start():
    """Initialize and start the BLE server"""
    global handles
    
    # Ensure clean state
    stop()
    
    if not control_callbacks:
        raise ValueError("No callbacks defined!")
        
    ble.config(gap_name=DEVICE_NAME)
    
    # Create characteristics for each callback
    characteristics = []
    for i in range(len(control_callbacks)):
        uuid = bluetooth.UUID('12345678-1234-5678-1234-56789abcdef' + str(i))
        characteristics.append(
            (uuid, bluetooth.FLAG_READ | bluetooth.FLAG_WRITE | bluetooth.FLAG_NOTIFY)
        )
    
    # Register all characteristics under one service
    handles = ble.gatts_register_services(((
        bluetooth.UUID('12345678-1234-5678-1234-56789abcdef0'),  # Service UUID
        tuple(characteristics),
    ),))
    
    # Initialize all characteristics to 0
    for handle in handles[0]:
        ble.gatts_write(handle, bytes([0]))
        
    # Set up event handler and start advertising
    ble.irq(_ble_irq)
    _advertise()
    
    print(f"BLE Server started with {len(control_callbacks)} controls")
    print(f"Advertising as {DEVICE_NAME}")

# Example usage:
led = machine.Pin(2, machine.Pin.OUT)

def set_led(value):
    led.value(1 if value else 0)
    print('LED set to:', value)

# Add the callback to the list
control_callbacks.append(set_led)

# Start the server
start()