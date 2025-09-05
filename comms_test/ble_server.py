import machine
import bluetooth
from micropython import const

# Global BLE setup
DEVICE_NAME = "MicroLED"
_ble = bluetooth.BLE()
_ble.active(True)

# Global state
control_callbacks = {}  # Dictionary mapping control IDs to callback functions
_connections = set()    # Set of connected devices
_handles = {}           # Dictionary mapping control IDs to characteristic handles

def _ble_irq(event, data):
    """Handle BLE events through interrupts"""
    if event == 1:  # _IRQ_CENTRAL_CONNECT
        conn_handle, _, _ = data
        _connections.add(conn_handle)
        print('Client connected')
        
    elif event == 2:  # _IRQ_CENTRAL_DISCONNECT
        conn_handle, _, _ = data
        _connections.remove(conn_handle)
        print('Client disconnected')
        _advertise()
        
    elif event == 3:  # _IRQ_GATTS_WRITE
        conn_handle, value_handle = data
        if conn_handle in _connections:
            # Find which control ID this handle belongs to
            for control_id, handle in _handles.items():
                if handle == value_handle:
                    payload = _ble.gatts_read(value_handle)
                    if payload and control_id in control_callbacks:
                        # Call the corresponding callback with the integer value
                        control_callbacks[control_id](payload[0])
                    break

def _advertise(interval_us=100000):
    """Start advertising the BLE service"""
    adv_data = bytearray([
        0x02, 0x01, 0x06,  # General discoverable mode
        0x09, 0x09]) + DEVICE_NAME.encode()
    _ble.gap_advertise(interval_us, adv_data)

def stop():
    """Stop the BLE server and clean up"""
    # Stop advertising
    _ble.gap_advertise(None)
    
    # Remove IRQ handler
    _ble.irq(None)
    
    # Disconnect any active connections
    for conn_handle in _connections.copy():  # Use copy since we're modifying the set
        _ble.gap_disconnect(conn_handle)
        _connections.remove(conn_handle)
    
    print("BLE Server stopped")

def start():
    """Initialize and start the BLE server"""
    global _handles
    
    # Ensure clean state
    stop()
    
    if not control_callbacks:
        raise ValueError("No callbacks defined!")
        
    _ble.config(gap_name=DEVICE_NAME)
    
    # Create characteristics for each callback
    characteristics = []
    control_ids = sorted(control_callbacks.keys())  # Sort for consistent ordering
    
    for control_id in control_ids:
        uuid = bluetooth.UUID('12345678-1234-5678-1234-56789abcdef' + hex(control_id)[2:])
        characteristics.append(
            (uuid, bluetooth.FLAG_READ | bluetooth.FLAG_WRITE | bluetooth.FLAG_NOTIFY)
        )
    
    # Register all characteristics under one service
    service_handles = _ble.gatts_register_services(((
        bluetooth.UUID('12345678-1234-5678-1234-56789abcdef0'),  # Service UUID
        tuple(characteristics),
    ),))
    
    # Map control IDs to their handles and initialize to 0
    _handles.clear()
    for i, control_id in enumerate(control_ids):
        handle = service_handles[0][i]
        _handles[control_id] = handle
        _ble.gatts_write(handle, bytes([0]))
        
    # Set up event handler and start advertising
    _ble.irq(_ble_irq)
    _advertise()
    
    print(f"BLE Server started with {len(control_callbacks)} controls")
    print(f"Advertising as {DEVICE_NAME}")

# Conversion helpers
def to_norm(value):
    """Convert 0-255 to 0.0-1.0 float"""
    return value / 256.0

def to_bipolar(value):
    """Convert 0-255 to -1.0-1.0 float"""
    return (value - 128) / 128.0

def to_range(value, out_min, out_max):
    """Convert 0-255 to custom float range"""
    return (value * (out_max - out_min) / 255.0) + out_min

def to_char(value):
    """Convert to ASCII character"""
    return chr(value)
