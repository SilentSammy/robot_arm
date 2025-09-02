import socket

ESP32_IP = '192.168.137.122'  # Change to your ESP32's IP address
PORT = 12345

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((ESP32_IP, PORT))
    print(f"Connected to {ESP32_IP}:{PORT}")
    while True:
        cmd = input('Enter command (or "quit" to exit): ')
        if cmd.strip().lower() == 'quit':
            break
        s.sendall((cmd + '\n').encode())
        resp = s.recv(1024)
        print('Response:', resp.decode().strip())
print('Disconnected.')
