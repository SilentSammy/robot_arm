import time
import ure
import ujson
import socket
import ntptime
import machine
import network

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
wlan = network.WLAN(network.STA_IF)

def sync_time():
    try:
        ntptime.settime()
        print("Time synced")
    except OSError:
        print("Failed to sync time")

def start_access_point(pwd=""):
    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    essid = 'MyESP_Hotspot'
    
    if pwd:
        # Protected access point with WPA/WPA2
        ap.config(essid=essid, password=pwd, authmode=network.AUTH_WPA_WPA2_PSK)
    else:
        # Open access point (no password)
        ap.config(essid=essid, authmode=network.AUTH_OPEN)
    
    print("Access Point started with IP:", ap.ifconfig()[0])

def connect_wifi(wait=True):
    with open("wifi.txt", "r") as file:
        ssid = file.readline().strip()
        password = file.readline().strip()
    wlan.active(True)
    wlan.connect(ssid, password)

    if wait:
        for i in range(10):
            if wlan.isconnected():
                break
            time.sleep(1)
        if wlan.isconnected():
            print("Connected to", wlan.config('ssid'), "with IP", wlan.ifconfig()[0])
        else:
            print("Failed to connect to", ssid)

def start_webserver(endpoints):
    def handle_client(client_socket):
        # Set a timeout so we don't block indefinitely
        client_socket.settimeout(2)
        while True:
            try:
                data = client_socket.recv(1024)
                if not data:
                    break  # Client closed connection
            except OSError:
                break  # Timeout or socket error

            # Parse the HTTP request
            request = parse_http_request(data.decode())
            
            # Default response (404 Not Found with empty body)
            default_body = ""
            default_response = ("HTTP/1.1 404 Not Found\r\n"
                                "Connection: keep-alive\r\n"
                                "Content-Length: " + str(len(default_body)) + "\r\n\r\n" +
                                default_body)
            response = default_response

            # Check if endpoint exists in our endpoints dict
            endpoint = request['endpoint'].strip('/') if request['endpoint'] else ''
            if endpoint in endpoints:
                result = endpoints[endpoint](request)
                body = ""
                if isinstance(result, dict):
                    body = ujson.dumps(result)
                elif result is not None:
                    body = str(result)
                response = ("HTTP/1.1 200 OK\r\n"
                            "Access-Control-Allow-Origin: *\r\n"
                            "Connection: keep-alive\r\n"
                            "Content-Length: " + str(len(body)) + "\r\n\r\n" +
                            body)
            try:
                client_socket.send(response)
            except Exception:
                break  # Exit if sending fails
        client_socket.close()

    global server_socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('', 80))
    server_socket.listen(5)
    print("Web server started")
    while True:
        try:
            client_socket, addr = server_socket.accept()
            print("Client connected from", addr)
            handle_client(client_socket)
        except Exception as e:
            print("Error accepting connection:", e)

def parse_http_request(http_request):
    # Split request into headers and body
    parts = http_request.split('\n\n', 1)
    headers = parts[0]
    body = parts[1] if len(parts) > 1 else ''

    # Use regex to extract method, endpoint, and parameters
    method_match = ure.search(r'^(\w+)', headers)
    endpoint_match = ure.search(r'^\w+\s+([^?\s]+)', headers)
    params_match = ure.search(r'\?([^?\s]+)\s', headers)

    params_string = params_match.group(1) if params_match else None
    params = {}
    if params_string:
        for param in params_string.split('&'):
            key, value = param.split('=')
            params[key] = value

    result = {
        'method': method_match.group(1) if method_match else None,
        'endpoint': endpoint_match.group(1) if endpoint_match else None,
        'params': params,
        'body': body if body != '' else None
    }
    return result