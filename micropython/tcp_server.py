import socket
import sys
import network
import time

def parse_command_line(line):
	"""
	Parse a command line in the format CMD:arg1,arg2,...
	Returns (cmd, args) where cmd is uppercase and args is a list of strings.
	"""
	line = line.strip()
	colon_idx = line.find(':')
	if colon_idx < 0:
		cmd = line.upper()
		args = []
	elif colon_idx == 0:
		return None, None  # Invalid command
	else:
		cmd = line[:colon_idx].strip().upper()
		arg_str = line[colon_idx+1:].strip()
		if not arg_str:
			args = []
		else:
			args = [a.strip() for a in arg_str.split(',') if a.strip()]
	return cmd, args

def execute_command(cmd, args, command_map):
	"""
	Executes the command using the command_map. Returns response string.
	"""
	if cmd is None:
		return 'ERR: Invalid command'
	handler = command_map.get(cmd)
	if handler:
		try:
			resp = handler(args)
			if resp is None:
				resp = 'OK'
			return str(resp)
		except Exception as e:
			return 'ERR: ' + str(e)
	else:
		return 'UNKNOWN CMD'

def connect_wifi(ssid, password, wait=True):
	"""
	Connect to WiFi using provided ssid and password. Optionally wait for connection.
	"""
	wlan = network.WLAN(network.STA_IF)
	wlan.active(True)
	wlan.connect(ssid, password)
	if wait:
		for _ in range(20):
			if wlan.isconnected():
				break
			time.sleep(0.5)
		if wlan.isconnected():
			print("Connected to", ssid, "with IP", wlan.ifconfig()[0])
		else:
			print("Failed to connect to", ssid)
	return wlan

def connect_to_wifi_from_file(config_file="wifi.txt"):
	"""
	Connect to WiFi using credentials from a config file.
	"""
	with open(config_file, "r") as f:
		ssid = f.readline().strip()
		password = f.readline().strip()
	return connect_wifi(ssid, password)

def start_tcp_server(command_map, port=12345, led=None):
	"""
	Starts a TCP server. command_map: dict of command string -> handler(list_of_str_args)
	Each handler is called as handler(args: list of str). Handler should return a string (response).
	Optionally pass led=Pin instance for demo/test.
	Command format: CMD:arg1,arg2,... (whitespace ignored, case-insensitive)
	"""
	HOST = ''
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	s.bind((HOST, port))
	s.listen(1)
	print('TCP server listening on port', port)
	while True:
		print('Waiting for connection...')
		conn, addr = s.accept()
		print('Client connected from', addr)
		try:
			while True:
				data = conn.recv(1024)
				if not data:
					break
				line = data.decode().strip()
				print('Received:', line)
				if not line:
					conn.send(b'EMPTY\n')
					continue
				cmd, args = parse_command_line(line)
				resp = execute_command(cmd, args, command_map)
				conn.send((resp + '\n').encode())
		except Exception as e:
			print('Connection error:', e)
		finally:
			conn.close()
			print('Client disconnected')
