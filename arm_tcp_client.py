import socket
import threading

class ArmTCPClient:
	def __init__(self, host="192.168.137.210", port=12345):
		self.host = host
		self.port = port
		self.prev_vx = None
		self.prev_vy = None
		self.prev_w = None

	def _send_tcp(self, msg):
		"""
		Send a raw TCP command string to the robot arm server synchronously.
		No threading; minimizes overhead for real-time control.
		"""
		try:
			with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
				s.settimeout(1)
				s.connect((self.host, self.port))
				s.sendall((msg + '\n').encode())
				# Optionally read response (not required)
				try:
					s.recv(1024)
				except Exception:
					pass
		except Exception:
			pass

	def send_vel(self, vx=None, vy=None, w=None, force=False):
		"""
		Send velocity command: VEL: vx,vy,w (any can be None to skip)
		Only sends if value changed, unless force=True.
		"""
		if not force and vx == self.prev_vx and vy == self.prev_vy and w == self.prev_w:
			return
		args = []
		for v in (vx, vy, w):
			args.append(str(v) if v is not None else '')
		cmd = f"VEL: {','.join(args)}"
		self._send_tcp(cmd)
		self.prev_vx = vx
		self.prev_vy = vy
		self.prev_w = w

	def send_dp(self, dx=0, dy=0, dt=0):
		"""
		Send delta position command: DP: dx,dy,dt
		Resets prev_vx, prev_vy, prev_w to None to avoid stale velocity caching.
		"""
		args = [str(dx), str(dy), str(dt)]
		cmd = f"DP: {','.join(args)}"
		# self.prev_vx = None
		# self.prev_vy = None
		# self.prev_w = None
		self._send_tcp(cmd)

	def set_magnet(self, state=None):
		"""
		MAG: 1 (on), 0 (off), toggle if None
		"""
		if state is None:
			cmd = "MAG:"
		else:
			cmd = f"MAG: {int(bool(state))}"
		self._send_tcp(cmd)

	def stop_all(self):
		self.send_vel(0, 0, 0, force=True)

	def toggle_led(self):
		self._send_tcp("LED:")

# Interactive keyboard control (adapted from HTTP client)
if __name__ == "__main__":
	import time
	from input_manager.input_man import is_pressed, rising_edge

	arm = ArmTCPClient(host="192.168.137.141")
	w_mag = 45.0
	vxy_mag = 10.0
	xy_delta = 1.0
	theta_delta = 5.0
	print("Controls: q/e=platform left/right, w/s=arm up/down, a/d=arm left/right, x=exit")
	print("Hold keys for continuous movement. Press x to exit.")
	print("Slower control: u/o=platform, i/k=Y, j/l=X")
	try:
		while True:
			boost = 2 if is_pressed('c') else 1

			# Velocity control
			w = (int(is_pressed('q')) - int(is_pressed('e'))) * w_mag * boost
			x = (-int(is_pressed('a')) + int(is_pressed('d'))) * vxy_mag * boost
			y = (int(is_pressed('w')) - int(is_pressed('s'))) * vxy_mag * boost

			# Delta position control (ijkl, compact) and U/P for dtheta
			dx = (-int(rising_edge('j')) + int(rising_edge('l'))) * boost * xy_delta
			dy = (int(rising_edge('i')) - int(rising_edge('k'))) * boost * xy_delta
			dtheta = (-int(rising_edge('u')) + int(rising_edge('o'))) * boost * theta_delta
			if dx or dy or dtheta:
				arm.send_dp(dx=dx, dy=dy, dt=dtheta)

			# Magnet controls using rising_edge
			if rising_edge('m'):
				arm.set_magnet()
			if rising_edge('n'):
				arm.set_magnet(True)
			if rising_edge('b'):
				arm.set_magnet(False)

			# Led control (for debugging)
			if rising_edge('x'):
				arm.toggle_led()
			arm.send_vel(vx=x, vy=y, w=w)
			print(f"\r[w={w:.2f} x={x:.2f} y={y:.2f}] > ", end="", flush=True)
	except KeyboardInterrupt:
		print("\nExiting...")
	finally:
		print("\nStopping all...")
		arm.stop_all()
		print("Done.")
