import requests

class ArmClient:
    def _fire_and_forget(self, endpoint, params=None):
        """
        Helper to send a GET request to the given endpoint with params in a background thread.
        """
        import threading
        def _send():
            try:
                requests.get(f"{self.base_url}/{endpoint}", params=params, timeout=1)
            except Exception:
                pass
        threading.Thread(target=_send, daemon=True).start()

    def __init__(self, base_url="http://192.168.137.210"):
        self.base_url = base_url
        self.prev_w = None
        self.prev_vx = None  
        self.prev_vy = None
        
    def send_vels(self, w=None, x=None, y=None, force=False):
        """
        Send velocity control commands to the robot arm via /set_vels endpoint.
        """
        if not force and w == self.prev_w and x == self.prev_vx and y == self.prev_vy:
            return None  # No changes, skip sending
        params = {}
        if w is not None:
            params["w"] = w
            self.prev_w = w
        if x is not None:
            params["x"] = x
            self.prev_vx = x
        if y is not None:
            params["y"] = y
            self.prev_vy = y
        self._fire_and_forget("set_vels", params)
        return None

    def send_delta_pos(self, dx=0, dy=0):
        """
        Send incremental (delta) position command to the robot arm via /delta_pos endpoint.
        Resets prev_w, prev_vx, prev_vy to None to avoid stale velocity caching.
        """
        params = {"dx": int(dx), "dy": int(dy)}
        self.prev_w = None
        self.prev_vx = None
        self.prev_vy = None
        self._fire_and_forget("delta_pos", params)
        return None
    
    def set_magnet(self, state=None):
        """
        Control the electromagnet. If state is None, toggles. Accepts True/False, 'on'/'off', 1/0, or 'toggle'.
        Returns the new state ('on' or 'off') or None on error.
        """
        params = {}
        if state is not None:
            if isinstance(state, bool):
                params['state'] = 'on' if state else 'off'
            elif isinstance(state, (int, float)):
                params['state'] = 'on' if state else 'off'
            elif isinstance(state, str):
                params['state'] = state
        self._fire_and_forget("magnet", params)
        return None

    def stop_all(self):
        """Emergency stop - set all motors and velocities to zero"""
        return self.send_vels(w=0, x=0, y=0)
    
    def toggle_led(self):
        """Toggle the onboard LED for connectivity testing (fire-and-forget)"""
        self._fire_and_forget("toggle")
        return None

# Example usage:
if __name__ == "__main__":
    arm = ArmClient(base_url="http://192.168.137.51")
    pass # break here for debug console

    import time
    from input_manager.input_man import is_pressed, rising_edge

    w_mag = 0.12
    xy_mag = 10.0
    print("Controls: q/e=platform left/right, w/s=arm up/down, a/d=arm left/right, x=exit")
    print("Hold keys for continuous movement. Press x to exit.")
    print("Slower control: u/o=platform, i/k=Y, j/l=X")
    try:
        while True:
            boost = 2 if is_pressed('c') else 1

            # Velocity control
            w = (int(is_pressed('q')) - int(is_pressed('e'))) * w_mag * boost
            x = (-int(is_pressed('a')) + int(is_pressed('d'))) * xy_mag * boost
            y = (int(is_pressed('w')) - int(is_pressed('s'))) * xy_mag * boost

            # Delta position control (ijkl, compact)
            dx = (-int(rising_edge('j')) + int(rising_edge('l'))) * boost
            dy = (int(rising_edge('i')) - int(rising_edge('k'))) * boost
            if dx or dy:
                arm.send_delta_pos(dx=dx, dy=dy)

            # Magnet controls using rising_edge
            if rising_edge('m'):
                arm.set_magnet()
            if rising_edge('n'):
                arm.set_magnet(True)
            if rising_edge('b'):
                arm.set_magnet(False)

            if is_pressed('x'):
                break
            arm.send_vels(w=w, x=x, y=y)
            print(f"\r[w={w:.2f} x={x:.2f} y={y:.2f}] > ", end="", flush=True)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        print("\nStopping all...")
        arm.stop_all()
        print("Done.")
