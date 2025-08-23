import requests

class ArmClient:
    def __init__(self, base_url="http://192.168.137.210"):
        self.base_url = base_url
        self.prev_w = None
        self.prev_x = None  
        self.prev_y = None
        
    def send(self, w=None, x=None, y=None, force=False):
        """
        Send control commands to the robot arm.
        
        Args:
            w: Motor power (-1 to 1) for platform rotation
            x: Arm X velocity for cartesian movement
            y: Arm Y velocity for cartesian movement  
            force: Skip duplicate checking, always send
            
        Returns:
            JSON response from the robot, or None on error
        """
        # Check if any values have changed (unless forced)
        if not force and w == self.prev_w and x == self.prev_x and y == self.prev_y:
            return None  # No changes, skip sending
            
        # Build parameters dict
        params = {}
        if w is not None:
            params["w"] = w
            self.prev_w = w
        if x is not None:
            params["x"] = x
            self.prev_x = x
        if y is not None:
            params["y"] = y
            self.prev_y = y
            
        # Send request
        try:
            response = requests.get(f"{self.base_url}/control", params=params)
            if response.status_code == 200:
                return response.json()
            else:
                print(f"HTTP Error {response.status_code}: {response.text}")
                return None
        except Exception as ex:
            print(f"Error sending command: {ex}")
            return None
    
    def stop_all(self):
        """Emergency stop - set all motors and velocities to zero"""
        return self.send(w=0, x=0, y=0)
    
    def toggle_led(self):
        """Toggle the onboard LED for connectivity testing"""
        try:
            response = requests.get(f"{self.base_url}/toggle")
            if response.status_code == 200:
                return response.json()
            else:
                print(f"HTTP Error {response.status_code}: {response.text}")
                return None
        except Exception as ex:
            print(f"Error toggling LED: {ex}")
            return None

# Example usage:
if __name__ == "__main__":
    arm = ArmClient()
    pass # break here for debug console

    import time
    from input_manager.input_man import is_pressed

    w_mag = 0.15
    xy_mag = 20.0
    slow_w_mag = 0.1
    slow_xy_mag = 7
    print("Controls: q/e=platform left/right, w/s=arm up/down, a/d=arm left/right, x=exit")
    print("Hold keys for continuous movement. Press x to exit.")
    print("Slower control: u/o=platform, i/k=Y, j/l=X")
    try:
        while True:
            # Fast/normal keys
            w = (int(is_pressed('q')) - int(is_pressed('e'))) * w_mag
            x = (-int(is_pressed('a')) + int(is_pressed('d'))) * xy_mag
            y = (int(is_pressed('w')) - int(is_pressed('s'))) * xy_mag

            # Slower keys override if pressed
            if is_pressed('u') or is_pressed('o'):
                w = (int(is_pressed('u')) - int(is_pressed('o'))) * slow_w_mag
            if is_pressed('j') or is_pressed('l'):
                x = (-int(is_pressed('j')) + int(is_pressed('l'))) * slow_xy_mag
            if is_pressed('i') or is_pressed('k'):
                y = (int(is_pressed('i')) - int(is_pressed('k'))) * slow_xy_mag

            if is_pressed('x'):
                break
            arm.send(w=w, x=x, y=y)
            print(f"\r[w={w:.2f} x={x:.2f} y={y:.2f}] > ", end="", flush=True)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        print("\nStopping all...")
        arm.stop_all()
        print("Done.")
