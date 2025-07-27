import time
from input_manager.input_man import is_pressed
from robot_arm import RobotArm

def joint_velocity_input(velocity_scale=20):
    # Joint 0 controls
    joint0_vel = int(is_pressed("i")) - int(is_pressed("k"))
    
    # Joint 1 controls
    joint1_vel = int(is_pressed("o")) - int(is_pressed("l"))
    
    # Apply velocity scaling
    scaled_vel0 = joint0_vel * velocity_scale
    scaled_vel1 = joint1_vel * velocity_scale
    
    return scaled_vel0, scaled_vel1

# Main control loop
with RobotArm(port="COM10", on_message=lambda msg: print(f"Arm: {msg}")) as arm:
    try:
        while True:
            # Get velocity inputs
            vel0, vel1 = joint_velocity_input()
            
            # Send velocity commands to both joints atomically
            arm.set_velocity({0: vel0, 1: vel1})
            
            time.sleep(0.02)  # 50Hz update rate
            
    except KeyboardInterrupt:
        print("\nStopping robot arm...")
        arm.stop()  # Stop all joints before exiting
        print("Exited.")
