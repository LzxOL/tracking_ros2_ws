import os
import sys
import time
import math
import random

# autopep8: off
sys.path.append(
    str(os.path.dirname(os.path.abspath(__file__)) + "/lib"))
from RobotLib import Robot
# autopep8: on

def handle_gripper_error(component_type):
    name = "Left Gripper" if component_type == 1 else "Right Gripper"
    # Clear error codes
    robot.clear_gripper_err_code(component_type):
    print(f"[Info] Cleared error codes for {name}")

if __name__ == '__main__':
    # Inits the robot
    robot = Robot("192.168.11.3:50051", "", "")
    
    handle_gripper_error(1)
    handle_gripper_error(2)
