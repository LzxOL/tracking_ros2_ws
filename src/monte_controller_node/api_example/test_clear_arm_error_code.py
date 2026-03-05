import os
import sys
import time
import math
import random
import getch

# autopep8: off
sys.path.append(
    str(os.path.dirname(os.path.abspath(__file__)) + "/lib"))
from RobotLib import Robot
# autopep8: on

def get_char():
    return getch.getch()

def handle_arm_errors(component_type):
    name = "Left Arm" if component_type == 1 else "Right Arm"

    # Get error codes
    success, error_codes = robot.get_arm_err_warn_code(component_type)
    if success:
        if error_codes:
            print(f"[Info] {name} error codes: {error_codes}")
        else:
            print(f"[Info] {name} has no error codes.")
    else:
        print(f"[Error] Failed to get error codes for {name}")
        return
    
    # print("press any key to continue, clean arm error code...")
    # get_char()

    # # Clear error codes
    # if robot.clear_arm_err_warn_code(component_type):
    #     print(f"[Info] Cleared error codes for {name}")
    # else:
    #     print(f"[Error] Failed to clear error codes for {name}")


if __name__ == '__main__':
    robot = Robot("192.168.11.3:50051", "", "")
    handle_arm_errors(1)
    handle_arm_errors(2)
    
    