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

def get_char():
    return getch.getch()

if __name__ == '__main__':
    # Inits the robot
    #help(Robot)
    robot = Robot("192.168.11.3:50051", "", "")

    success = robot.set_soft_emergency_stop(True)
    print(f"set_soft_emergency_stop enable: {success}")
    
    time.sleep(0.2)
    print("press any key to continue, get emergency stop state...")
    get_char()
    success, state = robot.get_emergency_stop_state()
    print(f"get_emergency_stop_state: {success}, state: {state}")
    
    time.sleep(0.2)
    print("press any key to continue, set soft emergency stop disable...")
    get_char()
    success = robot.set_soft_emergency_stop(False)
    print(f"set_soft_emergency_stop diable: {success}")
    
    time.sleep(0.2)
    print("press any key to continue, get emergency stop state...")
    get_char()
    success, state = robot.get_emergency_stop_state()
    print(f"get_emergency_stop_state: {success}, state: {state}")

    print("press any key to continue, exit...")
    get_char()
    exit(0)