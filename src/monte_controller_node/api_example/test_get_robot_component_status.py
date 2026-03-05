import os
import sys
import time
import math
import getch
import random

# autopep8: off
sys.path.append(
    str(os.path.dirname(os.path.abspath(__file__)) + "/lib"))
from RobotLib import Robot
# autopep8: on

def get_char():
    return getch.getch()

if __name__ == '__main__':
    robot = Robot("192.168.11.3:50051", "", "")
    success, status = robot.get_robot_component_status()
    print(f"get robot component status success: {success}")
    print(f"trunk_enabled: {status[0]}")
    print(f"head_enabled: {status[1]}")
    print(f"left_arm_enabled: {status[2]}")
    print(f"right_arm_enabled: {status[3]}")
    print(f"chassis_enabled: {status[4]}")
    print(f"chassis_break_on: {status[5]}")