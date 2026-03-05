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
    # Inits the robot
    #help(Robot)
    robot = Robot("192.168.11.3:50051", "", "")

    print("press any key to set led mode...")
    get_char()
    success = robot.set_led_mode(1)
    print(f"set_led_mode success: {success}")

    print("press any key to get led mode...")
    get_char()
    success, mode = robot.get_led_mode()
    print(f"get_led_mode success: {success}, mode: {mode}")

    print("press any key to set head led color...")
    get_char()
    success = robot.set_head_led_color(0, 0, 255, 0)
    print(f"set_head_led_color success: {success}")

    print("press any key to set led mode...")
    get_char()
    success = robot.set_led_mode(0)
    print(f"set_led_mode success: {success}")

    exit(0)
    