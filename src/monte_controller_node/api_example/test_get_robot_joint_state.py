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

if __name__ == '__main__':
    robot = Robot("192.168.11.3:50051", "", "")

    # get robot joint state, over 19 joints
    # 1-3   is trunk
    # 4-5   is head
    # 6-12  is left arm
    # 13-19 is right arm
    for i in range(5):
        print(f"press any key to continue, get robot joint state...")
        get_char()
        success, pos, vel, acc, tor = robot.get_robot_joint_state()
        print(f"get robot joint state success: {success}")
        print(f"position: {pos}")
        print(f"velocity: {vel}")
        print(f"acceleration: {acc}")
        print(f"torque: {tor}")
        time.sleep(0.1)

    # 1-7: left arm
    for i in range(5):
        print(f"press any key to continue, get left arm joint state...")
        get_char()
        success, pos, vel, acc, tor = robot.get_arm_joint_state(1)
        print(f"get left arm joint state success: {success}")
        print(f"position: {pos}")
        print(f"velocity: {vel}")
        print(f"acceleration: {acc}")
        print(f"torque: {tor}")
        time.sleep(0.1)

    # 1-5: body
    for i in range(5):
        print(f"press any key to continue, get body joint state...")
        get_char()
        success, pos, vel, acc, tor = robot.get_body_joint_state()
        print(f"get body joint state success: {success}")
        print(f"position: {pos}")
        print(f"velocity: {vel}")
        print(f"acceleration: {acc}")
        print(f"torque: {tor}")
        time.sleep(0.1)

    # get any joint state [1, 2] or [7, 8, 9]
    for i in range(5):
        print(f"press any key to continue, get any joint state...")
        get_char()
        success, pos, vel, acc, tor = robot.get_joint_state([i + 1])
        print(f"get robot joint state success: {success}")
        print(f"position: {pos}")
        print(f"velocity: {vel}")
        print(f"acceleration: {acc}")
        print(f"torque: {tor}")
        time.sleep(0.1)
    exit(0)
    