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

    left_arm_component  = 1
    right_arm_component = 2

    # enbale left and right arm 
    print("press any key to continue, enable left and right arm...")
    get_char()
    success = robot.set_arm_enable(left_arm_component, True)
    print(f"left arm enable success: {success}")
    success = robot.set_arm_enable(right_arm_component, True)
    print(f"right arm enable success: {success}")

    print("press any key to continue, set robot servo angle...")
    get_char()
    success, angles = robot.get_robot_servo_angle()
    print(f"get_robot_servo_angle success: {success}, angles: {angles}")

    print("press any key to continue, set robot servo angle...")
    get_char()
    angles[8] += 0.1
    angles[15] += 0.1
    max_speed = 0.5
    max_acc = 2.0
    success = robot.set_robot_servo_angle(angles, 0.5, 0, 1)
    print(f"set_robot_servo_angle success: {success}")
    success, angles = robot.get_robot_servo_angle()
    print(f"get_robot_servo_angle success: {success}, angles: {angles}")

    print("press any key to continue, set robot servo angle...")
    get_char()
    angles[8] -= 0.1
    angles[15] -= 0.1
    max_speed = 0.5
    max_acc = 2.0
    success = robot.set_robot_servo_angle(angles, 0.5, 0, 1)
    print(f"set_robot_servo_angle success: {success}")
    success, angles = robot.get_robot_servo_angle()
    print(f"get_robot_servo_angle success: {success}, angles: {angles}")

    exit(0)