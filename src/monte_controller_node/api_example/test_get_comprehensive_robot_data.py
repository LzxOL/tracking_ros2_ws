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
    data_types = ["joint_state", "arm_end_pose", "force_sensor_data", "gripper_data", "hand_data", "hand_tactile_data"]
    success, data = robot.get_comprehensive_robot_data(data_types)
    print(f"get comprehensive robot data success: {success}")
    print(f"joint_state: {data['joint_state']}")
    print(f"arm_end_pose: {data['arm_end_pose']}")
    print(f"force_sensor_data: {data['force_sensor_data']}")
    print(f"gripper_data: {data['gripper_data']}")
    print(f"hand_data: {data['hand_data']}")
    print(f"hand_tactile_data: {data['hand_tactile_data']}")

