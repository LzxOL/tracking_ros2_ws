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
    # Inits the robot
    #help(Robot)
    robot = Robot("192.168.11.3:50051", "", "")

    print("press any key to continue, get arm position by ik ...")
    get_char()

    left_arm_component = 1

    success, reference_angles = robot.get_arm_servo_angle(left_arm_component)
    print(f"get_arm_servo_angle success: {success}, angles: {reference_angles}")

    success, arm_pose = robot.get_arm_end_pose_by_fk(left_arm_component, reference_angles)
    print(f"get_arm_end_pose_by_fk success: {success}, pose: {arm_pose}")

    arm_pose[2] += 0.001
    success, target_angles = robot.get_arm_joint_state_by_ik(left_arm_component, arm_pose, reference_angles)
    print(f"get_arm_position_by_ik success: {success}, angles: {target_angles}")


    exit(0)