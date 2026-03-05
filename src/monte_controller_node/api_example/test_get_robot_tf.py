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
    # robot = Robot("192.168.23.168:50051", "", "")
    robot = Robot("192.168.22.63:50051", "", "")

    print("press any key to get_robot_tf_transform ...")
    get_char()
    parent_id = "link_r0_arm_base"
    success, frame_id, transform = robot.get_robot_tf_transform(parent_id)
    
    if not success:
        print("get_robot_tf_transform failed")
        exit(0)
    
    # 打印每个link的tf
    for i in range(len(frame_id)):
        parent_id = frame_id[i]
        pose = transform[i]
        print(f"{parent_id}: {pose}")


    print("press any key to get_tf_transform ...")
    get_char()
    parent_id = "link_r0_arm_base"
    child_id = "joint_r7_wrist_roll"
    success, transform = robot.get_tf_transform(parent_id, child_id)
    print(f"get_tf_transform success: {success}, transform: {transform}")

    print("press any key to add_tf_frame and remove_tf_frame...")
    get_char()
    target_frame_id = "link_l1_shoulder_pitch_new"
    success = robot.add_tf_frame(target_frame_id, transform, child_id)
    print(f"add_tf_frame success: {success}")

    # remove_tf_frame
    success = robot.remove_tf_frame(target_frame_id)
    print(f"remove_tf_frame success: {success}")

    print("press any key to exit ...")
    get_char()

    exit(0)

    