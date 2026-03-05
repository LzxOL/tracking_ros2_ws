import os
import sys
import time
import math
import getch

# autopep8: off
sys.path.append(
    str(os.path.dirname(os.path.abspath(__file__)) + "/lib"))
from RobotLib import Robot
# autopep8: on

def get_char():
    return getch.getch()

if __name__ == '__main__':
    robot = Robot("192.168.22.63:50051", "", "")
    
    left_arm_component  = 1
    right_arm_component = 2

    # enable left and right arm 
    print("press any key to continue, set arm mode to servo motion mode...")
    get_char()
    success = robot.set_arm_enable(left_arm_component, True)
    print(f"left arm enable success: {success}")
    # success = robot.set_able success: {success}")
    
    # set arm mode before moving
    # 1: servo motion mode
    # 2: joint teaching mode
    print("press any key to continue, set arm mode to servo motion mode...")
    get_char()
    success = robot.set_arm_mode(left_arm_component, 1)
    print(f"left arm mode success: {success} mode: {1}")
    # success = robot.set_arm_mode(right_arm_s} mode: {1}")
    
    # move arm with simple position
    print("press any key to continue, move arm with simple position...")
    get_char()
    # positions = [0, 0, 0, 1.57, -1.57, 0, 0]
    positions = [0, 0, 0, 0.1, 0, 0, 0]
    speed = 0.1
    acc = 1
    wait = True
    success = robot.set_arm_servo_angle(left_arm_component, positions, speed, acc, wait)
    print(f"left arm servo angle success: {success}")
    # success = robot.set_arm_angle success: {success}")
    # wait for the motion to complete
    time.sleep(2)
    success, angles = robot.get_arm_servo_angle(left_arm_component)
    print(f"left arm servo angle success: {success}, angles: {angles}")
    # success, angles = robot.gngle success: {success}, angles: {angles}")
    # get arm position in arm frame
    print("press any key to continue, get arm position in arm...")
    get_char()
    success, pose = robot.get_arm_position(left_arm_component)
    print(f"left arm position success: {success}, pose: {pose}")

    # modify z and set arm position (left only)
    print("press any key to continue, set left arm position (z += 0.1) ...")
    get_char()
    pose[2] += 0.1
    success = robot.set_arm_position(left_arm_component, pose, speed, acc, wait)
    print(f"left arm set position success: {success}, pose: {pose}")
    time.sleep(2)
    
    print("press any key to continue, exit...")
    get_char()

    exit(0)

