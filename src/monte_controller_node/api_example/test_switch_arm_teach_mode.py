
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

def test_switch_arm_teach_mode():
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
    
    # set arm mode before moving
    # 1: servo motion mode
    # 2: joint teaching mode
    print("press any key to continue, set arm mode to joint teaching mode...")
    get_char()
    success = robot.set_arm_mode(left_arm_component, 2)
    print(f"left arm mode success: {success} mode: {2}")
    success = robot.set_arm_mode(right_arm_component, 2)
    print(f"right arm mode success: {success} mode: {2}")
    
    print("press any key to continue, get arm servo angle...")
    get_char()
    for i in range(100):
        success, angles = robot.get_arm_servo_angle(left_arm_component)
        print(f"left arm servo angle success: {success}, angles: {angles}")
        success, angles = robot.get_arm_servo_angle(right_arm_component)
        print(f"right arm servo angle success: {success}, angles: {angles}")
        time.sleep(0.1)

if __name__ == '__main__':
   test_switch_arm_teach_mode()




    