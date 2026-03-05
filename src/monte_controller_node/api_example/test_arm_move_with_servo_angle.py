
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

def test_arm_moving():
    robot = Robot("192.168.11.3:50051", "", "")
    
    left_arm_component  = 1
    right_arm_component = 2

    # enbale left and right arm 
    print("press any key to continue, set arm enable to True...")
    get_char()
    success = robot.set_arm_enable(left_arm_component, True)
    print(f"left arm enable success: {success}")
    success = robot.set_arm_enable(right_arm_component, True)
    print(f"right arm enable success: {success}")
    
    # set arm mode before moving
    # 1: servo motion mode
    # 2: joint teaching mode
    print("press any key to continue, set arm mode to servo motion mode...")
    get_char()
    success = robot.set_arm_mode(left_arm_component, 1)
    print(f"left arm mode success: {success} mode: {1}")
    success = robot.set_arm_mode(right_arm_component, 1)
    print(f"right arm mode success: {success} mode: {1}")
    
    # test arm moving
    # speed  : 1 m/s, range: 0.0 - 1.0
    # acc    : 5 m/s^2, range: 0.0 - 10.0 , if acc is 0 speed * 10 
    # wait   : True or False , True will wait for the motion to complete
    # Warning: joint 4 zero is mapped to 6 degrees

    # move arm to initial position
    print("press any key to continue, move arm to initial position...")
    get_char()
    positions = [0, 0, 0, 6.0 * math.pi / 180, 0, 0, 0]
    speed = 0.5
    acc = 1
    wait = True
    success = robot.set_arm_servo_angle(left_arm_component, positions, speed, acc, wait)
    print(f"left arm servo angle success: {success}")
    success = robot.set_arm_servo_angle(right_arm_component, positions, speed, acc, wait)
    print(f"right arm servo angle success: {success}")
    # wait for the motion to complete
    time.sleep(2)
    success, angles = robot.get_arm_servo_angle(left_arm_component)
    print(f"left arm servo angle success: {success}, angles: {angles}")
    success, angles = robot.get_arm_servo_angle(right_arm_component)
    print(f"right arm servo angle success: {success}, angles: {angles}")


    # move arm to front position
    print("press any key to continue, move arm to front position...")
    get_char()
    positions = [0.30, 0.20, 0, 0.45, 0, 0, 0]
    speed = 0.5
    acc = 1
    wait = True
    success = robot.set_arm_servo_angle(left_arm_component, positions, speed, acc, wait)
    print(f"left arm servo angle success: {success}")
    success = robot.set_arm_servo_angle(right_arm_component, positions, speed, acc, wait)
    print(f"right arm servo angle success: {success}")
    # wait for the motion to complete
    time.sleep(2)
    success, angles = robot.get_arm_servo_angle(left_arm_component)
    print(f"left arm servo angle success: {success}, angles: {angles}")
    success, angles = robot.get_arm_servo_angle(right_arm_component)
    print(f"right arm servo angle success: {success}, angles: {angles}")

    # move arm to initial position
    print("press any key to continue, move arm to initial position...")
    get_char()
    positions = [0, 0, 0, 6.0 * math.pi / 180, 0, 0, 0]
    speed = 0.5
    acc = 1
    wait = True
    success = robot.set_arm_servo_angle(left_arm_component, positions, speed, acc, wait)
    print(f"left arm servo angle success: {success}")
    success = robot.set_arm_servo_angle(right_arm_component, positions, speed, acc, wait)
    print(f"right arm servo angle success: {success}")
    time.sleep(2)
    success, angles = robot.get_arm_servo_angle(left_arm_component)
    print(f"left arm servo angle success: {success}, angles: {angles}")
    success, angles = robot.get_arm_servo_angle(right_arm_component)
    print(f"right arm servo angle success: {success}, angles: {angles}")

if __name__ == '__main__':
   test_arm_moving()




    