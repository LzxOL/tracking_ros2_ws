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

def test_gripper_moving():
    robot = Robot("192.168.11.3:50051", "", "")
    
    left_arm_component  = 1
    right_arm_component = 2
    # find gripper home 
    # find gripper home must run after disable gripper
    print("press any key to continue, find gripper home...")
    get_char()
    success = robot.set_gripper_enable(left_arm_component, False)
    print(f"left gripper disable success: {success}")
    success = robot.set_gripper_enable(right_arm_component, False)
    print(f"right gripper disable success: {success}")
    time.sleep(0.1)
    success = robot.find_gripper_home(left_arm_component)
    print(f"left gripper find home success: {success}")
    success = robot.find_gripper_home(right_arm_component)
    print(f"right gripper find home success: {success}")
    # find girpper home need 3s to complete
    time.sleep(3.0)
    get_char()

    # enbale left and right gripper 
    print("press any key to continue, enable left and right gripper...")
    get_char()
    success = robot.set_gripper_enable(left_arm_component, True)
    print(f"left gripper enable success: {success}")
    success = robot.set_gripper_enable(right_arm_component, True)
    print(f"right gripper enable success: {success}")

    # test gripper moving
    print("press any key to continue, test gripper moving...")
    get_char()
    success = robot.set_gripper_position(left_arm_component, 0.1)
    print(f"left gripper position success: {success}")
    success = robot.set_gripper_position(right_arm_component, 0.1)
    print(f"right gripper position success: {success}")
    success, position, torque = robot.get_gripper_position(left_arm_component)
    print(f"left gripper position: {position}, torque: {torque}")
    success, position, torque = robot.get_gripper_position(right_arm_component)
    print(f"right gripper position: {position}, torque: {torque}")


    # move gripper to initial position
    print("press any key to continue, move gripper to initial position...")
    get_char()
    success = robot.set_gripper_position(left_arm_component, 0.0)
    print(f"left gripper position success: {success}")
    success = robot.set_gripper_position(right_arm_component, 0.0)
    print(f"right gripper position success: {success}") 
    success, position, torque = robot.get_gripper_position(left_arm_component)
    print(f"left gripper position: {position}, torque: {torque}")
    success, position, torque = robot.get_gripper_position(right_arm_component)
    print(f"right gripper position: {position}, torque: {torque}")

    # set gripper effort
    # effort range: 1.0 N ~ 15.0 N
    print("press any key to continue, set gripper effort...")
    get_char()
    success = robot.set_gripper_effort(left_arm_component, 8.0)
    print(f"left gripper effort success: {success}")
    success = robot.set_gripper_effort(right_arm_component, 8.0)
    print(f"right gripper effort success: {success}")
    success, position, torque = robot.get_gripper_position(left_arm_component)
    print(f"left gripper position: {position}, torque: {torque}")
    success, position, torque = robot.get_gripper_position(right_arm_component)
    print(f"right gripper position: {position}, torque: {torque}")

    print("press any key to continue, test gripper effort...")
    get_char()
    success = robot.set_gripper_effort(left_arm_component, 15.0)
    print(f"left gripper effort success: {success}")
    success = robot.set_gripper_effort(right_arm_component, 15.0)
    print(f"right gripper effort success: {success}")
    success, position, torque = robot.get_gripper_position(left_arm_component)
    print(f"left gripper position: {position}, torque: {torque}")
    success, position, torque = robot.get_gripper_position(right_arm_component)
    print(f"right gripper position: {position}, torque: {torque}")

if __name__ == '__main__':
   test_gripper_moving()
