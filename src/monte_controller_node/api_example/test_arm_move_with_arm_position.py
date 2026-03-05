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
    robot = Robot("192.168.22.49:50051", "", "")
    
    left_arm_component  = 1
    right_arm_component = 2

    # enbale left and right arm 
    print("press any key to continue, set arm mode to servo motion mode...")
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
    
    # 先使用servo angle移动到一个安全的位置
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

    #set arm position
    speed = 0.5
    acc = 1
    wait = True

    #get arm position
    print("press any key to continue, get arm position in arm...")
    get_char()
    success, left_arm_pose = robot.get_arm_position(left_arm_component)
    print(f"left arm position success: {success}, pose: {left_arm_pose}")
    success, right_arm_pose = robot.get_arm_position(right_arm_component)
    print(f"right arm position success: {success}, pose: {right_arm_pose}")

    left_arm_pose[2] += 0.1
    right_arm_pose[2] += 0.1
    success = robot.set_arm_position(1, left_arm_pose, speed, acc, wait)
    print(f"left arm position success: {success}, pose: {left_arm_pose}")
    success = robot.set_arm_position(2, right_arm_pose, speed, acc, wait)
    print(f"right arm position success: {success}, pose: {right_arm_pose}")

    #get arm position in base
    print("press any key to continue, get arm position in arm...")
    get_char()
    success, left_arm_pose = robot.get_arm_position(1)
    print(f"left arm position success: {success}, pose: {left_arm_pose}")
    success, right_arm_pose = robot.get_arm_position(2)
    print(f"right arm position success: {success}, pose: {right_arm_pose}")

    # set arm position in base
    print("press any key to continue, set arm position in base...")
    get_char()
    success, left_arm_pose = robot.get_arm_position_in_base(1)
    print(f"left arm position success: {success}, pose: {left_arm_pose}")
    success, right_arm_pose = robot.get_arm_position_in_base(2)
    print(f"right arm position success: {success}, pose: {right_arm_pose}")
    
    
    left_arm_pose[2] += 0.1
    right_arm_pose[2] += 0.1
    success = robot.set_arm_position_in_base(1, left_arm_pose, speed, acc, wait)
    print(f"left arm position success: {success}, pose: {left_arm_pose}")
    success = robot.set_arm_position_in_base(2, right_arm_pose, speed, acc, wait)
    print(f"right arm position success: {success}, pose: {right_arm_pose}")

    #get arm position in base
    print("press any key to continue, get arm position in base...")
    get_char()
    success, left_arm_pose = robot.get_arm_position_in_base(1)
    print(f"left arm position success: {success}, pose: {left_arm_pose}")
    success, right_arm_pose = robot.get_arm_position_in_base(2)
    print(f"right arm position success: {success}, pose: {right_arm_pose}")


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
    
    print("press any key to continue, exit...")
    get_char()

    exit(0)


    
    
    