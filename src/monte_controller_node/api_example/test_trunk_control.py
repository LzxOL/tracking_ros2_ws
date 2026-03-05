import os
import sys
import time
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

    # speed range: 0.0 ~ 2.0
    # acc   range: 0.0 ~ 10.0
    # wait  range: True or False
    speed = 0.5
    acc   = 1
    wait  = False
    trunk_ids = [1, 2, 3]

    success = robot.set_trunk_joint_enable(True)
    print(f"set trunk joint enable success: {success}")

    # set_trunk_enable will sleep 1s
    time.sleep(1)
    
    # get_trunk_position
    success, position = robot.get_trunk_joint_position(trunk_ids)
    print(f"success: {success}, positon: {position}")
    
    # set_trunk_position
    print("press any key to continue, get trunk position...")
    get_char()
    position[2] = position[2] + 0.15
    success = robot.set_trunk_servo_angle(trunk_ids, position, speed, acc, wait)
    print(f"set_trunk_servo_angle success: {success}")

    # wait for the motion to complete
    time.sleep(2)
    success, position = robot.get_trunk_joint_position(trunk_ids)
    print(f"success: {success}, positon: {position}")

    print("press any key to continue, get trunk position...")
    get_char()
    position[2] = position[2] - 0.15
    success = robot.set_trunk_servo_angle(trunk_ids, position, speed, acc, wait)
    print(f"set_trunk_servo_angle success: {success}")  

    # wait for the motion to complete
    time.sleep(2)
    success, position = robot.get_trunk_joint_position(trunk_ids)
    print(f"success: {success}, positon: {position}")

    print("press any key to continue, disable trunk joint...")
    get_char()
    exit(0)