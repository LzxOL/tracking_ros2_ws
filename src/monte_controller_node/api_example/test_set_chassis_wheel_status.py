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
    # Inits the robot
    #help(Robot)
    robot = Robot("192.168.11.3:50051", "", "")
    
    print("press any key to continue, set chasiss enable to False...")
    get_char()
    success = robot.set_chasiss_enable(False)
    print(f"{success}, set chasiss enable: {success}")

    print("press any key to continue, get chasiss enable...")
    get_char()
    success, state = robot.get_chasiss_enable()
    print(f"{success}, get_chasiss_enable: {state} ")

    print("press any key to continue, set chasiss enable to True...")
    get_char()
    success = robot.set_chasiss_enable(True)
    print(f"set chasiss enable: {success}")

    print("press any key to continue, get chasiss enable...")
    get_char()
    success, state = robot.get_chasiss_enable()
    print(f"get_chasiss_enable: {state} , {success}")   


    print("press any key to continue, get chasiss brake on...")
    get_char()
    success, state = robot.get_chasiss_brake_on()
    print(f"{success}, get_chasiss_brake_on: {state} ")

    print("press any key to continue, set chasiss brake on to True...")
    get_char()
    success = robot.set_chasiss_brake_on(True)
    print(f"set chasiss brake on: {success}")

    print("press any key to continue, get chasiss brake on...")
    get_char()
    success, state = robot.get_chasiss_brake_on()
    print(f"{success}, get_chasiss_brake_on: {state} ")

    print("press any key to continue, set chasiss brake on to False...")
    get_char()
    success = robot.set_chasiss_brake_on(False)
    print(f"set chasiss brake on: {success}")

    exit(0)
    # waring use this 

    print("press any key to continue, set chasiss enable to True...")
    get_char()
    success = robot.set_chasiss_enable(True)
    print(f"set chasiss enable: {success}")

    for i in range(10):
        success, pose, linear_speed, angular_speed = robot.get_chasiss_odom()
        print(f"{linear_speed}, {angular_speed}")
        time.sleep(0.1)
        print("press any key to continue, set chassis velocity to 0.5, 0.5...")
        get_char()
        success = robot.set_chasiss_velocity(0.1, 0.0)
        print(f"set chassis velocity: {success}")


    

