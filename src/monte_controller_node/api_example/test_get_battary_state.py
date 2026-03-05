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

    for i in range(100):
        success, state = robot.get_battery_status()
        print(f"get_battery_status: {state} , {success}")
        time.sleep(1.0)
    
    
