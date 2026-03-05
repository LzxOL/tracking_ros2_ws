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
    robot = Robot("192.168.11.3:50051", "", "")
    
    for i in range(100):
        print("press any key to continue, get force sensor data...")
        get_char()
        succ, force = robot.get_force_sensor_data(1)
        print(f"Success: {succ}")
        print(f"left arm force: {force}")
        succ, force = robot.get_force_sensor_data(2)
        print(f"Success: {succ}")
        print(f"right arm force: {force}")
        time.sleep(0.1)