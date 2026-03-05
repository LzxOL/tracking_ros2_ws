'''
Description: 
Version: V1.0
Author: hongyuan.liu@corenetic.ai
Date: 2025-07-29 14:06:06
LastEditors: hongyuan.liu@corenetic.ai
LastEditTime: 2025-07-29 14:40:16
Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
'''
import os
import sys
import time
import math
import random

# autopep8: off
sys.path.append(
    str(os.path.dirname(os.path.abspath(__file__)) + "/lib"))
from RobotLib import Robot
# autopep8: on

if __name__ == '__main__':
    # Inits the robot
    #help(Robot)
    robot = Robot("192.168.11.3:50051", "", "")
    
    success, version = robot.get_module_version(1)
    print(f"success: {success}, version: {version}")