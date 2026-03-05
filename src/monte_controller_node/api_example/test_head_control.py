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
    #return

if __name__ == '__main__':
    robot = Robot("192.168.11.3:50051", "", "")
    
    robot.set_head_joint_enable(True)
    time.sleep(0.2)

    id = [1, 2]
    print("press any key to continue, set head position to [0.0, 0.0]...")
    get_char()
    position = [0.0, 0.0]
    robot.set_head_joint_position(id, position)

    print("press any key to continue, set head position to [0.2, 0.5]...")
    get_char()
    position = [0.2, 0.5]
    robot.set_head_joint_position(id, position)

    
    print("press any key to continue, set head position to [0.0, 0.0]...")
    get_char()
    position = [0.0, 0.0]
    robot.set_head_joint_position(id, position)

    print("press any key to continue, exit...")
    get_char()
    exit(0)