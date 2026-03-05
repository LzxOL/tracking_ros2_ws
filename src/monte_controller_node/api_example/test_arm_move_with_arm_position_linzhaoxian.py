import os
import sys
import time
import math
import getch

# autopep8: off
sys.path.append(
    str(os.path.dirname(os.path.abspath(__file__)) + "/lib"))
from RobotLib import Robot
# autopep8: on

def get_char():
    return getch.getch()

if __name__ == '__main__':
    # 与 test_arm_move_simple_linzhaoxian.py 保持一致的地址
    robot = Robot("192.168.22.63:50051", "", "")

    left_arm_component = 1
    right_arm_component = 2
    # 仅启用左臂并设置为伺服模式
    print("press any key to continue, enable left arm and set servo motion mode...")
    get_char()
    success = robot.set_arm_enable(right_arm_component, True)
    print(f"left arm enable success: {success}")
    success = robot.set_arm_mode(right_arm_component, 1)
    print(f"left arm mode success: {success} mode: {1}")

    # 可选：先到一个简单安全位姿
    print("press any key to continue, move left arm to a simple safe pose...")
    get_char()
    positions = [0, 0, 0, 1.57, 0, 0, 0]
    speed = 0.1
    acc = 1
    wait = True
    success = robot.set_arm_servo_angle(right_arm_component, positions, speed, acc, wait)
    print(f"left arm servo angle success: {success}")
    time.sleep(1.5)

    # ===== 对应 test_arm_move_with_arm_position.py 84-98 的左臂版 =====
    # 获取左臂在 arm 坐标系下的末端位姿
    print("press any key to continue, get left arm position in arm...")
    get_char()
    success, left_arm_pose = robot.get_arm_position(right_arm_component)
    print(f"left arm position success: {success}, pose: {left_arm_pose}")

    # # 修改 z 分量并下发位姿
    # print("press any key to continue, set left arm position (z += 0.1)...")
    # get_char()
    # left_arm_pose[2] = 0
    # success = robot.set_arm_position(right_arm_component, left_arm_pose, speed, acc, wait)
    # print(f"left arm set position success: {success}, pose: {left_arm_pose}")
    # time.sleep(1.5)

    # # 再读一次进行确认
    # print("press any key to continue, get left arm position again...")
    # get_char()
    # success, left_arm_pose = robot.get_arm_position(right_arm_component)
    # print(f"left arm position success: {success}, pose: {left_arm_pose}")

    # print("press any key to continue, exit...")
    # get_char()
    exit(0)

