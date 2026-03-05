
import sys
import os
import time
import getch

# autopep8: off
sys.path.append(
    str(os.path.dirname(os.path.abspath(__file__)) + "/lib"))
from RobotLib import Robot


def get_char():
    return getch.getch()
    #return

def generate_map_name():
    """生成随机地图名称"""
    import random
    import datetime
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    random_num = random.randint(1000, 9999)
    return f"test_map_{timestamp}_{random_num}"

def test_basic_interfaces():
    """测试基本接口"""
    print("=== 地图和导航接口测试 ===")
    
    # 1. 测试获取地图列表
    print("\n1. 获取地图列表...")
    success, map_list = robot.get_map_list()
    if success:
        print(f"成功获取地图列表，共 {len(map_list)} 个地图")
        for i, map_name in enumerate(map_list[:3]):  # 只显示前3个
            print(f"  - {map_name}")
    else:
        print("获取地图列表失败")

def test_mapping_operations(map_name):
    """测试建图操作"""
    print("\n=== 建图操作测试 ===")
    
    # 1. 开始建图
    print(f"\n1. 开始建图 (地图名: {map_name})...")
    success = robot.start_mapping(map_name)
    if success:
        print("建图启动成功")
        time.sleep(2)  # 等待2秒
    else:
        print("建图启动失败")
        
    get_char()
    
    # 2. 检查建图状态
    print("\n2. 检查建图状态...")
    success, mapping_status, mapping_map_name = robot.get_mapping_status()
    if success:
        status_text = "正在建图" if mapping_status else "未在建图"
        print(f"当前建图状态: {status_text}")
        print(f"当前建图地图名称: {mapping_map_name}")
    else:
        print("获取建图状态失败")


    get_char()
    
    # 3. 获取建图位置
    print("\n3. 获取建图位置...")
    success, mapping_pose, is_valid = robot.get_map_chassis_location()
    if success:
        print(f"建图位置: {mapping_pose}")
        print(f"建图位置是否有效: {is_valid}")
    else:
        print("获取建图位置失败")
        
    get_char()

    # 3. 停止建图（保存地图）
    print("\n3. 停止建图并保存地图...")
    success = robot.stop_mapping(save_map=True)
    if success:
        print("建图停止成功")
    else:
        print("建图停止失败")
        
    get_char()
    
    # 4. 再次检查建图状态
    print("\n4. 再次检查建图状态...")
    success, mapping_status, mapping_map_name = robot.get_mapping_status()
    if success:
        status_text = "正在建图" if mapping_status else "未在建图"
        print(f"当前建图状态: {status_text}")
        print(f"当前建图地图名称: {mapping_map_name}")
    else:
        print("获取建图状态失败")
        
    get_char()
    
    # 5. 删除地图
    # print("\n5. 删除地图...")
    # success = robot.remove_map(map_name)
    # if success:
    #     print("地图删除成功")
    # else:
    #     print("地图删除失败")
        
    # get_char()
    
    return is_valid, mapping_pose

def test_navigation_operations(map_name, is_pose_valid, mapping_pose, timeout):
    """测试导航操作"""
    print("\n=== 导航操作测试 ===")
    
    # 1. 启动导航
    print(f"\n1. 启动导航 (使用地图: {map_name})...")
    success = robot.start_navigation(map_name)
    if success:
        print("导航启动成功")
    else:
        print("导航启动失败")
    
    get_char()
    
    # 2. 获取底盘位置
    print("\n2. 获取底盘位置...")
    success, pose, is_valid = robot.get_navi_chassis_location()
    if success and len(pose) == 7:
        print(f"pose: {pose}")
        print(f"is_valid: {is_valid}")
    else:
        print("获取底盘位置失败")

    get_char()
    
    # 3. 导航到目标位置
    print("\n3. 导航到目标位置...")
    # 目标位置：[x, y, z, qw, qx, qy, qz]
    target_pose = mapping_pose  # 向前1米
    print(f"目标位置: {target_pose}")
    timeout = 60.0  # 10秒超时
    
    if is_pose_valid:
        success = robot.navigate_to(target_pose, timeout, navi_mode=True)
        if success:
            print(f"导航命令发送成功，目标位置: {target_pose}")
        else:
            print("导航命令发送失败")
    else:
        print("建图位置无效，无法导航")
        
    get_char()
    
    # 4. 停止导航
    print("\n4. 停止导航...")
    success = robot.stop_navigation()
    if success:
        print("导航停止成功")
    else:
        print("导航停止失败")


def get_navi_chassis_location():
    success, pose, is_valid = robot.get_navi_chassis_location()
    if success and len(pose) == 7:
        print(f"pose: {pose}")
        print(f"is_valid: {is_valid}")
    else:
        print("获取底盘位置失败")

def navigate_to(pose):
    timeout = 10.0
    success = robot.navigate_to(pose, timeout=timeout, navi_mode=True)
    if success:
        print(f"导航命令发送成功，目标位置: {pose}")
    else:
        print("导航命令发送失败")

def stop_navigation():
    success = robot.stop_navigation()
    if success:
        print("导航停止成功")
    else:
        print("导航停止失败")

def main():
    global robot
    
    # 初始化机器人连接
    robot = Robot("192.168.11.3:50051", "", "")
    
    # stop_navigation()
    # exit(0)
    
    map_name = "test_map_20250928_161422_2243"
    is_pose_valid = True
    mapping_pose = [-3.7802529335021973, -0.4622921943664551, 0.0, 0.3712494671344757, 0.0, 0.0, 0.928533136844635]
    # mapping_pose = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
    timeout = 300.0
    
    # 随机生成地图名称
    map_name = generate_map_name()
    map_name = "test_001"
    
    # 测试基本接口
    # test_basic_interfaces()

    # 测试建图操作
    is_pose_valid, mapping_pose = test_mapping_operations(map_name)
    if not is_pose_valid:
        print("建图失败")
    
    # 测试导航操作
    test_navigation_operations(map_name, is_pose_valid, mapping_pose, timeout)
    

if __name__ == "__main__":
    main()


# ros2 service call /navigate_to_pose robot_interface/srv/NavToPose "{pose: {position: {x: 0.9372761249542236, y: 2.1606643199920654, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.4458915591239929, w: 0.8950870037078857}}, use_refine_nav_mode: true}"