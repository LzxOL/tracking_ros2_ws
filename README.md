# Monte02 + Track-On 关键点跟踪与机械臂控制

本次使用 **Ubuntu 22.04 + ROS2 Humble** 完成了以下测试：

* **笔记本 + Monte02 真机 + Track-on 关键点跟踪 + 机械臂控制**

---

## 1. 系统概述

本项目实现以下功能：
- 粗定位：点击目标点，机械臂移动到目标点附近
- 关键点跟踪：添加多个关键点，使用 Track-On 算法进行 2D 关键点跟踪
- 3D 坐标转换：将 2D 关键点转换为机械臂基座坐标系下的 3D 坐标
- 机械臂控制：根据跟踪结果控制机械臂运动

**工作流程**：先点击一个粗定位的目标点，机械臂会移动到目标点附近，再添加多个关键点实现关键点跟踪。

详细功能说明参考 [tracking_with_arm_control.sh](./tracking_with_arm_control.sh)。

---

## 2. Monte02 视频流获取（Head + Left / Right hand）

要进行后续测试，需要首先完成 **Monte02 的视频流获取（有线连接）**。

视频流获取方式: 使用 robot_video_client，通过 RTSP 协议获取视频流** （推荐）

RTSP 协议包含 **Server（机器人）** 与 **Client（本机）**，Server 已配置好，只需配置 Client。

robot_video_client 配置可参考 @赵斌斌 的文档，如有疑问可联系他。

---

## 2.1 源码拉取与编译

```bash
# 拉取
mkdir -p 01_ws_robot_video_client/src
cd 01_ws_robot_video_client/src
git clone https://github.com/JackZhaobin/robot_video_client.git
cd robot_video_client/
git checkout feature/ir_rtsp_release_7rtsp
cd ../../

# 编译
colcon build --packages-select robot_video_client
```

> 运行部分可跳转至 **2.3 节**

---

## 2.2 配置文件

### 2.2.1 图像传参配置

打开：

```
robot_video_client/config/video_config.json
```

该文件定义各个摄像头及其 RTSP 视频流地址，默认 IP 为 **192.168.11.2**（有线连接 IP）。

---

### 2.2.2 局域网 DDS 传输配置

编辑：

```
robot_video_client/config/dds_config.json
```

需要注意的是：

* `ethName` 必须与 **本机连接 AGX 的网口名称一致**
* 必须确保本机网段为 **192.168.11.x**

查看网卡名称：

```bash
ifconfig
```

示例配置：

```json
{
  "domain": {
    "id": 66,
    "multicastIp": "224.5.6.7",
    "multicastPort": 8888,
    "period": 3
  },
  "network": {
    "ethName": "eno1"
  },
  "transport": {
    "portRange": "10050-10100",
    "excludePort": [],
    "domainSockRange": "10050-10100",
    "shmKeyRange": "10050-10100"
  },
  "mempool": {
    "maxSize": 50000000
  }
}
```

---

### 2.2.3 运行 Client

#### 1. 添加到组播

```bash
sudo route add -net 224.5.6.7 netmask 255.255.255.255 dev eth0
```

（若需删除组播）

```bash
sudo route del -net 224.5.6.7 netmask 255.255.255.255 dev eth0
```

#### 2. 初始化环境

```bash
source install/setup.bash
```

#### 3. 启动 client

```bash
ros2 launch robot_video_client robot_video_client.launch.py
```

运行后，新开终端查看话题：

```bash
ros2 topic list
```

在 **rviz2** 中订阅视频流，若深度图无法获取说明 DDS 配置有误。

预期话题包括：

```
/camera_head_front/color/video
/camera_head_front/depth/stream
/left/color/video
/left/depth/stream
/left/left_ir/video
/left/right_ir/video
/right/color/video
/right/depth/stream
/right/left_ir/video
/right/right_ir/video
```

---

## 3. Track-On 关键点跟踪

---

## 3.1 环境配置

### 3.1.1 Track-On 推理环境

Track-On 使用 **Python 3.8**，但 ROS2 Humble 使用 **Python 3.10**。
因此运行 ROS 时必须退出 Track-On 的 conda 环境。

模型权重：

```
https://huggingface.co/gaydemir/track_on/resolve/main/track_on_checkpoint.pt?download=true
```

无需参考官方 track_on 仓库的环境配置步骤。

#### 查看系统 Python

```bash
which python3
python3 -V
```

#### 退出所有 conda 环境并确保使用系统 Python

```bash
conda deactivate
export PATH=/usr/bin:$PATH
unset PYTHONPATH PYTHONHOME

which python3
python3 -V
```

#### 安装依赖

```bash
sudo apt-get update
sudo apt-get install -y python3-pip
```

安装 torch 与 mmcv：

```bash
python3 -m pip install --no-cache-dir torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
python3 -m pip install --no-cache-dir mmcv==2.2.0 -f https://download.openmmlab.com/mmcv/dist/cu121/torch2.4/index.html
```

安装 Track-on 依赖：

```bash
cd path/to/trackon
python3 -m pip install --no-cache-dir -r src/track_on/requirements.txt
```

编译：

```bash
cd path/to/work_space/tracking_ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

### 3.1.2 robot_video_client 环境配置

参考 **第二章**。

---

## 3.2 编译与运行

### 3.2.1 编译

```bash
colcon build
source install/setup.bash
```

### 3.2.2 生成相机内外参

> **注意：在运行程序前，需要修改对应的内外参文件。**

本节介绍如何生成和更新相机内外参配置文件。

#### 3.2.2.1 脚本功能

`generate_camera_params.sh` 脚本用于自动生成以下配置文件：

1. **外参文件**：手腕到相机的变换矩阵（TF 变换）
   - `joint_lt_sensor_rgbd_<suffix>.txt`（左手腕到左侧相机）
   - `joint_rt_sensor_rgbd_<suffix>.txt`（右手腕到右侧相机）

2. **内参文件**：相机内参（焦距、主点、畸变系数等）
   - `camera_lhand_front_intrinsics_02_<suffix>.txt`（左侧相机）
   - `camera_rhand_front_intrinsics_02_<suffix>.txt`（右侧相机）

脚本会自动更新以下配置文件：
- `src/monte_controller_node/config/config.yaml`
- `src/track_on_ros2/config/config.yaml`

#### 3.2.2.2 使用方法

**基本用法：**

```bash
./generate_camera_params.sh -r <robot_name>
```

**参数说明：**

| 参数 | 说明 | 示例 |
|------|------|------|
| `-r, --robot-name` | 机器人名称（必填） | `mt029`, `mt0210` |
| `--robot-ip` | 机器人 IP 和端口（可选） | `192.168.11.3:50051` |

**使用示例：**

```bash
# 示例 1：使用 mt029 机器人
./generate_camera_params.sh -r mt029

# 示例 2：指定机器人 IP
./generate_camera_params.sh -r mt0210 --robot-ip 192.168.11.3:50051
```

运行后会提示：
1. 自动获取手腕到相机的外参（TF 变换）
2. 询问是否获取相机内参：
   - 输入目标机器人的 `domain_id` 获取相机内参
   - 直接回车跳过

**注意事项：**

- 机器人名称格式：`mt02<编号>`，如 `mt029` → 后缀 `9`，`mt0210` → 后缀 `10`
- 确保机器人已开机并连接网络
- 获取内参时需要确保机器人端正在发布 camera_info 话题

```bash
./tracking_with_arm_control.sh
```

### 3.2.3 可视化界面（Web UI）

除默认的 OpenCV 可视化窗口外，还提供 **Web 可视化界面**：

#### 环境配置

参考 [Web UI 环境配置](./src/tracking_web_ui/README.md)。

#### 启动 Web UI

```bash
# 1. 先启动跟踪系统
./tracking_with_arm_control.sh

# 2. 新开终端，启动 Web UI
cd src/tracking_web_ui
./run.sh
```

浏览器访问：<http://localhost:8080>

#### 使用方法了网页端上位机

1. 在图像上点击目标位置
2. 点击「执行粗定位」或「设置关键点」
3. 机械臂开始运动

> 更多使用方法详见 [tracking_web_ui/README.md](./src/tracking_web_ui/README.md)

### 3.2.4 使用方法（OpenCV 窗口）

1. 运行上述命令后，会弹出相机可视化窗口
2. 在窗口中**随机点击**一个目标点，作为跟踪点
3. 按下**空格键**，机械臂即会开始运动，距离跟踪点一定距离后会停止

> 注意：必须先点击目标点，再按空格键才会触发机械臂运动。

### 3.2.5 脚本说明

| 脚本名                                 | 功能                                              |
| ----------------------------------- | ----------------------------------------------- |
| `generate_camera_params.sh`         | 生成相机内外参（外参 TF 变换 + 内参从机器人获取）                |
| `tracking_with_arm_control.sh`      | 启动完整的跟踪与机械臂控制系统（视频流 + Tracking + 3D-TF转换 + 机械臂控制） |
| `start_robot_video_stream.sh`       | 单独启动机器人视频流（robot_video_client）                    |

---

## 4. 机械臂控制节点说明

### 4.1 3D-TF 转换节点

`points3d_tf_to_arm_base_node` 负责以下功能：

1. 订阅跟踪节点发布的 2D 关键点话题
2. 获取深度信息，将 2D 关键点转换为 3D 坐标
3. 通过 TF 变换将 3D 坐标从相机坐标系转换到机械臂基座坐标系
4. 发布转换后的 3D 坐标，供机械臂控制使用

---

**Finish on 2025.12.2**
Any question please contact **zhaoxian.lin**

---
