# 跟踪系统 Web 可视化界面

基于 **FastAPI + ROS2** 实现的网页可视化控制界面，用于替代或补充 OpenCV 可视化窗口。

---

## 功能特性

- **实时视频流**：通过订阅 ROS2 话题 `/tracking/visualization` 获取跟踪图像
- **目标点选择**：在网页图像上点击选择目标位置
- **粗定位控制**：点击「执行粗定位」控制机械臂移动到目标点
- **关键点跟踪**：可添加多个关键点并进行跟踪
- **状态显示**：实时显示跟踪状态、关键点坐标、机械臂轨迹

---

## 前置条件

1. 已完成 **tracking_ros2_ws** 工作空间的编译
2. 已安装 Python 依赖

### 安装 Python 依赖

```bash
cd src/tracking_web_ui
pip install -r requirements.txt
```
---

## 启动方式

### 预览模式（无需 ROS2）

用于查看界面布局，视频区显示占位图，API 调用会提示需启动完整服务。

```bash
cd src/tracking_web_ui
./run_preview.sh
# 或
python app.py --preview
```

浏览器访问：<http://localhost:8080>

---

### 完整模式（需先启动跟踪系统）

```bash
# 1. 启动跟踪系统（在终端 1）
cd tracking_ros2_ws
source install/setup.bash
./tracking_with_arm_control.sh

# 2. 启动 Web UI（在终端 2）
cd src/tracking_web_ui
./run.sh
```

浏览器访问：<http://localhost:8080>

---

## 使用流程

1. **启动系统**：按上述步骤启动跟踪系统和 Web UI
2. **选择目标**：在网页图像上点击目标位置
3. **执行粗定位**：点击「执行粗定位」→ 机械臂移向该点
4. **关键点跟踪**：点击添加多个关键点 → 点击「设置关键点」→ 点击「开始跟踪」
5. **重置**：点击「重置」清除状态，可重新选择点

---

## API 接口

| 接口 | 方法 | 说明 |
|------|------|------|
| `/stream` | GET | MJPEG 视频流 |
| `/` | GET | Web 界面首页 |
| `/static/<path>` | GET | 静态资源 |
| `/api/status` | GET | 获取状态（keypoints, coarse_done） |
| `/api/set_keypoints` | POST | 设置跟踪关键点 `{ x: [], y: [] }` |
| `/api/control` | POST | 控制跟踪 `{ command: "start" \| "stop" }` |
| `/api/reset` | POST | 重置状态 |
| `/api/set_coarse_point` | POST | 粗定位 `{ x, y }` |


---

## 常见问题

**Q: 视频流无法显示？**

A: 确保已正确启动 `tracking_with_arm_control.sh`，ROS2 话题 `/tracking/visualization` 正在发布。

**Q: 点击按钮无响应？**

A: 检查浏览器控制台是否有报错，确认后端服务已启动。

**Q: 机械臂无动作？**

A: 确保 ROS2 话题和服务正常，查看后端日志输出。
