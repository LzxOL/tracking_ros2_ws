#!/usr/bin/env python3
"""
跟踪系统 Web 上位机 - FastAPI + ROS2 桥接 (UI Enhanced)

用法：
  预览模式（无需 ROS2）：  python app.py --preview
  完整模式：              source install/setup.bash && python app.py
"""

import os
import sys
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path

import numpy as np
import cv2
from fastapi import FastAPI, HTTPException
from fastapi.responses import StreamingResponse, HTMLResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# ================= 配置与路径 =================
STATIC_DIR = Path(__file__).parent / "static"
PREVIEW_MODE = "--preview" in sys.argv or os.environ.get("TRACKING_WEB_PREVIEW") == "1"
# 确保 ROS2 DDS 不被限制在 localhost（否则可能收不到 /tracking/visualization）
if not PREVIEW_MODE:
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "0")

# ================= 全局状态 =================
ros_node = None
executor = None
ros_thread = None
latest_jpeg = None
latest_keypoints = None
coarse_done_status = False
latest_points3d = None
gripper_traj = []
MAX_TRAJ_POINTS = 1500

def _make_placeholder_jpeg():
    """生成占位图：网格 + 动态提示"""
    img = np.full((480, 640, 3), 20, dtype=np.uint8)
    for i in range(0, 640, 40):
        cv2.line(img, (i, 0), (i, 480), (30, 30, 30), 1)
    for i in range(0, 480, 40):
        cv2.line(img, (0, i), (640, i), (30, 30, 30), 1)
    cv2.putText(img, "WAITING FOR ROS2 STREAM", (120, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 136), 2)
    cv2.putText(img, "Preview Mode" if PREVIEW_MODE else "No Signal", (240, 260), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 1)
    t = time.time()
    x = int(320 + 100 * np.sin(t * 2))
    y = int(240 + 50 * np.cos(t * 2))
    cv2.circle(img, (x, y), 10, (0, 100, 255), -1)
    _, jpeg = cv2.imencode(".jpg", img)
    return jpeg.tobytes()


def _imgmsg_to_cv2(msg):
    """ROS2 Image -> OpenCV，避免 cv_bridge 与 numpy 2.x 不兼容"""
    try:
        from cv_bridge import CvBridge
        return CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except (ImportError, AttributeError):
        pass
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if msg.encoding in ("bgr8", "rgb8"):
        h, w = int(msg.height), int(msg.width)
        step = int(msg.step) if msg.step else w * 3
        img = data.reshape(h, step)[:, : w * 3].copy().reshape(h, w, 3)
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img
    raise ValueError(f"不支持的编码: {msg.encoding}")


class SetKeypointsRequest(BaseModel):
    x: list[float]
    y: list[float]


class ControlRequest(BaseModel):
    command: str


class SetCoarsePointRequest(BaseModel):
    x: float
    y: float


class WebBridgeNode:
    """ROS2 桥接节点：订阅话题、调用服务"""

    def __init__(self, node_cls):
        self._n = node_cls("tracking_web_bridge")
        self._n.declare_parameter("viz_topic", "tracking/visualization")
        self._n.declare_parameter("keypoints_topic", "tracking/keypoints")
        self._n.declare_parameter("coarse_done_topic", "tracking/coarse_done")
        self._n.declare_parameter("points3d_topic", "tracking/points3d_in_arm_base")
        self._n.declare_parameter("traj_topic", "tracking/gripper_traj")

        from sensor_msgs.msg import Image, PointCloud
        from geometry_msgs.msg import PointStamped
        from std_msgs.msg import Bool
        from rclpy.qos import qos_profile_sensor_data

        viz_topic = self._n.get_parameter("viz_topic").value
        kp_topic = self._n.get_parameter("keypoints_topic").value
        coarse_topic = self._n.get_parameter("coarse_done_topic").value
        points3d_topic = self._n.get_parameter("points3d_topic").value
        traj_topic = self._n.get_parameter("traj_topic").value

        # 使用 sensor_data QoS，兼容 best-effort 图像发布者
        self._n.create_subscription(Image, viz_topic, self.viz_cb, qos_profile_sensor_data)
        try:
            from track_on_ros2_msgs.msg import Keypoints
            self._n.create_subscription(Keypoints, kp_topic, self.keypoints_cb, 5)
        except ImportError:
            self._n.get_logger().warn("无法导入 Keypoints，keypoints 状态不可用")
        self._n.create_subscription(Bool, coarse_topic, self.coarse_done_cb, 5)
        self._n.create_subscription(PointCloud, points3d_topic, self.points3d_cb, qos_profile_sensor_data)
        self._n.create_subscription(PointStamped, traj_topic, self.traj_cb, qos_profile_sensor_data)

        self._setup_clients()
        self._n.get_logger().info(f"订阅: {viz_topic}, {kp_topic}, {coarse_topic}, {points3d_topic}, {traj_topic}")

    def _setup_clients(self):
        self.set_kp_client = self.control_client = self.reset_client = self.coarse_client = None
        self.stop_motion_client = None
        self.toggle_mode_client = None
        self.set_arm_mode_client = None
        self.return_home_client = None
        try:
            from track_on_ros2_srv.srv import SetKeypoints, ControlTracking, ResetTracking
            self.set_kp_client = self._n.create_client(SetKeypoints, "tracking/set_keypoints")
            self.control_client = self._n.create_client(ControlTracking, "tracking/control")
            self.reset_client = self._n.create_client(ResetTracking, "tracking/reset")
        except ImportError as e:
            self._n.get_logger().warn(f"跟踪服务不可用: {e}")
        try:
            from track_on_ros2_srv.srv import SetCoarsePoint
            self.coarse_client = self._n.create_client(SetCoarsePoint, "tracking/set_coarse_point")
        except ImportError:
            self._n.get_logger().warn("SetCoarsePoint 不可用，请先 colcon build track_on_ros2_srv")
        try:
            from std_srvs.srv import SetBool
            self.stop_motion_client = self._n.create_client(SetBool, "tracking/stop_motion")
        except ImportError:
            self._n.get_logger().warn("SetBool 不可用，停止运动服务无法使用")
        try:
            from std_srvs.srv import Trigger
            self.toggle_mode_client = self._n.create_client(Trigger, "tracking/toggle_arm_mode")
        except ImportError:
            self._n.get_logger().warn("Trigger 不可用，模式切换服务无法使用")
        try:
            from std_srvs.srv import SetBool
            self.set_arm_mode_client = self._n.create_client(SetBool, "tracking/set_arm_mode")
        except ImportError:
            self._n.get_logger().warn("SetBool 不可用，设置机械臂模式服务无法使用")
        try:
            from std_srvs.srv import Trigger
            self.return_home_client = self._n.create_client(Trigger, "tracking/return_home")
        except ImportError:
            self._n.get_logger().warn("Trigger 不可用，回位服务无法使用")

    def destroy_node(self):
        self._n.destroy_node()

    def viz_cb(self, msg):
        global latest_jpeg
        try:
            cv_img = _imgmsg_to_cv2(msg)
            _, jpeg = cv2.imencode(".jpg", cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
            latest_jpeg = jpeg.tobytes()
        except Exception as e:
            self._n.get_logger().warn(f"图像转换失败: {e}")

    def keypoints_cb(self, msg):
        global latest_keypoints
        try:
            latest_keypoints = [{"id": kp.id, "x": kp.x, "y": kp.y, "visible": kp.visible} for kp in msg.keypoints]
        except Exception:
            latest_keypoints = None

    def coarse_done_cb(self, msg):
        global coarse_done_status
        coarse_done_status = msg.data

    def points3d_cb(self, msg):
        global latest_points3d
        try:
            ids = None
            for ch in msg.channels:
                if ch.name == "id":
                    ids = ch.values
                    break
            pts = []
            for i, p in enumerate(msg.points):
                pid = int(ids[i]) if ids is not None and i < len(ids) else i
                pts.append({"id": pid, "x": float(p.x), "y": float(p.y), "z": float(p.z)})
            latest_points3d = pts
        except Exception as e:
            self._n.get_logger().warn(f"points3d 解析失败: {e}")

    def traj_cb(self, msg):
        global gripper_traj
        try:
            gripper_traj.append({"x": float(msg.point.x), "y": float(msg.point.y), "z": float(msg.point.z)})
            if len(gripper_traj) > MAX_TRAJ_POINTS:
                gripper_traj = gripper_traj[-MAX_TRAJ_POINTS:]
        except Exception:
            pass


def ros_spin_thread():
    import rclpy
    global executor, ros_node
    while rclpy.ok() and executor is not None and ros_node is not None:
        try:
            executor.spin_once(timeout_sec=0.1)
        except Exception:
            break


@asynccontextmanager
async def lifespan(app: FastAPI):
    """启动时初始化 ROS2"""
    global ros_node, executor, ros_thread
    if PREVIEW_MODE:
        print(">>> 预览模式 (Preview Mode) <<<")
        yield
        return

    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor

    rcl_initialized = False
    try:
        rclpy.init()
        rcl_initialized = True
        ros_node = WebBridgeNode(Node)
        executor = SingleThreadedExecutor()
        executor.add_node(ros_node._n)
        ros_thread = threading.Thread(target=ros_spin_thread, daemon=True)
        ros_thread.start()
        print(">>> ROS2 桥接已启动 <<<")
    except Exception as e:
        print(f"ROS2 初始化失败: {e}")
        ros_node = None

    yield

    if ros_node:
        ros_node.destroy_node()
    if rcl_initialized and rclpy.ok():
        rclpy.shutdown()


app = FastAPI(title="跟踪系统 Web 上位机", lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_credentials=True, allow_methods=["*"], allow_headers=["*"])


def call_ros_service(client, request, timeout_sec=5.0):
    """通过 WebBridgeNode 客户端同步调用 ROS2 服务"""
    global ros_node, ros_thread, executor
    if PREVIEW_MODE:
        raise HTTPException(status_code=503, detail="预览模式下不可用")
    if ros_node is None or client is None:
        raise HTTPException(status_code=503, detail="ROS2 节点未就绪")
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise HTTPException(status_code=503, detail="服务不可用，请确认 track_on_ros2 已启动")
    future = client.call_async(request)
    import rclpy
    deadline = time.time() + timeout_sec
    while rclpy.ok() and time.time() < deadline:
        if future.done():
            result = future.result()
            if result is not None:
                return result
            break
        # 避免与后台 ROS 线程重复 spin，防止 re-entrancy 报错
        if ros_thread is not None and ros_thread.is_alive():
            time.sleep(0.02)
        elif executor:
            executor.spin_once(timeout_sec=0.1)
    raise HTTPException(status_code=500, detail="服务调用超时")


@app.get("/")
async def index():
    return FileResponse(STATIC_DIR / "index.html")


@app.get("/stream")
async def stream():
    """MJPEG 视频流"""
    global latest_jpeg

    def generate():
        last_frame = None
        placeholder = _make_placeholder_jpeg()
        while True:
            if latest_jpeg:
                last_frame = latest_jpeg
            frame = last_frame or placeholder
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            time.sleep(0.04)

    return StreamingResponse(generate(), media_type="multipart/x-mixed-replace; boundary=frame", headers={"Cache-Control": "no-cache"})


@app.get("/api/status")
async def get_status():
    global latest_keypoints, coarse_done_status
    kp = latest_keypoints if latest_keypoints is not None else ([{"x": 100, "y": 100}] if PREVIEW_MODE else [])
    return {"keypoints": kp, "coarse_done": coarse_done_status}


@app.get("/api/spatial")
async def get_spatial():
    if PREVIEW_MODE:
        return {"points3d": [], "keypoints": [], "traj": []}
    return {
        "points3d": latest_points3d or [],
        "keypoints": latest_keypoints or [],
        "traj": gripper_traj[-800:] if gripper_traj else [],
    }


@app.post("/api/set_keypoints")
async def set_keypoints(req: SetKeypointsRequest):
    if PREVIEW_MODE or ros_node is None:
        raise HTTPException(status_code=503, detail="预览模式下不可用")
    try:
        from track_on_ros2_srv.srv import SetKeypoints
        srv_req = SetKeypoints.Request()
        srv_req.x = [float(x) for x in req.x]
        srv_req.y = [float(y) for y in req.y]
        resp = call_ros_service(ros_node.set_kp_client, srv_req)
        return {"success": resp.success, "message": resp.message}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/control")
async def control_tracking(req: ControlRequest):
    if PREVIEW_MODE or ros_node is None:
        raise HTTPException(status_code=503, detail="预览模式下不可用")
    try:
        from track_on_ros2_srv.srv import ControlTracking
        srv_req = ControlTracking.Request()
        srv_req.command = req.command
        resp = call_ros_service(ros_node.control_client, srv_req)
        return {"success": resp.success, "message": resp.message}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/reset")
async def reset_tracking():
    if PREVIEW_MODE or ros_node is None:
        raise HTTPException(status_code=503, detail="预览模式下不可用")
    try:
        from track_on_ros2_srv.srv import ResetTracking
        srv_req = ResetTracking.Request()
        resp = call_ros_service(ros_node.reset_client, srv_req)
        return {"success": resp.success, "message": resp.message}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/stop_motion")
async def stop_motion():
    if PREVIEW_MODE or ros_node is None:
        raise HTTPException(status_code=503, detail="预览模式下不可用")
    if ros_node.stop_motion_client is None:
        raise HTTPException(status_code=503, detail="停止运动服务不可用")
    try:
        from std_srvs.srv import SetBool
        srv_req = SetBool.Request()
        srv_req.data = True
        # 服务由 points3d_tf_to_arm_base_node 提供，给更明确的错误信息与更长等待
        if not ros_node.stop_motion_client.wait_for_service(timeout_sec=10.0):
            raise HTTPException(status_code=503, detail="服务不可用，请确认 points3d_tf_to_arm_base_node 已启动")
        resp = call_ros_service(ros_node.stop_motion_client, srv_req)
        return {"success": resp.success, "message": resp.message}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/resume_motion")
async def resume_motion():
    if PREVIEW_MODE or ros_node is None:
        raise HTTPException(status_code=503, detail="预览模式下不可用")
    if ros_node.stop_motion_client is None:
        raise HTTPException(status_code=503, detail="停止运动服务不可用")
    try:
        from std_srvs.srv import SetBool
        srv_req = SetBool.Request()
        srv_req.data = False
        if not ros_node.stop_motion_client.wait_for_service(timeout_sec=10.0):
            raise HTTPException(status_code=503, detail="服务不可用，请确认 points3d_tf_to_arm_base_node 已启动")
        resp = call_ros_service(ros_node.stop_motion_client, srv_req)
        return {"success": resp.success, "message": resp.message}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


class SetArmModeRequest(BaseModel):
    mode: str  # "position" | "zero_force"


@app.post("/api/set_arm_mode")
async def set_arm_mode(req: SetArmModeRequest):
    """设置机械臂模式：position(位置模式) 或 zero_force(零力模式)"""
    if PREVIEW_MODE or ros_node is None:
        raise HTTPException(status_code=503, detail="预览模式下不可用")
    if ros_node.set_arm_mode_client is None:
        raise HTTPException(status_code=503, detail="设置机械臂模式服务不可用")
    try:
        from std_srvs.srv import SetBool
        srv_req = SetBool.Request()
        srv_req.data = req.mode.lower() == "position"
        if not ros_node.set_arm_mode_client.wait_for_service(timeout_sec=10.0):
            raise HTTPException(status_code=503, detail="服务不可用，请确认 points3d_tf_to_arm_base_node 已启动")
        resp = call_ros_service(ros_node.set_arm_mode_client, srv_req)
        return {"success": resp.success, "message": resp.message}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/toggle_arm_mode")
async def toggle_arm_mode():
    if PREVIEW_MODE or ros_node is None:
        raise HTTPException(status_code=503, detail="预览模式下不可用")
    if ros_node.toggle_mode_client is None:
        raise HTTPException(status_code=503, detail="模式切换服务不可用")
    try:
        from std_srvs.srv import Trigger
        srv_req = Trigger.Request()
        # 服务由 points3d_tf_to_arm_base_node 提供，给更明确的错误信息与更长等待
        if not ros_node.toggle_mode_client.wait_for_service(timeout_sec=10.0):
            raise HTTPException(status_code=503, detail="服务不可用，请确认 points3d_tf_to_arm_base_node 已启动")
        resp = call_ros_service(ros_node.toggle_mode_client, srv_req)
        return {"success": resp.success, "message": resp.message}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/return_home")
async def return_home():
    if PREVIEW_MODE or ros_node is None:
        raise HTTPException(status_code=503, detail="预览模式下不可用")
    if ros_node.return_home_client is None:
        raise HTTPException(status_code=503, detail="回位服务不可用")
    try:
        from std_srvs.srv import Trigger
        srv_req = Trigger.Request()
        if not ros_node.return_home_client.wait_for_service(timeout_sec=10.0):
            raise HTTPException(status_code=503, detail="服务不可用，请确认 points3d_tf_to_arm_base_node 已启动")
        resp = call_ros_service(ros_node.return_home_client, srv_req)
        return {"success": resp.success, "message": resp.message}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/toggle_motion")
async def toggle_motion(req: dict):
    if PREVIEW_MODE or ros_node is None:
        raise HTTPException(status_code=503, detail="预览模式下不可用")
    if ros_node.stop_motion_client is None:
        raise HTTPException(status_code=503, detail="停止运动服务不可用")
    try:
        from std_srvs.srv import SetBool
        should_stop = bool(req.get("stop", True))
        srv_req = SetBool.Request()
        srv_req.data = should_stop
        if not ros_node.stop_motion_client.wait_for_service(timeout_sec=10.0):
            raise HTTPException(status_code=503, detail="服务不可用，请确认 points3d_tf_to_arm_base_node 已启动")
        resp = call_ros_service(ros_node.stop_motion_client, srv_req)
        return {"success": resp.success, "message": resp.message}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/set_coarse_point")
async def set_coarse_point(req: SetCoarsePointRequest):
    if PREVIEW_MODE or ros_node is None:
        raise HTTPException(status_code=503, detail="预览模式下不可用")
    if ros_node.coarse_client is None:
        raise HTTPException(status_code=503, detail="粗定位服务不可用")
    try:
        from track_on_ros2_srv.srv import SetCoarsePoint
        srv_req = SetCoarsePoint.Request()
        srv_req.x = float(req.x)
        srv_req.y = float(req.y)
        resp = call_ros_service(ros_node.coarse_client, srv_req)
        return {"success": resp.success, "message": resp.message}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


STATIC_DIR.mkdir(parents=True, exist_ok=True)
app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")


def main():
    import uvicorn
    print("Visit http://localhost:8080")
    uvicorn.run(app, host="0.0.0.0", port=8080)


if __name__ == "__main__":
    main()
