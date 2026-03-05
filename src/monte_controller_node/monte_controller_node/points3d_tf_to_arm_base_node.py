import os
import sys
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Bool
from geometry_msgs.msg import Point32, PointStamped
from std_srvs.srv import SetBool, Trigger
import yaml


class Points3DTFToArmBaseNode(Node):
    """
    订阅 tracking/points3d (sensor_msgs/PointCloud)，
    处理流程：
      1) 要求输入点的 frame_id == source_frame（optical）
      2) 外参 T_{wrist<-camera}：p_wrist = R_ext·p_camera + t_ext
      3) RobotLib TF T_{base<-wrist}：p_base = R·p_wrist + t
    打印三坐标系下的点，并可选发布到 tracking/points3d_in_arm_base。
    """

    @staticmethod
    def _quat_mul_wxyz(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """Quaternion multiply (wxyz)."""
        w1, x1, y1, z1 = [float(v) for v in q1]
        w2, x2, y2, z2 = [float(v) for v in q2]
        return np.array([
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ], dtype=float)

    @staticmethod
    def _quat_to_rot_wxyz(q: np.ndarray) -> np.ndarray:
        """Quaternion (wxyz) -> 3x3 rotation."""
        qw, qx, qy, qz = [float(v) for v in q]
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 0:
            return np.eye(3, dtype=float)
        qw, qx, qy, qz = qw / norm, qx / norm, qy / norm, qz / norm
        xx, yy, zz = qx * qx, qy * qy, qz * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        wx, wy, wz = qw * qx, qw * qy, qw * qz
        return np.array([
            [1 - 2 * (yy + zz),     2 * (xy - wz),         2 * (xz + wy)],
            [2 * (xy + wz),         1 - 2 * (xx + zz),     2 * (yz - wx)],
            [2 * (xz - wy),         2 * (yz + wx),         1 - 2 * (xx + yy)],
        ], dtype=float)

    @classmethod
    def _compose_tq(cls, t1: np.ndarray, q1: np.ndarray, t2: np.ndarray, q2: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Compose two transforms: (t1,q1) o (t2,q2). Quaternions are wxyz."""
        R1 = cls._quat_to_rot_wxyz(q1)
        t = t1 + (R1 @ t2)
        q = cls._quat_mul_wxyz(q1, q2)
        return t, q

    def _load_config(self):
        """加载配置文件"""
        # 获取包路径
        try:
            from ament_index_python.packages import get_package_share_directory
            package_path = get_package_share_directory('monte_controller_node')
            config_file = os.path.join(package_path, 'config', 'config.yaml')
        except Exception:
            # 如果找不到包路径，使用相对路径
            current_dir = os.path.dirname(os.path.abspath(__file__))
            config_file = os.path.join(current_dir, '..', 'config', 'config.yaml')

        if not os.path.exists(config_file):
            self.get_logger().error(f'配置文件不存在: {config_file}')
            return None

        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            self.get_logger().info(f'成功加载配置文件: {config_file}')
            return config
        except Exception as e:
            self.get_logger().error(f'加载配置文件失败: {e}')
            return None

    def __init__(self):
        super().__init__('points3d_tf_to_arm_base_node')

        # 加载配置文件
        self.config = self._load_config()
        if self.config is None:
            self.get_logger().error('无法加载配置文件，使用默认参数')
            self.config = {}

        # 参数：RobotLib 连接
        default_robot_ip = self.config.get('robot', {}).get('ip', '192.168.22.112:50051')
        self.declare_parameter('robot_ip', default_robot_ip)
        try:
            from ament_index_python.packages import get_package_share_directory  # type: ignore
            default_lib = os.path.join(get_package_share_directory('monte_controller_node'), 'lib')
        except Exception:
            default_lib = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'lib')
        self.declare_parameter('robot_lib_path', default_lib)

        # 优先声明 component_type，便于根据左/右臂选择 frames 默认值
        default_component_type = self.config.get('robot', {}).get('component_type', 1)
        self.declare_parameter('component_type', default_component_type)  # 1: 左臂  2: 右臂
        component_type_for_defaults = int(self.get_parameter('component_type').value)

        frames_cfg = self.config.get('frames', {}) or {}
        default_input_topic = frames_cfg.get('input_topic', 'tracking/points3d')

        # 兼容两种 config：
        # 1) 新格式：frames.left / frames.right
        # 2) 旧格式：frames.source_frame / frames.wrist_frame / frames.target_frame
        arm_key = 'right' if component_type_for_defaults == 2 else 'left'
        arm_frames_cfg = frames_cfg.get(arm_key, {}) if isinstance(frames_cfg.get(arm_key, {}), dict) else {}
        if not arm_frames_cfg:
            arm_frames_cfg = frames_cfg

        if component_type_for_defaults == 2:
            fallback_source = 'right_camera_color_optical_frame'
            fallback_wrist = 'joint_r7_wrist_roll'
            fallback_target = 'link_r0_arm_base'
        else:
            fallback_source = 'left_camera_color_optical_frame'
            fallback_wrist = 'joint_l7_wrist_roll'
            fallback_target = 'link_l0_arm_base'

        default_source_frame = arm_frames_cfg.get('source_frame', fallback_source)
        default_wrist_frame = arm_frames_cfg.get('wrist_frame', fallback_wrist)
        default_target_frame = arm_frames_cfg.get('target_frame', fallback_target)
        self.declare_parameter('input_topic', default_input_topic)
        self.declare_parameter('source_frame', default_source_frame)
        self.declare_parameter('wrist_frame', default_wrist_frame)
        self.declare_parameter('target_frame', default_target_frame)
        self.declare_parameter('publish_transformed', True)
        self.declare_parameter('print_limit', 10)
        # 粗定位控制参数
        default_enable_coarse_move = self.config.get('control', {}).get('enable_coarse_move', True)
        self.declare_parameter('enable_coarse_move', default_enable_coarse_move)
        default_distance_stop = self.config.get('control', {}).get('distance_stop', 0.5)
        default_step_size = self.config.get('control', {}).get('step_size', 0.08)
        default_cmd_interval = self.config.get('control', {}).get('cmd_interval', 0.3)
        default_z_first = self.config.get('control', {}).get('z_first', True)
        default_z_align_threshold = self.config.get('control', {}).get('z_align_threshold', 0.15)
        default_max_speed = self.config.get('control', {}).get('max_speed', 0.10)
        default_max_acc = self.config.get('control', {}).get('max_acc', 0.15)
        default_use_wait = self.config.get('control', {}).get('use_wait', False)
        default_target_id = self.config.get('control', {}).get('target_id', -1)
        default_initial_servo_angles = self.config.get('control', {}).get(
            'initial_servo_angles',
            [-0.6006123423576355, 0.11826176196336746, 0.028828054666519165,
             1.8238468170166016, -1.4655508995056152, 0.1113307848572731,
             0.38727688789367676],
        )
        self.declare_parameter('distance_stop', default_distance_stop)  # m
        self.declare_parameter('step_size', default_step_size)     # 每步前进距离 m
        self.declare_parameter('cmd_interval', default_cmd_interval)   # 连续set_arm_position发送最小间隔 s（当wait=False）
        self.declare_parameter('z_first', default_z_first)  # 先对齐Z再直线靠近
        self.declare_parameter('z_align_threshold', default_z_align_threshold)  # m, |dz|<=阈值后进入直线靠近
        self.declare_parameter('max_speed', default_max_speed)     # m/s
        self.declare_parameter('max_acc', default_max_acc)       # m/s^2
        self.declare_parameter('use_wait', default_use_wait)     # set_arm_position 的 wait
        self.declare_parameter('target_id', default_target_id)       # 若>=0，则优先匹配channel 'id'
        self.declare_parameter('initial_servo_angles', default_initial_servo_angles)

        # TCP 相关：粗定位控制使用夹爪 TCP（参考 record_traj/record_left_end_traj.py）
        default_gripper_base_frame = self.config.get('frames', {}).get('gripper_base_frame', None)
        if not default_gripper_base_frame:
            default_gripper_base_frame = 'link_lt_gripper_base' if int(default_component_type) == 1 else 'link_rt_gripper_base'
        self.declare_parameter('gripper_base_frame', default_gripper_base_frame)
        self.declare_parameter('tcp_offset_xyz', self.config.get('frames', {}).get('tcp_offset_xyz', [0.0, 0.0, -0.2900]))

        # 外参文件 + 方向 + 坐标系约定（根据 component_type 选择左/右臂外参）
        ws_root = self._get_workspace_root()
        extrinsics_cfg = self.config.get('extrinsics', {}) or {}
        arm_key = 'right' if component_type_for_defaults == 2 else 'left'
        arm_extrinsic_cfg = extrinsics_cfg.get(arm_key, {}) if isinstance(extrinsics_cfg.get(arm_key, {}), dict) else {}
        if not arm_extrinsic_cfg:
            arm_extrinsic_cfg = extrinsics_cfg

        if component_type_for_defaults == 2:
            fallback_extrinsic = 'config/joint_rt_sensor_rgbd_12.txt'
        else:
            fallback_extrinsic = 'config/joint_lt_sensor_rgbd_12.txt'

        config_extrinsic = arm_extrinsic_cfg.get('wrist_extrinsic_file', fallback_extrinsic)
        default_extrinsic = os.path.join(ws_root, config_extrinsic) if ws_root else ''
        self.declare_parameter('wrist_extrinsic_file', default_extrinsic)
        self.declare_parameter('invert_extrinsic', False)  # 若文件给的是 T_{source<-wrist}，则置 True 取逆
        # 若外参以相机笛卡尔坐标（x前,y左,z上）为源，而输入为光学坐标（x右,y下,z前），需先做 optical->camera 固定旋转
        self.declare_parameter('apply_optical_to_camera_rotation', False)  # 与参考代码一致，默认 False

        # 读取参数
        robot_ip = self.get_parameter('robot_ip').value
        robot_lib_path = self.get_parameter('robot_lib_path').value
        self.input_topic = self.get_parameter('input_topic').value
        self.source_frame = self.get_parameter('source_frame').value
        self.wrist_frame = self.get_parameter('wrist_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.publish_transformed = bool(self.get_parameter('publish_transformed').value)
        self.print_limit = int(self.get_parameter('print_limit').value)
        wrist_extrinsic_file = self.get_parameter('wrist_extrinsic_file').value
        self.invert_extrinsic = bool(self.get_parameter('invert_extrinsic').value)
        self.apply_optical_to_camera_rotation = bool(self.get_parameter('apply_optical_to_camera_rotation').value)

        self.get_logger().info(f'Robot IP: {robot_ip}')
        self.get_logger().info(f'RobotLib Path: {robot_lib_path}')
        self.get_logger().info(f'外参文件: {wrist_extrinsic_file}, invert={self.invert_extrinsic}')
        self.get_logger().info(
            f"订阅: {self.input_topic}, 流程: {self.target_frame} <- {self.wrist_frame} <-ext- {self.source_frame}")

        # 载入外参（相机 -> 手腕）
        try:
            # 解析外参文件
            vals = {}
            with open(wrist_extrinsic_file, 'r') as f:
                for ln in f:
                    ln = ln.strip()
                    if not ln or ln.startswith('#'):
                        continue
                    if ':' in ln:
                        k, v = ln.split(':', 1)
                        k = k.strip()
                        v = v.strip()
                        # 顶层 key 跳过（例如 joint_r7_wrist_roll:）
                        if k in ('joint_r7_wrist_roll', 'joint_l7_wrist_roll'):
                            continue
                        try:
                            vals[k] = float(v)
                        except ValueError:
                            pass
            tx = float(vals.get('x', 0.0))
            ty = float(vals.get('y', 0.0))
            tz = float(vals.get('z', 0.0))
            roll = float(vals.get('roll', 0.0))
            pitch = float(vals.get('pitch', 0.0))
            yaw = float(vals.get('yaw', 0.0))
            
            # RPY 转旋转矩阵 (ZYX: yaw->pitch->roll)
            cr, sr = math.cos(roll), math.sin(roll)
            cp, sp = math.cos(pitch), math.sin(pitch)
            cy, sy = math.cos(yaw), math.sin(yaw)
            Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
            Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
            Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
            R_ext = Rz @ Ry @ Rx
            t_ext = np.array([tx, ty, tz], dtype=float)
            
            # 如果需要取逆
            if self.invert_extrinsic:
                R_ext = R_ext.T
                t_ext = -R_ext @ t_ext
            
            self.R_cam2wrist = R_ext
            self.t_cam2wrist = t_ext
            self.get_logger().info(
                f'外参 OK: t=({self.t_cam2wrist[0]:.6f},{self.t_cam2wrist[1]:.6f},{self.t_cam2wrist[2]:.6f})')
        except Exception as e:
            self.get_logger().error(f'解析外参文件失败: {e}')
            self.R_cam2wrist = np.eye(3, dtype=float)
            self.t_cam2wrist = np.zeros(3, dtype=float)

        # 导入并连接 RobotLib
        if robot_lib_path:
            ld_path = os.environ.get('LD_LIBRARY_PATH', '')
            if robot_lib_path not in ld_path.split(':'):
                os.environ['LD_LIBRARY_PATH'] = robot_lib_path + (':' + ld_path if ld_path else '')
            if robot_lib_path not in sys.path:
                sys.path.append(robot_lib_path)
        try:
            from RobotLib import Robot  # type: ignore
            self.robot = Robot(robot_ip, '', '')
            self.get_logger().info('Robot connected successfully')
        except Exception as e:
            self.get_logger().error(f'导入/连接 RobotLib 失败: {e}')
            self.robot = None

        # 订阅与发布
        self.sub = self.create_subscription(PointCloud, self.input_topic, self.cb_points, 10)
        self.pub = None
        if self.publish_transformed:
            self.pub = self.create_publisher(PointCloud, 'tracking/points3d_in_arm_base', 10)
        # 粗定位完成通知
        self.coarse_done_pub = self.create_publisher(Bool, 'tracking/coarse_done', 10)
        self._last_msg_was_coarse = False
        # 夹爪轨迹发布
        self.traj_pub = self.create_publisher(PointStamped, 'tracking/gripper_traj', 10)
        self.timer_traj = self.create_timer(0.1, self._publish_traj)  # 10Hz
        # 运动停止控制服务（True=停止，False=恢复）
        self.motion_paused = False
        self.stop_srv = self.create_service(SetBool, 'tracking/stop_motion', self._handle_stop_motion)
        # 机械臂模式切换服务（position <-> zero_force）
        self.arm_tgt_mode = "position"
        self.toggle_mode_srv = self.create_service(Trigger, 'tracking/toggle_arm_mode', self._handle_toggle_arm_mode)
        # 机械臂模式设置服务（SetBool: data=true->位置模式1, data=false->零力模式2）
        self.set_arm_mode_srv = self.create_service(SetBool, 'tracking/set_arm_mode', self._handle_set_arm_mode)
        # 回到初始位姿
        self.return_home_srv = self.create_service(Trigger, 'tracking/return_home', self._handle_return_home)

        # ---- 粗定位控制相关 ----
        self.enable_coarse_move = bool(self.get_parameter('enable_coarse_move').value)

        self._last_log_ts = 0.0
        self._move_fail_count = 0
        self._max_fail_count = 3  # 连续失败3次就暂停

        self.component_type = int(self.get_parameter('component_type').value)
        self.distance_stop = float(self.get_parameter('distance_stop').value)
        self.step_size = float(self.get_parameter('step_size').value)
        self.cmd_interval = float(self.get_parameter('cmd_interval').value)
        self.z_first = bool(self.get_parameter('z_first').value)
        self.z_align_threshold = float(self.get_parameter('z_align_threshold').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_acc = float(self.get_parameter('max_acc').value)
        self.use_wait = bool(self.get_parameter('use_wait').value)
        self.target_id = int(self.get_parameter('target_id').value)
        try:
            self.initial_servo_angles = [float(v) for v in self.get_parameter('initial_servo_angles').value]
        except Exception:
            self.initial_servo_angles = default_initial_servo_angles

        # TCP 参数（用于“末端”为夹爪 TCP 的粗定位控制）
        self.gripper_base_frame = str(self.get_parameter('gripper_base_frame').value)
        try:
            self.tcp_offset_xyz = np.array(self.get_parameter('tcp_offset_xyz').value, dtype=float).reshape(3)
        except Exception:
            self.tcp_offset_xyz = np.array([0.0, 0.0, -0.2900], dtype=float)
        self._wrist_to_tcp_t = None  # 3,
        self._wrist_to_tcp_q = None  # 4, wxyz

        # 最新目标点（Base系）与平滑缓存
        self.target_point = None
        self.last_cmd_ts = 0.0
        self.cached_quat_wxyz = None  # 保持当前姿态
        self._z_align_goal_tcp_z = None
        self._z_align_cmd_sent = False

        if self.enable_coarse_move:
            self.timer_ctrl = self.create_timer(0.05, self.control_loop)  # 20Hz
            self.get_logger().info(
                f"粗定位控制已启用: stop={self.distance_stop}m, step={self.step_size}m, wait={self.use_wait}, "
                f"z_first={self.z_first}, z_align={self.z_align_threshold}m"
            )

    def _handle_stop_motion(self, request: SetBool.Request, response: SetBool.Response):
        """停止/恢复机械臂运动。True=停止，False=恢复。"""
        if request.data:
            self.motion_paused = True
            ok = self._stop_robot_motion()
            response.success = bool(ok)
            response.message = "停止运动指令已发送" if ok else "停止运动失败（未连接或指令失败）"
        else:
            self.motion_paused = False
            response.success = True
            response.message = "已恢复运动"
        return response

    def _stop_robot_motion(self) -> bool:
        """尽量让机械臂立即停止：清理控制状态 + 下发“保持当前姿态”的指令。"""
        if self.robot is None:
            return False

        # 清理控制状态，避免继续发送移动指令
        self.target_point = None
        self.cached_quat_wxyz = None
        self._z_align_goal_tcp_z = None
        self._z_align_cmd_sent = False
        self._last_msg_was_coarse = False
        self._move_fail_count = 0

        # 优先使用关节伺服角度停止（更“硬”）
        try:
            ok, angles = self.robot.get_arm_servo_angle(self.component_type)
            if ok and angles and len(angles) >= 7:
                self.robot.set_arm_servo_angle(self.component_type, angles, 0.25, 0.25, False)
                return True
        except Exception:
            pass

        # 退化：使用当前末端位姿“锁定”
        try:
            ok, pose_aw = self.robot.get_arm_position(self.component_type)
            if ok and pose_aw and len(pose_aw) >= 7:
                self.robot.set_arm_position(self.component_type, pose_aw, float(self.max_speed), float(self.max_acc), False)
                return True
        except Exception:
            pass

        return False

    def _handle_toggle_arm_mode(self, request: Trigger.Request, response: Trigger.Response):
        """切换机械臂模式：position <-> zero_force。"""
        if self.robot is None:
            response.success = False
            response.message = "机器人未连接，无法切换模式"
            return response

        if self.arm_tgt_mode == "position":
            ok = self.robot.set_arm_mode(self.component_type, 1)
            if ok:
                self.arm_tgt_mode = "zero_force"
                response.success = True
                response.message = "已切换到位置模式 (mode=1)"
            else:
                response.success = False
                response.message = "切换到位置模式失败"
        else:
            ok = self.robot.set_arm_mode(self.component_type, 2)
            if ok:
                self.arm_tgt_mode = "position"
                response.success = True
                response.message = "已切换到零力模式 (mode=2)"
            else:
                response.success = False
                response.message = "切换到零力模式失败"
        return response

    def _handle_set_arm_mode(self, request: SetBool.Request, response: SetBool.Response):
        """设置机械臂模式：data=True->位置模式(1)，data=False->零力模式(2)。参考 record_left_end_traj.py"""
        if self.robot is None:
            response.success = False
            response.message = "机器人未连接，无法设置模式"
            return response
        mode = 1 if request.data else 2
        ok = self.robot.set_arm_mode(self.component_type, mode)
        if ok:
            self.arm_tgt_mode = "position" if mode == 2 else "zero_force"
            response.success = True
            response.message = "已切换到位置模式 (mode=1)" if mode == 1 else "已切换到零力模式 (mode=2)"
        else:
            response.success = False
            response.message = f"切换到{'位置' if mode == 1 else '零力'}模式失败"
        return response

    def _publish_traj(self):
        """发布夹爪 TCP 轨迹点（base 坐标系）。"""
        if self.robot is None:
            return
        try:
            ok, pose_aw = self.robot.get_arm_position(self.component_type)
            if (not ok) or pose_aw is None or len(pose_aw) < 7:
                return
            t_aw = np.array([float(v) for v in pose_aw[:3]], dtype=float)
            q_aw = np.array([float(v) for v in pose_aw[3:7]], dtype=float)  # wxyz

            t_wt, _q_wt = self._get_wrist_to_tcp()
            if t_wt is None:
                return
            R_aw = self._quat_to_rot_wxyz(q_aw)
            p_tcp = t_aw + (R_aw @ t_wt)

            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.target_frame
            msg.point.x = float(p_tcp[0])
            msg.point.y = float(p_tcp[1])
            msg.point.z = float(p_tcp[2])
            self.traj_pub.publish(msg)
        except Exception:
            return

    def _handle_return_home(self, request: Trigger.Request, response: Trigger.Response):
        """回到初始伺服角度位姿。"""
        if self.robot is None:
            response.success = False
            response.message = "机器人未连接，无法回位"
            return response
        try:
            # 回位时先暂停粗定位控制
            self.motion_paused = True
            self.target_point = None
            self.cached_quat_wxyz = None
            ok = self.robot.set_arm_servo_angle(self.component_type, self.initial_servo_angles, 0.3, 1.0, True)
            response.success = bool(ok)
            response.message = "已回到初始位姿" if ok else "回位失败"
        except Exception as e:
            response.success = False
            response.message = f"回位异常: {e}"
        return response

    def _get_workspace_root(self):
        """获取工作空间根目录（tracking_ros2_ws）"""
        current_file_dir = os.path.dirname(os.path.abspath(__file__))
        path = current_file_dir
        for _ in range(10):
            if os.path.basename(path) == 'tracking_ros2_ws':
                return path
            parent = os.path.dirname(path)
            if parent == path:
                break
            path = parent
        return None

    def cb_points(self, msg: PointCloud):
        if self.robot is None or not msg.points:
            return

        # Step 0: 提取光学坐标系下的点
        pts_optical = [np.array([p.x, p.y, p.z], dtype=float) for p in msg.points]
        msg_frame = msg.header.frame_id

        if (not msg_frame) or (msg_frame != self.source_frame):
            self.get_logger().warn(f'收到的点云 frame_id={msg_frame}，但预期为 {self.source_frame}，丢弃本帧')
            return

        # 提取 id channel
        ids = None
        try:
            ids = next((ch.values for ch in msg.channels if ch.name == 'id'), None)
        except Exception:
            ids = None

        # 检测是否为粗定位请求（PointCloud.channels 包含 'coarse'）
        try:
            for ch in msg.channels:
                if ch.name == 'coarse' and ch.values and float(ch.values[0]) > 0.0:
                    self._last_msg_was_coarse = True
                    break
        except Exception:
            pass

        # Step 0: optical → camera（默认恒等，与参考代码一致）
        pts_camera = []
        if self.apply_optical_to_camera_rotation:
            R_cam_from_opt = np.array([[0,  0, 1],
                                       [-1, 0, 0],
                                       [0, -1, 0]], dtype=float)
            for p_opt in pts_optical:
                p_cam = R_cam_from_opt @ p_opt
                pts_camera.append(p_cam)
            # self._log_points(np.array(pts_camera), f'{self.source_frame}_camera', '源(相机)', ids, self.print_limit)
        else:
            pts_camera = pts_optical  # 与参考代码 _optical_to_camera() 行为一致

        # Step 1: camera → wrist
        pts_wrist = []
        for p_cam in pts_camera:
            p_wr = self.R_cam2wrist @ p_cam + self.t_cam2wrist
            pts_wrist.append(p_wr)
        # self._log_points(np.array(pts_wrist), self.wrist_frame, '手腕', ids, self.print_limit)

        # Step 2: wrist → base
        try:
            ok, tf_bw = self.robot.get_tf_transform(self.target_frame, self.wrist_frame)
            if not ok or tf_bw is None or len(tf_bw) < 7:
                self.get_logger().warn(f'获取TF失败 {self.target_frame} <- {self.wrist_frame}')
                return
            # [tx, ty, tz, qw, qx, qy, qz]
            tx, ty, tz, qw, qx, qy, qz = [float(tf_bw[i]) for i in range(7)]
            # 四元数转旋转矩阵 (wrist → base)
            norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
            if norm > 0:
                qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
            xx, yy, zz = qx*qx, qy*qy, qz*qz
            xy, xz, yz = qx*qy, qx*qz, qy*qz
            wx, wy, wz = qw*qx, qw*qy, qw*qz
            R_wb = np.array([
                [1 - 2*(yy + zz),     2*(xy - wz),         2*(xz + wy)],
                [2*(xy + wz),         1 - 2*(xx + zz),     2*(yz - wx)],
                [2*(xz - wy),         2*(yz + wx),         1 - 2*(xx + yy)]
            ], dtype=float) if norm > 0 else np.eye(3, dtype=float)
            t_wb = np.array([tx, ty, tz], dtype=float)

            pts_base = []
            for p_wr in pts_wrist:
                p_b = R_wb @ p_wr + t_wb
                pts_base.append(p_b)

        except Exception as e:
            self.get_logger().warn(f'TF异常 {self.target_frame} <- {self.wrist_frame}: {e}')
            return

        # 打印基座坐标
        n_print = min(self.print_limit, len(pts_base))
        if n_print > 0:
            self.get_logger().info(f"转换到 {self.target_frame} 坐标系的点(前{n_print}/{len(pts_base)}个):")
            for i in range(n_print):
                x, y, z = pts_base[i]
                tag = f"#{int(ids[i])}" if ids is not None and i < len(ids) else f"#{i}"
                self.get_logger().info(f"  {tag}: X={x:.4f} Y={y:.4f} Z={z:.4f}")

        # 发布
        if self.pub is not None:
            out_msg = PointCloud()
            out_msg.header = msg.header
            out_msg.header.frame_id = self.target_frame
            out_msg.points = [Point32(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in pts_base]
            out_msg.channels = msg.channels
            self.pub.publish(out_msg)

        # 缓存基座系点用于粗定位控制（暂停时不更新目标）
        if self.enable_coarse_move and not self.motion_paused:
            self.latest_pts_base = pts_base
            self.latest_ids = ids
            # 选择目标点：优先匹配target_id，否则取第一个
            target = None
            if self.target_id >= 0 and ids is not None:
                for i, p in enumerate(pts_base):
                    try:
                        if int(ids[i]) == self.target_id:
                            target = p
                            break
                    except Exception:
                        continue
            if target is None and len(pts_base) > 0:
                target = pts_base[0]

            # >>> 新增：安全过滤 <<<
            valid_target = None
            if target is not None:
                target = np.array(target, dtype=float)
                dist_from_base = np.linalg.norm(target)
                z_val = target[2]

                # 条件1: 距离 <= 2.0m
                # 条件2: Z轴的上下限 -1.0 <= Z <= 0.0 
                if dist_from_base <= 2.0 and (-1.0 <= z_val <= 0.0):
                    valid_target = target
                else:
                    self.get_logger().warn(
                        f'目标点被过滤: distance={dist_from_base:.3f}m, Z={z_val:.3f} (要求: dist≤2.0m 且 -1.0≤Z≤0.0)'
                    )
            # <<<

            self.target_point = valid_target

    def _get_wrist_to_tcp(self) -> tuple[np.ndarray, np.ndarray] | tuple[None, None]:
        """返回 wrist->tcp 的固定变换 (t,q)，并缓存。

        注意：粗定位的距离误差在 TCP 上计算，但下发给机器人的是 wrist 的 set_arm_position。
        """
        if self.robot is None:
            return None, None
        if self._wrist_to_tcp_t is not None and self._wrist_to_tcp_q is not None:
            return self._wrist_to_tcp_t, self._wrist_to_tcp_q

        wrist_link = str(self.wrist_frame)
        if wrist_link.startswith('joint_'):
            wrist_link = 'link_' + wrist_link[len('joint_'):]

        ok, tf_wg = self.robot.get_tf_transform(wrist_link, self.gripper_base_frame)
        if (not ok) or tf_wg is None or len(tf_wg) < 7:
            return None, None

        t_wg = np.array([float(v) for v in tf_wg[:3]], dtype=float)
        q_wg = np.array([float(v) for v in tf_wg[3:7]], dtype=float)  # wxyz

        # gripper_base -> tcp (offset, no rotation)
        t_gt = np.array(self.tcp_offset_xyz, dtype=float)
        q_gt = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

        t_wt, q_wt = self._compose_tq(t_wg, q_wg, t_gt, q_gt)  # wrist -> tcp
        self._wrist_to_tcp_t = t_wt
        self._wrist_to_tcp_q = q_wt
        return t_wt, q_wt

    # ------------------- 末端粗定位控制（基于 set_arm_position） -------------------
    def control_loop(self):
        if not self.enable_coarse_move or self.robot is None:
            return
        if self.motion_paused:
            return
        if self.target_point is None:
            return
        
        # 读取当前 wrist 位姿（get_arm_position） + 固定 wrist->tcp，用于：
        # - 距离误差在 tcp 上算
        # - 下发给机器人的是 wrist 的 set_arm_position
        ok, pose_aw = self.robot.get_arm_position(self.component_type)
        if (not ok) or pose_aw is None or len(pose_aw) < 7:
            return
        t_aw = np.array([float(v) for v in pose_aw[:3]], dtype=float)
        q_aw = np.array([float(v) for v in pose_aw[3:7]], dtype=float)  # wxyz

        t_wt, _q_wt = self._get_wrist_to_tcp()
        if t_wt is None:
            return
        R_aw = self._quat_to_rot_wxyz(q_aw)
        p_curr_tcp = t_aw + (R_aw @ t_wt)

        # 首次缓存姿态，粗定位阶段保持
        if self.cached_quat_wxyz is None:
            self.cached_quat_wxyz = q_aw

        # 误差与距离
        e = self.target_point - p_curr_tcp
        d = float(np.linalg.norm(e))
        dz = float(e[2])

        # 日志：1Hz
        now = time.time()
        if now - self._last_log_ts > 1.0:
            self.get_logger().info(
                f'[粗定位] dist={d:.3f} m, dz={dz:.3f} m, curr=({p_curr_tcp[0]:.3f},{p_curr_tcp[1]:.3f},{p_curr_tcp[2]:.3f}), '
                f'target=({self.target_point[0]:.3f},{self.target_point[1]:.3f},{self.target_point[2]:.3f})'
            )
            self._last_log_ts = now

        # 到达阈值则不再发送 —— 如果是粗定位请求，发布 coarse_done
        if d <= self.distance_stop:
            try:
                if getattr(self, "_last_msg_was_coarse", False):
                    msg = Bool()
                    msg.data = True
                    self.coarse_done_pub.publish(msg)
                    # clear flag so we don't repeatedly publish
                    self._last_msg_was_coarse = False
            except Exception:
                pass
            return

        # 计算下一步 TCP 目标：
        # - Z-first：先把 Z 对齐到 |dz| <= z_align_threshold（用一次 set_arm_position 直接移动，不做步进）
        # - 否则：按空间直线靠近（步进）
        if d <= 1e-6:
            return

        # 姿态保持 cached_quat_wxyz（用于 tcp<->wrist 互算）
        R_cmd = self._quat_to_rot_wxyz(self.cached_quat_wxyz)

        if self.z_first and abs(dz) > float(self.z_align_threshold):
            # 目标：把 TCP 的 Z 直接移动到 target_z - sign(dz)*z_align_threshold
            desired_goal_z = float(self.target_point[2] - math.copysign(float(self.z_align_threshold), dz))
            if self._z_align_goal_tcp_z is None or abs(desired_goal_z - float(self._z_align_goal_tcp_z)) > 0.02:
                self._z_align_goal_tcp_z = desired_goal_z
                self._z_align_cmd_sent = False

            # 已经下发过一次 Z 对齐命令，就等待运动完成（通过 dz 进入阈值判断）
            if self._z_align_cmd_sent:
                return

            p_next_tcp = p_curr_tcp.copy()
            p_next_tcp[2] = float(self._z_align_goal_tcp_z)
            p_next_wrist = p_next_tcp - (R_cmd @ t_wt)
            pose_next = [
                float(p_next_wrist[0]), float(p_next_wrist[1]), float(p_next_wrist[2]),
                float(self.cached_quat_wxyz[0]), float(self.cached_quat_wxyz[1]),
                float(self.cached_quat_wxyz[2]), float(self.cached_quat_wxyz[3]),
            ]

            try:
                if self.use_wait:
                    self.robot.set_arm_position(self.component_type, pose_next, float(self.max_speed), float(self.max_acc), True)
                    self.last_cmd_ts = now
                    self._z_align_cmd_sent = True
                else:
                    if (now - self.last_cmd_ts) >= self.cmd_interval:
                        self.robot.set_arm_position(self.component_type, pose_next, float(self.max_speed), float(self.max_acc), False)
                        self.last_cmd_ts = now
                        self._z_align_cmd_sent = True
            except Exception as e:
                self.get_logger().warn(f'Z 对齐 set_arm_position 失败: {e}')
            return

        # Z 已经足够接近，进入直线靠近：清理 Z 对齐状态
        self._z_align_goal_tcp_z = None
        self._z_align_cmd_sent = False

        step = min(float(self.step_size), max(0.0, d - self.distance_stop))
        p_next_tcp = p_curr_tcp + (e / d) * float(step)

        # TCP->wrist：t_wrist = t_tcp - R(wrist)*t_wt（姿态保持 cached_quat_wxyz）
        p_next_wrist = p_next_tcp - (R_cmd @ t_wt)

        # 构建位姿 [x, y, z, w, x, y, z]
        pose_next = [float(p_next_wrist[0]), float(p_next_wrist[1]), float(p_next_wrist[2]),
                     float(self.cached_quat_wxyz[0]), float(self.cached_quat_wxyz[1]), 
                     float(self.cached_quat_wxyz[2]), float(self.cached_quat_wxyz[3])]

        # 频率/等待控制
        try:
            cmd_ok = False
            if self.use_wait:
                # 阻塞式：调用即等待完成
                self.robot.set_arm_position(self.component_type, pose_next, float(self.max_speed), float(self.max_acc), True)
                cmd_ok = True
                self.last_cmd_ts = now
            else:
                if (now - self.last_cmd_ts) >= self.cmd_interval:
                    self.robot.set_arm_position(self.component_type, pose_next, float(self.max_speed), float(self.max_acc), False)
                    cmd_ok = True
                    self.last_cmd_ts = now

            if cmd_ok:
                self._move_fail_count = 0  # 成功则清零
            else:
                # 非发送时机，不算失败
                pass

        except Exception as e:
            self.get_logger().warn(f'set_arm_position 失败: {e}')
            self._move_fail_count += 1

            if self._move_fail_count >= self._max_fail_count:
                self.get_logger().error(f'❌ 粗定位连续失败 {self._max_fail_count} 次，暂停控制')
                self.enable_coarse_move = False  # 自动关闭


def main(args=None):
    rclpy.init(args=args)
    node = Points3DTFToArmBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
