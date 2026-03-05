import argparse
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Sequence

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from monte_controller_node.points3d_tf_to_arm_base_node import Points3DTFToArmBaseNode


@dataclass(frozen=True)
class LaunchTarget:
    name: str
    package: str
    launch_file: str
    launch_args: list[str]


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='run_node',
        description='Start robot video + tracking + arm-base transform/control in one command.',
        add_help=True,
    )

    parser.add_argument('--skip_video_client', action='store_true', help='Do not start robot_video_client launch.')
    parser.add_argument('--skip_tracking', action='store_true', help='Do not start track_on_ros2 launch.')
    parser.add_argument('--video_wait_s', type=float, default=5.0, help='Seconds to wait after starting video client.')
    parser.add_argument('--tracking_wait_s', type=float, default=5.0, help='Seconds to wait after starting tracking.')

    parser.add_argument('--video_pkg', default='robot_video_client', help='Package for video launch.')
    parser.add_argument('--video_launch', default='robot_video_client.launch.py', help='Launch file for video client.')
    parser.add_argument(
        '--video_arg',
        action='append',
        default=[],
        help='Extra launch arg for video (repeatable), e.g. --video_arg foo:=bar',
    )

    parser.add_argument('--tracking_pkg', default='track_on_ros2', help='Package for tracking launch.')
    parser.add_argument('--tracking_launch', default='track_camera_hand.launch.py', help='Launch file for tracking.')
    parser.add_argument(
        '--tracking_arg',
        action='append',
        default=[],
        help='Extra launch arg for tracking (repeatable), e.g. --tracking_arg camera_topic:=/xxx',
    )
    return parser


def _start_ros2_launch(logger, target: LaunchTarget) -> subprocess.Popen:
    cmd = ['ros2', 'launch', target.package, target.launch_file, *target.launch_args]
    logger.info(f"[run_node] starting {target.name}: {' '.join(cmd)}")
    proc = subprocess.Popen(cmd, start_new_session=True)
    logger.info(f"[run_node] {target.name} started (pid={proc.pid})")
    return proc


def _stop_process_group(logger, proc: subprocess.Popen, name: str, timeout_s: float = 5.0) -> None:
    if proc.poll() is not None:
        return

    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return

    logger.info(f'[run_node] stopping {name} (pgid={pgid}) ...')
    try:
        os.killpg(pgid, signal.SIGINT)
    except ProcessLookupError:
        return

    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if proc.poll() is not None:
            return
        time.sleep(0.1)

    logger.warn(f'[run_node] {name} did not exit after {timeout_s:.1f}s, sending SIGKILL')
    try:
        os.killpg(pgid, signal.SIGKILL)
    except ProcessLookupError:
        return


class _SupervisorNode(Node):
    def __init__(self, procs: dict[str, subprocess.Popen]):
        super().__init__('run_node')
        self._procs = procs
        self._timer = self.create_timer(0.5, self._check_procs)

    def _check_procs(self) -> None:
        for name, proc in list(self._procs.items()):
            rc = proc.poll()
            if rc is None:
                continue
            self.get_logger().error(f'[run_node] child process "{name}" exited with code {rc}; shutting down')
            rclpy.shutdown()
            return


def main(args: Sequence[str] | None = None) -> None:
    # 避免 CycloneDDS 绑定到不存在的网卡 (192.168.x.x does not match available interface)
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
    rclpy.init(args=args)
    logger = rclpy.logging.get_logger('run_node')

    non_ros_args = remove_ros_args(args if args is not None else sys.argv)
    parsed = _build_arg_parser().parse_args(non_ros_args[1:])

    procs: dict[str, subprocess.Popen] = {}
    supervisor: _SupervisorNode | None = None
    points_node: Points3DTFToArmBaseNode | None = None

    try:
        if not parsed.skip_video_client:
            video_target = LaunchTarget(
                name='robot_video_client',
                package=str(parsed.video_pkg),
                launch_file=str(parsed.video_launch),
                launch_args=[str(a) for a in parsed.video_arg],
            )
            procs[video_target.name] = _start_ros2_launch(logger, video_target)
            time.sleep(max(0.0, float(parsed.video_wait_s)))

        if not parsed.skip_tracking:
            tracking_target = LaunchTarget(
                name='track_on_ros2',
                package=str(parsed.tracking_pkg),
                launch_file=str(parsed.tracking_launch),
                launch_args=[str(a) for a in parsed.tracking_arg],
            )
            procs[tracking_target.name] = _start_ros2_launch(logger, tracking_target)
            time.sleep(max(0.0, float(parsed.tracking_wait_s)))

        points_node = Points3DTFToArmBaseNode()
        supervisor = _SupervisorNode(procs)

        executor = MultiThreadedExecutor()
        executor.add_node(points_node)
        executor.add_node(supervisor)

        logger.info('[run_node] all started; Ctrl+C to stop')
        executor.spin()

    except KeyboardInterrupt:
        pass
    finally:
        # Stop children first (they may depend on ROS graph still being alive).
        for name, proc in list(procs.items())[::-1]:
            _stop_process_group(logger, proc, name)

        if supervisor is not None:
            supervisor.destroy_node()
        if points_node is not None:
            points_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
